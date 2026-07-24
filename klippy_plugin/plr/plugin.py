"""Config-section handling and g-code command registration.

This module owns the object klippy stores for the ``[plr]`` section:
it parses and validates every schema key (bad values fail config
parsing with klippy's standard errors, klippy/configfile.py:29-60),
registers the ``PLR_*`` console commands, and exposes
``get_status(eventtime)`` so Moonraker/plrd can read plugin state
through klippy's API socket (any printer object with a get_status
method is queryable via ``printer.objects``).

Command handlers live in the sibling modules as module-level
``cmd_PLR_<NAME>(plugin, gcmd)`` entry points; this module only wires
them into klippy's dispatcher (klippy/gcode.py:133-157
``register_command`` with the conventional ``cmd_FOO_help`` strings).
The drag-oracle commands (PLR_NOISE_TEST / PLR_DRAG_PROBE) are
registered here already and route into the scaffold modules, so the
drag milestone only fills in those modules — no re-wiring.

SAVE_CONFIG-persisted state (written via ``configfile.set`` under
section "plr", read back from the autosave block on restart as plain
config options — klippy/configfile.py:311-324):

* ``self_locking_z`` — operator attestation staged by PLR_SETUP;
* ``probe_resolution`` — measured by PLR_PROBE_TEST;
* ``noise_floor_rms`` / ``noise_floor_still_rms`` / ``noise_floor_peak``
  / ``noise_floor_speed`` — measured by PLR_NOISE_TEST.  Parsed here at
  config time (not read
  lazily by the drag modules) for the same reason probe_resolution is:
  the live value doubles as this-session state that PLR_NOISE_TEST
  updates in place, and config-time getfloat bounds reject a
  hand-mangled autosave value with klippy's standard error instead of
  a mid-probe surprise.
"""

import math
import os
import time

from . import (
    calibration_meta,
    daemon_link,
    drag_calibrate,
    drag_probe,
    noise_test,
    probe_test,
    setup_checks,
    touch_sequence,
    tunables,
)

PROBE_METHODS = ["tap", "load_cell", "adxl_drag"]

# get_status re-stats the heartbeat file at most this often (seconds of
# reactor time): Moonraker polls status several times per second and a
# stat per poll is wasteful.
_DAEMON_ALIVE_CACHE_SECS = 1.0


class PLRPlugin:
    """The object klippy stores for the ``[plr]`` config section."""

    def __init__(self, config):
        self.config = config
        self.name = config.get_name()
        self.printer = config.get_printer()
        # --- oracle selection ---------------------------------------
        self.probe_method = config.getchoice("probe_method", PROBE_METHODS, "tap")
        # accel_chip names a section like "adxl345" or "adxl345 bed"
        # (klippy/extras/adxl345.py:313-320 registers the printer
        # object under the full section name).  Required iff the drag
        # oracle is selected; allowed-but-unused otherwise so a config
        # can stage a chip before switching methods.
        self.accel_chip_name = config.get("accel_chip", None)
        if self.probe_method == "adxl_drag" and not self.accel_chip_name:
            raise config.error(
                "Option 'accel_chip' in section 'plr' must be specified "
                "when probe_method is adxl_drag"
            )
        # --- recorder coordination ----------------------------------
        self.wal_dir = config.get("wal_dir", "/var/lib/plrd/wal")
        self.control_socket = config.get("control_socket", "/var/lib/plrd/plrd.sock")
        self.daemon = daemon_link.DaemonLink(self.control_socket)
        # --- tunables (schema + ranges live in tunables.TUNABLES) ---
        self.tunables = tunables.load_from_config(config)
        # --- SAVE_CONFIG-persisted state (autosave block; defaults
        # keep a fresh install working before first SAVE_CONFIG) ------
        self.self_locking_z = config.getboolean("self_locking_z", False)
        self.probe_resolution = config.getfloat("probe_resolution", None, above=0.0)
        self.noise_floor_rms = config.getfloat("noise_floor_rms", None, above=0.0)
        self.noise_floor_still_rms = config.getfloat(
            "noise_floor_still_rms", None, minval=0.0
        )
        self.noise_floor_peak = config.getfloat("noise_floor_peak", None, minval=0.0)
        # Baseline capture speed (mm/s), read by plrd's plan validation
        # to warn on drag-speed mismatch.  getfloat's above=0.0 rejects
        # non-positive values but happily parses "inf"/"nan" (nan even
        # passes the bound comparisons), so finiteness is checked
        # explicitly — a mangled autosave value must fail at config
        # time, not at plan time.
        self.noise_floor_speed = config.getfloat("noise_floor_speed", None, above=0.0)
        if self.noise_floor_speed is not None and not math.isfinite(
            self.noise_floor_speed
        ):
            raise config.error(
                "Option 'noise_floor_speed' in section 'plr' must be finite"
            )
        # Temperature covariate for the drag oracle.  noise_floor_temp is
        # a sibling autosave key staged by PLR_NOISE_TEST (the temperature
        # the moving baseline was captured at); noise_floor_temp_sensor
        # names the klippy sensor object to read the current temperature
        # from at probe time.  Both optional: parsed here at config time —
        # not lazily in the drag modules — so a hand-mangled autosave
        # value fails config parsing with klippy's standard error, and so
        # klippy's unused-option check accepts the keys.  When no sensor
        # is configured the covariate is skipped silently (no guessing).
        self.noise_floor_temp = config.getfloat("noise_floor_temp", None)
        if self.noise_floor_temp is not None and not math.isfinite(
            self.noise_floor_temp
        ):
            raise config.error(
                "Option 'noise_floor_temp' in section 'plr' must be finite"
            )
        self.noise_floor_temp_sensor = config.get("noise_floor_temp_sensor", None)
        # --- drag-oracle session state (PLR_DRAG_PROBE outcome; plrd
        # reads these through get_status like probe status) ------------
        self.last_drag_result = None
        self.last_drag_error = None
        # PLR_DRAG_CALIBRATE outcome: the accepted/recommended sensitivity
        # knobs from the last sweep (None until it runs), for observability.
        self.last_drag_calibrate = None
        # --- consensus-touch session state (PLR_TOUCH outcome; the Rust
        # side consumes last_touch_result through get_status) ----------
        self.last_touch_result = None
        # Z floor input for the drag staircase, cached at config time
        # (config is immutable per session).
        self.z_position_min, _ = setup_checks.z_position_min(config)
        # Options PLR_SET / PLR_SETUP / PLR_PROBE_TEST staged this
        # session (live but not yet written by SAVE_CONFIG).
        self._pending_save = set()
        # --- calibration stamps + three-tier validation --------------
        # The version/fingerprint stamps staged alongside each calibration
        # value (calibration_meta).  Read here so klippy's unused-option
        # check accepts the autosave keys, and to drive validation.  A
        # major.minor plugin regression or a hardware-fingerprint change
        # marks the affected value-group INVALID; its live values are then
        # nulled below so every consumer (drag commands, get_status, plrd)
        # sees an uncalibrated machine — treat-as-absent, our supported
        # stand-in for carto's "remove the incompatible model" (klippy has
        # no plugin-reachable autosave delete).
        self.cal_plugin_version = config.get(calibration_meta.PLUGIN_VERSION_KEY, None)
        self.cal_klipper_version = config.get(
            calibration_meta.KLIPPER_VERSION_KEY, None
        )
        self._cal_fingerprints = {
            group: config.get(calibration_meta.fingerprint_key(group), None)
            for group in calibration_meta.GROUPS
        }
        self._legacy_warned = False
        self.calibrations = self._validate_calibrations()
        self._apply_calibration_gating()
        # --- static commissioning checks (config is immutable per
        # klippy session, so config-time results stay valid) ----------
        self.static_check_results = setup_checks.run_static_checks(
            config, self.probe_method, self.accel_chip_name
        )
        # --- daemon_alive cache for get_status ----------------------
        self._daemon_alive = False
        self._daemon_alive_checked = None
        # --- console commands ---------------------------------------
        self._register_commands()

    # -- command wiring ----------------------------------------------

    cmd_PLR_SETUP_help = (
        "Report plr commissioning checks; ACCEPT_SELF_LOCKING_Z=1 stages "
        "the self-locking-Z attestation for SAVE_CONFIG"
    )
    cmd_PLR_SET_help = (
        "Set a plr tunable (PARAM= VALUE=) and stage it for SAVE_CONFIG; "
        "no arguments lists current values"
    )
    cmd_PLR_PROBE_TEST_help = (
        "Verify probe repeatability at the current XY by running consensus "
        "touch sequences (requires START=1; moves the toolhead) and stage "
        "probe_resolution for SAVE_CONFIG"
    )
    cmd_PLR_TOUCH_help = (
        "Run one sliding-window consensus touch at the current XY (moves the "
        "toolhead) and expose the result as last_touch_result"
    )
    cmd_PLR_STATUS_help = "Report plr plugin state and plrd daemon status"
    cmd_PLR_RECOVER_help = (
        "Power-loss recovery via plrd: dry run by default; "
        "EXECUTE=1 CONFIRM=YES executes"
    )
    cmd_PLR_NOISE_TEST_help = (
        "Measure the accel-chip noise floor (requires START=1; moves the "
        "toolhead) and stage noise_floor_* for SAVE_CONFIG"
    )
    cmd_PLR_DRAG_PROBE_help = (
        "Locate the part surface by dragging lateral passes down a Z "
        "staircase with the accel chip (CHIP= SPEED= Z_STEP= SENSITIVITY=)"
    )
    cmd_PLR_DRAG_CALIBRATE_help = (
        "Find the most-sensitive drag knob that never false-triggers, "
        "entirely at a guaranteed-clear Z (requires START=1; moves the "
        "toolhead laterally only) and stage drag_sensitivity for SAVE_CONFIG"
    )

    def _register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        # (name, module entry point) — each handler is a module-level
        # cmd_PLR_*(plugin, gcmd); the drag agent replaces the bodies
        # in noise_test/drag_probe without touching this table.
        commands = [
            ("PLR_SETUP", setup_checks.cmd_PLR_SETUP),
            ("PLR_SET", tunables.cmd_PLR_SET),
            ("PLR_PROBE_TEST", probe_test.cmd_PLR_PROBE_TEST),
            ("PLR_TOUCH", touch_sequence.cmd_PLR_TOUCH),
            ("PLR_STATUS", daemon_link.cmd_PLR_STATUS),
            ("PLR_RECOVER", daemon_link.cmd_PLR_RECOVER),
            ("PLR_NOISE_TEST", noise_test.cmd_PLR_NOISE_TEST),
            ("PLR_DRAG_PROBE", drag_probe.cmd_PLR_DRAG_PROBE),
            ("PLR_DRAG_CALIBRATE", drag_calibrate.cmd_PLR_DRAG_CALIBRATE),
        ]
        for name, func in commands:
            gcode.register_command(
                name,
                self._bind(func),
                desc=getattr(self, "cmd_%s_help" % (name,)),
            )

    def _bind(self, func):
        # functools.partial would also do; an explicit closure keeps
        # the handler a plain function of gcmd, as klippy expects
        # (klippy/gcode.py:151 wraps handlers the same way).
        def handler(gcmd):
            return func(self, gcmd)

        return handler

    # -- shared state helpers used by the command modules -------------

    def note_pending_save(self, option):
        """Record that ``option`` was staged for SAVE_CONFIG this session."""
        self._pending_save.add(option)

    def is_pending_save(self, option):
        return option in self._pending_save

    def lookup_accel_chip(self):
        """Resolve the configured accel chip printer object, lazily.

        Deferred past config time on purpose: section objects are
        created in config order, so [plr] may load before the chip
        section (resonance_tester defers the same way,
        klippy/extras/resonance_tester.py:300-304 resolves chip names
        at 'connect' time).  Raises printer.command_error if the chip
        is absent so callers surface a console error.
        """
        if not self.accel_chip_name:
            raise self.printer.command_error("no accel_chip configured in [plr]")
        chip = self.printer.lookup_object(self.accel_chip_name, None)
        if chip is None:
            raise self.printer.command_error(
                "accel chip '%s' not found — is its config section present?"
                % (self.accel_chip_name,)
            )
        return chip

    # -- calibration validation (calibration_meta three-tier) ---------

    def _validate_calibrations(self):
        """Classify each value-group at config time (VALID/LEGACY/INVALID/
        UNSET) and log LEGACY/INVALID once per session (config load is
        naturally once, so this cannot spam)."""
        present = {
            calibration_meta.GROUP_NOISE_FLOOR: self.noise_floor_rms is not None,
            calibration_meta.GROUP_PROBE_RESOLUTION: self.probe_resolution is not None,
        }
        running = calibration_meta.plugin_version()
        results = {}
        for group in calibration_meta.GROUPS:
            result = calibration_meta.validate_group(
                self.config,
                group,
                value_present=present[group],
                stored_fingerprint=self._cal_fingerprints[group],
                stored_plugin_version=self.cal_plugin_version,
                running_plugin_version=running,
            )
            results[group] = result
            if result.tier == calibration_meta.TIER_INVALID:
                calibration_meta.logger.warning(
                    "plr: %s calibration is stale and will be ignored (%s); "
                    "re-run the matching PLR_*_TEST",
                    group,
                    "; ".join(result.reasons),
                )
            elif result.tier == calibration_meta.TIER_LEGACY:
                calibration_meta.logger.warning(
                    "plr: %s calibration predates fingerprint stamping — "
                    "accepted, but consider re-running the matching "
                    "PLR_*_TEST; a future version may refuse unstamped values",
                    group,
                )
        return results

    def _apply_calibration_gating(self):
        """Null the live values of every INVALID group so they are absent
        everywhere (treat-as-absent = our autosave-delete stand-in)."""
        if (
            self.calibrations[calibration_meta.GROUP_NOISE_FLOOR].tier
            == calibration_meta.TIER_INVALID
        ):
            self.noise_floor_rms = None
            self.noise_floor_still_rms = None
            self.noise_floor_peak = None
            self.noise_floor_speed = None
            self.noise_floor_temp = None
        if (
            self.calibrations[calibration_meta.GROUP_PROBE_RESOLUTION].tier
            == calibration_meta.TIER_INVALID
        ):
            self.probe_resolution = None

    def calibration_tier(self, group):
        """The validation tier of ``group`` (valid/legacy/invalid/unset)."""
        return self.calibrations[group].tier

    def stale_calibration_message(self, group, commands):
        """Console remediation text when ``group`` is INVALID (its values
        were nulled), else ``None``.  ``commands`` names the PLR command(s)
        that re-establish it."""
        result = self.calibrations.get(group)
        if result is None or result.tier != calibration_meta.TIER_INVALID:
            return None
        return (
            "the recorded %s calibration was made under a different hardware "
            "configuration and is being ignored (%s) — re-run %s (then "
            "SAVE_CONFIG)" % (group, "; ".join(result.reasons), commands)
        )

    def warn_legacy_calibration_once(self, gcmd):
        """Emit the LEGACY 'consider recalibrating' notice to the console at
        most once per session (not per command)."""
        if self._legacy_warned:
            return
        legacy = sorted(
            group
            for group, result in self.calibrations.items()
            if result.tier == calibration_meta.TIER_LEGACY
        )
        if not legacy:
            return
        self._legacy_warned = True
        gcmd.respond_info(
            "plr: calibration(s) %s predate fingerprint stamping — accepted, "
            "but consider re-running PLR_NOISE_TEST / PLR_PROBE_TEST; a future "
            "version may refuse unstamped values" % (", ".join(legacy),)
        )

    def calibrations_valid(self):
        """Aggregate validity for get_status: ``False`` if any group is
        INVALID, ``"legacy"`` if any is LEGACY (and none INVALID), else
        ``True``."""
        tiers = [result.tier for result in self.calibrations.values()]
        if calibration_meta.TIER_INVALID in tiers:
            return False
        if calibration_meta.TIER_LEGACY in tiers:
            return "legacy"
        return True

    # -- status for Moonraker / API-socket clients --------------------

    def _daemon_alive_now(self, eventtime):
        """Recorder-liveness hint from the heartbeat file mtime.

        Deliberately NOT a socket ping: get_status runs on the klippy
        reactor several times a second and must never block on connect
        timeouts.  Freshness semantics match
        setup_checks.check_recorder_heartbeat (a hint, not proof).
        """
        if (
            self._daemon_alive_checked is not None
            and eventtime - self._daemon_alive_checked < _DAEMON_ALIVE_CACHE_SECS
        ):
            return self._daemon_alive
        self._daemon_alive_checked = eventtime
        path = os.path.join(self.wal_dir, "heartbeat.bin")
        try:
            age = time.time() - os.stat(path).st_mtime
            self._daemon_alive = age <= setup_checks.HEARTBEAT_FRESH_SECS
        except OSError:
            self._daemon_alive = False
        return self._daemon_alive

    def get_status(self, eventtime):
        """Status dict for ``printer.objects`` queries.

        'configured' means no static commissioning check fails (warns
        allowed); full commissioning additionally requires the
        self_locking_z attestation ('attested') — mirrored from the
        PLR_SETUP report so API clients can tell the two apart.
        """
        configured = all(res.verdict != "fail" for res in self.static_check_results)
        return {
            "probe_method": self.probe_method,
            "configured": configured,
            "attested": self.self_locking_z,
            "probe_resolution": self.probe_resolution,
            "daemon_alive": self._daemon_alive_now(eventtime),
            "noise_floor_rms": self.noise_floor_rms,
            "noise_floor_temp": self.noise_floor_temp,
            "last_drag_result": self.last_drag_result,
            "last_drag_error": self.last_drag_error,
            "last_drag_calibrate": self.last_drag_calibrate,
            "last_touch_result": self.last_touch_result,
            # Calibration-stamp validity (calibration_meta three-tier).
            # ``calibrations_valid`` is False when any group is stale (its
            # values are treated as absent), "legacy" for accepted-but-
            # unstamped values, True otherwise; ``calibration_status`` gives
            # the per-group tier so clients see which group is affected.
            "calibrations_valid": self.calibrations_valid(),
            "calibration_status": {
                group: result.tier for group, result in self.calibrations.items()
            },
        }
