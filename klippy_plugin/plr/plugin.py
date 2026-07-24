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
    daemon_link,
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
        # --- drag-oracle session state (PLR_DRAG_PROBE outcome; plrd
        # reads these through get_status like probe status) ------------
        self.last_drag_result = None
        self.last_drag_error = None
        # --- consensus-touch session state (PLR_TOUCH outcome; the Rust
        # side consumes last_touch_result through get_status) ----------
        self.last_touch_result = None
        # Z floor input for the drag staircase, cached at config time
        # (config is immutable per session).
        self.z_position_min, _ = setup_checks.z_position_min(config)
        # Options PLR_SET / PLR_SETUP / PLR_PROBE_TEST staged this
        # session (live but not yet written by SAVE_CONFIG).
        self._pending_save = set()
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
            "last_drag_result": self.last_drag_result,
            "last_drag_error": self.last_drag_error,
            "last_touch_result": self.last_touch_result,
        }
