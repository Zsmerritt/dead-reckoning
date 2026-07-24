"""Commissioning checks and the PLR_SETUP report.

Two kinds of check back the ``PLR_SETUP`` command:

* **Static (config) checks** run once at config time, against the
  ConfigWrapper klippy hands to ``load_config`` — klippy's config is
  immutable for the life of a session (changing printer.cfg requires a
  RESTART), so caching these results on the plugin object is exact, not
  an approximation.
* **Dynamic checks** (currently: the recorder heartbeat file) are
  re-evaluated on every ``PLR_SETUP`` run.

Each check is a small function returning a :class:`CheckResult` so it
can be unit-tested over realistic config dicts without a console.

The self-locking-Z attestation is the one thing that cannot be checked
from software: whether the Z axis holds position unpowered (leadscrew
printers do; many belted-Z printers do not).  ``PLR_SETUP
ACCEPT_SELF_LOCKING_Z=1`` stages ``self_locking_z`` into the autosave
section via ``configfile.set`` and asks the user to run SAVE_CONFIG —
the same persist-then-restart convention klippy's probe calibration
uses (klippy/extras/probe.py:100-105 respond + configfile.set).
"""

import collections
import importlib.util
import math
import os
import time

from . import calibration_meta, classifier

# A single commissioning check outcome.  verdict is one of
# classifier.VERDICTS; hint is the remediation shown on warn/fail (may
# be empty on pass).
CheckResult = collections.namedtuple(
    "CheckResult", ["name", "verdict", "detail", "hint"]
)

# Freshness window for the recorder heartbeat file.  plrd rewrites
# <wal_dir>/heartbeat.bin at 10 Hz (docs/architecture.md, "Heartbeat
# file"), so 2 s of slack tolerates scheduler hiccups while still
# catching a dead recorder.  This is a LIVENESS HINT only: a fresh
# mtime proves a writer was recently alive, not that the WAL is durable
# or complete — durability is plrd's job to prove, at recovery time.
HEARTBEAT_FRESH_SECS = 2.0

# Accel-chip section prefixes klippy registers via load_config /
# load_config_prefix (klippy/extras/adxl345.py:313-320 registers both
# "[adxl345]" and "[adxl345 name]"; the other chips follow the same
# pattern).  Used only to LIST candidate chips for the drag oracle.
ACCEL_CHIP_PREFIXES = ("adxl345", "lis2dw", "lis3dh", "mpu9250", "icm20948")

# Sections that provide the probe object for each probe_method.  Both
# [probe] (klippy/extras/probe.py:630-631) and [load_cell_probe]
# (klippy/extras/load_cell_probe.py:518 add_object('probe', self))
# register the printer object named 'probe'.
_PROBE_SECTION_BY_METHOD = {"tap": "probe", "load_cell": "load_cell_probe"}


# ---------------------------------------------------------------------
# Contact-operation nozzle-temperature gate (the shared safety helper)
#
# Every PLR command that can bring the nozzle to the part — PLR_TOUCH,
# PLR_PROBE_TEST, PLR_DRAG_PROBE and PLR_DRAG_CALIBRATE — must refuse
# while the nozzle is hot.  A molten nozzle oozes filament onto the part
# and bed (a descending touch smears it; a drag pass runs at a clear Z
# but a dripping nozzle still contaminates the surface it reads), and the
# contact reading is only trustworthy from a clean, cold tip.  The gate
# lives HERE, in exactly one function, and all four commands reach it
# through their existing ready/motion-gate helpers
# (touch_sequence.require_touch_ready and drag_probe.check_motion_gates),
# so the threshold comparison is single-sourced — there is no duplicated
# temperature logic to drift.
#
# Schema for the FROZEN [plr] ``max_probe_nozzle_temp`` key (parsed and
# bounds-checked in plugin.py against these constants).
MAX_PROBE_NOZZLE_TEMP_DEFAULT = 150.0
MAX_PROBE_NOZZLE_TEMP_MIN = 80.0
MAX_PROBE_NOZZLE_TEMP_MAX = 160.0


def active_extruder(printer):
    """The active extruder printer object, or ``None`` when there is none.

    Prefers the toolhead's currently-selected extruder
    (klippy/toolhead.py ``get_extruder`` returns the active
    PrinterExtruder) — on a toolchanger that is the nozzle that will
    actually reach the probe point — and falls back to the primary
    ``extruder`` printer object.  Returns ``None`` on a machine with no
    extruder/heater at all: there is nothing to ooze, so nothing to gate.
    """
    toolhead = printer.lookup_object("toolhead", None)
    if toolhead is not None and hasattr(toolhead, "get_extruder"):
        extruder = toolhead.get_extruder()
        if extruder is not None and hasattr(extruder, "get_status"):
            return extruder
    extruder = printer.lookup_object("extruder", None)
    if extruder is not None and hasattr(extruder, "get_status"):
        return extruder
    return None


def nozzle_temperatures(printer):
    """``(current, target)`` °C of the active extruder, or ``None``.

    Reads the extruder's ``get_status``, which delegates to its heater
    (klippy/heaters.py ``Heater.get_status`` reports ``temperature`` and
    ``target``; klippy/kinematics/extruder.py ``get_status`` returns that
    dict).  A missing field is read as ``0.0`` so a partially-populated
    status can never hide heat.  ``None`` when no extruder is present or
    neither field is reported.
    """
    extruder = active_extruder(printer)
    if extruder is None:
        return None
    status = extruder.get_status(printer.get_reactor().monotonic())
    current = status.get("temperature")
    target = status.get("target")
    if current is None and target is None:
        return None
    return (current or 0.0, target or 0.0)


def nozzle_too_hot_message(printer, max_temp, command):
    """Console refusal text when the nozzle is too hot for ``command`` to
    bring it to the part, else ``None``.

    Gates on ``max(current, target)``: a nozzle at 45 °C already
    *commanded* to 250 °C is on its way up and must be refused now, not
    after it melts onto the part.  Strictly greater than ``max_temp``
    refuses; at-or-below passes.  THE THRESHOLD COMPARISON LIVES ONLY
    HERE — the four contact commands all reach it through their shared
    gate helpers, so there is nothing to keep in sync.
    """
    temps = nozzle_temperatures(printer)
    if temps is None:
        return None
    current, target = temps
    if max(current, target) <= max_temp:
        return None
    return (
        "%s refused: nozzle is %.0f°C (target %.0f°C) — a hot "
        "nozzle oozes onto the part and skews contact readings. Cool the "
        "nozzle below %d°C / M104 S0 and wait for it to drop, then retry."
        % (command, current, target, max_temp)
    )


def require_nozzle_cool(plugin, gcmd, command):
    """Raise ``gcmd.error`` if the nozzle is too hot for ``command``.

    The single call the contact-gate helpers make; keeps the refusal
    one line at every call site while the comparison stays in
    :func:`nozzle_too_hot_message`.
    """
    message = nozzle_too_hot_message(
        plugin.printer, plugin.max_probe_nozzle_temp, command
    )
    if message:
        raise gcmd.error(message)


# ---------------------------------------------------------------------
# Clean-nozzle detection (config lookup) + the PLR_SETUP mode row.

_GCODE_MACRO_PREFIX = "gcode_macro "


def gcode_macro_available(config, macro_name):
    """True when a ``[gcode_macro <macro_name>]`` section is configured.

    Klipper registers a gcode_macro's command as its section-name suffix
    uppercased (klippy/extras/gcode_macro.py: the alias is
    ``config.get_name().split()[1].upper()``), so macro names are
    effectively case-insensitive — match the suffix that way.  Enumerated
    via the same prefix scan klippy tooling uses
    (klippy/configfile.py:124-126 ``get_prefix_sections``).
    """
    wanted = macro_name.strip().upper()
    for section in config.get_prefix_sections(_GCODE_MACRO_PREFIX):
        suffix = section.get_name()[len(_GCODE_MACRO_PREFIX) :].strip().upper()
        if suffix == wanted:
            return True
    return False


def clean_nozzle_check_result(plugin):
    """PLR_SETUP row naming which nozzle-cleanliness mode applies: an
    auto-run clean macro, or the recovery wizard's manual confirmation.

    Informational only (a ``warn`` at most — the CLEAN_NOZZLE macro is a
    convention, not a requirement, so its absence never blocks
    COMMISSIONED)."""
    macro = plugin.clean_nozzle_macro
    if plugin.clean_nozzle_macro_available:
        return CheckResult(
            "clean nozzle",
            "pass",
            "auto: [gcode_macro %s] present — recovery cleans the nozzle "
            "before contact probing" % (macro,),
            "",
        )
    return CheckResult(
        "clean nozzle",
        "warn",
        "manual: no [gcode_macro %s] — the recovery wizard asks you to "
        "confirm the nozzle is clean before contact probing" % (macro,),
        "add a [gcode_macro %s] that wipes/cleans the nozzle to automate this "
        "(or just confirm cleanliness at the wizard prompt)" % (macro,),
    )


def check_force_move(config):
    """[force_move] must exist with enable_force_move on.

    Recovery re-establishes machine position without homing against the
    part; that needs SET_KINEMATIC_POSITION, which klipper only
    registers when enable_force_move is set
    (klippy/extras/force_move.py:42-47).
    """
    if not config.has_section("force_move"):
        return CheckResult(
            "force_move",
            "fail",
            "no [force_move] section",
            "add a [force_move] section with enable_force_move: True",
        )
    enabled = config.getsection("force_move").getboolean("enable_force_move", False)
    if not enabled:
        return CheckResult(
            "force_move",
            "fail",
            "[force_move] present but enable_force_move is off",
            "set enable_force_move: True (needed for SET_KINEMATIC_POSITION)",
        )
    return CheckResult("force_move", "pass", "enable_force_move on", "")


def check_probe_section(config, probe_method, accel_chip):
    """Exactly one probe section matching probe_method must exist."""
    if probe_method == "adxl_drag":
        if not accel_chip:
            return CheckResult(
                "probe section",
                "fail",
                "probe_method adxl_drag but no accel_chip configured",
                "set accel_chip in [plr] (e.g. accel_chip: adxl345)",
            )
        if not config.has_section(accel_chip):
            return CheckResult(
                "probe section",
                "fail",
                "accel_chip '%s' has no matching config section" % (accel_chip,),
                "add an [%s] section (klippy/extras/adxl345.py supports both "
                "[adxl345] and [adxl345 <name>])" % (accel_chip,),
            )
        return CheckResult(
            "probe section", "pass", "accel chip section [%s] found" % (accel_chip,), ""
        )
    wanted = _PROBE_SECTION_BY_METHOD[probe_method]
    other = _PROBE_SECTION_BY_METHOD["load_cell" if probe_method == "tap" else "tap"]
    if not config.has_section(wanted):
        return CheckResult(
            "probe section",
            "fail",
            "probe_method %s needs a [%s] section" % (probe_method, wanted),
            "add [%s] or change probe_method" % (wanted,),
        )
    if config.has_section(other):
        # Both sections register the 'probe' printer object, so klippy
        # itself would refuse this config (Printer.add_object raises on
        # duplicates) — but report it here too so PLR_SETUP explains
        # WHY klippy failed to start last time.
        return CheckResult(
            "probe section",
            "fail",
            "both [%s] and [%s] present — they conflict" % (wanted, other),
            "remove the one that does not match probe_method=%s" % (probe_method,),
        )
    return CheckResult("probe section", "pass", "[%s] found" % (wanted,), "")


def _pin_chip_name(pin_desc):
    """Return the chip prefix of a klippy pin description.

    Mirrors klippy/pins.py:67-93 ``parse_pin``: strip the pullup
    (``^``/``~``) and invert (``!``) modifiers, then split on the first
    ``:``; a bare pin name belongs to the primary MCU chip ``mcu``.
    HONEST LIMITATION: this is a textual heuristic over the config —
    it does not consult the live pin registry, so an exotic setup that
    registers a *different* chip named ``mcu`` (klippy forbids this for
    real MCUs) or routes Z through a virtual pin chip is not detected.
    """
    desc = str(pin_desc).strip()
    while desc[:1] in ("^", "~", "!"):
        desc = desc[1:].strip()
    if ":" not in desc:
        return "mcu"
    return desc.split(":", 1)[0].strip()


def check_z_steppers_on_primary_mcu(config):
    """Every stepper_z* section's pins must live on the primary MCU.

    The recorder's committed-motion boundary comes from dump_stepper
    step history with raw MCU clocks; mixing Z steppers across MCUs
    breaks the single-clock reconstruction.  Sections are found the way
    klippy tools enumerate related sections, via prefix scan
    (klippy/configfile.py:124-126 ``get_prefix_sections``).
    """
    sections = config.get_prefix_sections("stepper_z")
    if not sections:
        return CheckResult(
            "z steppers",
            "warn",
            "no stepper_z* sections found (non-cartesian kinematics?)",
            "dead-reckoning currently assumes stepper_z-style Z axes",
        )
    offending = []
    for section in sections:
        for option in ("step_pin", "dir_pin", "enable_pin"):
            pin = section.get(option, None)
            if pin is None:
                continue
            chip = _pin_chip_name(pin)
            if chip != "mcu":
                offending.append(
                    "[%s] %s: %s (chip '%s')" % (section.get_name(), option, pin, chip)
                )
    if offending:
        return CheckResult(
            "z steppers",
            "fail",
            "Z stepper pins on secondary MCU: %s" % ("; ".join(offending),),
            "move Z steppers to the primary MCU — recovery needs all Z step "
            "history on one clock",
        )
    return CheckResult(
        "z steppers",
        "pass",
        "%d stepper_z* section(s), all pins on primary MCU" % (len(sections),),
        "",
    )


def check_probe_gcode_empty(config, probe_method):
    """Probe activate/deactivate g-code must be empty.

    Verified by reading the raw option values from the probe section —
    the same options klippy loads as templates with an empty-string
    default (klippy/extras/probe.py:550-554).  A probe that runs g-code
    to deploy cannot be trusted mid-recovery (the scripts may move the
    toolhead or assume a homed state), so any non-whitespace content
    fails.  This check replaces a blind user attestation.
    """
    if probe_method == "adxl_drag":
        return CheckResult(
            "probe gcode", "pass", "not applicable (adxl_drag has no probe section)", ""
        )
    section_name = _PROBE_SECTION_BY_METHOD[probe_method]
    if not config.has_section(section_name):
        return CheckResult(
            "probe gcode",
            "warn",
            "cannot verify: [%s] section missing" % (section_name,),
            "fix the probe-section check first",
        )
    section = config.getsection(section_name)
    dirty = []
    for option in ("activate_gcode", "deactivate_gcode"):
        raw = section.get(option, "")
        if raw is not None and str(raw).strip():
            dirty.append(option)
    if dirty:
        return CheckResult(
            "probe gcode",
            "fail",
            "[%s] has non-empty %s" % (section_name, ", ".join(dirty)),
            "recovery probing cannot run deploy scripts; use a probe that "
            "needs no activate/deactivate g-code",
        )
    return CheckResult("probe gcode", "pass", "activate/deactivate g-code empty", "")


def z_position_min(config):
    """The configured lower Z bound as ``(value, source)``.

    Mirrors the spirit of klippy/extras/probe.py:188-193
    ``lookup_minimum_z``: prefer the Z endstop section's position_min,
    else [printer] minimum_z_position; ``(None, None)`` when neither
    exists.  Simplification vs klippy: we read [stepper_z] directly
    instead of resolving which stepper owns the Z endstop (klippy walks
    endstop_pin via manual_probe.lookup_z_endstop_config); for multi-Z
    gantries all stepper_z* share [stepper_z]'s endstop config, so this
    matches in practice and is documented as a heuristic.  Shared by
    the commissioning check below and the drag oracle's Z-floor
    computation (plugin.__init__ caches it; drag_probe consumes it).
    """
    if config.has_section("stepper_z"):
        value = config.getsection("stepper_z").getfloat("position_min", None)
        if value is not None:
            return value, "[stepper_z] position_min"
    if config.has_section("printer"):
        value = config.getsection("printer").getfloat("minimum_z_position", None)
        if value is not None:
            return value, "[printer] minimum_z_position"
    return None, None


def check_z_position_min(config):
    """A finite lower Z bound must exist for the probing descent."""
    value, source = z_position_min(config)
    if value is None:
        return CheckResult(
            "z position_min",
            "fail",
            "no [stepper_z] position_min and no [printer] minimum_z_position",
            "set position_min in [stepper_z] so the recovery probe has a hard floor",
        )
    if not math.isfinite(value):
        return CheckResult(
            "z position_min",
            "fail",
            "%s is not finite (%r)" % (source, value),
            "set a finite position_min",
        )
    return CheckResult("z position_min", "pass", "%s = %g" % (source, value), "")


def check_recorder_heartbeat(wal_dir, now=None):
    """Recorder heartbeat file must exist and be fresh.

    plrd rewrites ``<wal_dir>/heartbeat.bin`` at 10 Hz; an mtime within
    HEARTBEAT_FRESH_SECS of now means a recorder process was recently
    alive.  LIVENESS HINT ONLY — it does not prove the WAL is durable,
    complete, or even for this print; plrd re-proves all of that from
    the WAL contents at recovery time.
    """
    path = os.path.join(wal_dir, "heartbeat.bin")
    try:
        mtime = os.stat(path).st_mtime
    except OSError:
        return CheckResult(
            "recorder heartbeat",
            "fail",
            "%s missing or unreadable" % (path,),
            "is plrd running? (systemctl status plrd); check wal_dir in [plr]",
        )
    if now is None:
        now = time.time()
    age = now - mtime
    if age > HEARTBEAT_FRESH_SECS:
        return CheckResult(
            "recorder heartbeat",
            "fail",
            "%s is stale (%.1fs old, want < %.1fs)" % (path, age, HEARTBEAT_FRESH_SECS),
            "plrd recorder is not writing — systemctl status plrd",
        )
    return CheckResult(
        "recorder heartbeat",
        "pass",
        "fresh (%.1fs old) — liveness hint only, not a durability proof" % (age,),
        "",
    )


def list_accel_chips(config):
    """Informational: accel-chip sections present in the config.

    Matches both the bare and the named section forms klippy registers
    (klippy/extras/adxl345.py:313-320 load_config/load_config_prefix,
    giving section names like "adxl345" and "adxl345 bed").
    """
    found = []
    for prefix in ACCEL_CHIP_PREFIXES:
        for section in config.get_prefix_sections(prefix):
            name = section.get_name()
            # A prefix scan for "adxl345" must not claim e.g. a
            # hypothetical "adxl345x" section: accept only the exact
            # name or "<prefix> <suffix>".
            if name == prefix or name.startswith(prefix + " "):
                found.append(name)
    if not found:
        return CheckResult(
            "accel chips",
            "warn",
            "none detected",
            "only needed for probe_method adxl_drag / resonance tooling",
        )
    return CheckResult("accel chips", "pass", ", ".join(sorted(found)), "")


def run_static_checks(config, probe_method, accel_chip):
    """All config-time checks, in report order."""
    return [
        check_force_move(config),
        check_probe_section(config, probe_method, accel_chip),
        check_z_steppers_on_primary_mcu(config),
        check_probe_gcode_empty(config, probe_method),
        check_z_position_min(config),
        list_accel_chips(config),
    ]


def format_report(results, attested, probe_method):
    """Render the commissioning report for the console."""
    marker = {"pass": "[PASS]", "warn": "[WARN]", "fail": "[FAIL]"}
    lines = ["PLR commissioning report (probe_method=%s)" % (probe_method,)]
    for res in results:
        lines.append("%s %s: %s" % (marker[res.verdict], res.name, res.detail))
        if res.verdict != "pass" and res.hint:
            lines.append("       hint: %s" % (res.hint,))
    if attested:
        lines.append("[PASS] self_locking_z: attested by operator")
    else:
        lines.append(
            "[FAIL] self_locking_z: not attested — if (and only if) your Z "
            "axis holds position unpowered (e.g. leadscrews), run "
            "PLR_SETUP ACCEPT_SELF_LOCKING_Z=1 then SAVE_CONFIG"
        )
    overall = classifier.worst_verdict([r.verdict for r in results])
    if overall != "fail" and attested:
        lines.append("Overall: COMMISSIONED — plr is ready to protect prints")
    elif overall != "fail":
        lines.append("Overall: NOT COMMISSIONED (attestation missing)")
    else:
        lines.append("Overall: NOT COMMISSIONED (failed checks above)")
    return "\n".join(lines)


_CAL_REMEDIATION = {
    calibration_meta.GROUP_NOISE_FLOOR: "re-run PLR_NOISE_TEST",
    calibration_meta.GROUP_PROBE_RESOLUTION: "re-run PLR_PROBE_TEST",
}


def calibration_check_results(plugin):
    """Commissioning rows for the persisted-calibration validity (one per
    value-group that carries a value): VALID -> pass, LEGACY -> warn,
    INVALID -> fail with the old-vs-new fingerprint in the detail."""
    results = []
    for group in calibration_meta.GROUPS:
        result = plugin.calibrations[group]
        if result.tier == calibration_meta.TIER_UNSET:
            continue
        name = "calibration:%s" % (group,)
        if result.tier == calibration_meta.TIER_VALID:
            results.append(
                CheckResult(
                    name,
                    "pass",
                    "stamped, fingerprint %s current" % (result.stored_fingerprint,),
                    "",
                )
            )
        elif result.tier == calibration_meta.TIER_LEGACY:
            results.append(
                CheckResult(
                    name,
                    "warn",
                    "no fingerprint stamp (calibrated before stamping)",
                    "%s to stamp it; a future version may refuse unstamped values"
                    % (_CAL_REMEDIATION[group],),
                )
            )
        else:
            results.append(
                CheckResult(
                    name,
                    "fail",
                    "stale — staged fingerprint %s, current %s (ignored)"
                    % (result.stored_fingerprint, result.current_fingerprint),
                    _CAL_REMEDIATION[group],
                )
            )
    return results


def cmd_PLR_SETUP(plugin, gcmd):
    """PLR_SETUP [ACCEPT_SELF_LOCKING_Z=1] — commissioning report."""
    if gcmd.get_int("ACCEPT_SELF_LOCKING_Z", 0):
        configfile = plugin.printer.lookup_object("configfile")
        # Staged into the [plr] autosave block; config.getboolean reads
        # it back on the post-SAVE_CONFIG restart (configparser accepts
        # "True"; klippy/configfile.py:73-75 getboolean).
        configfile.set("plr", "self_locking_z", "True")
        plugin.self_locking_z = True
        gcmd.respond_info(
            "plr: self_locking_z attestation staged.\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "and restart the printer."
        )
        return
    results = list(plugin.static_check_results)
    results.append(check_recorder_heartbeat(plugin.wal_dir))
    # Persisted-calibration validity rows (fingerprint/version stamps): a
    # stale calibration reports [FAIL] with the old-vs-new fingerprint.
    results.extend(calibration_check_results(plugin))
    # Nozzle-cleanliness mode row (auto-macro vs manual wizard confirm).
    results.append(clean_nozzle_check_result(plugin))
    gcmd.respond_info(
        format_report(results, plugin.self_locking_z, plugin.probe_method)
    )


# ---------------------------------------------------------------------
# Optional-dependency helpers (numpy is optional inside klippy; present
# on installs that use input_shaper/resonance tooling).  Used by the
# drag-oracle diagnostics.

NUMPY_HINT = (
    "numpy is required for this command but is not installed in the "
    "klippy environment; install it the same way Klipper's resonance "
    "tooling does (e.g. ~/klippy-env/bin/pip install numpy) and restart "
    "Klipper"
)


def numpy_available():
    """Return True if numpy can be imported in this environment."""
    return importlib.util.find_spec("numpy") is not None


def require_numpy(error_type=RuntimeError):
    """Raise ``error_type`` with a clear install hint if numpy is absent.

    ``error_type`` lets callers raise klippy's command error (e.g.
    ``gcode.error``) so the message reaches the console, not the log.
    """
    if not numpy_available():
        raise error_type(NUMPY_HINT)
