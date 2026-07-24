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

from . import classifier

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


def check_z_position_min(config):
    """A finite lower Z bound must exist for the probing descent.

    Mirrors the spirit of klippy/extras/probe.py:188-193
    ``lookup_minimum_z``: prefer the Z endstop section's position_min,
    else [printer] minimum_z_position.  Simplification vs klippy: we
    read [stepper_z] directly instead of resolving which stepper owns
    the Z endstop (klippy walks endstop_pin via
    manual_probe.lookup_z_endstop_config); for multi-Z gantries all
    stepper_z* share [stepper_z]'s endstop config, so this matches in
    practice and is documented as a heuristic.
    """
    value = None
    source = None
    if config.has_section("stepper_z"):
        value = config.getsection("stepper_z").getfloat("position_min", None)
        source = "[stepper_z] position_min"
    if value is None and config.has_section("printer"):
        value = config.getsection("printer").getfloat("minimum_z_position", None)
        source = "[printer] minimum_z_position"
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
