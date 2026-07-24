"""Runtime-tunable [plr] parameters: schema, validation, and PLR_SET.

Single source of truth for every float tunable in the ``[plr]`` section:
:data:`TUNABLES` drives both the config-time parse (so klippy rejects a
bad printer.cfg with its usual "Option ... in section ..." error, see
klippy/configfile.py:29-60 ``_get_wrapper``) and the runtime ``PLR_SET``
command (so the console enforces exactly the same ranges).

``PLR_SET PARAM=<name> VALUE=<v>`` applies the value to the live plugin
object immediately AND stages it for persistence via
``configfile.set('plr', name, value)`` — klippy's SAVE_CONFIG autosave
mechanism (klippy/configfile.py:311-324 ``PrinterConfig.set`` stages the
value; klippy/configfile.py:345-380 ``cmd_SAVE_CONFIG`` rewrites the
config and restarts).  Until the user runs SAVE_CONFIG the change is
live-but-volatile, which the no-argument listing marks explicitly.
"""

import collections

# One tunable's schema.  Bounds mirror klippy's ConfigWrapper vocabulary
# (klippy/configfile.py:29-60): ``minval``/``maxval`` are inclusive,
# ``above`` is exclusive.  Exactly one of minval/above is set per spec.
TunableSpec = collections.namedtuple(
    "TunableSpec", ["name", "default", "minval", "maxval", "above", "unit", "desc"]
)

# Ordered so listings and error messages are stable.  The keys, defaults
# and ranges are the FIXED [plr] schema shared with the plrd daemon —
# renaming a key here breaks the Rust side.
TUNABLES = collections.OrderedDict(
    (spec.name, spec)
    for spec in [
        TunableSpec(
            "probe_speed", 1.5, 1.0, 2.0, None, "mm/s", "recovery probe descent speed"
        ),
        TunableSpec(
            "envelope_margin",
            0.5,
            0.0,
            None,
            None,
            "mm",
            "extra clearance added around the reconstructed part envelope",
        ),
        TunableSpec(
            "sag_allowance",
            0.2,
            0.0,
            None,
            None,
            "mm",
            "expected unpowered Z sag budget when matching the stop point",
        ),
        TunableSpec(
            "drag_speed",
            20.0,
            None,
            100.0,
            0.0,
            "mm/s",
            "lateral speed of the drag-oracle pass",
        ),
        TunableSpec(
            "drag_z_step",
            0.05,
            None,
            0.2,
            0.0,
            "mm",
            "Z staircase decrement between drag passes",
        ),
        TunableSpec(
            "drag_sensitivity",
            30.0,
            0.0,
            100.0,
            None,
            "",
            "0-100 knob; drag threshold multiplier over the measured noise floor",
        ),
        TunableSpec(
            "exclusion_radius",
            5.0,
            0.0,
            None,
            None,
            "mm",
            "radius around the probed contact kept out of the resume path",
        ),
        TunableSpec(
            "entry_feedrate",
            1800.0,
            None,
            1800.0,
            0.0,
            "mm/min",
            "feedrate cap for the re-entry approach move",
        ),
    ]
)


def range_text(spec):
    """Human-readable range for console errors and the PLR_SET listing."""
    if spec.above is not None and spec.maxval is not None:
        return "(%g, %g]" % (spec.above, spec.maxval)
    if spec.minval is not None and spec.maxval is not None:
        return "[%g, %g]" % (spec.minval, spec.maxval)
    if spec.minval is not None:
        return ">= %g" % (spec.minval,)
    # Unreachable with the current table; kept so a future spec without
    # bounds still renders something honest.
    return "(unbounded)"


def load_from_config(config):
    """Parse every tunable from the [plr] section with schema ranges.

    Uses config.getfloat's minval/maxval/above so a bad printer.cfg
    fails config parsing with klippy's standard error text
    (klippy/configfile.py:48-59) instead of a plugin-specific one.
    Returns an OrderedDict name -> float.
    """
    values = collections.OrderedDict()
    for name, spec in TUNABLES.items():
        values[name] = config.getfloat(
            name,
            spec.default,
            minval=spec.minval,
            maxval=spec.maxval,
            above=spec.above,
        )
    return values


def validate(name, raw_value):
    """Validate one PLR_SET assignment; return the parsed float.

    Raises ValueError with a console-ready message on an unknown name,
    an unparsable value, or a value outside the schema range.  The
    caller re-raises through gcmd.error so the message reaches the
    console (klippy/gcode.py:24-25: GCodeCommand.error is CommandError).
    """
    spec = TUNABLES.get(name)
    if spec is None:
        raise ValueError("unknown PARAM=%s (valid: %s)" % (name, ", ".join(TUNABLES)))
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        raise ValueError(
            "unable to parse VALUE=%s as a number" % (raw_value,)
        ) from None
    in_range = True
    if spec.minval is not None and value < spec.minval:
        in_range = False
    if spec.maxval is not None and value > spec.maxval:
        in_range = False
    if spec.above is not None and value <= spec.above:
        in_range = False
    if not in_range:
        raise ValueError(
            "VALUE=%g out of range for %s (valid range: %s %s)"
            % (value, name, range_text(spec), spec.unit or "unitless")
        )
    return value


def format_value(value):
    """Canonical string form staged into the autosave section.

    klippy stringifies whatever configfile.set receives
    (klippy/configfile.py:314 ``svalue = str(value)``); formatting here
    keeps the persisted file free of float repr noise.
    """
    return "%.6f" % (value,)


def cmd_PLR_SET(plugin, gcmd):
    """PLR_SET [PARAM=<name> VALUE=<v>] — set or list runtime tunables."""
    name = gcmd.get("PARAM", None)
    raw = gcmd.get("VALUE", None)
    if name is None and raw is None:
        _respond_listing(plugin, gcmd)
        return
    if name is None or raw is None:
        raise gcmd.error("PLR_SET needs both PARAM= and VALUE= (or neither to list)")
    name = name.lower()
    try:
        value = validate(name, raw)
    except ValueError as e:
        raise gcmd.error("PLR_SET: %s" % (e,)) from None
    plugin.tunables[name] = value
    configfile = plugin.printer.lookup_object("configfile")
    configfile.set("plr", name, format_value(value))
    plugin.note_pending_save(name)
    gcmd.respond_info(
        "plr: %s = %g %s (live now; run SAVE_CONFIG to persist across restarts)"
        % (name, value, TUNABLES[name].unit)
    )


def _respond_listing(plugin, gcmd):
    lines = ["PLR tunables (PLR_SET PARAM=<name> VALUE=<v>):"]
    for name, spec in TUNABLES.items():
        pending = " [awaiting SAVE_CONFIG]" if plugin.is_pending_save(name) else ""
        lines.append(
            "  %s = %g %s  range %s%s — %s"
            % (
                name,
                plugin.tunables[name],
                spec.unit or "(unitless)",
                range_text(spec),
                pending,
                spec.desc,
            )
        )
    gcmd.respond_info("\n".join(lines))


def clamp(value, low, high):
    """Clamp ``value`` into the inclusive range [low, high].

    Raises ValueError if the range itself is inverted — that is a
    programming error in a tunable definition, not user input.
    """
    if low > high:
        raise ValueError("clamp: low %r is greater than high %r" % (low, high))
    return max(low, min(high, value))
