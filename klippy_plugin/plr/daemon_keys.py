"""``[plr]`` options that plrd consumes and this plugin only DECLARES.

============================================================================
DO NOT DELETE A READ IN THIS MODULE BECAUSE "NOTHING USES THE VALUE".
Every read here is load-bearing for BOOT.  Removing one makes klippy
REFUSE TO START for any operator who set that option in printer.cfg.
============================================================================

WHY.  Klipper tracks which options a module actually read, and rejects the
whole config if an option in a configured section went unclaimed:

* ``ConfigWrapper._get_wrapper`` (klippy/configfile.py:29-60) records
  ``access_tracking[(section.lower(), option.lower())] = value`` for a
  PRESENT option (line 46-47), and — only when the supplied default is
  not ``None`` — for an ABSENT one too (lines 31-36).
* ``ConfigValidate.check_unused`` (klippy/configfile.py:424-441) walks
  every option of every configured section and
  ``raise error("Option '%s' is not valid in section '%s'")`` for any
  ``(section, option)`` missing from that map.  klippy calls it from
  ``Klippy._read_config`` (klippy/klippy.py:127) during startup, so the
  failure is a printer that will not boot, not a warning.
* ``ConfigValidate._build_status_settings`` (klippy/configfile.py:447-450)
  builds the ``configfile.settings`` status map from THE SAME access map,
  which is where plrd reads the ``[plr]`` section from
  (``crates/plrd/src/plrcfg.rs`` ``PlrSettings::parse``).  So the read is
  also what makes the operator's value REACH the daemon at all:
  ``get_prefix_options`` does NOT register access
  (klippy/configfile.py:127-129), and neither does anything else.

One read therefore buys both halves: klippy accepts the section, and plrd
sees the value.  Without it, an option plrd documents is unreachable AND
fatal.  A test pins this: ``tests/test_daemon_keys.py`` derives the key
set from plrd's own parser and fails if any consumed key stops being read.

WHY THE DEFAULT IS ALWAYS ``None`` (and why that is not laziness).
``_get_wrapper`` lines 31-36 register the DEFAULT into the access map when
an option is absent and the default is not ``None``.  A plugin-side
default would therefore be published into ``configfile.settings`` as
though the operator had written it, and plrd would read the PLUGIN's
default instead of its own (``PlanConfig::default()``).  Two defaults that
can drift is the same class of defect as two validators that can disagree.
With ``None``, an absent option registers nothing, never reaches
``settings``, and plrd applies its own default — one authority per value.

VALIDATION BOUNDARY: LOOSE HERE, STRICT IN plrd.  ONE VALIDATOR PER RULE.

What this module checks is exactly "is this the type plrd can read, and is
it representable":

1. the type plrd expects (klippy's own ``getfloat`` / ``getboolean`` /
   ``get`` parser, so a malformed value fails with klippy's standard
   "Unable to parse option ..." text at config time), and
2. finiteness for floats — ``float("nan")`` and ``float("inf")`` parse
   happily, and NaN/Infinity have no JSON encoding, so they would break
   the API-socket payload plrd reads the whole section from rather than
   producing a diagnosable value.

What this module deliberately does NOT check is RANGES — no bands, no
minima, no maxima, and (see below) no sign rules either.  plrd is the
single authority on every band: ``[50, 20000]`` for the accel overrides,
``[30, 3600]`` for ``confirm_timeout_s``, ``(0, 0.015]`` for
``touch_sample_range``, and so on, enforced in
``crates/plr-recovery`` ``PlanConfig::validate`` where an out-of-band
value REFUSES recovery with a structured diagnosis naming the key.

That is a deliberate choice, and it is scar tissue.  This project has
already shipped a deadlock built out of two components enforcing the same
limit from opposite directions: the plugin's contact-temperature CEILING
and the plan's COMMANDED probe temperature met at exactly the same
number, so the daemon's own probe command was refused by the plugin's
gate — a recovery that could not proceed and could not be retried,
because the refusal landed after the Z frame was declared.  Nothing was
wrong with either check in isolation.  The defect was that the same rule
had two enforcers that could disagree.  So: one validator plus a
permissive reader, never two validators.

No sign checks either, and ``purge_z`` is why.  A negative ``purge_z``
drives a hot nozzle into the bed, which is the single hardest refusal in
the system — and also the ONE refusal with an escape hatch
(``UNSAFE_allow_purge_z_below_bed``).  The value whose sign matters most
is precisely the value that MUST reach plrd with its sign intact so plrd
can adjudicate it against the override.  Once one signed coordinate is
exempt, "obviously positive" stops being obvious for the rest
(``reheat_park_x/y`` and ``purge_x/y`` are machine coordinates that can be
negative on a real printer), and every remaining sign rule is a policy
edge that coincides with a plrd band edge.  A uniform "finite number of
the right type" rule has no edges to collide with, and
``test_daemon_keys.py`` asserts that no read here passes a bound to
klippy, so the boundary cannot erode by accident.

The consequence is intended and worth stating plainly: an out-of-band
value in ``[plr]`` boots the printer and is refused later, by plrd, with
an explanation naming the key.  ``get_status()['daemon_config']`` exposes
exactly what the daemon will see so a report command can surface it
before a recovery is attempted.
"""

import collections
import math

# Value kinds.  Each maps to the klippy getter whose parsed type matches
# what plrd's parser expects from `configfile.settings.plr`
# (crates/plrd/src/plrcfg.rs: `opt_f64`/`opt_opt_f64` want a JSON number,
# `opt_bool` a JSON bool, `opt_str` a JSON string).  Reading a key with
# the wrong kind here would publish a type plrd hard-errors on.
FLOAT = "float"
BOOLEAN = "boolean"
STRING = "string"

# The declared surface: every `[plr]` option plrd consumes that no other
# part of this plugin reads for its own purposes.  Ordered so listings and
# `daemon_config` are stable.
#
# ADDING A KEY: when plrd starts consuming a new `[plr]` option,
# tests/test_daemon_keys.py goes RED until it is added here.  That is the
# guard working, not a broken test.
#
# REMOVING A KEY: only when plrd stops consuming it.  See the module
# banner — an unused-looking read is a boot requirement.
#
# `touch_samples` is a FLOAT, not an int, deliberately: plrd's field is
# `f64` (`opt_f64(plr, "touch_samples", ...)`), and reading it as an int
# here would put klippy's int in `settings` while plrd's band `[3, 7]`
# does the integrality talking.  Mirror the consumer's type.
DAEMON_KEYS = collections.OrderedDict(
    [
        # --- consensus touch (plrd bands: [3,7] / (0,0.015] / >=1.0 /
        # [50,1000]) ---
        ("touch_samples", FLOAT),
        ("touch_sample_range", FLOAT),
        ("touch_retract", FLOAT),
        ("touch_accel", FLOAT),
        # --- reheat park + pre-home lift ---
        ("reheat_park_x", FLOAT),
        ("reheat_park_y", FLOAT),
        ("reheat_park_delta_z", FLOAT),
        ("pre_home_z_lift", FLOAT),
        # --- purge (the three coherent paths live in plrd) ---
        ("purge_enable", BOOLEAN),
        ("purge_macro", STRING),
        ("purge_amount", FLOAT),
        ("purge_x", FLOAT),
        ("purge_y", FLOAT),
        ("purge_z", FLOAT),
        ("purge_speed", FLOAT),
        ("purge_retract", FLOAT),
        # --- drag thermal state ---
        ("drag_nozzle_temp", FLOAT),
        # --- acceleration overrides (plrd band [50,20000] each) ---
        ("recovery_accel", FLOAT),
        ("accel_home", FLOAT),
        ("accel_travel", FLOAT),
        ("accel_probe", FLOAT),
        ("accel_entry", FLOAT),
        # --- confirm-points ---
        ("confirm_z_before_resume", BOOLEAN),
        ("debug_confirm_each_step", BOOLEAN),
        ("confirm_timeout_s", FLOAT),
        # The system's ONE hard-refusal escape hatch.  Spelled exactly as
        # the documentation spells it: klippy's config parser is a
        # configparser.RawConfigParser (klippy/configfile.py:170-176) with
        # the default `optionxform`, so FILE option names are lowercased
        # on read and a lookup by either spelling finds the same option.
        # `access_tracking` is keyed lowercased regardless
        # (klippy/configfile.py:34,47), so plrd receives
        # `unsafe_allow_purge_z_below_bed` — which is why its reader
        # accepts both spellings (`plrcfg.rs` `unsafe_flag`).  Declaring
        # it in the documented mixed case keeps this file readable against
        # the docs and works identically.
        ("UNSAFE_allow_purge_z_below_bed", BOOLEAN),
    ]
)


def load_from_config(config):
    """Declare every key in :data:`DAEMON_KEYS`; return name -> value.

    ``None`` means "the operator did not set it", which is materially
    different from any concrete value: the option then never enters
    ``configfile.settings`` and plrd applies its own default.  See the
    module docstring for why no plugin-side default is supplied and why
    no bound is passed to klippy's getters.
    """
    values = collections.OrderedDict()
    for name, kind in DAEMON_KEYS.items():
        if kind == FLOAT:
            # No minval/maxval/above/below on purpose: plrd owns every
            # band (module docstring, "VALIDATION BOUNDARY").
            value = config.getfloat(name, None)
            if value is not None and not math.isfinite(value):
                raise config.error(
                    "Option '%s' in section '%s' must be finite"
                    % (name, config.get_name())
                )
        elif kind == BOOLEAN:
            value = config.getboolean(name, None)
        else:
            value = config.get(name, None)
        values[name] = value
    return values
