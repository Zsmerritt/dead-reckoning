"""Version/geometry fingerprinting and three-tier validation of the
persisted calibration values.

Ported from Cartographer3D's stale-model defense
(``src/cartographer/config/model_validator.py`` — the three-tier
``_is_model_compatible`` at 30-63, ``validate_and_remove_incompatible_models``
at 66-96 — and the save-time version stamping in
``adapters/klipper/configuration.py:70-116`` which REFUSES to save when the
identity source is unavailable).  The pattern, not the code.

Carto stamps every saved model with the plugin ``software_version`` and the
probe ``mcu_version``, and at ready-time either removes an incompatible model
(hard mismatch), warns for a pre-stamping model, or accepts it.  Our
persisted calibration is a handful of ``[plr]`` autosave options rather than a
named model, so:

* the identity a model change would invalidate becomes a **fingerprint** — a
  short stable hash over the CANONICALIZED, calibration-relevant slice of the
  config (the ``stepper_z*`` kinematics, the active probe/accel-chip section,
  and the ``[plr]`` hardware-selection keys).  Any change to that slice changes
  the hash; unrelated config churn (``[fan]``, ``[display]``) does not;
* two independent **value-groups** carry independent fingerprints, so a stale
  noise floor does not invalidate a still-good probe_resolution and vice
  versa (the two depend on different config: probe_resolution on the touch
  probe section, the noise floor on the accel-chip section);
* Klipper cannot delete an autosave option programmatically in any supported
  way (there is no ``configfile.remove`` reachable from a plugin), so carto's
  "remove the incompatible model" becomes **treat-as-absent**: an INVALID
  group's live values are nulled on the plugin so every consumer — the drag
  commands, ``get_status``, and plrd through the API socket — sees an
  uncalibrated machine.  This is a documented divergence from carto: the stale
  option text stays in ``printer.cfg`` until the operator re-runs the
  calibration (which overwrites it), we just never trust it.

Fingerprint canonicalization (the exact spec — the Rust side in
``crates/plrd/src/plrcfg.rs`` ports it byte-for-byte, pinned by shared
literal-hash fixtures):

1. Take the group's relevant sections (see ``_relevant_section_names``): every
   ``stepper_z*`` section plus, for the probe_resolution group, the active
   touch-probe section (``[probe]`` for tap / ``[load_cell_probe]`` for
   load_cell), and for the noise-floor group the configured accel-chip
   section.
2. Sort the section names; within each section, sort the option keys.
3. Append a synthetic ``[plr]`` block carrying only the group's
   hardware-selection keys (``probe_method``; plus ``accel_chip`` for the
   noise-floor group), sorted — NOT the whole ``[plr]`` section (its tunables
   and the calibration values themselves must never feed their own
   fingerprint).
4. Normalize every value: collapse all whitespace runs to a single space and
   strip the ends, then, when the result parses as a finite number, re-emit it
   in a canonical numeric form (integer-valued floats without a decimal
   point).  This is what lets the Python side (which reads raw config strings)
   and the Rust side (which reads Klipper's typed ``configfile.settings``)
   agree: ``"-2"`` and the JSON number ``-2.0`` both canonicalize to ``-2``.
5. Serialize as ``[section]\\n key=value\\n ...`` and hash with CRC-32
   (``zlib.crc32``, IEEE 802.3 reflected poly ``0xEDB88320``), rendered as
   eight lowercase hex digits.  CRC-32 rather than a truncated SHA-256 purely
   so the Rust port needs no third-party crate (a ~10-line reflected CRC-32 in
   plrcfg.rs reproduces ``zlib.crc32`` exactly); collision resistance is not a
   security property here, only change-detection, for which CRC-32 is ample.

Source-compatibility: Python 3.7 (runs inside klippy).
"""

import collections
import logging
import math
import re
import zlib

logger = logging.getLogger(__name__)

# --- value groups -----------------------------------------------------------
GROUP_NOISE_FLOOR = "noise_floor"
GROUP_PROBE_RESOLUTION = "probe_resolution"
GROUPS = (GROUP_NOISE_FLOOR, GROUP_PROBE_RESOLUTION)

# --- stamp option names (staged into the [plr] autosave block) --------------
PLUGIN_VERSION_KEY = "cal_plugin_version"
KLIPPER_VERSION_KEY = "cal_klipper_version"


def fingerprint_key(group):
    """The per-group fingerprint autosave option name."""
    return "cal_fingerprint_%s" % (group,)


# --- validation tiers -------------------------------------------------------
TIER_VALID = "valid"
TIER_LEGACY = "legacy"
TIER_INVALID = "invalid"
TIER_UNSET = "unset"

# Per-group [plr] hardware-selection keys folded into the fingerprint.
# probe_resolution deliberately excludes accel_chip (it does not depend on the
# accelerometer); the noise floor deliberately includes it.
_PLR_KEYS = {
    GROUP_PROBE_RESOLUTION: ("probe_method",),
    GROUP_NOISE_FLOOR: ("accel_chip", "probe_method"),
}

# The touch-probe section that backs each descending probe method.  adxl_drag
# has no touch-probe section (and never stages probe_resolution), so it maps to
# nothing.
_PROBE_SECTION_BY_METHOD = {"tap": "probe", "load_cell": "load_cell_probe"}

_STEPPER_Z_RE = re.compile(r"^stepper_z[0-9]*$")

# The outcome of validating one value-group.  ``reasons`` is empty unless the
# tier is LEGACY or INVALID.  ``stored_fingerprint`` / ``current_fingerprint``
# are surfaced so PLR_SETUP can show the old-vs-new hashes on a mismatch.
GroupValidation = collections.namedtuple(
    "GroupValidation",
    ["group", "tier", "reasons", "stored_fingerprint", "current_fingerprint"],
)


def plugin_version():
    """The single-source plugin version (``plr.__version__``).

    A bare ``import plr`` only resolves under pytest, where the package is
    also importable as a top-level ``plr`` (see the pytest path config);
    under real klippy the package is loaded as ``extras.plr`` and there is
    no top-level ``plr`` module at all, so that import raises
    ``ModuleNotFoundError``.  A relative import names the parent package
    however it was actually loaded, so it works under both.  It stays
    lazy (imported inside the function rather than at module scope) to
    avoid an import cycle (``plr/__init__`` imports ``plugin`` which
    imports this module during package initialization)."""
    from . import __version__

    return __version__


# --- canonicalization -------------------------------------------------------


def _canonical_number(value):
    """A canonical decimal string for a finite float, else ``None``.

    Integer-valued floats within the exact-integer range render without a
    decimal point (``-2.0`` -> ``"-2"``); other finite floats use Python's
    shortest round-tripping ``repr`` (which matches Rust's ``f64`` ``Display``
    for the clean decimals real configs carry).  Non-finite / non-numeric
    input returns ``None`` so the caller keeps the original text.
    """
    if not math.isfinite(value):
        return None
    if abs(value) < 1e15 and value == int(value):
        return str(int(value))
    return repr(value)


def _normalize_value(value):
    """Whitespace-collapse, then numeric-canonicalize a config value.

    The numeric step is what makes the Python (raw string) and Rust (typed
    ``configfile.settings``) canonicalizations agree on the same machine."""
    text = " ".join(str(value).split())
    try:
        number = float(text)
    except (TypeError, ValueError):
        return text
    canonical = _canonical_number(number)
    return text if canonical is None else canonical


def _canonical_string(sections, section_names, plr_keys):
    """The canonical serialization of the relevant config slice (see the
    module docstring for the exact grammar)."""
    parts = []
    for name in sorted(section_names):
        body = sections.get(name)
        if body is None:
            continue
        parts.append("[%s]" % (name,))
        for key in sorted(body):
            parts.append("%s=%s" % (key, _normalize_value(body[key])))
    parts.append("[plr]")
    plr_body = sections.get("plr") or {}
    for key in sorted(plr_keys):
        if key in plr_body:
            parts.append("%s=%s" % (key, _normalize_value(plr_body[key])))
    return "\n".join(parts)


def _crc32_hex(text):
    return "%08x" % (zlib.crc32(text.encode("utf-8")) & 0xFFFFFFFF)


def fingerprint(sections, section_names, plr_keys):
    """The low-level fingerprint over an explicit section/key selection.

    ``sections`` is a ``{section_name: {option: value}}`` mapping.  This is the
    surface the cross-language literal-hash fixtures pin (Python and Rust feed
    identical string-valued maps and assert the same hex)."""
    return _crc32_hex(_canonical_string(sections, section_names, plr_keys))


def _relevant_section_names(sections, group):
    """The section names that feed ``group``'s fingerprint, derived from the
    config's ``[plr]`` selection."""
    names = [name for name in sections if _STEPPER_Z_RE.match(name)]
    plr = sections.get("plr") or {}
    method = plr.get("probe_method")
    method = "tap" if method is None else str(method).strip()
    if group == GROUP_PROBE_RESOLUTION:
        probe_section = _PROBE_SECTION_BY_METHOD.get(method)
        if probe_section:
            names.append(probe_section)
    else:
        accel = plr.get("accel_chip")
        if accel:
            names.append(str(accel).strip())
    return names


def compute_fingerprint(sections, group):
    """Group fingerprint over a ``{section: {option: value}}`` mapping.

    The high-level entry point: both the Python config adapter
    (``fingerprint_from_config``) and the Rust ``machine_from_settings`` build
    such a mapping from their native config view and call the equivalent of
    this."""
    return fingerprint(
        sections, _relevant_section_names(sections, group), _PLR_KEYS[group]
    )


# --- ConfigWrapper adapter (staging + validation time) ----------------------


def _config_to_sections(config):
    """Extract the calibration-relevant sections from a klippy ConfigWrapper.

    Reads every option of each relevant section via
    ``get_prefix_options("")`` + ``get(..., note_valid=False)`` (the
    ConfigWrapper API, klippy/configfile.py:61-63,127-129).

    ``note_valid=False`` MATTERS AND IS NOT AN OPTIMIZATION.  A plain
    ``get`` records the option's RAW STRING into klippy's access map
    (klippy/configfile.py:46-47), and that map is what becomes
    ``configfile.settings`` — the typed view plrd parses the ``[plr]``
    section out of.  Enumerating ``[plr]`` for the hash therefore used to
    OVERWRITE every already-recorded typed value with its string form, and
    plrd hard-errors on a string where it expects a number or a bool
    (``crates/plrd/src/plrcfg.rs`` ``opt_f64`` / ``opt_bool``): on a
    stamped-calibration machine that reached this code, a configured
    ``purge_amount`` made plrd refuse the whole section, and
    ``UNSAFE_allow_purge_z_below_bed`` arrived as ``"True"`` — a bool read
    of which is ``None``, i.e. the one escape hatch in the system failing
    silently CLOSED.  Hashing must not publish values.

    Suppressing the record is safe for klippy's unused-option check
    (klippy/configfile.py:424-441): the ``[plr]`` options are all claimed
    explicitly by the plugin (``plr/plugin.py``, ``plr/tunables.py``,
    ``plr/daemon_keys.py``) and autosaved options are exempt anyway
    (klippy/configfile.py:426-427), while the other sections enumerated
    here belong to klippy's own modules, which claim them themselves."""
    sections = {}
    for wrapper in config.get_prefix_sections("stepper_z"):
        name = wrapper.get_name()
        if _STEPPER_Z_RE.match(name):
            sections[name] = _section_options(wrapper)
    plr = config.getsection("plr")
    sections["plr"] = _section_options(plr)
    method = plr.get("probe_method", "tap")
    candidates = [_PROBE_SECTION_BY_METHOD.get(method), plr.get("accel_chip", None)]
    for name in candidates:
        if name and name not in sections and config.has_section(name):
            sections[name] = _section_options(config.getsection(name))
    return sections


def _section_options(wrapper):
    # note_valid=False: a hash input is not a claim on the option, and must
    # not overwrite the typed value in configfile.settings.  See
    # _config_to_sections' docstring.
    return {
        option: wrapper.get(option, note_valid=False)
        for option in wrapper.get_prefix_options("")
    }


def fingerprint_from_config(config, group):
    """Compute ``group``'s fingerprint from a live klippy ConfigWrapper."""
    return compute_fingerprint(_config_to_sections(config), group)


# --- version comparison -----------------------------------------------------


def _major_minor(version):
    """The ``(major, minor)`` of a version string, or ``None`` if unparseable.

    Tolerant of a leading ``v`` and of trailing pre-release / git suffixes
    (``"v0.13.0-462-g7046bd00e"`` -> ``(0, 13)``), matching carto's
    ``meets_minimum_version`` regex approach (model_validator.py:18-27)."""
    if version is None:
        return None
    match = re.match(r"^\s*v?(\d+)\.(\d+)", str(version))
    if not match:
        return None
    return (int(match.group(1)), int(match.group(2)))


def is_version_regression(stored, running):
    """True when the running plugin ``major.minor`` is BELOW the version the
    calibration was staged under (a downgrade the calibration may predate).

    Unparseable versions on either side are treated as "no regression"
    (tolerant): the fingerprint is the primary guard, the version check only
    catches an explicit backslide."""
    stored_mm = _major_minor(stored)
    running_mm = _major_minor(running)
    if stored_mm is None or running_mm is None:
        return False
    return running_mm < stored_mm


# --- three-tier validation --------------------------------------------------


def validate_group(
    config,
    group,
    value_present,
    stored_fingerprint,
    stored_plugin_version,
    running_plugin_version,
):
    """Classify one value-group into VALID / LEGACY / INVALID / UNSET.

    * UNSET   — the group has no persisted value; nothing to validate.
    * LEGACY  — a value is present but carries no stamps (a pre-stamping
                install); accepted with a warn-once.
    * INVALID — the recomputed fingerprint differs from the staged one, OR the
                plugin has regressed below the staging version; the value is
                treated as absent everywhere.
    * VALID   — the stamps match.
    """
    if not value_present:
        return GroupValidation(group, TIER_UNSET, [], stored_fingerprint, None)
    if stored_fingerprint is None and stored_plugin_version is None:
        return GroupValidation(
            group,
            TIER_LEGACY,
            ["calibrated before fingerprint stamping (no stamps on record)"],
            None,
            None,
        )
    current = fingerprint_from_config(config, group)
    reasons = []
    if stored_fingerprint != current:
        reasons.append(
            "hardware fingerprint changed (staged %s, current %s)"
            % (stored_fingerprint, current)
        )
    if is_version_regression(stored_plugin_version, running_plugin_version):
        reasons.append(
            "plugin version regressed (staged %s, running %s)"
            % (stored_plugin_version, running_plugin_version)
        )
    if reasons:
        return GroupValidation(
            group, TIER_INVALID, reasons, stored_fingerprint, current
        )
    return GroupValidation(group, TIER_VALID, [], stored_fingerprint, current)


# --- staging ----------------------------------------------------------------


def resolve_klipper_version(printer):
    """The running Klipper version, or ``None`` when unavailable.

    Canonical in-process access: klippy stores the git-describe version under
    ``software_version`` in the process start-args
    (klippy/klippy.py ``Printer.get_start_args`` returns the dict main() built
    with ``start_args['software_version'] = util.get_git_version()``).  This is
    the same string Klipper reports over the API socket ``info`` command and
    that plrd already parses (crates/plr-klipper/src/message.rs:143)."""
    getter = getattr(printer, "get_start_args", None)
    if getter is None:
        return None
    args = getter() or {}
    version = args.get("software_version")
    if version is None:
        return None
    version = str(version).strip()
    return version or None


class UnstampableError(Exception):
    """Raised when a calibration cannot be stamped because the Klipper version
    is unavailable — staging must then write NOTHING (carto
    configuration.py:73-75 refuses to save an unstamped model)."""


def stage_calibration(plugin, group, values):
    """Atomically stage a value-group plus its version/fingerprint stamps.

    ``values`` is a list of ``(option, formatted_string)`` pairs.  The Klipper
    version is resolved FIRST: if it is unavailable this raises
    ``UnstampableError`` before any ``configfile.set`` call, so a refused
    staging writes nothing (no partial value/stamp — the criterion the
    refuse-to-stage-unstamped test pins).
    """
    printer = plugin.printer
    klipper_version = resolve_klipper_version(printer)
    if klipper_version is None:
        raise UnstampableError(
            "cannot determine the running Klipper version (software_version "
            "from klippy start-args) — refusing to stage an unstamped "
            "calibration; retry once Klipper is fully started"
        )
    fp = fingerprint_from_config(plugin.config, group)
    configfile = printer.lookup_object("configfile")
    staged = list(values) + [
        (fingerprint_key(group), fp),
        (PLUGIN_VERSION_KEY, plugin_version()),
        (KLIPPER_VERSION_KEY, klipper_version),
    ]
    for option, svalue in staged:
        configfile.set("plr", option, svalue)
        plugin.note_pending_save(option)


def require_klipper_version(printer, error_type):
    """Fail fast (before any motion) when the Klipper version is unavailable.

    Called from the START gate of every staging command so an unstampable
    calibration refuses up front instead of after moving the toolhead.
    ``error_type`` is the caller's console-error class (``gcmd.error``)."""
    if resolve_klipper_version(printer) is None:
        raise error_type(
            "cannot determine the running Klipper version — this calibration "
            "cannot be stamped and would be refused at save time; retry once "
            "Klipper has fully started"
        )
