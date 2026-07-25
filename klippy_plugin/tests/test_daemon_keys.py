"""The boot guard: klippy must accept every ``[plr]`` option plrd reads.

WHAT IS BEING PREVENTED.  Klipper builds ``configfile.settings`` — the only
window plrd has onto the ``[plr]`` section — from the options a module
actually ACCESSED (klippy/configfile.py:29-60, :447-450), and refuses to
start when a configured section carries an option nobody accessed
(``ConfigValidate.check_unused``, klippy/configfile.py:424-441, called from
``Klippy._read_config``, klippy/klippy.py:127).  So for every key plrd
consumes there must be a matching read in the plugin, or the operator who
follows the documentation gets a printer that will not boot.

WHY IT WENT UNNOTICED FOR SO LONG (worth knowing before "simplifying"
anything here): ``calibration_meta._config_to_sections`` enumerates the
whole ``[plr]`` section to compute the calibration fingerprint, which used
to mark every option accessed as a side effect — but only on a machine
whose calibration carries stamps, because ``validate_group`` returns
UNSET/LEGACY before it ever fingerprints.  A FRESH INSTALL — the state
every new operator is in — never reached it.  ``test_fresh_install_...``
below pins that case specifically, and the enumeration no longer records
accesses at all (it must not publish string values into a typed map; see
``test_fingerprinting_does_not_clobber_types``).

HOW THE EXPECTED SET IS DERIVED — and why not from the docs.  The
authority is plrd's own parser, ``PlrSettings::parse`` in
``crates/plrd/src/plrcfg.rs``: the literal option names it looks up ARE
the set of keys plrd consumes, so a key added there fails this test until
someone wires the plugin read.  Documentation was considered first and
rejected as an authority, not for brittleness but for INCOMPLETENESS:
``deploy/plrd.conf.example`` mentions 28 of the 46 keys (it documents the
recovery-UX surface, since the rest is the plugin's own), never mentions
``probe_method`` / ``drag_speed`` / ``self_locking_z``, and does mention
``probe_nozzle_temp``, which is NOT a ``[plr]`` key at all; and
``klippy_plugin/README.md`` covers the complementary half.  A test built
on either would have passed vacuously for exactly the keys that were
broken.  A hand-maintained list would drift in silence.  Parsing the
consumer cannot: it is the code that does the consuming.

Extraction robustness is not assumed — ``test_extraction_is_not_vacuous``
fails if a Rust-side refactor makes the parse return a suspiciously small
or canary-free set, so this suite cannot degrade into a no-op.
"""

import math
import os
import re

import fake_klippy
import pytest

import plr
from plr import daemon_keys

# --- the authority: plrd's own parser ---------------------------------

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_TESTS_DIR))
_PLRCFG = os.path.join(_REPO_ROOT, "crates", "plrd", "src", "plrcfg.rs")
_DIAGNOSIS = os.path.join(_REPO_ROOT, "crates", "plr-recovery", "src", "diagnosis.rs")

# `opt_f64(plr, "key", ...)` and friends, plus the bare `plr.get("key")`
# lookups.  Whitespace is collapsed first because rustfmt wraps these
# calls across lines (`let probe_method = plr\n    .get("probe_method")`).
_HELPER_READ_RE = re.compile(
    r'\b(?:opt_f64|opt_opt_f64|opt_bool|opt_str|opt_stamp)\s*\(\s*plr\s*,\s*"([A-Za-z0-9_]+)"'
)
_DIRECT_READ_RE = re.compile(r'\bplr\s*\.\s*get\(\s*"([A-Za-z0-9_]+)"')
# The UNSAFE key is read through a shared constant, not a literal.
_UNSAFE_CONST_RE = re.compile(
    r'pub const UNSAFE_PURGE_Z_BELOW_BED:\s*&str\s*=\s*"([^"]+)"'
)

# Sanity floor for the extraction (see test_extraction_is_not_vacuous):
# the count at the time this guard was written.  Raise it freely; never
# lower it to make a red test green.
_MIN_CONSUMED_KEYS = 46

# Keys whose absence would mean the extraction silently stopped working.
_CANARY_KEYS = (
    "probe_method",
    "purge_z",
    "touch_samples",
    "confirm_timeout_s",
    "recovery_accel",
    "UNSAFE_allow_purge_z_below_bed",
)


def _read(path):
    assert os.path.isfile(path), (
        "cannot find plrd's config parser at %s — this guard derives the "
        "expected [plr] key set from it; if the layout moved, update the "
        "path, never delete the guard" % (path,)
    )
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def plrd_consumed_keys():
    """Every ``[plr]`` option name plrd's parser looks up.

    Restricted to the production half of plrcfg.rs (everything before the
    first ``#[cfg(test)]``) so Rust-side test fixtures cannot inflate the
    set.
    """
    production = _read(_PLRCFG).split("#[cfg(test)]")[0]
    flattened = re.sub(r"\s+", " ", production)
    keys = set(_HELPER_READ_RE.findall(flattened))
    keys.update(_DIRECT_READ_RE.findall(flattened))
    match = _UNSAFE_CONST_RE.search(_read(_DIAGNOSIS))
    assert match is not None, (
        "UNSAFE_PURGE_Z_BELOW_BED is no longer a string constant in "
        "crates/plr-recovery/src/diagnosis.rs — the one hard-override "
        "escape hatch must stay covered by this guard"
    )
    keys.add(match.group(1))
    return keys


# plrd also sweeps every ``noise_floor_*`` option by PREFIX rather than by
# name (plrcfg.rs, `key.strip_prefix("noise_floor_")`), so the concrete
# members of that family come from the side that WRITES them: the
# plugin's PLR_NOISE_TEST autosave keys.  All are already read at config
# time in plr/plugin.py.
#
# NOTE for whoever next touches the Rust side: that prefix sweep requires
# every ``noise_floor_*`` value to be a NUMBER, so a configured
# ``noise_floor_temp_sensor`` (a string) makes plrd reject the whole
# section.  That is a plrd-side defect, out of scope here, and recorded in
# this branch's report.
_NOISE_FLOOR_FAMILY = {
    "noise_floor_rms": "0.020",
    "noise_floor_still_rms": "0.010",
    "noise_floor_peak": "0.050",
    "noise_floor_speed": "20.0",
    "noise_floor_temp": "40.0",
    "noise_floor_temp_sensor": "extruder",
}

# In-band values for the keys the plugin reads for its OWN use (these
# still carry the plugin's real schema ranges — only the daemon-only keys
# are read loosely).  A key plrd consumes that appears neither here nor in
# daemon_keys.DAEMON_KEYS fails _plr_section with instructions.
_PLUGIN_OWNED_VALUES = {
    "probe_method": "tap",
    "accel_chip": "adxl345",
    "wal_dir": "/var/lib/plrd/wal",
    "control_socket": "/var/lib/plrd/plrd.sock",
    "probe_speed": "1.5",
    "envelope_margin": "0.5",
    "sag_allowance": "0.2",
    "drag_speed": "20.0",
    "drag_z_step": "0.05",
    "drag_sensitivity": "30.0",
    "exclusion_radius": "5.0",
    "entry_feedrate": "1800.0",
    "max_probe_nozzle_temp": "150.0",
    "clean_nozzle_macro": "CLEAN_NOZZLE",
    "self_locking_z": "True",
    "probe_resolution": "0.010",
    "cal_plugin_version": plr.__version__,
    "cal_klipper_version": "v0.13.0",
    "cal_fingerprint_noise_floor": "crc32:00000000",
    "cal_fingerprint_probe_resolution": "crc32:00000000",
}

# Sample raw values per kind.  Deliberately OUTSIDE plrd's bands where a
# band exists (touch_samples' is [3, 7], the accel keys' is [50, 20000]):
# the plugin must accept them, because plrd is the single authority on
# bands and a second opinion here is how validators deadlock.
_SAMPLE_BY_KIND = {
    daemon_keys.FLOAT: "7.5",
    daemon_keys.BOOLEAN: "True",
    daemon_keys.STRING: "PURGE_BUCKET",
}


def _plr_section(extra=None):
    """A ``[plr]`` section carrying every option plrd consumes."""
    section = {}
    for key in sorted(plrd_consumed_keys()):
        kind = daemon_keys.DAEMON_KEYS.get(key)
        if kind is not None:
            section[key] = _SAMPLE_BY_KIND[kind]
        elif key in _PLUGIN_OWNED_VALUES:
            section[key] = _PLUGIN_OWNED_VALUES[key]
        else:
            pytest.fail(
                "plrd consumes [plr] option '%s' but the plugin has no read "
                "for it: klippy REFUSES TO START on a config that sets it "
                "(klippy/configfile.py:424-441).  Declare it in "
                "plr/daemon_keys.py DAEMON_KEYS (or, if the plugin uses the "
                "value itself, read it where it is used and add it to "
                "_PLUGIN_OWNED_VALUES here)." % (key,)
            )
    section.update(_NOISE_FLOOR_FAMILY)
    section.update(extra or {})
    return section


def _load(fake_printer, tmp_path, options, sections=None):
    """Load the plugin over a ``[plr]`` section; return (plugin, config)."""
    import conftest

    merged = conftest.good_sections()
    for name, value in (sections or {}).items():
        if value is None:
            merged.pop(name, None)
        else:
            merged[name] = value
    merged.setdefault("gcode_macro CLEAN_NOZZLE", {"gcode": "M117 clean"})
    merged.setdefault("gcode_macro PURGE_BUCKET", {"gcode": "M117 purge"})
    opts = dict(options)
    opts.setdefault("wal_dir", str(tmp_path / "wal"))
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    config = fake_klippy.FakeConfig(
        fake_printer, name="plr", options=opts, sections=merged
    )
    return plr.load_config(config), config


# --- the extraction itself must not rot -------------------------------


def test_extraction_is_not_vacuous():
    keys = plrd_consumed_keys()
    assert len(keys) >= _MIN_CONSUMED_KEYS, (
        "only %d [plr] keys extracted from plrd's parser (expected at least "
        "%d) — the extraction patterns have probably stopped matching after a "
        "Rust-side refactor, which would make this whole guard a no-op"
        % (len(keys), _MIN_CONSUMED_KEYS)
    )
    missing = [key for key in _CANARY_KEYS if key not in keys]
    assert not missing, "extraction lost canary keys %s" % (missing,)


def test_daemon_keys_table_declares_only_keys_plrd_consumes():
    # The table is a boot requirement, not a dumping ground: an entry plrd
    # does not consume would be an option klippy accepts and nothing reads.
    consumed = plrd_consumed_keys()
    strays = sorted(set(daemon_keys.DAEMON_KEYS) - consumed)
    assert not strays, "daemon_keys.DAEMON_KEYS declares unconsumed keys %s" % (strays,)


def test_daemon_keys_table_does_not_shadow_a_used_value():
    # Keys the plugin reads for its own purposes must be read where they
    # are used, with their real schema ranges — not loosely here.
    overlap = sorted(set(daemon_keys.DAEMON_KEYS) & set(_PLUGIN_OWNED_VALUES))
    assert not overlap, "declared loosely AND used by the plugin: %s" % (overlap,)


# --- the boot guard ---------------------------------------------------


def test_klippy_accepts_every_key_plrd_consumes(fake_printer, tmp_path):
    """The defect itself: no ``[plr]`` option may go unclaimed.

    ``unused_options`` is the harness's mirror of
    ``ConfigValidate.check_unused``; a non-empty result is precisely the
    startup error an operator would hit.
    """
    _, config = _load(fake_printer, tmp_path, _plr_section())
    unclaimed = config.unused_options("plr")
    assert not unclaimed, (
        "klippy would refuse to start: no plugin read claims [plr] option(s) "
        "%s.  Declare them in plr/daemon_keys.py DAEMON_KEYS." % (unclaimed,)
    )


def test_every_consumed_key_reaches_configfile_settings(fake_printer, tmp_path):
    """The other half: the value must be visible to plrd, not just legal."""
    _, config = _load(fake_printer, tmp_path, _plr_section())
    published = config.accessed_options("plr")
    missing = sorted(
        key for key in plrd_consumed_keys() if key.lower() not in published
    )
    assert not missing, (
        "plrd reads these from configfile.settings but the plugin never "
        "accessed them, so klippy never publishes them: %s" % (missing,)
    )


def test_fresh_install_is_covered(fake_printer, tmp_path):
    """A never-calibrated printer is the case that actually failed.

    No calibration values and no stamps: ``validate_group`` returns UNSET
    without fingerprinting, so nothing incidentally claims the section.
    """
    section = _plr_section()
    for key in list(_NOISE_FLOOR_FAMILY) + [
        "probe_resolution",
        "cal_plugin_version",
        "cal_klipper_version",
        "cal_fingerprint_noise_floor",
        "cal_fingerprint_probe_resolution",
        "self_locking_z",
    ]:
        section.pop(key, None)
    _, config = _load(fake_printer, tmp_path, section)
    assert not config.unused_options("plr")


def test_settings_types_match_what_plrd_parses(fake_printer, tmp_path):
    """plrd hard-errors on a wrong-typed value, so publish the right type.

    ``opt_f64`` needs a JSON number, ``opt_bool`` a bool, ``opt_str`` a
    string (crates/plrd/src/plrcfg.rs).
    """
    _, config = _load(fake_printer, tmp_path, _plr_section())
    settings = config.accessed_settings("plr")
    expected_type = {
        daemon_keys.FLOAT: float,
        daemon_keys.BOOLEAN: bool,
        daemon_keys.STRING: str,
    }
    for key, kind in daemon_keys.DAEMON_KEYS.items():
        value = settings[key.lower()]
        assert isinstance(value, expected_type[kind]), (
            "[plr] %s published as %r (%s); plrd expects %s"
            % (key, value, type(value).__name__, kind)
        )


def test_fingerprinting_does_not_clobber_types(fake_printer, tmp_path):
    """A stamped calibration must not turn the typed map back into strings.

    ``calibration_meta._config_to_sections`` enumerates the whole section
    for the hash; if it recorded accesses it would overwrite each typed
    value with its raw string and plrd would reject the section — and the
    UNSAFE override would arrive as ``"True"``, i.e. fail silently closed.
    """
    section = _plr_section()
    _, config = _load(fake_printer, tmp_path, section)
    settings = config.accessed_settings("plr")
    # The stamps are present, so validate_group DID fingerprint.
    assert settings["cal_fingerprint_noise_floor"] == "crc32:00000000"
    assert isinstance(settings["purge_amount"], float)
    assert settings["unsafe_allow_purge_z_below_bed"] is True


# --- one authority per value ------------------------------------------


def test_absent_daemon_keys_are_not_published(fake_printer, tmp_path):
    """An unset key must stay out of ``settings`` so plrd's default wins.

    klippy publishes a getter's DEFAULT for an absent option whenever that
    default is not ``None`` (klippy/configfile.py:31-36).  A plugin-side
    default would therefore masquerade as an operator value and shadow
    ``PlanConfig::default()`` — two defaults that can drift.
    """
    _, config = _load(fake_printer, tmp_path, {"probe_method": "tap"})
    published = config.accessed_options("plr")
    leaked = sorted(key for key in daemon_keys.DAEMON_KEYS if key.lower() in published)
    assert not leaked, (
        "unset daemon key(s) %s were published into configfile.settings with a "
        "plugin-side default; plrd must supply its own defaults" % (leaked,)
    )


def test_unset_daemon_keys_read_as_none(fake_printer, tmp_path):
    plugin, _ = _load(fake_printer, tmp_path, {"probe_method": "tap"})
    assert set(plugin.daemon_keys) == set(daemon_keys.DAEMON_KEYS)
    assert set(plugin.daemon_keys.values()) == {None}


# --- the loose/strict boundary ----------------------------------------


@pytest.mark.parametrize(
    "name",
    [key for key, kind in daemon_keys.DAEMON_KEYS.items() if kind == daemon_keys.FLOAT],
)
@pytest.mark.parametrize("raw", ["-1e6", "0", "1e6"])
def test_no_plugin_side_band_on_a_daemon_float(fake_printer, tmp_path, name, raw):
    """Finite values pass whatever their magnitude or sign.

    Every band belongs to plrd (``PlanConfig::validate``), which refuses
    out-of-band values with a diagnosis naming the key.  A duplicate limit
    here is how the probe-temperature ceiling once deadlocked against the
    plan's own commanded temperature: same rule, two enforcers, no way
    through.  ``purge_z`` is the sharpest case — it MUST be able to arrive
    negative so plrd can weigh it against
    ``UNSAFE_allow_purge_z_below_bed``.
    """
    plugin, _ = _load(fake_printer, tmp_path, {"probe_method": "tap", name: raw})
    assert plugin.daemon_keys[name] == float(raw)


@pytest.mark.parametrize("raw", ["nan", "inf", "-inf"])
def test_non_finite_daemon_float_is_refused(fake_printer, tmp_path, raw):
    """NaN/Infinity parse as floats but have no JSON encoding, so they
    would corrupt the whole status payload plrd reads the section from —
    a representability failure, not a policy range."""
    with pytest.raises(fake_klippy.FakeConfigError) as excinfo:
        _load(fake_printer, tmp_path, {"probe_method": "tap", "purge_z": raw})
    assert "must be finite" in str(excinfo.value)


def test_unparsable_daemon_float_is_refused(fake_printer, tmp_path):
    with pytest.raises(fake_klippy.FakeConfigError) as excinfo:
        _load(fake_printer, tmp_path, {"probe_method": "tap", "purge_speed": "fast"})
    assert "Unable to parse option 'purge_speed'" in str(excinfo.value)


def test_unparsable_daemon_boolean_is_refused(fake_printer, tmp_path):
    with pytest.raises(fake_klippy.FakeConfigError) as excinfo:
        _load(fake_printer, tmp_path, {"probe_method": "tap", "purge_enable": "maybe"})
    assert "Unable to parse option 'purge_enable'" in str(excinfo.value)


def test_daemon_string_is_passed_through_verbatim(fake_printer, tmp_path):
    plugin, _ = _load(
        fake_printer, tmp_path, {"probe_method": "tap", "purge_macro": "MY_PURGE"}
    )
    assert plugin.daemon_keys["purge_macro"] == "MY_PURGE"


def test_finite_check_accepts_every_ordinary_float():
    # The finiteness guard must not reject anything a printer.cfg would
    # plausibly hold; math.isfinite is the whole rule.
    assert all(math.isfinite(v) for v in (0.0, -2.0, 1e-9, 20000.0))


# --- the UNSAFE escape hatch -----------------------------------------

_UNSAFE_KEY = "UNSAFE_allow_purge_z_below_bed"


@pytest.mark.parametrize(
    "spelling",
    [_UNSAFE_KEY, _UNSAFE_KEY.lower(), "unsafe_ALLOW_purge_z_BELOW_bed"],
    ids=["documented", "lowercase", "mangled-case"],
)
def test_unsafe_override_reaches_the_daemon_in_any_casing(
    fake_printer, tmp_path, spelling
):
    """The system's only hard-refusal override must actually work.

    printer.cfg is parsed by a ``configparser.RawConfigParser`` with the
    default ``optionxform`` (klippy/configfile.py:170-176), so FILE option
    names are LOWERCASED on read and a lookup by any casing finds the same
    option; ``access_tracking`` is keyed lowercased regardless
    (klippy/configfile.py:34,47).  Whatever the operator types therefore
    reaches plrd as ``unsafe_allow_purge_z_below_bed`` — the spelling
    ``plrcfg.rs``'s ``unsafe_flag`` falls back to.
    """
    plugin, config = _load(
        fake_printer,
        tmp_path,
        {"probe_method": "tap", spelling: "True", "purge_z": "-0.5"},
    )
    assert not config.unused_options("plr")
    settings = config.accessed_settings("plr")
    assert settings[_UNSAFE_KEY.lower()] is True
    assert plugin.daemon_keys[_UNSAFE_KEY] is True
    # And the value it exists to permit survives with its sign intact.
    assert plugin.daemon_keys["purge_z"] == -0.5


def test_unsafe_override_defaults_to_absent(fake_printer, tmp_path):
    plugin, _ = _load(fake_printer, tmp_path, {"probe_method": "tap"})
    assert plugin.daemon_keys[_UNSAFE_KEY] is None


# --- observability ----------------------------------------------------


def test_get_status_reports_the_daemon_config(fake_printer, tmp_path):
    plugin, _ = _load(
        fake_printer,
        tmp_path,
        {"probe_method": "tap", "purge_enable": "True", "purge_amount": "8.0"},
    )
    status = plugin.get_status(1.0)
    reported = status["daemon_config"]
    assert set(reported) == set(daemon_keys.DAEMON_KEYS)
    assert reported["purge_enable"] is True
    assert reported["purge_amount"] == 8.0
    # None = not configured, i.e. plrd's own default applies.
    assert reported["purge_speed"] is None


def test_get_status_daemon_config_is_a_copy(fake_printer, tmp_path):
    plugin, _ = _load(fake_printer, tmp_path, {"probe_method": "tap"})
    plugin.get_status(1.0)["daemon_config"]["purge_enable"] = "tampered"
    assert plugin.daemon_keys["purge_enable"] is None
