"""Tests for calibration_meta: fingerprinting, three-tier validation, the
version/fingerprint stamping of the persisted calibration values, and the
refuse-to-stage-unstamped / treat-stale-as-absent behaviors.

The three ``SHARED_*`` fixtures below carry the SAME expected hex constants
asserted in the Rust suite (crates/plrd/src/plrcfg.rs ``calibration_meta``
tests): a byte-identical cross-language fingerprint is the contract that lets
plrd re-derive the fingerprint the plugin stamped.
"""

import conftest
import fake_klippy
import pytest
import stream_fixtures as sf

import plr
from plr import calibration_meta as cm

NF = cm.GROUP_NOISE_FLOOR
PR = cm.GROUP_PROBE_RESOLUTION

# --- cross-language shared literal fixtures ---------------------------------
# (sections, section_names, plr_keys, expected_hex).  String-valued so there
# is no float-formatting dependency; the Rust suite asserts the same hexes.
SHARED_FIXTURES = [
    (
        {
            "stepper_z": {
                "step_pin": "PF11",
                "dir_pin": "!PH1",
                "position_min": "-2",
                "position_max": "250",
            },
            "probe": {"z_offset": "0.5", "pin": "^PA1"},
            "plr": {"probe_method": "tap", "accel_chip": "adxl345"},
        },
        ["stepper_z", "probe"],
        ["probe_method"],
        "ca910c12",
    ),
    (
        {
            "stepper_z": {"step_pin": "PF11", "position_min": "-2"},
            "stepper_z1": {"step_pin": "PG0"},
            "adxl345": {"cs_pin": "PB1", "axes_map": "x,y,z"},
            "plr": {"probe_method": "adxl_drag", "accel_chip": "adxl345"},
        },
        ["stepper_z", "stepper_z1", "adxl345"],
        ["accel_chip", "probe_method"],
        "cecd3842",
    ),
    (
        {
            "stepper_z": {"position_min": "-2.0", "microsteps": "16"},
            "plr": {"probe_method": "tap"},
        },
        ["stepper_z"],
        ["probe_method"],
        "404202d1",
    ),
]


def test_shared_fixture_hashes_match_rust():
    for sections, names, keys, expected in SHARED_FIXTURES:
        assert cm.fingerprint(sections, names, keys) == expected


def test_crc32_matches_zlib_reference():
    # The Rust port reproduces zlib.crc32 exactly; pin the canonical vector.
    assert cm._crc32_hex("123456789") == "cbf43926"
    assert cm._crc32_hex("") == "00000000"


# --- fingerprint stability & sensitivity ------------------------------------


def _base_sections():
    return {
        "stepper_z": {"step_pin": "PF11", "position_min": "-2", "position_max": "250"},
        "stepper_z1": {"step_pin": "PG0"},
        "probe": {"z_offset": "0.5", "pin": "^PA1"},
        "adxl345": {"cs_pin": "PB1"},
        "plr": {"probe_method": "adxl_drag", "accel_chip": "adxl345"},
        "fan": {"pin": "PA8"},
        "display": {"lcd_type": "uc1701"},
    }


def test_order_and_whitespace_invariant_for_both_groups():
    base = _base_sections()
    # Permute key order and vary whitespace; canonicalization must absorb it.
    varied = {
        "plr": {"accel_chip": " adxl345 ", "probe_method": "adxl_drag"},
        "adxl345": {"cs_pin": "PB1"},
        "probe": {"pin": "^PA1", "z_offset": "0.5"},
        "stepper_z1": {"step_pin": "PG0"},
        "stepper_z": {
            "position_max": "250",
            "step_pin": "PF11",
            "position_min": " -2 ",
        },
        "fan": {"pin": "PA8"},
        "display": {"lcd_type": "uc1701"},
    }
    for group in (NF, PR):
        assert cm.compute_fingerprint(base, group) == cm.compute_fingerprint(
            varied, group
        )


def test_irrelevant_sections_do_not_change_either_fingerprint():
    base = _base_sections()
    mutated = _base_sections()
    mutated["fan"]["pin"] = "PA9"
    mutated["display"]["lcd_type"] = "st7920"
    mutated["heater_bed"] = {"sensor_type": "NTC 100K"}
    for group in (NF, PR):
        assert cm.compute_fingerprint(base, group) == cm.compute_fingerprint(
            mutated, group
        )


def test_stepper_z_pin_change_moves_both_fingerprints():
    base = _base_sections()
    mutated = _base_sections()
    mutated["stepper_z"]["step_pin"] = "PF99"
    for group in (NF, PR):
        assert cm.compute_fingerprint(base, group) != cm.compute_fingerprint(
            mutated, group
        )


def test_probe_z_offset_change_is_probe_resolution_only():
    base = _base_sections()
    # probe_method must select the probe section for it to matter.
    base["plr"]["probe_method"] = "tap"
    mutated = dict((k, dict(v)) for k, v in base.items())
    mutated["probe"] = {"z_offset": "0.9", "pin": "^PA1"}
    assert cm.compute_fingerprint(base, PR) != cm.compute_fingerprint(mutated, PR)
    # The noise floor does not depend on the touch-probe section.
    assert cm.compute_fingerprint(base, NF) == cm.compute_fingerprint(mutated, NF)


def test_accel_chip_change_is_noise_floor_only():
    base = _base_sections()
    mutated = dict((k, dict(v)) for k, v in base.items())
    mutated["plr"] = {"probe_method": "adxl_drag", "accel_chip": "lis2dw"}
    mutated["lis2dw"] = {"cs_pin": "PB2"}
    assert cm.compute_fingerprint(base, NF) != cm.compute_fingerprint(mutated, NF)
    # probe_resolution excludes accel_chip entirely.
    assert cm.compute_fingerprint(base, PR) == cm.compute_fingerprint(mutated, PR)


def test_probe_method_change_moves_both_fingerprints():
    base = _base_sections()
    base["plr"]["probe_method"] = "tap"
    mutated = dict((k, dict(v)) for k, v in base.items())
    mutated["plr"] = {"probe_method": "load_cell", "accel_chip": "adxl345"}
    mutated["load_cell_probe"] = {"z_offset": "0.4"}
    for group in (NF, PR):
        assert cm.compute_fingerprint(base, group) != cm.compute_fingerprint(
            mutated, group
        )


def test_numeric_canonicalization_equates_int_and_float_forms():
    a = {"stepper_z": {"position_min": "-2"}, "plr": {"probe_method": "tap"}}
    b = {"stepper_z": {"position_min": "-2.0"}, "plr": {"probe_method": "tap"}}
    assert cm.compute_fingerprint(a, PR) == cm.compute_fingerprint(b, PR)


# --- version comparison -----------------------------------------------------


@pytest.mark.parametrize(
    "stored,running,regressed",
    [
        ("0.3.0", "0.3.0", False),
        ("0.3.0", "0.4.0", False),  # upgrade is fine
        ("0.4.0", "0.3.0", True),  # downgrade
        ("0.3.5", "0.3.9", False),  # patch bump within a minor
        ("v0.13.0-462-gabc", "v0.13.0", False),
        (None, "0.3.0", False),  # unparseable -> tolerant
        ("garbage", "0.3.0", False),
    ],
)
def test_version_regression(stored, running, regressed):
    assert cm.is_version_regression(stored, running) is regressed


# --- config-adapter fingerprint (ConfigWrapper path) ------------------------


def test_fingerprint_from_config_matches_dict_path(plr_config):
    config = plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    # good_sections(): stepper_z(pins,position_min=-2), probe(z_offset=0.5),
    # adxl345(cs_pin=PB1).  Build the equivalent dict and compare.
    from_config = cm.fingerprint_from_config(config, NF)
    assert isinstance(from_config, str) and len(from_config) == 8
    # Stable across two independent reads of the same config content.
    again = plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    assert cm.fingerprint_from_config(again, NF) == from_config


# --- plugin loading helpers -------------------------------------------------


def _load(fake_printer, plr_config, options=None, sections=None, chip=False):
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    if chip:
        fake_printer.add_object("adxl345", fake_klippy.FakeAccelChip(fake_printer))
    return plr.load_config(plr_config(options=options, sections=sections))


def _matching_fp(plr_config, group, hw_options):
    return cm.fingerprint_from_config(plr_config(options=hw_options), group)


def _stamped_options(
    plr_config, group, hw_options, values, plugin_ver=None, klipper="v1"
):
    opts = dict(hw_options)
    opts.update(values)
    opts[cm.fingerprint_key(group)] = _matching_fp(plr_config, group, hw_options)
    opts[cm.PLUGIN_VERSION_KEY] = plugin_ver or plr.__version__
    opts[cm.KLIPPER_VERSION_KEY] = klipper
    return opts


# --- three-tier validation (plugin integration) -----------------------------


def test_valid_noise_floor_round_trip(fake_printer, plr_config):
    hw = {"probe_method": "adxl_drag", "accel_chip": "adxl345"}
    opts = _stamped_options(plr_config, NF, hw, {"noise_floor_rms": "42.5"})
    plugin = _load(fake_printer, plr_config, options=opts, chip=True)
    assert plugin.calibration_tier(NF) == cm.TIER_VALID
    assert plugin.noise_floor_rms == pytest.approx(42.5)
    assert plugin.calibrations_valid() is True
    assert plugin.get_status(100.0)["calibration_status"][NF] == "valid"


def test_legacy_noise_floor_is_accepted_without_stamps(fake_printer, plr_config):
    hw = {"probe_method": "adxl_drag", "accel_chip": "adxl345"}
    plugin = _load(
        fake_printer, plr_config, options=dict(hw, noise_floor_rms="42.5"), chip=True
    )
    assert plugin.calibration_tier(NF) == cm.TIER_LEGACY
    assert plugin.noise_floor_rms == pytest.approx(42.5)  # retained
    assert plugin.calibrations_valid() == "legacy"


def test_invalid_on_fingerprint_mismatch_nulls_the_value(fake_printer, plr_config):
    hw = {"probe_method": "adxl_drag", "accel_chip": "adxl345"}
    opts = _stamped_options(plr_config, NF, hw, {"noise_floor_rms": "42.5"})
    # Reload with a CHANGED Z stepper pin -> recomputed fingerprint differs.
    plugin = _load(
        fake_printer,
        plr_config,
        options=opts,
        sections={
            "stepper_z": {
                "step_pin": "PF99",
                "dir_pin": "!PH1",
                "enable_pin": "!PA0",
                "position_min": "-2",
            }
        },
        chip=True,
    )
    assert plugin.calibration_tier(NF) == cm.TIER_INVALID
    assert plugin.noise_floor_rms is None  # treated as absent
    assert plugin.noise_floor_peak is None
    assert plugin.calibrations_valid() is False
    reasons = plugin.calibrations[NF].reasons
    assert any("fingerprint changed" in r for r in reasons)


def test_invalid_on_plugin_version_regression(fake_printer, plr_config):
    hw = {"probe_method": "adxl_drag", "accel_chip": "adxl345"}
    # Stamped under a FUTURE plugin version; running plr.__version__ regresses.
    opts = _stamped_options(
        plr_config, NF, hw, {"noise_floor_rms": "42.5"}, plugin_ver="99.0.0"
    )
    plugin = _load(fake_printer, plr_config, options=opts, chip=True)
    assert plugin.calibration_tier(NF) == cm.TIER_INVALID
    assert plugin.noise_floor_rms is None
    assert any("version regressed" in r for r in plugin.calibrations[NF].reasons)


def test_groups_are_validated_independently(fake_printer, plr_config):
    # tap machine: probe_resolution VALID, noise_floor INVALID at once.
    hw = {"probe_method": "tap", "accel_chip": "adxl345"}
    opts = _stamped_options(
        plr_config, PR, hw, {"probe_resolution": "0.012", "noise_floor_rms": "42.5"}
    )
    # Overwrite the noise-floor stamp with a wrong hash so only that group is
    # stale; the probe_resolution stamp stays correct.
    opts[cm.fingerprint_key(NF)] = "00000000"
    plugin = _load(fake_printer, plr_config, options=opts, chip=True)
    assert plugin.calibration_tier(PR) == cm.TIER_VALID
    assert plugin.probe_resolution == pytest.approx(0.012)  # untouched
    assert plugin.calibration_tier(NF) == cm.TIER_INVALID
    assert plugin.noise_floor_rms is None  # the stale group is absent
    assert plugin.calibrations_valid() is False


def test_unset_groups_report_valid_aggregate(plugin):
    # The default good plugin persists nothing -> both groups UNSET.
    assert plugin.calibration_tier(NF) == cm.TIER_UNSET
    assert plugin.calibration_tier(PR) == cm.TIER_UNSET
    assert plugin.calibrations_valid() is True


# --- stale calibration is refused by the consuming command ------------------


def _drag_ready(fake_printer, plr_config, options):
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, 5.0, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    fake_printer.add_object("adxl345", fake_klippy.FakeAccelChip(fake_printer))
    return plr.load_config(plr_config(options=options))


def test_drag_probe_refuses_stale_floor_with_remediation(fake_printer, plr_config):
    hw = {"probe_method": "adxl_drag", "accel_chip": "adxl345"}
    opts = _stamped_options(plr_config, NF, hw, {"noise_floor_rms": "42.5"})
    opts[cm.fingerprint_key(NF)] = "00000000"  # force stale
    plr_config_sections = {
        "stepper_z": {
            "step_pin": "PF11",
            "dir_pin": "!PH1",
            "enable_pin": "!PA0",
            "position_min": "-2",
        }
    }
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, 5.0, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    fake_printer.add_object("adxl345", fake_klippy.FakeAccelChip(fake_printer))
    plugin = plr.load_config(plr_config(options=opts, sections=plr_config_sections))
    assert plugin.noise_floor_rms is None
    gcode = fake_printer.lookup_object("gcode")
    gcmd = gcode.create_gcode_command("PLR_DRAG_PROBE", "PLR_DRAG_PROBE", {})
    with pytest.raises(fake_klippy.FakeCommandError) as exc:
        gcode.commands["PLR_DRAG_PROBE"](gcmd)
    message = str(exc.value)
    assert "different hardware configuration" in message
    assert "re-run PLR_NOISE_TEST" in message


# --- refuse to stage when the calibration cannot be stamped -----------------


def test_noise_test_refuses_when_klipper_version_unavailable(fake_printer, plr_config):
    fake_printer.start_args = {}  # no software_version
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, 5.0, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    chip = fake_klippy.FakeAccelChip(fake_printer)
    fake_printer.add_object("adxl345", chip)
    plugin = plr.load_config(plr_config(options={"accel_chip": "adxl345"}))
    gcode = fake_printer.lookup_object("gcode")
    gcmd = gcode.create_gcode_command(
        "PLR_NOISE_TEST", "PLR_NOISE_TEST START=1", {"START": "1"}
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="Klipper version"):
        gcode.commands["PLR_NOISE_TEST"](gcmd)
    # Nothing staged and nothing moved before the refusal.
    assert fake_printer.lookup_object("configfile").pending == {}
    assert plugin._pending_save == set()
    assert toolhead.moves == []
    assert chip.clients == []


def test_probe_test_refuses_when_klipper_version_unavailable(fake_printer, plr_config):
    fake_printer.start_args = {}
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, 5.0, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    fake_printer.add_object(
        "probe", fake_klippy.FakeProbe(fake_printer, [0.1, 0.1, 0.1])
    )
    plugin = plr.load_config(plr_config())
    gcode = fake_printer.lookup_object("gcode")
    gcmd = gcode.create_gcode_command(
        "PLR_PROBE_TEST", "PLR_PROBE_TEST START=1", {"START": "1"}
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="Klipper version"):
        gcode.commands["PLR_PROBE_TEST"](gcmd)
    assert fake_printer.lookup_object("configfile").pending == {}
    assert plugin._pending_save == set()


def test_drag_calibrate_refuses_when_klipper_version_unavailable(
    fake_printer, plr_config
):
    fake_printer.start_args = {}
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, 20.0, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    fake_printer.add_object("adxl345", fake_klippy.FakeAccelChip(fake_printer))
    plugin = plr.load_config(
        plr_config(
            options={
                "probe_method": "adxl_drag",
                "accel_chip": "adxl345",
                "noise_floor_rms": "42.5",
            }
        )
    )
    # Legacy floor, but the refusal is about the missing Klipper version.
    gcode = fake_printer.lookup_object("gcode")
    gcmd = gcode.create_gcode_command(
        "PLR_DRAG_CALIBRATE", "PLR_DRAG_CALIBRATE START=1", {"START": "1"}
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="Klipper version"):
        gcode.commands["PLR_DRAG_CALIBRATE"](gcmd)
    assert fake_printer.lookup_object("configfile").pending == {}
    assert plugin._pending_save == set()


# --- staging writes the stamps ----------------------------------------------


def test_noise_test_stages_stamps_alongside_values(fake_printer, plr_config, run_cmd):
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, 5.0, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    chip = fake_klippy.FakeAccelChip(
        fake_printer, script=[sf.quiet(seed=1), sf.wobbly(seed=2)]
    )
    fake_printer.add_object("adxl345", chip)
    plr.load_config(plr_config(options={"accel_chip": "adxl345"}))
    run_cmd("PLR_NOISE_TEST", START=1)
    pending = fake_printer.lookup_object("configfile").pending["plr"]
    assert pending[cm.PLUGIN_VERSION_KEY] == plr.__version__
    assert pending[cm.KLIPPER_VERSION_KEY] == "v0.12.0-321-gabcdef012"
    assert len(pending[cm.fingerprint_key(NF)]) == 8
    # The staged fingerprint is exactly what a reload recomputes -> VALID.
    fp = cm.fingerprint_from_config(plr_config(options={"accel_chip": "adxl345"}), NF)
    assert pending[cm.fingerprint_key(NF)] == fp


def test_staged_then_reloaded_calibration_is_valid(fake_printer, plr_config, run_cmd):
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, 5.0, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    chip = fake_klippy.FakeAccelChip(
        fake_printer, script=[sf.quiet(seed=1), sf.wobbly(seed=2)]
    )
    fake_printer.add_object("adxl345", chip)
    plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    )
    run_cmd("PLR_NOISE_TEST", START=1)
    pending = dict(fake_printer.lookup_object("configfile").pending["plr"])
    # Simulate the post-SAVE_CONFIG restart on a FRESH printer: the pending
    # block is now ordinary [plr] config.
    restart = fake_klippy.FakePrinter()
    restart.add_object("gcode", fake_klippy.FakeGCode())
    restart.add_object("configfile", fake_klippy.FakeConfigfile())
    restart.add_object("toolhead", fake_klippy.FakeToolhead())
    restart.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    restart.add_object("adxl345", fake_klippy.FakeAccelChip(restart))
    reloaded = plr.load_config(
        fake_klippy.FakeConfig(
            restart,
            name="plr",
            options=dict(pending, probe_method="adxl_drag", accel_chip="adxl345"),
            sections=conftest.good_sections(),
        )
    )
    assert reloaded.calibration_tier(NF) == cm.TIER_VALID


# --- warn-once for legacy ---------------------------------------------------


def test_legacy_warning_is_emitted_once_not_per_command(fake_printer, plr_config):
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, 5.0, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    fake_printer.add_object("adxl345", fake_klippy.FakeAccelChip(fake_printer))
    plugin = plr.load_config(
        plr_config(
            options={
                "probe_method": "adxl_drag",
                "accel_chip": "adxl345",
                "noise_floor_rms": "42.5",
            }
        )
    )
    assert plugin.calibration_tier(NF) == cm.TIER_LEGACY
    gcode = fake_printer.lookup_object("gcode")

    def warn_count():
        return sum("predate fingerprint stamping" in r for r in gcode.responses)

    plugin.warn_legacy_calibration_once(gcode.create_gcode_command("A", "A", {}))
    plugin.warn_legacy_calibration_once(gcode.create_gcode_command("B", "B", {}))
    assert warn_count() == 1


# --- PLR_SETUP surfaces calibration validity --------------------------------


def test_plr_setup_reports_stale_calibration_as_fail(fake_printer, plr_config, run_cmd):
    hw = {"probe_method": "adxl_drag", "accel_chip": "adxl345"}
    opts = _stamped_options(plr_config, NF, hw, {"noise_floor_rms": "42.5"})
    opts[cm.fingerprint_key(NF)] = "00000000"
    _load(fake_printer, plr_config, options=opts, chip=True)
    gcode = run_cmd("PLR_SETUP")
    report = gcode.responses[-1]
    assert "[FAIL] calibration:noise_floor" in report
    assert "stale" in report and "00000000" in report


def test_plr_setup_reports_legacy_calibration_as_warn(
    fake_printer, plr_config, run_cmd
):
    plugin = _load(
        fake_printer,
        plr_config,
        options={
            "probe_method": "adxl_drag",
            "accel_chip": "adxl345",
            "noise_floor_rms": "42.5",
        },
        chip=True,
    )
    assert plugin.calibration_tier(NF) == cm.TIER_LEGACY
    gcode = run_cmd("PLR_SETUP")
    assert "[WARN] calibration:noise_floor" in gcode.responses[-1]
