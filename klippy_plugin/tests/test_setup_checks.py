"""Tests for commissioning checks and the PLR_SETUP command."""

import importlib.util
import os
import time

import fake_klippy
import pytest
from conftest import good_sections

import plr
from plr import setup_checks


def make_config(fake_printer, sections):
    return fake_klippy.FakeConfig(fake_printer, name="plr", sections=sections)


# --- force_move --------------------------------------------------------


def test_force_move_missing_section_fails(fake_printer):
    res = setup_checks.check_force_move(make_config(fake_printer, {}))
    assert res.verdict == "fail"
    assert "enable_force_move" in res.hint


def test_force_move_disabled_fails(fake_printer):
    config = make_config(fake_printer, {"force_move": {"enable_force_move": "false"}})
    assert setup_checks.check_force_move(config).verdict == "fail"


def test_force_move_section_without_option_fails(fake_printer):
    # klippy's default is False (klippy/extras/force_move.py:42).
    config = make_config(fake_printer, {"force_move": {}})
    assert setup_checks.check_force_move(config).verdict == "fail"


def test_force_move_enabled_passes(fake_printer):
    config = make_config(fake_printer, {"force_move": {"enable_force_move": "true"}})
    assert setup_checks.check_force_move(config).verdict == "pass"


# --- probe section per method -----------------------------------------


def test_tap_needs_probe_section(fake_printer):
    config = make_config(fake_printer, {})
    res = setup_checks.check_probe_section(config, "tap", None)
    assert res.verdict == "fail"
    assert "[probe]" in res.detail or "probe" in res.detail


def test_tap_with_probe_passes(fake_printer):
    config = make_config(fake_printer, {"probe": {}})
    assert setup_checks.check_probe_section(config, "tap", None).verdict == "pass"


def test_load_cell_with_section_passes(fake_printer):
    config = make_config(fake_printer, {"load_cell_probe": {}})
    res = setup_checks.check_probe_section(config, "load_cell", None)
    assert res.verdict == "pass"


def test_conflicting_probe_sections_fail(fake_printer):
    # Both sections register the 'probe' printer object; klippy itself
    # would refuse the config, and the report must say why.
    config = make_config(fake_printer, {"probe": {}, "load_cell_probe": {}})
    res = setup_checks.check_probe_section(config, "tap", None)
    assert res.verdict == "fail"
    assert "conflict" in res.detail


def test_adxl_drag_without_chip_fails(fake_printer):
    config = make_config(fake_printer, {"adxl345": {}})
    res = setup_checks.check_probe_section(config, "adxl_drag", None)
    assert res.verdict == "fail"


def test_adxl_drag_chip_section_missing_fails(fake_printer):
    config = make_config(fake_printer, {})
    res = setup_checks.check_probe_section(config, "adxl_drag", "adxl345 bed")
    assert res.verdict == "fail"
    assert "adxl345 bed" in res.detail


@pytest.mark.parametrize("chip", ["adxl345", "adxl345 bed"])
def test_adxl_drag_chip_section_naming_variants(fake_printer, chip):
    config = make_config(fake_printer, {chip: {}})
    res = setup_checks.check_probe_section(config, "adxl_drag", chip)
    assert res.verdict == "pass"


# --- z stepper pins on primary MCU ------------------------------------


@pytest.mark.parametrize(
    ("pin", "chip"),
    [
        ("PF11", "mcu"),
        ("!PH1", "mcu"),
        ("^!PA4", "mcu"),
        ("~PA4", "mcu"),
        ("mcu:PA1", "mcu"),
        ("z_board:PA1", "z_board"),
        ("! z_board : PA1", "z_board"),
    ],
)
def test_pin_chip_name_mirrors_klippy_parse_pin(pin, chip):
    # klippy/pins.py:67-93: strip ^~! modifiers, then chip:pin with a
    # bare name defaulting to the primary 'mcu' chip.
    assert setup_checks._pin_chip_name(pin) == chip


def test_z_steppers_all_primary_pass(fake_printer):
    config = make_config(
        fake_printer,
        {
            "stepper_z": {"step_pin": "PF11", "dir_pin": "!PH1"},
            "stepper_z1": {"step_pin": "mcu:PC1", "enable_pin": "^!PC2"},
        },
    )
    res = setup_checks.check_z_steppers_on_primary_mcu(config)
    assert res.verdict == "pass"
    assert "2 stepper_z*" in res.detail


def test_z_stepper_on_secondary_mcu_fails_and_names_pin(fake_printer):
    config = make_config(
        fake_printer,
        {
            "stepper_z": {"step_pin": "PF11"},
            "stepper_z1": {"step_pin": "z_mcu:PA1", "dir_pin": "z_mcu:PA2"},
        },
    )
    res = setup_checks.check_z_steppers_on_primary_mcu(config)
    assert res.verdict == "fail"
    assert "[stepper_z1] step_pin: z_mcu:PA1" in res.detail
    assert "z_mcu:PA2" in res.detail


def test_no_z_steppers_warns(fake_printer):
    res = setup_checks.check_z_steppers_on_primary_mcu(make_config(fake_printer, {}))
    assert res.verdict == "warn"


# --- probe activate/deactivate gcode ----------------------------------


def test_probe_gcode_absent_options_pass(fake_printer):
    config = make_config(fake_printer, {"probe": {}})
    assert setup_checks.check_probe_gcode_empty(config, "tap").verdict == "pass"


def test_probe_gcode_whitespace_only_passes(fake_printer):
    config = make_config(fake_printer, {"probe": {"activate_gcode": "  \n   \n"}})
    assert setup_checks.check_probe_gcode_empty(config, "tap").verdict == "pass"


def test_probe_gcode_nonempty_fails_naming_option(fake_printer):
    config = make_config(
        fake_printer,
        {"load_cell_probe": {"deactivate_gcode": "G91\nG1 Z2\nG90"}},
    )
    res = setup_checks.check_probe_gcode_empty(config, "load_cell")
    assert res.verdict == "fail"
    assert "deactivate_gcode" in res.detail


def test_probe_gcode_not_applicable_for_adxl_drag(fake_printer):
    config = make_config(fake_printer, {})
    assert setup_checks.check_probe_gcode_empty(config, "adxl_drag").verdict == "pass"


def test_probe_gcode_unverifiable_without_section_warns(fake_printer):
    config = make_config(fake_printer, {})
    assert setup_checks.check_probe_gcode_empty(config, "tap").verdict == "warn"


# --- z position_min ----------------------------------------------------


def test_z_position_min_from_stepper_z(fake_printer):
    config = make_config(fake_printer, {"stepper_z": {"position_min": "-2"}})
    res = setup_checks.check_z_position_min(config)
    assert res.verdict == "pass"
    assert "[stepper_z] position_min = -2" in res.detail


def test_z_position_min_falls_back_to_printer(fake_printer):
    config = make_config(
        fake_printer,
        {"stepper_z": {}, "printer": {"minimum_z_position": "-1.5"}},
    )
    res = setup_checks.check_z_position_min(config)
    assert res.verdict == "pass"
    assert "minimum_z_position" in res.detail


def test_z_position_min_missing_fails(fake_printer):
    config = make_config(fake_printer, {"stepper_z": {}, "printer": {}})
    assert setup_checks.check_z_position_min(config).verdict == "fail"


def test_z_position_min_non_finite_fails(fake_printer):
    config = make_config(fake_printer, {"stepper_z": {"position_min": "-inf"}})
    res = setup_checks.check_z_position_min(config)
    assert res.verdict == "fail"
    assert "not finite" in res.detail


# --- recorder heartbeat ------------------------------------------------


def test_heartbeat_missing_fails_with_service_hint(tmp_path):
    res = setup_checks.check_recorder_heartbeat(str(tmp_path))
    assert res.verdict == "fail"
    assert "systemctl status plrd" in res.hint


def test_heartbeat_stale_fails(tmp_path):
    hb = tmp_path / "heartbeat.bin"
    hb.write_bytes(b"\0" * 128)
    old = time.time() - 30.0
    os.utime(str(hb), (old, old))
    res = setup_checks.check_recorder_heartbeat(str(tmp_path))
    assert res.verdict == "fail"
    assert "stale" in res.detail


def test_heartbeat_fresh_passes_as_liveness_hint(tmp_path):
    hb = tmp_path / "heartbeat.bin"
    hb.write_bytes(b"\0" * 128)
    res = setup_checks.check_recorder_heartbeat(str(tmp_path))
    assert res.verdict == "pass"
    assert "liveness hint" in res.detail


def test_heartbeat_freshness_uses_injected_now(tmp_path):
    hb = tmp_path / "heartbeat.bin"
    hb.write_bytes(b"\0" * 128)
    future = time.time() + 30.0
    res = setup_checks.check_recorder_heartbeat(str(tmp_path), now=future)
    assert res.verdict == "fail"


# --- accel chip listing ------------------------------------------------


def test_accel_chips_none_warns(fake_printer):
    res = setup_checks.list_accel_chips(make_config(fake_printer, {}))
    assert res.verdict == "warn"


def test_accel_chips_lists_bare_and_named_sections(fake_printer):
    config = make_config(
        fake_printer,
        {"adxl345": {}, "adxl345 bed": {}, "lis2dw": {}, "adxl345x_not_a_chip": {}},
    )
    res = setup_checks.list_accel_chips(config)
    assert res.verdict == "pass"
    assert res.detail == "adxl345, adxl345 bed, lis2dw"


# --- report + command --------------------------------------------------


def test_format_report_marks_and_hints():
    results = [
        setup_checks.CheckResult("alpha", "pass", "fine", ""),
        setup_checks.CheckResult("beta", "fail", "broken", "fix beta"),
        setup_checks.CheckResult("gamma", "warn", "meh", "maybe gamma"),
    ]
    report = setup_checks.format_report(results, attested=False, probe_method="tap")
    assert "[PASS] alpha: fine" in report
    assert "[FAIL] beta: broken" in report
    assert "hint: fix beta" in report
    assert "hint: maybe gamma" in report
    assert "NOT COMMISSIONED (failed checks above)" in report


def test_format_report_commissioned_requires_green_and_attested():
    results = [setup_checks.CheckResult("alpha", "pass", "fine", "")]
    report = setup_checks.format_report(results, attested=True, probe_method="tap")
    assert "COMMISSIONED — plr is ready" in report
    report = setup_checks.format_report(results, attested=False, probe_method="tap")
    assert "NOT COMMISSIONED (attestation missing)" in report
    assert "ACCEPT_SELF_LOCKING_Z=1" in report


def test_plr_setup_reports_all_checks(plugin, run_cmd):
    gcode = run_cmd("PLR_SETUP")
    report = gcode.responses[-1]
    assert "PLR commissioning report (probe_method=tap)" in report
    for check in (
        "force_move",
        "probe section",
        "z steppers",
        "probe gcode",
        "z position_min",
        "accel chips",
        "recorder heartbeat",
        "self_locking_z",
    ):
        assert check in report, check
    # No heartbeat file in tmp wal_dir -> not commissioned.
    assert "NOT COMMISSIONED" in report


def test_plr_setup_accept_attestation_stages_save_config(plugin, run_cmd, fake_printer):
    gcode = run_cmd("PLR_SETUP", ACCEPT_SELF_LOCKING_Z=1)
    configfile = fake_printer.lookup_object("configfile")
    assert configfile.pending == {"plr": {"self_locking_z": "True"}}
    assert plugin.self_locking_z is True
    assert "SAVE_CONFIG" in gcode.responses[-1]


def test_plr_setup_happy_path_reports_commissioned(
    fake_printer, plr_config, run_cmd, tmp_path
):
    wal = tmp_path / "wal"
    os.makedirs(str(wal))
    (wal / "heartbeat.bin").write_bytes(b"\0" * 128)
    plr.load_config(plr_config(options={"self_locking_z": "True"}))
    gcode = run_cmd("PLR_SETUP")
    assert "Overall: COMMISSIONED" in gcode.responses[-1]


def test_static_checks_cover_good_sections_end_to_end(fake_printer):
    # The conftest good_sections() fixture config must stay green — the
    # other tests derive their scenarios from it.
    config = make_config(fake_printer, good_sections())
    results = setup_checks.run_static_checks(config, "tap", None)
    assert [r.verdict for r in results] == ["pass"] * len(results)


# --- optional-dependency helpers (numpy degradation path) --------------


def test_numpy_available_reflects_find_spec(monkeypatch):
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: object())
    assert setup_checks.numpy_available() is True
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: None)
    assert setup_checks.numpy_available() is False


def test_require_numpy_noop_when_present(monkeypatch):
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: object())
    setup_checks.require_numpy()  # must not raise


def test_require_numpy_raises_clear_hint_when_absent(monkeypatch):
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: None)
    with pytest.raises(RuntimeError, match="klippy-env/bin/pip install numpy"):
        setup_checks.require_numpy()


def test_require_numpy_raises_caller_error_type(monkeypatch):
    # Callers pass gcode.error so the message reaches the console.
    class FakeGCodeError(Exception):
        pass

    monkeypatch.setattr(importlib.util, "find_spec", lambda name: None)
    with pytest.raises(FakeGCodeError, match="numpy is required"):
        setup_checks.require_numpy(error_type=FakeGCodeError)
