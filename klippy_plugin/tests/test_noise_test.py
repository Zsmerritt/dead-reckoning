"""Tests for the PLR_NOISE_TEST capture flow and staging.

The chip script decides WHAT samples arrive (glue); the statistics
computed over them are always the real classifier math, and staged
values are asserted against classifier.stream_stats on the same
fixtures.
"""

import fake_klippy
import pytest
import stream_fixtures as sf

import plr
from plr import classifier, noise_test

STILL = sf.quiet(seed=21)
MOVING = sf.wobbly(seed=22)


@pytest.fixture
def noise_setup(fake_printer, plr_config):
    """Plugin + homed toolhead + scripted chip for noise-test tests."""

    def build(chip_script=None, options=None, start_z=5.0):
        toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, start_z, 0.0))
        fake_printer.add_object("toolhead", toolhead)
        fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
        chip = fake_klippy.FakeAccelChip(fake_printer, script=chip_script)
        fake_printer.add_object("adxl345", chip)
        merged = {"accel_chip": "adxl345"}
        merged.update(options or {})
        plugin = plr.load_config(plr_config(options=merged))
        return plugin, toolhead, chip

    return build


def staged(fake_printer):
    configfile = fake_printer.lookup_object("configfile")
    return configfile.pending.get("plr", {})


# ---------------------------------------------------------------------
# Consent / plan mode


def test_without_start_prints_plan_and_moves_nothing(noise_setup, run_cmd):
    plugin, toolhead, chip = noise_setup()
    gcode = run_cmd("PLR_NOISE_TEST")
    assert "no motion yet" in gcode.responses[-1]
    assert "START=1" in gcode.responses[-1]
    assert "away from any printed part" in gcode.responses[-1]
    assert toolhead.moves == []
    assert chip.clients == []


# ---------------------------------------------------------------------
# The measurement itself


def test_successful_run_stages_all_four_keys(noise_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    run_cmd("PLR_NOISE_TEST", START=1)
    still_stats = classifier.stream_stats(STILL)
    moving_stats = classifier.stream_stats(MOVING)
    pending = staged(fake_printer)
    assert pending["noise_floor_rms"] == "%.6f" % (moving_stats.rms,)
    assert pending["noise_floor_still_rms"] == "%.6f" % (still_stats.rms,)
    assert pending["noise_floor_peak"] == "%.6f" % (moving_stats.peak_rms,)
    # Default SPEED is the drag_speed tunable (20.0).
    assert pending["noise_floor_speed"] == "%.6f" % (20.0,)
    for key in (
        "noise_floor_rms",
        "noise_floor_still_rms",
        "noise_floor_peak",
        "noise_floor_speed",
    ):
        assert plugin.is_pending_save(key), key


def test_stages_noise_floor_temp_when_sensor_configured(
    noise_setup, run_cmd, fake_printer
):
    plugin, toolhead, chip = noise_setup(
        chip_script=[STILL, MOVING],
        options={"noise_floor_temp_sensor": "temperature_sensor cham"},
    )
    fake_printer.add_object("temperature_sensor cham", fake_klippy.FakeTempSensor(48.5))
    run_cmd("PLR_NOISE_TEST", START=1)
    pending = staged(fake_printer)
    assert pending["noise_floor_temp"] == "%.6f" % (48.5,)
    assert plugin.is_pending_save("noise_floor_temp")
    assert plugin.noise_floor_temp == pytest.approx(48.5)


def test_no_temp_sensor_stages_no_temp(noise_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    run_cmd("PLR_NOISE_TEST", START=1)
    assert "noise_floor_temp" not in staged(fake_printer)
    assert plugin.noise_floor_temp is None


def test_sensor_configured_but_no_reading_stages_no_temp(
    noise_setup, run_cmd, fake_printer
):
    plugin, toolhead, chip = noise_setup(
        chip_script=[STILL, MOVING],
        options={"noise_floor_temp_sensor": "temperature_sensor cham"},
    )
    fake_printer.add_object("temperature_sensor cham", fake_klippy.FakeTempSensor(None))
    run_cmd("PLR_NOISE_TEST", START=1)
    assert "noise_floor_temp" not in staged(fake_printer)
    assert plugin.noise_floor_temp is None


def test_noise_floor_speed_is_the_capture_speed(noise_setup, run_cmd, fake_printer):
    """noise_floor_speed records the SPEED the moving baseline was
    captured at — plrd warns when a plan's drag speed differs."""
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    run_cmd("PLR_NOISE_TEST", START=1, SPEED=7.5)
    assert staged(fake_printer)["noise_floor_speed"] == "%.6f" % (7.5,)
    assert plugin.noise_floor_speed == pytest.approx(7.5)
    # And it matches what the moving passes actually ran at.
    lateral = [m for m in toolhead.moves if m[0][0] is not None]
    assert all(m[1] == pytest.approx(7.5) for m in lateral)


def test_noise_floor_rms_is_the_moving_rms_not_still(
    noise_setup, run_cmd, fake_printer
):
    """The persisted reference is the MOVING baseline: drag passes are
    classified while moving, so motion-correlated vibration must be in
    the floor."""
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    run_cmd("PLR_NOISE_TEST", START=1)
    moving_stats = classifier.stream_stats(MOVING)
    still_stats = classifier.stream_stats(STILL)
    assert moving_stats.rms != pytest.approx(still_stats.rms)
    assert staged(fake_printer)["noise_floor_rms"] == "%.6f" % (moving_stats.rms,)


def test_live_values_set_for_same_session_drag_probe(noise_setup, run_cmd):
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    assert plugin.noise_floor_rms is None
    assert plugin.noise_floor_speed is None
    run_cmd("PLR_NOISE_TEST", START=1)
    moving_stats = classifier.stream_stats(MOVING)
    assert plugin.noise_floor_rms == pytest.approx(moving_stats.rms)
    assert plugin.noise_floor_still_rms == pytest.approx(
        classifier.stream_stats(STILL).rms
    )
    assert plugin.noise_floor_peak == pytest.approx(moving_stats.peak_rms)
    assert plugin.noise_floor_speed == pytest.approx(20.0)


def test_capture_sequence_still_then_moving(noise_setup, run_cmd):
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    run_cmd("PLR_NOISE_TEST", START=1, DURATION=3.5)
    assert len(chip.clients) == 2
    assert all(c.finished for c in chip.clients)
    # Still capture: a dwell of DURATION, no motion while it ran.
    assert 3.5 in toolhead.dwells
    # Moving capture: MOVE_PASSES passes of 3 segments each, at the
    # drag_speed tunable default, at fixed Z.
    lateral = [m for m in toolhead.moves if m[0][0] is not None]
    assert len(lateral) == noise_test.MOVE_PASSES * 3
    assert all(m[1] == pytest.approx(20.0) for m in lateral)
    assert all(m[0][2] is None for m in lateral)
    # Pass geometry: +L/4 / -L/4 / center around the invocation XY.
    seg = 8.0 / 4.0
    assert lateral[0][0][:2] == [150.0 + seg, 150.0]
    assert lateral[1][0][:2] == [150.0 - seg, 150.0]
    assert lateral[2][0][:2] == [150.0, 150.0]
    # No Z move at all in the whole sequence.
    assert all(m[0][2] is None for m in toolhead.moves)


def test_report_contains_threshold_and_save_config_hint(noise_setup, run_cmd):
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    gcode = run_cmd("PLR_NOISE_TEST", START=1)
    report = gcode.responses[-1]
    moving_stats = classifier.stream_stats(MOVING)
    sens = plugin.tunables["drag_sensitivity"]
    threshold = moving_stats.rms * classifier.multiplier(sens)
    assert "%.3f" % (threshold,) in report
    assert "SAVE_CONFIG" in report
    assert "away from any printed part" in report
    assert "noise_floor_speed" in report


def test_speed_arg_overrides_drag_speed(noise_setup, run_cmd):
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    run_cmd("PLR_NOISE_TEST", START=1, SPEED=7.5)
    lateral = [m for m in toolhead.moves if m[0][0] is not None]
    assert all(m[1] == pytest.approx(7.5) for m in lateral)


def test_spaced_chip_arg(noise_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = noise_setup(chip_script=None)
    other = fake_klippy.FakeAccelChip(fake_printer, script=[STILL, MOVING])
    fake_printer.add_object("adxl345 bed", other)
    run_cmd("PLR_NOISE_TEST", START=1, CHIP="adxl345 bed")
    assert len(other.clients) == 2
    assert chip.clients == []


# ---------------------------------------------------------------------
# Degenerate captures refuse the measurement, staging nothing


def test_no_data_still_capture_refused(noise_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = noise_setup(chip_script=[None])
    with pytest.raises(fake_klippy.FakeCommandError, match="measured no data"):
        run_cmd("PLR_NOISE_TEST", START=1)
    assert staged(fake_printer) == {}
    assert plugin.noise_floor_rms is None


def test_degenerate_still_capture_refused(noise_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = noise_setup(chip_script=[sf.quiet(n=100)])
    with pytest.raises(fake_klippy.FakeCommandError, match="too_few_samples"):
        run_cmd("PLR_NOISE_TEST", START=1)
    assert staged(fake_printer) == {}


def test_degenerate_moving_capture_stages_nothing(noise_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, sf.constant()])
    with pytest.raises(fake_klippy.FakeCommandError, match="constant_signal"):
        run_cmd("PLR_NOISE_TEST", START=1)
    # The still capture was fine — but a half-measured noise floor must
    # never be staged.
    assert staged(fake_printer) == {}
    assert plugin.noise_floor_rms is None


def test_zero_rms_capture_refused(noise_setup, run_cmd, fake_printer, monkeypatch):
    plugin, toolhead, chip = noise_setup(chip_script=[STILL, MOVING])
    monkeypatch.setattr(
        classifier, "stream_stats", lambda samples: classifier.StreamStats(0.0, 0.0)
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="RMS is zero"):
        run_cmd("PLR_NOISE_TEST", START=1)
    assert staged(fake_printer) == {}


# ---------------------------------------------------------------------
# Invocation gates


def test_refused_while_printing(noise_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = noise_setup()
    fake_printer.objects["idle_timeout"].state = "Printing"
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_NOISE_TEST", START=1)
    assert toolhead.moves == []
    assert chip.clients == []


def test_refused_unless_fully_homed(noise_setup, run_cmd):
    plugin, toolhead, chip = noise_setup()
    toolhead.homed_axes = "xz"
    with pytest.raises(fake_klippy.FakeCommandError, match="must be homed"):
        run_cmd("PLR_NOISE_TEST", START=1)
    assert toolhead.moves == []


def test_refused_without_any_chip(fake_printer, plr_config, run_cmd):
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    plr.load_config(plr_config())  # no accel_chip option
    with pytest.raises(fake_klippy.FakeCommandError, match="no accel chip"):
        run_cmd("PLR_NOISE_TEST", START=1)


def test_refused_when_chip_object_missing(noise_setup, run_cmd):
    plugin, toolhead, chip = noise_setup()
    with pytest.raises(
        fake_klippy.FakeCommandError, match="chip sections found: adxl345"
    ):
        run_cmd("PLR_NOISE_TEST", START=1, CHIP="mpu9250")


@pytest.mark.parametrize(
    ("param", "value", "text"),
    [
        ("SPEED", "0", "must be above"),
        ("SPEED", "150", "maximum"),
        ("DURATION", "0.1", "minimum"),
        ("DURATION", "60", "maximum"),
    ],
)
def test_out_of_range_args_refused(noise_setup, run_cmd, param, value, text):
    plugin, toolhead, chip = noise_setup()
    with pytest.raises(fake_klippy.FakeCommandError, match=text):
        run_cmd("PLR_NOISE_TEST", START=1, **{param: value})
    assert toolhead.moves == []
