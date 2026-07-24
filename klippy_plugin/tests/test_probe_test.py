"""Tests for the PLR_PROBE_TEST repeatability command."""

import fake_klippy
import pytest

import plr
from plr import probe_test

HEIGHTS = [0.101, 0.103, 0.099, 0.102, 0.100]


@pytest.fixture
def probed(fake_printer, plugin):
    """Wire a canned-heights probe into the good-config plugin."""
    probe = fake_klippy.FakeProbe(fake_printer, HEIGHTS)
    fake_printer.add_object("probe", probe)
    return probe


# --- pure helpers ------------------------------------------------------


def test_minimum_count_is_accepted():
    assert probe_test.validate_sample_count(probe_test.MIN_SAMPLES) == (
        probe_test.MIN_SAMPLES
    )


def test_too_few_samples_rejected_with_console_message():
    with pytest.raises(ValueError, match=r"SAMPLES=1 is too low"):
        probe_test.validate_sample_count(1)


def test_custom_minimum_is_honored():
    assert probe_test.validate_sample_count(2, minimum=2) == 2
    with pytest.raises(ValueError):
        probe_test.validate_sample_count(4, minimum=5)


def test_compute_stats_known_values():
    stats = probe_test.compute_stats([1.0, 2.0, 3.0, 4.0])
    assert stats["range"] == pytest.approx(3.0)
    assert stats["mean"] == pytest.approx(2.5)
    assert stats["median"] == pytest.approx(2.5)  # even count: midpoint
    assert stats["stddev"] == pytest.approx(1.118033988749895)


def test_compute_stats_odd_count_median():
    assert probe_test.compute_stats([3.0, 1.0, 2.0])["median"] == 2.0


def test_compute_stats_rejects_empty():
    with pytest.raises(ValueError, match="no probe samples"):
        probe_test.compute_stats([])


def test_print_active_false_on_bare_printer(fake_printer):
    # Neither print_stats nor idle_timeout configured (minimal setups):
    # nothing indicates an active print, so the gate stays open.
    assert probe_test._print_active(fake_printer) is False


def test_resolution_floor():
    assert probe_test.resolution_from_stddev(0.0) == probe_test.MIN_PROBE_RESOLUTION
    assert probe_test.resolution_from_stddev(0.01) == pytest.approx(0.02)


# --- refusal paths -----------------------------------------------------


def test_refuses_adxl_drag_method(fake_printer, plr_config, run_cmd):
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="adxl_drag"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_while_printing_via_print_stats(probed, fake_printer, run_cmd):
    fake_printer.add_object("print_stats", fake_klippy.FakePrintStats("printing"))
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_while_paused_via_print_stats(probed, fake_printer, run_cmd):
    fake_printer.add_object("print_stats", fake_klippy.FakePrintStats("paused"))
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_on_idle_timeout_fallback(probed, fake_printer, run_cmd):
    # No [virtual_sdcard] -> no print_stats; idle_timeout 'Printing'
    # (recent motion) is the only signal left and must refuse.
    fake_printer.objects["idle_timeout"] = fake_klippy.FakeIdleTimeout("Printing")
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_print_stats_standby_overrides_idle_timeout(probed, fake_printer, run_cmd):
    # print_stats is authoritative when present: standby + idle_timeout
    # 'Printing' (e.g. the positioning move just made) must not refuse.
    fake_printer.add_object("print_stats", fake_klippy.FakePrintStats("standby"))
    fake_printer.objects["idle_timeout"] = fake_klippy.FakeIdleTimeout("Printing")
    gcode = run_cmd("PLR_PROBE_TEST", START=1, SAMPLES=5)
    assert "probe test" in gcode.responses[-1]


def test_refuses_unhomed(probed, fake_printer, run_cmd):
    fake_printer.objects["toolhead"] = fake_klippy.FakeToolhead(homed_axes="xy")
    with pytest.raises(fake_klippy.FakeCommandError, match="G28"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_without_probe_object(plugin, run_cmd):
    with pytest.raises(fake_klippy.FakeCommandError, match=r"\[probe\] section"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_samples_out_of_range_rejected(probed, run_cmd):
    with pytest.raises(fake_klippy.FakeCommandError, match="minimum of 3"):
        run_cmd("PLR_PROBE_TEST", START=1, SAMPLES=2)
    with pytest.raises(fake_klippy.FakeCommandError, match="maximum of 50"):
        run_cmd("PLR_PROBE_TEST", START=1, SAMPLES=51)


def test_without_start_prints_plan_and_does_not_move(probed, fake_printer, run_cmd):
    gcode = run_cmd("PLR_PROBE_TEST", SAMPLES=5)
    plan = gcode.responses[-1]
    assert "START=1" in plan
    assert "no motion" in plan
    assert probed.sessions == []
    assert fake_printer.lookup_object("toolhead").moves == []
    assert fake_printer.lookup_object("configfile").pending == {}


# --- the measurement itself -------------------------------------------


def test_happy_path_mirrors_probe_accuracy_loop(probed, plugin, fake_printer, run_cmd):
    gcode = run_cmd("PLR_PROBE_TEST", START=1, SAMPLES=5)
    (session,) = probed.sessions
    assert session.ended is True
    assert len(session.run_gcmds) == 5
    # Each per-sample command is the PROBE_ACCURACY-style dummy:
    # SAMPLES forced to 1, PROBE_SPEED pinned to the [plr] tunable,
    # and the consent flag stripped.
    fo_params = session.run_gcmds[0].get_command_parameters()
    assert fo_params["SAMPLES"] == "1"
    assert fo_params["PROBE_SPEED"] == "1.500"
    assert "START" not in fo_params
    # Retract between samples: manual_move to trigger z + retract at
    # lift speed, holding the starting XY.
    toolhead = fake_printer.lookup_object("toolhead")
    params = probed.get_probe_params()
    assert len(toolhead.moves) == 5
    for (coord, speed), height in zip(toolhead.moves, HEIGHTS):
        assert coord[0] == 150.0 and coord[1] == 150.0
        assert coord[2] == pytest.approx(height + params["sample_retract_dist"])
        assert speed == params["lift_speed"]
    report = gcode.responses[-1]
    assert "range" in report and "stddev" in report and "median" in report
    assert "SAVE_CONFIG" in report


def test_happy_path_stages_probe_resolution(probed, plugin, fake_printer, run_cmd):
    run_cmd("PLR_PROBE_TEST", START=1, SAMPLES=5)
    stats = probe_test.compute_stats(HEIGHTS)
    expected = probe_test.resolution_from_stddev(stats["stddev"])
    assert plugin.probe_resolution == pytest.approx(expected)
    configfile = fake_printer.lookup_object("configfile")
    staged = configfile.pending["plr"]["probe_resolution"]
    assert staged == "%.6f" % (expected,)
    assert plugin.is_pending_save("probe_resolution")


def test_probe_speed_tunable_reaches_probe_params(
    probed, plugin, fake_printer, run_cmd
):
    plugin.tunables["probe_speed"] = 1.25
    run_cmd("PLR_PROBE_TEST", START=1, SAMPLES=3)
    (session,) = probed.sessions
    assert session.run_gcmds[0].get_float("PROBE_SPEED") == 1.25


def test_session_ended_even_when_a_probe_fails(probed, fake_printer, run_cmd):
    # Only 5 canned heights; asking for more makes run_probe raise
    # mid-loop.  The session must still be closed (probe.py's
    # gcode:command_error handler expects sessions not to leak).
    with pytest.raises(fake_klippy.FakeCommandError, match="canned heights"):
        run_cmd("PLR_PROBE_TEST", START=1, SAMPLES=10)
    (session,) = probed.sessions
    assert session.ended is True
    # Nothing staged on failure.
    assert fake_printer.lookup_object("configfile").pending == {}
