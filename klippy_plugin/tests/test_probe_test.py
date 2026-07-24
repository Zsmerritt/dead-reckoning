"""Tests for the reworked, two-tier PLR_PROBE_TEST verification command."""

import fake_klippy
import pytest

import plr
from plr import probe_test


class ScriptedProbe(fake_klippy.FakeProbe):
    """A probe whose successive sessions serve different height lists.

    PLR_PROBE_TEST opens one probe session per consensus sequence; this
    fake pops the next scripted height list for each, so tests can make
    the per-sequence medians agree or diverge on purpose.
    """

    def __init__(self, printer, scripts, **kw):
        super().__init__(printer, [], **kw)
        self._scripts = list(scripts)

    def start_probe_session(self, gcmd):
        toolhead = self._printer.lookup_object("toolhead")
        heights = self._scripts.pop(0)
        session = fake_klippy.FakeProbeSession(self, heights, toolhead)
        self.sessions.append(session)
        return session


@pytest.fixture
def scripted(fake_printer, plugin):
    def build(scripts):
        probe = ScriptedProbe(fake_printer, scripts)
        fake_printer.add_object("probe", probe)
        return probe

    return build


def extract_retry(message, command):
    for line in message.splitlines():
        stripped = line.strip()
        if stripped.startswith(command + " ") and "MAX_SAMPLES=" in stripped:
            return stripped
    raise AssertionError("no retry command found in:\n%s" % (message,))


def parse_line(line):
    parts = line.split()
    params = {}
    for token in parts[1:]:
        key, _, value = token.partition("=")
        params[key] = value
    return parts[0], params


# --- pure helpers -----------------------------------------------------


def test_compute_stats_known_values():
    stats = probe_test.compute_stats([1.0, 2.0, 3.0, 4.0])
    assert stats["range"] == pytest.approx(3.0)
    assert stats["mean"] == pytest.approx(2.5)
    assert stats["median"] == pytest.approx(2.5)
    assert stats["stddev"] == pytest.approx(1.118033988749895)


def test_compute_stats_odd_count_median():
    assert probe_test.compute_stats([3.0, 1.0, 2.0])["median"] == 2.0


def test_compute_stats_rejects_empty():
    with pytest.raises(ValueError, match="no samples"):
        probe_test.compute_stats([])


def test_resolution_formula_each_term_can_dominate():
    # Microstep floor dominates when the medians barely moved.
    assert probe_test.resolution_from_medians(0.0, 0.0) == pytest.approx(0.005)
    # 2*stddev dominates when the spread is scattered.
    assert probe_test.resolution_from_medians(0.0, 0.01) == pytest.approx(0.02)
    # median_range/2 dominates when the peak-to-peak swing is large.
    assert probe_test.resolution_from_medians(0.02, 0.0) == pytest.approx(0.01)


def test_print_active_false_on_bare_printer(fake_printer):
    assert probe_test._print_active(fake_printer) is False


# --- refusal paths ----------------------------------------------------


def test_refuses_adxl_drag_method(fake_printer, plr_config, run_cmd):
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="adxl_drag"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_while_printing(scripted, fake_printer, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    fake_printer.add_object("print_stats", fake_klippy.FakePrintStats("printing"))
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_while_paused(scripted, fake_printer, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    fake_printer.add_object("print_stats", fake_klippy.FakePrintStats("paused"))
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_on_idle_timeout_fallback(scripted, fake_printer, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    fake_printer.objects["idle_timeout"] = fake_klippy.FakeIdleTimeout("Printing")
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_unhomed(scripted, fake_printer, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    fake_printer.objects["toolhead"] = fake_klippy.FakeToolhead(homed_axes="xy")
    with pytest.raises(fake_klippy.FakeCommandError, match="G28"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_refuses_without_probe_object(plugin, run_cmd):
    with pytest.raises(fake_klippy.FakeCommandError, match=r"\[probe\] section"):
        run_cmd("PLR_PROBE_TEST", START=1)


def test_sequences_out_of_range(scripted, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    with pytest.raises(fake_klippy.FakeCommandError, match="minimum of 3"):
        run_cmd("PLR_PROBE_TEST", START=1, SEQUENCES=2)
    with pytest.raises(fake_klippy.FakeCommandError, match="maximum of 10"):
        run_cmd("PLR_PROBE_TEST", START=1, SEQUENCES=11)


def test_samples_below_min_refused(scripted, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    with pytest.raises(fake_klippy.FakeCommandError, match="minimum of 3"):
        run_cmd("PLR_PROBE_TEST", START=1, SAMPLES=2)


def test_sample_range_over_cap_refused(scripted, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    with pytest.raises(fake_klippy.FakeCommandError, match="hard cap of 0.015"):
        run_cmd("PLR_PROBE_TEST", START=1, SAMPLE_RANGE=0.05)


# VERIFY_RANGE relational checks are refused loudly, not clamped.
def test_verify_range_below_sample_range_refused(scripted, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    with pytest.raises(fake_klippy.FakeCommandError, match="must be >= SAMPLE_RANGE"):
        run_cmd("PLR_PROBE_TEST", START=1, SAMPLE_RANGE=0.010, VERIFY_RANGE=0.005)


def test_verify_range_above_four_x_refused(scripted, run_cmd):
    scripted([[0.1, 0.1, 0.1]])
    with pytest.raises(fake_klippy.FakeCommandError, match="must be <= 4x"):
        run_cmd("PLR_PROBE_TEST", START=1, SAMPLE_RANGE=0.010, VERIFY_RANGE=0.05)


def test_without_start_prints_plan_and_does_not_move(scripted, fake_printer, run_cmd):
    probe = scripted([[0.1, 0.1, 0.1]])
    gcode = run_cmd("PLR_PROBE_TEST", SEQUENCES=4)
    plan = gcode.responses[-1]
    assert "START=1" in plan
    assert "no motion" in plan
    assert probe.sessions == []
    assert fake_printer.lookup_object("configfile").pending == {}


# --- the verification itself ------------------------------------------


def test_happy_path_runs_sequences_and_stages_resolution(
    scripted, plugin, fake_printer, run_cmd
):
    probe = scripted([[0.100, 0.101, 0.102]] * 3)
    gcode = run_cmd(
        "PLR_PROBE_TEST", START=1, SEQUENCES=3, SAMPLES=3, SAMPLE_RANGE=0.010
    )
    report = gcode.responses[-1]
    assert "probe_resolution" in report and "SAVE_CONFIG" in report
    assert len(probe.sessions) == 3
    # Medians all 0.101 -> range 0 -> resolution hits the microstep floor.
    assert plugin.probe_resolution == pytest.approx(0.005)
    staged = fake_printer.lookup_object("configfile").pending["plr"]
    assert staged["probe_resolution"] == "0.005000"
    assert plugin.is_pending_save("probe_resolution")


def test_inconsistent_medians_fail_with_early_exit_and_retry(
    scripted, plugin, fake_printer, run_cmd
):
    probe = scripted(
        [
            [0.100, 0.101, 0.102],  # median 0.101
            [0.130, 0.131, 0.132],  # median 0.131 -> running range 0.030 > 0.020
            [0.160, 0.161, 0.162],  # never reached (early exit)
        ]
    )
    with pytest.raises(fake_klippy.FakeCommandError) as exc:
        run_cmd("PLR_PROBE_TEST", START=1, SEQUENCES=3, SAMPLES=3, SAMPLE_RANGE=0.010)
    message = str(exc.value)
    assert "medians disagree" in message
    # Early-exit: the third sequence never ran.
    assert len(probe.sessions) == 2
    # Nothing staged on failure.
    assert fake_printer.lookup_object("configfile").pending == {}
    assert not plugin.is_pending_save("probe_resolution")
    # Retry escalates SEQUENCES and loosens VERIFY_RANGE (capped), and
    # parses back cleanly.
    retry = extract_retry(message, "PLR_PROBE_TEST")
    name, params = parse_line(retry)
    assert name == "PLR_PROBE_TEST"
    assert int(params["SEQUENCES"]) == 5  # ceil(3*1.5)
    assert float(params["VERIFY_RANGE"]) == pytest.approx(0.030)  # min(4x, 1.5x)


def test_sequence_cannot_reach_consensus_fails_loudly(scripted, run_cmd):
    # Ten scattered heights: the very first sequence never reaches
    # consensus, so the whole test fails naming the consensus criteria.
    probe = scripted([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]])
    with pytest.raises(fake_klippy.FakeCommandError) as exc:
        run_cmd(
            "PLR_PROBE_TEST",
            START=1,
            SEQUENCES=3,
            SAMPLES=3,
            SAMPLE_RANGE=0.010,
            MAX_SAMPLES=10,
        )
    message = str(exc.value)
    assert "could not reach consensus" in message
    assert len(probe.sessions) == 1
    retry = extract_retry(message, "PLR_PROBE_TEST")
    _name, params = parse_line(retry)
    assert params["MAX_SAMPLES"] == "15"  # per-sequence budget escalated


def test_passing_but_nonzero_range_reports_and_stages(
    scripted, plugin, fake_printer, run_cmd
):
    # Medians 0.100 / 0.101 / 0.102 -> range 0.002 <= VERIFY_RANGE 0.020.
    scripted(
        [
            [0.099, 0.100, 0.101],  # median 0.100
            [0.100, 0.101, 0.102],  # median 0.101
            [0.101, 0.102, 0.103],  # median 0.102
        ]
    )
    gcode = run_cmd(
        "PLR_PROBE_TEST", START=1, SEQUENCES=3, SAMPLES=3, SAMPLE_RANGE=0.010
    )
    report = gcode.responses[-1]
    assert "median range" in report
    assert plugin.probe_resolution >= 0.005
    assert plugin.is_pending_save("probe_resolution")
