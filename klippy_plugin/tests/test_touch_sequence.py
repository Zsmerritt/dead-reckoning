"""Tests for the consensus touch engine (plr.touch_sequence).

Covers the pure sliding-window sampler, the safety-invariant orchestrator,
and the PLR_TOUCH command, including the hostile-review acceptance
criteria: anti-cherry-pick windowing, accel restore on every path,
retract invariants over the move list, the SAMPLE_RANGE hard-cap refusal,
the window bound refusal, and a parseable escalated retry command.
"""

import fake_klippy
import pytest

import plr
from plr import touch_sequence as ts

# --- helpers ----------------------------------------------------------


def stream(values):
    """A ``probe_once`` callable yielding ``values`` in order."""
    it = iter(values)

    def probe_once():
        return next(it)

    return probe_once


def make_params(**kw):
    """A TouchParams with sane defaults, overridable per test."""
    config_kw = {
        key: kw.pop(key)
        for key in ("samples", "max_samples", "max_noisy", "sample_range")
        if key in kw
    }
    config = ts.ConsensusConfig(**config_kw)
    defaults = {"speed": 1.5, "retract": 2.0, "touch_accel": 100.0}
    defaults.update(kw)
    return ts.TouchParams(config=config, **defaults)


def extract_retry(message, command):
    """Pull the copy-pasteable retry line out of a failure message."""
    for line in message.splitlines():
        stripped = line.strip()
        if stripped.startswith(command + " ") and "MAX_SAMPLES=" in stripped:
            return stripped
    raise AssertionError("no retry command found in:\n%s" % (message,))


def parse_line(line):
    """Split ``NAME K=V K=V`` into (name, {K: V}) like klippy's parser."""
    parts = line.split()
    params = {}
    for token in parts[1:]:
        key, _, value = token.partition("=")
        params[key] = value
    return parts[0], params


# --- pure helpers -----------------------------------------------------


def test_compute_range_needs_two_samples():
    assert ts.compute_range([]) == float("inf")
    assert ts.compute_range([1.0]) == float("inf")
    assert ts.compute_range([1.0, 1.5]) == pytest.approx(0.5)
    assert ts.compute_range([0.1, 0.3, 0.2]) == pytest.approx(0.2)


def test_find_best_subset_picks_tightest_group():
    window = [0.0, 0.1, 0.11, 0.12, 5.0]
    best = ts.find_best_subset(window, 3)
    assert set(best) == {0.1, 0.11, 0.12}
    assert ts.compute_range(best) == pytest.approx(0.02)


def test_find_best_subset_none_when_window_too_small():
    assert ts.find_best_subset([1.0, 2.0], 3) is None


def test_find_best_subset_refuses_oversized_window():
    with pytest.raises(ValueError, match="exceeds the maximum"):
        ts.find_best_subset(list(range(11)), 3)


def test_median_odd_and_even():
    assert ts._median([3.0, 1.0, 2.0]) == 2.0
    assert ts._median([1.0, 2.0, 3.0, 4.0]) == pytest.approx(2.5)


def test_format_distance_rounds_up_and_handles_infinity():
    # A real nonzero spread must never print as 0.000.
    assert ts.format_distance(0.0004) == "0.001"
    assert ts.format_distance(0.010) == "0.010"
    assert ts.format_distance(0.0) == "0.000"
    assert ts.format_distance(float("inf")) == "inf"


# --- ConsensusConfig validation (refusals, acceptance #4 and #5) ------


def test_config_defaults_window():
    config = ts.ConsensusConfig()
    assert config.samples == 3
    assert config.max_samples == 10
    assert config.window == 5


def test_config_refuses_too_few_samples():
    with pytest.raises(ValueError, match="at least 3"):
        ts.ConsensusConfig(samples=2)


def test_config_refuses_negative_max_noisy():
    with pytest.raises(ValueError, match="max_noisy"):
        ts.ConsensusConfig(max_noisy=-1)


def test_config_refuses_max_samples_below_samples():
    with pytest.raises(ValueError, match="must be >= SAMPLES"):
        ts.ConsensusConfig(samples=5, max_samples=4)


def test_config_refuses_max_samples_over_cap():
    # Acceptance #5: max_samples > 20 refused.
    with pytest.raises(ValueError, match="exceeds the maximum of 20"):
        ts.ConsensusConfig(samples=3, max_samples=21)


def test_config_refuses_window_over_bound():
    # Acceptance #5: window > 10 refused (samples 9 + max_noisy 2 = 11).
    with pytest.raises(ValueError, match="window 11"):
        ts.ConsensusConfig(samples=9, max_samples=9)


def test_config_refuses_nonpositive_sample_range():
    with pytest.raises(ValueError, match="positive distance"):
        ts.ConsensusConfig(sample_range=0.0)


def test_config_refuses_sample_range_over_hard_cap():
    # Acceptance #4: SAMPLE_RANGE over the cap is a refusal naming the cap,
    # not a clamp.
    with pytest.raises(ValueError, match="hard cap of 0.015"):
        ts.ConsensusConfig(sample_range=0.05)


def test_config_accepts_exactly_the_cap():
    assert ts.ConsensusConfig(sample_range=0.015).sample_range == 0.015


# --- run_consensus: the sliding-window sampler ------------------------


def test_run_consensus_accepts_first_tight_window():
    result = ts.run_consensus(
        stream([0.100, 0.101, 0.102]),
        ts.ConsensusConfig(samples=3, sample_range=0.010),
    )
    assert result.median == pytest.approx(0.101)
    assert result.range == pytest.approx(0.002)
    assert result.touches_used == 3
    assert set(result.subset) == {0.100, 0.101, 0.102}
    assert result.all_samples == (0.100, 0.101, 0.102)


def test_run_consensus_logs_each_touch():
    logs = []
    ts.run_consensus(
        stream([0.100, 0.101, 0.102]),
        ts.ConsensusConfig(samples=3, sample_range=0.010),
        log=logs.append,
    )
    assert sum(1 for line in logs if line.startswith("touch ")) == 3
    assert any("consensus" in line for line in logs)


def test_run_consensus_keeps_collecting_when_no_subset(monkeypatch):
    # find_best_subset returning None means "the window yielded no subset,
    # keep collecting"; with it always None the loop exhausts its budget.
    monkeypatch.setattr(ts, "find_best_subset", lambda window, size: None)
    config = ts.ConsensusConfig(samples=3, max_samples=4, sample_range=0.010)
    with pytest.raises(ts.ConsensusError) as exc:
        ts.run_consensus(stream([0.1, 0.1, 0.1, 0.1]), config)
    assert exc.value.touches == 4


def test_run_consensus_exhausts_into_typed_error():
    config = ts.ConsensusConfig(samples=3, max_samples=5, sample_range=0.010)
    with pytest.raises(ts.ConsensusError) as exc:
        ts.run_consensus(stream([1.0, 2.0, 3.0, 4.0, 5.0]), config)
    err = exc.value
    assert err.samples == 3
    assert err.sample_range == 0.010
    assert err.window == 5
    assert err.touches == 5
    assert err.all_samples == (1.0, 2.0, 3.0, 4.0, 5.0)


# Acceptance #1: anti-cherry-pick. A stream whose good touches are
# scattered so no sliding window ever holds `samples` of them MUST fail,
# even though a GLOBAL subset search would pass.
def test_scattered_good_samples_never_pass_a_window():
    config = ts.ConsensusConfig(samples=3, max_samples=10, sample_range=0.010)
    # Goods (0.000, 0.005, 0.010) sit at indices 0, 4, 9 — never three in
    # any window of 5; every other value is far from any pair.
    scattered = [0.000, 1.0, 2.0, 3.0, 0.005, 4.0, 5.0, 6.0, 7.0, 0.010]
    with pytest.raises(ts.ConsensusError):
        ts.run_consensus(stream(scattered), config)
    # ...yet a naive global search WOULD find a passing triple, which is
    # exactly the cherry-pick the window forbids.
    global_best = ts.find_best_subset(scattered, 3)
    assert ts.compute_range(global_best) <= config.sample_range


# Acceptance #1 converse: noisy early, consistent late MUST pass, using
# only the late samples.
def test_noisy_early_consistent_late_passes_on_late_samples():
    config = ts.ConsensusConfig(samples=3, max_samples=10, sample_range=0.010)
    result = ts.run_consensus(stream([5.0, 6.0, 7.0, 0.100, 0.101, 0.102]), config)
    assert result.touches_used == 6
    assert result.median == pytest.approx(0.101)
    assert set(result.subset) <= {0.100, 0.101, 0.102}


# --- orchestrator: safety invariants ----------------------------------


@pytest.fixture
def touch_probe(fake_printer, plugin):
    """Wire a canned-heights probe into the good-config plugin."""

    def build(heights, **kw):
        probe = fake_klippy.FakeProbe(fake_printer, heights, **kw)
        fake_printer.add_object("probe", probe)
        return probe

    return build


def test_orchestrator_happy_path_returns_result(touch_probe, plugin):
    touch_probe([0.100, 0.101, 0.102])
    result = ts.perform_consensus_touch(plugin, make_params(sample_range=0.010))
    assert result.median == pytest.approx(0.101)
    assert result.touches_used == 3


# Acceptance #2: accel is restored on EVERY path, including a probe that
# raises mid-sequence.
def test_accel_clamped_then_restored_each_touch(touch_probe, fake_printer, plugin):
    touch_probe([0.100, 0.101, 0.102])
    toolhead = fake_printer.lookup_object("toolhead")
    ts.perform_consensus_touch(plugin, make_params(touch_accel=100.0))
    # Three touches: clamp to 100 then restore to 3000, three times.
    accels = [entry[1] for entry in toolhead.velocity_limits]
    assert accels == [100.0, 3000.0, 100.0, 3000.0, 100.0, 3000.0]
    assert toolhead.max_accel == 3000.0


def test_accel_restored_when_probe_raises_mid_sequence(
    touch_probe, fake_printer, plugin
):
    # Only one canned height: the second touch's run_probe raises.
    touch_probe([0.100])
    toolhead = fake_printer.lookup_object("toolhead")
    with pytest.raises(fake_klippy.FakeCommandError, match="canned heights"):
        ts.perform_consensus_touch(plugin, make_params(touch_accel=100.0))
    # The clamp was applied (100 seen) and restored despite the raise.
    accels = [entry[1] for entry in toolhead.velocity_limits]
    assert 100.0 in accels
    assert accels[-1] == 3000.0
    assert toolhead.max_accel == 3000.0


# Acceptance #3: over a multi-touch run, no probe descent begins below
# the retract height and every touch is followed by a retract.
def test_retract_invariants_over_the_move_list(touch_probe, fake_printer, plugin):
    probe = touch_probe([0.50, 0.60, 0.100, 0.101, 0.102])
    toolhead = fake_printer.lookup_object("toolhead")
    result = ts.perform_consensus_touch(
        plugin, make_params(samples=3, sample_range=0.010, retract=2.0)
    )
    assert result.touches_used == 5
    (session,) = probe.sessions
    # No descent ever begins below the retract height.
    assert all(z >= 2.0 for z in session.probe_start_zs)
    assert len(session.probe_start_zs) == 5
    # Every touch is followed by a retract-after move (never below retract),
    # and the toolhead ends parked at/above the retract height.
    assert len(toolhead.moves) == 5
    assert all(coord[2] >= 2.0 for coord, _speed in toolhead.moves)
    assert toolhead.get_position()[2] >= 2.0


def test_retract_before_arm_lifts_from_below(touch_probe, fake_printer, plugin):
    # Start the toolhead below the retract height: the first touch must
    # lift to retract BEFORE arming, so its descent still begins at >=2.0.
    fake_printer.objects["toolhead"] = fake_klippy.FakeToolhead(
        position=(150.0, 150.0, 0.5, 0.0)
    )
    probe = touch_probe([0.100, 0.101, 0.102])
    ts.perform_consensus_touch(plugin, make_params(samples=3, retract=2.0))
    (session,) = probe.sessions
    assert session.probe_start_zs[0] >= 2.0


def test_retract_after_lifts_to_retract_floor(touch_probe, fake_printer, plugin):
    # A trigger far above retract still retracts to trigger+retract.
    probe = touch_probe([1.00, 1.001, 1.002])
    toolhead = fake_printer.lookup_object("toolhead")
    ts.perform_consensus_touch(plugin, make_params(samples=3, retract=2.0))
    # Last retract target is max(trigger+retract, retract) = 1.002 + 2.0.
    assert toolhead.moves[-1][0][2] == pytest.approx(3.002)
    assert probe.sessions[0].ended is True


# --- PLR_TOUCH command ------------------------------------------------


def test_touch_happy_path_sets_status_and_reports(touch_probe, plugin, run_cmd):
    touch_probe([0.100, 0.101, 0.102])
    gcode = run_cmd("PLR_TOUCH", SAMPLES=3, SAMPLE_RANGE=0.010)
    report = gcode.responses[-1]
    assert "median" in report and "range" in report
    assert plugin.last_touch_result == {
        "median_z": pytest.approx(0.101),
        "range": pytest.approx(0.002),
        "samples_used": 3,
        "touches": 3,
    }
    # And the result is visible through get_status for the Rust side.
    status = plugin.get_status(100.0)["last_touch_result"]
    assert status["median_z"] == pytest.approx(0.101)


def test_touch_refuses_adxl_drag(fake_printer, plr_config, run_cmd):
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="adxl_drag"):
        run_cmd("PLR_TOUCH")


def test_touch_refuses_while_printing(touch_probe, fake_printer, run_cmd):
    touch_probe([0.1, 0.1, 0.1])
    fake_printer.add_object("print_stats", fake_klippy.FakePrintStats("printing"))
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_TOUCH")


def test_touch_refuses_unhomed(touch_probe, fake_printer, run_cmd):
    touch_probe([0.1, 0.1, 0.1])
    fake_printer.objects["toolhead"] = fake_klippy.FakeToolhead(homed_axes="xy")
    with pytest.raises(fake_klippy.FakeCommandError, match="G28"):
        run_cmd("PLR_TOUCH")


def test_touch_refuses_without_probe(plugin, run_cmd):
    with pytest.raises(fake_klippy.FakeCommandError, match=r"\[probe\] section"):
        run_cmd("PLR_TOUCH")


# Acceptance #4: SAMPLE_RANGE over the cap is refused naming the cap.
def test_touch_refuses_sample_range_over_cap(touch_probe, run_cmd):
    touch_probe([0.1, 0.1, 0.1])
    with pytest.raises(fake_klippy.FakeCommandError, match="hard cap of 0.015"):
        run_cmd("PLR_TOUCH", SAMPLE_RANGE=0.05)


# Acceptance #5: an over-cap MAX_SAMPLES / window is refused.
def test_touch_refuses_max_samples_over_cap(touch_probe, run_cmd):
    touch_probe([0.1, 0.1, 0.1])
    with pytest.raises(fake_klippy.FakeCommandError, match="exceeds the maximum of 20"):
        run_cmd("PLR_TOUCH", MAX_SAMPLES=25)


def test_touch_refuses_window_over_bound(touch_probe, run_cmd):
    touch_probe([0.1, 0.1, 0.1])
    with pytest.raises(fake_klippy.FakeCommandError, match="window 11"):
        run_cmd("PLR_TOUCH", SAMPLES=9, MAX_SAMPLES=9)


# Acceptance #6: the failure text carries a syntactically valid retry
# command that parses back through the gcmd param parser, with
# MAX_SAMPLES escalated.
def test_touch_failure_has_parseable_escalated_retry(touch_probe, plugin, run_cmd):
    # Ten scattered heights: no window of 5 ever holds 3 within range.
    touch_probe([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0])
    with pytest.raises(fake_klippy.FakeCommandError) as exc:
        run_cmd("PLR_TOUCH", SAMPLES=3, SAMPLE_RANGE=0.010)
    message = str(exc.value)
    assert plugin.last_touch_result is None
    assert "sliding window" in message
    retry = extract_retry(message, "PLR_TOUCH")
    name, params = parse_line(retry)
    assert name == "PLR_TOUCH"
    # MAX_SAMPLES escalated 1.5x (ceil): 10 -> 15.
    assert params["MAX_SAMPLES"] == "15"
    # The retry parses cleanly back through the shared param parser.
    gcode = plugin.printer.lookup_object("gcode")
    gcmd = gcode.create_gcode_command(name, retry, params)
    reparsed = ts.parse_touch_params(gcmd, plugin.tunables["probe_speed"])
    assert reparsed.config.max_samples == 15


def test_escalated_max_samples_caps_at_hard_maximum():
    assert ts.escalated_max_samples(10) == 15
    assert ts.escalated_max_samples(18) == 20  # ceil(27) capped at 20
    assert ts.escalated_max_samples(20) == 20
