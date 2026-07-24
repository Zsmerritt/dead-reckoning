"""Tests for verdict combination and the drag-oracle pass classifier.

The classifier gets REAL math tests over synthetic streams
(stream_fixtures): no faked physics, no mocked numerics.  The numpy
path and the pure-python fallback are pinned against each other on the
same fixtures.
"""

import math

import pytest
import stream_fixtures as sf

from plr import classifier

# ---------------------------------------------------------------------
# Verdict vocabulary


@pytest.mark.parametrize(
    ("verdicts", "expected"),
    [
        (["pass"], "pass"),
        (["pass", "warn", "pass"], "warn"),
        (["warn", "fail", "pass"], "fail"),
        (["fail"], "fail"),
        (("pass", "pass"), "pass"),  # any iterable, not just list
    ],
)
def test_worst_verdict(verdicts, expected):
    assert classifier.worst_verdict(verdicts) == expected


def test_unknown_verdict_rejected():
    with pytest.raises(ValueError, match="unknown verdict 'maybe'"):
        classifier.worst_verdict(["pass", "maybe"])


def test_empty_iterable_rejected():
    with pytest.raises(ValueError, match="no verdicts"):
        classifier.worst_verdict([])


def test_vocabulary_and_severity_stay_in_sync():
    assert set(classifier.VERDICTS) == set(classifier._SEVERITY)


# ---------------------------------------------------------------------
# Sensitivity -> multiplier mapping


@pytest.mark.parametrize(
    ("knob", "expected"),
    [(0.0, 8.0), (50.0, 4.0), (100.0, 1.5)],
)
def test_multiplier_anchors_exact(knob, expected):
    assert classifier.multiplier(knob) == pytest.approx(expected)


def test_multiplier_log_interpolates_between_anchors():
    # Log interpolation: the midpoint multiplier is the geometric mean
    # of the surrounding anchors.
    assert classifier.multiplier(25.0) == pytest.approx(math.sqrt(8.0 * 4.0))
    assert classifier.multiplier(75.0) == pytest.approx(math.sqrt(4.0 * 1.5))


def test_multiplier_monotone_decreasing():
    knobs = [i * 2.5 for i in range(41)]  # 0..100
    values = [classifier.multiplier(k) for k in knobs]
    assert all(a > b for a, b in zip(values, values[1:]))


@pytest.mark.parametrize("bad", [-0.1, 100.1, float("nan"), float("inf"), "high", None])
def test_multiplier_rejects_out_of_domain(bad):
    with pytest.raises(ValueError):
        classifier.multiplier(bad)


def test_multiplier_accepts_int_knob():
    assert classifier.multiplier(50) == pytest.approx(4.0)


# ---------------------------------------------------------------------
# Degenerate-input taxonomy


def test_too_few_samples_refused():
    result = classifier.validate_pass_samples(sf.quiet(n=127))
    assert result.reason == classifier.INVALID_TOO_FEW


def test_min_samples_boundary_accepted():
    assert classifier.validate_pass_samples(sf.quiet(n=128)) is None


@pytest.mark.parametrize("field", [0, 1, 2, 3])
def test_non_finite_refused_in_any_field(field):
    stream = sf.with_nan(sf.quiet(), index=300, field=field)
    result = classifier.validate_pass_samples(stream)
    assert result.reason == classifier.INVALID_NON_FINITE


@pytest.mark.parametrize(
    "value", [(0.0, 0.0, 0.0), (0.0, 0.0, sf.GRAVITY), (1.5, -2.5, 3.5)]
)
def test_constant_signal_refused(value):
    result = classifier.validate_pass_samples(sf.constant(value=value))
    assert result.reason == classifier.INVALID_CONSTANT


def test_dropout_gap_refused():
    result = classifier.validate_pass_samples(sf.with_dropout(sf.quiet()))
    assert result.reason == classifier.INVALID_RATE_COLLAPSE
    assert "gap" in result.detail


def test_dropout_refused_with_even_dt_count():
    # 129 samples -> 128 gaps: exercises the even-length median branch
    # of the dt-gap check.
    result = classifier.validate_pass_samples(
        sf.with_dropout(sf.quiet(n=129), at_frac=0.5)
    )
    assert result.reason == classifier.INVALID_RATE_COLLAPSE


def test_fallback_path_with_odd_sample_count():
    # Odd n exercises the pure-python _median odd branch.
    verdict = classifier.classify_pass(
        sf.quiet(n=129), QUIET_FLOOR, 50.0, use_numpy=False
    )
    assert verdict.contact is False


def test_non_increasing_timestamps_refused():
    stream = sf.quiet()
    stream[10] = (stream[11][0], stream[10][1], stream[10][2], stream[10][3])
    result = classifier.validate_pass_samples(stream)
    assert result.reason == classifier.INVALID_RATE_COLLAPSE


def test_taxonomy_is_total_no_exception_escapes():
    """Every hostile input yields PassVerdict or PassInvalid, never a
    raise (given valid noise floor and sensitivity)."""
    hostiles = [
        [],
        sf.quiet(n=1),
        sf.quiet(n=127),
        sf.constant(),
        sf.constant(value=(0.0, 0.0, 0.0)),
        sf.with_nan(sf.quiet()),
        sf.with_nan(sf.quiet(), index=0, field=0),
        sf.with_dropout(sf.quiet()),
        sf.with_dropout(sf.quiet(), at_frac=0.01),
        sf.quiet(),
        sf.wobbly(),
        sf.with_drift(sf.quiet()),
        sf.with_contact(sf.quiet(), amplitude=500.0),
    ]
    for stream in hostiles:
        result = classifier.classify_pass(stream, 10.0, 50.0)
        assert isinstance(result, (classifier.PassVerdict, classifier.PassInvalid)), (
            result
        )


def test_invalid_reasons_vocabulary_closed():
    assert set(classifier.INVALID_REASONS) == {
        "too_few_samples",
        "non_finite",
        "constant_signal",
        "sample_rate_collapse",
    }


# ---------------------------------------------------------------------
# Classification behavior

# Noise floor measured the way PLR_NOISE_TEST measures it: stream RMS
# of a baseline capture from the same generator family (different
# seed), keeping the tests self-consistent with the real flow.
QUIET_FLOOR = classifier.stream_stats(sf.quiet(seed=99)).rms
WOBBLY_FLOOR = classifier.stream_stats(sf.wobbly(seed=98)).rms


def test_quiet_baseline_not_contact_at_default():
    verdict = classifier.classify_pass(sf.quiet(), QUIET_FLOOR, 50.0)
    assert verdict.contact is False


@pytest.mark.parametrize("knob", [0.0, 20.0, 40.0, 60.0, 80.0])
@pytest.mark.parametrize("seed", [1, 7, 42])
def test_no_contact_on_pure_baseline_up_to_knob_80(knob, seed):
    """A pure baseline must never trigger at any knob <= 80 (multiplier
    >= ~2.2): windowed-RMS peaks of stationary noise stay well inside
    2x the stream RMS."""
    verdict = classifier.classify_pass(sf.quiet(seed=seed), QUIET_FLOOR, knob)
    assert verdict.contact is False


@pytest.mark.parametrize("amplitude", [200.0, 500.0, 2000.0])
def test_contact_burst_detected_at_default_knob(amplitude):
    stream = sf.with_contact(sf.quiet(), amplitude=amplitude)
    verdict = classifier.classify_pass(stream, QUIET_FLOOR, 50.0)
    assert verdict.contact is True
    assert verdict.ratio > 1.0


def test_wobbly_baseline_with_wobbly_floor_not_contact():
    """The wobbly-machine story: a noisy baseline is clean as long as
    the noise floor was measured on the same machine."""
    verdict = classifier.classify_pass(sf.wobbly(), WOBBLY_FLOOR, 30.0)
    assert verdict.contact is False


def test_drift_ramp_not_contact_at_default():
    stream = sf.with_drift(sf.quiet(), ramp=20.0)
    verdict = classifier.classify_pass(stream, QUIET_FLOOR, 50.0)
    assert verdict.contact is False


@pytest.mark.parametrize("amplitude", [30.0, 60.0, 120.0, 300.0, 900.0])
def test_monotone_in_sensitivity(amplitude):
    """Higher knob => contact detected at lower SNR: the detection
    indicator over the knob axis is monotone non-decreasing for any
    fixed stream."""
    stream = sf.with_contact(sf.quiet(), amplitude=amplitude)
    knobs = [0.0, 12.5, 25.0, 37.5, 50.0, 62.5, 75.0, 87.5, 100.0]
    detections = [
        classifier.classify_pass(stream, QUIET_FLOOR, k).contact for k in knobs
    ]
    assert all(a <= b for a, b in zip(detections, detections[1:])), detections


def test_higher_knob_catches_fainter_contact():
    """End-to-end sensitivity meaning: an amplitude exists that knob
    100 detects and knob 0 does not."""
    stream = sf.with_contact(sf.quiet(), amplitude=45.0)
    assert classifier.classify_pass(stream, QUIET_FLOOR, 100.0).contact is True
    assert classifier.classify_pass(stream, QUIET_FLOOR, 0.0).contact is False


def test_verdict_fields_are_consistent():
    stream = sf.with_contact(sf.quiet(), amplitude=500.0)
    verdict = classifier.classify_pass(stream, QUIET_FLOOR, 50.0)
    threshold = QUIET_FLOOR * classifier.multiplier(50.0)
    assert verdict.ratio == pytest.approx(verdict.peak_rms / threshold)
    assert verdict.peak_rms > 0.0


# ---------------------------------------------------------------------
# Confidence


def test_confidence_bounded_and_zero_at_threshold():
    assert classifier._confidence(1.0) == pytest.approx(0.0)
    for ratio in [0.01, 0.5, 0.99, 1.01, 2.0, 100.0]:
        assert 0.0 <= classifier._confidence(ratio) <= 1.0


def test_confidence_monotone_away_from_threshold():
    above = [1.01, 1.5, 2.0, 3.0, 10.0]
    values = [classifier._confidence(r) for r in above]
    assert all(a < b for a, b in zip(values, values[1:]))
    below = [0.99, 0.66, 0.5, 0.33, 0.1]
    values = [classifier._confidence(r) for r in below]
    assert all(a < b for a, b in zip(values, values[1:]))


def test_confidence_examples_documented():
    assert classifier._confidence(2.0) == pytest.approx(0.6)
    assert classifier._confidence(0.5) == pytest.approx(0.6)
    assert classifier._confidence(3.0) == pytest.approx(0.8)


# ---------------------------------------------------------------------
# Caller-bug guards (typed ValueError, never a silent classification)


@pytest.mark.parametrize("floor", [0.0, -1.0, float("nan"), float("inf"), None, "loud"])
def test_bad_noise_floor_raises(floor):
    with pytest.raises(ValueError):
        classifier.classify_pass(sf.quiet(), floor, 50.0)


def test_bad_sensitivity_raises():
    with pytest.raises(ValueError):
        classifier.classify_pass(sf.quiet(), 10.0, 101.0)


def test_use_numpy_true_without_numpy_raises(monkeypatch):
    monkeypatch.setattr(classifier, "_np", None)
    with pytest.raises(ValueError, match="numpy is not importable"):
        classifier.classify_pass(sf.quiet(), 10.0, 50.0, use_numpy=True)


def test_default_path_without_numpy_falls_back(monkeypatch):
    monkeypatch.setattr(classifier, "_np", None)
    verdict = classifier.classify_pass(sf.quiet(), QUIET_FLOOR, 50.0)
    assert isinstance(verdict, classifier.PassVerdict)


# ---------------------------------------------------------------------
# Stream stats (the PLR_NOISE_TEST statistic)


def test_stream_stats_peak_at_least_rms_shape():
    stats = classifier.stream_stats(sf.wobbly())
    assert stats.rms > 0.0
    # The max windowed RMS cannot be far below the whole-stream RMS.
    assert stats.peak_rms >= stats.rms * 0.9


def test_stream_stats_propagates_invalid():
    result = classifier.stream_stats(sf.constant())
    assert isinstance(result, classifier.PassInvalid)


def test_gravity_and_tilt_removed_from_baseline():
    """Median high-pass: a huge DC offset must not inflate the RMS."""
    flat = classifier.stream_stats(sf.quiet(tilt=(0.0, 0.0)))
    tilted = classifier.stream_stats(sf.quiet(tilt=(3000.0, -2000.0)))
    assert tilted.rms == pytest.approx(flat.rms, rel=0.05)


# ---------------------------------------------------------------------
# numpy path vs pure-python fallback: identical verdicts on shared
# fixtures (numpy is a pinned dev dep precisely for this test).

PARITY_STREAMS = [
    ("quiet", sf.quiet()),
    ("wobbly", sf.wobbly()),
    ("drift", sf.with_drift(sf.quiet())),
    ("faint-contact", sf.with_contact(sf.quiet(), amplitude=45.0)),
    ("contact", sf.with_contact(sf.quiet(), amplitude=500.0)),
    ("near-threshold", sf.with_contact(sf.quiet(), amplitude=80.0)),
    ("short", sf.quiet(n=127)),
    ("nan", sf.with_nan(sf.quiet())),
    ("dropout", sf.with_dropout(sf.quiet())),
    ("constant", sf.constant()),
]


@pytest.mark.parametrize("knob", [0.0, 30.0, 50.0, 100.0])
@pytest.mark.parametrize(
    "stream", [s for _, s in PARITY_STREAMS], ids=[n for n, _ in PARITY_STREAMS]
)
def test_numpy_and_fallback_agree(stream, knob):
    with_np = classifier.classify_pass(stream, 10.0, knob, use_numpy=True)
    without = classifier.classify_pass(stream, 10.0, knob, use_numpy=False)
    assert type(with_np) is type(without)
    if isinstance(with_np, classifier.PassInvalid):
        assert with_np == without
    else:
        assert with_np.contact == without.contact
        assert with_np.peak_rms == pytest.approx(without.peak_rms, rel=1e-9)
        assert with_np.ratio == pytest.approx(without.ratio, rel=1e-9)
        assert with_np.confidence == pytest.approx(without.confidence, rel=1e-9)


def test_numpy_and_fallback_stream_stats_agree():
    for _, stream in PARITY_STREAMS:
        with_np = classifier.stream_stats(stream, use_numpy=True)
        without = classifier.stream_stats(stream, use_numpy=False)
        assert type(with_np) is type(without)
        if isinstance(with_np, classifier.StreamStats):
            assert with_np.rms == pytest.approx(without.rms, rel=1e-9)
            assert with_np.peak_rms == pytest.approx(without.peak_rms, rel=1e-9)


# ---------------------------------------------------------------------
# Window bookkeeping


def test_window_offsets_cover_tail():
    offsets = classifier._window_offsets(200, 64)
    assert offsets[0] == 0
    assert offsets[-1] == 200 - 64  # tail-aligned window present
    assert all(o + 64 <= 200 for o in offsets)


def test_window_offsets_exact_fit_no_duplicate_tail():
    offsets = classifier._window_offsets(128, 64)
    assert offsets == [0, 32, 64]
