"""Verdict classification for the diagnostic tests + the drag oracle.

Two layers live here:

* the check-verdict vocabulary (:data:`VERDICTS`, :func:`worst_verdict`)
  shared by every ``PLR_*`` diagnostic report; and
* the drag-oracle pass classifier: pure functions that turn one lateral
  pass worth of accelerometer samples into a typed verdict
  (:class:`PassVerdict`) or a typed refusal (:class:`PassInvalid`).

Everything in this module is deliberately free of klippy imports and
side effects so the math can be tested exhaustively on synthetic
streams — the motion orchestration in :mod:`plr.drag_probe` and
:mod:`plr.noise_test` supplies real samples from klippy's accelerometer
internal-client API (klippy/extras/adxl345.py:72-87
``AccelQueryHelper.get_samples`` yields ``(time, ax, ay, az)`` tuples in
mm/s^2).

Safety posture: a degenerate sample stream is NEVER classified.  Too few
samples, non-finite values, a constant signal, or a collapsed sample
rate each return a :class:`PassInvalid` with a machine-readable reason;
the caller must treat an invalid pass as an abort, because "assume clean
and descend" is the unsafe direction.

numpy is optional (klippy installs may not have it): every computation
has a pure-python fallback that implements the identical algorithm, and
the test suite pins the two paths against each other on shared fixtures.
"""

import math

try:
    import numpy as _np
except ImportError:  # pragma: no cover - exercised via use_numpy=False
    _np = None

import collections

VERDICTS = ("pass", "warn", "fail")

_SEVERITY = {"pass": 0, "warn": 1, "fail": 2}


def worst_verdict(verdicts):
    """Reduce an iterable of verdicts to the most severe one.

    Severity order is fail > warn > pass.  Raises ValueError on an
    unknown verdict or an empty iterable — an overall verdict computed
    from nothing would hide a broken test.
    """
    worst = None
    for verdict in verdicts:
        if verdict not in _SEVERITY:
            raise ValueError("unknown verdict %r" % (verdict,))
        if worst is None or _SEVERITY[verdict] > _SEVERITY[worst]:
            worst = verdict
    if worst is None:
        raise ValueError("worst_verdict: no verdicts given")
    return worst


# ---------------------------------------------------------------------
# Drag-oracle pass classification


# Minimum samples for one pass to be classifiable.  Below this the
# windowed-RMS statistic is dominated by estimation noise (only a
# handful of RMS_WINDOW windows fit), so the pass is refused instead of
# guessed at.  At the adxl345 default 3200 Hz this is 40 ms of data —
# any real pass yields far more.
MIN_PASS_SAMPLES = 128

# Windowed-RMS window length in samples (~20 ms at 3200 Hz): long
# enough to average sensor noise, short enough that a contact burst in
# the final third of a pass dominates at least one full window.
RMS_WINDOW = 64

# A time gap between consecutive samples larger than this multiple of
# the median gap means the stream lost chunks (SPI overruns, comms
# stalls) and cannot be trusted to contain the contact signature.
DT_GAP_FACTOR = 5.0

# Sensitivity-knob anchor points: (knob value, threshold multiplier
# over the measured noise floor).  The knob is deliberately inverted
# relative to the multiplier: LOW knob values mean a HIGH multiplier,
# i.e. fewer false triggers — wobbly/noisy machines should run low
# numbers, per the design requirement.  Interpolation between anchors
# is linear in log(multiplier) (see :func:`multiplier`) so each knob
# step scales the threshold by a constant factor instead of squashing
# all the useful range into one end.
SENSITIVITY_ANCHORS = ((0.0, 8.0), (50.0, 4.0), (100.0, 1.5))

# Machine-readable PassInvalid reasons (the complete taxonomy).
INVALID_TOO_FEW = "too_few_samples"
INVALID_NON_FINITE = "non_finite"
INVALID_CONSTANT = "constant_signal"
INVALID_RATE_COLLAPSE = "sample_rate_collapse"
# Coverage gap: the capture's sample window does not span the pass
# motion (a batch that started late or ended early).  Detected by the
# drag staircase, not validate_pass_samples, because only the caller
# knows the motion window — but it lives in this taxonomy so every
# unclassifiable-pass abort shares one machine-readable vocabulary.
INVALID_COVERAGE = "coverage_gap"
INVALID_REASONS = (
    INVALID_TOO_FEW,
    INVALID_NON_FINITE,
    INVALID_CONSTANT,
    INVALID_RATE_COLLAPSE,
    INVALID_COVERAGE,
)

# One classified pass: ``contact`` is the decision, ``peak_rms`` the
# max windowed RMS of the preprocessed magnitude (mm/s^2), ``ratio``
# peak_rms/threshold, ``confidence`` in [0, 1] (see _confidence).
PassVerdict = collections.namedtuple(
    "PassVerdict", ["contact", "peak_rms", "ratio", "confidence"]
)

# A refused classification: ``reason`` is one of INVALID_REASONS,
# ``detail`` a console-ready explanation.
PassInvalid = collections.namedtuple("PassInvalid", ["reason", "detail"])

# Baseline statistics of one capture, used by PLR_NOISE_TEST: ``rms``
# is the whole-stream RMS of the preprocessed magnitude, ``peak_rms``
# the max windowed RMS (same statistic classify_pass thresholds on).
StreamStats = collections.namedtuple("StreamStats", ["rms", "peak_rms"])


def multiplier(sensitivity):
    """Map the 0-100 sensitivity knob to a threshold multiplier.

    Anchor table (log-interpolated between rows):

    ====  ==========  =============================================
    knob  multiplier  meaning
    ====  ==========  =============================================
       0         8.0  least sensitive — noisy/wobbly setups
      25        5.66
      50         4.0  default territory
      75        2.45
     100         1.5  most sensitive — rigid, quiet setups
    ====  ==========  =============================================

    Log interpolation (linear in ln(multiplier)) keeps every knob
    increment a constant *ratio* change of the threshold; linear
    interpolation would make the 0-50 half of the knob four times
    coarser than the 50-100 half.  Raises ValueError outside [0, 100]
    or on a non-finite knob — callers validate user input first, so
    this is a programming-error guard, not a user-facing path.
    """
    try:
        s = float(sensitivity)
    except (TypeError, ValueError):
        raise ValueError("sensitivity %r is not a number" % (sensitivity,)) from None
    if not math.isfinite(s) or s < 0.0 or s > 100.0:
        raise ValueError("sensitivity %r outside [0, 100]" % (sensitivity,))
    for (s0, m0), (s1, m1) in zip(SENSITIVITY_ANCHORS, SENSITIVITY_ANCHORS[1:]):
        if s <= s1:
            frac = (s - s0) / (s1 - s0)
            return m0 * (m1 / m0) ** frac
    # Unreachable: the last anchor is at 100 and s <= 100 was checked.
    raise AssertionError(  # pragma: no cover
        "sensitivity anchors do not cover %r" % (s,)
    )


def validate_pass_samples(samples):
    """Typed degenerate-input detection; returns PassInvalid or None.

    Checks run in a fixed order so tests can pin the taxonomy:

    1. too_few_samples — fewer than MIN_PASS_SAMPLES tuples;
    2. non_finite — any NaN/inf in any field;
    3. constant_signal — every axis is exactly constant (a dead or
       disconnected chip reads a frozen register; klippy's own driver
       flags wiring faults the same spirit, adxl345.py:276-284);
    4. sample_rate_collapse — a non-increasing timestamp, or a gap
       between consecutive samples above DT_GAP_FACTOR x the median
       gap (bulk sensor batches lost in transit).
    """
    n = len(samples)
    if n < MIN_PASS_SAMPLES:
        return PassInvalid(
            INVALID_TOO_FEW,
            "only %d samples (need >= %d)" % (n, MIN_PASS_SAMPLES),
        )
    for tup in samples:
        for value in tup:
            if not math.isfinite(value):
                return PassInvalid(
                    INVALID_NON_FINITE, "non-finite value %r in samples" % (value,)
                )
    constant = True
    first = samples[0]
    for tup in samples:
        if tup[1] != first[1] or tup[2] != first[2] or tup[3] != first[3]:
            constant = False
            break
    if constant:
        return PassInvalid(
            INVALID_CONSTANT,
            "all %d samples identical on every axis (dead/frozen chip?)" % (n,),
        )
    dts = [samples[i + 1][0] - samples[i][0] for i in range(n - 1)]
    for dt in dts:
        if dt <= 0.0:
            return PassInvalid(
                INVALID_RATE_COLLAPSE, "non-increasing sample timestamps"
            )
    ordered = sorted(dts)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        median_dt = ordered[mid]
    else:
        median_dt = (ordered[mid - 1] + ordered[mid]) / 2.0
    worst_dt = max(dts)
    if worst_dt > DT_GAP_FACTOR * median_dt:
        return PassInvalid(
            INVALID_RATE_COLLAPSE,
            "sample gap %.6fs is %.1fx the median gap %.6fs (dropouts)"
            % (worst_dt, worst_dt / median_dt, median_dt),
        )
    return None


def _resolve_use_numpy(use_numpy):
    if use_numpy is None:
        return _np is not None
    if use_numpy and _np is None:
        raise ValueError("use_numpy=True but numpy is not importable")
    return bool(use_numpy)


def _median(values):
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def _window_offsets(n, window):
    """Start offsets of the RMS windows over ``n`` samples.

    Half-overlapping windows (stride window/2) plus a tail-aligned
    window: the final samples of a pass are where contact energy lands
    when the toolhead clips a part edge late in the segment, so the
    tail guard guarantees they are covered by a full-length window
    instead of being truncated or dropped.
    """
    stride = max(1, window // 2)
    offsets = list(range(0, n - window + 1, stride))
    tail = n - window
    if offsets[-1] != tail:
        offsets.append(tail)
    return offsets


def _preprocess_py(samples):
    """High-passed magnitude, pure python: per-axis median removal.

    The median subtracts gravity and static tilt (a robust DC estimate
    that a short contact burst cannot drag, unlike the mean) so the
    magnitude reflects vibration energy only.
    """
    med_x = _median([s[1] for s in samples])
    med_y = _median([s[2] for s in samples])
    med_z = _median([s[3] for s in samples])
    return [
        math.sqrt((s[1] - med_x) ** 2 + (s[2] - med_y) ** 2 + (s[3] - med_z) ** 2)
        for s in samples
    ]


def _stats_py(samples, window):
    mag = _preprocess_py(samples)
    total_rms = math.sqrt(sum(m * m for m in mag) / len(mag))
    peak = 0.0
    for off in _window_offsets(len(mag), window):
        chunk = mag[off : off + window]
        rms = math.sqrt(sum(m * m for m in chunk) / window)
        if rms > peak:
            peak = rms
    return StreamStats(total_rms, peak)


def _stats_np(samples, window):
    data = _np.asarray([(s[1], s[2], s[3]) for s in samples], dtype=_np.float64)
    centered = data - _np.median(data, axis=0)
    mag = _np.sqrt((centered * centered).sum(axis=1))
    total_rms = float(_np.sqrt((mag * mag).mean()))
    peak = 0.0
    for off in _window_offsets(len(mag), window):
        rms = float(_np.sqrt((mag[off : off + window] ** 2).mean()))
        if rms > peak:
            peak = rms
    return StreamStats(total_rms, peak)


def stream_stats(samples, use_numpy=None):
    """Baseline statistics of one capture, or PassInvalid if degenerate.

    Applies exactly the classifier's preprocessing (median high-pass,
    magnitude, RMS_WINDOW windowed RMS) so a noise floor measured from
    these stats is directly comparable with classify_pass's peak_rms.
    """
    invalid = validate_pass_samples(samples)
    if invalid is not None:
        return invalid
    if _resolve_use_numpy(use_numpy):
        return _stats_np(samples, RMS_WINDOW)
    return _stats_py(samples, RMS_WINDOW)


def _confidence(ratio):
    """Confidence in the verdict, bounded to [0, 1].

    Based on s(r) = r^2 / (1 + r^2), a sigmoid in log-ratio space with
    s(1) = 0.5 exactly at the threshold: confidence = 2*|s(r) - 0.5|.
    It is 0 at the threshold (a coin-flip verdict), grows monotonically
    as the ratio moves away from 1 in either direction, and saturates
    at 1 (ratio -> 0 or ratio -> inf).  Examples: ratio 2 or 0.5 ->
    0.6; ratio 3 or 1/3 -> 0.8; ratio 10 -> 0.98.
    """
    s = ratio * ratio / (1.0 + ratio * ratio)
    value = 2.0 * abs(s - 0.5)
    return min(1.0, max(0.0, value))


def classify_pass(samples, noise_floor_rms, sensitivity, use_numpy=None):
    """Classify one pass; returns PassVerdict or PassInvalid.

    ``samples`` is the pass's complete capture as (t, ax, ay, az)
    tuples; ``noise_floor_rms`` the persisted moving-baseline RMS from
    PLR_NOISE_TEST; ``sensitivity`` the 0-100 knob.  Decision rule:
    contact iff max windowed RMS > noise_floor_rms *
    multiplier(sensitivity).

    Degenerate sample streams return PassInvalid (see
    validate_pass_samples).  A non-positive or non-finite noise floor
    or an out-of-range sensitivity raises ValueError: those are caller
    configuration bugs, not sample data, and must never be silently
    classified around.
    """
    try:
        floor = float(noise_floor_rms)
    except (TypeError, ValueError):
        raise ValueError(
            "noise_floor_rms %r is not a number" % (noise_floor_rms,)
        ) from None
    if not math.isfinite(floor) or floor <= 0.0:
        raise ValueError("noise_floor_rms %r must be finite and > 0" % (floor,))
    mult = multiplier(sensitivity)
    stats = stream_stats(samples, use_numpy=use_numpy)
    if isinstance(stats, PassInvalid):
        return stats
    threshold = floor * mult
    ratio = stats.peak_rms / threshold
    return PassVerdict(
        contact=ratio > 1.0,
        peak_rms=stats.peak_rms,
        ratio=ratio,
        confidence=_confidence(ratio),
    )
