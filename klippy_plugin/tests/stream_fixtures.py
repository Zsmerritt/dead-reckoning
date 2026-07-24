"""Deterministic synthetic accelerometer streams for classifier tests.

These generate (t, ax, ay, az) tuples shaped like klippy's
AccelQueryHelper.get_samples output (adxl345.py:72-87), in mm/s^2 with
gravity on Z, at an adxl345-like sample rate.  Pure python + the random
module with fixed seeds: byte-for-byte reproducible without numpy.

The physics here is the TEST'S choice of input, not a simulation the
plugin trusts — the classifier math over these streams is always real.
"""

import math
import random

RATE_HZ = 3200.0
GRAVITY = 9810.0  # mm/s^2, on the Z axis


def times(n, rate=RATE_HZ, start=100.0):
    return [start + i / rate for i in range(n)]


def quiet(n=1024, noise=5.0, seed=1, tilt=(20.0, -35.0)):
    """Still/rigid baseline: gaussian noise around gravity + tilt DC."""
    rng = random.Random(seed)
    return [
        (
            t,
            tilt[0] + rng.gauss(0.0, noise),
            tilt[1] + rng.gauss(0.0, noise),
            GRAVITY + rng.gauss(0.0, noise),
        )
        for t in times(n)
    ]


def wobbly(n=1024, noise=5.0, sway=40.0, sway_hz=12.0, seed=2):
    """Noisy/wobbly baseline: low-frequency frame sway over noise."""
    rng = random.Random(seed)
    samples = []
    for i, t in enumerate(times(n)):
        phase = 2.0 * math.pi * sway_hz * i / RATE_HZ
        samples.append(
            (
                t,
                sway * math.sin(phase) + rng.gauss(0.0, noise),
                sway * math.cos(phase) + rng.gauss(0.0, noise),
                GRAVITY + 0.5 * sway * math.sin(phase) + rng.gauss(0.0, noise),
            )
        )
    return samples


def with_contact(base, amplitude, burst_hz=180.0, start_frac=0.7, seed=3):
    """Contact signature: band-limited burst added over the final third.

    A drag contact rings the toolhead at some structural frequency and
    adds broadband grinding; modelled as a sine at burst_hz plus
    amplitude-scaled noise from ``start_frac`` of the stream onward.
    """
    rng = random.Random(seed)
    out = []
    start = int(len(base) * start_frac)
    for i, (t, ax, ay, az) in enumerate(base):
        if i >= start:
            phase = 2.0 * math.pi * burst_hz * i / RATE_HZ
            ax = ax + amplitude * math.sin(phase) + rng.gauss(0.0, amplitude * 0.3)
            az = az + amplitude * math.cos(phase) + rng.gauss(0.0, amplitude * 0.3)
        out.append((t, ax, ay, az))
    return out


def with_drift(base, ramp=20.0):
    """Slow thermal/tilt drift: linear ramp over the whole stream.

    Median removal only takes out DC, so a ramp raises the high-passed
    magnitude a little — the classifier must not call that contact at
    sane sensitivities.  The default 20 mm/s^2 over one pass window is
    already generous for thermal drift.
    """
    n = len(base)
    return [
        (t, ax + ramp * i / n, ay - 0.5 * ramp * i / n, az + 0.25 * ramp * i / n)
        for i, (t, ax, ay, az) in enumerate(base)
    ]


def with_dropout(base, at_frac=0.5, gap_s=0.05):
    """Sample-rate collapse: a comms stall opens a >5x-median dt gap."""
    cut = int(len(base) * at_frac)
    return [
        (t + gap_s, ax, ay, az) if i >= cut else (t, ax, ay, az)
        for i, (t, ax, ay, az) in enumerate(base)
    ]


def constant(n=1024, value=(0.0, 0.0, GRAVITY)):
    """Frozen chip: every sample identical (includes the all-zero case)."""
    return [(t, value[0], value[1], value[2]) for t in times(n)]


def with_nan(base, index=200, field=2):
    """One non-finite value in the middle of an otherwise good stream."""
    out = [list(s) for s in base]
    out[index][field] = float("nan")
    return [tuple(s) for s in out]
