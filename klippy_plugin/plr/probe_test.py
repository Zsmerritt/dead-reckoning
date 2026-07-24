"""Probe repeatability diagnostic (planned ``PLR_PROBE_TEST``).

Will drive a configurable number of probe cycles at the current XY
position, collect the trigger heights, and report their spread so users
can judge whether the probe is consistent enough for recovery homing.
Scaffold only: just the g-code parameter validation shared by the
planned command.
"""

MIN_SAMPLES = 3


def validate_sample_count(count, minimum=MIN_SAMPLES):
    """Validate the SAMPLES= g-code parameter; return it unchanged.

    Raises ValueError with a console-ready message when the count is too
    low to compute a meaningful spread.
    """
    if count < minimum:
        raise ValueError(
            "SAMPLES=%d is too low: at least %d probe samples are needed"
            % (count, minimum)
        )
    return count
