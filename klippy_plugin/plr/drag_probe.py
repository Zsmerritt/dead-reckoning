"""Drag-probe surface diagnostic (planned ``PLR_DRAG_PROBE``).

Will drag the probe across the solidified part surface along a chosen
path at low speed, recording trigger events, to verify that the part
surface can be located reliably before a recovery plan is executed.
Scaffold only: just the move-duration helper used to size command
timeouts.
"""


def travel_seconds(distance_mm, speed_mm_s):
    """Duration in seconds of a straight move; used for command timeouts.

    Raises ValueError on a non-positive speed or negative distance — both
    indicate a malformed g-code parameter, not a physics question.
    """
    if speed_mm_s <= 0:
        raise ValueError("SPEED=%s must be positive" % (speed_mm_s,))
    if distance_mm < 0:
        raise ValueError("distance %s must not be negative" % (distance_mm,))
    return distance_mm / speed_mm_s
