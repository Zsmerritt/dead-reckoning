"""Drag-probe surface diagnostic (planned ``PLR_DRAG_PROBE``).

Will drag the probe across the solidified part surface along a chosen
path at low speed, recording trigger events, to verify that the part
surface can be located reliably before a recovery plan is executed.
Scaffold only: just the move-duration helper used to size command
timeouts.
"""

NOT_IMPLEMENTED = (
    "PLR_DRAG_PROBE is not implemented yet — awaiting the drag-oracle "
    "milestone (this command will drag the nozzle across the part "
    "surface using drag_speed/drag_z_step/drag_sensitivity from [plr])"
)


def cmd_PLR_DRAG_PROBE(plugin, gcmd):
    """PLR_DRAG_PROBE — drag-oracle surface diagnostic (pending).

    Entry point already wired by plugin.PLRPlugin._register_commands;
    the drag-oracle milestone replaces this body with the real
    diagnostic without touching the registration table.
    """
    gcmd.respond_info(NOT_IMPLEMENTED)


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
