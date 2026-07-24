"""Probe/endstop electrical noise diagnostic (planned ``PLR_NOISE_TEST``).

Will sample the probe input at rest (and optionally while steppers are
energized on a chosen axis) over a fixed duration, counting spurious
trigger transitions, so wiring/EMI problems are caught before they can
ruin a recovery.  Scaffold only: just the g-code parameter
normalization shared by the planned command.
"""

VALID_AXES = ("x", "y", "z")

NOT_IMPLEMENTED = (
    "PLR_NOISE_TEST is not implemented yet — awaiting the drag-oracle "
    "milestone (this command will measure the accel-chip noise floor "
    "and stage noise_floor_* for SAVE_CONFIG)"
)


def cmd_PLR_NOISE_TEST(plugin, gcmd):
    """PLR_NOISE_TEST — drag-oracle noise-floor measurement (pending).

    Entry point already wired by plugin.PLRPlugin._register_commands;
    the drag-oracle milestone replaces this body with the real
    diagnostic without touching the registration table.
    """
    gcmd.respond_info(NOT_IMPLEMENTED)


def parse_axis(raw):
    """Normalize an AXIS= g-code parameter to lowercase 'x', 'y' or 'z'.

    Raises ValueError with a console-ready message for anything else.
    """
    axis = str(raw).strip().lower()
    if axis not in VALID_AXES:
        raise ValueError(
            "invalid AXIS=%s (expected one of: %s)" % (raw, ", ".join(VALID_AXES))
        )
    return axis
