"""Client link from the plugin to the plrd recovery daemon.

Will implement the plugin side of the plugin<->plrd control channel:
status queries, tunable updates, and recovery arm/disarm, with strict
timeouts and clear console errors when the daemon is unreachable (plrd
is Linux-only and may simply not be running).  The transport rides on
the daemon's existing on-disk/socket surface and is specified with the
first real command.  Scaffold only: just the timeout validation every
call site will share.
"""

MIN_TIMEOUT = 0.05
MAX_TIMEOUT = 60.0


def validate_timeout(seconds, minimum=MIN_TIMEOUT, maximum=MAX_TIMEOUT):
    """Validate a daemon-call timeout in seconds; return it as float.

    Raises ValueError outside [minimum, maximum]: a too-small timeout
    fails spuriously, and an over-long one would stall the klippy
    reactor from a console command.
    """
    seconds = float(seconds)
    if seconds < minimum or seconds > maximum:
        raise ValueError(
            "timeout %.3fs out of range [%.3fs, %.3fs]" % (seconds, minimum, maximum)
        )
    return seconds
