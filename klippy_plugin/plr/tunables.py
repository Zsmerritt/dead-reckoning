"""Read/write access to PLR runtime tunables from the g-code console.

Will map tunable names to daemon parameters for ``PLR_GET``/``PLR_SET``
style commands: validating values against their documented ranges before
anything is sent to plrd, and persisting accepted values through
klippy's configfile workflow (``configfile.set()`` followed by the
standard SAVE_CONFIG restart prompt).  Scaffold only: just the shared
range-validation helper for now.
"""


def clamp(value, low, high):
    """Clamp ``value`` into the inclusive range [low, high].

    Raises ValueError if the range itself is inverted — that is a
    programming error in a tunable definition, not user input.
    """
    if low > high:
        raise ValueError("clamp: low %r is greater than high %r" % (low, high))
    return max(low, min(high, value))
