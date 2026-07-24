"""Verdict classification for the diagnostic tests.

Will turn raw samples from the probe/noise/drag diagnostics into
pass/warn/fail verdicts with human-readable explanations for the
console, so every ``PLR_*`` test reports its outcome the same way.
Scaffold only: just the verdict vocabulary and the combinator that
reduces per-check verdicts to an overall one.
"""

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
