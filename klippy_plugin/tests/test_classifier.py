"""Tests for verdict combination."""

import pytest

from plr import classifier


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
