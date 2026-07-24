"""Tests for tunable range validation."""

import pytest

from plr.tunables import clamp


@pytest.mark.parametrize(
    ("value", "low", "high", "expected"),
    [
        (5.0, 0.0, 10.0, 5.0),  # inside: unchanged
        (-1.0, 0.0, 10.0, 0.0),  # below: clamped up
        (11.0, 0.0, 10.0, 10.0),  # above: clamped down
        (7.0, 7.0, 7.0, 7.0),  # degenerate range is valid
    ],
)
def test_clamp(value, low, high, expected):
    assert clamp(value, low, high) == expected


def test_clamp_rejects_inverted_range():
    with pytest.raises(ValueError, match="greater than"):
        clamp(5.0, 10.0, 0.0)
