"""Tests for noise-test parameter normalization."""

import pytest

from plr import noise_test


@pytest.mark.parametrize(
    ("raw", "expected"),
    [("x", "x"), ("X", "x"), (" Z ", "z"), ("y", "y")],
)
def test_parse_axis_normalizes(raw, expected):
    assert noise_test.parse_axis(raw) == expected


@pytest.mark.parametrize("raw", ["a", "", "xy", "1"])
def test_parse_axis_rejects_invalid(raw):
    with pytest.raises(ValueError, match="invalid AXIS="):
        noise_test.parse_axis(raw)
