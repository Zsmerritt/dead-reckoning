"""Tests for daemon-call timeout validation."""

import pytest

from plr import daemon_link


def test_valid_timeout_returned_as_float():
    result = daemon_link.validate_timeout(2)
    assert result == 2.0
    assert isinstance(result, float)


def test_bounds_are_inclusive():
    assert daemon_link.validate_timeout(daemon_link.MIN_TIMEOUT) == (
        daemon_link.MIN_TIMEOUT
    )
    assert daemon_link.validate_timeout(daemon_link.MAX_TIMEOUT) == (
        daemon_link.MAX_TIMEOUT
    )


@pytest.mark.parametrize("seconds", [0.0, 0.01, 61.0, -5.0])
def test_out_of_range_rejected(seconds):
    with pytest.raises(ValueError, match="out of range"):
        daemon_link.validate_timeout(seconds)


def test_custom_bounds():
    assert daemon_link.validate_timeout(90, maximum=120.0) == 90.0
