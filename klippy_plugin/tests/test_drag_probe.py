"""Tests for drag-probe timeout sizing."""

import pytest

from plr import drag_probe


def test_travel_seconds_basic():
    assert drag_probe.travel_seconds(30.0, 5.0) == pytest.approx(6.0)


def test_zero_distance_is_instant():
    assert drag_probe.travel_seconds(0.0, 5.0) == 0.0


@pytest.mark.parametrize("speed", [0.0, -1.0])
def test_nonpositive_speed_rejected(speed):
    with pytest.raises(ValueError, match="must be positive"):
        drag_probe.travel_seconds(10.0, speed)


def test_negative_distance_rejected():
    with pytest.raises(ValueError, match="must not be negative"):
        drag_probe.travel_seconds(-1.0, 5.0)
