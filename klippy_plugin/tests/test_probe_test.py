"""Tests for probe-test parameter validation."""

import pytest

from plr import probe_test


def test_minimum_count_is_accepted():
    assert probe_test.validate_sample_count(probe_test.MIN_SAMPLES) == (
        probe_test.MIN_SAMPLES
    )


def test_higher_count_passes_through():
    assert probe_test.validate_sample_count(10) == 10


def test_too_few_samples_rejected_with_console_message():
    with pytest.raises(ValueError, match=r"SAMPLES=1 is too low"):
        probe_test.validate_sample_count(1)


def test_custom_minimum_is_honored():
    assert probe_test.validate_sample_count(2, minimum=2) == 2
    with pytest.raises(ValueError):
        probe_test.validate_sample_count(4, minimum=5)
