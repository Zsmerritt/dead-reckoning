"""Tests for optional-dependency checks (numpy degradation path)."""

import importlib.util

import pytest

from plr import setup_checks


def test_numpy_available_reflects_find_spec(monkeypatch):
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: object())
    assert setup_checks.numpy_available() is True
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: None)
    assert setup_checks.numpy_available() is False


def test_require_numpy_noop_when_present(monkeypatch):
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: object())
    setup_checks.require_numpy()  # must not raise


def test_require_numpy_raises_clear_hint_when_absent(monkeypatch):
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: None)
    with pytest.raises(RuntimeError, match="klippy-env/bin/pip install numpy"):
        setup_checks.require_numpy()


def test_require_numpy_raises_caller_error_type(monkeypatch):
    # Callers pass gcode.error so the message reaches the console.
    class FakeGCodeError(Exception):
        pass

    monkeypatch.setattr(importlib.util, "find_spec", lambda name: None)
    with pytest.raises(FakeGCodeError, match="numpy is required"):
        setup_checks.require_numpy(error_type=FakeGCodeError)
