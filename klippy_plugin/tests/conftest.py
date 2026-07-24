"""Shared fixtures: a wired-together fake klippy environment.

``fake_printer`` carries the objects klippy would have created before
extras load (gcode, configfile); ``fake_config`` is the ConfigWrapper
klippy would hand to ``plr.load_config`` for a ``[plr]`` section.
"""

import fake_klippy
import pytest


@pytest.fixture
def fake_printer():
    printer = fake_klippy.FakePrinter()
    printer.add_object("gcode", fake_klippy.FakeGCode())
    printer.add_object("configfile", fake_klippy.FakeConfigfile())
    return printer


@pytest.fixture
def fake_config(fake_printer):
    return fake_klippy.FakeConfig(fake_printer, name="plr")
