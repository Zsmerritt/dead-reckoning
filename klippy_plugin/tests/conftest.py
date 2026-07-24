"""Shared fixtures: a wired-together fake klippy environment.

``fake_printer`` carries the objects klippy would have created before
extras load (gcode, configfile); ``fake_config`` is the ConfigWrapper
klippy would hand to ``plr.load_config`` for a ``[plr]`` section.
``plr_config`` / ``plugin`` build a realistic, fully-commissionable
printer config so per-test overrides only state what differs.
"""

import fake_klippy
import pytest

import plr


def good_sections():
    """A realistic single-MCU cartesian config that passes every static
    commissioning check (fresh copy per call so tests may mutate)."""
    return {
        "force_move": {"enable_force_move": "true"},
        "probe": {"pin": "^PA1", "z_offset": "0.5"},
        "stepper_z": {
            "step_pin": "PF11",
            "dir_pin": "!PH1",
            "enable_pin": "!PA0",
            "position_min": "-2",
        },
        "printer": {"kinematics": "cartesian"},
        "adxl345": {"cs_pin": "PB1"},
    }


@pytest.fixture
def fake_printer():
    printer = fake_klippy.FakePrinter()
    printer.add_object("gcode", fake_klippy.FakeGCode())
    printer.add_object("configfile", fake_klippy.FakeConfigfile())
    return printer


@pytest.fixture
def fake_config(fake_printer):
    return fake_klippy.FakeConfig(fake_printer, name="plr")


@pytest.fixture
def plr_config(fake_printer, tmp_path):
    """Factory for [plr] configs over the good_sections() printer.

    ``options`` overrides/extends the [plr] section; ``sections``
    replaces whole config sections (set a section to None to delete
    it).  wal_dir defaults into tmp_path so heartbeat checks never
    touch the real filesystem.
    """

    def build(options=None, sections=None):
        merged_sections = good_sections()
        for name, value in (sections or {}).items():
            if value is None:
                merged_sections.pop(name, None)
            else:
                merged_sections[name] = value
        merged_options = {"wal_dir": str(tmp_path / "wal")}
        merged_options.update(options or {})
        return fake_klippy.FakeConfig(
            fake_printer,
            name="plr",
            options=merged_options,
            sections=merged_sections,
        )

    return build


@pytest.fixture
def plugin(fake_printer, plr_config):
    """A PLRPlugin over the good config, with a homed toolhead wired in."""
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    return plr.load_config(plr_config())


@pytest.fixture
def run_cmd(fake_printer):
    """Dispatch a registered PLR command the way klippy would.

    Builds a GCodeCommand from keyword params (stringified, as klippy's
    parser yields string values) and invokes the registered handler.
    Returns the FakeGCode so callers can inspect .responses.
    """
    gcode = fake_printer.lookup_object("gcode")

    def run(name, **params):
        str_params = {key: str(value) for key, value in params.items()}
        commandline = " ".join(
            [name] + ["%s=%s" % (k, v) for k, v in str_params.items()]
        )
        gcmd = gcode.create_gcode_command(name, commandline, str_params)
        gcode.commands[name](gcmd)
        return gcode

    return run
