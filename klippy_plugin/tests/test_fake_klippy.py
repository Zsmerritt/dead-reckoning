"""Tests for the fake klippy harness itself.

The fakes are load-bearing test infrastructure: if their klippy-mirroring
error behavior drifts, plugin tests would pass against semantics real
klippy does not have.  Not counted toward plr coverage (the gate measures
plr/ only) — these exist to keep the harness honest.
"""

import fake_klippy
import pytest


def test_config_get_with_and_without_default(fake_printer):
    config = fake_klippy.FakeConfig(fake_printer, options={"mode": "auto"})
    assert config.get("mode") == "auto"
    assert config.get("missing", "fallback") == "fallback"
    with pytest.raises(fake_klippy.FakeConfigError, match="must be specified"):
        config.get("missing")


def test_config_getfloat_parses_and_bounds(fake_printer):
    config = fake_klippy.FakeConfig(fake_printer, options={"z": "2.5", "bad": "x"})
    assert config.getfloat("z") == 2.5
    assert config.getfloat("absent", default=None) is None
    with pytest.raises(fake_klippy.FakeConfigError, match="Unable to parse"):
        config.getfloat("bad")
    with pytest.raises(fake_klippy.FakeConfigError, match="minimum"):
        config.getfloat("z", minval=3.0)
    with pytest.raises(fake_klippy.FakeConfigError, match="maximum"):
        config.getfloat("z", maxval=2.0)


def test_config_getsection_shares_printer(fake_printer):
    config = fake_klippy.FakeConfig(fake_printer, name="plr")
    section = config.getsection("stepper_z")
    assert section.get_name() == "stepper_z"
    assert section.get_printer() is fake_printer
    assert config.getsection("stepper_z") is section


def test_printer_lookup_and_duplicate_registration(fake_printer):
    assert isinstance(fake_printer.lookup_object("gcode"), fake_klippy.FakeGCode)
    assert fake_printer.lookup_object("nope", None) is None
    with pytest.raises(fake_klippy.FakeConfigError, match="Unknown config object"):
        fake_printer.lookup_object("nope")
    with pytest.raises(fake_klippy.FakeConfigError, match="already created"):
        fake_printer.add_object("gcode", object())


def test_printer_event_handlers_accumulate(fake_printer):
    handler = object()
    fake_printer.register_event_handler("klippy:connect", handler)
    assert fake_printer.event_handlers["klippy:connect"] == [handler]


def test_gcode_command_registry_mirrors_klippy(fake_printer):
    gcode = fake_printer.lookup_object("gcode")

    def cmd(gcmd):
        gcode.respond_info("ran")

    gcode.register_command("PLR_STATUS", cmd, desc="Report status")
    assert gcode.commands["PLR_STATUS"] is cmd
    assert gcode.command_help["PLR_STATUS"] == "Report status"
    # Re-registering an existing command is an error, as in klippy.
    with pytest.raises(fake_klippy.FakeConfigError, match="already registered"):
        gcode.register_command("PLR_STATUS", cmd)
    # Passing func=None unregisters and returns the old handler.
    assert gcode.register_command("PLR_STATUS", None) is cmd
    assert "PLR_STATUS" not in gcode.commands


def test_gcode_respond_info_collects_console_output(fake_printer):
    gcode = fake_printer.lookup_object("gcode")
    gcode.respond_info("hello")
    gcode.respond_info("world", log=False)
    assert gcode.responses == ["hello", "world"]


def test_configfile_set_stages_values(fake_printer):
    configfile = fake_printer.lookup_object("configfile")
    configfile.set("plr", "z_offset", 0.15)
    configfile.set("plr", "samples", 5)
    assert configfile.pending == {"plr": {"z_offset": 0.15, "samples": 5}}
