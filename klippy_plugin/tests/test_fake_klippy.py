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


def test_configfile_set_stages_values_stringified(fake_printer):
    # PrinterConfig.set stringifies (klippy/configfile.py:314); the fake
    # must record what SAVE_CONFIG would actually write.
    configfile = fake_printer.lookup_object("configfile")
    configfile.set("plr", "z_offset", 0.15)
    configfile.set("plr", "samples", 5)
    assert configfile.pending == {"plr": {"z_offset": "0.15", "samples": "5"}}


def test_config_getboolean_vocabulary(fake_printer):
    config = fake_klippy.FakeConfig(
        fake_printer, options={"a": "true", "b": "0", "c": "purple"}
    )
    assert config.getboolean("a") is True
    assert config.getboolean("b") is False
    assert config.getboolean("missing", False) is False
    with pytest.raises(fake_klippy.FakeConfigError, match="Unable to parse"):
        config.getboolean("c")


def test_config_getchoice_mirrors_klippy(fake_printer):
    config = fake_klippy.FakeConfig(fake_printer, options={"mode": "tap"})
    assert config.getchoice("mode", ["tap", "load_cell"]) == "tap"
    with pytest.raises(fake_klippy.FakeConfigError, match="not a valid choice"):
        config.getchoice("mode", ["a", "b"])
    with pytest.raises(fake_klippy.FakeConfigError, match="not a valid choice"):
        config.getchoice("missing", ["a", "b"], "zzz")


def test_config_sections_registry(fake_printer):
    config = fake_klippy.FakeConfig(
        fake_printer,
        name="plr",
        sections={"stepper_z": {"step_pin": "PF11"}, "stepper_z1": {}},
    )
    assert config.has_section("stepper_z")
    assert not config.has_section("stepper_x")
    assert config.getsection("stepper_z").get("step_pin") == "PF11"
    names = [s.get_name() for s in config.get_prefix_sections("stepper_z")]
    assert names == ["stepper_z", "stepper_z1"]


def test_config_getfloat_above_bound(fake_printer):
    config = fake_klippy.FakeConfig(fake_printer, options={"v": "0"})
    with pytest.raises(fake_klippy.FakeConfigError, match="must be above"):
        config.getfloat("v", above=0.0)


def test_gcode_command_parameter_parsing(fake_printer):
    gcode = fake_printer.lookup_object("gcode")
    gcmd = gcode.create_gcode_command("PLR_X", "PLR_X A=5", {"A": "5"})
    assert gcmd.get_int("A") == 5
    assert gcmd.get_float("A", minval=0.0) == 5.0
    assert gcmd.get("B", "dflt") == "dflt"
    with pytest.raises(fake_klippy.FakeCommandError, match="missing B"):
        gcmd.get("B")
    with pytest.raises(fake_klippy.FakeCommandError, match="minimum of 10"):
        gcmd.get_int("A", minval=10)
    bad = gcode.create_gcode_command("PLR_X", "PLR_X A=x", {"A": "x"})
    with pytest.raises(fake_klippy.FakeCommandError, match="unable to parse"):
        bad.get_float("A")


def test_toolhead_records_and_applies_moves():
    toolhead = fake_klippy.FakeToolhead(position=(10.0, 20.0, 5.0, 0.0))
    toolhead.manual_move([10.0, 20.0, 7.5], 4.0)
    assert toolhead.moves == [([10.0, 20.0, 7.5], 4.0)]
    assert toolhead.get_position()[2] == 7.5
    assert toolhead.get_status(0.0)["homed_axes"] == "xyz"


def test_probe_session_serves_canned_heights_only(fake_printer):
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    probe = fake_klippy.FakeProbe(fake_printer, [0.1])
    gcode = fake_printer.lookup_object("gcode")
    gcmd = gcode.create_gcode_command("", "", {})
    session = probe.start_probe_session(gcmd)
    session.run_probe(gcmd)
    (result,) = session.pull_probed_results()
    assert result.bed_z == 0.1
    # Honesty guard: the fake never invents data past its canned list.
    with pytest.raises(fake_klippy.FakeCommandError, match="canned heights"):
        session.run_probe(gcmd)
