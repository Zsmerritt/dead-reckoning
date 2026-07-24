"""Tests for the tunable schema, validation, and PLR_SET."""

import fake_klippy
import pytest

from plr import tunables
from plr.tunables import clamp

# The FIXED [plr] schema keys the Rust daemon codes against.  If this
# test fails, someone renamed a key — that is a cross-repo breaking
# change, not a refactor.
FIXED_KEYS = [
    "probe_speed",
    "envelope_margin",
    "sag_allowance",
    "drag_speed",
    "drag_z_step",
    "drag_sensitivity",
    "exclusion_radius",
    "entry_feedrate",
]


def test_schema_keys_are_fixed():
    assert list(tunables.TUNABLES) == FIXED_KEYS


def test_load_from_config_defaults(fake_config):
    values = tunables.load_from_config(fake_config)
    assert values["probe_speed"] == 1.5
    assert values["entry_feedrate"] == 1800.0
    assert list(values) == FIXED_KEYS


def test_load_from_config_reads_overrides(fake_printer):
    config = fake_klippy.FakeConfig(
        fake_printer, name="plr", options={"probe_speed": "1.8", "drag_speed": "50"}
    )
    values = tunables.load_from_config(config)
    assert values["probe_speed"] == 1.8
    assert values["drag_speed"] == 50.0


@pytest.mark.parametrize(
    ("option", "raw"),
    [
        ("probe_speed", "0.5"),  # below minval 1.0
        ("probe_speed", "2.5"),  # above maxval 2.0
        ("drag_speed", "0"),  # exclusive lower bound
        ("drag_z_step", "0.3"),  # above maxval
        ("entry_feedrate", "0"),  # exclusive lower bound
        ("envelope_margin", "-0.1"),  # below minval 0
    ],
)
def test_load_from_config_range_violations_fail_parse(fake_printer, option, raw):
    config = fake_klippy.FakeConfig(fake_printer, name="plr", options={option: raw})
    with pytest.raises(fake_klippy.FakeConfigError, match=option):
        tunables.load_from_config(config)


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("probe_speed", 1.0),
        ("probe_speed", 2.0),
        ("drag_speed", 100.0),
        ("drag_z_step", 0.2),
        ("drag_sensitivity", 0.0),
        ("drag_sensitivity", 100.0),
        ("envelope_margin", 0.0),
        ("entry_feedrate", 1800.0),
    ],
)
def test_validate_accepts_boundary_values(name, value):
    assert tunables.validate(name, str(value)) == value


@pytest.mark.parametrize(
    ("name", "value", "fragment"),
    [
        ("probe_speed", "0.9", "[1, 2]"),
        ("probe_speed", "2.1", "[1, 2]"),
        ("drag_speed", "0", "(0, 100]"),
        ("drag_speed", "101", "(0, 100]"),
        ("drag_z_step", "0.21", "(0, 0.2]"),
        ("drag_sensitivity", "101", "[0, 100]"),
        ("exclusion_radius", "-1", ">= 0"),
        ("entry_feedrate", "1801", "(0, 1800]"),
    ],
)
def test_validate_rejects_out_of_range_with_range_text(name, value, fragment):
    with pytest.raises(ValueError) as excinfo:
        tunables.validate(name, value)
    assert fragment in str(excinfo.value)


def test_validate_unknown_name_lists_valid_params():
    with pytest.raises(ValueError) as excinfo:
        tunables.validate("warp_factor", "9")
    message = str(excinfo.value)
    for key in FIXED_KEYS:
        assert key in message


def test_validate_unparsable_value():
    with pytest.raises(ValueError, match="unable to parse VALUE=fast"):
        tunables.validate("probe_speed", "fast")


def test_range_text_forms():
    assert tunables.range_text(tunables.TUNABLES["probe_speed"]) == "[1, 2]"
    assert tunables.range_text(tunables.TUNABLES["drag_speed"]) == "(0, 100]"
    assert tunables.range_text(tunables.TUNABLES["envelope_margin"]) == ">= 0"
    unbounded = tunables.TunableSpec("x", 0.0, None, None, None, "", "")
    assert tunables.range_text(unbounded) == "(unbounded)"


# --- PLR_SET command ---------------------------------------------------


def test_plr_set_applies_live_and_stages_save(plugin, run_cmd, fake_printer):
    gcode = run_cmd("PLR_SET", PARAM="probe_speed", VALUE="1.8")
    assert plugin.tunables["probe_speed"] == 1.8
    configfile = fake_printer.lookup_object("configfile")
    assert configfile.pending == {"plr": {"probe_speed": "1.800000"}}
    assert plugin.is_pending_save("probe_speed")
    assert "SAVE_CONFIG" in gcode.responses[-1]


def test_plr_set_param_name_is_case_insensitive(plugin, run_cmd):
    run_cmd("PLR_SET", PARAM="PROBE_SPEED", VALUE="1.2")
    assert plugin.tunables["probe_speed"] == 1.2


def test_plr_set_without_args_lists_values_and_pending_markers(plugin, run_cmd):
    run_cmd("PLR_SET", PARAM="drag_speed", VALUE="42")
    gcode = run_cmd("PLR_SET")
    listing = gcode.responses[-1]
    assert "PLR tunables" in listing
    assert "drag_speed = 42" in listing
    assert "[awaiting SAVE_CONFIG]" in listing
    # Params never set have no pending marker.
    for line in listing.splitlines():
        if "probe_speed" in line:
            assert "[awaiting SAVE_CONFIG]" not in line
    # Every tunable and its range appear.
    for key in FIXED_KEYS:
        assert key in listing
    assert "[1, 2]" in listing and "(0, 100]" in listing


def test_plr_set_half_arguments_rejected(plugin, run_cmd):
    with pytest.raises(fake_klippy.FakeCommandError, match="both PARAM= and VALUE="):
        run_cmd("PLR_SET", PARAM="probe_speed")
    with pytest.raises(fake_klippy.FakeCommandError, match="both PARAM= and VALUE="):
        run_cmd("PLR_SET", VALUE="1.5")


def test_plr_set_unknown_param_errors_with_valid_list(plugin, run_cmd):
    with pytest.raises(fake_klippy.FakeCommandError, match="probe_speed"):
        run_cmd("PLR_SET", PARAM="warp_factor", VALUE="9")


def test_plr_set_out_of_range_errors_with_range(plugin, run_cmd, fake_printer):
    with pytest.raises(fake_klippy.FakeCommandError, match=r"\[1, 2\]"):
        run_cmd("PLR_SET", PARAM="probe_speed", VALUE="3")
    # Refused values change nothing, live or staged.
    assert plugin.tunables["probe_speed"] == 1.5
    assert fake_printer.lookup_object("configfile").pending == {}


# --- clamp helper ------------------------------------------------------


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
