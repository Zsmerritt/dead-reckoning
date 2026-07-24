"""Tests for the klippy entry point, [plr] parsing, and command wiring."""

import os
import time

import fake_klippy
import pytest

import plr
from plr.plugin import PLRPlugin

ALL_COMMANDS = [
    "PLR_SETUP",
    "PLR_SET",
    "PLR_PROBE_TEST",
    "PLR_TOUCH",
    "PLR_STATUS",
    "PLR_RECOVER",
    "PLR_NOISE_TEST",
    "PLR_DRAG_PROBE",
]


def test_load_config_returns_plugin_wired_to_printer(fake_config, fake_printer):
    plugin = plr.load_config(fake_config)
    assert isinstance(plugin, PLRPlugin)
    assert plugin.printer is fake_printer
    assert plugin.config is fake_config
    assert plugin.name == "plr"


def test_load_config_is_a_package_top_level_callable():
    # Klippy resolves a [plr] section by importing extras.plr and calling
    # its module-level load_config; the attribute must live on the
    # package itself, not a submodule.
    assert callable(plr.load_config)


def test_defaults_match_schema(plugin):
    assert plugin.probe_method == "tap"
    assert plugin.accel_chip_name is None
    assert plugin.control_socket == "/var/lib/plrd/plrd.sock"
    assert plugin.tunables["probe_speed"] == 1.5
    assert plugin.tunables["envelope_margin"] == 0.5
    assert plugin.tunables["sag_allowance"] == 0.2
    assert plugin.tunables["drag_speed"] == 20.0
    assert plugin.tunables["drag_z_step"] == 0.05
    assert plugin.tunables["drag_sensitivity"] == 30.0
    assert plugin.tunables["exclusion_radius"] == 5.0
    assert plugin.tunables["entry_feedrate"] == 1800.0
    assert plugin.self_locking_z is False
    assert plugin.probe_resolution is None


def test_default_wal_dir_when_not_overridden(fake_config):
    plugin = plr.load_config(fake_config)
    assert plugin.wal_dir == "/var/lib/plrd/wal"


def test_invalid_probe_method_is_a_config_error(plr_config):
    with pytest.raises(fake_klippy.FakeConfigError, match="not a valid choice"):
        plr.load_config(plr_config(options={"probe_method": "ouija"}))


def test_adxl_drag_requires_accel_chip(plr_config):
    with pytest.raises(fake_klippy.FakeConfigError, match="accel_chip"):
        plr.load_config(plr_config(options={"probe_method": "adxl_drag"}))


def test_adxl_drag_with_accel_chip_parses(plr_config):
    plugin = plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    )
    assert plugin.probe_method == "adxl_drag"
    assert plugin.accel_chip_name == "adxl345"


def test_out_of_range_tunable_fails_config_parse(plr_config):
    with pytest.raises(fake_klippy.FakeConfigError, match="probe_speed"):
        plr.load_config(plr_config(options={"probe_speed": "2.5"}))


def test_persisted_autosave_options_read_back(plr_config):
    # SAVE_CONFIG writes these into the [plr] autosave block; a restart
    # hands them back as ordinary config options.
    plugin = plr.load_config(
        plr_config(
            options={
                "self_locking_z": "True",
                "probe_resolution": "0.012",
                "noise_floor_rms": "42.5",
                "noise_floor_still_rms": "10.25",
                "noise_floor_peak": "88.0",
                "noise_floor_speed": "12.5",
            }
        )
    )
    assert plugin.self_locking_z is True
    assert plugin.probe_resolution == 0.012
    assert plugin.noise_floor_rms == 42.5
    assert plugin.noise_floor_still_rms == 10.25
    assert plugin.noise_floor_peak == 88.0
    assert plugin.noise_floor_speed == 12.5


def test_noise_floor_defaults_none(plugin):
    assert plugin.noise_floor_rms is None
    assert plugin.noise_floor_still_rms is None
    assert plugin.noise_floor_peak is None
    assert plugin.noise_floor_speed is None


def test_mangled_noise_floor_fails_config_parse(plr_config):
    # A hand-edited autosave value outside the schema is a config-time
    # error (klippy's standard message), not a mid-probe surprise.
    with pytest.raises(fake_klippy.FakeConfigError, match="noise_floor_rms"):
        plr.load_config(plr_config(options={"noise_floor_rms": "0"}))


@pytest.mark.parametrize("bad", ["0", "-5"])
def test_nonpositive_noise_floor_speed_rejected(plr_config, bad):
    with pytest.raises(fake_klippy.FakeConfigError, match="noise_floor_speed"):
        plr.load_config(plr_config(options={"noise_floor_speed": bad}))


@pytest.mark.parametrize("bad", ["inf", "nan"])
def test_non_finite_noise_floor_speed_rejected(plr_config, bad):
    # getfloat's above=0.0 lets "inf" through, and "nan" fails every
    # bound comparison silently — the explicit finiteness check must
    # refuse both at config time.
    with pytest.raises(fake_klippy.FakeConfigError, match="must be finite"):
        plr.load_config(plr_config(options={"noise_floor_speed": bad}))


def test_z_position_min_cached_from_good_sections(plugin):
    # good_sections() carries [stepper_z] position_min = -2.
    assert plugin.z_position_min == -2.0


def test_drag_state_initialized_null(plugin):
    assert plugin.last_drag_result is None
    assert plugin.last_drag_error is None


def test_touch_state_initialized_null(plugin):
    assert plugin.last_touch_result is None


def test_all_commands_registered_with_help(plugin, fake_printer):
    gcode = fake_printer.lookup_object("gcode")
    for name in ALL_COMMANDS:
        assert name in gcode.commands, name
        assert gcode.command_help[name], name


def test_get_status_shape_on_good_config(plugin):
    status = plugin.get_status(eventtime=100.0)
    assert status == {
        "probe_method": "tap",
        "configured": True,
        "attested": False,
        "probe_resolution": None,
        "daemon_alive": False,
        "noise_floor_rms": None,
        "last_drag_result": None,
        "last_drag_error": None,
        "last_touch_result": None,
    }


def test_get_status_not_configured_when_a_check_fails(fake_printer, plr_config):
    plugin = plr.load_config(plr_config(sections={"force_move": None}))
    assert plugin.get_status(100.0)["configured"] is False


def test_daemon_alive_tracks_heartbeat_and_caches(plugin, tmp_path):
    hb_dir = tmp_path / "wal"
    os.makedirs(str(hb_dir))
    hb = hb_dir / "heartbeat.bin"
    hb.write_bytes(b"\0" * 128)
    assert plugin.get_status(100.0)["daemon_alive"] is True
    # Staleness within the 1s cache window is not observed...
    old = time.time() - 60.0
    os.utime(str(hb), (old, old))
    assert plugin.get_status(100.5)["daemon_alive"] is True
    # ...but a later eventtime re-stats the file.
    assert plugin.get_status(101.5)["daemon_alive"] is False


def test_lookup_accel_chip_without_config_raises(plugin):
    with pytest.raises(fake_klippy.FakeCommandError, match="no accel_chip"):
        plugin.lookup_accel_chip()


def test_lookup_accel_chip_missing_object_raises(plr_config):
    plugin = plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="adxl345"):
        plugin.lookup_accel_chip()


def test_lookup_accel_chip_resolves_lazily(fake_printer, plr_config):
    plugin = plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345 bed"})
    )
    chip = object()
    # Registered AFTER [plr] loaded — the lookup must be lazy.
    fake_printer.add_object("adxl345 bed", chip)
    assert plugin.lookup_accel_chip() is chip


def test_pending_save_bookkeeping(plugin):
    assert plugin.is_pending_save("probe_speed") is False
    plugin.note_pending_save("probe_speed")
    assert plugin.is_pending_save("probe_speed") is True
