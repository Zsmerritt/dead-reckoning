"""Tests for the klippy entry point and plugin object wiring."""

import plr
from plr.plugin import PLRPlugin


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
