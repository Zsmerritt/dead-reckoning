"""Klipper extras package for dead-reckoning power-loss recovery.

Klippy loads this package when a printer config contains a ``[plr]``
section: it imports ``klippy.extras.plr`` and calls ``load_config()``
below with the section's ConfigWrapper.  All user interaction happens
through the ``PLR_*`` g-code console commands this package registers;
the heavy lifting (journaling, reconstruction, recovery planning) lives
in the Rust daemon ``plrd``, reached through :mod:`plr.daemon_link`.

Source-compatibility note: everything under ``plr/`` must stay Python
3.7 syntax-compatible because it runs inside klippy (Klipper supports
3.7+).  The dev tooling (tests, ruff, coverage) has a separate floor of
Python 3.9 — see ``klippy_plugin/pyproject.toml``.
"""

from . import plugin


def load_config(config):
    """Klippy entry point for the ``[plr]`` config section."""
    return plugin.PLRPlugin(config)
