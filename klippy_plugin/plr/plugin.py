"""Config-section handling and g-code command registration.

This module owns the object klippy stores for the ``[plr]`` section.  It
will read the section's options, look up the printer objects the plugin
depends on (``gcode``, ``configfile``, toolhead), and register the
``PLR_*`` g-code commands that expose setup checks, tunable read/write,
the probe/noise/drag diagnostics, and daemon status.  Scaffold only: the
plugin object currently just wires itself to the printer; no commands
are registered yet.
"""


class PLRPlugin:
    """The object klippy stores for the ``[plr]`` config section."""

    def __init__(self, config):
        self.config = config
        self.name = config.get_name()
        self.printer = config.get_printer()
