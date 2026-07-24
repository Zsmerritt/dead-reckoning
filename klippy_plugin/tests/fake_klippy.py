"""Tiny fakes for the klippy objects the plugin touches.

Glue only: just enough of ConfigWrapper / Printer / GCodeDispatch /
PrinterConfig for wiring tests (what got registered, what was looked
up, what would be persisted).  Never fake physics, timing, or
durability here — behavior that depends on real klippy semantics
belongs in an integration harness against klippy itself, not in these
fakes.  Method names and error behavior mirror klippy
(klippy/configfile.py, klippy/klippy.py, klippy/gcode.py) so tests read
like plugin code.
"""

_SENTINEL = object()


class FakeConfigError(Exception):
    """Stands in for klippy's config error (raised by config/printer)."""


class FakeCommandError(Exception):
    """Stands in for klippy's gcode.CommandError."""


class FakeConfig:
    """Stands in for klippy's ConfigWrapper over one config section."""

    error = FakeConfigError

    def __init__(self, printer, name="plr", options=None, sections=None):
        self._printer = printer
        self._name = name
        self._options = dict(options or {})
        self._sections = dict(sections or {})

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def get(self, option, default=_SENTINEL):
        if option in self._options:
            return self._options[option]
        if default is not _SENTINEL:
            return default
        raise self.error(
            "Option '%s' in section '%s' must be specified" % (option, self._name)
        )

    def getfloat(self, option, default=_SENTINEL, minval=None, maxval=None):
        raw = self.get(option, default)
        if raw is None:
            return None
        try:
            value = float(raw)
        except (TypeError, ValueError):
            raise self.error(
                "Unable to parse option '%s' in section '%s'" % (option, self._name)
            ) from None
        if minval is not None and value < minval:
            raise self.error(
                "Option '%s' in section '%s' must have minimum of %s"
                % (option, self._name, minval)
            )
        if maxval is not None and value > maxval:
            raise self.error(
                "Option '%s' in section '%s' must have maximum of %s"
                % (option, self._name, maxval)
            )
        return value

    def getsection(self, section):
        if section not in self._sections:
            self._sections[section] = FakeConfig(self._printer, name=section)
        return self._sections[section]


class FakePrinter:
    """Stands in for klippy's Printer: object registry + events."""

    def __init__(self):
        self.objects = {}
        self.event_handlers = {}

    def add_object(self, name, obj):
        if name in self.objects:
            raise FakeConfigError("Printer object '%s' already created" % (name,))
        self.objects[name] = obj

    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is not _SENTINEL:
            return default
        raise FakeConfigError("Unknown config object '%s'" % (name,))

    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)


class FakeGCode:
    """Stands in for klippy's GCodeDispatch (command registry + console)."""

    error = FakeCommandError

    def __init__(self):
        self.commands = {}
        self.command_help = {}
        self.responses = []

    def register_command(self, name, func, desc=None):
        if func is not None and name in self.commands:
            raise FakeConfigError("gcode command %s already registered" % (name,))
        if func is None:
            return self.commands.pop(name, None)
        self.commands[name] = func
        self.command_help[name] = desc
        return None

    def respond_info(self, msg, log=True):
        self.responses.append(msg)


class FakeConfigfile:
    """Stands in for klippy's PrinterConfig SAVE_CONFIG staging."""

    def __init__(self):
        self.pending = {}

    def set(self, section, option, value):
        self.pending.setdefault(section, {})[option] = value
