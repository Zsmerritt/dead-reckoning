"""Tiny fakes for the klippy objects the plugin touches.

Glue only: just enough of ConfigWrapper / Printer / GCodeDispatch /
GCodeCommand / PrinterConfig / toolhead-and-friends for wiring tests
(what got registered, what was looked up, what would be persisted,
which moves were requested).  Never fake physics, timing, or
durability here — behavior that depends on real klippy semantics
belongs in an integration harness against klippy itself, not in these
fakes.  Method names and error behavior mirror klippy
(klippy/configfile.py, klippy/klippy.py, klippy/gcode.py,
klippy/extras/probe.py) so tests read like plugin code; each class
cites the klippy shape it mirrors.
"""

import collections

_SENTINEL = object()


class FakeConfigError(Exception):
    """Stands in for klippy's config error (raised by config/printer)."""


class FakeCommandError(Exception):
    """Stands in for klippy's gcode.CommandError."""


# configparser's boolean vocabulary, which klippy's getboolean uses
# (klippy/configfile.py:73-75 delegates to fileconfig.getboolean).
_BOOLEAN_STATES = {
    "1": True,
    "yes": True,
    "true": True,
    "on": True,
    "0": False,
    "no": False,
    "false": False,
    "off": False,
}


class FakeConfig:
    """Stands in for klippy's ConfigWrapper over one config section.

    All wrappers created from one root share a single section registry
    (mirroring how every ConfigWrapper shares one fileconfig,
    klippy/configfile.py:119-121 ``getsection``).  ``sections`` maps
    section name -> dict of raw option strings.
    """

    error = FakeConfigError

    def __init__(self, printer, name="plr", options=None, sections=None):
        self._printer = printer
        self._name = name
        self._file_sections = dict(sections) if sections is not None else {}
        for key in list(self._file_sections):
            self._file_sections[key] = dict(self._file_sections[key])
        if options is not None:
            self._file_sections.setdefault(name, {}).update(options)
        self._options = self._file_sections.get(name, {})
        self._wrappers = {name: self}

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

    def _get_typed(
        self, option, default, parser, minval=None, maxval=None, above=None, below=None
    ):
        # Mirrors ConfigWrapper._get_wrapper bounds handling
        # (klippy/configfile.py:29-60): a missing option returns the
        # provided default as-is, unparsed and unbounded (configfile.py
        # lines 31-36); get() already raised if there was no default.
        raw = self.get(option, default)
        if option not in self._options:
            return raw
        try:
            value = parser(raw)
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
        if above is not None and value <= above:
            raise self.error(
                "Option '%s' in section '%s' must be above %s"
                % (option, self._name, above)
            )
        if below is not None and value >= below:
            raise self.error(
                "Option '%s' in section '%s' must be below %s"
                % (option, self._name, below)
            )
        return value

    def getfloat(
        self,
        option,
        default=_SENTINEL,
        minval=None,
        maxval=None,
        above=None,
        below=None,
    ):
        return self._get_typed(
            option,
            default,
            float,
            minval=minval,
            maxval=maxval,
            above=above,
            below=below,
        )

    def getint(self, option, default=_SENTINEL, minval=None, maxval=None):
        return self._get_typed(option, default, int, minval=minval, maxval=maxval)

    def _parse_boolean(self, raw):
        if isinstance(raw, bool):
            return raw
        key = str(raw).strip().lower()
        if key not in _BOOLEAN_STATES:
            raise ValueError(raw)
        return _BOOLEAN_STATES[key]

    def getboolean(self, option, default=_SENTINEL):
        return self._get_typed(option, default, self._parse_boolean)

    def getchoice(self, option, choices, default=_SENTINEL):
        # Mirrors klippy/configfile.py:76-86.
        if isinstance(choices, list):
            choices = {i: i for i in choices}
        c = self.get(option, default)
        if c not in choices:
            raise self.error(
                "Choice '%s' for option '%s' in section '%s'"
                " is not a valid choice" % (c, option, self._name)
            )
        return choices[c]

    def getsection(self, section):
        # ConfigWrapper.getsection never checks existence
        # (klippy/configfile.py:119-121); reads on a missing section
        # fail per-option instead.  Wrappers are cached for identity.
        if section not in self._wrappers:
            wrapper = FakeConfig.__new__(FakeConfig)
            wrapper._printer = self._printer
            wrapper._name = section
            wrapper._file_sections = self._file_sections
            wrapper._options = self._file_sections.get(section, {})
            wrapper._wrappers = self._wrappers
            self._wrappers[section] = wrapper
        return self._wrappers[section]

    def has_section(self, section):
        # klippy/configfile.py:122-123.
        return section in self._file_sections

    def get_prefix_sections(self, prefix):
        # klippy/configfile.py:124-126.
        return [
            self.getsection(s)
            for s in sorted(self._file_sections)
            if s.startswith(prefix)
        ]


class FakeReactor:
    """Stands in for klippy's reactor: deterministic monotonic clock."""

    def __init__(self, start=100.0):
        self.now = start

    def monotonic(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakePrinter:
    """Stands in for klippy's Printer: object registry + events."""

    command_error = FakeCommandError  # klippy/klippy.py Printer.command_error

    def __init__(self):
        self.objects = {}
        self.event_handlers = {}
        self.reactor = FakeReactor()

    def get_reactor(self):
        return self.reactor

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


class FakeGCodeCommand:
    """Stands in for klippy's GCodeCommand (klippy/gcode.py:24-96)."""

    error = FakeCommandError

    class sentinel:
        pass

    def __init__(self, gcode, command, commandline, params):
        self._command = command
        self._commandline = commandline
        self._params = dict(params)
        self.respond_info = gcode.respond_info

    def get_command(self):
        return self._command

    def get_command_parameters(self):
        return self._params

    def get(
        self,
        name,
        default=sentinel,
        parser=str,
        minval=None,
        maxval=None,
        above=None,
        below=None,
    ):
        # Mirrors klippy/gcode.py:65-90 including its error texts.
        value = self._params.get(name)
        if value is None:
            if default is self.sentinel:
                raise self.error(
                    "Error on '%s': missing %s" % (self._commandline, name)
                )
            return default
        try:
            value = parser(value)
        except (TypeError, ValueError):
            raise self.error(
                "Error on '%s': unable to parse %s" % (self._commandline, value)
            ) from None
        if minval is not None and value < minval:
            raise self.error(
                "Error on '%s': %s must have minimum of %s"
                % (self._commandline, name, minval)
            )
        if maxval is not None and value > maxval:
            raise self.error(
                "Error on '%s': %s must have maximum of %s"
                % (self._commandline, name, maxval)
            )
        if above is not None and value <= above:
            raise self.error(
                "Error on '%s': %s must be above %s" % (self._commandline, name, above)
            )
        if below is not None and value >= below:
            raise self.error(
                "Error on '%s': %s must be below %s" % (self._commandline, name, below)
            )
        return value

    def get_int(self, name, default=sentinel, minval=None, maxval=None):
        return self.get(name, default, parser=int, minval=minval, maxval=maxval)

    def get_float(
        self, name, default=sentinel, minval=None, maxval=None, above=None, below=None
    ):
        return self.get(
            name,
            default,
            parser=float,
            minval=minval,
            maxval=maxval,
            above=above,
            below=below,
        )


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

    def create_gcode_command(self, command, commandline, params):
        # klippy/gcode.py:244-246.
        return FakeGCodeCommand(self, command, commandline, params)

    def respond_info(self, msg, log=True):
        self.responses.append(msg)


class FakeConfigfile:
    """Stands in for klippy's PrinterConfig SAVE_CONFIG staging.

    Values are stringified exactly like PrinterConfig.set
    (klippy/configfile.py:314 ``svalue = str(value)``) so tests assert
    on what SAVE_CONFIG would actually write.
    """

    def __init__(self):
        self.pending = {}

    def set(self, section, option, value):
        self.pending.setdefault(section, {})[option] = str(value)


class FakeToolhead:
    """Stands in for the toolhead surface the plugin uses.

    get_status mirrors klippy/toolhead.py:503-513 merged with the
    kinematics dict (klippy/kinematics/cartesian.py:123 'homed_axes');
    manual_move records the request and applies the coordinate, which
    is bookkeeping, not motion physics.  wait_moves/dwell are recorded
    no-ops (klippy/toolhead.py:417-427).

    ``position_min`` gives the fake klippy's kinematic Z-limit BACKSTOP
    semantics: a commanded move ending below it is recorded, then
    rejected without applying, mirroring cartesian check_move ->
    _check_endstops raising move_error "Move out of range"
    (klippy/kinematics/cartesian.py:97-115).  Hostile drag-oracle tests
    set it to prove the plugin's own floor check fires FIRST — the
    backstop must never be the mechanism.
    """

    def __init__(
        self,
        homed_axes="xyz",
        position=(150.0, 150.0, 5.0, 0.0),
        position_min=None,
    ):
        self.homed_axes = homed_axes
        self.position = list(position)
        self.position_min = position_min
        self.moves = []
        self.wait_moves_calls = 0
        self.dwells = []
        self._last_move_time = 0.0

    def get_status(self, eventtime):
        return {"homed_axes": self.homed_axes, "position": tuple(self.position)}

    def get_position(self):
        return list(self.position)

    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        if (
            self.position_min is not None
            and len(coord) > 2
            and coord[2] is not None
            and coord[2] < self.position_min
        ):
            raise FakeCommandError("Move out of range")
        for i, value in enumerate(coord):
            if value is not None:
                self.position[i] = value

    def wait_moves(self):
        self.wait_moves_calls += 1

    def dwell(self, delay):
        self.dwells.append(delay)
        self._last_move_time += delay

    def get_last_move_time(self):
        return self._last_move_time


class FakeAccelClient:
    """Stands in for adxl345.AccelQueryHelper (klippy/extras/
    adxl345.py:34-87): finish_measurements waits for motion
    (adxl345.py:42-46), has_valid_samples reports whether any batch
    arrived (adxl345.py:55-71), get_samples yields (t, ax, ay, az)
    tuples (adxl345.py:72-87).

    Samples are canned by the test's script — the WHAT of the stream is
    glue; the classifier math over it is never faked.  ``samples=None``
    scripts a no-data capture (has_valid_samples False).
    """

    def __init__(self, samples, toolhead):
        self._samples = samples
        self._toolhead = toolhead
        self.finished = False

    def finish_measurements(self):
        self._toolhead.wait_moves()
        self.finished = True

    def has_valid_samples(self):
        return bool(self._samples)

    def get_samples(self):
        return list(self._samples or [])


class FakeAccelChip:
    """Stands in for an accel chip's internal-client surface
    (klippy/extras/adxl345.py:251-254 ``start_internal_client``).

    ``script`` is a list consumed one entry per client: each entry is a
    sample list, None (no-data capture), or a callable(toolhead) ->
    sample list evaluated when the client starts (so a hostile script
    can key the stream off the CURRENT toolhead Z).  ``default`` (also
    callable(toolhead) or a list) serves any capture after the script
    runs out — e.g. "always clean" for iteration-bound tests.  With
    neither left, starting a client raises, mirroring an exhausted
    test plan (a plugin bug, loudly).
    """

    def __init__(self, printer, script=None, default=None):
        self._printer = printer
        self.script = list(script) if script is not None else []
        self.default = default
        self.clients = []

    def start_internal_client(self):
        toolhead = self._printer.lookup_object("toolhead")
        if self.script:
            entry = self.script.pop(0)
        elif self.default is not None:
            entry = self.default
        else:
            raise FakeCommandError("FakeAccelChip: out of scripted captures")
        if callable(entry):
            entry = entry(toolhead)
        client = FakeAccelClient(entry, toolhead)
        self.clients.append(client)
        return client


class FakeIdleTimeout:
    """get_status mirrors klippy/extras/idle_timeout.py:34-40."""

    def __init__(self, state="Idle"):
        self.state = state

    def get_status(self, eventtime):
        return {"state": self.state, "printing_time": 0.0, "idle_timeout": 600.0}


class FakePrintStats:
    """get_status mirrors klippy/extras/print_stats.py:99-118 'state'."""

    def __init__(self, state="standby"):
        self.state = state

    def get_status(self, eventtime):
        return {"state": self.state}


# Mirrors the fields plugin code reads off klippy probe results
# (manual_probe.create_probe_result objects: bed_x/bed_y/bed_z, as
# consumed in klippy/extras/probe.py:152-164).
ProbeResult = collections.namedtuple("ProbeResult", ["bed_x", "bed_y", "bed_z"])


class FakeProbeSession:
    """Session shape from klippy/extras/probe.py:578-605
    (ProbeEndstopWrapper: start_probe_session / run_probe /
    pull_probed_results / end_probe_session).

    Trigger heights are canned by the test — no physics.  run_probe
    parks the toolhead at the trigger height, which is the one
    observable side effect the plugin relies on (the retract move adds
    sample_retract_dist to the CURRENT z, probe.py:149).
    """

    def __init__(self, probe, heights, toolhead):
        self._probe = probe
        self._heights = list(heights)
        self._toolhead = toolhead
        self._results = []
        self.ended = False
        self.run_gcmds = []

    def run_probe(self, gcmd):
        if not self._heights:
            raise FakeCommandError("FakeProbeSession: out of canned heights")
        self.run_gcmds.append(gcmd)
        z = self._heights.pop(0)
        pos = self._toolhead.get_position()
        self._toolhead.position[2] = z
        self._results.append(ProbeResult(pos[0], pos[1], z))

    def pull_probed_results(self):
        results = self._results
        self._results = []
        return results

    def end_probe_session(self):
        self.ended = True


class FakeProbe:
    """Stands in for the 'probe' printer object surface the plugin uses
    (klippy/extras/probe.py:608-628 PrinterProbe: get_probe_params /
    start_probe_session)."""

    def __init__(self, printer, heights, lift_speed=5.0, retract=2.0):
        self._printer = printer
        self.heights = list(heights)
        self.lift_speed = lift_speed
        self.retract = retract
        self.sessions = []

    def get_probe_params(self, gcmd=None):
        # Mirrors ProbeParameterHelper.get_probe_params reading gcode
        # overrides (klippy/extras/probe.py:296-315), for the params
        # the plugin consumes.
        probe_speed = 5.0
        lift_speed = self.lift_speed
        retract = self.retract
        samples = 1
        if gcmd is not None:
            probe_speed = gcmd.get_float("PROBE_SPEED", probe_speed, above=0.0)
            lift_speed = gcmd.get_float("LIFT_SPEED", lift_speed, above=0.0)
            samples = gcmd.get_int("SAMPLES", samples, minval=1)
            retract = gcmd.get_float("SAMPLE_RETRACT_DIST", retract, above=0.0)
        return {
            "probe_speed": probe_speed,
            "lift_speed": lift_speed,
            "samples": samples,
            "sample_retract_dist": retract,
        }

    def start_probe_session(self, gcmd):
        toolhead = self._printer.lookup_object("toolhead")
        session = FakeProbeSession(self, self.heights, toolhead)
        self.sessions.append(session)
        return session
