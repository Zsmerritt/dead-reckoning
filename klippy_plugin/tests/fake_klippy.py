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
import math
import threading
import time

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

    OPTION-NAME CASE.  klippy parses printer.cfg with a
    ``configparser.RawConfigParser`` (klippy/configfile.py:170-176) and
    does not override ``optionxform``, so configparser LOWERCASES every
    option name as the file is read, and a lookup by any casing finds the
    same option.  Option names here are lowercased on both sides for the
    same reason — otherwise a test would "prove" a mixed-case key like
    ``UNSAFE_allow_purge_z_below_bed`` works only for the exact spelling
    the test happened to use.  SECTION names keep their case (configparser
    does not transform them; ``[gcode_macro CLEAN_NOZZLE]`` is
    case-sensitive), matching ``has_section`` / ``get_prefix_sections``.

    ACCESS TRACKING.  Every getter records the option it read into a
    registry shared by all wrappers from one root, mirroring
    ``ConfigWrapper._get_wrapper``'s ``access_tracking``
    (klippy/configfile.py:29-60) including its two subtleties: a PRESENT
    option records its PARSED value (line 46-47), and an ABSENT option
    records the default ONLY when that default is not ``None``
    (lines 31-36).  That map is what klippy turns into
    ``configfile.settings`` for status consumers
    (klippy/configfile.py:447-450) AND what it validates the config
    against — see :meth:`unused_options`.
    """

    error = FakeConfigError

    def __init__(self, printer, name="plr", options=None, sections=None):
        self._printer = printer
        self._name = name
        self._file_sections = dict(sections) if sections is not None else {}
        for key in list(self._file_sections):
            self._file_sections[key] = {
                option.lower(): value
                for option, value in self._file_sections[key].items()
            }
        if options is not None:
            self._file_sections.setdefault(name, {}).update(
                {option.lower(): value for option, value in options.items()}
            )
        self._options = self._file_sections.get(name, {})
        self._wrappers = {name: self}
        # (section.lower(), option.lower()) -> value, as klippy keys it.
        self._access = {}

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def _note_access(self, option, value):
        self._access[(self._name.lower(), option.lower())] = value

    def accessed_options(self, section=None):
        """The option names recorded as accessed in ``section``.

        The fake's equivalent of reading ``configfile.settings[section]``
        back (klippy/configfile.py:447-452).
        """
        name = (section if section is not None else self._name).lower()
        return {option for sect, option in self._access if sect == name}

    def accessed_settings(self, section=None):
        """``{option: recorded value}`` for ``section`` — the typed view
        plrd parses out of ``configfile.settings``."""
        name = (section if section is not None else self._name).lower()
        return {
            option: value
            for (sect, option), value in self._access.items()
            if sect == name
        }

    def unused_options(self, section=None):
        """Options present in ``section`` that no getter claimed.

        klippy's ``ConfigValidate.check_unused``
        (klippy/configfile.py:424-441) raises
        ``"Option '%s' is not valid in section '%s'"`` for each of these
        during startup (``Klippy._read_config``, klippy/klippy.py:127), so
        a non-empty result here is a printer that will not boot.

        Scoped to ONE section rather than the whole config on purpose: in
        a real klippy every other section is claimed by the klippy module
        that owns it, and this harness has no such modules — only the
        ``[plr]`` section is this plugin's responsibility.

        klippy additionally EXEMPTS options that came from the SAVE_CONFIG
        autosave block (klippy/configfile.py:426-427), which this harness
        does not model: every option here is treated as file-written,
        which is the strict case and the one that fails to boot.
        """
        name = (section if section is not None else self._name).lower()
        accessed = self.accessed_options(name)
        return sorted(
            option
            for option in self._file_sections.get(
                section if section is not None else self._name, {}
            )
            if option not in accessed
        )

    def get(self, option, default=_SENTINEL, note_valid=True):
        # ``note_valid=False`` suppresses access recording, exactly as
        # klippy/configfile.py:61-63 threads it into _get_wrapper: a read
        # that is not a claim on the option (the calibration fingerprint
        # enumerates sections it does not own).  klippy accepts the same
        # keyword on the typed getters; the plugin never uses it there, so
        # this harness does not offer it there — a future caller gets a
        # loud TypeError rather than a silently ignored flag.
        key = option.lower()
        if key in self._options:
            value = self._options[key]
            if note_valid:
                self._note_access(key, value)
            return value
        if default is not _SENTINEL:
            # klippy/configfile.py:33-35 — an absent option records its
            # default only when the default is not None.
            if note_valid and default is not None:
                self._note_access(key, default)
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
        if option.lower() not in self._options:
            return raw
        try:
            value = parser(raw)
        except (TypeError, ValueError):
            raise self.error(
                "Unable to parse option '%s' in section '%s'" % (option, self._name)
            ) from None
        # klippy records the PARSED value, not the raw string
        # (klippy/configfile.py:46-47), which is what makes
        # configfile.settings a typed map — and typed is exactly what
        # plrd's parser requires of it.  Recorded before the bound checks,
        # as klippy does (lines 46-59).
        self._note_access(option, value)
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
            # One access registry per root, as klippy passes one
            # access_tracking dict to every wrapper
            # (klippy/configfile.py:119-121).
            wrapper._access = self._access
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

    def get_prefix_options(self, prefix):
        # klippy/configfile.py:127-129: every option in THIS section whose
        # name starts with prefix (""=all), in file order.  Used by
        # calibration_meta to enumerate a section for fingerprinting.
        #
        # Records NO access, exactly as klippy does not: enumerating a
        # section never satisfies check_unused, which is precisely why the
        # plugin cannot lean on it to claim the [plr] options plrd
        # consumes (see plr/daemon_keys.py).
        return [o for o in self._options if o.startswith(prefix)]


class FakeReactor:
    """Stands in for klippy's reactor: clock, timers, async callbacks.

    ``auto_advance`` (seconds) is added to ``now`` after each
    ``monotonic()`` read, so a test can drive wall-clock elapsed time
    forward without any real sleeping — the drag-oracle time-budget
    bound reads the reactor once per pass, and a nonzero auto_advance
    makes those reads march forward deterministically.  Default 0.0
    keeps monotonic() a stable clock (every existing caller reads a
    constant value).

    DISPATCH GLUE, NOT A REACTOR.  ``register_async_callback`` /
    ``register_timer`` mirror the SHAPES klippy offers
    (klippy/reactor.py:195-220 for the callback pair and its self-pipe
    wakeup, :151-189 for timers) so plugin code can be driven from tests,
    but nothing here reproduces klippy's single-threaded dispatch: queued
    callbacks run only when a test calls :meth:`pump_async`, on the
    TEST's thread.  The claim that matters — that a g-code handler never
    holds the reactor while a daemon call blocks — cannot be proven
    against a fake that has no loop, so it is proven against a real
    select() loop instead (tests/reactor_harness.py).

    Klippy's own contract, which this honours: ``register_async_callback``
    is callable from ANY thread, the callback runs later on the reactor
    thread and takes ``eventtime``; a timer callback takes ``eventtime``
    and RETURNS its next waketime (``NEVER`` to stop).
    """

    # klippy/reactor.py:8-9 (_NOW / _NEVER).
    NOW = 0.0
    NEVER = 9999999999999999.0

    def __init__(self, start=100.0, auto_advance=0.0):
        self.now = start
        self.auto_advance = auto_advance
        # Async callbacks queued from other threads, plus the event a test
        # waits on (standing in for klippy's self-pipe wakeup).
        self._async_lock = threading.Lock()
        self._async_queue = []
        self._async_event = threading.Event()
        self.timers = []

    def monotonic(self):
        value = self.now
        self.now += self.auto_advance
        return value

    def advance(self, seconds):
        self.now += seconds

    # -- async callbacks (klippy/reactor.py:199-220) -------------------

    def register_async_callback(self, callback, waketime=NOW):
        with self._async_lock:
            self._async_queue.append(callback)
            self._async_event.set()

    def pending_async(self):
        with self._async_lock:
            return len(self._async_queue)

    def pump_async(self, expected=1, timeout=5.0):
        """Wait for ``expected`` queued callbacks, then run all of them.

        Returns the number invoked.  The wait is real (the callbacks come
        from real worker threads); the dispatch is not — see the class
        docstring.  A timeout returns what arrived, so a test asserting
        "nothing was delivered" states that with ``expected=0``.
        """
        deadline = time.time() + timeout
        while True:
            with self._async_lock:
                ready = len(self._async_queue)
            if ready >= expected or time.time() >= deadline:
                break
            self._async_event.wait(0.01)
            self._async_event.clear()
        with self._async_lock:
            batch = self._async_queue
            self._async_queue = []
            self._async_event.clear()
        for callback in batch:
            callback(self.monotonic())
        return len(batch)

    # -- timers (klippy/reactor.py:151-189) ---------------------------

    def register_timer(self, callback, waketime=NEVER):
        timer = FakeTimer(callback, waketime)
        self.timers.append(timer)
        return timer

    def update_timer(self, timer, waketime):
        timer.waketime = waketime

    def unregister_timer(self, timer):
        if timer in self.timers:
            self.timers.remove(timer)

    def run_due_timers(self):
        """Fire every timer whose waketime has passed; return how many.

        klippy's ``_check_timers`` re-arms a timer from its callback's
        return value (klippy/reactor.py:172-189); so does this.
        """
        fired = 0
        for timer in list(self.timers):
            if timer.waketime <= self.now:
                timer.waketime = timer.callback(self.now)
                fired += 1
        return fired


class FakeTimer:
    """Mirrors klippy's ReactorTimer (klippy/reactor.py:16-20)."""

    def __init__(self, callback, waketime):
        self.callback = callback
        self.waketime = waketime


class FakePrinter:
    """Stands in for klippy's Printer: object registry + events + state."""

    command_error = FakeCommandError  # klippy/klippy.py Printer.command_error

    def __init__(self, start_args=None):
        self.objects = {}
        self.event_handlers = {}
        self.reactor = FakeReactor()
        # klippy/klippy.py:55-56 Printer.is_shutdown reports the latched
        # shutdown state; invoke_shutdown (:204-220) sets it and then runs
        # the klippy:shutdown handlers.
        self.in_shutdown_state = False
        # klippy/klippy.py: Printer.get_start_args returns the dict main()
        # built, which carries 'software_version' (util.get_git_version()).
        # Defaults to a realistic git-describe string; tests set it to {}
        # or a version-less dict to exercise the unstampable path.
        self.start_args = (
            {"software_version": "v0.12.0-321-gabcdef012"}
            if start_args is None
            else start_args
        )

    def get_reactor(self):
        return self.reactor

    def get_start_args(self):
        # klippy/klippy.py Printer.get_start_args.
        return self.start_args

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

    def send_event(self, event, *params):
        # klippy/klippy.py:226-227.
        return [cb(*params) for cb in self.event_handlers.get(event, [])]

    def is_shutdown(self):
        # klippy/klippy.py:55-56.
        return self.in_shutdown_state

    def invoke_shutdown(self, msg="test shutdown"):
        """Latch the shutdown state, then run the handlers, as klippy does.

        Mirrors klippy/klippy.py:204-220 including the ORDER (state first,
        handlers second) and the fact that a handler raising does not stop
        the others.  klippy additionally runs them inside
        ``reactor.assert_no_pause()``; nothing here can pause, so the
        guarantee this harness offers is the order and the latch.
        """
        if self.in_shutdown_state:
            return
        self.in_shutdown_state = True
        for cb in list(self.event_handlers.get("klippy:shutdown", [])):
            cb()


class FakeGCodeCommand:
    """Stands in for klippy's GCodeCommand (klippy/gcode.py:24-96)."""

    error = FakeCommandError

    class sentinel:
        pass

    def __init__(self, gcode, command, commandline, params):
        self._command = command
        self._commandline = commandline
        self._params = dict(params)
        # klippy/gcode.py:31-33 wires both wrappers straight through to the
        # dispatcher, which is why plugin code may use either one.
        self.respond_info = gcode.respond_info
        self.respond_raw = gcode.respond_raw

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
        self.raw_responses = []

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

    def respond_raw(self, msg):
        # klippy/gcode.py:247-249 — the raw output path respond_info builds
        # on, and the one klippy uses for the '!!' error prefix
        # (klippy/gcode.py:255-263, klippy/extras/respond.py:8-12).
        self.raw_responses.append(msg)


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
        max_accel=3000.0,
        extruder=None,
    ):
        self.homed_axes = homed_axes
        self.position = list(position)
        self.position_min = position_min
        self.moves = []
        self.wait_moves_calls = 0
        self.dwells = []
        # Active extruder the toolhead reports (klippy/toolhead.py
        # get_extruder returns the selected PrinterExtruder); None on a
        # machine with no extruder wired for a test.
        self._extruder = extruder
        self._last_move_time = 0.0
        # Velocity-limit surface (klippy/toolhead.py:503-550): max_accel
        # is reported in get_status and mutated by set_max_velocities,
        # the SET_VELOCITY_LIMIT primitive the touch accel-clamp uses.
        self.max_accel = max_accel
        self.velocity_limits = []

    def get_status(self, eventtime):
        return {
            "homed_axes": self.homed_axes,
            "position": tuple(self.position),
            "max_accel": self.max_accel,
        }

    def set_max_velocities(
        self, max_velocity, max_accel, square_corner_velocity, min_cruise_ratio
    ):
        # Mirrors klippy/toolhead.py:538-550: only non-None fields are
        # applied; every call is recorded so tests can assert the
        # touch clamp/restore sequence (SET_VELOCITY_LIMIT-equivalent).
        self.velocity_limits.append(
            (max_velocity, max_accel, square_corner_velocity, min_cruise_ratio)
        )
        if max_accel is not None:
            self.max_accel = max_accel
        return (0.0, self.max_accel, 0.0, 0.0)

    def get_position(self):
        return list(self.position)

    def get_extruder(self):
        # klippy/toolhead.py get_extruder returns the active extruder.
        return self._extruder

    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        if (
            self.position_min is not None
            and len(coord) > 2
            and coord[2] is not None
            and coord[2] < self.position_min
        ):
            raise FakeCommandError("Move out of range")
        # Advance the move-time clock by the geometric travel time of this
        # move (distance / speed) so get_last_move_time() marches forward
        # per move, mirroring klippy's print-time clock (toolhead.py:410-427).
        # Bookkeeping, not physics: it lets the drag-oracle coverage check
        # compare accel-sample timestamps against a real motion window.
        dist_sq = 0.0
        for i, value in enumerate(coord):
            if value is not None and i < 3:
                dist_sq += (value - self.position[i]) ** 2
        if speed > 0.0:
            self._last_move_time += math.sqrt(dist_sq) / speed
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
        # The accel streams during the same motion the toolhead is
        # running, so its sample timestamps live on the toolhead's
        # move-time clock.  Capture that clock at client-start and rebase
        # the canned stream onto it (preserving every inter-sample gap):
        # each capture's samples then bracket that capture's own motion
        # window, so multi-pass coverage checks see a consistent clock
        # instead of every canned stream restarting at the same absolute
        # time.  Shift-invariant statistics (RMS, dt) are unaffected.
        self._t0 = toolhead.get_last_move_time()

    def finish_measurements(self):
        self._toolhead.wait_moves()
        self.finished = True

    def has_valid_samples(self):
        return bool(self._samples)

    def get_samples(self):
        raw = list(self._samples or [])
        if not raw:
            return []
        offset = self._t0 - raw[0][0]
        return [(t + offset, ax, ay, az) for (t, ax, ay, az) in raw]


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


class FakeExtruder:
    """Stands in for a PrinterExtruder's get_status surface.

    klippy/kinematics/extruder.py ``get_status`` delegates to the
    heater, and klippy/heaters.py ``Heater.get_status`` reports
    ``temperature``/``target``/``power``; the nozzle-temperature gate
    reads exactly those two fields.  ``target`` defaults to 0 (heater
    off).  Set ``report`` to override the whole dict (e.g. to script a
    status missing a field, for the defensive-read tests).
    """

    def __init__(self, temperature=25.0, target=0.0, report=None):
        self.temperature = temperature
        self.target = target
        self._report = report

    def get_status(self, eventtime):
        if self._report is not None:
            return dict(self._report)
        return {
            "temperature": self.temperature,
            "target": self.target,
            "power": 0.0,
        }


class FakeTempSensor:
    """Stands in for a klippy temperature sensor object.

    Every klippy sensor printer object (heater_bed, extruder,
    ``temperature_sensor <name>``) exposes a get_status reporting the
    latest reading under the ``temperature`` key
    (klippy/extras/temperature_sensor.py:34-40).  ``temperature=None``
    scripts a sensor that has not produced a reading yet.
    """

    def __init__(self, temperature=25.0):
        self.temperature = temperature

    def get_status(self, eventtime):
        return {"temperature": self.temperature}


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
        # Toolhead Z at the moment each probe descent begins, so the
        # touch retract-invariant test can assert no descent starts from
        # below the retract height.
        self.probe_start_zs = []

    def run_probe(self, gcmd):
        self.probe_start_zs.append(self._toolhead.get_position()[2])
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
