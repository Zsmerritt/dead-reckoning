"""A real single-threaded reactor, for the one claim a fake cannot make.

WHAT THIS IS FOR.  The defect this branch fixes is not a wrong string or a
missing field: it is that a g-code handler HELD KLIPPY'S ONE THREAD while
it waited for plrd.  No amount of fake-object testing can show that,
because tests/fake_klippy.py has no loop to hold.  So this module runs a
faithful reduction of klippy's reactor — one thread, one
``select``/dispatch loop, callbacks and timers invoked INLINE — and
measures how long plugin work keeps that thread.

WHAT IT REPRODUCES, with sources:

* the loop itself — klippy/reactor.py:314-327 ``SelectReactor._dispatch_loop``
  (``_check_timers``, then ``select``, then ``_check_fds``) and
  :299-313 ``_check_fds``, which calls each fd's read callback INLINE on
  the dispatch thread;
* the cross-thread wakeup — klippy/reactor.py:199-205
  ``register_async_callback`` queues the callback and writes a byte to a
  self-pipe whose read end is a registered fd (:221-225), and :212-220
  ``_got_pipe_signal`` drains the queue on the reactor thread;
* how an operator command arrives — klippy/webhooks.py:238-262: the API
  socket's fd callback parses the request and queues
  ``_process_request`` as a reactor callback, which for ``gcode/script``
  (:439, :447-448) calls ``GCodeDispatch.run_script``;
* the g-code mutex — klippy/gcode.py:111 creates it and :239-241
  ``run_script`` holds it for the WHOLE script, so a second script (for
  instance plrd's, arriving through Moonraker) cannot run while the first
  is inside a handler.

WHAT IT DELIBERATELY DOES NOT REPRODUCE, and why that is honest:

* **greenlets.** klippy's ``ReactorMutex`` and ``reactor.pause`` are
  greenlet switches (klippy/reactor.py:257-297); nothing in the recovery
  UI pauses, so the mutex here is a plain flag with a deferral queue —
  which models the property under test (a held mutex defers other scripts)
  without pretending to be klippy's scheduler.
* **the heater/MCU watchdogs.** The numbers are asserted against
  MEASURED stalls, with the klippy sources cited on the constants in
  tests/test_reactor_nonblocking.py.  Nothing here simulates a heater; the
  harness measures the input those watchdogs consume (reactor
  unavailability) and the test compares it against klippy's own thresholds.
* **a socketpair instead of ``os.pipe``** for the wakeup: ``select`` on
  Windows accepts sockets only, and the suite has to run on the developer
  host.  The mechanism (write a byte, wake the loop, drain a queue) is
  identical.
"""

import json
import select
import socket
import threading
import time


class MiniReactor:
    """One thread, one select loop, inline dispatch — like klippy's."""

    # klippy/reactor.py:8-9.
    NOW = 0.0
    NEVER = 9999999999999999.0

    def __init__(self):
        self._wake_r, self._wake_w = socket.socketpair()
        self._wake_r.setblocking(False)
        self._async_lock = threading.Lock()
        self._async_queue = []
        self._timers = []
        self._fds = {}
        # The longest single piece of work the reactor thread ran without
        # returning to the loop.  This is the quantity klippy's watchdogs
        # care about: while it elapses, no timer fires and no fd is
        # serviced (klippy/reactor.py:299-327).
        self.max_stall = 0.0
        self.stalls = []
        self.register_fd(self._wake_r, self._drain_async)

    # -- klippy's reactor API (the parts the plugin uses) -------------

    def monotonic(self):
        return time.monotonic()

    def register_async_callback(self, callback, waketime=NOW):
        # Callable from ANY thread (klippy/reactor.py:199-205).
        with self._async_lock:
            self._async_queue.append(callback)
        try:
            self._wake_w.send(b".")
        except OSError:
            pass

    def register_timer(self, callback, waketime=NEVER):
        timer = _Timer(callback, waketime)
        self._timers.append(timer)
        return timer

    def update_timer(self, timer, waketime):
        timer.waketime = waketime

    def unregister_timer(self, timer):
        if timer in self._timers:
            self._timers.remove(timer)

    # -- fds ----------------------------------------------------------

    def register_fd(self, sock, read_callback):
        self._fds[sock] = read_callback

    # -- the loop -----------------------------------------------------

    def run_until(self, predicate, timeout=10.0):
        """Dispatch until ``predicate()`` is true or ``timeout`` elapses.

        Returns True if the predicate became true.  Every callback runs on
        THIS thread, timed, exactly as klippy runs it.
        """
        deadline = time.monotonic() + timeout
        while True:
            if predicate():
                return True
            if time.monotonic() >= deadline:
                return False
            self.poll_once(0.01)

    def poll_once(self, timeout):
        readable, _w, _x = select.select(list(self._fds), [], [], timeout)
        for sock in readable:
            self._run(self._fds[sock])
        now = time.monotonic()
        for timer in list(self._timers):
            if timer.waketime <= now:
                self._run_timer(timer)

    def _drain_async(self):
        try:
            self._wake_r.recv(4096)
        except OSError:
            pass
        with self._async_lock:
            batch = self._async_queue
            self._async_queue = []
        for callback in batch:
            self._run(lambda cb=callback: cb(time.monotonic()))

    def _run(self, work):
        start = time.monotonic()
        try:
            work()
        finally:
            elapsed = time.monotonic() - start
            self.stalls.append(elapsed)
            self.max_stall = max(self.max_stall, elapsed)

    def _run_timer(self, timer):
        def work():
            timer.waketime = timer.callback(time.monotonic())

        self._run(work)

    def close(self):
        for sock in (self._wake_r, self._wake_w):
            try:
                sock.close()
            except OSError:
                pass


class _Timer:
    def __init__(self, callback, waketime):
        self.callback = callback
        self.waketime = waketime


class GCodeMutexHeld(Exception):
    """Raised by the harness when a script cannot run: the mutex is held."""


class MiniGCode:
    """Output + the g-code mutex, as klippy's GCodeDispatch has them.

    ``run_script`` mirrors klippy/gcode.py:239-241: it takes the mutex for
    the whole script.  ``respond_info`` / ``respond_raw`` mirror
    :247-254 — output only, NO mutex, which is why the recovery UI may
    call them from a reactor callback while plrd holds the mutex to move
    the machine.
    """

    def __init__(self):
        self.commands = {}
        self.command_help = {}
        self.responses = []
        self.raw_responses = []
        self.scripts_run = []
        self.mutex_held_by = None
        self.deferred_scripts = []
        # Anything that is NOT a CommandError escaping a handler: klippy
        # turns one into a printer shutdown (klippy/gcode.py:231-235).
        self.internal_errors = []

    # -- registration / dispatch --------------------------------------

    def register_command(self, name, func, desc=None):
        self.commands[name] = func
        self.command_help[name] = desc

    def create_gcode_command(self, command, commandline, params):
        return MiniGCodeCommand(self, command, commandline, params)

    def run_script(self, script, source="operator"):
        """Run a script under the mutex, or defer it if the mutex is held.

        klippy would BLOCK the caller's greenlet here (ReactorMutex);
        deferring makes the same fact observable in a harness with no
        greenlets: while ``mutex_held_by`` is set, no other script can run.
        """
        if self.mutex_held_by is not None:
            self.deferred_scripts.append((script, source))
            raise GCodeMutexHeld(
                "g-code mutex held by %s; %r deferred" % (self.mutex_held_by, script)
            )
        self.mutex_held_by = source
        try:
            for line in script.split("\n"):
                line = line.strip()
                if not line:
                    continue
                self.scripts_run.append((source, line))
                handler = self.commands.get(line.split()[0])
                if handler is None:
                    continue
                gcmd = self.create_gcode_command(
                    line.split()[0], line, _parse_params(line)
                )
                # klippy/gcode.py:225-235: a CommandError from a handler
                # becomes an error response, and anything else becomes a
                # printer shutdown.  Reproduced because it decides what an
                # operator sees when a command refuses — and because a
                # harness that let an exception escape into the loop would
                # be lying about what klippy does with one.
                try:
                    handler(gcmd)
                except MiniCommandError as e:
                    self.respond_raw("!! %s" % (e,))
                except Exception as e:  # pragma: no cover - a plugin bug
                    self.internal_errors.append(e)
                    self.respond_raw("!! Internal error on command:%r" % (line,))
        finally:
            self.mutex_held_by = None
        self._run_deferred()

    def _run_deferred(self):
        pending = self.deferred_scripts
        self.deferred_scripts = []
        for script, source in pending:
            self.run_script(script, source=source)

    # -- output (no mutex) --------------------------------------------

    def respond_info(self, msg, log=True):
        self.responses.append(msg)

    def respond_raw(self, msg):
        self.raw_responses.append(msg)


def _parse_params(line):
    params = {}
    for part in line.split()[1:]:
        if "=" in part:
            key, value = part.split("=", 1)
            params[key.upper()] = value
    return params


class MiniGCodeCommand:
    """Mirrors klippy/gcode.py:24-96 (the parts plugin handlers use)."""

    error = Exception

    class sentinel:
        pass

    def __init__(self, gcode, command, commandline, params):
        self._commandline = commandline
        self._params = dict(params)
        self.respond_info = gcode.respond_info
        self.respond_raw = gcode.respond_raw
        self.error = MiniCommandError

    def get(self, name, default=sentinel, parser=str):
        value = self._params.get(name)
        if value is None:
            if default is self.sentinel:
                raise self.error("missing %s" % (name,))
            return default
        return parser(value)

    def get_int(self, name, default=sentinel, minval=None, maxval=None):
        return self.get(name, default, parser=int)

    def get_float(self, name, default=sentinel, **kwargs):
        return self.get(name, default, parser=float)

    def get_command_parameters(self):
        return dict(self._params)


class MiniCommandError(Exception):
    """Stands in for klippy's CommandError."""


class MoonrakerBridge:
    """plrd's g-code path into klippy, from plrd's own thread.

    A real recovery drives the machine with Moonraker JSON-RPC
    ``printer.gcode.script`` calls (crates/plrd/src/moonraker.rs:167-172)
    which Moonraker forwards to klippy's API socket ``gcode/script``
    endpoint (klippy/webhooks.py:439, :447-448).  Klippy services that on
    the reactor (:195 registers the connection's fd) and runs it under the
    g-code mutex.

    So: ``send_script`` is called from plrd's thread, queues the script as
    a reactor callback (klippy/webhooks.py:262 does exactly that), and
    blocks until the reactor has run it — which is what makes the
    resolve-only-when-complete semantics plrd relies on, and what makes
    the deadlock a deadlock.
    """

    def __init__(self, reactor, gcode):
        self.reactor = reactor
        self.gcode = gcode
        self.completed = []
        self.refused = []

    def send_script(self, script, timeout=2.0):
        done = threading.Event()

        def run(eventtime):
            try:
                self.gcode.run_script(script, source="plrd")
                self.completed.append(script)
            except GCodeMutexHeld as e:
                self.refused.append(str(e))
            finally:
                done.set()

        self.reactor.register_async_callback(run)
        return done.wait(timeout)


def request_line(cmd, args=None):
    return json.dumps({"cmd": cmd, "args": args or {}}) + "\n"
