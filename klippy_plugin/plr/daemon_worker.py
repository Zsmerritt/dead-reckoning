"""Off-reactor plrd calls: a worker thread plus klippy's async callback.

=============================================================================
NO DAEMON SOCKET CALL MAY EVER RUN INSIDE A G-CODE HANDLER.  NOT ONE, AT
ANY TIMEOUT.  Every call goes through this module.
=============================================================================

WHY (the numbers, with sources).  Klipper's reactor is a single thread
running one `select`/`poll` loop that dispatches timers and fd callbacks
INLINE (klippy/reactor.py:314-327 ``_dispatch_loop`` /
:299-313 ``_check_fds``).  A g-code handler runs on that thread — an
operator command arrives on the API socket, is queued as a reactor
callback (klippy/webhooks.py:262), and is executed by
``GCodeDispatch.run_script`` under the g-code mutex
(klippy/webhooks.py:447-448, klippy/gcode.py:239-241, mutex created at
:111).  So a blocking ``sock.recv()`` in a handler stops the whole reactor
AND holds the g-code mutex.  Two independent things then break:

1. **Heaters get switched off, then the printer can fault.**
   ``Heater.set_pwm`` forces the PWM value to zero once
   ``read_time > self.verify_mainthread_time``
   (klippy/extras/heaters.py:72-74), and that deadline is refreshed ONLY
   from ``Heater.stats`` on the reactor
   (heaters.py:138-141, ``est_print_time + MAX_MAINTHREAD_TIME``, with
   ``MAX_MAINTHREAD_TIME = 5.0`` at heaters.py:17).  A reactor stalled
   past ~5 s therefore silently drops every heater to 0 % while its
   target stays set; ``verify_heater`` can then fault the printer into
   shutdown ("Heater %s not heating at expected rate",
   klippy/extras/verify_heater.py:86-91).  The MCU-side watchdog is
   tighter still: heaters arm ``setup_max_duration(MAX_HEAT_TIME)`` with
   ``MAX_HEAT_TIME = 3.0`` (heaters.py:14, :62), and an MCU pwm pin left
   at a non-default value with no further update inside that window
   shuts the MCU down (src/pwmcmds.c:45-53 arms ``pwm_end_event``).  The
   host escapes that only because heater PWM is refreshed from the
   SERIAL background thread (klippy/serialhdl.py:41-65 dispatches
   registered responses; ``MCU_adc`` registers its ADC callback at
   klippy/mcu.py:628-630) — the refresh keeps happening, but from
   heaters.py:72-74 it happens with the value clamped to zero.  So the
   honest chain for a stalled reactor is *heaters off, then possibly a
   verify_heater shutdown*, not an immediate MCU fault; mid-print there is
   a second hazard in the same class, since a reactor that stops flushing
   the motion queue and then resumes is the classic source of Klipper's
   "Timer too close".  Either way: recovery sets the bed temperature first
   and holds for the probe temperature before any motion, so a recovery is
   exactly the window in which heaters are active.

2. **plrd cannot drive the machine while its own client blocks.**
   plrd executes a plan by calling Moonraker's ``printer.gcode.script``
   (crates/plrd/src/moonraker.rs:167-172) and ``printer.objects.query``
   (:175), which Moonraker forwards to klippy's API socket
   ``gcode/script`` / ``objects/query`` endpoints
   (klippy/webhooks.py:439-448).  Both need the reactor, and the script
   needs the g-code mutex the blocking handler is holding.  So the
   handler waits for plrd, plrd waits for klippy, and the only exit is a
   client-side timeout — after which the reactor frees and plrd's queued
   commands run, i.e. **the machine moves after the operator was told
   the recovery failed**.  plrd's own patience is long: recover.rs:322
   sets the Moonraker call timeout to the executor's ``temp_timeout``
   (15 min by default, crates/plrd/src/executor.rs:152), so it will not
   rescue anybody.

WHAT THIS MODULE DOES INSTEAD.  ``AsyncDaemon.call`` starts a daemon
thread that performs the blocking conversation and hands the result back
with ``reactor.register_async_callback`` (klippy/reactor.py:199-205) —
klippy's own documented cross-thread wakeup: it queues the callback and
writes a byte into the reactor's self-pipe, whose read end is a
registered fd (:221-225), so the reactor wakes and runs the callback on
its own thread (:212-220).  klippy uses it from exactly this position:
``buttons.py:82``, ``replicape.py:80``, ``temperature_probe.py:151`` and
``Printer.invoke_async_shutdown`` (klippy/klippy.py:221-223).

RULES THIS MODULE ENFORCES, because breaking any one of them is worse
than the bug being fixed:

* **One call in flight per channel.**  A second ``call`` while one is
  outstanding is refused (returns False) instead of spawning threads: an
  operator holding down a button must not be able to fork the
  conversation, or exhaust threads against a hung daemon.  Cancelling
  does NOT free the slot early — the orphaned worker still holds a socket
  and an fd until its own deadline expires, so the slot stays taken until
  its result comes back to be dropped.  Freeing it on cancel would let a
  looping macro start one worker per iteration against a hung daemon.
* **No exception ever escapes a reactor callback.**  klippy treats one as
  fatal: an exception out of a reactor callback lands in
  ``Printer.run``'s handler, which logs "Unhandled exception during run"
  and calls ``invoke_shutdown`` (klippy/klippy.py:170-186).  Every
  delivery is wrapped, and a broken callback becomes a console error
  instead of a shutdown.
* **Nothing from a stale or post-teardown call is ever delivered.**  A
  worker cannot be killed while blocked in ``recv``, so it is orphaned
  instead: its result is dropped by generation number, and after
  ``klippy:disconnect`` no callback runs at all — a worker thread must
  never touch a printer that has gone away.

  ``klippy:shutdown`` is deliberately NOT terminal here.  A shutdown
  leaves the object graph intact and the reactor running (klippy stays up
  until ``FIRMWARE_RESTART``), and that is exactly the moment an operator
  most needs ``PLR_STATUS`` to still tell them what plrd thinks it is
  doing — so channels keep working through it.  What must not happen
  during a shutdown is *acting*: the recovery session refuses to start or
  continue one (plr/recovery.py, gated on ``printer.is_shutdown()``).
* **Threads are daemon threads.**  klippy restarts by re-running in the
  same process (``Printer.run``'s ``run_result``, klippy/klippy.py:186-198),
  and must be able to exit while a socket read is still blocked.
"""

import logging
import threading
import time

from . import daemon_link

logger = logging.getLogger(__name__)


class AsyncDaemon:
    """Serialized, off-reactor plrd calls for one logical channel.

    One instance per independent conversation (the recovery flow has its
    own; ``PLR_STATUS`` has its own) so a status query can never be
    refused by, or interfere with, a recovery in progress.
    """

    def __init__(self, printer, get_link, label):
        self.printer = printer
        self.reactor = printer.get_reactor()
        # Resolved per call, not captured: the plugin owns the one
        # DaemonLink, and binding it here would freeze whatever object
        # existed at config time.
        self.get_link = get_link
        self.label = label
        # Guards _busy/_generation across the reactor thread and workers.
        self._lock = threading.Lock()
        self._busy = False
        self._generation = 0
        self._closed = False
        # What is in flight, so a refusal can name it and say how long it
        # can still hold the slot (`refusal_text`).
        self._in_flight_cmd = None
        self._in_flight_until = None
        self._in_flight_generation = 0
        # klippy lifecycle: `klippy:disconnect` is the teardown event —
        # klippy/klippy.py:195 sends it on every exit and restart, as the
        # run loop unwinds — so after it no callback may run.  A SHUTDOWN
        # is not teardown and does not close the channel (module docs).
        printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

    # -- state -------------------------------------------------------

    def is_busy(self):
        with self._lock:
            return self._busy

    def is_closed(self):
        with self._lock:
            return self._closed

    def refusal_text(self, command):
        """Why a :meth:`call` would be refused right now, or ``None``.

        Exists so no caller has to GUESS: reporting "a query is already in
        flight" when the truth is "this channel is closed" promises the
        operator a report that will never arrive, at the moment they most
        need the truth.  Every call site renders this string.

        It also has to be honest about a CANCELLED call.  Its answer is
        deliberately discarded (:meth:`cancel`), so promising a report would
        be the same lie in a different place — and the slot stays taken until
        the orphan's own deadline, which on a dry run is minutes.  So the
        wait is named, in seconds, from the deadline the call was given.
        """
        with self._lock:
            closed, busy = self._closed, self._busy
            dropped = self._in_flight_generation != self._generation
            cmd, until = self._in_flight_cmd, self._in_flight_until
        if closed:
            return "%s: klippy is shutting down, so nothing was asked of plrd." % (
                command,
            )
        if not busy:
            return None
        remaining = max(0.0, (until or 0.0) - time.monotonic())
        if dropped:
            return (
                "%s: the previous plrd call (%s) was dismissed and its answer "
                "will be discarded, but its socket is still open — this "
                "channel frees in up to %d s. Nothing is coming before then; "
                "try again after that." % (command, cmd, int(remaining) + 1)
            )
        return (
            "%s: a plrd call (%s) has not answered yet; its report appears "
            "when it does, or within %d s at the latest."
            % (command, cmd, int(remaining) + 1)
        )

    # -- the one entry point -----------------------------------------

    def call(self, cmd, args, timeout, on_result, on_error):
        """Start one plrd call on a worker thread.

        Returns True when the worker started, False when this channel is
        already busy or klippy is stopping — the caller decides what to
        tell the operator, because "busy" is a different sentence in every
        flow.  ``on_result(response)`` / ``on_error(exception)`` are
        invoked on the REACTOR thread, at most one of them, at most once.
        """
        with self._lock:
            if self._closed or self._busy:
                return False
            self._busy = True
            generation = self._generation
            self._in_flight_cmd = cmd
            self._in_flight_generation = generation
            self._in_flight_until = time.monotonic() + float(timeout)
        thread = threading.Thread(
            target=self._work,
            args=(generation, cmd, args, timeout, on_result, on_error),
            # See the module docs: klippy must be able to exit while a
            # socket read is still blocked.
            name="plr-%s" % (self.label,),
        )
        thread.daemon = True
        try:
            thread.start()
        except RuntimeError:
            # Thread creation can fail outright (a memory-pressured Pi is
            # the realistic case).  Leaving `_busy` set would refuse every
            # later attempt for the rest of the session, and letting this
            # escape a g-code handler would be worse still: klippy turns a
            # non-CommandError out of a handler into invoke_shutdown
            # (klippy/gcode.py:231-235), so the recovery command itself
            # would shut the printer down.
            with self._lock:
                self._busy = False
                self._in_flight_cmd = None
                self._in_flight_until = None
            logger.exception("plr: cannot start the %s worker thread", self.label)
            return False
        return True

    def cancel(self):
        """Abandon whatever is in flight: its result will never be delivered.

        The worker thread cannot be interrupted (it is inside a blocking
        ``recv``), so it is orphaned: the generation bump means its answer
        is dropped when it arrives.

        The slot is NOT freed here.  The orphan still holds a socket and an
        fd until its own deadline expires, and freeing the slot would let a
        caller start one worker per cancel — 200 cancels, 200 live workers
        — against a daemon that never answers, which is precisely what the
        single-flight rule exists to prevent.  The slot frees itself when
        the orphan's result comes back to be discarded.
        """
        with self._lock:
            self._generation += 1

    # -- worker thread ------------------------------------------------

    def _work(self, generation, cmd, args, timeout, on_result, on_error):
        # Runs on the worker thread.  Touches nothing but the socket and
        # the reactor's async queue.
        try:
            response = self.get_link().call(cmd, args, timeout=timeout)
        except daemon_link.DaemonError as e:
            self._deliver(generation, on_error, e)
            return
        except Exception as e:
            # A non-DaemonError here is a plugin bug (or a hostile
            # response the parser did not classify).  It must still reach
            # the operator as a console error rather than dying silently
            # on a thread nobody is watching.
            logger.exception("plr: unexpected error in %s worker", self.label)
            self._deliver(
                generation,
                on_error,
                daemon_link.DaemonError(
                    "internal error talking to plrd (%s): %s" % (type(e).__name__, e)
                ),
            )
            return
        self._deliver(generation, on_result, response)

    def _deliver(self, generation, callback, payload):
        # Still the worker thread: hand the result to the reactor.
        try:
            self.reactor.register_async_callback(
                lambda eventtime: self._invoke(generation, callback, payload)
            )
        except Exception:
            # The reactor is gone (finalized).  Nothing to deliver to and
            # nothing that can be done from a background thread.
            logger.exception("plr: cannot deliver %s result to the reactor", self.label)

    # -- reactor thread ----------------------------------------------

    def _invoke(self, generation, callback, payload):
        # Runs on the reactor thread.  MUST NOT raise: klippy turns an
        # exception out of a reactor callback into a printer shutdown
        # (klippy/klippy.py:170-186).
        with self._lock:
            stale = self._closed or generation != self._generation
            # The worker is finished either way, so the slot frees here and
            # ONLY here — that is what bounds orphaned workers to one per
            # channel (see `cancel`).
            self._busy = False
        if stale:
            logger.info(
                "plr: dropping stale %s result (klippy stopped or flow cancelled)",
                self.label,
            )
            return
        try:
            callback(payload)
        except Exception:
            logger.exception("plr: error handling %s result", self.label)
            try:
                self.printer.lookup_object("gcode").respond_info(
                    "plr: internal error handling the plrd response (see "
                    "klippy.log). The recovery flow has been left alone; "
                    "plrd is unaffected."
                )
            except Exception:
                logger.exception("plr: cannot report %s callback failure", self.label)

    # -- lifecycle ----------------------------------------------------

    def _handle_disconnect(self):
        # Teardown: the reactor is unwinding, so nothing may be delivered
        # to it again.  Must not block or wait (a lock held for
        # microseconds by workers is fine).  `_busy` is left alone: no
        # further call can start anyway, and the orphan's own deadline
        # ends it.
        with self._lock:
            self._closed = True
            self._generation += 1
