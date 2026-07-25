"""The one in-flight recovery execution, and its confirm-point loop.

This module owns everything about a live ``recover_execute``: the
single-flight guard, the confirm-point conversation, the operator's
answer, the local deadline that must outlast plrd's own, and the klippy
lifecycle handling.  Both entry points — ``PLR_RECOVER EXECUTE=1
CONFIRM=YES`` and the wizard's ``PLR_WIZARD_EXECUTE`` — go through the one
instance the plugin holds, so a second attempt from either is refused by
the same guard.

NOTHING HERE BLOCKS THE REACTOR.  Every daemon call runs on a worker
thread (:mod:`plr.daemon_worker`, whose module docstring carries the
klippy evidence and the watchdog numbers).  A g-code handler starts the
conversation and returns; progress, confirmation prompts and the final
report all reach the operator from reactor callbacks.

THE PROTOCOL, from its producer
(crates/plrd/src/ctrlsock.rs:47-77 and :602-864):

* ``recover_execute {"confirm": true, "on_confirm": "ask"}`` — ``"ask"``
  is what makes a ``Tier::Confirmable`` diagnosis PAUSE instead of
  aborting (:612-627).  Omitting it (what this plugin used to do) means
  every confirm-point aborts the recovery, which made three shipped
  features — the "continue anyway" offer, ``confirm_z_before_resume`` and
  ``debug_confirm_each_step`` — unreachable from the only UI an operator
  has.
* the response is either the final report (``data.outcome`` ``completed``
  / ``aborted-or-refused``, :826-833) or a pause (``awaiting_confirmation``
  with ``resume_token``, :849-861);
* ``recover_confirm {"token": ..., "answer": "continue"|"abort"}``
  answers the ONE outstanding pause and returns *whatever happens next*
  (:736-786) — another pause, or the final report.  So the loop is:
  execute → (pause → answer)* → final.
* exactly one execution, and one outstanding pause, exists at a time
  server-side (:203-215, :631-645); a second ``recover_execute`` gets
  ``busy``, and a token that is not the outstanding one gets
  ``unknown-token`` — never a silent no-op.

STEP-BY-STEP MODE IS A CONFIG KEY, NOT AN ARGUMENT.  ``recover_execute``
REJECTS ``"step": true`` outright ("per-step mode is CLI-only",
ctrlsock.rs:603-605), so the ``step`` argument the plugin used to send
could only ever produce a refusal.  Pausing before every step over the
socket is ``[plr] debug_confirm_each_step`` (crates/plrd/src/executor.rs:
684-692, ``RecoveryPlan::debug_confirm_each_step``), which arrives as
``confirm_kind: "step-debug"`` pauses in this very loop.

===========================================================================
THE DEADLINE INTERLOCK — who owns which number
===========================================================================

plrd bounds an unanswered confirm-point itself and aborts cleanly when it
expires (crates/plrd/src/executor.rs:820-855 ``ask``; the abort applies
frame invalidation exactly as a decline at the same step would).  Its
deadline D is:

* ``[plr] confirm_timeout_s`` when the operator set it — the plan carries
  it (crates/plr-recovery/src/plan.rs:981-990) and the executor prefers
  it over its own default (executor.rs:649-655);
* otherwise plrd's own ``CONFIRM_TIMEOUT_DEFAULT_S`` = 600 s
  (crates/plr-recovery/src/build.rs:324-328, matching
  ``DEFAULT_CONFIRM_TIMEOUT`` at executor.rs:136-143) — and **no daemon
  response reports it**.

The plugin's local prompt deadline P must STRICTLY EXCEED D, because if P
were shorter the operator would be told the confirmation had expired while
plrd was still holding the machine and would still accept an answer, and
if they were EQUAL the race would decide which.  This project has already
shipped one deadlock built exactly that way — a plugin temperature ceiling
and the daemon's commanded probe temperature meeting at 150 °C
(plr/daemon_keys.py's "VALIDATION BOUNDARY" note) — so P is derived, never
restated:

* operator set ``confirm_timeout_s`` → D is that value, read from the SAME
  ``printer.cfg`` ``[plr]`` section plrd reads it from (the plugin already
  declares the key, plr/daemon_keys.py), and P = D + headroom;
* operator did not set it → D is unknown to the plugin, so P is derived
  from the largest value plrd can possibly be using: the top of the band
  plrd itself enforces on the key (``CONFIRM_TIMEOUT_MAX_S`` = 3600 s,
  build.rs:330-337; out-of-band values are REFUSED at planning time,
  build.rs:731-741).  That bounds the unreported default too, and
  tests/test_recovery_deadlines.py parses those constants out of the Rust
  source and fails if the relation ever stops holding.

P is a *fallback*: in every ordinary case plrd's own abort arrives first
and IS what the operator sees, including when the answer loses the race —
``recover_confirm`` then reports ``unknown-token`` with "the confirmation
timed out before the answer arrived; the recovery aborted"
(ctrlsock.rs:776-784), which this module renders as an abort, never as a
transport error.  When P does fire, all it does is stop the plugin from
claiming a recovery is still in flight; it never claims to have continued
or stopped anything itself.
"""

import logging
import threading

from . import confirm_ui, daemon_link, daemon_worker, prompts

logger = logging.getLogger(__name__)

# Re-exported so plr.plugin's command table and the wizard name the same
# strings the dialog's buttons fire.
CMD_CONTINUE = confirm_ui.CMD_CONTINUE
CMD_ABORT = confirm_ui.CMD_ABORT

ANSWER_CONTINUE = "continue"
ANSWER_ABORT = "abort"

# --- session states ---------------------------------------------------
STATE_IDLE = "idle"
STATE_RUNNING = "running"  # plrd is executing; nothing to answer
STATE_AWAITING = "awaiting_confirmation"  # a pause is outstanding

# --- the deadline interlock (see the module docstring) -----------------

# How much longer than plrd's own confirm deadline the plugin waits before
# it stops believing a pause is live.  It exists so the two deadlines can
# never coincide, and it is sized for what plrd still has to do AFTER its
# deadline expires: run the aborting step's cleanup commands through
# Moonraker (a round trip each), persist the frame-invalid marker and
# finish the transcript, plus this plugin's own reactor scheduling.  30 s
# is generous for all of that and negligible against any D.
CONFIRM_HEADROOM_S = 30.0

# The largest confirm deadline plrd can be using when it has not told us
# which one it is: the top of the band plrd enforces on
# ``[plr] confirm_timeout_s`` (crates/plr-recovery/src/build.rs:330-337
# CONFIRM_TIMEOUT_MAX_S).  NOT plrd's default — a plugin-side copy of
# another component's default is exactly the latent divergence this
# project has been bitten by.  An upper bound cannot diverge; it can only
# become loose.
DAEMON_CONFIRM_CEILING_S = 3600.0


def daemon_confirm_deadline(configured):
    """``(seconds, exact)`` — plrd's confirm deadline for this printer.

    ``configured`` is ``[plr] confirm_timeout_s`` as the plugin read it
    (``None`` when the operator did not set it —
    plr/daemon_keys.py:175-200).  ``exact`` is False when the value is
    absent or unusable, in which case the ceiling stands in: an
    unreadable setting must never shorten the plugin's wait.
    """
    if isinstance(configured, bool) or not isinstance(configured, (int, float)):
        return (DAEMON_CONFIRM_CEILING_S, False)
    value = float(configured)
    # NaN fails every comparison; inf is not a deadline.  Both fall back.
    if not (value > 0.0) or value == float("inf"):
        return (DAEMON_CONFIRM_CEILING_S, False)
    return (value, True)


def prompt_deadline(configured):
    """The plugin's local prompt deadline: strictly longer than plrd's."""
    deadline, _exact = daemon_confirm_deadline(configured)
    return deadline + CONFIRM_HEADROOM_S


def deadline_text(configured):
    """The honest sentence about plrd's deadline for the dialog.

    Says a number only when the plugin knows it exactly, because the one
    thing worse than not telling the operator how long they have is
    telling them wrongly.
    """
    deadline, exact = daemon_confirm_deadline(configured)
    if exact:
        return (
            "If nothing answers within about %d s (your [plr] "
            "confirm_timeout_s), plrd aborts the recovery cleanly on its "
            "own." % (int(deadline),)
        )
    return (
        "If nothing answers, plrd aborts the recovery cleanly on its own "
        "deadline (its default; set [plr] confirm_timeout_s to choose it)."
    )


class RecoverySession:
    """The single in-flight ``recover_execute`` conversation."""

    def __init__(self, plugin):
        self.plugin = plugin
        self.printer = plugin.printer
        self.reactor = self.printer.get_reactor()
        self._async = daemon_worker.AsyncDaemon(
            self.printer, lambda: plugin.daemon, "recovery"
        )
        self._state = STATE_IDLE
        self._token = None
        self._data = None  # the outstanding pause's data map, for re-show
        self._answering = None  # the answer currently in flight, if any
        self._source = None  # which command started this recovery
        self._on_finished = None
        self._pauses = 0
        self._timer = None
        self._closed = False
        self.printer.register_event_handler("klippy:shutdown", self._handle_stop)
        self.printer.register_event_handler("klippy:disconnect", self._handle_stop)

    # -- introspection ------------------------------------------------

    def state(self):
        return self._state

    def is_active(self):
        return self._state != STATE_IDLE

    def is_awaiting(self):
        return self._state == STATE_AWAITING and self._token is not None

    def status_lines(self):
        """Lines for ``PLR_STATUS`` — always safe, never a daemon call."""
        if self._state == STATE_IDLE:
            return ["recovery: idle"]
        if self._state == STATE_RUNNING:
            return [
                "recovery: RUNNING (started by %s) — plrd is driving the "
                "machine; watch the console for its report" % (self._source,)
            ]
        lines = [
            "recovery: AWAITING CONFIRMATION — question %d of this recovery "
            "(started by %s)" % (self._pauses, self._source)
        ]
        if isinstance(self._data, dict):
            lines.append(
                "  %s"
                % (
                    confirm_ui.where_line(
                        self._data.get("step"),
                        self._data.get("phase"),
                        self._data.get("confirm_kind"),
                    ),
                )
            )
        lines.append("  answer with %s or %s" % (CMD_CONTINUE, CMD_ABORT))
        return lines

    # -- output -------------------------------------------------------

    def _gcode(self):
        return self.printer.lookup_object("gcode")

    def _respond(self, msg):
        """Console output only — never g-code.

        ``respond_info`` walks ``output_callbacks`` (klippy/gcode.py:
        247-252) and takes NO g-code mutex, which is what makes it safe
        from a reactor callback while plrd holds that mutex to run a
        script.  Nothing in this module may call ``run_script`` for the
        same reason: it would queue behind plrd's own motion.
        """
        self._gcode().respond_info(msg)

    def _end_dialog(self):
        self._respond(prompts.action_prompt_end())

    # -- entry point --------------------------------------------------

    def start(self, gcmd, source, on_finished=None):
        """Begin a recovery execution.  Returns immediately.

        Raises ``gcmd.error`` when one is already in flight (from either
        entry point) or when klippy is not in a state to run one.
        """
        if self._closed or self.printer.is_shutdown():
            raise gcmd.error(
                "%s: klippy is shut down — clear the shutdown "
                "(FIRMWARE_RESTART) before attempting a recovery." % (source,)
            )
        if self.is_active():
            raise gcmd.error(self._busy_message(source))
        # A worker still holding the channel with no session state means a
        # previous conversation was abandoned (transport error) and its
        # thread has not unblocked yet.  Refusing here is the safe
        # direction: plrd may still be executing that recovery.
        if self._async.is_busy():
            raise gcmd.error(
                "%s: the previous plrd conversation has not finished. plrd "
                "may still be executing that recovery — check PLR_STATUS and "
                "the plrd log (journalctl -u plrd) before retrying." % (source,)
            )
        self._source = source
        self._on_finished = on_finished
        self._pauses = 0
        self._token = None
        self._data = None
        self._state = STATE_RUNNING
        started = self._async.call(
            "recover_execute",
            # `on_confirm: "ask"` is the whole point of this branch; `step`
            # is deliberately absent (the daemon refuses it — module docs).
            {"confirm": True, "on_confirm": "ask"},
            daemon_link.EXECUTE_TIMEOUT,
            self._on_response,
            self._on_error,
        )
        if not started:
            self._state = STATE_IDLE
            raise gcmd.error(
                "%s: another plrd conversation is already in flight; nothing "
                "was started." % (source,)
            )
        gcmd.respond_info(
            "%s: plrd is executing the recovery — THE PRINTER WILL MOVE.\n"
            "This command returns immediately; the recovery runs in plrd and "
            "reports here as it goes. Do not send other commands to the "
            "printer until it reports.\n"
            "If plrd stops to ask you something, a prompt appears and the "
            "console names the exact command to answer it (%s / %s)."
            % (source, CMD_CONTINUE, CMD_ABORT)
        )

    def _busy_message(self, source):
        if self.is_awaiting():
            return (
                "%s: a recovery is already in flight and is WAITING FOR YOUR "
                "ANSWER — run %s to continue it or %s to stop it. Run "
                "PLR_STATUS to see the question again."
                % (source, CMD_CONTINUE, CMD_ABORT)
            )
        return (
            "%s: a recovery is already in flight (started by %s). plrd runs "
            "exactly one at a time; wait for its report. Use M112 if the "
            "machine must stop NOW." % (source, self._source)
        )

    # -- answering ----------------------------------------------------

    def answer(self, gcmd, answer, source):
        """Answer the outstanding confirm-point (``continue``/``abort``)."""
        if answer not in (ANSWER_CONTINUE, ANSWER_ABORT):
            # Not reachable from the two commands; a guard against a future
            # caller inventing a third answer plrd would reject.
            raise gcmd.error("%s: unknown answer %r" % (source, answer))
        if not self.is_awaiting():
            raise gcmd.error(self._nothing_to_answer(source))
        if answer == ANSWER_CONTINUE and self.printer.is_shutdown():
            # Continuing means telling plrd to drive a printer that cannot
            # move.  Refuse; aborting is still allowed (and is what the
            # shutdown handler already did).
            raise gcmd.error(
                "%s: klippy is shut down — the recovery cannot continue. "
                "Run %s to stop it." % (source, CMD_ABORT)
            )
        token = self._token
        # Hand the token to plrd and forget it locally: the pause is no
        # longer outstanding as far as this plugin is concerned, so a
        # double-click cannot answer twice.
        self._token = None
        self._state = STATE_RUNNING
        self._answering = answer
        self._disarm_timer()
        started = self._async.call(
            "recover_confirm",
            {"token": token, "answer": answer},
            daemon_link.EXECUTE_TIMEOUT,
            self._on_response,
            self._on_error,
        )
        if not started:
            # The channel is busy: something else is already talking to
            # plrd on it.  Put the pause back rather than losing it.
            self._token = token
            self._state = STATE_AWAITING
            self._answering = None
            self._arm_timer()
            raise gcmd.error(
                "%s: still waiting for plrd's previous reply; try again in a "
                "moment (the question is still open)." % (source,)
            )
        self._end_dialog()
        gcmd.respond_info(
            "%s: answered '%s'. plrd is continuing; its next report appears "
            "here." % (source, answer)
        )

    def _nothing_to_answer(self, source):
        if self._state == STATE_RUNNING:
            return (
                "%s: plrd is not asking anything right now (the recovery is "
                "running). A running recovery cannot be interrupted from "
                "here: plrd aborts by itself on any failed verification, and "
                "M112 stops the machine immediately." % (source,)
            )
        return (
            "%s: no recovery confirmation is outstanding. Start a recovery "
            "with PLR_WIZARD_START (or PLR_RECOVER EXECUTE=1 CONFIRM=YES)." % (source,)
        )

    def cancel(self, gcmd, source):
        """Operator cancellation from the wizard's Dismiss/Cancel path.

        Answering an outstanding pause with ``abort`` is a real
        cancellation; a running recovery is NOT cancellable from here, and
        saying so is better than resetting local state while plrd still
        drives the machine.  Returns True when something was cancelled.
        """
        if self.is_awaiting():
            self.answer(gcmd, ANSWER_ABORT, source)
            return True
        if self._state == STATE_RUNNING:
            raise gcmd.error(
                "%s: plrd is executing the recovery and it cannot be "
                "cancelled from here. It stops by itself on any failed "
                "verification; use M112 if the machine must stop NOW." % (source,)
            )
        return False

    # -- the confirm loop ---------------------------------------------

    def _on_response(self, response):
        """One plrd response, on the reactor thread.  Never raises."""
        if self._closed:
            return
        data = response.get("data")
        if not isinstance(data, dict):
            # A response with no data map cannot be classified.  Report it
            # verbatim and stop believing anything about the recovery.
            self._finish(
                response,
                "plrd sent a response with no data; treat the report above as "
                "the whole story and check 'journalctl -u plrd'.",
            )
            return
        outcome = data.get("outcome")
        if outcome == "awaiting_confirmation":
            self._pause(response, data)
            return
        self._finish(response, self._final_note(response, outcome))

    def _final_note(self, response, outcome):
        if response.get("ok") is True:
            return "PLR recovery complete — plrd has resumed the print."
        if outcome == "busy":
            return (
                "plrd already has a recovery in flight, so this attempt "
                "changed nothing. Wait for that one to report."
            )
        if outcome == "unknown-token":
            # ctrlsock.rs:755-784: no execution awaiting, or the pause
            # timed out between the question and the answer.  Either way
            # plrd is not waiting on us any more.
            return (
                "plrd is no longer waiting for that answer — its own "
                "confirmation deadline expired first and the recovery "
                "aborted (that abort is the safe direction, and plrd "
                "invalidates the Z frame exactly as a decline would). "
                "Re-run PLR_WIZARD_START for a fresh dry run."
            )
        if outcome == "refused":
            return (
                "plrd refused to execute (see the report above); nothing was "
                "sent to the printer."
            )
        return (
            "PLR recovery did not complete (see the report above for the "
            "typed failure and how to remediate). Re-run PLR_WIZARD_START "
            "for a fresh dry run before retrying."
        )

    def _pause(self, response, data):
        token = data.get("resume_token")
        if not isinstance(token, str) or not token:
            # plrd is paused and there is no way to answer it.  It will
            # abort on its own deadline; say so plainly rather than
            # pretending either outcome.
            self._finish(
                response,
                "plrd paused for a confirmation but sent no usable resume "
                "token, so it cannot be answered from here. It aborts the "
                "recovery on its own deadline — do not touch the printer "
                "until it reports, then re-run PLR_WIZARD_START.",
            )
            return
        self._token = token
        self._data = data
        self._state = STATE_AWAITING
        self._pauses += 1
        # The daemon's own paused report first (it carries the plan prefix
        # and the diagnosis in full), then the dialog.
        text = response.get("text")
        if isinstance(text, str) and text.strip():
            self._respond(text)
        prompt = confirm_ui.confirm_prompt(
            data, deadline_text(self.plugin.daemon_keys.get("confirm_timeout_s"))
        )
        prompts.emit_prompt(self._gcode().respond_info, prompt)
        self._arm_timer()

    def _finish(self, response, note):
        self._answering = None
        self._token = None
        self._data = None
        self._state = STATE_IDLE
        self._disarm_timer()
        text = response.get("text")
        if isinstance(text, str) and text.strip():
            self._respond(text)
        else:
            self._respond("plrd returned an empty recovery report")
        self._end_dialog()
        self._respond(note)
        self._notify_finished()

    def _notify_finished(self):
        callback = self._on_finished
        self._on_finished = None
        if callback is None:
            return
        try:
            callback()
        except Exception:
            # A listener (the wizard's own state reset) must not be able to
            # take klippy down from a reactor callback.
            logger.exception("plr: recovery completion listener failed")

    def _on_error(self, error):
        """Transport failure, on the reactor thread.  Never raises."""
        if self._closed:
            return
        # Whether the call that failed was the operator's ANSWER decides
        # what is now unknown: an answer may or may not have reached plrd
        # before the transport broke, and that is materially different from
        # losing contact with a recovery nobody was being asked about.
        answering = self._answering
        self._answering = None
        self._token = None
        self._data = None
        self._state = STATE_IDLE
        self._disarm_timer()
        self._end_dialog()
        self._respond(
            "PLR recovery: lost contact with plrd (%s).\n"
            "plrd may still be executing the recovery — it does not need this "
            "plugin to finish, and it refuses a second execution while one "
            "runs. DO NOT touch the printer until you have checked "
            "'journalctl -u plrd'; use M112 if the machine must stop NOW.%s"
            % (
                error,
                (
                    "\nYour answer ('%s') may or may not have reached plrd, and "
                    "the confirmation can no longer be answered from here: if it "
                    "did not arrive, plrd aborts the recovery on its own "
                    "deadline." % (answering,)
                    if answering is not None
                    else ""
                ),
            )
        )
        self._notify_finished()

    # -- local deadline (see the module docstring) --------------------

    def _arm_timer(self):
        waketime = self.reactor.monotonic() + prompt_deadline(
            self.plugin.daemon_keys.get("confirm_timeout_s")
        )
        if self._timer is None:
            self._timer = self.reactor.register_timer(self._on_expiry, waketime)
        else:
            self.reactor.update_timer(self._timer, waketime)

    def _disarm_timer(self):
        if self._timer is not None:
            self.reactor.update_timer(self._timer, self.reactor.NEVER)

    def _on_expiry(self, eventtime):
        # Reactor timer callback: MUST NOT raise (klippy/klippy.py:170-186).
        try:
            if self._state == STATE_AWAITING:
                self._token = None
                self._data = None
                self._state = STATE_IDLE
                self._end_dialog()
                self._respond(
                    "PLR recovery: the confirmation went unanswered past "
                    "plrd's own deadline, so plrd has aborted the recovery by "
                    "now (a clean abort is the safe direction). Nothing was "
                    "answered on your behalf. Re-run PLR_WIZARD_START for a "
                    "fresh dry run."
                )
                self._notify_finished()
        except Exception:
            logger.exception("plr: recovery confirm-timeout handler failed")
        return self.reactor.NEVER

    # -- klippy lifecycle ---------------------------------------------

    def _handle_stop(self):
        """``klippy:shutdown`` / ``klippy:disconnect`` mid-recovery.

        Runs inside ``reactor.assert_no_pause()`` for a shutdown
        (klippy/klippy.py:210), so: no waiting, no g-code, nothing that
        can pause.  An outstanding pause is answered ``abort`` from a
        detached thread — plrd applies the answer the moment it arrives
        (ctrlsock.rs:776), so nobody has to read the reply — which ends
        the recovery now instead of at plrd's deadline.  An M112 or an MCU
        shutdown at a confirm-point is exactly this path.
        """
        if self._closed:
            return
        self._closed = True
        token = self._token
        self._token = None
        self._data = None
        self._state = STATE_IDLE
        self._disarm_timer()
        if token is not None:
            self._abort_detached(token)
            self._respond(
                "PLR recovery: klippy stopped while plrd was waiting for a "
                "confirmation. The answer 'abort' has been sent to plrd so it "
                "stops now rather than at its deadline; it invalidates the Z "
                "frame, so a fresh dry run is required before any resume."
            )
            self._end_dialog()
        # Tell the listener too, or the wizard sits in its "running" state
        # for the rest of the klippy session (a shutdown does not tear the
        # plugin down — klippy stays up until FIRMWARE_RESTART), reporting a
        # recovery that is over.  That is the wedged-UI failure this whole
        # branch exists to remove.
        self._notify_finished()

    def _abort_detached(self, token):
        """Fire-and-forget ``recover_confirm ... abort``; the reply is dropped.

        Deliberately NOT an :class:`~plr.daemon_worker.AsyncDaemon` call:
        that channel is closed by now, and there is nothing left on the
        reactor that could act on a reply.
        """
        link = self.plugin.daemon

        def run():
            try:
                link.call(
                    "recover_confirm",
                    {"token": token, "answer": ANSWER_ABORT},
                    timeout=daemon_link.STATUS_TIMEOUT,
                )
            except Exception:
                # Nobody is listening and nothing can be done: plrd's own
                # deadline remains the backstop.
                logger.exception("plr: shutdown abort of the recovery failed")

        thread = threading.Thread(target=run, name="plr-abort")
        thread.daemon = True
        thread.start()


# --- console command entry points (wired in plr.plugin) --------------
#
# These two ARE the interaction: the dialog's buttons fire exactly these
# commands and nothing else, and the console fallback names exactly these,
# so the whole confirm-point conversation is completable from a bare
# console on a client that renders no dialog at all.


def cmd_PLR_RECOVER_CONTINUE(plugin, gcmd):
    """PLR_RECOVER_CONTINUE — answer plrd's confirmation with 'continue'."""
    plugin.recovery.answer(gcmd, ANSWER_CONTINUE, CMD_CONTINUE)


def cmd_PLR_RECOVER_ABORT(plugin, gcmd):
    """PLR_RECOVER_ABORT — answer plrd's confirmation with 'abort'."""
    plugin.recovery.answer(gcmd, ANSWER_ABORT, CMD_ABORT)
