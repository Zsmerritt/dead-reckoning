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

**The wait and the claim end at different times.**  Keeping P at the
ceiling is right — a wait shorter than plrd's real deadline would tell the
operator a live question is dead — but on the common configuration
(``confirm_timeout_s`` unset) plrd's real deadline is 600 s while P is
3630 s.  Asserting ``awaiting_confirmation`` for the 50 minutes in
between, and refusing new recoveries all the while, would be a guess
dressed as a fact.  So there is a second, earlier stage: at
:func:`claim_deadline` the pause is marked DOUBTFUL — the plugin says plrd
has probably aborted already, publishes :data:`STATE_UNKNOWN` instead of
``awaiting_confirmation``, and stops refusing a fresh recovery — while
still keeping the token, because a late answer is worth attempting and
plrd's own reply is the only thing that can actually settle it.  The
observation that would remove the guess entirely (a session-liveness
query) does not exist: ``build_status`` takes only the config
(ctrlsock.rs:387) and never sees ``CtrlState.session``, so ``status``
cannot report whether an execution is paused.  That is a required daemon
follow-up, not a nice-to-have.

**A SHUTDOWN IS NOT TEARDOWN.**  ``klippy:shutdown`` leaves the object
graph intact and the reactor running — klippy stays up until
``FIRMWARE_RESTART`` — so this session keeps reporting through it, and so
do the query channels: an operator who has just hit M112 because a
recovery looked wrong is exactly the operator who needs ``PLR_STATUS`` to
work.  What a shutdown forbids is *acting*: starting a recovery, or
answering ``continue``, are both refused while ``printer.is_shutdown()``.
Only ``klippy:disconnect`` (klippy/klippy.py:195, sent as the run loop
unwinds) is terminal.
"""

import logging
import threading

from . import confirm_ui, daemon_link, daemon_worker, prompts

logger = logging.getLogger(__name__)

# Re-exported so plr.plugin's command table and the wizard name the same
# strings the dialog's buttons fire.
CMD_CONTINUE = confirm_ui.CMD_CONTINUE
CMD_ABORT = confirm_ui.CMD_ABORT
CMD_ACCEPT = confirm_ui.CMD_ACCEPT
CMD_NEXT = confirm_ui.CMD_NEXT
CMD_PREV = confirm_ui.CMD_PREV
CMD_NUDGE = confirm_ui.CMD_NUDGE
KIND_PREVIEW = confirm_ui.KIND_PREVIEW

# The wire ``answer`` verbs plrd accepts (crates/plrd/src/ctrlsock.rs
# ``cmd_recover_confirm`` / ``parse_preview_answer``): a BINARY pause takes
# continue/abort; a resume-PREVIEW pause takes accept/next/prev/nudge/abort,
# with ``nudge`` carrying a signed integer ``count`` (+n forward, -n back).
# ``abort`` is valid on either kind.
ANSWER_CONTINUE = "continue"
ANSWER_ABORT = "abort"
ANSWER_ACCEPT = "accept"
ANSWER_NEXT = "next"
ANSWER_PREV = "prev"
ANSWER_NUDGE = "nudge"

# Verbs valid only on a preview pause, and the full recognized set.  A
# preview verb on a binary pause (or ``continue`` on a preview pause) is
# refused BEFORE the token is spent, so the pause and its answerability
# survive a mis-typed command (the daemon guards the same mismatch, with
# the pause restored — this is the local, token-preserving mirror of it).
_PREVIEW_ANSWERS = (ANSWER_ACCEPT, ANSWER_NEXT, ANSWER_PREV, ANSWER_NUDGE)
_ALL_ANSWERS = (ANSWER_CONTINUE, ANSWER_ABORT) + _PREVIEW_ANSWERS

# --- session states ---------------------------------------------------
#
# ONE AUTHORITY.  ``_state`` always holds the state as PUBLISHED: there is
# no mapping at any read site, because two surfaces deriving the published
# state independently is how the console ended up telling an operator to
# answer a question the JSON had already downgraded.  Every surface —
# ``status_lines`` for the console, ``get_status`` for clients, and the
# wizard's gates — reads :meth:`RecoverySession.state` and nothing else.
#
# Whether an ANSWER can still be sent is a separate fact (``_token``), not a
# state: a question can outlive the plugin's ability to vouch for it.
STATE_IDLE = "idle"  # nothing known to be happening
# A request of OURS is in flight, so plrd is engaged with this session and
# its reply comes to this console.  Set the moment the request goes out,
# which is what makes it honest: it claims a live conversation, not a
# completed handshake, and plrd's reply moves the state again within one
# round trip.
STATE_RUNNING = "running"
STATE_AWAITING = "awaiting_confirmation"  # paused, answerable, demonstrably live
# plrd told us it IS executing a recovery (`busy`) that this session cannot
# report on or answer.  Positive evidence of liveness — the opposite of
# idle — and usually our own recovery, seen again after we lost contact
# with it (crates/plrd/src/ctrlsock.rs:631-645 answers `busy` exactly when
# the execution task is NOT finished).
STATE_PLRD_BUSY = "plrd_busy"
# plrd's status is genuinely unknown: it answered something that does not
# say what it is doing, or the plugin can no longer show that a pause is
# live.  Never collapsed to idle — plrd may be paused with the nozzle at
# standoff and the heaters at target.
STATE_UNKNOWN = "unknown"

# HOW ALARMING EACH STATE IS — the ordering the transition guard enforces.
#
# Not "how bad" but "how much of the machine may be moving without this
# plugin being able to say so".  Moving UP this scale is always allowed:
# evidence that the machine may be busy needs no permission.  Moving DOWN
# — publishing something calmer — requires the caller to name its reason,
# which is what :meth:`RecoverySession._transition` enforces and what makes
# the four rounds of "published idle while plrd drove the machine" bugs
# unwriteable rather than merely absent.
#
# `running` and `awaiting_confirmation` sit at the SAME level on purpose:
# both mean "plrd is engaged with THIS session and we have a live
# conversation", and the confirm loop moves between them freely in both
# directions (pause -> answer -> pause).
_ALARM = {
    STATE_IDLE: 0,  # demonstrable: nothing is in flight
    STATE_RUNNING: 1,  # demonstrable: plrd is engaged, and talking to us
    STATE_AWAITING: 1,
    STATE_PLRD_BUSY: 2,  # demonstrable: plrd is engaged, but not with us
    STATE_UNKNOWN: 3,  # nothing is demonstrable
}

# The outcomes that mean NOTHING IS LEFT RUNNING, enumerated from the
# producer rather than assumed: crates/plrd/src/ctrlsock.rs `outcome_tag`
# (:587-595) for the non-plan pipeline results and :826-833 for the final
# execution report, plus the `error_response` tag at :602-611 (`refused`).
#
# TWO TAGS ARE DELIBERATELY EXCLUDED, for the same reason:
#
# * `error` — covers both "pipeline task failed" (nothing was sent) and
#   "execution task failed" (:834: the execution task did not even return,
#   so its abort cleanup never ran and the machine may be mid-motion);
# * `malformed` — `cmd_recover_confirm` returns it at :740-751, BEFORE
#   `session.outstanding.take()`, so plrd is still paused at the confirm
#   point with the nozzle where it left it.  On `recover_execute` the same
#   tag means nothing ran (:617-627).
#
# One tag, two opposite machine states, so both take the conservative
# branch.  The rule is the tag has to prove the machine is free, not merely
# be a failure.
TERMINAL_OUTCOMES = frozenset(
    [
        "completed",
        "aborted-or-refused",
        "refused",
        "clean-shutdown",
        "machine-rejected",
        "manual-fallback",
        "not-possible",
    ]
)

# Terminal AND provably nothing-happened, because plrd rejected the request
# before it dispatched anything: `unknown-cmd` (ctrlsock.rs:371) and
# `oversized` (:324, sent from `read_request_line` before dispatch).  Split
# out from the set above because they deserve their own sentence — a plugin
# newer than its daemon hits `unknown-cmd` on every attempt, and telling
# that operator "DO NOT touch the printer" would be both wrong and the
# reason they stop believing the warnings.
PROTOCOL_REFUSAL_OUTCOMES = frozenset(["unknown-cmd", "oversized"])

# --- ONE reading of plrd's tags ---------------------------------------
#
# Every consumer classifies a response through :func:`classify` and nothing
# else.  A second bespoke reading is how the detached abort came to treat
# six typed refusals — and no reply at all — as acceptance: it had its own
# idea of what plrd's answers meant.
ANSWER_PAUSE = "pause"  # plrd is paused and answerable
ANSWER_BUSY = "busy"  # plrd is executing something (positive proof)
ANSWER_STALE_TOKEN = "stale-token"  # the token is not the outstanding one
ANSWER_PROTOCOL_REFUSAL = "protocol-refusal"  # rejected before dispatch
ANSWER_TERMINAL = "terminal"  # nothing is left running
ANSWER_UNCLASSIFIABLE = "unclassifiable"  # says nothing about the machine


def classify(response):
    """``(kind, outcome)`` for one plrd response — the single reading.

    Pure: no state, no side effects, so the confirm loop and the detached
    abort's report cannot disagree about what an answer meant.
    """
    if not isinstance(response, dict):
        return (ANSWER_UNCLASSIFIABLE, None)
    data = response.get("data")
    if not isinstance(data, dict):
        return (ANSWER_UNCLASSIFIABLE, None)
    outcome = data.get("outcome")
    if outcome == "awaiting_confirmation":
        return (ANSWER_PAUSE, outcome)
    if outcome == "busy":
        return (ANSWER_BUSY, outcome)
    if outcome == "unknown-token":
        return (ANSWER_STALE_TOKEN, outcome)
    if outcome in PROTOCOL_REFUSAL_OUTCOMES:
        return (ANSWER_PROTOCOL_REFUSAL, outcome)
    if outcome in TERMINAL_OUTCOMES:
        return (ANSWER_TERMINAL, outcome)
    if response.get("ok") is True:
        # `ok` is true only when the command reached its good outcome
        # (ctrlsock.rs:25-29), so an unrecognized tag with ok:true is a
        # completion under a name this plugin does not know yet.
        return (ANSWER_TERMINAL, outcome)
    return (ANSWER_UNCLASSIFIABLE, outcome)


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

# plrd's OWN default deadline (crates/plr-recovery/src/build.rs:324-328
# CONFIRM_TIMEOUT_DEFAULT_S, matching DEFAULT_CONFIRM_TIMEOUT at
# crates/plrd/src/executor.rs:136-143), used for ONE purpose: to know when
# the plugin has stopped being able to claim a pause is live.
#
# It is emphatically NOT used as the plugin's wait — that stays the ceiling
# above, because a wait shorter than plrd's real deadline is the worse
# direction (it would tell the operator a live question is dead).  What it
# bounds is the CLAIM: once this much has passed with no answer, on a
# printer that never set `confirm_timeout_s`, plrd has almost certainly
# aborted already, and asserting `awaiting_confirmation` for another 50
# minutes — refusing new recoveries all the while — would be a fabrication.
# tests/test_recovery_deadlines.py parses the value out of the Rust and
# fails if this copy drifts from it; the safe direction if it ever does is
# for this number to be too SMALL (an earlier downgrade), never too large.
DAEMON_CONFIRM_DEFAULT_S = 600.0


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


def claim_deadline(configured):
    """When the plugin must stop CLAIMING the pause is live, or ``None``.

    ``None`` when the operator set ``confirm_timeout_s``: the plugin then
    knows plrd's deadline exactly, so :func:`prompt_deadline` is both the
    end of the claim and the end of the wait.  Otherwise it is plrd's own
    default plus the same headroom — the point past which "awaiting
    confirmation" would be a guess dressed as a fact.
    """
    _deadline, exact = daemon_confirm_deadline(configured)
    if exact:
        return None
    return DAEMON_CONFIRM_DEFAULT_S + CONFIRM_HEADROOM_S


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
        # The initial value is the only direct assignment: there is no
        # prior state to compare against, and `_transition` is what every
        # later change goes through.
        self._state = STATE_IDLE
        self._token = None
        self._data = None  # the outstanding pause's data map, for re-show
        self._answering = None  # the answer currently in flight, if any
        self._source = None  # which command started this recovery
        self._on_finished = None
        self._pauses = 0
        # Bumped for every new conversation, so an answer that arrives from a
        # detached thread cannot land on a LATER one (the precedent is
        # AsyncDaemon._generation).  The abort delivery carries the epoch it
        # was started in.
        self._epoch = 0
        self._answering_token = None
        # The claim that was standing BEFORE the current request went out, so
        # an answer that is only about the request can put it back instead of
        # inventing one.
        self._prior_state = STATE_IDLE
        self._timer = None
        self._claim_timer = None
        self._closed = False
        # A SHUTDOWN is not teardown: klippy stays up until
        # FIRMWARE_RESTART, so the session must keep answering questions
        # about itself (and must not act).  Only `disconnect` is terminal.
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )

    # -- introspection ------------------------------------------------

    # -- the one writer -----------------------------------------------

    def _transition(self, state, reason=None):
        """THE ONLY place ``_state`` is assigned.  Enforces the asymmetry.

        Moving toward MORE alarming (:data:`_ALARM`) needs nothing: evidence
        that the machine may be busy is always publishable.  Moving toward
        CALMER requires ``reason`` — a short phrase naming the evidence — and
        a call that omits it is REFUSED: the more alarming state stands and
        the mistake is logged.

        This exists because four review rounds found the same bug in four
        different places: a write site that published a calmer state than
        the evidence supported.  Reads were made safe last round (one
        authority, no second mapping) while fifteen writes stayed free to
        invent any value, so the invariant was still enforced by author
        vigilance — which had already failed four times.  Now the vigilance
        is mechanical: ``tests/test_recovery_confirm.py`` asserts by AST
        that nothing outside this method assigns ``_state``, and asserts
        that an unreasoned calming transition is refused at runtime.
        """
        current = self._state
        if state == current:
            return
        if _ALARM[state] < _ALARM[current] and not reason:
            # Refuse, do not raise: this runs on the reactor, and the safe
            # direction is to keep publishing the alarm.
            logger.error(
                "plr: refusing to publish %r over %r with no stated reason "
                "(recovery state may only get calmer for a named reason)",
                state,
                current,
            )
            return
        logger.info(
            "plr: recovery state %r -> %r%s",
            current,
            state,
            " (%s)" % (reason,) if reason else "",
        )
        self._state = state

    def _raise_alarm(self, state):
        """Publish ``state`` only if it is at least as alarming as now.

        The ONLY transition primitive the detached-abort path may use, which
        is what makes that path structurally incapable of publishing
        anything calmer — a rule rather than a careful reading, because the
        careful reading is what failed.  ``tests/test_recovery_confirm.py``
        asserts by AST that the abort path calls no other one.
        """
        if _ALARM[state] < _ALARM[self._state]:
            logger.info(
                "plr: keeping %r rather than %r (an alarm may only be raised "
                "here, never lowered)",
                self._state,
                state,
            )
            return
        self._transition(state)

    # -- introspection ------------------------------------------------

    def state(self):
        """THE published state — the single authority every surface reads.

        ``_state`` is stored as published, so this is a plain read and there
        is nowhere for a second interpretation to grow.  ``status_lines``,
        ``plugin.get_status`` and the wizard's gates all come through here.
        """
        return self._state

    # Two questions that used to share one boolean, which is exactly why the
    # console and the API could disagree and why `start()` could throw away a
    # token the same breath had promised to keep.  Each call site names the
    # question it is asking.

    def may_start_new(self):
        """May a NEW recovery be started right now?

        False only when the plugin KNOWS plrd is occupied with this
        session's recovery (running, or paused on a question we can still
        answer).  True for both unknowable states, because the plugin cannot
        resolve them and plrd can: a fresh ``recover_execute`` either runs
        or answers ``busy``, and that answer is the only observation
        available (``status`` cannot report session state — see the module
        docstring).  Starting one abandons any kept question, loudly and on
        purpose (:meth:`start`).
        """
        return self._state in (STATE_IDLE, STATE_PLRD_BUSY, STATE_UNKNOWN)

    def needs_attention(self):
        """Is there anything the operator must not walk away from?

        True for everything except idle — including both unknowable states,
        whose whole point is that the machine may be under plrd's control.
        Callers that want "may I start one" must ask
        :meth:`may_start_new`; conflating the two is what published
        ``idle`` while plrd was driving.
        """
        return self._state != STATE_IDLE

    def can_answer(self):
        """Can an answer still be sent for an outstanding question?

        A property of the TOKEN, not of the state: a question outlives the
        plugin's ability to vouch for it (see the claim/wait split in the
        module docstring), and a late answer is still worth attempting
        because plrd's own reply adjudicates.
        """
        return self._token is not None

    def status_lines(self):
        """Console lines for ``PLR_STATUS`` — never a daemon call.

        Branches on :meth:`state` (never on the raw field), so the console
        and ``get_status`` cannot drift apart.
        """
        state = self.state()
        if state == STATE_IDLE:
            return ["recovery: idle"]
        if state == STATE_PLRD_BUSY:
            lines = [
                "recovery: plrd IS EXECUTING A RECOVERY — it told us so "
                "('busy'), so the machine is under its control.",
                "  This session cannot report on it or stop it (it may be the "
                "recovery this plugin lost contact with). DO NOT touch the "
                "printer; check 'journalctl -u plrd'.",
                "  Re-run PLR_RECOVER EXECUTE=1 CONFIRM=YES to ask again — "
                "plrd answers 'busy' for as long as it is still working.",
            ]
        elif state == STATE_UNKNOWN:
            lines = [
                "recovery: UNKNOWN — plrd did not say what it is doing now, "
                "and this plugin cannot tell.",
                "  It may still be executing or paused. DO NOT touch the "
                "printer; check 'journalctl -u plrd'.",
                "  PLR_RECOVER EXECUTE=1 CONFIRM=YES is safe as a probe: plrd "
                "answers 'busy' if it is still working.",
            ]
        elif state == STATE_RUNNING:
            lines = [
                "recovery: RUNNING (started by %s) — plrd is driving the "
                "machine; watch the console for its report" % (self._source,)
            ]
        else:
            lines = [
                "recovery: AWAITING CONFIRMATION — question %d of this "
                "recovery (started by %s)" % (self._pauses, self._source)
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
        if self.can_answer():
            lines.append(
                "  a question can still be answered: %s" % (self._answer_commands(),)
            )
        return lines

    def reshow(self, respond=None):
        """Re-emit the outstanding question, prompt and all.

        The operator has to be able to get the question back: on a
        console-first client the pause scrolls away, and the three-part
        why/fix/offer message is otherwise shown exactly once, at emission.
        Somebody who cannot see it any more is somebody clicking Continue
        blind.  Called from PLR_STATUS and PLR_WIZARD_START — explicit
        operator actions, never a poll.
        """
        if not self.can_answer() or not isinstance(self._data, dict):
            return False
        prompts.emit_prompt(
            respond or self._gcode().respond_info,
            confirm_ui.confirm_prompt(self._data, self._deadline_text()),
        )
        return True

    def _deadline_text(self):
        if self._state != STATE_AWAITING:
            # The question is still answerable but the plugin can no longer
            # vouch for it (the state has been downgraded), so it must not
            # repeat the confident deadline sentence.
            return (
                "This question is past plrd's own default deadline, so plrd "
                "has probably aborted the recovery already — answering may "
                "simply report that."
            )
        return deadline_text(self.plugin.daemon_keys.get("confirm_timeout_s"))

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
        if self._closed or self._async.is_closed() or self.printer.is_shutdown():
            raise gcmd.error(
                "%s: klippy is shut down or shutting down — clear the "
                "shutdown (FIRMWARE_RESTART) before attempting a recovery." % (source,)
            )
        if not self.may_start_new():
            raise gcmd.error(self._busy_message(source))
        prior_epoch = self._epoch
        # A worker still holding the channel means a previous conversation
        # was abandoned (a transport error, or a cancelled flow whose orphan
        # has not unblocked).  Refusing is the safe direction: plrd may
        # still be executing that recovery.
        if self._async.is_busy():
            raise gcmd.error(
                "%s: the previous plrd conversation has not finished. plrd "
                "may still be executing that recovery — check PLR_STATUS and "
                "the plrd log (journalctl -u plrd) before retrying." % (source,)
            )
        # THE STATE IS NOT TOUCHED UNTIL THE LAUNCH SUCCEEDS.  There is
        # nothing to roll back, which is what makes the old bug unwriteable:
        # `may_start_new()` admits `plrd_busy` and `unknown`, so a rollback
        # to `idle` published "nothing is happening" over positive evidence
        # that plrd was executing — on the very command this plugin
        # advertises as the probe for those states.  Nothing can interleave
        # here: the callbacks arrive through the reactor, which is this
        # thread.
        self._source = source
        self._on_finished = on_finished
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
            # `refusal_text` says WHICH refusal it was — busy, closed, or a
            # worker thread that could not be created — instead of guessing.
            # The published state, and any kept question, are exactly as they
            # were: nothing was started, so nothing is claimed.
            raise gcmd.error(
                self._async.refusal_text(source)
                or "%s: plrd could not be contacted; nothing was started." % (source,)
            )
        self._pauses = 0
        self._data = None
        self._answering = None
        self._answering_token = None
        self._epoch += 1
        self._prior_state = self._state
        # A kept question is ABANDONED here — after the launch is real, never
        # before — and out loud.  The doubtful branch promises the operator
        # that CONTINUE/ABORT still work; starting a new recovery destroys
        # that, so it says so and asks plrd to drop the question rather than
        # leaving it paused against its own deadline.
        abandoned = self._token
        self._token = None
        if abandoned is not None:
            self._abort_detached(
                abandoned, what="the abandoned question", epoch=prior_epoch
            )
            gcmd.respond_info(
                "%s: abandoning the confirmation that was still open. An "
                "'abort' for it has been SENT to plrd — whatever plrd answers "
                "cannot confirm it was applied, so nothing here will claim it "
                "was; %s / %s will no longer answer it."
                % (source, CMD_CONTINUE, CMD_ABORT)
            )
            self._end_dialog()
        # Accurate about what is known at this instant: the worker started
        # and a request is in flight, which is exactly what `running` means.
        # plrd has NOT answered yet and may reply `busy` — that reply is one
        # round trip away and puts the state back, which is the same bounded,
        # self-correcting claim as N1 in `answer()`.
        self._transition(
            STATE_RUNNING,
            reason="a recover_execute is in flight; plrd's reply adjudicates",
        )
        # Conditional on purpose: plrd has not answered yet and may refuse
        # (it is entitled to say `busy`, or that the machine does not
        # validate), so the first line the operator reads must not assert
        # motion that may never happen.
        gcmd.respond_info(
            "%s: asked plrd to execute the recovery. If it accepts, THE "
            "PRINTER WILL MOVE — plrd reports here either way.\n"
            "This command returns immediately. Send NOTHING else to the "
            "printer until it reports: every command you send holds the "
            "g-code mutex plrd needs for its own, so this must be the last "
            "line of any macro.\n"
            "If plrd stops to ask you something, a prompt appears and the "
            "console names the exact command to answer it (it depends on the "
            "question — a resume-point preview is answered differently from a "
            "yes/no confirmation)." % (source,)
        )

    def _busy_message(self, source):
        if self._state == STATE_AWAITING:
            return (
                "%s: a recovery is already in flight and is WAITING FOR YOUR "
                "ANSWER — run %s. Run PLR_STATUS to see the question again."
                % (source, self._answer_commands())
            )
        return (
            "%s: a recovery is already in flight (started by %s). plrd runs "
            "exactly one at a time; wait for its report. Use M112 if the "
            "machine must stop NOW." % (source, self._source)
        )

    # -- answering ----------------------------------------------------

    def _pause_kind(self):
        """The outstanding pause's ``confirm_kind``, or None.

        Read from the cached pause payload (``report_pause``'s ``data``
        map), which is the same source :meth:`reshow` renders from, so the
        kind this method discriminates on is exactly the kind the operator
        was shown.
        """
        if isinstance(self._data, dict):
            return self._data.get("confirm_kind")
        return None

    def _answer_commands(self):
        """The console commands that answer the CURRENT outstanding pause,
        branched on its kind.

        A resume-preview pause (the routine first pause under the default
        ``ask``) is answered with the reposition verbs, NOT continue/abort —
        so every place the plugin tells an operator how to answer must name
        the right ones. Pointing a preview operator at ``PLR_RECOVER_CONTINUE``
        would hand them a command the daemon refuses on a healthy pause, and
        would contradict the dialog's own fallback line on the same screen.
        Falls back to the binary pair when the kind is unknown (no pause
        cached yet, or a non-preview pause).
        """
        if self._pause_kind() == KIND_PREVIEW:
            return "%s / %s / %s / %s FWD=|BACK= / %s" % (
                CMD_ACCEPT,
                CMD_NEXT,
                CMD_PREV,
                CMD_NUDGE,
                CMD_ABORT,
            )
        return "%s or %s" % (CMD_CONTINUE, CMD_ABORT)

    def answer(self, gcmd, answer, source, count=None):
        """Answer the outstanding confirm-point.

        ``continue``/``abort`` answer a binary pause; ``accept``/``next``/
        ``prev``/``nudge`` answer a resume-preview pause (``nudge`` carries a
        signed ``count``).  ``abort`` answers either.  The vocabulary is
        widened for preview pauses ONLY, discriminated by the outstanding
        pause's kind (design §D.3 / §F.2).
        """
        if answer not in _ALL_ANSWERS:
            # Not reachable from the commands; a guard against a future
            # caller inventing an answer plrd would reject.
            raise gcmd.error("%s: unknown answer %r" % (source, answer))
        if not self.can_answer():
            raise gcmd.error(self._nothing_to_answer(source))
        # THE SHUTDOWN RULE (design §D.2).  A shut-down machine cannot move,
        # so every verb that would resume or REPOSITION is refused —
        # accept, next, prev and nudge as well as continue.  Only abort is
        # allowed, because aborting is the one thing a shut-down recovery
        # can still do (and is what the shutdown handler itself does).
        if answer != ANSWER_ABORT and self.printer.is_shutdown():
            raise gcmd.error(
                "%s: klippy is shut down — the recovery cannot %s a printer "
                "that cannot move. Run %s to stop it."
                % (
                    source,
                    "reposition" if answer in _PREVIEW_ANSWERS else "continue",
                    CMD_ABORT,
                )
            )
        # KIND DISCRIMINATION (design §D.3).  A preview verb is valid only on
        # a preview pause, and ``continue`` only on a binary one; ``abort``
        # is valid on either.  Refuse a mismatch HERE, before the token is
        # spent, so the pause stays answerable — the token-preserving mirror
        # of the daemon's own "wrong-kind → pause restored" guard.
        kind = self._pause_kind()
        if answer in _PREVIEW_ANSWERS and kind != KIND_PREVIEW:
            raise gcmd.error(
                "%s: this pause is not a resume preview (it is a %r "
                "confirmation) — answer it with %s or %s."
                % (source, kind or "binary", CMD_CONTINUE, CMD_ABORT)
            )
        if answer == ANSWER_CONTINUE and kind == KIND_PREVIEW:
            raise gcmd.error(
                "%s: this is a resume-preview pause — answer it with %s to "
                "accept the shown point, %s / %s to step between candidates, "
                "%s FWD=/BACK= to move along the toolpath, or %s to stop."
                % (source, CMD_ACCEPT, CMD_NEXT, CMD_PREV, CMD_NUDGE, CMD_ABORT)
            )
        token = self._token
        was = self._state
        # Remember the deadlines so a rollback restores them rather than
        # re-arming from now, which would let the plugin assert a live
        # question well past plrd's real one.
        was_waketimes = self._waketimes()
        # Hand the token to plrd and forget it locally: the pause is no
        # longer outstanding as far as this plugin is concerned, so a
        # double-click cannot answer twice.
        self._token = None
        self._answering_token = token
        self._epoch += 1
        self._prior_state = was
        # N1, justified here: republishing `running` over a DOWNGRADED
        # question is a positive claim the plugin cannot prove — plrd may
        # have aborted the question an hour ago.  It is accepted because it
        # is bounded and self-correcting: the answer is already on its way to
        # plrd, whose reply arrives on this same channel and adjudicates
        # within one round trip (`unknown-token` -> back to `unknown`, a
        # pause -> `awaiting`, a terminal tag -> `idle`).  Publishing
        # `unknown` instead would say "nothing is being asked of plrd" while
        # a request is in flight, which is a different lie.
        self._transition(
            STATE_RUNNING,
            reason="an answer for the outstanding question is in flight; "
            "plrd's reply adjudicates",
        )
        self._answering = answer
        self._disarm_timer()
        # ``nudge`` carries a signed ``count`` the daemon requires
        # (parse_preview_answer: non-zero i32); every other verb is
        # answer-only.  Built here, after the mismatch guards, so a stray
        # count on a non-nudge verb never reaches the wire.
        args = {"token": token, "answer": answer}
        if answer == ANSWER_NUDGE:
            args["count"] = count
        started = self._async.call(
            "recover_confirm",
            args,
            daemon_link.EXECUTE_TIMEOUT,
            self._on_response,
            self._on_error,
        )
        if not started:
            # The channel is busy: something else is already talking to
            # plrd on it.  Put the pause back rather than losing it.
            self._token = token
            self._answering_token = None
            # Back to whatever it was — NOT unconditionally AWAITING, which
            # would re-assert a liveness the downgrade had withdrawn.
            self._transition(was, reason="the answer was never sent")
            self._answering = None
            self._restore_timers(was_waketimes)
            raise gcmd.error(
                "%s: still waiting for plrd's previous reply; try again in a "
                "moment (the question is still open)." % (source,)
            )
        self._end_dialog()
        if answer in _PREVIEW_ANSWERS and answer != ANSWER_ACCEPT:
            # A reposition (next/prev/nudge): plrd moves the hover point and
            # PAUSES AGAIN with the new stop, which arrives here as the next
            # prompt — not a completion.
            gcmd.respond_info(
                "%s: repositioning. plrd moves the hover point and shows the "
                "next stop here." % (source,)
            )
        else:
            gcmd.respond_info(
                "%s: answered '%s'. plrd is continuing; its next report "
                "appears here." % (source, answer)
            )

    def _nothing_to_answer(self, source):
        if self._state in (STATE_RUNNING, STATE_PLRD_BUSY):
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
        if self.can_answer():
            self.answer(gcmd, ANSWER_ABORT, source)
            return True
        if self._state in (STATE_PLRD_BUSY, STATE_UNKNOWN):
            # Nothing to cancel and nothing to claim: the warning belongs to
            # the machine's state, not to a dialog, so dismissing a dialog
            # must not clear it.
            raise gcmd.error(
                "%s: there is no question to cancel, and this plugin cannot "
                "stop what plrd may be doing.\n%s"
                % (source, "\n".join(self.status_lines()))
            )
        raise gcmd.error(
            "%s: plrd is executing the recovery and it cannot be cancelled "
            "from here. It stops by itself on any failed verification; use "
            "M112 if the machine must stop NOW." % (source,)
        )

    # -- the confirm loop ---------------------------------------------

    def _on_response(self, response):
        """One plrd response, on the reactor thread.  Never raises.

        THE TERMINAL PATH IS REACHABLE ONLY FROM TERMINAL OUTCOMES.  Every
        other answer resolves toward liveness, because the states this
        plugin can publish are not symmetric: publishing ``idle`` while plrd
        drives the machine is the failure that gets somebody's hand into an
        enclosure, and publishing "unknown" when plrd was in fact finished
        costs one probe.
        """
        if self._closed:
            return
        kind, outcome = classify(response)
        if kind == ANSWER_PAUSE:
            self._pause(response, response["data"])
            return
        if kind == ANSWER_BUSY:
            self._plrd_busy(response)
            return
        if kind == ANSWER_STALE_TOKEN:
            self._unknown_token(response)
            return
        if kind == ANSWER_PROTOCOL_REFUSAL:
            # plrd rejected the REQUEST before dispatching anything, so this
            # attempt started nothing.  That is a fact about the REQUEST, not
            # about the machine: a prior `plrd_busy` is positive proof plrd is
            # executing, and publishing `idle` over it would discard the
            # evidence.  So the request is closed out and the machine claim is
            # left exactly as it was.
            self._close_out_request(
                response,
                "plrd rejected the request itself (%s), so nothing was started "
                "by it. That usually means this plugin is NEWER than the plrd "
                "it is talking to — update plrd (and check 'plrd --version' "
                "against the plugin's)." % (outcome,),
            )
            return
        if kind == ANSWER_TERMINAL:
            self._finish(
                response,
                self._final_note(response, outcome),
                reason=(
                    "plrd reported the terminal outcome %r" % (outcome,)
                    if outcome in TERMINAL_OUTCOMES
                    # The tag is unknown by construction here; what justifies
                    # the move is plrd's `ok` invariant, so say that.
                    else "plrd answered ok:true, which it does only for a "
                    "command that reached its good outcome"
                ),
            )
            return
        # A `malformed` reply to an ANSWER we sent (not a start — those carry
        # no `_answering_token`) means plrd REFUSED the verb but KEPT the
        # pause standing: `cmd_recover_confirm` returns malformed before
        # taking the outstanding question, and RESTORES it on a wrong-kind
        # answer (a preview verb on a binary pause, or `continue` on a
        # preview pause; ctrlsock.rs). The question is still answerable, so
        # the token is kept rather than thrown away — the daemon deliberately
        # preserved it. (The plugin discriminates kind locally so it should
        # never send a wrong-kind verb, but the daemon's guard is the
        # backstop, and losing the token when the daemon kept the pause is
        # the bug this branch removes.)
        if outcome == "malformed" and self._answering_token is not None:
            self._wrong_kind_restore(response)
            return
        # Unclassifiable — including `error` (ctrlsock.rs:834: one tag for
        # "the pipeline failed before anything was sent" and "the execution
        # task never returned, so its cleanup never ran") and a `malformed`
        # with nothing in flight (a rejected recover_execute — nothing ran).
        # A protocol addition must not be able to make this plugin claim a
        # finish it cannot see.
        self._unresolved(
            response,
            "plrd answered %r, which this plugin cannot classify, so what it "
            "is doing now is unknown." % (outcome,),
        )

    def _close_out_request(self, response, note):
        """End the conversation WITHOUT making any claim about the machine.

        For answers that are about the REQUEST rather than the printer.  The
        claim that was standing before this request goes back: `running` was
        published because a request was in flight, and that request turned
        out to have been rejected before plrd dispatched anything, so the
        evidence about the machine is exactly what it was — including a
        `plrd_busy` that must not be discarded.
        """
        self._answering = None
        self._answering_token = None
        self._token = None
        self._data = None
        self._disarm_timer()
        self._transition(
            self._prior_state,
            reason="plrd rejected the request before dispatching anything, so "
            "nothing about the machine changed",
        )
        text = response.get("text")
        if isinstance(text, str) and text.strip():
            self._respond(text)
        self._end_dialog()
        self._respond(note)
        if self._state != STATE_IDLE:
            self._respond(
                "This says nothing about what plrd is doing, so the recovery "
                "state is unchanged:\n%s" % ("\n".join(self.status_lines()),)
            )
        self._notify_finished()

    def _wrong_kind_restore(self, response):
        """plrd refused an answer as ``malformed`` but KEPT the pause.

        Mirrors :meth:`_plrd_busy`'s token handling: the daemon restored the
        outstanding question (ctrlsock.rs), so the same token still answers
        it — keep the token and the cached readout (so :meth:`reshow` still
        works and :meth:`_answer_commands` still names the right verbs), and
        re-arm the deadline plrd is still enforcing. The published state is
        the honest uncertainty (UNKNOWN): the plugin cannot demonstrate the
        pause is live, it can only infer it from the typed refusal, so it
        does not re-claim ``awaiting_confirmation`` — but it never pretends
        the recovery ended either.
        """
        answering_token = self._answering_token
        self._answering = None
        self._answering_token = None
        # Keep the token (and, with it, the cached pause payload): the daemon
        # deliberately preserved the question.
        self._token = answering_token
        if answering_token is None:
            self._data = None
        # Toward alarming — no reason required.
        self._transition(STATE_UNKNOWN)
        self._disarm_timer()
        if answering_token is not None:
            self._arm_timer()
        text = response.get("text")
        if isinstance(text, str) and text.strip():
            self._respond(text)
        self._end_dialog()
        self._respond(
            "PLR recovery: plrd refused that answer as malformed but KEPT the "
            "question open (it restores the pause on a wrong-kind answer). It "
            "is still answerable — run PLR_STATUS to see it again, then answer "
            "with %s. Do NOT assume the recovery ended." % (self._answer_commands(),)
        )
        self._notify_finished()

    def _plrd_busy(self, response):
        """plrd answered ``busy`` — POSITIVE PROOF that it is executing.

        ctrlsock.rs:631-645 returns ``busy`` exactly when the execution
        task is not finished (or the session lock is held), so this is the
        strongest evidence of liveness the protocol offers.  Treating it as
        a finish — which is what fell out of the generic terminal path —
        published ``idle`` in answer to proof of the opposite, right after
        the plugin had told the operator not to touch the printer.
        """
        # If this was the operator's ANSWER, plrd refused it because the
        # session lock was contended (ctrlsock.rs:752-754) — the answer never
        # landed, so the question may well still be outstanding.  Put the
        # token back rather than destroying answerability: retrying is free,
        # and plrd's typed reply adjudicates.
        answering_token = self._answering_token
        self._answering = None
        self._answering_token = None
        self._token = answering_token
        self._data = self._data if answering_token else None
        # Toward alarming: never needs a reason.
        self._transition(STATE_PLRD_BUSY)
        self._disarm_timer()
        if answering_token is not None:
            self._arm_timer()
        text = response.get("text")
        if isinstance(text, str) and text.strip():
            self._respond(text)
        self._end_dialog()
        self._respond(
            "PLR recovery: plrd IS executing a recovery and refused to start "
            "another. That is proof the machine is under its control — most "
            "likely the recovery this plugin lost contact with.\n"
            "DO NOT touch the printer. Its report goes to whatever started "
            "it; check 'journalctl -u plrd', and re-run PLR_RECOVER EXECUTE=1 "
            "CONFIRM=YES to ask again. Use M112 if the machine must stop NOW."
        )
        self._notify_finished()

    def _unknown_token(self, response):
        """plrd's ``unknown-token``, which has FOUR causes — not one.

        crates/plrd/src/ctrlsock.rs:755-784 emits the same tag for:

        1. no session at all (:755-760) — plrd is idle;
        2. a session running but not awaiting (:761-766) — plrd is EXECUTING;
        3. a token that is not the outstanding one (:767-775) — and it puts
           the question BACK (``session.outstanding = Some(outstanding)``),
           so plrd is STILL PAUSED and still answerable;
        4. the pause timed out between the lookup and the send (:776-784) —
           plrd aborted.

        Only (4) means "the recovery is over".  Telling that story for (2)
        or (3) would leave an operator believing a recovery aborted while
        the nozzle sits at standoff with the heaters at target — the exact
        failure this branch exists to remove.  So the text plrd sent is
        what distinguishes them, and anything not provably (4) takes the
        conservative branch: no abort claimed, and NOT idle.
        """
        text = response.get("text")
        text = text if isinstance(text, str) else ""
        if "timed out before the answer arrived" in text:
            # plrd's deadline expired and it is ABORTING — which means it is
            # inside `finish_abort` pushing this step's cleanup commands
            # through Moonraker right now (executor.rs).  The recovery is
            # over in intent, but the machine is not yet still, so this is
            # not `idle`: re-running the wizard here would answer `busy`.
            self._unresolved(
                response,
                "plrd is no longer waiting for that answer — its own "
                "confirmation deadline expired first and it is ABORTING the "
                "recovery now (that abort is the safe direction, and it "
                "invalidates the Z frame exactly as a decline would). It is "
                "still sending that step's cleanup commands.",
                advice="Wait for plrd to go quiet (watch 'journalctl -u "
                "plrd'), then re-run PLR_WIZARD_START for a fresh dry run.",
            )
            return
        self._unresolved(
            response,
            "plrd is not waiting for THAT answer any more, and it did not say "
            "the recovery ended: it may still be executing, or still paused "
            "on a different question.",
        )

    def _unresolved(self, response, headline, advice=None):
        """Terminal for this plugin, unresolved for the machine."""
        self._answering = None
        self._answering_token = None
        self._token = None
        self._data = None
        # Toward alarming: never needs a reason.
        self._transition(STATE_UNKNOWN)
        self._disarm_timer()
        text = response.get("text")
        if isinstance(text, str) and text.strip():
            self._respond(text)
        self._end_dialog()
        self._respond(
            "PLR recovery: %s\n"
            "DO NOT touch the printer and do not assume it has stopped. Check "
            "PLR_STATUS and 'journalctl -u plrd'; use M112 if the machine must "
            "stop NOW.\n%s"
            % (
                headline,
                advice
                or "Running PLR_RECOVER EXECUTE=1 CONFIRM=YES again is safe as "
                "a probe — plrd answers 'busy' if it is still working.",
            )
        )
        self._notify_finished()

    def _final_note(self, response, outcome):
        if response.get("ok") is True:
            note = "PLR recovery complete — plrd has resumed the print."
            if outcome != "completed":
                # Resting on a documented protocol invariant (ctrlsock.rs:
                # 25-29: `ok` is true only when the command reached its good
                # outcome), but say that the tag itself was new, so the
                # operator is not the last to know.
                note += (
                    " (plrd reported success under an outcome this plugin does "
                    "not know, %r — read its report above.)" % (outcome,)
                )
            return note
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
            # plrd is PAUSED and there is no way to answer it: it is still
            # holding the machine until its own deadline.  Unresolved, not
            # finished.
            self._unresolved(
                response,
                "plrd paused for a confirmation but sent no usable resume "
                "token, so it cannot be answered from here — it is still "
                "paused, and aborts on its own deadline.",
            )
            return
        if self.printer.is_shutdown():
            # The pause arrived AFTER the shutdown, so the dialog's primary
            # button could never work and nothing else would abort it — plrd
            # would hold the machine to its own deadline.  Answer it now, in
            # the only direction a shut-down printer allows.  (The shutdown
            # handler covers a pause outstanding AT shutdown; this covers one
            # arriving later, which is the likelier order: plrd was mid-step
            # when the M112 landed.)
            self._answering = None
            self._token = None
            self._data = None
            self._disarm_timer()
            text = response.get("text")
            if isinstance(text, str) and text.strip():
                self._respond(text)
            self._end_dialog()
            self._respond(
                "PLR recovery: plrd stopped to ask a question, but klippy is "
                "shut down so the recovery cannot continue. Clear the shutdown "
                "(FIRMWARE_RESTART), then start again with a fresh dry run."
            )
            self._abort_shutdown_pause(token)
            self._notify_finished()
            return
        self._answering = None
        self._token = token
        self._data = data
        self._transition(
            STATE_AWAITING, reason="plrd reported a confirm-point we can answer"
        )
        self._pauses += 1
        # The daemon's own paused report first (it carries the plan prefix
        # and the diagnosis in full), then the dialog.
        text = response.get("text")
        if isinstance(text, str) and text.strip():
            self._respond(text)
        prompts.emit_prompt(
            self._gcode().respond_info,
            confirm_ui.confirm_prompt(data, self._deadline_text()),
        )
        self._arm_timer()

    def _finish(self, response, note, reason="plrd reported a terminal outcome"):
        self._answering = None
        self._answering_token = None
        self._token = None
        self._data = None
        self._transition(STATE_IDLE, reason=reason)
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
        # UNKNOWN, not idle: a transport failure says nothing about whether
        # plrd is still driving the machine.  Toward alarming, so no reason
        # is required.
        self._transition(STATE_UNKNOWN)
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

    # -- local deadlines (see the module docstring) --------------------
    #
    # TWO stages, because the wait and the claim end at different times:
    #
    #   * the CLAIM ends at `claim_deadline` — only when the plugin does
    #     not know plrd's deadline — after which the pause is reported as
    #     doubtful and stops refusing a fresh recovery;
    #   * the WAIT ends at `prompt_deadline`, which strictly outlasts every
    #     deadline plrd could be using, after which the token is dropped.

    def _arm_timer(self):
        configured = self.plugin.daemon_keys.get("confirm_timeout_s")
        now = self.reactor.monotonic()
        waketime = now + prompt_deadline(configured)
        if self._timer is None:
            self._timer = self.reactor.register_timer(self._on_expiry, waketime)
        else:
            self.reactor.update_timer(self._timer, waketime)
        claim = claim_deadline(configured)
        if claim is None:
            self._disarm_claim_timer()
            return
        if self._claim_timer is None:
            self._claim_timer = self.reactor.register_timer(
                self._on_claim_expiry, now + claim
            )
        else:
            self.reactor.update_timer(self._claim_timer, now + claim)

    def _waketimes(self):
        """The two deadlines' current waketimes, for an exact restore."""
        return (
            self._timer.waketime if self._timer is not None else None,
            self._claim_timer.waketime if self._claim_timer is not None else None,
        )

    def _restore_timers(self, waketimes):
        """Put the deadlines back where they were, not where now is."""
        wait, claim = waketimes
        if wait is None and claim is None:
            self._arm_timer()
            return
        if self._timer is not None and wait is not None:
            self.reactor.update_timer(self._timer, wait)
        if self._claim_timer is not None and claim is not None:
            self.reactor.update_timer(self._claim_timer, claim)

    def _disarm_timer(self):
        if self._timer is not None:
            self.reactor.update_timer(self._timer, self.reactor.NEVER)
        self._disarm_claim_timer()

    def _disarm_claim_timer(self):
        if self._claim_timer is not None:
            self.reactor.update_timer(self._claim_timer, self.reactor.NEVER)

    def _on_claim_expiry(self, eventtime):
        # Reactor timer callback: MUST NOT raise (klippy/klippy.py:170-186).
        #
        # THE DOWNGRADE.  The state stops claiming a demonstrable pause and
        # becomes UNKNOWN; the token is KEPT so the question can still be
        # answered.  Because `_state` is the one published value, the
        # console, get_status and the wizard's gates all change together
        # here — there is no second surface left to disagree.
        try:
            if self._state == STATE_AWAITING:
                # Toward alarming: the plugin has stopped being able to show
                # the question is live.
                self._transition(STATE_UNKNOWN)
                self._end_dialog()
                self._respond(
                    "PLR recovery: this confirmation has been open longer than "
                    "plrd's own default deadline, so plrd may well have aborted "
                    "the recovery already — this plugin cannot tell, because "
                    "plrd does not report the deadline it is using.\n"
                    "Nothing was answered on your behalf, and %s still "
                    "work: plrd's reply will say whether it moved on. Starting "
                    "a NEW recovery is no longer refused, but doing so ABANDONS "
                    "this question (plrd is told to drop it). Check PLR_STATUS "
                    "first." % (self._answer_commands(),)
                )
        except Exception:
            logger.exception("plr: recovery confirm-claim handler failed")
        return self.reactor.NEVER

    def _on_expiry(self, eventtime):
        # Reactor timer callback: MUST NOT raise (klippy/klippy.py:170-186).
        try:
            # Fires for a kept question in EITHER state: the wait outlasts
            # the claim, so by here it may already have been downgraded.
            if self._token is not None and self._state in (
                STATE_AWAITING,
                STATE_UNKNOWN,
            ):
                self._answering = None
                self._token = None
                self._data = None
                self._transition(
                    STATE_IDLE,
                    reason="the wait outlasted every deadline plrd could be "
                    "using, so its clean abort has happened",
                )
                self._disarm_claim_timer()
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

    def _handle_shutdown(self):
        """``klippy:shutdown`` mid-recovery — M112, or an MCU fault.

        NOT teardown: klippy stays up until ``FIRMWARE_RESTART``, so this
        session keeps working (and keeps reporting) afterwards.  What it
        must not do is act — ``start`` and a ``continue`` answer are both
        gated on ``printer.is_shutdown()``.

        Runs inside ``reactor.assert_no_pause()`` (klippy/klippy.py:210), so:
        no waiting, no g-code, nothing that can pause.  An outstanding pause
        is answered ``abort`` from a detached thread — plrd applies the
        answer the moment it arrives (ctrlsock.rs:776), so nobody has to
        read the reply — which ends the recovery now instead of at plrd's
        deadline.
        """
        if self._closed:
            return
        token = self._token
        state = self._state
        self._answering = None
        self._token = None
        self._data = None
        self._disarm_timer()
        if token is not None:
            self._abort_shutdown_pause(token)
            self._end_dialog()
            # Tell the listener, or the wizard sits in its "running" state
            # for the rest of the klippy session reporting a recovery that
            # is over — the wedged-UI failure this branch exists to remove.
            self._notify_finished()
            return
        if state == STATE_RUNNING:
            # plrd is mid-execution on THIS session's channel and cannot be
            # interrupted from here; its own commands will start failing
            # against a shut-down printer and it will abort.  Say so, and
            # stay RUNNING so its report is still delivered and no second
            # recovery starts meanwhile.
            self._respond(
                "PLR recovery: klippy stopped while plrd was EXECUTING the "
                "recovery. plrd cannot be interrupted from here; its commands "
                "will start failing and it will abort by itself. Its report "
                "still appears here — wait for it, and do not touch the "
                "printer until it arrives."
            )
        elif state == STATE_PLRD_BUSY:
            # NOT the same case: no call of ours is in flight, so no report
            # is coming to this console.  Telling the operator to wait for
            # one would be telling them to wait forever.
            self._respond(
                "PLR recovery: klippy stopped while plrd was executing a "
                "recovery this plugin is not connected to. NO report for it "
                "will appear here — its commands will start failing against "
                "the shut-down printer and it will abort by itself. Watch "
                "'journalctl -u plrd', and do not touch the printer until it "
                "has stopped."
            )

    def _handle_disconnect(self):
        """``klippy:disconnect`` — real teardown (exit or restart).

        klippy/klippy.py:195 sends this as the run loop unwinds, so nothing
        may be delivered to the reactor again.  An outstanding question is
        still worth aborting for plrd's sake.
        """
        if self._closed:
            return
        self._closed = True
        token = self._token
        self._answering = None
        self._token = None
        self._data = None
        # Teardown: nothing this session publishes can be read again, and the
        # next klippy run builds a fresh session.  Calmer with a reason, so
        # the guard is satisfied by intent rather than by accident.
        self._transition(
            STATE_IDLE, reason="klippy is tearing down; this session is over"
        )
        self._disarm_timer()
        if token is not None:
            self._abort_detached(token, what="the outstanding question")
        self._notify_finished()

    def _abort_shutdown_pause(self, token):
        """Abort a question klippy can no longer answer, and say only that.

        Used from both shutdown paths (a pause outstanding AT the shutdown,
        and one ARRIVING after it).  The state goes to UNKNOWN and STAYS
        there: sending an abort is not the same as knowing it was applied,
        and if it was not, plrd is still paused and will run the aborting
        step's cleanup commands when its own deadline expires — into a klippy
        the operator has since restarted.  Nothing in the abort path can
        publish anything calmer (see :meth:`_abort_detached`).
        """
        self._raise_alarm(STATE_UNKNOWN)
        self._abort_detached(token, what="the confirmation klippy interrupted")
        self._respond(
            "PLR recovery: klippy stopped while plrd was waiting for a "
            "confirmation. An 'abort' for it has been SENT to plrd so it stops "
            "now rather than at its deadline — this line does NOT mean it was "
            "applied, and nothing here will claim it was: plrd only replies "
            "once it has finished aborting.\n"
            "Treat plrd as still working: DO NOT touch the printer, and check "
            "'journalctl -u plrd'. An abort invalidates the Z frame, so a fresh "
            "dry run is required before any resume."
        )

    def _abort_detached(self, token, what="the outstanding question", epoch=None):
        """Send ``recover_confirm ... abort`` from a detached thread.

        Deliberately NOT an :class:`~plr.daemon_worker.AsyncDaemon` call:
        this is used from ``klippy:shutdown`` (which must not block or pause)
        and from teardown, where the channel is closed.

        =====================================================================
        THIS PATH CAN NEVER PUBLISH A CALMER STATE.  THAT IS A RULE, NOT A
        CAREFUL READING — the careful reading is what failed.
        =====================================================================

        The previous attempt classified the reply and called a *success*
        transition.  It read six typed refusals — and no reply at all — as
        acceptance, and so retracted a do-not-touch warning the plugin had
        printed five seconds earlier.  The honest terminal state of a
        detached abort is UNKNOWN, because a confirmation genuinely cannot
        arrive inside any send window worth waiting for: ``recover_confirm``
        returns only after ``drive_session`` has let ``finish_abort`` push
        every cleanup command through Moonraker, so on the INTENDED SUCCESS
        path plrd is still working when the window closes.

        Structurally: :meth:`_abort_reported` is the only consumer, and it
        may use :meth:`_raise_alarm` — which cannot lower — but never
        :meth:`_transition`.  ``tests/test_recovery_confirm.py`` asserts
        that by AST, so the calming call cannot come back.

        The reply is still READ, and classified through :func:`classify`, the
        same single reading the confirm loop uses — because a positive
        refusal (``malformed``: plrd is still standing at the confirm point)
        is information worth showing.  It just never becomes acceptance.

        The outcome carries the session's epoch, so it cannot land on a later
        conversation (the precedent is
        :attr:`~plr.daemon_worker.AsyncDaemon._generation`).
        """
        link = self.plugin.daemon
        reactor = self.reactor
        # The conversation this abort belongs to.  An ABANDONED question
        # belongs to the previous one, so its caller passes that epoch.
        epoch = self._epoch if epoch is None else epoch

        def deliver(status, detail):
            try:
                reactor.register_async_callback(
                    lambda eventtime: self._abort_reported(epoch, status, detail, what)
                )
            except Exception:
                logger.info("plr: no reactor left to report the %s abort to", what)

        def run():
            try:
                response = link.call(
                    "recover_confirm",
                    {"token": token, "answer": ANSWER_ABORT},
                    timeout=daemon_link.ABORT_SEND_TIMEOUT,
                )
            except daemon_link.DaemonError as e:
                message = str(e)
                if "did not answer" in message or "did not finish answering" in message:
                    # THE EXPECTED CASE, and emphatically not success: plrd
                    # does not reply until it has finished aborting.  Logged at
                    # info, never as a traceback — the false failure this used
                    # to write into klippy.log was this branch.
                    logger.info(
                        "plr: %s abort was sent; plrd did not reply inside the "
                        "send window, which is the normal case and proves "
                        "nothing either way",
                        what,
                    )
                    deliver("no-reply", None)
                    return
                logger.warning("plr: %s abort may not have been sent: %s", what, e)
                deliver("send-failed", message)
                return
            except Exception as e:
                logger.exception("plr: %s abort failed", what)
                deliver("send-failed", "%s: %s" % (type(e).__name__, e))
                return
            deliver("answered", response)

        thread = threading.Thread(target=run, name="plr-abort")
        thread.daemon = True
        thread.start()

    def _abort_reported(self, epoch, status, detail, what):
        """The detached abort's outcome, on the reactor.  Never raises.

        Reports, and may only RAISE the alarm (see :meth:`_abort_detached`).
        Nothing here can make the published state calmer, whatever plrd
        said: "the abort was sent" and "the machine is free" are different
        claims, and only plrd finishing supports the second.
        """
        try:
            if self._closed:
                return
            # AN OUTCOME MAY NEVER LAND ON A LATER CONVERSATION.  The epoch
            # gates the STATE, not the report: what plrd said is still worth
            # printing — it may be "I am still paused at a confirm point" —
            # but it cannot move a state that now belongs to a different
            # request.  (The old gate was `if self._token is None`, which
            # tests for "no NEW question" rather than identity.)
            stale = epoch != self._epoch
            if stale:
                logger.info(
                    "plr: the %s abort outcome (%s) belongs to an earlier "
                    "conversation; reporting it without touching the state",
                    what,
                    status,
                )
            else:
                self._raise_alarm(STATE_UNKNOWN)
            prefix = (
                "PLR recovery (about an earlier conversation): "
                if stale
                else "PLR recovery: "
            )
            if status == "send-failed":
                self._respond(
                    "%sthe abort for %s could NOT be sent to plrd (%s).\n"
                    "Assume plrd is STILL PAUSED: it will run that step's "
                    "cleanup commands when its own deadline expires, which may "
                    "be after you clear the shutdown. DO NOT touch the printer; "
                    "check 'journalctl -u plrd' and stop plrd there if it must "
                    "not continue." % (prefix, what, detail)
                )
                return
            if status == "no-reply":
                self._respond(
                    "%sthe abort for %s was sent, and plrd did not reply inside "
                    "the send window. That is the NORMAL case — plrd only "
                    "replies once it has finished aborting, which includes "
                    "sending that step's cleanup commands — and it is NOT "
                    "confirmation that the abort was applied.\n"
                    "Treat plrd as still working: DO NOT touch the printer "
                    "until 'journalctl -u plrd' shows it idle." % (prefix, what)
                )
                return
            # plrd answered something.  Classified with the same reading the
            # confirm loop uses, and REPORTED — never converted into a claim
            # about the machine.
            kind, outcome = classify(detail)
            if kind == ANSWER_PAUSE:
                extra = (
                    "plrd is PAUSED at a confirm-point (it answered %r), so the "
                    "abort did not end the recovery." % (outcome,)
                )
            elif kind == ANSWER_BUSY:
                extra = "plrd answered 'busy', so it is still executing a recovery."
            elif kind == ANSWER_STALE_TOKEN:
                extra = (
                    "plrd answered 'unknown-token': it is not waiting for that "
                    "question any more, but it did not say what it is doing."
                )
            elif kind == ANSWER_TERMINAL:
                extra = (
                    "plrd reported the recovery as over (%r); its own report in "
                    "this console, not this line, is the thing to trust." % (outcome,)
                )
            else:
                extra = (
                    "plrd answered %r, which says nothing about whether it "
                    "applied the abort." % (outcome,)
                )
            self._respond(
                "%splrd replied to the abort for %s. %s\n"
                "This plugin does NOT treat any reply as confirmation that the "
                "abort was applied. DO NOT touch the printer until "
                "'journalctl -u plrd' shows plrd idle." % (prefix, what, extra)
            )
        except Exception:
            logger.exception("plr: cannot report the %s abort outcome", what)


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


# --- resume-preview reposition answers (design §F.2) ------------------
#
# These four are the preview's console floor: the dialog's Accept / Prev /
# Next / nudge buttons fire exactly these, the preview fallback lines name
# exactly these, and every one calls the same widened ``answer()`` so the
# shutdown rule, kind discrimination and token handling are shared with the
# binary answers.


def cmd_PLR_RECOVER_ACCEPT(plugin, gcmd):
    """PLR_RECOVER_ACCEPT — accept the previewed resume point."""
    plugin.recovery.answer(gcmd, ANSWER_ACCEPT, CMD_ACCEPT)


def cmd_PLR_RECOVER_NEXT(plugin, gcmd):
    """PLR_RECOVER_NEXT — step to the next representative preview stop."""
    plugin.recovery.answer(gcmd, ANSWER_NEXT, CMD_NEXT)


def cmd_PLR_RECOVER_PREV(plugin, gcmd):
    """PLR_RECOVER_PREV — step to the previous representative preview stop."""
    plugin.recovery.answer(gcmd, ANSWER_PREV, CMD_PREV)


def cmd_PLR_RECOVER_NUDGE(plugin, gcmd):
    """PLR_RECOVER_NUDGE FWD=<n> | BACK=<n> — nudge along the toolpath.

    Exactly one of FWD / BACK, each 1 (fine) or 10 (coarse), per the ruled
    two-size nudge.  The signed ``count`` (+n forward, -n back) is what plrd
    parses; ``answer()`` applies the shutdown rule and kind discrimination
    before it reaches the wire.
    """
    fwd = gcmd.get_int("FWD", None)
    back = gcmd.get_int("BACK", None)
    if (fwd is None) == (back is None):
        raise gcmd.error(
            "%s: give exactly one of FWD=<n> or BACK=<n> (n = 1 or 10)." % (CMD_NUDGE,)
        )
    magnitude = fwd if fwd is not None else back
    if magnitude not in (1, 10):
        raise gcmd.error(
            "%s: the nudge step must be 1 (fine) or 10 (coarse), got %d."
            % (CMD_NUDGE, magnitude)
        )
    count = magnitude if fwd is not None else -magnitude
    plugin.recovery.answer(gcmd, ANSWER_NUDGE, CMD_NUDGE, count=count)
