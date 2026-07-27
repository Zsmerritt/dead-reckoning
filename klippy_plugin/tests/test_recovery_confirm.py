"""The confirm-point loop: pause -> render -> answer -> continue.

THE POINT OF THE BRANCH.  plrd's confirm-point mechanism was fully built
and completely unreachable, because the plugin never asked for it: both
call sites sent ``recover_execute`` without ``on_confirm``, and omitting
it means every ``Tier::Confirmable`` diagnosis ABORTS the recovery
(crates/plrd/src/ctrlsock.rs:612-627).  These tests hold the whole chain
down end to end, through the registered console commands: a Confirmable
diagnosis reaches a RENDERED PROMPT, and the operator's "continue"
actually continues the same execution.

Every daemon response here comes from tests/confirm_fixtures.py, which
cites the Rust that produces each field.  Every command is dispatched the
way klippy dispatches it, and every daemon call really runs on a worker
thread and comes back through ``reactor.register_async_callback`` — the
``pump`` counts below are part of the assertion, because a handler that
blocked would have delivered its answer before returning.
"""

import threading
import time

import confirm_fixtures as fx
import fake_klippy
import pytest

from plr import daemon_link, recovery


class ScriptedDaemon:
    """Canned responses per command, in order, over the real link API.

    ``script`` maps command -> list of responses (or DaemonError instances
    to raise).  A command called more times than it has entries reuses the
    last one, which is what a real daemon does for ``busy``.
    """

    def __init__(self, script=None):
        self.script = {cmd: list(v) for cmd, v in (script or {}).items()}
        self.calls = []

    def call(self, cmd, args=None, timeout=None):
        self.calls.append((cmd, args, timeout))
        queue = self.script.get(cmd)
        if not queue:
            return {"ok": True, "text": "", "data": {}}
        response = queue.pop(0) if len(queue) > 1 else queue[0]
        if isinstance(response, Exception):
            raise response
        return response

    def answers(self):
        """The answer values sent to plrd, in order."""
        return [
            args.get("answer")
            for cmd, args, _t in self.calls
            if cmd == "recover_confirm" and isinstance(args, dict)
        ]

    def tokens(self):
        return [
            args.get("token")
            for cmd, args, _t in self.calls
            if cmd == "recover_confirm" and isinstance(args, dict)
        ]


@pytest.fixture
def run(run_cmd, pump):
    """Dispatch a command and deliver the worker result it hands off."""

    def go(name, calls=1, **params):
        gcode = run_cmd(name, **params)
        assert pump(calls, timeout=5.0) == calls
        return gcode

    return go


def responses_since(gcode):
    start = len(gcode.responses)
    return lambda: gcode.responses[start:]


def execute(plugin, run):
    """Start a recovery through PLR_RECOVER and deliver plrd's first reply."""
    return run("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")


# --- the whole loop ---------------------------------------------------


def test_a_confirmable_diagnosis_reaches_a_rendered_prompt_and_continue_continues(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause()],
            "recover_confirm": [fx.completed()],
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    lines = since()
    # plrd asked to be consulted...
    assert plugin.daemon.calls[0] == (
        "recover_execute",
        {"confirm": True, "on_confirm": "ask"},
        daemon_link.EXECUTE_TIMEOUT,
    )
    # ...the pause is a RENDERED PROMPT, not a log line...
    assert "action:prompt_begin Power-loss recovery — confirmation needed" in lines
    assert "action:prompt_button Continue anyway|PLR_RECOVER_CONTINUE|warning" in lines
    assert "action:prompt_footer_button Abort recovery|PLR_RECOVER_ABORT|error" in lines
    assert "action:prompt_show" in lines
    # ...carrying all three parts of the requirement...
    body = "\n".join(lines)
    assert "Why: " in body and "Suggested fix: " in body
    # ...and the daemon's own paused report is printed verbatim too.
    assert any("awaiting confirmation" in line for line in lines)
    assert plugin.recovery.state() == "awaiting_confirmation"
    assert plugin.recovery.can_answer() is True
    assert plugin.get_status(100.0)["recovery_awaiting_confirmation"] is True

    # The operator continues: the SAME execution resumes and completes.
    since = responses_since(gcode)
    run("PLR_RECOVER_CONTINUE")
    assert plugin.daemon.calls[1] == (
        "recover_confirm",
        {"token": "plrc-17bd4c0f9a2-3", "answer": "continue"},
        daemon_link.EXECUTE_TIMEOUT,
    )
    after = "\n".join(since())
    assert "plan complete" in after
    assert "PLR recovery complete" in after
    assert "action:prompt_end" in since()
    assert plugin.recovery.state() == "idle"


def test_abort_answers_abort_and_reports_the_daemons_own_abort(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.aborted()]}
    )
    gcode = fake_printer.lookup_object("gcode")
    execute(plugin, run)
    since = responses_since(gcode)
    run("PLR_RECOVER_ABORT")
    assert plugin.daemon.answers() == ["abort"]
    body = "\n".join(since())
    assert "confirmation-declined" in body
    assert "did not complete" in body
    assert plugin.recovery.state() == "idle"


def test_several_confirm_points_in_one_recovery_are_all_answered(
    plugin, run, fake_printer
):
    # debug_confirm_each_step pauses before EVERY step
    # (crates/plrd/src/executor.rs:684-692), so a real run of that mode is a
    # chain of pauses on ONE execution.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.step_debug_pause(token="t1", step=1)],
            "recover_confirm": [
                fx.step_debug_pause(token="t2", step=2),
                fx.step_debug_pause(token="t3", step=3),
                fx.completed(),
            ],
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    execute(plugin, run)
    steps_seen = []
    for _ in range(3):
        assert plugin.recovery.can_answer() is True
        since = responses_since(gcode)
        run("PLR_RECOVER_CONTINUE")
        steps_seen.append("\n".join(since()))
    assert plugin.daemon.tokens() == ["t1", "t2", "t3"]
    assert plugin.daemon.answers() == ["continue"] * 3
    assert "PLR recovery complete" in steps_seen[-1]
    assert plugin.recovery.state() == "idle"
    # Each pause named its own step, so the operator can tell them apart.
    assert "Paused at step 2" in steps_seen[0]
    assert "Paused at step 3" in steps_seen[1]


def test_a_confirm_point_can_be_the_very_first_response(plugin, run, fake_printer):
    # The pre-flight pause happens BEFORE step 1 (executor.rs:642-663), so
    # the FIRST thing recover_execute can return is a question.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause(step=1, phase="preamble")],
            "recover_confirm": [fx.completed()],
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    assert "action:prompt_show" in since()
    assert plugin.recovery.can_answer() is True
    run("PLR_RECOVER_CONTINUE")
    assert plugin.recovery.state() == "idle"


@pytest.mark.parametrize(
    "pause_response,kind_text",
    [
        pytest.param(fx.z_height_pause(), "look right", id="z-height"),
        pytest.param(fx.step_debug_pause(), "Run the next step?", id="step-debug"),
        pytest.param(fx.pause(), "Continue despite this?", id="diagnosis"),
    ],
)
def test_each_confirm_kind_asks_its_own_question_in_the_dialog(
    plugin, run, fake_printer, pause_response, kind_text
):
    plugin.daemon = ScriptedDaemon({"recover_execute": [pause_response]})
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    assert any(kind_text in line for line in since())


# --- single flight ----------------------------------------------------


def test_a_second_recovery_is_refused_from_either_entry_point(plugin, run, run_cmd):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    execute(plugin, run)
    # PLR_RECOVER again...
    with pytest.raises(fake_klippy.FakeCommandError, match="WAITING FOR YOUR ANSWER"):
        run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    # ...and the wizard's own execute, which shares the one session.
    plugin.wizard._state = "execute"
    with pytest.raises(fake_klippy.FakeCommandError, match="WAITING FOR YOUR ANSWER"):
        run_cmd("PLR_WIZARD_EXECUTE")
    # Neither attempt talked to plrd, and the question is still answerable.
    assert [c[0] for c in plugin.daemon.calls] == ["recover_execute"]
    assert plugin.recovery.can_answer() is True


def test_a_second_recovery_while_running_is_refused_and_names_m112(
    plugin, run_cmd, pump
):
    # A running (not paused) recovery cannot be interrupted from the
    # console; the message must not pretend otherwise.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.completed()]})
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert plugin.recovery.state() == "running"
    with pytest.raises(fake_klippy.FakeCommandError, match="M112") as excinfo:
        run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert "already in flight" in str(excinfo.value)
    assert pump() == 1


def test_an_unrelated_command_mid_flow_does_not_lose_the_token(plugin, run, run_cmd):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.completed()]}
    )
    execute(plugin, run)
    # Anything an operator might type while thinking about the question.
    run_cmd("PLR_SET")
    run_cmd("PLR_WIZARD_CLOSE")
    run_cmd("PLR_SETUP_WIZARD")
    assert plugin.recovery.can_answer() is True
    run("PLR_RECOVER_CONTINUE")
    assert plugin.daemon.tokens() == ["plrc-17bd4c0f9a2-3"]
    assert plugin.recovery.state() == "idle"


def test_answering_twice_cannot_answer_twice(plugin, run, run_cmd, pump):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.completed()]}
    )
    execute(plugin, run)
    run_cmd("PLR_RECOVER_CONTINUE")
    # The token is gone the moment it is handed to plrd, so a double click
    # cannot send a second answer.
    with pytest.raises(fake_klippy.FakeCommandError, match="not asking anything"):
        run_cmd("PLR_RECOVER_CONTINUE")
    assert pump() == 1
    assert plugin.daemon.answers() == ["continue"]


def test_answering_when_nothing_is_outstanding_is_a_clear_refusal(plugin, run_cmd):
    plugin.daemon = ScriptedDaemon()
    with pytest.raises(fake_klippy.FakeCommandError, match="no recovery confirmation"):
        run_cmd("PLR_RECOVER_CONTINUE")
    with pytest.raises(fake_klippy.FakeCommandError, match="no recovery confirmation"):
        run_cmd("PLR_RECOVER_ABORT")
    assert plugin.daemon.calls == []


def test_answer_rejects_an_answer_plrd_would_not_accept(plugin):
    # A guard against a future caller inventing a third answer: plrd only
    # accepts "continue" / "abort" (ctrlsock.rs:742-751).
    gcmd = fake_klippy.FakeGCodeCommand(
        plugin.printer.lookup_object("gcode"), "X", "X", {}
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="unknown answer"):
        plugin.recovery.answer(gcmd, "maybe", "TEST")


# --- what plrd can answer instead -------------------------------------


def test_a_pause_with_no_usable_token_is_reported_as_unanswerable(
    plugin, run, fake_printer
):
    # FAIL-SAFE: without a token there is no way to answer, and plrd is
    # STILL PAUSED holding the machine.  So: not answerable, not "aborted",
    # and NOT reported as idle.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause(token=None)]})
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    body = "\n".join(since())
    assert "no usable resume token" in body
    assert "still paused" in body
    assert "DO NOT touch the printer" in body
    assert "action:prompt_end" in since()
    assert plugin.recovery.state() == "unknown"
    assert plugin.recovery.can_answer() is False
    # ...and a retry is permitted, because plrd's `busy` is the only
    # observation available.
    assert plugin.recovery.may_start_new() is True


@pytest.mark.parametrize("token", ["", 7, [], {}])
def test_a_malformed_token_is_treated_the_same_way(plugin, run, fake_printer, token):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause(token=token)]})
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    assert "no usable resume token" in "\n".join(since())


def test_the_timed_out_unknown_token_says_plrd_is_still_aborting(
    plugin, run, fake_printer
):
    # plrd's confirm deadline expired between the question and the answer,
    # so recover_confirm answers `unknown-token` (ctrlsock.rs:776-784).
    # That IS the outcome, and the operator must be told the recovery is
    # over — not shown a transport error.
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.unknown_token()]}
    )
    gcode = fake_printer.lookup_object("gcode")
    execute(plugin, run)
    since = responses_since(gcode)
    run("PLR_RECOVER_CONTINUE")
    body = "\n".join(since())
    assert "no longer waiting for that answer" in body
    assert "ABORTING the recovery now" in body
    assert "invalidates the Z frame" in body
    # NOT idle: plrd is inside finish_abort, pushing that step's cleanup
    # commands through Moonraker, so the machine is not yet still — and the
    # old advice to re-run the wizard would have answered `busy`.
    assert plugin.recovery.state() == "unknown"
    assert "still sending that step's cleanup commands" in body
    assert "Wait for plrd to go quiet" in body


def test_busy_is_treated_as_PROOF_that_plrd_is_executing(plugin, run, fake_printer):
    # THE BLOCKER.  ctrlsock.rs:631-645 answers `busy` exactly when the
    # execution task is NOT finished, so it is positive evidence that plrd
    # holds the machine.  Publishing `idle` in reply to that — which is what
    # the generic terminal path did — is the failure that gets somebody's
    # hand into an enclosure, and it landed on the very command the plugin
    # tells the operator to run as a probe.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.busy()]})
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    body = "\n".join(since())
    assert plugin.recovery.state() == "plrd_busy"
    assert "IS executing a recovery" in body
    assert "DO NOT touch the printer" in body
    assert "M112" in body
    # The console agrees, because it reads the same authority.
    status = "\n".join(plugin.recovery.status_lines())
    assert "plrd IS EXECUTING A RECOVERY" in status
    assert "DO NOT touch the printer" in status
    assert plugin.get_status(100.0)["recovery_state"] == "plrd_busy"
    # Something must still be attended to...
    assert plugin.recovery.needs_attention() is True
    # ...but the probe stays available, because plrd is the only thing that
    # can tell us when it has finished.
    assert plugin.recovery.may_start_new() is True


def test_the_probe_the_plugin_advertises_never_publishes_idle(plugin, run):
    # The cruel path, end to end: UNKNOWN -> "run the probe" -> busy.  At no
    # point may the console print a bare "recovery: idle".
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [
                {"ok": True, "text": "??", "data": None},  # -> UNKNOWN
                fx.busy(),  # the probe's answer
            ]
        }
    )
    execute(plugin, run)
    assert plugin.recovery.state() == "unknown"
    advice = "\n".join(plugin.recovery.status_lines())
    assert "PLR_RECOVER EXECUTE=1 CONFIRM=YES" in advice
    # Do exactly what the plugin just told the operator to do.
    execute(plugin, run)
    assert plugin.recovery.state() == "plrd_busy"
    assert "idle" not in "\n".join(plugin.recovery.status_lines())
    assert plugin.get_status(100.0)["recovery_state"] != "idle"


def test_a_response_with_no_data_map_is_not_classified_as_success(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [{"ok": True, "text": "??", "data": None}]}
    )
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    body = "\n".join(since())
    assert "cannot classify" in body
    assert "PLR recovery complete" not in body
    # Unclassifiable says nothing about whether plrd is still executing, so
    # it is UNKNOWN — never idle.
    assert plugin.recovery.state() == "unknown"


@pytest.mark.parametrize(
    "text,expected_state,claims_abort",
    [
        # ctrlsock.rs:776-784 — the ONE case that means the recovery ended.
        pytest.param(
            "the confirmation timed out before the answer arrived; the "
            "recovery aborted",
            "unknown",
            True,
            id="timed-out-aborting",
        ),
        # :755-760 — no execution at all: plrd is not saying what happened.
        pytest.param(
            "no execution is awaiting confirmation; the token is unknown or expired",
            "unknown",
            False,
            id="no-session",
        ),
        # :761-766 — plrd is EXECUTING.  Claiming an abort here is the
        # "believing a recovery aborted while it is still running" failure.
        pytest.param(
            "the execution is running but not awaiting confirmation; the "
            "token is expired",
            "unknown",
            False,
            id="still-running",
        ),
        # :767-775 — the question was PUT BACK, so plrd is still paused with
        # the nozzle at standoff and the heaters at target.
        pytest.param(
            "that token does not match the outstanding confirmation",
            "unknown",
            False,
            id="still-paused",
        ),
        pytest.param("", "unknown", False, id="no-text-at-all"),
    ],
)
def test_unknown_token_tells_the_truth_for_each_of_its_four_causes(
    plugin, run, fake_printer, text, expected_state, claims_abort
):
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause()],
            "recover_confirm": [fx.unknown_token(text=text)],
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    execute(plugin, run)
    since = responses_since(gcode)
    run("PLR_RECOVER_CONTINUE")
    body = "\n".join(since())
    assert plugin.recovery.state() == expected_state
    if claims_abort:
        assert "ABORTING the recovery now" in body
    else:
        assert "aborted" not in body
        assert "may still be executing, or still paused" in body
        assert "DO NOT touch the printer" in body
        assert "safe as a probe" in body


def test_an_empty_report_still_says_something(plugin, run, fake_printer):
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [
                {"ok": True, "text": "", "data": {"outcome": "completed"}}
            ]
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    assert "plrd returned an empty recovery report" in since()


def test_a_refusal_says_nothing_was_sent(plugin, run, fake_printer):
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [
                {
                    "ok": False,
                    "text": "recover: REFUSED — printer is printing",
                    "data": {"outcome": "refused"},
                }
            ]
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    assert "nothing was" in "\n".join(since())


# --- transport failure ------------------------------------------------


def test_losing_contact_never_claims_the_recovery_failed(plugin, run, fake_printer):
    # A transport failure says nothing about what plrd is doing, so the
    # state must be UNKNOWN rather than idle (checked below).
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [
                daemon_link.DaemonError("plrd did not answer 'recover_execute'")
            ]
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    body = "\n".join(since())
    assert "lost contact with plrd" in body
    assert "may still be executing" in body
    assert "M112" in body
    # It also does not claim success, and it releases the local guard so the
    # DAEMON's own busy refusal (ctrlsock.rs:631-645) is the authority on a
    # second attempt — while still not reporting "idle".
    assert "complete" not in body
    assert plugin.recovery.state() == "unknown"
    assert plugin.recovery.may_start_new() is True


def test_losing_contact_while_a_question_is_open_says_the_dialog_is_dead(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause()],
            "recover_confirm": [daemon_link.DaemonError("plrd closed the connection")],
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    execute(plugin, run)
    since = responses_since(gcode)
    run("PLR_RECOVER_CONTINUE")
    body = "\n".join(since())
    assert "may or may not have reached plrd" in body
    assert "can no longer be answered from here" in body
    assert "action:prompt_end" in since()
    assert plugin.recovery.state() == "unknown"


def test_an_exception_in_a_reactor_callback_never_escapes(
    plugin, run_cmd, pump, monkeypatch
):
    # klippy turns an exception out of a reactor callback into a printer
    # shutdown (klippy/klippy.py:170-186), so the delivery wrapper has to
    # swallow one.  Break the renderer to prove the wrapper, not the
    # renderer, is what holds.
    def boom(*args, **kwargs):
        raise RuntimeError("renderer exploded")

    monkeypatch.setattr(recovery.confirm_ui, "confirm_prompt", boom)
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    # The pump would propagate anything the wrapper let through.
    assert pump() == 1
    gcode = plugin.printer.lookup_object("gcode")
    assert any("internal error" in line for line in gcode.responses)


# --- klippy lifecycle -------------------------------------------------


def test_a_shutdown_at_a_confirm_point_aborts_the_recovery_and_clears_the_dialog(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.aborted()]}
    )
    gcode = fake_printer.lookup_object("gcode")
    execute(plugin, run)
    since = responses_since(gcode)
    # M112 / an MCU shutdown at a confirm point.
    fake_printer.invoke_shutdown("Manual stop (M112)")
    # The abort is sent to plrd from a detached thread; wait for it.
    for _ in range(400):
        if plugin.daemon.answers():
            break
        time.sleep(0.005)
    assert plugin.daemon.answers() == ["abort"]
    body = "\n".join(since())
    assert "klippy stopped while plrd was waiting" in body
    # SENT is not APPLIED, and no reply can make it so: recover_confirm only
    # returns once plrd has FINISHED aborting, so the send window closing is
    # the normal case on the success path.
    assert "has been SENT" in body
    assert "does NOT mean it was applied" in body
    assert "action:prompt_end" in since()
    assert plugin.recovery.state() == "unknown"
    assert plugin.recovery.can_answer() is False
    # plrd's reply is READ and reported — and changes nothing about the
    # published state.
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    after = "\n".join(since())
    assert "plrd replied to the abort" in after
    assert "does NOT treat any reply as confirmation" in after
    assert "accepted the abort" not in after
    assert plugin.recovery.state() == "unknown"


def test_a_shutdown_during_a_running_recovery_says_so_and_still_reports(
    plugin, run_cmd, fake_printer
):
    # A shutdown cannot interrupt plrd, so the operator is told what will
    # actually happen — and plrd's report is still delivered, because a
    # shutdown is not teardown (klippy stays up until FIRMWARE_RESTART).
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.aborted()]})
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    fake_printer.invoke_shutdown("Manual stop (M112)")
    body = "\n".join(since())
    assert "klippy stopped while plrd was EXECUTING" in body
    assert "do not touch the printer" in body
    assert plugin.daemon.answers() == []
    # Still RUNNING: a second recovery must not start while plrd finishes.
    assert plugin.recovery.state() == "running"
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert any("ABORTED" in line for line in gcode.responses)
    assert plugin.recovery.state() == "idle"


def test_a_shutdown_leaves_the_query_paths_working(plugin, run_cmd, pump, fake_printer):
    # REGRESSION GUARD: before this fix, a shutdown closed every channel and
    # PLR_STATUS promised a daemon block that never arrived — at exactly the
    # moment the operator has hit M112 and most needs to know what plrd
    # thinks it is doing.
    plugin.daemon = ScriptedDaemon(
        {"status": [{"ok": True, "text": "plrd armed", "data": {}}]}
    )
    fake_printer.invoke_shutdown("Manual stop (M112)")
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    run_cmd("PLR_STATUS")
    assert pump() == 1
    body = "\n".join(since())
    assert "plrd armed" in body
    assert "already in flight" not in body


def test_a_disconnect_is_terminal_for_the_channels(plugin, run_cmd, fake_printer):
    plugin.daemon = ScriptedDaemon(
        {"status": [{"ok": True, "text": "plrd armed", "data": {}}]}
    )
    fake_printer.send_event("klippy:disconnect")
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    run_cmd("PLR_STATUS")
    body = "\n".join(since())
    # No fabricated in-flight query: it says what is actually happening.
    assert "shutting down" in body
    assert "already in flight" not in body
    assert fake_printer.reactor.pending_async() == 0


def test_no_recovery_can_be_started_after_a_shutdown(plugin, run_cmd, fake_printer):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.completed()]})
    fake_printer.invoke_shutdown("Manual stop (M112)")
    with pytest.raises(fake_klippy.FakeCommandError, match="klippy is shut down"):
        run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert plugin.daemon.calls == []


def test_continue_is_refused_after_a_shutdown_but_the_state_is_already_over(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    execute(plugin, run)
    fake_printer.in_shutdown_state = True  # shutdown without running handlers
    with pytest.raises(fake_klippy.FakeCommandError, match="cannot continue"):
        run_cmd_continue(plugin)


def run_cmd_continue(plugin):
    gcode = plugin.printer.lookup_object("gcode")
    gcmd = fake_klippy.FakeGCodeCommand(gcode, "PLR_RECOVER_CONTINUE", "X", {})
    gcode.commands["PLR_RECOVER_CONTINUE"](gcmd)


def test_a_disconnect_mid_recovery_leaves_no_dialog_and_no_state(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    execute(plugin, run)
    fake_printer.send_event("klippy:disconnect")
    assert plugin.recovery.state() == "idle"
    assert plugin.recovery.can_answer() is False


# --- PLR_STATUS reflects the flow -------------------------------------


def test_status_shows_the_outstanding_question_and_how_to_answer_it(
    plugin, run, fake_printer, pump
):
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.z_height_pause()],
            "status": [{"ok": True, "text": "plrd armed", "data": {}}],
        }
    )
    execute(plugin, run)
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    run("PLR_STATUS")
    body = "\n".join(since())
    assert "recovery: AWAITING CONFIRMATION — question 1 of this recovery" in body
    assert "Paused at step 9 [z-confirm-standoff] (z-height confirmation)" in body
    assert "PLR_RECOVER_CONTINUE" in body and "PLR_RECOVER_ABORT" in body


def test_status_names_the_running_state_without_touching_the_socket(
    plugin, run_cmd, pump
):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.completed()]})
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    lines = plugin.recovery.status_lines()
    assert any("RUNNING (started by PLR_RECOVER)" in line for line in lines)
    assert pump() == 1


# --- the wizard's cancel path ----------------------------------------


def test_wizard_cancel_at_a_confirm_point_answers_abort(plugin, run, run_cmd, pump):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.aborted()]}
    )
    execute(plugin, run)
    run_cmd("PLR_WIZARD_CANCEL")
    assert plugin.daemon.answers() == ["abort"]
    assert pump() == 1
    assert plugin.recovery.state() == "idle"


def test_wizard_cancel_while_running_refuses_rather_than_lying(plugin, run_cmd, pump):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.completed()]})
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    with pytest.raises(fake_klippy.FakeCommandError, match="cannot be cancelled"):
        run_cmd("PLR_WIZARD_CANCEL")
    assert pump() == 1


def test_wizard_start_during_a_recovery_reports_it_instead_of_re_offering(
    plugin, run, run_cmd
):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    execute(plugin, run)
    gcode = plugin.printer.lookup_object("gcode")
    since = responses_since(gcode)
    run_cmd("PLR_WIZARD_START")
    lines = since()
    body = "\n".join(lines)
    assert "a recovery confirmation is still open" in body
    assert "AWAITING CONFIRMATION" in body
    # The question is put back on screen rather than buried under a fresh
    # "Attempt recovery" dialog (reshow()'s whole purpose).
    assert "action:prompt_show" in lines
    assert any("PLR_RECOVER_CONTINUE" in line for line in lines)
    # It did NOT ask plrd for status again.
    assert [c[0] for c in plugin.daemon.calls] == ["recover_execute"]


# --- the local prompt deadline (the interlock, in situ) ---------------
#
# tests/test_recovery_deadlines.py pins the NUMBERS against the daemon's own
# constants.  These pin the BEHAVIOUR: the timer plrd's deadline has to beat.


def confirm_timeout_plugin(fake_printer, plr_config, seconds):
    """A plugin whose printer.cfg sets [plr] confirm_timeout_s."""
    import plr as plr_pkg

    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    return plr_pkg.load_config(plr_config(options={"confirm_timeout_s": seconds}))


def test_the_local_deadline_is_armed_at_the_pause_and_outlasts_the_daemons(
    fake_printer, plr_config, run_cmd, pump
):
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "120")
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    armed_at = reactor.now
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    (timer,) = reactor.timers
    # plrd's own deadline for this printer is the configured 120 s; the
    # plugin's must come strictly later, so plrd's clean abort always wins.
    daemon_deadline, exact = recovery.daemon_confirm_deadline(120.0)
    assert exact is True
    assert timer.waketime > armed_at + daemon_deadline
    assert timer.waketime == armed_at + 120.0 + recovery.CONFIRM_HEADROOM_S


def test_the_local_deadline_does_not_fire_at_the_daemons_deadline(
    fake_printer, plr_config, run_cmd, pump
):
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "120")
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    # Sit exactly on plrd's deadline: the plugin must still believe the
    # question is live, because plrd may be about to abort it and that abort
    # is the outcome the operator should see.
    reactor.advance(120.0)
    assert reactor.run_due_timers() == 0
    assert plugin.recovery.can_answer() is True
    # ...and one headroom later it gives up.
    reactor.advance(recovery.CONFIRM_HEADROOM_S)
    assert reactor.run_due_timers() == 1
    assert plugin.recovery.state() == "idle"


def test_the_local_deadline_reports_the_daemons_abort_and_answers_nothing(
    fake_printer, plr_config, run_cmd, pump
):
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "30")
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    gcode = fake_printer.lookup_object("gcode")
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    since = responses_since(gcode)
    reactor.advance(30.0 + recovery.CONFIRM_HEADROOM_S)
    assert reactor.run_due_timers() == 1
    body = "\n".join(since())
    assert "past plrd's own deadline" in body
    assert "aborted the recovery by now" in body
    assert "Nothing was answered on your behalf" in body
    assert "action:prompt_end" in since()
    # It answered NOTHING: inventing an answer for an absent operator is the
    # one thing a timeout must never do.
    assert plugin.daemon.answers() == []
    assert plugin.recovery.state() == "idle"


def test_answering_disarms_the_local_deadline(fake_printer, plr_config, run_cmd, pump):
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "30")
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.completed()]}
    )
    reactor = fake_printer.reactor
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    run_cmd("PLR_RECOVER_CONTINUE")
    assert pump() == 1
    (timer,) = reactor.timers
    assert timer.waketime == reactor.NEVER
    # A stale timer must not fire against the next recovery either.
    reactor.advance(10000.0)
    assert reactor.run_due_timers() == 0


def test_the_deadline_is_re_armed_for_each_pause(
    fake_printer, plr_config, run_cmd, pump
):
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "30")
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.step_debug_pause(token="t1", step=1)],
            "recover_confirm": [
                fx.step_debug_pause(token="t2", step=2),
                fx.completed(),
            ],
        }
    )
    reactor = fake_printer.reactor
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    first = reactor.timers[0].waketime
    reactor.advance(5.0)
    run_cmd("PLR_RECOVER_CONTINUE")
    assert pump() == 1
    second = reactor.timers[0].waketime
    assert second == first + 5.0
    assert plugin.recovery.can_answer() is True


def test_a_shutdown_disarms_the_local_deadline(fake_printer, plr_config, run_cmd, pump):
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "30")
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.aborted()]}
    )
    reactor = fake_printer.reactor
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    fake_printer.invoke_shutdown("Manual stop (M112)")
    reactor.advance(10000.0)
    assert reactor.run_due_timers() == 0
    # Unknown, and it stays unknown: nothing plrd can say inside the send
    # window proves the abort was applied.
    assert plugin.recovery.state() == "unknown"
    assert reactor.pump_async(1, timeout=2.0) == 1
    assert plugin.recovery.state() == "unknown"


def test_an_exception_in_the_expiry_handler_never_escapes(
    fake_printer, plr_config, run_cmd, pump, monkeypatch
):
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "30")
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1

    def boom(_msg):
        raise RuntimeError("respond exploded")

    monkeypatch.setattr(plugin.recovery, "_respond", boom)
    reactor.advance(10000.0)
    # A raise here would reach klippy's reactor loop, which turns it into a
    # printer shutdown (klippy/klippy.py:170-186).
    assert reactor.run_due_timers() == 1
    assert reactor.timers[0].waketime == reactor.NEVER


def test_a_broken_completion_listener_cannot_take_klippy_down(
    plugin, run, run_cmd, pump
):
    # The wizard registers a listener so it can drop back to idle.  A
    # listener that raises must not become a printer shutdown by way of the
    # reactor callback that called it (klippy/klippy.py:170-186).
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.completed()]})

    def boom():
        raise RuntimeError("listener exploded")

    gcmd = fake_klippy.FakeGCodeCommand(
        plugin.printer.lookup_object("gcode"), "X", "X", {}
    )
    plugin.recovery.start(gcmd, "TEST", on_finished=boom)
    assert pump() == 1
    assert plugin.recovery.state() == "idle"


def test_shutdown_then_disconnect_is_handled_once(plugin, run, fake_printer):
    # klippy sends klippy:disconnect on every exit as well as shutting down
    # (klippy/klippy.py:186-198), so both handlers fire in one teardown.
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.aborted()]}
    )
    execute(plugin, run)
    fake_printer.invoke_shutdown("Manual stop (M112)")
    fake_printer.send_event("klippy:disconnect")
    for _ in range(400):
        if plugin.daemon.answers():
            break
        time.sleep(0.005)
    # Exactly one abort, not two.
    assert plugin.daemon.answers() == ["abort"]


def test_an_abort_that_could_not_be_delivered_is_reported_and_stays_unknown(
    plugin, run, fake_printer
):
    # THE HAZARD: M112 at a confirm point, the abort fails because plrd is
    # unreachable, the console says "sent" and publishes idle, the operator
    # clears the shutdown — and plrd, still paused, reaches its own deadline
    # and runs the aborting step's cleanup through Moonraker into a now-live
    # klippy.  The machine moves after the operator was told it was over.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause()],
            "recover_confirm": [daemon_link.DaemonError("plrd not reachable at /x")],
        }
    )
    execute(plugin, run)
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    fake_printer.invoke_shutdown("Manual stop (M112)")
    for _ in range(400):
        if plugin.daemon.tokens():
            break
        time.sleep(0.005)
    assert plugin.daemon.tokens() == ["plrc-17bd4c0f9a2-3"]
    # The failure is marshalled back to the reactor and reported.
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    body = "\n".join(since())
    assert "could NOT be sent to plrd" in body
    assert "Assume plrd is STILL PAUSED" in body
    assert "DO NOT touch the printer" in body
    # And the state never got calmer on the strength of a send.
    assert plugin.recovery.state() == "unknown"


def test_a_shutdown_mid_recovery_does_not_leave_the_wizard_wedged(
    plugin, run_cmd, pump, fake_printer
):
    # klippy stays UP in the shutdown state until FIRMWARE_RESTART, so the
    # plugin object survives.  If the session forgot to tell its listener,
    # the wizard would report "a recovery is already in flight" for the rest
    # of the session and PLR_WIZARD_START would never work again.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    # Walk the wizard to its execute prompt, then execute through it.
    plugin.daemon.script["status"] = [
        {
            "ok": True,
            "text": "",
            "data": {"pending": {"file": "/x/bench.gcode", "percent": 10.0}},
        }
    ]
    plugin.daemon.script["recover_dryrun"] = [
        {"ok": True, "text": "PLAN", "data": {"outcome": "plan"}}
    ]
    run_cmd("PLR_WIZARD_START")
    assert pump() == 1
    run_cmd("PLR_WIZARD_DRYRUN")
    assert pump() == 1
    run_cmd("PLR_WIZARD_CONFIRM_CLEAN")
    run_cmd("PLR_WIZARD_EXECUTE")
    assert pump() == 1
    assert plugin.wizard.state() == "running"
    assert plugin.recovery.can_answer() is True

    fake_printer.invoke_shutdown("Manual stop (M112)")
    # The wizard must not stay wedged in "running" for the rest of the klippy
    # session — but the SESSION stays unknown until plrd confirms the abort.
    assert plugin.recovery.state() == "unknown"
    assert plugin.wizard.is_active() is False
    assert plugin.get_status(100.0)["wizard_active"] is False


# --- the liveness downgrade (MAJOR 5) ---------------------------------


def test_after_plrds_default_deadline_the_plugin_stops_asserting_liveness(
    plugin, run, fake_printer
):
    # The default install sets no confirm_timeout_s, so the plugin waits the
    # ceiling (3630 s) but plrd's real deadline is 600 s.  For the 50 minutes
    # in between it must NOT keep telling everyone a question is live.
    assert plugin.daemon_keys.get("confirm_timeout_s") is None
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    gcode = fake_printer.lookup_object("gcode")
    execute(plugin, run)
    assert plugin.recovery.state() == "awaiting_confirmation"
    assert plugin.get_status(100.0)["recovery_awaiting_confirmation"] is True

    since = responses_since(gcode)
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    assert reactor.run_due_timers() >= 1
    body = "\n".join(since())
    assert "longer than plrd's own default deadline" in body
    assert "may well have aborted" in body
    assert "Nothing was answered on your behalf" in body
    assert "action:prompt_end" in since()

    # What it now reports: unknown, not awaiting.
    assert plugin.recovery.state() == "unknown"
    status = plugin.get_status(100.0)
    assert status["recovery_state"] == "unknown"
    assert status["recovery_awaiting_confirmation"] is False
    # A fresh recovery is no longer refused on the strength of the guess...
    assert plugin.recovery.may_start_new() is True
    # ...but the answer commands still work, because plrd's reply is the
    # only thing that can actually settle it.
    assert plugin.recovery.can_answer() is True


def test_a_late_answer_is_still_attempted_after_the_downgrade(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.completed()]}
    )
    reactor = fake_printer.reactor
    execute(plugin, run)
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    reactor.run_due_timers()
    run("PLR_RECOVER_CONTINUE")
    # The token was kept: plrd got the answer and adjudicated.
    assert plugin.daemon.answers() == ["continue"]
    assert plugin.recovery.state() == "idle"


def test_a_fresh_recovery_after_the_downgrade_abandons_the_question_out_loud(
    plugin, run, run_cmd, pump
):
    # The contradiction the review found: the downgrade promises CONTINUE
    # still works, and starting a new recovery destroys that promise.  So
    # starting says so AND tells plrd to drop the question, rather than
    # leaving it paused against its own deadline.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause(), fx.busy()]})
    reactor = plugin.printer.get_reactor()
    execute(plugin, run)
    with pytest.raises(fake_klippy.FakeCommandError, match="WAITING FOR YOUR ANSWER"):
        run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    reactor.run_due_timers()
    assert plugin.recovery.can_answer() is True
    gcode = plugin.printer.lookup_object("gcode")
    since = responses_since(gcode)
    # Now permitted — and plrd itself adjudicates, answering `busy` if it is
    # in fact still paused.
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    body = "\n".join(since())
    assert "abandoning the confirmation that was still open" in body
    # SENT, and no reply can confirm it was applied.
    assert "has been SENT to plrd" in body
    assert "cannot confirm it was applied" in body
    assert plugin.recovery.can_answer() is False
    for _ in range(400):
        if "recover_confirm" in [c[0] for c in plugin.daemon.calls]:
            break
        time.sleep(0.005)
    assert plugin.daemon.answers() == ["abort"]
    # Two callbacks now: the new execute's answer, and the abandoned
    # question's abort reporting whether it actually landed.
    assert pump(2) == 2
    # Order is not asserted: the abandoned question's abort runs on its own
    # detached thread and races the new execute, deliberately — neither waits
    # for the other.
    assert sorted(c[0] for c in plugin.daemon.calls) == [
        "recover_confirm",
        "recover_execute",
        "recover_execute",
    ]
    joined = "\n".join(plugin.printer.lookup_object("gcode").responses)
    assert "plrd replied to the abort for the abandoned question" in joined
    assert "accepted the abort" not in joined


def test_a_known_deadline_asserts_liveness_right_up_to_it(
    fake_printer, plr_config, run_cmd, pump
):
    # With confirm_timeout_s set there is nothing to be doubtful about, so no
    # downgrade fires before the wait expires.
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "120")
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    reactor.advance(119.0)
    assert reactor.run_due_timers() == 0
    assert plugin.recovery.state() == "awaiting_confirmation"
    assert plugin.get_status(100.0)["recovery_awaiting_confirmation"] is True


def test_the_dialog_says_the_question_may_be_stale_when_it_is_re_shown(
    plugin, run, fake_printer
):
    # A re-shown doubtful question must not repeat the confident deadline
    # sentence: by then the honest statement is "plrd has probably aborted
    # this already".
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause()],
            "status": [{"ok": True, "text": "plrd armed", "data": {}}],
        }
    )
    reactor = fake_printer.reactor
    execute(plugin, run)
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    reactor.run_due_timers()
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    run("PLR_STATUS")
    body = "\n".join(since())
    assert "past plrd's own default deadline" in body
    assert "probably aborted the recovery already" in body
    # ...and PLR_WIZARD_START now offers a FRESH recovery rather than
    # re-showing a question it cannot vouch for; plrd adjudicates.
    assert plugin.recovery.may_start_new() is True


# --- re-showing the question (MINOR) ----------------------------------


def test_plr_status_re_shows_the_outstanding_question(plugin, run, run_cmd, pump):
    # On a console-first client the pause scrolls away, and the
    # why/fix/offer message is otherwise emitted exactly once.  An operator
    # who cannot see it again is an operator clicking Continue blind.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.z_height_pause()],
            "status": [{"ok": True, "text": "plrd armed", "data": {}}],
        }
    )
    execute(plugin, run)
    gcode = plugin.printer.lookup_object("gcode")
    since = responses_since(gcode)
    run("PLR_STATUS")
    lines = since()
    body = "\n".join(lines)
    assert "The outstanding recovery question, again:" in lines
    # The whole three-part message, not just a pointer to it.
    assert "action:prompt_begin Power-loss recovery — confirmation needed" in lines
    assert "Why: " in body
    assert "Suggested fix: " in body
    assert "action:prompt_button Continue anyway|PLR_RECOVER_CONTINUE|warning" in lines
    assert "Z was derived as: " in body


def test_wizard_start_re_shows_the_outstanding_question(plugin, run, run_cmd):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    execute(plugin, run)
    gcode = plugin.printer.lookup_object("gcode")
    since = responses_since(gcode)
    run_cmd("PLR_WIZARD_START")
    lines = since()
    assert "action:prompt_show" in lines
    assert any("PLR_RECOVER_CONTINUE" in line for line in lines)


def test_plr_status_re_shows_nothing_when_nothing_is_outstanding(plugin, run_cmd, pump):
    plugin.daemon = ScriptedDaemon(
        {"status": [{"ok": True, "text": "plrd armed", "data": {}}]}
    )
    gcode = plugin.printer.lookup_object("gcode")
    since = responses_since(gcode)
    run_cmd("PLR_STATUS")
    assert pump() == 1
    assert not any(line.startswith("action:") for line in since())
    assert plugin.recovery.reshow() is False


def test_status_lines_for_the_unknown_state_tell_the_operator_what_to_do(
    plugin, run, fake_printer
):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [{"ok": True, "text": "??", "data": None}]}
    )
    execute(plugin, run)
    assert plugin.recovery.state() == "unknown"
    lines = plugin.recovery.status_lines()
    body = "\n".join(lines)
    assert "recovery: UNKNOWN" in body
    assert "DO NOT touch the printer" in body
    assert "journalctl -u plrd" in body
    assert "safe as a probe" in body
    # PLR_STATUS shows the same thing.
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    run("PLR_STATUS")
    assert "recovery: UNKNOWN" in "\n".join(since())


def test_an_exception_in_the_claim_downgrade_never_escapes(
    plugin, run, fake_printer, monkeypatch
):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    execute(plugin, run)

    def boom(_msg):
        raise RuntimeError("respond exploded")

    monkeypatch.setattr(plugin.recovery, "_respond", boom)
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    # A raise here would reach klippy's reactor loop, which turns it into a
    # printer shutdown (klippy/klippy.py:170-186).
    assert reactor.run_due_timers() >= 1


# --- ONE state authority: the console and the API cannot disagree ------


def _console_state(plugin):
    """Which state the CONSOLE is asserting, read back from its own text."""
    body = "\n".join(plugin.recovery.status_lines())
    markers = {
        "idle": "recovery: idle",
        "running": "recovery: RUNNING",
        "awaiting_confirmation": "recovery: AWAITING CONFIRMATION",
        "plrd_busy": "recovery: plrd IS EXECUTING A RECOVERY",
        "unknown": "recovery: UNKNOWN",
    }
    found = [state for state, marker in markers.items() if marker in body]
    assert len(found) == 1, (found, body)
    return found[0]


def _scripted(**script):
    return ScriptedDaemon(script)


@pytest.mark.parametrize(
    "responses,expected",
    [
        pytest.param([fx.completed()], "idle", id="completed"),
        pytest.param([fx.aborted()], "idle", id="aborted"),
        pytest.param([fx.pause()], "awaiting_confirmation", id="paused"),
        pytest.param([fx.busy()], "plrd_busy", id="busy"),
        pytest.param([fx.unknown_token(text="no execution")], "unknown", id="unknown"),
        pytest.param(
            [{"ok": True, "text": "?", "data": None}], "unknown", id="no-data"
        ),
        pytest.param(
            [{"ok": False, "text": "?", "data": {"outcome": "error"}}],
            "unknown",
            id="error-tag",
        ),
        pytest.param(
            [{"ok": False, "text": "?", "data": {"outcome": "brand-new-tag"}}],
            "unknown",
            id="unrecognized-tag",
        ),
    ],
)
def test_the_console_and_get_status_always_report_the_same_state(
    plugin, run, responses, expected
):
    # STRUCTURAL: `status_lines` and `get_status` must both come from
    # `state()`.  They used to derive it independently, so for the whole
    # doubtful window the JSON said `unknown` while the console still told
    # the operator to answer a question.
    plugin.daemon = _scripted(recover_execute=list(responses))
    execute(plugin, run)
    assert plugin.recovery.state() == expected
    assert plugin.get_status(100.0)["recovery_state"] == expected
    assert _console_state(plugin) == expected


def test_the_console_agrees_with_the_api_through_the_whole_downgrade(
    plugin, run, fake_printer
):
    plugin.daemon = _scripted(recover_execute=[fx.pause()])
    reactor = fake_printer.reactor
    execute(plugin, run)
    assert _console_state(plugin) == "awaiting_confirmation"
    assert plugin.get_status(100.0)["recovery_awaiting_confirmation"] is True
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    reactor.run_due_timers()
    # THE MAJOR: both surfaces move together.
    assert _console_state(plugin) == "unknown"
    assert plugin.recovery.state() == "unknown"
    assert plugin.get_status(100.0)["recovery_state"] == "unknown"
    assert plugin.get_status(100.0)["recovery_awaiting_confirmation"] is False
    # ...and the console still says the question can be answered, because
    # that is a separate fact from the state.
    body = "\n".join(plugin.recovery.status_lines())
    assert "a question can still be answered" in body
    assert "AWAITING CONFIRMATION" not in body


def test_the_two_questions_the_predicates_answer_are_not_the_same(plugin, run):
    # `may_start_new` and `needs_attention` diverge exactly in the two
    # unknowable states — which is why one boolean could not answer both.
    plugin.daemon = _scripted(recover_execute=[fx.busy()])
    execute(plugin, run)
    assert plugin.recovery.needs_attention() is True
    assert plugin.recovery.may_start_new() is True


def test_close_keeps_the_do_not_touch_warning(plugin, run, run_cmd):
    plugin.daemon = _scripted(recover_execute=[fx.busy()])
    execute(plugin, run)
    gcode = plugin.printer.lookup_object("gcode")
    since = responses_since(gcode)
    run_cmd("PLR_WIZARD_CLOSE")
    body = "\n".join(since())
    assert "action:prompt_end" in since()
    assert "DO NOT touch the printer" in body


def test_cancel_cannot_clear_a_machine_state_warning(plugin, run, run_cmd):
    plugin.daemon = _scripted(recover_execute=[fx.busy()])
    execute(plugin, run)
    with pytest.raises(fake_klippy.FakeCommandError, match="cannot stop what plrd"):
        run_cmd("PLR_WIZARD_CANCEL")
    # The warning survives a dismissal, because it belongs to the machine.
    assert plugin.recovery.state() == "plrd_busy"


def test_the_wizard_still_works_in_the_two_unknowable_states(
    plugin, run, run_cmd, pump
):
    # A state a session can enter and never leave is a defect on its own.
    # Refusing the wizard here left a bare PLR_RECOVER EXECUTE=1 — which
    # skips the dry-run review the wizard exists to impose — or a firmware
    # restart as the only ways out, for the rest of the klippy session.
    plugin.daemon = _scripted(
        recover_execute=[fx.busy()],
        status=[
            {
                "ok": True,
                "text": "",
                "data": {"pending": {"file": "/x/bench.gcode", "percent": 12.0}},
            }
        ],
    )
    execute(plugin, run)
    assert plugin.recovery.state() == "plrd_busy"
    assert plugin.recovery.may_start_new() is True
    gcode = plugin.printer.lookup_object("gcode")
    since = responses_since(gcode)
    run_cmd("PLR_WIZARD_START")
    assert pump() == 1
    lines = since()
    body = "\n".join(lines)
    # The flow opens...
    assert "action:prompt_begin Power-loss recovery" in lines
    assert "action:prompt_button Attempt recovery|PLR_WIZARD_DRYRUN|primary" in lines
    assert plugin.wizard.state() == "offered"
    # ...and it carries the warning INTO the dialog rather than replacing it.
    assert "plrd IS EXECUTING A RECOVERY" in body
    assert "DO NOT touch the printer" in body


def test_the_wizard_is_refused_only_while_plrd_talks_to_this_session(
    plugin, run_cmd, pump
):
    plugin.daemon = _scripted(recover_execute=[fx.completed()])
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert plugin.recovery.state() == "running"
    gcode = plugin.printer.lookup_object("gcode")
    since = responses_since(gcode)
    run_cmd("PLR_WIZARD_START")
    body = "\n".join(since())
    assert "not offering a new one until it reports" in body
    assert not any(line.startswith("action:prompt_begin") for line in since())
    assert pump() == 1


# --- a pause that arrives after the shutdown --------------------------


def test_a_pause_arriving_after_a_shutdown_is_aborted_not_rendered(
    plugin, run_cmd, pump, fake_printer
):  # noqa: D103 - see the comment below
    # The shutdown handler covers a question outstanding AT shutdown; this is
    # the likelier order — plrd was mid-step when M112 landed and asks
    # afterwards.  A live dialog whose Continue cannot work, with nothing to
    # abort it, would leave plrd holding the machine to its own deadline.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    fake_printer.invoke_shutdown("Manual stop (M112)")
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    assert pump() == 1
    lines = since()
    body = "\n".join(lines)
    assert "klippy is shut down so the recovery cannot continue" in body
    assert "has been SENT to plrd" in body
    assert "action:prompt_end" in lines
    # No live dialog was opened for a question nobody could answer.
    assert not any(line.startswith("action:prompt_button") for line in lines)
    assert plugin.recovery.can_answer() is False
    # Unknown, not idle: the abort is unconfirmed.
    assert plugin.recovery.state() == "unknown"
    for _ in range(400):
        if plugin.daemon.answers():
            break
        time.sleep(0.005)
    assert plugin.daemon.answers() == ["abort"]


# --- THE MECHANISM: the asymmetry is enforced, not remembered ----------
#
# Four review rounds found the same bug — a write site publishing a calmer
# state than the evidence supported — in four different places.  Making the
# READS safe (one authority, one plain read) did not stop it, because fifteen
# writes were still free to invent any value.  These tests are the
# compiler-equivalent for the invariant.


def _recovery_source():
    import os

    from plr import recovery as module

    with open(module.__file__, encoding="utf-8") as handle:
        return handle.read(), os.path.basename(module.__file__)


def _state_write_sites(source):
    """Every function that assigns ``self._state`` in ``source``."""
    import ast

    tree = ast.parse(source)
    scopes = [
        (node.lineno, getattr(node, "end_lineno", node.lineno), node.name)
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    ]
    writers = set()
    for node in ast.walk(tree):
        targets = []
        if isinstance(node, ast.Assign):
            targets = list(node.targets)
        elif isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            targets = [node.target]
        for target in targets:
            if (
                isinstance(target, ast.Attribute)
                and target.attr == "_state"
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                enclosing = sorted(
                    (start, fn)
                    for start, end, fn in scopes
                    if start <= node.lineno <= end
                )
                writers.add(enclosing[-1][1] if enclosing else "<module>")
    return writers


def test_only_the_guarded_transition_may_assign_the_state():
    # THE STRUCTURAL GUARANTEE.  ``_state`` is assignable in exactly two
    # places: its initial value in __init__ (nothing to compare against) and
    # inside ``_transition`` (which enforces the ordering).  Any new write
    # site fails here, which is how this round's blocker stops being
    # writeable rather than merely absent today.
    source, name = _recovery_source()
    assert _state_write_sites(source) == {"__init__", "_transition"}, name


def test_the_write_site_scan_catches_the_rollback_this_round_removed():
    # Proof the structural test is not vacuous: the exact shape of the
    # blocker — start() rolling a failed launch back to idle — is found.
    reintroduced = (
        "class S:\n"
        "    def start(self):\n"
        "        started = self._async.call()\n"
        "        if not started:\n"
        "            self._state = STATE_IDLE\n"
        "            raise gcmd.error('nope')\n"
    )
    assert _state_write_sites(reintroduced) == {"start"}


def test_the_alarm_ordering_covers_every_state():
    # A state missing from the ordering would raise KeyError inside a reactor
    # callback, i.e. be swallowed by the wrapper and silently skip the
    # transition.
    states = {
        recovery.STATE_IDLE,
        recovery.STATE_RUNNING,
        recovery.STATE_AWAITING,
        recovery.STATE_PLRD_BUSY,
        recovery.STATE_UNKNOWN,
    }
    assert set(recovery._ALARM) == states
    # idle is the calmest, unknown the most alarming, and the two
    # "engaged with us" states are equal so the confirm loop moves freely.
    assert recovery._ALARM[recovery.STATE_IDLE] == 0
    assert (
        recovery._ALARM[recovery.STATE_RUNNING]
        == recovery._ALARM[recovery.STATE_AWAITING]
    )
    assert (
        recovery._ALARM[recovery.STATE_UNKNOWN]
        > recovery._ALARM[recovery.STATE_PLRD_BUSY]
        > recovery._ALARM[recovery.STATE_RUNNING]
        > recovery._ALARM[recovery.STATE_IDLE]
    )


@pytest.mark.parametrize(
    "from_state,to_state",
    [
        pytest.param(recovery.STATE_UNKNOWN, recovery.STATE_IDLE, id="unknown-idle"),
        pytest.param(recovery.STATE_PLRD_BUSY, recovery.STATE_IDLE, id="busy-idle"),
        pytest.param(
            recovery.STATE_PLRD_BUSY, recovery.STATE_RUNNING, id="busy-running"
        ),
        pytest.param(
            recovery.STATE_UNKNOWN, recovery.STATE_AWAITING, id="unknown-awaiting"
        ),
        pytest.param(recovery.STATE_RUNNING, recovery.STATE_IDLE, id="running-idle"),
    ],
)
def test_a_calming_transition_without_a_reason_is_refused(plugin, from_state, to_state):
    # THE RUNTIME HALF.  Refused rather than raised: this runs on the
    # reactor, and the safe direction is to keep publishing the alarm.
    session = plugin.recovery
    session._transition(from_state)  # toward alarming: always allowed
    assert session.state() == from_state
    session._transition(to_state)  # no reason given
    assert session.state() == from_state, "a calmer state was published unreasoned"
    session._transition(to_state, reason="a test said so")
    assert session.state() == to_state


@pytest.mark.parametrize(
    "from_state,to_state",
    [
        pytest.param(recovery.STATE_IDLE, recovery.STATE_RUNNING, id="idle-running"),
        pytest.param(
            recovery.STATE_RUNNING, recovery.STATE_PLRD_BUSY, id="running-busy"
        ),
        pytest.param(
            recovery.STATE_PLRD_BUSY, recovery.STATE_UNKNOWN, id="busy-unknown"
        ),
        pytest.param(
            recovery.STATE_RUNNING, recovery.STATE_AWAITING, id="running-awaiting"
        ),
        pytest.param(
            recovery.STATE_AWAITING, recovery.STATE_RUNNING, id="awaiting-running"
        ),
    ],
)
def test_a_transition_toward_alarming_or_sideways_needs_no_reason(
    plugin, from_state, to_state
):
    session = plugin.recovery
    session._transition(from_state, reason="test setup")
    session._transition(to_state)
    assert session.state() == to_state


def test_a_failed_launch_never_publishes_a_calmer_state(plugin, run_cmd, pump):
    # THE BLOCKER, end to end.  may_start_new() admits plrd_busy, and this
    # command is what the plugin advertises as the probe there — so a
    # rollback to idle published "nothing is happening" over positive
    # evidence that plrd was executing.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.busy()]})
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    assert plugin.recovery.state() == "plrd_busy"

    started = []

    def refuse(self):
        started.append(True)
        raise RuntimeError("can't start new thread")

    import threading as _threading

    original = _threading.Thread.start
    _threading.Thread.start = refuse
    try:
        with pytest.raises(fake_klippy.FakeCommandError) as excinfo:
            run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    finally:
        _threading.Thread.start = original
    assert started, "the launch was never attempted"
    # The refusal blames the right component...
    assert "could not start a worker thread" in str(excinfo.value)
    assert "not the daemon" in str(excinfo.value)
    # ...and nothing about the machine got calmer.
    assert plugin.recovery.state() == "plrd_busy"
    assert plugin.recovery.needs_attention() is True
    assert plugin.get_status(100.0)["recovery_state"] == "plrd_busy"
    assert "plrd IS EXECUTING A RECOVERY" in "\n".join(plugin.recovery.status_lines())


def test_a_failed_launch_keeps_a_question_answerable(
    plugin, run, run_cmd, fake_printer
):
    # The same rollback also destroyed a kept token AFTER telling the
    # operator an abort had been sent for it.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    execute(plugin, run)
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    reactor.run_due_timers()
    assert plugin.recovery.can_answer() is True

    import threading as _threading

    original = _threading.Thread.start
    _threading.Thread.start = lambda self: (_ for _ in ()).throw(
        RuntimeError("can't start new thread")
    )
    try:
        with pytest.raises(fake_klippy.FakeCommandError):
            run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    finally:
        _threading.Thread.start = original
    # The question is still answerable, and no abort was announced for it.
    assert plugin.recovery.can_answer() is True
    assert plugin.recovery.state() == "unknown"
    assert not any(
        "abandoning the confirmation" in line
        for line in fake_printer.lookup_object("gcode").responses
    )


# --- the protocol tags this round re-derived --------------------------


def test_malformed_is_not_treated_as_terminal(plugin, run, fake_printer):
    # ctrlsock.rs:740-751 returns `malformed` from recover_confirm BEFORE
    # session.outstanding.take(), so plrd is still standing at the confirm
    # point.  Routing it to idle was the same mis-derivation the `error`
    # exclusion exists to avoid.
    assert "malformed" not in recovery.TERMINAL_OUTCOMES
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause()],
            "recover_confirm": [
                {
                    "ok": False,
                    "text": "recover_confirm requires a string token",
                    "data": {"outcome": "malformed"},
                }
            ],
        }
    )
    execute(plugin, run)
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    run("PLR_RECOVER_CONTINUE")
    body = "\n".join(since())
    assert plugin.recovery.state() == "unknown"
    assert "DO NOT touch the printer" in body


@pytest.mark.parametrize("outcome", ["unknown-cmd", "oversized"])
def test_a_protocol_refusal_says_the_daemon_is_too_old(
    plugin, run, fake_printer, outcome
):
    # ctrlsock.rs:371 / :324 answer before dispatch, so nothing happened.
    # "DO NOT touch the printer" would be wrong here — and a plugin newer
    # than its daemon hits this on EVERY attempt, so it is also how an
    # operator learns to ignore the warnings.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [
                {
                    "ok": False,
                    "text": "unknown cmd recover_execute",
                    "data": {"outcome": outcome},
                }
            ]
        }
    )
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    body = "\n".join(since())
    # From idle, a rejected REQUEST leaves idle standing...
    assert plugin.recovery.state() == "idle"
    assert "nothing was started by it" in body
    assert "NEWER than the plrd" in body
    assert "DO NOT touch" not in body


@pytest.mark.parametrize("outcome", ["unknown-cmd", "oversized"])
def test_a_protocol_refusal_never_discards_evidence_about_the_machine(
    plugin, run, fake_printer, outcome
):
    # "plrd rejected the request" is a fact about the REQUEST.  Publishing
    # idle over a prior `plrd_busy` would discard positive proof that plrd is
    # executing — the same class of error as every other calming claim.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [
                fx.busy(),
                {
                    "ok": False,
                    "text": "unknown cmd recover_execute",
                    "data": {"outcome": outcome},
                },
            ]
        }
    )
    execute(plugin, run)
    assert plugin.recovery.state() == "plrd_busy"
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    body = "\n".join(since())
    assert plugin.recovery.state() == "plrd_busy"
    assert "the recovery state is unchanged" in body
    assert "plrd IS EXECUTING A RECOVERY" in body


def test_the_agreement_matrix_covers_running_too(plugin, run_cmd, pump):
    # The matrix above drives states through plrd's answers, which cannot
    # produce `running` (that is the state WHILE a call is in flight).  So it
    # is checked here, where it exists.
    plugin.daemon = _scripted(recover_execute=[fx.completed()])
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert plugin.recovery.state() == "running"
    assert plugin.get_status(100.0)["recovery_state"] == "running"
    assert _console_state(plugin) == "running"
    assert pump() == 1


def test_can_answer_is_published_on_both_surfaces(plugin, run, fake_printer):
    # The console says "a question can still be answered"; the API must say
    # the same, or a UI hides buttons the console is offering.
    plugin.daemon = _scripted(recover_execute=[fx.pause()])
    execute(plugin, run)
    assert plugin.get_status(100.0)["recovery_can_answer"] is True
    reactor = fake_printer.reactor
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    reactor.run_due_timers()
    # Downgraded: the state is no longer awaiting, but the question stands —
    # and both surfaces say so.
    assert plugin.recovery.state() == "unknown"
    assert plugin.get_status(100.0)["recovery_can_answer"] is True
    assert "a question can still be answered" in "\n".join(
        plugin.recovery.status_lines()
    )
    reactor.advance(recovery.DAEMON_CONFIRM_CEILING_S)
    reactor.run_due_timers()
    assert plugin.get_status(100.0)["recovery_can_answer"] is False


# --- THE ABORT PATH CANNOT PUBLISH A CALMER STATE ---------------------
#
# The fifth instance of the calmer-state defect lived inside the machinery
# built to prevent the first four: `_transition` forced a REASON to exist but
# could not tell whether the sentence was true, and the detached abort's
# reason ("plrd accepted the abort") rested on a send with no reply.  So the
# abort path no longer has access to a calming transition at all.


def _function_calls(source, function_name):
    """Every ``self.<name>(...)`` called inside ``function_name``.

    Nested functions count as part of their parent: the abort's worker thread
    body is a closure inside `_abort_detached`, and it must be covered by the
    same rule.
    """
    import ast

    tree = ast.parse(source)
    target = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == function_name:
            target = node
            break
    assert target is not None, function_name
    calls = set()
    for node in ast.walk(target):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if isinstance(node.func.value, ast.Name) and node.func.value.id == "self":
                calls.add(node.func.attr)
    return calls


@pytest.mark.parametrize(
    "function_name",
    ["_abort_detached", "_abort_reported", "_abort_shutdown_pause"],
)
def test_the_abort_path_cannot_call_the_calming_transition(function_name):
    # THE STRUCTURAL RULE.  `_raise_alarm` is allowed (it cannot lower);
    # `_transition` is not, because it can.  Reintroducing the acceptance
    # claim requires calling `_transition` here, which fails this test.
    source, _name = _recovery_source()
    calls = _function_calls(source, function_name)
    assert "_transition" not in calls, sorted(calls)


def test_raise_alarm_cannot_lower_the_state(plugin):
    session = plugin.recovery
    session._transition(recovery.STATE_UNKNOWN)
    session._raise_alarm(recovery.STATE_IDLE)
    assert session.state() == "unknown"
    session._raise_alarm(recovery.STATE_PLRD_BUSY)
    assert session.state() == "unknown"
    # Equal or more alarming is fine.
    session._transition(recovery.STATE_IDLE, reason="test")
    session._raise_alarm(recovery.STATE_PLRD_BUSY)
    assert session.state() == "plrd_busy"


def _shutdown_with_abort_reply(plugin, run, fake_printer, reply):
    """Drive M112 at a confirm point with plrd's abort reply scripted."""
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [reply]}
    )
    execute(plugin, run)
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    fake_printer.invoke_shutdown("Manual stop (M112)")
    for _ in range(400):
        if plugin.daemon.tokens():
            break
        time.sleep(0.005)
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    return since


@pytest.mark.parametrize(
    "reply,expected",
    [
        # THE CASE THAT RETRACTED A WARNING: plrd is still standing at the
        # confirm point (ctrlsock.rs:740-751 answers before it takes the
        # outstanding question), and this used to read as acceptance.
        pytest.param(
            {
                "ok": False,
                "text": "recover_confirm requires a string token",
                "data": {"outcome": "malformed"},
            },
            "says nothing about whether it applied the abort",
            id="malformed",
        ),
        pytest.param(
            fx.busy(),
            "still executing a recovery",
            id="busy",
        ),
        pytest.param(
            fx.unknown_token(text="no execution is awaiting confirmation"),
            "not waiting for that question any more",
            id="unknown-token",
        ),
        pytest.param(
            {
                "ok": False,
                "text": "unknown cmd recover_confirm",
                "data": {"outcome": "unknown-cmd"},
            },
            "says nothing about whether it applied the abort",
            id="unknown-cmd",
        ),
        pytest.param(
            fx.pause(token="plrc-other"),
            "is PAUSED at a confirm-point",
            id="still-paused",
        ),
        pytest.param(
            fx.aborted(),
            "reported the recovery as over",
            id="terminal",
        ),
    ],
)
def test_no_plrd_reply_to_an_abort_is_read_as_acceptance(
    plugin, run, fake_printer, reply, expected
):
    since = _shutdown_with_abort_reply(plugin, run, fake_printer, reply)
    body = "\n".join(since())
    assert "plrd replied to the abort" in body
    assert expected in body
    assert "does NOT treat any reply as confirmation" in body
    # THE INVARIANT: whatever plrd said, nothing got calmer.
    assert plugin.recovery.state() == "unknown"
    assert plugin.get_status(100.0)["recovery_state"] == "unknown"
    assert "DO NOT touch the printer" in "\n".join(plugin.recovery.status_lines())


def test_no_reply_at_all_is_not_acceptance(plugin, run, fake_printer):
    # THE NORMAL SUCCESS PATH: recover_confirm only returns once plrd has
    # finished aborting, which includes pushing cleanup g-code through
    # Moonraker — so the send window closing is the RULE, not a failure, and
    # it proves nothing either way.
    since = _shutdown_with_abort_reply(
        plugin,
        run,
        fake_printer,
        daemon_link.DaemonError(
            "plrd did not answer 'recover_confirm' within 5s at /x"
        ),
    )
    body = "\n".join(since())
    assert "did not reply inside the send window" in body
    assert "NOT confirmation that the abort was applied" in body
    assert "accepted" not in body
    assert plugin.recovery.state() == "unknown"


def test_an_abort_outcome_cannot_land_on_a_later_conversation(
    plugin, run, run_cmd, fake_printer
):
    # The old gate was `if self._token is None`, which tests for "no NEW
    # question" rather than identity — so an abandoned question's outcome
    # could take a fresh recover_execute from running to idle while that
    # request was still in flight.  The epoch makes it impossible.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause(), fx.completed()],
            "recover_confirm": [fx.aborted()],
        }
    )
    reactor = fake_printer.reactor
    execute(plugin, run)
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    reactor.run_due_timers()
    # Start a fresh recovery, abandoning the (still answerable) question.
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert plugin.recovery.state() == "running"
    for _ in range(400):
        if any(c[0] == "recover_confirm" for c in plugin.daemon.calls):
            break
        time.sleep(0.005)
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    # Deliver the abandoned abort's outcome FIRST, while the new execute is
    # still outstanding.
    assert reactor.pump_async(1, timeout=2.0) >= 1
    body = "\n".join(since())
    # The outcome IS reported — what plrd said can matter — but it is labelled
    # as belonging to the earlier conversation, and it moved nothing: the new
    # request's own state stands.
    assert "about an earlier conversation" in body
    assert "does NOT treat any reply as confirmation" in body
    assert plugin.recovery.state() in ("running", "idle")


# --- one reading of plrd's tags ---------------------------------------


@pytest.mark.parametrize(
    "response,kind",
    [
        pytest.param(fx.pause(), recovery.ANSWER_PAUSE, id="pause"),
        pytest.param(fx.busy(), recovery.ANSWER_BUSY, id="busy"),
        pytest.param(fx.unknown_token(), recovery.ANSWER_STALE_TOKEN, id="stale"),
        pytest.param(fx.completed(), recovery.ANSWER_TERMINAL, id="completed"),
        pytest.param(fx.aborted(), recovery.ANSWER_TERMINAL, id="aborted"),
        pytest.param(
            {"ok": False, "text": "", "data": {"outcome": "unknown-cmd"}},
            recovery.ANSWER_PROTOCOL_REFUSAL,
            id="unknown-cmd",
        ),
        pytest.param(
            {"ok": False, "text": "", "data": {"outcome": "oversized"}},
            recovery.ANSWER_PROTOCOL_REFUSAL,
            id="oversized",
        ),
        # The two excluded tags: one tag, two opposite machine states.
        pytest.param(
            {"ok": False, "text": "", "data": {"outcome": "error"}},
            recovery.ANSWER_UNCLASSIFIABLE,
            id="error",
        ),
        pytest.param(
            {"ok": False, "text": "", "data": {"outcome": "malformed"}},
            recovery.ANSWER_UNCLASSIFIABLE,
            id="malformed",
        ),
        pytest.param(
            {"ok": False, "text": "", "data": {"outcome": "brand-new"}},
            recovery.ANSWER_UNCLASSIFIABLE,
            id="future-tag",
        ),
        # ok:true under an unknown tag rests on plrd's own `ok` invariant.
        pytest.param(
            {"ok": True, "text": "", "data": {"outcome": "brand-new"}},
            recovery.ANSWER_TERMINAL,
            id="ok-true-unknown-tag",
        ),
        pytest.param(
            {"ok": True, "text": "", "data": None},
            recovery.ANSWER_UNCLASSIFIABLE,
            id="no-data",
        ),
        pytest.param(None, recovery.ANSWER_UNCLASSIFIABLE, id="not-a-dict"),
    ],
)
def test_classify_is_the_single_reading_of_plrds_tags(response, kind):
    # Both the confirm loop and the detached abort's report go through this
    # one function: a second bespoke reading is how the abort came to treat
    # six typed refusals as acceptance.
    assert recovery.classify(response)[0] == kind


def test_classify_is_pure():
    # No state, no side effects — the property that lets two very different
    # callers share it.
    response = fx.busy()
    before = dict(response)
    recovery.classify(response)
    recovery.classify(response)
    assert response == before


# --- busy on the CONFIRM path keeps the question answerable ------------


def test_busy_in_reply_to_an_answer_keeps_the_question_answerable(
    plugin, run, run_cmd, fake_printer
):
    # ctrlsock.rs:752-754 answers `busy` when the session lock is contended,
    # which means the ANSWER never landed — so the question may well still be
    # outstanding at plrd, and destroying the token would strand the operator
    # with a live question they can no longer answer.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause()],
            "recover_confirm": [fx.busy(), fx.completed()],
        }
    )
    execute(plugin, run)
    token_before = plugin.recovery._token
    run("PLR_RECOVER_CONTINUE")
    assert plugin.recovery.state() == "plrd_busy"
    # The token came back, so a retry is possible...
    assert plugin.recovery.can_answer() is True
    assert plugin.recovery._token == token_before
    assert plugin.get_status(100.0)["recovery_can_answer"] is True
    assert "a question can still be answered" in "\n".join(
        plugin.recovery.status_lines()
    )
    # ...and it works.
    run("PLR_RECOVER_CONTINUE")
    assert plugin.daemon.answers() == ["continue", "continue"]
    assert plugin.recovery.state() == "idle"


def test_busy_in_reply_to_an_execute_has_no_question_to_keep(plugin, run):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.busy()]})
    execute(plugin, run)
    assert plugin.recovery.state() == "plrd_busy"
    assert plugin.recovery.can_answer() is False


# --- the plrd_busy shutdown branch ------------------------------------


def test_a_shutdown_while_plrd_is_busy_elsewhere_promises_no_report(
    plugin, run, fake_printer
):
    # `was_running` used to conflate `running` with `plrd_busy`, but in
    # plrd_busy no call of ours is in flight, so "its report still appears
    # here" told the operator to wait forever.
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.busy()]})
    execute(plugin, run)
    assert plugin.recovery.state() == "plrd_busy"
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    fake_printer.invoke_shutdown("Manual stop (M112)")
    body = "\n".join(since())
    assert "NO report for it will appear here" in body
    assert "journalctl -u plrd" in body
    assert "still appears here" not in body
    # No abort is sent: there is no question, and this plugin cannot stop
    # what it is not connected to.
    assert plugin.daemon.answers() == []
    assert plugin.recovery.state() == "plrd_busy"


# --- the rollback restores deadlines, it does not extend them ----------


def test_a_failed_answer_restores_the_deadlines_it_found(
    fake_printer, plr_config, run_cmd, pump
):
    # Re-arming from `now` would let the plugin assert a live question up to a
    # full wait past plrd's real deadline.
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "120")
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    waketimes = [t.waketime for t in reactor.timers]
    # Time passes, then an answer that cannot start (the channel is busy).
    reactor.advance(60.0)
    release = threading.Event()

    class Hanging:
        calls = []

        def call(self, cmd, args=None, timeout=None):
            self.calls.append((cmd, args, timeout))
            release.wait(5.0)
            return fx.completed()

    plugin.daemon = Hanging()
    plugin.recovery._async.call("status", None, 1.0, lambda r: None, lambda e: None)
    try:
        with pytest.raises(fake_klippy.FakeCommandError, match="still waiting"):
            run_cmd("PLR_RECOVER_CONTINUE")
    finally:
        release.set()
    # The question is back, and its deadlines are where they were — NOT
    # 60 seconds later.
    assert plugin.recovery.can_answer() is True
    assert [t.waketime for t in reactor.timers] == waketimes
    fake_printer.reactor.pump_async(1, timeout=2.0)


def test_an_unexpected_error_in_the_abort_thread_is_reported_as_a_send_failure(
    plugin, run, fake_printer
):
    class Exploding:
        calls = []

        def call(self, cmd, args=None, timeout=None):
            self.calls.append((cmd, args, timeout))
            if cmd == "recover_confirm":
                raise ValueError("something we never classified")
            return fx.pause()

    plugin.daemon = Exploding()
    execute(plugin, run)
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    fake_printer.invoke_shutdown("Manual stop (M112)")
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    body = "\n".join(since())
    assert "could NOT be sent to plrd" in body
    assert "ValueError" in body
    assert "Assume plrd is STILL PAUSED" in body
    assert plugin.recovery.state() == "unknown"


def test_an_abort_outcome_after_teardown_is_dropped(plugin, run, fake_printer):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.aborted()]}
    )
    execute(plugin, run)
    fake_printer.invoke_shutdown("Manual stop (M112)")
    for _ in range(400):
        if plugin.daemon.tokens():
            break
        time.sleep(0.005)
    # Teardown lands before the outcome is delivered: nothing may be said to
    # a printer that has gone away.
    fake_printer.send_event("klippy:disconnect")
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert since() == []


def test_a_broken_console_cannot_take_klippy_down_from_the_abort_report(
    plugin, run, fake_printer, monkeypatch
):
    plugin.daemon = ScriptedDaemon(
        {"recover_execute": [fx.pause()], "recover_confirm": [fx.aborted()]}
    )
    execute(plugin, run)
    fake_printer.invoke_shutdown("Manual stop (M112)")

    def boom(_msg):
        raise RuntimeError("respond exploded")

    monkeypatch.setattr(plugin.recovery, "_respond", boom)
    # A raise here would reach klippy's reactor loop, which turns it into a
    # printer shutdown (klippy/klippy.py:170-186).
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1


# --- resume preview: the reposition loop over the socket (design §F.2) --
#
# A preview pause rides the SAME awaiting/running loop every binary pause
# does (no new plugin state): a reposition answer (next/prev/nudge) sends a
# recover_confirm and plrd PAUSES AGAIN with the next stop, which arrives
# through the identical path a second binary confirm-point would. These
# tests drive the whole loop through the registered console commands.


def execute_preview(plugin, run, **script):
    """Start a recovery whose first reply is a preview pause."""
    calls = {"recover_execute": [fx.preview_pause()]}
    calls.update(script)
    plugin.daemon = ScriptedDaemon(calls)
    execute(plugin, run)


def test_a_preview_pause_reaches_a_rendered_preview_prompt(plugin, run, fake_printer):
    execute_preview(plugin, run)
    gcode = fake_printer.lookup_object("gcode")
    lines = gcode.responses
    # A RENDERED preview prompt, with the reposition buttons wired to plain
    # commands (the console is the floor, dialogs the enhancement).
    assert any("action:prompt_begin Power-loss recovery — align" in ln for ln in lines)
    assert "action:prompt_button Accept|PLR_RECOVER_ACCEPT|primary" in lines
    assert any("PLR_RECOVER_NUDGE FWD=1" in ln for ln in lines)
    assert any("PLR_RECOVER_NUDGE BACK=10" in ln for ln in lines)
    assert "action:prompt_footer_button Abort recovery|PLR_RECOVER_ABORT|error" in lines
    # The per-stop readout is on the wire (offset + XY), not just a log line.
    body = "\n".join(lines)
    assert "byte 244,118" in body and "X132.4 Y88.1" in body
    assert plugin.recovery.state() == "awaiting_confirmation"
    assert plugin.recovery.can_answer() is True


def test_accept_sends_accept_and_the_recovery_completes(plugin, run, fake_printer):
    execute_preview(plugin, run, recover_confirm=[fx.completed()])
    since = responses_since(fake_printer.lookup_object("gcode"))
    run("PLR_RECOVER_ACCEPT")
    assert plugin.daemon.calls[1] == (
        "recover_confirm",
        {"token": "plrc-17bd4c0f9a2-5", "answer": "accept"},
        daemon_link.EXECUTE_TIMEOUT,
    )
    assert "plan complete" in "\n".join(since())
    assert plugin.recovery.state() == "idle"


def test_next_repositions_and_the_next_stop_is_rendered(plugin, run, fake_printer):
    # next -> plrd repositions and PAUSES AGAIN with the new stop, which the
    # plugin renders exactly as it rendered the first (offset refreshed).
    execute_preview(
        plugin,
        run,
        recover_confirm=[
            fx.preview_pause(token="plrc-17bd4c0f9a2-6", offset=250000, position=4)
        ],
    )
    since = responses_since(fake_printer.lookup_object("gcode"))
    run("PLR_RECOVER_NEXT")
    assert plugin.daemon.answers() == ["next"]
    body = "\n".join(since())
    assert "byte 250,000" in body  # the NEW stop's offset, re-emitted
    assert "stop 4 of 5" in body
    # Still awaiting, still answerable: the loop moved awaiting -> running ->
    # awaiting with no new state invented.
    assert plugin.recovery.state() == "awaiting_confirmation"
    assert plugin.recovery.can_answer() is True


@pytest.mark.parametrize(
    "params,expected_count",
    [
        ({"FWD": 1}, 1),
        ({"FWD": 10}, 10),
        ({"BACK": 1}, -1),
        ({"BACK": 10}, -10),
    ],
)
def test_nudge_sends_the_signed_count_plrd_parses(
    plugin, run, fake_printer, params, expected_count
):
    execute_preview(
        plugin, run, recover_confirm=[fx.preview_pause(token="plrc-17bd4c0f9a2-6")]
    )
    run("PLR_RECOVER_NUDGE", **params)
    cmd, args, _t = plugin.daemon.calls[1]
    assert cmd == "recover_confirm"
    assert args["answer"] == "nudge"
    assert args["count"] == expected_count


def test_nudge_requires_exactly_one_direction(plugin, run):
    execute_preview(plugin, run)
    with pytest.raises(fake_klippy.FakeCommandError, match="exactly one of FWD"):
        run("PLR_RECOVER_NUDGE", FWD=1, BACK=1, calls=0)
    with pytest.raises(fake_klippy.FakeCommandError, match="exactly one of FWD"):
        run("PLR_RECOVER_NUDGE", calls=0)


def test_nudge_rejects_a_step_that_is_not_one_or_ten(plugin, run):
    execute_preview(plugin, run)
    with pytest.raises(fake_klippy.FakeCommandError, match="1 .fine. or 10"):
        run("PLR_RECOVER_NUDGE", FWD=5, calls=0)


def test_a_full_preview_conversation_pause_nudge_pause_accept(
    plugin, run, fake_printer
):
    # The whole loop end to end: open on a stop, nudge to another, accept.
    execute_preview(
        plugin,
        run,
        recover_confirm=[
            fx.preview_pause(token="plrc-17bd4c0f9a2-6", offset=245000),
            fx.completed(),
        ],
    )
    gcode = fake_printer.lookup_object("gcode")
    run("PLR_RECOVER_NUDGE", FWD=1)
    assert "byte 245,000" in "\n".join(gcode.responses)
    since = responses_since(gcode)
    run("PLR_RECOVER_ACCEPT")
    assert plugin.daemon.answers() == ["nudge", "accept"]
    assert "plan complete" in "\n".join(since())
    assert plugin.recovery.state() == "idle"


# --- the shutdown rule under every verb (design §D.2) -----------------


@pytest.mark.parametrize(
    "command,params",
    [
        ("PLR_RECOVER_ACCEPT", {}),
        ("PLR_RECOVER_NEXT", {}),
        ("PLR_RECOVER_PREV", {}),
        ("PLR_RECOVER_NUDGE", {"FWD": 1}),
        ("PLR_RECOVER_NUDGE", {"BACK": 10}),
    ],
)
def test_shutdown_refuses_every_repositioning_verb(
    plugin, run, fake_printer, command, params
):
    # A shut-down machine cannot move, so accept AND every reposition are
    # refused — the generalization of the continue-when-shutdown rule.
    execute_preview(plugin, run)
    fake_printer.in_shutdown_state = True  # shutdown without running handlers
    with pytest.raises(fake_klippy.FakeCommandError, match="shut down"):
        run(command, calls=0, **params)
    # Nothing was sent to plrd: the pause is untouched and still answerable.
    assert [c for c in plugin.daemon.calls if c[0] == "recover_confirm"] == []
    assert plugin.recovery.can_answer() is True


def test_shutdown_still_allows_abort_on_a_preview_pause(plugin, run, fake_printer):
    execute_preview(plugin, run, recover_confirm=[fx.aborted()])
    fake_printer.in_shutdown_state = True
    since = responses_since(fake_printer.lookup_object("gcode"))
    run("PLR_RECOVER_ABORT")
    assert plugin.daemon.answers() == ["abort"]
    assert "did not complete" in "\n".join(since())


# --- kind discrimination (design §D.3) --------------------------------


def test_a_preview_verb_on_a_binary_pause_is_refused_and_keeps_the_token(
    plugin, run, fake_printer
):
    # The binary z-height pause takes continue/abort; a preview verb is
    # refused BEFORE the token is spent, so the pause stays answerable (the
    # token-preserving mirror of the daemon's own wrong-kind guard).
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.z_height_pause()]})
    execute(plugin, run)
    for command in ("PLR_RECOVER_ACCEPT", "PLR_RECOVER_NEXT", "PLR_RECOVER_PREV"):
        with pytest.raises(fake_klippy.FakeCommandError, match="not a resume preview"):
            run(command, calls=0)
    with pytest.raises(fake_klippy.FakeCommandError, match="not a resume preview"):
        run("PLR_RECOVER_NUDGE", FWD=1, calls=0)
    assert [c for c in plugin.daemon.calls if c[0] == "recover_confirm"] == []
    assert plugin.recovery.can_answer() is True
    # The binary answer still works.
    plugin.daemon.script["recover_confirm"] = [fx.completed()]
    run("PLR_RECOVER_CONTINUE")
    assert plugin.daemon.answers() == ["continue"]


def test_continue_on_a_preview_pause_is_refused_and_names_the_preview_verbs(
    plugin, run
):
    execute_preview(plugin, run)
    with pytest.raises(fake_klippy.FakeCommandError, match="resume-preview pause"):
        run("PLR_RECOVER_CONTINUE", calls=0)
    assert plugin.recovery.can_answer() is True


def test_abort_works_on_a_preview_pause(plugin, run, fake_printer):
    execute_preview(plugin, run, recover_confirm=[fx.aborted()])
    run("PLR_RECOVER_ABORT")
    assert plugin.daemon.answers() == ["abort"]
    assert plugin.recovery.state() == "idle"


def test_reshow_re_emits_the_current_preview_stops_full_readout(
    plugin, run, fake_printer
):
    # PLR_STATUS reshow reads the last pause payload, so it must re-emit the
    # CURRENT stop's full readout (design §F.2, carried obligation).
    execute_preview(plugin, run)
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    shown = plugin.recovery.reshow(gcode.respond_info)
    assert shown is True
    body = "\n".join(since())
    assert "byte 244,118" in body and "X132.4 Y88.1" in body
    assert "stop 3 of 5" in body
    assert "action:prompt_button Accept|PLR_RECOVER_ACCEPT|primary" in since()
