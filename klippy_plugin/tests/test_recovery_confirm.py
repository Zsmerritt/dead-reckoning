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
    assert plugin.recovery.is_awaiting() is True
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
        assert plugin.recovery.is_awaiting() is True
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
    assert plugin.recovery.is_awaiting() is True
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
    assert plugin.recovery.is_awaiting() is True


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
    assert plugin.recovery.is_awaiting() is True
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
    assert plugin.recovery.is_awaiting() is False
    # ...and a retry is permitted, because plrd's `busy` is the only
    # observation available.
    assert plugin.recovery.is_active() is False


@pytest.mark.parametrize("token", ["", 7, [], {}])
def test_a_malformed_token_is_treated_the_same_way(plugin, run, fake_printer, token):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause(token=token)]})
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    assert "no usable resume token" in "\n".join(since())


def test_only_the_timed_out_unknown_token_is_rendered_as_an_abort(
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
    assert "aborted" in body
    assert "invalidates the Z frame" in body
    assert "PLR_WIZARD_START" in body
    assert plugin.recovery.state() == "idle"


def test_a_busy_daemon_is_reported_as_changing_nothing(plugin, run, fake_printer):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.busy()]})
    gcode = fake_printer.lookup_object("gcode")
    since = responses_since(gcode)
    execute(plugin, run)
    body = "\n".join(since())
    assert "already has a recovery in flight" in body
    assert "changed nothing" in body
    assert plugin.recovery.state() == "idle"


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
    assert "no data" in body
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
            "idle",
            True,
            id="timed-out-aborted",
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
        assert "the recovery aborted" in body
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
    assert plugin.recovery.is_active() is False


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
    assert "'abort' has been sent" in body
    assert "action:prompt_end" in since()
    assert plugin.recovery.state() == "idle"
    assert plugin.recovery.is_awaiting() is False


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
    assert plugin.recovery.is_awaiting() is False


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
    body = "\n".join(since())
    assert "a recovery is already in flight" in body
    assert "AWAITING CONFIRMATION" in body
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
    assert plugin.recovery.is_awaiting() is True
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
    assert plugin.recovery.is_awaiting() is True


def test_a_shutdown_disarms_the_local_deadline(fake_printer, plr_config, run_cmd, pump):
    plugin = confirm_timeout_plugin(fake_printer, plr_config, "30")
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause()]})
    reactor = fake_printer.reactor
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    fake_printer.invoke_shutdown("Manual stop (M112)")
    reactor.advance(10000.0)
    assert reactor.run_due_timers() == 0
    assert plugin.recovery.state() == "idle"


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


def test_a_failing_shutdown_abort_is_contained(plugin, run, fake_printer, caplog):
    # The detached abort is best-effort: plrd's own deadline is the backstop,
    # so a failure there must not raise on a thread nobody is watching.
    plugin.daemon = ScriptedDaemon(
        {
            "recover_execute": [fx.pause()],
            "recover_confirm": [daemon_link.DaemonError("plrd is gone")],
        }
    )
    execute(plugin, run)
    fake_printer.invoke_shutdown("Manual stop (M112)")
    for _ in range(400):
        if plugin.daemon.tokens():
            break
        time.sleep(0.005)
    assert plugin.daemon.tokens() == ["plrc-17bd4c0f9a2-3"]
    time.sleep(0.05)
    assert plugin.recovery.state() == "idle"


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
    assert plugin.recovery.is_awaiting() is True

    fake_printer.invoke_shutdown("Manual stop (M112)")
    assert plugin.recovery.state() == "idle"
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
    assert plugin.recovery.is_active() is False
    # ...but the answer commands still work, because plrd's reply is the
    # only thing that can actually settle it.
    assert plugin.recovery.is_awaiting() is True


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


def test_a_fresh_recovery_is_allowed_after_the_downgrade(plugin, run, run_cmd, pump):
    plugin.daemon = ScriptedDaemon({"recover_execute": [fx.pause(), fx.busy()]})
    reactor = plugin.printer.get_reactor()
    execute(plugin, run)
    with pytest.raises(fake_klippy.FakeCommandError, match="WAITING FOR YOUR ANSWER"):
        run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    reactor.advance(recovery.DAEMON_CONFIRM_DEFAULT_S + recovery.CONFIRM_HEADROOM_S)
    reactor.run_due_timers()
    # Now permitted — and plrd itself adjudicates, answering `busy` if it is
    # in fact still paused.
    run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    assert [c[0] for c in plugin.daemon.calls] == ["recover_execute", "recover_execute"]


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
    assert plugin.recovery.is_active() is False


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
