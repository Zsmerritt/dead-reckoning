"""The prompt-driven recovery and commissioning wizards.

Drives the ``PLR_WIZARD_*`` state machine and ``PLR_SETUP_WIZARD`` through
the registered console commands against a fake daemon returning canned
socket responses.  Asserts the ORDERED respond sequences (action lines +
their plain-text fallbacks), the exact literal action strings against the
researched Mainsail spec, every branch (clean-confirm true/false/absent),
double-START, and daemon-down at each step (clear error + state reset).

EVERY DAEMON CALL IS ASYNCHRONOUS HERE, and that is the contract under
test: the handler hands the call to a worker thread and returns, so a
test must ``pump`` to deliver the answer (see the ``step`` fixture).  The
count is load-bearing — a handler that blocked would have printed its
result before returning, and the ``calls=0`` steps assert that a command
touched no socket at all.
"""

import ast
import os

import fake_klippy
import pytest

import plr
from plr import daemon_link, wizard


class FakeDaemon:
    """Canned control-socket responses, per command, for the wizard.

    ``responses`` maps command -> ``{"ok","text","data"}``; ``errors``
    maps command -> a DaemonError to raise instead (daemon-down at that
    step).  Every call is recorded for ordering/args assertions.
    """

    def __init__(self, responses=None, errors=None):
        self.responses = responses or {}
        self.errors = errors or {}
        self.calls = []

    def call(self, cmd, args=None, timeout=None):
        self.calls.append((cmd, args, timeout))
        if cmd in self.errors:
            raise self.errors[cmd]
        return self.responses.get(cmd, {"ok": True, "text": "", "data": {}})


def pending_recovery(
    file="/home/pi/printer_data/gcodes/bench.gcode",
    file_position=1048576,
    file_size=4194304,
    percent=25.0,
    crash_class="HardPowerLoss { residual_ms: 12 }",
    **overrides,
):
    """A serialized ``PendingRecovery`` exactly as the daemon emits it.

    MIRRORS THE PRODUCER: crates/plrd/src/detect.rs ``PendingRecovery``
    has a plain serde derive (no rename_all), so the JSON keys are the
    struct field names verbatim.  ``percent`` is a 0-100 percentage
    (ctrlsock.rs renders it ``(~{p:.0}%)``), ``file`` an absolute path.
    ``overrides`` lets a test null/drop a field to prove the defensive
    reads.
    """
    obj = {
        "detected_wall_ns": 1737000000000000000,
        "file": file,
        "file_position": file_position,
        "file_size": file_size,
        "percent": percent,
        "crash_class": crash_class,
    }
    obj.update(overrides)
    return obj


def status_data(pending=None):
    """The full ``status`` data map as the daemon builds it.

    MIRRORS THE PRODUCER: crates/plrd/src/ctrlsock.rs ``build_status``
    inserts exactly these keys — ``wal_dir``, ``segments`` (int|null),
    ``wal_bytes``, ``heartbeat_age_s`` (float|null), ``pending``
    (**explicit null** when nothing is pending, else the serialized
    PendingRecovery), ``machine_mode``, ``machine_validation``.
    """
    return {
        "wal_dir": "/var/lib/plrd/wal",
        "segments": 3,
        "wal_bytes": 40960,
        "heartbeat_age_s": 0.4,
        "pending": pending,
        "machine_mode": "plr",
        "machine_validation": "OK (;TYPE: annotation check deferred to recover time)",
    }


def _status(pending=None, text="plrd armed", data=None):
    """A ``status`` response; ``pending`` defaults to null (nothing pending)."""
    return {
        "ok": True,
        "text": text,
        "data": status_data(pending) if data is None else data,
    }


def _status_pending(**kwargs):
    """A ``status`` response carrying a pending recovery."""
    return _status(pending=pending_recovery(**kwargs))


def _dryrun(ok=True, text="PLAN: resume bench.gcode at layer 42", **data):
    """A ``recover_dryrun`` response.

    MIRRORS THE PRODUCER: crates/plrd/src/ctrlsock.rs
    ``cmd_recover_dryrun`` returns ``data: {"outcome": <tag>}``; the
    clean-nozzle flag is added on top by the tests that exercise it.
    """
    payload = {"outcome": "pending-recovery"}
    payload.update(data)
    return {"ok": ok, "text": text, "data": payload}


def _new_responses(gcode):
    """Return a function yielding responses appended since it was called."""
    start = len(gcode.responses)

    def since():
        return gcode.responses[start:]

    return since


@pytest.fixture
def step(run_cmd, pump):
    """Run one wizard command and deliver the worker result it started.

    ``calls`` is how many plrd round trips the command hands off — 0 for
    the commands that touch no socket.  ``pump`` asserts exactly that many
    callbacks arrive, so an accidental extra (or missing) daemon call
    fails the test rather than passing quietly.
    """

    def run(name, calls=1, **params):
        gcode = run_cmd(name, **params)
        assert pump(calls, timeout=5.0) == calls
        return gcode

    return run


@pytest.fixture
def plugin_with_clean_macro(fake_printer, plr_config):
    """A plugin whose config HAS the [gcode_macro CLEAN_NOZZLE] section.

    The default ``plugin`` fixture has no such section, so
    ``clean_nozzle_macro_available`` is False there — the clean-nozzle
    branch needs both variants to cover the agree/disagree cases.
    """
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    return plr.load_config(
        plr_config(sections={"gcode_macro CLEAN_NOZZLE": {"gcode": "M117 clean"}})
    )


# --- literal action-string builders (the researched Mainsail spec) ---


def test_action_string_literals():
    assert wizard.action_prompt_begin("Power-loss recovery") == (
        "action:prompt_begin Power-loss recovery"
    )
    assert wizard.action_prompt_text("Hello") == "action:prompt_text Hello"
    assert wizard.action_prompt_show() == "action:prompt_show"
    assert wizard.action_prompt_end() == "action:prompt_end"
    # <label>|<gcode?>|<color?> — pipe-separated, color forces the middle.
    assert wizard.action_prompt_button("Go", "PLR_WIZARD_DRYRUN", "primary") == (
        "action:prompt_button Go|PLR_WIZARD_DRYRUN|primary"
    )
    assert wizard.action_prompt_button("Go", "PLR_WIZARD_DRYRUN") == (
        "action:prompt_button Go|PLR_WIZARD_DRYRUN"
    )
    assert wizard.action_prompt_button("Go") == "action:prompt_button Go"
    assert wizard.action_prompt_button("Go", None, "error") == (
        "action:prompt_button Go||error"
    )
    assert wizard.action_prompt_footer_button(
        "Cancel", "PLR_WIZARD_CANCEL", "error"
    ) == ("action:prompt_footer_button Cancel|PLR_WIZARD_CANCEL|error")


def test_summarize_reads_real_pending_recovery_fields():
    # detect.rs PendingRecovery: file (abs path), file_position, percent
    # (already 0-100), crash_class.  The basename is shown, not the path.
    assert wizard._summarize(pending_recovery()) == [
        "Interrupted print: bench.gcode",
        "Progress at power loss: ~25%",
        "Resume point: byte 1048576",
        "Crash classification: HardPowerLoss { residual_ms: 12 }",
    ]


def test_summarize_omits_null_percent_without_fabricating():
    # percent is Option<f64> in detect.rs — null must omit the clause.
    lines = wizard._summarize(pending_recovery(percent=None))
    assert not any("Progress" in line for line in lines)
    assert "Interrupted print: bench.gcode" in lines


def test_summarize_tolerates_missing_and_retyped_fields():
    # A renamed/dropped field must omit its clause, never crash.
    assert wizard._summarize({}) == ["An interrupted print is ready to resume."]
    lines = wizard._summarize(
        {"file": 42, "percent": "lots", "file_position": None, "crash_class": ""}
    )
    assert lines == ["An interrupted print is ready to resume."]


def test_summarize_keeps_pathless_file_name():
    assert "Interrupted print: bench.gcode" in wizard._summarize(
        pending_recovery(file="bench.gcode")
    )


# --- START ------------------------------------------------------------


def test_start_offers_recovery_with_ordered_prompt(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_START")
    assert since() == [
        "PLR wizard: asking plrd whether a recovery is pending...",
        "Power-loss recovery available.",
        "action:prompt_begin Power-loss recovery",
        "action:prompt_text Interrupted print: bench.gcode",
        "action:prompt_text Progress at power loss: ~25%",
        "action:prompt_text Resume point: byte 1048576",
        "action:prompt_text Crash classification: HardPowerLoss { residual_ms: 12 }",
        "action:prompt_text Attempt recovery, or dismiss this prompt.",
        "action:prompt_button Attempt recovery|PLR_WIZARD_DRYRUN|primary",
        "action:prompt_footer_button Dismiss|PLR_WIZARD_CANCEL|error",
        "action:prompt_show",
        "Console: run PLR_WIZARD_DRYRUN to review the recovery plan, "
        "or PLR_WIZARD_CANCEL to dismiss.",
    ]
    assert plugin.wizard.is_active() is True
    assert plugin.get_status(100.0)["wizard_active"] is True
    assert plugin.daemon.calls == [("status", None, daemon_link.STATUS_TIMEOUT)]


def test_start_no_pending_recovery_uses_explicit_null(plugin, step, fake_printer):
    # build_status ALWAYS inserts "pending"; nothing-pending is a JSON
    # null VALUE, not an absent key.  Pin that exact shape.
    response = _status()
    assert "pending" in response["data"] and response["data"]["pending"] is None
    plugin.daemon = FakeDaemon(responses={"status": response})
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_START")
    out = "\n".join(since())
    assert "no power-loss recovery is pending" in out
    assert not any(line.startswith("action:") for line in since())
    assert plugin.wizard.is_active() is False


def test_start_absent_pending_key_treated_as_nothing(plugin, step, fake_printer):
    # Belt-and-braces: a daemon that omits the key entirely (or renames
    # it) must not be read as a pending recovery either.
    plugin.daemon = FakeDaemon(
        responses={"status": {"ok": True, "text": "", "data": {}}}
    )
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_START")
    assert "no power-loss recovery is pending" in "\n".join(since())
    assert plugin.wizard.is_active() is False


def test_start_non_object_pending_treated_as_nothing(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(responses={"status": _status(pending="yes")})
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_START")
    assert "no power-loss recovery is pending" in "\n".join(since())
    assert plugin.wizard.is_active() is False


def test_start_empty_pending_object_still_offers_recovery(plugin, step, fake_printer):
    # A pending object whose fields are all unreadable is STILL a pending
    # recovery — offer it with the honest generic summary.
    plugin.daemon = FakeDaemon(responses={"status": _status(pending={})})
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_START")
    assert "action:prompt_text An interrupted print is ready to resume." in since()
    assert plugin.wizard.is_active() is True


def test_double_start_reshows_current_prompt(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    # calls=0: the re-show is local, so it does not talk to plrd at all.
    step("PLR_WIZARD_START", calls=0)
    lines = since()
    assert lines[0] == "PLR wizard already in progress — re-showing the current prompt."
    assert "action:prompt_begin Power-loss recovery" in lines
    assert "action:prompt_button Attempt recovery|PLR_WIZARD_DRYRUN|primary" in lines
    # The re-show does NOT re-query the daemon.
    assert plugin.daemon.calls == [("status", None, daemon_link.STATUS_TIMEOUT)]


# --- DRYRUN branches --------------------------------------------------


def test_dryrun_requires_clean_shows_clean_prompt(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(requires_clean_nozzle_confirmation=True),
        }
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    lines = since()
    assert lines[0].startswith("PLR wizard: asking plrd for the recovery plan")
    assert lines[1] == "PLAN: resume bench.gcode at layer 42"
    assert "action:prompt_begin Power-loss recovery" in lines
    assert "action:prompt_button Nozzle is clean|PLR_WIZARD_CONFIRM_CLEAN|primary" in (
        lines
    )
    assert "action:prompt_footer_button It's dirty - abort|PLR_WIZARD_CANCEL|error" in (
        lines
    )
    # Fallback names the advancing console command.
    assert any(
        "PLR_WIZARD_CONFIRM_CLEAN" in line and not line.startswith("action:")
        for line in lines
    )
    assert plugin.daemon.calls[-1] == (
        "recover_dryrun",
        None,
        daemon_link.DRYRUN_TIMEOUT,
    )


def test_clean_flag_tri_state_read_at_both_contract_locations():
    # The Rust side may land the flag top level in `data` or nested under
    # a `plan` object; a valid boolean at the top level wins.  Anything
    # unreadable is None, which the decision layer resolves conservatively.
    assert wizard._clean_flag({"requires_clean_nozzle_confirmation": True}) is True
    assert wizard._clean_flag({"requires_clean_nozzle_confirmation": False}) is False
    assert (
        wizard._clean_flag({"plan": {"requires_clean_nozzle_confirmation": True}})
        is True
    )
    assert (
        wizard._clean_flag({"plan": {"requires_clean_nozzle_confirmation": False}})
        is False
    )
    # Top level wins when both are present and disagree.
    assert (
        wizard._clean_flag(
            {
                "requires_clean_nozzle_confirmation": False,
                "plan": {"requires_clean_nozzle_confirmation": True},
            }
        )
        is False
    )
    # Absent, non-boolean, or a non-object plan -> unreadable (None).
    assert wizard._clean_flag({"outcome": "pending-recovery"}) is None
    assert wizard._clean_flag({"plan": "nope"}) is None
    assert wizard._clean_flag({"requires_clean_nozzle_confirmation": "yes"}) is None
    assert wizard._clean_flag({"requires_clean_nozzle_confirmation": None}) is None
    # A non-boolean at the top level still falls back to a valid nested one.
    assert (
        wizard._clean_flag(
            {
                "requires_clean_nozzle_confirmation": "yes",
                "plan": {"requires_clean_nozzle_confirmation": True},
            }
        )
        is True
    )


@pytest.mark.parametrize(
    "flag,available,expected",
    [
        # Skip ONLY when both sources agree cleaning is automatic.
        pytest.param(False, True, (False, None), id="false+macro-skips"),
        # Daemon says nothing cleans it -> ask.
        pytest.param(True, True, (True, wizard._ASK_NO_MACRO), id="true-asks"),
        pytest.param(True, False, (True, wizard._ASK_NO_MACRO), id="true-nomacro-asks"),
        # FAIL-SAFE: unknown flag always asks, macro or not.
        pytest.param(None, True, (True, wizard._ASK_UNKNOWN), id="absent-asks"),
        pytest.param(
            None, False, (True, wizard._ASK_UNKNOWN), id="absent-nomacro-asks"
        ),
        # Sources disagree: daemon says auto, plugin sees no macro -> ask.
        pytest.param(False, False, (True, wizard._ASK_DISAGREE), id="disagree-asks"),
    ],
)
def test_clean_decision_matrix(flag, available, expected):
    assert wizard._clean_decision(flag, available, "CLEAN_NOZZLE") == expected


@pytest.mark.parametrize(
    "data",
    [
        pytest.param({"requires_clean_nozzle_confirmation": True}, id="top-level-true"),
        pytest.param(
            {"plan": {"requires_clean_nozzle_confirmation": True}}, id="nested-true"
        ),
        # The fail-safe cases: a daemon predating the flag, or a field
        # that never lands / lands unreadable, must ASK — never silently
        # promise an automatic clean that nothing performs.
        pytest.param({}, id="absent"),
        pytest.param({"requires_clean_nozzle_confirmation": "yes"}, id="non-boolean"),
        pytest.param({"plan": "nope"}, id="non-object-plan"),
    ],
)
def test_dryrun_asks_for_clean_confirmation(plugin, step, fake_printer, data):
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending(), "recover_dryrun": _dryrun(**data)}
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    lines = since()
    assert "action:prompt_button Nozzle is clean|PLR_WIZARD_CONFIRM_CLEAN|primary" in (
        lines
    )
    # It must never claim an automatic clean on the ask branch.
    assert not any("will run to clean the nozzle" in line for line in lines)


def test_dryrun_absent_flag_explains_why_it_asks(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending(), "recover_dryrun": _dryrun()}
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    assert any("did not report whether the nozzle" in line for line in since())


def test_dryrun_sources_disagree_asks_and_says_why(plugin, step, fake_printer):
    # plrd says cleaning is automatic, but this printer has no
    # [gcode_macro CLEAN_NOZZLE] — the conservative branch must win.
    assert plugin.clean_nozzle_macro_available is False
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(requires_clean_nozzle_confirmation=False),
        }
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    lines = since()
    assert "action:prompt_button Nozzle is clean|PLR_WIZARD_CONFIRM_CLEAN|primary" in (
        lines
    )
    assert any("the two disagree" in line for line in lines)
    assert any("[gcode_macro CLEAN_NOZZLE]" in line for line in lines)


@pytest.mark.parametrize(
    "data",
    [
        pytest.param({"requires_clean_nozzle_confirmation": False}, id="top-level"),
        pytest.param(
            {"plan": {"requires_clean_nozzle_confirmation": False}}, id="nested"
        ),
    ],
)
def test_dryrun_skips_to_execute_only_when_both_sources_agree(
    plugin_with_clean_macro, step, fake_printer, data
):
    plugin = plugin_with_clean_macro
    assert plugin.clean_nozzle_macro_available is True
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending(), "recover_dryrun": _dryrun(**data)}
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    lines = since()
    assert "action:prompt_text Execute the recovery plan? The printer WILL MOVE." in (
        lines
    )
    # The auto-clean note is grounded in the plugin's own config, not the
    # daemon boolean alone: it names the configured macro SECTION.
    assert (
        "action:prompt_text [gcode_macro CLEAN_NOZZLE] is configured and plrd "
        "reports it will run to clean the nozzle first." in lines
    )
    assert "action:prompt_button Execute|PLR_WIZARD_EXECUTE|primary" in lines
    assert not any("Nozzle is clean" in line for line in lines)


def test_dryrun_absent_flag_asks_even_when_macro_is_available(
    plugin_with_clean_macro, step, fake_printer
):
    # Redundant ask costs one click; skipping when nothing cleans the
    # nozzle corrupts the reference measurement. Unknown always asks.
    plugin = plugin_with_clean_macro
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending(), "recover_dryrun": _dryrun()}
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    assert "action:prompt_button Nozzle is clean|PLR_WIZARD_CONFIRM_CLEAN|primary" in (
        since()
    )


def test_dryrun_before_start_is_error(plugin, run_cmd):
    plugin.daemon = FakeDaemon()
    with pytest.raises(
        fake_klippy.FakeCommandError, match="run PLR_WIZARD_START first"
    ):
        run_cmd("PLR_WIZARD_DRYRUN")
    assert plugin.daemon.calls == []


def test_dryrun_failure_resets_and_ends_prompt(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(ok=False, text="machine validation failed"),
        }
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    lines = since()
    assert "machine validation failed" in lines
    assert "action:prompt_end" in lines
    # A failure a worker discovers lands as klippy's own error line
    # instead of a raise: there is no gcmd left to raise on.
    assert "dry run reported failure" in gcode.raw_responses[-1]
    assert plugin.wizard.is_active() is False


# --- CONFIRM_CLEAN / EXECUTE happy path -------------------------------


def test_full_happy_path_with_clean_confirm(plugin, step, run_cmd, pump, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(requires_clean_nozzle_confirmation=True),
            "recover_execute": {
                "ok": True,
                "text": "recovery complete",
                "data": {"outcome": "completed", "exit": 0},
            },
        }
    )
    step("PLR_WIZARD_START")
    step("PLR_WIZARD_DRYRUN")
    gcode = fake_printer.lookup_object("gcode")

    since_confirm = _new_responses(gcode)
    # calls=0: the clean confirmation is local; it must not touch plrd.
    step("PLR_WIZARD_CONFIRM_CLEAN", calls=0)
    confirm_lines = since_confirm()
    assert "action:prompt_text Execute the recovery plan? The printer WILL MOVE." in (
        confirm_lines
    )
    assert "action:prompt_button Execute|PLR_WIZARD_EXECUTE|primary" in confirm_lines
    # After a manual clean confirmation there is NO auto-clean macro note.
    assert not any("will run automatically" in line for line in confirm_lines)

    since_exec = _new_responses(gcode)
    run_cmd("PLR_WIZARD_EXECUTE")
    # The handler returns with the recovery still in flight: that IS the
    # fix.  Nothing about the outcome exists yet.
    started = since_exec()
    assert any("WILL MOVE" in line for line in started)
    assert "recovery complete" not in started
    assert plugin.wizard.state() == "running"
    assert plugin.recovery.state() == "running"
    assert plugin.get_status(100.0)["recovery_state"] == "running"

    since_report = _new_responses(gcode)
    assert pump() == 1
    assert since_report() == [
        "recovery complete",
        "action:prompt_end",
        "PLR recovery complete — plrd has resumed the print.",
    ]
    assert plugin.wizard.is_active() is False
    assert plugin.recovery.state() == "idle"
    assert plugin.daemon.calls[-1] == (
        "recover_execute",
        # `on_confirm: "ask"` — the argument that makes plrd's confirm
        # points reachable at all (ctrlsock.rs:612-627).
        {"confirm": True, "on_confirm": "ask"},
        daemon_link.EXECUTE_TIMEOUT,
    )


def test_execute_typed_failure_reports_remediation(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(),
            "recover_execute": {
                "ok": False,
                "text": "transcript mismatch [wal_gap] — re-run plrd verify",
                "data": {"outcome": "aborted-or-refused", "exit": 1},
            },
        }
    )
    step("PLR_WIZARD_START")
    step("PLR_WIZARD_DRYRUN")
    # Flag absent -> the fail-safe branch asks; confirm to reach execute.
    step("PLR_WIZARD_CONFIRM_CLEAN", calls=0)
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_EXECUTE")
    lines = since()
    assert "transcript mismatch [wal_gap] — re-run plrd verify" in lines
    assert "action:prompt_end" in lines
    assert any("did not complete" in line for line in lines)
    assert plugin.wizard.is_active() is False


# --- out-of-order guards ----------------------------------------------


def test_confirm_clean_out_of_order_is_error(plugin, step, run_cmd):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    step("PLR_WIZARD_START")  # state OFFERED, not CLEAN_CHECK
    with pytest.raises(fake_klippy.FakeCommandError, match="not awaiting"):
        run_cmd("PLR_WIZARD_CONFIRM_CLEAN")


def test_execute_out_of_order_is_error(plugin, step, run_cmd):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    step("PLR_WIZARD_START")  # state OFFERED, not EXECUTE
    with pytest.raises(fake_klippy.FakeCommandError, match="not ready to execute"):
        run_cmd("PLR_WIZARD_EXECUTE")
    assert not any(c[0] == "recover_execute" for c in plugin.daemon.calls)


# --- CANCEL -----------------------------------------------------------


def test_cancel_active_ends_prompt_and_resets(plugin, step, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_CANCEL")
    lines = since()
    assert lines[0] == "action:prompt_end"
    assert any("cancelled" in line for line in lines)
    assert plugin.wizard.is_active() is False


def test_cancel_when_idle_is_benign(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon()
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_CANCEL")
    lines = since()
    assert "action:prompt_end" in lines
    assert any("nothing to cancel" in line for line in lines)


# --- daemon-down at each step -----------------------------------------


def test_daemon_down_at_start(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(
        errors={"status": daemon_link.DaemonError("plrd not reachable at /x")}
    )
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_START")
    assert "action:prompt_end" in since()
    assert "not reachable" in gcode.raw_responses[-1]
    assert plugin.wizard.is_active() is False


def test_daemon_down_at_dryrun(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending()},
        errors={"recover_dryrun": daemon_link.DaemonError("plrd timed out")},
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    assert "action:prompt_end" in since()
    assert "timed out" in gcode.raw_responses[-1]
    assert plugin.wizard.is_active() is False


def test_daemon_down_at_execute(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending(), "recover_dryrun": _dryrun()},
        errors={"recover_execute": daemon_link.DaemonError("plrd closed connection")},
    )
    step("PLR_WIZARD_START")
    step("PLR_WIZARD_DRYRUN")
    step("PLR_WIZARD_CONFIRM_CLEAN", calls=0)
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_EXECUTE")
    lines = since()
    assert "action:prompt_end" in lines
    # Losing contact mid-execute is NOT reported as "the recovery failed":
    # plrd does not need this plugin in order to keep going.
    joined = "\n".join(lines)
    assert "closed connection" in joined
    assert "may still be executing" in joined
    assert plugin.wizard.is_active() is False


# --- graceful degradation: every prompt's fallback names a command ----


def test_every_prompt_fallback_names_next_command(plugin, step, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(requires_clean_nozzle_confirmation=True),
            "recover_execute": {"ok": True, "text": "done", "data": {}},
        }
    )
    gcode = fake_printer.lookup_object("gcode")

    def fallback_lines(lines):
        # Plain-text lines that follow a prompt_show in the same step.
        assert "action:prompt_show" in lines
        idx = lines.index("action:prompt_show")
        return [ln for ln in lines[idx + 1 :] if not ln.startswith("action:")]

    since = _new_responses(gcode)
    step("PLR_WIZARD_START")
    assert any("PLR_WIZARD_DRYRUN" in ln for ln in fallback_lines(since()))

    since = _new_responses(gcode)
    step("PLR_WIZARD_DRYRUN")
    assert any("PLR_WIZARD_CONFIRM_CLEAN" in ln for ln in fallback_lines(since()))

    since = _new_responses(gcode)
    step("PLR_WIZARD_CONFIRM_CLEAN", calls=0)
    assert any("PLR_WIZARD_EXECUTE" in ln for ln in fallback_lines(since()))


# --- no motion g-code ever leaves the wizard --------------------------


# The four modules that make up the recovery UI.  None of them may send
# motion: the ONLY machine motion in the flow is plrd's own, over the
# control socket.
_UI_MODULES = ("wizard.py", "recovery.py", "confirm_ui.py", "prompts.py")


def _module_path(name):
    return os.path.join(os.path.dirname(wizard.__file__), name)


def _identifiers(path):
    """Every attribute/name IDENTIFIER a module uses, from its AST.

    Parsed rather than grepped so prose in a docstring cannot trip the
    check (these modules discuss ``run_script`` at length, precisely
    because they must never call it) and so a real call cannot hide from
    it inside a concatenated string.
    """
    tree = ast.parse(open(path, encoding="utf-8").read())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, ast.Name):
            names.add(node.id)
    return names


def _lookup_object_targets(path):
    """The literal printer objects a module resolves."""
    tree = ast.parse(open(path, encoding="utf-8").read())
    targets = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", "")
        if name != "lookup_object" or not node.args:
            continue
        first = node.args[0]
        targets.add(first.value if isinstance(first, ast.Constant) else "<non-literal>")
    return targets


@pytest.mark.parametrize("module", _UI_MODULES)
def test_recovery_ui_never_sends_motion_gcode(module):
    forbidden = {
        # G-code dispatch of any kind.  run_script would ALSO take the
        # g-code mutex (klippy/gcode.py:239-241) and queue behind plrd's
        # own motion, which is why not even a harmless M117 belongs here.
        "run_script",
        "run_script_from_command",
        # Direct toolhead commands.
        "manual_move",
        "get_position",
        "set_position",
        "dwell",
        "wait_moves",
    }
    used = _identifiers(_module_path(module))
    assert not (used & forbidden), sorted(used & forbidden)
    # The only printer object the recovery UI may resolve is the g-code
    # dispatcher, and only for its OUTPUT calls.
    assert _lookup_object_targets(_module_path(module)) <= {"gcode"}


# --- PLR_SETUP_WIZARD -------------------------------------------------


def test_setup_wizard_tap_lists_actionable_buttons(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon()
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_SETUP_WIZARD")
    lines = since()
    # Reuses the PLR_SETUP report (single source of truth for the checks).
    assert any("PLR commissioning report" in ln for ln in lines)
    assert "action:prompt_begin PLR commissioning" in lines
    # Not attested -> attestation button; tap -> probe-test button.
    assert (
        "action:prompt_button Attest self-locking Z|PLR_SETUP ACCEPT_SELF_LOCKING_Z=1"
        "|primary" in lines
    )
    assert (
        "action:prompt_button Run probe test (moves)|PLR_PROBE_TEST START=1|warning"
        in lines
    )
    assert "action:prompt_footer_button SAVE_CONFIG|SAVE_CONFIG|primary" in lines
    # Fallback names every command including SAVE_CONFIG.
    joined = "\n".join(lines)
    assert "PLR_SETUP ACCEPT_SELF_LOCKING_Z=1" in joined
    assert "SAVE_CONFIG" in joined
    # The setup wizard is independent of the recovery state machine.
    assert plugin.wizard.is_active() is False


def test_setup_wizard_drag_machine_uses_drag_buttons(fake_printer, plr_config, run_cmd):
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    )
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_SETUP_WIZARD")
    lines = since()
    assert (
        "action:prompt_button Measure noise floor (moves)|PLR_NOISE_TEST START=1"
        "|warning" in lines
    )
    assert any("PLR_DRAG_CALIBRATE START=1" in ln for ln in lines)
    assert not any("PLR_PROBE_TEST" in ln for ln in lines)


def test_setup_wizard_hides_attestation_when_already_attested(
    fake_printer, plr_config, run_cmd
):
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    plr.load_config(plr_config(options={"self_locking_z": "True"}))
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_SETUP_WIZARD")
    lines = since()
    assert not any("Attest self-locking Z" in ln for ln in lines)


# --- dialog termination: no wizard may leave a prompt open ------------


def test_setup_wizard_offers_a_closing_footer_button(plugin, run_cmd, fake_printer):
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_SETUP_WIZARD")
    lines = since()
    # A dialog with no prompt_end path sits over the UI forever.
    assert "action:prompt_footer_button Close|PLR_WIZARD_CLOSE" in lines
    # ...and the plain-text fallback names it for prompt-less clients.
    assert any(
        "PLR_WIZARD_CLOSE" in ln and not ln.startswith("action:") for ln in lines
    )


def test_setup_wizard_closes_previous_dialog_before_reopening(
    plugin, run_cmd, fake_printer
):
    run_cmd("PLR_SETUP_WIZARD")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_SETUP_WIZARD")
    lines = since()
    # prompt_end precedes the new prompt_begin: prompts never stack.
    assert lines[0] == "action:prompt_end"
    assert lines.index("action:prompt_end") < lines.index(
        "action:prompt_begin PLR commissioning"
    )


def test_wizard_close_emits_prompt_end(plugin, run_cmd, fake_printer):
    run_cmd("PLR_SETUP_WIZARD")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_CLOSE")
    assert since() == ["action:prompt_end", "PLR: dialog closed."]


def test_wizard_close_does_not_abandon_an_active_recovery(plugin, step, fake_printer):
    # Closing the dialog is display-only: an in-flight recovery survives,
    # so a stray Close cannot silently drop the operator out of the flow.
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    step("PLR_WIZARD_CLOSE", calls=0)
    lines = since()
    assert lines[0] == "action:prompt_end"
    assert any("still in progress" in ln for ln in lines)
    assert plugin.wizard.is_active() is True
    # ...and START re-shows it rather than starting a second flow.
    step("PLR_WIZARD_START", calls=0)
    assert plugin.daemon.calls == [("status", None, daemon_link.STATUS_TIMEOUT)]


@pytest.mark.parametrize(
    "terminal",
    [
        pytest.param(["PLR_WIZARD_CANCEL"], id="cancel"),
        pytest.param(["PLR_WIZARD_CLOSE"], id="close"),
        pytest.param(
            ["PLR_WIZARD_DRYRUN", "PLR_WIZARD_CONFIRM_CLEAN", "PLR_WIZARD_EXECUTE"],
            id="execute",
        ),
    ],
)
def test_every_recovery_terminal_path_emits_prompt_end(
    plugin, step, run_cmd, pump, fake_printer, terminal
):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(requires_clean_nozzle_confirmation=True),
            "recover_execute": {
                "ok": True,
                "text": "done",
                "data": {"outcome": "completed", "exit": 0},
            },
        }
    )
    step("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    for command in terminal:
        run_cmd(command)
        # Deliver whatever that step handed off (0 for the local ones).
        pump(0, timeout=1.0)
    assert "action:prompt_end" in since()
