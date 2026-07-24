"""The prompt-driven recovery and commissioning wizards.

Drives the ``PLR_WIZARD_*`` state machine and ``PLR_SETUP_WIZARD`` through
the registered console commands against a fake daemon returning canned
socket responses.  Asserts the ORDERED respond sequences (action lines +
their plain-text fallbacks), the exact literal action strings against the
researched Mainsail spec, every branch (clean-confirm true/false/absent),
double-START, and daemon-down at each step (clear error + state reset).
"""

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


def test_start_offers_recovery_with_ordered_prompt(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_START")
    assert since() == [
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


def test_start_no_pending_recovery_uses_explicit_null(plugin, run_cmd, fake_printer):
    # build_status ALWAYS inserts "pending"; nothing-pending is a JSON
    # null VALUE, not an absent key.  Pin that exact shape.
    response = _status()
    assert "pending" in response["data"] and response["data"]["pending"] is None
    plugin.daemon = FakeDaemon(responses={"status": response})
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_START")
    out = "\n".join(since())
    assert "no power-loss recovery is pending" in out
    assert not any(line.startswith("action:") for line in since())
    assert plugin.wizard.is_active() is False


def test_start_absent_pending_key_treated_as_nothing(plugin, run_cmd, fake_printer):
    # Belt-and-braces: a daemon that omits the key entirely (or renames
    # it) must not be read as a pending recovery either.
    plugin.daemon = FakeDaemon(
        responses={"status": {"ok": True, "text": "", "data": {}}}
    )
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_START")
    assert "no power-loss recovery is pending" in "\n".join(since())
    assert plugin.wizard.is_active() is False


def test_start_non_object_pending_treated_as_nothing(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(responses={"status": _status(pending="yes")})
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_START")
    assert "no power-loss recovery is pending" in "\n".join(since())
    assert plugin.wizard.is_active() is False


def test_start_empty_pending_object_still_offers_recovery(
    plugin, run_cmd, fake_printer
):
    # A pending object whose fields are all unreadable is STILL a pending
    # recovery — offer it with the honest generic summary.
    plugin.daemon = FakeDaemon(responses={"status": _status(pending={})})
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_START")
    assert "action:prompt_text An interrupted print is ready to resume." in since()
    assert plugin.wizard.is_active() is True


def test_double_start_reshows_current_prompt(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    run_cmd("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_START")
    lines = since()
    assert lines[0] == "PLR wizard already in progress — re-showing the current prompt."
    assert "action:prompt_begin Power-loss recovery" in lines
    assert "action:prompt_button Attempt recovery|PLR_WIZARD_DRYRUN|primary" in lines
    # The re-show does NOT re-query the daemon.
    assert plugin.daemon.calls == [("status", None, daemon_link.STATUS_TIMEOUT)]


# --- DRYRUN branches --------------------------------------------------


def test_dryrun_requires_clean_shows_clean_prompt(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(requires_clean_nozzle_confirmation=True),
        }
    )
    run_cmd("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_DRYRUN")
    lines = since()
    assert lines[0] == "PLAN: resume bench.gcode at layer 42"
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
        daemon_link.RECOVER_TIMEOUT,
    )


def test_requires_clean_read_at_both_contract_locations():
    # The Rust side may land the flag top level in `data` or nested under
    # a `plan` object; both must be honoured, absent in both -> False.
    assert wizard._requires_clean({"requires_clean_nozzle_confirmation": True}) is True
    assert (
        wizard._requires_clean({"plan": {"requires_clean_nozzle_confirmation": True}})
        is True
    )
    assert (
        wizard._requires_clean({"requires_clean_nozzle_confirmation": False}) is False
    )
    assert (
        wizard._requires_clean({"plan": {"requires_clean_nozzle_confirmation": False}})
        is False
    )
    # Top level wins when both are present and disagree.
    assert (
        wizard._requires_clean(
            {
                "requires_clean_nozzle_confirmation": False,
                "plan": {"requires_clean_nozzle_confirmation": True},
            }
        )
        is False
    )
    # Absent in both, and a non-object plan, read as False.
    assert wizard._requires_clean({"outcome": "pending-recovery"}) is False
    assert wizard._requires_clean({"plan": "nope"}) is False


def test_dryrun_nested_plan_clean_flag_shows_clean_prompt(
    plugin, run_cmd, fake_printer
):
    # The in-flight Rust change may nest the flag under `plan`.
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(
                plan={"requires_clean_nozzle_confirmation": True}
            ),
        }
    )
    run_cmd("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_DRYRUN")
    lines = since()
    assert "action:prompt_button Nozzle is clean|PLR_WIZARD_CONFIRM_CLEAN|primary" in (
        lines
    )


@pytest.mark.parametrize(
    "data",
    [
        {"requires_clean_nozzle_confirmation": False},
        {"plan": {"requires_clean_nozzle_confirmation": False}},
        {},
    ],
)
def test_dryrun_no_clean_skips_to_execute(plugin, run_cmd, fake_printer, data):
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending(), "recover_dryrun": _dryrun(**data)}
    )
    run_cmd("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_DRYRUN")
    lines = since()
    assert "action:prompt_text Execute the recovery plan? The printer WILL MOVE." in (
        lines
    )
    # The auto-clean note names the configured macro.
    assert any("CLEAN_NOZZLE macro will run automatically" in line for line in lines)
    assert "action:prompt_button Execute|PLR_WIZARD_EXECUTE|primary" in lines
    assert not any("Nozzle is clean" in line for line in lines)


def test_dryrun_before_start_is_error(plugin, run_cmd):
    plugin.daemon = FakeDaemon()
    with pytest.raises(
        fake_klippy.FakeCommandError, match="run PLR_WIZARD_START first"
    ):
        run_cmd("PLR_WIZARD_DRYRUN")
    assert plugin.daemon.calls == []


def test_dryrun_failure_resets_and_ends_prompt(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(ok=False, text="machine validation failed"),
        }
    )
    run_cmd("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    with pytest.raises(fake_klippy.FakeCommandError, match="dry run reported failure"):
        run_cmd("PLR_WIZARD_DRYRUN")
    lines = since()
    assert "machine validation failed" in lines
    assert "action:prompt_end" in lines
    assert plugin.wizard.is_active() is False


# --- CONFIRM_CLEAN / EXECUTE happy path -------------------------------


def test_full_happy_path_with_clean_confirm(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(requires_clean_nozzle_confirmation=True),
            "recover_execute": {"ok": True, "text": "recovery complete", "data": {}},
        }
    )
    run_cmd("PLR_WIZARD_START")
    run_cmd("PLR_WIZARD_DRYRUN")
    gcode = fake_printer.lookup_object("gcode")

    since_confirm = _new_responses(gcode)
    run_cmd("PLR_WIZARD_CONFIRM_CLEAN")
    confirm_lines = since_confirm()
    assert "action:prompt_text Execute the recovery plan? The printer WILL MOVE." in (
        confirm_lines
    )
    assert "action:prompt_button Execute|PLR_WIZARD_EXECUTE|primary" in confirm_lines
    # After a manual clean confirmation there is NO auto-clean macro note.
    assert not any("will run automatically" in line for line in confirm_lines)

    since_exec = _new_responses(gcode)
    run_cmd("PLR_WIZARD_EXECUTE")
    exec_lines = since_exec()
    assert exec_lines == [
        "recovery complete",
        "action:prompt_end",
        "PLR recovery complete — resuming print.",
    ]
    assert plugin.wizard.is_active() is False
    assert plugin.daemon.calls[-1] == (
        "recover_execute",
        {"confirm": True},
        daemon_link.RECOVER_TIMEOUT,
    )


def test_execute_typed_failure_reports_remediation(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={
            "status": _status_pending(),
            "recover_dryrun": _dryrun(),
            "recover_execute": {
                "ok": False,
                "text": "transcript mismatch [wal_gap] — re-run plrd verify",
                "data": {},
            },
        }
    )
    run_cmd("PLR_WIZARD_START")
    run_cmd("PLR_WIZARD_DRYRUN")  # no clean confirm -> execute prompt
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    with pytest.raises(fake_klippy.FakeCommandError, match="did not complete"):
        run_cmd("PLR_WIZARD_EXECUTE")
    lines = since()
    assert "transcript mismatch [wal_gap] — re-run plrd verify" in lines
    assert "action:prompt_end" in lines
    assert plugin.wizard.is_active() is False


# --- out-of-order guards ----------------------------------------------


def test_confirm_clean_out_of_order_is_error(plugin, run_cmd):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    run_cmd("PLR_WIZARD_START")  # state OFFERED, not CLEAN_CHECK
    with pytest.raises(fake_klippy.FakeCommandError, match="not awaiting"):
        run_cmd("PLR_WIZARD_CONFIRM_CLEAN")


def test_execute_out_of_order_is_error(plugin, run_cmd):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    run_cmd("PLR_WIZARD_START")  # state OFFERED, not EXECUTE
    with pytest.raises(fake_klippy.FakeCommandError, match="not ready to execute"):
        run_cmd("PLR_WIZARD_EXECUTE")
    assert not any(c[0] == "recover_execute" for c in plugin.daemon.calls)


# --- CANCEL -----------------------------------------------------------


def test_cancel_active_ends_prompt_and_resets(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(responses={"status": _status_pending()})
    run_cmd("PLR_WIZARD_START")
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


def test_daemon_down_at_start(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(
        errors={"status": daemon_link.DaemonError("plrd not reachable at /x")}
    )
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    with pytest.raises(fake_klippy.FakeCommandError, match="not reachable"):
        run_cmd("PLR_WIZARD_START")
    assert "action:prompt_end" in since()
    assert plugin.wizard.is_active() is False


def test_daemon_down_at_dryrun(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending()},
        errors={"recover_dryrun": daemon_link.DaemonError("plrd timed out")},
    )
    run_cmd("PLR_WIZARD_START")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    with pytest.raises(fake_klippy.FakeCommandError, match="timed out"):
        run_cmd("PLR_WIZARD_DRYRUN")
    assert "action:prompt_end" in since()
    assert plugin.wizard.is_active() is False


def test_daemon_down_at_execute(plugin, run_cmd, fake_printer):
    plugin.daemon = FakeDaemon(
        responses={"status": _status_pending(), "recover_dryrun": _dryrun()},
        errors={"recover_execute": daemon_link.DaemonError("plrd closed connection")},
    )
    run_cmd("PLR_WIZARD_START")
    run_cmd("PLR_WIZARD_DRYRUN")
    gcode = fake_printer.lookup_object("gcode")
    since = _new_responses(gcode)
    with pytest.raises(fake_klippy.FakeCommandError, match="closed connection"):
        run_cmd("PLR_WIZARD_EXECUTE")
    assert "action:prompt_end" in since()
    assert plugin.wizard.is_active() is False


# --- graceful degradation: every prompt's fallback names a command ----


def test_every_prompt_fallback_names_next_command(plugin, run_cmd, fake_printer):
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
    run_cmd("PLR_WIZARD_START")
    assert any("PLR_WIZARD_DRYRUN" in ln for ln in fallback_lines(since()))

    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_DRYRUN")
    assert any("PLR_WIZARD_CONFIRM_CLEAN" in ln for ln in fallback_lines(since()))

    since = _new_responses(gcode)
    run_cmd("PLR_WIZARD_CONFIRM_CLEAN")
    assert any("PLR_WIZARD_EXECUTE" in ln for ln in fallback_lines(since()))


# --- no motion g-code ever leaves the wizard --------------------------


def test_wizard_never_sends_motion_gcode():
    # The wizard's only machine-motion path is the daemon's recover_execute;
    # it must never run_script/dispatch g-code or command the toolhead.
    src = open(
        os.path.join(os.path.dirname(wizard.__file__), "wizard.py"), encoding="utf-8"
    ).read()
    for forbidden in (
        "run_script",
        "run_script_from_command",
        "manual_move",
        'lookup_object("toolhead")',
        "lookup_object('toolhead')",
        "get_position",
    ):
        assert forbidden not in src, forbidden


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
