"""Prompt-driven recovery and commissioning wizards.

Two console-command-driven flows that render as interactive dialogs on
clients that support Klipper's *action prompt* protocol (Mainsail,
Fluidd, KlipperScreen, OctoApp) and degrade to plain console text
everywhere else:

* the **recovery wizard** (``PLR_WIZARD_*``) — a small state machine that
  walks an operator through a pending power-loss recovery: offer → review
  plan → (confirm the nozzle is clean) → execute → answer plrd's
  confirm-points until it finishes;
* the **commissioning wizard** (``PLR_SETUP_WIZARD``) — a one-shot dialog
  that reuses the exact ``PLR_SETUP`` report and adds a button per
  remaining commissioning step.

Prompts are sugar; console commands are the contract.  Every prompt this
module emits is paired with plain-text fallback lines that name the exact
console command which advances the flow, so a client without prompt
support is never stuck.  The action-line builders live in
:mod:`plr.prompts` (with the wire format, the client-support notes and the
portability rule) and are re-exported here.

NO WIZARD COMMAND BLOCKS THE REACTOR.  Every daemon call this module makes
runs on a worker thread and reports from a reactor callback
(:mod:`plr.daemon_worker`, whose docstring carries the klippy evidence:
a blocking socket read inside a g-code handler stalls klippy's single
reactor thread, which switches the heaters off after ~5 s and risks an
MCU-side shutdown after 3 s).  Handlers therefore print "asking plrd ..."
and return; the answer arrives later, in :meth:`RecoveryWizard._offer` /
:meth:`RecoveryWizard._plan`.

Because a reactor callback has no ``gcmd`` to raise on — and because an
exception escaping one is a printer shutdown (klippy/klippy.py:170-186) —
failures discovered by a worker are reported with
``daemon_link.respond_error`` (klippy's own ``!!`` convention) instead of
``gcmd.error``.  Everything decided synchronously in a handler still
raises, so console/macro semantics are unchanged there.

SAFETY INVARIANT: no wizard command ever sends motion g-code to the
printer.  The ONLY machine motion a wizard triggers is the daemon's own
``recover_execute`` over the control socket; recovery prompt buttons fire
other ``PLR_*`` console commands, and the setup wizard's buttons fire the
existing ``PLR_SETUP`` / ``PLR_*_TEST`` / ``SAVE_CONFIG`` commands (each of
which owns its own consent + gates).  This module calls ``plugin.daemon``
(off-reactor), ``gcode.respond_info`` and nothing else — in particular it
never calls ``run_script``, which would queue behind plrd's own motion on
the g-code mutex (klippy/gcode.py:239-241).
"""

import posixpath

from . import daemon_link, setup_checks
from .prompts import Prompt as _Prompt
from .prompts import (
    action_prompt_begin,
    action_prompt_button,
    action_prompt_end,
    action_prompt_footer_button,
    action_prompt_show,
    action_prompt_text,
    emit_prompt,
)

# --- recovery wizard states -----------------------------------------
STATE_IDLE = "idle"  # no wizard in flight
STATE_QUERY = "query"  # a plrd call is in flight for the wizard
STATE_OFFERED = "offered"  # offer prompt shown; awaiting DRYRUN/CANCEL
STATE_CLEAN_CHECK = "clean_check"  # clean-nozzle prompt; CONFIRM_CLEAN/CANCEL
STATE_EXECUTE = "execute"  # execute prompt shown; EXECUTE/CANCEL
STATE_RUNNING = "running"  # handed off to plugin.recovery (its state now)

_TITLE = "Power-loss recovery"

# Re-exported for callers (and tests) that reach for the builders here.
__all__ = [
    "RecoveryWizard",
    "action_prompt_begin",
    "action_prompt_button",
    "action_prompt_end",
    "action_prompt_footer_button",
    "action_prompt_show",
    "action_prompt_text",
    "cmd_PLR_SETUP_WIZARD",
    "cmd_PLR_WIZARD_CANCEL",
    "cmd_PLR_WIZARD_CLOSE",
    "cmd_PLR_WIZARD_CONFIRM_CLEAN",
    "cmd_PLR_WIZARD_DRYRUN",
    "cmd_PLR_WIZARD_EXECUTE",
    "cmd_PLR_WIZARD_START",
    "emit_prompt",
]


# --- defensive readers over the daemon response data ----------------


def _pending_recovery(data):
    """The pending-recovery object from a ``status`` response, else None.

    CONTRACT SOURCE — crates/plrd/src/ctrlsock.rs ``build_status``: the
    daemon inserts ``data["pending"]`` on EVERY status response.  It is
    an explicit JSON ``null`` when no pending-recovery file exists or
    parses, otherwise the serialized ``PendingRecovery``
    (crates/plrd/src/detect.rs).  "Nothing pending" is therefore a null
    VALUE, not an absent key — and anything that is not a JSON object
    (null, a renamed/re-typed field, a daemon predating the key) reads as
    nothing pending, so the wizard can never invent a recovery.
    """
    pending = data.get("pending")
    if isinstance(pending, dict):
        return pending
    return None


CLEAN_FLAG_KEY = "requires_clean_nozzle_confirmation"


def _clean_flag(data):
    """Tri-state read of the clean-nozzle-confirmation flag.

    Returns ``True`` (daemon wants the user asked), ``False`` (daemon
    reports cleaning is handled automatically), or ``None`` when the flag
    is absent, non-boolean, or otherwise unreadable.

    CONTRACT SOURCE — crates/plrd/src/ctrlsock.rs ``cmd_recover_dryrun``
    emits ``data: {"outcome": <tag>, "requires_clean_nozzle_confirmation":
    <bool>}``: the flag is TOP LEVEL, an explicit boolean, and present on
    every outcome (``false`` when there is no plan).  It is set from
    ``RecoveryPlan::requires_clean_nozzle_confirmation``, which
    crates/plr-recovery/src/build.rs raises in the ``CleanNozzle`` phase
    when no ``[gcode_macro <clean_nozzle_macro>]`` exists — that step
    carries the macro call when it does exist, and no command when it
    does not.

    The nested ``data.plan.*`` spelling is still accepted as a fallback so
    a future reshaping of the response cannot silently turn the
    confirmation off; a valid boolean at the TOP LEVEL wins.  Anything
    unreadable yields ``None`` for the caller to resolve conservatively
    (see :func:`_clean_decision`) — which also covers any daemon older
    than the field.
    """
    value = data.get(CLEAN_FLAG_KEY)
    if isinstance(value, bool):
        return value
    plan = data.get("plan")
    if isinstance(plan, dict):
        nested = plan.get(CLEAN_FLAG_KEY)
        if isinstance(nested, bool):
            return nested
    return None


# Why the wizard asks, when it asks (drives the extra prompt line).
_ASK_NO_MACRO = "no_macro"  # daemon: nothing cleans it automatically
_ASK_UNKNOWN = "unknown"  # flag absent/unreadable — conservative branch
_ASK_DISAGREE = "disagree"  # daemon says auto, plugin sees no macro section


def _clean_decision(flag, macro_available, macro_name):
    """Resolve the clean-nozzle branch: ``(ask, reason)``.

    FAIL-SAFE DEFAULT — the confirmation is skipped ONLY when both
    independent sources agree that something will actually clean the
    nozzle: the daemon says so with an explicit boolean ``False`` AND the
    plugin can see the configured ``[gcode_macro <name>]`` section.
    Every other case asks:

    * ``True``  — the daemon reports no cleaning macro server-side;
    * ``None``  — the flag is absent/unreadable (a daemon predating it, or
      a renamed field): UNKNOWN TAKES THE CONSERVATIVE BRANCH. Asking
      redundantly when a macro does exist costs the operator one click;
      skipping the check when nothing cleans the nozzle silently corrupts
      the very reference measurement recovery depends on — contact
      readings (touch AND drag) are only trustworthy from a clean tip;
    * ``False`` but no visible macro — the two sources DISAGREE, so the
      conservative branch wins and the prompt says why rather than
      promising a macro run the plugin cannot see.
    """
    if flag is False and macro_available:
        return (False, None)
    if flag is None:
        return (True, _ASK_UNKNOWN)
    if flag is False:
        return (True, _ASK_DISAGREE)
    return (True, _ASK_NO_MACRO)


def _ask_reason_text(reason, macro_name):
    """The extra prompt line explaining WHY the wizard is asking."""
    if reason == _ASK_UNKNOWN:
        return (
            "plrd did not report whether the nozzle gets cleaned "
            "automatically, so you are being asked to be safe."
        )
    if reason == _ASK_DISAGREE:
        return (
            "plrd reports an automatic nozzle clean, but no "
            "[gcode_macro %s] is configured here — the two disagree, so "
            "you are being asked." % (macro_name,)
        )
    return (
        "No [gcode_macro %s] will run for this recovery, so nothing "
        "cleans the nozzle automatically." % (macro_name,)
    )


def _is_number(value):
    """True for a real JSON number (bool is not a number here)."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _print_name(path):
    """Basename of the interrupted print file, or None.

    plrd is Linux-only (see :mod:`plr.daemon_link`), so ``file`` is an
    absolute POSIX path; ``posixpath`` splits it identically on every
    host running these tests.  A path with no separator is returned
    unchanged.
    """
    if not isinstance(path, str) or not path:
        return None
    return posixpath.basename(path) or path


def _summarize(pending):
    """Offer-prompt summary lines built from a ``PendingRecovery``.

    CONTRACT SOURCE — crates/plrd/src/detect.rs ``PendingRecovery``
    (plain serde derive, so the JSON names are the struct's own):
    ``detected_wall_ns`` (int), ``file`` (absolute path string),
    ``file_position`` (int bytes), ``file_size`` (int|null), ``percent``
    (float|null, already a 0-100 percentage — ctrlsock.rs renders it
    ``(~{p:.0}%)``), ``crash_class`` (string, which may carry a trailing
    frame-invalid note).

    Every field is read defensively: a missing, null, or wrongly-typed
    field simply omits its clause — never a crash, never a fabricated
    number.  If nothing at all is readable the prompt still says
    something honest.
    """
    lines = []
    name = _print_name(pending.get("file"))
    if name:
        lines.append("Interrupted print: %s" % (name,))
    percent = pending.get("percent")
    if _is_number(percent):
        lines.append("Progress at power loss: ~%.0f%%" % (percent,))
    position = pending.get("file_position")
    if _is_number(position):
        lines.append("Resume point: byte %d" % (position,))
    crash_class = pending.get("crash_class")
    if isinstance(crash_class, str) and crash_class:
        lines.append("Crash classification: %s" % (crash_class,))
    if not lines:
        lines.append("An interrupted print is ready to resume.")
    return lines


def _daemon_down_text(command, err):
    return (
        "%s: %s\nThe recovery wizard has been reset; fix the daemon "
        "(systemctl status plrd) and run PLR_WIZARD_START again." % (command, err)
    )


class RecoveryWizard:
    """Single in-flight, console-driven power-loss recovery flow.

    All state lives on the one instance the plugin holds; a second
    ``PLR_WIZARD_START`` while active re-shows the current prompt rather
    than starting a parallel flow, and any daemon error resets the flow
    to idle.  Execution itself is owned by
    :class:`plr.recovery.RecoverySession` (shared with ``PLR_RECOVER``), so
    the wizard hands off and then reflects that session's state.
    """

    def __init__(self, plugin):
        self.plugin = plugin
        self.printer = plugin.printer
        self._state = STATE_IDLE
        self._prompt = None  # last emitted _Prompt, for re-show

    # -- introspection (get_status) ----------------------------------

    def is_active(self):
        return self._state != STATE_IDLE

    def state(self):
        return self._state

    # -- emission ----------------------------------------------------

    def _respond(self):
        """The console output callable.

        ``gcmd.respond_info`` IS ``gcode.respond_info`` (klippy/gcode.py:32
        wires the wrapper), so using the dispatcher directly makes handler
        output and worker-callback output byte-identical — and it is the
        only output path available from a callback, which has no gcmd.
        """
        return self.printer.lookup_object("gcode").respond_info

    def _emit_prompt(self, prompt):
        """Emit one prompt as ordered action lines, then its fallback.

        Order is fixed and asserted by tests: begin, text*, button*,
        footer_button*, show, then the plain-text fallback line(s) that
        name the advancing console command.
        """
        emit_prompt(self._respond(), prompt)

    def _show(self, prompt, state):
        self._prompt = prompt
        self._state = state
        self._emit_prompt(prompt)

    def _reshow(self):
        if self._prompt is not None:
            self._emit_prompt(self._prompt)

    def _reset(self):
        self._state = STATE_IDLE
        self._prompt = None
        # Drop any answer still in flight for this flow.  Without this, a
        # PLR_WIZARD_CANCEL issued while the `status` query was outstanding
        # would be undone the moment plrd answered: the callback would open
        # the offer prompt again and resurrect a wizard the operator had
        # just dismissed.
        self.plugin.daemon_wizard.cancel()

    def _fail_daemon(self, command, err):
        """Reset, clear any shown dialog, and report a console error.

        Called from worker callbacks, so it cannot raise ``gcmd.error``:
        see the module docstring.
        """
        self._reset()
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info(action_prompt_end())
        daemon_link.respond_error(gcode, _daemon_down_text(command, err))

    def _query(self, command, cmd, timeout, on_result):
        """Start one wizard daemon call; raise if one is already in flight."""

        def on_error(err):
            self._fail_daemon(command, err)

        if not self.plugin.daemon_wizard.call(cmd, None, timeout, on_result, on_error):
            raise self.printer.command_error(
                "%s: the wizard is still waiting for plrd's previous answer; "
                "try again in a moment." % (command,)
            )
        self._state = STATE_QUERY

    # -- transitions -------------------------------------------------

    def start(self, gcmd):
        """PLR_WIZARD_START — offer recovery if the daemon has one pending.

        A second START while a wizard is active re-shows the current
        prompt (idempotent), never a second parallel flow.  Returns as
        soon as the ``status`` query is handed to a worker.
        """
        if self._state == STATE_RUNNING or self.plugin.recovery.is_active():
            gcmd.respond_info(
                "PLR wizard: a recovery is already in flight.\n%s"
                % ("\n".join(self.plugin.recovery.status_lines()),)
            )
            return
        if self._state == STATE_QUERY:
            # There is nothing to re-show yet: the answer that decides what
            # the prompt says has not arrived.  Say that, rather than
            # claiming a re-show that emits nothing.
            gcmd.respond_info(
                "PLR wizard: still waiting for plrd's answer — the prompt "
                "appears as soon as it replies."
            )
            return
        if self._state != STATE_IDLE:
            gcmd.respond_info(
                "PLR wizard already in progress — re-showing the current prompt."
            )
            self._reshow()
            return
        gcmd.respond_info("PLR wizard: asking plrd whether a recovery is pending...")
        self._query(
            "PLR_WIZARD_START", "status", daemon_link.STATUS_TIMEOUT, self._offer
        )

    def _offer(self, resp):
        """The ``status`` answer, on the reactor thread."""
        data = resp.get("data", {})
        if not isinstance(data, dict):
            data = {}
        # ctrlsock.rs build_status: data["pending"] is null when nothing
        # is pending, else the serialized detect.rs PendingRecovery.
        pending = _pending_recovery(data)
        respond = self._respond()
        if pending is None:
            self._reset()
            respond(
                "PLR wizard: no power-loss recovery is pending — nothing to do.\n"
                "If you believe a print was interrupted, check PLR_STATUS."
            )
            return
        prompt = _Prompt(
            title=_TITLE,
            texts=_summarize(pending) + ["Attempt recovery, or dismiss this prompt."],
            buttons=[("Attempt recovery", "PLR_WIZARD_DRYRUN", "primary")],
            footers=[("Dismiss", "PLR_WIZARD_CANCEL", "error")],
            fallbacks=[
                "Console: run PLR_WIZARD_DRYRUN to review the recovery plan, "
                "or PLR_WIZARD_CANCEL to dismiss."
            ],
        )
        respond("Power-loss recovery available.")
        self._show(prompt, STATE_OFFERED)

    def dryrun(self, gcmd):
        """PLR_WIZARD_DRYRUN — fetch the plan and prompt for the next step."""
        if self._state == STATE_IDLE:
            raise gcmd.error(
                "PLR_WIZARD_DRYRUN: no recovery in progress — run "
                "PLR_WIZARD_START first."
            )
        if self._state in (STATE_QUERY, STATE_RUNNING):
            raise gcmd.error(
                "PLR_WIZARD_DRYRUN: the wizard is busy (%s) — wait for plrd's "
                "report." % (self._state,)
            )
        gcmd.respond_info(
            "PLR wizard: asking plrd for the recovery plan (no motion). This "
            "can take a while on a large journal."
        )
        self._query(
            "PLR_WIZARD_DRYRUN",
            "recover_dryrun",
            daemon_link.DRYRUN_TIMEOUT,
            self._plan,
        )

    def _plan(self, resp):
        """The ``recover_dryrun`` answer, on the reactor thread."""
        respond = self._respond()
        respond(resp.get("text") or "plrd returned an empty dry-run report")
        if not resp.get("ok"):
            self._reset()
            gcode = self.printer.lookup_object("gcode")
            gcode.respond_info(action_prompt_end())
            daemon_link.respond_error(
                gcode,
                "PLR_WIZARD_DRYRUN: plrd dry run reported failure (see the "
                "report above); wizard reset. Fix the reported issue and run "
                "PLR_WIZARD_START again.",
            )
            return
        data = resp.get("data", {})
        if not isinstance(data, dict):
            data = {}
        # ctrlsock.rs cmd_recover_dryrun emits the clean-nozzle flag top
        # level on every outcome (see _clean_flag).  The branch is decided
        # by BOTH the daemon flag and the plugin's own macro detection,
        # and the unknown case asks (see _clean_decision).
        macro = self.plugin.clean_nozzle_macro
        ask, reason = _clean_decision(
            _clean_flag(data), self.plugin.clean_nozzle_macro_available, macro
        )
        if ask:
            self._show_clean_check(reason)
        else:
            self._show_execute(auto_clean=True)

    def _show_clean_check(self, reason=None):
        texts = [
            "Contact readings need a clean nozzle — filament or ooze on "
            "the tip skews every reading.",
        ]
        if reason is not None:
            texts.append(_ask_reason_text(reason, self.plugin.clean_nozzle_macro))
        texts.append("Is the nozzle clean?")
        prompt = _Prompt(
            title=_TITLE,
            texts=texts,
            buttons=[("Nozzle is clean", "PLR_WIZARD_CONFIRM_CLEAN", "primary")],
            footers=[("It's dirty - abort", "PLR_WIZARD_CANCEL", "error")],
            fallbacks=[
                "Console: if the nozzle is clean run PLR_WIZARD_CONFIRM_CLEAN; "
                "if it is dirty run PLR_WIZARD_CANCEL and clean it first."
            ],
        )
        self._show(prompt, STATE_CLEAN_CHECK)

    def _show_execute(self, auto_clean):
        """Emit the execute prompt.

        ``auto_clean`` is only ever set when BOTH sources agree the nozzle
        gets cleaned automatically (:func:`_clean_decision`), so the copy
        can name the macro as a fact grounded in the plugin's own config
        rather than in the daemon's boolean alone.

        The two confirm-point keys are announced here when set, because
        they change what the operator is about to be asked to do — plrd
        will stop mid-recovery and wait for them.
        """
        texts = ["Execute the recovery plan? The printer WILL MOVE."]
        if auto_clean:
            texts.append(
                "[gcode_macro %s] is configured and plrd reports it will run "
                "to clean the nozzle first." % (self.plugin.clean_nozzle_macro,)
            )
        if self.plugin.daemon_keys.get("confirm_z_before_resume") is True:
            texts.append(
                "[plr] confirm_z_before_resume is set: plrd will lift to a "
                "standoff and ask you to check the nozzle height before it "
                "resumes."
            )
        if self.plugin.daemon_keys.get("debug_confirm_each_step") is True:
            texts.append(
                "[plr] debug_confirm_each_step is set: plrd will stop and ask "
                "before EVERY step."
            )
        prompt = _Prompt(
            title=_TITLE,
            texts=texts,
            buttons=[("Execute", "PLR_WIZARD_EXECUTE", "primary")],
            footers=[("Cancel", "PLR_WIZARD_CANCEL", "error")],
            fallbacks=[
                "Console: run PLR_WIZARD_EXECUTE to execute the recovery (the "
                "printer WILL MOVE), or PLR_WIZARD_CANCEL to abort."
            ],
        )
        self._show(prompt, STATE_EXECUTE)

    def confirm_clean(self, gcmd):
        """PLR_WIZARD_CONFIRM_CLEAN — nozzle attested clean → execute prompt."""
        if self._state != STATE_CLEAN_CHECK:
            raise gcmd.error(
                "PLR_WIZARD_CONFIRM_CLEAN: not awaiting a nozzle-clean "
                "confirmation — run PLR_WIZARD_START then PLR_WIZARD_DRYRUN."
            )
        self._show_execute(auto_clean=False)

    def execute(self, gcmd):
        """PLR_WIZARD_EXECUTE — hand off to the recovery session.

        Returns as soon as plrd has been asked to start: the execution, its
        confirm-points and its final report belong to
        :class:`plr.recovery.RecoverySession`, which reports them from
        reactor callbacks.  The wizard stays in STATE_RUNNING until that
        session finishes, so ``wizard_active`` stays honest.
        """
        if self._state != STATE_EXECUTE:
            raise gcmd.error(
                "PLR_WIZARD_EXECUTE: not ready to execute — review the plan "
                "with PLR_WIZARD_START then PLR_WIZARD_DRYRUN first."
            )
        # start() raises on refusal, which leaves the execute prompt in
        # place: the operator can retry without walking the flow again.
        self.plugin.recovery.start(
            gcmd, "PLR_WIZARD_EXECUTE", on_finished=self._recovery_finished
        )
        self._state = STATE_RUNNING
        self._prompt = None
        gcmd.respond_info(action_prompt_end())

    def _recovery_finished(self):
        """The recovery session has reported; drop back to idle."""
        if self._state == STATE_RUNNING:
            self._reset()

    def cancel(self, gcmd):
        """PLR_WIZARD_CANCEL — dismiss the prompt and reset to idle.

        A recovery that is already executing is NOT dismissed here: the
        session refuses (or, at a confirm-point, answers ``abort``), because
        resetting the plugin's own view while plrd still drives the machine
        is precisely the outcome nobody can act on.
        """
        if self.plugin.recovery.is_active():
            # Raises for a running (unanswerable) recovery; answers abort
            # for an outstanding confirm-point.
            self.plugin.recovery.cancel(gcmd, "PLR_WIZARD_CANCEL")
            self._reset()
            gcmd.respond_info(
                "PLR wizard cancelled. Run PLR_WIZARD_START to reopen recovery."
            )
            return
        was_active = self._state != STATE_IDLE
        self._reset()
        gcmd.respond_info(action_prompt_end())
        if was_active:
            gcmd.respond_info(
                "PLR wizard cancelled. Run PLR_WIZARD_START to reopen recovery."
            )
        else:
            gcmd.respond_info("PLR wizard: nothing to cancel.")

    # -- commissioning wizard (independent of the recovery state) ----

    def setup_wizard(self, gcmd):
        """PLR_SETUP_WIZARD — the commissioning report as a button dialog.

        Reuses the exact ``PLR_SETUP`` report (single source of truth for
        the checks) and adds a button per remaining actionable step; the
        underlying commands stay the authority — this is orchestration
        only, and it never touches the recovery state machine.
        """
        plugin = self.plugin
        # Close any dialog still on screen from a previous run before
        # opening a new one, so re-running never stacks prompts.
        gcmd.respond_info(action_prompt_end())
        gcmd.respond_info(
            setup_checks.format_report(
                setup_checks.full_report_results(plugin),
                plugin.self_locking_z,
                plugin.probe_method,
            )
        )

        texts = ["Work through the remaining commissioning steps, then SAVE_CONFIG."]
        buttons = []
        if not plugin.self_locking_z:
            texts.append(
                "Self-locking Z is not attested (leadscrew printers only — "
                "only attest if your Z holds position unpowered)."
            )
            buttons.append(
                (
                    "Attest self-locking Z",
                    "PLR_SETUP ACCEPT_SELF_LOCKING_Z=1",
                    "primary",
                )
            )
        if plugin.probe_method == "adxl_drag":
            texts.append(
                "Drag oracle: measure the noise floor, then calibrate "
                "sensitivity (both MOVE the toolhead — clear the head first)."
            )
            buttons.append(
                ("Measure noise floor (moves)", "PLR_NOISE_TEST START=1", "warning")
            )
            buttons.append(
                (
                    "Calibrate drag sensitivity (moves)",
                    "PLR_DRAG_CALIBRATE START=1",
                    "warning",
                )
            )
        else:
            texts.append(
                "Probe repeatability test measures probe_resolution (MOVES "
                "the toolhead — home first)."
            )
            buttons.append(
                ("Run probe test (moves)", "PLR_PROBE_TEST START=1", "warning")
            )

        fallbacks = ["Console commands for each step:"]
        for label, gcode, _color in buttons:
            fallbacks.append("  %s  ->  %s" % (label, gcode))
        fallbacks.append("  Persist everything  ->  SAVE_CONFIG")
        fallbacks.append("  Close this dialog  ->  PLR_WIZARD_CLOSE")

        prompt = _Prompt(
            title="PLR commissioning",
            texts=texts,
            buttons=buttons,
            # Every dialog needs a path that emits prompt_end, or it sits
            # over the UI forever: SAVE_CONFIG restarts klippy (which
            # tears the dialog down) and Close dismisses it explicitly.
            footers=[
                ("SAVE_CONFIG", "SAVE_CONFIG", "primary"),
                ("Close", "PLR_WIZARD_CLOSE", None),
            ],
            fallbacks=fallbacks,
        )
        # A standalone dialog: emit without touching recovery state.
        self._emit_prompt(prompt)

    def close(self, gcmd):
        """PLR_WIZARD_CLOSE — dismiss whatever dialog is on screen.

        Display-only: it emits ``prompt_end`` and deliberately does NOT
        touch the recovery state machine, so closing the commissioning
        dialog cannot silently abandon an in-flight recovery (dismiss that
        with ``PLR_WIZARD_CANCEL``; ``PLR_WIZARD_START`` re-shows it).  In
        particular it never touches an outstanding confirm-point: the token
        lives on the recovery session, so ``PLR_RECOVER_CONTINUE`` still
        works after the dialog is closed.
        """
        gcmd.respond_info(action_prompt_end())
        if self.plugin.recovery.is_active():
            gcmd.respond_info(
                "PLR: dialog closed. %s"
                % (" ".join(self.plugin.recovery.status_lines()),)
            )
        elif self.is_active():
            gcmd.respond_info(
                "PLR: dialog closed. The recovery wizard is still in "
                "progress — PLR_WIZARD_START re-shows it, PLR_WIZARD_CANCEL "
                "dismisses it."
            )
        else:
            gcmd.respond_info("PLR: dialog closed.")


# --- console command entry points (wired in plr.plugin) --------------


def cmd_PLR_WIZARD_START(plugin, gcmd):
    """PLR_WIZARD_START — open/refresh the power-loss recovery wizard."""
    plugin.wizard.start(gcmd)


def cmd_PLR_WIZARD_DRYRUN(plugin, gcmd):
    """PLR_WIZARD_DRYRUN — show the recovery plan (no motion)."""
    plugin.wizard.dryrun(gcmd)


def cmd_PLR_WIZARD_CONFIRM_CLEAN(plugin, gcmd):
    """PLR_WIZARD_CONFIRM_CLEAN — attest the nozzle is clean; advance."""
    plugin.wizard.confirm_clean(gcmd)


def cmd_PLR_WIZARD_EXECUTE(plugin, gcmd):
    """PLR_WIZARD_EXECUTE — execute the recovery plan (the printer moves)."""
    plugin.wizard.execute(gcmd)


def cmd_PLR_WIZARD_CANCEL(plugin, gcmd):
    """PLR_WIZARD_CANCEL — dismiss the recovery wizard and reset."""
    plugin.wizard.cancel(gcmd)


def cmd_PLR_SETUP_WIZARD(plugin, gcmd):
    """PLR_SETUP_WIZARD — prompt-driven commissioning walk."""
    plugin.wizard.setup_wizard(gcmd)


def cmd_PLR_WIZARD_CLOSE(plugin, gcmd):
    """PLR_WIZARD_CLOSE — close the on-screen prompt (display only)."""
    plugin.wizard.close(gcmd)
