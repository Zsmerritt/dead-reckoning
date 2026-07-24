"""Prompt-driven recovery and commissioning wizards.

Two console-command-driven flows that render as interactive dialogs on
clients that support Klipper's *action prompt* protocol (Mainsail,
Fluidd, KlipperScreen, OctoApp) and degrade to plain console text
everywhere else:

* the **recovery wizard** (``PLR_WIZARD_*``) — a small state machine that
  walks an operator through a pending power-loss recovery: offer → review
  plan → (confirm the nozzle is clean) → execute;
* the **commissioning wizard** (``PLR_SETUP_WIZARD``) — a one-shot dialog
  that reuses the exact ``PLR_SETUP`` report and adds a button per
  remaining commissioning step.

Prompts are sugar; console commands are the contract.  Every prompt this
module emits is paired with plain-text fallback lines that name the exact
console command which advances the flow, so a client without prompt
support is never stuck.

Action-prompt wire format (Mainsail "Macro Prompts" spec, requires
Klipper's ``[respond]`` module; supported since Mainsail 2.9.0):

    // action:prompt_begin <headline>
    // action:prompt_text <text>
    // action:prompt_button <label>|<gcode?>|<color?>
    // action:prompt_footer_button <label>|<gcode?>|<color?>
    // action:prompt_show
    // action:prompt_end

Colors: primary | secondary | info | warning | error (else a default).
The plugin emits each action line through ``gcmd.respond_info``, which
klippy prepends with the ``// `` transport prefix on every line
(klippy/gcode.py ``respond_info``) — byte-identical to what
``RESPOND TYPE=command MSG="action:..."`` puts on the wire
(klippy/extras/respond.py maps TYPE=command to the ``//`` prefix).  The
literal strings the tests assert are therefore the ``action:...`` payload
without that prefix.

SAFETY INVARIANT: no wizard command ever sends motion g-code to the
printer.  The ONLY machine motion a wizard triggers is the daemon's own
``recover_execute`` over the control socket; recovery prompt buttons fire
other ``PLR_WIZARD_*`` console commands, and the setup wizard's buttons
fire the existing ``PLR_SETUP`` / ``PLR_*_TEST`` / ``SAVE_CONFIG``
commands (each of which owns its own consent + gates).  This module calls
``plugin.daemon`` and ``gcmd.respond_info`` and nothing else.
"""

import collections

from . import daemon_link, setup_checks

# --- recovery wizard states -----------------------------------------
STATE_IDLE = "idle"  # no wizard in flight
STATE_OFFERED = "offered"  # offer prompt shown; awaiting DRYRUN/CANCEL
STATE_CLEAN_CHECK = "clean_check"  # clean-nozzle prompt; CONFIRM_CLEAN/CANCEL
STATE_EXECUTE = "execute"  # execute prompt shown; EXECUTE/CANCEL


# One prompt to render: a headline, descriptive text lines, primary
# buttons and footer buttons (each ``(label, gcode, color)``), and the
# plain-text fallback lines that name the advancing console command(s).
_Prompt = collections.namedtuple(
    "_Prompt", ["title", "texts", "buttons", "footers", "fallbacks"]
)

_TITLE = "Power-loss recovery"


# --- action-string builders (pure; unit-testable literals) ----------


def _button_spec(label, gcode, color):
    """``<label>|<gcode?>|<color?>`` with the pipes the Mainsail spec uses.

    A color forces the middle field (empty gcode defaults to the label
    on the client); gcode alone yields ``label|gcode``; label alone is
    bare.
    """
    if color is not None:
        return "|".join([label, gcode or "", color])
    if gcode is not None:
        return "|".join([label, gcode])
    return label


def action_prompt_begin(title):
    return "action:prompt_begin %s" % (title,)


def action_prompt_text(text):
    return "action:prompt_text %s" % (text,)


def action_prompt_button(label, gcode=None, color=None):
    return "action:prompt_button %s" % (_button_spec(label, gcode, color),)


def action_prompt_footer_button(label, gcode=None, color=None):
    return "action:prompt_footer_button %s" % (_button_spec(label, gcode, color),)


def action_prompt_show():
    return "action:prompt_show"


def action_prompt_end():
    return "action:prompt_end"


# --- defensive readers over the daemon response data ----------------


def _recovery_pending(data):
    """True when the daemon reports a pending recovery.

    COORDINATION POINT (Rust): the daemon's ``status`` response advertises
    a pending recovery via ``data["recovery_pending"]`` (bool).  Read
    defensively — a daemon predating this field is treated as "nothing
    pending" so the wizard never invents a recovery.
    """
    return bool(data.get("recovery_pending", False))


def _requires_clean(data):
    """Plan-level clean-nozzle-confirmation flag; absent → False.

    FROZEN contract: ``recover_dryrun``/``status`` carry
    ``requires_clean_nozzle_confirmation`` in ``data`` for daemons that
    implement it; older daemons omit it, which reads as "no confirmation
    required" (the auto-clean / no-clean path).
    """
    return bool(data.get("requires_clean_nozzle_confirmation", False))


def _format_progress(progress):
    """Best-effort human progress string from a loosely-typed field."""
    if isinstance(progress, bool):
        return str(progress)
    if isinstance(progress, (int, float)):
        pct = progress * 100.0 if 0.0 <= progress <= 1.0 else progress
        return "%.1f%%" % (pct,)
    return str(progress)


def _summarize(data):
    """Print file/progress summary lines for the offer prompt."""
    lines = []
    print_file = data.get("print_file")
    if print_file:
        lines.append("Interrupted print: %s" % (print_file,))
    progress = data.get("progress")
    if progress is not None:
        lines.append("Progress at power loss: %s" % (_format_progress(progress),))
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
    to idle.
    """

    def __init__(self, plugin):
        self.plugin = plugin
        self._state = STATE_IDLE
        self._prompt = None  # last emitted _Prompt, for re-show

    # -- introspection (get_status) ----------------------------------

    def is_active(self):
        return self._state != STATE_IDLE

    # -- emission ----------------------------------------------------

    def _emit_prompt(self, gcmd, prompt):
        """Emit one prompt as ordered action lines, then its fallback.

        Order is fixed and asserted by tests: begin, text*, button*,
        footer_button*, show, then the plain-text fallback line(s) that
        name the advancing console command.
        """
        respond = gcmd.respond_info
        respond(action_prompt_begin(prompt.title))
        for text in prompt.texts:
            respond(action_prompt_text(text))
        for label, gcode, color in prompt.buttons:
            respond(action_prompt_button(label, gcode, color))
        for label, gcode, color in prompt.footers:
            respond(action_prompt_footer_button(label, gcode, color))
        respond(action_prompt_show())
        for line in prompt.fallbacks:
            respond(line)

    def _show(self, gcmd, prompt, state):
        self._prompt = prompt
        self._state = state
        self._emit_prompt(gcmd, prompt)

    def _reshow(self, gcmd):
        if self._prompt is not None:
            self._emit_prompt(gcmd, self._prompt)

    def _reset(self):
        self._state = STATE_IDLE
        self._prompt = None

    def _fail_daemon(self, gcmd, command, err):
        """Reset, clear any shown dialog, and raise a console error."""
        self._reset()
        gcmd.respond_info(action_prompt_end())
        raise gcmd.error(_daemon_down_text(command, err))

    # -- transitions -------------------------------------------------

    def start(self, gcmd):
        """PLR_WIZARD_START — offer recovery if the daemon has one pending.

        A second START while a wizard is active re-shows the current
        prompt (idempotent), never a second parallel flow.
        """
        if self._state != STATE_IDLE:
            gcmd.respond_info(
                "PLR wizard already in progress — re-showing the current prompt."
            )
            self._reshow(gcmd)
            return
        try:
            resp = self.plugin.daemon.call("status", timeout=daemon_link.STATUS_TIMEOUT)
        except daemon_link.DaemonError as e:
            self._fail_daemon(gcmd, "PLR_WIZARD_START", e)
        data = resp.get("data", {})
        if not _recovery_pending(data):
            gcmd.respond_info(
                "PLR wizard: no power-loss recovery is pending — nothing to do.\n"
                "If you believe a print was interrupted, check PLR_STATUS."
            )
            return
        prompt = _Prompt(
            title=_TITLE,
            texts=_summarize(data) + ["Attempt recovery, or dismiss this prompt."],
            buttons=[("Attempt recovery", "PLR_WIZARD_DRYRUN", "primary")],
            footers=[("Dismiss", "PLR_WIZARD_CANCEL", "error")],
            fallbacks=[
                "Console: run PLR_WIZARD_DRYRUN to review the recovery plan, "
                "or PLR_WIZARD_CANCEL to dismiss."
            ],
        )
        gcmd.respond_info("Power-loss recovery available.")
        self._show(gcmd, prompt, STATE_OFFERED)

    def dryrun(self, gcmd):
        """PLR_WIZARD_DRYRUN — fetch the plan and prompt for the next step."""
        if self._state == STATE_IDLE:
            raise gcmd.error(
                "PLR_WIZARD_DRYRUN: no recovery in progress — run "
                "PLR_WIZARD_START first."
            )
        try:
            resp = self.plugin.daemon.call(
                "recover_dryrun", timeout=daemon_link.RECOVER_TIMEOUT
            )
        except daemon_link.DaemonError as e:
            self._fail_daemon(gcmd, "PLR_WIZARD_DRYRUN", e)
        gcmd.respond_info(resp.get("text") or "plrd returned an empty dry-run report")
        if not resp.get("ok"):
            self._reset()
            gcmd.respond_info(action_prompt_end())
            raise gcmd.error(
                "PLR_WIZARD_DRYRUN: plrd dry run reported failure (see the "
                "report above); wizard reset. Fix the reported issue and run "
                "PLR_WIZARD_START again."
            )
        data = resp.get("data", {})
        if _requires_clean(data):
            self._show_clean_check(gcmd)
        else:
            self._show_execute(gcmd, auto_clean=True)

    def _show_clean_check(self, gcmd):
        prompt = _Prompt(
            title=_TITLE,
            texts=[
                "Contact readings need a clean nozzle — filament or ooze on "
                "the tip skews every reading.",
                "Is the nozzle clean?",
            ],
            buttons=[("Nozzle is clean", "PLR_WIZARD_CONFIRM_CLEAN", "primary")],
            footers=[("It's dirty - abort", "PLR_WIZARD_CANCEL", "error")],
            fallbacks=[
                "Console: if the nozzle is clean run PLR_WIZARD_CONFIRM_CLEAN; "
                "if it is dirty run PLR_WIZARD_CANCEL and clean it first."
            ],
        )
        self._show(gcmd, prompt, STATE_CLEAN_CHECK)

    def _show_execute(self, gcmd, auto_clean):
        texts = ["Execute the recovery plan? The printer WILL MOVE."]
        if auto_clean:
            texts.append(
                "Your %s macro will run automatically to clean the nozzle "
                "first." % (self.plugin.clean_nozzle_macro,)
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
        self._show(gcmd, prompt, STATE_EXECUTE)

    def confirm_clean(self, gcmd):
        """PLR_WIZARD_CONFIRM_CLEAN — nozzle attested clean → execute prompt."""
        if self._state != STATE_CLEAN_CHECK:
            raise gcmd.error(
                "PLR_WIZARD_CONFIRM_CLEAN: not awaiting a nozzle-clean "
                "confirmation — run PLR_WIZARD_START then PLR_WIZARD_DRYRUN."
            )
        self._show_execute(gcmd, auto_clean=False)

    def execute(self, gcmd):
        """PLR_WIZARD_EXECUTE — run recover_execute; end the prompt; report."""
        if self._state != STATE_EXECUTE:
            raise gcmd.error(
                "PLR_WIZARD_EXECUTE: not ready to execute — review the plan "
                "with PLR_WIZARD_START then PLR_WIZARD_DRYRUN first."
            )
        try:
            resp = self.plugin.daemon.call(
                "recover_execute",
                {"confirm": True},
                timeout=daemon_link.RECOVER_TIMEOUT,
            )
        except daemon_link.DaemonError as e:
            self._fail_daemon(gcmd, "PLR_WIZARD_EXECUTE", e)
        # The flow is over regardless of outcome: reset before reporting.
        self._reset()
        gcmd.respond_info(resp.get("text") or "plrd returned an empty recovery report")
        gcmd.respond_info(action_prompt_end())
        if resp.get("ok"):
            gcmd.respond_info("PLR recovery complete — resuming print.")
            return
        raise gcmd.error(
            "PLR recovery did not complete (see the report above for the typed "
            "failure and how to remediate). Re-run PLR_WIZARD_START to retry."
        )

    def cancel(self, gcmd):
        """PLR_WIZARD_CANCEL — dismiss the prompt and reset to idle."""
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
        results = list(plugin.static_check_results)
        results.append(setup_checks.check_recorder_heartbeat(plugin.wal_dir))
        results.extend(setup_checks.calibration_check_results(plugin))
        results.append(setup_checks.clean_nozzle_check_result(plugin))
        gcmd.respond_info(
            setup_checks.format_report(
                results, plugin.self_locking_z, plugin.probe_method
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

        prompt = _Prompt(
            title="PLR commissioning",
            texts=texts,
            buttons=buttons,
            footers=[("SAVE_CONFIG", "SAVE_CONFIG", "primary")],
            fallbacks=fallbacks,
        )
        # A standalone dialog: emit without touching recovery state.
        self._emit_prompt(gcmd, prompt)


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
