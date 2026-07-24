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
import posixpath

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
    returns ``data: {"outcome": <tag>}`` today; the frozen
    ``requires_clean_nozzle_confirmation`` flag is being added by the Rust
    side and may land either at the top level of ``data`` or nested under
    a ``plan`` object.  Both are read, a valid boolean at the TOP LEVEL
    winning over a nested one; anything else yields ``None`` for the
    caller to resolve conservatively (see :func:`_clean_decision`).
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
        # ctrlsock.rs build_status: data["pending"] is null when nothing
        # is pending, else the serialized detect.rs PendingRecovery.
        pending = _pending_recovery(data)
        if pending is None:
            gcmd.respond_info(
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
        # ctrlsock.rs cmd_recover_dryrun: data carries {"outcome": <tag>}
        # today; the frozen clean-nozzle flag may arrive top level or
        # nested under "plan" (see _clean_flag).  The branch is decided by
        # BOTH the daemon flag and the plugin's own macro detection, and
        # the unknown case asks (see _clean_decision).
        macro = self.plugin.clean_nozzle_macro
        ask, reason = _clean_decision(
            _clean_flag(data), self.plugin.clean_nozzle_macro_available, macro
        )
        if ask:
            self._show_clean_check(gcmd, reason)
        else:
            self._show_execute(gcmd, auto_clean=True)

    def _show_clean_check(self, gcmd, reason=None):
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
        self._show(gcmd, prompt, STATE_CLEAN_CHECK)

    def _show_execute(self, gcmd, auto_clean):
        """Emit the execute prompt.

        ``auto_clean`` is only ever set when BOTH sources agree the nozzle
        gets cleaned automatically (:func:`_clean_decision`), so the copy
        can name the macro as a fact grounded in the plugin's own config
        rather than in the daemon's boolean alone.
        """
        texts = ["Execute the recovery plan? The printer WILL MOVE."]
        if auto_clean:
            texts.append(
                "[gcode_macro %s] is configured and plrd reports it will run "
                "to clean the nozzle first." % (self.plugin.clean_nozzle_macro,)
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
