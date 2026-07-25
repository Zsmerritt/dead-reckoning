"""Rendering for plrd's recovery confirm-points.

Turns one ``awaiting_confirmation`` response into the dialog (and the
console lines) an operator answers.  Pure functions over the response's
``data`` map: no state, no I/O, no printer access — :mod:`plr.recovery`
owns all of that.

CONTRACT SOURCE — crates/plrd/src/ctrlsock.rs ``report_pause``
(the ``json!`` at :849-861) is the producer of every field read here::

    {"ok": false,
     "text": "<prefix>recover: PAUSED at step N [phase] awaiting \\
              confirmation (kind)\\n<diagnosis.full()>",
     "data": {"outcome": "awaiting_confirmation",
              "resume_token": "plrc-<hex>-<hex>",
              "confirm_kind": "diagnosis" | "z-height" | "step-debug",
              "step": <u32>,
              "phase": "<phase name>",
              "diagnosis": {...},
              "detail": {...}}}

``confirm_kind`` is ``ConfirmKind::tag()`` (crates/plrd/src/executor.rs:
170-180) — exactly the three strings above, one per feature that raises a
pause (executor.rs:159-168).  ``diagnosis`` is the same JSON object every
diagnosis in the system is (crates/plr-recovery/src/diagnosis.rs:145-186
``Diagnosis``): ``code``, ``tier`` (``advisory``/``confirmable``/``hard``,
diagnosis.rs:76-88), ``what``, ``why``, ``suggested_fix``, ``measured``
(``{quantity, value, unit}`` or null, diagnosis.rs:107-114), ``expected``
(``{quantity, min, max, unit}`` or null, diagnosis.rs:116-127) and
``override_key`` (a string or null, diagnosis.rs:161-186).  ``detail`` is
feature-specific: ``{"raised_by": "pre-flight"}`` for a diagnosis pause
(executor.rs:786-792), ``{"standoff_target_z", "live_toolhead_z",
"derivation"}`` for a Z-height pause (executor.rs:961-971), and
``{"summary", "commands", "pre_verify", "verify", "cleanup_commands"}``
for a step-debug pause (executor.rs:882-888).

THE THREE-PART REQUIREMENT.  Every rendered confirmation says WHY it
stopped, SUGGESTS a fix, and OFFERS to continue anyway — in the dialog
text and in the console fallback, so a client that renders no dialog at
all still shows all three.  When a field the daemon should have sent is
missing or the wrong type the renderer says so in place of it rather than
dropping the part: a confirmation that silently loses its "why" is how an
operator ends up clicking Continue blind.

OVERRIDE KEYS ARE NEVER BUTTONS.  ``override_key`` names an
``UNSAFE_``-prefixed ``printer.cfg`` key (diagnosis.rs:57-75).  It is
reported as a fact — "this key exists, edit it while nothing is at
stake" — and never wired to a control, which is the whole point of the
tier split (diagnosis.rs:22-42).
"""

from .prompts import Prompt

# The two commands that answer an outstanding confirm-point.  Buttons fire
# exactly these; the console fallback names exactly these; nothing else
# can answer.  Defined here because the renderer writes them into the
# dialog, and re-exported by :mod:`plr.recovery`, which implements them.
CMD_CONTINUE = "PLR_RECOVER_CONTINUE"
CMD_ABORT = "PLR_RECOVER_ABORT"

TITLE = "Power-loss recovery"

# ConfirmKind::tag() values (crates/plrd/src/executor.rs:170-180).
KIND_DIAGNOSIS = "diagnosis"
KIND_Z_HEIGHT = "z-height"
KIND_STEP_DEBUG = "step-debug"

# Tier::tag() values (crates/plr-recovery/src/diagnosis.rs:90-98).
TIER_CONFIRMABLE = "confirmable"

# Longest command list a step-debug pause prints in full.  A stock step
# carries a handful; the cap keeps a pathological step from burying the
# question, and the remainder is counted rather than silently dropped.
_MAX_DETAIL_COMMANDS = 12


def _text(value):
    """A non-empty string, or None — every prose field read defensively."""
    if isinstance(value, str) and value.strip():
        return value.strip()
    return None


def _num(value):
    """A finite real number, or None (bool is not a number here)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    # Rejects nan/inf without importing math for one comparison: a
    # non-finite number is not a measurement, whatever it decodes to.
    if value != value or value in (float("inf"), float("-inf")):
        return None
    return value


def _fmt(value):
    return ("%g" % (value,)).strip()


def _unit(value):
    unit = _text(value)
    return " %s" % (unit,) if unit else ""


def question(kind):
    """The one line saying what this pause is asking, per ConfirmKind.

    An unrecognized kind gets the generic question rather than a guess:
    the daemon is paused either way, and the operator still has to answer.
    """
    if kind == KIND_Z_HEIGHT:
        return (
            "Does this look right? Compare the nozzle standoff you can see "
            "against the height plrd reports below."
        )
    if kind == KIND_STEP_DEBUG:
        return "Run the next step? Its exact commands are listed below."
    if kind == KIND_DIAGNOSIS:
        return "Continue despite this?"
    return (
        "plrd has paused and needs an answer (it did not report which kind "
        "of confirmation this is, so treat it as: continue despite this?)."
    )


def diagnosis_lines(diagnosis):
    """The WHY / numbers / FIX lines for one diagnosis object.

    Missing or wrongly-typed fields produce an explicit "plrd did not
    report ..." line instead of silence.
    """
    if not isinstance(diagnosis, dict):
        return [
            "plrd sent no readable explanation for this pause (the "
            "diagnosis field was missing or malformed).",
            "Why: unknown — treat this as unexplained and prefer Abort "
            "unless you know why the recovery stopped.",
            "Suggested fix: check the plrd report above and "
            "'journalctl -u plrd' before continuing.",
        ]
    lines = []
    code = _text(diagnosis.get("code"))
    what = _text(diagnosis.get("what"))
    if what:
        lines.append("What: %s%s" % (what, " [%s]" % (code,) if code else ""))
    elif code:
        lines.append("What: %s (plrd sent no description)" % (code,))
    else:
        lines.append("What: plrd sent neither a code nor a description.")
    why = _text(diagnosis.get("why"))
    lines.append("Why: %s" % (why if why else "plrd did not report a reason."))
    lines.extend(_measured_lines(diagnosis))
    fix = _text(diagnosis.get("suggested_fix"))
    lines.append(
        "Suggested fix: %s"
        % (fix if fix else "plrd did not suggest one — see its report above.")
    )
    tier = _text(diagnosis.get("tier"))
    if tier != TIER_CONFIRMABLE:
        # A confirm-point is a Confirmable tier by construction
        # (executor.rs raises pauses only for Tier::Confirmable, and
        # preflight_confirmations routes Hard to a refusal instead,
        # ctrlsock/executor.rs:797-811).  Anything else here means the
        # daemon and this plugin disagree about what is being asked, and
        # the operator gets told rather than reassured.
        lines.append(
            "Note: plrd reported this as tier '%s', not '%s' — that is "
            "unexpected for a confirmation. Prefer Abort and check the "
            "report above." % (tier if tier else "unreported", TIER_CONFIRMABLE)
        )
    override = _text(diagnosis.get("override_key"))
    if override:
        lines.append(
            "There is a config override for this (%s). It is set in "
            "printer.cfg's [plr] section while the machine is idle and "
            "nothing is at stake — deliberately not from this dialog." % (override,)
        )
    return lines


def _measured_lines(diagnosis):
    """``measured:`` / ``expected:`` lines, when the daemon sent numbers."""
    lines = []
    measured = diagnosis.get("measured")
    if isinstance(measured, dict):
        value = _num(measured.get("value"))
        quantity = _text(measured.get("quantity")) or "value"
        if value is not None:
            lines.append(
                "Measured: %s = %s%s"
                % (quantity, _fmt(value), _unit(measured.get("unit")))
            )
    expected = diagnosis.get("expected")
    if isinstance(expected, dict):
        low = _num(expected.get("min"))
        high = _num(expected.get("max"))
        quantity = _text(expected.get("quantity")) or "value"
        unit = _unit(expected.get("unit"))
        band = None
        if low is not None and high is not None:
            band = (
                "%s%s" % (_fmt(low), unit)
                if low == high
                else "[%s, %s]%s" % (_fmt(low), _fmt(high), unit)
            )
        elif low is not None:
            band = ">= %s%s" % (_fmt(low), unit)
        elif high is not None:
            band = "<= %s%s" % (_fmt(high), unit)
        if band is not None:
            lines.append("Expected: %s %s" % (quantity, band))
    return lines


def detail_lines(kind, detail):
    """Feature-specific evidence lines for the pause (see module docs)."""
    if not isinstance(detail, dict):
        return []
    if kind == KIND_Z_HEIGHT:
        return _z_detail_lines(detail)
    if kind == KIND_STEP_DEBUG:
        return _step_detail_lines(detail)
    raised_by = _text(detail.get("raised_by"))
    if raised_by:
        return ["Raised by: %s" % (raised_by,)]
    return []


def _z_detail_lines(detail):
    lines = []
    live = _num(detail.get("live_toolhead_z"))
    target = _num(detail.get("standoff_target_z"))
    if live is not None:
        lines.append("Toolhead is standing off at Z = %s mm." % (_fmt(live),))
    else:
        lines.append(
            "plrd could not read the live toolhead Z back; judge the standoff by eye."
        )
    if target is not None:
        lines.append("Standoff target was Z = %s mm." % (_fmt(target),))
    derivation = _text(detail.get("derivation"))
    if derivation:
        lines.append("Z was derived as: %s" % (derivation,))
    return lines


def _step_detail_lines(detail):
    lines = []
    summary = _text(detail.get("summary"))
    if summary:
        lines.append("Step: %s" % (summary,))
    commands = detail.get("commands")
    if isinstance(commands, list) and commands:
        shown = [_text(c) or repr(c) for c in commands[:_MAX_DETAIL_COMMANDS]]
        lines.append("Commands about to be sent: %s" % ("; ".join(shown),))
        extra = len(commands) - len(shown)
        if extra > 0:
            lines.append("(and %d more command(s) — see the report above.)" % (extra,))
    elif isinstance(commands, list):
        lines.append("This step sends no commands (it only verifies).")
    checks = detail.get("verify")
    if isinstance(checks, list) and checks:
        shown = [_text(c) or repr(c) for c in checks[:_MAX_DETAIL_COMMANDS]]
        lines.append("Verified afterwards: %s" % ("; ".join(shown),))
    return lines


def where_line(step, phase, kind):
    """ "Paused at step N [phase] (kind)" — best effort, never invented."""
    parts = []
    if isinstance(step, bool) or not isinstance(step, int):
        parts.append("Paused at an unreported step")
    else:
        parts.append("Paused at step %d" % (step,))
    name = _text(phase)
    if name:
        parts.append("[%s]" % (name,))
    parts.append("(%s confirmation)" % (_text(kind) or "unspecified",))
    return " ".join(parts)


def confirm_prompt(data, deadline_text):
    """The full confirm-point dialog for one ``awaiting_confirmation``.

    ``deadline_text`` is the caller's honest statement of plrd's own
    abort-on-no-answer deadline (see :mod:`plr.recovery`), or None to say
    nothing about it — never a guessed number.
    """
    if not isinstance(data, dict):
        data = {}
    kind = _text(data.get("confirm_kind"))
    texts = [where_line(data.get("step"), data.get("phase"), kind)]
    texts.append(question(kind))
    texts.extend(diagnosis_lines(data.get("diagnosis")))
    texts.extend(detail_lines(kind, data.get("detail")))
    if deadline_text:
        texts.append(deadline_text)
    return Prompt(
        title="%s — confirmation needed" % (TITLE,),
        texts=texts,
        buttons=[("Continue anyway", CMD_CONTINUE, "warning")],
        footers=[("Abort recovery", CMD_ABORT, "error")],
        fallbacks=[
            "Console: run %s to continue the recovery anyway, or %s to stop "
            "it here." % (CMD_CONTINUE, CMD_ABORT),
        ],
    )
