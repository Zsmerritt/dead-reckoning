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
170-180) plus ``"preview"`` (the resume-preview kind); the console
fallback is the working path for every one of them.  ``diagnosis`` is the
same JSON object every diagnosis in the system is
(crates/plr-recovery/src/diagnosis.rs:145-186 ``Diagnosis``): ``code``,
``tier`` (``advisory``/``confirmable``/``hard``, diagnosis.rs:76-88),
``what``, ``why``, ``suggested_fix``, ``measured`` (``{quantity, value,
unit}`` or null, diagnosis.rs:107-114), ``expected`` (``{quantity, min,
max, unit}`` or null, diagnosis.rs:116-127) and ``override_key`` (a string
or null, diagnosis.rs:161-186).  ``detail`` is feature-specific:
``{"raised_by": "pre-flight"}`` for a diagnosis pause (executor.rs:
786-792), ``{"standoff_target_z", "live_toolhead_z", "derivation"}`` for a
Z-height pause (executor.rs:961-971), and ``{"summary", "commands",
"pre_verify", "verify", "cleanup_commands"}`` for a step-debug pause
(executor.rs:882-888).

THE RESUME-PREVIEW ``detail`` CONTRACT — crates/plrd/src/executor.rs
``preview_detail`` is the producer, and every field it emits is read here
BYTE-FOR-BYTE (agent-contract discipline: never read a field the producer
does not send; this project's worst plugin bug was reading fields that did
not exist).  The 14 fields, with their Rust source types::

    offset             u64   this deposition line's byte offset (M26-safe);
                             updates every reposition — the alignment
                             feedback, because adjacent stops can be <1 mm
                             apart and the operator must not rely on seeing
                             the nozzle move
    resume_offset      u64   where a resume STARTS if this stop is accepted
    xy                 [f64; 2]  hover target, Klipper-internal frame;
                             ALWAYS shown — within one arc source line a ±1
                             nudge moves xy while offset holds (design §12)
    z                  f64   this stop's deposition Z (NOT the hover plane)
    layer              u32 | null   layer active at the move; null before
                             the first deposition
    layer_provenance   str | null   "journal" when ``layer`` is corroborated
                             by a journaled slicer mark
                             (plr_wal::Context::current_layer, an upper bound
                             on the physical layer, validated under the
                             absolute-frame rule), "inferred" when derived
                             from the model alone, null when there is no
                             layer.  ABSENT on an old daemon that predates the
                             field — the renderer then states the layer with
                             no provenance claim, never inventing "journal"
    feature            str   FeatureClass Debug name (see _FEATURE_LABELS)
    on_infill          bool  the stop's line is internal/solid infill
    is_candidate       bool  the stop matched the crash evidence (vs a
                             nudge-only line reachable outside the matcher's
                             tolerance box) — the stop's provenance
    position           int   1-based "stop N of ..."
    count              int   total selectable stops ("... of M")
    before_skip_forward bool the cursor is EARLIER than the safe
                             skip-forward default → accepting re-prints
                             geometry that may exist (advisory warning)
    acceptable         bool  false for a stop with no resumable line (empty
                             entry moves); the executor also refuses accept
                             on it, so the dialog renders it non-acceptable
    at_boundary        str | null   "first" when the cursor is on the
                             earliest stop, "last" on the final one, null in
                             between.  A ±nudge past a boundary CLAMPS and
                             re-emits the same stop; the renderer states this
                             so the operator learns why nothing changed
                             rather than seeing a byte-identical prompt.
                             ABSENT on an old daemon predating the field

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

# The two commands that answer a BINARY confirm-point.  Buttons fire
# exactly these; the console fallback names exactly these; nothing else
# can answer.  Defined here because the renderer writes them into the
# dialog, and re-exported by :mod:`plr.recovery`, which implements them.
CMD_CONTINUE = "PLR_RECOVER_CONTINUE"
CMD_ABORT = "PLR_RECOVER_ABORT"

# The commands that answer a resume-PREVIEW confirm-point (design §F.2).
# Every one fires a plain g-code command so the whole preview is
# completable from a bare console (the portability floor); the dialog
# buttons are the enhancement.  Re-exported by :mod:`plr.recovery`, which
# implements them, and named in this renderer's fallback lines.
CMD_ACCEPT = "PLR_RECOVER_ACCEPT"
CMD_NEXT = "PLR_RECOVER_NEXT"
CMD_PREV = "PLR_RECOVER_PREV"
CMD_NUDGE = "PLR_RECOVER_NUDGE"

TITLE = "Power-loss recovery"

# ConfirmKind::tag() values (crates/plrd/src/executor.rs:170-180) plus the
# resume-preview kind (crates/plrd/src/ctrlsock.rs PREVIEW_KIND_TAG,
# ConfirmKind::Preview).
KIND_DIAGNOSIS = "diagnosis"
KIND_Z_HEIGHT = "z-height"
KIND_STEP_DEBUG = "step-debug"
KIND_PREVIEW = "preview"

# Human labels for the ``feature`` field, which the producer serializes as
# the Rust ``FeatureClass`` Debug name (crates/plr-analyzer/src/model.rs:
# 49-70, emitted by executor.rs ``preview_detail`` via ``format!("{:?}")``).
# An unrecognized name is shown verbatim — never invented — so a daemon
# that grows a class still renders honestly.
_FEATURE_LABELS = {
    "InternalInfill": "internal infill",
    "SolidInfill": "solid infill",
    "InnerWall": "inner wall",
    "OuterWall": "outer wall",
    "Surface": "surface",
    "Bridge": "bridge",
    "GapFill": "gap fill",
    "SkirtBrim": "skirt/brim",
    "Support": "support",
    "Other": "other/unclassified",
}

# Tier::tag() values (crates/plr-recovery/src/diagnosis.rs:90-98).  A
# binary confirm-point is always Confirmable; the resume-preview pause is
# Advisory by construction (executor.rs ``preview_point`` builds
# ``Tier::Advisory``), so the renderer's tier check is told which tier to
# expect and only flags a genuine mismatch.
TIER_CONFIRMABLE = "confirmable"
TIER_ADVISORY = "advisory"

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
    if kind == KIND_PREVIEW:
        return (
            "Where did printing stop? The part shows a ragged edge where "
            "extrusion ended. Step the hover point to that edge (Next/Prev "
            "between candidates, nudge to move along the toolpath), then "
            "Accept — the resume starts at the next line."
        )
    if kind == KIND_DIAGNOSIS:
        return "Continue despite this?"
    return (
        "plrd has paused and needs an answer (it did not report which kind "
        "of confirmation this is, so treat it as: continue despite this?)."
    )


def diagnosis_lines(diagnosis, expected_tier=TIER_CONFIRMABLE):
    """The WHY / numbers / FIX lines for one diagnosis object.

    Missing or wrongly-typed fields produce an explicit "plrd did not
    report ..." line instead of silence.  ``expected_tier`` is the tier
    this pause is supposed to carry (Confirmable for a binary safety
    pause, Advisory for the resume preview); anything else means the
    daemon and this plugin disagree about what is being asked, and the
    operator is told rather than reassured.
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
    if tier != expected_tier:
        # A binary confirm-point is Confirmable by construction
        # (executor.rs raises pauses only for Tier::Confirmable, and
        # preflight_confirmations routes Hard to a refusal instead,
        # ctrlsock/executor.rs:797-811); the resume preview is Advisory.
        # Anything else here means the daemon and this plugin disagree
        # about what is being asked, and the operator gets told rather
        # than reassured.
        lines.append(
            "Note: plrd reported this as tier '%s', not '%s' — that is "
            "unexpected for this pause. Prefer Abort and check the "
            "report above." % (tier if tier else "unreported", expected_tier)
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


# --- the resume-preview renderer (design §F.1) ------------------------


def _preview_int(value):
    """A non-negative integer, or None (bool is not an int here).

    ``offset``/``resume_offset``/``position``/``count``/``layer`` are
    serialized from Rust unsigned integers, so a real value is a plain
    ``int``.  A missing or wrongly-typed one reads as None and the caller
    says so rather than printing a guess.
    """
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _bytes_text(value):
    """A byte offset as ``244,118``, or None."""
    offset = _preview_int(value)
    if offset is None:
        return None
    return "{:,}".format(offset)


def _xy_text(value):
    """``X132.4 Y88.1`` from a two-element numeric list, or None."""
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    x = _num(value[0])
    y = _num(value[1])
    if x is None or y is None:
        return None
    return "X%s Y%s" % (_fmt(x), _fmt(y))


def _feature_text(value):
    """A human feature label from the FeatureClass Debug name, or None.

    An unrecognized name is returned verbatim — the producer may grow a
    class, and inventing a translation would be the contract-drift this
    module exists to avoid.
    """
    name = _text(value)
    if name is None:
        return None
    return _FEATURE_LABELS.get(name, name)


def _provenance_suffix(value):
    """ " (confirmed by the slicer's layer mark)" / " (inferred from the
    model)" / "" for the ``layer_provenance`` wire field.

    Only the two values the producer documents are rendered; an absent field
    (an old daemon) or any unrecognized value yields the empty suffix — the
    layer is then stated with no provenance claim rather than a guessed one
    (agent-contract discipline: never label a provenance the wire does not
    carry).
    """
    provenance = _text(value)
    if provenance == "journal":
        return " (confirmed by the slicer's layer mark)"
    if provenance == "inferred":
        return " (inferred from the model)"
    return ""


def preview_position_line(detail):
    """ "stop N of M" — the rep-position headline, best effort.

    Re-derived on every reposition from the fresh ``detail`` map, so the
    operator always sees which stop they are on even when adjacent stops
    are <1 mm apart and the nozzle appears not to move.
    """
    if not isinstance(detail, dict):
        return "Resume-point preview (plrd sent no stop details)"
    position = _preview_int(detail.get("position"))
    count = _preview_int(detail.get("count"))
    if position is not None and count is not None:
        return "Resume-point preview — stop %d of %d" % (position, count)
    if position is not None:
        return "Resume-point preview — stop %d" % (position,)
    return "Resume-point preview"


def preview_detail_lines(detail):
    """The current stop's full readout, re-emitted on EVERY reposition.

    Adjacent stops can be sub-millimetre apart, so the byte offset and the
    hover XY — not visible motion — are the alignment feedback (the ruling
    forbids relying on the nozzle moving).  Both are printed here every
    pause; within one arc source line a ±1 nudge moves the XY while the
    offset holds, which is why the XY is ALWAYS shown, not only when the
    offset changes (design §12).

    Missing or wrongly-typed fields produce an explicit "plrd did not
    report ..." line rather than silence — the same defensive posture the
    binary-pause renderer takes.
    """
    if not isinstance(detail, dict):
        return [
            "plrd sent no readable stop details for this preview point "
            "(the detail field was missing or malformed) — prefer Abort and "
            "check the plrd report above."
        ]
    lines = []

    # The deposition line (byte offset) and the hover target (XY), the two
    # feedback quantities the ruling names.
    offset = _bytes_text(detail.get("offset"))
    xy = _xy_text(detail.get("xy"))
    z = _num(detail.get("z"))
    where = []
    if offset is not None:
        where.append("deposition line at byte %s" % (offset,))
    else:
        where.append("plrd did not report this stop's byte offset")
    if xy is not None:
        z_text = " (Z %s mm)" % (_fmt(z),) if z is not None else ""
        where.append("hover target %s%s" % (xy, z_text))
    else:
        where.append("plrd did not report a hover XY")
    lines.append("Position: %s." % ("; ".join(where),))

    # Layer + feature.  When a layer is marked its PROVENANCE is stated
    # honestly from the wire's ``layer_provenance``: "journal" (corroborated
    # by the slicer's layer mark) vs "inferred" (from the model alone).  An
    # old daemon predating the field sends nothing here — ``_provenance``
    # is then None and the layer is stated with no provenance claim, never a
    # fabricated "journal" (agent-contract discipline).
    layer = _preview_int(detail.get("layer"))
    feature = _feature_text(detail.get("feature"))
    facts = []
    if layer is not None:
        suffix = _provenance_suffix(detail.get("layer_provenance"))
        facts.append("layer %d%s" % (layer, suffix))
    else:
        facts.append("layer not yet marked (before the first layer change)")
    if feature is not None:
        facts.append("feature: %s" % (feature,))
    if detail.get("on_infill") is True:
        facts.append("infill — a good seam-hiding resume")
    lines.append("This point: %s." % ("; ".join(facts),))

    # Provenance of the stop: matched the evidence, or nudge-only.
    is_candidate = detail.get("is_candidate")
    if is_candidate is True:
        lines.append(
            "Provenance: this stop matched the crash evidence (a candidate "
            "resume line)."
        )
    elif is_candidate is False:
        lines.append(
            "Provenance: this stop is reachable only by nudging — it is "
            "outside the matched set, but still a valid last-printed line "
            "(the ragged edge can sit just outside the matcher's tolerance)."
        )

    # Where a resume would actually begin.
    resume = _bytes_text(detail.get("resume_offset"))
    if resume is not None:
        lines.append(
            "Accepting resumes at byte %s (the next deposition line)." % (resume,)
        )

    # The advisory re-print warning (before the skip-forward default).
    if detail.get("before_skip_forward") is True:
        lines.append(
            "WARNING: this point is BEFORE the safe skip-forward line; "
            "accepting re-prints existing geometry (the nozzle plows the "
            "printed wall)."
        )

    # A non-acceptable stop: the executor refuses accept here, so say so.
    if detail.get("acceptable") is False:
        lines.append(
            "This stop cannot be accepted — nothing remains to print past "
            "it. Nudge or step to a stop with printing still ahead, then "
            "accept that one."
        )

    # Boundary notice: a nudge past the first/last stop CLAMPS and re-emits
    # this same stop, so without this the prompt would look byte-identical
    # and the operator could not tell the nudge did nothing.  Stated from the
    # wire's ``at_boundary`` ("first"/"last"); absent/null -> nothing.
    at_boundary = _text(detail.get("at_boundary"))
    if at_boundary == "first":
        lines.append(
            "You are at the FIRST stop — a backward nudge or Prev cannot go "
            "earlier (it stays here)."
        )
    elif at_boundary == "last":
        lines.append(
            "You are at the LAST stop — a forward nudge or Next cannot go "
            "further (it stays here)."
        )
    return lines


def preview_prompt(data, deadline_text):
    """The resume-preview dialog for one preview ``awaiting_confirmation``.

    Buttons fire plain g-code commands and the console fallback names every
    one of them, so the whole preview is completable from a bare console on
    a client that renders no dialog at all (the portability floor).  A stop
    the daemon marks non-acceptable drops the Accept button — the operator
    is told why, and the console note repeats it — while every navigation
    command stays available.
    """
    if not isinstance(data, dict):
        data = {}
    detail = data.get("detail")
    kind = _text(data.get("confirm_kind"))
    texts = [where_line(data.get("step"), data.get("phase"), kind)]
    texts.append(preview_position_line(detail))
    texts.append(question(kind))
    texts.append(
        "The part shows where printing stopped: a ragged edge where "
        "extrusion ends. Move the hover point to that edge, then Accept."
    )
    texts.extend(preview_detail_lines(detail))
    # The daemon's own advisory message (what/why/how-to-answer). Rendered
    # with the Advisory tier expected, so the legitimate advisory preview
    # does not trip the mismatch note the binary pauses use.
    texts.extend(diagnosis_lines(data.get("diagnosis"), expected_tier=TIER_ADVISORY))
    if deadline_text:
        texts.append(deadline_text)

    acceptable = not (isinstance(detail, dict) and detail.get("acceptable") is False)
    buttons = []
    if acceptable:
        buttons.append(("Accept", CMD_ACCEPT, "primary"))
    buttons.extend(
        [
            ("< Prev", CMD_PREV, "secondary"),
            ("Next >", CMD_NEXT, "secondary"),
            ("-10", "%s BACK=10" % (CMD_NUDGE,), "info"),
            ("-1", "%s BACK=1" % (CMD_NUDGE,), "info"),
            ("+1", "%s FWD=1" % (CMD_NUDGE,), "info"),
            ("+10", "%s FWD=10" % (CMD_NUDGE,), "info"),
        ]
    )
    accept_fallback = (
        "%s to accept this point, " % (CMD_ACCEPT,)
        if acceptable
        else "(this stop is not acceptable — accept is unavailable here) "
    )
    fallbacks = [
        "Console: %s%s / %s to step between candidate points, "
        "%s FWD=1|BACK=1|FWD=10|BACK=10 to move along the toolpath, or %s to "
        "stop." % (accept_fallback, CMD_NEXT, CMD_PREV, CMD_NUDGE, CMD_ABORT),
    ]
    return Prompt(
        title="%s — align the resume point" % (TITLE,),
        texts=texts,
        buttons=buttons,
        footers=[("Abort recovery", CMD_ABORT, "error")],
        fallbacks=fallbacks,
    )


def confirm_prompt(data, deadline_text):
    """The full confirm-point dialog for one ``awaiting_confirmation``.

    ``deadline_text`` is the caller's honest statement of plrd's own
    abort-on-no-answer deadline (see :mod:`plr.recovery`), or None to say
    nothing about it — never a guessed number.  A resume-preview pause
    routes to :func:`preview_prompt`; every other kind renders the binary
    continue/abort dialog below.
    """
    if not isinstance(data, dict):
        data = {}
    kind = _text(data.get("confirm_kind"))
    if kind == KIND_PREVIEW:
        return preview_prompt(data, deadline_text)
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
