"""``awaiting_confirmation`` responses, exactly as plrd assembles them.

BUILT FROM THE PRODUCER, not from a plausible invention.  Each builder
cites the Rust that emits the shape and reproduces its field names,
types and null-ness:

* the response envelope and ``data`` map —
  crates/plrd/src/ctrlsock.rs ``report_pause`` (the ``json!`` at
  :849-861): ``outcome`` (the literal ``"awaiting_confirmation"``),
  ``resume_token`` (``plrc-<nanos hex>-<seq hex>``, next_token at :148-154),
  ``confirm_kind`` (``ConfirmKind::tag()``), ``step`` (u32), ``phase``,
  ``diagnosis``, ``detail``; ``ok`` is ``false`` and ``text`` is the
  daemon's own paused report (prefix + ``Diagnosis::full()``);
* the diagnosis object — crates/plr-recovery/src/diagnosis.rs
  ``Diagnosis`` (:145-186), serialized by plain serde derive, so the JSON
  names are the struct's fields.  ``tier`` is ``Tier``'s snake_case
  rename (:76-88).  ``measured`` / ``expected`` are the ``Measured`` /
  ``Expected`` structs or JSON null;
* the per-kind ``detail`` — crates/plrd/src/executor.rs:786-792
  (diagnosis pause), :961-971 (Z-height) and :882-888 (step-debug);
* the two diagnoses plrd itself constructs for the Z-height and
  step-debug pauses — executor.rs ``z_confirm_point`` (:903-971) and
  ``step_debug_point`` (:859-890), including their exact ``code``s
  (``z_confirm_before_resume`` / ``step_debug_pause``), which the daemon's
  own socket tests assert (ctrlsock.rs:1556, :1761).
"""


def diagnosis(
    code="drag_temp_below_floor",
    tier="confirmable",
    what="the drag baseline was captured 18 C colder than the nozzle is now",
    why=(
        "the ADXL baseline is temperature dependent, so a reading taken this "
        "far from the baseline temperature can trigger early or not at all"
    ),
    suggested_fix=(
        "set `drag_nozzle_temp` in printer.cfg's [plr] section to the "
        "temperature the baseline was captured at, or re-run PLR_NOISE_TEST"
    ),
    measured=None,
    expected=None,
    override_key=None,
):
    """One diagnosis object in the frozen wire shape (diagnosis.rs:145-186).

    Every field is always present: serde emits ``measured`` / ``expected`` /
    ``override_key`` as explicit JSON nulls when absent
    (no ``skip_serializing_if`` on those fields).
    """
    return {
        "code": code,
        "tier": tier,
        "what": what,
        "why": why,
        "suggested_fix": suggested_fix,
        "measured": measured,
        "expected": expected,
        "override_key": override_key,
    }


def measured(quantity="extruder.temperature", value=42.0, unit="C"):
    """``Measured`` (diagnosis.rs:107-114)."""
    return {"quantity": quantity, "value": value, "unit": unit}


def expected(quantity="extruder.temperature", low=55.0, high=65.0, unit="C"):
    """``Expected`` (diagnosis.rs:116-127); either bound may be null."""
    return {"quantity": quantity, "min": low, "max": high, "unit": unit}


# Distinguishes "the caller did not override this" from "the caller passed
# None", which the defensive-read tests need: a JSON null is a value plrd
# can really send.
_KEEP = object()


def pause_data(
    kind="diagnosis",
    token="plrc-17bd4c0f9a2-3",
    step=1,
    phase="preamble",
    diag=_KEEP,
    detail=_KEEP,
):
    """The ``data`` map of an ``awaiting_confirmation`` response."""
    return {
        "outcome": "awaiting_confirmation",
        "resume_token": token,
        "confirm_kind": kind,
        "step": step,
        "phase": phase,
        "diagnosis": diagnosis() if diag is _KEEP else diag,
        "detail": {"raised_by": "pre-flight"} if detail is _KEEP else detail,
    }


def pause(text=None, **kwargs):
    """A full ``recover_execute`` / ``recover_confirm`` pause response.

    ``ok`` is FALSE for a pause (ctrlsock.rs:850) — a paused recovery has
    not succeeded — which is why the plugin must branch on
    ``data.outcome`` and not on ``ok``.
    """
    data = pause_data(**kwargs)
    if text is None:
        text = (
            "recover: executing plan: 14 steps; resume bench.gcode @ byte 1048576\n"
            "recover: PAUSED at step %s [%s] awaiting confirmation (%s)\n"
            "CONFIRM [%s] %s\n"
            % (
                data["step"],
                data["phase"],
                data["confirm_kind"],
                data["diagnosis"]["code"],
                data["diagnosis"]["what"],
            )
        )
    return {"ok": False, "text": text, "data": data}


def z_height_pause(token="plrc-17bd4c0f9a2-7", live_z=0.6, target_z=0.6):
    """A ``confirm_z_before_resume`` pause (executor.rs:903-971)."""
    return pause(
        kind="z-height",
        token=token,
        step=9,
        phase="z-confirm-standoff",
        diag=diagnosis(
            code="z_confirm_before_resume",
            what=(
                "the toolhead is standing off at Z %s mm; confirm this matches "
                "what you see" % (live_z,)
            ),
            why=(
                "confirm_z_before_resume is set. Z was established by touching "
                "the part once and doing arithmetic on the result"
            ),
            suggested_fix=(
                "Answer `continue` if the standoff looks right. If it does not, "
                "answer `abort`"
            ),
            measured=measured("toolhead.position.2", live_z, "mm"),
            expected=expected("standoff target", target_z, target_z, "mm"),
        ),
        detail={
            "standoff_target_z": target_z,
            "live_toolhead_z": live_z,
            "derivation": (
                "true_Z = z_prev_top 0.4 + (halt_Z - trigger_Z), trigger read "
                "from plr.last_touch_result.median_z + z_offset (consensus touch)"
            ),
        },
    )


def step_debug_pause(token="plrc-17bd4c0f9a2-1", step=3, commands=None):
    """A ``debug_confirm_each_step`` pause (executor.rs:859-890)."""
    if commands is None:
        commands = ["M140 S60", "M104 S150"]
    return pause(
        kind="step-debug",
        token=token,
        step=step,
        phase="bed-heat",
        diag=diagnosis(
            code="step_debug_pause",
            what="about to run step %d [bed-heat]: set bed temperature" % (step,),
            why=(
                "debug_confirm_each_step is set, so execution stops before every "
                "step. Nothing is wrong"
            ),
            suggested_fix=(
                "Answer `continue` to send this step, or `abort` to stop here."
            ),
        ),
        detail={
            "summary": "set bed temperature",
            "commands": commands,
            "pre_verify": [],
            "verify": ["heater_bed.temperature within 60 +/- 2"],
            "cleanup_commands": [],
        },
    )


def preview_detail(
    offset=244118,
    resume_offset=244140,
    xy=None,
    z=1.0,
    layer=42,
    feature="InternalInfill",
    on_infill=True,
    is_candidate=True,
    position=3,
    count=5,
    before_skip_forward=False,
    acceptable=True,
):
    """The resume-preview ``detail`` map (crates/plrd/src/executor.rs
    ``preview_detail``), field-for-field.

    Every field the producer emits is present: ``offset`` / ``resume_offset``
    (u64 byte offsets), ``xy`` ([f64; 2], Klipper-internal frame), ``z``
    (f64, the stop's deposition Z), ``layer`` (u32 or JSON null — the
    producer carries only presence, no journal/inferred provenance),
    ``feature`` (the ``FeatureClass`` Debug name), ``on_infill`` /
    ``is_candidate`` / ``before_skip_forward`` / ``acceptable`` (bools), and
    ``position`` / ``count`` (1-based rep position).
    """
    return {
        "offset": offset,
        "resume_offset": resume_offset,
        "xy": [132.4, 88.1] if xy is None else xy,
        "z": z,
        "layer": layer,
        "feature": feature,
        "on_infill": on_infill,
        "is_candidate": is_candidate,
        "position": position,
        "count": count,
        "before_skip_forward": before_skip_forward,
        "acceptable": acceptable,
    }


def preview_pause(token="plrc-17bd4c0f9a2-5", step=7, detail=_KEEP, **detail_kwargs):
    """A resume-preview pause (executor.rs ``preview_point``).

    ``confirm_kind`` is ``"preview"`` and the diagnosis is ADVISORY tier
    with code ``resume_preview`` (the producer builds ``Tier::Advisory``, not
    Confirmable).  ``detail`` overrides the whole map; ``**detail_kwargs``
    tweak individual fields of the default one.
    """
    detail_map = preview_detail(**detail_kwargs) if detail is _KEEP else detail
    return pause(
        kind="preview",
        token=token,
        step=step,
        phase="resume-preview",
        diag=diagnosis(
            code="resume_preview",
            tier="advisory",
            what=(
                "hovering over stop 3 of 5 at X132.4 Y88.1 (byte 244118); move "
                "to the ragged edge on the part, then accept"
            ),
            why=(
                "accepting resumes at the next deposition line after this point "
                "(skip-forward)"
            ),
            suggested_fix=(
                "Answer accept to resume here, next/prev to step between "
                "representative points, nudge +/-1 (fine) or +/-10 (coarse) to "
                "move along the toolpath, or abort to stop."
            ),
        ),
        detail=detail_map,
    )


def completed(text="recover: plan complete; print resumed", exit_code=0):
    """The final success response (ctrlsock.rs:826-833)."""
    return {
        "ok": True,
        "text": text,
        "data": {"outcome": "completed", "exit": exit_code},
    }


def aborted(text="recover: ABORTED at step 9: confirmation-declined", exit_code=1):
    """The final abort/refusal response (ctrlsock.rs:826-833)."""
    return {
        "ok": False,
        "text": text,
        "data": {"outcome": "aborted-or-refused", "exit": exit_code},
    }


def unknown_token(
    text=("the confirmation timed out before the answer arrived; the recovery aborted"),
):
    """plrd's typed answer to a stale/expired token (ctrlsock.rs:755-784)."""
    return {"ok": False, "text": text, "data": {"outcome": "unknown-token"}}


def busy(text="another recover_execute is already running or awaiting confirmation"):
    """plrd's serialization refusal (ctrlsock.rs:631-645)."""
    return {"ok": False, "text": text, "data": {"outcome": "busy"}}
