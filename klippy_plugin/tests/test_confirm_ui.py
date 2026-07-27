"""Rendering of plrd's confirm-points: the three-part message, per kind.

The requirement these tests hold the line on is stated three ways and all
three must be visible to the operator: say WHY the recovery stopped,
SUGGEST a fix, and OFFER to continue anyway.  Every fixture here is built
from the producer (tests/confirm_fixtures.py cites the Rust for each
field), and the defensive cases feed the renderer things a REAL wire
message could carry — nulls, absent keys, wrong types — never invented
shapes.
"""

import confirm_fixtures as fx
import pytest

from plr import confirm_ui, prompts

DEADLINE = "If nothing answers, plrd aborts the recovery cleanly."


def texts(data, deadline=DEADLINE):
    return list(confirm_ui.confirm_prompt(data, deadline).texts)


def joined(data, deadline=DEADLINE):
    return "\n".join(texts(data, deadline))


# --- the three-part requirement ---------------------------------------


@pytest.mark.parametrize(
    "data",
    [
        pytest.param(fx.pause_data(), id="diagnosis"),
        pytest.param(fx.z_height_pause()["data"], id="z-height"),
        pytest.param(fx.step_debug_pause()["data"], id="step-debug"),
    ],
)
def test_every_kind_says_why_suggests_a_fix_and_offers_to_continue(data):
    prompt = confirm_ui.confirm_prompt(data, DEADLINE)
    body = "\n".join(prompt.texts)
    # WHY
    assert "Why: " in body
    assert data["diagnosis"]["why"][:30] in body
    # SUGGEST
    assert "Suggested fix: " in body
    assert data["diagnosis"]["suggested_fix"][:25] in body
    # OFFER — as a button AND as a console command, because the console is
    # the floor: a client that renders no dialog must still be able to act.
    assert prompt.buttons == [("Continue anyway", "PLR_RECOVER_CONTINUE", "warning")]
    assert prompt.footers == [("Abort recovery", "PLR_RECOVER_ABORT", "error")]
    fallback = "\n".join(prompt.fallbacks)
    assert "PLR_RECOVER_CONTINUE" in fallback
    assert "PLR_RECOVER_ABORT" in fallback


def test_the_question_distinguishes_the_three_confirm_kinds():
    # The three ConfirmKinds ask three different things
    # (crates/plrd/src/executor.rs:159-180), and conflating them is how an
    # operator answers the wrong question.
    assert "look right" in confirm_ui.question("z-height")
    assert "Run the next step?" in confirm_ui.question("step-debug")
    assert "Continue despite this?" in confirm_ui.question("diagnosis")
    # Three distinct sentences, not one with cosmetic edits.
    assert (
        len(
            {
                confirm_ui.question(kind)
                for kind in ("z-height", "step-debug", "diagnosis")
            }
        )
        == 3
    )


def test_an_unknown_kind_still_asks_and_admits_it_does_not_know():
    # A daemon that grows a fourth kind must not produce a prompt that
    # claims to know what it is asking.
    text = confirm_ui.question("teleport-check")
    assert "did not report which kind" in text
    prompt = confirm_ui.confirm_prompt(fx.pause_data(kind="teleport-check"), None)
    assert prompt.buttons  # it still offers both answers
    assert prompt.footers


# --- the diagnosis body ------------------------------------------------


def test_numbers_are_rendered_from_the_typed_fields():
    data = fx.pause_data(
        diag=fx.diagnosis(
            measured=fx.measured("purge_z", -0.25, "mm"),
            expected=fx.expected("purge_z", 0.0, None, "mm"),
        )
    )
    body = joined(data)
    assert "Measured: purge_z = -0.25 mm" in body
    assert "Expected: purge_z >= 0 mm" in body


def test_a_closed_band_and_a_point_band_read_naturally():
    band = joined(
        fx.pause_data(diag=fx.diagnosis(expected=fx.expected("t", 55.0, 65.0, "C")))
    )
    assert "Expected: t [55, 65] C" in band
    point = joined(
        fx.pause_data(
            diag=fx.diagnosis(expected=fx.expected("standoff", 0.6, 0.6, "mm"))
        )
    )
    assert "Expected: standoff 0.6 mm" in point
    upper = joined(
        fx.pause_data(diag=fx.diagnosis(expected=fx.expected("t", None, 65.0, "C")))
    )
    assert "Expected: t <= 65 C" in upper


@pytest.mark.parametrize(
    "value",
    [
        pytest.param(None, id="null"),
        pytest.param("hot", id="string"),
        pytest.param(True, id="bool"),
        pytest.param(float("nan"), id="nan"),
        pytest.param(float("inf"), id="inf"),
    ],
)
def test_an_unusable_measurement_is_omitted_not_fabricated(value):
    body = joined(
        fx.pause_data(diag=fx.diagnosis(measured=fx.measured("purge_z", value, "mm")))
    )
    assert "Measured:" not in body
    # ...and the parts that matter are still there.
    assert "Why: " in body and "Suggested fix: " in body


def test_a_null_measured_and_expected_are_simply_absent():
    # Serde emits both as explicit nulls when the diagnosis carries no
    # numbers (the common case), which must not read as "0".
    body = joined(fx.pause_data())
    assert "Measured:" not in body
    assert "Expected:" not in body
    assert "0" not in body.split("Why: ")[0]


def test_an_override_key_is_reported_as_a_config_edit_never_a_button():
    data = fx.pause_data(
        diag=fx.diagnosis(override_key="UNSAFE_allow_purge_z_below_bed")
    )
    prompt = confirm_ui.confirm_prompt(data, DEADLINE)
    body = "\n".join(prompt.texts)
    assert "UNSAFE_allow_purge_z_below_bed" in body
    assert "printer.cfg" in body and "[plr]" in body
    # THE RULE: no control anywhere in the dialog fires or names the
    # override — the escape hatch is an edit made while nothing is at stake
    # (crates/plr-recovery/src/diagnosis.rs:22-42).
    for label, gcode, _color in list(prompt.buttons) + list(prompt.footers):
        assert "UNSAFE" not in label
        assert "UNSAFE" not in (gcode or "")
    assert "UNSAFE" not in "\n".join(prompt.fallbacks)


def test_no_override_key_means_no_override_sentence():
    assert "override" not in joined(fx.pause_data()).lower()


@pytest.mark.parametrize(
    "diag",
    [
        pytest.param(None, id="null"),
        pytest.param("something went wrong", id="string"),
        pytest.param([], id="list"),
        pytest.param(42, id="number"),
    ],
)
def test_an_unreadable_diagnosis_still_asks_and_says_it_is_unexplained(diag):
    # FAIL-SAFE: the daemon is paused either way.  Rendering nothing, or
    # auto-answering, are both worse than telling the operator that the
    # explanation did not arrive and letting them decide.
    prompt = confirm_ui.confirm_prompt(fx.pause_data(diag=diag), DEADLINE)
    body = "\n".join(prompt.texts)
    assert "no readable explanation" in body
    assert "Why: unknown" in body
    assert "prefer Abort" in body
    assert "Suggested fix: " in body
    assert prompt.buttons and prompt.footers


@pytest.mark.parametrize("field", ["why", "suggested_fix"])
def test_a_missing_why_or_fix_is_named_rather_than_dropped(field):
    diag = fx.diagnosis()
    del diag[field]
    body = joined(fx.pause_data(diag=diag))
    assert "Why: " in body
    assert "Suggested fix: " in body
    assert "did not" in body


@pytest.mark.parametrize("value", [None, "", "   ", 7, {}])
def test_an_empty_prose_field_is_treated_as_missing(value):
    diag = fx.diagnosis(why=value)
    body = joined(fx.pause_data(diag=diag))
    assert "Why: plrd did not report a reason." in body


def test_a_missing_code_and_what_are_both_reported_honestly():
    diag = fx.diagnosis()
    del diag["what"]
    assert "What: drag_temp_below_floor (plrd sent no description)" in joined(
        fx.pause_data(diag=diag)
    )
    del diag["code"]
    assert "What: plrd sent neither a code nor a description." in joined(
        fx.pause_data(diag=diag)
    )


@pytest.mark.parametrize(
    "tier,expected_text",
    [
        pytest.param("hard", "tier 'hard'", id="hard"),
        pytest.param("advisory", "tier 'advisory'", id="advisory"),
        pytest.param(None, "tier 'unreported'", id="missing"),
    ],
)
def test_a_tier_other_than_confirmable_is_flagged_as_unexpected(tier, expected_text):
    # Only Tier::Confirmable reaches a confirm-point (executor.rs:785-811
    # routes Hard to a refusal instead).  Anything else means the plugin and
    # the daemon disagree, and the operator is told so rather than
    # reassured.
    body = joined(fx.pause_data(diag=fx.diagnosis(tier=tier)))
    assert expected_text in body
    assert "Prefer Abort" in body


def test_the_confirmable_tier_adds_no_noise():
    assert "unexpected for a confirmation" not in joined(fx.pause_data())


# --- per-kind detail ---------------------------------------------------


def test_z_height_detail_reports_the_height_and_its_derivation():
    body = joined(fx.z_height_pause(live_z=0.62, target_z=0.6)["data"])
    assert "Toolhead is standing off at Z = 0.62 mm." in body
    assert "Standoff target was Z = 0.6 mm." in body
    assert "z_prev_top" in body


def test_z_height_without_a_live_readback_says_to_judge_by_eye():
    # executor.rs:909 makes the live readback best-effort: a failed status
    # query reports null rather than turning a confirmation into an abort.
    pause = fx.z_height_pause()
    pause["data"]["detail"]["live_toolhead_z"] = None
    body = joined(pause["data"])
    assert "could not read the live toolhead Z" in body
    assert "judge the standoff by eye" in body


def test_step_debug_detail_lists_the_commands_about_to_be_sent():
    body = joined(fx.step_debug_pause(commands=["M140 S60", "M104 S150"])["data"])
    assert "Commands about to be sent: M140 S60; M104 S150" in body
    assert "Verified afterwards: heater_bed.temperature within 60 +/- 2" in body


def test_step_debug_truncates_a_pathological_command_list_and_counts_it():
    commands = ["G1 X%d" % i for i in range(20)]
    body = joined(fx.step_debug_pause(commands=commands)["data"])
    assert "G1 X11" in body
    assert "G1 X12" not in body
    assert "(and 8 more command(s)" in body


def test_a_step_that_only_verifies_says_so():
    body = joined(fx.step_debug_pause(commands=[])["data"])
    assert "sends no commands" in body


def test_a_diagnosis_pause_reports_what_raised_it():
    assert "Raised by: pre-flight" in joined(fx.pause_data())


@pytest.mark.parametrize("detail", [None, "nope", 7, []])
def test_an_unreadable_detail_never_breaks_the_prompt(detail):
    body = joined(fx.pause_data(detail=detail))
    assert "Why: " in body and "Suggested fix: " in body


# --- where the pause happened -----------------------------------------


def test_the_prompt_says_which_step_and_phase_paused():
    assert confirm_ui.where_line(9, "z-confirm-standoff", "z-height") == (
        "Paused at step 9 [z-confirm-standoff] (z-height confirmation)"
    )


@pytest.mark.parametrize("step", [None, "nine", 9.5, True])
def test_an_unreadable_step_number_is_not_invented(step):
    line = confirm_ui.where_line(step, "bed-heat", "step-debug")
    assert "unreported step" in line
    assert "9" not in line


def test_a_missing_phase_and_kind_degrade_to_something_honest():
    assert confirm_ui.where_line(4, None, None) == (
        "Paused at step 4 (unspecified confirmation)"
    )


# --- the deadline sentence --------------------------------------------


def test_the_deadline_sentence_is_included_when_given_and_omitted_otherwise():
    assert DEADLINE in texts(fx.pause_data())
    assert DEADLINE not in texts(fx.pause_data(), deadline=None)


# --- the envelope -----------------------------------------------------


def test_the_title_names_the_flow_and_the_prompt_is_ordered():
    prompt = confirm_ui.confirm_prompt(fx.pause_data(), DEADLINE)
    assert prompt.title == "Power-loss recovery — confirmation needed"
    # Where, then the question, then the explanation: the operator reads
    # what is being asked before the detail behind it.
    assert prompt.texts[0].startswith("Paused at step")
    assert prompt.texts[1] == confirm_ui.question("diagnosis")


@pytest.mark.parametrize("data", [None, "nope", [], 7])
def test_a_non_object_data_map_still_produces_an_answerable_prompt(data):
    prompt = confirm_ui.confirm_prompt(data, DEADLINE)
    assert prompt.buttons and prompt.footers
    assert "no readable explanation" in "\n".join(prompt.texts)


def test_no_prompt_promises_an_image():
    # No client renders images in an action prompt; promising one is a lie
    # the operator cannot check.
    prompt = confirm_ui.confirm_prompt(fx.z_height_pause()["data"], DEADLINE)
    body = "\n".join(list(prompt.texts) + list(prompt.fallbacks)).lower()
    for word in ("image", "photo", "picture", "screenshot", "webcam"):
        assert word not in body


# --- the action protocol is line-oriented -----------------------------


def test_a_newline_in_daemon_prose_cannot_shred_the_prompt():
    # `respond_info` splits on newlines and prefixes each line with `// `
    # (klippy/gcode.py:250-254), so a `\n` inside an action line emits a
    # second `// ` line no client can parse — from there the dialog is
    # gone.  The prose is plrd's, and confirm_ui deliberately renders
    # whatever arrives, so the collapse happens at the one choke point.
    data = fx.pause_data(
        diag=fx.diagnosis(
            what="first line\nsecond line",
            why="why line one\r\nwhy line two",
            suggested_fix="fix\tone\nfix two",
        )
    )
    emitted = []
    prompts.emit_prompt(emitted.append, confirm_ui.confirm_prompt(data, DEADLINE))
    action_lines = [line for line in emitted if line.startswith("action:")]
    assert action_lines, emitted
    for line in action_lines:
        assert "\n" not in line
        assert "\r" not in line
    # ...and the words survive, just on one line.
    body = "\n".join(action_lines)
    assert "first line second line" in body
    assert "why line one why line two" in body


@pytest.mark.parametrize(
    "raw,expected",
    [
        pytest.param("plain", "plain", id="plain"),
        pytest.param("two\nlines", "two lines", id="lf"),
        pytest.param("two\r\nlines", "two lines", id="crlf"),
        pytest.param("two\rlines", "two lines", id="cr"),
        pytest.param("wide\u2028break", "wide break", id="line-separator"),
        pytest.param("tab\tsplit", "tab split", id="tab"),
        pytest.param("  padded  ", "padded", id="padding"),
        pytest.param(None, None, id="non-string-passes-through"),
    ],
)
def test_one_line_collapses_everything_that_would_break_a_line(raw, expected):
    assert prompts.one_line(raw) == expected


def test_button_fields_are_collapsed_too():
    # A label or a gcode string with a newline would break the pipe fields
    # as well; nothing in the plugin does that today, but the label text is
    # one edit away from being built from daemon prose.
    line = prompts.action_prompt_button("two\nwords", "PLR_X\nY", "primary")
    assert line == "action:prompt_button two words|PLR_X Y|primary"


@pytest.mark.parametrize(
    "raw,expected",
    [
        pytest.param("Continue", "Continue", id="plain"),
        pytest.param("a|b", "a/b", id="pipe"),
        pytest.param("a|b\nc", "a/b c", id="pipe-and-newline"),
        pytest.param(None, None, id="none"),
    ],
)
def test_button_fields_neutralize_the_field_separator(raw, expected):
    # `|` separates <label>|<gcode>|<color>, so a pipe inside a field shifts
    # every following one — a client would render the tail of a label as the
    # g-code the button fires.  `one_line` deliberately does NOT cover this
    # (it handles line breaks); `field` does, where the separator lives.
    assert prompts.field(raw) == expected


def test_one_line_says_what_it_does_not_cover():
    # Documented boundary, pinned: one_line is about LINES.
    assert prompts.one_line("a|b") == "a|b"


def test_a_pipe_in_a_button_field_cannot_shift_the_other_fields():
    line = prompts.action_prompt_button("Continue|now", "PLR_X|Y", "warning")
    assert line == "action:prompt_button Continue/now|PLR_X/Y|warning"
    assert line.count("|") == 2


# --- the resume-preview renderer (design §F.1) ------------------------
#
# Every field asserted here is one the producer really sends
# (crates/plrd/src/executor.rs ``preview_detail`` — see
# tests/confirm_fixtures.py ``preview_detail`` for the field-by-field
# citation). The point of the branch is the console floor: the whole
# preview is completable from bare g-code, and the per-reposition readout
# is the alignment feedback because adjacent stops can be <1 mm apart.


def preview_texts(data, deadline=DEADLINE):
    return list(confirm_ui.confirm_prompt(data, deadline).texts)


def test_preview_routes_to_the_preview_prompt_and_names_the_stop():
    prompt = confirm_ui.confirm_prompt(fx.preview_pause()["data"], DEADLINE)
    body = "\n".join(prompt.texts)
    assert "Resume-point preview — stop 3 of 5" in body
    # The alignment quantities the ruling names: the byte offset AND the
    # hover XY, both from the detail map.
    assert "byte 244,118" in body
    assert "X132.4 Y88.1" in body
    assert "layer 42" in body
    assert "internal infill" in body
    assert "byte 244,140" in body  # where a resume would begin


def test_preview_buttons_fire_plain_commands_and_the_console_names_them_all():
    # THE PORTABILITY FLOOR: every button is a plain PLR_* command, and the
    # console fallback names every one, so a client that renders no dialog
    # can still drive the whole preview.
    prompt = confirm_ui.confirm_prompt(fx.preview_pause()["data"], DEADLINE)
    gcodes = [gcode for _label, gcode, _color in prompt.buttons]
    assert "PLR_RECOVER_ACCEPT" in gcodes
    assert "PLR_RECOVER_PREV" in gcodes
    assert "PLR_RECOVER_NEXT" in gcodes
    assert "PLR_RECOVER_NUDGE BACK=10" in gcodes
    assert "PLR_RECOVER_NUDGE BACK=1" in gcodes
    assert "PLR_RECOVER_NUDGE FWD=1" in gcodes
    assert "PLR_RECOVER_NUDGE FWD=10" in gcodes
    assert prompt.footers == [("Abort recovery", "PLR_RECOVER_ABORT", "error")]
    fallback = "\n".join(prompt.fallbacks)
    for cmd in (
        "PLR_RECOVER_ACCEPT",
        "PLR_RECOVER_NEXT",
        "PLR_RECOVER_PREV",
        "PLR_RECOVER_NUDGE",
        "PLR_RECOVER_ABORT",
    ):
        assert cmd in fallback


def test_the_readout_refreshes_on_every_reposition():
    # Adjacent stops can be sub-millimetre apart, so the operator must not
    # rely on visible motion: the byte offset and XY are re-emitted for each
    # stop.  Two different stops must produce different readouts.
    a = "\n".join(
        preview_texts(
            fx.preview_pause(offset=1000, xy=[10.0, 20.0], position=2)["data"]
        )
    )
    b = "\n".join(
        preview_texts(
            fx.preview_pause(offset=1004, xy=[10.4, 20.0], position=3)["data"]
        )
    )
    assert "byte 1,000" in a and "X10 Y20" in a and "stop 2 of" in a
    assert "byte 1,004" in b and "X10.4 Y20" in b and "stop 3 of" in b
    assert a != b


def test_within_arc_nudge_shows_the_xy_even_when_the_offset_holds():
    # A ±1 nudge between two chords of one arc keeps the byte offset (they
    # share the arc's source line) but moves the hover XY (design §12).  The
    # renderer must show the XY so the nudge is not read as "no effect".
    chord_a = "\n".join(
        preview_texts(fx.preview_pause(offset=500, xy=[100.0, 50.0])["data"])
    )
    chord_b = "\n".join(
        preview_texts(fx.preview_pause(offset=500, xy=[100.6, 50.3])["data"])
    )
    assert "byte 500" in chord_a and "byte 500" in chord_b  # offset holds
    assert "X100 Y50" in chord_a
    assert "X100.6 Y50.3" in chord_b  # XY moved — visible feedback
    assert chord_a != chord_b


def test_a_point_before_the_skip_forward_default_warns_about_reprinting():
    body = "\n".join(preview_texts(fx.preview_pause(before_skip_forward=True)["data"]))
    assert "BEFORE the safe skip-forward line" in body
    assert "re-prints existing geometry" in body


def test_a_non_acceptable_stop_drops_accept_and_says_why():
    prompt = confirm_ui.confirm_prompt(
        fx.preview_pause(acceptable=False)["data"], DEADLINE
    )
    gcodes = [gcode for _label, gcode, _color in prompt.buttons]
    assert "PLR_RECOVER_ACCEPT" not in gcodes  # accept is unavailable
    # Navigation stays available so the operator can move OFF the bad stop.
    assert "PLR_RECOVER_NEXT" in gcodes
    assert "PLR_RECOVER_NUDGE FWD=1" in gcodes
    body = "\n".join(prompt.texts)
    assert "cannot be accepted" in body
    fallback = "\n".join(prompt.fallbacks)
    assert "not acceptable" in fallback


def test_the_first_stop_says_a_backward_nudge_clamps():
    body = "\n".join(preview_texts(fx.preview_pause(at_boundary="first")["data"]))
    assert "FIRST stop" in body
    assert "cannot go earlier" in body


def test_the_last_stop_says_a_forward_nudge_clamps():
    body = "\n".join(preview_texts(fx.preview_pause(at_boundary="last")["data"]))
    assert "LAST stop" in body
    assert "cannot go further" in body


def test_an_interior_stop_carries_no_boundary_notice():
    body = "\n".join(preview_texts(fx.preview_pause(at_boundary=None)["data"]))
    assert "FIRST stop" not in body
    assert "LAST stop" not in body


def test_an_old_daemon_without_at_boundary_shows_no_notice():
    body = "\n".join(preview_texts(fx.preview_pause(at_boundary=fx._OMIT)["data"]))
    assert "FIRST stop" not in body
    assert "LAST stop" not in body


def test_a_nudge_only_stop_is_labelled_reachable_by_nudging():
    body = "\n".join(preview_texts(fx.preview_pause(is_candidate=False)["data"]))
    assert "reachable only by nudging" in body
    matched = "\n".join(preview_texts(fx.preview_pause(is_candidate=True)["data"]))
    assert "matched the crash evidence" in matched


def test_an_unmarked_layer_is_admitted_not_invented():
    body = "\n".join(preview_texts(fx.preview_pause(layer=None)["data"]))
    assert "layer not yet marked" in body
    # And no provenance the wire does not carry.
    assert "journal-confirmed" not in body
    assert "inferred" not in body
    assert "slicer's layer mark" not in body


def test_a_journal_confirmed_layer_states_its_provenance():
    body = "\n".join(
        preview_texts(fx.preview_pause(layer=42, layer_provenance="journal")["data"])
    )
    assert "layer 42" in body
    assert "confirmed by the slicer's layer mark" in body


def test_an_inferred_layer_says_so():
    body = "\n".join(
        preview_texts(fx.preview_pause(layer=42, layer_provenance="inferred")["data"])
    )
    assert "layer 42" in body
    assert "inferred from the model" in body


def test_an_old_daemon_without_provenance_states_the_layer_plainly():
    # The field is ABSENT (a daemon predating it): the layer is stated with
    # no provenance claim — never a fabricated "journal".
    body = "\n".join(
        preview_texts(fx.preview_pause(layer=42, layer_provenance=fx._OMIT)["data"])
    )
    assert "layer 42" in body
    assert "confirmed by the slicer" not in body
    assert "inferred from the model" not in body


def test_an_unrecognized_provenance_value_is_not_rendered():
    # A value the contract does not define (a daemon that grows a third
    # provenance) yields no claim rather than a guessed translation.
    body = "\n".join(
        preview_texts(fx.preview_pause(layer=42, layer_provenance="fabricated")["data"])
    )
    assert "layer 42" in body
    assert "confirmed by the slicer" not in body
    assert "inferred from the model" not in body


def test_an_unknown_feature_name_is_shown_verbatim():
    # A daemon that grows a FeatureClass must not render as a guess.
    body = "\n".join(preview_texts(fx.preview_pause(feature="LightningInfill")["data"]))
    assert "LightningInfill" in body


@pytest.mark.parametrize(
    "name,label",
    [
        ("InternalInfill", "internal infill"),
        ("OuterWall", "outer wall"),
        ("Surface", "surface"),
        ("SkirtBrim", "skirt/brim"),
    ],
)
def test_feature_labels_humanize_the_debug_names(name, label):
    body = "\n".join(preview_texts(fx.preview_pause(feature=name)["data"]))
    assert label in body


def test_the_advisory_preview_does_not_trip_the_tier_mismatch_note():
    # The preview diagnosis is Advisory by construction; the binary pauses'
    # "not confirmable" warning must NOT fire for it.
    body = "\n".join(preview_texts(fx.preview_pause()["data"]))
    assert "not 'confirmable'" not in body
    assert "unexpected for this pause" not in body


def test_a_preview_with_the_wrong_tier_is_still_flagged():
    data = fx.preview_pause()["data"]
    data["diagnosis"]["tier"] = "hard"
    body = "\n".join(preview_texts(data))
    assert "unexpected for this pause" in body


def test_a_missing_preview_detail_is_admitted_not_faked():
    data = fx.preview_pause()["data"]
    data["detail"] = None
    body = "\n".join(preview_texts(data))
    assert "no readable stop details" in body
    # The prompt still offers navigation and abort, so the operator can act.
    prompt = confirm_ui.confirm_prompt(data, DEADLINE)
    assert prompt.footers == [("Abort recovery", "PLR_RECOVER_ABORT", "error")]


def test_the_preview_question_is_distinct_and_names_the_workflow():
    text = confirm_ui.question("preview")
    assert "ragged edge" in text
    # Distinct from every other kind's question.
    others = {confirm_ui.question(k) for k in ("z-height", "step-debug", "diagnosis")}
    assert text not in others
