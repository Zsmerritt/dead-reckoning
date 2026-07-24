//! Whole-itinerary pre-flight: validate every commanded coordinate the
//! plan will emit BEFORE the plan is returned.
//!
//! Cartographer validates an entire calibration itinerary up front,
//! aggregating every out-of-bounds point rather than failing at the
//! first (`macros/axis_twist_compensation.py:87-111`,
//! `_validate_options`/`_validate_point`). This module does the same for
//! a recovery plan: it walks every step's commands, collects **every**
//! violation, and returns them all as one typed
//! [`PlanRejection::ItineraryOutOfBounds`].
//!
//! # What is checked, and the frames it is sound in
//!
//! A recovery plan runs in several coordinate frames (the homed XY
//! frame, the shifted kinematic frame, and — after the true-frame
//! re-declaration — the file's g-code frame), so a naive "every Z is
//! within `[position_min, position_max]`" would false-positive on
//! g-code-frame moves. The pre-flight therefore checks only what is
//! frame-sound:
//!
//! * **Contact anchoring** — the probe-approach `G0`/`G1` XY target must
//!   equal the analyzer's already-selected contact point. This is the
//!   central "the emitted travel targets equal the selected zone"
//!   guarantee; it is frame-clean because both are absolute homed XY.
//! * **Probe site within the machine** — the selected contact point lies
//!   inside the known X/Y travel limits (skipped when unknown).
//! * **Absolute-frame travel bounds** — walking the plan while tracking
//!   `G90`/`G91`, every *absolute* `G0`/`G1` literal X/Y is inside the
//!   known X/Y limits, and every absolute literal Z is at or above the Z
//!   rail floor `position_min` (and below `z_max` when known). Relative
//!   (`G91`) moves carry deltas, not positions, so their coordinates are
//!   not bounds-checked. Placeholder coordinates (`{true_z}`,
//!   `{restore_accel}`) are runtime values and are skipped.
//! * **Shifted-frame declaration** — the `SET_KINEMATIC_POSITION Z=` of
//!   the shifted-frame step equals the envelope's `shifted_declare_z`
//!   and sits within `[position_min, z_max]`.
//!
//! Every finding is a [`BoundsViolation`]; a non-empty set is a
//! [`PlanRejection`].
//!
//! # The generated recovery file is checked too
//!
//! Motion that used to live in the plan's `Entry` step — the entry moves
//! (travel above the part, descend, prime) — now lives inside the
//! GENERATED RECOVERY FILE, together with the post-`G28` re-park travel
//! and the purge. That file is played back by Klipper directly, so it has
//! no per-step verification mechanism and a bad coordinate there surfaces
//! only as a mid-recovery "Move out of range" — AFTER the probe
//! established the Z reference. [`preflight_recovery_file`] therefore
//! applies the identical absolute-frame bounds walk to the generated
//! file's preamble, aggregating into the same
//! [`PlanRejection::ItineraryOutOfBounds`]. Both are run on the same build
//! path, so "every commanded coordinate is bounds-checked before the plan
//! is returned" remains literally true across the plan/file split.

use serde::Serialize;

use crate::plan::{fmt_num, Phase, RecoveryPlan, RESTORE_ACCEL_PLACEHOLDER, TRUE_Z_PLACEHOLDER};
use crate::resume_file::GeneratedRecoveryFile;

/// Quantization slack: commands render coordinates at five decimals
/// ([`fmt_num`]), so a re-parsed value may differ from the exact one by
/// up to half an ulp of that quantization. Bounds and equalities allow
/// this much slop.
const SLACK: f64 = 1e-4;

/// The bounds the pre-flight validates a plan against.
#[derive(Debug, Clone, Copy)]
pub struct ItineraryBounds {
    /// `(min, max)` X travel limits, mm (skipped when `None`).
    pub x: Option<(f64, f64)>,
    /// `(min, max)` Y travel limits, mm (skipped when `None`).
    pub y: Option<(f64, f64)>,
    /// Z rail `position_max`, mm (skipped when `None`).
    pub z_max: Option<f64>,
    /// Z rail `position_min`, mm — the shifted-frame anchor and the
    /// floor every absolute Z must clear.
    pub position_min: f64,
    /// The analyzer's selected contact point the probe approach must
    /// travel to.
    pub contact_point: [f64; 2],
}

/// Which kind of itinerary check a violation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum ViolationKind {
    /// A commanded coordinate fell outside a known axis travel limit.
    AxisLimit,
    /// The probe-approach travel target did not equal the selected
    /// contact point.
    ContactMismatch,
    /// The shifted-frame declaration disagreed with the envelope.
    ShiftedFrameZ,
}

impl ViolationKind {
    /// Stable tag string.
    #[must_use]
    pub fn tag(self) -> &'static str {
        match self {
            ViolationKind::AxisLimit => "axis-limit",
            ViolationKind::ContactMismatch => "contact-mismatch",
            ViolationKind::ShiftedFrameZ => "shifted-frame-z",
        }
    }
}

/// One out-of-bounds finding.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BoundsViolation {
    /// The step whose command carried the coordinate.
    pub step_id: u32,
    /// The axis (`'X'`, `'Y'`, `'Z'`).
    pub axis: char,
    /// The offending value.
    pub value: f64,
    /// The lower bound it should have respected, if any.
    pub min: Option<f64>,
    /// The upper bound it should have respected, if any.
    pub max: Option<f64>,
    /// Which check failed.
    pub kind: ViolationKind,
}

impl BoundsViolation {
    /// A human-readable one-liner.
    #[must_use]
    pub fn describe(&self) -> String {
        let bound = match (self.min, self.max) {
            (Some(lo), Some(hi)) => format!("[{}, {}]", fmt_num(lo), fmt_num(hi)),
            (Some(lo), None) => format!(">= {}", fmt_num(lo)),
            (None, Some(hi)) => format!("<= {}", fmt_num(hi)),
            (None, None) => "the selected value".to_owned(),
        };
        format!(
            "step {} {} {} = {} violates {} ({})",
            self.step_id,
            self.kind.tag(),
            self.axis,
            fmt_num(self.value),
            bound,
            self.kind.tag()
        )
    }
}

/// A typed pre-flight rejection.
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize)]
pub enum PlanRejection {
    /// One or more commanded coordinates would leave the machine's
    /// bounds or disagree with the selected contact zone. Every
    /// violation is listed (aggregated, not first-fail).
    #[error("itinerary out of bounds: {} violation(s)", violations.len())]
    ItineraryOutOfBounds {
        /// Every finding.
        violations: Vec<BoundsViolation>,
    },
}

/// The first word of a command, uppercased.
fn first_word(command: &str) -> String {
    command
        .split_whitespace()
        .next()
        .unwrap_or("")
        .to_ascii_uppercase()
}

/// A literal axis coordinate (`X10`, `Z=-1.15`) in a `G0`/`G1` word, or
/// `None` for a placeholder / missing / unparsable value.
fn axis_literal(command: &str, axis: char) -> Option<f64> {
    let upper = axis.to_ascii_uppercase();
    let lower = axis.to_ascii_lowercase();
    for word in command.split_whitespace().skip(1) {
        // Skip words that are not this axis (e.g. `X20`, `F1800` when
        // scanning for `Y`); only the requested axis letter is examined.
        // NOTE: must `continue`, not `?` — a `?` here would abandon the
        // scan at the first non-matching word and never reach a later
        // coordinate (e.g. the `Y` in `G0 X.. Y..`).
        let Some(rest) = word
            .strip_prefix(upper)
            .or_else(|| word.strip_prefix(lower))
        else {
            continue;
        };
        let value = rest.strip_prefix('=').unwrap_or(rest);
        if value.contains('{') {
            return None; // a placeholder, not a literal
        }
        if let Ok(v) = value.parse::<f64>() {
            return Some(v);
        }
    }
    None
}

/// Validates the whole plan itinerary (see the module docs).
///
/// # Errors
///
/// [`PlanRejection::ItineraryOutOfBounds`] listing every violation.
pub fn preflight_itinerary(
    plan: &RecoveryPlan,
    bounds: &ItineraryBounds,
) -> Result<(), PlanRejection> {
    let mut v: Vec<BoundsViolation> = Vec::new();

    check_contact_anchor(plan, bounds, &mut v);
    check_contact_within_limits(plan, bounds, &mut v);
    check_shifted_frame(plan, bounds, &mut v);
    check_absolute_travel(plan, bounds, &mut v);

    if v.is_empty() {
        Ok(())
    } else {
        Err(PlanRejection::ItineraryOutOfBounds { violations: v })
    }
}

/// The probe-approach travel target must equal the selected contact
/// point.
fn check_contact_anchor(
    plan: &RecoveryPlan,
    bounds: &ItineraryBounds,
    out: &mut Vec<BoundsViolation>,
) {
    let Some(step) = plan.steps_in_phase(Phase::ProbeApproach).next() else {
        return;
    };
    let [cx, cy] = bounds.contact_point;
    for command in &step.commands {
        if !matches!(first_word(command).as_str(), "G0" | "G1") {
            continue;
        }
        if let Some(x) = axis_literal(command, 'X') {
            if (x - cx).abs() > SLACK {
                out.push(BoundsViolation {
                    step_id: step.id,
                    axis: 'X',
                    value: x,
                    min: Some(cx),
                    max: Some(cx),
                    kind: ViolationKind::ContactMismatch,
                });
            }
        }
        if let Some(y) = axis_literal(command, 'Y') {
            if (y - cy).abs() > SLACK {
                out.push(BoundsViolation {
                    step_id: step.id,
                    axis: 'Y',
                    value: y,
                    min: Some(cy),
                    max: Some(cy),
                    kind: ViolationKind::ContactMismatch,
                });
            }
        }
    }
}

/// The selected contact point must lie within the known XY limits.
fn check_contact_within_limits(
    plan: &RecoveryPlan,
    bounds: &ItineraryBounds,
    out: &mut Vec<BoundsViolation>,
) {
    let step_id = plan
        .steps_in_phase(Phase::ProbeApproach)
        .next()
        .map_or(0, |s| s.id);
    let [cx, cy] = bounds.contact_point;
    if let Some((lo, hi)) = bounds.x {
        if cx < lo - SLACK || cx > hi + SLACK {
            out.push(BoundsViolation {
                step_id,
                axis: 'X',
                value: cx,
                min: Some(lo),
                max: Some(hi),
                kind: ViolationKind::AxisLimit,
            });
        }
    }
    if let Some((lo, hi)) = bounds.y {
        if cy < lo - SLACK || cy > hi + SLACK {
            out.push(BoundsViolation {
                step_id,
                axis: 'Y',
                value: cy,
                min: Some(lo),
                max: Some(hi),
                kind: ViolationKind::AxisLimit,
            });
        }
    }
}

/// The shifted-frame `SET_KINEMATIC_POSITION Z=` must equal the
/// envelope's declaration and sit within `[position_min, z_max]`.
fn check_shifted_frame(
    plan: &RecoveryPlan,
    bounds: &ItineraryBounds,
    out: &mut Vec<BoundsViolation>,
) {
    let Some(step) = plan.steps_in_phase(Phase::ShiftedFrame).next() else {
        return;
    };
    let expected = plan.envelope.shifted_declare_z;
    for command in &step.commands {
        if first_word(command) != "SET_KINEMATIC_POSITION" {
            continue;
        }
        // The declaration's Z word (never a placeholder on the shifted
        // frame step).
        if let Some(z) = z_word(command) {
            if (z - expected).abs() > SLACK
                || z < bounds.position_min - SLACK
                || bounds.z_max.is_some_and(|zm| z > zm + SLACK)
            {
                out.push(BoundsViolation {
                    step_id: step.id,
                    axis: 'Z',
                    value: z,
                    min: Some(bounds.position_min),
                    max: bounds.z_max,
                    kind: ViolationKind::ShiftedFrameZ,
                });
            }
        }
    }
}

/// A `Z=<v>` word (as in `SET_KINEMATIC_POSITION Z=...`), literal only.
fn z_word(command: &str) -> Option<f64> {
    for word in command.split_whitespace().skip(1) {
        if let Some(rest) = word.strip_prefix("Z=").or_else(|| word.strip_prefix('Z')) {
            if rest.contains('{') {
                return None;
            }
            return rest.parse::<f64>().ok();
        }
    }
    None
}

/// Bounds-checks one ABSOLUTE-frame motion command's literal X/Y/Z
/// against the machine limits, attributing findings to `step_id`. Shared
/// by the plan walk and the recovery-file walk so both enforce exactly
/// the same rule.
fn check_absolute_command(
    command: &str,
    step_id: u32,
    bounds: &ItineraryBounds,
    out: &mut Vec<BoundsViolation>,
) {
    if let (Some((lo, hi)), Some(x)) = (bounds.x, axis_literal(command, 'X')) {
        if x < lo - SLACK || x > hi + SLACK {
            out.push(BoundsViolation {
                step_id,
                axis: 'X',
                value: x,
                min: Some(lo),
                max: Some(hi),
                kind: ViolationKind::AxisLimit,
            });
        }
    }
    if let (Some((lo, hi)), Some(y)) = (bounds.y, axis_literal(command, 'Y')) {
        if y < lo - SLACK || y > hi + SLACK {
            out.push(BoundsViolation {
                step_id,
                axis: 'Y',
                value: y,
                min: Some(lo),
                max: Some(hi),
                kind: ViolationKind::AxisLimit,
            });
        }
    }
    if let Some(z) = axis_literal(command, 'Z') {
        let below = z < bounds.position_min - SLACK;
        let above = bounds.z_max.is_some_and(|zm| z > zm + SLACK);
        if below || above {
            out.push(BoundsViolation {
                step_id,
                axis: 'Z',
                value: z,
                min: Some(bounds.position_min),
                max: bounds.z_max,
                kind: ViolationKind::AxisLimit,
            });
        }
    }
}

/// Walk the plan tracking `G90`/`G91`; bounds-check every absolute
/// `G0`/`G1` literal coordinate.
fn check_absolute_travel(
    plan: &RecoveryPlan,
    bounds: &ItineraryBounds,
    out: &mut Vec<BoundsViolation>,
) {
    // The first motion command (probe approach) sends its own `G90`; a
    // conservative absolute start also matches Klipper's default.
    let mut absolute = true;
    for step in &plan.steps {
        for command in &step.commands {
            match first_word(command).as_str() {
                "G90" => absolute = true,
                "G91" => absolute = false,
                "G0" | "G1" if absolute => {
                    check_absolute_command(command, step.id, bounds, out);
                }
                _ => {}
            }
        }
    }
    // Placeholder coordinates never reach here (axis_literal returns
    // None for `{...}`), so the true-Z and restore-accel steps do not
    // false-positive.
    debug_assert!(
        !out.iter().any(|viol| viol.value.is_nan()),
        "no NaN coordinate should be reported: {out:?}"
    );
    let _ = (RESTORE_ACCEL_PLACEHOLDER, TRUE_Z_PLACEHOLDER); // documented as skipped
}

/// Step id attributed to violations found in the generated recovery
/// file's preamble (the file is not a plan step; `0` marks "the generated
/// file" in [`BoundsViolation::describe`] output).
pub const RECOVERY_FILE_STEP_ID: u32 = 0;

/// Whole-file pre-flight for the GENERATED RECOVERY FILE: the same
/// absolute-frame bounds walk [`preflight_itinerary`] applies to the plan,
/// applied to the file's preamble — the re-park travel, the purge, and the
/// entry moves that used to be the plan's `Entry` step (see the module
/// docs).
///
/// Only the preamble is walked: the verbatim tail is the operator's own
/// sliced file, whose coordinates the printer already accepted before the
/// crash, and re-validating it would false-positive on every legitimate
/// g-code-frame move.
///
/// # Errors
///
/// [`PlanRejection::ItineraryOutOfBounds`] listing every violation.
pub fn preflight_recovery_file(
    file: &GeneratedRecoveryFile,
    bounds: &ItineraryBounds,
) -> Result<(), PlanRejection> {
    let mut v: Vec<BoundsViolation> = Vec::new();
    // Klipper powers up absolute; the generated preamble asserts G90
    // before its own moves, and the entry moves may switch modes.
    let mut absolute = true;
    for line in String::from_utf8_lossy(file.preamble()).lines() {
        let command = line.trim();
        let word = first_word(command);
        match word.as_str() {
            "G90" => absolute = true,
            "G91" => absolute = false,
            // The SHARED motion set (arcs included), so this walk and the
            // heating gate cannot drift apart on which g-codes move.
            _ if absolute && crate::resume_file::is_motion_command(&word) => {
                check_absolute_command(command, RECOVERY_FILE_STEP_ID, bounds, &mut v);
            }
            _ => {}
        }
    }
    if v.is_empty() {
        Ok(())
    } else {
        Err(PlanRejection::ItineraryOutOfBounds { violations: v })
    }
}

#[cfg(test)]
mod tests {
    use super::{
        preflight_itinerary, BoundsViolation, ItineraryBounds, PlanRejection, ViolationKind,
    };
    use crate::envelope::{compute_envelope, EnvelopeParams, OvershootTerm};
    use crate::plan::{
        AbortReason, FailureAction, Phase, Predicate, RecoveryPlan, RecoveryStep, Verification,
    };

    fn bare_step(id: u32, phase: Phase, commands: Vec<String>) -> RecoveryStep {
        RecoveryStep {
            id,
            phase,
            summary: String::new(),
            commands,
            pre_verify: vec![],
            verify: vec![],
            compute: None,
            cleanup_commands: vec![],
            on_failure: FailureAction::Abort {
                reason: AbortReason::ApproachFailed,
            },
        }
    }

    /// A minimal plan: a shifted-frame declare and a probe approach to
    /// the contact point, with a `position_min` of -2 and a declared Z
    /// of -1.15 (envelope 0.85).
    fn plan(approach: Vec<String>, shifted_z: &str) -> RecoveryPlan {
        let envelope = compute_envelope(
            EnvelopeParams {
                expected_gap: 0.2,
                overshoot: OvershootTerm::PostTriggerTravel { probe_speed: 1.0 },
                margin: 0.5,
            },
            -2.0,
        )
        .unwrap();
        RecoveryPlan {
            steps: vec![
                bare_step(
                    1,
                    Phase::ShiftedFrame,
                    vec![format!("SET_KINEMATIC_POSITION Z={shifted_z}")],
                ),
                bare_step(2, Phase::ProbeApproach, approach),
            ],
            envelope,
            resume_file: "x_RECOVERY.gcode".to_owned(),
            resume_offset: 0,
            requires_clean_nozzle_confirmation: false,
            recovery_file: crate::resume_file::RecoveryFileSpec::default(),
            debug_confirm_each_step: false,
            warnings: vec![],
        }
    }

    fn bounds() -> ItineraryBounds {
        ItineraryBounds {
            x: Some((0.0, 200.0)),
            y: Some((0.0, 200.0)),
            z_max: Some(250.0),
            position_min: -2.0,
            contact_point: [20.0, 10.0],
        }
    }

    #[test]
    fn clean_itinerary_passes() {
        let p = plan(
            vec!["G90".to_owned(), "G0 X20 Y10 F6000".to_owned()],
            "-1.15",
        );
        assert!(preflight_itinerary(&p, &bounds()).is_ok());
        let _ = Verification::new("x", "y", Predicate::BoolTrue); // keep import used
    }

    #[test]
    fn contact_mismatch_is_caught() {
        // Approach travels somewhere other than the contact point.
        let p = plan(
            vec!["G90".to_owned(), "G0 X50 Y10 F6000".to_owned()],
            "-1.15",
        );
        let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
            preflight_itinerary(&p, &bounds())
        else {
            panic!("expected rejection");
        };
        assert!(violations
            .iter()
            .any(|v| v.kind == ViolationKind::ContactMismatch && v.axis == 'X'));
    }

    #[test]
    fn out_of_axis_limit_is_caught_and_aggregated() {
        // X beyond the 200 limit AND the contact point corrupted: BOTH
        // are reported (aggregate, not first-fail).
        let mut b = bounds();
        b.contact_point = [250.0, 10.0]; // outside X limit and != command
        let p = plan(
            vec!["G90".to_owned(), "G0 X250 Y10 F6000".to_owned()],
            "-1.15",
        );
        let Err(PlanRejection::ItineraryOutOfBounds { violations }) = preflight_itinerary(&p, &b)
        else {
            panic!("expected rejection");
        };
        assert!(violations
            .iter()
            .any(|v| v.kind == ViolationKind::AxisLimit && v.axis == 'X'));
    }

    #[test]
    fn shifted_frame_disagreement_is_caught() {
        // A corrupted shifted-frame Z that disagrees with the envelope.
        let p = plan(vec!["G90".to_owned(), "G0 X20 Y10 F6000".to_owned()], "-99");
        let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
            preflight_itinerary(&p, &bounds())
        else {
            panic!("expected rejection");
        };
        assert!(violations
            .iter()
            .any(|v| v.kind == ViolationKind::ShiftedFrameZ));
    }

    #[test]
    fn contact_mismatch_on_y_is_caught() {
        let p = plan(
            vec!["G90".to_owned(), "G0 X20 Y99 F6000".to_owned()],
            "-1.15",
        );
        let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
            preflight_itinerary(&p, &bounds())
        else {
            panic!("expected rejection");
        };
        assert!(violations
            .iter()
            .any(|v| v.kind == ViolationKind::ContactMismatch && v.axis == 'Y'));
    }

    #[test]
    fn contact_point_outside_the_y_limit_is_caught() {
        // The contact point itself is beyond the Y travel limit; the
        // approach faithfully travels to it, so only the AxisLimit fires
        // (no ContactMismatch).
        let mut b = bounds();
        b.contact_point = [20.0, 250.0];
        let p = plan(
            vec!["G90".to_owned(), "G0 X20 Y250 F6000".to_owned()],
            "-1.15",
        );
        let Err(PlanRejection::ItineraryOutOfBounds { violations }) = preflight_itinerary(&p, &b)
        else {
            panic!("expected rejection");
        };
        assert!(violations
            .iter()
            .any(|v| v.kind == ViolationKind::AxisLimit && v.axis == 'Y'));
        assert!(!violations
            .iter()
            .any(|v| v.kind == ViolationKind::ContactMismatch));
    }

    #[test]
    fn absolute_travel_out_of_x_y_and_z_bounds_all_caught() {
        // A later absolute move breaches X, Y (above their maxes) and Z
        // (below position_min and above z_max, tested separately).
        let mut p = plan(
            vec!["G90".to_owned(), "G0 X20 Y10 F6000".to_owned()],
            "-1.15",
        );
        p.steps.push(bare_step(
            3,
            Phase::ParkForReheat,
            vec![
                "G90".to_owned(),
                "G0 X999 Y999 F1200".to_owned(),
                "G1 Z-99 F1200".to_owned(),
            ],
        ));
        let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
            preflight_itinerary(&p, &bounds())
        else {
            panic!("expected rejection");
        };
        assert!(violations
            .iter()
            .any(|v| v.kind == ViolationKind::AxisLimit && v.axis == 'X' && v.step_id == 3));
        assert!(violations
            .iter()
            .any(|v| v.kind == ViolationKind::AxisLimit && v.axis == 'Y' && v.step_id == 3));
        // Z below the rail floor (position_min -2).
        assert!(violations
            .iter()
            .any(|v| v.kind == ViolationKind::AxisLimit && v.axis == 'Z' && v.step_id == 3));

        // And a Z above z_max is caught too.
        let mut p2 = plan(
            vec!["G90".to_owned(), "G0 X20 Y10 F6000".to_owned()],
            "-1.15",
        );
        p2.steps.push(bare_step(
            3,
            Phase::ParkForReheat,
            vec!["G90".to_owned(), "G1 Z9999 F1200".to_owned()],
        ));
        let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
            preflight_itinerary(&p2, &bounds())
        else {
            panic!("expected rejection for Z above z_max");
        };
        assert!(violations.iter().any(|v| v.axis == 'Z' && v.value > 9000.0));
    }

    #[test]
    fn unknown_limits_skip_the_axis_checks() {
        // With no XY limits and no z_max known, out-of-range XY/Z travel
        // is NOT flagged (the "where known" contract), and only the
        // contact anchor + shifted-frame checks run.
        let b = ItineraryBounds {
            x: None,
            y: None,
            z_max: None,
            position_min: -2.0,
            contact_point: [20.0, 10.0],
        };
        let mut p = plan(
            vec!["G90".to_owned(), "G0 X20 Y10 F6000".to_owned()],
            "-1.15",
        );
        p.steps.push(bare_step(
            3,
            Phase::ParkForReheat,
            vec!["G90".to_owned(), "G0 X999 Y999 F1200".to_owned()],
        ));
        assert!(preflight_itinerary(&p, &b).is_ok());
    }

    #[test]
    fn placeholder_z_in_the_shifted_frame_is_not_bounds_checked() {
        // A shifted-frame declaration carrying a runtime placeholder is
        // skipped by z_word (the value is not known at build time).
        let p = plan(
            vec!["G90".to_owned(), "G0 X20 Y10 F6000".to_owned()],
            "{true_z}",
        );
        assert!(preflight_itinerary(&p, &bounds()).is_ok());
    }

    #[test]
    fn violation_and_rejection_render_every_arm() {
        // Cover BoundsViolation::describe over all four bound shapes and
        // every ViolationKind tag, plus the PlanRejection Display.
        let both = BoundsViolation {
            step_id: 1,
            axis: 'X',
            value: 5.0,
            min: Some(0.0),
            max: Some(2.0),
            kind: ViolationKind::AxisLimit,
        };
        assert!(both.describe().contains("[0, 2]"));
        assert_eq!(ViolationKind::AxisLimit.tag(), "axis-limit");
        assert_eq!(ViolationKind::ContactMismatch.tag(), "contact-mismatch");
        assert_eq!(ViolationKind::ShiftedFrameZ.tag(), "shifted-frame-z");
        let lo_only = BoundsViolation {
            min: Some(-2.0),
            max: None,
            kind: ViolationKind::ShiftedFrameZ,
            ..both.clone()
        };
        assert!(lo_only.describe().contains(">= -2"));
        let hi_only = BoundsViolation {
            min: None,
            max: Some(9.0),
            kind: ViolationKind::ContactMismatch,
            ..both.clone()
        };
        assert!(hi_only.describe().contains("<= 9"));
        let neither = BoundsViolation {
            min: None,
            max: None,
            ..both.clone()
        };
        assert!(neither.describe().contains("the selected value"));
        let rejection = PlanRejection::ItineraryOutOfBounds {
            violations: vec![both, lo_only],
        };
        assert!(rejection.to_string().contains("2 violation(s)"));
    }

    #[test]
    fn relative_moves_and_placeholders_are_skipped() {
        // A relative lift (G91 Z1) and a placeholder Z must NOT trip the
        // absolute-frame Z floor.
        let mut p = plan(
            vec!["G90".to_owned(), "G0 X20 Y10 F6000".to_owned()],
            "-1.15",
        );
        p.steps.push(bare_step(
            3,
            Phase::RestoreFrame,
            vec![
                "G91".to_owned(),
                "G1 Z-5 F1200".to_owned(),
                "G90".to_owned(),
            ],
        ));
        p.steps.push(bare_step(
            4,
            Phase::TrueZDeclare,
            vec!["SET_KINEMATIC_POSITION Z={true_z}".to_owned()],
        ));
        // An absolute G-code word carrying a runtime placeholder is a
        // non-literal: axis_literal returns None and it is skipped.
        p.steps.push(bare_step(
            5,
            Phase::ParkForReheat,
            vec!["G90".to_owned(), "G1 Z={true_z} F1200".to_owned()],
        ));
        assert!(preflight_itinerary(&p, &bounds()).is_ok());
    }
}
