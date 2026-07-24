//! The recovery plan as **data**: strictly ordered, typed steps, each
//! carrying the commands to send, machine-readable verification
//! predicates, and a typed failure action. This crate never executes
//! anything — the daemon walks the plan, sends each step's commands,
//! evaluates each verification against Klipper status objects, and
//! aborts on the step's failure action when a predicate fails. It
//! **never** continues past a failed verification.
//!
//! # Verification semantics (contract with the daemon)
//!
//! * [`RecoveryStep::pre_verify`] — predicates that must hold **before**
//!   the step's commands are sent (e.g. the mandatory nozzle
//!   temperature check before `PROBE`).
//! * [`RecoveryStep::verify`] — predicates that must hold **after** the
//!   commands complete. Slow-converging predicates (temperatures) are
//!   polled until they hold or the daemon's timeout fires; a timeout is
//!   a verification failure.
//! * A failed predicate triggers [`RecoveryStep::on_failure`] — always
//!   [`FailureAction::Abort`] in v1.
//!
//! # Runtime-computed values
//!
//! The true-Z re-declaration (design doc §8, step 6) depends on the
//! probe result, which does not exist at plan time. Steps that need it
//! carry [`RecoveryStep::compute`] — a typed formula — and use the
//! literal placeholder `{true_z}` in their command strings. The daemon
//! evaluates the formula with [`true_z_at_halt`] and substitutes the
//! [`fmt_num`]-formatted result. No other placeholder exists.

use serde::{Deserialize, Serialize};

use crate::envelope::Envelope;
use crate::error::RecoveryError;

/// The placeholder substituted by the daemon with the computed true-Z
/// value (see the module docs).
pub const TRUE_Z_PLACEHOLDER: &str = "{true_z}";

/// Which §8 phase a step belongs to. The builder emits phases in
/// exactly this declaration order; the ordering invariants
/// ([`RecoveryPlan::idle_timeout_first`] and friends) verify it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Phase {
    /// §8.1a — disarm the idle timeout before anything else.
    IdleTimeout,
    /// §8.1b — energize the Z steppers (enabling never touches homed
    /// state; there is no M17 in Klipper).
    StepperEnable,
    /// §8.2 — bed to target, nozzle to the warm-but-below-ooze band.
    Preheat,
    /// §8.3 — home XY only. Never bare `G28`, never Z.
    HomeXy,
    /// §8.4 — freeze `z_thermal_adjust` before the shifted frame.
    TransformFreeze,
    /// §8.5a — declare the shifted frame (`SET_KINEMATIC_POSITION`).
    ShiftedFrame,
    /// §8.5b — XY travel to the selected contact point.
    ProbeApproach,
    /// §8.5c — the single-sample probe.
    Probe,
    /// §8.6 — true-Z arithmetic and kinematic re-declaration.
    TrueZDeclare,
    /// §8.7 — load the bed-mesh profile (probe already done).
    MeshLoad,
    /// §8.8 — final true-frame declaration.
    FinalDeclare,
    /// §8.9 — replay offsets, factors, modes, skew, temps, fans.
    RestoreFrame,
    /// §8.11 — entry move from above the part interior, prime.
    Entry,
    /// §8.12a — select the file, restore exclude-object state, seek.
    FileSelect,
    /// §8.12b — start playback (`M24`).
    ResumeStart,
}

impl Phase {
    /// Stable kebab-case name for rendering.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Phase::IdleTimeout => "idle-timeout",
            Phase::StepperEnable => "stepper-enable",
            Phase::Preheat => "preheat",
            Phase::HomeXy => "home-xy",
            Phase::TransformFreeze => "transform-freeze",
            Phase::ShiftedFrame => "shifted-frame",
            Phase::ProbeApproach => "probe-approach",
            Phase::Probe => "probe",
            Phase::TrueZDeclare => "true-z-declare",
            Phase::MeshLoad => "mesh-load",
            Phase::FinalDeclare => "final-declare",
            Phase::RestoreFrame => "restore-frame",
            Phase::Entry => "entry",
            Phase::FileSelect => "file-select",
            Phase::ResumeStart => "resume-start",
        }
    }
}

/// A machine-readable predicate over one status field.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Predicate {
    /// Numeric field within `epsilon` of `expected`.
    NumWithin {
        /// Expected value.
        expected: f64,
        /// Allowed absolute deviation.
        epsilon: f64,
    },
    /// Numeric field at least `min`.
    NumAtLeast {
        /// Inclusive lower bound.
        min: f64,
    },
    /// Temperature in `[min, max]` °C (polled until it holds).
    TempWithin {
        /// Inclusive lower bound, °C.
        min: f64,
        /// Inclusive upper bound, °C.
        max: f64,
    },
    /// String field contains `needle` (e.g. `homed_axes` contains
    /// `"z"`).
    Contains {
        /// Substring that must be present.
        needle: String,
    },
    /// String field equals `value` exactly.
    Equals {
        /// Expected value.
        value: String,
    },
    /// Boolean field is `true`.
    BoolTrue,
    /// Boolean field is `false`.
    BoolFalse,
    /// Field is present and, if numeric, finite.
    FinitePresent,
    /// 2-D matrix field has at least one non-empty row (the
    /// `mesh_matrix` gate).
    NonEmptyMatrix,
    /// Numeric field within `epsilon` of the step's runtime-computed
    /// value ([`RecoveryStep::compute`]). Only valid on steps that
    /// carry a computation.
    NumWithinComputed {
        /// Allowed absolute deviation.
        epsilon: f64,
    },
}

impl Predicate {
    /// Human-readable description for the rendered plan.
    #[must_use]
    pub fn describe(&self) -> String {
        match self {
            Predicate::NumWithin { expected, epsilon } => {
                format!("within {} of {}", fmt_num(*epsilon), fmt_num(*expected))
            }
            Predicate::NumAtLeast { min } => format!(">= {}", fmt_num(*min)),
            Predicate::TempWithin { min, max } => {
                format!("in [{}, {}] C", fmt_num(*min), fmt_num(*max))
            }
            Predicate::Contains { needle } => format!("contains {needle:?}"),
            Predicate::Equals { value } => format!("equals {value:?}"),
            Predicate::BoolTrue => "is true".to_owned(),
            Predicate::BoolFalse => "is false".to_owned(),
            Predicate::FinitePresent => "present and finite".to_owned(),
            Predicate::NonEmptyMatrix => "non-empty matrix".to_owned(),
            Predicate::NumWithinComputed { epsilon } => {
                format!("within {} of the computed true Z", fmt_num(*epsilon))
            }
        }
    }
}

/// One verification: which status object and field to read, and the
/// predicate that must hold.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Verification {
    /// Klipper status object name (e.g. `"toolhead"`, `"idle_timeout"`,
    /// `"extruder"`).
    pub object: String,
    /// Dotted field path within the object's status; numeric path
    /// segments index arrays (e.g. `"position.2"` is Z).
    pub field: String,
    /// The predicate that must hold.
    pub predicate: Predicate,
}

impl Verification {
    /// Builds a verification.
    #[must_use]
    pub fn new(object: &str, field: &str, predicate: Predicate) -> Self {
        Self {
            object: object.to_owned(),
            field: field.to_owned(),
            predicate,
        }
    }
}

/// Reason code carried by an abort.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AbortReason {
    /// `SET_IDLE_TIMEOUT` did not take effect.
    IdleTimeoutNotApplied,
    /// A Z stepper failed to enable.
    StepperEnableFailed,
    /// Preheat targets were not reached.
    PreheatFailed,
    /// XY homing failed.
    HomingFailed,
    /// `z_thermal_adjust` could not be frozen.
    TransformFreezeFailed,
    /// The shifted frame was not declared.
    ShiftedFrameNotDeclared,
    /// The XY approach did not reach the contact point.
    ApproachFailed,
    /// `PROBE` reported no trigger over the full envelope ("No trigger
    /// on probe after full movement"): the part was never touched and
    /// remains untouched beyond the envelope. Manual recovery required.
    ProbeNoTrigger,
    /// The true-Z re-declaration failed.
    TrueZDeclareFailed,
    /// The bed-mesh profile failed to load.
    MeshLoadFailed,
    /// The final true-frame declaration failed.
    FinalDeclareFailed,
    /// Frame/temperature restore failed.
    RestoreFailed,
    /// The entry move failed.
    EntryFailed,
    /// File selection / seek failed.
    FileSelectFailed,
    /// Playback did not start.
    ResumeStartFailed,
}

impl AbortReason {
    /// Stable reason-code string.
    #[must_use]
    pub fn code(self) -> &'static str {
        match self {
            AbortReason::IdleTimeoutNotApplied => "idle-timeout-not-applied",
            AbortReason::StepperEnableFailed => "stepper-enable-failed",
            AbortReason::PreheatFailed => "preheat-failed",
            AbortReason::HomingFailed => "homing-failed",
            AbortReason::TransformFreezeFailed => "transform-freeze-failed",
            AbortReason::ShiftedFrameNotDeclared => "shifted-frame-not-declared",
            AbortReason::ApproachFailed => "approach-failed",
            AbortReason::ProbeNoTrigger => "probe-no-trigger",
            AbortReason::TrueZDeclareFailed => "true-z-declare-failed",
            AbortReason::MeshLoadFailed => "mesh-load-failed",
            AbortReason::FinalDeclareFailed => "final-declare-failed",
            AbortReason::RestoreFailed => "restore-failed",
            AbortReason::EntryFailed => "entry-failed",
            AbortReason::FileSelectFailed => "file-select-failed",
            AbortReason::ResumeStartFailed => "resume-start-failed",
        }
    }
}

/// What the daemon does when a verification fails. v1 is always
/// abort-safe: heaters to safe state, motion stopped, **never**
/// continue past a failed verification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FailureAction {
    /// Abort the recovery with the given reason code.
    Abort {
        /// Why the recovery aborted.
        reason: AbortReason,
    },
}

/// Where the daemon reads the probe trigger Z from, per probe type
/// (design doc §8, step 6).
///
/// Klipper builds probe results as `bed_z = test_z − z_offset` where
/// `test_z` is the raw toolhead Z at trigger
/// (`klippy/extras/manual_probe.py`, `create_probe_result`). The
/// nozzle-as-stylus datum needs the **raw** trigger Z.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum TriggerSource {
    /// Read `probe.last_z_result` — already the raw toolhead Z at
    /// trigger. Used for Tap-style `[probe]`.
    RawLastZResult,
    /// Read `probe.last_probe_position[2]` (`bed_z`, which has
    /// `z_offset` subtracted) and add `z_offset` back to recover the
    /// raw trigger Z. Used for `[load_cell_probe]`.
    BedZPlusOffset {
        /// The configured probe `z_offset`, mm.
        z_offset: f64,
    },
}

/// The typed true-Z formula (design doc §8, step 6):
///
/// ```text
/// true_Z_at_halt = z_prev_top + (halt − trigger)
/// ```
///
/// evaluated in the shifted frame, where `trigger` is the **raw**
/// trigger Z (see [`TriggerSource`]) and `halt` is the **toolhead**
/// status `position[2]` — the raw kinematic position. Never
/// `gcode_move.position`, which reads back through the transform stack.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TrueZFormula {
    /// Top-of-layer-N−1 Z at the probe point (Klipper-internal frame at
    /// capture — the frame the frozen `z_thermal_adjust` value is baked
    /// into), mm.
    pub z_prev_top: f64,
    /// Where the trigger Z comes from.
    pub trigger_source: TriggerSource,
    /// The `z_thermal_adjust` offset frozen by §8 step 4 (`None` when
    /// the module is not configured). Carried so the daemon can
    /// cross-check the frozen `current_z_adjust` against the value the
    /// capture-frame arithmetic assumes.
    pub frozen_z_adjust: Option<f64>,
}

/// Evaluates the true-Z formula.
///
/// `trigger_reading` is the raw value read per
/// [`TrueZFormula::trigger_source`] (already-raw `last_z_result`, or
/// `bed_z` to which the formula adds `z_offset` back). `halt_z` is
/// `toolhead.position[2]` after `PROBE` returned (with `SAMPLES=1` the
/// toolhead rests exactly at the halt position).
///
/// # Errors
///
/// [`RecoveryError::NonFinite`] on any non-finite input or result —
/// the daemon must abort, never substitute, on such an error.
pub fn true_z_at_halt(
    formula: &TrueZFormula,
    trigger_reading: f64,
    halt_z: f64,
) -> Result<f64, RecoveryError> {
    if !formula.z_prev_top.is_finite() {
        return Err(RecoveryError::NonFinite {
            field: "z_prev_top",
        });
    }
    if !trigger_reading.is_finite() {
        return Err(RecoveryError::NonFinite {
            field: "trigger_reading",
        });
    }
    if !halt_z.is_finite() {
        return Err(RecoveryError::NonFinite { field: "halt_z" });
    }
    let trigger = match formula.trigger_source {
        TriggerSource::RawLastZResult => trigger_reading,
        TriggerSource::BedZPlusOffset { z_offset } => {
            if !z_offset.is_finite() {
                return Err(RecoveryError::NonFinite { field: "z_offset" });
            }
            trigger_reading + z_offset
        }
    };
    let true_z = formula.z_prev_top + (halt_z - trigger);
    if !true_z.is_finite() {
        return Err(RecoveryError::NonFinite { field: "true_z" });
    }
    Ok(true_z)
}

/// A runtime computation attached to a step (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum RuntimeComputation {
    /// Evaluate [`true_z_at_halt`] and substitute the result for
    /// [`TRUE_Z_PLACEHOLDER`] in the step's commands.
    TrueZ(TrueZFormula),
}

/// One strictly ordered recovery step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecoveryStep {
    /// 1-based position in the plan.
    pub id: u32,
    /// Which §8 phase this step implements.
    pub phase: Phase,
    /// One-line human summary.
    pub summary: String,
    /// G-code / extended commands to send, in order. May contain
    /// [`TRUE_Z_PLACEHOLDER`] only when [`Self::compute`] is set.
    pub commands: Vec<String>,
    /// Predicates that must hold before the commands are sent.
    pub pre_verify: Vec<Verification>,
    /// Predicates that must hold after the commands complete.
    pub verify: Vec<Verification>,
    /// Runtime computation feeding the command placeholder, if any.
    pub compute: Option<RuntimeComputation>,
    /// What to do when any predicate fails.
    pub on_failure: FailureAction,
}

/// A non-fatal planning observation surfaced to the operator.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PlanWarning {
    /// The WAL shows an active mesh (`mesh_matrix` non-empty) but no
    /// loadable profile name (adaptive meshes have empty names): the
    /// mesh cannot be restored and the plan continues without it.
    AdaptiveMeshNotRestorable,
    /// Skew correction was active but no profile name was recorded;
    /// skew is not restored.
    SkewProfileUnknown,
    /// No bed target was found in the WAL or the file; the bed is left
    /// unheated.
    NoBedTarget,
    /// The resume point is not on infill (the match did not allow an
    /// infill start).
    ResumeNotOnInfill,
    /// A fan could not be restored (unrecognized name shape).
    UnrestorableFan {
        /// The fan's WAL name.
        name: String,
    },
}

/// The complete, strictly ordered recovery plan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecoveryPlan {
    /// The steps, in execution order.
    pub steps: Vec<RecoveryStep>,
    /// The probe envelope and shifted-frame declaration behind §8
    /// steps 5–6.
    pub envelope: Envelope,
    /// Top-level filename passed to `M23`.
    pub resume_file: String,
    /// Line-boundary byte offset passed to `M26 S`.
    pub resume_offset: u64,
    /// Non-fatal observations.
    pub warnings: Vec<PlanWarning>,
}

/// First word of a command, uppercased.
fn first_word(command: &str) -> String {
    command
        .split_whitespace()
        .next()
        .unwrap_or("")
        .to_ascii_uppercase()
}

/// `true` when the command moves the toolhead (or could).
fn is_motion_command(command: &str) -> bool {
    matches!(
        first_word(command).as_str(),
        "G0" | "G1" | "G2" | "G3" | "G28" | "PROBE" | "FORCE_MOVE"
    )
}

impl RecoveryPlan {
    /// Index of the first step in `phase`.
    #[must_use]
    pub fn first_index(&self, phase: Phase) -> Option<usize> {
        self.steps.iter().position(|s| s.phase == phase)
    }

    /// §8.1: the very first step disarms the idle timeout.
    #[must_use]
    pub fn idle_timeout_first(&self) -> bool {
        self.steps.first().is_some_and(|s| {
            s.phase == Phase::IdleTimeout
                && s.commands
                    .first()
                    .is_some_and(|c| first_word(c) == "SET_IDLE_TIMEOUT")
        })
    }

    /// §8.1: every Z stepper is enabled before any motion command.
    #[must_use]
    pub fn steppers_enabled_before_motion(&self) -> bool {
        let Some(enable) = self.first_index(Phase::StepperEnable) else {
            return false;
        };
        let first_motion = self
            .steps
            .iter()
            .position(|s| s.commands.iter().any(|c| is_motion_command(c)));
        first_motion.is_none_or(|m| enable < m)
    }

    /// §8.2: the probe step itself pre-verifies the nozzle temperature
    /// band (mandatory — no probe type has a temperature interlock).
    #[must_use]
    pub fn temp_verify_precedes_probe(&self) -> bool {
        let Some(probe) = self.first_index(Phase::Probe) else {
            return false;
        };
        self.steps[probe]
            .pre_verify
            .iter()
            .any(|v| matches!(v.predicate, Predicate::TempWithin { .. }))
    }

    /// §8.4: `z_thermal_adjust` is frozen before the shifted-frame
    /// declaration (vacuously true when the module is absent).
    #[must_use]
    pub fn z_thermal_freeze_precedes_shifted_declare(&self) -> bool {
        match (
            self.first_index(Phase::TransformFreeze),
            self.first_index(Phase::ShiftedFrame),
        ) {
            (None, _) => true,
            (Some(freeze), Some(shifted)) => freeze < shifted,
            (Some(_), None) => false,
        }
    }

    /// §8.7: the probe happens before the mesh is loaded (the probe
    /// must be transform-free with respect to the mesh).
    #[must_use]
    pub fn probe_step_precedes_mesh_load(&self) -> bool {
        match (
            self.first_index(Phase::Probe),
            self.first_index(Phase::MeshLoad),
        ) {
            (_, None) => self.first_index(Phase::Probe).is_some(),
            (Some(probe), Some(mesh)) => probe < mesh,
            (None, Some(_)) => false,
        }
    }

    /// §8.8: the mesh (when restored) is loaded before the final
    /// true-frame declaration.
    #[must_use]
    pub fn mesh_load_precedes_final_declare(&self) -> bool {
        match (
            self.first_index(Phase::MeshLoad),
            self.first_index(Phase::FinalDeclare),
        ) {
            (None, Some(_)) => true,
            (Some(mesh), Some(declare)) => mesh < declare,
            (_, None) => false,
        }
    }

    /// No `G28` appears in any command at or after the shifted-frame
    /// declaration (re-homing after the frame is declared would crash
    /// the bed into the nozzle).
    #[must_use]
    pub fn no_g28_after_shifted_declare(&self) -> bool {
        let Some(shifted) = self.first_index(Phase::ShiftedFrame) else {
            return false;
        };
        self.steps[shifted..]
            .iter()
            .flat_map(|s| s.commands.iter())
            .all(|c| first_word(c) != "G28")
    }

    /// The `M26 S<byte>` offset actually present in the commands
    /// (`None` when absent or malformed). Tests cross-check it against
    /// [`Self::resume_offset`] and the line-boundary contract.
    #[must_use]
    pub fn m26_offset(&self) -> Option<u64> {
        self.steps
            .iter()
            .flat_map(|s| s.commands.iter())
            .find(|c| first_word(c) == "M26")
            .and_then(|c| {
                c.split_whitespace()
                    .find_map(|w| w.strip_prefix('S'))
                    .and_then(|v| v.parse::<u64>().ok())
            })
    }

    /// Every step in `phase`, in order.
    pub fn steps_in_phase(&self, phase: Phase) -> impl Iterator<Item = &RecoveryStep> {
        self.steps.iter().filter(move |s| s.phase == phase)
    }

    /// Renders the numbered, commented human-review form.
    #[must_use]
    pub fn render(&self) -> String {
        use std::fmt::Write as _;
        // `write!` into a String is infallible; results discarded.
        let mut out = String::new();
        out.push_str("# dead-reckoning recovery plan\n");
        let _ = writeln!(
            out,
            "# resume: {} @ byte {}",
            self.resume_file, self.resume_offset
        );
        let p = self.envelope.params;
        let _ = writeln!(
            out,
            "# envelope: gap {} + 0.15 x speed {} + margin {} = {} mm",
            fmt_num(p.expected_gap),
            fmt_num(p.probe_speed),
            fmt_num(p.margin),
            fmt_num(self.envelope.envelope),
        );
        let _ = writeln!(
            out,
            "# shifted frame: Z declared {} above position_min {}",
            fmt_num(self.envelope.envelope),
            fmt_num(self.envelope.position_min),
        );
        for warning in &self.warnings {
            let _ = writeln!(out, "# warning: {warning:?}");
        }
        for step in &self.steps {
            let _ = writeln!(
                out,
                "{:>2}. [{}] {}",
                step.id,
                step.phase.name(),
                step.summary
            );
            for v in &step.pre_verify {
                let _ = writeln!(
                    out,
                    "      pre:  {}.{} {}",
                    v.object,
                    v.field,
                    v.predicate.describe()
                );
            }
            for command in &step.commands {
                let _ = writeln!(out, "      send: {command}");
            }
            for v in &step.verify {
                let _ = writeln!(
                    out,
                    "      ok?:  {}.{} {}",
                    v.object,
                    v.field,
                    v.predicate.describe()
                );
            }
            let FailureAction::Abort { reason } = step.on_failure;
            let _ = writeln!(out, "      fail: abort ({})", reason.code());
        }
        out
    }
}

/// Formats a finite number for command/render output: up to five
/// decimal places, trailing zeros trimmed, `-0` normalized to `0`.
///
/// Total: a non-finite input renders as `"invalid"` instead of
/// panicking — but plan construction validates every number first, so
/// the string can never reach a generated plan (property-tested).
#[must_use]
pub fn fmt_num(v: f64) -> String {
    if !v.is_finite() {
        return "invalid".to_owned();
    }
    let mut s = format!("{v:.5}");
    while s.ends_with('0') {
        s.pop();
    }
    if s.ends_with('.') {
        s.pop();
    }
    if s == "-0" {
        return "0".to_owned();
    }
    s
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::{
        fmt_num, true_z_at_halt, AbortReason, Phase, Predicate, TriggerSource, TrueZFormula,
    };

    #[test]
    fn fmt_num_trims_and_normalizes() {
        assert_eq!(fmt_num(86_400.0), "86400");
        assert_eq!(fmt_num(1.5), "1.5");
        assert_eq!(fmt_num(0.15), "0.15");
        assert_eq!(fmt_num(-0.000_001), "0");
        assert_eq!(fmt_num(-2.0), "-2");
        assert_eq!(fmt_num(0.123_456), "0.12346");
        assert_eq!(fmt_num(f64::NAN), "invalid");
        assert_eq!(fmt_num(f64::INFINITY), "invalid");
    }

    #[test]
    fn true_z_raw_source_is_plain_offset_arithmetic() {
        let f = TrueZFormula {
            z_prev_top: 12.4,
            trigger_source: TriggerSource::RawLastZResult,
            frozen_z_adjust: None,
        };
        // Trigger at shifted 0.90, halt pressed 0.15 deeper at 0.75:
        // true Z = 12.4 - 0.15.
        let z = true_z_at_halt(&f, 0.90, 0.75).unwrap();
        assert!((z - 12.25).abs() < 1e-12);
    }

    #[test]
    fn true_z_bed_z_source_adds_z_offset_back() {
        let f = TrueZFormula {
            z_prev_top: 12.4,
            trigger_source: TriggerSource::BedZPlusOffset { z_offset: -0.1 },
            frozen_z_adjust: Some(0.02),
        };
        // bed_z reading 1.00 means raw trigger 0.90; halt 0.75.
        let z = true_z_at_halt(&f, 1.00, 0.75).unwrap();
        assert!((z - 12.25).abs() < 1e-12);
    }

    #[test]
    fn true_z_rejects_every_non_finite_input() {
        let good = TrueZFormula {
            z_prev_top: 12.4,
            trigger_source: TriggerSource::RawLastZResult,
            frozen_z_adjust: None,
        };
        assert!(true_z_at_halt(&good, f64::NAN, 0.5).is_err());
        assert!(true_z_at_halt(&good, 0.5, f64::INFINITY).is_err());
        let bad_prev = TrueZFormula {
            z_prev_top: f64::NAN,
            ..good
        };
        assert!(true_z_at_halt(&bad_prev, 0.5, 0.5).is_err());
        let bad_offset = TrueZFormula {
            trigger_source: TriggerSource::BedZPlusOffset { z_offset: f64::NAN },
            ..good
        };
        assert!(true_z_at_halt(&bad_offset, 0.5, 0.5).is_err());
        let overflow = TrueZFormula {
            z_prev_top: f64::MAX,
            trigger_source: TriggerSource::BedZPlusOffset { z_offset: f64::MAX },
            frozen_z_adjust: None,
        };
        assert!(true_z_at_halt(&overflow, -f64::MAX, f64::MAX).is_err());
    }

    #[test]
    fn phase_names_and_reason_codes_are_stable() {
        assert_eq!(Phase::IdleTimeout.name(), "idle-timeout");
        assert_eq!(Phase::ResumeStart.name(), "resume-start");
        assert_eq!(AbortReason::ProbeNoTrigger.code(), "probe-no-trigger");
    }

    #[test]
    fn predicate_descriptions_are_readable() {
        assert_eq!(
            Predicate::NumWithin {
                expected: 86_400.0,
                epsilon: 0.5
            }
            .describe(),
            "within 0.5 of 86400"
        );
        assert_eq!(
            Predicate::TempWithin {
                min: 140.0,
                max: 160.0
            }
            .describe(),
            "in [140, 160] C"
        );
        assert_eq!(
            Predicate::Contains {
                needle: "z".to_owned()
            }
            .describe(),
            "contains \"z\""
        );
        assert_eq!(Predicate::BoolTrue.describe(), "is true");
        assert_eq!(Predicate::BoolFalse.describe(), "is false");
        assert_eq!(Predicate::FinitePresent.describe(), "present and finite");
        assert_eq!(Predicate::NonEmptyMatrix.describe(), "non-empty matrix");
        assert_eq!(Predicate::NumAtLeast { min: 57.0 }.describe(), ">= 57");
        assert_eq!(
            Predicate::Equals {
                value: "Printing".to_owned()
            }
            .describe(),
            "equals \"Printing\""
        );
        assert!(Predicate::NumWithinComputed { epsilon: 0.05 }
            .describe()
            .contains("computed true Z"));
    }
}
