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
use crate::resume_file::RecoveryFileSpec;

/// The placeholder substituted by the daemon with the computed true-Z
/// value (see the module docs).
pub const TRUE_Z_PLACEHOLDER: &str = "{true_z}";

/// The placeholder substituted by the daemon with the machine's
/// pre-clamp `toolhead.max_accel`, recorded by the
/// [`Phase::AccelClamp`] step's [`RuntimeComputation::RecordMaxAccel`]
/// (see the accel-clamp discussion on [`RuntimeComputation`]). It
/// appears only in the [`Phase::AccelRestore`] step's command and in
/// the accel-clamp step's [`RecoveryStep::cleanup_commands`]; the daemon
/// substitutes the recorded value it captured before clamping.
pub const RESTORE_ACCEL_PLACEHOLDER: &str = "{restore_accel}";

/// The placeholder substituted by the daemon with the MACHINE's own
/// `toolhead.max_accel`, recorded once by the [`Phase::RecoveryAccel`]
/// step's [`RuntimeComputation::RecordMachineAccel`].
///
/// Two slots exist deliberately. [`RESTORE_ACCEL_PLACEHOLDER`] is the
/// *phase* slot: the value in force immediately before the touch clamp,
/// which under a `recovery_accel` override is the recovery accel, not
/// the machine's. This one is the *machine* slot: the value the printer
/// had before plrd touched anything. Collapsing them into one would make
/// the touch clamp's restore and the end-of-recovery restore fight over
/// a single variable, and whichever ran last would win — leaving the
/// machine at a recovery acceleration after a successful resume.
pub const MACHINE_ACCEL_PLACEHOLDER: &str = "{machine_accel}";

/// The placeholder substituted by the daemon with the ABSOLUTE park Z
/// computed by [`RuntimeComputation::ParkZ`] — `min(current_z + delta,
/// z_max)`.
///
/// The park lift cannot be a blind relative `G1 Z<delta>`: Klipper does
/// NOT clamp an out-of-range move, it raises "Move out of range"
/// (`klippy/kinematics/cartesian.py:105`, `check_move`), which would abort
/// the recovery AFTER the probe established the Z reference and force a
/// full re-run. Computing the clamped absolute target at execute time —
/// when the true Z is finally known — keeps the lift inside the rail.
pub const PARK_Z_PLACEHOLDER: &str = "{park_z}";

/// Which phase a step belongs to. The builder emits phases in exactly
/// this declaration order (the strict recovery-UX order that replaces
/// the old §8 ordering); the ordering invariants
/// ([`RecoveryPlan::idle_timeout_first`] and friends) verify it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Phase {
    /// 1a — disarm the idle timeout before anything else.
    IdleTimeout,
    /// 1b — energize the Z steppers (enabling never touches homed
    /// state; there is no M17 in Klipper).
    StepperEnable,
    /// 1c — record the machine's own `toolhead.max_accel` (into the
    /// MACHINE slot, see [`RuntimeComputation::RecordMachineAccel`]) and,
    /// when `recovery_accel` is configured, clamp acceleration for the
    /// whole recovery. Its [`RecoveryStep::cleanup_commands`] restore the
    /// machine value on ANY later abort; [`Phase::RecoveryAccelRestore`]
    /// does it on the success path.
    ///
    /// Emitted only when at least one acceleration override is
    /// configured, so a machine that sets none has a byte-identical plan
    /// to before the feature existed.
    RecoveryAccel,
    /// 2 — the FIRST heating action: non-blocking `M140` (bed is the
    /// long pole) plus a non-blocking `M104` toward the clamped probe
    /// temperature. Convergence is gated later, at the probe's
    /// `pre_verify`.
    ImmediateBedHeat,
    /// 3 — declare the conservative believed Z (upper bound of the
    /// possible-stop set) then lift by `pre_home_z_lift` so the XY
    /// homing moves cannot drag the nozzle across the part.
    BelievedZDeclare,
    /// 4 — home XY only (now AFTER the believed-Z lift). Never bare
    /// `G28`, never Z.
    HomeXy,
    /// 5 — heat to the probe/drag temperature and HOLD for it (`M109`,
    /// which Klipper blocks on natively) before any contact operation.
    ///
    /// Placed after [`Phase::HomeXy`] and before [`Phase::CleanNozzle`],
    /// and that order is physical rather than cosmetic: reaching
    /// temperature BEFORE the clean means any ooze the heat-up produces
    /// is wiped away by the clean, instead of being deposited on the
    /// nozzle during the probe and corrupting the Z reference.
    ///
    /// Absent entirely on a drag machine with `drag_nozzle_temp = 0` (the
    /// documented cold-drag opt-out: never wait for the nozzle to cool).
    ///
    /// # `M109` waits in BOTH directions on a PID hotend
    ///
    /// Klipper's `M109` waits for the heater to SETTLE, not merely to
    /// rise: with a PID-controlled extruder a nozzle currently HOTTER
    /// than the target will wait while it cools. With bang-bang control
    /// it only waits while heating.
    ///
    /// # What bounds the wait (and why it cannot wedge)
    ///
    /// Klipper's own wait is UNBOUNDED — it will sit there indefinitely
    /// if the target is unreachable (a chamber hotter than the requested
    /// temperature, a failed heater). The bound is plrd's: the executor
    /// sends this step's command with the `temp_timeout` call deadline
    /// (`ExecOptions::temp_timeout`, 15 minutes by default, applied to
    /// the Moonraker client in `recover::execute_with_gates`), and a
    /// timeout is a step failure that aborts the recovery.
    ///
    /// That abort is CLEAN — but not because no `SET_KINEMATIC_POSITION`
    /// has run: [`Phase::BelievedZDeclare`] issues one two phases
    /// earlier. It is clean because that earlier declare is the
    /// CONSERVATIVE believed Z (the upper bound of the possible-stop set)
    /// followed by a lift, which moves the toolhead AWAY from the part
    /// and leaves the frame no less trustworthy than it already was. The
    /// frame becomes unverifiable only at [`Phase::ShiftedFrame`], which
    /// this phase precedes — so the executor's `frame_invalid` stays
    /// false, no frame-invalidation marker is written, and a re-run after
    /// fixing the thermal problem is immediately possible. (The probe step's contact-ceiling gate is a
    /// different guard for a different failure — it never sees a nozzle
    /// stuck at this step, because `M109` blocks long before the probe
    /// step is reached.)
    ProbeTempHold,
    /// 6 — call the operator's clean-nozzle macro when it exists; when
    /// it does not, emit no command and set
    /// [`RecoveryPlan::requires_clean_nozzle_confirmation`].
    CleanNozzle,
    /// 6a — freeze `z_thermal_adjust` before the shifted frame.
    TransformFreeze,
    /// 6b — declare the shifted frame (`SET_KINEMATIC_POSITION`).
    ShiftedFrame,
    /// 6c — XY travel to the selected contact point.
    ProbeApproach,
    /// 6c′ — clamp `toolhead.max_accel` to the touch accel around the
    /// consensus-touch phase (Cartographer `touch_mode.py:262-274`
    /// clamps `max_accel` before `z_probing_move` and restores it in a
    /// `finally`). Emitted only for the consensus `PLR_TOUCH` path
    /// (see [`crate::build`]); records the pre-clamp accel so the
    /// restore step and the abort cleanup can put it back.
    AccelClamp,
    /// 6d — the probe (consensus `PLR_TOUCH`, or a single `PROBE`
    /// on the legacy/load-cell path, or the ADXL drag staircase).
    Probe,
    /// 6d′ — restore the pre-clamp `max_accel` on the success path
    /// (the abort path restores via the accel-clamp step's
    /// [`RecoveryStep::cleanup_commands`]). Present iff [`AccelClamp`]
    /// is.
    AccelRestore,
    /// 7a — true-Z arithmetic and kinematic re-declaration.
    TrueZDeclare,
    /// 7a′ — the OPERATOR Z-CONFIRMATION standoff (`[plr]`
    /// `confirm_z_before_resume`, default `false`).
    ///
    /// Lifts to the entry standoff — `min(current_Z + entry_hop, z_max)`,
    /// the same rail-clamped [`RuntimeComputation::ParkZ`] arithmetic the
    /// reheat park uses, with `entry_hop` (the documented safe standoff
    /// above the resume point) as the delta — so the operator can look at
    /// the nozzle while the daemon reports the Z it believes it is at and
    /// how that number was derived.
    ///
    /// It deliberately **cannot descend**: [`park_z_at`] clamps the
    /// target down to the rail but never below the current Z. Showing
    /// the operator "where Z is" by driving toward the bed would be a
    /// new, unreviewed descent — the exact class of motion this design
    /// exists to eliminate.
    ///
    /// Emitted only when `confirm_z_before_resume` is set; the daemon
    /// pauses for confirmation AFTER this step's verifications pass.
    ZConfirmStandoff,
    /// 7a″ — the interactive RESUME-POINT PREVIEW entry (`[plr]`
    /// `resume_candidate_policy = ask`, design §D). Present iff the plan
    /// carries a [`RecoveryPlan::preview`]; sits AFTER the frame is fully
    /// declared (post [`Self::TrueZDeclare`]/[`Self::ZConfirmStandoff`]) so
    /// an abort/timeout during the reposition loop invalidates the Z frame
    /// exactly as a [`Self::ZConfirmStandoff`] abort would (the nozzle has
    /// been driven around the part).
    ///
    /// This one step is the LIFT to the single hover plane
    /// ([`RuntimeComputation::HoverPlane`] — `min(z_max, max(current_z,
    /// hover_target_z))`, an absolute floor above every stop that CANNOT
    /// descend) plus the cool-down `M104 S{preview_nozzle_temp}` (§E.2).
    /// The daemon then runs the XY-only reposition loop against
    /// [`RecoveryPlan::preview`]; there is no per-stop Z step in the
    /// vocabulary, so never-descend is structural, not checked.
    ///
    /// Under `policy = ask` the preview SUBSUMES the standalone
    /// [`Self::ZConfirmStandoff`] (the operator's Accept IS the Z
    /// confirmation — ruling), so the two are never both emitted.
    ResumePreview,
    /// 7b — load the bed-mesh profile (probe already done).
    MeshLoad,
    /// 7c — final true-frame declaration.
    FinalDeclare,
    /// 8a — park the nozzle at the reheat park XY (configured or
    /// computed) at the current Z plus `reheat_park_delta_z`, so the
    /// print-temperature reheat (done inside the recovery file) never
    /// dwells against the part.
    ParkForReheat,
    /// 8b — replay offsets, factors, modes, skew, fans (print
    /// temperatures move into the recovery file).
    RestoreFrame,
    /// 8c — success-path restore of the machine's own `max_accel`
    /// (`{machine_accel}`), placed BEFORE the recovery file starts so the
    /// resumed print runs at the machine's normal acceleration rather
    /// than a recovery override. Present exactly when
    /// [`Phase::RecoveryAccel`] is; the abort path restores through that
    /// step's [`RecoveryStep::cleanup_commands`] instead.
    RecoveryAccelRestore,
    /// 9 — select the generated recovery file (`M23`) and start it
    /// (`M24`). No `M26`: the recovery file already begins at the resume
    /// boundary.
    RecoveryFileSelect,
}

impl Phase {
    /// Stable kebab-case name for rendering.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Phase::IdleTimeout => "idle-timeout",
            Phase::StepperEnable => "stepper-enable",
            Phase::RecoveryAccel => "recovery-accel",
            Phase::ZConfirmStandoff => "z-confirm-standoff",
            Phase::ResumePreview => "resume-preview",
            Phase::RecoveryAccelRestore => "recovery-accel-restore",
            Phase::ImmediateBedHeat => "immediate-bed-heat",
            Phase::BelievedZDeclare => "believed-z-declare",
            Phase::HomeXy => "home-xy",
            Phase::ProbeTempHold => "probe-temp-hold",
            Phase::CleanNozzle => "clean-nozzle",
            Phase::TransformFreeze => "transform-freeze",
            Phase::ShiftedFrame => "shifted-frame",
            Phase::ProbeApproach => "probe-approach",
            Phase::AccelClamp => "accel-clamp",
            Phase::Probe => "probe",
            Phase::AccelRestore => "accel-restore",
            Phase::TrueZDeclare => "true-z-declare",
            Phase::MeshLoad => "mesh-load",
            Phase::FinalDeclare => "final-declare",
            Phase::ParkForReheat => "park-for-reheat",
            Phase::RestoreFrame => "restore-frame",
            Phase::RecoveryFileSelect => "recovery-file-select",
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
    /// Numeric field at most `max`. Used both for the consensus-touch
    /// sample-range gate (`plr.last_touch_result.range` ≤
    /// `touch_sample_range`) and the extruder-**target** temperature
    /// interlock (`extruder.target` ≤ the probe band max — the
    /// `max(current, target)` guard from Cartographer
    /// `touch_mode.py:299-303`).
    NumAtMost {
        /// Inclusive upper bound.
        max: f64,
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
            Predicate::NumAtMost { max } => format!("<= {}", fmt_num(*max)),
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
                format!("within {} of the step's computed value", fmt_num(*epsilon))
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
    /// The recovery-wide `SET_VELOCITY_LIMIT` override did not take
    /// effect (or its `max_accel` record failed).
    RecoveryAccelFailed,
    /// Restoring the machine's own `max_accel` on the success path
    /// failed.
    RecoveryAccelRestoreFailed,
    /// The operator Z-confirmation standoff lift failed.
    ZConfirmStandoffFailed,
    /// The resume-preview hover-plane lift or cool-down failed (the entry
    /// to the XY reposition loop, design §D.2).
    ResumePreviewFailed,
    /// The immediate (non-blocking) bed/nozzle heat commands did not
    /// take effect (their targets were not set).
    ImmediateBedHeatFailed,
    /// The believed-Z declaration or its pre-home lift failed.
    BelievedZDeclareFailed,
    /// XY homing failed.
    HomingFailed,
    /// The nozzle did not reach and hold the probe/drag temperature.
    ProbeTempHoldFailed,
    /// The clean-nozzle macro call failed.
    CleanNozzleFailed,
    /// `z_thermal_adjust` could not be frozen.
    TransformFreezeFailed,
    /// The shifted frame was not declared.
    ShiftedFrameNotDeclared,
    /// The XY approach did not reach the contact point.
    ApproachFailed,
    /// The `SET_VELOCITY_LIMIT` accel clamp did not take effect.
    AccelClampFailed,
    /// Restoring the pre-clamp `max_accel` on the success path failed.
    AccelRestoreFailed,
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
    /// Parking the nozzle for the print-temperature reheat failed.
    ParkForReheatFailed,
    /// Frame restore (offsets/factors/skew/fans) failed.
    RestoreFailed,
    /// Selecting or starting the recovery file failed.
    RecoveryFileSelectFailed,
}

impl AbortReason {
    /// Stable reason-code string.
    #[must_use]
    pub fn code(self) -> &'static str {
        match self {
            AbortReason::IdleTimeoutNotApplied => "idle-timeout-not-applied",
            AbortReason::StepperEnableFailed => "stepper-enable-failed",
            AbortReason::RecoveryAccelFailed => "recovery-accel-failed",
            AbortReason::RecoveryAccelRestoreFailed => "recovery-accel-restore-failed",
            AbortReason::ZConfirmStandoffFailed => "z-confirm-standoff-failed",
            AbortReason::ResumePreviewFailed => "resume-preview-failed",
            AbortReason::ImmediateBedHeatFailed => "immediate-bed-heat-failed",
            AbortReason::BelievedZDeclareFailed => "believed-z-declare-failed",
            AbortReason::HomingFailed => "homing-failed",
            AbortReason::ProbeTempHoldFailed => "probe-temp-hold-failed",
            AbortReason::CleanNozzleFailed => "clean-nozzle-failed",
            AbortReason::TransformFreezeFailed => "transform-freeze-failed",
            AbortReason::ShiftedFrameNotDeclared => "shifted-frame-not-declared",
            AbortReason::ApproachFailed => "approach-failed",
            AbortReason::AccelClampFailed => "accel-clamp-failed",
            AbortReason::AccelRestoreFailed => "accel-restore-failed",
            AbortReason::ProbeNoTrigger => "probe-no-trigger",
            AbortReason::TrueZDeclareFailed => "true-z-declare-failed",
            AbortReason::MeshLoadFailed => "mesh-load-failed",
            AbortReason::FinalDeclareFailed => "final-declare-failed",
            AbortReason::ParkForReheatFailed => "park-for-reheat-failed",
            AbortReason::RestoreFailed => "restore-failed",
            AbortReason::RecoveryFileSelectFailed => "recovery-file-select-failed",
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
    /// Read `plr.last_drag_result.trigger_z` — the raw toolhead Z of
    /// the first contacting drag pass, reported by the plugin's
    /// `PLR_DRAG_PROBE` on the `plr` status object (alongside `passes`
    /// and `confidence`). Already in raw toolhead coordinates (the
    /// nozzle is the stylus; there is no probe `z_offset`), so it is
    /// consumed like `last_z_result`. Used for
    /// [`crate::machine::ProbeKind::AdxlDrag`].
    DragResult,
    /// Read `plr.last_touch_result.median_z` — the consensus TRIGGER Z
    /// of the multi-touch `PLR_TOUCH` sequence, reported by the plugin's
    /// `PLR_TOUCH` on the `plr` status object (alongside `range`,
    /// `samples_used`, `touches`). The plugin obtains each touch through
    /// a klippy probe session (`pull_probed_results()[-1].bed_z`), so
    /// `median_z` is in the **`z_offset`-subtracted** bed-probing frame
    /// (`bed_z = trigger_z − z_offset`) — exactly like
    /// [`TriggerSource::BedZPlusOffset`]. The formula therefore adds the
    /// configured `z_offset` back to recover the raw trigger Z, for both
    /// Tap and load-cell machines (the plugin uses the same probe-session
    /// convention regardless of which contact mechanism underlies it).
    /// The consensus median replaces the old single-`PROBE` halt=trigger
    /// reading; the plugin's retract bookkeeping leaves the toolhead
    /// resting retracted above it when the command returns. Used for the
    /// consensus `PLR_TOUCH` path on Tap / load-cell machines.
    TouchResult {
        /// The configured probe `z_offset`, mm (added back to
        /// `median_z` to recover the raw trigger Z).
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
        TriggerSource::RawLastZResult | TriggerSource::DragResult => trigger_reading,
        // Both the load-cell `bed_z` readback and the `PLR_TOUCH`
        // consensus median arrive in the z_offset-subtracted bed-probing
        // frame; add the offset back to recover the raw trigger Z.
        TriggerSource::BedZPlusOffset { z_offset } | TriggerSource::TouchResult { z_offset } => {
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
    /// Read `toolhead.max_accel` **before** the step's commands run and
    /// record it as the value the daemon substitutes for
    /// [`RESTORE_ACCEL_PLACEHOLDER`] — in the [`Phase::AccelRestore`]
    /// step and in this step's [`RecoveryStep::cleanup_commands`]. The
    /// accel-clamp step carries this so the pre-clamp accel is captured
    /// exactly once, before `SET_VELOCITY_LIMIT` overwrites it
    /// (Cartographer `touch_mode.py:262-274` reads `get_max_accel()`
    /// then `set_max_accel(TOUCH_ACCEL)`). The recorded value persists
    /// across the intervening steps in the daemon's execution state.
    RecordMaxAccel,
    /// Read `toolhead.max_accel` **before** the step's commands run and
    /// record it in the MACHINE slot, substituted for
    /// [`MACHINE_ACCEL_PLACEHOLDER`] (see that constant for why the two
    /// slots are separate). Carried by the [`Phase::RecoveryAccel`] step,
    /// which is the first step in the plan that can change acceleration —
    /// so the value recorded is genuinely the machine's own.
    RecordMachineAccel,
    /// Read `toolhead.position[2]` **before** the step's commands run and
    /// substitute `min(current_z + delta_z, z_max)` for
    /// [`PARK_Z_PLACEHOLDER`] — the rail-clamped absolute park height (see
    /// that constant for why a blind relative lift is unsafe). `z_max`
    /// `None` (limit unknown, the legacy path) leaves the lift unclamped.
    ParkZ {
        /// Lift above the current Z, mm.
        delta_z: f64,
        /// The Z rail's `position_max`, mm, when known.
        z_max: Option<f64>,
    },
    /// Read `toolhead.position[2]` **before** the step's commands run and
    /// substitute `min(max(current_z, target_z), z_max)` for
    /// [`PARK_Z_PLACEHOLDER`] — the resume-preview hover plane
    /// ([`hover_plane_at`], design §E.1).
    ///
    /// Unlike [`Self::ParkZ`], the floor is an ABSOLUTE `target_z`
    /// (`max` over the preview stops of their g-code Z plus
    /// `preview_standoff`), not a relative delta: the single hover plane
    /// must sit above EVERY stop's geometry regardless of where the probe
    /// left the nozzle. The `max(current_z, …)` term makes never-descend a
    /// structural fact — a nozzle already higher than the plane stays put
    /// rather than being driven down to it — and the `min(…, z_max)` term
    /// keeps the lift inside the Z rail exactly as `ParkZ` does.
    HoverPlane {
        /// Absolute g-code-frame floor for the plane, mm: the highest stop
        /// Z plus the standoff. Baked at plan time.
        target_z: f64,
        /// The Z rail's `position_max`, mm, when known.
        z_max: Option<f64>,
    },
}

/// Evaluates [`RuntimeComputation::ParkZ`]: the rail-clamped absolute park
/// height `min(current_z + delta_z, z_max)`.
///
/// # Errors
///
/// [`RecoveryError::NonFinite`] on any non-finite input or result — the
/// daemon must abort, never substitute, on such an error.
pub fn park_z_at(current_z: f64, delta_z: f64, z_max: Option<f64>) -> Result<f64, RecoveryError> {
    if !current_z.is_finite() {
        return Err(RecoveryError::NonFinite { field: "current_z" });
    }
    if !delta_z.is_finite() {
        return Err(RecoveryError::NonFinite { field: "delta_z" });
    }
    let mut target = current_z + delta_z;
    if let Some(zm) = z_max {
        if !zm.is_finite() {
            return Err(RecoveryError::NonFinite { field: "z_max" });
        }
        // Clamp DOWN to the rail only: never push the nozzle lower than
        // where it already is (a z_max below the current Z would
        // otherwise command a descent into the part).
        target = target.min(zm).max(current_z);
    }
    if !target.is_finite() {
        return Err(RecoveryError::NonFinite { field: "park_z" });
    }
    Ok(target)
}

/// Evaluates [`RuntimeComputation::HoverPlane`]: the resume-preview hover
/// plane `min(max(current_z, target_z), z_max)`.
///
/// The plane sits at least at `target_z` (above every stop's geometry) and
/// at least at `current_z` (never a descent), then is clamped down to the
/// Z rail. Because the result is `>= current_z` always, this is a lift or a
/// no-op, never a move toward the part — the never-descend guarantee §E.1
/// makes structural.
///
/// # Errors
///
/// [`RecoveryError::NonFinite`] on any non-finite input or result — the
/// daemon must abort, never substitute, on such an error.
pub fn hover_plane_at(
    current_z: f64,
    target_z: f64,
    z_max: Option<f64>,
) -> Result<f64, RecoveryError> {
    if !current_z.is_finite() {
        return Err(RecoveryError::NonFinite { field: "current_z" });
    }
    if !target_z.is_finite() {
        return Err(RecoveryError::NonFinite { field: "target_z" });
    }
    // At least the absolute floor, and at least where the nozzle already is.
    let mut plane = target_z.max(current_z);
    if let Some(zm) = z_max {
        if !zm.is_finite() {
            return Err(RecoveryError::NonFinite { field: "z_max" });
        }
        // Clamp DOWN to the rail, but never below the current Z — a z_max
        // beneath the nozzle must not command a descent into the part.
        plane = plane.min(zm).max(current_z);
    }
    if !plane.is_finite() {
        return Err(RecoveryError::NonFinite {
            field: "hover_plane",
        });
    }
    Ok(plane)
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
    /// Commands the daemon runs on the ABORT path only, after the
    /// failure and after the abort reason is fixed — the plan-level
    /// `finally` (Cartographer wraps the accel clamp in a `try/finally`,
    /// `touch_mode.py:270-274`). They may carry
    /// [`RESTORE_ACCEL_PLACEHOLDER`]. The daemon registers a step's
    /// cleanup once the step's commands have been sent (its side effect
    /// is in force), runs the registered cleanups in reverse order on
    /// any subsequent abort, and — crucially — a cleanup failure is
    /// logged but never masks or replaces the original abort reason.
    /// On the success path the explicit [`Phase::AccelRestore`] step
    /// does the restore instead, so cleanups never double-run.
    #[serde(default)]
    pub cleanup_commands: Vec<String>,
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
    /// No `reheat_park_x`/`reheat_park_y` was configured, so the park
    /// point was computed and VERIFIED to clear the part's bounding box.
    /// Configure an explicit park position to control where the nozzle
    /// reheats.
    ReheatParkComputed {
        /// The computed park point `[x, y]`, mm.
        point: [f64; 2],
    },
    /// No `reheat_park_x`/`reheat_park_y` was configured AND the model
    /// carries no part geometry to check against, so the park point is a
    /// bare fallback that could NOT be verified clear of the part.
    ReheatParkUnverified {
        /// The fallback park point `[x, y]`, mm.
        point: [f64; 2],
    },
    /// The resolved BUILT-IN PURGE point lies inside the part's XY
    /// bounding box, so the purge deposits filament onto printed
    /// geometry. A warning, not a refusal: an operator may be purging
    /// onto a sacrificial area (a prime tower, a skirt region) on purpose.
    PurgeInsidePart {
        /// The purge point `[x, y]`, mm.
        point: [f64; 2],
        /// `true` when the operator set `purge_x`/`purge_y` explicitly;
        /// `false` when it defaulted to the reheat park point.
        configured: bool,
        /// The explicit `purge_z` in force, when one is set. With a low
        /// `purge_z` inside the footprint this is not merely "deposits
        /// filament on the part" — the file DESCENDS to that Z first, so
        /// it is a COLLISION with printed geometry.
        purge_z: Option<f64>,
    },
    /// `purge_z` sits below the resume Z — i.e. below the top of what has
    /// already been printed — so the descent to it may drive the nozzle
    /// into the part. Not a refusal: the purge point may be clear of the
    /// footprint (a bare patch of bed), where a low Z is exactly right.
    PurgeZBelowResume {
        /// The configured purge Z, mm.
        purge_z: f64,
        /// The resume Z (the part's current top at the resume point), mm.
        resume_z: f64,
    },
    /// The reheat park point lies INSIDE the part's XY bounding box: the
    /// nozzle will reheat to print temperature (and purge) over printed
    /// geometry. Either the operator configured it there, or no side of
    /// the footprint stayed clear once clamped into the machine's travel
    /// limits.
    ReheatParkInsidePart {
        /// The park point `[x, y]`, mm.
        point: [f64; 2],
        /// `true` when the operator configured this point explicitly;
        /// `false` when it was computed and no clear side existed.
        configured: bool,
    },
    /// The resume point is not on infill (the match did not allow an
    /// infill start).
    ResumeNotOnInfill,
    /// A fan could not be restored (unrecognized name shape).
    UnrestorableFan {
        /// The fan's WAL name.
        name: String,
    },
    /// The ADXL noise floor was calibrated at a drag speed that
    /// differs from the plan's `drag_speed` by more than 20% (the
    /// noise floor is speed-specific: faster passes excite more
    /// baseline vibration). A warning, not a refusal — the sensitivity
    /// knob usually absorbs the difference — but the calibration
    /// should be repeated at the current speed.
    NoiseFloorSpeedMismatch {
        /// Drag speed the noise floor was measured at, mm/s
        /// (the `[plr]` `noise_floor_speed` autosave).
        calibrated_at: f64,
        /// The plan's `drag_speed`, mm/s.
        drag_speed: f64,
    },
    /// A NONZERO `drag_nozzle_temp` below
    /// [`crate::build::DRAG_TEMP_FLOOR`], on a machine that actually
    /// drags. The blocking `M109` may then wait for a cooldown that never
    /// converges — but only on an enclosed or heated-chamber machine, and
    /// the wait is bounded by the executor's step timeout and aborts
    /// cleanly BEFORE the Z frame is declared. So this asks rather than
    /// refuses ([`crate::diagnosis::Tier::Confirmable`]).
    ///
    /// Raised only for [`crate::machine::ProbeKind::AdxlDrag`]: on a
    /// Tap / load-cell machine the key is never read, and stopping a
    /// recovery to ask about an inert setting is exactly the pointless
    /// obstruction this framework exists to remove.
    DragTempBelowFloor {
        /// The configured drag hold temperature, °C.
        drag_nozzle_temp: f64,
        /// The floor it sits below, °C.
        floor: f64,
    },
    /// `accel_entry` was configured but could not be written into the
    /// generated recovery file, because the machine's own `max_accel` —
    /// the value the file would have to restore afterwards as a literal —
    /// is unknown (the legacy `[machine]` path cannot see it).
    AccelEntryNotAppliedToFile {
        /// The configured value that did not reach the file, mm/s².
        accel_entry: f64,
    },
    /// An `UNSAFE_`-prefixed `[plr]` key is set and permitted a
    /// [`crate::diagnosis::Tier::Hard`] refusal that would otherwise
    /// have stopped this plan. Always surfaced: the escape hatch is
    /// legitimate, but it must never be silent.
    UnsafeOverrideActive {
        /// The `UNSAFE_` key in force (as the operator spells it).
        key: String,
        /// The diagnosis code it permitted (e.g. `purge_z_below_bed`).
        permitted: String,
    },
    /// `accel_probe` was configured on a machine that uses the consensus
    /// `PLR_TOUCH` path, where `touch_accel` owns the contact
    /// acceleration. The key is ignored rather than fought over.
    AccelProbeIgnoredOnTouchPath {
        /// The ignored value, mm/s².
        accel_probe: f64,
    },
}

impl PlanWarning {
    /// Operator-facing one-line description (rendered into the plan).
    #[must_use]
    #[allow(clippy::too_many_lines)] // one arm per variant, by design
    pub fn describe(&self) -> String {
        match self {
            PlanWarning::AdaptiveMeshNotRestorable => {
                "active bed mesh has no saved profile (adaptive?); it will not be restored"
                    .to_owned()
            }
            PlanWarning::SkewProfileUnknown => {
                "skew correction was active but no profile name is known; skew is not restored"
                    .to_owned()
            }
            PlanWarning::NoBedTarget => {
                "no bed target found in the WAL or the file; the bed is left unheated".to_owned()
            }
            PlanWarning::ReheatParkComputed { point } => format!(
                "no reheat_park_x/y configured; parking at computed ({}, {}), verified clear of \
                 the part bounding box — set an explicit park position",
                fmt_num(point[0]),
                fmt_num(point[1])
            ),
            PlanWarning::ReheatParkUnverified { point } => format!(
                "no reheat_park_x/y configured and no part geometry available; parking at \
                 ({}, {}) — NOT verified against the part — set an explicit park position",
                fmt_num(point[0]),
                fmt_num(point[1])
            ),
            PlanWarning::PurgeInsidePart {
                point,
                configured,
                purge_z,
            } => {
                let source = if *configured {
                    " (configured via purge_x/purge_y)"
                } else {
                    " (defaulted to the reheat park point)"
                };
                match purge_z {
                    // With an explicit purge_z the file DESCENDS to it
                    // before extruding: over the part that is a crash,
                    // not a cosmetic blob.
                    Some(z) => format!(
                        "COLLISION RISK: the built-in purge point ({}, {}) is INSIDE the part \
                         bounding box{source} AND purge_z is set to {}: the file descends to \
                         that Z over printed geometry before extruding. Move the purge clear \
                         of the part, or raise purge_z above the part's top",
                        fmt_num(point[0]),
                        fmt_num(point[1]),
                        fmt_num(*z)
                    ),
                    None => format!(
                        "the built-in purge point ({}, {}) is INSIDE the part bounding \
                         box{source}: purging at the parked height will drop filament onto \
                         printed geometry; move it clear unless this is a deliberate \
                         sacrificial area",
                        fmt_num(point[0]),
                        fmt_num(point[1])
                    ),
                }
            }
            PlanWarning::PurgeZBelowResume { purge_z, resume_z } => format!(
                "purge_z {} is below the resume Z {} (the top of what is already printed); the \
                 descent is safe only if the purge point is clear of the part",
                fmt_num(*purge_z),
                fmt_num(*resume_z)
            ),
            PlanWarning::ReheatParkInsidePart { point, configured } => format!(
                "the reheat park point ({}, {}) is INSIDE the part bounding box{} — the nozzle \
                 will reheat and purge over printed geometry; move it clear of the part",
                fmt_num(point[0]),
                fmt_num(point[1]),
                if *configured {
                    " (configured via reheat_park_x/y)"
                } else {
                    " (computed: no side of the part stayed clear within the axis limits)"
                }
            ),
            PlanWarning::ResumeNotOnInfill => {
                "the resume point is not on infill; the seam may be visible".to_owned()
            }
            PlanWarning::UnrestorableFan { name } => {
                format!("fan {name:?} has an unrecognized shape and is not restored")
            }
            PlanWarning::NoiseFloorSpeedMismatch {
                calibrated_at,
                drag_speed,
            } => format!(
                "noise floor was calibrated at {} mm/s but drag_speed is {} mm/s \
                 (>20% apart); re-run PLR_NOISE_TEST at the current speed",
                fmt_num(*calibrated_at),
                fmt_num(*drag_speed)
            ),
            PlanWarning::DragTempBelowFloor {
                drag_nozzle_temp,
                floor,
            } => format!(
                "drag_nozzle_temp {} C is below the {} C floor; the drag M109 may wait for a \
                 cooldown that never converges on an enclosed machine (bounded by the step \
                 timeout, and a clean abort before the Z frame is declared)",
                fmt_num(*drag_nozzle_temp),
                fmt_num(*floor)
            ),
            PlanWarning::AccelEntryNotAppliedToFile { accel_entry } => format!(
                "accel_entry ({} mm/s^2) is not written into the recovery file: the \
                 machine's own max_accel is unknown, so there would be nothing to restore to",
                fmt_num(*accel_entry)
            ),
            PlanWarning::UnsafeOverrideActive { key, permitted } => format!(
                "UNSAFE override {key} is set and permitted {permitted}; the refusal that \
                 normally prevents this is switched off"
            ),
            PlanWarning::AccelProbeIgnoredOnTouchPath { accel_probe } => format!(
                "accel_probe ({} mm/s^2) is ignored on the consensus-touch path; \
                 touch_accel owns the contact acceleration",
                fmt_num(*accel_probe)
            ),
        }
    }
}

/// The recovery-file late-binding for ONE preview stop (design §4). Baked
/// at plan time, PARALLEL to [`PreviewSpec::stops`] (binding `i` binds stop
/// `i`), because the entry moves depend on the stop's resume position in
/// the g-code frame — which the plan builder knows and the analyzer's
/// [`plr_analyzer::PreviewStop`] does not carry (it has no E coordinate and
/// no origin/extrude-factor frame).
///
/// On preview Accept the daemon's recovery-file writer swaps this stop's
/// [`Self::tail_offset`] and [`Self::entry_commands`] into the plan's
/// [`crate::resume_file::RecoveryFileSpec`] template and generates the file
/// — so the same [`crate::build::build_entry_commands`] derivation the
/// non-preview path uses builds every stop's entry moves (one formatter, no
/// drift). A separate accept-time formatter is deliberately avoided.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PreviewBinding {
    /// The verbatim-tail byte offset in the ORIGINAL file if this stop is
    /// accepted — `PreviewStop::resume_offset` for the same stop, so the
    /// generated file's tail matches the previewed resume byte-for-byte.
    pub tail_offset: u64,
    /// The entry-move commands (travel above the part, descend, prime,
    /// restore modes/feedrate) targeting THIS stop's resume XY/Z/E, built
    /// by the plan builder's [`crate::build::build_entry_commands`].
    pub entry_commands: Vec<String>,
}

/// The interactive resume-point preview the daemon drives at
/// [`Phase::ResumePreview`] (design §D). Present on a plan iff
/// `resume_candidate_policy = ask` produced a usable preview set.
///
/// The stop list and navigation anchors mirror the analyzer's
/// [`plr_analyzer::PreviewSet`] verbatim; the hover geometry, the cool-down
/// target, and the per-stop recovery [`Self::bindings`] are added here
/// because they need the plan builder's g-code frame and [`PlanConfig`],
/// which the analyzer has neither of.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PreviewSpec {
    /// The full nudge domain — every navigable stop, execution order.
    /// Identical to [`plr_analyzer::PreviewSet::stops`].
    pub stops: Vec<plr_analyzer::PreviewStop>,
    /// Representative stop indices (into [`Self::stops`]) the operator
    /// steps through with next/prev. Identical to
    /// [`plr_analyzer::PreviewSet::representatives`].
    pub representatives: Vec<u32>,
    /// Policy `first` anchor — the minimum-offset candidate stop's index.
    pub first_index: u32,
    /// Policy `mid` anchor — the median-offset candidate stop's index.
    pub mid_index: u32,
    /// Policy `last` anchor — the skip-forward stop's index (the resume it
    /// commits equals today's `select_resume_target`).
    pub last_index: u32,
    /// The cursor the reposition loop OPENS on: [`Self::last_index`], the
    /// safe skip-forward default. An operator who just accepts gets the
    /// same resume a headless `last` would.
    pub default_index: u32,
    /// Per-stop recovery-file late-binding, PARALLEL to [`Self::stops`].
    pub bindings: Vec<PreviewBinding>,
    /// The hover-plane absolute floor, g-code frame, mm: the highest stop's
    /// g-code Z plus `preview_standoff`. The daemon lifts to
    /// `min(z_max, max(current_z, hover_target_z))` once at entry
    /// ([`RuntimeComputation::HoverPlane`]) and never lowers.
    pub hover_target_z: f64,
    /// The Z rail `position_max`, mm, when known — the hover-plane ceiling.
    pub z_max: Option<f64>,
    /// The `M104` target commanded on preview entry so a nozzle hovering
    /// for minutes cannot ooze (§E.2) — resolved
    /// [`PlanConfig::preview_nozzle_temp_c`].
    pub cool_nozzle_temp: f64,
}

impl PreviewSpec {
    /// The default-cursor stop (the safe skip-forward), the point dry-run
    /// renders the file preview against and the loop opens on.
    #[must_use]
    pub fn default_stop(&self) -> Option<&plr_analyzer::PreviewStop> {
        self.stops.get(self.default_index as usize)
    }

    /// The recovery-file binding for stop `index`, when in range.
    #[must_use]
    pub fn binding(&self, index: u32) -> Option<&PreviewBinding> {
        self.bindings.get(index as usize)
    }
}

/// The complete, strictly ordered recovery plan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecoveryPlan {
    /// The steps, in execution order.
    pub steps: Vec<RecoveryStep>,
    /// The probe envelope and shifted-frame declaration behind the probe
    /// phase.
    pub envelope: Envelope,
    /// Top-level filename passed to `M23` — the GENERATED recovery file
    /// (`<original_stem>_RECOVERY.gcode`), not the original print.
    pub resume_file: String,
    /// Line-boundary byte offset in the ORIGINAL file where the
    /// generated recovery file's verbatim tail begins (consumed by the
    /// recovery-file generator; no longer emitted as `M26 S`).
    pub resume_offset: u64,
    /// `true` when no clean-nozzle macro exists on the machine, so the
    /// clean-nozzle step carries no command: the wizard / plugin must
    /// obtain the operator's confirmation that the nozzle is clean, and
    /// the CLI `--execute` prompt must say so.
    #[serde(default)]
    pub requires_clean_nozzle_confirmation: bool,
    /// The specification the daemon feeds to
    /// [`crate::resume_file::build_recovery_file`] to emit the generated
    /// recovery file (the file the final `M23` step selects). Carried in
    /// the plan so the entry-move / temperature / park derivation lives
    /// in one place.
    #[serde(default)]
    pub recovery_file: RecoveryFileSpec,
    /// `[plr]` `debug_confirm_each_step` (default `false`): pause before
    /// EVERY step and report that step's commands and verifications,
    /// waiting for the operator to answer over the control socket.
    ///
    /// Carried on the plan rather than in the daemon's runtime options
    /// because it is a `printer.cfg` setting like every other `[plr]`
    /// key, and because the rendered plan should say so.
    /// `skip_serializing_if` keeps a plan that does not use it
    /// byte-identical to one produced before the field existed.
    #[serde(default, skip_serializing_if = "core::ops::Not::not")]
    pub debug_confirm_each_step: bool,
    /// `[plr]` `confirm_timeout_s`: how long a confirm-point waits for an
    /// answer before aborting cleanly. `None` leaves the daemon's own
    /// default in force ([`crate::build::CONFIRM_TIMEOUT_DEFAULT_S`]).
    ///
    /// Carried on the plan because it is a `printer.cfg` `[plr]` key like
    /// every other one, and validated with the rest of them — so an
    /// absurd value is refused with a diagnosis at planning time rather
    /// than discovered at the pause.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub confirm_timeout_s: Option<f64>,
    /// `[plr]` `gcode_barrier_timeout_s`: how long execution waits for
    /// Klipper's g-code mutex, at each point where it is about to send
    /// after a gap it does not control, before refusing. `None` leaves the
    /// daemon's own default
    /// ([`crate::build::GCODE_BARRIER_TIMEOUT_DEFAULT_S`]).
    ///
    /// Carried on the plan for the same reason as
    /// [`RecoveryPlan::confirm_timeout_s`]: it is a `printer.cfg` `[plr]`
    /// key, validated with the rest of them, so an absurd value is refused
    /// with a diagnosis at planning time rather than discovered when the
    /// recovery is already half-executed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gcode_barrier_timeout_s: Option<f64>,
    /// The interactive resume-point preview (design §D), present iff
    /// `resume_candidate_policy = ask` produced a usable preview set. When
    /// present the plan carries a [`Phase::ResumePreview`] step (the hover
    /// lift + cool-down) and the daemon drives the XY-only reposition loop
    /// against this before selecting the recovery file; when absent the
    /// plan is the ordinary automatic one (first/mid/last, or ask with no
    /// preview available — see the pipeline routing).
    ///
    /// `skip_serializing_if` keeps a non-preview plan byte-identical to one
    /// produced before the field existed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preview: Option<PreviewSpec>,
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
        "G0" | "G1" | "G2" | "G3" | "G28" | "PROBE" | "FORCE_MOVE" | "PLR_DRAG_PROBE" | "PLR_TOUCH"
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

    /// The probe step itself pre-verifies the nozzle temperature ceiling
    /// (mandatory — no probe type has a temperature interlock): the
    /// current extruder temperature is bounded above (a warm band for
    /// touch/`PROBE`, a bare ceiling for the drag path, which has no warm
    /// minimum) AND the extruder TARGET is bounded above by the same
    /// ceiling.
    #[must_use]
    pub fn temp_verify_precedes_probe(&self) -> bool {
        let Some(probe) = self.first_index(Phase::Probe) else {
            return false;
        };
        let pv = &self.steps[probe].pre_verify;
        let current_bounded = pv.iter().any(|v| {
            v.object == "extruder"
                && v.field == "temperature"
                && matches!(
                    v.predicate,
                    Predicate::TempWithin { .. } | Predicate::NumAtMost { .. }
                )
        });
        let target_bounded = pv.iter().any(|v| {
            v.object == "extruder"
                && v.field == "target"
                && matches!(v.predicate, Predicate::NumAtMost { .. })
        });
        current_bounded && target_bounded
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

    /// The accel clamp (when present) precedes the probe: the touch
    /// accel must be in force before the consensus touches run. Vacuous
    /// when no accel-clamp step exists (drag / legacy single-`PROBE`).
    #[must_use]
    pub fn accel_clamp_precedes_probe(&self) -> bool {
        match (
            self.first_index(Phase::AccelClamp),
            self.first_index(Phase::Probe),
        ) {
            (None, _) => true,
            (Some(clamp), Some(probe)) => clamp < probe,
            (Some(_), None) => false,
        }
    }

    /// The success-path accel restore (when present) follows the probe,
    /// and an accel restore exists exactly when an accel clamp does (the
    /// clamp is never left un-restored on the success path). Vacuous
    /// when neither step exists.
    #[must_use]
    pub fn accel_restore_follows_probe(&self) -> bool {
        let clamp = self.first_index(Phase::AccelClamp);
        let restore = self.first_index(Phase::AccelRestore);
        if clamp.is_some() != restore.is_some() {
            return false;
        }
        match (self.first_index(Phase::Probe), restore) {
            (_, None) => true,
            (Some(probe), Some(restore)) => probe < restore,
            (None, Some(_)) => false,
        }
    }

    /// The abort cleanup for the accel clamp is declared: whenever an
    /// accel-clamp step exists it carries a non-empty
    /// [`RecoveryStep::cleanup_commands`] restoring the accel on abort.
    #[must_use]
    pub fn accel_clamp_declares_cleanup(&self) -> bool {
        self.first_index(Phase::AccelClamp).is_none_or(|i| {
            self.steps[i]
                .cleanup_commands
                .iter()
                .any(|c| c.contains(TRUE_Z_PLACEHOLDER) || c.contains(RESTORE_ACCEL_PLACEHOLDER))
        })
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

    /// The first heating action is a `M140` (bed) sent before ANY
    /// motion command: bed heating is the long pole, so it starts before
    /// homing or any move (vacuously true when there is no bed target
    /// and hence no `M140`, but then no motion may precede any other
    /// heat command either — the immediate-bed-heat step still runs
    /// first among heating).
    #[must_use]
    pub fn bed_heat_precedes_motion(&self) -> bool {
        let first_motion = self
            .steps
            .iter()
            .position(|s| s.commands.iter().any(|c| is_motion_command(c)));
        let first_m140 = self.steps.iter().position(|s| {
            s.phase == Phase::ImmediateBedHeat && s.commands.iter().any(|c| first_word(c) == "M140")
        });
        match (first_m140, first_motion) {
            // A bed target exists: its M140 must precede all motion.
            (Some(heat), Some(motion)) => heat < motion,
            // No motion at all, or no bed target (no M140): nothing to
            // check — the immediate-bed-heat step is positioned before
            // motion by construction.
            _ => true,
        }
    }

    /// The believed-Z declaration and its pre-home lift precede XY
    /// homing (so the homing moves cannot drag the nozzle across the
    /// part).
    #[must_use]
    pub fn believed_z_precedes_home_xy(&self) -> bool {
        match (
            self.first_index(Phase::BelievedZDeclare),
            self.first_index(Phase::HomeXy),
        ) {
            (Some(believed), Some(home)) => believed < home,
            (_, None) => true,
            (None, Some(_)) => false,
        }
    }

    /// The clean-nozzle step sits after XY homing and before the shifted
    /// frame (so a physically-clean, homed nozzle enters the probe
    /// phase).
    #[must_use]
    pub fn clean_nozzle_between_home_and_shifted(&self) -> bool {
        let Some(clean) = self.first_index(Phase::CleanNozzle) else {
            return false;
        };
        let after_home = self
            .first_index(Phase::HomeXy)
            .is_some_and(|home| home < clean);
        let before_shifted = self
            .first_index(Phase::ShiftedFrame)
            .is_some_and(|shifted| clean < shifted);
        after_home && before_shifted
    }

    /// The probe-temperature hold (when present) sits after XY homing and
    /// before the clean-nozzle step, so heat-up ooze is wiped by the
    /// clean instead of being deposited during the probe.
    ///
    /// Vacuously true when there is no hold step — the documented
    /// cold-drag opt-out (`drag_nozzle_temp = 0`).
    #[must_use]
    pub fn probe_temp_hold_precedes_clean_nozzle(&self) -> bool {
        let Some(hold) = self.first_index(Phase::ProbeTempHold) else {
            return true;
        };
        let after_home = self
            .first_index(Phase::HomeXy)
            .is_some_and(|home| home < hold);
        let before_clean = self
            .first_index(Phase::CleanNozzle)
            .is_some_and(|clean| hold < clean);
        after_home && before_clean
    }

    /// The reheat park precedes the frame restore (park first, so the
    /// restore/reheat never dwells against the part).
    #[must_use]
    pub fn park_precedes_restore(&self) -> bool {
        match (
            self.first_index(Phase::ParkForReheat),
            self.first_index(Phase::RestoreFrame),
        ) {
            (Some(park), Some(restore)) => park < restore,
            _ => false,
        }
    }

    /// The recovery-file select (`M23`/`M24`) is the final step of the
    /// plan.
    #[must_use]
    pub fn recovery_file_select_last(&self) -> bool {
        self.steps
            .last()
            .is_some_and(|s| s.phase == Phase::RecoveryFileSelect)
    }

    /// The [`Phase::ResumePreview`] step exists in the plan exactly when
    /// [`Self::preview`] carries a spec, and vice versa: a preview the
    /// executor cannot enter (no step) and a step with nothing to drive (no
    /// spec) are both malformed plans (design §D.2).
    #[must_use]
    pub fn resume_preview_step_iff_spec(&self) -> bool {
        self.first_index(Phase::ResumePreview).is_some() == self.preview.is_some()
    }

    /// The resume-preview step (when present) sits AFTER the true-Z declare
    /// and BEFORE the recovery-file select — i.e. after the frame is fully
    /// declared, so an abort during the reposition loop invalidates the Z
    /// frame exactly as a `ZConfirmStandoff` abort would (design §D.2 /
    /// §6). Vacuous when there is no preview step.
    #[must_use]
    pub fn resume_preview_after_true_z(&self) -> bool {
        let Some(preview) = self.first_index(Phase::ResumePreview) else {
            return true;
        };
        let after_declare = self
            .first_index(Phase::TrueZDeclare)
            .is_some_and(|tz| tz < preview);
        let before_select = self
            .first_index(Phase::RecoveryFileSelect)
            .is_some_and(|sel| preview < sel);
        after_declare && before_select
    }

    /// The `ask`-policy preview SUBSUMES the standalone Z-confirm pause
    /// (ruling §E.4): the two never coexist. A plan with both a
    /// [`Phase::ResumePreview`] and a [`Phase::ZConfirmStandoff`] step
    /// would pause twice for the same Z, so this refuses it.
    #[must_use]
    pub fn preview_subsumes_z_confirm(&self) -> bool {
        !(self.first_index(Phase::ResumePreview).is_some()
            && self.first_index(Phase::ZConfirmStandoff).is_some())
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
            "# recovery file: {} (verbatim tail from original byte {})",
            self.resume_file, self.resume_offset
        );
        if self.requires_clean_nozzle_confirmation {
            out.push_str(
                "# note: no clean-nozzle macro configured; confirm the nozzle is clean before executing\n",
            );
        }
        if self.debug_confirm_each_step {
            out.push_str(
                "# note: debug_confirm_each_step is set; execution pauses before EVERY step\n",
            );
        }
        let p = self.envelope.params;
        match p.overshoot {
            crate::envelope::OvershootTerm::PostTriggerTravel { probe_speed } => {
                let _ = writeln!(
                    out,
                    "# envelope: gap {} + 0.15 x speed {} + margin {} = {} mm",
                    fmt_num(p.expected_gap),
                    fmt_num(probe_speed),
                    fmt_num(p.margin),
                    fmt_num(self.envelope.envelope),
                );
            }
            crate::envelope::OvershootTerm::DragStep { drag_z_step } => {
                let _ = writeln!(
                    out,
                    "# envelope: gap {} + drag_z_step {} + margin {} = {} mm",
                    fmt_num(p.expected_gap),
                    fmt_num(drag_z_step),
                    fmt_num(p.margin),
                    fmt_num(self.envelope.envelope),
                );
            }
        }
        let _ = writeln!(
            out,
            "# shifted frame: Z declared {} above position_min {}",
            fmt_num(self.envelope.envelope),
            fmt_num(self.envelope.position_min),
        );
        for warning in &self.warnings {
            let _ = writeln!(out, "# warning: {}", warning.describe());
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
            for command in &step.cleanup_commands {
                let _ = writeln!(out, "      undo: {command}");
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
        fmt_num, hover_plane_at, true_z_at_halt, AbortReason, Phase, PlanWarning, Predicate,
        TriggerSource, TrueZFormula,
    };

    #[test]
    fn hover_plane_never_descends_and_clears_geometry() {
        // The §10 attack #3: a stop Z that would make the hover descend.
        // The hover plane is min(z_max, max(current_z, target_z)), so it is
        // ALWAYS >= current_z (never a descent) and >= target_z unless the
        // rail forces it lower — and even then never below current.
        //
        // target_z is the highest stop's g-code Z plus the standoff; the
        // reviewer crafts a "high-then-low layer order" so max(stop.z) is
        // large. That only RAISES target_z, so the plane rises — it cannot
        // be driven down toward a low stop.
        let cases = [
            // (current_z, target_z, z_max, expected)
            (0.3, 5.0, Some(200.0), 5.0), // ordinary post-probe lift
            (8.0, 5.0, Some(200.0), 8.0), // already above the plane: stay
            (0.3, 5.0, Some(4.0), 4.0),   // rail below target: clamp to rail
            (0.3, 5.0, Some(0.1), 0.3),   // rail below current: never descend
            (5.0, 5.0, Some(200.0), 5.0), // exactly at the plane
            (0.0, 0.0, None, 0.0),        // no rail known
        ];
        for (current, target, z_max, want) in cases {
            let got = hover_plane_at(current, target, z_max).unwrap();
            assert!(
                (got - want).abs() < 1e-12,
                "hover_plane_at({current}, {target}, {z_max:?}) = {got}, want {want}"
            );
            assert!(got >= current - 1e-12, "hover plane must never descend");
        }
        // Non-finite inputs are refused (the daemon aborts, never
        // substitutes a bad Z).
        assert!(hover_plane_at(f64::NAN, 5.0, None).is_err());
        assert!(hover_plane_at(0.0, f64::INFINITY, None).is_err());
        assert!(hover_plane_at(0.0, 5.0, Some(f64::NAN)).is_err());
    }

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
    fn true_z_drag_source_uses_the_reading_raw() {
        // The drag trigger Z is already raw toolhead Z (nozzle as
        // stylus, no probe z_offset): identical arithmetic to the raw
        // last_z_result source.
        let f = TrueZFormula {
            z_prev_top: 12.4,
            trigger_source: TriggerSource::DragResult,
            frozen_z_adjust: None,
        };
        // Contact pass at shifted 0.90; the staircase halts there.
        let z = true_z_at_halt(&f, 0.90, 0.90).unwrap();
        assert!((z - 12.4).abs() < 1e-12);
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
        assert_eq!(Phase::RecoveryFileSelect.name(), "recovery-file-select");
        assert_eq!(Phase::ImmediateBedHeat.name(), "immediate-bed-heat");
        assert_eq!(Phase::CleanNozzle.name(), "clean-nozzle");
        assert_eq!(AbortReason::ProbeNoTrigger.code(), "probe-no-trigger");
        assert_eq!(
            AbortReason::RecoveryFileSelectFailed.code(),
            "recovery-file-select-failed"
        );
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
            .contains("computed value"));
        assert_eq!(Predicate::NumAtMost { max: 0.015 }.describe(), "<= 0.015");
    }

    /// Every phase and abort reason has a stable kebab-case name, and no
    /// two share one.
    ///
    /// These strings are the transcript's and the socket's vocabulary:
    /// an operator greps for them and a client branches on them, so a
    /// duplicate or a typo is a silent contract break rather than a
    /// visible bug.
    #[test]
    fn phase_and_abort_names_are_unique_and_kebab_case() {
        let phases = [
            Phase::IdleTimeout,
            Phase::StepperEnable,
            Phase::RecoveryAccel,
            Phase::ImmediateBedHeat,
            Phase::BelievedZDeclare,
            Phase::HomeXy,
            Phase::ProbeTempHold,
            Phase::CleanNozzle,
            Phase::TransformFreeze,
            Phase::ShiftedFrame,
            Phase::ProbeApproach,
            Phase::AccelClamp,
            Phase::Probe,
            Phase::AccelRestore,
            Phase::TrueZDeclare,
            Phase::ZConfirmStandoff,
            Phase::MeshLoad,
            Phase::FinalDeclare,
            Phase::ParkForReheat,
            Phase::RestoreFrame,
            Phase::RecoveryAccelRestore,
            Phase::RecoveryFileSelect,
        ];
        let mut names: Vec<&str> = phases.iter().map(|p| p.name()).collect();
        assert_eq!(names.len(), 22, "a Phase was added: name it here too");
        for name in &names {
            assert!(
                !name.is_empty()
                    && name
                        .chars()
                        .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-'),
                "{name:?} is not kebab-case"
            );
        }
        names.sort_unstable();
        let count = names.len();
        names.dedup();
        assert_eq!(names.len(), count, "duplicate phase name");
        // Declaration order IS emission order (the builder relies on it).
        assert!(phases.windows(2).all(|w| w[0] < w[1]));

        let reasons = [
            AbortReason::IdleTimeoutNotApplied,
            AbortReason::StepperEnableFailed,
            AbortReason::RecoveryAccelFailed,
            AbortReason::RecoveryAccelRestoreFailed,
            AbortReason::ZConfirmStandoffFailed,
            AbortReason::ImmediateBedHeatFailed,
            AbortReason::BelievedZDeclareFailed,
            AbortReason::HomingFailed,
            AbortReason::ProbeTempHoldFailed,
            AbortReason::CleanNozzleFailed,
            AbortReason::TransformFreezeFailed,
            AbortReason::ShiftedFrameNotDeclared,
            AbortReason::ApproachFailed,
            AbortReason::AccelClampFailed,
            AbortReason::AccelRestoreFailed,
            AbortReason::ProbeNoTrigger,
            AbortReason::TrueZDeclareFailed,
            AbortReason::MeshLoadFailed,
            AbortReason::FinalDeclareFailed,
            AbortReason::ParkForReheatFailed,
            AbortReason::RestoreFailed,
            AbortReason::RecoveryFileSelectFailed,
        ];
        let mut codes: Vec<&str> = reasons.iter().map(|r| r.code()).collect();
        assert_eq!(
            codes.len(),
            22,
            "an AbortReason was added: code it here too"
        );
        codes.sort_unstable();
        let count = codes.len();
        codes.dedup();
        assert_eq!(codes.len(), count, "duplicate abort reason code");
    }

    /// The two configuration warnings render into the plan rather than
    /// only into the structured diagnosis: an operator reading the plan
    /// text must see that an override is in force.
    #[test]
    fn the_configuration_warnings_describe_themselves() {
        let unsafe_active = PlanWarning::UnsafeOverrideActive {
            key: "UNSAFE_allow_purge_z_below_bed".to_owned(),
            permitted: "purge_z_below_bed".to_owned(),
        }
        .describe();
        assert!(unsafe_active.contains("UNSAFE_allow_purge_z_below_bed"));
        assert!(unsafe_active.contains("purge_z_below_bed"));
        let ignored = PlanWarning::AccelProbeIgnoredOnTouchPath { accel_probe: 400.0 }.describe();
        assert!(ignored.contains("400"), "{ignored}");
        assert!(ignored.contains("touch_accel"), "{ignored}");
    }
}
