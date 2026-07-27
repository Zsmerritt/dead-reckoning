//! Recovery orchestration for power-loss recovery on a moving-bed-Z
//! machine: machine-prerequisite validation, probe-envelope arithmetic,
//! and generation of the strictly ordered, verifiable recovery/resume
//! plan.
//!
//! # The machine
//!
//! The bed rises into a fixed gantry: XY can be re-homed at will, but
//! **Z must NEVER be re-homed** — there is no Z homing move that does
//! not risk driving the bed into the nozzle. Every Z motion in a
//! recovery is a bounded move inside a frame this crate declares
//! explicitly (`SET_KINEMATIC_POSITION`), sized by the probe envelope
//! ([`envelope`]) so that Klipper's own rail-limit checking
//! structurally bounds the descent even with a faulty probe.
//!
//! # Pure logic
//!
//! No I/O, no sockets. The plan is **data** ([`RecoveryPlan`]): typed
//! steps carrying command strings, machine-readable verification
//! predicates, and typed failure actions. The `plrd` daemon executes
//! it and never continues past a failed verification.
//!
//! # Pipeline position
//!
//! ```text
//! plr-reconstruct ──► plr-analyzer ──► plr-recovery ──► plrd (executes)
//!   possible-stop      match + contact    validated plan
//! ```
//!
//! [`build::plan_recovery`] consumes the sibling crates' outputs
//! ([`plr_reconstruct::Reconstruction`],
//! [`plr_analyzer::ContactOutcome`], [`plr_analyzer::MatchResult`]) and
//! produces a [`build::PlanOutcome`]: a plan, a typed manual-recovery
//! fallback, or "no recovery needed" for clean shutdowns.
//!
//! # Totality
//!
//! No public function panics on any input; hostile numbers (NaN,
//! infinity) surface as [`RecoveryError`] and can never reach a
//! generated plan (property-tested in `tests/properties.rs`).

pub mod build;
pub mod diagnosis;
pub mod envelope;
pub mod error;
pub mod guard;
pub mod machine;
pub mod plan;
pub mod preflight;
pub mod preheat;
pub mod resume_file;

pub use build::{
    plan_recovery, preflight_generated_file, resolve_resume_with_preview, select_resume_target,
    select_resume_target_with_policy, ExcludeObjectDef, FallbackReason, PlanConfig, PlanInputs,
    PlanOutcome, ResumePolicy, ResumeTarget, ACCEL_MAX, ACCEL_MIN, CONFIRM_TIMEOUT_DEFAULT,
    CONFIRM_TIMEOUT_DEFAULT_S, CONFIRM_TIMEOUT_MAX_S, CONFIRM_TIMEOUT_MIN_S, DRAG_TEMP_FLOOR,
    GCODE_BARRIER_TIMEOUT_DEFAULT, GCODE_BARRIER_TIMEOUT_DEFAULT_S, GCODE_BARRIER_TIMEOUT_MAX_S,
    GCODE_BARRIER_TIMEOUT_MIN_S, PROBE_HOLD_BAND, PROBE_TEMP_HEADROOM,
    PROBE_TEMP_MEASURED_TOLERANCE,
};
pub use diagnosis::{Diagnose, Diagnosis, Expected, Measured, Tier, UNSAFE_PURGE_Z_BELOW_BED};
pub use envelope::{
    compute_envelope, Envelope, EnvelopeParams, OvershootTerm, POST_TRIGGER_TRAVEL_S,
    PROBE_SPEED_MAX, PROBE_SPEED_MIN,
};
pub use error::RecoveryError;
pub use guard::{
    sanitize_macro_text, scan_macro_text, GuardHit, GuardOutcome, GuardScan, GUARDED_COMMANDS,
};
pub use machine::{
    validate_machine, AxisLimits, MachineConfig, MachineRejection, PrereqFailure, ProbeConfig,
    ProbeKind, ValidatedMachine, ZStepper,
};
pub use plan::{
    fmt_num, hover_plane_at, park_z_at, true_z_at_halt, AbortReason, FailureAction, Phase,
    PlanWarning, Predicate, PreviewBinding, PreviewSpec, RecoveryPlan, RecoveryStep,
    RuntimeComputation, TriggerSource, TrueZFormula, Verification, MACHINE_ACCEL_PLACEHOLDER,
    PARK_Z_PLACEHOLDER, RESTORE_ACCEL_PLACEHOLDER, TRUE_Z_PLACEHOLDER,
};
pub use preflight::{
    preflight_itinerary, preflight_recovery_file, BoundsViolation, ItineraryBounds, PlanRejection,
    ViolationKind, RECOVERY_FILE_STEP_ID,
};
pub use preheat::{derive_preheat, scan_file_temps, FileTemps, PreheatTargets};
pub use resume_file::{
    build_recovery_file, is_motion_command, recovery_file_name, sanitize_name, verify_heating_gate,
    GeneratedRecoveryFile, HeatingGateViolation, PurgePlan, RecoveryFileSpec, ZIntent,
    MOTION_COMMANDS,
};
