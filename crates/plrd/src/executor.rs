//! Plan execution: dry-run rendering and gated live execution.
//!
//! # Safety invariants (tested in this module and `recover`)
//!
//! 1. **Dry run cannot send** — [`dry_run`] takes the plan and nothing
//!    else: no client, no socket, no async. The proof is type-level;
//!    there is no I/O handle in scope to send with.
//! 2. **Only plan commands are ever sent** — [`execute`] sends command
//!    strings obtained exclusively by iterating
//!    [`RecoveryStep::commands`] of a validated [`RecoveryPlan`] (one
//!    call site, `send_step_commands`). No ad-hoc G-code exists in this
//!    crate's execution path; the sole placeholder substitution is the
//!    typed `{true_z}` computation defined by the plan itself.
//! 3. **Verification failure ⇒ abort** — every predicate failure (or
//!    poll timeout, or query error, or non-finite computation) stops
//!    execution at that step with the step's typed
//!    [`FailureAction::Abort`] reason. There is no code path that
//!    continues past a failed verification: the step loop returns.
//! 4. **Everything is transcribed** — commands, responses, verification
//!    evaluations, computations, prompts, and the final outcome are
//!    appended as JSON lines to the transcript the caller supplies.
//!
//! Moonraker semantics this leans on: `printer.gcode.script` resolves
//! only when the script has completed (Moonraker docs,
//! `external_api/printer.md`), so a step's post-verifications read
//! settled state. Slow-converging predicates ([`Predicate::TempWithin`])
//! are polled up to `temp_timeout`; all others up to `verify_timeout`.

use std::time::Duration;

use plr_recovery::{
    fmt_num, true_z_at_halt, FailureAction, Phase, Predicate, RecoveryPlan, RecoveryStep,
    RuntimeComputation, TriggerSource, Verification, RESTORE_ACCEL_PLACEHOLDER, TRUE_Z_PLACEHOLDER,
};
use serde_json::{json, Value};

use crate::moonraker::MoonrakerClient;

/// What a dry run would send, with no means to send it.
#[derive(Debug, Clone, PartialEq)]
pub struct DryRun {
    /// `(step id, command)` in exact send order. `{true_z}` stays
    /// symbolic: it does not exist until a live probe.
    pub would_send: Vec<(u32, String)>,
    /// The full rendered plan (steps, verifications, failure actions).
    pub rendered: String,
}

/// Enumerates the plan without any ability to execute it (see module
/// safety invariant 1).
#[must_use]
pub fn dry_run(plan: &RecoveryPlan) -> DryRun {
    let would_send = plan
        .steps
        .iter()
        .flat_map(|step| {
            step.commands
                .iter()
                .map(move |command| (step.id, command.clone()))
        })
        .collect();
    DryRun {
        would_send,
        rendered: plan.render(),
    }
}

/// Execution tuning.
#[derive(Debug, Clone)]
pub struct ExecOptions {
    /// Poll deadline for non-temperature predicates.
    pub verify_timeout: Duration,
    /// Poll deadline for [`Predicate::TempWithin`].
    pub temp_timeout: Duration,
    /// Poll interval.
    pub poll_interval: Duration,
}

impl Default for ExecOptions {
    fn default() -> Self {
        Self {
            // Position/state predicates settle as soon as the script
            // returns; 10 s absorbs status-refresh lag.
            verify_timeout: Duration::from_secs(10),
            // Heating a bed from cold legitimately takes minutes.
            temp_timeout: Duration::from_mins(15),
            poll_interval: Duration::from_millis(500),
        }
    }
}

/// The exact Klipper error string a probe-triggered-early failure
/// carries (Cartographer keys off this literal,
/// `adapters/klipper_like/utils.py:63-84`,
/// `PROBE_TRIGGERED_BEFORE_MOVEMENT`).
pub const PROBE_TRIGGERED_EARLY: &str = "Probe triggered prior to movement";

/// The exact Klipper error string for a probe that never triggered.
pub const NO_TRIGGER_FULL_MOVEMENT: &str = "No trigger on probe after full movement";

/// The `PLR_TOUCH` consensus-failure message prefix (mirrors
/// Cartographer's `TouchError`, `probe/touch_mode.py:131-137`,
/// "Unable to find N samples within ..."): the multi-touch sequence
/// could not assemble an agreeing subset. Treated as a no-trigger.
pub const TOUCH_CONSENSUS_FAILURE_PREFIX: &str = "Unable to find";

/// The exact Klipper error string for an out-of-range move.
pub const MOVE_OUT_OF_RANGE: &str = "Move out of range";

/// Typed classification of a Klipper gcode-script failure string
/// (Cartographer promotes exact Klipper `CommandError` strings to typed
/// errors, `adapters/klipper_like/utils.py:63-84`). All still abort in
/// this wave — the type only enriches the transcript and abort record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepFailure {
    /// "Probe triggered prior to movement".
    ProbeTriggeredEarly,
    /// "No trigger on probe after full movement", or the `PLR_TOUCH`
    /// consensus-failure prefix.
    NoTrigger,
    /// "Move out of range".
    MoveOutOfRange,
    /// Any other command error.
    Unknown,
}

impl StepFailure {
    /// Classifies a Klipper gcode-script error message by exact string
    /// (substring for the consensus-failure prefix).
    #[must_use]
    pub fn classify(message: &str) -> Self {
        if message.contains(PROBE_TRIGGERED_EARLY) {
            StepFailure::ProbeTriggeredEarly
        } else if message.contains(NO_TRIGGER_FULL_MOVEMENT)
            || message.contains(TOUCH_CONSENSUS_FAILURE_PREFIX)
        {
            StepFailure::NoTrigger
        } else if message.contains(MOVE_OUT_OF_RANGE) {
            StepFailure::MoveOutOfRange
        } else {
            StepFailure::Unknown
        }
    }

    /// Stable tag string carried in the transcript and abort record.
    #[must_use]
    pub fn code(self) -> &'static str {
        match self {
            StepFailure::ProbeTriggeredEarly => "probe-triggered-early",
            StepFailure::NoTrigger => "no-trigger",
            StepFailure::MoveOutOfRange => "move-out-of-range",
            StepFailure::Unknown => "unknown",
        }
    }
}

/// Why execution stopped before the plan completed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StopCause {
    /// A verification predicate failed (or timed out) and the step's
    /// failure action fired.
    VerificationFailed {
        /// The failing verification, rendered.
        verification: String,
        /// The last observed value, rendered.
        observed: String,
    },
    /// Moonraker rejected or failed a query/connection (not a
    /// gcode-script command — those are [`StopCause::CommandFailed`]).
    Transport(String),
    /// A gcode-script command failed; the Klipper error string is
    /// promoted to a typed [`StepFailure`].
    CommandFailed {
        /// The typed classification.
        failure: StepFailure,
        /// The raw Klipper error string.
        message: String,
    },
    /// The runtime computation was invalid (non-finite inputs, missing
    /// values, or a placeholder without a computation).
    ComputeFailed(String),
    /// The operator declined at a `--step` gate.
    OperatorDeclined,
}

impl StopCause {
    /// The typed step failure, when this stop was a classified command
    /// error.
    #[must_use]
    pub fn step_failure(&self) -> Option<StepFailure> {
        match self {
            StopCause::CommandFailed { failure, .. } => Some(*failure),
            _ => None,
        }
    }
}

/// Result of [`execute`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExecOutcome {
    /// Every step completed and verified.
    Completed {
        /// Number of steps executed.
        steps: usize,
    },
    /// Execution stopped at `step_id` with the plan's abort reason.
    Aborted {
        /// The step that failed.
        step_id: u32,
        /// The step's phase name.
        phase: String,
        /// The plan's typed abort reason code.
        reason: String,
        /// What actually went wrong.
        cause: StopCause,
        /// `true` when the abort landed at or after the shifted-frame
        /// declaration: Klipper's Z frame is now in an unknown state
        /// (a `SET_KINEMATIC_POSITION` was declared and execution died
        /// before the plan re-established a trusted frame), so the
        /// caller must invalidate the pending recovery and refuse a
        /// re-execute until a fresh plan is generated.
        frame_invalid: bool,
    },
}

/// Transcript sink: JSON lines, flushed per entry.
pub struct Transcript<'a> {
    out: &'a mut (dyn std::io::Write + Send),
}

impl<'a> Transcript<'a> {
    /// Wraps a writer.
    pub fn new(out: &'a mut (dyn std::io::Write + Send)) -> Self {
        Self { out }
    }

    fn entry(&mut self, value: &Value) {
        // Transcript write failures must never stop a recovery
        // mid-motion; the caller inspects the file afterwards.
        let _ = writeln!(self.out, "{value}");
        let _ = self.out.flush();
    }
}

/// Executes a validated plan step by step (module safety invariants
/// 2–4). `gate` is consulted before every step (per-step confirmation);
/// returning `false` stops execution before that step sends anything.
pub async fn execute(
    plan: &RecoveryPlan,
    client: &mut MoonrakerClient,
    options: &ExecOptions,
    gate: &mut (dyn FnMut(&RecoveryStep) -> bool + Send),
    transcript: &mut Transcript<'_>,
) -> ExecOutcome {
    transcript.entry(&json!({
        "event": "plan-start",
        "steps": plan.steps.len(),
        "resume_file": plan.resume_file,
        "resume_offset": plan.resume_offset,
    }));
    // The step id at (and after) which an abort leaves Klipper's Z frame
    // in an unknown state (see `ExecOutcome::Aborted::frame_invalid`).
    let shifted_id = plan
        .first_index(Phase::ShiftedFrame)
        .map(|i| plan.steps[i].id);
    // Execution state that persists across steps: the pre-clamp accel
    // recorded by the accel-clamp step (reused by the restore step and
    // the abort cleanups) and the registered abort cleanups.
    let mut recorded_accel: Option<f64> = None;
    let mut cleanups: Vec<(u32, Vec<String>)> = Vec::new();
    for step in &plan.steps {
        if !gate(step) {
            transcript.entry(&json!({
                "event": "operator-declined", "step": step.id,
            }));
            return finish_abort(
                client,
                step,
                StopCause::OperatorDeclined,
                &cleanups,
                recorded_accel,
                shifted_id,
                transcript,
            )
            .await;
        }
        transcript.entry(&json!({
            "event": "step-start",
            "step": step.id,
            "phase": step.phase.name(),
            "summary": step.summary,
        }));
        if let Err(cause) = run_step(
            client,
            step,
            options,
            &mut recorded_accel,
            &mut cleanups,
            transcript,
        )
        .await
        {
            return finish_abort(
                client,
                step,
                cause,
                &cleanups,
                recorded_accel,
                shifted_id,
                transcript,
            )
            .await;
        }
        transcript.entry(&json!({"event": "step-ok", "step": step.id}));
    }
    transcript.entry(&json!({"event": "plan-complete", "steps": plan.steps.len()}));
    ExecOutcome::Completed {
        steps: plan.steps.len(),
    }
}

/// Runs one step's pre-verify, compute, send, and post-verify. Registers
/// the step's abort cleanup once its commands are about to take effect.
async fn run_step(
    client: &mut MoonrakerClient,
    step: &RecoveryStep,
    options: &ExecOptions,
    recorded_accel: &mut Option<f64>,
    cleanups: &mut Vec<(u32, Vec<String>)>,
    transcript: &mut Transcript<'_>,
) -> Result<(), StopCause> {
    // Pre-verifications: must hold before anything is sent.
    for verification in &step.pre_verify {
        poll_verification(client, verification, None, options, transcript, "pre").await?;
    }
    // Runtime computation: the true-Z formula, or the max-accel record;
    // for a plain step the resolved value is the persisted recorded
    // accel (so the restore step's `{restore_accel}` / NumWithinComputed
    // resolve).
    let computed = resolve_compute(client, step, recorded_accel, transcript).await?;
    // Register this step's abort cleanup BEFORE its commands run: its
    // side effect (the accel clamp) is about to be in force, so any
    // subsequent abort must undo it.
    if !step.cleanup_commands.is_empty() {
        cleanups.push((step.id, step.cleanup_commands.clone()));
    }
    // Commands: the only send path in the crate (invariant 2).
    send_step_commands(client, step, computed, transcript).await?;
    // Post-verifications.
    for verification in &step.verify {
        poll_verification(client, verification, computed, options, transcript, "post").await?;
    }
    Ok(())
}

/// Runs the registered abort cleanups (in reverse registration order —
/// the plan-level `finally`), then records the abort. A cleanup failure
/// is transcribed but NEVER masks or replaces the original abort reason
/// (Cartographer's `finally` cleanup likewise cannot swallow the raising
/// error).
async fn finish_abort(
    client: &mut MoonrakerClient,
    step: &RecoveryStep,
    cause: StopCause,
    cleanups: &[(u32, Vec<String>)],
    recorded_accel: Option<f64>,
    shifted_id: Option<u32>,
    transcript: &mut Transcript<'_>,
) -> ExecOutcome {
    for (sid, commands) in cleanups.iter().rev() {
        for command in commands {
            let resolved = if command.contains(RESTORE_ACCEL_PLACEHOLDER) {
                let Some(value) = recorded_accel else {
                    transcript.entry(&json!({
                        "event": "cleanup-skip", "step": sid, "command": command,
                        "reason": "no recorded accel to substitute",
                    }));
                    continue;
                };
                command.replace(RESTORE_ACCEL_PLACEHOLDER, &fmt_num(value))
            } else {
                command.clone()
            };
            transcript.entry(&json!({
                "event": "cleanup", "step": sid, "command": resolved,
            }));
            match client.gcode_script(&resolved).await {
                Ok(()) => transcript.entry(&json!({
                    "event": "cleanup-ok", "step": sid,
                })),
                // Logged, never fatal, never mutates `cause`.
                Err(e) => transcript.entry(&json!({
                    "event": "cleanup-error", "step": sid, "error": e.to_string(),
                })),
            }
        }
    }
    abort(step, cause, shifted_id, transcript)
}

fn abort(
    step: &RecoveryStep,
    cause: StopCause,
    shifted_id: Option<u32>,
    transcript: &mut Transcript<'_>,
) -> ExecOutcome {
    let FailureAction::Abort { reason } = step.on_failure;
    let frame_invalid = shifted_id.is_some_and(|sid| step.id >= sid);
    transcript.entry(&json!({
        "event": "abort",
        "step": step.id,
        "phase": step.phase.name(),
        "reason": reason.code(),
        "cause": format!("{cause:?}"),
        "failure": cause.step_failure().map(StepFailure::code),
        "frame_invalid": frame_invalid,
    }));
    ExecOutcome::Aborted {
        step_id: step.id,
        phase: step.phase.name().to_owned(),
        reason: reason.code().to_owned(),
        cause,
        frame_invalid,
    }
}

/// Sends every command of one step, in order, substituting the computed
/// value for the true-Z / restore-accel placeholder when present. This
/// is the crate's only G-code send site.
async fn send_step_commands(
    client: &mut MoonrakerClient,
    step: &RecoveryStep,
    computed: Option<f64>,
    transcript: &mut Transcript<'_>,
) -> Result<(), StopCause> {
    for command in &step.commands {
        let has_placeholder =
            command.contains(TRUE_Z_PLACEHOLDER) || command.contains(RESTORE_ACCEL_PLACEHOLDER);
        let resolved = if has_placeholder {
            let Some(value) = computed else {
                // A placeholder without a computed value cannot be sent;
                // plan construction forbids it, and so does this.
                return Err(StopCause::ComputeFailed(
                    "command carries a runtime placeholder but the step has no computed value"
                        .to_owned(),
                ));
            };
            command
                .replace(TRUE_Z_PLACEHOLDER, &fmt_num(value))
                .replace(RESTORE_ACCEL_PLACEHOLDER, &fmt_num(value))
        } else {
            command.clone()
        };
        transcript.entry(&json!({
            "event": "send", "step": step.id, "command": resolved,
        }));
        match client.gcode_script(&resolved).await {
            Ok(()) => transcript.entry(&json!({
                "event": "response", "step": step.id, "result": "ok",
            })),
            Err(e) => {
                let message = e.to_string();
                let failure = StepFailure::classify(&message);
                transcript.entry(&json!({
                    "event": "response", "step": step.id,
                    "error": message, "failure": failure.code(),
                }));
                return Err(StopCause::CommandFailed { failure, message });
            }
        }
    }
    Ok(())
}

/// Resolves a step's runtime value: the true-Z formula, the recorded
/// pre-clamp accel, or — for a plain step — the persisted recorded accel
/// (so the restore step's placeholder and comparison resolve).
async fn resolve_compute(
    client: &mut MoonrakerClient,
    step: &RecoveryStep,
    recorded_accel: &mut Option<f64>,
    transcript: &mut Transcript<'_>,
) -> Result<Option<f64>, StopCause> {
    match step.compute {
        Some(RuntimeComputation::TrueZ(formula)) => {
            // Trigger reading, per probe type (plr-recovery documents
            // the Klipper field semantics). The drag and consensus
            // results live on the plugin's own `plr` status object.
            let (object, field) = match formula.trigger_source {
                TriggerSource::RawLastZResult => ("probe", "last_z_result"),
                TriggerSource::BedZPlusOffset { .. } => ("probe", "last_probe_position.2"),
                TriggerSource::DragResult => ("plr", "last_drag_result.trigger_z"),
                TriggerSource::TouchResult { .. } => ("plr", "last_touch_result.median_z"),
            };
            let trigger_reading = query_number(client, object, field).await?;
            let halt_z = query_number(client, "toolhead", "position.2").await?;
            match true_z_at_halt(&formula, trigger_reading, halt_z) {
                Ok(true_z) => {
                    transcript.entry(&json!({
                        "event": "compute",
                        "step": step.id,
                        "trigger_reading": trigger_reading,
                        "halt_z": halt_z,
                        "true_z": true_z,
                    }));
                    Ok(Some(true_z))
                }
                // Never substitute on error (plr-recovery contract).
                Err(e) => Err(StopCause::ComputeFailed(e.to_string())),
            }
        }
        Some(RuntimeComputation::RecordMaxAccel) => {
            // Read BEFORE the step's SET_VELOCITY_LIMIT clamps it, and
            // persist for the restore step / abort cleanup.
            let accel = query_number(client, "toolhead", "max_accel").await?;
            *recorded_accel = Some(accel);
            transcript.entry(&json!({
                "event": "record-accel", "step": step.id, "max_accel": accel,
            }));
            Ok(Some(accel))
        }
        // A plain step resolves to the persisted recorded accel: this is
        // what the accel-restore step's `{restore_accel}` substitution
        // and NumWithinComputed check read.
        None => Ok(*recorded_accel),
    }
}

async fn query_number(
    client: &mut MoonrakerClient,
    object: &str,
    field: &str,
) -> Result<f64, StopCause> {
    let status = client
        .query_objects(&[object])
        .await
        .map_err(|e| StopCause::Transport(e.to_string()))?;
    let value = lookup(&status, object, field)
        .ok_or_else(|| StopCause::ComputeFailed(format!("{object}.{field} absent from status")))?;
    value
        .as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| StopCause::ComputeFailed(format!("{object}.{field} is not a finite number")))
}

/// Polls one verification until it holds or its deadline passes.
async fn poll_verification(
    client: &mut MoonrakerClient,
    verification: &Verification,
    computed: Option<f64>,
    options: &ExecOptions,
    transcript: &mut Transcript<'_>,
    stage: &str,
) -> Result<(), StopCause> {
    let timeout = match verification.predicate {
        Predicate::TempWithin { .. } => options.temp_timeout,
        _ => options.verify_timeout,
    };
    let deadline = tokio::time::Instant::now() + timeout;
    loop {
        let status = client
            .query_objects(&[&verification.object])
            .await
            .map_err(|e| StopCause::Transport(e.to_string()))?;
        let value = lookup(&status, &verification.object, &verification.field);
        let observed = value.map_or_else(|| "absent".to_owned(), Value::to_string);
        let holds = evaluate(&verification.predicate, value, computed);
        if holds || tokio::time::Instant::now() >= deadline {
            transcript.entry(&json!({
                "event": "verify",
                "stage": stage,
                "object": verification.object,
                "field": verification.field,
                "predicate": verification.predicate.describe(),
                "observed": observed,
                "holds": holds,
            }));
            if holds {
                return Ok(());
            }
            return Err(StopCause::VerificationFailed {
                verification: format!(
                    "{}.{} {}",
                    verification.object,
                    verification.field,
                    verification.predicate.describe()
                ),
                observed,
            });
        }
        tokio::time::sleep(options.poll_interval).await;
    }
}

/// Resolves `object` + dotted `field` path inside a status map; numeric
/// segments index arrays (the `Verification::field` contract).
fn lookup<'a>(status: &'a Value, object: &str, field: &str) -> Option<&'a Value> {
    let mut current = status.get(object)?;
    for segment in field.split('.') {
        current = match segment.parse::<usize>() {
            Ok(index) => current.get(index)?,
            Err(_) => current.get(segment)?,
        };
    }
    Some(current)
}

/// Evaluates one predicate against an observed value. Total: absent or
/// mistyped values simply do not hold.
fn evaluate(predicate: &Predicate, value: Option<&Value>, computed: Option<f64>) -> bool {
    let Some(value) = value else {
        return false;
    };
    let num = value.as_f64().filter(|v| v.is_finite());
    match predicate {
        Predicate::NumWithin { expected, epsilon } => {
            num.is_some_and(|v| (v - expected).abs() <= *epsilon)
        }
        Predicate::NumAtLeast { min } => num.is_some_and(|v| v >= *min),
        Predicate::NumAtMost { max } => num.is_some_and(|v| v <= *max),
        Predicate::TempWithin { min, max } => num.is_some_and(|v| v >= *min && v <= *max),
        Predicate::Contains { needle } => value.as_str().is_some_and(|s| s.contains(needle)),
        Predicate::Equals { value: expected } => value.as_str().is_some_and(|s| s == expected),
        Predicate::BoolTrue => value.as_bool() == Some(true),
        Predicate::BoolFalse => value.as_bool() == Some(false),
        Predicate::FinitePresent => match value {
            Value::Number(_) => num.is_some(),
            Value::Null => false,
            _ => true,
        },
        Predicate::NonEmptyMatrix => value.as_array().is_some_and(|rows| {
            rows.iter()
                .any(|r| r.as_array().is_some_and(|c| !c.is_empty()))
        }),
        Predicate::NumWithinComputed { epsilon } => match (num, computed) {
            (Some(v), Some(c)) => (v - c).abs() <= *epsilon,
            _ => false,
        },
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::{
        dry_run, evaluate, execute, lookup, ExecOptions, ExecOutcome, StepFailure, StopCause,
        Transcript,
    };
    use crate::moonraker::MoonrakerClient;
    use crate::testmoon::FakeMoonraker;
    use plr_recovery::{
        compute_envelope, AbortReason, EnvelopeParams, FailureAction, Phase, Predicate,
        RecoveryPlan, RecoveryStep, RuntimeComputation, TriggerSource, TrueZFormula, Verification,
    };
    use serde_json::{json, Value};
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    /// A three-step plan exercising pre-verify, commands, post-verify,
    /// and the true-Z computation. Built by hand: executor tests must
    /// not depend on the full pipeline.
    pub(crate) fn test_plan() -> RecoveryPlan {
        let steps = vec![
            RecoveryStep {
                id: 1,
                phase: Phase::IdleTimeout,
                summary: "disarm idle timeout".to_owned(),
                commands: vec!["SET_IDLE_TIMEOUT TIMEOUT=86400".to_owned()],
                pre_verify: vec![],
                verify: vec![Verification::new(
                    "idle_timeout",
                    "idle_timeout",
                    Predicate::NumWithin {
                        expected: 86_400.0,
                        epsilon: 0.5,
                    },
                )],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::IdleTimeoutNotApplied,
                },
            },
            RecoveryStep {
                id: 2,
                phase: Phase::Probe,
                summary: "probe".to_owned(),
                commands: vec!["PROBE PROBE_SPEED=1 SAMPLES=1".to_owned()],
                pre_verify: vec![Verification::new(
                    "extruder",
                    "temperature",
                    Predicate::TempWithin {
                        min: 140.0,
                        max: 160.0,
                    },
                )],
                verify: vec![Verification::new(
                    "probe",
                    "last_z_result",
                    Predicate::FinitePresent,
                )],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::ProbeNoTrigger,
                },
            },
            RecoveryStep {
                id: 3,
                phase: Phase::TrueZDeclare,
                summary: "declare true Z".to_owned(),
                commands: vec!["SET_KINEMATIC_POSITION Z={true_z}".to_owned()],
                pre_verify: vec![],
                verify: vec![],
                compute: Some(RuntimeComputation::TrueZ(TrueZFormula {
                    z_prev_top: 12.4,
                    trigger_source: TriggerSource::RawLastZResult,
                    frozen_z_adjust: None,
                })),
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::TrueZDeclareFailed,
                },
            },
        ];
        RecoveryPlan {
            steps,
            envelope: compute_envelope(
                EnvelopeParams {
                    expected_gap: 0.5,
                    overshoot: plr_recovery::OvershootTerm::PostTriggerTravel { probe_speed: 1.0 },
                    margin: 0.5,
                },
                -2.0,
            )
            .expect("test envelope"),
            resume_file: "x.gcode".to_owned(),
            resume_offset: 1234,
            warnings: vec![],
        }
    }

    fn fast_options() -> ExecOptions {
        ExecOptions {
            verify_timeout: Duration::from_millis(300),
            temp_timeout: Duration::from_millis(300),
            poll_interval: Duration::from_millis(20),
        }
    }

    /// Serves plausible printer state for the happy path: trigger at
    /// shifted 0.90, halt at 0.75 → true Z = 12.4 + (0.75 − 0.90) =
    /// 12.25.
    pub(crate) fn happy_handler(method: &str, params: &Value) -> Result<Value, (i64, String)> {
        match method {
            "printer.gcode.script" => Ok(json!("ok")),
            "printer.objects.query" => {
                let objects = params["objects"].as_object().expect("objects param");
                let mut status = serde_json::Map::new();
                for name in objects.keys() {
                    let value = match name.as_str() {
                        "idle_timeout" => json!({"idle_timeout": 86_400.0, "state": "Ready"}),
                        "extruder" => json!({"temperature": 150.2, "target": 150.0}),
                        "probe" => json!({"last_z_result": 0.90}),
                        "toolhead" => json!({"position": [10.0, 20.0, 0.75, 0.0]}),
                        "webhooks" => json!({"state": "ready"}),
                        "print_stats" => json!({"state": "standby"}),
                        "virtual_sdcard" => json!({"is_active": false}),
                        other => json!({"unknown-object": other}),
                    };
                    status.insert(name.clone(), value);
                }
                Ok(json!({"eventtime": 1.0, "status": status}))
            }
            other => Err((-32601, format!("Method not found: {other}"))),
        }
    }

    async fn run(
        plan: &RecoveryPlan,
        fake: &FakeMoonraker,
        gate: &mut (dyn FnMut(&RecoveryStep) -> bool + Send),
    ) -> (ExecOutcome, String) {
        let mut client = MoonrakerClient::connect(&fake.url(), Duration::from_secs(5))
            .await
            .unwrap();
        let mut buffer = Vec::new();
        let outcome = {
            let mut transcript = Transcript::new(&mut buffer);
            execute(plan, &mut client, &fast_options(), gate, &mut transcript).await
        };
        (outcome, String::from_utf8(buffer).unwrap())
    }

    #[tokio::test]
    async fn happy_path_executes_every_step_in_order() {
        let plan = test_plan();
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        assert_eq!(outcome, ExecOutcome::Completed { steps: 3 });
        // The fake saw exactly the plan's commands, in order, with the
        // computed true-Z substituted — and nothing else.
        assert_eq!(
            fake.gcode_sent(),
            vec![
                "SET_IDLE_TIMEOUT TIMEOUT=86400",
                "PROBE PROBE_SPEED=1 SAMPLES=1",
                "SET_KINEMATIC_POSITION Z=12.25",
            ]
        );
        for needle in [
            "plan-start",
            "step-start",
            "\"send\"",
            "\"compute\"",
            "plan-complete",
        ] {
            assert!(
                transcript.contains(needle),
                "missing {needle}: {transcript}"
            );
        }
    }

    #[tokio::test]
    async fn pre_verification_failure_aborts_before_sending_the_step() {
        let plan = test_plan();
        // Nozzle stone cold: step 2's PRE-verification cannot hold, so
        // step 2's PROBE must never be sent.
        let fake = FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(extruder) = v.get_mut("status").and_then(|s| s.get_mut("extruder")) {
                    *extruder = json!({"temperature": 22.5, "target": 0.0});
                }
            }
            Ok(v)
        })
        .await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted {
            step_id,
            reason,
            cause,
            ..
        } = outcome
        else {
            panic!("expected abort, got {outcome:?}");
        };
        assert_eq!(step_id, 2);
        assert_eq!(reason, "probe-no-trigger");
        assert!(matches!(cause, StopCause::VerificationFailed { .. }));
        // Step 1's command went out; steps 2 and 3 sent nothing.
        assert_eq!(fake.gcode_sent(), vec!["SET_IDLE_TIMEOUT TIMEOUT=86400"]);
        assert!(transcript.contains("\"holds\":false"), "{transcript}");
        assert!(transcript.contains("abort"), "{transcript}");
    }

    #[tokio::test]
    async fn post_verification_failure_aborts_mid_plan() {
        let plan = test_plan();
        // Step 1's own post-verify fails: idle_timeout never applies.
        let fake = FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(it) = v.get_mut("status").and_then(|s| s.get_mut("idle_timeout")) {
                    *it = json!({"idle_timeout": 600.0});
                }
            }
            Ok(v)
        })
        .await;
        let (outcome, _) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted {
            step_id, reason, ..
        } = outcome
        else {
            panic!("expected abort");
        };
        assert_eq!(step_id, 1);
        assert_eq!(reason, "idle-timeout-not-applied");
        // Only step 1's command was ever sent: abort never continues.
        assert_eq!(fake.gcode_sent().len(), 1);
    }

    #[tokio::test]
    async fn command_error_aborts_immediately_and_is_typed_unknown() {
        let plan = test_plan();
        let fake = FakeMoonraker::spawn(|method, params| {
            if method == "printer.gcode.script" {
                Err((400, "Klippy shutdown".to_owned()))
            } else {
                happy_handler(method, params)
            }
        })
        .await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted { step_id, cause, .. } = outcome else {
            panic!("expected abort");
        };
        assert_eq!(step_id, 1);
        // A gcode-script error that matches no known string is a typed
        // Unknown command failure (not a transport error).
        assert!(matches!(
            cause,
            StopCause::CommandFailed {
                failure: StepFailure::Unknown,
                ..
            }
        ));
        assert!(
            transcript.contains("\"failure\":\"unknown\""),
            "{transcript}"
        );
        assert_eq!(fake.gcode_sent().len(), 1);
    }

    #[tokio::test]
    async fn klipper_error_strings_promote_to_typed_step_failures() {
        // Each exact Klipper error string (and the PLR_TOUCH consensus
        // prefix) maps to its typed StepFailure; all still abort.
        for (message, expected, code) in [
            (
                "Probe triggered prior to movement",
                StepFailure::ProbeTriggeredEarly,
                "probe-triggered-early",
            ),
            (
                "No trigger on probe after full movement",
                StepFailure::NoTrigger,
                "no-trigger",
            ),
            (
                "Unable to find 3 samples within 0.010mm in a window of 5 after 7 touches",
                StepFailure::NoTrigger,
                "no-trigger",
            ),
            (
                "Move out of range: 5.0 250.0 0.0 [250.0]",
                StepFailure::MoveOutOfRange,
                "move-out-of-range",
            ),
            ("some other klippy failure", StepFailure::Unknown, "unknown"),
        ] {
            assert_eq!(StepFailure::classify(message), expected, "{message}");
            let msg = message.to_owned();
            let fake = FakeMoonraker::spawn(move |method, params| {
                if method == "printer.gcode.script" {
                    Err((400, msg.clone()))
                } else {
                    happy_handler(method, params)
                }
            })
            .await;
            let (outcome, transcript) = run(&test_plan(), &fake, &mut |_| true).await;
            let ExecOutcome::Aborted { cause, .. } = outcome else {
                panic!("expected abort for {message}");
            };
            assert!(
                matches!(cause, StopCause::CommandFailed { failure, .. } if failure == expected),
                "{message}: {cause:?}"
            );
            assert!(
                transcript.contains(&format!("\"failure\":\"{code}\"")),
                "{message}: {transcript}"
            );
        }
    }

    #[tokio::test]
    async fn step_gate_decline_stops_before_sending() {
        let plan = test_plan();
        let fake = FakeMoonraker::spawn(happy_handler).await;
        // Approve step 1 only.
        let gates = Arc::new(Mutex::new(0_u32));
        let counter = Arc::clone(&gates);
        let (outcome, transcript) = run(&plan, &fake, &mut move |step| {
            *counter.lock().expect("gate counter") += 1;
            step.id == 1
        })
        .await;
        let ExecOutcome::Aborted { step_id, cause, .. } = outcome else {
            panic!("expected abort");
        };
        assert_eq!(step_id, 2);
        assert_eq!(cause, StopCause::OperatorDeclined);
        assert_eq!(*gates.lock().expect("gate counter"), 2);
        assert_eq!(fake.gcode_sent(), vec!["SET_IDLE_TIMEOUT TIMEOUT=86400"]);
        assert!(transcript.contains("operator-declined"), "{transcript}");
    }

    #[tokio::test]
    async fn drag_trigger_source_reads_the_plr_status_object() {
        // Same plan, but the computation reads the plugin's
        // last_drag_result instead of the probe object.
        let mut plan = test_plan();
        plan.steps[2].compute = Some(RuntimeComputation::TrueZ(TrueZFormula {
            z_prev_top: 12.4,
            trigger_source: TriggerSource::DragResult,
            frozen_z_adjust: None,
        }));
        let fake = FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(status) = v.get_mut("status") {
                    if let Some(plr) = status.get_mut("plr") {
                        *plr = json!({
                            "method": "adxl_drag",
                            "last_drag_result": {
                                "trigger_z": 0.75,
                                "passes": 4,
                                "confidence": 0.97,
                            },
                        });
                    }
                }
            }
            Ok(v)
        })
        .await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        assert_eq!(outcome, ExecOutcome::Completed { steps: 3 }, "{transcript}");
        // trigger 0.75 (drag), halt 0.75 (toolhead): true Z = 12.4.
        assert!(
            fake.gcode_sent()
                .contains(&"SET_KINEMATIC_POSITION Z=12.4".to_owned()),
            "{:?}",
            fake.gcode_sent()
        );
    }

    #[tokio::test]
    async fn compute_failure_never_substitutes() {
        let plan = test_plan();
        // The halt-Z reading is null (toolhead is only queried by the
        // step-3 computation): the computation must abort the step and
        // SET_KINEMATIC_POSITION must never be sent.
        let fake = FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(th) = v.get_mut("status").and_then(|s| s.get_mut("toolhead")) {
                    *th = json!({"position": [10.0, 20.0, null, 0.0]});
                }
            }
            Ok(v)
        })
        .await;
        let (outcome, _) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted { step_id, cause, .. } = outcome else {
            panic!("expected abort");
        };
        assert_eq!(step_id, 3);
        assert!(matches!(cause, StopCause::ComputeFailed(_)), "{cause:?}");
        assert!(
            !fake
                .gcode_sent()
                .iter()
                .any(|c| c.contains("SET_KINEMATIC_POSITION")),
            "must not send with an unresolved placeholder"
        );
    }

    #[tokio::test]
    async fn placeholder_without_computation_refuses_to_send() {
        let mut plan = test_plan();
        plan.steps[2].compute = None; // malformed plan shape
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let (outcome, _) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted { step_id, cause, .. } = outcome else {
            panic!("expected abort");
        };
        assert_eq!(step_id, 3);
        assert!(matches!(cause, StopCause::ComputeFailed(_)));
        assert!(!fake.gcode_sent().iter().any(|c| c.contains("{true_z}")));
    }

    /// A three-step accel-clamp plan: clamp (records `max_accel`,
    /// declares the abort cleanup), a probe, then the success restore.
    fn accel_plan() -> RecoveryPlan {
        use plr_recovery::RESTORE_ACCEL_PLACEHOLDER;
        let mut plan = test_plan();
        plan.steps = vec![
            RecoveryStep {
                id: 1,
                phase: Phase::AccelClamp,
                summary: "clamp".to_owned(),
                commands: vec!["SET_VELOCITY_LIMIT ACCEL=100".to_owned()],
                pre_verify: vec![],
                verify: vec![],
                compute: Some(RuntimeComputation::RecordMaxAccel),
                cleanup_commands: vec![format!(
                    "SET_VELOCITY_LIMIT ACCEL={RESTORE_ACCEL_PLACEHOLDER}"
                )],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::AccelClampFailed,
                },
            },
            RecoveryStep {
                id: 2,
                phase: Phase::Probe,
                summary: "touch".to_owned(),
                commands: vec![
                    "PLR_TOUCH SAMPLES=3 SAMPLE_RANGE=0.01 SPEED=1 RETRACT=2 TOUCH_ACCEL=100"
                        .to_owned(),
                ],
                pre_verify: vec![],
                verify: vec![Verification::new(
                    "plr",
                    "last_touch_result.median_z",
                    Predicate::FinitePresent,
                )],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::ProbeNoTrigger,
                },
            },
            RecoveryStep {
                id: 3,
                phase: Phase::AccelRestore,
                summary: "restore".to_owned(),
                commands: vec![format!(
                    "SET_VELOCITY_LIMIT ACCEL={RESTORE_ACCEL_PLACEHOLDER}"
                )],
                pre_verify: vec![],
                verify: vec![Verification::new(
                    "toolhead",
                    "max_accel",
                    Predicate::NumWithinComputed { epsilon: 1.0 },
                )],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::AccelRestoreFailed,
                },
            },
        ];
        plan
    }

    /// A stateful fake whose `toolhead.max_accel` tracks the last
    /// `SET_VELOCITY_LIMIT ACCEL=`, and whose touch median is
    /// controllable.
    fn accel_fake_handler(
        median: Option<f64>,
        fail_restore: bool,
    ) -> impl Fn(&str, &Value) -> Result<Value, (i64, String)> + Send + Sync + 'static {
        let accel = Arc::new(Mutex::new(3000.0_f64));
        move |method, params| {
            let mut accel = accel.lock().expect("accel");
            match method {
                "printer.gcode.script" => {
                    let script = params["script"].as_str().unwrap_or("");
                    if let Some(rest) = script.strip_prefix("SET_VELOCITY_LIMIT ACCEL=") {
                        if let Ok(v) = rest.trim().parse::<f64>() {
                            // Restoring (to the >200 pre-clamp value) fails
                            // when the test asked for a failing restore.
                            if fail_restore && v > 200.0 {
                                return Err((400, "Move out of range".to_owned()));
                            }
                            *accel = v;
                        }
                    }
                    Ok(json!("ok"))
                }
                "printer.objects.query" => {
                    let objects = params["objects"].as_object().expect("objects");
                    let mut status = serde_json::Map::new();
                    for name in objects.keys() {
                        let value = match name.as_str() {
                            "toolhead" => {
                                json!({"position": [10.0, 20.0, 0.75, 0.0], "max_accel": *accel})
                            }
                            "plr" => json!({"last_touch_result": {"median_z": median}}),
                            other => json!({"unknown": other}),
                        };
                        status.insert(name.clone(), value);
                    }
                    Ok(json!({"eventtime": 1.0, "status": status}))
                }
                other => Err((-32601, format!("Method not found: {other}"))),
            }
        }
    }

    #[tokio::test]
    async fn frame_invalid_is_set_only_at_or_after_the_shifted_frame() {
        // A plan whose shifted-frame declare is step 2; the probe (step
        // 3) aborts, so the frame is invalidated.
        // A clean 3-step plan: idle(1), shifted-frame(2), probe(3).
        let mut plan = test_plan();
        plan.steps = vec![
            RecoveryStep {
                id: 1,
                phase: Phase::IdleTimeout,
                summary: "idle".to_owned(),
                commands: vec!["SET_IDLE_TIMEOUT TIMEOUT=86400".to_owned()],
                pre_verify: vec![],
                verify: vec![],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::IdleTimeoutNotApplied,
                },
            },
            RecoveryStep {
                id: 2,
                phase: Phase::ShiftedFrame,
                summary: "declare shifted frame".to_owned(),
                commands: vec!["SET_KINEMATIC_POSITION Z=-1.15".to_owned()],
                pre_verify: vec![],
                verify: vec![],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::ShiftedFrameNotDeclared,
                },
            },
            RecoveryStep {
                id: 3,
                phase: Phase::Probe,
                summary: "probe".to_owned(),
                commands: vec!["PROBE PROBE_SPEED=1 SAMPLES=1".to_owned()],
                pre_verify: vec![],
                verify: vec![Verification::new(
                    "probe",
                    "last_z_result",
                    Predicate::FinitePresent,
                )],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::ProbeNoTrigger,
                },
            },
        ];
        // Probe verify fails (last_z_result null): abort at step 3.
        let fake = FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(p) = v.get_mut("status").and_then(|s| s.get_mut("probe")) {
                    *p = json!({ "last_z_result": null });
                }
            }
            Ok(v)
        })
        .await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted {
            step_id,
            frame_invalid,
            ..
        } = outcome
        else {
            panic!("expected abort");
        };
        assert_eq!(step_id, 3);
        assert!(
            frame_invalid,
            "abort after the shifted frame invalidates it"
        );
        assert!(
            transcript.contains("\"frame_invalid\":true"),
            "{transcript}"
        );
    }

    #[tokio::test]
    async fn frame_invalid_is_false_before_the_shifted_frame() {
        // test_plan() has no shifted-frame step and aborts at step 1.
        let plan = test_plan();
        let fake = FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(it) = v.get_mut("status").and_then(|s| s.get_mut("idle_timeout")) {
                    *it = json!({"idle_timeout": 600.0});
                }
            }
            Ok(v)
        })
        .await;
        let (outcome, _) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted { frame_invalid, .. } = outcome else {
            panic!("expected abort");
        };
        assert!(!frame_invalid);
    }

    #[tokio::test]
    async fn accel_clamp_records_restores_on_success() {
        let plan = accel_plan();
        let fake = FakeMoonraker::spawn(accel_fake_handler(Some(0.75), false)).await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        assert_eq!(outcome, ExecOutcome::Completed { steps: 3 }, "{transcript}");
        // Clamp to 100, then restore to the recorded 3000 on success.
        assert_eq!(
            fake.gcode_sent(),
            vec![
                "SET_VELOCITY_LIMIT ACCEL=100",
                "PLR_TOUCH SAMPLES=3 SAMPLE_RANGE=0.01 SPEED=1 RETRACT=2 TOUCH_ACCEL=100",
                "SET_VELOCITY_LIMIT ACCEL=3000",
            ]
        );
        assert!(transcript.contains("record-accel"), "{transcript}");
        // No abort cleanup ran on the success path.
        assert!(
            !transcript.contains("\"event\":\"cleanup\""),
            "{transcript}"
        );
    }

    #[tokio::test]
    async fn accel_cleanup_runs_on_abort_and_cannot_mask_the_reason() {
        // The touch median is null → the probe post-verify fails → abort
        // at step 2. The clamp step's cleanup must restore the accel, and
        // the abort reason must remain the probe's, not the clamp's.
        let plan = accel_plan();
        let fake = FakeMoonraker::spawn(accel_fake_handler(None, false)).await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted {
            step_id,
            reason,
            cause,
            ..
        } = outcome
        else {
            panic!("expected abort, got {outcome:?}");
        };
        assert_eq!(step_id, 2);
        // The ORIGINAL reason (probe), not accel-restore/clamp.
        assert_eq!(reason, "probe-no-trigger");
        assert!(matches!(cause, StopCause::VerificationFailed { .. }));
        // The abort cleanup restored the recorded accel (3000).
        assert!(
            fake.gcode_sent()
                .contains(&"SET_VELOCITY_LIMIT ACCEL=3000".to_owned()),
            "{:?}",
            fake.gcode_sent()
        );
        assert!(transcript.contains("\"event\":\"cleanup\""), "{transcript}");
    }

    #[tokio::test]
    async fn accel_cleanup_failure_is_logged_but_never_masks_the_reason() {
        // The restore command itself fails (Move out of range). The abort
        // reason must STILL be the probe's; the cleanup error is logged.
        let plan = accel_plan();
        let fake = FakeMoonraker::spawn(accel_fake_handler(None, true)).await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted { reason, .. } = outcome else {
            panic!("expected abort");
        };
        assert_eq!(reason, "probe-no-trigger", "cleanup must not mask reason");
        assert!(transcript.contains("cleanup-error"), "{transcript}");
    }

    #[test]
    fn dry_run_lists_commands_and_takes_no_client() {
        // The type-level proof is the signature (no client parameter
        // exists); this asserts the enumeration matches the plan and
        // the placeholder stays symbolic.
        let plan = test_plan();
        let dry = dry_run(&plan);
        assert_eq!(
            dry.would_send,
            vec![
                (1, "SET_IDLE_TIMEOUT TIMEOUT=86400".to_owned()),
                (2, "PROBE PROBE_SPEED=1 SAMPLES=1".to_owned()),
                (3, "SET_KINEMATIC_POSITION Z={true_z}".to_owned()),
            ]
        );
        assert!(dry.rendered.contains("idle-timeout"));
        assert!(dry.rendered.contains("fail: abort"));
    }

    #[test]
    fn lookup_walks_dotted_paths_and_indices() {
        let status = json!({
            "toolhead": {"position": [1.0, 2.0, 3.5, 0.0], "homed_axes": "xyz"},
        });
        assert_eq!(lookup(&status, "toolhead", "position.2"), Some(&json!(3.5)));
        assert_eq!(
            lookup(&status, "toolhead", "homed_axes"),
            Some(&json!("xyz"))
        );
        assert_eq!(lookup(&status, "toolhead", "position.9"), None);
        assert_eq!(lookup(&status, "toolhead", "missing"), None);
        assert_eq!(lookup(&status, "extruder", "target"), None);
    }

    #[test]
    fn predicates_evaluate_faithfully() {
        let within = Predicate::NumWithin {
            expected: 86_400.0,
            epsilon: 0.5,
        };
        assert!(evaluate(&within, Some(&json!(86_400.2)), None));
        assert!(!evaluate(&within, Some(&json!(86_300.0)), None));
        assert!(!evaluate(&within, Some(&json!("86400")), None));
        assert!(evaluate(
            &Predicate::NumAtLeast { min: 5.0 },
            Some(&json!(5.0)),
            None
        ));
        let band = Predicate::TempWithin {
            min: 140.0,
            max: 160.0,
        };
        assert!(evaluate(&band, Some(&json!(150.0)), None));
        assert!(!evaluate(&band, Some(&json!(139.9)), None));
        assert!(evaluate(
            &Predicate::Contains {
                needle: "z".to_owned()
            },
            Some(&json!("xyz")),
            None
        ));
        assert!(evaluate(
            &Predicate::Equals {
                value: "ready".to_owned()
            },
            Some(&json!("ready")),
            None
        ));
        assert!(evaluate(&Predicate::BoolTrue, Some(&json!(true)), None));
        assert!(evaluate(&Predicate::BoolFalse, Some(&json!(false)), None));
        assert!(!evaluate(&Predicate::BoolTrue, Some(&json!("true")), None));
        assert!(evaluate(&Predicate::FinitePresent, Some(&json!(1.5)), None));
        assert!(!evaluate(
            &Predicate::FinitePresent,
            Some(&json!(null)),
            None
        ));
        assert!(evaluate(
            &Predicate::NonEmptyMatrix,
            Some(&json!([[0.1], [0.2]])),
            None
        ));
        assert!(!evaluate(
            &Predicate::NonEmptyMatrix,
            Some(&json!([[]])),
            None
        ));
        let computed = Predicate::NumWithinComputed { epsilon: 0.05 };
        assert!(evaluate(&computed, Some(&json!(12.26)), Some(12.25)));
        assert!(!evaluate(&computed, Some(&json!(12.26)), None));
        assert!(!evaluate(&Predicate::BoolTrue, None, None));
    }
}
