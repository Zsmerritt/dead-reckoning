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
//!
//! # Confirm-points (invariant 3, refined)
//!
//! Invariant 3 says a verification failure aborts, and it still does.
//! What this module adds is a *deliberate* stop: a [`ConfirmPoint`] —
//! execution suspended, an explanation reported, and a yes/no answer
//! awaited. Three features share the one mechanism:
//!
//! * a **[`plr_recovery::Tier::Confirmable`] diagnosis** raised by the
//!   pre-flight (today: the plan's own warnings) pauses instead of
//!   aborting outright;
//! * **`confirm_z_before_resume`** pauses after the
//!   [`Phase::ZConfirmStandoff`] step, reporting the believed Z and the
//!   arithmetic behind it;
//! * **`debug_confirm_each_step`** pauses before every step, reporting
//!   that step's commands and verifications.
//!
//! A pause is not a special execution state: it is one `await` on the
//! caller-supplied [`Confirmer`], bounded by the plan's
//! `confirm_timeout_s` (or [`ExecOptions::confirm_timeout`]), whose three
//! possible answers ([`ConfirmAnswer`]) are *continue*, *abort*, and
//! *timed out* — and the last two are the same abort path, so a pause can
//! never leave the machine anywhere an abort could not have left it.
//!
//! # 5. The Z frame is recorded before it is risked ([`FrameGuard`])
//!
//! [`ExecOutcome::Aborted::frame_invalid`] is a *report*. It is not the
//! interlock, and it must never be the only record that the Z frame was
//! put at risk, because producing it requires this function to return —
//! and a runtime drop, a SIGTERM, a panic, a SIGKILL or a second power
//! loss all prevent that while leaving the frame just as fabricated.
//!
//! So the interlock is armed through [`FrameGuard::arm`] immediately
//! before the shifted-frame declare is ISSUED, and arming is fail-closed:
//! if it cannot be persisted, execution refuses to issue the declare at
//! all. The invariant is therefore structural rather than procedural —
//! *if the frame was ever risked, the record of that exists, whatever
//! happened next.*
//!
//! The default [`AbortConfirmer`] answers "abort" to everything, which
//! is what preserves today's behaviour for non-interactive callers: a
//! Confirmable diagnosis aborts unless somebody asked to be asked.

use std::future::Future;
use std::pin::Pin;
use std::time::Duration;

use plr_recovery::{
    fmt_num, true_z_at_halt, Diagnose, Diagnosis, FailureAction, Phase, Predicate, RecoveryPlan,
    RecoveryStep, RuntimeComputation, Tier, TriggerSource, Verification, MACHINE_ACCEL_PLACEHOLDER,
    PARK_Z_PLACEHOLDER, RESTORE_ACCEL_PLACEHOLDER, TRUE_Z_PLACEHOLDER,
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
    /// Fallback deadline for a [`ConfirmPoint`], used when the plan
    /// carries no [`RecoveryPlan::confirm_timeout_s`] of its own (see
    /// [`DEFAULT_CONFIRM_TIMEOUT`]). The plan's value always wins: it is
    /// the operator's `[plr]` setting, and this is only the daemon's
    /// default.
    ///
    /// A blocking confirmer — the CLI's terminal prompt — cannot be
    /// pre-empted by either: it holds its thread, so its bound is the
    /// operator standing in front of it. The bound exists for the
    /// control socket, where the client that asked to be consulted may
    /// simply go away.
    pub confirm_timeout: Duration,
    /// Fallback budget for the g-code mutex barrier, used when the plan
    /// carries no [`RecoveryPlan::gcode_barrier_timeout_s`] of its own.
    /// The plan's value always wins: it is the operator's `[plr]` setting,
    /// and this is only the daemon's default.
    ///
    /// See [`crate::recover`]'s gate 4b. A source that holds the mutex
    /// longer than this while a recovery is being asked for is doing work
    /// that must not overlap one, so the timeout is a refusal and not a
    /// retry.
    pub gcode_barrier_timeout: Duration,
}

/// Default [`ExecOptions::confirm_timeout`].
///
/// Long enough that an operator can walk to the printer, look at the
/// nozzle, and walk back — which is exactly what a Z confirmation asks
/// them to do — and short enough that an abandoned pause resolves itself
/// the same day. Timing out is not a failure mode to be avoided at all
/// costs: it aborts cleanly, and an abort is the safe direction.
///
/// **Derived, not restated.** This is literally
/// [`plr_recovery::CONFIRM_TIMEOUT_DEFAULT`], which is also what
/// `plr-recovery` quotes to the operator as the default of the `[plr]`
/// `confirm_timeout_s` key. The two used to be separate literals in
/// separate units (`Duration::from_mins(10)` here, `600.0` there); a
/// config default that restates the value another component enforces is a
/// divergence waiting to happen, and one nobody could diagnose from the
/// operator's side.
pub const DEFAULT_CONFIRM_TIMEOUT: Duration = plr_recovery::CONFIRM_TIMEOUT_DEFAULT;

impl Default for ExecOptions {
    fn default() -> Self {
        Self {
            // Position/state predicates settle as soon as the script
            // returns; 10 s absorbs status-refresh lag.
            verify_timeout: Duration::from_secs(10),
            // Heating a bed from cold legitimately takes minutes.
            temp_timeout: Duration::from_mins(15),
            poll_interval: Duration::from_millis(500),
            confirm_timeout: DEFAULT_CONFIRM_TIMEOUT,
            gcode_barrier_timeout: DEFAULT_GCODE_BARRIER_TIMEOUT,
        }
    }
}

/// Default [`ExecOptions::gcode_barrier_timeout`].
///
/// **Derived, not restated** — the same single-definition rule as
/// [`DEFAULT_CONFIRM_TIMEOUT`]. `plr-recovery` owns the value because it
/// is what the `[plr] gcode_barrier_timeout_s` diagnosis quotes back to
/// the operator.
pub const DEFAULT_GCODE_BARRIER_TIMEOUT: Duration = plr_recovery::GCODE_BARRIER_TIMEOUT_DEFAULT;

/// Which of the three confirm-point features raised this pause.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfirmKind {
    /// A [`Tier::Confirmable`] diagnosis from the pre-flight.
    Diagnosis,
    /// The `confirm_z_before_resume` Z-height confirmation.
    ZHeight,
    /// The `debug_confirm_each_step` per-step pause.
    StepDebug,
    /// The interactive resume-point preview reposition pause (design §D):
    /// the toolhead is hovering over a candidate resume point and the
    /// operator answers with a [`PreviewAnswer`] (accept / next / prev /
    /// nudge / abort), NOT the binary continue/abort.
    Preview,
}

impl ConfirmKind {
    /// Stable tag carried in the transcript and the socket response.
    #[must_use]
    pub fn tag(self) -> &'static str {
        match self {
            ConfirmKind::Diagnosis => "diagnosis",
            ConfirmKind::ZHeight => "z-height",
            ConfirmKind::StepDebug => "step-debug",
            ConfirmKind::Preview => "preview",
        }
    }
}

/// The answer to a resume-preview reposition pause (design §D.1).
///
/// Deliberately a SEPARATE type from [`ConfirmAnswer`]: the binary `ask`
/// stays binary (continue/abort), and preview's richer vocabulary — jump
/// to the next/previous representative, nudge ±1/±10 stops along the
/// toolpath, accept the current point — never leaks into it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreviewAnswer {
    /// Commit the current stop as the resume point.
    Accept,
    /// Jump the cursor to the next representative.
    NextRep,
    /// Jump the cursor to the previous representative.
    PrevRep,
    /// Nudge the cursor by this many stops along the toolpath (±1 fine,
    /// ±10 coarse); clamped to the stop-list bounds.
    Nudge(i32),
    /// Stop; abort the recovery.
    Abort,
    /// Nobody answered within the deadline. Treated exactly as
    /// [`Self::Abort`], distinguished only for the transcript.
    TimedOut,
}

impl PreviewAnswer {
    /// Stable tag carried in the transcript.
    #[must_use]
    pub fn tag(self) -> &'static str {
        match self {
            PreviewAnswer::Accept => "accept",
            PreviewAnswer::NextRep => "next",
            PreviewAnswer::PrevRep => "prev",
            PreviewAnswer::Nudge(_) => "nudge",
            PreviewAnswer::Abort => "abort",
            PreviewAnswer::TimedOut => "timeout",
        }
    }
}

/// Everything a confirmer needs to put the question to a human.
#[derive(Debug, Clone)]
pub struct ConfirmPoint {
    /// Which feature raised it.
    pub kind: ConfirmKind,
    /// The step this pause is anchored to. Every pause has one, even the
    /// pre-flight's (which anchors on step 1, before anything is sent):
    /// the anchor is what makes the abort path — and therefore the
    /// frame-invalidation rule — identical to any other abort.
    pub step_id: u32,
    /// The anchor step's phase name.
    pub phase: String,
    /// The explanation, in the one shape every diagnosis takes.
    pub diagnosis: Diagnosis,
    /// Feature-specific evidence: the step's commands and verifications
    /// for a step-debug pause, the believed Z and its derivation for a
    /// Z-height pause.
    pub detail: Value,
    /// **The deadline this pause will actually be enforced against**:
    /// the operator's `[plr]` `confirm_timeout_s` when the plan carries
    /// one, else [`ExecOptions::confirm_timeout`].
    ///
    /// Stamped by [`ask`], which is also the only thing that starts the
    /// timer — and it starts it *from this field*. A [`Confirmer`] that
    /// reports this number to a client is therefore reporting the number
    /// the executor enforces, not a default re-derived at the call site
    /// (which is how the control socket's clients ended up assuming the
    /// band ceiling and believing a lapsed confirmation was still live).
    ///
    /// A value never stamped is [`Duration::ZERO`], i.e. "already
    /// expired" — the fail-closed direction, because a deadline nobody
    /// set must not read as a deadline far in the future.
    pub deadline: Duration,
}

/// The answer to a [`ConfirmPoint`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfirmAnswer {
    /// Proceed anyway.
    Continue,
    /// Stop; abort the recovery.
    Abort,
    /// Nobody answered within [`ExecOptions::confirm_timeout`]. Treated
    /// exactly as [`ConfirmAnswer::Abort`], and distinguished only so the
    /// transcript records which one it was.
    TimedOut,
}

impl ConfirmAnswer {
    /// Stable tag carried in the transcript.
    #[must_use]
    pub fn tag(self) -> &'static str {
        match self {
            ConfirmAnswer::Continue => "continue",
            ConfirmAnswer::Abort => "abort",
            ConfirmAnswer::TimedOut => "timeout",
        }
    }
}

/// How execution asks a human whether to proceed.
///
/// Boxed-future rather than `async fn` in trait: the control socket
/// drives execution from a spawned task, so the whole execution future
/// must be `Send`, and this shape says so at the type level without
/// pulling in an async-trait dependency.
pub trait Confirmer: Send {
    /// Puts `point` to the operator and returns their answer.
    fn confirm<'a>(
        &'a mut self,
        point: &'a ConfirmPoint,
    ) -> Pin<Box<dyn Future<Output = ConfirmAnswer> + Send + 'a>>;

    /// Puts a resume-preview reposition `point` to the operator and returns
    /// their [`PreviewAnswer`] (design §D). The DEFAULT refuses with
    /// [`PreviewAnswer::Abort`]: a confirmer that does not implement the
    /// interactive protocol (the CLI's [`AbortConfirmer`], every test fake)
    /// declines rather than committing a resume point nobody chose — the
    /// same fail-closed direction `confirm`'s non-interactive callers take.
    /// Only [`crate::ctrlsock::SocketConfirmer`] overrides it to drive the
    /// real accept/next/prev/nudge loop.
    fn confirm_preview<'a>(
        &'a mut self,
        _point: &'a ConfirmPoint,
    ) -> Pin<Box<dyn Future<Output = PreviewAnswer> + Send + 'a>> {
        Box::pin(async { PreviewAnswer::Abort })
    }
}

/// Materialises and writes the generated recovery file for a resume point
/// chosen at execute time (design §4, the late-binding seam).
///
/// Under `resume_candidate_policy = ask` the resume point is not known
/// until the operator accepts in the preview loop, so the file cannot be
/// pre-generated at dry-run. The executor calls [`Self::write_for`] with
/// the accepted stop's binding, THEN proceeds to the `M23`/`M24` steps that
/// select the now-materialised file.
///
/// Dry-run is handed a [`NoopFileWriter`], which HOLDS NO filesystem
/// capability — so "dry-run cannot write the recovery file" is a type fact,
/// not a discipline (design §4 / §10 attack #5, the twelfth corollary):
/// the writer the dry-run path owns simply cannot write.
pub trait RecoveryFileWriter: Send {
    /// Generate the recovery file for `binding` (the accepted stop's tail
    /// offset + entry moves) and write it. `Err(reason)` aborts the
    /// recovery through the ordinary path (frame invalidated — the pause
    /// sits past the shifted-frame declare).
    fn write_for(&mut self, binding: &plr_recovery::PreviewBinding) -> Result<(), String>;
}

/// A [`RecoveryFileWriter`] that writes nothing — the dry-run path's writer.
///
/// It carries no path, no bytes, no filesystem handle: the dry-run code
/// literally cannot materialise a file through it. This is the type-level
/// enforcement of "dry-run never writes" (design §4).
pub struct NoopFileWriter;

impl RecoveryFileWriter for NoopFileWriter {
    fn write_for(&mut self, _binding: &plr_recovery::PreviewBinding) -> Result<(), String> {
        // Intentionally does nothing: a dry-run has no file to write.
        Ok(())
    }
}

/// Re-establishes exclusive control of Klipper's g-code channel at every
/// point where execution is about to send something after a gap it does not
/// control.
///
/// # Why this is a per-step obligation and not a one-time gate
///
/// The caller's pre-execution gate (`recover`'s gates 4 and 4b) is a
/// *sample*, and every gap after it is somebody else's opportunity. The
/// widest of those gaps is inside this module: `preflight_confirmations`
/// and the `debug_confirm_each_step` pause both run **before any command is
/// issued**, and both can block for the operator's whole
/// `confirm_timeout_s` — up to an hour at the top of the permitted band, or
/// unbounded human time at a CLI `--step` prompt. During it an autostart
/// macro or a queued job can begin printing, and execution would then
/// answer "continue" by issuing `SET_KINEMATIC_POSITION` and `PROBE` into a
/// running print and reporting `COMPLETED`.
///
/// So the check is re-run once per step, immediately before that step's
/// commands go out, which covers all three gaps with one call site: the gap
/// since the caller's gate, the pre-flight pause, and any per-step pause or
/// gate. `Err(reason)` aborts through the ordinary abort path, so the
/// frame-invalidation rule applies exactly as it would to a verification
/// failure at the same step.
///
/// The residual is one Moonraker round trip wide: another g-code source can
/// still take the mutex between the check and the send. That is not
/// closable from outside Klipper — but it is *microseconds*, not the
/// operator's coffee break, and the difference between those two is the
/// whole point of this trait.
pub trait Exclusivity: Send {
    /// Re-asserts exclusive g-code access. `Err(reason)` aborts.
    fn recheck<'a>(
        &'a mut self,
        client: &'a mut MoonrakerClient,
        budget: Duration,
    ) -> Pin<Box<dyn Future<Output = Result<(), String>> + Send + 'a>>;
}

/// An [`Exclusivity`] that checks nothing.
///
/// For tests that drive [`execute`] against a hand-built plan and a static
/// fake, where there is no second g-code source to detect. Named so that
/// choosing it is visible: production callers pass `recover`'s real one.
/// `dead_code` is allowed because plrd is a binary crate, so `pub` alone
/// does not count as a use.
#[cfg_attr(not(test), allow(dead_code))]
pub struct NoExclusivity;

impl Exclusivity for NoExclusivity {
    fn recheck<'a>(
        &'a mut self,
        _client: &'a mut MoonrakerClient,
        _budget: Duration,
    ) -> Pin<Box<dyn Future<Output = Result<(), String>> + Send + 'a>> {
        Box::pin(async { Ok(()) })
    }
}

/// The default, non-interactive confirmer: every confirm-point aborts.
///
/// This is what preserves today's behaviour for callers that never asked
/// to be consulted — a Confirmable diagnosis stops the recovery rather
/// than silently proceeding, and there is nobody to ask.
pub struct AbortConfirmer;

impl Confirmer for AbortConfirmer {
    fn confirm<'a>(
        &'a mut self,
        _point: &'a ConfirmPoint,
    ) -> Pin<Box<dyn Future<Output = ConfirmAnswer> + Send + 'a>> {
        Box::pin(async { ConfirmAnswer::Abort })
    }
}

/// The exact Klipper error string a probe-triggered-early failure
/// carries (Cartographer keys off this literal,
/// `adapters/klipper_like/utils.py:63-84`,
/// `PROBE_TRIGGERED_BEFORE_MOVEMENT`).
pub const PROBE_TRIGGERED_EARLY: &str = "Probe triggered prior to movement";

/// The exact Klipper error string for a probe that never triggered.
pub const NO_TRIGGER_FULL_MOVEMENT: &str = "No trigger on probe after full movement";

/// A distinctive, stable substring of the `PLR_TOUCH` consensus-failure
/// message (the plugin's `consensus_failure_text`,
/// `klippy_plugin/plr/touch_sequence.py`, the `consensus_failure_text`
/// "... failed: could not find N touches within ... in a sliding window
/// of ..."): the multi-touch
/// sequence could not assemble an agreeing subset. Treated as a
/// no-trigger. Matched as a substring (`contains`), not a prefix, so
/// klippy's own command-error wrapping does not defeat it.
pub const TOUCH_CONSENSUS_FAILURE_MARKER: &str = "in a sliding window of";

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
            || message.contains(TOUCH_CONSENSUS_FAILURE_MARKER)
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

/// Every classified Klipper command failure explains itself.
///
/// Exhaustive with no catch-all arm, like every other [`Diagnose`]
/// implementation: a new [`StepFailure`] variant fails to compile until
/// its diagnosis is written. All four are [`Tier::Hard`] — these are
/// things that went wrong *while moving*, next to the part, and none of
/// them is a state anybody should be offered a "continue anyway" button
/// for.
impl Diagnose for StepFailure {
    fn diagnosis(&self) -> Diagnosis {
        match self {
            StepFailure::ProbeTriggeredEarly => Diagnosis::new(
                "probe_triggered_early",
                Tier::Hard,
                "the probe reported a trigger before the descent had moved",
                "Klipper saw the probe already in contact when the move started. Either \
                 the nozzle was resting on the part (the believed Z was too low), or the \
                 probe is stuck triggered. Either way the reading that would come out of \
                 this is not a measurement of anything, and the true-Z arithmetic built on \
                 it would put the frame in the wrong place for the rest of the print.",
                "Check the nozzle is not touching the part and that the probe reads \
                 untriggered at rest (`QUERY_PROBE`). On a Tap-style probe this usually \
                 means debris or a filament blob on the nozzle — clean it, then re-run a \
                 fresh dry run before retrying."
                    .to_owned(),
            ),
            StepFailure::NoTrigger => Diagnosis::new(
                "probe_no_trigger",
                Tier::Hard,
                "the probe descended the full envelope without ever contacting the part",
                "The envelope is deliberately bounded, so this is the safe failure: the \
                 nozzle stopped short rather than pressing on indefinitely, and the part \
                 was never touched. But it also means Z is still unknown — the one thing \
                 the whole recovery exists to establish. On the consensus-touch path the \
                 same code covers touches that could not agree with each other, which is \
                 the same problem with more evidence.",
                "The part is likely lower than the reconstruction believed (bed sag, or a \
                 Z estimate biased high). Raise `envelope_margin` in printer.cfg's [plr] \
                 section by a few tenths and re-run a fresh dry run. If the touches simply \
                 disagreed, raise `touch_sample_range` toward its 0.015 cap, or reduce \
                 machine vibration."
                    .to_owned(),
            ),
            StepFailure::MoveOutOfRange => Diagnosis::new(
                "move_out_of_range",
                Tier::Hard,
                "Klipper refused a commanded move as outside the machine's limits",
                "Klipper does not clamp an out-of-range move, it refuses it — which is \
                 the backstop working. It also means the plan's arithmetic and the \
                 machine's rail limits disagree, and until they are reconciled every other \
                 coordinate in the plan is equally suspect.",
                "Check that the Z rail's `position_min`/`position_max` and the XY travel \
                 limits in printer.cfg match the physical machine, then re-run a fresh dry \
                 run: the whole-itinerary pre-flight checks every coordinate against those \
                 limits before anything is sent."
                    .to_owned(),
            ),
            StepFailure::Unknown => Diagnosis::new(
                "klipper_command_error",
                Tier::Hard,
                "Klipper rejected a command with an error plrd does not recognize",
                "plrd never continues past a command it could not confirm succeeded. \
                 Because the error is unrecognized, plrd cannot say whether the command \
                 partly took effect — so the only defensible assumption is that the \
                 machine state is no longer what the plan believes.",
                "Read the raw Klipper message in the transcript (and klippy.log) — it \
                 names the real problem. Fix that, then re-run a fresh dry run before \
                 retrying."
                    .to_owned(),
            ),
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
    /// The frame-invalidation interlock could not be written, so
    /// execution refused to ISSUE the shifted-frame declare. Nothing was
    /// declared, so the Z frame is still valid — this is the fail-closed
    /// direction: never enter a state we cannot record that we are in.
    FrameGuardUnwritable(String),
    /// A [`Tier::Hard`] diagnosis was raised by the pre-flight. A
    /// refusal is not a question, so it aborts instead of being offered
    /// a continue-anyway button.
    HardDiagnosis {
        /// The refusing diagnosis's code.
        code: &'static str,
    },
    /// A [`ConfirmPoint`] was answered "abort" — or was never asked at
    /// all, because the caller supplied the default [`AbortConfirmer`].
    ConfirmationDeclined {
        /// Which feature raised the pause.
        kind: ConfirmKind,
        /// The diagnosis code that was put to the operator.
        code: &'static str,
    },
    /// A [`ConfirmPoint`] went unanswered for
    /// [`ExecOptions::confirm_timeout`]. Aborts on exactly the same path
    /// as [`StopCause::ConfirmationDeclined`]; the distinction is only
    /// for the transcript.
    ConfirmationTimedOut {
        /// Which feature raised the pause.
        kind: ConfirmKind,
        /// The diagnosis code that went unanswered.
        code: &'static str,
    },
    /// Exclusive control of Klipper's g-code channel could not be
    /// re-established before this step's commands went out — the printer
    /// stopped being idle, or another source is holding the g-code mutex
    /// (see [`Exclusivity`]). Nothing from this step was sent.
    ExclusivityLost(String),
}

impl StopCause {
    /// The typed step failure, when this stop was a classified command
    /// error.
    #[must_use]
    pub fn step_failure(&self) -> Option<StepFailure> {
        // Enumerated, not `_ => None`: a new StopCause must state whether
        // it carries a classified Klipper failure rather than inheriting
        // "no" from a catch-all nobody revisits.
        match self {
            StopCause::CommandFailed { failure, .. } => Some(*failure),
            StopCause::VerificationFailed { .. }
            | StopCause::Transport(_)
            | StopCause::ComputeFailed(_)
            | StopCause::OperatorDeclined
            | StopCause::FrameGuardUnwritable(_)
            | StopCause::HardDiagnosis { .. }
            | StopCause::ConfirmationDeclined { .. }
            | StopCause::ConfirmationTimedOut { .. }
            | StopCause::ExclusivityLost(_) => None,
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

/// The two acceleration values execution carries across steps.
///
/// Two slots, not one: see [`MACHINE_ACCEL_PLACEHOLDER`] for why
/// collapsing them leaves the machine at a recovery acceleration after a
/// successful resume.
#[derive(Debug, Clone, Copy, Default)]
struct AccelSlots {
    /// The value in force immediately before the touch clamp, recorded
    /// by [`RuntimeComputation::RecordMaxAccel`]; substituted for
    /// `{restore_accel}`.
    phase: Option<f64>,
    /// The machine's own value, recorded once by
    /// [`RuntimeComputation::RecordMachineAccel`] before the plan
    /// changes anything; substituted for `{machine_accel}`.
    machine: Option<f64>,
}

/// Records, BEFORE the shifted-frame declare is issued, that Klipper's Z
/// frame is about to become potentially fabricated.
///
/// # Why this is armed on entry rather than written on abort
///
/// The Z frame stops being trustworthy the instant the **shifted-frame**
/// `SET_KINEMATIC_POSITION` is *issued* — not when some later code
/// decides an abort has happened. Anything that writes the interlock on
/// the abort path is a code path that has to RUN, and the ways it can
/// fail to run are exactly the ways this daemon dies: a runtime drop on
/// `systemctl restart`, a SIGTERM, a panic in the execution task, a
/// SIGKILL, or a second power loss. Every one of those would leave the
/// frame fabricated and the interlock absent, and the next `--execute`
/// would re-drive the plan against it.
///
/// # Why the believed-Z declare is deliberately outside the armed zone
///
/// [`Phase::BelievedZDeclare`] also issues a `SET_KINEMATIC_POSITION`,
/// two phases earlier, and is NOT guarded. That is not an oversight: it
/// declares the CONSERVATIVE believed Z (the upper bound of the
/// possible-stop set) and then LIFTS, so every motion it enables moves
/// away from the part, XY homing after it never touches Z, and the
/// shifted-frame step re-declares Z absolutely rather than building on
/// it. Re-running it after an interrupted attempt biases the subsequent
/// probe toward a `NoTrigger` — the bounded, safe failure — rather than
/// toward a collision. The shifted frame is where Z first becomes a
/// number nobody can re-derive, so that is where the interlock belongs.
///
/// So the marker is written on ENTRY to the danger zone and cleared only
/// on a fully successful completion. Then it persists by construction:
/// nothing has to run for it to survive.
///
/// Arming is **fail-closed**. `Err` aborts the recovery *before* the
/// declare is issued, because a state we cannot record that we are in is
/// a state we must not enter.
pub trait FrameGuard: Send {
    /// Arms the interlock for `step` (the shifted-frame declare).
    ///
    /// # Errors
    ///
    /// The reason the interlock could not be persisted. Execution then
    /// refuses to issue the declare.
    fn arm(&mut self, step: &RecoveryStep) -> Result<(), String>;
}

/// A [`FrameGuard`] that records nothing.
///
/// For tests and for callers that own the interlock themselves. Named
/// unmistakably: choosing it means choosing to have no interlock, and
/// that should never be a quiet default — which is also why it is not
/// the default anywhere in production. `dead_code` is allowed because
/// plrd is a binary crate, so `pub` alone does not count as a use and
/// only the test harness constructs this.
#[cfg_attr(not(test), allow(dead_code))]
pub struct NoFrameGuard;

impl FrameGuard for NoFrameGuard {
    fn arm(&mut self, _step: &RecoveryStep) -> Result<(), String> {
        Ok(())
    }
}

/// Executes a validated plan step by step (module safety invariants
/// 2–4).
///
/// `gate` is consulted before every step (the CLI's `--step` mode);
/// returning `false` stops execution before that step sends anything.
/// `confirmer` answers every [`ConfirmPoint`] — pass
/// [`AbortConfirmer`] for the non-interactive behaviour in which any
/// Confirmable diagnosis aborts. `frame_guard` is armed immediately
/// before the shifted-frame declare and refuses entry if it cannot
/// persist (see [`FrameGuard`]). `exclusivity` is re-asserted before every
/// step's commands go out (see [`Exclusivity`] for why once is not enough).
#[allow(clippy::too_many_arguments)] // one collaborator per safety invariant
#[allow(clippy::too_many_lines)] // linear step loop + the two post-step hooks
pub async fn execute(
    plan: &RecoveryPlan,
    client: &mut MoonrakerClient,
    options: &ExecOptions,
    gate: &mut (dyn FnMut(&RecoveryStep) -> bool + Send),
    confirmer: &mut dyn Confirmer,
    frame_guard: &mut dyn FrameGuard,
    exclusivity: &mut dyn Exclusivity,
    writer: &mut dyn RecoveryFileWriter,
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
    // Execution state that persists across steps: the two accel slots
    // and the registered abort cleanups.
    let mut accel = AccelSlots::default();
    let mut cleanups: Vec<(u32, Vec<String>)> = Vec::new();

    // Pre-flight: every plan warning is a diagnosis, and a Confirmable
    // one pauses here — BEFORE step 1, so declining costs nothing and
    // sends nothing.
    let Some(anchor) = plan.steps.first() else {
        transcript.entry(&json!({"event": "plan-complete", "steps": 0}));
        return ExecOutcome::Completed { steps: 0 };
    };
    // The operator's `[plr]` confirm_timeout_s wins over the daemon's
    // default; a plan that carries none keeps the default (which is what
    // tests shrink).
    let confirm_deadline = plan_duration_or(plan.confirm_timeout_s, options.confirm_timeout);
    let barrier_budget =
        plan_duration_or(plan.gcode_barrier_timeout_s, options.gcode_barrier_timeout);
    if let Some(cause) =
        preflight_confirmations(plan, anchor, confirm_deadline, confirmer, transcript).await
    {
        return finish_abort(
            client, anchor, cause, &cleanups, accel, shifted_id, transcript,
        )
        .await;
    }

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
                accel,
                shifted_id,
                transcript,
            )
            .await;
        }
        // `debug_confirm_each_step`: pause BEFORE the step's commands, so
        // the operator sees exactly what is about to be sent and what
        // will be checked afterwards.
        if plan.debug_confirm_each_step {
            let point = step_debug_point(step);
            if let Some(cause) = ask(point, confirm_deadline, confirmer, transcript).await {
                return finish_abort(
                    client, step, cause, &cleanups, accel, shifted_id, transcript,
                )
                .await;
            }
        }
        // LAST thing before anything is sent: re-assert exclusive g-code
        // access. Placed here, after the `--step` gate and after the
        // step-debug pause, one call site covers every gap this loop can
        // contain — including the pre-flight pause before step 1, which is
        // bounded only by `confirm_timeout_s`. See `Exclusivity`.
        //
        // It cannot refuse this recovery's own progress: the only command
        // in any plan that makes the printer non-idle is the `M24` in
        // `Phase::RecoveryFileSelect`, which is property-tested to be the
        // FINAL step (`RecoveryPlan::recovery_file_select_last`,
        // `plr-recovery/tests/properties.rs`), so no step is ever reached
        // with a print this recovery itself started.
        if let Some(cause) =
            reassert_exclusivity(step, exclusivity, client, barrier_budget, transcript).await
        {
            return finish_abort(
                client, step, cause, &cleanups, accel, shifted_id, transcript,
            )
            .await;
        }
        // Entry to the danger zone: arm the interlock BEFORE the declare
        // is issued, and refuse to issue it if that cannot be persisted.
        if let Some(cause) = arm_frame_if_entering(step, shifted_id, frame_guard, transcript) {
            return finish_abort(
                client, step, cause, &cleanups, accel, shifted_id, transcript,
            )
            .await;
        }
        transcript.entry(&json!({
            "event": "step-start",
            "step": step.id,
            "phase": step.phase.name(),
            "summary": step.summary,
        }));
        let computed =
            match run_step(client, step, options, &mut accel, &mut cleanups, transcript).await {
                Ok(computed) => computed,
                Err(cause) => {
                    return finish_abort(
                        client, step, cause, &cleanups, accel, shifted_id, transcript,
                    )
                    .await;
                }
            };
        transcript.entry(&json!({"event": "step-ok", "step": step.id}));
        // `confirm_z_before_resume`: pause AFTER the standoff lift has
        // run and verified, so the reported Z is a settled readback and
        // the nozzle is already where the operator is being asked to look.
        if step.phase == Phase::ZConfirmStandoff {
            let point = z_confirm_point(client, plan, step, computed).await;
            if let Some(cause) = ask(point, confirm_deadline, confirmer, transcript).await {
                return finish_abort(
                    client, step, cause, &cleanups, accel, shifted_id, transcript,
                )
                .await;
            }
        }
        // The resume-preview reposition loop runs AFTER the hover lift +
        // cool-down step has run and verified (design §D.2). The nozzle is
        // at the single hover plane; the loop only ever moves in XY there.
        // Accept materialises the recovery file for the chosen stop (via
        // `writer`) then falls through to the M23/M24 steps; abort/timeout
        // aborts with the Z frame invalidated (the step sits past
        // ShiftedFrame). A plan without a preview skips this entirely.
        if step.phase == Phase::ResumePreview {
            if let Some(cause) = run_preview_loop(
                plan,
                client,
                options,
                confirmer,
                exclusivity,
                writer,
                barrier_budget,
                confirm_deadline,
                step,
                transcript,
            )
            .await
            {
                return finish_abort(
                    client, step, cause, &cleanups, accel, shifted_id, transcript,
                )
                .await;
            }
        }
    }
    transcript.entry(&json!({"event": "plan-complete", "steps": plan.steps.len()}));
    ExecOutcome::Completed {
        steps: plan.steps.len(),
    }
}

/// Resolves one `[plr]`-configurable duration: the operator's value from
/// the plan when it carries a usable one, else the daemon's fallback.
///
/// One function for both the confirm deadline and the barrier budget, so
/// "the operator's setting wins" cannot mean two different things in two
/// places — and so the fail-safe treatment of nonsense is written once.
/// Non-finite and non-positive values fall back rather than being honoured:
/// a zero deadline would abort every pause and a zero barrier budget would
/// refuse every recovery, and while both are the safe *direction* they are
/// not what an operator who typed a bad number meant.
#[must_use]
pub fn plan_duration_or(plan_value: Option<f64>, fallback: Duration) -> Duration {
    plan_value
        .filter(|s| s.is_finite() && *s > 0.0)
        .map_or(fallback, Duration::from_secs_f64)
}

/// Re-asserts exclusive g-code access before `step` sends anything.
///
/// `Some(cause)` means somebody else has the printer and the step must NOT
/// be issued — the fail-closed direction, and the reason this is the last
/// thing to happen before any command goes out (see [`Exclusivity`]).
async fn reassert_exclusivity(
    step: &RecoveryStep,
    exclusivity: &mut dyn Exclusivity,
    client: &mut MoonrakerClient,
    budget: Duration,
    transcript: &mut Transcript<'_>,
) -> Option<StopCause> {
    match exclusivity.recheck(client, budget).await {
        Ok(()) => None,
        Err(reason) => {
            transcript.entry(&json!({
                "event": "exclusivity-lost",
                "step": step.id,
                "phase": step.phase.name(),
                "reason": reason,
            }));
            Some(StopCause::ExclusivityLost(reason))
        }
    }
}

/// Arms the frame interlock when `step` is the shifted-frame declare.
///
/// `Some(cause)` means the interlock could not be persisted and the
/// declare must NOT be issued — the fail-closed direction, because a
/// state we cannot record that we are in is a state we must not enter.
fn arm_frame_if_entering(
    step: &RecoveryStep,
    shifted_id: Option<u32>,
    frame_guard: &mut dyn FrameGuard,
    transcript: &mut Transcript<'_>,
) -> Option<StopCause> {
    if Some(step.id) != shifted_id {
        return None;
    }
    match frame_guard.arm(step) {
        Ok(()) => {
            transcript.entry(&json!({"event": "frame-armed", "step": step.id}));
            None
        }
        Err(reason) => {
            transcript.entry(&json!({
                "event": "frame-arm-failed",
                "step": step.id,
                "reason": reason,
            }));
            Some(StopCause::FrameGuardUnwritable(reason))
        }
    }
}

/// Turns every plan warning into a diagnosis, transcribes it, and pauses
/// on the Confirmable ones. `Some(cause)` means the recovery must abort.
async fn preflight_confirmations(
    plan: &RecoveryPlan,
    anchor: &RecoveryStep,
    deadline: Duration,
    confirmer: &mut dyn Confirmer,
    transcript: &mut Transcript<'_>,
) -> Option<StopCause> {
    for warning in &plan.warnings {
        let diagnosis = warning.diagnosis();
        match diagnosis.tier {
            // Proceeds by default — but loudly, and in the same shape
            // every other diagnosis takes.
            Tier::Advisory => transcript.entry(&json!({
                "event": "advisory",
                "diagnosis": diagnosis,
            })),
            Tier::Confirmable => {
                let point = ConfirmPoint {
                    kind: ConfirmKind::Diagnosis,
                    step_id: anchor.id,
                    phase: anchor.phase.name().to_owned(),
                    diagnosis,
                    detail: json!({"raised_by": "pre-flight"}),
                    deadline: Duration::ZERO,
                };
                if let Some(cause) = ask(point, deadline, confirmer, transcript).await {
                    return Some(cause);
                }
            }
            // A refusal is not a question. No `PlanWarning` produces a
            // Hard diagnosis today and a test pins that, but routing one
            // into the ask path would offer a continue-anyway button for
            // something whose whole definition is "only a deliberate
            // printer.cfg edit may permit this" — so it is unreachable by
            // construction here, not merely by convention elsewhere.
            Tier::Hard => {
                transcript.entry(&json!({
                    "event": "hard-refusal",
                    "diagnosis": diagnosis,
                }));
                return Some(StopCause::HardDiagnosis {
                    code: diagnosis.code,
                });
            }
        }
    }
    None
}

/// Puts one confirm-point to the confirmer, bounded by the deadline the
/// caller resolved, and records the question and the answer in the
/// transcript. `None` means "continue".
///
/// `deadline` is stamped onto [`ConfirmPoint::deadline`] *before* the
/// timer is created, and the timer is created from that field. So the
/// bound a confirmer can read and report is the same value, read twice —
/// there is no second expression anywhere that could drift from it. The
/// transcript records it too, so an audit after the fact can see which
/// deadline a given pause was held to.
async fn ask(
    mut point: ConfirmPoint,
    deadline: Duration,
    confirmer: &mut dyn Confirmer,
    transcript: &mut Transcript<'_>,
) -> Option<StopCause> {
    point.deadline = deadline;
    transcript.entry(&json!({
        "event": "confirm-pause",
        "kind": point.kind.tag(),
        "step": point.step_id,
        "phase": point.phase,
        "diagnosis": point.diagnosis,
        "detail": point.detail,
        "deadline_s": point.deadline.as_secs_f64(),
    }));
    let bound = point.deadline;
    let answer = tokio::time::timeout(bound, confirmer.confirm(&point))
        .await
        .unwrap_or(ConfirmAnswer::TimedOut);
    transcript.entry(&json!({
        "event": "confirm-answer",
        "kind": point.kind.tag(),
        "step": point.step_id,
        "code": point.diagnosis.code,
        "answer": answer.tag(),
    }));
    match answer {
        ConfirmAnswer::Continue => None,
        ConfirmAnswer::Abort => Some(StopCause::ConfirmationDeclined {
            kind: point.kind,
            code: point.diagnosis.code,
        }),
        ConfirmAnswer::TimedOut => Some(StopCause::ConfirmationTimedOut {
            kind: point.kind,
            code: point.diagnosis.code,
        }),
    }
}

/// The diagnosis code a preview abort/timeout records.
const PREVIEW_CODE: &str = "resume_preview";

/// The resume-preview reposition loop (design §D.2). Runs AFTER the hover
/// lift + cool-down step; the nozzle is at the single hover plane and every
/// reposition is XY-only there (never-descend is structural — there is no
/// per-stop Z move). `None` means the operator accepted (and the recovery
/// file for the chosen stop has been written — execution proceeds to the
/// `M23`/`M24` steps); `Some(cause)` aborts (abort/timeout/exclusivity/
/// write failure), and because this step sits past `ShiftedFrame` the abort
/// invalidates the Z frame.
#[allow(clippy::too_many_arguments)] // one collaborator per concern, as execute()
async fn run_preview_loop(
    plan: &RecoveryPlan,
    client: &mut MoonrakerClient,
    _options: &ExecOptions,
    confirmer: &mut dyn Confirmer,
    exclusivity: &mut dyn Exclusivity,
    writer: &mut dyn RecoveryFileWriter,
    barrier_budget: Duration,
    deadline: Duration,
    step: &RecoveryStep,
    transcript: &mut Transcript<'_>,
) -> Option<StopCause> {
    // A ResumePreview step with no spec is a malformed plan
    // (`resume_preview_step_iff_spec`); treat it as nothing to drive rather
    // than panic (`?` returns None = no abort cause).
    let spec = plan.preview.as_ref()?;
    let len = spec.stops.len();
    if len == 0 {
        return None;
    }
    let last = len - 1;
    // The loop opens on the skip-forward default (clamped defensively).
    let mut cursor = index_usize(spec.default_index).min(last);
    let mut rep_ptr = nearest_rep_ptr(&spec.representatives, cursor);
    loop {
        // ELEVENTH COROLLARY: re-take the exclusivity barrier before EVERY
        // reposition send. Each answer is a human-time gap up to the
        // confirm deadline; a preview that reasserts once at entry is the
        // bug. `reassert_exclusivity` is the exact same call the step loop
        // makes, so the fake `Exclusivity` counts one recheck per send.
        if let Some(cause) =
            reassert_exclusivity(step, exclusivity, client, barrier_budget, transcript).await
        {
            return Some(cause);
        }
        let cur = &spec.stops[cursor];
        let command = format!(
            "G1 X{} Y{} F{}",
            fmt_num(cur.xy[0]),
            fmt_num(cur.xy[1]),
            fmt_num(spec.travel_feed)
        );
        transcript.entry(&json!({
            "event": "preview-reposition",
            "step": step.id,
            "cursor": cursor,
            "offset": cur.offset,
            "command": command,
        }));
        if let Err(e) = client.gcode_script(&command).await {
            let message = e.to_string();
            let failure = StepFailure::classify(&message);
            return Some(StopCause::CommandFailed { failure, message });
        }
        let point = preview_point(spec, cursor, step);
        match ask_preview(point, deadline, confirmer, transcript).await {
            PreviewAnswer::Accept => {
                // Materialise the recovery file for THIS stop, then fall
                // through so the M23/M24 steps select it (design §4).
                let Some(binding) = spec.binding(cursor_u32(cursor)) else {
                    return Some(StopCause::ComputeFailed(
                        "accepted preview stop has no recovery binding".to_owned(),
                    ));
                };
                transcript.entry(&json!({
                    "event": "preview-accept",
                    "step": step.id,
                    "cursor": cursor,
                    "resume_offset": binding.tail_offset,
                }));
                if let Err(reason) = writer.write_for(binding) {
                    transcript.entry(&json!({
                        "event": "preview-write-failed",
                        "step": step.id,
                        "reason": reason,
                    }));
                    return Some(StopCause::ComputeFailed(format!(
                        "recovery file write failed on accept: {reason}"
                    )));
                }
                return None;
            }
            PreviewAnswer::NextRep => {
                if !spec.representatives.is_empty() {
                    rep_ptr = (rep_ptr + 1).min(spec.representatives.len() - 1);
                    cursor = index_usize(spec.representatives[rep_ptr]).min(last);
                }
            }
            PreviewAnswer::PrevRep => {
                if !spec.representatives.is_empty() {
                    rep_ptr = rep_ptr.saturating_sub(1);
                    cursor = index_usize(spec.representatives[rep_ptr]).min(last);
                }
            }
            PreviewAnswer::Nudge(delta) => {
                // Clamp the cursor to the stop list (design §D.1: ±1/±10),
                // using saturating usize arithmetic (no signed casts).
                let step_by = index_usize(delta.unsigned_abs());
                cursor = if delta < 0 {
                    cursor.saturating_sub(step_by)
                } else {
                    cursor.saturating_add(step_by).min(last)
                };
                rep_ptr = nearest_rep_ptr(&spec.representatives, cursor);
            }
            // Abort/timeout: past ShiftedFrame, so the caller's finish_abort
            // invalidates the Z frame — the nozzle has been driven around
            // the part and Z trust must be re-established (design §D.2 / §6).
            PreviewAnswer::Abort => {
                return Some(StopCause::ConfirmationDeclined {
                    kind: ConfirmKind::Preview,
                    code: PREVIEW_CODE,
                });
            }
            PreviewAnswer::TimedOut => {
                return Some(StopCause::ConfirmationTimedOut {
                    kind: ConfirmKind::Preview,
                    code: PREVIEW_CODE,
                });
            }
        }
    }
}

/// A stop index (`u32`) as a `usize`, saturating (the preview domain is
/// bounded by `PREVIEW_MAX_STOPS`, far below either type's max, so the
/// saturation is unreachable — it only avoids a lossy `as` cast).
fn index_usize(index: u32) -> usize {
    usize::try_from(index).unwrap_or(usize::MAX)
}

/// A cursor (`usize`) as a `u32` stop index, saturating (see
/// [`index_usize`] for why the saturation is unreachable).
fn cursor_u32(cursor: usize) -> u32 {
    u32::try_from(cursor).unwrap_or(u32::MAX)
}

/// The index into `reps` of the representative nearest `cursor` (by stop
/// index distance). `0` when there are no representatives — the loop then
/// relies on nudge to move the cursor.
fn nearest_rep_ptr(reps: &[u32], cursor: usize) -> usize {
    reps.iter()
        .enumerate()
        .min_by_key(|(_, &r)| index_usize(r).abs_diff(cursor))
        .map_or(0, |(i, _)| i)
}

/// Builds the [`ConfirmPoint`] for the stop at `cursor`: the `detail` map
/// the socket renders (offset / line-position / XY / Z / layer / feature /
/// `is_candidate` / rep-position / re-print warning), documented field-for-
/// field so increment 3's plugin can read it byte-for-byte (design §D.3 /
/// §10 attack #7). See `ctrlsock`'s producer docs for the field contract.
fn preview_point(
    spec: &plr_recovery::PreviewSpec,
    cursor: usize,
    step: &RecoveryStep,
) -> ConfirmPoint {
    let cur = &spec.stops[cursor];
    // Advisory re-print warning: the cursor is EARLIER than the safe
    // skip-forward default, so accepting re-prints geometry that may exist.
    let before_default = cursor < index_usize(spec.default_index);
    let diagnosis = Diagnosis::new(
        PREVIEW_CODE,
        Tier::Advisory,
        format!(
            "hovering over stop {} of {} at X{} Y{} (byte {}); move to the ragged edge on \
             the part, then accept",
            cursor + 1,
            spec.stops.len(),
            fmt_num(cur.xy[0]),
            fmt_num(cur.xy[1]),
            cur.offset,
        ),
        if before_default {
            "this point is BEFORE the safe skip-forward line; accepting re-prints existing \
             geometry (the nozzle plows the printed wall)"
                .to_owned()
        } else {
            "accepting resumes at the next deposition line after this point (skip-forward)"
                .to_owned()
        },
        "Answer accept to resume here, next/prev to step between representative points, \
         nudge +/-1 (fine) or +/-10 (coarse) to move along the toolpath, or abort to stop."
            .to_owned(),
    );
    ConfirmPoint {
        kind: ConfirmKind::Preview,
        step_id: step.id,
        phase: step.phase.name().to_owned(),
        diagnosis,
        detail: preview_detail(spec, cursor),
        deadline: Duration::ZERO,
    }
}

/// The current-stop `detail` map — the increment-3 plugin contract
/// (design §D.3). Every field here is consumed by `ctrlsock`'s
/// `report_pause`; the field names and shapes are fixed.
fn preview_detail(spec: &plr_recovery::PreviewSpec, cursor: usize) -> Value {
    let cur = &spec.stops[cursor];
    json!({
        // Byte offset of this deposition line (safe for M26; updates every
        // reposition — adjacent stops can be <1mm apart, so the offset, not
        // visible motion, is the alignment feedback).
        "offset": cur.offset,
        // Where a resume STARTS if this stop is accepted.
        "resume_offset": cur.resume_offset,
        // Hover target, Klipper-internal frame.
        "xy": [cur.xy[0], cur.xy[1]],
        "z": cur.z,
        "layer": cur.layer,
        // Feature class name (infill/wall/...) for the prompt.
        "feature": format!("{:?}", cur.feature),
        "on_infill": cur.on_infill,
        // Whether this stop matched the evidence (vs a nudge-only line).
        "is_candidate": cur.is_candidate,
        // Rep position "stop N of M" (1-based N).
        "position": cursor + 1,
        "count": spec.stops.len(),
        // Advisory: cursor is earlier than the skip-forward default →
        // accepting risks re-printing existing geometry.
        "before_skip_forward": cursor < index_usize(spec.default_index),
    })
}

/// Puts one preview reposition point to the confirmer, bounded by a FRESH
/// full deadline per pause (design §D.2: the operator gets the whole budget
/// per interaction, not a shrinking global one), and records the question
/// and answer in the transcript. Mirrors [`ask`] but for [`PreviewAnswer`].
async fn ask_preview(
    mut point: ConfirmPoint,
    deadline: Duration,
    confirmer: &mut dyn Confirmer,
    transcript: &mut Transcript<'_>,
) -> PreviewAnswer {
    point.deadline = deadline;
    transcript.entry(&json!({
        "event": "confirm-pause",
        "kind": point.kind.tag(),
        "step": point.step_id,
        "phase": point.phase,
        "diagnosis": point.diagnosis,
        "detail": point.detail,
        "deadline_s": point.deadline.as_secs_f64(),
    }));
    let bound = point.deadline;
    let answer = tokio::time::timeout(bound, confirmer.confirm_preview(&point))
        .await
        .unwrap_or(PreviewAnswer::TimedOut);
    transcript.entry(&json!({
        "event": "confirm-answer",
        "kind": point.kind.tag(),
        "step": point.step_id,
        "code": point.diagnosis.code,
        "answer": answer.tag(),
    }));
    answer
}

/// The `debug_confirm_each_step` question: this step, its commands, and
/// what will be verified afterwards.
fn step_debug_point(step: &RecoveryStep) -> ConfirmPoint {
    ConfirmPoint {
        kind: ConfirmKind::StepDebug,
        step_id: step.id,
        phase: step.phase.name().to_owned(),
        diagnosis: Diagnosis::new(
            "step_debug_pause",
            Tier::Confirmable,
            format!(
                "about to run step {} [{}]: {}",
                step.id,
                step.phase.name(),
                step.summary
            ),
            "debug_confirm_each_step is set, so execution stops before every step. Nothing \
             is wrong — this pause exists so the exact commands below can be read before \
             they are sent."
                .to_owned(),
            "Answer `continue` to send this step, or `abort` to stop here. Unset \
             `debug_confirm_each_step` in printer.cfg's [plr] section to run without \
             these pauses."
                .to_owned(),
        ),
        detail: json!({
            "summary": step.summary,
            "commands": step.commands,
            "pre_verify": step.pre_verify.iter().map(describe_verification).collect::<Vec<_>>(),
            "verify": step.verify.iter().map(describe_verification).collect::<Vec<_>>(),
            "cleanup_commands": step.cleanup_commands,
        }),
        deadline: Duration::ZERO,
    }
}

fn describe_verification(v: &Verification) -> String {
    format!("{}.{} {}", v.object, v.field, v.predicate.describe())
}

/// The `confirm_z_before_resume` question: what Z the daemon believes it
/// is at, how that number was derived, and where the toolhead is now.
///
/// Deliberately read-only beyond the standoff lift the plan already
/// performed. The live readback is best-effort: a status query that fails
/// here must not turn an operator confirmation into an abort, so the
/// field simply reports that it was unavailable.
async fn z_confirm_point(
    client: &mut MoonrakerClient,
    plan: &RecoveryPlan,
    step: &RecoveryStep,
    computed: Option<f64>,
) -> ConfirmPoint {
    let live_z = query_number(client, "toolhead", "position.2").await.ok();
    let formula = plan
        .first_index(Phase::TrueZDeclare)
        .and_then(|i| plan.steps[i].compute)
        .and_then(|c| match c {
            RuntimeComputation::TrueZ(f) => Some(f),
            RuntimeComputation::RecordMaxAccel
            | RuntimeComputation::RecordMachineAccel
            | RuntimeComputation::ParkZ { .. }
            | RuntimeComputation::HoverPlane { .. } => None,
        });
    let derivation = formula.map_or_else(
        || "unavailable (no true-Z step in this plan)".to_owned(),
        |f| {
            format!(
                "true_Z = z_prev_top {} + (halt_Z - trigger_Z), trigger read from {}",
                fmt_num(f.z_prev_top),
                trigger_source_name(f.trigger_source)
            )
        },
    );
    let what = match live_z {
        Some(z) => format!(
            "the toolhead is standing off at Z {} mm; confirm this matches what you see",
            fmt_num(z)
        ),
        None => "the toolhead is standing off above the resume point; confirm this matches \
                 what you see"
            .to_owned(),
    };
    let mut diagnosis = Diagnosis::new(
        "z_confirm_before_resume",
        Tier::Confirmable,
        what,
        format!(
            "confirm_z_before_resume is set. Z was established by touching the part once \
             and doing arithmetic on the result ({derivation}); everything the resume does \
             from here trusts that number. This is the last moment at which a human can \
             compare it against the actual nozzle before the print continues. The toolhead \
             was lifted to this standoff and never lowered — nothing here can descend."
        ),
        "Answer `continue` if the standoff looks right. If it does not, answer `abort`: \
         the recovery stops and the Z frame is invalidated, so a fresh dry run is required \
         before any resume. Unset `confirm_z_before_resume` in printer.cfg's [plr] section \
         to skip this pause."
            .to_owned(),
    );
    if let Some(z) = live_z {
        diagnosis = diagnosis.measured("toolhead.position.2", z, "mm");
    }
    if let Some(z) = computed {
        diagnosis = diagnosis.expected("standoff target", Some(z), Some(z), "mm");
    }
    ConfirmPoint {
        kind: ConfirmKind::ZHeight,
        step_id: step.id,
        phase: step.phase.name().to_owned(),
        diagnosis,
        detail: json!({
            "standoff_target_z": computed,
            "live_toolhead_z": live_z,
            "derivation": derivation,
        }),
        deadline: Duration::ZERO,
    }
}

fn trigger_source_name(source: TriggerSource) -> &'static str {
    match source {
        TriggerSource::RawLastZResult => "probe.last_z_result (raw trigger Z)",
        TriggerSource::BedZPlusOffset { .. } => "probe.last_probe_position[2] + z_offset",
        TriggerSource::DragResult => "plr.last_drag_result.trigger_z (ADXL drag)",
        TriggerSource::TouchResult { .. } => {
            "plr.last_touch_result.median_z + z_offset (consensus touch)"
        }
    }
}

/// Runs one step's pre-verify, compute, send, and post-verify. Registers
/// the step's abort cleanup once its commands are about to take effect.
/// Returns the step's resolved runtime value, which the Z-confirmation
/// pause reports as the standoff target.
async fn run_step(
    client: &mut MoonrakerClient,
    step: &RecoveryStep,
    options: &ExecOptions,
    accel: &mut AccelSlots,
    cleanups: &mut Vec<(u32, Vec<String>)>,
    transcript: &mut Transcript<'_>,
) -> Result<Option<f64>, StopCause> {
    // Pre-verifications: must hold before anything is sent.
    for verification in &step.pre_verify {
        poll_verification(client, verification, None, options, transcript, "pre").await?;
    }
    // Runtime computation: the true-Z formula, a park height, or one of
    // the two accel records; for a plain step the resolved value is the
    // persisted phase accel (so the restore step's `{restore_accel}` /
    // NumWithinComputed resolve).
    let computed = resolve_compute(client, step, accel, transcript).await?;
    // Register this step's abort cleanup BEFORE its commands run: its
    // side effect (the accel clamp) is about to be in force, so any
    // subsequent abort must undo it.
    if !step.cleanup_commands.is_empty() {
        cleanups.push((step.id, step.cleanup_commands.clone()));
    }
    // Commands: the only send path in the crate (invariant 2).
    send_step_commands(client, step, computed, accel.machine, transcript).await?;
    // Post-verifications.
    for verification in &step.verify {
        poll_verification(client, verification, computed, options, transcript, "post").await?;
    }
    Ok(computed)
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
    accel: AccelSlots,
    shifted_id: Option<u32>,
    transcript: &mut Transcript<'_>,
) -> ExecOutcome {
    for (sid, commands) in cleanups.iter().rev() {
        for command in commands {
            // Reverse registration order means the touch clamp's restore
            // (phase slot) runs before the recovery-accel restore
            // (machine slot), so the machine ends at its own value even
            // when both were in force.
            let slot = if command.contains(RESTORE_ACCEL_PLACEHOLDER) {
                Some((RESTORE_ACCEL_PLACEHOLDER, accel.phase))
            } else if command.contains(MACHINE_ACCEL_PLACEHOLDER) {
                Some((MACHINE_ACCEL_PLACEHOLDER, accel.machine))
            } else {
                None
            };
            let resolved = match slot {
                Some((placeholder, Some(value))) => command.replace(placeholder, &fmt_num(value)),
                Some((_, None)) => {
                    transcript.entry(&json!({
                        "event": "cleanup-skip", "step": sid, "command": command,
                        "reason": "no recorded accel to substitute",
                    }));
                    continue;
                }
                None => command.clone(),
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
    // A confirm-point abort did not fail the step's verification, so
    // reporting the step's own abort reason would name a failure that
    // never happened. It reports why it really stopped instead — but it
    // is the same abort, at the same anchor step, and therefore obeys the
    // same frame-invalidation rule.
    let reason = match &cause {
        StopCause::ConfirmationDeclined { .. } => "confirmation-declined".to_owned(),
        StopCause::ConfirmationTimedOut { .. } => "confirmation-timeout".to_owned(),
        StopCause::FrameGuardUnwritable(_) => "frame-interlock-unwritable".to_owned(),
        // Not the step's own abort reason: the step never ran. What
        // stopped the recovery is that somebody else had the printer.
        StopCause::ExclusivityLost(_) => "exclusive-gcode-access-lost".to_owned(),
        StopCause::HardDiagnosis { code } => (*code).to_owned(),
        StopCause::VerificationFailed { .. }
        | StopCause::Transport(_)
        | StopCause::CommandFailed { .. }
        | StopCause::ComputeFailed(_)
        | StopCause::OperatorDeclined => reason.code().to_owned(),
    };
    // Anchoring on the step id is right for every abort EXCEPT the
    // fail-closed refusal to enter: there the declare was never issued,
    // so the frame is exactly as valid as it was before this run — and
    // saying otherwise would demand a marker we have just proven we
    // cannot write.
    //
    // `ExclusivityLost` is deliberately NOT given that exemption even
    // though it too stops before the step's commands. At the shifted-frame
    // step itself the frame is in fact still valid, so the marker
    // over-reports by exactly one step — but the marker is writable here
    // (unlike the FrameGuard case), the cost of over-reporting is one dry
    // run, and the operator has just been told that something else was
    // printing on their machine, which is a state they should re-plan
    // from rather than resume into.
    let frame_invalid = !matches!(cause, StopCause::FrameGuardUnwritable(_))
        && shifted_id.is_some_and(|sid| step.id >= sid);
    let diagnosis = cause.step_failure().map(|f| f.diagnosis());
    transcript.entry(&json!({
        "event": "abort",
        "step": step.id,
        "phase": step.phase.name(),
        "reason": reason,
        "cause": format!("{cause:?}"),
        "failure": cause.step_failure().map(StepFailure::code),
        "diagnosis": diagnosis,
        "frame_invalid": frame_invalid,
    }));
    ExecOutcome::Aborted {
        step_id: step.id,
        phase: step.phase.name().to_owned(),
        reason,
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
    machine_accel: Option<f64>,
    transcript: &mut Transcript<'_>,
) -> Result<(), StopCause> {
    for command in &step.commands {
        let has_computed_placeholder = command.contains(TRUE_Z_PLACEHOLDER)
            || command.contains(RESTORE_ACCEL_PLACEHOLDER)
            || command.contains(PARK_Z_PLACEHOLDER);
        let mut resolved = if has_computed_placeholder {
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
                .replace(PARK_Z_PLACEHOLDER, &fmt_num(value))
        } else {
            command.clone()
        };
        // The machine-accel slot is separate from the computed value (see
        // MACHINE_ACCEL_PLACEHOLDER); it is recorded by an earlier step,
        // never by this one.
        if resolved.contains(MACHINE_ACCEL_PLACEHOLDER) {
            let Some(value) = machine_accel else {
                return Err(StopCause::ComputeFailed(
                    "command carries {machine_accel} but no machine accel was recorded".to_owned(),
                ));
            };
            resolved = resolved.replace(MACHINE_ACCEL_PLACEHOLDER, &fmt_num(value));
        }
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
    accel: &mut AccelSlots,
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
        Some(RuntimeComputation::ParkZ { delta_z, z_max }) => {
            // Read the CURRENT Z (before the step's lift runs) and compute
            // the rail-clamped absolute park height. Klipper does not clamp
            // an out-of-range move — it raises "Move out of range" — so the
            // clamp must happen here, where the true Z is finally known.
            let current_z = query_number(client, "toolhead", "position.2").await?;
            match plr_recovery::park_z_at(current_z, delta_z, z_max) {
                Ok(park_z) => {
                    transcript.entry(&json!({
                        "event": "compute",
                        "step": step.id,
                        "current_z": current_z,
                        "delta_z": delta_z,
                        "z_max": z_max,
                        "park_z": park_z,
                    }));
                    Ok(Some(park_z))
                }
                // Never substitute on error (plr-recovery contract).
                Err(e) => Err(StopCause::ComputeFailed(e.to_string())),
            }
        }
        Some(RuntimeComputation::HoverPlane { target_z, z_max }) => {
            // Read the CURRENT Z (before the lift) and compute the
            // resume-preview hover plane. Like ParkZ this is a lift or a
            // no-op — never a descent — but the floor is an ABSOLUTE
            // target_z above every stop rather than a relative delta.
            let current_z = query_number(client, "toolhead", "position.2").await?;
            match plr_recovery::hover_plane_at(current_z, target_z, z_max) {
                Ok(hover_z) => {
                    transcript.entry(&json!({
                        "event": "compute",
                        "step": step.id,
                        "current_z": current_z,
                        "target_z": target_z,
                        "z_max": z_max,
                        "hover_plane": hover_z,
                    }));
                    Ok(Some(hover_z))
                }
                // Never substitute on error (plr-recovery contract).
                Err(e) => Err(StopCause::ComputeFailed(e.to_string())),
            }
        }
        Some(RuntimeComputation::RecordMaxAccel) => {
            // Read BEFORE the step's SET_VELOCITY_LIMIT clamps it, and
            // persist in the PHASE slot for the restore step / abort
            // cleanup.
            let value = query_number(client, "toolhead", "max_accel").await?;
            accel.phase = Some(value);
            transcript.entry(&json!({
                "event": "record-accel", "step": step.id, "max_accel": value,
            }));
            Ok(Some(value))
        }
        Some(RuntimeComputation::RecordMachineAccel) => {
            // The MACHINE slot: read before the plan changes acceleration
            // at all, so what goes back at the end is the printer's own
            // value rather than whatever the recovery was running at.
            let value = query_number(client, "toolhead", "max_accel").await?;
            accel.machine = Some(value);
            transcript.entry(&json!({
                "event": "record-machine-accel", "step": step.id, "max_accel": value,
            }));
            Ok(Some(value))
        }
        // A plain step resolves to the persisted PHASE accel: this is
        // what the accel-restore step's `{restore_accel}` substitution
        // and NumWithinComputed check read.
        None => Ok(accel.phase),
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
        dry_run, evaluate, execute, lookup, AbortConfirmer, ConfirmAnswer, ConfirmKind,
        ConfirmPoint, Confirmer, Exclusivity, ExecOptions, ExecOutcome, FrameGuard, NoExclusivity,
        NoFrameGuard, NoopFileWriter, PreviewAnswer, RecoveryFileWriter, StepFailure, StopCause,
        Transcript,
    };
    use crate::moonraker::MoonrakerClient;
    use crate::testmoon::FakeMoonraker;
    use plr_analyzer::{FeatureClass, PreviewStop};
    use plr_recovery::{
        compute_envelope, AbortReason, EnvelopeParams, FailureAction, Phase, Predicate,
        PreviewBinding, PreviewSpec, RecoveryPlan, RecoveryStep, RuntimeComputation, TriggerSource,
        TrueZFormula, Verification,
    };
    use serde_json::{json, Value};
    use std::future::Future;
    use std::pin::Pin;
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
            resume_file: "x_RECOVERY.gcode".to_owned(),
            resume_offset: 1234,
            requires_clean_nozzle_confirmation: false,
            recovery_file: plr_recovery::RecoveryFileSpec::default(),
            debug_confirm_each_step: false,
            confirm_timeout_s: None,
            gcode_barrier_timeout_s: None,
            preview: None,
            warnings: vec![],
        }
    }

    fn fast_options() -> ExecOptions {
        ExecOptions {
            verify_timeout: Duration::from_millis(300),
            temp_timeout: Duration::from_millis(300),
            poll_interval: Duration::from_millis(20),
            confirm_timeout: Duration::from_millis(300),
            gcode_barrier_timeout: Duration::from_millis(300),
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
        run_with(plan, fake, gate, &mut AbortConfirmer, &fast_options()).await
    }

    /// The full harness: an explicit confirmer and explicit options, so a
    /// confirm-point test can supply both.
    pub(crate) async fn run_with(
        plan: &RecoveryPlan,
        fake: &FakeMoonraker,
        gate: &mut (dyn FnMut(&RecoveryStep) -> bool + Send),
        confirmer: &mut dyn Confirmer,
        options: &ExecOptions,
    ) -> (ExecOutcome, String) {
        run_guarded(
            plan,
            fake,
            gate,
            confirmer,
            &mut NoFrameGuard,
            &mut NoExclusivity,
            options,
        )
        .await
    }

    /// [`run_with`] plus an explicit [`Exclusivity`], for the tests about
    /// losing the g-code channel mid-plan.
    pub(crate) async fn run_exclusive(
        plan: &RecoveryPlan,
        fake: &FakeMoonraker,
        confirmer: &mut dyn Confirmer,
        exclusivity: &mut dyn Exclusivity,
        options: &ExecOptions,
    ) -> (ExecOutcome, String) {
        run_guarded(
            plan,
            fake,
            &mut |_| true,
            confirmer,
            &mut NoFrameGuard,
            exclusivity,
            options,
        )
        .await
    }

    /// [`run_with`] plus an explicit [`FrameGuard`], for the tests that
    /// care about the interlock.
    pub(crate) async fn run_guarded(
        plan: &RecoveryPlan,
        fake: &FakeMoonraker,
        gate: &mut (dyn FnMut(&RecoveryStep) -> bool + Send),
        confirmer: &mut dyn Confirmer,
        frame_guard: &mut dyn FrameGuard,
        exclusivity: &mut dyn Exclusivity,
        options: &ExecOptions,
    ) -> (ExecOutcome, String) {
        let mut client = MoonrakerClient::connect(&fake.url(), Duration::from_secs(5))
            .await
            .unwrap();
        let mut buffer = Vec::new();
        let mut writer = NoopFileWriter;
        let outcome = {
            let mut transcript = Transcript::new(&mut buffer);
            execute(
                plan,
                &mut client,
                options,
                gate,
                confirmer,
                frame_guard,
                exclusivity,
                &mut writer,
                &mut transcript,
            )
            .await
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
                // The plugin's real consensus-failure text
                // (touch_sequence.py consensus_failure_text).
                "PLR_TOUCH failed: could not find 3 touches within 0.010 mm of each \
                 other in a sliding window of 5, after 10 touches.",
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

    // --- Confirm-points --------------------------------------------------
    //
    // Three features, one mechanism. The tests below hold the mechanism
    // to the promise made in the module docs: a pause is an `await` whose
    // abort path is byte-for-byte the ordinary abort path, so nothing a
    // pause can do is something an abort could not already do.

    /// A confirmer that answers from a fixed script and records every
    /// question it was asked.
    struct ScriptedConfirmer {
        answers: std::collections::VecDeque<ConfirmAnswer>,
        /// Used once the script runs out.
        default: ConfirmAnswer,
        asked: Arc<Mutex<Vec<(ConfirmKind, String, u32)>>>,
    }

    impl ScriptedConfirmer {
        fn new(answers: &[ConfirmAnswer], default: ConfirmAnswer) -> Self {
            Self {
                answers: answers.iter().copied().collect(),
                default,
                asked: Arc::new(Mutex::new(Vec::new())),
            }
        }
    }

    impl Confirmer for ScriptedConfirmer {
        fn confirm<'a>(
            &'a mut self,
            point: &'a ConfirmPoint,
        ) -> std::pin::Pin<Box<dyn std::future::Future<Output = ConfirmAnswer> + Send + 'a>>
        {
            Box::pin(async move {
                self.asked.lock().expect("asked").push((
                    point.kind,
                    point.diagnosis.code.to_owned(),
                    point.step_id,
                ));
                self.answers.pop_front().unwrap_or(self.default)
            })
        }
    }

    /// A confirmer that never answers, so the executor's own
    /// `confirm_timeout` is what ends the pause.
    struct SilentConfirmer;

    impl Confirmer for SilentConfirmer {
        fn confirm<'a>(
            &'a mut self,
            _point: &'a ConfirmPoint,
        ) -> std::pin::Pin<Box<dyn std::future::Future<Output = ConfirmAnswer> + Send + 'a>>
        {
            Box::pin(async {
                // Far longer than any test's confirm_timeout.
                tokio::time::sleep(Duration::from_hours(1)).await;
                ConfirmAnswer::Continue
            })
        }
    }

    /// Records the deadline stamped on each confirm-point it is asked, and
    /// answers `Abort` immediately so the test does not have to wait one
    /// out.
    #[derive(Default)]
    struct DeadlineSpy {
        seen: std::sync::Arc<std::sync::Mutex<Vec<Duration>>>,
    }

    impl Confirmer for DeadlineSpy {
        fn confirm<'a>(
            &'a mut self,
            point: &'a ConfirmPoint,
        ) -> std::pin::Pin<Box<dyn std::future::Future<Output = ConfirmAnswer> + Send + 'a>>
        {
            let seen = std::sync::Arc::clone(&self.seen);
            let deadline = point.deadline;
            Box::pin(async move {
                seen.lock().expect("spy lock").push(deadline);
                ConfirmAnswer::Abort
            })
        }
    }

    /// The deadline a confirmer is handed is the one the executor will
    /// enforce, for both of the two ways it can be resolved.
    ///
    /// This is the field the control socket reports to its clients. Before
    /// it existed the socket said nothing, so the plugin had to assume the
    /// top of the permitted band (3600 s) to stay fail-safe and then
    /// claimed a confirmation was live for the better part of an hour after
    /// the daemon had aborted it.
    #[tokio::test]
    async fn the_confirm_point_carries_the_deadline_the_executor_enforces() {
        // (1) The operator's `[plr]` value wins, and it is what is
        //     reported — not the daemon default, and not the band ceiling.
        let mut plan = z_confirm_plan();
        plan.confirm_timeout_s = Some(45.0);
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut spy = DeadlineSpy::default();
        let seen = std::sync::Arc::clone(&spy.seen);
        let (outcome, _transcript) =
            run_with(&plan, &fake, &mut |_| true, &mut spy, &fast_options()).await;
        assert!(
            matches!(outcome, ExecOutcome::Aborted { .. }),
            "{outcome:?}"
        );
        assert_eq!(
            seen.lock().unwrap().as_slice(),
            &[Duration::from_secs(45)],
            "the plan's confirm_timeout_s must be the reported deadline"
        );

        // (2) With no plan value the daemon's own default is reported, and
        //     it is the ONE default — `plr-recovery`'s, in seconds.
        let mut plan = z_confirm_plan();
        plan.confirm_timeout_s = None;
        let options = ExecOptions {
            confirm_timeout: ExecOptions::default().confirm_timeout,
            ..fast_options()
        };
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut spy = DeadlineSpy::default();
        let seen = std::sync::Arc::clone(&spy.seen);
        let _ = run_with(&plan, &fake, &mut |_| true, &mut spy, &options).await;
        assert_eq!(
            seen.lock().unwrap().as_slice(),
            &[Duration::from_secs_f64(
                plr_recovery::CONFIRM_TIMEOUT_DEFAULT_S
            )],
            "with no [plr] value the reported deadline must be the shared default"
        );
    }

    /// An [`Exclusivity`] that succeeds for the first `allow` calls and
    /// then reports the printer busy — standing in for a job that started
    /// while execution was paused, or a `[delayed_gcode]` that grabbed the
    /// g-code channel between two steps.
    struct LosesAccessAfter {
        allow: usize,
        calls: std::sync::Arc<std::sync::Mutex<Vec<Duration>>>,
    }

    impl LosesAccessAfter {
        fn new(allow: usize) -> Self {
            Self {
                allow,
                calls: std::sync::Arc::default(),
            }
        }
    }

    impl Exclusivity for LosesAccessAfter {
        fn recheck<'a>(
            &'a mut self,
            _client: &'a mut MoonrakerClient,
            budget: Duration,
        ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), String>> + Send + 'a>>
        {
            Box::pin(async move {
                let mut calls = self.calls.lock().expect("call log");
                calls.push(budget);
                if calls.len() <= self.allow {
                    Ok(())
                } else {
                    Err("printer is not idle (print_stats.state \"printing\")".to_owned())
                }
            })
        }
    }

    /// Losing exclusive g-code access aborts BEFORE the step's commands go
    /// out, at every step — including the first, where nothing has been
    /// sent at all.
    ///
    /// This is the hole gates 4 and 4b alone left open: they are a sample,
    /// and `preflight_confirmations` plus the step-debug pause both run
    /// before step 1 and are bounded only by `confirm_timeout_s`. A job that
    /// starts during that pause used to be answered by issuing
    /// `SET_KINEMATIC_POSITION` and `PROBE` into a running print and
    /// reporting `COMPLETED`.
    #[tokio::test]
    async fn losing_exclusive_gcode_access_aborts_before_the_step_sends_anything() {
        // (1) Lost before step 1: literally nothing reaches the printer.
        let plan = z_confirm_plan();
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut exclusivity = LosesAccessAfter::new(0);
        let (outcome, transcript) = run_exclusive(
            &plan,
            &fake,
            &mut AbortConfirmer,
            &mut exclusivity,
            &fast_options(),
        )
        .await;
        let ExecOutcome::Aborted {
            step_id,
            reason,
            cause,
            frame_invalid,
            ..
        } = outcome
        else {
            panic!("expected an abort, got {outcome:?}");
        };
        assert_eq!(step_id, 1);
        assert_eq!(reason, "exclusive-gcode-access-lost");
        assert!(
            matches!(cause, StopCause::ExclusivityLost(ref why) if why.contains("not idle")),
            "{cause:?}"
        );
        // Step 1 of this plan IS the shifted-frame declare, which never
        // went out — so the frame is in fact still valid and the marker
        // over-reports by one step. Deliberate, and documented on `abort`:
        // the marker is writable here, over-reporting costs one dry run,
        // and an operator who has just been told another job owns their
        // printer should re-plan rather than resume.
        assert!(frame_invalid);
        assert!(
            transcript.contains("\"event\":\"exclusivity-lost\""),
            "{transcript}"
        );
        assert!(
            fake.gcode_sent().is_empty(),
            "nothing may be sent once access is lost: {:?}",
            fake.gcode_sent()
        );

        // (2) Lost mid-plan: the steps before it ran, the step it fired on
        // sent nothing, and nothing after it ran either.
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut exclusivity = LosesAccessAfter::new(2);
        let (outcome, _transcript) = run_exclusive(
            &plan,
            &fake,
            &mut AbortConfirmer,
            &mut exclusivity,
            &fast_options(),
        )
        .await;
        let ExecOutcome::Aborted { step_id, .. } = outcome else {
            panic!("expected an abort, got {outcome:?}");
        };
        assert_eq!(step_id, 3, "aborts at the step whose re-check failed");
        // Steps 1 and 2 of this plan are SET_IDLE_TIMEOUT and PROBE; step 3
        // is the true-Z declare that never went out.
        let sent = fake.gcode_sent();
        assert!(
            sent.iter().any(|c| c.starts_with("SET_IDLE_TIMEOUT")),
            "step 1 ran: {sent:?}"
        );
        assert!(
            sent.iter().any(|c| c.starts_with("PROBE")),
            "step 2 ran: {sent:?}"
        );
        assert!(
            !sent.iter().any(|c| c.starts_with("SET_KINEMATIC_POSITION")),
            "step 3's commands must never have been issued: {sent:?}"
        );
    }

    /// The budget handed to each re-check is the operator's `[plr]`
    /// `gcode_barrier_timeout_s` when the plan carries one, else the
    /// daemon's default — the same resolution rule as the confirm deadline,
    /// and it must reach the re-check rather than being re-derived there.
    #[tokio::test]
    async fn the_recheck_budget_comes_from_the_plan_then_the_daemon_default() {
        for (plan_value, expected) in [
            (Some(90.0), Duration::from_secs(90)),
            (None, Duration::from_millis(300)),
            // Nonsense values fall back rather than being honoured: a zero
            // budget would refuse every recovery.
            (Some(0.0), Duration::from_millis(300)),
            (Some(f64::NAN), Duration::from_millis(300)),
        ] {
            let mut plan = z_confirm_plan();
            plan.gcode_barrier_timeout_s = plan_value;
            let fake = FakeMoonraker::spawn(happy_handler).await;
            // Fails on the first call, so exactly one budget is recorded and
            // the test does not depend on how far the plan gets.
            let mut exclusivity = LosesAccessAfter::new(0);
            let calls = std::sync::Arc::clone(&exclusivity.calls);
            let _ = run_exclusive(
                &plan,
                &fake,
                &mut AbortConfirmer,
                &mut exclusivity,
                &fast_options(),
            )
            .await;
            assert_eq!(
                calls.lock().unwrap().as_slice(),
                &[expected],
                "plan value {plan_value:?}"
            );
        }
    }

    /// The `[plr]` key's documented default and the deadline this daemon
    /// enforces are one value, and it lies inside the band the key is
    /// validated against.
    ///
    /// The band containment was never pinned; the equality reads the
    /// daemon's default through `ExecOptions::default()` — an expression
    /// independent of the constant — so re-introducing a second literal
    /// there (which is what the two crates used to have, in two different
    /// units) fails here rather than shipping.
    #[test]
    fn the_confirm_timeout_default_is_one_value_inside_its_own_band() {
        assert!(
            (plr_recovery::CONFIRM_TIMEOUT_MIN_S..=plr_recovery::CONFIRM_TIMEOUT_MAX_S)
                .contains(&plr_recovery::CONFIRM_TIMEOUT_DEFAULT_S),
            "the default must be a value an operator could also have set: {} not in {}..={}",
            plr_recovery::CONFIRM_TIMEOUT_DEFAULT_S,
            plr_recovery::CONFIRM_TIMEOUT_MIN_S,
            plr_recovery::CONFIRM_TIMEOUT_MAX_S,
        );
        // Compared as `Duration`s, so this is exact rather than a float
        // comparison dressed up as one.
        assert_eq!(
            ExecOptions::default().confirm_timeout,
            Duration::from_secs_f64(plr_recovery::CONFIRM_TIMEOUT_DEFAULT_S),
            "the daemon's fallback deadline and the [plr] key's default must be one number"
        );
        // Same two properties for the barrier budget.
        assert!(
            (plr_recovery::GCODE_BARRIER_TIMEOUT_MIN_S..=plr_recovery::GCODE_BARRIER_TIMEOUT_MAX_S)
                .contains(&plr_recovery::GCODE_BARRIER_TIMEOUT_DEFAULT_S),
            "the barrier default must be a value an operator could also have set: {} not in \
             {}..={}",
            plr_recovery::GCODE_BARRIER_TIMEOUT_DEFAULT_S,
            plr_recovery::GCODE_BARRIER_TIMEOUT_MIN_S,
            plr_recovery::GCODE_BARRIER_TIMEOUT_MAX_S,
        );
        assert_eq!(
            ExecOptions::default().gcode_barrier_timeout,
            Duration::from_secs_f64(plr_recovery::GCODE_BARRIER_TIMEOUT_DEFAULT_S),
        );
    }

    /// `test_plan()` with a shifted-frame declare as step 1, so every
    /// later abort must invalidate the Z frame.
    fn framed_plan() -> RecoveryPlan {
        let mut plan = test_plan();
        plan.steps[0].phase = Phase::ShiftedFrame;
        plan
    }

    /// [`framed_plan`] plus the operator Z-confirmation standoff the
    /// builder emits for `confirm_z_before_resume`: a fourth step that
    /// lifts to `min(current_Z + entry_hop, z_max)` — the same
    /// rail-clamped arithmetic the reheat park uses, which never
    /// descends — after the true-Z declare (step 3).
    fn z_confirm_plan() -> RecoveryPlan {
        let mut plan = framed_plan();
        plan.steps.push(RecoveryStep {
            id: 4,
            phase: Phase::ZConfirmStandoff,
            summary: "lift to the entry standoff for the operator Z confirmation".to_owned(),
            commands: vec!["G90".to_owned(), "G1 Z{park_z} F1200".to_owned()],
            pre_verify: vec![],
            verify: vec![],
            compute: Some(RuntimeComputation::ParkZ {
                delta_z: 1.0,
                z_max: None,
            }),
            cleanup_commands: vec![],
            on_failure: FailureAction::Abort {
                reason: AbortReason::ZConfirmStandoffFailed,
            },
        });
        plan
    }

    #[tokio::test]
    async fn an_advisory_diagnosis_proceeds_and_is_transcribed() {
        let mut plan = test_plan();
        plan.warnings = vec![plr_recovery::PlanWarning::ResumeNotOnInfill];
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut confirmer = ScriptedConfirmer::new(&[], ConfirmAnswer::Abort);
        let asked = Arc::clone(&confirmer.asked);
        let (outcome, transcript) =
            run_with(&plan, &fake, &mut |_| true, &mut confirmer, &fast_options()).await;
        assert_eq!(outcome, ExecOutcome::Completed { steps: 3 }, "{transcript}");
        // Advisory means "warn loudly, proceed": nobody was asked, and
        // the diagnosis is in the transcript in the one shape.
        assert!(asked.lock().expect("asked").is_empty());
        assert!(
            transcript.contains("\"event\":\"advisory\""),
            "{transcript}"
        );
        assert!(transcript.contains("resume_not_on_infill"), "{transcript}");
        assert!(transcript.contains("\"tier\":\"advisory\""), "{transcript}");
    }

    #[tokio::test]
    async fn a_confirmable_diagnosis_aborts_by_default_before_anything_is_sent() {
        // The default confirmer is the non-interactive one: preserving
        // exactly the behaviour a caller that never asked to be consulted
        // had before confirm-points existed.
        let mut plan = test_plan();
        plan.warnings = vec![plr_recovery::PlanWarning::PurgeZBelowResume {
            purge_z: 0.1,
            resume_z: 0.6,
        }];
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted {
            step_id,
            reason,
            cause,
            frame_invalid,
            ..
        } = outcome
        else {
            panic!("expected abort, got {outcome:?}");
        };
        // Anchored on step 1, before any command: declining costs nothing.
        assert_eq!(step_id, 1);
        assert_eq!(reason, "confirmation-declined");
        assert!(!frame_invalid, "nothing was declared, nothing is unknown");
        assert_eq!(
            cause,
            StopCause::ConfirmationDeclined {
                kind: ConfirmKind::Diagnosis,
                code: "purge_z_below_resume",
            }
        );
        assert!(fake.gcode_sent().is_empty(), "{:?}", fake.gcode_sent());
        assert!(
            transcript.contains("\"event\":\"confirm-pause\""),
            "{transcript}"
        );
        assert!(transcript.contains("\"answer\":\"abort\""), "{transcript}");
    }

    #[tokio::test]
    async fn a_confirmable_diagnosis_answered_continue_proceeds() {
        let mut plan = test_plan();
        plan.warnings = vec![plr_recovery::PlanWarning::PurgeZBelowResume {
            purge_z: 0.1,
            resume_z: 0.6,
        }];
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut confirmer =
            ScriptedConfirmer::new(&[ConfirmAnswer::Continue], ConfirmAnswer::Abort);
        let asked = Arc::clone(&confirmer.asked);
        let (outcome, transcript) =
            run_with(&plan, &fake, &mut |_| true, &mut confirmer, &fast_options()).await;
        assert_eq!(outcome, ExecOutcome::Completed { steps: 3 }, "{transcript}");
        let asked = asked.lock().expect("asked").clone();
        assert_eq!(asked.len(), 1);
        assert_eq!(asked[0].0, ConfirmKind::Diagnosis);
        assert_eq!(asked[0].1, "purge_z_below_resume");
        assert!(
            transcript.contains("\"answer\":\"continue\""),
            "{transcript}"
        );
        assert_eq!(fake.gcode_sent().len(), 3);
    }

    /// `confirm_z_before_resume`: a plan carrying the standoff step
    /// pauses after it, reports the believed Z and how it was derived,
    /// and continues on `continue`.
    #[tokio::test]
    async fn the_z_confirmation_reports_the_believed_z_and_continues() {
        let plan = z_confirm_plan();
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut confirmer =
            ScriptedConfirmer::new(&[ConfirmAnswer::Continue], ConfirmAnswer::Abort);
        let asked = Arc::clone(&confirmer.asked);
        let (outcome, transcript) =
            run_with(&plan, &fake, &mut |_| true, &mut confirmer, &fast_options()).await;
        assert_eq!(outcome, ExecOutcome::Completed { steps: 4 }, "{transcript}");
        let asked = asked.lock().expect("asked").clone();
        assert_eq!(
            asked,
            vec![(
                ConfirmKind::ZHeight,
                "z_confirm_before_resume".to_owned(),
                4
            )]
        );
        // The pause is AFTER the step ran and verified: the reported Z is
        // a settled readback, not a prediction.
        let pause_at = transcript.find("\"confirm-pause\"").expect("pause");
        let step_ok = transcript
            .find("\"step-ok\",\"step\":4")
            .expect("step-ok 4");
        assert!(step_ok < pause_at, "{transcript}");
        // It explains where the number came from.
        assert!(transcript.contains("z_prev_top"), "{transcript}");
        assert!(transcript.contains("last_z_result"), "{transcript}");
        assert!(transcript.contains("live_toolhead_z"), "{transcript}");
    }

    #[tokio::test]
    async fn declining_the_z_confirmation_aborts_and_invalidates_the_frame() {
        let plan = z_confirm_plan();
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut confirmer = ScriptedConfirmer::new(&[ConfirmAnswer::Abort], ConfirmAnswer::Abort);
        let (outcome, transcript) =
            run_with(&plan, &fake, &mut |_| true, &mut confirmer, &fast_options()).await;
        let ExecOutcome::Aborted {
            step_id,
            reason,
            frame_invalid,
            ..
        } = outcome
        else {
            panic!("expected abort, got {outcome:?}");
        };
        assert_eq!(step_id, 4);
        assert_eq!(reason, "confirmation-declined");
        assert!(
            frame_invalid,
            "a pause past the shifted-frame declare aborts like any other abort there"
        );
        assert!(
            transcript.contains("\"frame_invalid\":true"),
            "{transcript}"
        );
    }

    #[tokio::test]
    async fn an_unanswered_confirmation_times_out_into_the_same_clean_abort() {
        let plan = z_confirm_plan();
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let options = ExecOptions {
            confirm_timeout: Duration::from_millis(50),
            ..fast_options()
        };
        let (outcome, transcript) =
            run_with(&plan, &fake, &mut |_| true, &mut SilentConfirmer, &options).await;
        let ExecOutcome::Aborted {
            step_id,
            reason,
            cause,
            frame_invalid,
            ..
        } = outcome
        else {
            panic!("expected abort, got {outcome:?}");
        };
        assert_eq!(step_id, 4);
        assert_eq!(reason, "confirmation-timeout");
        assert_eq!(
            cause,
            StopCause::ConfirmationTimedOut {
                kind: ConfirmKind::ZHeight,
                code: "z_confirm_before_resume",
            }
        );
        // The frame rule is honored on the timeout path exactly as on the
        // decline path — this is the "no ambiguous frame" requirement.
        assert!(frame_invalid);
        assert!(
            transcript.contains("\"answer\":\"timeout\""),
            "{transcript}"
        );
        assert!(
            transcript.contains("\"frame_invalid\":true"),
            "{transcript}"
        );
    }

    /// The operator's `[plr]` `confirm_timeout_s` wins over the daemon's
    /// default: somebody inspecting a Z standoff with a flashlight may
    /// want longer than ten minutes, and a headless drill may want less.
    #[tokio::test]
    async fn the_plans_confirm_timeout_overrides_the_daemon_default() {
        let mut plan = z_confirm_plan();
        // A generous daemon default that the plan's own (tiny) value must
        // beat — otherwise this test would hang for 60 s instead of
        // timing out promptly.
        plan.confirm_timeout_s = Some(0.05);
        let options = ExecOptions {
            confirm_timeout: Duration::from_mins(1),
            ..fast_options()
        };
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let started = std::time::Instant::now();
        let (outcome, transcript) =
            run_with(&plan, &fake, &mut |_| true, &mut SilentConfirmer, &options).await;
        let ExecOutcome::Aborted { reason, .. } = outcome else {
            panic!("expected abort, got {outcome:?}");
        };
        assert_eq!(reason, "confirmation-timeout");
        assert!(
            started.elapsed() < Duration::from_secs(30),
            "the plan's timeout must win over the daemon default"
        );
        assert!(
            transcript.contains("\"answer\":\"timeout\""),
            "{transcript}"
        );

        // With no plan value the daemon default applies, unchanged.
        let mut defaulted = z_confirm_plan();
        defaulted.confirm_timeout_s = None;
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let (outcome, _) = run_with(
            &defaulted,
            &fake,
            &mut |_| true,
            &mut SilentConfirmer,
            &fast_options(),
        )
        .await;
        assert!(matches!(outcome, ExecOutcome::Aborted { .. }));
    }

    #[tokio::test]
    async fn step_debug_pauses_before_every_step_and_reports_it() {
        let mut plan = test_plan();
        plan.debug_confirm_each_step = true;
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut confirmer = ScriptedConfirmer::new(&[], ConfirmAnswer::Continue);
        let asked = Arc::clone(&confirmer.asked);
        let (outcome, transcript) =
            run_with(&plan, &fake, &mut |_| true, &mut confirmer, &fast_options()).await;
        assert_eq!(outcome, ExecOutcome::Completed { steps: 3 }, "{transcript}");
        let asked = asked.lock().expect("asked").clone();
        assert_eq!(asked.len(), 3, "one pause per step: {asked:?}");
        assert!(asked
            .iter()
            .all(|(k, c, _)| *k == ConfirmKind::StepDebug && c == "step_debug_pause"));
        assert_eq!(asked.iter().map(|a| a.2).collect::<Vec<_>>(), vec![1, 2, 3]);
        // The pause carries what is about to be sent and what will be
        // checked — that is the whole point of the mode.
        assert!(
            transcript.contains("SET_IDLE_TIMEOUT TIMEOUT=86400"),
            "{transcript}"
        );
        assert!(transcript.contains("\"pre_verify\""), "{transcript}");
        // Declining a step-debug pause stops before that step sends.
        let mut declining = ScriptedConfirmer::new(
            &[ConfirmAnswer::Continue, ConfirmAnswer::Abort],
            ConfirmAnswer::Abort,
        );
        let fake2 = FakeMoonraker::spawn(happy_handler).await;
        let (outcome, _) = run_with(
            &plan,
            &fake2,
            &mut |_| true,
            &mut declining,
            &fast_options(),
        )
        .await;
        let ExecOutcome::Aborted { step_id, .. } = outcome else {
            panic!("expected abort");
        };
        assert_eq!(step_id, 2);
        assert_eq!(fake2.gcode_sent(), vec!["SET_IDLE_TIMEOUT TIMEOUT=86400"]);
    }

    #[tokio::test]
    async fn confirm_points_are_byte_identical_to_today_when_disabled() {
        // The inertness proof: with no Confirmable warning, no standoff
        // step and debug off, the transcript and the sent commands are
        // exactly what they were before confirm-points existed — and a
        // confirmer that would abort everything is never consulted.
        let plan = test_plan();
        assert!(!plan.debug_confirm_each_step);
        let fake_a = FakeMoonraker::spawn(happy_handler).await;
        let (outcome_a, transcript_a) = run(&plan, &fake_a, &mut |_| true).await;
        let fake_b = FakeMoonraker::spawn(happy_handler).await;
        let mut confirmer = ScriptedConfirmer::new(&[], ConfirmAnswer::Abort);
        let asked = Arc::clone(&confirmer.asked);
        let (outcome_b, transcript_b) = run_with(
            &plan,
            &fake_b,
            &mut |_| true,
            &mut confirmer,
            &fast_options(),
        )
        .await;
        assert_eq!(outcome_a, ExecOutcome::Completed { steps: 3 });
        assert_eq!(outcome_a, outcome_b);
        assert_eq!(transcript_a, transcript_b);
        assert_eq!(fake_a.gcode_sent(), fake_b.gcode_sent());
        assert!(asked.lock().expect("asked").is_empty());
        for absent in ["confirm-pause", "confirm-answer", "advisory"] {
            assert!(!transcript_a.contains(absent), "{absent}: {transcript_a}");
        }
    }

    // --- The two acceleration slots --------------------------------------

    /// A plan that exercises BOTH slots: a machine-accel record/clamp,
    /// then the touch clamp inside it, then both restores.
    fn two_slot_accel_plan() -> RecoveryPlan {
        use plr_recovery::{MACHINE_ACCEL_PLACEHOLDER, RESTORE_ACCEL_PLACEHOLDER};
        let mut plan = test_plan();
        plan.steps = vec![
            RecoveryStep {
                id: 1,
                phase: Phase::RecoveryAccel,
                summary: "recovery accel".to_owned(),
                commands: vec!["SET_VELOCITY_LIMIT ACCEL=1000".to_owned()],
                pre_verify: vec![],
                verify: vec![],
                compute: Some(RuntimeComputation::RecordMachineAccel),
                cleanup_commands: vec![format!(
                    "SET_VELOCITY_LIMIT ACCEL={MACHINE_ACCEL_PLACEHOLDER}"
                )],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::RecoveryAccelFailed,
                },
            },
            RecoveryStep {
                id: 2,
                phase: Phase::AccelClamp,
                summary: "touch clamp".to_owned(),
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
                id: 3,
                phase: Phase::Probe,
                summary: "touch".to_owned(),
                commands: vec!["PLR_TOUCH SAMPLES=3".to_owned()],
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
                id: 4,
                phase: Phase::AccelRestore,
                summary: "restore the pre-touch accel".to_owned(),
                commands: vec![format!(
                    "SET_VELOCITY_LIMIT ACCEL={RESTORE_ACCEL_PLACEHOLDER}"
                )],
                pre_verify: vec![],
                verify: vec![],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::AccelRestoreFailed,
                },
            },
            RecoveryStep {
                id: 5,
                phase: Phase::RecoveryAccelRestore,
                summary: "restore the machine accel".to_owned(),
                commands: vec![format!(
                    "SET_VELOCITY_LIMIT ACCEL={MACHINE_ACCEL_PLACEHOLDER}"
                )],
                pre_verify: vec![],
                verify: vec![],
                compute: None,
                cleanup_commands: vec![],
                on_failure: FailureAction::Abort {
                    reason: AbortReason::RecoveryAccelRestoreFailed,
                },
            },
        ];
        plan
    }

    #[tokio::test]
    async fn both_accel_slots_restore_independently_on_the_success_path() {
        let plan = two_slot_accel_plan();
        let fake = FakeMoonraker::spawn(accel_fake_handler(Some(0.75), false)).await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        assert_eq!(outcome, ExecOutcome::Completed { steps: 5 }, "{transcript}");
        assert_eq!(
            fake.gcode_sent(),
            vec![
                // machine 3000 -> recovery 1000 -> touch 100 ...
                "SET_VELOCITY_LIMIT ACCEL=1000",
                "SET_VELOCITY_LIMIT ACCEL=100",
                "PLR_TOUCH SAMPLES=3",
                // ... back to the value in force before the touch (the
                // RECOVERY accel, not the machine's) ...
                "SET_VELOCITY_LIMIT ACCEL=1000",
                // ... and finally back to the machine's own.
                "SET_VELOCITY_LIMIT ACCEL=3000",
            ],
            "one slot would have collapsed these two restores into one"
        );
        assert!(transcript.contains("record-machine-accel"), "{transcript}");
    }

    #[tokio::test]
    async fn both_accel_slots_restore_in_reverse_order_on_abort() {
        // The touch median is null: the probe post-verify fails at step 3
        // and both registered cleanups run, newest first.
        let plan = two_slot_accel_plan();
        let fake = FakeMoonraker::spawn(accel_fake_handler(None, false)).await;
        let (outcome, transcript) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted {
            step_id, reason, ..
        } = outcome
        else {
            panic!("expected abort, got {outcome:?}");
        };
        assert_eq!(step_id, 3);
        assert_eq!(reason, "probe-no-trigger", "cleanups must not mask it");
        assert_eq!(
            fake.gcode_sent(),
            vec![
                "SET_VELOCITY_LIMIT ACCEL=1000",
                "SET_VELOCITY_LIMIT ACCEL=100",
                "PLR_TOUCH SAMPLES=3",
                // Reverse registration order: the touch clamp's restore
                // first, then the machine's.
                "SET_VELOCITY_LIMIT ACCEL=1000",
                "SET_VELOCITY_LIMIT ACCEL=3000",
            ],
            "{transcript}"
        );
    }

    #[tokio::test]
    async fn a_machine_accel_placeholder_without_a_record_refuses_to_send() {
        let mut plan = two_slot_accel_plan();
        // Strip the recording computation: the restore step can no longer
        // resolve, and must refuse rather than send a literal placeholder.
        plan.steps[0].compute = None;
        plan.steps[0].cleanup_commands = vec![];
        let fake = FakeMoonraker::spawn(accel_fake_handler(Some(0.75), false)).await;
        let (outcome, _) = run(&plan, &fake, &mut |_| true).await;
        let ExecOutcome::Aborted { step_id, cause, .. } = outcome else {
            panic!("expected abort, got {outcome:?}");
        };
        assert_eq!(step_id, 5);
        assert!(matches!(cause, StopCause::ComputeFailed(_)), "{cause:?}");
        assert!(!fake
            .gcode_sent()
            .iter()
            .any(|c| c.contains("{machine_accel}")));
    }

    #[test]
    fn every_step_failure_variant_explains_itself() {
        // Exhaustive with no catch-all in the impl; this asserts the arms
        // somebody wrote are usable and correctly tiered.
        for failure in [
            StepFailure::ProbeTriggeredEarly,
            StepFailure::NoTrigger,
            StepFailure::MoveOutOfRange,
            StepFailure::Unknown,
        ] {
            let d = plr_recovery::Diagnose::diagnosis(&failure);
            assert_eq!(d.tier, plr_recovery::Tier::Hard, "{failure:?}");
            assert_eq!(d.override_key, None, "a step failure is never overridable");
            assert!(!d.what.trim().is_empty(), "{failure:?}");
            assert!(!d.why.trim().is_empty(), "{failure:?}");
            assert!(d.suggested_fix.len() > 20, "{failure:?}");
        }
    }

    #[tokio::test]
    async fn an_aborting_command_error_carries_its_diagnosis_into_the_transcript() {
        let fake = FakeMoonraker::spawn(|method, params| {
            if method == "printer.gcode.script" {
                Err((400, "No trigger on probe after full movement".to_owned()))
            } else {
                happy_handler(method, params)
            }
        })
        .await;
        let (_, transcript) = run(&test_plan(), &fake, &mut |_| true).await;
        assert!(transcript.contains("probe_no_trigger"), "{transcript}");
        assert!(transcript.contains("envelope_margin"), "{transcript}");
        assert!(transcript.contains("\"tier\":\"hard\""), "{transcript}");
    }

    // ---- resume-preview reposition loop (design §D.2, §10 attacks) -----

    /// A preview confirmer that plays a scripted sequence of answers.
    struct ScriptedPreviewConfirmer {
        answers: std::collections::VecDeque<PreviewAnswer>,
    }
    impl Confirmer for ScriptedPreviewConfirmer {
        fn confirm<'a>(
            &'a mut self,
            _point: &'a ConfirmPoint,
        ) -> Pin<Box<dyn Future<Output = ConfirmAnswer> + Send + 'a>> {
            Box::pin(async { ConfirmAnswer::Abort })
        }
        fn confirm_preview<'a>(
            &'a mut self,
            _point: &'a ConfirmPoint,
        ) -> Pin<Box<dyn Future<Output = PreviewAnswer> + Send + 'a>> {
            let answer = self.answers.pop_front().unwrap_or(PreviewAnswer::Abort);
            Box::pin(async move { answer })
        }
    }

    /// A writer that records every binding it is asked to materialise.
    #[derive(Clone, Default)]
    struct RecordingWriter {
        offsets: Arc<Mutex<Vec<u64>>>,
    }
    impl RecoveryFileWriter for RecordingWriter {
        fn write_for(&mut self, binding: &PreviewBinding) -> Result<(), String> {
            self.offsets.lock().unwrap().push(binding.tail_offset);
            Ok(())
        }
    }

    /// An exclusivity fake that counts how many times it was re-checked —
    /// the §10 attack #2 instrument (calls must equal step sends +
    /// repositions, so a preview that reasserts once fails this).
    #[derive(Clone, Default)]
    struct CountingExclusivity {
        count: Arc<Mutex<u32>>,
    }
    impl Exclusivity for CountingExclusivity {
        fn recheck<'a>(
            &'a mut self,
            _client: &'a mut MoonrakerClient,
            _budget: Duration,
        ) -> Pin<Box<dyn Future<Output = Result<(), String>> + Send + 'a>> {
            *self.count.lock().unwrap() += 1;
            Box::pin(async { Ok(()) })
        }
    }

    fn preview_stop(index: u32, offset: u64, resume_offset: u64) -> PreviewStop {
        PreviewStop {
            index,
            offset,
            resume_offset,
            xy: [10.0 + f64::from(index), 20.0],
            z: 0.4,
            layer: Some(1),
            feature: FeatureClass::InternalInfill,
            on_infill: true,
            is_candidate: true,
        }
    }

    /// A three-stop preview spec; bindings' tail offsets are 100/200/300.
    fn preview_spec() -> PreviewSpec {
        PreviewSpec {
            stops: vec![
                preview_stop(0, 100, 100),
                preview_stop(1, 200, 200),
                preview_stop(2, 300, 300),
            ],
            representatives: vec![0, 2],
            first_index: 0,
            mid_index: 1,
            last_index: 2,
            default_index: 2,
            bindings: vec![
                PreviewBinding {
                    tail_offset: 100,
                    entry_commands: vec!["G1 X10 Y20".to_owned()],
                },
                PreviewBinding {
                    tail_offset: 200,
                    entry_commands: vec!["G1 X11 Y20".to_owned()],
                },
                PreviewBinding {
                    tail_offset: 300,
                    entry_commands: vec!["G1 X12 Y20".to_owned()],
                },
            ],
            hover_target_z: 0.5,
            z_max: Some(300.0),
            cool_nozzle_temp: 150.0,
            travel_feed: 6000.0,
        }
    }

    fn resume_preview_step(id: u32) -> RecoveryStep {
        RecoveryStep {
            id,
            phase: Phase::ResumePreview,
            summary: "preview".to_owned(),
            commands: vec![
                "G90".to_owned(),
                "G1 Z{park_z} F1200".to_owned(),
                "M104 S150".to_owned(),
            ],
            pre_verify: vec![],
            // No post-verify: the fake's position (0.75) already exceeds
            // the hover target (0.5), so HoverPlane resolves to 0.75 and
            // the lift is a no-op — the loop is what this test exercises.
            verify: vec![],
            compute: Some(RuntimeComputation::HoverPlane {
                target_z: 0.5,
                z_max: Some(300.0),
            }),
            cleanup_commands: vec![],
            on_failure: FailureAction::Abort {
                reason: AbortReason::ResumePreviewFailed,
            },
        }
    }

    fn shifted_frame_step(id: u32) -> RecoveryStep {
        RecoveryStep {
            id,
            phase: Phase::ShiftedFrame,
            summary: "shifted".to_owned(),
            commands: vec!["SET_KINEMATIC_POSITION Z=1".to_owned()],
            pre_verify: vec![],
            verify: vec![],
            compute: None,
            cleanup_commands: vec![],
            on_failure: FailureAction::Abort {
                reason: AbortReason::ShiftedFrameNotDeclared,
            },
        }
    }

    fn preview_plan(steps: Vec<RecoveryStep>) -> RecoveryPlan {
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
            .expect("envelope"),
            resume_file: "x_RECOVERY.gcode".to_owned(),
            resume_offset: 300,
            requires_clean_nozzle_confirmation: false,
            recovery_file: plr_recovery::RecoveryFileSpec::default(),
            debug_confirm_each_step: false,
            confirm_timeout_s: None,
            gcode_barrier_timeout_s: None,
            preview: Some(preview_spec()),
            warnings: vec![],
        }
    }

    async fn run_preview(
        plan: &RecoveryPlan,
        fake: &FakeMoonraker,
        confirmer: &mut dyn Confirmer,
        exclusivity: &mut dyn Exclusivity,
        writer: &mut dyn RecoveryFileWriter,
    ) -> ExecOutcome {
        let mut client = MoonrakerClient::connect(&fake.url(), Duration::from_secs(5))
            .await
            .unwrap();
        let mut buffer = Vec::new();
        let mut transcript = Transcript::new(&mut buffer);
        execute(
            plan,
            &mut client,
            &fast_options(),
            &mut |_| true,
            confirmer,
            &mut NoFrameGuard,
            exclusivity,
            writer,
            &mut transcript,
        )
        .await
    }

    #[tokio::test]
    async fn preview_accept_writes_the_chosen_stop_once_and_rechecks_per_reposition() {
        let fake = FakeMoonraker::spawn(happy_handler).await;
        // Open on last (index 2); nudge -1 twice to index 0, then accept.
        let mut confirmer = ScriptedPreviewConfirmer {
            answers: [
                PreviewAnswer::Nudge(-1),
                PreviewAnswer::Nudge(-1),
                PreviewAnswer::Accept,
            ]
            .into(),
        };
        let writer = RecordingWriter::default();
        let excl = CountingExclusivity::default();
        let plan = preview_plan(vec![resume_preview_step(1)]);
        let outcome = run_preview(
            &plan,
            &fake,
            &mut confirmer,
            &mut excl.clone(),
            &mut writer.clone(),
        )
        .await;
        assert!(matches!(outcome, ExecOutcome::Completed { .. }));
        // Writer called EXACTLY once, with the accepted stop's tail offset
        // (index 0 → 100), never the default (index 2 → 300).
        assert_eq!(*writer.offsets.lock().unwrap(), vec![100]);
        // One step recheck + one recheck per reposition (3 repositions:
        // cursor 2, 1, 0). A preview that reasserted exclusivity once would
        // score 2, not 4 (§10 attack #2).
        assert_eq!(*excl.count.lock().unwrap(), 4);
    }

    #[tokio::test]
    async fn preview_abort_invalidates_the_frame_and_never_writes() {
        let fake = FakeMoonraker::spawn(happy_handler).await;
        let mut confirmer = ScriptedPreviewConfirmer {
            answers: [PreviewAnswer::Abort].into(),
        };
        let writer = RecordingWriter::default();
        // ShiftedFrame (id 1) precedes ResumePreview (id 2): the abort sits
        // past the frame declare, so frame_invalid MUST be true (§10 #4).
        let plan = preview_plan(vec![shifted_frame_step(1), resume_preview_step(2)]);
        let outcome = run_preview(
            &plan,
            &fake,
            &mut confirmer,
            &mut NoExclusivity,
            &mut writer.clone(),
        )
        .await;
        let ExecOutcome::Aborted {
            frame_invalid,
            phase,
            ..
        } = outcome
        else {
            panic!("expected an abort, got {outcome:?}");
        };
        assert!(frame_invalid, "preview abort must invalidate the Z frame");
        assert_eq!(phase, "resume-preview");
        assert!(
            writer.offsets.lock().unwrap().is_empty(),
            "an aborted preview must never write a recovery file"
        );
    }

    #[tokio::test]
    async fn dry_run_holds_a_writer_that_cannot_write() {
        // The type fact (§10 attack #5): NoopFileWriter — the dry-run
        // path's writer — has no path, no bytes, and its write is a no-op.
        let mut writer = NoopFileWriter;
        let binding = PreviewBinding {
            tail_offset: 42,
            entry_commands: vec![],
        };
        assert!(writer.write_for(&binding).is_ok());
    }
}
