//! `plrd recover`: the operator-facing recovery flow and its gates.
//!
//! # The gate stack (in order; every gate is tested)
//!
//! 1. **Dry run is the default** — without `--execute` the command
//!    prints the plan and what would be sent, and returns before any
//!    network client for gcode ever exists (`executor::dry_run` takes
//!    no client; nothing in the dry path connects anywhere).
//! 2. **`--execute` requires `--confirm`** — refused as a usage error
//!    otherwise; still nothing has connected.
//! 3. **Interactive consent** — the plan summary is shown and the
//!    operator must answer `y`/`yes` to an explicit prompt. Declining
//!    aborts before any connection.
//! 4. **Printer must be ready and idle** — queried via Moonraker
//!    (`printer.objects.query`): `webhooks.state == "ready"`,
//!    `print_stats.state` ∈ {standby, complete, cancelled, error}
//!    (i.e. not printing or paused), and `virtual_sdcard.is_active ==
//!    false`. Refusal sends no gcode.
//!    * **Gate 4b — a g-code mutex barrier, then gate 4 again**; see
//!      below. The one thing this sends is the read-only sentinel
//!      [`GCODE_BARRIER_SENTINEL`], which changes nothing.
//! 5. **Machine prerequisites** — already fatal inside the pipeline
//!    (`pipeline::run_pipeline` runs `validate_machine` before any
//!    planning); no plan exists to execute if they failed.
//! 6. **Per-step gate** — with `--step`, every step asks again.
//!
//! Execution writes a JSONL transcript
//! (`recovery-transcript-<unix-seconds>.jsonl` in the WAL directory);
//! refusing to create it refuses to execute.
//!
//! # Gate 4b: why gate 4's answer needs a barrier to be worth anything
//!
//! Gate 4 is a *sample*. Between taking it and the plan's first command
//! landing, anything else with a g-code channel can change the machine —
//! and there is one such thing that is guaranteed to be mid-flight
//! whenever a recovery is started from the console, namely the
//! `[gcode_macro]` the operator's `PLR_RECOVER` may be sitting inside.
//!
//! Klipper runs every externally-submitted script under the g-code mutex
//! (`gcode.run_script`, `../klipper/klippy/gcode.py:239-241`), and a
//! macro's body runs *nested inside* its caller's critical section
//! (`run_script_from_command`, `gcode.py:237-238`, holds no lock of its
//! own). So while the macro that invoked `PLR_RECOVER` still has commands
//! left, the mutex is held for all of them — and dead-reckoning's first
//! `printer.gcode.script` will queue behind them
//! (`ReactorMutex::__enter__`, `../klipper/klippy/reactor.py:77-88`,
//! parks the caller's greenlet rather than failing). Gate 4's answer is
//! therefore stale by exactly the length of the rest of that macro, and
//! the macro can spend that time homing, moving Z, or starting a print.
//!
//! Gate 4b closes that with a *synchronisation* rather than a guess:
//!
//! 1. send [`GCODE_BARRIER_SENTINEL`] — a read-only command whose reply
//!    cannot arrive until dead-reckoning has actually held and released
//!    the g-code mutex, i.e. until everything that was queued ahead of it
//!    has finished;
//! 2. re-run gate 4, whose answer now describes the machine *after* the
//!    other source finished, not before it started.
//!
//! A source that never releases the mutex — the deadlock shape, an old
//! blocking plugin whose macro is itself waiting on this daemon — resolves
//! into the `[plr]` `gcode_barrier_timeout_s` budget (or
//! [`ExecOptions::gcode_barrier_timeout`]) and a refusal, which is the
//! fail-closed direction.
//!
//! ## Once is not enough: the gap between this gate and the first command
//!
//! This gate is still only a sample, and the gap after it is not small. On
//! the socket path it contains the **entire pre-flight confirm pause** —
//! `executor::preflight_confirmations` and the `debug_confirm_each_step`
//! pause both run before any command is issued — which is bounded only by
//! `confirm_timeout_s`, i.e. up to an hour at the top of the permitted band,
//! or unbounded human time at a CLI `--step` prompt. During it an autostart
//! macro or a queued job can begin printing, and answering "continue" would
//! then issue `SET_KINEMATIC_POSITION` and `PROBE` into a running print and
//! report `COMPLETED`.
//!
//! So [`BarrierGate`] re-runs this whole check once per step, immediately
//! before that step's commands go out — see [`crate::executor::Exclusivity`]
//! for the placement argument and for why it cannot refuse the recovery's
//! own progress.
//!
//! **The residual, stated accurately.** What is left is one Moonraker round
//! trip: another g-code source can take the mutex between the re-check and
//! the send. That gap is *microseconds* and cannot be closed from outside
//! Klipper, which is a genuinely different claim from the one this comment
//! used to make — it said a `[delayed_gcode]` between two plan steps was
//! unobservable, and that was false and harmful, because observing it costs
//! exactly one `printer.objects.query` and is now what happens.
//!
//! ## What was considered and rejected: `idle_timeout.state`
//!
//! `idle_timeout.state == "Printing"` looks like the natural detector and
//! is not one, in both directions. Read at source
//! (`../klipper/klippy/extras/idle_timeout.py`):
//!
//! * The transition *into* `"Printing"` that tracks the toolhead is
//!   `handle_sync_print_time` (`idle_timeout.py:98-107`), fired from the
//!   `toolhead:sync_print_time` event, which `ToolHead._calc_print_time`
//!   emits only when it advances `print_time` past the estimate
//!   (`../klipper/klippy/toolhead.py:260-268`). Nothing about *entering* a
//!   macro or taking the mutex does that. A macro whose motion comes *after*
//!   its `PLR_RECOVER` line — which is precisely the dangerous shape — is
//!   still `"Idle"`/`"Ready"` at gate time. **False negative in the unsafe
//!   direction.**
//! * Conversely `_calc_print_time` also runs from
//!   `ToolHead.get_last_move_time` (`toolhead.py:320-326`), which
//!   `register_lookahead_callback` calls on an empty lookahead
//!   (`toolhead.py:526-530`). That is reached by setting a heater
//!   (`extras/heaters.py:360-363`), any fan or pin request
//!   (`extras/fan.py:71-72` → `extras/output_pin.py:65-67`),
//!   `SET_STEPPER_ENABLE` (`extras/stepper_enable.py:106`), `G4`
//!   (`toolhead.py:419`), and TMC/endstop register reads
//!   (`extras/tmc.py:368`, `extras/query_endstops.py:28`). So a periodic
//!   `[delayed_gcode]` that pokes a fan every couple of seconds keeps the
//!   state at `"Printing"` almost continuously — and each poke holds it
//!   there for ~1.5 s, since the exit path needs `lookahead_empty`, a
//!   negative buffer *and* a free mutex (`idle_timeout.py:80-97`).
//!   **False refusal of a legitimate recovery, on a common configuration.**
//! * And there is a *second* assignment of `"Printing"` that has nothing to
//!   do with the toolhead at all: `transition_idle_state` sets it while it
//!   runs the idle g-code (`idle_timeout.py:46-49`) and only then settles to
//!   `"Idle"` (`:56`). So the field also reads `"Printing"` for the duration
//!   of `TURN_OFF_HEATERS` + `M84` — the moment the machine is *becoming*
//!   maximally idle. Another transient false positive, and it lands exactly
//!   where an operator is most likely to start a recovery: on a printer that
//!   has been sitting untouched.
//!
//! Boot and `FIRMWARE_RESTART` both land on `"Idle"` — the field is
//! initialised there (`idle_timeout.py:32`) and a firmware restart builds
//! a fresh object — so those two edges would have been safe; and gate 4
//! runs exactly once, before anything is sent, so it could not have
//! refused this daemon's own later steps. Neither of those redeems the two
//! failures above.
//!
//! The one field that *would* answer the question, `GCodeMacro.in_script`
//! (`../klipper/klippy/extras/gcode_macro.py:190-200`), is not published:
//! that object's `get_status` returns only its variables
//! (`gcode_macro.py:172-173`). Detecting "called from inside a macro" is
//! therefore only possible in-process, i.e. in the Klipper plugin.

use std::io::{BufRead, Write};
use std::path::Path;
use std::time::Duration;

use crate::config::Config;
use crate::executor::{
    dry_run, execute, AbortConfirmer, Confirmer, ExecOptions, ExecOutcome, FrameGuard, Transcript,
};
use crate::moonraker::MoonrakerClient;
use crate::pipeline::{run_pipeline, PipelineOutcome, PlanBundle};
use crate::{EXIT_OK, EXIT_RUNTIME, EXIT_USAGE};

/// Flags for one `plrd recover` invocation.
#[derive(Debug, Clone)]
pub struct RecoverOptions {
    /// Execute instead of dry-running.
    pub execute: bool,
    /// The second consent flag `--execute` requires.
    pub confirm: bool,
    /// Ask before every step.
    pub step: bool,
    /// Executor timing (tests shrink these).
    pub exec_options: ExecOptions,
    /// Moonraker connect/call timeout.
    pub connect_timeout: Duration,
}

impl RecoverOptions {
    /// Production options for the given CLI flags.
    #[must_use]
    pub fn new(execute: bool, confirm: bool, step: bool) -> Self {
        Self {
            execute,
            confirm,
            step,
            exec_options: ExecOptions::default(),
            connect_timeout: Duration::from_secs(10),
        }
    }
}

/// Entry point for the subcommand: pipeline, then the gate stack.
pub fn run_recover(
    config_path: &Path,
    options: &RecoverOptions,
    stdin: &mut (dyn BufRead + Send),
    out: &mut (dyn Write + Send),
) -> u8 {
    let config = match Config::load(config_path) {
        Ok(config) => config,
        Err(e) => {
            let _ = writeln!(out, "plrd recover: {e}");
            return EXIT_RUNTIME;
        }
    };
    let outcome = match run_pipeline(&config, out) {
        Ok(outcome) => outcome,
        Err(e) => {
            let _ = writeln!(out, "plrd recover: {e}");
            return EXIT_RUNTIME;
        }
    };
    drive(&outcome, &config, options, stdin, out)
}

/// The gate stack over a pipeline outcome (split from [`run_recover`]
/// so the gates are testable with hand-built plans).
pub(crate) fn drive(
    outcome: &PipelineOutcome,
    config: &Config,
    options: &RecoverOptions,
    stdin: &mut (dyn BufRead + Send),
    out: &mut (dyn Write + Send),
) -> u8 {
    macro_rules! say {
        ($($arg:tt)*) => { let _ = writeln!(out, $($arg)*); };
    }
    let bundle = match outcome {
        PipelineOutcome::CleanShutdown => {
            say!("recover: the WAL ends with a CLEAN print end; nothing to recover.");
            return EXIT_OK;
        }
        // The log ended torn, but the print had finished: report the cause
        // accurately instead of claiming the log ended cleanly, and name
        // the end-sequence commands that did not run so the operator can
        // decide about them. Deliberately NOT offered for execution — an
        // end macro homes and moves Z, and no part of the plan's envelope
        // or pre-flight analysis covers an opaque macro body.
        PipelineOutcome::Complete(report) => {
            report_completion(report, out);
            return EXIT_OK;
        }
        PipelineOutcome::MachineRejected(rejection) => {
            say!("recover: REFUSED — machine prerequisites failed:");
            for failure in &rejection.failures {
                say!("  - {failure}");
            }
            say!("  Fix the machine/config, set machine.validated_config_hash to the");
            say!("  computed hash printed above once re-validated, and retry.");
            return EXIT_RUNTIME;
        }
        PipelineOutcome::ManualFallback(reason) => {
            say!("recover: automation declined — {reason}");
            say!("  Manual recovery is required; the report above is the evidence.");
            return EXIT_RUNTIME;
        }
        PipelineOutcome::NotPossible(reason) => {
            say!("recover: not possible — {reason}");
            return EXIT_RUNTIME;
        }
        PipelineOutcome::Plan(bundle) => bundle,
    };

    let dry = dry_run(&bundle.plan);
    say!("{}", dry.rendered);
    say!(
        "recover: plan has {} steps, {} commands; resume {} @ byte {}",
        bundle.plan.steps.len(),
        dry.would_send.len(),
        bundle.plan.resume_file,
        bundle.plan.resume_offset,
    );

    // Gate 1: dry run by default. This path provably cannot send: no
    // Moonraker client is ever constructed on it — and it MUST NOT write
    // the recovery file (only a preview is rendered).
    if !options.execute {
        preview_recovery_file(bundle, out);
        // A fresh dry run over a newly-generated plan clears any
        // frame-invalidation marker (deliverable 5): the operator is now
        // reviewing a fresh plan that re-establishes the frame from
        // scratch, so a conscious re-execute is safe to re-enable.
        if crate::detect::read_frame_invalid(&config.wal_dir).is_some() {
            crate::detect::clear_frame_invalid(&config.wal_dir);
            say!(
                "recover: cleared the stale frame-invalid marker; this fresh plan re-establishes \
                 the Z frame from scratch, so --execute is re-enabled after review."
            );
        }
        say!("recover: DRY RUN — nothing was sent. Re-run with --execute --confirm");
        say!("         to execute after review.");
        return EXIT_OK;
    }
    // Gate 2: --confirm.
    if !options.confirm {
        say!("recover: REFUSED — --execute requires --confirm.");
        return EXIT_USAGE;
    }
    // Gate 3: interactive consent.
    say!("recover: about to EXECUTE the plan above on the printer.");
    let _ = write!(out, "Execute this recovery on the printer? [y/N] ");
    let _ = out.flush();
    if !read_yes(stdin) {
        say!("recover: declined by operator; nothing was sent.");
        return EXIT_RUNTIME;
    }

    let runtime = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            say!("recover: cannot build async runtime: {e}");
            return EXIT_RUNTIME;
        }
    };
    let mut gate = PromptGate {
        stdin,
        step_mode: options.step,
    };
    // The CLI answers every confirm-point with "abort" (the default,
    // fail-closed behaviour). Interactive confirmation is the control
    // socket's job: the CLI's own consent channel is a blocking stdin
    // read, and `stdin` is already borrowed by the per-step gate below —
    // a second interactive borrow of the same reader would be a lie about
    // who is being asked what. An operator who wants to be consulted runs
    // the recovery through the plugin, which speaks the socket protocol.
    runtime.block_on(execute_with_gates(
        bundle,
        config,
        &options.exec_options,
        options.connect_timeout,
        &mut gate,
        &mut AbortConfirmer,
        out,
    ))
}

/// Per-step confirmation policy for [`execute_with_gates`]. The CLI
/// prompts on the operator's terminal ([`PromptGate`]); the control
/// socket has no interactive channel and always proceeds
/// ([`AutoGate`]) — its consent is the request's explicit
/// `confirm: true` flag, and per-step mode is rejected up front as
/// CLI-only.
/// `Send` supertrait: the control socket drives this from a spawned
/// task, so the whole execution future must be `Send`.
pub(crate) trait StepGate: Send {
    /// Decides whether `step` may run; may write a prompt to `out`.
    fn confirm(&mut self, step: &plr_recovery::RecoveryStep, out: &mut (dyn Write + Send)) -> bool;
}

/// The CLI gate: with `--step`, ask before every step.
struct PromptGate<'a> {
    stdin: &'a mut (dyn BufRead + Send),
    step_mode: bool,
}

impl StepGate for PromptGate<'_> {
    fn confirm(&mut self, step: &plr_recovery::RecoveryStep, out: &mut (dyn Write + Send)) -> bool {
        if !self.step_mode {
            return true;
        }
        let _ = write!(out, "  proceed with step {}? [y/N] ", step.id);
        let _ = out.flush();
        read_yes(self.stdin)
    }
}

/// The production [`FrameGuard`]: writes the frame-invalidation marker
/// into the WAL directory before the shifted-frame declare is issued.
///
/// See [`FrameGuard`] for why this is eager rather than written on the
/// abort path, and [`crate::detect::write_frame_invalid`] for why the
/// write itself is atomic and durable.
pub(crate) struct MarkerFrameGuard {
    wal_dir: std::path::PathBuf,
}

impl FrameGuard for MarkerFrameGuard {
    fn arm(&mut self, step: &plr_recovery::RecoveryStep) -> Result<(), String> {
        let marker = crate::detect::FrameInvalid {
            detected_wall_ns: wall_ns(),
            step_id: step.id,
            phase: step.phase.name().to_owned(),
            // The frame is not "broken" yet — it is about to become
            // unverifiable. An abort later overwrites this with the real
            // reason; if nothing ever runs again, THIS is the record.
            reason: "shifted-frame-declared".to_owned(),
        };
        crate::detect::write_frame_invalid(&self.wal_dir, &marker).map_err(|e| e.to_string())
    }
}

/// Wall-clock nanoseconds since the epoch (cross-platform; the
/// Linux-only hostclock is not available on this path).
fn wall_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX))
}

/// The non-interactive gate used by the control socket (Linux-only
/// caller; the type itself is cross-platform).
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub(crate) struct AutoGate;

impl StepGate for AutoGate {
    fn confirm(
        &mut self,
        _step: &plr_recovery::RecoveryStep,
        _out: &mut (dyn Write + Send),
    ) -> bool {
        true
    }
}

/// Gates 4–6 and execution proper — the SAME stack for the CLI and the
/// control socket: Moonraker reachability, klippy ready + printer
/// idle, transcript-or-refuse, abort on any failed verification. Only
/// the per-step confirmation policy differs (`gate`).
pub(crate) async fn execute_with_gates(
    bundle: &PlanBundle,
    config: &Config,
    exec_options: &ExecOptions,
    connect_timeout: Duration,
    gate: &mut dyn StepGate,
    confirmer: &mut dyn Confirmer,
    out: &mut (dyn Write + Send),
) -> u8 {
    // Frame-invalidation refuse gate (deliverable 5): a prior recovery
    // aborted at/after the shifted-frame declaration left Klipper's Z
    // frame unknown. Refuse a re-execute — for BOTH the CLI and the
    // control socket, since both drive through here — until a fresh dry
    // run regenerates the plan and clears the marker. Refusal connects
    // to nothing and sends no gcode.
    if let Some(marker) = crate::detect::read_frame_invalid(&config.wal_dir) {
        let _ = writeln!(
            out,
            "recover: REFUSED — the printer's Z frame is unknown after an aborted recovery \
             (aborted at step {} [{}]: {}). Re-run a dry run (plrd scan / plrd recover without \
             --execute) for a fresh plan before resuming.",
            marker.step_id, marker.phase, marker.reason
        );
        return EXIT_RUNTIME;
    }
    let mut client = match MoonrakerClient::connect(&config.moonraker_url, connect_timeout).await {
        Ok(client) => client,
        Err(e) => {
            let _ = writeln!(out, "recover: cannot reach Moonraker: {e}");
            return EXIT_RUNTIME;
        }
    };
    // G-code scripts legitimately run for minutes (heat soak, probe).
    client.set_call_timeout(exec_options.temp_timeout);

    // The two filesystem gates come BEFORE the printer gates, deliberately.
    // Both are pure local I/O with no RPC, so a disk failure refuses with
    // "nothing was sent" in the literal sense — not "nothing that changes
    // the machine". Ordering them the other way round would trade that
    // exact wording for nothing: the printer gates would still run, and
    // their sentinel would already be on the wire by the time the disk
    // failed. The cost is that a *printer* refusal now happens with a
    // transcript and a recovery file already on disk, which is handled
    // where that refusal is raised.

    // Transcript file: no transcript, no execution.
    let transcript_path = config.wal_dir.join(format!(
        "recovery-transcript-{}.jsonl",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs())
    ));
    let mut transcript_file = match std::fs::File::create(&transcript_path) {
        Ok(file) => file,
        Err(e) => {
            let _ = writeln!(
                out,
                "recover: REFUSED — cannot create transcript {}: {e}",
                transcript_path.display()
            );
            return EXIT_RUNTIME;
        }
    };
    let _ = writeln!(out, "recover: transcript: {}", transcript_path.display());

    // WriteRecoveryFile phase gate (before step 1): a write failure
    // aborts before any motion — no client has sent anything yet. The
    // gate may re-resolve the file name (and patch the plan's M23) if the
    // planned name was claimed since planning, so it works on a local
    // copy of the bundle which is what actually gets executed.
    let mut bundle = bundle.clone();
    if !write_recovery_file(&mut bundle, &mut transcript_file, out) {
        return EXIT_RUNTIME;
    }
    let bundle = &bundle;

    // Gates 4 and 4b. First point at which anything reaches the printer.
    if let Err(reason) = idle_and_exclusive(
        &mut client,
        barrier_budget(&bundle.plan, exec_options),
        exec_options.temp_timeout,
    )
    .await
    {
        refuse_at_printer_gate(&reason, bundle, &mut transcript_file, out);
        return EXIT_RUNTIME;
    }

    let mut frame_guard = MarkerFrameGuard {
        wal_dir: config.wal_dir.clone(),
    };
    let mut gate_fn = |step: &plr_recovery::RecoveryStep| -> bool {
        let _ = writeln!(
            out,
            "recover: step {} [{}] {}",
            step.id,
            step.phase.name(),
            step.summary
        );
        gate.confirm(step, out)
    };

    let mut exclusivity = BarrierGate {
        restore_timeout: exec_options.temp_timeout,
    };
    let outcome = {
        let mut transcript = Transcript::new(&mut transcript_file);
        execute(
            &bundle.plan,
            &mut client,
            exec_options,
            &mut gate_fn,
            confirmer,
            &mut frame_guard,
            &mut exclusivity,
            &mut transcript,
        )
        .await
    };
    match outcome {
        ExecOutcome::Completed { steps } => {
            let _ = writeln!(
                out,
                "recover: COMPLETED — {steps} steps executed and verified."
            );
            // The pending-recovery announcement is now stale, and any
            // frame-invalidation marker is superseded by a clean resume.
            let _ = std::fs::remove_file(config.wal_dir.join(crate::detect::PENDING_FILE_NAME));
            crate::detect::clear_frame_invalid(&config.wal_dir);
            EXIT_OK
        }
        aborted @ ExecOutcome::Aborted { .. } => report_abort(&aborted, config, out),
    }
}

/// Renders an abort for the operator and re-asserts the frame interlock
/// when the abort landed inside the danger zone.
fn report_abort(outcome: &ExecOutcome, config: &Config, out: &mut (dyn Write + Send)) -> u8 {
    let ExecOutcome::Aborted {
        step_id,
        phase,
        reason,
        cause,
        frame_invalid,
    } = outcome
    else {
        return EXIT_RUNTIME;
    };
    let _ = writeln!(
        out,
        "recover: ABORTED at step {step_id} [{phase}]: {reason} ({cause:?})"
    );
    if *frame_invalid {
        record_frame_invalid(&config.wal_dir, *step_id, phase, reason, out);
    }
    if let crate::executor::StopCause::FrameGuardUnwritable(why) = cause {
        // Name BOTH sides. What this refusal buys is narrow — the shifted
        // probing frame and the probe — and everything before it has
        // already happened. Telling an operator the printer is
        // "untouched" would send them away to fix a disk while a hot
        // nozzle sits over their part with the idle timeout disarmed.
        let _ = writeln!(
            out,
            "recover: REFUSED to declare the shifted Z frame — the frame-invalid \
             interlock could not be written ({why}).\n\
             recover:   AVOIDED: the shifted probing frame was never declared and the \
             probe never ran, so no fabricated Z reference exists and no fresh dry run \
             is needed — retrying is safe once the write works.\n\
             recover:   ALREADY DONE: everything before it, exactly as any abort at this \
             point leaves it — the conservative believed-Z declaration and its lift, XY \
             homing, and the commanded heater targets.\n\
             recover:   *** THE HEATERS MAY STILL BE HOT AND THE IDLE TIMEOUT IS EXTENDED \
             (motors stay energized; nothing will shut down on its own). Do not walk away \
             from the machine. *** \n\
             recover:   Fix the WAL directory {} (it also holds the transcript), then \
             retry or cool the printer down by hand.",
            config.wal_dir.display()
        );
    } else if let crate::executor::StopCause::ExclusivityLost(why) = cause {
        // The operator's problem is not this recovery — it is that
        // something else owns their printer. Say that, and say what this
        // step did NOT do, because the abort landed before the step's
        // commands went out.
        let _ = writeln!(
            out,
            "recover: STOPPED before step {step_id} — {why}\n\
             recover:   Step {step_id} sent nothing. Everything before it ran and is left \
             exactly as any abort at this point leaves it.\n\
             recover:   Something else is driving this printer: an autostart macro, a queued \
             job, or a [gcode_macro] that called PLR_RECOVER and kept going. Let it finish (or \
             cancel it), then run a fresh dry run before resuming — the plan is now stale."
        );
    } else {
        let _ = writeln!(
            out,
            "recover: the printer was left as-is; review the transcript before retrying."
        );
    }
    EXIT_RUNTIME
}

/// Records the frame-invalidation marker after an abort at/after the
/// shifted-frame declare, so a re-execute is refused until a fresh plan
/// is generated (deliverable 5). The abort landed after a
/// `SET_KINEMATIC_POSITION`, leaving Klipper's Z frame unknown.
fn record_frame_invalid(
    wal_dir: &Path,
    step_id: u32,
    phase: &str,
    reason: &str,
    out: &mut (dyn Write + Send),
) {
    // Wall-clock ns since the epoch (cross-platform; the Linux-only
    // hostclock is not available here).
    let detected_wall_ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX));
    let marker = crate::detect::FrameInvalid {
        detected_wall_ns,
        step_id,
        phase: phase.to_owned(),
        reason: reason.to_owned(),
    };
    // A RE-ASSERT, not the only writer. `MarkerFrameGuard` wrote this
    // marker BEFORE the declare was issued, and refused to issue it at
    // all if that write failed — so by the time execution can abort in
    // the danger zone, the interlock is already on disk. This only
    // enriches it with the real abort reason, and if that fails the eager
    // marker still stands and `--execute` is still refused.
    if let Err(e) = crate::detect::write_frame_invalid(wal_dir, &marker) {
        let _ = writeln!(
            out,
            "recover: note — could not update the frame-invalid marker with the abort \
             reason ({e}); the marker written before the declare still stands, so \
             --execute remains refused."
        );
    }
    let _ = writeln!(
        out,
        "recover: the printer's Z frame is now UNKNOWN — re-run a dry run \
         (plrd scan / plrd recover without --execute) for a fresh plan before \
         any resume; --execute is refused until then."
    );
}

/// Renders the recovery-file preview for a DRY RUN: the target path, the
/// total size, and the first ~40 lines. NEVER writes the file (dry-run is
/// preview-only).
fn preview_recovery_file(bundle: &PlanBundle, out: &mut (dyn Write + Send)) {
    const PREVIEW_LINES: usize = 40;
    let _ = writeln!(
        out,
        "recover: recovery file (NOT written in dry run): {} ({} bytes)",
        bundle.recovery_file_path.display(),
        bundle.recovery_file_content.len()
    );
    let _ = writeln!(
        out,
        "recover: --- recovery file preview (first {PREVIEW_LINES} lines) ---"
    );
    // The content is raw bytes (the tail may not be UTF-8); the preview is
    // display-only, so a lossy decode is correct HERE and only here.
    let text = String::from_utf8_lossy(&bundle.recovery_file_content);
    for line in text.lines().take(PREVIEW_LINES) {
        let _ = writeln!(out, "  {line}");
    }
    let total = text.lines().count();
    if total > PREVIEW_LINES {
        let _ = writeln!(out, "  ... ({} more lines)", total - PREVIEW_LINES);
    }
    let _ = writeln!(out, "recover: --- end preview ---");
}

/// How many times the write gate re-resolves a fresh recovery-file name
/// when the chosen one was taken between planning and writing.
const RECOVERY_NAME_RETRIES: u32 = 16;

/// Creates `path` exclusively and writes `content`. `Ok(None)` means the
/// path already existed (caller re-resolves); `Ok(Some(()))` means written.
fn create_new_write(path: &std::path::Path, content: &[u8]) -> std::io::Result<Option<()>> {
    match std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
    {
        Ok(mut file) => {
            file.write_all(content)?;
            file.flush()?;
            Ok(Some(()))
        }
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => Ok(None),
        Err(e) => Err(e),
    }
}

/// The `WriteRecoveryFile` phase gate: writes the generated recovery file
/// into the sdcard root BEFORE any step runs (before step 1). A write
/// failure aborts the recovery before any motion; the final path is
/// recorded in the transcript. Returns the written path, or `None` on
/// failure (the caller refuses).
///
/// The name was chosen at plan time by scanning the directory, but the
/// write happens later — so this uses `create_new` (never an
/// unconditional truncate) and, when the chosen name has since appeared,
/// re-resolves a fresh collision-free name and patches the plan's `M23`
/// to match. A file that showed up in between is never clobbered.
fn write_recovery_file(
    bundle: &mut PlanBundle,
    transcript_file: &mut std::fs::File,
    out: &mut (dyn Write + Send),
) -> bool {
    let mut path = bundle.recovery_file_path.clone();
    for attempt in 0..RECOVERY_NAME_RETRIES {
        match create_new_write(&path, &bundle.recovery_file_content) {
            Ok(Some(())) => {
                if path != bundle.recovery_file_path {
                    // The name changed: keep the plan's M23 and the
                    // bundle's path consistent with what was written.
                    let new_name = path
                        .file_name()
                        .map(|n| n.to_string_lossy().into_owned())
                        .unwrap_or_default();
                    let patched = retarget_recovery_file(bundle, &new_name);
                    let _ = writeln!(
                        out,
                        "recover: recovery file name was taken since planning; wrote {new_name} \
                         ({patched} M23 command(s) repointed)"
                    );
                    let _ = writeln!(
                        transcript_file,
                        "{}",
                        serde_json::json!({
                            "event": "recovery-file-renamed",
                            "name": new_name,
                            "m23_repointed": patched,
                        })
                    );
                    bundle.recovery_file_path.clone_from(&path);
                }
                let _ = writeln!(
                    transcript_file,
                    "{}",
                    serde_json::json!({
                        "event": "recovery-file-written",
                        "path": path.display().to_string(),
                        "bytes": bundle.recovery_file_content.len(),
                        "attempt": attempt,
                    })
                );
                let _ = transcript_file.flush();
                let _ = writeln!(
                    out,
                    "recover: wrote recovery file {} ({} bytes)",
                    path.display(),
                    bundle.recovery_file_content.len()
                );
                return true;
            }
            // Taken since planning: re-resolve against the live directory.
            Ok(None) => {
                let taken: std::collections::BTreeSet<String> =
                    std::fs::read_dir(&bundle.sdcard_root)
                        .into_iter()
                        .flatten()
                        .filter_map(Result::ok)
                        .filter_map(|e| e.file_name().into_string().ok())
                        .collect();
                let name = plr_recovery::recovery_file_name(&bundle.recovery_source_name, &|n| {
                    taken.contains(n)
                });
                path = bundle.sdcard_root.join(name);
            }
            Err(e) => {
                let _ = writeln!(
                    out,
                    "recover: REFUSED — cannot write recovery file {}: {e}; nothing was sent.",
                    path.display()
                );
                return false;
            }
        }
    }
    let _ = writeln!(
        out,
        "recover: REFUSED — could not claim a free recovery file name in {} \
         under {}; nothing was sent.",
        RECOVERY_NAME_RETRIES,
        bundle.sdcard_root.display()
    );
    false
}

/// Points the plan's `M23` (and the plan/spec name fields) at `new_name`.
///
/// Retargets EVERY `M23`, not only one whose argument still equals the
/// old name: an exact-match loop silently no-ops if the two ever drift,
/// which would leave `M23` naming the squatter file this retry just
/// refused to overwrite — the recovery would then resume someone else's
/// g-code. Returns how many `M23` commands were repointed; the caller
/// reports the count in its operator message and the transcript so a plan
/// that carried none is visible after the fact.
fn retarget_recovery_file(bundle: &mut PlanBundle, new_name: &str) -> usize {
    let mut patched = 0;
    for step in &mut bundle.plan.steps {
        for command in &mut step.commands {
            if command.starts_with("M23 ") {
                *command = format!("M23 {new_name}");
                patched += 1;
            }
        }
    }
    new_name.clone_into(&mut bundle.plan.recovery_file.name);
    new_name.clone_into(&mut bundle.plan.resume_file);
    patched
}

/// The g-code mutex barrier's sentinel command.
///
/// `M110` (set line number) is chosen because it is inert *by
/// construction*, which is the only property that matters here:
///
/// * it goes through `gcode.run_script`, so it takes and releases the
///   g-code mutex — that is the whole point
///   (`../klipper/klippy/webhooks.py:447-448`, `gcode.py:239-241`);
/// * **its handler is literally `pass`** (`gcode.py:338-340`). It changes
///   no state, queues no move, touches no toolhead or heater, and emits no
///   output at all;
/// * it is registered in the block of commands that exist *before* the
///   config is loaded (`gcode.py:117-124`), so it cannot fail because of
///   how the operator's printer is configured.
///
/// # Why not `M115`, which this used to be
///
/// Two reasons, both found in review, and both disqualifying:
///
/// * **It is not unconditionally inert.** `[gcode_macro M115]` with
///   `rename_existing: M115.1` is legal Klipper config — the rename passes
///   the `is_traditional_gcode` consistency check (`gcode.py:137-142`) — so
///   on such a printer the "read-only" barrier would run an arbitrary macro
///   body, possibly homing or heating, as the recovery's first act, and
///   that body could `gcmd.error` the barrier into a refusal.
/// * **It is not silent.** The API socket calls `run_script` with
///   `need_ack=False`, so `gcmd.ack` returns false and `cmd_M115` falls
///   through to `respond_info` (`gcode.py:344-351`): every recovery
///   attempt, *including every refusal*, would drop an unexplained
///   firmware banner into the operator's console and Moonraker's g-code
///   store.
///
/// `M110` is renameable in principle too — nothing in Klipper forbids
/// `[gcode_macro M110]` — so "inert" is a property of stock Klipper plus a
/// config nobody writes, not a proof. The barrier's failure direction is a
/// refusal, so a hijacked sentinel that errors costs a false refusal rather
/// than a hidden hazard; one that silently succeeds costs nothing the
/// following ready-and-idle re-sample does not re-check.
pub(crate) const GCODE_BARRIER_SENTINEL: &str = "M110";

/// Gates 4 and 4b together, in the order that makes each worth having.
///
/// The `Err` string is the whole operator-facing reason, so the three
/// refusals stay distinguishable in the report and in the tests: a printer
/// that was busy all along, a mutex nobody released, and a printer that
/// stopped being idle while we waited are three different diagnoses and
/// three different things for the operator to do.
async fn idle_and_exclusive(
    client: &mut MoonrakerClient,
    barrier_budget: Duration,
    restore_timeout: Duration,
) -> Result<(), String> {
    // Gate 4, FIRST: a plainly busy printer is refused without this daemon
    // putting a single byte of g-code on the wire.
    if let Err(reason) = printer_ready_and_idle(client).await {
        return Err(format!("{reason}; nothing was sent."));
    }
    // Gate 4b: synchronise with Klipper's g-code mutex, then re-sample
    // gate 4 (module docs). Until the sentinel returns, gate 4's answer
    // describes the machine as it was *before* whatever was already holding
    // the mutex — most importantly the `[gcode_macro]` a console
    // `PLR_RECOVER` is nested in — got to run its remaining commands.
    if let Err(reason) = gcode_mutex_barrier(client, barrier_budget, restore_timeout).await {
        return Err(format!(
            "{reason}. Nothing but the read-only {GCODE_BARRIER_SENTINEL} was sent. If this \
             recovery was started from a [gcode_macro], that macro still holds the mutex: \
             call PLR_RECOVER as the macro's LAST command, or run it directly."
        ));
    }
    if let Err(reason) = printer_ready_and_idle(client).await {
        return Err(format!(
            "the printer stopped being idle while dead-reckoning was waiting for Klipper's \
             g-code mutex ({reason}). Another g-code source ran between the check and now — \
             most likely the [gcode_macro] this recovery was started from. Nothing but the \
             read-only {GCODE_BARRIER_SENTINEL} was sent."
        ));
    }
    Ok(())
}

/// Refuses at gates 4/4b, after the filesystem gates have already run.
///
/// Two clean-ups the earlier ordering did not need, both consequences of
/// doing the disk work first (see the ordering note in
/// [`execute_with_gates`]):
///
/// * **journal it.** The transcript exists by now, and a gate refusal used
///   to be recorded nowhere but stdout — which on the socket path means it
///   survived only as long as the response the plugin printed.
/// * **remove the recovery file.** Nothing will select it, and leaving it
///   behind would make the next attempt re-resolve to a `-2` name and
///   litter the sdcard root one file per refusal. `write_recovery_file`
///   created it with `create_new`, so this cannot remove somebody else's
///   file; if the removal fails, the existing collision handling copes,
///   which is already tested.
fn refuse_at_printer_gate(
    reason: &str,
    bundle: &PlanBundle,
    transcript_file: &mut std::fs::File,
    out: &mut (dyn Write + Send),
) {
    let _ = writeln!(
        transcript_file,
        "{}",
        serde_json::json!({"event": "refused", "gate": "idle-and-exclusive", "reason": reason})
    );
    let _ = transcript_file.flush();
    let _ = std::fs::remove_file(&bundle.recovery_file_path);
    let _ = writeln!(out, "recover: REFUSED — {reason}");
}

/// The barrier budget in force for `plan`: the operator's `[plr]`
/// `gcode_barrier_timeout_s` when set, else the daemon's default.
///
/// The same rule the executor applies to its own per-step re-checks, in one
/// function so the pre-execution gate and the per-step gate cannot disagree
/// about how long the operator asked to wait.
fn barrier_budget(plan: &plr_recovery::RecoveryPlan, exec_options: &ExecOptions) -> Duration {
    crate::executor::plan_duration_or(
        plan.gcode_barrier_timeout_s,
        exec_options.gcode_barrier_timeout,
    )
}

/// The production [`Exclusivity`]: [`idle_and_exclusive`], re-run before
/// every step's commands go out.
///
/// It is the *same* check as gates 4 and 4b, deliberately — a second,
/// weaker predicate for the per-step case would be a second thing to keep
/// true. `restore_timeout` is what the client's per-call budget is put back
/// to afterwards, so a barrier does not silently shorten the multi-minute
/// budget a heat-soak step needs.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub(crate) struct BarrierGate {
    pub(crate) restore_timeout: Duration,
}

impl crate::executor::Exclusivity for BarrierGate {
    fn recheck<'a>(
        &'a mut self,
        client: &'a mut MoonrakerClient,
        budget: Duration,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), String>> + Send + 'a>> {
        let restore = self.restore_timeout;
        Box::pin(async move { idle_and_exclusive(client, budget, restore).await })
    }
}

/// Gate 4b's first half: block until dead-reckoning has held and released
/// Klipper's g-code mutex, so gate 4 can be re-sampled against a machine
/// nobody else is mid-command on (see the module docs).
///
/// Fail-closed: any error — timeout, RPC error, dropped connection — is a
/// refusal, because "we could not establish that we have exclusive g-code
/// access" and "we have exclusive g-code access" must not share a branch.
async fn gcode_mutex_barrier(
    client: &mut MoonrakerClient,
    budget: Duration,
    restore_timeout: Duration,
) -> Result<(), String> {
    client.set_call_timeout(budget);
    let result = client.gcode_script(GCODE_BARRIER_SENTINEL).await;
    client.set_call_timeout(restore_timeout);
    result.map_err(|e| {
        format!(
            "could not confirm exclusive g-code access — the read-only \
             {GCODE_BARRIER_SENTINEL} barrier did not complete ({e}), so another g-code \
             source may still hold Klipper's g-code mutex"
        )
    })
}

/// Gate 4 predicate (see module docs for the exact fields, cited from
/// Moonraker `printer.objects.query`).
async fn printer_ready_and_idle(client: &mut MoonrakerClient) -> Result<(), String> {
    let status = client
        .query_objects(&["webhooks", "print_stats", "virtual_sdcard"])
        .await
        .map_err(|e| format!("status query failed: {e}"))?;
    let webhooks_state = status["webhooks"]["state"].as_str().unwrap_or("unknown");
    if webhooks_state != "ready" {
        return Err(format!("klippy state is {webhooks_state:?}, not \"ready\""));
    }
    let print_state = status["print_stats"]["state"].as_str().unwrap_or("unknown");
    if !matches!(print_state, "standby" | "complete" | "cancelled" | "error") {
        return Err(format!(
            "printer is not idle (print_stats.state {print_state:?})"
        ));
    }
    if status["virtual_sdcard"]["is_active"].as_bool() == Some(true) {
        return Err("virtual_sdcard is actively printing".to_owned());
    }
    Ok(())
}

/// Renders a finished print: the accurate cause, and the end-sequence
/// commands that did not run.
///
/// Deliberately does **not** offer to execute them. An end macro homes,
/// drops the bed or moves Z, and none of the plan's envelope or pre-flight
/// analysis covers an opaque macro body.
fn report_completion(report: &crate::pipeline::CompletionReport, out: &mut (dyn Write + Send)) {
    macro_rules! say {
        ($($arg:tt)*) => { let _ = writeln!(out, $($arg)*); };
    }
    say!(
        "recover: the print is COMPLETE — no extrusion remains after byte {} of {}; \
         the {} trailing bytes are the slicer's config-block footer.",
        report.tested_offset,
        report.file_size,
        report.trailing_bytes(),
    );
    let unrun = report.unrun_commands();
    if unrun.is_empty() {
        say!("  Nothing to recover; the print ran its end sequence too.");
    } else {
        say!("  Nothing to recover. These end-sequence commands did not run:");
        say!("    {}", unrun.join(" "));
        say!(
            "  They are NOT offered for execution: an end macro homes, drops the bed \
             or moves Z, and none of the plan's envelope or pre-flight checks apply \
             to a macro body. Run them by hand if you want them."
        );
    }
}

/// Reads one line; only `y`/`yes` (case-insensitive) is consent.
fn read_yes(stdin: &mut dyn BufRead) -> bool {
    let mut line = String::new();
    if stdin.read_line(&mut line).is_err() {
        return false;
    }
    let answer = line.trim();
    answer.eq_ignore_ascii_case("y") || answer.eq_ignore_ascii_case("yes")
}

#[cfg(test)]
mod tests {
    use super::{drive, read_yes, RecoverOptions};
    use crate::config::Config;
    use crate::executor::tests::{happy_handler, test_plan};
    use crate::executor::ExecOptions;
    use crate::pipeline::{machine_config, PipelineOutcome, PlanBundle};
    use crate::testmoon::FakeMoonraker;
    use serde_json::{json, Value};
    use std::io::Cursor;
    use std::path::PathBuf;
    use std::time::Duration;

    fn temp_wal_dir(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-recover-test-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn plan_outcome() -> PipelineOutcome {
        plan_outcome_in(&temp_wal_dir("recfile"))
    }

    /// A plan outcome whose recovery file is written under `dir` (so the
    /// `WriteRecoveryFile` gate has a writable target during execute).
    fn plan_outcome_in(dir: &std::path::Path) -> PipelineOutcome {
        let machine = machine_config(&crate::config::MachineSection::default(), true, None);
        PipelineOutcome::Plan(Box::new(PlanBundle {
            plan: test_plan(),
            file_path: "/g/x.gcode".to_owned(),
            machine,
            recovery_file_content: b"; recovery\nG28 X Y\n".to_vec(),
            recovery_file_path: dir.join("x_RECOVERY.gcode"),
            sdcard_root: dir.to_path_buf(),
            recovery_source_name: "x.gcode".to_owned(),
        }))
    }

    fn test_config(tag: &str, url: &str) -> Config {
        Config {
            wal_dir: temp_wal_dir(tag),
            moonraker_url: url.to_owned(),
            ..Config::default()
        }
    }

    fn fast_recover(execute: bool, confirm: bool, step: bool) -> RecoverOptions {
        RecoverOptions {
            execute,
            confirm,
            step,
            exec_options: ExecOptions {
                verify_timeout: Duration::from_millis(300),
                temp_timeout: Duration::from_millis(300),
                poll_interval: Duration::from_millis(20),
                confirm_timeout: Duration::from_millis(300),
                gcode_barrier_timeout: Duration::from_millis(300),
            },
            connect_timeout: Duration::from_secs(2),
        }
    }

    fn run_drive(
        outcome: &PipelineOutcome,
        config: &Config,
        options: &RecoverOptions,
        input: &str,
    ) -> (u8, String) {
        let mut stdin = Cursor::new(input.as_bytes().to_vec());
        let mut out = Vec::new();
        let code = drive(outcome, config, options, &mut stdin, &mut out);
        (code, String::from_utf8(out).unwrap())
    }

    /// An unreachable URL: any attempt to connect fails fast, so a test
    /// passing with it proves the path made no network connection that
    /// mattered (dry run and pre-connect refusals must succeed).
    const DEAD_URL: &str = "ws://127.0.0.1:9/websocket";

    #[test]
    fn dry_run_is_default_prints_plan_and_never_connects() {
        let config = test_config("dry", DEAD_URL);
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(false, false, false),
            "",
        );
        assert_eq!(code, crate::EXIT_OK, "{output}");
        assert!(output.contains("DRY RUN"), "{output}");
        assert!(output.contains("SET_IDLE_TIMEOUT"), "{output}");
        assert!(output.contains("nothing was sent"), "{output}");
        // No transcript file: nothing executed.
        assert!(std::fs::read_dir(&config.wal_dir).unwrap().next().is_none());
    }

    #[test]
    fn execute_without_confirm_is_a_usage_refusal() {
        let config = test_config("noconfirm", DEAD_URL);
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, false, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_USAGE, "{output}");
        assert!(output.contains("--execute requires --confirm"), "{output}");
    }

    #[test]
    fn prompt_decline_refuses_before_any_connection() {
        let config = test_config("decline", DEAD_URL);
        for answer in ["n\n", "\n", "no\n", "Y es\n"] {
            let (code, output) = run_drive(
                &plan_outcome(),
                &config,
                &fast_recover(true, true, false),
                answer,
            );
            assert_eq!(code, crate::EXIT_RUNTIME, "answer {answer:?}: {output}");
            assert!(output.contains("declined by operator"), "{output}");
        }
    }

    #[test]
    fn printer_not_idle_refuses_with_zero_gcode() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(ps) = v.get_mut("status").and_then(|s| s.get_mut("print_stats")) {
                    *ps = json!({"state": "printing"});
                }
            }
            Ok(v)
        }));
        let config = test_config("notidle", &fake.url());
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(output.contains("not idle"), "{output}");
        assert!(fake.gcode_sent().is_empty(), "refusal must send nothing");
    }

    #[test]
    fn klippy_not_ready_refuses_with_zero_gcode() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(wh) = v.get_mut("status").and_then(|s| s.get_mut("webhooks")) {
                    *wh = json!({"state": "shutdown"});
                }
            }
            Ok(v)
        }));
        let config = test_config("notready", &fake.url());
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(output.contains("klippy state"), "{output}");
        assert!(fake.gcode_sent().is_empty());
    }

    // --- Gate 4b: the g-code mutex barrier -------------------------------
    //
    // The hazard, restated so the tests below can be read against it: gate
    // 4 is a sample, and Klipper holds the g-code mutex for the whole body
    // of a `[gcode_macro]` (`gcode.run_script`,
    // `../klipper/klippy/gcode.py:239-241`, wrapping a nested
    // `run_script_from_command` that takes no lock of its own). So a
    // recovery started from inside a macro passes gate 4 against a machine
    // the macro has not finished changing, and the plan's first command
    // does not land until the macro's last one has.
    //
    // Neither test below restates a constant: each one builds a printer
    // that *behaves* like the hazard and checks that the barrier catches
    // it. Removing the barrier makes both fail (verified: the macro test
    // reaches `COMPLETED` and sends `M24` with the barrier removed).

    /// A macro that starts a print *after* calling `PLR_RECOVER`.
    ///
    /// The printer answers "idle" until the sentinel is served, which is
    /// exactly when the mutex would be handed over — and by then the
    /// macro's `M24` has run. Gate 4's first sample is therefore honest and
    /// stale, and only the re-sample can see it.
    #[test]
    fn a_macro_that_starts_a_print_after_calling_us_is_refused_at_the_barrier() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let printing = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let flag = std::sync::Arc::clone(&printing);
        let fake = rt.block_on(FakeMoonraker::spawn(move |method, params| {
            if method == "printer.gcode.script"
                && params["script"].as_str() == Some(super::GCODE_BARRIER_SENTINEL)
            {
                // The mutex was released to us; the macro had already run
                // its `M24` to get here.
                flag.store(true, std::sync::atomic::Ordering::SeqCst);
                return Ok(json!("ok"));
            }
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" && flag.load(std::sync::atomic::Ordering::SeqCst) {
                if let Some(status) = v.get_mut("status").and_then(Value::as_object_mut) {
                    status.insert("print_stats".to_owned(), json!({"state": "printing"}));
                    status.insert(
                        "virtual_sdcard".to_owned(),
                        json!({"is_active": true, "file_path": "/g/x.gcode", "file_position": 0}),
                    );
                }
            }
            Ok(v)
        }));
        let config = test_config("barrier-macro", &fake.url());
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(
            output.contains("stopped being idle while dead-reckoning was waiting"),
            "{output}"
        );
        assert!(output.contains("[gcode_macro]"), "{output}");
        // The ONLY thing sent is the read-only sentinel: no plan command,
        // and in particular nothing that moves or heats.
        assert_eq!(
            fake.gcode_sent(),
            vec![super::GCODE_BARRIER_SENTINEL.to_owned()],
            "the barrier refusal must send nothing but the sentinel"
        );
        // The refusal is journalled. A transcript exists because the two
        // filesystem gates now run first, and a gate refusal that survived
        // only in stdout was invisible after the fact on the socket path.
        let transcript = std::fs::read_dir(&config.wal_dir)
            .unwrap()
            .filter_map(Result::ok)
            .find(|e| {
                e.file_name()
                    .to_string_lossy()
                    .starts_with("recovery-transcript-")
            })
            .expect("transcript");
        let text = std::fs::read_to_string(transcript.path()).unwrap();
        assert!(text.contains("\"event\":\"refused\""), "{text}");
        assert!(text.contains("idle-and-exclusive"), "{text}");
        // No step ever started.
        assert!(!text.contains("step-start"), "{text}");
        // And the recovery file this run created was removed rather than
        // left to litter the sdcard root one file per refusal.
        assert!(
            !std::fs::read_dir(&config.wal_dir)
                .unwrap()
                .filter_map(Result::ok)
                .any(|e| e.file_name().to_string_lossy().contains("_RECOVERY")),
            "the unused recovery file must not be left behind"
        );
    }

    /// A g-code source that never releases the mutex in time — the
    /// deadlock shape, e.g. a macro that is itself blocked waiting on this
    /// daemon, or a klippy wedged inside somebody else's script.
    ///
    /// The fake runs on **its own runtime on its own thread**, and that is
    /// load-bearing rather than tidiness. A blocking handler on the runtime
    /// under test starves the very task whose timeout is being tested:
    /// `tokio::time::timeout` polls its inner future first, so a client task
    /// that is not polled until after the response has arrived sees the
    /// response and returns `Ok` no matter how long the deadline has been
    /// past. Written that way this test passed the barrier and reported
    /// `COMPLETED` — it was measuring the fake, not the code. A real klippy
    /// holding its own mutex cannot stall this daemon's event loop, so the
    /// isolated runtime is also the faithful model.
    #[test]
    fn a_held_gcode_mutex_times_out_into_a_refusal() {
        use std::sync::{Arc, Mutex};
        let scripts: Arc<Mutex<Vec<String>>> = Arc::default();
        let log = Arc::clone(&scripts);
        let (url_tx, url_rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .expect("fake runtime");
            rt.block_on(async move {
                let fake = FakeMoonraker::spawn(move |method, params| {
                    if method == "printer.gcode.script" {
                        let script = params["script"].as_str().unwrap_or_default().to_owned();
                        log.lock().expect("script log").push(script.clone());
                        if script == super::GCODE_BARRIER_SENTINEL {
                            // Five times the barrier budget below: the
                            // answer cannot arrive in time.
                            std::thread::sleep(Duration::from_secs(2));
                        }
                    }
                    happy_handler(method, params)
                })
                .await;
                url_tx.send(fake.url()).expect("url");
                // Outlive the refusal, then let the fake drop.
                tokio::time::sleep(Duration::from_secs(3)).await;
            });
        });
        let url = url_rx.recv().expect("fake url");
        let config = test_config("barrier-held", &url);
        let (code, output) = run_drive(
            &plan_outcome_in(&config.wal_dir),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(
            output.contains("may still hold Klipper's g-code mutex"),
            "the refusal must name the actual resource: {output}"
        );
        assert!(output.contains("timed out"), "{output}");
        assert!(
            output.contains("PLR_RECOVER as the macro's LAST command"),
            "{output}"
        );
        // The sentinel attempt and nothing else: no plan command ran.
        assert_eq!(
            scripts.lock().unwrap().as_slice(),
            &[super::GCODE_BARRIER_SENTINEL.to_owned()],
            "{output}"
        );
    }

    /// The barrier's other failure mode: klippy answers the sentinel with
    /// an error (`printer.gcode.script` -> `Klippy is shutdown` is
    /// Moonraker's real reply when klippy dies mid-script). "We could not
    /// establish exclusive g-code access" must not share a branch with "we
    /// have it".
    #[test]
    fn a_failed_barrier_sentinel_refuses_rather_than_assuming_success() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(|method, params| {
            if method == "printer.gcode.script"
                && params["script"].as_str() == Some(super::GCODE_BARRIER_SENTINEL)
            {
                return Err((400, "Klippy is shutdown: Lost communication".to_owned()));
            }
            happy_handler(method, params)
        }));
        let config = test_config("barrier-error", &fake.url());
        let (code, output) = run_drive(
            &plan_outcome_in(&config.wal_dir),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(
            output.contains("may still hold Klipper's g-code mutex"),
            "{output}"
        );
        assert!(output.contains("Klippy is shutdown"), "{output}");
        assert_eq!(
            fake.gcode_sent(),
            vec![super::GCODE_BARRIER_SENTINEL.to_owned()],
            "{output}"
        );
    }

    /// The barrier runs where it claims to: after gate 4 and before the
    /// plan's first command.
    #[test]
    fn the_barrier_sentinel_precedes_every_plan_command() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("barrier-order", &fake.url());
        let (code, output) = run_drive(
            &plan_outcome_in(&config.wal_dir),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_OK, "{output}");
        let sent = fake.gcode_sent();
        assert_eq!(
            sent.first().map(String::as_str),
            Some(super::GCODE_BARRIER_SENTINEL),
            "{sent:?}"
        );
        assert!(
            sent.len() > 1,
            "the plan must still have executed: {sent:?}"
        );
    }

    #[test]
    fn happy_path_executes_and_writes_a_transcript() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("happy", &fake.url());
        // Stale pending-recovery file must be cleared on completion.
        std::fs::write(config.wal_dir.join(crate::detect::PENDING_FILE_NAME), b"{}").unwrap();
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, true, false),
            "yes\n",
        );
        assert_eq!(code, crate::EXIT_OK, "{output}");
        assert!(output.contains("COMPLETED"), "{output}");
        // The full send order, spelled out: gate 4b's barrier, then one
        // per-step re-check barrier immediately before each step's commands
        // (`executor::Exclusivity`). Nothing else, and nothing out of order.
        assert_eq!(
            fake.gcode_sent(),
            vec![
                super::GCODE_BARRIER_SENTINEL,
                super::GCODE_BARRIER_SENTINEL,
                "SET_IDLE_TIMEOUT TIMEOUT=86400",
                super::GCODE_BARRIER_SENTINEL,
                "PROBE PROBE_SPEED=1 SAMPLES=1",
                super::GCODE_BARRIER_SENTINEL,
                "SET_KINEMATIC_POSITION Z=12.25",
            ]
        );
        // Transcript exists and carries the story.
        let transcript = std::fs::read_dir(&config.wal_dir)
            .unwrap()
            .filter_map(Result::ok)
            .find(|e| {
                e.file_name()
                    .to_string_lossy()
                    .starts_with("recovery-transcript-")
            })
            .expect("transcript file");
        let text = std::fs::read_to_string(transcript.path()).unwrap();
        assert!(text.contains("plan-start"), "{text}");
        assert!(text.contains("plan-complete"), "{text}");
        // Pending file cleared.
        assert!(!config
            .wal_dir
            .join(crate::detect::PENDING_FILE_NAME)
            .exists());
    }

    #[test]
    fn recovery_file_is_written_before_execution_and_recorded_in_the_transcript() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("recwrite", &fake.url());
        let rec_dir = temp_wal_dir("recwrite-sdcard");
        let outcome = plan_outcome_in(&rec_dir);
        let (code, output) = run_drive(&outcome, &config, &fast_recover(true, true, false), "y\n");
        assert_eq!(code, crate::EXIT_OK, "{output}");
        // The recovery file exists on disk with the generated content.
        let written = std::fs::read_to_string(rec_dir.join("x_RECOVERY.gcode")).unwrap();
        assert!(written.contains("G28 X Y"), "{written}");
        // The transcript records the write, before any command.
        let transcript = std::fs::read_dir(&config.wal_dir)
            .unwrap()
            .filter_map(Result::ok)
            .find(|e| {
                e.file_name()
                    .to_string_lossy()
                    .starts_with("recovery-transcript-")
            })
            .expect("transcript");
        let text = std::fs::read_to_string(transcript.path()).unwrap();
        let write_at = text.find("recovery-file-written").expect("write event");
        let first_send = text.find("\"send\"").expect("a send event");
        assert!(
            write_at < first_send,
            "the recovery file must be written before any command is sent"
        );
        assert!(text.contains("x_RECOVERY.gcode"), "{text}");
    }

    /// Finding 8 regression: the recovery file name is chosen at plan
    /// time but written later. A file that appeared in between must NEVER
    /// be clobbered — the write gate uses `create_new`, re-resolves a
    /// fresh name, and repoints the plan's `M23` at what it actually
    /// wrote.
    #[test]
    fn a_file_appearing_after_planning_is_never_clobbered() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("toctou", &fake.url());
        let rec_dir = temp_wal_dir("toctou-sdcard");
        // Somebody else claimed the planned name between planning and
        // execution, with content that must survive untouched.
        let squatter = rec_dir.join("x_RECOVERY.gcode");
        std::fs::write(&squatter, b"PRECIOUS DO NOT CLOBBER").unwrap();

        // The plan carries a real M23 selecting the PLANNED name, so the
        // retry's repoint has something to patch (a fixture without one
        // would let a silent no-op pass).
        let PipelineOutcome::Plan(mut bundle) = plan_outcome_in(&rec_dir) else {
            panic!("expected plan");
        };
        bundle.plan.steps.push(plr_recovery::RecoveryStep {
            id: u32::try_from(bundle.plan.steps.len() + 1).unwrap(),
            phase: plr_recovery::Phase::RecoveryFileSelect,
            summary: "select the recovery file".to_owned(),
            commands: vec!["M23 x_RECOVERY.gcode".to_owned(), "M24".to_owned()],
            pre_verify: vec![],
            verify: vec![],
            compute: None,
            cleanup_commands: vec![],
            on_failure: plr_recovery::FailureAction::Abort {
                reason: plr_recovery::AbortReason::RecoveryFileSelectFailed,
            },
        });
        bundle.plan.recovery_file.name = "x_RECOVERY.gcode".to_owned();
        let outcome = PipelineOutcome::Plan(bundle);
        let (code, output) = run_drive(&outcome, &config, &fast_recover(true, true, false), "y\n");
        assert_eq!(code, crate::EXIT_OK, "{output}");

        // The pre-existing file is byte-identical: never truncated.
        assert_eq!(
            std::fs::read(&squatter).unwrap(),
            b"PRECIOUS DO NOT CLOBBER"
        );
        // The recovery went to a fresh, re-resolved name.
        let fresh = rec_dir.join("x_RECOVERY-2.gcode");
        assert!(fresh.exists(), "{output}");
        assert!(String::from_utf8(std::fs::read(&fresh).unwrap())
            .unwrap()
            .contains("G28 X Y"));
        assert!(output.contains("name was taken since planning"), "{output}");
        // The M23 actually sent names the file that was WRITTEN — not the
        // squatter the retry refused to overwrite.
        let sent = fake.gcode_sent();
        assert!(
            sent.iter().any(|c| c == "M23 x_RECOVERY-2.gcode"),
            "M23 must name the written file, got {sent:?}"
        );
        assert!(
            !sent.iter().any(|c| c == "M23 x_RECOVERY.gcode"),
            "M23 must NOT name the squatter file: {sent:?}"
        );
        assert!(output.contains("1 M23 command(s) repointed"), "{output}");
        // The transcript records the path actually written.
        let transcript = std::fs::read_dir(&config.wal_dir)
            .unwrap()
            .filter_map(Result::ok)
            .find(|e| {
                e.file_name()
                    .to_string_lossy()
                    .starts_with("recovery-transcript-")
            })
            .expect("transcript");
        let text = std::fs::read_to_string(transcript.path()).unwrap();
        assert!(text.contains("x_RECOVERY-2.gcode"), "{text}");
    }

    #[test]
    fn recovery_file_write_failure_aborts_before_any_gcode() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("recfail", &fake.url());
        // A recovery file path whose parent directory does not exist: the
        // WriteRecoveryFile gate fails and the recovery aborts BEFORE any
        // motion (zero gcode sent).
        let machine = machine_config(&crate::config::MachineSection::default(), true, None);
        let bundle = PlanBundle {
            plan: test_plan(),
            file_path: "/g/x.gcode".to_owned(),
            machine,
            recovery_file_content: b"; recovery\nG28 X Y\n".to_vec(),
            recovery_file_path: std::path::PathBuf::from(
                "/nonexistent-plrd-dir-xyzzy/x_RECOVERY.gcode",
            ),
            sdcard_root: std::path::PathBuf::from("/nonexistent-plrd-dir-xyzzy"),
            recovery_source_name: "x.gcode".to_owned(),
        };
        let (code, output) = run_drive(
            &PipelineOutcome::Plan(Box::new(bundle)),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(output.contains("cannot write recovery file"), "{output}");
        // Literally nothing on the wire. The two filesystem gates run
        // BEFORE the printer gates precisely so that a disk failure keeps
        // this assertion in its strongest form.
        assert!(
            fake.gcode_sent().is_empty(),
            "a write failure must abort before anything is sent: {:?}",
            fake.gcode_sent()
        );
    }

    #[test]
    fn step_mode_gates_every_step_and_stops_on_no() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("step", &fake.url());
        // Consent, then approve steps 1 and 2, decline step 3.
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, true, true),
            "y\ny\ny\nn\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(output.contains("ABORTED at step 3"), "{output}");
        // The declined third step re-checks nothing and sends nothing: the
        // gate refuses before the per-step barrier.
        assert_eq!(
            fake.gcode_sent(),
            vec![
                super::GCODE_BARRIER_SENTINEL,
                super::GCODE_BARRIER_SENTINEL,
                "SET_IDLE_TIMEOUT TIMEOUT=86400",
                super::GCODE_BARRIER_SENTINEL,
                "PROBE PROBE_SPEED=1 SAMPLES=1",
            ]
        );
    }

    #[test]
    fn verification_failure_aborts_with_transcript_and_nonzero_exit() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(it) = v.get_mut("status").and_then(|s| s.get_mut("idle_timeout")) {
                    *it = json!({"idle_timeout": 600.0});
                }
            }
            Ok(v)
        }));
        let config = test_config("verifyfail", &fake.url());
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(output.contains("ABORTED at step 1"), "{output}");
        assert!(output.contains("idle-timeout-not-applied"), "{output}");
        assert_eq!(
            fake.gcode_sent(),
            vec![
                super::GCODE_BARRIER_SENTINEL,
                super::GCODE_BARRIER_SENTINEL,
                "SET_IDLE_TIMEOUT TIMEOUT=86400"
            ],
            "abort must not continue"
        );
    }

    #[test]
    fn non_plan_outcomes_map_to_exit_codes() {
        let config = test_config("outcomes", DEAD_URL);
        let options = fast_recover(false, false, false);
        let (code, output) = run_drive(&PipelineOutcome::CleanShutdown, &config, &options, "");
        assert_eq!(code, crate::EXIT_OK);
        assert!(output.contains("CLEAN"), "{output}");
        // A finished print whose LOG ended torn: exit 0, and the wording
        // must not claim the log ended cleanly, because it did not.
        let complete = |work| {
            PipelineOutcome::Complete(Box::new(crate::pipeline::CompletionReport {
                file: "/g/part.gcode".to_owned(),
                tested_offset: 500_000,
                file_size: 514_537,
                work,
            }))
        };
        let (code, output) = run_drive(
            &complete(plr_analyzer::RemainingWork::EndSequenceOnly {
                commands: vec!["M107".to_owned(), "M104".to_owned(), "M84".to_owned()],
            }),
            &config,
            &options,
            "",
        );
        assert_eq!(code, crate::EXIT_OK);
        assert!(output.contains("the print is COMPLETE"), "{output}");
        assert!(output.contains("14537 trailing bytes"), "{output}");
        assert!(output.contains("M107 M104 M84"), "{output}");
        assert!(output.contains("NOT offered for execution"), "{output}");
        assert!(
            !output.contains("WAL ends with a CLEAN"),
            "the log did NOT end cleanly: {output}"
        );
        // A print that ran its end sequence too has nothing to name.
        let (code, output) = run_drive(
            &complete(plr_analyzer::RemainingWork::Nothing),
            &config,
            &options,
            "",
        );
        assert_eq!(code, crate::EXIT_OK);
        assert!(output.contains("ran its end sequence too"), "{output}");
        let (code, output) = run_drive(
            &PipelineOutcome::ManualFallback("vase mode".to_owned()),
            &config,
            &options,
            "",
        );
        assert_eq!(code, crate::EXIT_RUNTIME);
        assert!(output.contains("automation declined"), "{output}");
        let (code, output) = run_drive(
            &PipelineOutcome::NotPossible("no file".to_owned()),
            &config,
            &options,
            "",
        );
        assert_eq!(code, crate::EXIT_RUNTIME);
        assert!(output.contains("not possible"), "{output}");
        // MachineRejected lists every failure.
        let machine = machine_config(&crate::config::MachineSection::default(), false, None);
        let rejection = plr_recovery::validate_machine(&machine).unwrap_err();
        let (code, output) = run_drive(
            &PipelineOutcome::MachineRejected(rejection),
            &config,
            &options,
            "",
        );
        assert_eq!(code, crate::EXIT_RUNTIME);
        assert!(output.contains("machine prerequisites failed"), "{output}");
        assert!(output.contains("force_move"), "{output}");
    }

    #[test]
    fn run_recover_end_to_end_dry_run_from_config_file() {
        // Full stack: config file → WAL fixture → pipeline → plan →
        // dry run. Moonraker URL is unreachable on purpose: the dry
        // path must never need it.
        let (dir, fixture_config) = crate::pipeline::e2e_tests::fixture("recover-e2e");
        let m = &fixture_config.machine;
        let config_text = format!(
            "wal_dir = {}\nmoonraker_url = {DEAD_URL}\n[machine]\n\
             force_move_enabled = true\nz_self_locking_attested = true\n\
             z_steppers = stepper_z\nprobe_kind = tap\nprobe_z_offset = -0.1\n\
             probe_activate_gcode_no_move = true\nprobe_deactivate_gcode_no_move = true\n\
             z_position_min = -2\nklipper_config_path = {}\n\
             validated_config_hash = {}\nvirtual_sdcard_root = {}\n",
            dir.display(),
            m.klipper_config_path.as_ref().unwrap().display(),
            m.validated_config_hash.as_ref().unwrap(),
            m.virtual_sdcard_root.as_ref().unwrap(),
        );
        let config_path = dir.join("plrd.conf");
        std::fs::write(&config_path, config_text).unwrap();
        let mut stdin = Cursor::new(Vec::new());
        let mut out = Vec::new();
        let code = super::run_recover(
            &config_path,
            &fast_recover(false, false, false),
            &mut stdin,
            &mut out,
        );
        let output = String::from_utf8(out).unwrap();
        assert_eq!(code, crate::EXIT_OK, "{output}");
        assert!(output.contains("dead-reckoning recovery plan"), "{output}");
        assert!(output.contains("DRY RUN"), "{output}");
        // The plan selects the generated recovery file (no M26 seek).
        assert!(
            output.contains("M23 part_RECOVERY.gcode"),
            "plan must select the recovery file: {output}"
        );
        assert!(!output.contains("M26 S"), "no M26 seek remains: {output}");
        // The dry run PREVIEWS the recovery file but never writes it.
        assert!(
            output.contains("recovery file preview"),
            "dry run must preview the recovery file: {output}"
        );
        assert!(
            !dir.join("part_RECOVERY.gcode").exists(),
            "dry run must NOT write the recovery file"
        );
        // Unreadable config path is a runtime error.
        let code = super::run_recover(
            std::path::Path::new("/nonexistent/plrd.conf"),
            &fast_recover(false, false, false),
            &mut Cursor::new(Vec::new()),
            &mut Vec::new(),
        );
        assert_eq!(code, crate::EXIT_RUNTIME);
    }

    #[test]
    fn abort_at_the_shifted_frame_writes_the_marker() {
        use super::{execute_with_gates, AutoGate};
        use plr_recovery::{
            AbortReason, FailureAction, Phase, Predicate, RecoveryPlan, RecoveryStep, Verification,
        };
        let rt = tokio::runtime::Runtime::new().unwrap();
        // A two-step plan whose shifted-frame declare (step 2) fails its
        // post-verify: the abort lands AT the shifted frame.
        let plan = RecoveryPlan {
            steps: vec![
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
                    // happy_handler's toolhead.position.2 is 0.75, never
                    // within 0.05 of -1.15 → this fails → abort at step 2.
                    verify: vec![Verification::new(
                        "toolhead",
                        "position.2",
                        Predicate::NumWithin {
                            expected: -1.15,
                            epsilon: 0.05,
                        },
                    )],
                    compute: None,
                    cleanup_commands: vec![],
                    on_failure: FailureAction::Abort {
                        reason: AbortReason::ShiftedFrameNotDeclared,
                    },
                },
            ],
            ..test_plan()
        };
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("writeabort", &fake.url());
        let bundle = PlanBundle {
            plan,
            file_path: "/g/x.gcode".to_owned(),
            machine: machine_config(&crate::config::MachineSection::default(), true, None),
            recovery_file_content: b"; recovery\nG28 X Y\n".to_vec(),
            recovery_file_path: config.wal_dir.join("x_RECOVERY.gcode"),
            sdcard_root: config.wal_dir.clone(),
            recovery_source_name: "x.gcode".to_owned(),
        };
        let opts = fast_recover(true, true, false);
        let mut out = Vec::new();
        let code = rt.block_on(execute_with_gates(
            &bundle,
            &config,
            &opts.exec_options,
            opts.connect_timeout,
            &mut AutoGate,
            &mut crate::executor::AbortConfirmer,
            &mut out,
        ));
        assert_eq!(code, crate::EXIT_RUNTIME);
        let output = String::from_utf8(out).unwrap();
        assert!(output.contains("ABORTED at step 2"), "{output}");
        assert!(output.contains("Z frame is now UNKNOWN"), "{output}");
        // The marker is on disk, carrying the step and reason.
        let marker = crate::detect::read_frame_invalid(&config.wal_dir).expect("marker");
        assert_eq!(marker.step_id, 2);
        assert_eq!(marker.reason, "shifted-frame-not-declared");
    }

    #[test]
    fn frame_invalid_marker_refuses_execute_and_a_dry_run_clears_it() {
        // Round trip: a frame-invalid marker (as an aborted recovery
        // would leave) → a re-execute is REFUSED with zero gcode → a
        // fresh dry run clears it → execute is no longer frame-refused.
        let config = test_config("frameinv", DEAD_URL);
        let marker = crate::detect::FrameInvalid {
            detected_wall_ns: 1,
            step_id: 7,
            phase: "shifted-frame".to_owned(),
            reason: "shifted-frame-not-declared".to_owned(),
        };
        crate::detect::write_frame_invalid(&config.wal_dir, &marker).unwrap();

        // Execute attempt: refused by the frame gate, before any connect
        // (DEAD_URL is never reached), and nothing is sent.
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(output.contains("Z frame is unknown"), "{output}");
        assert!(output.contains("step 7"), "{output}");
        // The marker survives the refused execute.
        assert!(crate::detect::read_frame_invalid(&config.wal_dir).is_some());

        // A fresh dry run clears the marker.
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(false, false, false),
            "",
        );
        assert_eq!(code, crate::EXIT_OK, "{output}");
        assert!(
            output.contains("cleared the stale frame-invalid"),
            "{output}"
        );
        assert!(crate::detect::read_frame_invalid(&config.wal_dir).is_none());

        // Now the frame gate no longer refuses: the execute proceeds far
        // enough to fail on the (unreachable) Moonraker connection
        // instead of the frame refusal.
        let (code, output) = run_drive(
            &plan_outcome(),
            &config,
            &fast_recover(true, true, false),
            "y\n",
        );
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        assert!(!output.contains("Z frame is unknown"), "{output}");
        assert!(output.contains("cannot reach Moonraker"), "{output}");
    }

    #[test]
    fn read_yes_accepts_only_explicit_consent() {
        for (input, expected) in [
            ("y\n", true),
            ("Y\n", true),
            ("yes\n", true),
            ("YES\n", true),
            ("n\n", false),
            ("\n", false),
            ("", false),
            ("yep\n", false),
            (" y es\n", false),
        ] {
            assert_eq!(
                read_yes(&mut Cursor::new(input.as_bytes().to_vec())),
                expected,
                "{input:?}"
            );
        }
    }

    // --- The frame interlock is armed on ENTRY, not written on abort ---
    //
    // The blocker these tests pin: `frame_invalid` used to be computed
    // inside `abort()` on the returned outcome, so any termination that
    // prevented the outcome from being produced — a runtime drop on
    // `systemctl restart`, a SIGTERM, a panic, a SIGKILL, a second power
    // loss — discarded the interlock while the Z frame was already
    // fabricated. The marker is now written before the declare is
    // ISSUED, so it survives by construction: nothing has to run.

    /// A plan whose step 2 declares the shifted frame and whose step 3
    /// pauses for the operator, i.e. the window an operator actually sits
    /// in for up to `confirm_timeout_s`.
    fn framed_pausing_bundle(config: &Config) -> PlanBundle {
        use plr_recovery::{
            AbortReason, FailureAction, Phase, RecoveryPlan, RecoveryStep, RuntimeComputation,
        };
        let plan = RecoveryPlan {
            steps: vec![
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
                    phase: Phase::ZConfirmStandoff,
                    summary: "standoff for the operator Z confirmation".to_owned(),
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
                },
            ],
            ..test_plan()
        };
        PlanBundle {
            plan,
            file_path: "/g/x.gcode".to_owned(),
            machine: machine_config(&crate::config::MachineSection::default(), true, None),
            recovery_file_content: b"; recovery\nG28 X Y\n".to_vec(),
            recovery_file_path: config.wal_dir.join("x_RECOVERY.gcode"),
            sdcard_root: config.wal_dir.clone(),
            recovery_source_name: "x.gcode".to_owned(),
        }
    }

    /// A confirmer that never answers, standing in for an operator who
    /// walked away — or a plugin that disconnected.
    struct NeverAnswers;

    impl crate::executor::Confirmer for NeverAnswers {
        fn confirm<'a>(
            &'a mut self,
            _point: &'a crate::executor::ConfirmPoint,
        ) -> std::pin::Pin<
            Box<dyn std::future::Future<Output = crate::executor::ConfirmAnswer> + Send + 'a>,
        > {
            Box::pin(async {
                std::future::pending::<()>().await;
                crate::executor::ConfirmAnswer::Continue
            })
        }
    }

    /// A confirmer that panics, standing in for a bug in the execution
    /// task.
    struct PanicsAtThePause;

    impl crate::executor::Confirmer for PanicsAtThePause {
        fn confirm<'a>(
            &'a mut self,
            _point: &'a crate::executor::ConfirmPoint,
        ) -> std::pin::Pin<
            Box<dyn std::future::Future<Output = crate::executor::ConfirmAnswer> + Send + 'a>,
        > {
            Box::pin(async { panic!("simulated bug inside the execution task") })
        }
    }

    /// Runs `execute_with_gates` on a spawned task with the given
    /// confirmer, waits until the interlock appears (i.e. the declare is
    /// about to be issued), and hands the caller the still-running
    /// handle.
    async fn spawn_until_armed(
        bundle: PlanBundle,
        config: Config,
        confirmer: Box<dyn crate::executor::Confirmer>,
    ) -> tokio::task::JoinHandle<u8> {
        let wal_dir = config.wal_dir.clone();
        let opts = fast_recover(true, true, false);
        let handle = tokio::spawn(async move {
            let mut confirmer = confirmer;
            let mut out = Vec::new();
            super::execute_with_gates(
                &bundle,
                &config,
                &opts.exec_options,
                opts.connect_timeout,
                &mut super::AutoGate,
                confirmer.as_mut(),
                &mut out,
            )
            .await
        });
        for _ in 0..400 {
            if crate::detect::read_frame_invalid(&wal_dir).is_some() {
                return handle;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!("the interlock was never armed");
    }

    /// Waits until a command starting with `prefix` has actually reached
    /// the fake printer.
    ///
    /// [`spawn_until_armed`] cannot stand in for this and must not be used
    /// as if it could. Arming is *strictly earlier* than the send:
    /// `executor::arm_frame_if_entering` runs before `run_step` issues the
    /// step's commands (`executor.rs`, the `for step in &plan.steps` loop),
    /// so the interlock file exists before any of `SET_KINEMATIC_POSITION`
    /// has been serialised, let alone acknowledged by the fake's
    /// WebSocket task. Asserting on `fake.gcode_sent()` the instant the
    /// file appears is therefore not a race that is *usually* won — it is
    /// an assertion about an event that has not been waited for at all.
    /// Measured on Linux, 24 cores, 24 concurrent copies of the test plus
    /// 24 busy-loop load processes: **18 failures in 2000 runs** before this
    /// helper, **0 in 2000** after, and every failure printed the same
    /// history — `["M115", "SET_IDLE_TIMEOUT TIMEOUT=86400"]` — i.e. the
    /// declare simply had not been sent yet.
    ///
    /// The fix is to wait for the observable the assertion is about. A
    /// longer arming budget would not have touched it: arming had already
    /// succeeded in every one of those failures.
    async fn wait_for_gcode(fake: &FakeMoonraker, prefix: &str) -> Vec<String> {
        for _ in 0..1_000 {
            let sent = fake.gcode_sent();
            if sent.iter().any(|c| c.starts_with(prefix)) {
                return sent;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!(
            "no command starting with {prefix:?} was ever sent; saw {:?}",
            fake.gcode_sent()
        );
    }

    /// BLOCKER 1: a daemon shutdown while paused must not discard the
    /// interlock. Dropping the runtime drops the execution future
    /// mid-`await`, so nothing on the abort path ever runs — and the
    /// marker must still be there.
    #[test]
    fn a_daemon_shutdown_while_paused_leaves_the_interlock_set() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("shutdown-paused", &fake.url());
        let wal_dir = config.wal_dir.clone();
        let bundle = framed_pausing_bundle(&config);
        rt.block_on(async {
            let handle = spawn_until_armed(bundle, config, Box::new(NeverAnswers)).await;
            // The declare really was issued: we are in the danger zone.
            // Waited for, not assumed — see `wait_for_gcode`.
            let sent = wait_for_gcode(&fake, "SET_KINEMATIC_POSITION").await;
            assert!(
                sent.iter().any(|c| c.starts_with("SET_KINEMATIC_POSITION")),
                "{sent:?}"
            );
            // `systemctl restart plrd`: the task is dropped mid-await.
            handle.abort();
            let _ = handle.await;
        });
        // Nothing on the abort path ran, and the interlock is still set.
        let marker = crate::detect::read_frame_invalid(&wal_dir)
            .expect("the interlock must survive an abrupt shutdown");
        assert_eq!(marker.step_id, 2);
        assert_eq!(marker.reason, "shifted-frame-declared");
    }

    /// A panic inside the execution task is the same class of event and
    /// must leave the interlock exactly as set.
    #[test]
    fn a_panic_mid_execution_leaves_the_interlock_set() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("panic-mid", &fake.url());
        let wal_dir = config.wal_dir.clone();
        let bundle = framed_pausing_bundle(&config);
        rt.block_on(async {
            let handle = spawn_until_armed(bundle, config, Box::new(PanicsAtThePause)).await;
            let joined = handle.await;
            assert!(joined.is_err(), "the task must have panicked");
        });
        assert!(
            crate::detect::read_frame_invalid(&wal_dir).is_some(),
            "the interlock must survive a panic"
        );
    }

    // BLOCKER 2 (an unwritable interlock must refuse before the shifted
    // frame) is exercised against a REAL pipeline-built plan in
    // `ctrlsock::tests::an_unwritable_interlock_refuses_before_the_shifted_frame_declare`.
    // A synthetic plan cannot pin that invariant honestly: the pipeline's
    // plans declare a believed-Z frame two phases before the shifted one,
    // so "no SET_KINEMATIC_POSITION was sent" is true only of a fixture
    // that omits it, and false of every plan the machine actually runs.

    /// The complement: a successful completion clears the interlock, so
    /// arming it eagerly does not wedge every subsequent recovery.
    #[test]
    fn a_successful_completion_clears_the_interlock() {
        use plr_recovery::{AbortReason, FailureAction, Phase, RecoveryPlan, RecoveryStep};
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("completion-clears", &fake.url());
        let wal_dir = config.wal_dir.clone();
        // Two steps: idle, then the shifted-frame declare. Both succeed.
        let plan = RecoveryPlan {
            steps: vec![
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
            ],
            ..test_plan()
        };
        let bundle = PlanBundle {
            plan,
            ..framed_pausing_bundle(&config)
        };
        let opts = fast_recover(true, true, false);
        let mut out = Vec::new();
        let code = rt.block_on(super::execute_with_gates(
            &bundle,
            &config,
            &opts.exec_options,
            opts.connect_timeout,
            &mut super::AutoGate,
            &mut crate::executor::AbortConfirmer,
            &mut out,
        ));
        let output = String::from_utf8(out).unwrap();
        assert_eq!(code, crate::EXIT_OK, "{output}");
        assert!(
            crate::detect::read_frame_invalid(&wal_dir).is_none(),
            "a completed recovery re-established the frame; the interlock must clear"
        );
        // And it really was armed on the way through.
        let transcript = std::fs::read_dir(&wal_dir)
            .unwrap()
            .filter_map(Result::ok)
            .find(|e| {
                e.file_name()
                    .to_string_lossy()
                    .starts_with("recovery-transcript-")
            })
            .expect("transcript");
        let text = std::fs::read_to_string(transcript.path()).unwrap();
        assert!(text.contains("\"event\":\"frame-armed\""), "{text}");
    }
}
