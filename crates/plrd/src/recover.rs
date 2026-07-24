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
//! 5. **Machine prerequisites** — already fatal inside the pipeline
//!    (`pipeline::run_pipeline` runs `validate_machine` before any
//!    planning); no plan exists to execute if they failed.
//! 6. **Per-step gate** — with `--step`, every step asks again.
//!
//! Execution writes a JSONL transcript
//! (`recovery-transcript-<unix-seconds>.jsonl` in the WAL directory);
//! refusing to create it refuses to execute.

use std::io::{BufRead, Write};
use std::path::Path;
use std::time::Duration;

use crate::config::Config;
use crate::executor::{dry_run, execute, ExecOptions, ExecOutcome, Transcript};
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
    // Moonraker client is ever constructed on it.
    if !options.execute {
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
    runtime.block_on(execute_with_gates(
        bundle,
        config,
        &options.exec_options,
        options.connect_timeout,
        &mut gate,
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

    // Gate 4: ready + idle.
    if let Err(reason) = printer_ready_and_idle(&mut client).await {
        let _ = writeln!(out, "recover: REFUSED — {reason}; nothing was sent.");
        return EXIT_RUNTIME;
    }

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

    let outcome = {
        let mut transcript = Transcript::new(&mut transcript_file);
        execute(
            &bundle.plan,
            &mut client,
            exec_options,
            &mut gate_fn,
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
        ExecOutcome::Aborted {
            step_id,
            phase,
            reason,
            cause,
            frame_invalid,
        } => {
            let _ = writeln!(
                out,
                "recover: ABORTED at step {step_id} [{phase}]: {reason} ({cause:?})"
            );
            if frame_invalid {
                record_frame_invalid(&config.wal_dir, step_id, &phase, &reason, out);
            }
            let _ = writeln!(
                out,
                "recover: the printer was left as-is; review the transcript before retrying."
            );
            EXIT_RUNTIME
        }
    }
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
    if let Err(e) = crate::detect::write_frame_invalid(wal_dir, &marker) {
        let _ = writeln!(
            out,
            "recover: WARNING — cannot write frame-invalid marker: {e}"
        );
    }
    let _ = writeln!(
        out,
        "recover: the printer's Z frame is now UNKNOWN — re-run a dry run \
         (plrd scan / plrd recover without --execute) for a fresh plan before \
         any resume; --execute is refused until then."
    );
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
    use serde_json::json;
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
        let machine = machine_config(&crate::config::MachineSection::default(), true, None);
        PipelineOutcome::Plan(Box::new(PlanBundle {
            plan: test_plan(),
            file_path: "/g/x.gcode".to_owned(),
            machine,
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
        assert_eq!(
            fake.gcode_sent(),
            vec![
                "SET_IDLE_TIMEOUT TIMEOUT=86400",
                "PROBE PROBE_SPEED=1 SAMPLES=1",
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
        assert_eq!(
            fake.gcode_sent(),
            vec![
                "SET_IDLE_TIMEOUT TIMEOUT=86400",
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
        assert_eq!(fake.gcode_sent().len(), 1, "abort must not continue");
    }

    #[test]
    fn non_plan_outcomes_map_to_exit_codes() {
        let config = test_config("outcomes", DEAD_URL);
        let options = fast_recover(false, false, false);
        let (code, output) = run_drive(&PipelineOutcome::CleanShutdown, &config, &options, "");
        assert_eq!(code, crate::EXIT_OK);
        assert!(output.contains("CLEAN"), "{output}");
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
        assert!(
            output.contains("M26 S"),
            "plan must seek the file: {output}"
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
        };
        let opts = fast_recover(true, true, false);
        let mut out = Vec::new();
        let code = rt.block_on(execute_with_gates(
            &bundle,
            &config,
            &opts.exec_options,
            opts.connect_timeout,
            &mut AutoGate,
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
}
