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

    let outcome = {
        let mut transcript = Transcript::new(&mut transcript_file);
        execute(
            &bundle.plan,
            &mut client,
            exec_options,
            &mut gate_fn,
            confirmer,
            &mut frame_guard,
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
        let _ = writeln!(
            out,
            "recover: REFUSED to declare the shifted Z frame — the frame-invalid \
             interlock could not be written ({why}). NOTHING was declared, so the \
             printer is untouched and its Z frame is exactly as it was. Fix the WAL \
             directory {} (it is also where the transcript lives) and retry.",
            config.wal_dir.display()
        );
    }
    let _ = writeln!(
        out,
        "recover: the printer was left as-is; review the transcript before retrying."
    );
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
        assert!(
            fake.gcode_sent().is_empty(),
            "a write failure must abort before any motion: {:?}",
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
            assert!(
                fake.gcode_sent()
                    .iter()
                    .any(|c| c.starts_with("SET_KINEMATIC_POSITION")),
                "{:?}",
                fake.gcode_sent()
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

    /// BLOCKER 2: if the interlock cannot be written, the declare must
    /// never be issued — and the next execute must still be refused,
    /// because there is nothing to refuse it with except not having gone
    /// in the first place.
    #[test]
    fn an_unwritable_interlock_refuses_before_any_kinematic_declare() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let fake = rt.block_on(FakeMoonraker::spawn(happy_handler));
        let config = test_config("interlock-blocked", &fake.url());
        let bundle = framed_pausing_bundle(&config);
        // Block ONLY the marker write, leaving the WAL directory writable
        // so the transcript gate passes and execution really reaches the
        // declare: a directory sitting on the staging path makes
        // `File::create` fail there and nowhere else.
        std::fs::create_dir(config.wal_dir.join(crate::detect::FRAME_INVALID_TEMP_NAME)).unwrap();

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
        assert_eq!(code, crate::EXIT_RUNTIME, "{output}");
        // The refusal happened BEFORE the frame was touched.
        assert!(
            !fake
                .gcode_sent()
                .iter()
                .any(|c| c.starts_with("SET_KINEMATIC_POSITION")),
            "a state we cannot record must never be entered: {:?}",
            fake.gcode_sent()
        );
        assert!(
            output.contains("REFUSED to declare the shifted Z frame"),
            "{output}"
        );
        assert!(output.contains("NOTHING was declared"), "{output}");
    }

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
