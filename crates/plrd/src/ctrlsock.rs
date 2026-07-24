//! The plrd control socket: a UNIX stream socket served by `plrd run`,
//! spoken by the Klipper plugin's `PLR_STATUS` / `PLR_RECOVER`
//! commands.
//!
//! # Protocol (fixed; the plugin implements the client verbatim)
//!
//! * Requests: one line of JSON, `{"cmd": "<name>", "args": {...}}\n`.
//! * Responses: one line of JSON,
//!   `{"ok": bool, "text": "<human-readable multi-line report>",
//!   "data": {...}}\n`.
//! * Commands: `ping`, `status`, `recover_dryrun`, `recover_execute`
//!   (args `{"confirm": true, "step": bool}`).
//! * Malformed JSON and unknown commands get an `ok: false` response —
//!   never a dropped connection. A request line larger than
//!   [`MAX_REQUEST_BYTES`] gets an error response and the connection is
//!   closed (the line can never complete).
//! * **One request per connection**: the response is written, then the
//!   connection is closed. Connections are cheap on a local UNIX
//!   socket, and one-shot framing means a wedged client can never hold
//!   protocol state hostage. (A request line terminated by EOF instead
//!   of `\n` is accepted — it is unambiguous.)
//!
//! `ok` semantics: `true` when the command ran AND reached its good
//! outcome (`ping`/`status` answered; dry-run produced a plan or found
//! a clean shutdown; execution completed). Refusals, declines, and
//! aborts are `ok: false` with the full report in `text` and a stable
//! `data.outcome` tag.
//!
//! # `recover_execute`
//!
//! Runs the SAME gate stack as `plrd recover --execute --confirm`
//! (`recover::execute_with_gates`): machine validation inside the
//! pipeline, Moonraker reachability, klippy ready + printer idle,
//! transcript-or-refuse, abort on any failed verification. The one
//! difference: the CLI's interactive TTY prompt is replaced by the
//! request's explicit `"confirm": true` (the plugin collects operator
//! consent on its side). `"step": true` is rejected with "per-step
//! mode is CLI-only" — v1 keeps the socket protocol one-shot rather
//! than inventing a multi-round confirmation dialogue.
//!
//! Executions are serialized: a second `recover_execute` while one is
//! running gets an immediate `busy` error (`try_lock`, never queued —
//! a queued recovery executing minutes later against a changed printer
//! would be indefensible).
//!
//! # Never starving the recorder
//!
//! The daemon runs a current-thread tokio runtime whose critical task
//! is the Klipper socket reader (Klipper disconnects slow clients) and
//! whose durability path is a dedicated OS thread. Therefore:
//!
//! * the accept loop and each connection are separate spawned tasks;
//! * CPU/file-heavy work (the WAL scan → reconstruct → plan pipeline)
//!   runs on `spawn_blocking`'s thread pool, never on the runtime
//!   thread;
//! * execution proper is await-yielding Moonraker I/O.
//!
//! # Socket permissions: mode 0666, deliberately
//!
//! The stock install runs plrd as root (systemd unit, `StateDirectory`)
//! while klippy — the socket's one intended client — runs as an
//! unprivileged user whose identity plrd cannot know at bind time, so
//! a same-user or fixed-group mode would break the plugin out of the
//! box. World-writable is acceptable here because the socket's mutating
//! surface is narrow and gated: `recover_execute` demands an explicit
//! `confirm`, then still passes machine validation, the klippy
//! ready + printer-idle gate, and transcript-or-refuse, and can only
//! ever execute the deterministic pipeline plan — the socket exposes
//! no arbitrary-gcode or configuration surface. Hardening story for
//! multi-user hosts: run plrd as the klippy user (systemd `User=`), or
//! add a drop-in with `ExecStartPost=chgrp <group> %S/plrd/plrd.sock`
//! + `chmod 0660`; plrd itself keeps no group database dependency.
//!
//! # Stale socket files and the unlink race
//!
//! Bind unlinks any existing socket file first: after a crash the old
//! inode survives and `bind` would otherwise fail forever. The
//! unlink-then-bind window is a TOCTOU race only if a *second* plrd
//! binds the same path concurrently — which the service manager
//! forbids (single unit instance), and losing the race yields a bind
//! error and a fatal, visible startup failure: the safe direction.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use serde_json::{json, Map, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::config::Config;
use crate::executor::ExecOptions;
use crate::pipeline::{self, MachineSource, PipelineOutcome};
use crate::recover::{self, AutoGate, RecoverOptions};
use crate::EXIT_OK;

/// Hard cap on one request line (documented in the module docs).
pub const MAX_REQUEST_BYTES: usize = 64 * 1024;

/// Shared state of the control server.
pub struct CtrlState {
    /// The daemon configuration (paths, Moonraker URL, machine
    /// section).
    pub config: Config,
    /// Executor timing for `recover_execute` (tests shrink these).
    pub exec_options: ExecOptions,
    /// Moonraker connect timeout for `recover_execute`.
    pub connect_timeout: Duration,
    /// Serializes `recover_execute` (module docs).
    exec_lock: tokio::sync::Mutex<()>,
}

impl CtrlState {
    /// Production state for a daemon config.
    #[must_use]
    pub fn new(config: Config) -> Self {
        Self {
            config,
            exec_options: ExecOptions::default(),
            connect_timeout: Duration::from_secs(10),
            exec_lock: tokio::sync::Mutex::new(()),
        }
    }
}

/// Binds the control socket (std, non-blocking, ready for
/// `tokio::net::UnixListener::from_std`). Failure is fatal at daemon
/// startup — a daemon whose console-side contract cannot come up
/// should say so loudly, not run half-featured.
pub fn bind(path: &Path) -> Result<std::os::unix::net::UnixListener, String> {
    use std::os::unix::fs::PermissionsExt as _;
    if let Some(parent) = path.parent() {
        // Best-effort: the stock path's parent is systemd's
        // StateDirectory; a custom path may need its directory.
        let _ = std::fs::create_dir_all(parent);
    }
    // Stale-socket unlink; see the module docs for the TOCTOU
    // reasoning. NotFound is the common case and fine; any other
    // unlink error will surface as the bind error below.
    let _ = std::fs::remove_file(path);
    let listener = std::os::unix::net::UnixListener::bind(path)
        .map_err(|e| format!("cannot bind control socket {}: {e}", path.display()))?;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o666))
        .map_err(|e| format!("cannot chmod control socket {}: {e}", path.display()))?;
    listener
        .set_nonblocking(true)
        .map_err(|e| format!("cannot make control socket non-blocking: {e}"))?;
    Ok(listener)
}

/// Serves the control socket forever. Runs as a spawned task beside
/// the recorder; every connection gets its own task.
pub async fn serve(listener: tokio::net::UnixListener, state: Arc<CtrlState>) {
    loop {
        match listener.accept().await {
            Ok((stream, _addr)) => {
                let state = Arc::clone(&state);
                tokio::spawn(async move {
                    handle_connection(stream, state).await;
                });
            }
            Err(e) => {
                // Accept errors (fd pressure) are transient; back off
                // briefly instead of spinning.
                eprintln!("plrd: control socket accept error: {e}");
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        }
    }
}

/// One connection: read one capped request line, answer, close.
async fn handle_connection(mut stream: tokio::net::UnixStream, state: Arc<CtrlState>) {
    let response = match read_request_line(&mut stream).await {
        Ok(Some(line)) => respond_line(&state, &line).await,
        // Peer connected and closed without sending anything: nothing
        // to answer.
        Ok(None) => return,
        Err(oversized) => oversized,
    };
    let mut bytes = response.to_string().into_bytes();
    bytes.push(b'\n');
    let _ = stream.write_all(&bytes).await;
    let _ = stream.shutdown().await;
}

/// Reads up to one `\n`-terminated line (cap [`MAX_REQUEST_BYTES`]).
/// `Ok(None)` for an empty connection, `Err(response)` when the cap is
/// exceeded (the connection cannot recover: the response is sent and
/// the caller closes).
async fn read_request_line(stream: &mut tokio::net::UnixStream) -> Result<Option<String>, Value> {
    let mut line: Vec<u8> = Vec::new();
    let mut buf = [0_u8; 4096];
    loop {
        let n = match stream.read(&mut buf).await {
            Ok(0) | Err(_) => break,
            Ok(n) => n,
        };
        if let Some(pos) = buf[..n].iter().position(|&b| b == b'\n') {
            line.extend_from_slice(&buf[..pos]);
            break;
        }
        line.extend_from_slice(&buf[..n]);
        if line.len() > MAX_REQUEST_BYTES {
            return Err(error_response(
                "oversized",
                &format!("request line exceeds {MAX_REQUEST_BYTES} bytes; closing"),
            ));
        }
    }
    if line.is_empty() {
        return Ok(None);
    }
    match String::from_utf8(line) {
        Ok(text) => Ok(Some(text)),
        Err(_) => Err(error_response("malformed", "request line is not UTF-8")),
    }
}

fn error_response(outcome: &str, text: &str) -> Value {
    json!({"ok": false, "text": text, "data": {"outcome": outcome}})
}

fn ok_response(text: &str, data: &Value) -> Value {
    json!({"ok": true, "text": text, "data": data})
}

/// Parses and dispatches one request line. Public within the crate so
/// protocol tests can exercise dispatch without a socket.
pub(crate) async fn respond_line(state: &CtrlState, line: &str) -> Value {
    let request: Value = match serde_json::from_str(line.trim()) {
        Ok(value) => value,
        Err(e) => return error_response("malformed", &format!("request is not valid JSON: {e}")),
    };
    let Some(cmd) = request.get("cmd").and_then(Value::as_str) else {
        return error_response("malformed", "request has no string `cmd` field");
    };
    let empty = Map::new();
    let args = request
        .get("args")
        .and_then(Value::as_object)
        .unwrap_or(&empty);
    match cmd {
        "ping" => ok_response(
            &format!("plrd {}", env!("CARGO_PKG_VERSION")),
            &json!({"version": env!("CARGO_PKG_VERSION")}),
        ),
        "status" => cmd_status(state).await,
        "recover_dryrun" => cmd_recover_dryrun(state).await,
        "recover_execute" => cmd_recover_execute(state, args).await,
        other => error_response("unknown-cmd", &format!("unknown cmd {other:?}")),
    }
}

/// `status`: recorder + recovery state, assembled off the runtime
/// thread (it reads WAL metadata and queries klippy).
async fn cmd_status(state: &CtrlState) -> Value {
    let config = state.config.clone();
    let result = tokio::task::spawn_blocking(move || build_status(&config)).await;
    match result {
        Ok((text, data)) => ok_response(&text, &data),
        Err(e) => error_response("error", &format!("status task failed: {e}")),
    }
}

/// Builds the status report (blocking context).
fn build_status(config: &Config) -> (String, Value) {
    use std::fmt::Write as _;
    let mut text = String::new();
    let mut data = Map::new();

    // WAL directory: segment count + bytes.
    let wal_dir = config.wal_dir.display().to_string();
    let _ = writeln!(text, "WAL dir: {wal_dir}");
    data.insert("wal_dir".to_owned(), json!(wal_dir));
    match crate::scan::list_segments(&config.wal_dir) {
        Ok(segments) => {
            let bytes: u64 = segments
                .iter()
                .filter_map(|(_, path)| std::fs::metadata(path).ok())
                .map(|m| m.len())
                .sum();
            let _ = writeln!(text, "segments: {} ({} bytes)", segments.len(), bytes);
            data.insert("segments".to_owned(), json!(segments.len()));
            data.insert("wal_bytes".to_owned(), json!(bytes));
        }
        Err(e) => {
            let _ = writeln!(text, "segments: unavailable ({e})");
            data.insert("segments".to_owned(), Value::Null);
        }
    }

    // Heartbeat age (wall clock: the heartbeat file journals wall_ns).
    match crate::scan::load_heartbeat(&config.heartbeat_file()) {
        Ok(recovery) => {
            let now = crate::hostclock::now_wall_ns();
            // Sub-microsecond precision loss is irrelevant for an age
            // display measured in seconds.
            #[allow(clippy::cast_precision_loss)]
            let age_s = (now.saturating_sub(recovery.heartbeat.wall_ns)) as f64 / 1_000_000_000.0;
            let _ = writeln!(
                text,
                "heartbeat: seq {} age {age_s:.1}s",
                recovery.heartbeat.sequence
            );
            data.insert("heartbeat_age_s".to_owned(), json!(age_s));
        }
        Err(reason) => {
            let _ = writeln!(text, "heartbeat: {reason}");
            data.insert("heartbeat_age_s".to_owned(), Value::Null);
        }
    }

    // Pending recovery.
    let pending_path = config.wal_dir.join(crate::detect::PENDING_FILE_NAME);
    if let Some(pending) = std::fs::read_to_string(&pending_path)
        .ok()
        .and_then(|text| serde_json::from_str::<crate::detect::PendingRecovery>(&text).ok())
    {
        let _ = writeln!(
            text,
            "pending recovery: {} at byte {}{} ({})",
            pending.file,
            pending.file_position,
            pending
                .percent
                .map_or(String::new(), |p| format!(" (~{p:.0}%)")),
            pending.crash_class,
        );
        data.insert(
            "pending".to_owned(),
            serde_json::to_value(&pending).unwrap_or(Value::Null),
        );
    } else {
        let _ = writeln!(text, "pending recovery: none");
        data.insert("pending".to_owned(), Value::Null);
    }

    // Machine-config mode + validation summary. `;TYPE:` annotations
    // are a per-file property checked at recover time; status assumes
    // them present so the summary reflects the machine, not the file.
    let (mode, summary) = machine_summary(config);
    let _ = writeln!(text, "machine-config mode: {mode}");
    let _ = write!(text, "machine validation: {summary}");
    data.insert("machine_mode".to_owned(), json!(mode));
    data.insert("machine_validation".to_owned(), json!(summary));

    (text, Value::Object(data))
}

/// `(mode tag, validation summary)` for `status`.
fn machine_summary(config: &Config) -> (String, String) {
    let validated = |machine: &plr_recovery::MachineConfig| -> String {
        match plr_recovery::validate_machine(machine) {
            Ok(_) => "OK (;TYPE: annotation check deferred to recover time)".to_owned(),
            Err(rejection) => {
                let mut s = format!("{} check(s) failed:", rejection.failures.len());
                for failure in &rejection.failures {
                    s.push_str("\n  - ");
                    s.push_str(&failure.to_string());
                }
                s
            }
        }
    };
    match pipeline::resolve_machine_source(config) {
        MachineSource::Plr(source) => {
            let (machine, _) = crate::plrcfg::machine_from_settings(
                &source.snapshot.settings,
                &source.snapshot.config,
                &source.plr,
                true,
            );
            ("plr".to_owned(), validated(&machine))
        }
        MachineSource::Legacy { note } => {
            let machine = pipeline::machine_config(
                &config.machine,
                true,
                config.machine.klipper_config_path.as_deref(),
            );
            let mut summary = validated(&machine);
            if let Some(note) = note {
                summary.push_str("\n  note: ");
                summary.push_str(&note);
            }
            ("legacy".to_owned(), summary)
        }
        MachineSource::Unavailable { reason } => ("undetermined".to_owned(), reason),
    }
}

/// `recover_dryrun`: the full pipeline dry run — identical text to
/// `plrd recover` without `--execute`, produced by the same
/// `recover::drive` gate stack (with `execute: false`, `drive`
/// provably cannot send: no Moonraker client exists on that path).
async fn cmd_recover_dryrun(state: &CtrlState) -> Value {
    let config = state.config.clone();
    let result = tokio::task::spawn_blocking(move || {
        let mut out: Vec<u8> = Vec::new();
        let outcome = match pipeline::run_pipeline(&config, &mut out) {
            Ok(outcome) => outcome,
            Err(e) => {
                return (
                    false,
                    format!("{}recover: {e}", String::from_utf8_lossy(&out)),
                    "error".to_owned(),
                )
            }
        };
        let options = RecoverOptions::new(false, false, false);
        let mut stdin = std::io::Cursor::new(Vec::new());
        let code = recover::drive(&outcome, &config, &options, &mut stdin, &mut out);
        (
            code == EXIT_OK,
            String::from_utf8_lossy(&out).into_owned(),
            outcome_tag(&outcome).to_owned(),
        )
    })
    .await;
    match result {
        Ok((ok, text, outcome)) => {
            json!({"ok": ok, "text": text, "data": {"outcome": outcome}})
        }
        Err(e) => error_response("error", &format!("dry-run task failed: {e}")),
    }
}

fn outcome_tag(outcome: &PipelineOutcome) -> &'static str {
    match outcome {
        PipelineOutcome::CleanShutdown => "clean-shutdown",
        PipelineOutcome::MachineRejected(_) => "machine-rejected",
        PipelineOutcome::ManualFallback(_) => "manual-fallback",
        PipelineOutcome::NotPossible(_) => "not-possible",
        PipelineOutcome::Plan(_) => "plan",
    }
}

/// `recover_execute`: consent check, serialization, pipeline, then the
/// shared gate stack (`recover::execute_with_gates`).
async fn cmd_recover_execute(state: &CtrlState, args: &Map<String, Value>) -> Value {
    if args.get("step").and_then(Value::as_bool) == Some(true) {
        return error_response("refused", "per-step mode is CLI-only");
    }
    if args.get("confirm").and_then(Value::as_bool) != Some(true) {
        return error_response(
            "refused",
            "recover_execute requires explicit \"confirm\": true",
        );
    }
    // Serialize executions; a concurrent request is refused, not
    // queued (module docs).
    let Ok(_guard) = state.exec_lock.try_lock() else {
        return error_response("busy", "another recover_execute is already running");
    };

    // Pipeline (blocking work) off the runtime thread.
    let config = state.config.clone();
    let pipeline_result = tokio::task::spawn_blocking(move || {
        let mut out: Vec<u8> = Vec::new();
        let outcome = pipeline::run_pipeline(&config, &mut out);
        (outcome, out)
    })
    .await;
    let (outcome, mut out) = match pipeline_result {
        Ok((Ok(outcome), out)) => (outcome, out),
        Ok((Err(e), out)) => {
            return json!({
                "ok": false,
                "text": format!("{}recover: {e}", String::from_utf8_lossy(&out)),
                "data": {"outcome": "error"},
            })
        }
        Err(e) => return error_response("error", &format!("pipeline task failed: {e}")),
    };

    let PipelineOutcome::Plan(bundle) = &outcome else {
        // Non-plan outcomes render exactly like the CLI (drive never
        // executes anything for them regardless of flags).
        let options = RecoverOptions::new(false, false, false);
        let mut stdin = std::io::Cursor::new(Vec::new());
        let _ = recover::drive(&outcome, &state.config, &options, &mut stdin, &mut out);
        return json!({
            "ok": false,
            "text": String::from_utf8_lossy(&out).into_owned(),
            "data": {"outcome": outcome_tag(&outcome)},
        });
    };

    {
        use std::io::Write as _;
        let _ = writeln!(
            out,
            "recover: executing plan: {} steps; resume {} @ byte {}",
            bundle.plan.steps.len(),
            bundle.plan.resume_file,
            bundle.plan.resume_offset,
        );
    }
    let code = recover::execute_with_gates(
        bundle,
        &state.config,
        &state.exec_options,
        state.connect_timeout,
        &mut AutoGate,
        &mut out,
    )
    .await;
    json!({
        "ok": code == EXIT_OK,
        "text": String::from_utf8_lossy(&out).into_owned(),
        "data": {
            "outcome": if code == EXIT_OK { "completed" } else { "aborted-or-refused" },
            "exit": code,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::{bind, respond_line, serve, CtrlState, MAX_REQUEST_BYTES};
    use crate::config::Config;
    use crate::executor::tests::happy_handler;
    use crate::executor::ExecOptions;
    use crate::testmoon::FakeMoonraker;
    use serde_json::{json, Value};
    use std::path::PathBuf;
    use std::sync::Arc;
    use std::time::Duration;
    use tokio::io::{AsyncReadExt as _, AsyncWriteExt as _};

    fn temp_dir(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-ctrlsock-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn fast_state(config: Config) -> Arc<CtrlState> {
        let mut state = CtrlState::new(config);
        state.exec_options = ExecOptions {
            verify_timeout: Duration::from_millis(300),
            temp_timeout: Duration::from_millis(300),
            poll_interval: Duration::from_millis(20),
        };
        state.connect_timeout = Duration::from_secs(2);
        Arc::new(state)
    }

    /// Binds a server in a temp dir and returns (socket path, state).
    /// Must run inside a tokio runtime (tests are `#[tokio::test]`).
    fn spawn_server(tag: &str, config: Config) -> (PathBuf, Arc<CtrlState>) {
        let path = temp_dir(tag).join("plrd.sock");
        let std_listener = bind(&path).expect("bind");
        let listener = tokio::net::UnixListener::from_std(std_listener).expect("tokio listener");
        let state = fast_state(config);
        let serve_state = Arc::clone(&state);
        tokio::spawn(serve(listener, serve_state));
        (path, state)
    }

    /// One request over a real socket; returns the parsed response.
    async fn roundtrip(path: &PathBuf, request: &str) -> Value {
        let mut stream = tokio::net::UnixStream::connect(path)
            .await
            .expect("connect");
        stream.write_all(request.as_bytes()).await.unwrap();
        let mut response = Vec::new();
        stream.read_to_end(&mut response).await.unwrap();
        let text = String::from_utf8(response).expect("utf8 response");
        assert!(text.ends_with('\n'), "response must be newline-terminated");
        assert_eq!(text.lines().count(), 1, "response must be a single line");
        serde_json::from_str(text.trim()).expect("json response")
    }

    #[tokio::test]
    async fn ping_answers_with_the_version() {
        let (path, _state) = spawn_server("ping", Config::default());
        let response = roundtrip(&path, "{\"cmd\": \"ping\"}\n").await;
        assert_eq!(response["ok"], json!(true));
        assert_eq!(
            response["data"]["version"],
            json!(env!("CARGO_PKG_VERSION"))
        );
        assert!(response["text"].as_str().unwrap().starts_with("plrd "));
    }

    #[tokio::test]
    async fn eof_terminated_requests_are_accepted() {
        // No trailing newline: the client half-closes instead.
        let (path, _state) = spawn_server("eof", Config::default());
        let mut stream = tokio::net::UnixStream::connect(&path).await.unwrap();
        stream.write_all(b"{\"cmd\": \"ping\"}").await.unwrap();
        stream.shutdown().await.unwrap();
        let mut response = Vec::new();
        stream.read_to_end(&mut response).await.unwrap();
        let value: Value = serde_json::from_slice(&response).unwrap();
        assert_eq!(value["ok"], json!(true));
    }

    #[tokio::test]
    async fn partial_writes_are_reassembled() {
        let (path, _state) = spawn_server("partial", Config::default());
        let mut stream = tokio::net::UnixStream::connect(&path).await.unwrap();
        for chunk in ["{\"cmd\"", ": \"pi", "ng\"}\n"] {
            stream.write_all(chunk.as_bytes()).await.unwrap();
            stream.flush().await.unwrap();
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let mut response = Vec::new();
        stream.read_to_end(&mut response).await.unwrap();
        let value: Value = serde_json::from_slice(&response).unwrap();
        assert_eq!(value["ok"], json!(true));
    }

    #[tokio::test]
    async fn malformed_and_unknown_requests_answer_instead_of_dropping() {
        let (path, _state) = spawn_server("malformed", Config::default());
        for (request, outcome) in [
            ("this is not json\n", "malformed"),
            ("[1, 2, 3]\n", "malformed"),
            ("{\"no_cmd\": true}\n", "malformed"),
            ("{\"cmd\": \"frobnicate\"}\n", "unknown-cmd"),
            ("{\"cmd\": 7}\n", "malformed"),
        ] {
            let response = roundtrip(&path, request).await;
            assert_eq!(response["ok"], json!(false), "{request}");
            assert_eq!(response["data"]["outcome"], json!(outcome), "{request}");
            assert!(response["text"].as_str().is_some(), "{request}");
        }
    }

    #[tokio::test]
    async fn oversized_request_line_gets_an_error_then_close() {
        let (path, _state) = spawn_server("oversized", Config::default());
        let mut stream = tokio::net::UnixStream::connect(&path).await.unwrap();
        // A newline-free flood beyond the cap.
        let flood = vec![b'x'; MAX_REQUEST_BYTES + 4096];
        // The server may close while we are still writing; ignore
        // write errors and read whatever answer was sent.
        let _ = stream.write_all(&flood).await;
        let mut response = Vec::new();
        let _ = stream.read_to_end(&mut response).await;
        let value: Value = serde_json::from_slice(&response).expect("error response");
        assert_eq!(value["ok"], json!(false));
        assert_eq!(value["data"]["outcome"], json!("oversized"));
    }

    #[tokio::test]
    async fn concurrent_connections_are_all_served() {
        let (path, _state) = spawn_server("concurrent", Config::default());
        let mut tasks = Vec::new();
        for _ in 0..8 {
            let path = path.clone();
            tasks.push(tokio::spawn(async move {
                roundtrip(&path, "{\"cmd\": \"ping\"}\n").await
            }));
        }
        for task in tasks {
            let response = task.await.unwrap();
            assert_eq!(response["ok"], json!(true));
        }
    }

    #[tokio::test]
    async fn status_reports_recorder_and_machine_state() {
        // The e2e fixture gives a real WAL dir with a pending-able WAL
        // and a commissioned legacy machine (klippy unreachable →
        // legacy fallback note).
        let (_dir, config) = crate::pipeline::e2e_tests::fixture("ctrl-status");
        // A pending file so the summary path is exercised.
        let pending = crate::detect::PendingRecovery {
            detected_wall_ns: 1,
            file: "/g/part.gcode".to_owned(),
            file_position: 500,
            file_size: Some(1000),
            percent: Some(50.0),
            crash_class: "HostDeathOrPowerLoss".to_owned(),
        };
        crate::detect::write_pending(&config.wal_dir, &pending).unwrap();
        let (path, _state) = spawn_server("status", config);
        let response = roundtrip(&path, "{\"cmd\": \"status\"}\n").await;
        assert_eq!(response["ok"], json!(true), "{response}");
        let text = response["text"].as_str().unwrap();
        assert!(text.contains("segments: 1"), "{text}");
        assert!(text.contains("pending recovery: /g/part.gcode"), "{text}");
        assert!(text.contains("machine-config mode: legacy"), "{text}");
        assert!(text.contains("machine validation: OK"), "{text}");
        assert_eq!(response["data"]["segments"], json!(1));
        assert_eq!(response["data"]["machine_mode"], json!("legacy"));
        assert_eq!(response["data"]["pending"]["file_position"], json!(500));
    }

    #[tokio::test]
    async fn recover_dryrun_returns_the_rendered_plan_and_sends_nothing() {
        let (_dir, config) = crate::pipeline::e2e_tests::fixture("ctrl-dryrun");
        // Unreachable Moonraker on purpose: the dry-run path must
        // never need it.
        let (path, _state) = spawn_server("dryrun", config);
        let response = roundtrip(&path, "{\"cmd\": \"recover_dryrun\"}\n").await;
        assert_eq!(response["ok"], json!(true), "{response}");
        assert_eq!(response["data"]["outcome"], json!("plan"));
        let text = response["text"].as_str().unwrap();
        assert!(text.contains("dead-reckoning recovery plan"), "{text}");
        assert!(text.contains("DRY RUN"), "{text}");
        assert!(text.contains("nothing was sent"), "{text}");
    }

    #[tokio::test]
    async fn recover_execute_demands_confirm_and_rejects_step_mode() {
        let (path, _state) = spawn_server("consent", Config::default());
        let response = roundtrip(&path, "{\"cmd\": \"recover_execute\"}\n").await;
        assert_eq!(response["ok"], json!(false));
        assert!(response["text"]
            .as_str()
            .unwrap()
            .contains("\"confirm\": true"));
        let response = roundtrip(
            &path,
            "{\"cmd\": \"recover_execute\", \"args\": {\"confirm\": true, \"step\": true}}\n",
        )
        .await;
        assert_eq!(response["ok"], json!(false));
        assert_eq!(response["text"], json!("per-step mode is CLI-only"));
        assert_eq!(response["data"]["outcome"], json!("refused"));
    }

    /// A stateful fake printer: applies the gcode it receives to a
    /// small state machine and answers `printer.objects.query` from
    /// it, so the FULL pipeline plan (shifted frame, probe, true-Z,
    /// entry, file select, M24) verifies end to end. `happy_handler`'s
    /// static answers cannot do that — every declaration/motion step
    /// verifies the state it just changed.
    #[derive(Debug)]
    struct SimPrinter {
        file_root: String,
        idle_timeout: f64,
        idle_state: String,
        steppers: std::collections::BTreeMap<String, bool>,
        extruder: (f64, f64),
        bed: (f64, f64),
        position: [f64; 4],
        homed: String,
        absolute_xyz: bool,
        speed_factor: f64,
        extrude_factor: f64,
        file_path: String,
        file_position: u64,
        is_active: bool,
        last_z_result: Option<f64>,
    }

    impl SimPrinter {
        fn new(file_root: &str) -> Self {
            Self {
                file_root: file_root.to_owned(),
                idle_timeout: 600.0,
                idle_state: "Ready".to_owned(),
                steppers: std::collections::BTreeMap::new(),
                extruder: (22.0, 0.0),
                bed: (22.0, 0.0),
                position: [0.0, 0.0, 0.0, 0.0],
                homed: String::new(),
                absolute_xyz: true,
                speed_factor: 1.0,
                extrude_factor: 1.0,
                file_path: String::new(),
                file_position: 0,
                is_active: false,
                last_z_result: None,
            }
        }

        /// `X10` / `Z=-1.15` / `S150` → the numeric payload.
        fn word_value(word: &str) -> Option<f64> {
            let rest = word.split_once('=').map_or_else(|| &word[1..], |(_, v)| v);
            rest.parse().ok()
        }

        fn axis_value(words: &[&str], axis: char) -> Option<f64> {
            words
                .iter()
                .find(|w| w.starts_with(axis) || w.starts_with(&format!("{axis}=")))
                .and_then(|w| Self::word_value(w))
        }

        fn apply_gcode(&mut self, script: &str) {
            let words: Vec<&str> = script.split_whitespace().collect();
            let Some(&command) = words.first() else {
                return;
            };
            match command {
                "SET_IDLE_TIMEOUT" => {
                    if let Some(v) = Self::axis_value(&words[1..], 'T') {
                        self.idle_timeout = v;
                    }
                }
                "SET_STEPPER_ENABLE" => {
                    if let Some(name) = words.iter().find_map(|w| w.strip_prefix("STEPPER=")) {
                        self.steppers.insert(name.to_owned(), true);
                    }
                }
                "M104" => {
                    if let Some(v) = Self::axis_value(&words[1..], 'S') {
                        self.extruder = (v, v); // instant heater
                    }
                }
                "M140" => {
                    if let Some(v) = Self::axis_value(&words[1..], 'S') {
                        self.bed = (v, v);
                    }
                }
                "G28" => {
                    self.homed.push_str("xy");
                    self.position[0] = 0.0;
                    self.position[1] = 0.0;
                }
                "SET_KINEMATIC_POSITION" => {
                    if let Some(v) = Self::axis_value(&words[1..], 'Z') {
                        self.position[2] = v;
                        if !self.homed.contains('z') {
                            self.homed.push('z');
                        }
                    }
                }
                "G90" => self.absolute_xyz = true,
                "G91" => self.absolute_xyz = false,
                "G0" | "G1" => {
                    for (index, axis) in ['X', 'Y', 'Z'].into_iter().enumerate() {
                        if let Some(v) = Self::axis_value(&words[1..], axis) {
                            if self.absolute_xyz {
                                self.position[index] = v;
                            } else {
                                self.position[index] += v;
                            }
                        }
                    }
                }
                "PROBE" => {
                    // Descend 0.35 mm and trigger; SAMPLES=1 halts at
                    // the trigger.
                    let trigger = self.position[2] - 0.35;
                    self.position[2] = trigger;
                    self.last_z_result = Some(trigger);
                }
                "M220" => {
                    if let Some(v) = Self::axis_value(&words[1..], 'S') {
                        self.speed_factor = v / 100.0;
                    }
                }
                "M221" => {
                    if let Some(v) = Self::axis_value(&words[1..], 'S') {
                        self.extrude_factor = v / 100.0;
                    }
                }
                "M23" => {
                    if let Some(name) = words.get(1) {
                        self.file_path = format!("{}/{name}", self.file_root);
                        self.file_position = 0;
                    }
                }
                "M26" => {
                    if let Some(v) = Self::axis_value(&words[1..], 'S') {
                        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                        {
                            self.file_position = v as u64;
                        }
                    }
                }
                "M24" => {
                    self.is_active = true;
                    self.idle_state = "Printing".to_owned();
                }
                _ => {} // offsets, fans, modes plrd never verifies
            }
        }

        fn status_of(&self, object: &str) -> Value {
            match object {
                "webhooks" => json!({"state": "ready"}),
                "print_stats" => json!({"state": "standby"}),
                "idle_timeout" => {
                    json!({"idle_timeout": self.idle_timeout, "state": self.idle_state})
                }
                "stepper_enable" => json!({"steppers": self.steppers}),
                "extruder" => {
                    json!({"temperature": self.extruder.0, "target": self.extruder.1})
                }
                "heater_bed" => json!({"temperature": self.bed.0, "target": self.bed.1}),
                "toolhead" => {
                    json!({"position": self.position, "homed_axes": self.homed})
                }
                "gcode_move" => json!({
                    "speed_factor": self.speed_factor,
                    "extrude_factor": self.extrude_factor,
                }),
                "virtual_sdcard" => json!({
                    "is_active": self.is_active,
                    "file_path": self.file_path,
                    "file_position": self.file_position,
                }),
                "probe" => json!({"last_z_result": self.last_z_result}),
                _ => json!({}),
            }
        }
    }

    fn sim_handler(
        sim: Arc<std::sync::Mutex<SimPrinter>>,
    ) -> impl Fn(&str, &Value) -> Result<Value, (i64, String)> + Send + Sync + 'static {
        move |method, params| {
            let mut sim = sim.lock().expect("sim lock");
            match method {
                "printer.gcode.script" => {
                    let script = params["script"].as_str().unwrap_or("");
                    sim.apply_gcode(script);
                    Ok(json!("ok"))
                }
                "printer.objects.query" => {
                    let mut status = serde_json::Map::new();
                    if let Some(objects) = params["objects"].as_object() {
                        for name in objects.keys() {
                            status.insert(name.clone(), sim.status_of(name));
                        }
                    }
                    Ok(json!({"eventtime": 1.0, "status": status}))
                }
                other => Err((-32601, format!("Method not found: {other}"))),
            }
        }
    }

    #[tokio::test]
    async fn recover_execute_happy_path_runs_the_full_gate_stack() {
        let (_dir, mut config) = crate::pipeline::e2e_tests::fixture("ctrl-exec");
        let sim = Arc::new(std::sync::Mutex::new(SimPrinter::new(
            config.wal_dir.to_str().unwrap(),
        )));
        let fake = FakeMoonraker::spawn(sim_handler(Arc::clone(&sim))).await;
        config.moonraker_url = fake.url();
        // A stale pending file must be cleared on completion (same as
        // the CLI path).
        std::fs::write(config.wal_dir.join(crate::detect::PENDING_FILE_NAME), b"{}").unwrap();
        let wal_dir = config.wal_dir.clone();
        let (path, _state) = spawn_server("exec", config);
        let response = roundtrip(
            &path,
            "{\"cmd\": \"recover_execute\", \"args\": {\"confirm\": true}}\n",
        )
        .await;
        assert_eq!(response["ok"], json!(true), "{response}");
        assert_eq!(response["data"]["outcome"], json!("completed"));
        let text = response["text"].as_str().unwrap();
        assert!(text.contains("transcript:"), "{text}");
        assert!(text.contains("COMPLETED"), "{text}");
        // The printer got exactly the plan's commands (the fixture plan
        // ends with M24 etc.; assert the fingerprint commands).
        let sent = fake.gcode_sent();
        assert!(
            sent.iter().any(|c| c.starts_with("SET_IDLE_TIMEOUT")),
            "{sent:?}"
        );
        assert!(sent.iter().any(|c| c == "M24"), "{sent:?}");
        // Transcript on disk; pending file cleared.
        let transcript = std::fs::read_dir(&wal_dir)
            .unwrap()
            .filter_map(Result::ok)
            .find(|e| {
                e.file_name()
                    .to_string_lossy()
                    .starts_with("recovery-transcript-")
            })
            .expect("transcript file");
        let transcript_text = std::fs::read_to_string(transcript.path()).unwrap();
        assert!(
            transcript_text.contains("plan-complete"),
            "{transcript_text}"
        );
        assert!(!wal_dir.join(crate::detect::PENDING_FILE_NAME).exists());
    }

    #[tokio::test]
    async fn recover_execute_refuses_when_printer_not_idle_and_sends_nothing() {
        let fake = FakeMoonraker::spawn(|method, params| {
            let mut v = happy_handler(method, params)?;
            if method == "printer.objects.query" {
                if let Some(ps) = v.get_mut("status").and_then(|s| s.get_mut("print_stats")) {
                    *ps = json!({"state": "printing"});
                }
            }
            Ok(v)
        })
        .await;
        let (_dir, mut config) = crate::pipeline::e2e_tests::fixture("ctrl-notidle");
        config.moonraker_url = fake.url();
        let (path, _state) = spawn_server("notidle", config);
        let response = roundtrip(
            &path,
            "{\"cmd\": \"recover_execute\", \"args\": {\"confirm\": true}}\n",
        )
        .await;
        assert_eq!(response["ok"], json!(false));
        assert!(response["text"].as_str().unwrap().contains("not idle"));
        assert!(fake.gcode_sent().is_empty(), "refusal must send nothing");
    }

    #[tokio::test]
    async fn concurrent_recover_execute_is_busy_not_queued() {
        let state = fast_state(Config::default());
        // Simulate an in-flight execution by holding the lock.
        let guard = state.exec_lock.try_lock().unwrap();
        let response = respond_line(
            &state,
            "{\"cmd\": \"recover_execute\", \"args\": {\"confirm\": true}}",
        )
        .await;
        assert_eq!(response["ok"], json!(false));
        assert_eq!(response["data"]["outcome"], json!("busy"));
        drop(guard);
    }

    #[tokio::test]
    async fn dryrun_reports_refusals_as_typed_text() {
        // An empty WAL dir: the pipeline errors (no segments); the
        // response is ok:false with the reason in text.
        let dir = temp_dir("ctrl-dryrun-empty");
        let config = Config {
            wal_dir: dir,
            ..Config::default()
        };
        let (path, _state) = spawn_server("dryrun-empty", config);
        let response = roundtrip(&path, "{\"cmd\": \"recover_dryrun\"}\n").await;
        assert_eq!(response["ok"], json!(false));
        assert!(response["text"]
            .as_str()
            .unwrap()
            .contains("no WAL segments"));
    }

    #[test]
    fn bind_replaces_a_stale_socket_and_reports_bad_paths() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = temp_dir("bind");
        let path = dir.join("plrd.sock");
        // First bind creates it; a second bind (stale file present)
        // must succeed by unlinking first.
        let first = bind(&path).expect("first bind");
        drop(first);
        assert!(path.exists(), "socket file survives the listener");
        let second = bind(&path).expect("rebind over stale socket");
        drop(second);
        // Permissions: 0666 (module docs).
        let mode = std::fs::metadata(&path).unwrap().permissions().mode();
        assert_eq!(mode & 0o777, 0o666, "mode {mode:o}");
        // An unbindable path (parent is a regular file) is a clear
        // error, not a panic.
        let blocker = dir.join("blocker");
        std::fs::write(&blocker, b"file").unwrap();
        let err = bind(&blocker.join("x.sock")).unwrap_err();
        assert!(err.contains("cannot bind control socket"), "{err}");
    }
}
