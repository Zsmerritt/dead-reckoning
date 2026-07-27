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
//! * Commands: `ping`, `status`, `recover_state`, `recover_dryrun`,
//!   `recover_execute` (args `{"confirm": true, "step": bool,
//!   "on_confirm": "abort"|"ask"}`), `recover_confirm` (args `{"token":
//!   string, "answer": "continue"|"abort"}`).
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
//! mode is CLI-only".
//!
//! Executions are serialized: a second `recover_execute` while one is
//! running (or paused) gets an immediate `busy` error (`try_lock`, never
//! queued — a queued recovery executing minutes later against a changed
//! printer would be indefensible).
//!
//! # Confirm-points: `on_confirm` and `recover_confirm`
//!
//! `recover_execute` takes an optional `on_confirm`:
//!
//! * omitted or `"abort"` (the default) — any
//!   [`plr_recovery::Tier::Confirmable`] diagnosis aborts the recovery.
//!   This is exactly the behaviour a non-interactive caller had before
//!   confirm-points existed, and it is the default precisely so that
//!   adding them changed nothing for such callers.
//! * `"ask"` — execution PAUSES at each confirm-point. The
//!   `recover_execute` response is then `ok:false` with
//!   `data.outcome = "awaiting_confirmation"`, the structured
//!   `data.diagnosis`, an opaque `data.resume_token`, and the deadline:
//!   `data.confirm_timeout_s` (the bound the executor is enforcing for
//!   this pause) plus `data.confirm_expires_in_s` (what is left of it as
//!   the response is written). Both come off
//!   [`crate::executor::ConfirmPoint::deadline`], the same field the
//!   executor's timer was constructed from — a client never has to guess,
//!   and in particular never has to assume the top of the permitted band
//!   to stay fail-safe.
//!
//! The paused execution is a live task, not stored state to be replayed:
//! it is sitting on one `await`. The client answers with
//! `recover_confirm {"token": ..., "answer": "continue"|"abort"}`, whose
//! response is whatever happens next — another
//! `awaiting_confirmation`, or the final `completed` / `aborted-or-refused`.
//! A token that is not the outstanding one (wrong, stale, or already
//! answered) gets a typed `unknown-token` error, never a silent no-op:
//! an operator who thinks they answered and did not is the worst
//! possible state to leave somebody in during a recovery.
//!
//! One paused execution at a time, through the same serialization as any
//! other execution. A pause never holds the runtime: it is a spawned
//! task awaiting a channel, so the recorder keeps recording throughout,
//! and the executor's own [`crate::executor::ExecOptions::confirm_timeout`]
//! bounds it — a client that goes away resolves to a clean abort, with
//! frame invalidation applied exactly as any other abort at that step
//! would apply it.
//!
//! # `recover_state`: asking without acting
//!
//! `recover_execute` and `recover_confirm` were the only commands that
//! could see [`CtrlState::session`], so the only way to find out whether a
//! recovery was in flight was to *try to start one* and read the `busy`
//! refusal — telling an operator to run a command that moves the machine
//! in order to learn whether the machine is moving. `recover_state` answers
//! that directly:
//!
//! ```text
//! {"cmd": "recover_state"}
//! -> {"ok": true, "text": "...", "data": {
//!      "outcome": "state",
//!      "executing": bool,              // same window as the `busy` refusal
//!      "running_for_s": number|null,
//!      "awaiting_confirmation": bool,
//!      "confirm_expired": bool,        // a pause lapsed unanswered
//!      "resume_token": string|null,
//!      "confirm_kind": string|null,    // "diagnosis"|"z-height"|"step-debug"
//!      "step": number|null,
//!      "phase": string|null,
//!      "diagnosis_code": string|null,
//!      "confirm_timeout_s": number|null,
//!      "confirm_expires_in_s": number|null}}
//! ```
//!
//! It is a **new** command and every field it introduces elsewhere is
//! additive, so an older plugin — which never sends it and ignores unknown
//! keys — is unaffected.
//!
//! **It cannot act**, and that is a property of its type rather than of its
//! body: [`cmd_recover_state`] is handed `&`[`Observed`], so the session
//! slot and the outstanding pause's `oneshot::Sender` are not reachable from
//! it, and [`render_state`] is handed a [`StateSnapshot`] of
//! `bool`/`String`/`f64`. Being synchronous is *not* what buys this —
//! `try_lock`, `Sender::send` and `std::fs::write` are all synchronous, so a
//! handler holding `&CtrlState` could answer an operator's Z-height
//! confirmation without ever awaiting. See [`Observed`] for the locking
//! argument.
//!
//! **On exposing the token — what it enables.** The resume token is a
//! *correlation* identifier, not a capability: it exists so a stale or
//! duplicate answer cannot be mistaken for a fresh one, and it never
//! authenticated anybody. This socket has no authentication at all (mode
//! 0666 by design, see below), so any client that can read the token could
//! already have sent `recover_execute {"confirm": true}` and could reach the
//! same printer through Moonraker's own unauthenticated local API.
//!
//! Be precise about what changes, though, because one thing does. The
//! confirm-points exist to require *a human at the machine* — the Z-height
//! pause asks somebody to look at the nozzle. Before `recover_state`, a
//! client could only answer a pause it had **received**; now any local
//! client can **discover** the token and answer it, including "continue",
//! having never been shown the diagnosis. That is a genuine widening, not
//! merely re-discovery of something already held.
//!
//! It is judged worth it because the state it fixes is worse: if the
//! connection that received the pause goes away (klippy restarts the
//! plugin, the operator reloads the browser) then *nobody* can answer today,
//! and the recovery sits paused with the heaters on and the nozzle over the
//! part until the deadline expires. An operator on a multi-user host who
//! does not want that widening has the same remedy as for every other
//! mutating command here: tighten the socket's mode (below).
//!
//! # Diagnoses on the wire
//!
//! Every diagnosis in every response is the same JSON object
//! ([`plr_recovery::Diagnosis`]): `code`, `tier`, `what`, `why`,
//! `suggested_fix`, `measured`, `expected`, `override_key`. Clients
//! branch on `code` and `tier` and render the rest verbatim, so one
//! renderer covers refusals, warnings and pauses alike.
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

use std::future::Future;
use std::path::Path;
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use plr_recovery::Diagnose;
use serde_json::{json, Map, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::config::Config;
use crate::executor::{
    AbortConfirmer, ConfirmAnswer, ConfirmPoint, Confirmer, ExecOptions, PreviewAnswer,
};
use crate::pipeline::{self, MachineSource, PipelineOutcome};
use crate::recover::{self, AutoGate, RecoverOptions};
use crate::EXIT_OK;

/// Hard cap on one request line (documented in the module docs).
pub const MAX_REQUEST_BYTES: usize = 64 * 1024;

/// Process-lifetime counter behind every resume token, so a token from
/// an earlier execution can never be mistaken for the current one.
static TOKEN_SEQ: AtomicU64 = AtomicU64::new(0);

fn next_token() -> String {
    let seq = TOKEN_SEQ.fetch_add(1, Ordering::Relaxed);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos());
    format!("plrc-{nanos:x}-{seq:x}")
}

/// One outstanding confirm-point: what the operator is being asked, and
/// the one-shot channel their answer travels back down.
struct PauseNotice {
    token: String,
    kind: &'static str,
    step_id: u32,
    phase: String,
    diagnosis: plr_recovery::Diagnosis,
    detail: Value,
    /// The deadline the executor is holding this pause to — read straight
    /// off [`ConfirmPoint::deadline`], which is the field the executor's
    /// own `tokio::time::timeout` was constructed from.
    deadline: Duration,
    /// When the deadline started running, on the monotonic clock.
    ///
    /// Sampled at the top of [`SocketConfirmer::confirm`], which is the
    /// first thing the executor's timeout wrapper polls, so it trails the
    /// real timer start by the cost of one future poll. Wall-clock time is
    /// deliberately not used: a client needs to know how long is left, and
    /// an NTP step must not be able to answer that question wrongly.
    raised_at: std::time::Instant,
    answer: tokio::sync::oneshot::Sender<PauseAnswer>,
}

/// The answer travelling back down a [`PauseNotice`]'s channel. Unified so
/// the single-flight machinery (one outstanding pause, one `oneshot`) is
/// unchanged by preview beyond the vocabulary widening (design §D.3): a
/// binary `ask` pause carries [`Self::Binary`], a preview reposition pause
/// carries [`Self::Preview`]. A confirmer that receives the wrong variant
/// (which `cmd_recover_confirm` never sends — it chooses by the outstanding
/// pause's kind) fails closed to abort.
enum PauseAnswer {
    Binary(ConfirmAnswer),
    Preview(PreviewAnswer),
}

/// The [`Confirmer`] the socket installs under `on_confirm: "ask"`:
/// publishes each confirm-point to the connection handler and waits for
/// an answer to come back.
///
/// A closed channel (nobody is listening any more) answers `Abort`. That
/// is the fail-closed direction and it is the only defensible one: an
/// unanswerable question must not be treated as a yes.
struct SocketConfirmer {
    pauses: tokio::sync::mpsc::Sender<PauseNotice>,
    /// Where each pause is mirrored for [`cmd_recover_state`] to read.
    /// Published here rather than in the connection handler so the mirror
    /// is written by the same task, next to the same `await`, that the
    /// executor's deadline is being measured against.
    observed: Arc<Observed>,
}

impl Confirmer for SocketConfirmer {
    fn confirm<'a>(
        &'a mut self,
        point: &'a ConfirmPoint,
    ) -> Pin<Box<dyn Future<Output = ConfirmAnswer> + Send + 'a>> {
        Box::pin(async move {
            let (tx, rx) = tokio::sync::oneshot::channel();
            let notice = PauseNotice {
                token: next_token(),
                kind: point.kind.tag(),
                step_id: point.step_id,
                phase: point.phase.clone(),
                diagnosis: point.diagnosis.clone(),
                detail: point.detail.clone(),
                // NOT re-derived from the config or from
                // `DEFAULT_CONFIRM_TIMEOUT`: the executor stamped the
                // deadline it is enforcing onto the point.
                deadline: point.deadline,
                raised_at: std::time::Instant::now(),
                answer: tx,
            };
            self.observed.publish_pause(&notice);
            if self.pauses.send(notice).await.is_err() {
                self.observed.clear_pause();
                return ConfirmAnswer::Abort;
            }
            let answer = match rx.await {
                Ok(PauseAnswer::Binary(a)) => a,
                // A closed channel or a preview answer to a binary pause
                // (which cannot happen — cmd_recover_confirm sends by kind)
                // both fail closed: an unanswerable question is not a yes.
                _ => ConfirmAnswer::Abort,
            };
            self.observed.clear_pause();
            answer
        })
    }

    fn confirm_preview<'a>(
        &'a mut self,
        point: &'a ConfirmPoint,
    ) -> Pin<Box<dyn Future<Output = PreviewAnswer> + Send + 'a>> {
        Box::pin(async move {
            let (tx, rx) = tokio::sync::oneshot::channel();
            let notice = PauseNotice {
                token: next_token(),
                kind: point.kind.tag(),
                step_id: point.step_id,
                phase: point.phase.clone(),
                diagnosis: point.diagnosis.clone(),
                detail: point.detail.clone(),
                deadline: point.deadline,
                raised_at: std::time::Instant::now(),
                answer: tx,
            };
            self.observed.publish_pause(&notice);
            if self.pauses.send(notice).await.is_err() {
                self.observed.clear_pause();
                return PreviewAnswer::Abort;
            }
            let answer = match rx.await {
                Ok(PauseAnswer::Preview(a)) => a,
                // Fail closed: a closed channel or a binary answer to a
                // preview pause both abort (never commit a resume nobody
                // chose).
                _ => PreviewAnswer::Abort,
            };
            self.observed.clear_pause();
            answer
        })
    }
}

/// The plain-data mirror of the session that `recover_state` reports.
///
/// # Why a mirror instead of reading the session
///
/// [`CtrlState::session`] cannot answer a status query. It is the
/// serialization lock, and `cmd_recover_execute` holds it across every
/// `await` of a recovery — the pipeline, the execution, a pause lasting up
/// to `confirm_timeout_s`. A `try_lock` from a status handler would
/// therefore fail for exactly as long as there was something to report,
/// and a `lock().await` would make the status query wait out the recovery
/// it was asking about.
///
/// So the execute path *publishes* into this, and the status path reads
/// only this. Consequences, stated as the properties they are:
///
/// * **A status query cannot stall the executor and cannot be stalled by
///   it.** Every critical section here is straight-line code over
///   `bool`/`String`/`Instant` with no `await` and no I/O inside it, held
///   for tens of nanoseconds. It is a `std::sync::Mutex` precisely so that
///   holding it across an `await` is not expressible here: the guard is
///   `!Send`, and every async context that touches it is `Send`-bounded —
///   connections are spawned tasks, and [`Confirmer::confirm`] returns a
///   boxed `Send` future — so such a hold is a compile error rather than a
///   review finding.
/// * **No lock cycle.** The execute path takes `session` then this; the
///   status path takes this alone. There is no path that takes them in the
///   other order.
/// * **Poisoning cannot silence observability.** A panic elsewhere while
///   the lock was held would leave plain data, not a torn invariant, so a
///   poisoned lock is recovered into rather than propagated. Refusing to
///   answer "are you still paused?" is not a safe failure mode.
#[derive(Debug, Default)]
pub struct Observed {
    inner: std::sync::Mutex<Option<LiveExecution>>,
}

/// One `recover_execute` in progress, as the mirror sees it.
#[derive(Debug)]
struct LiveExecution {
    /// When [`ExecutionLease::begin`] was taken.
    started: std::time::Instant,
    /// The outstanding confirm-point, if execution is paused on one.
    pause: Option<LivePause>,
}

/// The mirror of one outstanding confirm-point.
#[derive(Debug, Clone)]
struct LivePause {
    token: String,
    kind: &'static str,
    step_id: u32,
    phase: String,
    code: &'static str,
    raised_at: std::time::Instant,
    deadline: Duration,
}

/// Everything `recover_state` is allowed to see: plain owned data, and no
/// handle to anything that could act.
///
/// This type is the guarantee asked of the read-only query. The renderer
/// ([`render_state`]) takes *this* and nothing else — no `&CtrlState`, no
/// `JoinHandle`, no `oneshot::Sender` — so "the status path cannot start,
/// answer or cancel a recovery" is a property of what it is handed rather
/// than a property of what its body happens not to say. There is no
/// method on `bool`, `String` or `f64` that resumes a recovery.
#[derive(Debug, Default, PartialEq)]
struct StateSnapshot {
    /// A `recover_execute` is in progress (the same condition that makes a
    /// second one `busy`).
    executing: bool,
    /// How long it has been in progress.
    running_for_s: Option<f64>,
    /// It is paused on a confirm-point whose deadline has NOT lapsed.
    awaiting: bool,
    /// The outstanding pause's details, when `awaiting`.
    token: Option<String>,
    kind: Option<&'static str>,
    step_id: Option<u32>,
    phase: Option<String>,
    code: Option<&'static str>,
    /// The full deadline the executor is enforcing, seconds.
    confirm_timeout_s: Option<f64>,
    /// What is left of it, seconds.
    expires_in_s: Option<f64>,
    /// A pause was raised and its deadline lapsed without an answer.
    ///
    /// Distinguishes "never paused" from "paused, and you missed it" so a
    /// client can retire an operator prompt with a reason instead of
    /// silently dropping it.
    confirm_expired: bool,
}

impl Observed {
    /// Publishes "an execution is in progress". Called by
    /// [`ExecutionLease::begin`] only.
    fn begin(&self) {
        *self.guard() = Some(LiveExecution {
            started: std::time::Instant::now(),
            pause: None,
        });
    }

    /// Publishes "nothing is in progress". Called by
    /// [`ExecutionLease`]'s `Drop` only, so it runs on completion, on a
    /// panic inside the execution task, and on the task being dropped by a
    /// runtime shutdown.
    fn end(&self) {
        *self.guard() = None;
    }

    fn publish_pause(&self, notice: &PauseNotice) {
        if let Some(live) = self.guard().as_mut() {
            live.pause = Some(LivePause {
                token: notice.token.clone(),
                kind: notice.kind,
                step_id: notice.step_id,
                phase: notice.phase.clone(),
                code: notice.diagnosis.code,
                raised_at: notice.raised_at,
                deadline: notice.deadline,
            });
        }
    }

    fn clear_pause(&self) {
        if let Some(live) = self.guard().as_mut() {
            live.pause = None;
        }
    }

    /// The snapshot as of now.
    fn snapshot(&self) -> StateSnapshot {
        self.snapshot_at(std::time::Instant::now())
    }

    /// The snapshot as of `now` — a pure function of the mirror and the
    /// clock, which is what makes the expiry rule testable.
    ///
    /// **The expiry rule is why this is derived at read time rather than
    /// trusted as published.** The executor cancels
    /// `SocketConfirmer::confirm` when the deadline fires, so the future
    /// that would have cleared the pause is dropped instead of completing:
    /// a mirror that only ever believed what it was told would keep
    /// claiming a confirmation was outstanding after the daemon had
    /// already aborted the recovery. That is the exact defect this query
    /// exists to remove from the plugin, and reintroducing it one layer
    /// down would be worse than not answering at all. So a lapsed pause
    /// reads as *not* awaiting — the conservative answer for a client
    /// deciding whether to keep asking an operator for something.
    fn snapshot_at(&self, now: std::time::Instant) -> StateSnapshot {
        let guard = self.guard();
        let Some(live) = guard.as_ref() else {
            return StateSnapshot::default();
        };
        let mut snapshot = StateSnapshot {
            executing: true,
            running_for_s: Some(now.saturating_duration_since(live.started).as_secs_f64()),
            ..StateSnapshot::default()
        };
        if let Some(pause) = live.pause.as_ref() {
            let elapsed = now.saturating_duration_since(pause.raised_at);
            match pause.deadline.checked_sub(elapsed) {
                Some(left) if !left.is_zero() => {
                    snapshot.awaiting = true;
                    snapshot.token = Some(pause.token.clone());
                    snapshot.kind = Some(pause.kind);
                    snapshot.step_id = Some(pause.step_id);
                    snapshot.phase = Some(pause.phase.clone());
                    snapshot.code = Some(pause.code);
                    snapshot.confirm_timeout_s = Some(pause.deadline.as_secs_f64());
                    snapshot.expires_in_s = Some(left.as_secs_f64());
                }
                // Lapsed (or a deadline of zero, which is the never-stamped
                // sentinel): report it as gone, and say so.
                _ => snapshot.confirm_expired = true,
            }
        }
        snapshot
    }

    /// The lock, recovering from poisoning (see the type docs).
    fn guard(&self) -> std::sync::MutexGuard<'_, Option<LiveExecution>> {
        self.inner
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

/// Publishes "an execution is in progress" for as long as it exists.
///
/// Taken by `cmd_recover_execute` the moment the busy check passes, and
/// moved into the execution task, so `executing` covers the pipeline as
/// well as execution proper — i.e. exactly the window in which a second
/// `recover_execute` would be refused as `busy`. `Drop` is what publishes
/// the end, so completion, a panic, and a runtime shutdown that drops the
/// task all clear it; nothing has to *run* successfully for the mirror to
/// stop claiming an execution is live.
struct ExecutionLease(Arc<Observed>);

impl ExecutionLease {
    fn begin(observed: Arc<Observed>) -> Self {
        observed.begin();
        Self(observed)
    }
}

impl Drop for ExecutionLease {
    fn drop(&mut self) {
        self.0.end();
    }
}

/// A recovery execution in flight: running, or paused on a
/// confirm-point. Exactly one can exist at a time.
struct ExecSession {
    /// Confirm-points arriving from the execution task.
    pauses: tokio::sync::mpsc::Receiver<PauseNotice>,
    /// The execution task: `(exit code, operator-facing report)`.
    join: tokio::task::JoinHandle<(u8, String)>,
    /// The pause currently awaiting an answer, if any.
    outstanding: Option<PauseNotice>,
    /// Everything the pipeline printed before execution started, so the
    /// final report reads the same as the CLI's.
    prefix: String,
}

/// Shared state of the control server.
pub struct CtrlState {
    /// The daemon configuration (paths, Moonraker URL, machine
    /// section).
    pub config: Config,
    /// Executor timing for `recover_execute` (tests shrink these).
    pub exec_options: ExecOptions,
    /// Moonraker connect timeout for `recover_execute`.
    pub connect_timeout: Duration,
    /// Serializes `recover_execute` AND holds the paused execution
    /// between `recover_execute` and `recover_confirm` (module docs).
    /// `try_lock` failure and an occupied slot both mean "busy".
    session: tokio::sync::Mutex<Option<ExecSession>>,
    /// The read-only mirror `recover_state` reports from. See
    /// [`Observed`] for why the session itself cannot answer.
    observed: Arc<Observed>,
}

impl CtrlState {
    /// Production state for a daemon config.
    #[must_use]
    pub fn new(config: Config) -> Self {
        Self {
            config,
            exec_options: ExecOptions::default(),
            connect_timeout: Duration::from_secs(10),
            session: tokio::sync::Mutex::new(None),
            observed: Arc::new(Observed::default()),
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
        // Handed `&state.observed`, NOT `&state`. That is what makes the
        // handler unable to act, and it is deliberate: being synchronous is
        // not enough on its own — `try_lock`, `oneshot::Sender::send` and
        // `std::fs::write` are all synchronous, so a handler holding
        // `&CtrlState` could answer somebody's Z-height confirmation
        // without ever awaiting. Narrowing the argument removes the
        // capability instead of asking future authors not to use it.
        "recover_state" => cmd_recover_state(&state.observed),
        "recover_dryrun" => cmd_recover_dryrun(state).await,
        "recover_execute" => cmd_recover_execute(state, args).await,
        "recover_confirm" => cmd_recover_confirm(state, args).await,
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

/// `recover_state`: the read-only "is a recovery in flight, and is it
/// waiting on me?" query (module docs).
///
/// **Takes `&Observed`, not `&CtrlState`.** The session and the outstanding
/// confirm-point's `oneshot::Sender` are simply not reachable from here, so
/// "this handler cannot start, answer or cancel a recovery" is a fact about
/// its signature rather than a promise about its body. An earlier version
/// took `&CtrlState` and argued the point from being synchronous; that was
/// false — `try_lock`, `Sender::send` and `std::fs::write` are all
/// synchronous, and a reviewer answered a live Z-height confirmation from
/// inside this function to prove it.
fn cmd_recover_state(observed: &Observed) -> Value {
    render_state(&observed.snapshot())
}

/// Renders a [`StateSnapshot`]. Takes plain data by reference and returns
/// JSON: there is nothing here to act with even in principle.
fn render_state(snapshot: &StateSnapshot) -> Value {
    use std::fmt::Write as _;
    let mut text = String::new();
    if snapshot.executing {
        let _ = write!(text, "execution: in flight");
        if let Some(seconds) = snapshot.running_for_s {
            let _ = write!(text, " ({seconds:.1}s)");
        }
        let _ = writeln!(text);
    } else {
        let _ = writeln!(text, "execution: none");
    }
    if snapshot.awaiting {
        let _ = writeln!(
            text,
            "awaiting confirmation: step {} [{}] {} ({}); {:.1}s of {:.1}s left",
            snapshot.step_id.unwrap_or(0),
            snapshot.phase.as_deref().unwrap_or("?"),
            snapshot.kind.unwrap_or("?"),
            snapshot.code.unwrap_or("?"),
            snapshot.expires_in_s.unwrap_or(0.0),
            snapshot.confirm_timeout_s.unwrap_or(0.0),
        );
    } else if snapshot.confirm_expired {
        let _ = writeln!(
            text,
            "awaiting confirmation: no — a confirm-point lapsed unanswered; the recovery \
             is aborting or has aborted"
        );
    } else {
        let _ = writeln!(text, "awaiting confirmation: no");
    }
    ok_response(
        text.trim_end(),
        &json!({
            "outcome": "state",
            "executing": snapshot.executing,
            "running_for_s": snapshot.running_for_s,
            "awaiting_confirmation": snapshot.awaiting,
            "confirm_expired": snapshot.confirm_expired,
            "resume_token": snapshot.token,
            "confirm_kind": snapshot.kind,
            "step": snapshot.step_id,
            "phase": snapshot.phase,
            "diagnosis_code": snapshot.code,
            "confirm_timeout_s": snapshot.confirm_timeout_s,
            "confirm_expires_in_s": snapshot.expires_in_s,
        }),
    )
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
                    // No plan: the clean-nozzle confirmation flag is
                    // always present, and false when there is no plan.
                    false,
                    Value::Array(Vec::new()),
                );
            }
        };
        // The wizard reads `data.requires_clean_nozzle_confirmation` to
        // decide whether to prompt "confirm the nozzle is clean". Emit it
        // for every outcome (false unless a plan set it) so the field is
        // always present and never ambiguous.
        let requires_clean_nozzle_confirmation = match &outcome {
            PipelineOutcome::Plan(bundle) => bundle.plan.requires_clean_nozzle_confirmation,
            _ => false,
        };
        // Every diagnosis this outcome carries, in the frozen wire shape:
        // a plan's warnings (each with its tier, so the wizard can show
        // which ones will stop execution under `on_confirm: "ask"`), or a
        // machine rejection's failed prerequisites.
        let diagnoses = match &outcome {
            PipelineOutcome::Plan(bundle) => Value::Array(
                bundle
                    .plan
                    .warnings
                    .iter()
                    .map(|w| serde_json::to_value(w.diagnosis()).unwrap_or(Value::Null))
                    .collect(),
            ),
            other => outcome_diagnoses(other).unwrap_or_else(|| Value::Array(Vec::new())),
        };
        let options = RecoverOptions::new(false, false, false);
        let mut stdin = std::io::Cursor::new(Vec::new());
        let code = recover::drive(&outcome, &config, &options, &mut stdin, &mut out);
        (
            code == EXIT_OK,
            String::from_utf8_lossy(&out).into_owned(),
            outcome_tag(&outcome).to_owned(),
            requires_clean_nozzle_confirmation,
            diagnoses,
        )
    })
    .await;
    match result {
        Ok((ok, text, outcome, requires_clean_nozzle_confirmation, diagnoses)) => {
            json!({
                "ok": ok,
                "text": text,
                "data": {
                    "outcome": outcome,
                    // Top-level (not nested): the wizard reads it first.
                    "requires_clean_nozzle_confirmation": requires_clean_nozzle_confirmation,
                    "diagnoses": diagnoses,
                },
            })
        }
        Err(e) => error_response("error", &format!("dry-run task failed: {e}")),
    }
}

fn outcome_tag(outcome: &PipelineOutcome) -> &'static str {
    match outcome {
        PipelineOutcome::CleanShutdown => "clean-shutdown",
        // A distinct tag: the console UI must be able to say "the print
        // finished" rather than "the log ended cleanly", which is false
        // for a print whose host died during the cooldown.
        PipelineOutcome::Complete(_) => "complete",
        PipelineOutcome::MachineRejected(_) => "machine-rejected",
        PipelineOutcome::ManualFallback(_) => "manual-fallback",
        PipelineOutcome::NotPossible(_) => "not-possible",
        PipelineOutcome::Plan(_) => "plan",
    }
}

/// `recover_execute`: consent check, serialization, pipeline, then the
/// shared gate stack (`recover::execute_with_gates`) on a spawned task
/// so a confirm-point pause can be reported while the execution is still
/// alive.
#[allow(clippy::too_many_lines)] // linear: argument gates, then setup
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
    // `on_confirm`: omitted / "abort" keeps the pre-confirm-point
    // behaviour (a Confirmable diagnosis aborts); "ask" pauses. An
    // unrecognized value is refused rather than guessed — silently
    // treating a typo as "abort" would mean a client that asked to be
    // consulted never is.
    let ask = match args.get("on_confirm") {
        None | Some(Value::Null) => false,
        Some(Value::String(s)) if s == "abort" => false,
        Some(Value::String(s)) if s == "ask" => true,
        Some(other) => {
            return error_response(
                "malformed",
                &format!("on_confirm must be \"abort\" or \"ask\", got {other}"),
            )
        }
    };

    // Serialize executions; a concurrent request is refused, not
    // queued (module docs).
    let Ok(mut slot) = state.session.try_lock() else {
        return error_response("busy", "another recover_execute is already running");
    };
    if let Some(session) = slot.as_ref() {
        if !session.join.is_finished() {
            return error_response(
                "busy",
                "another recover_execute is already running or awaiting confirmation",
            );
        }
        // A previous execution ran to completion (most likely by timing
        // out an unanswered pause) but nobody collected it. Reap it so
        // this request is not blocked by a corpse.
        *slot = None;
    }

    // Publish "in flight" from HERE — the point past which a second
    // request is refused as busy — so `recover_state` and the busy refusal
    // describe the same window, pipeline included. Dropped on every early
    // return below; moved into the execution task on the success path.
    let lease = ExecutionLease::begin(Arc::clone(&state.observed));

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
        let mut data = json!({"outcome": outcome_tag(&outcome)});
        if let Some(list) = outcome_diagnoses(&outcome) {
            data["diagnoses"] = list;
        }
        return json!({
            "ok": false,
            "text": String::from_utf8_lossy(&out).into_owned(),
            "data": data,
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
    let prefix = String::from_utf8_lossy(&out).into_owned();

    // Execution runs as its own task, owning clones of everything it
    // needs: a paused execution must keep existing after this request's
    // response has been written, and it must never sit on the runtime
    // thread the recorder depends on.
    let (pause_tx, pause_rx) = tokio::sync::mpsc::channel::<PauseNotice>(1);
    let bundle = bundle.clone();
    let config = state.config.clone();
    let exec_options = state.exec_options.clone();
    let connect_timeout = state.connect_timeout;
    let observed = Arc::clone(&state.observed);
    let join = tokio::spawn(async move {
        // Owned by the task from here on: whenever the task stops existing
        // — return, panic, or being dropped by a runtime shutdown — this
        // drops and the mirror stops claiming an execution is live.
        let _lease = lease;
        let mut report: Vec<u8> = Vec::new();
        let mut abort_confirmer = AbortConfirmer;
        let mut socket_confirmer = SocketConfirmer {
            pauses: pause_tx,
            observed,
        };
        let confirmer: &mut dyn Confirmer = if ask {
            &mut socket_confirmer
        } else {
            &mut abort_confirmer
        };
        let code = recover::execute_with_gates(
            &bundle,
            &config,
            &exec_options,
            connect_timeout,
            &mut AutoGate,
            confirmer,
            &mut report,
        )
        .await;
        (code, String::from_utf8_lossy(&report).into_owned())
    });

    *slot = Some(ExecSession {
        pauses: pause_rx,
        join,
        outstanding: None,
        prefix,
    });
    drive_session(&mut slot).await
}

/// `recover_confirm`: answers the outstanding confirm-point and reports
/// whatever happens next.
/// The [`ConfirmKind::Preview`] tag, as it appears in a [`PauseNotice`]'s
/// `kind` — the discriminator `cmd_recover_confirm` uses to choose the
/// answer vocabulary.
const PREVIEW_KIND_TAG: &str = "preview";

/// Parses a resume-preview reposition answer from the socket args (design
/// §D.3 / §F.2). Vocabulary: `accept` / `next` / `prev` / `abort`, and
/// `nudge` with a signed integer `count` (the console `PLR_RECOVER_NUDGE
/// FWD=/BACK=` maps to +n / -n). `Err(msg)` is a malformed answer.
fn parse_preview_answer(args: &Map<String, Value>) -> Result<PreviewAnswer, String> {
    match args.get("answer").and_then(Value::as_str) {
        Some("accept") => Ok(PreviewAnswer::Accept),
        Some("next") => Ok(PreviewAnswer::NextRep),
        Some("prev") => Ok(PreviewAnswer::PrevRep),
        Some("abort") => Ok(PreviewAnswer::Abort),
        Some("nudge") => {
            let count = args.get("count").and_then(Value::as_i64).ok_or_else(|| {
                "recover_confirm \"nudge\" requires an integer \"count\"".to_owned()
            })?;
            let count = i32::try_from(count)
                .map_err(|_| "recover_confirm \"count\" is out of range".to_owned())?;
            if count == 0 {
                return Err("recover_confirm \"nudge\" count must be non-zero".to_owned());
            }
            Ok(PreviewAnswer::Nudge(count))
        }
        _ => Err("recover_confirm (preview) requires \"answer\": \
                  accept | next | prev | nudge | abort"
            .to_owned()),
    }
}

async fn cmd_recover_confirm(state: &CtrlState, args: &Map<String, Value>) -> Value {
    let Some(token) = args.get("token").and_then(Value::as_str) else {
        return error_response("malformed", "recover_confirm requires a string \"token\"");
    };
    // The answer must be one of the recognized verbs across BOTH
    // vocabularies — checked before the lock so an absent or nonsense
    // answer reports the same "malformed" it always has, whether or not
    // anything is paused. Whether a recognized verb is valid for the
    // OUTSTANDING pause's kind (binary continue/abort vs preview
    // accept/next/prev/nudge) is decided below, once the lock reveals the
    // kind.
    match args.get("answer").and_then(Value::as_str) {
        Some("continue" | "abort" | "accept" | "next" | "prev" | "nudge") => {}
        _ => {
            return error_response(
                "malformed",
                "recover_confirm requires \"answer\": continue|abort (binary) or \
                 accept|next|prev|nudge|abort (preview)",
            )
        }
    }
    let Ok(mut slot) = state.session.try_lock() else {
        return error_response("busy", "the paused execution is being answered already");
    };
    let Some(session) = slot.as_mut() else {
        return error_response(
            "unknown-token",
            "no execution is awaiting confirmation; the token is unknown or expired",
        );
    };
    let Some(outstanding) = session.outstanding.take() else {
        return error_response(
            "unknown-token",
            "the execution is running but not awaiting confirmation; the token is expired",
        );
    };
    if outstanding.token != token {
        // Put it back: a wrong token must not consume somebody else's
        // pending question.
        session.outstanding = Some(outstanding);
        return error_response(
            "unknown-token",
            "that token does not match the outstanding confirmation",
        );
    }
    // The answer vocabulary depends on which KIND of pause is outstanding
    // (design §D.3): a binary `ask`/diagnosis/step-debug/z-height pause
    // takes continue/abort; the resume PREVIEW pause takes the reposition
    // verbs accept/next/prev/nudge/abort (`nudge` carrying a signed
    // `count`). A wrong-vocabulary answer is malformed — but the pause is
    // put back first so a typo does not consume the outstanding question.
    let answer = if outstanding.kind == PREVIEW_KIND_TAG {
        match parse_preview_answer(args) {
            Ok(a) => PauseAnswer::Preview(a),
            Err(msg) => {
                session.outstanding = Some(outstanding);
                return error_response("malformed", &msg);
            }
        }
    } else {
        match args.get("answer").and_then(Value::as_str) {
            Some("continue") => PauseAnswer::Binary(ConfirmAnswer::Continue),
            Some("abort") => PauseAnswer::Binary(ConfirmAnswer::Abort),
            _ => {
                session.outstanding = Some(outstanding);
                return error_response(
                    "malformed",
                    "recover_confirm requires \"answer\": \"continue\" or \"abort\"",
                );
            }
        }
    };
    if outstanding.answer.send(answer).is_err() {
        // The pause timed out between the lookup and the send. The
        // execution is already aborting; report it as expired rather than
        // pretending the answer landed.
        return error_response(
            "unknown-token",
            "the confirmation timed out before the answer arrived; the recovery aborted",
        );
    }
    drive_session(&mut slot).await
}

/// Waits for the session's next event — another confirm-point, or the
/// execution finishing — and renders it. Clears the slot on completion.
async fn drive_session(slot: &mut Option<ExecSession>) -> Value {
    /// What the session did next.
    enum Next {
        Pause(Box<PauseNotice>),
        Done(Result<(u8, String), tokio::task::JoinError>),
    }
    let Some(session) = slot.as_mut() else {
        return error_response("error", "no execution session");
    };
    let next = tokio::select! {
        // Biased so a pause that is already queued is reported as a
        // pause, never lost to a simultaneously-ready join.
        biased;
        notice = session.pauses.recv() => notice.map(|n| Next::Pause(Box::new(n))),
        result = &mut session.join => Some(Next::Done(result)),
    };
    // `recv()` yielding `None` means every sender is gone, i.e. the task
    // has dropped its confirmer — so the join arm was not taken and the
    // handle is still pollable below.
    let finished = match next {
        Some(Next::Done(result)) => Some(result),
        Some(Next::Pause(notice)) => {
            let notice = *notice;
            return report_pause(session, notice);
        }
        None => None,
    };
    let Some(session) = slot.take() else {
        return error_response("error", "no execution session");
    };
    let prefix = session.prefix;
    let result = match finished {
        Some(result) => result,
        None => session.join.await,
    };
    match result {
        Ok((code, report)) => json!({
            "ok": code == EXIT_OK,
            "text": format!("{prefix}{report}"),
            "data": {
                "outcome": if code == EXIT_OK { "completed" } else { "aborted-or-refused" },
                "exit": code,
            },
        }),
        Err(e) => error_response("error", &format!("execution task failed: {e}")),
    }
}

/// Renders an `awaiting_confirmation` response and stores the pause as
/// the session's outstanding question.
fn report_pause(session: &mut ExecSession, notice: PauseNotice) -> Value {
    // The remaining budget at the moment the response is composed. It is
    // the total deadline minus what the pause has already spent getting
    // here (one channel hop), so a client that adds it to "now" lands on
    // the executor's own abort instant rather than on a guess. Saturating,
    // and reported as 0 when the deadline has already gone — never as a
    // negative number a client might sign-extend into the future.
    let expires_in = notice
        .deadline
        .checked_sub(notice.raised_at.elapsed())
        .unwrap_or_default();
    let text = format!(
        "{}recover: PAUSED at step {} [{}] awaiting confirmation ({}); answer within \
         {:.0}s or the recovery aborts\n{}",
        session.prefix,
        notice.step_id,
        notice.phase,
        notice.kind,
        expires_in.as_secs_f64(),
        notice.diagnosis.full(),
    );
    let response = json!({
        "ok": false,
        "text": text,
        "data": {
            "outcome": "awaiting_confirmation",
            "resume_token": notice.token,
            "confirm_kind": notice.kind,
            "step": notice.step_id,
            "phase": notice.phase,
            "diagnosis": notice.diagnosis,
            "detail": notice.detail,
            // ADDITIVE (a client that ignores both behaves exactly as
            // before). `confirm_timeout_s` is the deadline the executor is
            // enforcing — the operator's `[plr] confirm_timeout_s` when
            // set, else the daemon default — read off the ConfirmPoint the
            // executor's own timer was built from, so a client never has
            // to assume the band ceiling again.
            "confirm_timeout_s": notice.deadline.as_secs_f64(),
            "confirm_expires_in_s": expires_in.as_secs_f64(),
        },
    });
    session.outstanding = Some(notice);
    response
}

/// The diagnoses a non-plan pipeline outcome carries, in the frozen
/// wire shape, so a client renders a machine rejection exactly the way it
/// renders every other diagnosis.
fn outcome_diagnoses(outcome: &PipelineOutcome) -> Option<Value> {
    match outcome {
        PipelineOutcome::MachineRejected(rejection) => Some(Value::Array(
            rejection
                .failures
                .iter()
                .map(|f| serde_json::to_value(f.diagnosis()).unwrap_or(Value::Null))
                .collect(),
        )),
        PipelineOutcome::CleanShutdown
        | PipelineOutcome::Complete(_)
        | PipelineOutcome::ManualFallback(_)
        | PipelineOutcome::NotPossible(_)
        | PipelineOutcome::Plan(_) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        bind, respond_line, serve, CtrlState, ExecutionLease, Observed, PauseNotice, StateSnapshot,
        MAX_REQUEST_BYTES,
    };
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
            confirm_timeout: Duration::from_millis(300),
            gcode_barrier_timeout: Duration::from_millis(300),
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
            frame_invalid: false,
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
        // The wizard reads this top-level flag. The legacy fixture has no
        // clean-nozzle macro, so the plan requires operator confirmation.
        assert_eq!(
            response["data"]["requires_clean_nozzle_confirmation"],
            json!(true),
            "{response}"
        );
        let text = response["text"].as_str().unwrap();
        assert!(text.contains("dead-reckoning recovery plan"), "{text}");
        assert!(text.contains("DRY RUN"), "{text}");
        assert!(text.contains("nothing was sent"), "{text}");
    }

    #[tokio::test]
    async fn recover_dryrun_confirmation_flag_is_present_and_false_without_a_plan() {
        // No WAL directory: the pipeline fails before producing a plan;
        // the confirmation flag is still present in `data`, and false.
        let (path, _state) = spawn_server("dryrun-noplan", Config::default());
        let response = roundtrip(&path, "{\"cmd\": \"recover_dryrun\"}\n").await;
        assert!(response["data"]["requires_clean_nozzle_confirmation"].is_boolean());
        assert_eq!(
            response["data"]["requires_clean_nozzle_confirmation"],
            json!(false),
            "{response}"
        );
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
        /// The consensus-touch median the [plr]-mode plan reads back
        /// (`plr.last_touch_result.median_z`), plus the spread and sample
        /// count its post-verifications check.
        last_touch: Option<(f64, f64, f64)>,
        max_accel: f64,
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
                last_touch: None,
                max_accel: 3_000.0,
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
                "PLR_TOUCH" => {
                    // The plugin descends, touches, and leaves the
                    // toolhead resting retracted above the contact; the
                    // median it reports is in the z_offset-subtracted
                    // bed-probing frame (see TriggerSource::TouchResult).
                    let trigger = self.position[2] - 0.35;
                    self.position[2] = trigger;
                    let samples = Self::axis_value(&words[1..], 'S').unwrap_or(3.0);
                    self.last_touch = Some((trigger + 0.1, 0.001, samples));
                }
                "SET_VELOCITY_LIMIT" => {
                    if let Some(v) = words.iter().find_map(|w| w.strip_prefix("ACCEL=")) {
                        if let Ok(v) = v.parse::<f64>() {
                            self.max_accel = v;
                        }
                    }
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
                "toolhead" => json!({
                    "position": self.position,
                    "homed_axes": self.homed,
                    "max_accel": self.max_accel,
                }),
                "plr" => match self.last_touch {
                    Some((median, range, used)) => json!({
                        "last_touch_result": {
                            "median_z": median,
                            "range": range,
                            "samples_used": used,
                        }
                    }),
                    None => json!({"last_touch_result": {}}),
                },
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
        // Simulate an in-flight execution by holding the session lock.
        let guard = state.session.try_lock().unwrap();
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

    // --- Confirm-points over the real socket ------------------------------
    //
    // Everything below drives a bound UNIX socket with real connections,
    // a real spawned execution task and a real (simulated) printer: the
    // point of the exercise is the protocol, and a protocol tested
    // through its own dispatch function is a protocol tested twice and
    // shipped never.

    /// A `[plr]`-mode server whose plan carries the confirm-point keys,
    /// driving the stateful [`SimPrinter`].
    ///
    /// `[plr]` mode rather than the legacy fixture because
    /// `confirm_z_before_resume` / `debug_confirm_each_step` live in
    /// printer.cfg's `[plr]` section — the legacy `/etc/plrd.conf`
    /// `[machine]` path predates them and deliberately does not carry
    /// them.
    #[cfg(unix)]
    async fn spawn_confirm_server(
        tag: &str,
        plr_overrides: &[(&str, Value)],
    ) -> (PathBuf, PathBuf, FakeMoonraker) {
        let (path, wal_dir, fake, _sim) = spawn_confirm_server_sim(tag, plr_overrides).await;
        (path, wal_dir, fake)
    }

    /// [`spawn_confirm_server`], also handing back the simulated printer so
    /// a test can change the machine's state *while* execution is paused —
    /// which is the only way to reproduce the case where a job starts
    /// during an operator's confirm pause.
    #[cfg(unix)]
    #[allow(clippy::type_complexity)] // four related handles, not a struct
    async fn spawn_confirm_server_sim(
        tag: &str,
        plr_overrides: &[(&str, Value)],
    ) -> (
        PathBuf,
        PathBuf,
        FakeMoonraker,
        Arc<std::sync::Mutex<SimPrinter>>,
    ) {
        let (_dir, mut config) = crate::pipeline::e2e_tests::plr_fixture(tag, plr_overrides);
        let sim = Arc::new(std::sync::Mutex::new(SimPrinter::new(
            config.wal_dir.to_str().unwrap(),
        )));
        let fake = FakeMoonraker::spawn(sim_handler(Arc::clone(&sim))).await;
        config.moonraker_url = fake.url();
        let wal_dir = config.wal_dir.clone();
        let (path, state) = spawn_server(tag, config);
        // The state must outlive this helper: leak it deliberately (the
        // test process is the lifetime).
        std::mem::forget(state);
        (path, wal_dir, fake, sim)
    }

    const EXECUTE_ASK: &str =
        "{\"cmd\": \"recover_execute\", \"args\": {\"confirm\": true, \"on_confirm\": \"ask\"}}\n";

    fn confirm_request(token: &str, answer: &str) -> String {
        format!(
            "{{\"cmd\": \"recover_confirm\", \"args\": {{\"token\": \"{token}\", \
             \"answer\": \"{answer}\"}}}}\n"
        )
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn z_confirmation_round_trips_pause_then_continue_then_completes() {
        let (path, wal_dir, fake) = spawn_confirm_server(
            "confirm-z-continue",
            &[("confirm_z_before_resume", json!(true))],
        )
        .await;
        let paused = roundtrip(&path, EXECUTE_ASK).await;
        assert_eq!(paused["ok"], json!(false), "{paused}");
        assert_eq!(
            paused["data"]["outcome"],
            json!("awaiting_confirmation"),
            "{paused}"
        );
        assert_eq!(paused["data"]["confirm_kind"], json!("z-height"));
        // BACKWARD COMPATIBILITY. Every key an older plugin reads is still
        // present with the same meaning; the deadline fields are purely
        // additive, and a client that ignores them behaves exactly as it
        // did. Asserted as a list so removing or renaming one fails here
        // rather than in somebody's console during a recovery.
        for key in [
            "outcome",
            "resume_token",
            "confirm_kind",
            "step",
            "phase",
            "diagnosis",
            "detail",
        ] {
            assert!(
                paused["data"].get(key).is_some(),
                "the frozen `awaiting_confirmation` key {key} went missing: {paused}"
            );
        }
        // The diagnosis is the same JSON object as every other diagnosis.
        let d = &paused["data"]["diagnosis"];
        assert_eq!(d["code"], json!("z_confirm_before_resume"));
        assert_eq!(d["tier"], json!("confirmable"));
        for key in ["what", "why", "suggested_fix", "override_key"] {
            assert!(d.get(key).is_some(), "missing {key} in {d}");
        }
        assert!(!d["why"].as_str().unwrap().is_empty());
        // The believed Z and its derivation are reported as data.
        assert!(paused["data"]["detail"]["derivation"]
            .as_str()
            .unwrap()
            .contains("z_prev_top"));
        let token = paused["data"]["resume_token"]
            .as_str()
            .expect("resume token")
            .to_owned();
        assert!(token.starts_with("plrc-"), "{token}");

        // Answering `continue` resumes the SAME execution and runs it to
        // completion — the print is started, so this is the whole point.
        let done = roundtrip(&path, &confirm_request(&token, "continue")).await;
        assert_eq!(done["ok"], json!(true), "{done}");
        assert_eq!(done["data"]["outcome"], json!("completed"));
        assert!(
            fake.gcode_sent().iter().any(|c| c == "M24"),
            "{:?}",
            fake.gcode_sent()
        );
        // A completed recovery leaves no frame-invalid marker.
        assert!(crate::detect::read_frame_invalid(&wal_dir).is_none());
        // The pause and the answer are in the transcript, with the
        // diagnosis that caused them.
        let transcript = read_transcript(&wal_dir);
        assert!(
            transcript.contains("\"event\":\"confirm-pause\""),
            "{transcript}"
        );
        assert!(
            transcript.contains("z_confirm_before_resume"),
            "{transcript}"
        );
        assert!(
            transcript.contains("\"answer\":\"continue\""),
            "{transcript}"
        );
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn answering_abort_stops_cleanly_and_honors_the_frame_rule() {
        let (path, wal_dir, fake) = spawn_confirm_server(
            "confirm-z-abort",
            &[("confirm_z_before_resume", json!(true))],
        )
        .await;
        let paused = roundtrip(&path, EXECUTE_ASK).await;
        let token = paused["data"]["resume_token"].as_str().unwrap().to_owned();
        let done = roundtrip(&path, &confirm_request(&token, "abort")).await;
        assert_eq!(done["ok"], json!(false), "{done}");
        assert_eq!(done["data"]["outcome"], json!("aborted-or-refused"));
        let text = done["text"].as_str().unwrap();
        assert!(text.contains("confirmation-declined"), "{text}");
        // The Z-confirm pause sits after the shifted-frame declare, so
        // declining leaves the frame unknown — and the marker says so, so
        // a re-execute is refused until a fresh dry run.
        assert!(text.contains("Z frame is now UNKNOWN"), "{text}");
        let marker = crate::detect::read_frame_invalid(&wal_dir).expect("frame-invalid marker");
        assert_eq!(marker.reason, "confirmation-declined");
        // The print was never started.
        assert!(!fake.gcode_sent().iter().any(|c| c == "M24"));
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn an_unknown_or_expired_token_is_a_typed_error_never_a_silent_no_op() {
        let (path, _wal_dir, _fake) =
            spawn_confirm_server("confirm-token", &[("confirm_z_before_resume", json!(true))])
                .await;
        // Nothing is paused yet.
        let response = roundtrip(&path, &confirm_request("plrc-nope", "continue")).await;
        assert_eq!(response["ok"], json!(false));
        assert_eq!(response["data"]["outcome"], json!("unknown-token"));

        let paused = roundtrip(&path, EXECUTE_ASK).await;
        let token = paused["data"]["resume_token"].as_str().unwrap().to_owned();
        // A wrong token must not consume the outstanding question...
        let response = roundtrip(&path, &confirm_request("plrc-wrong", "continue")).await;
        assert_eq!(
            response["data"]["outcome"],
            json!("unknown-token"),
            "{response}"
        );
        // ...so the real one still works.
        let done = roundtrip(&path, &confirm_request(&token, "continue")).await;
        assert_eq!(done["ok"], json!(true), "{done}");
        // And once the execution is over the token is expired.
        let response = roundtrip(&path, &confirm_request(&token, "continue")).await;
        assert_eq!(
            response["data"]["outcome"],
            json!("unknown-token"),
            "{response}"
        );
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn recover_confirm_rejects_malformed_arguments() {
        let (path, _wal_dir, _fake) = spawn_confirm_server("confirm-malformed", &[]).await;
        for (request, outcome) in [
            ("{\"cmd\": \"recover_confirm\"}\n", "malformed"),
            (
                "{\"cmd\": \"recover_confirm\", \"args\": {\"token\": \"t\"}}\n",
                "malformed",
            ),
            (
                "{\"cmd\": \"recover_confirm\", \"args\": {\"token\": \"t\", \"answer\": \"maybe\"}}\n",
                "malformed",
            ),
            (
                "{\"cmd\": \"recover_confirm\", \"args\": {\"token\": 7, \"answer\": \"abort\"}}\n",
                "malformed",
            ),
        ] {
            let response = roundtrip(&path, request).await;
            assert_eq!(response["ok"], json!(false), "{request}");
            assert_eq!(response["data"]["outcome"], json!(outcome), "{request}");
        }
    }

    #[tokio::test]
    async fn recover_execute_rejects_an_unrecognized_on_confirm() {
        let (path, _state) = spawn_server("on-confirm-bad", Config::default());
        let response = roundtrip(
            &path,
            "{\"cmd\": \"recover_execute\", \"args\": {\"confirm\": true, \"on_confirm\": \"maybe\"}}\n",
        )
        .await;
        assert_eq!(response["ok"], json!(false));
        assert_eq!(response["data"]["outcome"], json!("malformed"));
        assert!(response["text"].as_str().unwrap().contains("on_confirm"));
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn an_unanswered_pause_times_out_into_a_clean_abort_with_the_frame_rule() {
        let (_dir, mut config) = crate::pipeline::e2e_tests::plr_fixture(
            "confirm-timeout",
            &[("confirm_z_before_resume", json!(true))],
        );
        let sim = Arc::new(std::sync::Mutex::new(SimPrinter::new(
            config.wal_dir.to_str().unwrap(),
        )));
        let fake = FakeMoonraker::spawn(sim_handler(Arc::clone(&sim))).await;
        config.moonraker_url = fake.url();
        let wal_dir = config.wal_dir.clone();
        let (path, state) = spawn_server("confirm-timeout", config);

        // Pause, then answer nothing at all. The executor's own
        // confirm_timeout (300 ms in tests) ends it.
        let paused = roundtrip(&path, EXECUTE_ASK).await;
        assert_eq!(paused["data"]["outcome"], json!("awaiting_confirmation"));
        let token = paused["data"]["resume_token"].as_str().unwrap().to_owned();
        // A second execute while the pause is outstanding is busy, never
        // queued — one paused execution at a time.
        let busy = roundtrip(&path, EXECUTE_ASK).await;
        assert_eq!(busy["data"]["outcome"], json!("busy"), "{busy}");

        // Wait past the timeout, then observe the aborted state.
        tokio::time::sleep(Duration::from_millis(900)).await;
        let response = roundtrip(&path, &confirm_request(&token, "continue")).await;
        assert_eq!(
            response["data"]["outcome"],
            json!("unknown-token"),
            "an expired token is typed, never silently accepted: {response}"
        );
        // The abort really happened, with the frame invalidated exactly
        // as a decline at the same step would have done.
        let marker = crate::detect::read_frame_invalid(&wal_dir).expect("frame-invalid marker");
        assert_eq!(marker.reason, "confirmation-timeout");
        assert!(!fake.gcode_sent().iter().any(|c| c == "M24"));
        let transcript = read_transcript(&wal_dir);
        assert!(
            transcript.contains("\"answer\":\"timeout\""),
            "{transcript}"
        );
        // The finished-but-uncollected session is reaped, not left to
        // block the socket forever: the next execute gets past the busy
        // gate (and then refuses on the frame marker, which is correct).
        let next = roundtrip(&path, EXECUTE_ASK).await;
        assert_ne!(next["data"]["outcome"], json!("busy"), "{next}");
        drop(state);
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn step_debug_pauses_before_every_step_over_the_socket() {
        let (path, wal_dir, fake) = spawn_confirm_server(
            "confirm-stepdebug",
            &[("debug_confirm_each_step", json!(true))],
        )
        .await;
        let mut response = roundtrip(&path, EXECUTE_ASK).await;
        let mut pauses = 0;
        while response["data"]["outcome"] == json!("awaiting_confirmation") {
            assert_eq!(response["data"]["confirm_kind"], json!("step-debug"));
            let d = &response["data"]["diagnosis"];
            assert_eq!(d["code"], json!("step_debug_pause"));
            // The commands about to be sent are reported, which is the
            // entire purpose of the mode.
            assert!(
                response["data"]["detail"]["commands"].is_array(),
                "{response}"
            );
            let token = response["data"]["resume_token"]
                .as_str()
                .unwrap()
                .to_owned();
            response = roundtrip(&path, &confirm_request(&token, "continue")).await;
            pauses += 1;
            assert!(pauses < 100, "runaway pause loop");
        }
        assert_eq!(response["ok"], json!(true), "{response}");
        assert!(pauses >= 10, "one pause per step, got {pauses}");
        assert!(fake.gcode_sent().iter().any(|c| c == "M24"));
        let transcript = read_transcript(&wal_dir);
        assert!(transcript.contains("step_debug_pause"), "{transcript}");
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn confirm_points_are_inert_when_the_keys_are_unset() {
        // Same [plr] machine, no confirm keys: `on_confirm: "ask"` never
        // pauses, and the transcript carries no confirm-point events at
        // all. Enabling the feature is what turns it on; asking to be
        // asked is not.
        let (path, wal_dir, fake) = spawn_confirm_server("confirm-inert", &[]).await;
        let response = roundtrip(&path, EXECUTE_ASK).await;
        assert_eq!(response["ok"], json!(true), "{response}");
        assert_eq!(response["data"]["outcome"], json!("completed"));
        assert!(fake.gcode_sent().iter().any(|c| c == "M24"));
        let transcript = read_transcript(&wal_dir);
        for absent in ["confirm-pause", "confirm-answer", "z_confirm_before_resume"] {
            assert!(!transcript.contains(absent), "{absent}: {transcript}");
        }
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn accel_overrides_appear_in_the_plan_and_restore_on_completion() {
        let (path, wal_dir, fake) = spawn_confirm_server(
            "confirm-accel",
            &[
                ("recovery_accel", json!(1500.0)),
                ("accel_home", json!(1000.0)),
            ],
        )
        .await;
        let response = roundtrip(&path, EXECUTE_ASK).await;
        assert_eq!(response["ok"], json!(true), "{response}");
        let sent = fake.gcode_sent();
        assert!(
            sent.iter().any(|c| c == "SET_VELOCITY_LIMIT ACCEL=1500"),
            "{sent:?}"
        );
        assert!(
            sent.iter().any(|c| c == "SET_VELOCITY_LIMIT ACCEL=1000"),
            "{sent:?}"
        );
        // The machine's own accel (3000 in the sim) is put back BEFORE
        // the recovery file starts: a resumed print must not inherit a
        // recovery acceleration.
        let restore = sent
            .iter()
            .rposition(|c| c == "SET_VELOCITY_LIMIT ACCEL=3000")
            .expect("machine accel restored");
        let m24 = sent.iter().position(|c| c == "M24").expect("M24");
        assert!(restore < m24, "{sent:?}");
        let transcript = read_transcript(&wal_dir);
        assert!(transcript.contains("record-machine-accel"), "{transcript}");
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn an_out_of_band_accel_refuses_with_a_diagnosis() {
        let (_dir, config) = crate::pipeline::e2e_tests::plr_fixture(
            "confirm-accel-bad",
            &[("recovery_accel", json!(1.0))],
        );
        let (path, _state) = spawn_server("confirm-accel-bad", config);
        let response = roundtrip(&path, "{\"cmd\": \"recover_dryrun\"}\n").await;
        assert_eq!(response["ok"], json!(false), "{response}");
        let text = response["text"].as_str().unwrap();
        assert!(text.contains("recovery_accel"), "{text}");
        assert!(text.contains("outside"), "{text}");
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn a_dry_run_reports_every_warning_as_a_structured_diagnosis() {
        let (path, _state) = {
            let (_dir, config) = crate::pipeline::e2e_tests::plr_fixture("confirm-diag", &[]);
            spawn_server("confirm-diag", config)
        };
        let response = roundtrip(&path, "{\"cmd\": \"recover_dryrun\"}\n").await;
        assert_eq!(response["ok"], json!(true), "{response}");
        let diagnoses = response["data"]["diagnoses"]
            .as_array()
            .expect("diagnoses array");
        assert!(!diagnoses.is_empty(), "{response}");
        for d in diagnoses {
            for key in [
                "code",
                "tier",
                "what",
                "why",
                "suggested_fix",
                "override_key",
            ] {
                assert!(d.get(key).is_some(), "missing {key} in {d}");
            }
            assert!(
                ["advisory", "confirmable", "hard"].contains(&d["tier"].as_str().unwrap()),
                "{d}"
            );
        }
    }

    /// **The operator case.** Power loss, `PLR_RECOVER`, the plan raises a
    /// Z-height confirmation, the operator walks to the printer to look at
    /// the nozzle — and while they are away an autostart macro or a queued
    /// job begins printing. They come back and answer "continue".
    ///
    /// Both pre-execution gates passed, honestly, before the pause. The
    /// window between them and the next command is the whole confirm
    /// deadline: up to an hour at the top of the permitted band. Without the
    /// per-step re-check, answering "continue" issued the plan's remaining
    /// commands — `SET_KINEMATIC_POSITION`, `PROBE`, and finally `M24` — into
    /// a running print, and reported `COMPLETED`.
    #[cfg(unix)]
    #[tokio::test]
    async fn a_job_that_starts_during_the_confirm_pause_stops_the_resume() {
        let (path, wal_dir, fake, sim) =
            spawn_confirm_server_sim("pause-window", &[("confirm_z_before_resume", json!(true))])
                .await;
        let paused = roundtrip(&path, EXECUTE_ASK).await;
        assert_eq!(
            paused["data"]["outcome"],
            json!("awaiting_confirmation"),
            "{paused}"
        );
        let token = paused["data"]["resume_token"].as_str().unwrap().to_owned();
        // Everything up to the standoff really did run, so this is the
        // dangerous moment and not a pause before anything happened.
        let before = fake.gcode_sent();
        assert!(
            before.iter().any(|c| c.starts_with("PLR_TOUCH")),
            "the contact operation should already have run: {before:?}"
        );
        assert!(!before.iter().any(|c| c == "M24"), "{before:?}");

        // The operator is at the printer. Something else starts a job.
        sim.lock().expect("sim").is_active = true;

        // They answer "continue".
        let done = roundtrip(&path, &confirm_request(&token, "continue")).await;
        assert_eq!(done["ok"], json!(false), "{done}");
        assert_eq!(
            done["data"]["outcome"],
            json!("aborted-or-refused"),
            "{done}"
        );
        let text = done["text"].as_str().unwrap();
        assert!(text.contains("exclusive-gcode-access-lost"), "{text}");
        assert!(
            text.contains("Something else is driving this printer"),
            "{text}"
        );
        // The resume never happened.
        assert!(
            !fake.gcode_sent().iter().any(|c| c == "M24"),
            "the print must not have been started: {:?}",
            fake.gcode_sent()
        );
        // And the frame is marked unknown, so a re-execute is refused until
        // a fresh dry run — correct, because the plan is now stale.
        let marker = crate::detect::read_frame_invalid(&wal_dir).expect("frame-invalid marker");
        assert_eq!(marker.reason, "exclusive-gcode-access-lost");
        let transcript = read_transcript(&wal_dir);
        assert!(
            transcript.contains("\"event\":\"exclusivity-lost\""),
            "{transcript}"
        );
    }

    /// The complement, so the test above is not passing for the trivial
    /// reason that the re-check refuses everything: the same plan, the same
    /// pause, nothing else touching the printer — and the resume completes.
    #[cfg(unix)]
    #[tokio::test]
    async fn an_undisturbed_confirm_pause_still_resumes() {
        let (path, wal_dir, fake) = spawn_confirm_server(
            "pause-window-ok",
            &[("confirm_z_before_resume", json!(true))],
        )
        .await;
        let paused = roundtrip(&path, EXECUTE_ASK).await;
        let token = paused["data"]["resume_token"].as_str().unwrap().to_owned();
        let done = roundtrip(&path, &confirm_request(&token, "continue")).await;
        assert_eq!(done["ok"], json!(true), "{done}");
        assert_eq!(done["data"]["outcome"], json!("completed"));
        assert!(fake.gcode_sent().iter().any(|c| c == "M24"));
        assert!(crate::detect::read_frame_invalid(&wal_dir).is_none());
    }

    // --- `recover_state`: observing without acting -----------------------

    #[tokio::test]
    async fn recover_state_answers_on_an_idle_daemon_and_starts_nothing() {
        // A real WAL fixture with a real recoverable plan and a reachable
        // (simulated) printer: if `recover_state` could start anything,
        // there is something here for it to start.
        let (_dir, mut config) = crate::pipeline::e2e_tests::fixture("ctrl-state-idle");
        let sim = Arc::new(std::sync::Mutex::new(SimPrinter::new(
            config.wal_dir.to_str().unwrap(),
        )));
        let fake = FakeMoonraker::spawn(sim_handler(Arc::clone(&sim))).await;
        config.moonraker_url = fake.url();
        let wal_dir = config.wal_dir.clone();
        let (path, _state) = spawn_server("state-idle", config);

        for _ in 0..3 {
            let response = roundtrip(&path, "{\"cmd\": \"recover_state\"}\n").await;
            assert_eq!(response["ok"], json!(true), "{response}");
            assert_eq!(response["data"]["outcome"], json!("state"));
            assert_eq!(response["data"]["executing"], json!(false), "{response}");
            assert_eq!(
                response["data"]["awaiting_confirmation"],
                json!(false),
                "{response}"
            );
            assert_eq!(response["data"]["confirm_expired"], json!(false));
            assert!(response["data"]["resume_token"].is_null(), "{response}");
            assert!(response["data"]["confirm_timeout_s"].is_null());
            assert!(response["text"]
                .as_str()
                .unwrap()
                .contains("execution: none"));
        }
        // It acted on nothing: no gcode, and no transcript (which
        // `execute_with_gates` creates before it sends anything at all, so
        // its absence rules out even a refused execution having started).
        assert!(fake.gcode_sent().is_empty(), "{:?}", fake.gcode_sent());
        assert!(
            !std::fs::read_dir(&wal_dir)
                .unwrap()
                .filter_map(Result::ok)
                .any(|e| e
                    .file_name()
                    .to_string_lossy()
                    .starts_with("recovery-transcript-")),
            "recover_state must not have begun an execution"
        );
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn recover_state_reports_the_outstanding_pause_its_token_and_its_deadline() {
        // The operator's own `[plr] confirm_timeout_s`, deliberately
        // different from BOTH the daemon default and the test harness's
        // 300 ms `exec_options.confirm_timeout` — so a response that echoed
        // either of those instead of the enforced value fails here.
        let (path, _wal_dir, _fake) = spawn_confirm_server(
            "state-paused",
            &[
                ("confirm_z_before_resume", json!(true)),
                ("confirm_timeout_s", json!(90.0)),
            ],
        )
        .await;
        let paused = roundtrip(&path, EXECUTE_ASK).await;
        assert_eq!(
            paused["data"]["outcome"],
            json!("awaiting_confirmation"),
            "{paused}"
        );
        let token = paused["data"]["resume_token"].as_str().unwrap().to_owned();
        // The pause response reports the deadline the executor enforces.
        assert_eq!(paused["data"]["confirm_timeout_s"], json!(90.0), "{paused}");
        let left = paused["data"]["confirm_expires_in_s"].as_f64().unwrap();
        assert!(left > 80.0 && left <= 90.0, "{paused}");
        assert!(
            paused["text"].as_str().unwrap().contains("answer within"),
            "{paused}"
        );

        // The read-only query agrees, without being told anything.
        let state = roundtrip(&path, "{\"cmd\": \"recover_state\"}\n").await;
        assert_eq!(state["ok"], json!(true), "{state}");
        assert_eq!(state["data"]["executing"], json!(true), "{state}");
        assert_eq!(
            state["data"]["awaiting_confirmation"],
            json!(true),
            "{state}"
        );
        assert_eq!(state["data"]["resume_token"], json!(token), "{state}");
        assert_eq!(state["data"]["confirm_kind"], json!("z-height"));
        assert_eq!(
            state["data"]["diagnosis_code"],
            json!("z_confirm_before_resume")
        );
        assert_eq!(state["data"]["confirm_timeout_s"], json!(90.0), "{state}");
        let left = state["data"]["confirm_expires_in_s"].as_f64().unwrap();
        assert!(left > 80.0 && left <= 90.0, "{state}");
        assert!(state["data"]["step"].as_u64().is_some(), "{state}");

        // And it agrees with the `busy` refusal, which was the only probe a
        // client used to have: both describe the same window.
        let busy = roundtrip(&path, EXECUTE_ASK).await;
        assert_eq!(busy["data"]["outcome"], json!("busy"), "{busy}");
        let state = roundtrip(&path, "{\"cmd\": \"recover_state\"}\n").await;
        assert_eq!(state["data"]["executing"], json!(true), "{state}");

        // The query consumed nothing: the token still answers.
        let done = roundtrip(&path, &confirm_request(&token, "continue")).await;
        assert_eq!(done["ok"], json!(true), "{done}");
        let state = roundtrip(&path, "{\"cmd\": \"recover_state\"}\n").await;
        assert_eq!(state["data"]["executing"], json!(false), "{state}");
        assert_eq!(state["data"]["awaiting_confirmation"], json!(false));
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn a_lapsed_pause_reads_as_not_awaiting_over_the_socket() {
        // This is the plugin defect being deleted: with nothing reported,
        // a client assumed the band ceiling (3600 s) and went on claiming a
        // confirmation was live for ~50 minutes after the daemon aborted.
        //
        // What this proves and what it does not. It proves the end-to-end
        // truth — the reported deadline is the enforced one, and once it has
        // passed the socket says "not awaiting" while the daemon really has
        // aborted. It does NOT exercise the read-time expiry derivation:
        // 900 ms after a 300 ms deadline the execution task has finished and
        // dropped its lease, so the answer would be the same with the
        // derivation removed (verified by mutation). The derivation is
        // pinned by `a_published_pause_expires_by_the_clock_not_by_being_told`,
        // which can place the clock where a live task with a lapsed pause
        // cannot be arranged deterministically here.
        let (path, wal_dir, _fake) =
            spawn_confirm_server("state-lapsed", &[("confirm_z_before_resume", json!(true))]).await;
        let paused = roundtrip(&path, EXECUTE_ASK).await;
        let token = paused["data"]["resume_token"].as_str().unwrap().to_owned();
        // The harness's confirm_timeout is 300 ms, and it is reported.
        assert_eq!(paused["data"]["confirm_timeout_s"], json!(0.3), "{paused}");
        tokio::time::sleep(Duration::from_millis(900)).await;
        let state = roundtrip(&path, "{\"cmd\": \"recover_state\"}\n").await;
        assert_eq!(
            state["data"]["awaiting_confirmation"],
            json!(false),
            "a lapsed confirmation must not read as live: {state}"
        );
        assert!(state["data"]["resume_token"].is_null(), "{state}");
        // The daemon really did abort, so "not awaiting" was the truth.
        let marker = crate::detect::read_frame_invalid(&wal_dir).expect("frame-invalid marker");
        assert_eq!(marker.reason, "confirmation-timeout");
        let expired = roundtrip(&path, &confirm_request(&token, "continue")).await;
        assert_eq!(expired["data"]["outcome"], json!("unknown-token"));
    }

    /// The expiry derivation itself, as a pure function of the clock.
    ///
    /// It has to be derived at read time and cannot be trusted as
    /// published: the executor's `tokio::time::timeout` *cancels*
    /// `SocketConfirmer::confirm` when the deadline fires, so the code that
    /// would have cleared the mirror is dropped rather than run. Both
    /// directions are checked against the same published pause.
    #[test]
    fn a_published_pause_expires_by_the_clock_not_by_being_told() {
        let observed = Observed::default();
        observed.begin();
        let raised_at = std::time::Instant::now();
        let (tx, _rx) = tokio::sync::oneshot::channel();
        observed.publish_pause(&PauseNotice {
            token: "plrc-test".to_owned(),
            kind: "z-height",
            step_id: 7,
            phase: "z-confirm-standoff".to_owned(),
            diagnosis: plr_recovery::Diagnosis::new(
                "z_confirm_before_resume",
                plr_recovery::Tier::Confirmable,
                "what".to_owned(),
                "why".to_owned(),
                "fix".to_owned(),
            ),
            detail: Value::Null,
            deadline: Duration::from_mins(10),
            raised_at,
            answer: tx,
        });

        // Halfway: live, with the remaining budget reported.
        let mid = observed.snapshot_at(raised_at + Duration::from_mins(5));
        assert!(mid.awaiting, "{mid:?}");
        assert!(!mid.confirm_expired);
        assert_eq!(mid.token.as_deref(), Some("plrc-test"));
        assert_eq!(mid.confirm_timeout_s, Some(600.0));
        assert_eq!(mid.expires_in_s, Some(300.0));

        // Exactly at the deadline, and past it: gone, and said to be gone.
        for after in [Duration::from_mins(10), Duration::from_mins(67)] {
            let late = observed.snapshot_at(raised_at + after);
            assert!(!late.awaiting, "{after:?}: {late:?}");
            assert!(late.confirm_expired, "{after:?}: {late:?}");
            assert!(late.token.is_none(), "{after:?}: {late:?}");
            // Still executing: the abort path is running.
            assert!(late.executing, "{after:?}: {late:?}");
        }

        // A never-stamped deadline is treated as already expired, not as a
        // deadline far in the future.
        let (tx, _rx) = tokio::sync::oneshot::channel();
        observed.publish_pause(&PauseNotice {
            token: "plrc-unstamped".to_owned(),
            kind: "diagnosis",
            step_id: 1,
            phase: "idle-timeout".to_owned(),
            diagnosis: plr_recovery::Diagnosis::new(
                "x",
                plr_recovery::Tier::Confirmable,
                "w".to_owned(),
                "y".to_owned(),
                "f".to_owned(),
            ),
            detail: Value::Null,
            deadline: Duration::ZERO,
            raised_at: std::time::Instant::now(),
            answer: tx,
        });
        let snapshot = observed.snapshot();
        assert!(!snapshot.awaiting, "{snapshot:?}");
        assert!(snapshot.confirm_expired, "{snapshot:?}");

        // And the lease's end is what stops the claim entirely.
        observed.end();
        assert_eq!(observed.snapshot(), StateSnapshot::default());
    }

    /// The mirror stops claiming an execution is live when the execution
    /// task is *dropped* rather than finishing — `systemctl restart plrd`.
    /// Nothing on a success path has to run for this to hold.
    #[tokio::test]
    async fn an_aborted_execution_task_releases_the_lease() {
        let observed = Arc::new(Observed::default());
        let lease = ExecutionLease::begin(Arc::clone(&observed));
        let join = tokio::spawn(async move {
            let _lease = lease;
            std::future::pending::<()>().await;
        });
        assert!(observed.snapshot().executing);
        join.abort();
        let _ = join.await;
        assert!(
            !observed.snapshot().executing,
            "a dropped execution task must not leave the daemon looking busy forever"
        );
    }

    /// The one transcript the WAL dir holds after an execution.
    fn read_transcript(wal_dir: &std::path::Path) -> String {
        let entry = std::fs::read_dir(wal_dir)
            .unwrap()
            .filter_map(Result::ok)
            .find(|e| {
                e.file_name()
                    .to_string_lossy()
                    .starts_with("recovery-transcript-")
            })
            .expect("transcript file");
        std::fs::read_to_string(entry.path()).unwrap()
    }

    /// An unwritable interlock refuses before the SHIFTED-frame declare —
    /// against the plan the pipeline really builds.
    ///
    /// The invariant is narrower than "no `SET_KINEMATIC_POSITION` was
    /// sent", and saying it that way would be false: a real plan declares
    /// the conservative believed-Z frame two phases earlier, and that
    /// declare has already run by the time this refusal fires. What the
    /// fail-closed guard buys is precisely the shifted probing frame and
    /// the probe — the point past which Z becomes a number nobody can
    /// re-derive — so that is what this asserts.
    #[cfg(unix)]
    #[tokio::test]
    async fn an_unwritable_interlock_refuses_before_the_shifted_frame_declare() {
        use crate::pipeline::PipelineOutcome;
        use plr_recovery::Phase;

        let (_dir, mut config) = crate::pipeline::e2e_tests::plr_fixture("interlock-pipeline", &[]);
        let sim = Arc::new(std::sync::Mutex::new(SimPrinter::new(
            config.wal_dir.to_str().unwrap(),
        )));
        let fake = FakeMoonraker::spawn(sim_handler(Arc::clone(&sim))).await;
        config.moonraker_url = fake.url();
        // Block ONLY the marker write: a directory on the staging path
        // makes `File::create` fail there and nowhere else, so the
        // transcript gate still passes and execution really reaches the
        // shifted-frame step.
        std::fs::create_dir(config.wal_dir.join(crate::detect::FRAME_INVALID_TEMP_NAME)).unwrap();

        // The real plan, from the real pipeline.
        let mut pipeline_out: Vec<u8> = Vec::new();
        let outcome = crate::pipeline::run_pipeline(&config, &mut pipeline_out).expect("pipeline");
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!(
                "expected a plan, got {outcome:?}\n{}",
                String::from_utf8_lossy(&pipeline_out)
            );
        };
        let command_of = |phase: Phase| -> String {
            let i = bundle
                .plan
                .first_index(phase)
                .unwrap_or_else(|| panic!("{phase:?} missing from the pipeline plan"));
            bundle.plan.steps[i]
                .commands
                .iter()
                .find(|c| c.starts_with("SET_KINEMATIC_POSITION"))
                .unwrap_or_else(|| panic!("{phase:?} has no kinematic declare"))
                .clone()
        };
        let believed = command_of(Phase::BelievedZDeclare);
        let shifted = command_of(Phase::ShiftedFrame);
        assert_ne!(believed, shifted, "the fixture must distinguish the two");

        let state = fast_state(config.clone());
        let mut out: Vec<u8> = Vec::new();
        let code = crate::recover::execute_with_gates(
            &bundle,
            &config,
            &state.exec_options,
            state.connect_timeout,
            &mut crate::recover::AutoGate,
            &mut crate::executor::AbortConfirmer,
            &mut out,
        )
        .await;
        let text = String::from_utf8(out).unwrap();
        assert_eq!(code, crate::EXIT_RUNTIME, "{text}");

        let sent = fake.gcode_sent();
        // The earlier believed-Z declare DID run — this refusal does not
        // and cannot undo it.
        assert!(
            sent.contains(&believed),
            "the believed-Z declare should have run: {sent:?}"
        );
        // The shifted-frame declare specifically never went out...
        assert!(
            !sent.contains(&shifted),
            "the shifted-frame declare must never be issued without the interlock: {sent:?}"
        );
        // ...and nothing downstream of it did either.
        assert!(
            !sent.iter().any(|c| {
                c.starts_with("PLR_TOUCH")
                    || c.starts_with("PROBE")
                    || c.starts_with("PLR_DRAG_PROBE")
            }),
            "no contact operation may run: {sent:?}"
        );
        assert!(!sent.iter().any(|c| c == "M24"), "{sent:?}");

        // No marker was written (that is the whole failure), and none is
        // needed: the frame was never fabricated, so a retry is safe.
        assert!(crate::detect::read_frame_invalid(&config.wal_dir).is_none());

        // The operator message names BOTH sides honestly.
        assert!(
            text.contains("REFUSED to declare the shifted Z frame"),
            "{text}"
        );
        assert!(text.contains("AVOIDED:"), "{text}");
        assert!(text.contains("ALREADY DONE:"), "{text}");
        assert!(text.contains("HEATERS MAY STILL BE HOT"), "{text}");
        assert!(text.contains("IDLE TIMEOUT IS EXTENDED"), "{text}");
        // It must NOT claim the printer is untouched.
        assert!(!text.contains("untouched"), "{text}");
        assert!(!text.contains("left as-is"), "{text}");
        // And the machine really is in the state the message describes.
        assert!(
            sent.iter().any(|c| c.starts_with("SET_IDLE_TIMEOUT")),
            "{sent:?}"
        );
        assert!(sent.iter().any(|c| c.starts_with("M104")), "{sent:?}");
    }
}
