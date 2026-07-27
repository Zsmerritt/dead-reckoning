//! Daemon composition (Linux): config → WAL thread + client task.
//!
//! # Layout
//!
//! ```text
//! main thread ── tokio current-thread runtime
//! │   └── client::run_client   (async socket I/O, conversion)
//! │         │  bounded mpsc (WalCmd)
//! └── "plrd-wal" thread        (sync file I/O, fdatasync — walsvc)
//! ```
//!
//! One async task and one sync thread: the socket must always be drained
//! (Klipper disconnects slow clients) while syncs may stall on the disk,
//! so the two never share a thread. Everything else (backpressure, drop
//! policy) lives in the channel between them (`sender`).
//!
//! Shutdown: SIGTERM/SIGINT cancels the client future, sends `Shutdown`
//! (final fdatasync) to the WAL thread, and joins it. Exit code 0 only
//! when the WAL thread exited cleanly.

use std::path::Path;

use crate::client::{run_client, ClientCfg};
use crate::config::Config;
use crate::convert::Recorder;
use crate::sender::{WalCmd, WalSender};
use crate::walsvc::{self, WalSvcCfg};
use crate::{EXIT_OK, EXIT_RUNTIME};

/// Maps the user config onto the WAL service's knobs.
fn wal_cfg(config: &Config) -> WalSvcCfg {
    WalSvcCfg {
        wal_dir: config.wal_dir.clone(),
        heartbeat_path: config.heartbeat_file(),
        receive_seq_path: config.receive_seq_file(),
        batch_interval: std::time::Duration::from_millis(config.batch_sync_ms),
        heartbeat_period: std::time::Duration::from_secs_f64(1.0 / config.heartbeat_hz),
        heartbeat_o_dsync: config.heartbeat_o_dsync,
        rotate_bytes: config.segment_rotate_bytes,
        wal_heartbeat_quiet_every: crate::convert::WAL_HEARTBEAT_QUIET_EVERY,
    }
}

/// Runs the daemon until a signal or a fatal error. Returns the process
/// exit code.
// Linear composition (bind control socket, boot detection, retention,
// spawn WAL thread + power-fail watcher, build the runtime, select on
// client/signal/power-fail, then the shutdown sequence). Splitting it would
// scatter the ownership of `sender`/`config` across helpers for no clarity
// gain — same call the project already made for `ctrlsock::cmd_recover_execute`.
#[allow(clippy::too_many_lines)]
pub fn run(config_path: &Path) -> u8 {
    let config = match Config::load(config_path) {
        Ok(config) => config,
        Err(e) => {
            eprintln!("plrd: {e}");
            return EXIT_RUNTIME;
        }
    };
    // The control socket binds BEFORE anything else spawns: a daemon
    // whose console-side contract (the Klipper plugin's PLR_STATUS /
    // PLR_RECOVER) cannot come up must fail loudly at startup, not run
    // half-featured. Stale-socket unlink + permission rationale:
    // `ctrlsock::bind` docs.
    let ctrl_listener = match crate::ctrlsock::bind(&config.control_socket) {
        Ok(listener) => listener,
        Err(e) => {
            eprintln!("plrd: FATAL: {e}");
            eprintln!(
                "plrd: the control socket path comes from `control_socket` in the config \
                 (and must match the [plr] section's `control_socket`)"
            );
            return EXIT_RUNTIME;
        }
    };

    // Boot-time pending-recovery detection runs BEFORE the WAL service
    // touches the directory, so it classifies exactly the previous
    // session's evidence. Bounded (newest segments only) and never
    // fatal: recording starts regardless of what it finds, and it never
    // executes anything — it only writes a state file and announces.
    let boot = boot_detection(&config);
    let announcement = boot.announcement;

    // Power-fail sidecar lifecycle: it is a WRITE-ONCE event file, so once
    // this boot's detection has CONSUMED it (the read happens inside
    // `boot_detection` above, via `detect` -> `scan::load_power_fail_edge`),
    // it must be deleted — a consumed genuine edge already lives on in
    // detection's output: the edge is PERSISTED into `pending_recovery.json`
    // (`PendingRecovery::power_fail_edge_mono_ns`), so a later `plrd recover`
    // still has the exact-T fact, and a stale/inert file left behind would
    // poison a LATER crash's early-uptime reconstruction (the admission
    // band's tail-anchor plus this deletion together close that hazard).
    //
    // But the delete is now GUARDED: when detection was inconclusive (an
    // unreadable/unanalyzable WAL, or a pending-write that failed) the edge
    // was NOT durably consumed, and deleting it would destroy an unconsumed
    // exact-T edge a later readable boot or `plrd recover` still needs. The
    // sidecar shares the pending file's fate — both are preserved together.
    // Ordering is load-bearing: this runs strictly AFTER the read above and
    // never before it, so a genuine edge is never dropped before it is used.
    // Failure to delete is logged, never fatal — the same posture as the
    // `pending_recovery.json` clear sites.
    dispose_power_fail_sidecar(
        &config.power_fail_sidecar_file(),
        boot.preserve_power_fail_sidecar,
    );

    // WAL retention: prune superseded old sessions down to the configured
    // cap. This MUST run here — after boot detection (which also reads the
    // previous session's tail) and BEFORE the WAL service spawns — so the
    // highest-numbered segment `walsvc` is about to create `max + 1` past
    // is still the current max, hence in the newest session, which pruning
    // never deletes. Read-only classification plus unlink + dir fsync;
    // never fatal. Returns console notices (corruption found, pin-driven
    // overage) for best-effort delivery to the operator.
    let retention_notices = crate::retention::run_pruning(&config);
    let retention_url = config.moonraker_url.clone();

    let (tx, rx) = std::sync::mpsc::sync_channel::<WalCmd>(config.channel_capacity);
    // A clone for the power-fail watcher (below) to journal its marker on
    // the same never-dropped path. Cloned before `WalSender` takes `tx`.
    let watcher_tx = tx.clone();
    let wal_thread = walsvc::spawn(wal_cfg(&config), rx);

    // The power-failing GPIO watcher (dormant unless `[power_fail_gpio]` is
    // configured — the operator's hold-up hardware does not exist yet). Its
    // best-effort clean-exit hook fires `powerfail_exit`, which the
    // runtime's `select!` below treats as a THIRD stop reason: distinct
    // from a SIGTERM/SIGINT signal because it must NOT journal a
    // `RecorderStopped` marker (that suppresses recovery — the exact wrong
    // outcome after a real power loss), yet must still take the final WAL
    // sync. `note_stop_reason` is the precedent for a side-channel WAL
    // writer; this is a second one, journaling directly to the WAL thread.
    let powerfail_exit = std::sync::Arc::new(tokio::sync::Notify::new());
    if let Some(pf) = config.power_fail_gpio.clone() {
        let exit = std::sync::Arc::clone(&powerfail_exit);
        let response = crate::powerfail::WalChannelResponse::new(
            config.power_fail_sidecar_file(),
            watcher_tx,
            // Best-effort tier: signal the clean exit. Runs only after the
            // watcher's mandatory tier has journaled the marker (see
            // `powerfail::run_watcher`), so it can never delay it.
            Box::new(move || exit.notify_one()),
        );
        // Detached: the thread blocks on the GPIO edge for the daemon's
        // life (or ends after firing once). Not joined — process exit at
        // shutdown reaps a thread blocked in a kernel edge read.
        drop(crate::powerfail::spawn(pf, response));
        eprintln!("plrd: power-fail GPIO watcher armed");
    } else {
        // No hardware configured: drop the spare sender so it does not keep
        // the WAL channel alive independently of `WalSender`.
        drop(watcher_tx);
    }

    // The daemon is operational once the WAL service owns its files —
    // Klipper being down is a normal state it records around.
    if let Err(e) = crate::sdnotify::notify_ready() {
        eprintln!("plrd: sd_notify failed (continuing): {e}");
    }

    let client_cfg = ClientCfg::new(
        config.klipper_socket.clone(),
        config.trapq_queues.clone(),
        config.z_steppers.clone(),
    );
    let runtime = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            eprintln!("plrd: cannot build async runtime: {e}");
            return EXIT_RUNTIME;
        }
    };

    // Register the already-bound control socket with the runtime's
    // reactor. Failure here is as fatal as the bind itself: the daemon
    // must not run with its console-side contract silently dead.
    let ctrl_listener = {
        let _guard = runtime.enter();
        match tokio::net::UnixListener::from_std(ctrl_listener) {
            Ok(listener) => listener,
            Err(e) => {
                eprintln!("plrd: FATAL: control socket cannot enter the runtime: {e}");
                return EXIT_RUNTIME;
            }
        }
    };
    let ctrl_state = std::sync::Arc::new(crate::ctrlsock::CtrlState::new(config.clone()));

    let mut sender = WalSender::new(tx);
    let mut recorder = Recorder::new();
    let moonraker_url = config.moonraker_url.clone();
    // Set only on the signal path — see the recorder-stopped marker below.
    let mut graceful = false;
    // Set only on the power-fail watcher's clean-exit path — see below.
    let mut power_fail = false;
    let client_result = runtime.block_on(async {
        // The control server runs beside the recorder. Its handlers do
        // heavy work on spawn_blocking and its executions await
        // Moonraker I/O, so it cannot starve the socket reader (see
        // ctrlsock's module docs).
        tokio::spawn(crate::ctrlsock::serve(ctrl_listener, ctrl_state));
        // Best-effort operator announcement, concurrent with recording;
        // it can never block, delay, or fail the recorder.
        if let Some(announcement) = announcement {
            tokio::spawn(announce_pending(moonraker_url, announcement));
        }
        // Retention notices (WAL corruption, or a pin holding usage over the
        // cap): tell the operator on the console, best-effort, concurrent
        // with recording.
        for commands in retention_notices {
            tokio::spawn(announce_overage(retention_url.clone(), commands));
        }
        tokio::select! {
            result = run_client(&client_cfg, &mut sender, &mut recorder) => result,
            () = shutdown_signal() => {
                graceful = true;
                Ok(())
            }
            () = powerfail_exit.notified() => {
                power_fail = true;
                Ok(())
            }
        }
    });

    if journals_recorder_stopped(graceful, power_fail) {
        note_stop_reason(&mut sender, graceful);
    } else if power_fail {
        // The watcher already journaled and fsync'd the PowerFailing
        // marker (its mandatory tier). This is a CLEAN exit — final WAL
        // sync below — but deliberately NOT a graceful *recorder* stop:
        // journaling `RecorderStopped` here would suppress the very
        // recovery announcement the power loss must produce.
        eprintln!(
            "plrd: power-fail clean exit — the PowerFailing marker is journaled; \
             not writing a recorder-stopped marker"
        );
    }

    // Final durability: ask the WAL thread to sync and exit, then judge
    // by its verdict.
    sender.shutdown();
    let wal_result = wal_thread.join();
    // Best-effort tidy-up; a leftover socket file is harmless (the next
    // start unlinks it) but confuses nobody if we remove it now.
    let _ = std::fs::remove_file(&config.control_socket);
    match (client_result, wal_result) {
        (Ok(()), Ok(Ok(()))) => EXIT_OK,
        (client, wal) => {
            if let Err(e) = client {
                eprintln!("plrd: client stopped: {e}");
            }
            match wal {
                Ok(Ok(())) => {}
                Ok(Err(e)) => eprintln!("plrd: WAL service failed: {e}"),
                Err(_) => eprintln!("plrd: WAL thread panicked"),
            }
            EXIT_RUNTIME
        }
    }
}

/// Records **why** this session ended, for the next start's benefit.
///
/// A graceful stop (SIGTERM/SIGINT — `systemctl restart plrd`, an upgrade,
/// a reboot) means the *recorder* stopped, not the print. The WAL's tail
/// then carries no `CleanShutdown` marker even though nothing died, and
/// the next boot's detection runs before the Klipper client can ask, so
/// without this the restart announces a recovery for a print that is still
/// running. `plr_wal::MarkerKind::RecorderStopped` says so in the log
/// itself; reconstruction surfaces it as
/// `plr_reconstruct::WalTimeline::recorder_stopped_tail`, and `detect`
/// turns that into "the print's fate is unknown" — which suppresses the
/// announcement and nothing else.
///
/// A **fatal** stop (the client or WAL thread failed) is not graceful and
/// deliberately journals nothing: that session really did lose its recorder
/// mid-print, and the operator must be told.
///
/// The marker goes through the ordinary WAL channel immediately before
/// `WalSender::shutdown`, so the shutdown's final `fdatasync` makes it
/// durable. A dead WAL thread (`WalGone`) means nothing can be journaled
/// at all, which costs one spurious announcement — logged, never fatal.
/// Whether a stop should journal the graceful recorder-stopped marker.
///
/// Only a graceful signal (SIGTERM/SIGINT) does. A **power-fail** clean
/// exit must not — a `RecorderStopped` marker suppresses the recovery
/// announcement, which is the exact wrong outcome after a real power loss
/// (the `PowerFailing` marker + sidecar already record the truth). A
/// client-ended or fatal stop journals nothing either. `power_fail` wins
/// over `graceful` defensively, though the `select!` sets at most one.
///
/// Extracted so the third `select!` arm's decision is unit-testable
/// without driving the whole async runtime (see the test below).
const fn journals_recorder_stopped(graceful: bool, power_fail: bool) -> bool {
    graceful && !power_fail
}

fn note_stop_reason(sender: &mut WalSender, graceful: bool) {
    if !graceful {
        return;
    }
    let marker = plr_wal::Marker {
        mono_ns: crate::hostclock::now_mono_ns(),
        kind: plr_wal::MarkerKind::RecorderStopped,
    };
    if sender.marker(marker).is_err() {
        eprintln!(
            "plrd: the WAL thread is gone; cannot journal the recorder-stopped marker \
             (the next start may announce a recovery for a print that is still running)"
        );
    }
}

/// What boot detection wants said, and about which print.
struct BootAnnouncement {
    /// `(primary, fallback)` G-Code commands.
    commands: (String, String),
    /// The print file the announcement is about, when the announcement is
    /// a *recovery offer*. `None` for informational messages (a completed
    /// print's un-run end sequence), which are never retracted.
    ///
    /// `announce_pending` re-checks this file against Klipper's live
    /// status before speaking: if the same file is still printing, the
    /// recorder restarted, the print did not die, and the offer is
    /// withdrawn instead of announced.
    offer_for: Option<String>,
}

/// The result of boot-time classification: what to announce, and whether
/// the power-fail sidecar's edge is now safe to delete.
struct BootOutcome {
    /// What to announce, if anything.
    announcement: Option<BootAnnouncement>,
    /// `true` when the write-once power-fail sidecar must be KEPT rather
    /// than deleted this boot, because its edge was NOT durably consumed —
    /// detection was inconclusive (an unreadable/unanalyzable WAL, or a
    /// pending-file write that failed), so deleting it would destroy an
    /// unconsumed exact-T edge a later `plrd recover` (or a later, readable
    /// boot) still needs. Mirrors the same fate as the pending file: an edge
    /// and the offer it belongs to are preserved together.
    preserve_power_fail_sidecar: bool,
}

/// The power-fail sidecar lifecycle decision, extracted so the keep/delete
/// site itself is unit-testable (not only `boot_detection`'s verdict).
///
/// The sidecar is a WRITE-ONCE event file. When `preserve` is `false` its
/// edge has been durably consumed — PERSISTED into `pending_recovery.json`
/// by boot detection — so the file is deleted; a stale copy left behind
/// would poison a LATER crash's early-uptime reconstruction. When `preserve`
/// is `true` detection was inconclusive (unreadable/unanalyzable WAL, or a
/// failed pending-write), the edge was NOT durably consumed, and deleting it
/// would destroy an unconsumed exact-T edge a later readable boot or `plrd
/// recover` still needs — so it is kept. Failure to delete is logged, never
/// fatal (the same posture as the `pending_recovery.json` clear sites); a
/// genuinely absent file (`NotFound`) is silent.
fn dispose_power_fail_sidecar(path: &std::path::Path, preserve: bool) {
    if preserve {
        eprintln!(
            "plrd: keeping the power-fail sidecar {} (its edge was not durably consumed)",
            path.display()
        );
    } else if let Err(e) = std::fs::remove_file(path) {
        if e.kind() != std::io::ErrorKind::NotFound {
            eprintln!(
                "plrd: could not delete consumed power-fail sidecar {} (continuing): {e}",
                path.display()
            );
        }
    }
}

/// Classifies the previous session and prepares the announcement
/// commands, if any (see `detect` for the semantics).
// One match over the four `Detection` outcomes, each arm resolving both the
// announcement and the power-fail-sidecar disposition; splitting it would
// scatter that paired decision for no clarity gain.
#[allow(clippy::too_many_lines)]
fn boot_detection(config: &Config) -> BootOutcome {
    use crate::detect::{self, Detection};
    let detection = detect::detect(
        &config.wal_dir,
        &config.heartbeat_file(),
        crate::hostclock::now_wall_ns(),
    );
    match detection {
        Detection::Pending(pending) => {
            eprintln!(
                "plrd: unfinished print detected: {} at byte {}{} ({}); run `plrd recover`",
                pending.file,
                pending.file_position,
                pending
                    .percent
                    .map_or(String::new(), |p| format!(" (~{p:.0}%)")),
                pending.crash_class,
            );
            // The pending file now carries the persisted power-fail edge
            // (`PendingRecovery::power_fail_edge_mono_ns`). Only once that
            // write SUCCEEDS is the sidecar's edge durably consumed and safe
            // to delete; a failed write means the sidecar is the sole
            // surviving copy, so keep it.
            let write_failed = if let Err(e) = detect::write_pending(&config.wal_dir, &pending) {
                eprintln!("plrd: cannot write pending-recovery state: {e}");
                true
            } else {
                false
            };
            BootOutcome {
                announcement: Some(BootAnnouncement {
                    commands: detect::announcement_commands(&pending),
                    offer_for: Some(pending.file.clone()),
                }),
                preserve_power_fail_sidecar: write_failed,
            }
        }
        Detection::Complete(completion) => {
            eprintln!(
                "plrd: {} is COMPLETE: no extrusion remains after byte {} of {} \
                 ({} trailing bytes are the slicer footer); nothing to recover",
                completion.file,
                completion.tested_offset,
                completion.file_size,
                completion.trailing_bytes,
            );
            // A finished print retracts any stale offer, exactly as a
            // clean shutdown does; a stale sidecar edge is likewise inert.
            detect::clear_pending(&config.wal_dir);
            BootOutcome {
                announcement: detect::completion_commands(&completion).map(|commands| {
                    BootAnnouncement {
                        commands,
                        offer_for: None,
                    }
                }),
                preserve_power_fail_sidecar: false,
            }
        }
        Detection::Clean => {
            detect::clear_pending(&config.wal_dir);
            // `frame_invalid.json` is deliberately left alone: see
            // `Detection::Clean`. Only a fresh dry run clears it. A clean end
            // makes any leftover sidecar edge stale — delete it.
            BootOutcome {
                announcement: None,
                preserve_power_fail_sidecar: false,
            }
        }
        Detection::Nothing(no_offer) => {
            eprintln!("plrd: no pending recovery: {}", no_offer.reason);
            if no_offer.preserve_pending {
                // We failed to derive an answer rather than establishing
                // there is none: a stale-looking pending file may be the
                // only surviving record of a genuine offer, because the
                // evidence behind it scrolls out of the segments detection
                // reads. Leave it alone — and, for the identical reason,
                // leave the power-fail sidecar's edge (it was NOT consumed
                // into any durable pending; an unreadable-WAL boot must not
                // destroy an unconsumed edge).
                eprintln!(
                    "plrd: leaving any existing pending-recovery state and power-fail edge in \
                     place (the verdict above is inconclusive)"
                );
            } else {
                detect::clear_pending(&config.wal_dir);
            }
            BootOutcome {
                announcement: None,
                preserve_power_fail_sidecar: no_offer.preserve_pending,
            }
        }
    }
}

/// Delivers the operator announcement via Moonraker
/// (`printer.gcode.script`; see `detect` module docs for the channel
/// choice). Klippy may be down at boot: retried on a slow cadence,
/// bounded, and abandoned silently — never affecting recording.
///
/// # The live-status pre-check
///
/// Before announcing a *recovery offer*, the printer's live state is
/// queried. `plrd.service` is `Restart=always`, so the daemon restarting
/// mid-print is routine — and boot detection runs before the Klipper
/// client connects, so the WAL of a still-running print looks exactly
/// like the WAL of a print that died. If Klipper reports the very same
/// file still active (or `print_stats` still `printing`/`paused`), the
/// print is alive and nothing is said.
///
/// It stays **silent** rather than deleting `pending_recovery.json`.
/// Suppressing the announcement is the whole requirement; deleting the file
/// would be an optimisation that turns any file-identity mistake — see
/// [`same_print_file`], which cannot fully rule out two same-named files in
/// different sdcard subdirectories — into the destruction of a genuine
/// offer. The next start re-derives the file from the WAL anyway, and that
/// derivation is authoritative.
///
/// This is the second of two defences; the first is the
/// `plr_wal::MarkerKind::RecorderStopped` marker journaled by
/// [`note_stop_reason`]. They are independent on purpose: the marker cannot
/// be written if the daemon is `SIGKILL`ed, and this pre-check cannot run
/// while Moonraker is down.
async fn announce_pending(url: String, announcement: BootAnnouncement) {
    use crate::moonraker::MoonrakerClient;
    const ATTEMPTS: u32 = 30;
    const RETRY: std::time::Duration = std::time::Duration::from_secs(10);
    let commands = announcement.commands;
    for _ in 0..ATTEMPTS {
        if let Ok(mut client) =
            MoonrakerClient::connect(&url, std::time::Duration::from_secs(5)).await
        {
            if let Some(file) = announcement.offer_for.as_deref() {
                if print_is_still_running(&mut client, file).await {
                    eprintln!(
                        "plrd: '{file}' is still printing — the recorder restarted, the print \
                         did not die; staying silent"
                    );
                    return;
                }
            }
            // Primary (RESPOND), then fallback (M117): either landing
            // in the console is success.
            if client.gcode_script(&commands.0).await.is_ok()
                || client.gcode_script(&commands.1).await.is_ok()
            {
                eprintln!("plrd: pending-recovery announcement delivered");
                return;
            }
        }
        tokio::time::sleep(RETRY).await;
    }
    eprintln!("plrd: could not deliver the pending-recovery announcement (gave up)");
}

/// Delivers a WAL-retention console notice (corruption found, or a pin
/// holding usage over the cap) via Moonraker (`printer.gcode.script`),
/// best-effort with retries because klippy may be down at boot. No
/// live-status pre-check (unlike [`announce_pending`]): these are standing
/// storage conditions, not offers that could be stale. The
/// `(primary, fallback)` commands come from `retention::run_pruning`
/// (RESPOND then M117; either landing is success).
async fn announce_overage(url: String, commands: (String, String)) {
    use crate::moonraker::MoonrakerClient;
    const ATTEMPTS: u32 = 30;
    const RETRY: std::time::Duration = std::time::Duration::from_secs(10);
    for _ in 0..ATTEMPTS {
        if let Ok(mut client) =
            MoonrakerClient::connect(&url, std::time::Duration::from_secs(5)).await
        {
            if client.gcode_script(&commands.0).await.is_ok()
                || client.gcode_script(&commands.1).await.is_ok()
            {
                eprintln!("plrd: WAL-retention notice delivered");
                return;
            }
        }
        tokio::time::sleep(RETRY).await;
    }
    eprintln!("plrd: could not deliver the WAL-retention notice (gave up)");
}

/// Is Klipper right now printing the file the offer is about?
///
/// Conservative in the direction that preserves the offer: any query
/// failure, any missing field, and any file-name mismatch answer `false`,
/// so a broken query can never silence a genuine recovery. The comparison
/// is on the base name because `virtual_sdcard.file_path` is relative to
/// the sdcard root while the WAL journals whatever Klipper reported at the
/// time.
async fn print_is_still_running(
    client: &mut crate::moonraker::MoonrakerClient,
    file: &str,
) -> bool {
    let Ok(status) = client
        .query_objects(&["virtual_sdcard", "print_stats"])
        .await
    else {
        return false;
    };
    let printing = status["virtual_sdcard"]["is_active"].as_bool() == Some(true)
        || matches!(
            status["print_stats"]["state"].as_str(),
            Some("printing" | "paused")
        );
    if !printing {
        return false;
    }
    let live = status["virtual_sdcard"]["file_path"]
        .as_str()
        .or_else(|| status["print_stats"]["filename"].as_str());
    let Some(live) = live else { return false };
    same_print_file(live, file)
}

/// Do these two print-file paths name the same file?
///
/// `virtual_sdcard.file_path` is relative to the sdcard root while the WAL
/// journals whatever Klipper reported at the time, so the strings routinely
/// differ for one file: `bench.gcode` against
/// `/home/pi/printer_data/gcodes/bench.gcode`. Equality, or one being a
/// path-boundary **suffix** of the other, covers that without the much
/// looser base-name match this used to do — under which
/// `sub_a/bench.gcode` and `sub_b/bench.gcode` were "the same file".
///
/// Two same-named files in different sdcard subdirectories can still tie
/// (`a/x.gcode` vs `b/a/x.gcode` cannot, but `x.gcode` reported bare vs
/// `b/x.gcode` can). That residual is why the caller only ever *stays
/// silent* on a match and never deletes anything.
fn same_print_file(a: &str, b: &str) -> bool {
    fn norm(path: &str) -> String {
        path.replace('\\', "/")
    }
    let (a, b) = (norm(a), norm(b));
    if a.is_empty() || b.is_empty() {
        return false;
    }
    if a == b {
        return true;
    }
    // Suffix, but only at a path boundary: `bench.gcode` matches
    // `/g/bench.gcode`, not `/g/notbench.gcode`.
    let (long, short) = if a.len() >= b.len() {
        (&a, &b)
    } else {
        (&b, &a)
    };
    long.strip_suffix(short.as_str())
        .is_some_and(|prefix| prefix.ends_with('/'))
}

/// Resolves when SIGTERM or SIGINT arrives.
async fn shutdown_signal() {
    use tokio::signal::unix::{signal, SignalKind};
    let mut term = signal(SignalKind::terminate()).expect("SIGTERM handler");
    let mut int = signal(SignalKind::interrupt()).expect("SIGINT handler");
    tokio::select! {
        _ = term.recv() => {}
        _ = int.recv() => {}
    }
}

#[cfg(test)]
mod tests {
    use super::wal_cfg;
    use crate::config::Config;

    #[test]
    fn wal_cfg_maps_every_knob() {
        let config = Config::parse(
            "wal_dir = /w\nheartbeat_path = /h/hb.bin\nheartbeat_hz = 20\n\
             batch_sync_ms = 250\nheartbeat_o_dsync = true\nsegment_rotate_bytes = 4096",
        )
        .unwrap();
        let cfg = wal_cfg(&config);
        assert_eq!(cfg.wal_dir, std::path::PathBuf::from("/w"));
        assert_eq!(cfg.heartbeat_path, std::path::PathBuf::from("/h/hb.bin"));
        assert_eq!(
            cfg.receive_seq_path,
            std::path::PathBuf::from("/w/receive_seq.bin")
        );
        assert_eq!(cfg.batch_interval, std::time::Duration::from_millis(250));
        assert_eq!(cfg.heartbeat_period, std::time::Duration::from_millis(50));
        assert!(cfg.heartbeat_o_dsync);
        assert_eq!(cfg.rotate_bytes, 4096);
        assert_eq!(
            cfg.wal_heartbeat_quiet_every,
            crate::convert::WAL_HEARTBEAT_QUIET_EVERY
        );
    }

    #[test]
    fn unreadable_config_exits_runtime_error() {
        assert_eq!(
            super::run(std::path::Path::new("/nonexistent/plrd.conf")),
            crate::EXIT_RUNTIME
        );
    }

    fn temp_config(tag: &str) -> Config {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-daemon-test-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        Config {
            wal_dir: dir,
            ..Config::default()
        }
    }

    /// Writes an unclean WAL whose print file has genuine unfinished work
    /// after byte 250 — the shape boot detection classifies as a recoverable
    /// Pending (same shape as the inline fixture in
    /// `boot_detection_writes_pending_and_prepares_announcement`).
    fn write_recoverable_wal(config: &Config) {
        use plr_wal::{SegmentHeader, WalRecord, WalWriter};
        let gcode = config.wal_dir.join("part.gcode");
        let mut text = String::from(";");
        while text.len() < 249 {
            text.push('p');
        }
        text.push_str("\nG1 X60 Y60 E900 F1800\n");
        std::fs::write(&gcode, text.as_bytes()).unwrap();
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer
            .append(&WalRecord::Heartbeat(plr_wal::Heartbeat {
                sequence: 1,
                mono_ns: 1_000_000_000,
                wall_ns: 1,
                print_time: 5.0,
                est_sample_mono_ns: 1_000_000_000,
                est_sample_print_time: 5.0,
                wal_offset: 32,
            }))
            .unwrap();
        writer
            .append(&WalRecord::Context(plr_wal::Context {
                mono_ns: 1_000_000_000,
                print_time: Some(5.0),
                virtual_sdcard: Some(plr_wal::VirtualSdState {
                    file_path: gcode.to_string_lossy().into_owned(),
                    file_position: 250,
                    file_size: None,
                }),
                gcode: plr_wal::GcodeState {
                    speed_factor: 1.0,
                    speed: 1500.0,
                    extrude_factor: 1.0,
                    absolute_coordinates: true,
                    absolute_extrude: true,
                    homing_origin: vec![0.0; 4],
                    position: vec![0.0; 4],
                    gcode_position: vec![0.0; 4],
                },
                transforms: plr_wal::TransformObservations {
                    bed_mesh_active: false,
                    bed_mesh_profile: None,
                    z_thermal_adjust_enabled: None,
                    z_thermal_adjust_offset: None,
                    skew_active: false,
                    skew_profile: None,
                },
                heaters: Vec::new(),
                fans: Vec::new(),
                exclude: None,
                print_state: None,
                current_layer: None,
                total_layer: None,
            }))
            .unwrap();
        std::fs::write(config.wal_dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
    }

    #[test]
    fn boot_detection_preserves_the_sidecar_only_when_the_edge_is_not_durably_consumed() {
        // MAJOR-2: the sidecar-preservation guard is NON-vacuous. Each arm
        // sets `preserve_power_fail_sidecar` deliberately, and the reviewer's
        // always-delete mutations at the Pending arm and the Nothing arm are
        // made to bite here.

        // Successful Pending consume: the edge is persisted into the pending
        // file, so the sidecar is safe to delete.
        let ok = temp_config("preserve-consume");
        write_recoverable_wal(&ok);
        let boot = super::boot_detection(&ok);
        assert!(
            boot.announcement.is_some(),
            "the fixture must be a recovery offer"
        );
        assert!(
            !boot.preserve_power_fail_sidecar,
            "a successful pending-consume persists the edge -> delete the sidecar"
        );

        // Pending whose PERSIST FAILS (a directory sits at the pending path
        // so `std::fs::write` cannot create the file): the edge was NOT
        // durably consumed, so KEEP the sidecar. Mutating the Pending arm to
        // always-delete makes this assertion fail.
        let failw = temp_config("preserve-write-fail");
        write_recoverable_wal(&failw);
        std::fs::create_dir(failw.wal_dir.join(crate::detect::PENDING_FILE_NAME)).unwrap();
        assert!(
            super::boot_detection(&failw).preserve_power_fail_sidecar,
            "a failed pending-write leaves the sidecar the sole copy -> keep it"
        );

        // Inconclusive (an empty WAL dir cannot be analysed -> a Nothing
        // whose verdict is 'I could not tell'): the edge was not consumed, so
        // KEEP it. Mutating the Nothing arm to always-delete makes this fail.
        let inconclusive = temp_config("preserve-inconclusive");
        assert!(
            super::boot_detection(&inconclusive).preserve_power_fail_sidecar,
            "an inconclusive verdict must not destroy an unconsumed edge"
        );
    }

    #[test]
    fn dispose_power_fail_sidecar_keeps_or_deletes_per_the_flag() {
        // MAJOR-2: the keep/delete site itself, exercised directly (a
        // mutation dropping the `preserve` guard fails this test).
        let config = temp_config("dispose-sidecar");
        let path = config.power_fail_sidecar_file();
        std::fs::write(&path, b"edge").unwrap();
        super::dispose_power_fail_sidecar(&path, true);
        assert!(path.exists(), "preserve=true must keep the sidecar");
        super::dispose_power_fail_sidecar(&path, false);
        assert!(!path.exists(), "preserve=false must delete the sidecar");
        // A genuinely absent file is silent, never fatal.
        super::dispose_power_fail_sidecar(&path, false);
    }

    #[test]
    fn boot_detection_writes_pending_and_prepares_announcement() {
        use plr_wal::{SegmentHeader, WalRecord, WalWriter};
        let config = temp_config("boot-pending");
        // Empty WAL dir: nothing pending, no announcement.
        assert!(super::boot_detection(&config).announcement.is_none());
        // Unclean WAL with a print in progress: pending + announcement,
        // and the state file exists.
        let gcode = config.wal_dir.join("part.gcode");
        // Real g-code with UNFINISHED work after byte 250: the completion
        // gate must let this through as a genuine recovery offer.
        let mut text = String::from(";");
        while text.len() < 249 {
            text.push('p');
        }
        text.push_str("\nG1 X60 Y60 E900 F1800\n");
        std::fs::write(&gcode, text.as_bytes()).unwrap();
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer
            .append(&WalRecord::Heartbeat(plr_wal::Heartbeat {
                sequence: 1,
                mono_ns: 1_000_000_000,
                wall_ns: 1,
                print_time: 5.0,
                est_sample_mono_ns: 1_000_000_000,
                est_sample_print_time: 5.0,
                wal_offset: 32,
            }))
            .unwrap();
        writer
            .append(&WalRecord::Context(plr_wal::Context {
                mono_ns: 1_000_000_000,
                // Matches this fixture's heartbeat print time: Klipper
                // reports the trapq append frontier in the same status pass
                // as the file position.
                print_time: Some(5.0),
                virtual_sdcard: Some(plr_wal::VirtualSdState {
                    file_path: gcode.to_string_lossy().into_owned(),
                    file_position: 250,
                    file_size: None,
                }),
                gcode: plr_wal::GcodeState {
                    speed_factor: 1.0,
                    speed: 1500.0,
                    extrude_factor: 1.0,
                    absolute_coordinates: true,
                    absolute_extrude: true,
                    homing_origin: vec![0.0; 4],
                    position: vec![0.0; 4],
                    gcode_position: vec![0.0; 4],
                },
                transforms: plr_wal::TransformObservations {
                    bed_mesh_active: false,
                    bed_mesh_profile: None,
                    z_thermal_adjust_enabled: None,
                    z_thermal_adjust_offset: None,
                    skew_active: false,
                    skew_profile: None,
                },
                heaters: Vec::new(),
                fans: Vec::new(),
                exclude: None,
                print_state: None,
                current_layer: None,
                total_layer: None,
            }))
            .unwrap();
        std::fs::write(config.wal_dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        let announcement = super::boot_detection(&config)
            .announcement
            .expect("announcement expected");
        let (primary, fallback) = &announcement.commands;
        assert!(primary.starts_with("RESPOND"), "{primary}");
        assert!(fallback.starts_with("M117"), "{fallback}");
        // It is an OFFER, so the live-status pre-check applies to it.
        assert_eq!(
            announcement.offer_for.as_deref(),
            Some(gcode.to_string_lossy().as_ref())
        );
        let pending_path = config.wal_dir.join(crate::detect::PENDING_FILE_NAME);
        assert!(pending_path.exists());
        // A clean tail clears the state file and announces nothing.
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer
            .append(&WalRecord::Marker(plr_wal::Marker {
                mono_ns: 2_000_000_000,
                kind: plr_wal::MarkerKind::CleanShutdown,
            }))
            .unwrap();
        std::fs::write(config.wal_dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        assert!(super::boot_detection(&config).announcement.is_none());
        assert!(!pending_path.exists());
    }

    /// An informational announcement (no offer attached) needs no live
    /// status check and is never withdrawn.
    fn info_announcement() -> super::BootAnnouncement {
        super::BootAnnouncement {
            commands: ("RESPOND MSG=\"x\"".to_owned(), "M117 x".to_owned()),
            offer_for: None,
        }
    }

    #[tokio::test]
    async fn announce_pending_delivers_primary_and_falls_back() {
        use crate::testmoon::FakeMoonraker;
        // Primary accepted.
        let fake = FakeMoonraker::spawn(|_, _| Ok(serde_json::json!("ok"))).await;
        super::announce_pending(fake.url(), info_announcement()).await;
        assert_eq!(fake.gcode_sent(), vec!["RESPOND MSG=\"x\""]);
        // Primary rejected ([respond] missing): fallback delivered.
        let fake = FakeMoonraker::spawn(|_, params| {
            let script = params["script"].as_str().unwrap_or("");
            if script.starts_with("RESPOND") {
                Err((400, "Unknown command: RESPOND".to_owned()))
            } else {
                Ok(serde_json::json!("ok"))
            }
        })
        .await;
        super::announce_pending(fake.url(), info_announcement()).await;
        assert_eq!(
            fake.gcode_sent(),
            vec!["RESPOND MSG=\"x\"", "M117 x"],
            "fallback must follow a rejected primary"
        );
    }

    /// **D3, second defence.** A plrd restart mid-print: boot detection
    /// ran before the Klipper client connected and produced an offer, but
    /// the printer says the very same file is still active. The offer is
    /// withdrawn and nothing is announced.
    #[tokio::test]
    async fn a_still_running_print_withdraws_the_offer_instead_of_announcing() {
        use crate::testmoon::FakeMoonraker;
        let config = temp_config("still-printing");
        let pending = crate::detect::PendingRecovery {
            detected_wall_ns: 1,
            file: "/home/pi/gcode_files/bench.gcode".to_owned(),
            file_position: 500,
            file_size: Some(2_000),
            percent: Some(25.0),
            crash_class: "HostDeathOrPowerLoss".to_owned(),
            frame_invalid: false,
            power_fail_edge_mono_ns: None,
            interrupted_by: None,
        };
        crate::detect::write_pending(&config.wal_dir, &pending).unwrap();
        let fake = FakeMoonraker::spawn(|method, _| {
            if method == "printer.objects.query" {
                Ok(serde_json::json!({"status": {
                    "virtual_sdcard": {"is_active": true, "file_path": "bench.gcode"},
                    "print_stats": {"state": "printing", "filename": "bench.gcode"},
                }}))
            } else {
                Ok(serde_json::json!("ok"))
            }
        })
        .await;
        super::announce_pending(
            fake.url(),
            super::BootAnnouncement {
                commands: ("RESPOND MSG=\"x\"".to_owned(), "M117 x".to_owned()),
                offer_for: Some(pending.file.clone()),
            },
        )
        .await;
        assert!(
            fake.gcode_sent().is_empty(),
            "a still-running print must not be announced as dead: {:?}",
            fake.gcode_sent()
        );
        assert!(
            config
                .wal_dir
                .join(crate::detect::PENDING_FILE_NAME)
                .exists(),
            "staying silent must not destroy the offer — see same_print_file"
        );
    }

    /// A genuine mid-print death: the printer is idle, so the offer is
    /// announced and the pending file survives.
    #[tokio::test]
    async fn a_genuinely_dead_print_is_still_announced() {
        use crate::testmoon::FakeMoonraker;
        let config = temp_config("dead-print");
        let pending = crate::detect::PendingRecovery {
            detected_wall_ns: 1,
            file: "/home/pi/gcode_files/bench.gcode".to_owned(),
            file_position: 500,
            file_size: Some(2_000),
            percent: Some(25.0),
            crash_class: "HostDeathOrPowerLoss".to_owned(),
            frame_invalid: false,
            power_fail_edge_mono_ns: None,
            interrupted_by: None,
        };
        crate::detect::write_pending(&config.wal_dir, &pending).unwrap();
        for status in [
            // Idle after a power loss.
            serde_json::json!({"status": {
                "virtual_sdcard": {"is_active": false, "file_path": null},
                "print_stats": {"state": "standby", "filename": ""},
            }}),
            // Printing, but something else entirely.
            serde_json::json!({"status": {
                "virtual_sdcard": {"is_active": true, "file_path": "other.gcode"},
                "print_stats": {"state": "printing", "filename": "other.gcode"},
            }}),
            // The query itself is unusable: never silence an offer on
            // missing information.
            serde_json::json!({"status": {}}),
            // Active, but nothing names a file.
            serde_json::json!({"status": {
                "virtual_sdcard": {"is_active": true, "file_path": null},
                "print_stats": {"state": "printing"},
            }}),
            // The query is REFUSED (sentinel: an empty object). A broken
            // status query must never silence a genuine offer either.
            serde_json::Value::Null,
        ] {
            let fake = FakeMoonraker::spawn(move |method, _| {
                if method != "printer.objects.query" {
                    Ok(serde_json::json!("ok"))
                } else if status.is_null() {
                    Err((400, "objects/query unavailable".to_owned()))
                } else {
                    Ok(status.clone())
                }
            })
            .await;
            super::announce_pending(
                fake.url(),
                super::BootAnnouncement {
                    commands: ("RESPOND MSG=\"x\"".to_owned(), "M117 x".to_owned()),
                    offer_for: Some(pending.file.clone()),
                },
            )
            .await;
            assert_eq!(fake.gcode_sent(), vec!["RESPOND MSG=\"x\""]);
            assert!(config
                .wal_dir
                .join(crate::detect::PENDING_FILE_NAME)
                .exists());
        }
    }

    /// Only a graceful stop journals the recorder-stopped marker: a fatal
    /// one really did lose the recorder mid-print and must stay
    /// announceable.
    #[test]
    fn only_a_graceful_stop_notes_the_recorder_stopped() {
        use crate::sender::WalCmd;
        fn journaled(graceful: bool) -> Vec<WalCmd> {
            let (tx, rx) = std::sync::mpsc::sync_channel::<WalCmd>(8);
            let mut sender = crate::sender::WalSender::new(tx);
            super::note_stop_reason(&mut sender, graceful);
            drop(sender);
            rx.try_iter().collect()
        }
        assert!(journaled(false).is_empty(), "a fatal stop journals nothing");
        let cmds = journaled(true);
        assert_eq!(cmds.len(), 1, "{cmds:?}");
        let WalCmd::Append { record, .. } = &cmds[0] else {
            panic!("expected a record, got {:?}", cmds[0]);
        };
        assert!(
            matches!(
                record,
                plr_wal::WalRecord::Marker(plr_wal::Marker {
                    kind: plr_wal::MarkerKind::RecorderStopped,
                    ..
                })
            ),
            "{record:?}"
        );
        // A dead WAL thread is logged, never fatal.
        let (tx, rx) = std::sync::mpsc::sync_channel::<WalCmd>(1);
        drop(rx);
        let mut sender = crate::sender::WalSender::new(tx);
        super::note_stop_reason(&mut sender, true);
    }

    /// The third `select!` arm's decision, unit-tested directly (the arm
    /// itself sets `power_fail` and `run` gates `note_stop_reason` behind
    /// this): only a graceful, non-power-fail stop journals
    /// `RecorderStopped`. A power-fail clean exit never does — that would
    /// suppress the recovery a power loss must announce.
    #[test]
    fn only_a_graceful_non_power_fail_stop_journals_recorder_stopped() {
        assert!(
            super::journals_recorder_stopped(true, false),
            "a SIGTERM/SIGINT stop journals RecorderStopped"
        );
        assert!(
            !super::journals_recorder_stopped(false, true),
            "a power-fail clean exit must not"
        );
        assert!(
            !super::journals_recorder_stopped(false, false),
            "a client-ended / fatal stop journals nothing"
        );
        // Defensive: power-fail wins over a stray graceful flag (the
        // select! sets at most one, but the marker must never suppress a
        // power-loss recovery).
        assert!(!super::journals_recorder_stopped(true, true));
    }

    #[test]
    fn print_files_are_compared_on_a_path_boundary() {
        // The normal shape: Klipper reports a root-relative name, the WAL
        // journaled the absolute path.
        assert!(super::same_print_file(
            "bench.gcode",
            "/home/pi/gcode_files/bench.gcode"
        ));
        assert!(super::same_print_file(
            "sub/bench.gcode",
            "/g/sub/bench.gcode"
        ));
        assert!(super::same_print_file("/g/bench.gcode", "/g/bench.gcode"));
        assert!(super::same_print_file(
            "C:\\g\\bench.gcode",
            "g/bench.gcode"
        ));
        // Not a base-name match any more: different directories are
        // different files.
        assert!(!super::same_print_file("a/bench.gcode", "/g/b/bench.gcode"));
        // Not a mid-name suffix either.
        assert!(!super::same_print_file("bench.gcode", "/g/notbench.gcode"));
        assert!(!super::same_print_file("bench.gcode", "other.gcode"));
        assert!(!super::same_print_file("", ""));
        assert!(!super::same_print_file("", "/g/x.gcode"));
    }

    /// **D1 + D4 through the daemon's own entry point.** A finished print
    /// produces no announcement and retracts a stale offer; an
    /// inconclusive verdict leaves the offer alone.
    #[test]
    fn boot_detection_handles_completion_and_stale_offers() {
        let config = temp_config("boot-complete");
        let pending_path = config.wal_dir.join(crate::detect::PENDING_FILE_NAME);
        let gcode = config.wal_dir.join("part.gcode");
        // 2000 bytes; from byte 500 on there is nothing but a footer.
        let mut text = String::from(";");
        while text.len() < 499 {
            text.push('p');
        }
        text.push('\n');
        let footer = "M107\nM104 S0\nM84\n";
        text.push_str(footer);
        text.push(';');
        while text.len() < 1_999 {
            text.push('p');
        }
        text.push('\n');
        assert_eq!(text.len(), 2_000);
        std::fs::write(&gcode, &text).unwrap();
        write_unclean_wal(&config, gcode.to_string_lossy().as_ref(), 500, false);

        // A stale offer from an earlier boot.
        std::fs::write(&pending_path, "{}").unwrap();
        assert!(
            super::boot_detection(&config).announcement.is_some(),
            "a completion message"
        );
        assert!(
            !pending_path.exists(),
            "a completed print must retract a stale offer"
        );

        // An unreadable WAL is inconclusive: the offer must survive.
        std::fs::write(&pending_path, "{}").unwrap();
        for entry in std::fs::read_dir(&config.wal_dir).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().is_some_and(|e| e == "plr") {
                std::fs::remove_file(path).unwrap();
            }
        }
        assert!(super::boot_detection(&config).announcement.is_none());
        assert!(
            pending_path.exists(),
            "an inconclusive verdict must not retract an offer"
        );
    }

    /// **Boot detection must never clear the Z-frame interlock.** The
    /// automatic clear this branch removed lived here, in `boot_detection`'s
    /// `Detection::Clean` arm, so this is the site that has to be guarded:
    /// a test that only exercises `detect()` passes while a clear
    /// reintroduced *here* goes unnoticed.
    ///
    /// The inference behind the removed clear was unsound.
    /// `SET_KINEMATIC_POSITION` — which `executor` issues to declare the
    /// shifted frame — marks axes homed by default
    /// (`klippy/extras/force_move.py`, `set_homed = gcmd.get('SET_HOMED',
    /// 'xyz')`), so after an aborted recovery Klipper believes Z is homed at
    /// the fabricated value and will not refuse the next print's motion.
    /// Only a fresh dry run clears the interlock.
    #[test]
    fn boot_detection_never_clears_the_frame_interlock() {
        use plr_wal::{Marker, MarkerKind, SegmentHeader, WalRecord, WalWriter};
        let config = temp_config("boot-keeps-interlock");
        // A clean tail: the arm that used to clear the interlock.
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer
            .append(&WalRecord::Marker(Marker {
                mono_ns: 2_000_000_000,
                kind: MarkerKind::CleanShutdown,
            }))
            .unwrap();
        std::fs::write(config.wal_dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        crate::detect::write_frame_invalid(
            &config.wal_dir,
            &crate::detect::FrameInvalid {
                detected_wall_ns: 1,
                step_id: 3,
                phase: "shifted-frame".to_owned(),
                reason: "shifted-frame-declared".to_owned(),
                arm_mono_ns: None,
            },
        )
        .unwrap();
        // A stale pending file, to prove the arm ran at all: it clears that
        // and only that.
        let pending_path = config.wal_dir.join(crate::detect::PENDING_FILE_NAME);
        std::fs::write(&pending_path, "{}").unwrap();

        assert!(super::boot_detection(&config).announcement.is_none());
        assert!(!pending_path.exists(), "the Clean arm must have run");
        assert!(
            crate::detect::read_frame_invalid(&config.wal_dir).is_some(),
            "boot detection must never clear the Z-frame interlock"
        );
    }

    /// A graceful recorder stop suppresses the announcement, through the
    /// WAL marker rather than any out-of-band state.
    #[test]
    fn boot_detection_honours_the_recorder_stopped_marker() {
        let config = temp_config("boot-recorder-stopped");
        let gcode = config.wal_dir.join("part.gcode");
        let mut text = String::from(";");
        while text.len() < 499 {
            text.push('p');
        }
        text.push_str(
            "
G1 X60 Y60 E900 F1800
",
        );
        std::fs::write(&gcode, &text).unwrap();
        // Unfinished work, no marker: a genuine offer.
        write_unclean_wal(&config, gcode.to_string_lossy().as_ref(), 500, false);
        assert!(super::boot_detection(&config).announcement.is_some());
        // The same WAL plus a graceful-stop marker: silence.
        write_unclean_wal(&config, gcode.to_string_lossy().as_ref(), 500, true);
        assert!(
            super::boot_detection(&config).announcement.is_none(),
            "a deliberate recorder stop must not announce"
        );
    }

    /// Writes a single-segment unclean WAL naming `file` at `position`,
    /// optionally ending with a graceful-stop marker.
    fn write_unclean_wal(config: &Config, file: &str, position: u64, recorder_stopped: bool) {
        use plr_wal::{SegmentHeader, WalRecord, WalWriter};
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer
            .append(&WalRecord::Heartbeat(plr_wal::Heartbeat {
                sequence: 1,
                mono_ns: 1_000_000_000,
                wall_ns: 1,
                print_time: 5.0,
                est_sample_mono_ns: 1_000_000_000,
                est_sample_print_time: 5.0,
                wal_offset: 32,
            }))
            .unwrap();
        writer
            .append(&WalRecord::Context(plr_wal::Context {
                mono_ns: 1_000_000_000,
                // Matches this fixture's heartbeat print time: Klipper
                // reports the trapq append frontier in the same status pass
                // as the file position.
                print_time: Some(5.0),
                virtual_sdcard: Some(plr_wal::VirtualSdState {
                    file_path: file.to_owned(),
                    file_position: position,
                    file_size: None,
                }),
                gcode: plr_wal::GcodeState {
                    speed_factor: 1.0,
                    speed: 1500.0,
                    extrude_factor: 1.0,
                    absolute_coordinates: true,
                    absolute_extrude: true,
                    homing_origin: vec![0.0; 4],
                    position: vec![0.0; 4],
                    gcode_position: vec![0.0; 4],
                },
                transforms: plr_wal::TransformObservations {
                    bed_mesh_active: false,
                    bed_mesh_profile: None,
                    z_thermal_adjust_enabled: None,
                    z_thermal_adjust_offset: None,
                    skew_active: false,
                    skew_profile: None,
                },
                heaters: Vec::new(),
                fans: Vec::new(),
                exclude: None,
                print_state: None,
                current_layer: None,
                total_layer: None,
            }))
            .unwrap();
        if recorder_stopped {
            writer
                .append(&WalRecord::Marker(plr_wal::Marker {
                    mono_ns: 2_000_000_000,
                    kind: plr_wal::MarkerKind::RecorderStopped,
                }))
                .unwrap();
        }
        std::fs::write(config.wal_dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
    }
}
