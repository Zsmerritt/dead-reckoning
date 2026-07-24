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
    }
}

/// Runs the daemon until a signal or a fatal error. Returns the process
/// exit code.
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
    let announcement = boot_detection(&config);

    let (tx, rx) = std::sync::mpsc::sync_channel::<WalCmd>(config.channel_capacity);
    let wal_thread = walsvc::spawn(wal_cfg(&config), rx);

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
    let client_result = runtime.block_on(async {
        // The control server runs beside the recorder. Its handlers do
        // heavy work on spawn_blocking and its executions await
        // Moonraker I/O, so it cannot starve the socket reader (see
        // ctrlsock's module docs).
        tokio::spawn(crate::ctrlsock::serve(ctrl_listener, ctrl_state));
        // Best-effort operator announcement, concurrent with recording;
        // it can never block, delay, or fail the recorder.
        if let Some(commands) = announcement {
            tokio::spawn(announce_pending(moonraker_url, commands));
        }
        tokio::select! {
            result = run_client(&client_cfg, &mut sender, &mut recorder) => result,
            () = shutdown_signal() => Ok(()),
        }
    });

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

/// Classifies the previous session and prepares the announcement
/// commands, if any (see `detect` for the semantics).
fn boot_detection(config: &Config) -> Option<(String, String)> {
    use crate::detect::{self, Detection};
    match detect::detect(
        &config.wal_dir,
        &config.heartbeat_file(),
        crate::hostclock::now_wall_ns(),
    ) {
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
            if let Err(e) = detect::write_pending(&config.wal_dir, &pending) {
                eprintln!("plrd: cannot write pending-recovery state: {e}");
            }
            Some(detect::announcement_commands(&pending))
        }
        Detection::Clean => {
            detect::clear_pending(&config.wal_dir);
            None
        }
        Detection::Nothing(reason) => {
            eprintln!("plrd: no pending recovery: {reason}");
            None
        }
    }
}

/// Delivers the operator announcement via Moonraker
/// (`printer.gcode.script`; see `detect` module docs for the channel
/// choice). Klippy may be down at boot: retried on a slow cadence,
/// bounded, and abandoned silently — never affecting recording.
async fn announce_pending(url: String, commands: (String, String)) {
    use crate::moonraker::MoonrakerClient;
    const ATTEMPTS: u32 = 30;
    const RETRY: std::time::Duration = std::time::Duration::from_secs(10);
    for _ in 0..ATTEMPTS {
        if let Ok(mut client) =
            MoonrakerClient::connect(&url, std::time::Duration::from_secs(5)).await
        {
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

    #[test]
    fn boot_detection_writes_pending_and_prepares_announcement() {
        use plr_wal::{SegmentHeader, WalRecord, WalWriter};
        let config = temp_config("boot-pending");
        // Empty WAL dir: nothing pending, no announcement.
        assert!(super::boot_detection(&config).is_none());
        // Unclean WAL with a print in progress: pending + announcement,
        // and the state file exists.
        let gcode = config.wal_dir.join("part.gcode");
        std::fs::write(&gcode, vec![b'G'; 1_000]).unwrap();
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
                virtual_sdcard: Some(plr_wal::VirtualSdState {
                    file_path: gcode.to_string_lossy().into_owned(),
                    file_position: 250,
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
            }))
            .unwrap();
        std::fs::write(config.wal_dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        let commands = super::boot_detection(&config).expect("announcement expected");
        assert!(commands.0.starts_with("RESPOND"), "{}", commands.0);
        assert!(commands.1.starts_with("M117"), "{}", commands.1);
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
        assert!(super::boot_detection(&config).is_none());
        assert!(!pending_path.exists());
    }

    #[tokio::test]
    async fn announce_pending_delivers_primary_and_falls_back() {
        use crate::testmoon::FakeMoonraker;
        // Primary accepted.
        let fake = FakeMoonraker::spawn(|_, _| Ok(serde_json::json!("ok"))).await;
        super::announce_pending(
            fake.url(),
            ("RESPOND MSG=\"x\"".to_owned(), "M117 x".to_owned()),
        )
        .await;
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
        super::announce_pending(
            fake.url(),
            ("RESPOND MSG=\"x\"".to_owned(), "M117 x".to_owned()),
        )
        .await;
        assert_eq!(
            fake.gcode_sent(),
            vec!["RESPOND MSG=\"x\"", "M117 x"],
            "fallback must follow a rejected primary"
        );
    }
}
