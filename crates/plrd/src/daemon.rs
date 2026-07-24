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

    let mut sender = WalSender::new(tx);
    let mut recorder = Recorder::new();
    let client_result = runtime.block_on(async {
        tokio::select! {
            result = run_client(&client_cfg, &mut sender, &mut recorder) => result,
            () = shutdown_signal() => Ok(()),
        }
    });

    // Final durability: ask the WAL thread to sync and exit, then judge
    // by its verdict.
    sender.shutdown();
    let wal_result = wal_thread.join();
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
}
