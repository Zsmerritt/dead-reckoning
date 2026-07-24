//! Boot-time pending-recovery detection.
//!
//! At daemon startup (before subscribing to Klipper) the previous
//! session's WAL is classified with the same reconstruction pipeline
//! `plrd scan` uses. An unclean end with a print in progress produces a
//! pending-recovery state file in the WAL directory and an operator
//! announcement; a clean end clears any stale state file.
//!
//! **Detection never executes anything.** Its entire output is a JSON
//! file and a console message; recovery starts only when the operator
//! runs `plrd recover` (and its gate stack) themselves.
//!
//! # Announcement channel
//!
//! Moonraker's announcement API cannot carry client-posted entries
//! (Moonraker docs, `external_api/announcements.md`: entries come from
//! RSS feeds and internal components only), so the supported channel
//! for an operator-visible message is a console message through
//! `printer.gcode.script` (Moonraker docs, `external_api/printer.md`):
//!
//! * primary: `RESPOND PREFIX=...` — shown in every console UI; needs
//!   `[respond]` in printer.cfg (Klipper docs, G-Codes → `[respond]`);
//! * fallback: `M117 ...` — the display status line; needs
//!   `[display_status]` (present in stock Mainsail/Fluidd configs).
//!
//! Delivery is best-effort with retries (klippy may be down at boot);
//! failures never affect recording — see `daemon::announce_pending`.

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::scan;

/// Name of the pending-recovery state file inside the WAL directory.
pub const PENDING_FILE_NAME: &str = "pending_recovery.json";

/// How many newest segments detection reads. Bounds the startup cost
/// regardless of how many segments have accumulated; classification
/// only needs the WAL tail (see `scan::load_merged_tail`).
const DETECT_SEGMENT_LIMIT: usize = 3;

/// What boot-time detection found.
#[derive(Debug, Clone, PartialEq)]
pub enum Detection {
    /// Unclean stop with a print in progress: recovery is available.
    Pending(PendingRecovery),
    /// The WAL ends cleanly; any stale pending file should be cleared.
    Clean,
    /// Nothing actionable (no WAL yet, no print context, or the WAL is
    /// unanalyzable). The reason is for the daemon log only.
    Nothing(String),
}

/// The pending-recovery state file contents.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PendingRecovery {
    /// Wall-clock time of detection (ns since the Unix epoch).
    pub detected_wall_ns: u64,
    /// Absolute path of the interrupted print file.
    pub file: String,
    /// Last durable file position, bytes.
    pub file_position: u64,
    /// Size of the print file on disk, if it was readable.
    pub file_size: Option<u64>,
    /// Rough completion percentage, if the size was known.
    pub percent: Option<f64>,
    /// Debug rendering of the crash classification.
    pub crash_class: String,
}

/// Classifies the WAL directory. Read-only; total.
#[must_use]
pub fn detect(wal_dir: &Path, heartbeat_path: &Path, detected_wall_ns: u64) -> Detection {
    let merged = match scan::load_merged_tail(wal_dir, DETECT_SEGMENT_LIMIT) {
        Ok(merged) => merged,
        Err(reason) => return Detection::Nothing(reason),
    };
    let heartbeat = scan::load_heartbeat(heartbeat_path).ok();
    let receive_seq = scan::load_receive_seq(&wal_dir.join(scan::RECEIVE_SEQ_FILE_NAME));
    // No file tail: detection only needs the classification, and must
    // stay cheap and independent of the print file's availability.
    let inputs = plr_reconstruct::ReconstructInputs {
        scan: &merged,
        heartbeat: heartbeat.as_ref(),
        file_tail: None,
        receive_seq,
    };
    let recovery =
        match plr_reconstruct::reconstruct(&inputs, &plr_reconstruct::ReconstructConfig::default())
        {
            Ok(plr_reconstruct::Reconstruction::CleanShutdown(_)) => return Detection::Clean,
            Ok(plr_reconstruct::Reconstruction::Recovery(recovery)) => recovery,
            Err(e) => return Detection::Nothing(format!("unclean WAL but unanalyzable: {e}")),
        };
    let Some((file, file_position)) = scan::last_print_file(&merged) else {
        return Detection::Nothing("unclean stop but no print was in progress".to_owned());
    };
    let file_size = std::fs::metadata(&file).map(|m| m.len()).ok();
    #[allow(clippy::cast_precision_loss)]
    let percent = file_size
        .filter(|size| *size > 0)
        .map(|size| (file_position.min(size) as f64 / size as f64) * 100.0);
    Detection::Pending(PendingRecovery {
        detected_wall_ns,
        file,
        file_position,
        file_size,
        percent,
        crash_class: format!("{:?}", recovery.window.class),
    })
}

/// Writes the state file (overwrite; plain write — this is operator
/// UX state, not recovery evidence).
pub fn write_pending(wal_dir: &Path, pending: &PendingRecovery) -> std::io::Result<()> {
    let json = serde_json::to_string_pretty(pending).map_err(std::io::Error::other)?;
    std::fs::write(wal_dir.join(PENDING_FILE_NAME), json)
}

/// Removes any stale state file.
pub fn clear_pending(wal_dir: &Path) {
    let _ = std::fs::remove_file(wal_dir.join(PENDING_FILE_NAME));
}

/// The operator announcement as `(primary, fallback)` G-Code commands
/// (see the module docs for why these two).
#[must_use]
pub fn announcement_commands(pending: &PendingRecovery) -> (String, String) {
    let name = pending
        .file
        .rsplit(['/', '\\'])
        .next()
        .unwrap_or(&pending.file)
        .replace(['"', '\''], "");
    let progress = pending
        .percent
        .map_or(String::new(), |p| format!(", ~{p:.0}% complete"));
    let message =
        format!("dead-reckoning: unfinished print '{name}' detected{progress}; run 'plrd recover' to inspect/resume");
    (
        format!("RESPOND PREFIX=dead-reckoning MSG=\"{message}\""),
        format!("M117 {message}"),
    )
}

#[cfg(test)]
mod tests {
    use super::{
        announcement_commands, clear_pending, detect, write_pending, Detection, PendingRecovery,
        PENDING_FILE_NAME,
    };
    use plr_wal::{
        Context, GcodeState, Heartbeat, Marker, MarkerKind, SegmentHeader, TransformObservations,
        VirtualSdState, WalRecord, WalWriter,
    };
    use std::path::{Path, PathBuf};

    fn temp_dir(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-detect-test-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn heartbeat(mono_ns: u64, print_time: f64) -> Heartbeat {
        Heartbeat {
            sequence: 5,
            mono_ns,
            wall_ns: 1_700_000_000_000_000_000,
            print_time,
            est_sample_mono_ns: mono_ns,
            est_sample_print_time: print_time,
            wal_offset: 64,
        }
    }

    fn context(mono_ns: u64, file_path: &str, file_position: u64) -> Context {
        Context {
            mono_ns,
            virtual_sdcard: Some(VirtualSdState {
                file_path: file_path.to_owned(),
                file_position,
            }),
            gcode: GcodeState {
                speed_factor: 1.0,
                speed: 1500.0,
                extrude_factor: 1.0,
                absolute_coordinates: true,
                absolute_extrude: true,
                homing_origin: vec![0.0; 4],
                position: vec![50.0, 50.0, 0.2, 10.0],
                gcode_position: vec![50.0, 50.0, 0.2, 10.0],
            },
            transforms: TransformObservations {
                bed_mesh_active: false,
                bed_mesh_profile: None,
                z_thermal_adjust_enabled: None,
                z_thermal_adjust_offset: None,
                skew_active: false,
                skew_profile: None,
            },
            heaters: Vec::new(),
            fans: Vec::new(),
        }
    }

    fn write_wal(dir: &Path, records: &[WalRecord]) {
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        for r in records {
            writer.append(r).unwrap();
        }
        std::fs::write(dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
    }

    #[test]
    fn unclean_print_yields_pending_with_percent() {
        let dir = temp_dir("pending");
        let gcode_path = dir.join("part.gcode");
        std::fs::write(&gcode_path, vec![b'G'; 2_000]).unwrap();
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(1_000_000_000, gcode_path.to_str().unwrap(), 500)),
            ],
        );
        let detection = detect(&dir, &dir.join("heartbeat.bin"), 777);
        let Detection::Pending(pending) = detection else {
            panic!("expected pending, got {detection:?}");
        };
        assert_eq!(pending.detected_wall_ns, 777);
        assert_eq!(pending.file_position, 500);
        assert_eq!(pending.file_size, Some(2_000));
        assert!((pending.percent.unwrap() - 25.0).abs() < 1e-9);
        assert!(pending.crash_class.contains("HostDeathOrPowerLoss"));

        // State file round-trips.
        write_pending(&dir, &pending).unwrap();
        let read: PendingRecovery =
            serde_json::from_str(&std::fs::read_to_string(dir.join(PENDING_FILE_NAME)).unwrap())
                .unwrap();
        assert_eq!(read, pending);
        clear_pending(&dir);
        assert!(!dir.join(PENDING_FILE_NAME).exists());
        clear_pending(&dir); // idempotent
    }

    #[test]
    fn clean_shutdown_detects_clean() {
        let dir = temp_dir("clean");
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(1_000_000_000, "/g/x.gcode", 500)),
                WalRecord::Marker(Marker {
                    mono_ns: 2_000_000_000,
                    kind: MarkerKind::CleanShutdown,
                }),
            ],
        );
        assert_eq!(
            detect(&dir, &dir.join("heartbeat.bin"), 1),
            Detection::Clean
        );
    }

    #[test]
    fn missing_wal_and_missing_context_are_nothing() {
        let dir = temp_dir("nothing");
        let Detection::Nothing(reason) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected nothing");
        };
        assert!(reason.contains("no WAL segments"), "{reason}");
        // Unclean but contextless: unanalyzable, not pending.
        write_wal(&dir, &[WalRecord::Heartbeat(heartbeat(1_000_000_000, 1.0))]);
        let Detection::Nothing(reason) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected nothing");
        };
        assert!(reason.contains("unanalyzable"), "{reason}");
    }

    #[test]
    fn unclean_print_with_missing_file_has_no_percent() {
        let dir = temp_dir("nofile");
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(1_000_000_000, "/nonexistent/x.gcode", 500)),
            ],
        );
        let Detection::Pending(pending) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected pending");
        };
        assert_eq!(pending.file_size, None);
        assert_eq!(pending.percent, None);
    }

    #[test]
    fn announcement_commands_are_console_safe() {
        let pending = PendingRecovery {
            detected_wall_ns: 1,
            file: "/g/dir/bench \"y\" 'z'.gcode".to_owned(),
            file_position: 500,
            file_size: Some(1_000),
            percent: Some(50.0),
            crash_class: "HostDeathOrPowerLoss".to_owned(),
        };
        let (primary, fallback) = announcement_commands(&pending);
        assert!(
            primary.starts_with("RESPOND PREFIX=dead-reckoning MSG=\""),
            "{primary}"
        );
        assert!(primary.contains("~50% complete"), "{primary}");
        assert!(primary.contains("plrd recover"), "{primary}");
        // Quotes are stripped from the file name so the MSG quoting
        // cannot be broken.
        assert!(!primary.contains("\"y\""), "{primary}");
        assert!(fallback.starts_with("M117 "), "{fallback}");
        // Unknown size: no percentage clause.
        let (primary, _) = announcement_commands(&PendingRecovery {
            percent: None,
            ..pending
        });
        assert!(!primary.contains('%'), "{primary}");
    }
}
