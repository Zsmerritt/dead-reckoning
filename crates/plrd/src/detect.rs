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

/// Name of the frame-invalidation marker file inside the WAL directory.
///
/// A separate file (not a field of [`PendingRecovery`]) so boot-time
/// detection can regenerate the pending state freely while this marker
/// persists independently until a fresh dry run consciously clears it.
pub const FRAME_INVALID_FILE_NAME: &str = "frame_invalid.json";

/// Staging name [`write_frame_invalid`] renames from. Fixed, not unique
/// — see that function for why.
pub const FRAME_INVALID_TEMP_NAME: &str = "frame_invalid.json.tmp";

/// Sentinel folded into [`PendingRecovery::crash_class`] and surfaced in
/// the operator announcement when the marker is present, so the pending
/// announcement notes the invalid frame without growing the pending
/// schema.
pub const FRAME_INVALID_NOTE: &str = "Z FRAME UNKNOWN";

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

/// The frame-invalidation marker: a recovery aborted at or after the
/// shifted-frame declaration, so Klipper's Z frame is in an unknown
/// state and a re-execute of the (now stale) plan is refused until a
/// fresh plan is generated. Written by the executor's caller on such an
/// abort, cleared by a fresh dry run (or a completed recovery).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FrameInvalid {
    /// Wall-clock time the frame was invalidated (ns since the epoch).
    pub detected_wall_ns: u64,
    /// The step id the recovery aborted at.
    pub step_id: u32,
    /// The phase name of that step.
    pub phase: String,
    /// The abort reason code.
    pub reason: String,
}

/// Path of the frame-invalidation marker in `wal_dir`.
#[must_use]
pub fn frame_invalid_path(wal_dir: &Path) -> std::path::PathBuf {
    wal_dir.join(FRAME_INVALID_FILE_NAME)
}

/// Reads the frame-invalidation marker, if present and parseable.
#[must_use]
pub fn read_frame_invalid(wal_dir: &Path) -> Option<FrameInvalid> {
    let text = std::fs::read_to_string(frame_invalid_path(wal_dir)).ok()?;
    serde_json::from_str(&text).ok()
}

/// Writes the frame-invalidation marker **atomically and durably**:
/// temp file in the same directory, `fsync` the file, `rename`, `fsync`
/// the directory.
///
/// # Why this file is durable and `pending_recovery.json` is not
///
/// The asymmetry is deliberate, and it is about what is lost when the
/// write does not survive.
///
/// `pending_recovery.json` is operator UX state: it says "a recovery is
/// available". Losing it costs an announcement, and boot-time detection
/// re-derives it from the WAL on the next start. A plain
/// [`std::fs::write`] is honest there ([`write_pending`]).
///
/// This file is a **safety interlock**. It asserts that Klipper's Z frame
/// is UNKNOWN because a recovery aborted at or after the shifted-frame
/// declaration, and [`crate::recover::execute_with_gates`] refuses
/// `--execute` for as long as it exists. Written into the page cache and
/// no further, a power cut between the write and the flush loses the
/// interlock silently — and the next `--execute` then drives the machine
/// against an unknown Z frame, which is the exact state the marker exists
/// to prevent. Losing it in a power event would be bad anywhere; losing
/// it in a power event is unacceptable in a *power-loss recovery* tool,
/// because that is not an unlikely coincidence here, it is the scenario.
///
/// So: rename-based publish (a reader sees the old marker or the new one,
/// never a half-written one), `fsync` before the rename so the bytes are
/// on the medium before the name points at them, and `fsync` of the
/// directory afterwards so the rename itself survives.
///
/// # What the staging file guarantees, precisely
///
/// The PUBLISHED marker is never partial: readers only ever see the file
/// under [`FRAME_INVALID_FILE_NAME`], and it only ever appears there by
/// `rename`. The staging file is a different claim: every failure path
/// *inside this process* unlinks it, but a `SIGKILL` or a power cut
/// between `create` and `rename` can leave one behind. That is harmless
/// — nothing reads it, and the next write truncates it — which is why
/// the name is fixed rather than unique.
///
/// # Errors
///
/// Any I/O failure, reported rather than swallowed. The caller
/// ([`crate::recover::MarkerFrameGuard`]) treats an error as "do not
/// enter the danger zone at all".
pub fn write_frame_invalid(wal_dir: &Path, marker: &FrameInvalid) -> std::io::Result<()> {
    use std::io::Write as _;

    let json = serde_json::to_string_pretty(marker).map_err(std::io::Error::other)?;
    // The temp file MUST live in the same directory as the target:
    // `rename` is only atomic within one filesystem, and a temp elsewhere
    // (say `/tmp`) can be on a different one.
    //
    // The name is FIXED rather than per-process/per-attempt. Every
    // in-process failure path unlinks it, but a SIGKILL between `create`
    // and `rename` cannot, and a unique name would leave that debris
    // behind forever. A fixed name is self-healing: the next write
    // truncates whatever is there. Writes are serialized anyway — one
    // recovery at a time, one writer.
    let temp_path = wal_dir.join(FRAME_INVALID_TEMP_NAME);
    let write_result = (|| -> std::io::Result<()> {
        let mut file = std::fs::File::create(&temp_path)?;
        file.write_all(json.as_bytes())?;
        file.flush()?;
        // fsync, not fdatasync: this is a fresh file, so its metadata
        // (the size) is as load-bearing as its contents.
        file.sync_all()
    })();
    if let Err(e) = write_result {
        let _ = std::fs::remove_file(&temp_path);
        return Err(e);
    }
    if let Err(e) = std::fs::rename(&temp_path, frame_invalid_path(wal_dir)) {
        let _ = std::fs::remove_file(&temp_path);
        return Err(e);
    }
    sync_dir(wal_dir)
}

/// `fsync`s a directory so a `rename` into it is durable.
///
/// Linux-gated the same way the rest of plrd's durability is: a directory
/// cannot be opened as a file on Windows, and plrd is a Linux daemon —
/// the non-Linux build exists so the cross-platform logic stays testable,
/// not to be deployed.
#[cfg(target_os = "linux")]
fn sync_dir(dir: &Path) -> std::io::Result<()> {
    std::fs::File::open(dir)?.sync_all()
}

/// See the Linux implementation. On other platforms the rename is still
/// atomic; only the directory-entry durability is unavailable.
#[cfg(not(target_os = "linux"))]
fn sync_dir(_dir: &Path) -> std::io::Result<()> {
    Ok(())
}

/// Removes any frame-invalidation marker (idempotent).
pub fn clear_frame_invalid(wal_dir: &Path) {
    let _ = std::fs::remove_file(frame_invalid_path(wal_dir));
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
        match plr_reconstruct::reconstruct(&inputs, &crate::convert::reconstruct_config(None)) {
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
    // Fold the frame-invalid note into the crash class so the pending
    // announcement can surface it without the pending schema growing a
    // field (the marker is authoritative; this is display only).
    //
    // TODO(daemon-in-scope): this crash_class sentinel is a workaround
    // because `announcement_commands(&PendingRecovery)` and its caller
    // `daemon.rs::boot_detection` are out of this change's scope. When
    // daemon.rs is next in scope, thread the frame-invalid flag through
    // explicitly (e.g. pass it to the announcement builder) and drop
    // both FRAME_INVALID_NOTE and this string fold.
    let mut crash_class = format!("{:?}", recovery.window.class);
    if read_frame_invalid(wal_dir).is_some() {
        crash_class.push_str("; ");
        crash_class.push_str(FRAME_INVALID_NOTE);
    }
    Detection::Pending(PendingRecovery {
        detected_wall_ns,
        file,
        file_position,
        file_size,
        percent,
        crash_class,
    })
}

/// Writes the state file (overwrite; plain write — this is operator UX
/// state, not recovery evidence).
///
/// Deliberately NOT durable, unlike [`write_frame_invalid`]: losing this
/// file costs an announcement, and boot-time [`detect`] re-derives it
/// from the WAL on the next start. Nothing refuses anything because of
/// it, so paying for `fsync` here would buy nothing. See
/// [`write_frame_invalid`] for the full asymmetry.
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
    // A prior recovery aborted after declaring the shifted frame: the
    // announcement warns the operator the Z frame is unknown and a fresh
    // dry run is required before resuming.
    let frame_note = if pending.crash_class.contains(FRAME_INVALID_NOTE) {
        "; Z frame is UNKNOWN after an aborted recovery — re-run a dry run (plrd scan / plrd recover) for a fresh plan before resuming"
    } else {
        ""
    };
    let message =
        format!("dead-reckoning: unfinished print '{name}' detected{progress}; run 'plrd recover' to inspect/resume{frame_note}");
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
            exclude: None,
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
    fn frame_invalid_marker_round_trips_and_is_noted_in_the_announcement() {
        use super::{
            clear_frame_invalid, read_frame_invalid, write_frame_invalid, FrameInvalid,
            FRAME_INVALID_NOTE,
        };
        let dir = temp_dir("frameinv");
        let gcode_path = dir.join("part.gcode");
        std::fs::write(&gcode_path, vec![b'G'; 2_000]).unwrap();
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(1_000_000_000, gcode_path.to_str().unwrap(), 500)),
            ],
        );
        // No marker: detection is a plain pending, no frame note.
        let Detection::Pending(pending) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected pending");
        };
        assert!(!pending.crash_class.contains(FRAME_INVALID_NOTE));
        let (primary, _) = announcement_commands(&pending);
        assert!(!primary.contains("Z frame is UNKNOWN"), "{primary}");

        // Write a marker: detection folds the note into the crash class,
        // and the announcement warns about the unknown frame.
        let marker = FrameInvalid {
            detected_wall_ns: 9,
            step_id: 7,
            phase: "shifted-frame".to_owned(),
            reason: "shifted-frame-not-declared".to_owned(),
        };
        write_frame_invalid(&dir, &marker).unwrap();
        assert_eq!(read_frame_invalid(&dir).unwrap(), marker);
        let Detection::Pending(pending) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected pending");
        };
        assert!(
            pending.crash_class.contains(FRAME_INVALID_NOTE),
            "{}",
            pending.crash_class
        );
        let (primary, fallback) = announcement_commands(&pending);
        assert!(primary.contains("Z frame is UNKNOWN"), "{primary}");
        assert!(fallback.contains("Z frame is UNKNOWN"), "{fallback}");

        // Clearing removes it; detection returns to a plain pending.
        clear_frame_invalid(&dir);
        assert!(read_frame_invalid(&dir).is_none());
        clear_frame_invalid(&dir); // idempotent
        let Detection::Pending(pending) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected pending");
        };
        assert!(!pending.crash_class.contains(FRAME_INVALID_NOTE));
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

    /// The interlock survives a write that is never explicitly flushed by
    /// the reader's path: `write_frame_invalid` returns only after the
    /// bytes AND the directory entry are on the medium, so a reader that
    /// opens the file fresh — as a rebooted daemon does — sees it.
    ///
    /// A unit test cannot cut the power, so what is asserted here is the
    /// observable contract: the call is synchronous through fsync, it
    /// publishes by rename, and it leaves no temp file behind for a
    /// later reader to trip over.
    #[test]
    fn the_frame_invalid_interlock_is_written_atomically_and_durably() {
        use super::{
            frame_invalid_path, read_frame_invalid, write_frame_invalid, FrameInvalid,
            FRAME_INVALID_FILE_NAME,
        };
        let dir = temp_dir("frame-durable");
        let marker = FrameInvalid {
            detected_wall_ns: 42,
            step_id: 9,
            phase: "shifted-frame".to_owned(),
            reason: "confirmation-timeout".to_owned(),
        };
        write_frame_invalid(&dir, &marker).expect("durable write");

        // Readable through a completely fresh path, with no flush of our
        // own: the write already synced.
        assert_eq!(read_frame_invalid(&dir), Some(marker.clone()));
        let raw = std::fs::read_to_string(frame_invalid_path(&dir)).unwrap();
        assert!(raw.contains("confirmation-timeout"), "{raw}");

        // Published by rename, and every in-process path cleans up after
        // itself, so a successful write leaves exactly the marker.
        // (A SIGKILL between `create` and `rename` can strand the staging
        // file; that is documented, harmless, and covered below.)
        let names: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(Result::ok)
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        assert_eq!(names, vec![FRAME_INVALID_FILE_NAME.to_owned()], "{names:?}");

        // Overwriting is equally atomic and leaves the same single file.
        let second = FrameInvalid {
            detected_wall_ns: 43,
            step_id: 10,
            phase: "probe".to_owned(),
            reason: "probe-no-trigger".to_owned(),
        };
        write_frame_invalid(&dir, &second).expect("durable overwrite");
        assert_eq!(read_frame_invalid(&dir), Some(second));
        assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 1);
    }

    /// An interlock that fails to persist must REPORT, never look like
    /// success — the caller has to be able to tell the operator that the
    /// automatic refusal is not in place.
    #[test]
    fn a_frame_invalid_write_failure_is_reported_not_swallowed() {
        use super::{read_frame_invalid, write_frame_invalid, FrameInvalid};
        let marker = FrameInvalid {
            detected_wall_ns: 1,
            step_id: 2,
            phase: "shifted-frame".to_owned(),
            reason: "shifted-frame-not-declared".to_owned(),
        };
        let missing = std::path::Path::new("/nonexistent-plrd-dir-frame-invalid-xyzzy");
        let error = write_frame_invalid(missing, &marker)
            .expect_err("an unwritable directory must be an error, not a silent no-op");
        // And nothing was left claiming success.
        assert!(read_frame_invalid(missing).is_none(), "{error}");
    }

    /// A staging file stranded by a previous SIGKILL is self-healing:
    /// the fixed name means the next write truncates it rather than
    /// accumulating debris, which is why the name is not unique.
    #[test]
    fn a_stranded_staging_file_is_reclaimed_by_the_next_write() {
        use super::{
            read_frame_invalid, write_frame_invalid, FrameInvalid, FRAME_INVALID_FILE_NAME,
            FRAME_INVALID_TEMP_NAME,
        };
        let dir = temp_dir("frame-stranded");
        // As a SIGKILL between create and rename would leave it.
        std::fs::write(dir.join(FRAME_INVALID_TEMP_NAME), b"{\"half\": written").unwrap();
        let marker = FrameInvalid {
            detected_wall_ns: 5,
            step_id: 2,
            phase: "shifted-frame".to_owned(),
            reason: "shifted-frame-declared".to_owned(),
        };
        write_frame_invalid(&dir, &marker).expect("write over the stranded staging file");
        assert_eq!(read_frame_invalid(&dir), Some(marker));
        let names: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(Result::ok)
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        assert_eq!(
            names,
            vec![FRAME_INVALID_FILE_NAME.to_owned()],
            "the stranded staging file must be consumed, not accumulated: {names:?}"
        );
    }

    /// The pending file's behaviour is unchanged: still a plain
    /// overwrite, still re-derivable, still no temp/rename dance. It is
    /// operator UX state, and paying for durability there would buy
    /// nothing (see `write_frame_invalid`'s docs for the asymmetry).
    #[test]
    fn the_pending_file_stays_a_plain_write() {
        use super::{write_pending, PendingRecovery, PENDING_FILE_NAME};
        let dir = temp_dir("pending-plain");
        let pending = PendingRecovery {
            detected_wall_ns: 7,
            file: "/g/part.gcode".to_owned(),
            file_position: 500,
            file_size: Some(1000),
            percent: Some(50.0),
            crash_class: "HostDeathOrPowerLoss".to_owned(),
        };
        write_pending(&dir, &pending).expect("write");
        let names: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(Result::ok)
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        assert_eq!(names, vec![PENDING_FILE_NAME.to_owned()], "{names:?}");
        let round_trip: PendingRecovery =
            serde_json::from_str(&std::fs::read_to_string(dir.join(PENDING_FILE_NAME)).unwrap())
                .unwrap();
        assert_eq!(round_trip, pending);
        // Overwrite in place, no rename step.
        write_pending(&dir, &pending).expect("overwrite");
        assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 1);
    }
}
