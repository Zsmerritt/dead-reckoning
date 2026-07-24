//! Offline `plrd scan`: read a WAL directory, validate everything, and
//! print a human recovery report.
//!
//! Pure file *reading* (no durability syscalls), so unlike the daemon it
//! works on any platform — a WAL directory copied off a printer can be
//! analyzed on a laptop.
//!
//! # Multi-segment handling
//!
//! Segments are scanned oldest→newest (their names embed a monotonic
//! index). Records from all segments are concatenated into one merged
//! scan for reconstruction; the truncation offset/reason come from the
//! newest segment, which is the only one that can legitimately end torn.
//! An earlier segment that does not end `CleanEof` is reported loudly:
//! rotation syncs a segment before opening its successor, so a torn
//! *earlier* segment means real corruption.

use std::io::Write;
use std::path::{Path, PathBuf};

use plr_reconstruct::{
    reconstruct, CrashClass, PossibleStopSet, ReceiveSeqObservation, ReconstructConfig,
    ReconstructInputs, Reconstruction, StopWindow,
};
use plr_wal::{
    recover_heartbeat, scan_read, HeartbeatRecovery, RecordKind, RecoveryScan, ScanEnd, WalRecord,
};

use crate::seqfile::decode_seq;

/// Hard cap on how much of one segment file is read into memory. Far
/// above any configured rotation size; bytes past it are treated as
/// truncation, exactly like `plr_wal::scan_read` documents.
const MAX_SEGMENT_READ: u64 = 1 << 30;

/// Default heartbeat file name inside the WAL directory (see
/// `Config::heartbeat_file`).
pub const HEARTBEAT_FILE_NAME: &str = "heartbeat.bin";

/// Receive-seq sidecar file name inside the WAL directory.
pub const RECEIVE_SEQ_FILE_NAME: &str = "receive_seq.bin";

/// File name of the WAL segment with the given index.
// The builder half lives beside the parser for symmetry; its production
// caller (`walsvc`) is Linux-only, so off-Linux only tests reach it.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
#[must_use]
pub fn segment_file_name(index: u64) -> String {
    format!("wal-{index:06}.plr")
}

/// Parses a segment index back out of a file name; `None` for files that
/// are not WAL segments.
#[must_use]
pub fn segment_index(name: &str) -> Option<u64> {
    let digits = name.strip_prefix("wal-")?.strip_suffix(".plr")?;
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    digits.parse().ok()
}

/// Runs the scan and writes the report to `out`. `Err` only for hard
/// failures (unreadable directory, no segments); data-quality problems
/// are part of the report.
pub fn run_scan(
    wal_dir: &Path,
    heartbeat_override: Option<&Path>,
    out: &mut dyn Write,
) -> Result<(), String> {
    let segments = list_segments(wal_dir)?;
    if segments.is_empty() {
        return Err(format!(
            "no WAL segments (wal-*.plr) found in {}",
            wal_dir.display()
        ));
    }

    let mut w = Report(out);
    w.line(&format!("plrd scan: {}", wal_dir.display()));

    let mut scans: Vec<RecoveryScan> = Vec::new();
    for (index, path) in &segments {
        let scan = scan_segment(path)?;
        report_segment(
            &mut w,
            *index,
            path,
            &scan,
            scans.len() + 1 == segments.len(),
        );
        scans.push(scan);
    }
    let merged = merge_scans(&scans);

    let heartbeat_path =
        heartbeat_override.map_or_else(|| wal_dir.join(HEARTBEAT_FILE_NAME), Path::to_path_buf);
    let heartbeat = read_heartbeat(&mut w, &heartbeat_path);

    let receive_seq = read_receive_seq(&mut w, &wal_dir.join(RECEIVE_SEQ_FILE_NAME));

    let file_tail_bytes = read_file_tail(&mut w, &merged);
    let file_tail = file_tail_bytes
        .as_deref()
        .map(|bytes| plr_reconstruct::FileTail {
            base_offset: 0,
            bytes,
        });

    let inputs = ReconstructInputs {
        scan: &merged,
        heartbeat: heartbeat.as_ref(),
        file_tail,
        receive_seq,
    };
    match reconstruct(&inputs, &ReconstructConfig::default()) {
        Ok(Reconstruction::CleanShutdown(_)) => {
            w.line("reconstruction: CLEAN SHUTDOWN — the print ended on purpose;");
            w.line("  no recovery is needed and none should be attempted.");
        }
        Ok(Reconstruction::Recovery(recovery)) => {
            report_window(&mut w, &recovery.window);
            report_stop_set(&mut w, &recovery.stop_set);
        }
        Err(e) => {
            w.line(&format!("reconstruction: not possible: {e}"));
            w.line("  (the WAL prefix above is still valid evidence)");
        }
    }
    Ok(())
}

/// Lists `(index, path)` for every segment, sorted by index. Shared
/// with the recovery pipeline (`pipeline`) and boot detection
/// (`detect`).
pub(crate) fn list_segments(wal_dir: &Path) -> Result<Vec<(u64, PathBuf)>, String> {
    let entries = std::fs::read_dir(wal_dir)
        .map_err(|e| format!("cannot read WAL directory {}: {e}", wal_dir.display()))?;
    let mut segments = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|e| format!("directory read error: {e}"))?;
        let name = entry.file_name();
        if let Some(index) = name.to_str().and_then(segment_index) {
            segments.push((index, entry.path()));
        }
    }
    segments.sort_unstable_by_key(|(index, _)| *index);
    Ok(segments)
}

pub(crate) fn scan_segment(path: &Path) -> Result<RecoveryScan, String> {
    let file = std::fs::File::open(path)
        .map_err(|e| format!("cannot open segment {}: {e}", path.display()))?;
    scan_read(file, MAX_SEGMENT_READ)
        .map_err(|e| format!("i/o error scanning {}: {e}", path.display()))
}

/// Concatenates the per-segment valid prefixes. Offsets stay
/// segment-relative (they are only used for reporting and resume, and
/// resume applies to the newest segment alone); header, truncation
/// offset, and end reason come from the newest segment.
pub(crate) fn merge_scans(scans: &[RecoveryScan]) -> RecoveryScan {
    let last = scans.last().expect("caller guarantees at least one scan");
    RecoveryScan {
        header: last.header.clone(),
        records: scans.iter().flat_map(|s| s.records.clone()).collect(),
        truncation_offset: last.truncation_offset,
        end: last.end.clone(),
    }
}

/// Loads and merges the whole WAL directory (segments only), without
/// printing. `Err` for an unreadable/empty directory.
pub(crate) fn load_merged(wal_dir: &Path) -> Result<RecoveryScan, String> {
    load_merged_tail(wal_dir, usize::MAX)
}

/// Like [`load_merged`] but bounded to the newest `max_segments`
/// segments. Boot-time detection uses a small bound so a months-old
/// WAL directory cannot delay the recorder at startup; the newest
/// segments carry everything classification needs (the tail heartbeat,
/// contexts, and markers).
pub(crate) fn load_merged_tail(
    wal_dir: &Path,
    max_segments: usize,
) -> Result<RecoveryScan, String> {
    let mut segments = list_segments(wal_dir)?;
    if segments.is_empty() {
        return Err(format!(
            "no WAL segments (wal-*.plr) found in {}",
            wal_dir.display()
        ));
    }
    if segments.len() > max_segments {
        segments.drain(..segments.len() - max_segments);
    }
    let mut scans = Vec::new();
    for (_, path) in &segments {
        scans.push(scan_segment(path)?);
    }
    Ok(merge_scans(&scans))
}

/// Loads the heartbeat file without printing; `Err` carries the
/// human-readable reason.
pub(crate) fn load_heartbeat(path: &Path) -> Result<HeartbeatRecovery, String> {
    let bytes = std::fs::read(path)
        .map_err(|e| format!("heartbeat {}: unreadable ({e})", path.display()))?;
    recover_heartbeat(&bytes)
        .map_err(|e| format!("heartbeat {}: unrecoverable: {e}", path.display()))
}

/// Loads the receive-seq sidecar without printing; `None` when absent
/// or invalid (both are the conservative direction).
pub(crate) fn load_receive_seq(path: &Path) -> Option<ReceiveSeqObservation> {
    let (mono_ns, widened_seq) = decode_seq(&std::fs::read(path).ok()?)?;
    Some(ReceiveSeqObservation {
        mono_ns,
        widened_seq,
    })
}

/// The newest context's print file path and position, if any context
/// named one.
pub(crate) fn last_print_file(merged: &RecoveryScan) -> Option<(String, u64)> {
    merged.records.iter().rev().find_map(|r| match &r.record {
        WalRecord::Context(c) => c
            .virtual_sdcard
            .as_ref()
            .map(|v| (v.file_path.clone(), v.file_position)),
        _ => None,
    })
}

struct Report<'a>(&'a mut dyn Write);

impl Report<'_> {
    fn line(&mut self, text: &str) {
        // A broken pipe while printing a report is not worth handling.
        let _ = writeln!(self.0, "{text}");
    }
}

fn report_segment(w: &mut Report<'_>, index: u64, path: &Path, scan: &RecoveryScan, newest: bool) {
    let mut counts = [0_usize; 5];
    for r in &scan.records {
        let slot = match r.record.kind() {
            RecordKind::TrapqSegment => 0,
            RecordKind::StepperRange => 1,
            RecordKind::Context => 2,
            RecordKind::Marker => 3,
            RecordKind::Heartbeat => 4,
        };
        counts[slot] += 1;
    }
    w.line(&format!(
        "segment {index} ({}): {} records (trapq {}, stepper {}, context {}, marker {}, heartbeat {})",
        path.display(),
        scan.records.len(),
        counts[0],
        counts[1],
        counts[2],
        counts[3],
        counts[4],
    ));
    let expected = scan.end.is_expected_after_power_loss();
    w.line(&format!(
        "  valid prefix ends at byte {}: {} (expected after power loss: {})",
        scan.truncation_offset,
        scan.end,
        if expected { "yes" } else { "NO" },
    ));
    if !newest && scan.end != ScanEnd::CleanEof {
        w.line("  WARNING: a non-newest segment should always end cleanly (rotation");
        w.line("  syncs before opening the successor); this indicates corruption of");
        w.line("  previously durable data.");
    }
}

fn read_heartbeat(w: &mut Report<'_>, path: &Path) -> Option<HeartbeatRecovery> {
    match std::fs::read(path) {
        Err(e) => {
            w.line(&format!("heartbeat {}: unreadable ({e})", path.display()));
            None
        }
        Ok(bytes) => match recover_heartbeat(&bytes) {
            Ok(recovery) => {
                let hb = &recovery.heartbeat;
                w.line(&format!(
                    "heartbeat {}: slot {:?} seq {} print_time {:.4}s wal_offset {}",
                    path.display(),
                    recovery.slot,
                    hb.sequence,
                    hb.print_time,
                    hb.wal_offset,
                ));
                if let Some((slot, err)) = &recovery.torn {
                    w.line(&format!(
                        "  other slot {slot:?} torn: {err} (expected after power loss mid-rewrite)"
                    ));
                }
                Some(recovery)
            }
            Err(e) => {
                w.line(&format!("heartbeat {}: unrecoverable: {e}", path.display()));
                None
            }
        },
    }
}

fn read_receive_seq(w: &mut Report<'_>, path: &Path) -> Option<ReceiveSeqObservation> {
    match std::fs::read(path) {
        Err(_) => {
            w.line("receive_seq sidecar: absent");
            None
        }
        Ok(bytes) => {
            let Some((mono_ns, widened_seq)) = decode_seq(&bytes) else {
                w.line("receive_seq sidecar: present but invalid (torn write); ignored");
                return None;
            };
            w.line(&format!(
                "receive_seq sidecar: widened {widened_seq} at mono {mono_ns} ns"
            ));
            Some(ReceiveSeqObservation {
                mono_ns,
                widened_seq,
            })
        }
    }
}

/// Reads the printed file named by the newest context record, for the
/// forward-simulation extension. Whole-file read with `base_offset` 0;
/// print files are tens of MB at most.
fn read_file_tail(w: &mut Report<'_>, merged: &RecoveryScan) -> Option<Vec<u8>> {
    let path = merged.records.iter().rev().find_map(|r| match &r.record {
        WalRecord::Context(c) => c.virtual_sdcard.as_ref().map(|v| v.file_path.clone()),
        _ => None,
    });
    let Some(path) = path else {
        w.line("print file: no context names one; forward extension disabled");
        return None;
    };
    match std::fs::read(&path) {
        Ok(bytes) => {
            w.line(&format!("print file: {path} ({} bytes)", bytes.len()));
            Some(bytes)
        }
        Err(e) => {
            w.line(&format!(
                "print file: {path} unreadable ({e}); forward extension disabled"
            ));
            None
        }
    }
}

fn report_window(w: &mut Report<'_>, window: &StopWindow) {
    w.line("reconstruction: RECOVERY");
    let class = match &window.class {
        CrashClass::CleanShutdown => "clean shutdown".to_owned(),
        CrashClass::ShutdownPowerRetained { evidence } => {
            format!("klippy/MCU shutdown with power retained (evidence: {evidence:?})")
        }
        CrashClass::HostDeathOrPowerLoss { torn_tail } => {
            format!("host death or power loss (torn WAL tail: {torn_tail})")
        }
    };
    w.line(&format!("  crash class: {class}"));
    w.line(&format!(
        "  stop window: t_a {:.4}s .. t_b {:.4}s (t_b source: {:?})",
        window.t_a, window.t_b, window.t_b_source,
    ));
    for anomaly in &window.anomalies {
        w.line(&format!("  window anomaly: {anomaly:?}"));
    }
}

fn report_stop_set(w: &mut Report<'_>, set: &PossibleStopSet) {
    w.line(&format!(
        "  WAL evaluation span: {:.4}s .. {:.4}s",
        set.t_a, set.wal_eval_end
    ));
    if let Some(window) = &set.file_window {
        w.line(&format!(
            "  file offset window: bytes {} .. {}",
            window.start, window.end
        ));
    }
    w.line(&format!("  Z candidates: {}", set.z_candidates.len()));
    for c in &set.z_candidates {
        w.line(&format!(
            "    z [{:.4}, {:.4}] mm  kind {:?}  provenance {:?}  known {}",
            c.z.lo, c.z.hi, c.kind, c.provenance, c.z_known
        ));
    }
    if let Some(xy) = &set.xy {
        w.line(&format!(
            "  XY region: x [{:.3}, {:.3}] mm, y [{:.3}, {:.3}] mm",
            xy.x.lo, xy.x.hi, xy.y.lo, xy.y.hi
        ));
    }
    if let Some(e) = &set.e_internal {
        w.line(&format!(
            "  E internal frame: [{:.4}, {:.4}] mm",
            e.lo, e.hi
        ));
    }
    if let Some(e) = &set.e_file {
        w.line(&format!("  E file frame: [{:.4}, {:.4}] mm", e.lo, e.hi));
    }
    if let Some(ext) = &set.extension {
        w.line(&format!("  forward extension: {ext:?}"));
    }
    w.line(&format!(
        "  confidence: {:?}; degradations: {:?}",
        set.degradation.confidence, set.degradation
    ));
}

#[cfg(test)]
mod tests {
    use super::{merge_scans, run_scan, segment_file_name, segment_index};
    use crate::seqfile::encode_seq;
    use plr_wal::{
        encode_slot, Context, GcodeState, Heartbeat, Marker, MarkerKind, ScanEnd, SegmentHeader,
        TransformObservations, VirtualSdState, WalRecord, WalWriter,
    };
    use std::path::{Path, PathBuf};

    /// A unique per-test temp dir (no tempfile dep by policy).
    fn temp_dir(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-scan-test-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn heartbeat(sequence: u64, mono_ns: u64, print_time: f64) -> Heartbeat {
        Heartbeat {
            sequence,
            mono_ns,
            wall_ns: 1_700_000_000_000_000_000,
            print_time,
            est_sample_mono_ns: mono_ns,
            est_sample_print_time: print_time,
            wal_offset: 64,
        }
    }

    fn context(mono_ns: u64, file_position: u64, file_path: &str) -> Context {
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

    fn write_segment(dir: &Path, index: u64, records: &[WalRecord]) -> Vec<u8> {
        let mut writer = WalWriter::create(
            Vec::new(),
            &SegmentHeader::new(1_700_000_000_000_000_000, 500),
        )
        .unwrap();
        for r in records {
            writer.append(r).unwrap();
        }
        let bytes = writer.into_inner();
        std::fs::write(dir.join(segment_file_name(index)), &bytes).unwrap();
        bytes
    }

    fn scan_to_string(dir: &Path) -> String {
        let mut buf = Vec::new();
        run_scan(dir, None, &mut buf).unwrap();
        String::from_utf8(buf).unwrap()
    }

    #[test]
    fn segment_names_round_trip_and_sort() {
        assert_eq!(segment_file_name(1), "wal-000001.plr");
        assert_eq!(segment_file_name(1_234_567), "wal-1234567.plr");
        assert_eq!(segment_index("wal-000042.plr"), Some(42));
        assert_eq!(segment_index("wal-1234567.plr"), Some(1_234_567));
        for bad in [
            "wal-.plr",
            "wal-12x.plr",
            "heartbeat.bin",
            "wal-000001.tmp",
            "receive_seq.bin",
        ] {
            assert_eq!(segment_index(bad), None, "{bad}");
        }
    }

    #[test]
    fn missing_dir_and_empty_dir_are_hard_errors() {
        let mut buf = Vec::new();
        assert!(run_scan(Path::new("/nonexistent-plrd"), None, &mut buf).is_err());
        let dir = temp_dir("empty");
        let err = run_scan(&dir, None, &mut buf).unwrap_err();
        assert!(err.contains("no WAL segments"), "{err}");
    }

    #[test]
    fn clean_shutdown_wal_reports_no_recovery() {
        let dir = temp_dir("clean");
        write_segment(
            &dir,
            1,
            &[
                WalRecord::Heartbeat(heartbeat(1, 1_000_000_000, 10.0)),
                WalRecord::Context(context(1_000_000_000, 100, "/nonexistent/x.gcode")),
                WalRecord::Marker(Marker {
                    mono_ns: 2_000_000_000,
                    kind: MarkerKind::CleanShutdown,
                }),
            ],
        );
        let report = scan_to_string(&dir);
        assert!(report.contains("CLEAN SHUTDOWN"), "{report}");
        assert!(report.contains("ends cleanly"), "{report}");
        assert!(report.contains("heartbeat"), "{report}");
    }

    #[test]
    fn unclean_stop_reports_recovery_with_stop_window() {
        let dir = temp_dir("recovery");
        // Heartbeat + context, then a marker whose append the power cut
        // tears mid-frame: a realistic power-loss shape.
        let full = write_segment(
            &dir,
            1,
            &[
                WalRecord::Heartbeat(heartbeat(3, 5_000_000_000, 42.0)),
                WalRecord::Context(context(5_000_000_000, 512, "/nonexistent/x.gcode")),
                WalRecord::Marker(Marker {
                    mono_ns: 5_100_000_000,
                    kind: MarkerKind::Resubscribed,
                }),
            ],
        );
        // Tear the last 5 bytes off to simulate power loss mid-append.
        std::fs::write(dir.join(segment_file_name(1)), &full[..full.len() - 5]).unwrap();
        // Heartbeat file with both slots.
        let mut hb_file = Vec::new();
        hb_file.extend_from_slice(&encode_slot(&heartbeat(2, 4_900_000_000, 41.9)));
        hb_file.extend_from_slice(&encode_slot(&heartbeat(3, 5_000_000_000, 42.0)));
        std::fs::write(dir.join("heartbeat.bin"), &hb_file).unwrap();
        // Sidecar observation.
        std::fs::write(dir.join("receive_seq.bin"), encode_seq(5_000_000_000, 900)).unwrap();

        let report = scan_to_string(&dir);
        assert!(report.contains("RECOVERY"), "{report}");
        assert!(
            report.contains("crash class: host death or power loss"),
            "{report}"
        );
        assert!(report.contains("stop window"), "{report}");
        assert!(
            report.contains("expected after power loss: yes"),
            "{report}"
        );
        assert!(report.contains("widened 900"), "{report}");
        assert!(report.contains("Z candidates"), "{report}");
        assert!(
            report.contains("forward extension disabled"),
            "missing print file must be reported: {report}"
        );
    }

    #[test]
    fn multi_segment_scan_merges_in_index_order() {
        let dir = temp_dir("multi");
        write_segment(
            &dir,
            2,
            &[WalRecord::Heartbeat(heartbeat(9, 9_000_000_000, 99.0))],
        );
        write_segment(
            &dir,
            1,
            &[WalRecord::Marker(Marker {
                mono_ns: 1,
                kind: MarkerKind::Resubscribed,
            })],
        );
        let report = scan_to_string(&dir);
        let seg1 = report.find("segment 1").unwrap();
        let seg2 = report.find("segment 2").unwrap();
        assert!(seg1 < seg2, "segments must report oldest first: {report}");
    }

    #[test]
    fn corrupt_middle_segment_warns_loudly() {
        let dir = temp_dir("corrupt");
        let full = write_segment(
            &dir,
            1,
            &[WalRecord::Heartbeat(heartbeat(1, 1_000_000_000, 1.0))],
        );
        // Corrupt a payload byte in segment 1 (not the newest).
        let mut bad = full.clone();
        let n = bad.len();
        bad[n - 6] ^= 0x20;
        std::fs::write(dir.join(segment_file_name(1)), &bad).unwrap();
        write_segment(
            &dir,
            2,
            &[WalRecord::Heartbeat(heartbeat(2, 2_000_000_000, 2.0))],
        );
        let report = scan_to_string(&dir);
        assert!(report.contains("WARNING"), "{report}");
        assert!(report.contains("expected after power loss: NO"), "{report}");
    }

    #[test]
    fn invalid_sidecar_and_heartbeat_are_reported_not_fatal() {
        let dir = temp_dir("degraded");
        write_segment(
            &dir,
            1,
            &[WalRecord::Heartbeat(heartbeat(1, 1_000_000_000, 1.0))],
        );
        std::fs::write(dir.join("receive_seq.bin"), [0_u8; 24]).unwrap();
        std::fs::write(dir.join("heartbeat.bin"), [0_u8; 128]).unwrap();
        let report = scan_to_string(&dir);
        assert!(report.contains("invalid (torn write)"), "{report}");
        assert!(report.contains("unrecoverable"), "{report}");
        // Reconstruction proceeds from WAL heartbeat records alone but
        // lacks a context: reported, not a crash.
        assert!(report.contains("not possible"), "{report}");
    }

    #[test]
    fn heartbeat_override_path_is_used() {
        let dir = temp_dir("hb-override");
        write_segment(
            &dir,
            1,
            &[WalRecord::Heartbeat(heartbeat(1, 1_000_000_000, 1.0))],
        );
        let hb_path = dir.join("custom-hb.bin");
        std::fs::write(&hb_path, encode_slot(&heartbeat(7, 1_000, 3.5))).unwrap();
        let mut buf = Vec::new();
        run_scan(&dir, Some(&hb_path), &mut buf).unwrap();
        let report = String::from_utf8(buf).unwrap();
        assert!(report.contains("custom-hb.bin"), "{report}");
        assert!(report.contains("seq 7"), "{report}");
    }

    #[test]
    fn merge_scans_takes_tail_metadata_from_newest() {
        let a = plr_wal::scan(&write_segment(
            &temp_dir("merge-a"),
            1,
            &[WalRecord::Marker(Marker {
                mono_ns: 1,
                kind: MarkerKind::Resubscribed,
            })],
        ));
        let bytes = {
            let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(9, 9)).unwrap();
            writer
                .append(&WalRecord::Marker(Marker {
                    mono_ns: 2,
                    kind: MarkerKind::SocketLost,
                }))
                .unwrap();
            writer.into_inner()
        };
        let b = plr_wal::scan(&bytes[..bytes.len() - 3]);
        assert_eq!(b.end, ScanEnd::TruncatedPayload);
        let merged = merge_scans(&[a.clone(), b.clone()]);
        assert_eq!(merged.records.len(), 1); // b's torn record yields none
        assert_eq!(merged.end, ScanEnd::TruncatedPayload);
        assert_eq!(merged.truncation_offset, b.truncation_offset);
        assert_eq!(merged.header, b.header);
        assert_eq!(merged.records[0].record.mono_ns(), 1);
    }
}
