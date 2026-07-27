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
    anchor_state_from_context, reconstruct, select_crash_epoch, CrashClass, EpochBoundaryKind,
    PossibleStopSet, ReceiveSeqObservation, ReconstructInputs, Reconstruction,
    RecoveryReconstruction, StopWindow,
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

/// Power-fail sidecar file name inside the WAL directory (the power-fail
/// watcher's first, channel-bypassing durability copy of the edge time).
/// Must match `crate::config::Config::power_fail_sidecar_file`'s basename.
pub const POWER_FAIL_FILE_NAME: &str = "power_fail.bin";

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
    report_epochs(&mut w, &merged);

    let heartbeat_path =
        heartbeat_override.map_or_else(|| wal_dir.join(HEARTBEAT_FILE_NAME), Path::to_path_buf);
    let heartbeat = read_heartbeat(&mut w, &heartbeat_path);

    let receive_seq = read_receive_seq(&mut w, &wal_dir.join(RECEIVE_SEQ_FILE_NAME));
    // Sidecar if present, else the edge boot detection persisted into the
    // pending file — the SAME resolution the recovery pipeline uses, so
    // `plrd scan`'s interlock verdict cannot disagree with `plrd recover`.
    let power_fail_edge_mono_ns = power_fail_edge(wal_dir);

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
        power_fail_edge_mono_ns,
    };
    // The crash epoch's tail power-fail edge, when reconstruction found one
    // — used below to decide whether an armed frame interlock was cut off by
    // power loss.
    let mut power_failing_tail = None;
    match reconstruct(&inputs, &crate::convert::reconstruct_config(None)) {
        Ok(Reconstruction::CleanShutdown(_)) => {
            w.line("reconstruction: CLEAN SHUTDOWN — the print ended on purpose;");
            w.line("  no recovery is needed and none should be attempted.");
        }
        Ok(Reconstruction::Recovery(recovery)) => {
            power_failing_tail = recovery.timeline.power_failing_tail();
            report_window(&mut w, &recovery.window);
            report_stop_set(&mut w, &recovery.stop_set);
            report_layer_attribution(&mut w, &recovery, file_tail_bytes.as_deref());
        }
        Err(e) => {
            w.line(&format!("reconstruction: not possible: {e}"));
            w.line("  (the WAL prefix above is still valid evidence)");
        }
    }
    report_frame_interlock(&mut w, wal_dir, power_failing_tail);
    Ok(())
}

/// Reports the Z-frame interlock (`frame_invalid.json`) when armed, naming
/// a power-loss interruption specifically when the reconstructed crash
/// epoch's `PowerFailing` edge postdates the arming (the same verdict boot
/// detection folds into the pending file). Silent when the interlock is
/// absent — nothing to warn about.
fn report_frame_interlock(w: &mut Report<'_>, wal_dir: &Path, power_failing_tail: Option<u64>) {
    let Some(marker) = crate::detect::read_frame_invalid(wal_dir) else {
        return;
    };
    if crate::detect::interrupted_by_power_fail(&marker, power_failing_tail) {
        w.line("frame interlock: ARMED — the previous recovery was interrupted by power loss;");
        w.line("  Z frame is UNKNOWN. A fresh dry run is required before resuming.");
    } else {
        w.line(&format!(
            "frame interlock: ARMED (reason: {}, phase {}) — a previous recovery declared the",
            marker.reason, marker.phase
        ));
        w.line(
            "  shifted Z frame; Z frame is UNKNOWN. A fresh dry run is required before resuming.",
        );
    }
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

/// Loads the power-fail sidecar (edge `mono_ns`); `None` when absent,
/// torn, or foreign — the conservative direction (a missing edge only
/// widens reconstruction). `crate::powerfail` owns the codec.
///
/// A file that is PRESENT but does not decode (torn/foreign/zeroed) is a
/// data-quality event, so it is logged naming the file — per project
/// convention — while still returning the safe `None`. A genuinely absent
/// file logs nothing (the common, expected case).
pub(crate) fn load_power_fail_edge(path: &Path) -> Option<u64> {
    let bytes = std::fs::read(path).ok()?;
    let edge = crate::powerfail::decode_power_fail_edge(&bytes);
    if edge.is_none() {
        // Present but undecodable: a data-quality event, logged naming the
        // file (per project convention) while still returning the safe None.
        eprintln!(
            "plrd: power-fail sidecar {} is present but unreadable \
             (torn/foreign/zeroed); ignoring it",
            path.display()
        );
    }
    edge
}

/// The crash's power-fail edge, resolved the ONE way every reconstruction
/// surface must resolve it: the write-once sidecar if it is still there,
/// else the copy boot detection PERSISTED into `pending_recovery.json`
/// (`crate::detect::PendingRecovery::power_fail_edge_mono_ns`) — because the
/// daemon deletes the sidecar at boot once detection has consumed it, so any
/// LATER run (`plrd recover`, `plrd scan`) finds it gone.
///
/// Shared by the recovery pipeline AND `plrd scan` so the two never give
/// different answers about one power-loss event: without this, `scan` would
/// read the (deleted) sidecar only, reconstruct with no edge, and print the
/// generic interlock branch while `recover` and the boot announcement — both
/// of which see the persisted edge — attribute the power loss.
///
/// The edge is fed downstream through the SAME `power_fail_edge_mono_ns`
/// input the sidecar used, so a persisted edge obeys the identical
/// `sidecar_admits` epoch band: one not adjacent to the current crash tail is
/// rejected exactly as a stale sidecar would be, so an old pending edge
/// cannot resurrect against an unrelated later crash.
pub(crate) fn power_fail_edge(wal_dir: &Path) -> Option<u64> {
    load_power_fail_edge(&wal_dir.join(POWER_FAIL_FILE_NAME)).or_else(|| {
        match crate::detect::read_pending_presence(wal_dir) {
            crate::detect::StatePresence::Present(pending) => pending.power_fail_edge_mono_ns,
            // Absent or torn pending file: no persisted edge to fall back to.
            _ => None,
        }
    })
}

/// The half-open `[start, end)` index range of the crash epoch within a
/// merged scan — the newest boot/firmware session that was printing (see
/// `plr_reconstruct::epoch`). Falls back to the whole stream when there
/// is nothing to partition. File selection and the reconstruction share
/// this so the print file always comes from the epoch being recovered,
/// never from an older boot or a post-crash idle boot.
pub(crate) fn crash_epoch_range(merged: &RecoveryScan) -> (usize, usize) {
    plr_reconstruct::select_crash_epoch(merged)
        .crash_epoch()
        .map_or((0, merged.records.len()), |e| (e.start, e.end))
}

/// The newest context's print file path and position within the crash
/// epoch, if any context there named one.
pub(crate) fn last_print_file(merged: &RecoveryScan) -> Option<(String, u64)> {
    let (start, end) = crash_epoch_range(merged);
    merged.records[start..end]
        .iter()
        .rev()
        .find_map(|r| match &r.record {
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
    let (start, end) = crash_epoch_range(merged);
    let path = merged.records[start..end]
        .iter()
        .rev()
        .find_map(|r| match &r.record {
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

/// Reports how the merged stream partitions into boot/firmware epochs and
/// which one recovery is scoped to. Silent for a single-epoch log (the
/// common case), so it adds noise only when partitioning actually
/// discarded another epoch's evidence.
fn report_epochs(w: &mut Report<'_>, merged: &RecoveryScan) {
    let selection = select_crash_epoch(merged);
    if !selection.partitioned() {
        return;
    }
    w.line(&format!(
        "epochs: {} found; recovering the newest printing epoch ({} older, {} newer discarded)",
        selection.epochs.len(),
        selection.discarded_older(),
        selection.discarded_newer(),
    ));
    for kind in selection.boundaries() {
        let desc = match kind {
            EpochBoundaryKind::HostReboot {
                last_mono_ns,
                next_mono_ns,
            } => format!("reboot (mono {last_mono_ns} -> {next_mono_ns} ns)"),
            EpochBoundaryKind::FirmwareRestart {
                socket_lost_mono_ns,
            } => format!("firmware restart (SocketLost at mono {socket_lost_mono_ns} ns)"),
        };
        w.line(&format!("  epoch boundary: {desc}"));
    }
    if let Some(epoch) = selection.crash_epoch() {
        w.line(&format!(
            "  crash epoch: {} records, mono {} .. {} ns",
            epoch.len(),
            epoch.min_mono_ns,
            epoch.max_mono_ns,
        ));
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

/// Part 2: map the (cap-narrowed) offset window through a layer model
/// built from the anchor context + print file, and report which layer(s)
/// the stop can be in, with the slicer mark (Part 3) as an upper-bound
/// cross-check. Needs the print file; `file_bytes` is `None` when it was
/// unreadable (an offline scan on a machine without the gcode), in which
/// case attribution is honestly reported as unavailable.
fn report_layer_attribution(
    w: &mut Report<'_>,
    recovery: &RecoveryReconstruction,
    file_bytes: Option<&[u8]>,
) {
    let Some(window) = recovery.stop_set.file_window.as_ref() else {
        w.line("  layer attribution: no offset window to map");
        return;
    };
    let Some(bytes) = file_bytes else {
        w.line("  layer attribution: unavailable (print file not readable — no layer model)");
        return;
    };
    let file = recovery
        .timeline
        .contexts
        .iter()
        .rev()
        .find_map(|c| c.virtual_sdcard.as_ref().map(|v| v.file_path.clone()));
    let Some(file) = file else {
        w.line("  layer attribution: unavailable (no context names a print file)");
        return;
    };
    let Some(anchor) =
        crate::detect::anchor_context(&recovery.timeline.contexts, &file, Some(window.start))
    else {
        w.line("  layer attribution: unavailable (no anchor context at/before the window)");
        return;
    };
    let base_offset = anchor
        .virtual_sdcard
        .as_ref()
        .map_or(0, |v| v.file_position);
    let Ok(base_usize) = usize::try_from(base_offset) else {
        w.line("  layer attribution: unavailable (context offset overflow)");
        return;
    };
    if base_usize > bytes.len() {
        w.line("  layer attribution: unavailable (context offset beyond the file — wrong file?)");
        return;
    }
    let Ok(state) = anchor_state_from_context(&anchor.gcode) else {
        w.line("  layer attribution: unavailable (anchor context state invalid)");
        return;
    };
    let model = plr_analyzer::build_layer_model(
        state,
        &bytes[base_usize..],
        base_offset,
        &plr_analyzer::ModelConfig::default(),
    );
    // OffsetWindow.end is inclusive; layers_in_window takes an exclusive end.
    let wl = model.layers_in_window(window.start, Some(window.end.saturating_add(1)));
    // `Layer::index` is window-relative; it equals an absolute file layer
    // only when the model spans from file start. The slicer mark is
    // absolute, so the cross-check is valid only then (see
    // `pipeline::narrate_layer_attribution` for the full rationale).
    let absolute = base_offset == 0;
    if absolute {
        w.line(&format!(
            "  layer attribution: {} (layer model from byte {base_offset}: {} layers)",
            wl.describe(),
            model.layers.len()
        ));
    } else {
        w.line(&format!(
            "  layer attribution: stop spans {} geometric layer(s){} (layer model from byte \
             {base_offset}: {} layers; layer numbers window-relative to the anchor, not absolute)",
            wl.layers.len(),
            if wl.before_first {
                " plus the pre-first-layer preamble"
            } else {
                ""
            },
            model.layers.len()
        ));
    }
    match anchor.current_layer {
        None => w.line(
            "  slicer layer marks: unavailable (no SET_PRINT_STATS_INFO); geometry carries the answer",
        ),
        Some(mark) => {
            let of = anchor.total_layer.map_or_else(String::new, |t| format!(" of {t}"));
            if !absolute {
                w.line(&format!(
                    "  slicer reported current_layer={mark}{of} (absolute); no consistency check — \
                     the attribution above is window-relative to a mid-file anchor"
                ));
            } else if wl.mark_is_consistent(mark) {
                w.line(&format!(
                    "  slicer layer mark: current_layer={mark}{of} (upper bound; consistent cross-check)"
                ));
            } else {
                w.line(&format!(
                    "  NOTE slicer layer mark current_layer={mark}{of} is below every attributed \
                     layer (upper-bound violation; trusting geometry)"
                ));
            }
        }
    }
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
            // The forensic scan prints records; the append frontier is not
            // part of what these fixtures assert.
            print_time: None,
            mono_ns,
            virtual_sdcard: Some(VirtualSdState {
                file_path: file_path.to_owned(),
                file_position,
                file_size: None,
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
            print_state: None,
            current_layer: None,
            total_layer: None,
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

    /// Builds a readable two-layer print + a WAL that reconstructs to a
    /// RECOVERY whose anchor context sits at `ctx_pos`, carrying the given
    /// slicer mark. Returns the scan report.
    fn layer_attr_report(tag: &str, ctx_pos: u64, mark: Option<u32>) -> String {
        let dir = temp_dir(tag);
        let gcode = "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\nG1 X20 Y10 E1 F1800\n\
                     G1 Z0.4 F7200\nG1 X10 Y10 F9000\nG1 X20 Y10 E1\n";
        let gpath = dir.join("part.gcode");
        std::fs::write(&gpath, gcode).unwrap();
        let path_str = gpath.to_str().unwrap().to_owned();
        let mut ctx = context(5_000_000_000, ctx_pos, &path_str);
        ctx.current_layer = mark;
        ctx.total_layer = mark.map(|_| 120);
        let full = write_segment(
            &dir,
            1,
            &[
                WalRecord::Heartbeat(heartbeat(3, 5_000_000_000, 42.0)),
                WalRecord::Context(ctx),
                WalRecord::Marker(Marker {
                    mono_ns: 5_100_000_000,
                    kind: MarkerKind::Resubscribed,
                }),
            ],
        );
        // Tear the tail: a realistic power-loss shape → RECOVERY.
        std::fs::write(dir.join(segment_file_name(1)), &full[..full.len() - 5]).unwrap();
        scan_to_string(&dir)
    }

    /// Part 2 in the scan report: with the print file readable, a layer
    /// model is built and the offset window is attributed to layer(s).
    /// MAJOR-fix case: at a **mid-file** anchor the geometric layer numbers
    /// are window-relative, so the absolute slicer mark gets NO consistency
    /// verdict — it is reported verbatim. (This is the case whose absence of
    /// a mid-file test let the relative-vs-absolute bug through.)
    #[test]
    fn mid_file_recovery_reports_the_mark_verbatim_without_a_verdict() {
        let pos = "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\n".len() as u64; // start of the layer-0 deposit line, mid-file
        assert!(pos > 0);
        let report = layer_attr_report("layerattr-mid", pos, Some(99));
        assert!(report.contains("RECOVERY"), "{report}");
        assert!(
            !report.contains("layer attribution: unavailable"),
            "a readable file must build a model: {report}"
        );
        // Mid-file: window-relative attribution, mark reported verbatim.
        assert!(report.contains("window-relative"), "{report}");
        assert!(
            report.contains("current_layer=99") && report.contains("no consistency check"),
            "a mid-file anchor must NOT emit a consistency verdict: {report}"
        );
        assert!(
            !report.contains("consistent cross-check"),
            "no absolute cross-check is possible mid-file: {report}"
        );
    }

    /// The companion: when the anchor sits at **file start** (`base_offset
    /// == 0`) the window ordinals ARE absolute, so the upper-bound
    /// cross-check is valid and fires — here a large mark is trivially
    /// consistent.
    #[test]
    fn file_start_recovery_runs_the_absolute_mark_cross_check() {
        let report = layer_attr_report("layerattr-start", 0, Some(99));
        assert!(report.contains("RECOVERY"), "{report}");
        assert!(report.contains("layer model from byte 0"), "{report}");
        assert!(
            report.contains("current_layer=99") && report.contains("consistent cross-check"),
            "a file-start anchor must run the absolute cross-check: {report}"
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
