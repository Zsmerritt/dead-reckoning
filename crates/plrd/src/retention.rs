//! WAL retention: prune superseded old **sessions** so the directory
//! stops growing without bound (measured: 253 MB in two days on the
//! reference printer, because nothing ever deleted anything).
//!
//! # Two halves, this is the second
//!
//! The recording *rate* is bounded elsewhere (the idle-heartbeat throttle
//! merged in `fix/wal-retention`): dense while printing, quiet while idle.
//! That caps how fast one boot's segments grow. This module caps the
//! *accumulation across boots*: at daemon startup, before the WAL service
//! opens or creates any segment, it deletes whole old sessions until the
//! directory fits a configured byte cap (`Config::wal_retention_bytes`).
//! Together they bound total disk use.
//!
//! # Sessions, not prints — and never "did the print end?"
//!
//! Retention groups segments into **sessions** and prunes at
//! whole-session granularity, oldest first. It NEVER asks whether a print
//! finished (that question is `detect`'s, and it is hard); it only asks
//! "is this whole session old, superseded, and unpinned?". A print's early
//! segments are load-bearing evidence for reconstructing that print, so a
//! session is kept or dropped as one unit — never its early segments alone.
//!
//! A **session** here is one host-monotonic boot's worth of segments. A
//! reboot resets `CLOCK_MONOTONIC` to ~0 and a fresh `plrd` process opens
//! a new segment, so the segment's [`SegmentHeader::created_mono_ns`]
//! regresses across a reboot. That is the *same* reboot delimiter
//! [`plr_reconstruct::epoch`] uses on the record stream — reused here at
//! segment-header granularity, with the identical threshold
//! [`plr_reconstruct::epoch::REBOOT_MONO_REGRESSION_NS`] and the identical
//! reasoning (within one boot `created_mono_ns` only increases across
//! rotations; a genuine regression is the full previous uptime, orders of
//! magnitude above the floor). The firmware-restart delimiter that module
//! also has (a `SocketLost` marker) is deliberately NOT used here: a
//! firmware restart neither opens a new segment nor resets the header
//! clock, so it cannot separate two *files*, which is the only granularity
//! deletion operates at.
//!
//! # Safety, by construction
//!
//! [`plan_pruning`] is the whole decision, and it is pure and total. It
//! never returns a segment belonging to:
//!
//! * **(a) the newest session** — it may be the live print, and the WAL
//!   service is about to resume recording into this directory;
//! * **(b) a pinned session** — see [`resolve_pins`]: anything backing
//!   `pending_recovery.json` / `frame_invalid.json`, from the pinned
//!   session onward;
//! * **(c) part of a session** — deletions are always whole sessions, so a
//!   partially-kept print is impossible to express;
//! * **(d) the highest-numbered segment** — it lives in the newest session
//!   (segment indices increase monotonically), which (a) already keeps.
//!   The WAL service creates `max + 1` on start (`walsvc::next_segment_index`),
//!   so deleting the current max would let a fresh segment reuse a live
//!   index; (a) forecloses that.
//!
//! # Pins beat the cap, loudly
//!
//! When pinned and/or newest sessions hold usage above the cap, the plan
//! deletes everything it safely can and reports the residual
//! [`Overage`]. The caller ([`run_pruning`], Linux) then surfaces it both
//! as a log line and — for a pin-driven overage — a console-visible
//! Moonraker message naming the pinned print. Silent unbounded growth and
//! silent deletion are both wrong; of the two, silent deletion is worse,
//! so evidence is kept and the operator is told.

use std::path::Path;

use plr_reconstruct::epoch::REBOOT_MONO_REGRESSION_NS;
use plr_wal::frame::SEGMENT_HEADER_LEN;
use plr_wal::{SegmentHeader, WalRecord};

use crate::scan::{list_segments, segment_file_name};

/// One WAL segment file, reduced to what retention decides on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SegmentMeta {
    /// Segment index (the `NNNNNN` in `wal-NNNNNN.plr`).
    pub index: u64,
    /// File size in bytes.
    pub size_bytes: u64,
    /// Host-monotonic creation time from the 32-byte segment header, or
    /// `None` when the header could not be decoded. A segment other than
    /// the newest whose header is unreadable is genuine corruption of
    /// previously durable data (rotation fsyncs a segment header before it
    /// is succeeded); `None` makes retention isolate and *keep* it rather
    /// than guess which boot it belonged to.
    pub created_mono_ns: Option<u64>,
}

/// One session: a maximal run of consecutive segments sharing a
/// host-monotonic boot, oldest segment first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Session {
    /// Segment indices in the session, ascending.
    pub segments: Vec<u64>,
    /// Sum of the segments' file sizes, bytes.
    pub total_bytes: u64,
    /// `false` when the session contains a segment with an unreadable
    /// header ([`SegmentMeta::created_mono_ns`] is `None`): such a session
    /// is never a deletion candidate (fail toward keeping evidence).
    pub deletable: bool,
}

impl Session {
    fn new() -> Self {
        Self {
            segments: Vec::new(),
            total_bytes: 0,
            deletable: true,
        }
    }

    fn push(&mut self, seg: &SegmentMeta) {
        self.segments.push(seg.index);
        self.total_bytes += seg.size_bytes;
        // A single unreadable header taints the whole session's
        // deletability: we cannot vouch for what boot it belongs to.
        self.deletable &= seg.created_mono_ns.is_some();
    }
}

/// Resolved pins: which sessions retention must keep regardless of the cap.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Pins {
    /// Oldest session ordinal (index into the `sessions` slice) that must
    /// be retained. Everything from here to the newest is kept ("pin from
    /// that session onward"). `None` means nothing is pinned; `Some(0)`
    /// means keep *everything* — used when a pin exists but cannot be
    /// localized to a session, so the safe answer is to keep all evidence.
    pub keep_from: Option<usize>,
    /// Human description of what is pinned, for the loud overage message
    /// (e.g. the print file path). `None` exactly when `keep_from` is `None`.
    pub pinned: Option<String>,
}

/// A residual overage: after deleting everything it safely could, the
/// plan still leaves usage above the cap.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Overage {
    /// Bytes over the cap.
    pub over_by: u64,
    /// Total bytes retained.
    pub kept_bytes: u64,
    /// The configured cap.
    pub cap: u64,
    /// The pin responsible, when a pin held the floor; `None` when the
    /// newest session alone exceeds the cap (the documented best-effort
    /// case: a single print, or one boot's growth, larger than the cap).
    pub pinned: Option<String>,
}

/// The pruning decision: the ratified `Vec<segment_index_to_delete>`
/// ([`Self::delete`]) plus the accounting the loud-overage requirement
/// needs, which a bare `Vec` cannot carry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PruningPlan {
    /// Segment indices to unlink, oldest first. Whole sessions only.
    pub delete: Vec<u64>,
    /// Bytes retained after applying `delete`.
    pub kept_bytes: u64,
    /// The configured cap this plan targeted.
    pub cap: u64,
    /// `Some` when `kept_bytes > cap` after doing everything safe.
    pub overage: Option<Overage>,
}

/// Groups segments (which MUST be sorted ascending by index — as
/// [`crate::scan::list_segments`] returns them) into sessions, oldest
/// first.
///
/// A new session begins at the first segment, at any segment whose header
/// is unreadable or whose predecessor's was (an uncomparable clock cannot
/// be shown to be the same boot, so it is isolated), and at a reboot: a
/// header `created_mono_ns` that regressed from the previous segment's by
/// at least [`REBOOT_MONO_REGRESSION_NS`]. Total; never panics.
#[must_use]
pub fn group_into_sessions(segments: &[SegmentMeta]) -> Vec<Session> {
    let mut sessions: Vec<Session> = Vec::new();
    let mut prev_mono: Option<u64> = None;
    for (i, seg) in segments.iter().enumerate() {
        let boundary = if i == 0 {
            true
        } else {
            match (prev_mono, seg.created_mono_ns) {
                // A backwards jump of at least the floor is a reboot. This
                // is `cur + REBOOT_MONO_REGRESSION_NS <= prev` from
                // `plr_reconstruct::epoch::partition`, written
                // overflow-free.
                (Some(prev), Some(cur)) => prev.saturating_sub(cur) >= REBOOT_MONO_REGRESSION_NS,
                // Either header uncomparable: isolate at a boundary so a
                // corrupt segment never merges two boots into one session.
                _ => true,
            }
        };
        if boundary {
            sessions.push(Session::new());
        }
        // Safe: the first iteration always pushed.
        sessions
            .last_mut()
            .expect("a session was pushed before the first fold")
            .push(seg);
        prev_mono = seg.created_mono_ns;
    }
    sessions
}

/// Decides which segments to delete. Pure, total, cross-platform — the
/// whole retention policy lives here so it can be proven on synthetic
/// sessions and on the real capture without touching a disk.
///
/// Oldest-first, whole-session: it deletes the oldest *deletable* session
/// strictly below the keep floor, then the next, until usage is at or
/// under `cap` or there is nothing left it may touch. The keep floor is
/// the older of the newest session (always kept) and the oldest pinned
/// session, so pinned and newest sessions — and everything between a pin
/// and the newest — are never candidates. A non-deletable (corrupt)
/// session is skipped, never deleted, and does not stop older deletable
/// sessions from being reclaimed.
#[must_use]
pub fn plan_pruning(sessions: &[Session], pins: &Pins, cap: u64) -> PruningPlan {
    let total: u64 = sessions.iter().map(|s| s.total_bytes).sum();
    if sessions.is_empty() {
        return PruningPlan {
            delete: Vec::new(),
            kept_bytes: 0,
            cap,
            overage: None,
        };
    }
    let newest = sessions.len() - 1;
    // Sessions at `keep_floor..` are retained unconditionally. The newest
    // is always kept; a pin lowers the floor to the oldest pinned session.
    let keep_floor = pins.keep_from.map_or(newest, |k| k.min(newest));

    let mut running = total;
    let mut delete: Vec<u64> = Vec::new();
    for session in &sessions[..keep_floor] {
        if running <= cap {
            break;
        }
        if !session.deletable {
            // Corrupt/unreadable session: never delete it, but keep
            // scanning — a newer deletable old session may still be
            // reclaimed. (It stays in `running`, so it counts against the
            // cap it is holding up: honest, not hidden.)
            continue;
        }
        delete.extend_from_slice(&session.segments);
        running -= session.total_bytes;
    }

    let overage = (running > cap).then(|| Overage {
        over_by: running - cap,
        kept_bytes: running,
        cap,
        // A pin is responsible whenever one exists: it is what forbade
        // deleting from the floor onward. With no pin, the residue is the
        // newest session alone — the best-effort-during-a-print case.
        pinned: pins.pinned.clone(),
    });

    PruningPlan {
        delete,
        kept_bytes: running,
        cap,
        overage,
    }
}

/// Reads every segment's index, size, and header monotonic time from
/// `wal_dir`. Read-only and cross-platform (no durability syscalls), like
/// `crate::scan`. `Err` only for an unreadable directory; an individual
/// segment whose header will not decode yields `created_mono_ns: None`
/// rather than failing the whole read.
pub fn read_segment_metas(wal_dir: &Path) -> Result<Vec<SegmentMeta>, String> {
    let segments = list_segments(wal_dir)?;
    let mut metas = Vec::with_capacity(segments.len());
    for (index, path) in segments {
        let size_bytes = std::fs::metadata(&path).map_or(0, |m| m.len());
        let created_mono_ns = read_header_mono(&path);
        metas.push(SegmentMeta {
            index,
            size_bytes,
            created_mono_ns,
        });
    }
    Ok(metas)
}

/// Reads and decodes just the 32-byte header of a segment file; `None`
/// when the file is short, unreadable, or the header fails its CRC/magic.
fn read_header_mono(path: &Path) -> Option<u64> {
    use std::io::Read as _;
    let mut file = std::fs::File::open(path).ok()?;
    let mut buf = [0_u8; SEGMENT_HEADER_LEN];
    file.read_exact(&mut buf).ok()?;
    SegmentHeader::decode(&buf).ok().map(|h| h.created_mono_ns)
}

/// Resolves the pins from the state files in `wal_dir`.
///
/// # Which session a pin refers to
///
/// `pending_recovery.json` names the interrupted print file
/// (`detect::PendingRecovery::file`); `detect` derives that file from the
/// crash epoch — the newest *printing* epoch — via `scan::last_print_file`
/// / `plr_reconstruct::select_crash_epoch`. Retention resolves it back to
/// a session the same way: the **newest session that holds a `Context`
/// naming that exact file** (the printing-context test mirrors
/// `plr_reconstruct::epoch`'s `is_printing_context` — a context whose
/// `virtual_sdcard` names a non-empty file). That session and everything
/// newer is pinned.
///
/// `frame_invalid.json` is a safety interlock (a recovery aborted with
/// Klipper's Z frame unknown) and names **no print file at all**
/// (`detect::FrameInvalid` carries only step/phase/reason). When it
/// coexists with `pending_recovery.json` — the usual case, since the abort
/// path writes both — the session is taken from the pending file and the
/// interlock is flagged in the message. When it stands alone, there is
/// nothing to map it to, so the safe answer is to keep everything.
///
/// # Failing toward keeping evidence
///
/// Any inability to localize a pin resolves to `keep_from: Some(0)` (keep
/// all segments): a pending file whose print is in none of the sessions
/// (it may have already been pruned in an earlier run, or the file scrolled
/// out), or a lone frame-invalid interlock. Deleting evidence we cannot
/// prove is superseded is the one outcome worse than growth.
#[must_use]
pub fn resolve_pins(wal_dir: &Path, sessions: &[Session]) -> Pins {
    let pending = crate::detect::read_pending(wal_dir);
    let frame_invalid = crate::detect::read_frame_invalid(wal_dir).is_some();

    let Some(pending) = pending else {
        if frame_invalid {
            // Interlock active but nothing names the print: keep all.
            return Pins {
                keep_from: Some(0),
                pinned: Some(
                    "frame-invalid interlock (no pending file to localize; keeping all WAL)"
                        .to_owned(),
                ),
            };
        }
        return Pins::default();
    };

    match newest_session_naming(wal_dir, sessions, &pending.file) {
        Some(ordinal) => {
            let note = if frame_invalid || pending.frame_invalid {
                " [Z frame UNKNOWN — fresh dry run required]"
            } else {
                ""
            };
            Pins {
                keep_from: Some(ordinal),
                pinned: Some(format!("pending recovery for '{}'{note}", pending.file)),
            }
        }
        None => Pins {
            // The pinned print is in none of the current sessions: keep
            // everything rather than risk deleting a live offer's backing.
            keep_from: Some(0),
            pinned: Some(format!(
                "pending recovery for '{}' (session not found; keeping all WAL)",
                pending.file
            )),
        },
    }
}

/// Ordinal of the newest session holding a `Context` whose
/// `virtual_sdcard.file_path` equals `file`, scanning newest-first so the
/// first match is the newest. `None` when no session names it.
fn newest_session_naming(wal_dir: &Path, sessions: &[Session], file: &str) -> Option<usize> {
    for (ordinal, session) in sessions.iter().enumerate().rev() {
        if session_names_file(wal_dir, session, file) {
            return Some(ordinal);
        }
    }
    None
}

/// Does any segment in `session` record a context naming `file`?
fn session_names_file(wal_dir: &Path, session: &Session, file: &str) -> bool {
    session.segments.iter().any(|&index| {
        let path = wal_dir.join(segment_file_name(index));
        let Ok(scan) = crate::scan::scan_segment(&path) else {
            return false;
        };
        scan.records.iter().any(|r| match &r.record {
            WalRecord::Context(c) => c
                .virtual_sdcard
                .as_ref()
                .is_some_and(|v| v.file_path == file),
            _ => false,
        })
    })
}

/// The primary/fallback G-Code commands for a console-visible overage
/// notice, mirroring `detect::announcement_commands`. Quote characters in
/// the pin description are stripped so the message is safe inside a
/// `RESPOND ... MSG="..."` argument.
#[cfg(target_os = "linux")]
fn overage_commands(overage: &Overage) -> Option<(String, String)> {
    let pinned = overage.pinned.as_ref()?;
    let safe = pinned.replace(['"', '\''], "");
    let message = format!(
        "dead-reckoning: WAL retention cannot meet its {} B cap — {} B over, held by {}. \
         Evidence is being KEPT, not deleted; free space or clear the pin.",
        overage.cap, overage.over_by, safe,
    );
    Some((
        format!("RESPOND PREFIX=dead-reckoning MSG=\"{message}\""),
        format!("M117 {message}"),
    ))
}

/// Runs retention against the configured WAL directory: read segment
/// headers, group into sessions, resolve pins, plan, and apply
/// (unlink + directory fsync). Returns the console-message commands for a
/// pin-driven overage, if any, for the caller to deliver via Moonraker.
///
/// **Linux only, and it must run before the WAL service spawns** so the
/// highest-numbered segment `walsvc` is about to create `max + 1` past is
/// still the current max — and thus in the newest session, which
/// [`plan_pruning`] never deletes. Never fatal: any failure is logged and
/// recording proceeds. Read-only classification syscalls plus `unlink` and
/// one directory `fsync`.
#[cfg(target_os = "linux")]
#[must_use]
pub fn run_pruning(config: &crate::config::Config) -> Option<(String, String)> {
    let wal_dir = config.wal_dir.as_path();
    let cap = config.wal_retention_bytes;

    let metas = match read_segment_metas(wal_dir) {
        Ok(metas) => metas,
        Err(e) => {
            // An unreadable/absent WAL dir is normal on a first-ever boot;
            // it is not retention's job to create it (walsvc does). Log at
            // most a quiet note and move on.
            eprintln!("plrd: WAL retention: skipping ({e})");
            return None;
        }
    };
    if metas.is_empty() {
        return None;
    }

    let sessions = group_into_sessions(&metas);
    let pins = resolve_pins(wal_dir, &sessions);
    let plan = plan_pruning(&sessions, &pins, cap);

    if !plan.delete.is_empty() {
        match apply_pruning(wal_dir, &plan.delete) {
            Ok(removed) => {
                let reclaimed: u64 = metas
                    .iter()
                    .filter(|m| plan.delete.contains(&m.index))
                    .map(|m| m.size_bytes)
                    .sum();
                eprintln!(
                    "plrd: WAL retention pruned {removed} old segment(s) ({reclaimed} B \
                     reclaimed); {} B retained against the {cap} B cap",
                    plan.kept_bytes,
                );
            }
            Err(e) => {
                eprintln!("plrd: WAL retention: some deletions failed: {e}");
            }
        }
    }

    if let Some(overage) = &plan.overage {
        // Always the log line...
        eprintln!(
            "plrd: WAL retention: {} B retained exceeds the {} B cap by {} B{}; \
             keeping all evidence (a pin or the current print beats the cap)",
            overage.kept_bytes,
            overage.cap,
            overage.over_by,
            overage
                .pinned
                .as_ref()
                .map_or(String::new(), |p| format!(", held by {p}")),
        );
        // ...and, when a pin holds it, the console-visible message.
        return overage_commands(overage);
    }
    None
}

/// Unlinks the chosen segments and fsyncs the directory so the removals
/// are durable. Attempts every deletion even if one fails; a missing file
/// (already gone) is not an error. Returns the number removed.
#[cfg(target_os = "linux")]
fn apply_pruning(wal_dir: &Path, delete: &[u64]) -> std::io::Result<usize> {
    let mut removed = 0_usize;
    let mut first_error: Option<std::io::Error> = None;
    for &index in delete {
        let path = wal_dir.join(segment_file_name(index));
        match std::fs::remove_file(&path) {
            Ok(()) => removed += 1,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                if first_error.is_none() {
                    first_error = Some(e);
                }
            }
        }
    }
    // fsync the directory so the unlinks survive a power cut, exactly as
    // `detect::sync_dir` does for the frame-invalid rename.
    std::fs::File::open(wal_dir)?.sync_all()?;
    match first_error {
        Some(e) => Err(e),
        None => Ok(removed),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        group_into_sessions, plan_pruning, read_segment_metas, resolve_pins, Pins, SegmentMeta,
        Session,
    };
    use crate::scan::segment_file_name;
    use plr_reconstruct::epoch::REBOOT_MONO_REGRESSION_NS;
    use plr_wal::{
        Context, GcodeState, SegmentHeader, TransformObservations, VirtualSdState, WalRecord,
        WalWriter,
    };
    use std::path::{Path, PathBuf};

    const S: u64 = 1_000_000_000; // one second in ns

    fn temp_dir(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-retention-test-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A segment with a readable header at `mono` and a given byte size.
    fn seg(index: u64, size_bytes: u64, mono: u64) -> SegmentMeta {
        SegmentMeta {
            index,
            size_bytes,
            created_mono_ns: Some(mono),
        }
    }

    /// A single-boot session holding one synthetic segment of `bytes`.
    fn one_seg_session(index: u64, bytes: u64) -> Session {
        Session {
            segments: vec![index],
            total_bytes: bytes,
            deletable: true,
        }
    }

    // ---- group_into_sessions ------------------------------------------

    #[test]
    fn empty_segments_yield_no_sessions() {
        assert!(group_into_sessions(&[]).is_empty());
    }

    #[test]
    fn one_boot_is_one_session() {
        // Rotations within a boot: mono only increases.
        let s =
            group_into_sessions(&[seg(1, 16, 100 * S), seg(2, 16, 200 * S), seg(3, 4, 300 * S)]);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].segments, vec![1, 2, 3]);
        assert_eq!(s[0].total_bytes, 36);
        assert!(s[0].deletable);
    }

    #[test]
    fn a_reboot_splits_sessions() {
        // seg 3's header mono regressed far below seg 2's => new boot.
        let s = group_into_sessions(&[
            seg(1, 16, 50_000 * S),
            seg(2, 16, 50_100 * S),
            seg(3, 16, 20 * S),
            seg(4, 16, 40 * S),
        ]);
        assert_eq!(s.len(), 2);
        assert_eq!(s[0].segments, vec![1, 2]);
        assert_eq!(s[1].segments, vec![3, 4]);
    }

    #[test]
    fn sub_threshold_mono_jitter_stays_one_session() {
        // A same-boot rotation whose clock ticked backwards by less than
        // the floor (jitter) must NOT split — mirrors epoch's threshold.
        let below = group_into_sessions(&[
            seg(1, 16, 100 * S),
            seg(2, 16, 100 * S - (REBOOT_MONO_REGRESSION_NS - 1)),
        ]);
        assert_eq!(below.len(), 1);
        // Exactly the floor splits.
        let at = group_into_sessions(&[
            seg(1, 16, 100 * S),
            seg(2, 16, 100 * S - REBOOT_MONO_REGRESSION_NS),
        ]);
        assert_eq!(at.len(), 2);
    }

    #[test]
    fn an_unreadable_header_isolates_and_is_kept() {
        let s = group_into_sessions(&[
            seg(1, 16, 100 * S),
            SegmentMeta {
                index: 2,
                size_bytes: 16,
                created_mono_ns: None,
            },
            seg(3, 16, 300 * S),
        ]);
        // The corrupt segment neither merges with its neighbours nor lets
        // them merge across it: three sessions, the middle non-deletable.
        assert_eq!(s.len(), 3);
        assert_eq!(s[1].segments, vec![2]);
        assert!(!s[1].deletable);
        assert!(s[0].deletable && s[2].deletable);
    }

    // ---- plan_pruning: the four safety invariants ---------------------

    #[test]
    fn under_cap_deletes_nothing() {
        let sessions = vec![one_seg_session(1, 10), one_seg_session(2, 10)];
        let plan = plan_pruning(&sessions, &Pins::default(), 1000);
        assert!(plan.delete.is_empty());
        assert_eq!(plan.kept_bytes, 20);
        assert!(plan.overage.is_none());
    }

    #[test]
    fn over_cap_prunes_oldest_first_and_keeps_the_newest_session() {
        // Four boots of 100 B each = 400 B; cap 250 B. Oldest-first,
        // deleting sessions 0 and 1 leaves 200 B <= 250 B; stop.
        let sessions = vec![
            one_seg_session(1, 100),
            one_seg_session(2, 100),
            one_seg_session(3, 100),
            one_seg_session(4, 100),
        ];
        let plan = plan_pruning(&sessions, &Pins::default(), 250);
        assert_eq!(plan.delete, vec![1, 2]); // (a) session 4 (newest) untouched
        assert_eq!(plan.kept_bytes, 200);
        assert!(plan.overage.is_none());
        // (d) the highest-numbered segment (4) is never deleted.
        assert!(!plan.delete.contains(&4));
    }

    #[test]
    fn deletion_is_whole_session_never_partial() {
        // (c) a multi-segment old session is deleted as a unit or not at all.
        let sessions = vec![
            Session {
                segments: vec![1, 2, 3],
                total_bytes: 300,
                deletable: true,
            },
            one_seg_session(4, 50),
        ];
        let plan = plan_pruning(&sessions, &Pins::default(), 100);
        // Over cap: the only deletable-below-newest session is #0; all three
        // of its segments go, none held back.
        assert_eq!(plan.delete, vec![1, 2, 3]);
    }

    #[test]
    fn the_newest_session_is_never_deleted_even_when_it_alone_exceeds_cap() {
        // (a)+(d): one giant newest session, cap far below it.
        let sessions = vec![Session {
            segments: vec![1, 2],
            total_bytes: 10_000,
            deletable: true,
        }];
        let plan = plan_pruning(&sessions, &Pins::default(), 100);
        assert!(plan.delete.is_empty());
        let overage = plan.overage.expect("newest over cap must report overage");
        assert_eq!(overage.over_by, 9_900);
        assert!(overage.pinned.is_none(), "no pin: best-effort case");
    }

    // ---- plan_pruning: pins beat the cap ------------------------------

    #[test]
    fn a_pinned_session_is_never_deleted_and_pins_from_it_onward() {
        // (b): sessions 0..4, pin at ordinal 2. Even wildly over cap, only
        // sessions strictly older than the pin may be deleted.
        let sessions = vec![
            one_seg_session(1, 100),
            one_seg_session(2, 100),
            one_seg_session(3, 100), // pinned
            one_seg_session(4, 100),
            one_seg_session(5, 100),
        ];
        let pins = Pins {
            keep_from: Some(2),
            pinned: Some("pending recovery for '/g/part.gcode'".to_owned()),
        };
        let plan = plan_pruning(&sessions, &pins, 100);
        // Sessions 0 and 1 deletable; 2 (pin), 3, 4 kept "onward".
        assert_eq!(plan.delete, vec![1, 2]);
        assert!(!plan.delete.contains(&3)); // the pinned session survives
                                            // 300 B retained (sessions 2,3,4) exceeds the 100 B cap: loud overage
                                            // naming the pin.
        let overage = plan.overage.expect("pin holds usage over cap");
        assert_eq!(overage.kept_bytes, 300);
        assert_eq!(overage.over_by, 200);
        assert_eq!(
            overage.pinned.as_deref(),
            Some("pending recovery for '/g/part.gcode'")
        );
    }

    #[test]
    fn keep_from_zero_keeps_everything() {
        // The "cannot localize a pin" answer: keep_from = Some(0).
        let sessions = vec![
            one_seg_session(1, 100),
            one_seg_session(2, 100),
            one_seg_session(3, 100),
        ];
        let pins = Pins {
            keep_from: Some(0),
            pinned: Some("frame-invalid interlock (keeping all WAL)".to_owned()),
        };
        let plan = plan_pruning(&sessions, &pins, 50);
        assert!(plan.delete.is_empty());
        assert_eq!(plan.overage.unwrap().over_by, 250);
    }

    #[test]
    fn a_corrupt_session_below_the_floor_is_skipped_not_deleted() {
        // Session 0 corrupt (non-deletable), session 1 an old deletable
        // boot, session 2 newest. Over cap: 0 is kept (corrupt), 1 is
        // reclaimed even though it is newer than 0.
        let sessions = vec![
            Session {
                segments: vec![1],
                total_bytes: 100,
                deletable: false,
            },
            one_seg_session(2, 100),
            one_seg_session(3, 100),
        ];
        let plan = plan_pruning(&sessions, &Pins::default(), 150);
        assert_eq!(plan.delete, vec![2]); // corrupt #1 skipped, not deleted
        assert!(!plan.delete.contains(&1));
        // 200 B kept still over 150 cap (corrupt + newest can't be freed).
        assert_eq!(plan.overage.unwrap().over_by, 50);
    }

    #[test]
    fn empty_sessions_plan_is_a_noop() {
        let plan = plan_pruning(&[], &Pins::default(), 0);
        assert!(plan.delete.is_empty());
        assert!(plan.overage.is_none());
    }

    // ---- read_segment_metas + on-disk round trip ----------------------

    fn write_segment(dir: &Path, index: u64, mono_ns: u64, records: &[WalRecord]) {
        let mut writer = WalWriter::create(
            Vec::new(),
            &SegmentHeader::new(1_700_000_000_000_000_000, mono_ns),
        )
        .unwrap();
        for r in records {
            writer.append(r).unwrap();
        }
        std::fs::write(dir.join(segment_file_name(index)), writer.into_inner()).unwrap();
    }

    fn printing_context(file: &str) -> WalRecord {
        WalRecord::Context(Context {
            print_time: Some(5.0),
            mono_ns: 5 * S,
            virtual_sdcard: Some(VirtualSdState {
                file_path: file.to_owned(),
                file_position: 100,
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
        })
    }

    #[test]
    fn read_metas_recovers_index_size_and_mono() {
        let dir = temp_dir("metas");
        write_segment(&dir, 1, 100 * S, &[printing_context("/g/a.gcode")]);
        write_segment(&dir, 2, 200 * S, &[printing_context("/g/a.gcode")]);
        let metas = read_segment_metas(&dir).unwrap();
        assert_eq!(metas.len(), 2);
        assert_eq!(metas[0].index, 1);
        assert_eq!(metas[0].created_mono_ns, Some(100 * S));
        assert_eq!(metas[1].created_mono_ns, Some(200 * S));
        assert!(metas[0].size_bytes >= 32);
    }

    #[test]
    fn a_garbage_header_reads_as_none_mono() {
        let dir = temp_dir("badhdr");
        std::fs::write(dir.join(segment_file_name(1)), [0_u8; 32]).unwrap();
        let metas = read_segment_metas(&dir).unwrap();
        assert_eq!(metas.len(), 1);
        assert_eq!(metas[0].created_mono_ns, None);
    }

    // ---- resolve_pins -------------------------------------------------

    #[test]
    fn no_state_files_means_no_pins() {
        let dir = temp_dir("nopins");
        write_segment(&dir, 1, 100 * S, &[printing_context("/g/a.gcode")]);
        let metas = read_segment_metas(&dir).unwrap();
        let sessions = group_into_sessions(&metas);
        assert_eq!(resolve_pins(&dir, &sessions), Pins::default());
    }

    #[test]
    fn pending_resolves_to_the_newest_session_naming_the_file() {
        let dir = temp_dir("pin-hit");
        // Boot 0 printed a.gcode; boot 1 (reboot: mono resets) printed b.gcode.
        write_segment(&dir, 1, 50_000 * S, &[printing_context("/g/a.gcode")]);
        write_segment(&dir, 2, 20 * S, &[printing_context("/g/b.gcode")]);
        let metas = read_segment_metas(&dir).unwrap();
        let sessions = group_into_sessions(&metas);
        assert_eq!(sessions.len(), 2);
        // A pending file for a.gcode pins from session 0 onward.
        let pending = crate::detect::PendingRecovery {
            detected_wall_ns: 1,
            file: "/g/a.gcode".to_owned(),
            file_position: 100,
            file_size: None,
            percent: None,
            crash_class: "x".to_owned(),
            frame_invalid: false,
        };
        crate::detect::write_pending(&dir, &pending).unwrap();
        let pins = resolve_pins(&dir, &sessions);
        assert_eq!(pins.keep_from, Some(0));
        assert!(pins.pinned.unwrap().contains("a.gcode"));
    }

    #[test]
    fn pending_for_an_absent_file_keeps_everything() {
        let dir = temp_dir("pin-miss");
        write_segment(&dir, 1, 100 * S, &[printing_context("/g/a.gcode")]);
        let metas = read_segment_metas(&dir).unwrap();
        let sessions = group_into_sessions(&metas);
        let pending = crate::detect::PendingRecovery {
            detected_wall_ns: 1,
            file: "/g/GONE.gcode".to_owned(),
            file_position: 0,
            file_size: None,
            percent: None,
            crash_class: "x".to_owned(),
            frame_invalid: false,
        };
        crate::detect::write_pending(&dir, &pending).unwrap();
        let pins = resolve_pins(&dir, &sessions);
        assert_eq!(pins.keep_from, Some(0)); // keep all
        assert!(pins.pinned.unwrap().contains("session not found"));
    }

    #[test]
    fn a_lone_frame_invalid_interlock_keeps_everything() {
        let dir = temp_dir("frame-only");
        write_segment(&dir, 1, 100 * S, &[printing_context("/g/a.gcode")]);
        let metas = read_segment_metas(&dir).unwrap();
        let sessions = group_into_sessions(&metas);
        crate::detect::write_frame_invalid(
            &dir,
            &crate::detect::FrameInvalid {
                detected_wall_ns: 1,
                step_id: 7,
                phase: "declare_frame".to_owned(),
                reason: "abort".to_owned(),
            },
        )
        .unwrap();
        let pins = resolve_pins(&dir, &sessions);
        assert_eq!(pins.keep_from, Some(0));
        assert!(pins.pinned.unwrap().contains("frame-invalid"));
    }

    #[test]
    fn frame_invalid_with_pending_resolves_the_session_and_flags_it() {
        let dir = temp_dir("frame-pending");
        write_segment(&dir, 1, 100 * S, &[printing_context("/g/a.gcode")]);
        let metas = read_segment_metas(&dir).unwrap();
        let sessions = group_into_sessions(&metas);
        crate::detect::write_pending(
            &dir,
            &crate::detect::PendingRecovery {
                detected_wall_ns: 1,
                file: "/g/a.gcode".to_owned(),
                file_position: 100,
                file_size: None,
                percent: None,
                crash_class: "x".to_owned(),
                frame_invalid: true,
            },
        )
        .unwrap();
        crate::detect::write_frame_invalid(
            &dir,
            &crate::detect::FrameInvalid {
                detected_wall_ns: 1,
                step_id: 7,
                phase: "declare_frame".to_owned(),
                reason: "abort".to_owned(),
            },
        )
        .unwrap();
        let pins = resolve_pins(&dir, &sessions);
        assert_eq!(pins.keep_from, Some(0));
        assert!(pins.pinned.unwrap().contains("Z frame UNKNOWN"));
    }

    // ---- end to end on a synthetic three-boot directory ---------------

    // ---- the real capture --------------------------------------------

    /// Demonstration against the real 253 MB power-loss capture. `#[ignore]`
    /// because it needs the capture, which is far too large to commit: point
    /// `PLRD_REAL_WAL` at a WRITABLE COPY of it (never the read-only
    /// original — this test writes a synthetic `pending_recovery.json` into
    /// the dir for the pin half) and run:
    ///
    /// ```text
    /// PLRD_REAL_WAL=/path/to/copy cargo test -p plrd --bin plrd \
    ///     retention::tests::real_capture -- --ignored --nocapture
    /// ```
    ///
    /// It prints the before/after inventory and asserts the safety
    /// invariants hold on real data.
    #[test]
    #[ignore = "needs a copy of the real WAL capture via PLRD_REAL_WAL"]
    fn real_capture_prunes_old_keeps_newest_and_honours_pins() {
        let Ok(dir) = std::env::var("PLRD_REAL_WAL") else {
            eprintln!("PLRD_REAL_WAL unset; skipping real-capture demonstration");
            return;
        };
        let dir = PathBuf::from(dir);
        let metas = read_segment_metas(&dir).expect("read real capture");
        let total: u64 = metas.iter().map(|m| m.size_bytes).sum();
        let sessions = group_into_sessions(&metas);

        eprintln!("=== real capture inventory ===");
        eprintln!("segments: {}  total: {total} B", metas.len());
        for (i, s) in sessions.iter().enumerate() {
            eprintln!(
                "  session {i}: segments {:?}  {} B  deletable={}",
                s.segments, s.total_bytes, s.deletable
            );
        }

        // --- no pins, 100 MB cap: oldest pruned, newest kept ---
        let cap = 100 * 1024 * 1024;
        let pins = resolve_pins(&dir, &sessions);
        let plan = plan_pruning(&sessions, &pins, cap);
        eprintln!("--- no-pin plan @ {cap} B cap ---");
        eprintln!("delete: {:?}  kept: {} B", plan.delete, plan.kept_bytes);
        eprintln!("overage: {:?}", plan.overage);

        let newest = sessions.last().expect("at least one session");
        let max_index = metas.iter().map(|m| m.index).max().unwrap();
        for &d in &plan.delete {
            assert!(
                !newest.segments.contains(&d),
                "a newest-session segment was scheduled for deletion"
            );
            assert_ne!(d, max_index, "the highest-numbered segment must be kept");
        }
        // Whole-session granularity: every deleted index belongs to a fully
        // deleted session.
        for s in &sessions {
            let deleted: Vec<u64> = s
                .segments
                .iter()
                .copied()
                .filter(|i| plan.delete.contains(i))
                .collect();
            assert!(
                deleted.is_empty() || deleted == s.segments,
                "session {:?} was partially deleted: {deleted:?}",
                s.segments
            );
        }

        // --- with a pin: keep from the pinned session onward, report overage ---
        // Find a real print file the capture names, and pin it.
        let printed = metas.iter().find_map(|m| {
            let path = dir.join(segment_file_name(m.index));
            let scan = crate::scan::scan_segment(&path).ok()?;
            scan.records.iter().find_map(|r| match &r.record {
                WalRecord::Context(c) => c
                    .virtual_sdcard
                    .as_ref()
                    .filter(|v| !v.file_path.is_empty())
                    .map(|v| v.file_path.clone()),
                _ => None,
            })
        });
        if let Some(file) = printed {
            crate::detect::write_pending(
                &dir,
                &crate::detect::PendingRecovery {
                    detected_wall_ns: 1,
                    file: file.clone(),
                    file_position: 0,
                    file_size: None,
                    percent: None,
                    crash_class: "demo".to_owned(),
                    frame_invalid: false,
                },
            )
            .unwrap();
            let pins = resolve_pins(&dir, &sessions);
            let plan = plan_pruning(&sessions, &pins, cap);
            eprintln!("--- pinned '{file}' plan @ {cap} B cap ---");
            eprintln!(
                "keep_from: {:?}  delete: {:?}  kept: {} B",
                pins.keep_from, plan.delete, plan.kept_bytes
            );
            eprintln!("overage: {:?}", plan.overage);
            let keep_from = pins.keep_from.expect("a pin was written");
            // Nothing from the pinned session onward may be deleted.
            for s in &sessions[keep_from..] {
                for i in &s.segments {
                    assert!(!plan.delete.contains(i), "pinned/onward segment deleted");
                }
            }
            crate::detect::clear_pending(&dir);
        }
    }

    #[test]
    fn end_to_end_prunes_oldest_boots_and_keeps_newest() {
        let dir = temp_dir("e2e");
        // Three boots, reboots between them (mono resets). No pins.
        write_segment(&dir, 1, 90_000 * S, &[printing_context("/g/old.gcode")]);
        write_segment(&dir, 2, 60_000 * S, &[printing_context("/g/mid.gcode")]);
        write_segment(&dir, 3, 20 * S, &[printing_context("/g/new.gcode")]);
        let metas = read_segment_metas(&dir).unwrap();
        let sessions = group_into_sessions(&metas);
        assert_eq!(sessions.len(), 3, "three boots");
        let one = metas[0].size_bytes; // each segment ~ same size
                                       // Cap that holds ~two segments: the oldest boot must be pruned.
        let pins = resolve_pins(&dir, &sessions);
        assert_eq!(pins, Pins::default());
        let plan = plan_pruning(&sessions, &pins, one * 2 + 10);
        assert_eq!(plan.delete, vec![1], "only the oldest boot is pruned");
        assert!(!plan.delete.contains(&3), "newest boot kept");
    }
}
