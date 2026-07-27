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
//!
//! # The completion gate
//!
//! An unclean WAL tail plus a named print file is **not** evidence that
//! anything needs recovering. Three things routinely produce exactly that
//! shape with nothing to recover, and each is handled here:
//!
//! * **the print finished.** Every slicer appends a footer — end g-code
//!   plus a ~12–14 KB serialized profile as comments — so a complete
//!   print's last depositing line sits far short of EOF. No percentage of
//!   the file size can separate that from "died on the last layer", so
//!   [`detect`] replays the remainder of the file and asks whether any of
//!   it deposits plastic ([`plr_analyzer::remaining_work`]). It does not
//!   compare offsets to a threshold, because there is no threshold that
//!   works. Result: [`Detection::Complete`].
//! * **the print was cancelled** in a way that journaled no
//!   `CleanShutdown` marker. That is fixed at the source in `convert.rs`
//!   ("Clean-shutdown detection"); the gate is the backstop.
//! * **the recorder stopped, not the print.** `plrd.service` is
//!   `Restart=always`, so a restart mid-print is routine: the graceful
//!   shutdown journals no marker and boot detection runs before the
//!   Klipper client connects, so the still-running print looks dead. Two
//!   independent defences: the daemon journals a
//!   [`plr_wal::MarkerKind::RecorderStopped`] marker on graceful shutdown
//!   (surfaced by reconstruction as
//!   [`plr_reconstruct::WalTimeline::recorder_stopped_tail`]), and the
//!   announcement is deferred until the Klipper client's first status can
//!   confirm the print is not simply still running
//!   (`daemon::announce_pending`). They are independent on purpose: the
//!   marker cannot be written if the daemon is `SIGKILL`ed, and the
//!   pre-check cannot run while Moonraker is down.
//! * **the print ended on purpose but the marker was lost.** The newest
//!   context's [`plr_wal::Context::print_state`] is checked directly, so a
//!   `complete`/`cancelled`/`standby` print is recognized from journaled
//!   evidence even if no `CleanShutdown` marker survived.
//!
//! ## The suppression asymmetry
//!
//! The gate may only ever suppress an announcement on **positive proof**
//! of completion. Every way of not knowing — an unreadable print file, a
//! tail past [`MAX_GATE_TAIL_BYTES`], a replay that failed, an anchor
//! context whose interpreter state is invalid, no offset window at all —
//! announces. A false offer costs the operator a dry run; a suppressed
//! offer costs them the print.
//!
//! ## Which offset is tested, and why
//!
//! `stop_set.file_window.start` — the **low** end of the possible-stop
//! window. Not the newest context's `file_position`: that is the
//! *processing* frontier, up to 3 s of queued motion ahead of what the
//! nozzle actually executed, so it overstates progress and could hide
//! real work. Not `resume.offset`/`file_window.end` either: those are
//! deliberately the maximum of an ambiguous range, chosen so a resume
//! never re-prints, which is the wrong direction for a test that decides
//! whether to stay silent.

use std::io::{Read as _, Seek as _, Write as _};
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

/// Sentinel appended to [`PendingRecovery::crash_class`] when the
/// frame-invalidation marker is present.
///
/// **Display only, and only for the Klipper-side wizard**
/// (`klippy_plugin/plr/wizard.py`), which renders `crash_class` verbatim
/// and cannot see [`PendingRecovery::frame_invalid`]. Every consumer in
/// this crate branches on that boolean instead; nothing here parses this
/// string.
///
/// Future item (not a scope artifact): teach the wizard to read
/// `frame_invalid` from the pending file and render its own wording, then
/// this sentinel can go. That is a `klippy_plugin` change with its own
/// python gates, and it is not blocked on anything here.
pub const FRAME_INVALID_NOTE: &str = "Z FRAME UNKNOWN";

/// How many newest segments detection reads. Bounds the startup cost
/// regardless of how many segments have accumulated; classification
/// only needs the WAL tail (see `scan::load_merged_tail`).
const DETECT_SEGMENT_LIMIT: usize = 3;

/// Hard cap on how many bytes of the print file the completion gate will
/// read. 64 MiB covers any real print file (the largest sliced files in
/// the wild are tens of MB); beyond it the gate refuses to answer and the
/// recovery is announced, because a boot-time read must not stall the
/// recorder and an unbounded read of an attacker-chosen path is not a
/// thing a daemon should do.
pub const MAX_GATE_TAIL_BYTES: u64 = 64 * 1024 * 1024;

/// The smallest remainder the completion gate will draw a conclusion from.
///
/// # Derivation
///
/// A finished print's remainder is its slicer footer, and the footer's *end
/// sequence* — the part that is commands rather than the trailing config
/// block — is the smallest thing that can legitimately be left. Measured
/// across the five footer fixtures (marker to config-block marker):
///
/// | fixture                       | end sequence |
/// |-------------------------------|--------------|
/// | `prusa_real_footer` (2.9.3)   | 1,239 B      |
/// | `orca_real_footer` (2.3.1)    |   689 B      |
/// | `cura_footer_complete`        |   286 B      |
/// | `prusa_footer_complete`       |   261 B      |
/// | `orca_footer_complete`        |   178 B      |
///
/// 128 B sits below every one of those, so no footer the corpus considers
/// plausible is ever refused, while the "a few dozen bytes" remainder a
/// truncated file leaves is.
///
/// # Why a threshold at all, rather than `> 0`
///
/// One byte of remainder is exactly as much evidence as zero. The hazard is
/// a 20 MB print that died at 40 % whose *file* then lost its tail to an
/// unclean unmount: the WAL is intact, its floor context sits at ~8 MB, and
/// the file is now 8 MB plus a few dozen bytes. With a `> 0` test the gate
/// examines those bytes, finds no extrusion, reports the print COMPLETE and
/// clears the pending offer the wizard reads — twelve megabytes never
/// offered.
///
/// The direction is safe: refusing to suppress only offers a recovery the
/// operator can decline, so a conservative floor costs a dry run at worst.
/// The cost is bounded and measurable — a print that genuinely died in the
/// last 128 bytes of its footer announces — which is under 1 % of every
/// footer above.
pub const MIN_TRUSTWORTHY_REMAINDER_BYTES: u64 = 128;

/// What boot-time detection found.
#[derive(Debug, Clone, PartialEq)]
pub enum Detection {
    /// Unclean stop with unfinished printing work: recovery is available.
    Pending(PendingRecovery),
    /// Unclean stop, but the print had no printing work left: it
    /// finished. No recovery, and any stale pending file is cleared.
    Complete(Completion),
    /// The WAL ends cleanly; any stale pending file should be cleared.
    ///
    /// Deliberately says nothing about `frame_invalid.json`. A clean print
    /// end is *not* evidence that a fabricated Z frame was superseded: the
    /// tempting argument — "the print must have homed, because Klipper
    /// refuses motion on unhomed axes" — is false precisely when it matters,
    /// because `SET_KINEMATIC_POSITION` (which
    /// `crate::executor` issues to declare the shifted frame) marks axes
    /// homed by default (`klippy/extras/force_move.py`,
    /// `set_homed = gcmd.get('SET_HOMED', 'xyz')`). After an aborted
    /// recovery Klipper believes Z is homed *at the fabricated value*, so it
    /// will not refuse the next print's motion, and `toolhead.homed_axes`
    /// cannot rescue the inference either — the same command sets that.
    ///
    /// The interlock therefore stays until something that actually knows
    /// clears it: a fresh dry run (`crate::recover`), which is the only
    /// thing that re-derives a plan against a real probe.
    Clean,
    /// Nothing to offer. The reason is for the daemon log only; see
    /// [`NoOffer::preserve_pending`] for the one thing it decides.
    Nothing(NoOffer),
}

/// Why detection has nothing to offer, and whether that verdict is
/// trustworthy enough to act on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NoOffer {
    /// Human-readable reason, for the daemon log.
    pub reason: String,
    /// `true` when the absence of an offer is **our failure to derive
    /// one**, not a positive finding — an unreadable or unanalyzable WAL,
    /// or a recorder that stopped while the print's fate stayed unknown.
    ///
    /// A stale `pending_recovery.json` must survive those: the wizard
    /// reads that file, not fresh detection, and the evidence behind a
    /// genuine offer scrolls out of [`DETECT_SEGMENT_LIMIT`] after a few
    /// rotations. Clearing on "I could not tell" would silently retract
    /// a real offer.
    pub preserve_pending: bool,
}

impl NoOffer {
    /// A positive finding: there is genuinely nothing to recover, so a
    /// stale pending file is stale and gets cleared.
    fn settled(reason: impl Into<String>) -> Self {
        Self {
            reason: reason.into(),
            preserve_pending: false,
        }
    }

    /// An inconclusive finding: keep whatever offer already exists.
    fn inconclusive(reason: impl Into<String>) -> Self {
        Self {
            reason: reason.into(),
            preserve_pending: true,
        }
    }
}

/// A print that stopped uncleanly but had nothing left to print.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Completion {
    /// Absolute path of the print file.
    pub file: String,
    /// The offset the gate tested at (`stop_set.file_window.start`).
    pub tested_offset: u64,
    /// Size of the print file, bytes.
    pub file_size: u64,
    /// How many bytes of the file sit after [`tested_offset`](Self::tested_offset).
    /// The number a percentage would have mistaken for progress.
    pub trailing_bytes: u64,
    /// What the replay found in those bytes.
    pub work: plr_analyzer::RemainingWork,
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
    ///
    /// **Display only. Never gate on this.** A functionally complete
    /// print reads as ~95% of a 300 KB file and ~99.93% of a 20 MB one,
    /// because the trailing slicer config block is a fixed ~12–14 KB
    /// regardless of file size, so no threshold on this number separates
    /// "finished" from "died on the last layer". The
    /// [module-level completion gate](self#the-completion-gate) is the
    /// only thing allowed to answer that question, and it does it by
    /// replaying content, not by comparing offsets.
    pub percent: Option<f64>,
    /// Debug rendering of the crash classification.
    ///
    /// When [`frame_invalid`](Self::frame_invalid) is set this string also
    /// carries a trailing `"; Z FRAME UNKNOWN"`
    /// ([`FRAME_INVALID_NOTE`]). That duplication is deliberate and is
    /// **display only**: the Klipper-side wizard
    /// (`klippy_plugin/plr/wizard.py`) renders `crash_class` verbatim and
    /// has no knowledge of the boolean, so dropping the note would silently
    /// remove the Z-frame warning from the wizard's screen. Nothing in this
    /// crate reads the sentinel — see
    /// [`frame_invalid`](Self::frame_invalid).
    pub crash_class: String,
    /// A previous recovery aborted at or after declaring the shifted Z
    /// frame, so Klipper's Z frame is UNKNOWN and a fresh dry run is
    /// required before resuming (`frame_invalid.json` exists).
    ///
    /// This is the field consumers must branch on;
    /// [`announcement_commands`] does. `#[serde(default)]` so a
    /// pending file written before the field existed still loads.
    #[serde(default)]
    pub frame_invalid: bool,
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
        // The WAL could not be read at all: we failed to derive an
        // answer, we did not establish that there is none.
        Err(reason) => return Detection::Nothing(NoOffer::inconclusive(reason)),
    };
    let heartbeat = scan::load_heartbeat(heartbeat_path).ok();
    let receive_seq = scan::load_receive_seq(&wal_dir.join(scan::RECEIVE_SEQ_FILE_NAME));
    // No file tail for the *classification*: it must stay cheap and
    // independent of the print file's availability. The completion gate
    // below does read a byte-capped tail, but only after classification
    // has already established that there is something to gate.
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
            Err(e) => {
                return Detection::Nothing(NoOffer::inconclusive(format!(
                    "unclean WAL but unanalyzable: {e}"
                )))
            }
        };
    let Some((file, file_position)) = scan::last_print_file(&merged) else {
        // Inconclusive, not settled: the segments detection reads are the
        // newest DETECT_SEGMENT_LIMIT, so the context naming the print can
        // simply have scrolled out of the window after a few rotations. That
        // is indistinguishable from "there was never a print", and one of
        // those two readings must not delete a live offer.
        return Detection::Nothing(NoOffer::inconclusive(
            "unclean stop and no print context in the segments read (it may have rotated out)",
        ));
    };
    // The print ended on purpose, and the log says so directly: the
    // newest context THAT NAMES THIS FILE carries `print_stats.state`. This
    // is positive evidence, independent of the `CleanShutdown` marker, so a
    // lost marker no longer costs a false offer — and, being tied to the
    // file, it cannot be forged by a restart baseline (see
    // `last_print_state_for`).
    if let Some(state) = last_print_state_for(&recovery.timeline, &file) {
        if crate::convert::PrintState::parse(&state).is_conclusive_end() {
            return Detection::Nothing(NoOffer::settled(format!(
                "the print ended on purpose: print_stats.state was {state:?} at the last \
                 journaled context"
            )));
        }
    }
    // The recorder was stopped on purpose. The WAL's tail therefore says
    // nothing about the print, which may well still be running — do not
    // announce, and do not retract an offer either.
    if let Some(mono_ns) = recovery.timeline.recorder_stopped_tail {
        return Detection::Nothing(NoOffer::inconclusive(format!(
            "the recorder was stopped on purpose at mono {mono_ns} ns; the print's fate is \
             unknown (run `plrd recover` if it did die — the WAL is intact)"
        )));
    }
    // The completion gate: replay what is left of the file.
    match completion_gate(&recovery, &file) {
        GateOutcome::Complete(completion) => return Detection::Complete(completion),
        GateOutcome::Announce(reason) => {
            if !reason.is_empty() {
                eprintln!("plrd: completion gate cannot suppress: {reason}");
            }
        }
    }
    let file_size = std::fs::metadata(&file).map(|m| m.len()).ok();
    #[allow(clippy::cast_precision_loss)]
    let percent = file_size
        .filter(|size| *size > 0)
        .map(|size| (file_position.min(size) as f64 / size as f64) * 100.0);
    // The interlock is carried as an explicit flag, which is what the
    // announcement branches on. The sentinel is *also* folded into the
    // crash-class string, for the out-of-crate wizard alone — see the
    // field docs.
    let frame_invalid = read_frame_invalid(wal_dir).is_some();
    let mut crash_class = format!("{:?}", recovery.window.class);
    if frame_invalid {
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
        frame_invalid,
    })
}

/// The newest journaled `print_stats.state` **that describes `file`**.
///
/// # Why the file has to match
///
/// `print_stats` and `virtual_sdcard` are separate Klipper objects, and a
/// klippy re-init resets both — but a `Context` is a merged snapshot, so a
/// reverse scan for "the newest `Some(state)`" happily takes the state from
/// one world and the file from another.
///
/// The shape that matters: an MCU shutdown kills a print mid-layer (fully
/// recoverable), the operator runs the standard `FIRMWARE_RESTART`, and
/// `PrintStats.reset()` journals `standby` while `virtual_sdcard` reports no
/// file. Read naively, the newest state is `standby` — "ended on purpose" —
/// while the print file comes from a pre-restart context. Detection would
/// then delete `pending_recovery.json`, and since the wizard reads that file
/// rather than re-running detection, the operator is never told a resumable
/// print exists.
///
/// So the state is only taken from a context that **still names the print
/// file**: state and file then describe the same instant. A restart baseline
/// carries `virtual_sdcard: None` and is skipped.
///
/// `convert.rs` guards the identical hazard on the write side with
/// `print_in_progress`; this is the read-side equivalent.
fn last_print_state_for(timeline: &plr_reconstruct::WalTimeline, file: &str) -> Option<String> {
    timeline
        .contexts
        .iter()
        .rev()
        .find(|c| {
            c.virtual_sdcard
                .as_ref()
                .is_some_and(|v| v.file_path == file)
        })
        .and_then(|c| c.print_state.clone())
}

/// What the completion gate decided.
enum GateOutcome {
    /// Positive proof: nothing left to print.
    Complete(Completion),
    /// Announce. The string is why the gate could not suppress (empty
    /// when the answer was simply "work remains", which is not a
    /// degradation and needs no log line).
    Announce(String),
}

/// Adapts [`plr_reconstruct::ExclusionReport`] onto the analyzer's
/// [`plr_analyzer::ExclusionOracle`].
///
/// The analyzer deliberately does not depend on `plr-reconstruct` (it sits
/// on `plr-gcode` alone); this daemon owns both crates, so the join
/// belongs here.
pub(crate) struct ReportOracle<'a>(pub(crate) &'a plr_reconstruct::ExclusionReport);

impl plr_analyzer::ExclusionOracle for ReportOracle<'_> {
    fn is_conclusive(&self) -> bool {
        self.0.is_conclusive()
    }
    fn is_excluded(&self, object: &str) -> bool {
        self.0.is_excluded(object)
    }
}

/// Everything the completion gate needs in order to answer.
///
/// A struct rather than a parameter list because the whole point of
/// [`completion_verdict`] is that **both** call sites supply every field:
/// see that function's "One gate, two call sites".
pub(crate) struct GateInputs<'a> {
    /// The context whose interpreter state seeded the replay. Its
    /// `virtual_sdcard` supplies the journaled file size for the identity
    /// check, so it must be the context the model was actually built from.
    pub anchor: &'a plr_wal::Context,
    /// The replay of `tail`, built at `base_offset` from `anchor`'s state.
    ///
    /// [`Deferred`](ModelSource::Deferred) lets the cheap preconditions run
    /// first: on a refusal the replay is never built. `plrd recover` needs
    /// the model for planning anyway and passes
    /// [`Built`](ModelSource::Built).
    pub model: ModelSource<'a>,
    /// The bytes the model covers.
    pub tail: &'a [u8],
    /// Stream offset `tail` begins at (the anchor's file position).
    pub base_offset: u64,
    /// The offset to test at — `stop_set.file_window.start`.
    pub tested_offset: u64,
    /// Size of the print file on disk, bytes.
    pub file_size: u64,
    /// The cancelled-object picture, for the excluded-work question.
    pub exclusions: &'a plr_reconstruct::ExclusionReport,
}

/// How [`completion_verdict`] obtains the replay it needs.
#[derive(Clone, Copy)]
pub(crate) enum ModelSource<'a> {
    /// Already built — the caller needed it for something else too.
    Built(&'a plr_analyzer::LayerModel),
    /// Build it only if the preconditions pass.
    Deferred(&'a dyn Fn() -> plr_analyzer::LayerModel),
}

/// What the gate concluded.
pub(crate) enum GateVerdict {
    /// Positive proof: nothing left to print.
    Complete(plr_analyzer::RemainingWork),
    /// Suppression is not available, and why.
    MustNotSuppress(GateRefusal),
}

/// Why the gate would not call a print finished.
///
/// Typed rather than a string because the two callers act on these
/// differently — in particular [`FileChanged`](Self::FileChanged) is not
/// merely "do not suppress", it means the byte offsets in the WAL no longer
/// address the file on disk, so planning a resume from them would be
/// meaningless. A string would have left that distinction to substring
/// matching at the call site.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum GateRefusal {
    /// Printing work remains. The ordinary answer, not a degradation: it
    /// needs no log line and no special handling.
    WorkRemains,
    /// The print file on disk is not the file the WAL recorded.
    FileChanged {
        /// Size Klipper reported while printing.
        journaled: u64,
        /// Size found now.
        on_disk: u64,
    },
    /// Too little remains after the tested offset to be evidence of
    /// anything — see [`MIN_TRUSTWORTHY_REMAINDER_BYTES`] for the
    /// derivation and for the truncation hazard this closes.
    RemainderTooSmall {
        /// Bytes remaining after the tested offset.
        remainder: u64,
        /// The floor it failed to reach.
        minimum: u64,
    },
    /// The replay covering the tested region could not be trusted.
    Untrusted(plr_analyzer::WorkUnknown),
    /// The remainder is nothing but comments and blank lines, and the print
    /// file's identity was never confirmed — so this may be a trailing config
    /// block, or it may be whatever comments happened to survive a
    /// truncation.
    ///
    /// # Why comments alone are not proof
    ///
    /// `plr_analyzer::RemainingWork::Nothing` means "no command lines at
    /// all", which is what the tail of a slicer's trailing config block looks
    /// like — and also what a *truncated* file's surviving comments look
    /// like. Measured over the fixture corpus, the longest comment-only run
    /// inside a fixture *body* is 17–49 bytes, comfortably under
    /// [`MIN_TRUSTWORTHY_REMAINDER_BYTES`]; but that is the weakest part of
    /// the corpus, because only the *footers* are real slicer output and the
    /// bodies are synthetic. The real footers settle it: their longest single
    /// comment lines are **404 bytes** (`; different_settings_to_system = …`,
    /// Orca 2.3.1) and **553 bytes** (`; end_gcode = …`, `PrusaSlicer` 2.9.3),
    /// so one genuine slicer comment already clears the floor without any
    /// multi-line run — and the synthetic *headers*, which are pure comment,
    /// run 446–1,188 bytes before a real thumbnail block is even considered.
    ///
    /// So a comment-only remainder above the floor is reachable outside
    /// genuine completion, and the branch needs positive evidence like the
    /// other two.
    ///
    /// # The evidence required, and why it costs nothing
    ///
    /// The file's size must have been *journaled* and must match what is on
    /// disk. Check 1 already refuses on a mismatch, so reaching here with a
    /// journaled size means the file is provably intact — and an intact file
    /// whose remainder is comment-only genuinely has no work left in it.
    /// Without a journaled size there is no evidence at all, so the gate
    /// refuses.
    ///
    /// The recorder journals the size whenever Klipper reports a non-zero one
    /// (`convert.rs`), so on any WAL written by this version the evidence is
    /// present and completion is still recognized. Only a pre-change WAL, or
    /// a printer that never reported a size, loses the `Nothing` proof — and
    /// those announce, which is the recoverable direction.
    UnverifiedCommentOnlyRemainder {
        /// Bytes of comment-only remainder that were examined.
        remainder: u64,
    },
}

impl GateRefusal {
    /// A human-readable reason, or `None` for [`WorkRemains`](Self::WorkRemains)
    /// which is not worth saying.
    pub(crate) fn reason(&self) -> Option<String> {
        match self {
            Self::WorkRemains => None,
            Self::FileChanged { journaled, on_disk } => Some(format!(
                "the print file is {on_disk} bytes but the WAL recorded {journaled}; \
                 it was truncated, replaced or re-sliced"
            )),
            Self::RemainderTooSmall { remainder, minimum } => Some(format!(
                "only {remainder} bytes remain after the tested offset, below the \
                 {minimum}-byte floor; too little to be evidence of completion (the smallest \
                 end sequence in the fixture corpus is 178 bytes)"
            )),
            Self::Untrusted(e) => Some(e.to_string()),
            Self::UnverifiedCommentOnlyRemainder { remainder } => Some(format!(
                "the {remainder}-byte remainder is comments only, and the WAL never recorded \
                 this file's size; comments alone cannot tell a trailing config block from \
                 the debris of a truncated file"
            )),
        }
    }

    /// `true` when the WAL's byte offsets no longer address the file on
    /// disk, so nothing derived from them — a completion verdict *or* a
    /// resume plan — can be trusted.
    pub(crate) fn invalidates_offsets(&self) -> bool {
        matches!(self, Self::FileChanged { .. })
    }
}

/// The completion gate: may this print be treated as finished?
///
/// # One gate, two call sites
///
/// Boot-time detection and `plrd recover` both ask this question, and an
/// earlier revision of this code asked it twice — with the preconditions
/// implemented in only one of the copies. The result was that the operator's
/// primary tool, `plrd recover`, printed "nothing to recover" and exited 0
/// for a re-sliced file, because the field added to defend exactly that
/// hazard was never read on that path.
///
/// So this function owns **every** precondition and is the only way to reach
/// a `Complete` answer. Callers supply inputs; they do not get to decide
/// which checks apply. Anything a future caller must also check belongs
/// here, not at the call site.
///
/// # The preconditions, and why each is a refusal rather than a warning
///
/// 1. **File identity.** A path is not an identity. `virtual_sdcard.file_size`
///    was journaled beside the position; if the file on disk is a different
///    size it is a different file — truncated, or re-sliced under the same
///    name, which is the ordinary iteration loop — and `base_offset` now
///    indexes into content we never saw. A replay from a stale offset can
///    land anywhere, including inside the new file's trailing config block,
///    where the gate would find no extrusion and call a 40 %-done print
///    finished.
/// 2. **A non-empty remainder.** Zero bytes examined is zero evidence. A
///    finished print leaves a 14–18 KB slicer footer after its last
///    deposition (see the module docs), so an empty remainder is the
///    signature of a truncated file, not of completion. Answering `Complete`
///    with `trailing_bytes: 0` would also print the self-refuting line "0
///    trailing bytes are the slicer footer".
/// 3. **A trustworthy replay**, delegated to
///    [`plr_analyzer::remaining_work`]: it refuses on a failed replay, an
///    offset outside the modeled window, or an extruder frame the file
///    contradicts.
///
/// The governing invariant is that this may only ever suppress on positive
/// proof, so every one of these is [`GateVerdict::MustNotSuppress`].
pub(crate) fn completion_verdict(inputs: &GateInputs<'_>) -> GateVerdict {
    let GateInputs {
        anchor,
        model,
        tail,
        base_offset,
        tested_offset,
        file_size,
        exclusions,
    } = *inputs;

    // 1. Identity.
    if let Some(journaled) = anchor
        .virtual_sdcard
        .as_ref()
        .and_then(|v| v.file_size)
        .filter(|journaled| *journaled != file_size)
    {
        return GateVerdict::MustNotSuppress(GateRefusal::FileChanged {
            journaled,
            on_disk: file_size,
        });
    }
    // 2. Enough remainder to be evidence. One byte is exactly as much
    //    evidence as zero, and a remainder far below any real footer is the
    //    signature of a truncated file rather than of completion — see
    //    `MIN_TRUSTWORTHY_REMAINDER_BYTES`.
    let remainder = file_size.saturating_sub(tested_offset);
    if remainder < MIN_TRUSTWORTHY_REMAINDER_BYTES {
        return GateVerdict::MustNotSuppress(GateRefusal::RemainderTooSmall {
            remainder,
            minimum: MIN_TRUSTWORTHY_REMAINDER_BYTES,
        });
    }
    // 3. A trustworthy replay, and the answer. Only now is the replay
    //    needed, so a `Deferred` source has cost nothing on the paths above.
    let replayed;
    let model = match model {
        ModelSource::Built(already) => already,
        ModelSource::Deferred(make) => {
            replayed = make();
            &replayed
        }
    };
    let oracle = ReportOracle(exclusions);
    let work = match plr_analyzer::remaining_work(
        model,
        tail,
        base_offset,
        tested_offset,
        plr_analyzer::AnchorFrame {
            absolute_coordinates: anchor.gcode.absolute_coordinates,
            absolute_extrude: anchor.gcode.absolute_extrude,
        },
        Some(&oracle),
    ) {
        Ok(work) => work,
        Err(e) => return GateVerdict::MustNotSuppress(GateRefusal::Untrusted(e)),
    };
    // 4. A comment-only remainder carries no content evidence at all, so it
    //    needs the file's identity positively CONFIRMED rather than merely
    //    un-contradicted. See `GateRefusal::UnverifiedCommentOnlyRemainder`.
    if work == plr_analyzer::RemainingWork::Nothing && journaled_size(anchor).is_none() {
        return GateVerdict::MustNotSuppress(GateRefusal::UnverifiedCommentOnlyRemainder {
            remainder,
        });
    }
    if work.is_complete() {
        GateVerdict::Complete(work)
    } else {
        GateVerdict::MustNotSuppress(GateRefusal::WorkRemains)
    }
}

/// The file size Klipper reported for the print this context describes, if it
/// was ever observed.
fn journaled_size(anchor: &plr_wal::Context) -> Option<u64> {
    anchor.virtual_sdcard.as_ref().and_then(|v| v.file_size)
}

/// Picks the context whose state seeds the replay, for a given print file.
///
/// The newest context at or before `tested_offset` **that names `file`**;
/// falling back to the oldest context naming it. Requiring the file match is
/// what makes the journaled size in [`GateInputs::anchor`] comparable to the
/// file on disk — a context describing a *different* print would supply a
/// size for that other file, and the identity check would compare two
/// unrelated numbers.
///
/// Shared with `pipeline`, so both paths anchor identically.
pub(crate) fn anchor_context<'a>(
    contexts: &'a [plr_wal::Context],
    file: &str,
    tested_offset: Option<u64>,
) -> Option<&'a plr_wal::Context> {
    let names_file = |c: &&plr_wal::Context| {
        c.virtual_sdcard
            .as_ref()
            .is_some_and(|v| v.file_path == file)
    };
    contexts
        .iter()
        .rev()
        .find(|c| {
            names_file(c)
                && c.virtual_sdcard
                    .as_ref()
                    .is_some_and(|v| tested_offset.is_none_or(|offset| v.file_position <= offset))
        })
        .or_else(|| contexts.iter().find(names_file))
}

/// Is there any printing work left in `file` after the low end of the
/// possible-stop window?
///
/// This is boot detection's half: it does the I/O and the replay, then hands
/// everything to [`completion_verdict`], which owns the preconditions.
///
/// Every failure path returns [`GateOutcome::Announce`]: see the
/// module-level "suppression asymmetry".
fn completion_gate(recovery: &plr_reconstruct::RecoveryReconstruction, file: &str) -> GateOutcome {
    let Some(window) = recovery.stop_set.file_window.as_ref() else {
        return GateOutcome::Announce("the stop set has no offset window".to_owned());
    };
    // The anchor supplies the interpreter state the replay starts from, so
    // its file position must be at or before the tested offset. Getting this
    // wrong would give the replay a stale absolute-E baseline, under which a
    // real extrusion can look like a retract, and the gate would then
    // suppress a needed offer.
    let Some(anchor) = anchor_context(&recovery.timeline.contexts, file, Some(window.start)) else {
        return GateOutcome::Announce(format!("no context names {file}"));
    };
    let base_offset = anchor
        .virtual_sdcard
        .as_ref()
        .map_or(0, |v| v.file_position);
    if base_offset > window.start {
        return GateOutcome::Announce(format!(
            "no context at or before the tested offset {} (oldest is {base_offset})",
            window.start
        ));
    }
    let state = match plr_reconstruct::anchor_state_from_context(&anchor.gcode) {
        Ok(state) => state,
        Err(e) => return GateOutcome::Announce(format!("anchor context state invalid: {e}")),
    };
    let tail = match read_capped_tail(Path::new(file), base_offset) {
        Ok(tail) => tail,
        Err(reason) => return GateOutcome::Announce(reason),
    };
    let file_size = base_offset.saturating_add(tail.len() as u64);
    let build = || {
        plr_analyzer::build_layer_model(
            state.clone(),
            &tail,
            base_offset,
            &plr_analyzer::ModelConfig::default(),
        )
    };
    match completion_verdict(&GateInputs {
        anchor,
        model: ModelSource::Deferred(&build),
        tail: &tail,
        base_offset,
        tested_offset: window.start,
        file_size,
        exclusions: &recovery.exclusions,
    }) {
        GateVerdict::MustNotSuppress(refusal) => {
            GateOutcome::Announce(refusal.reason().unwrap_or_default())
        }
        GateVerdict::Complete(work) => GateOutcome::Complete(Completion {
            file: file.to_owned(),
            tested_offset: window.start,
            file_size,
            trailing_bytes: file_size.saturating_sub(window.start),
            work,
        }),
    }
}

/// Reads `path` from `base_offset` to EOF, refusing anything larger than
/// [`MAX_GATE_TAIL_BYTES`].
///
/// The size is checked from the metadata *before* reading, and the read
/// itself is bounded by `take`, so neither a huge file nor a file that
/// grows under us can make this allocate without limit.
fn read_capped_tail(path: &Path, base_offset: u64) -> Result<Vec<u8>, String> {
    let metadata = std::fs::metadata(path)
        .map_err(|e| format!("print file {} unreadable: {e}", path.display()))?;
    let size = metadata.len();
    if base_offset > size {
        return Err(format!(
            "context offset {base_offset} exceeds {} ({size} bytes); wrong file?",
            path.display()
        ));
    }
    let remaining = size - base_offset;
    if remaining > MAX_GATE_TAIL_BYTES {
        return Err(format!(
            "{remaining} bytes remain after byte {base_offset}, over the {MAX_GATE_TAIL_BYTES}-byte gate cap"
        ));
    }
    let mut file = std::fs::File::open(path)
        .map_err(|e| format!("print file {} unreadable: {e}", path.display()))?;
    file.seek(std::io::SeekFrom::Start(base_offset))
        .map_err(|e| format!("print file {} unseekable: {e}", path.display()))?;
    let mut bytes = Vec::new();
    file.take(MAX_GATE_TAIL_BYTES)
        .read_to_end(&mut bytes)
        .map_err(|e| format!("print file {} unreadable: {e}", path.display()))?;
    Ok(bytes)
}

/// The operator message for a [`Detection::Complete`], or `None` when
/// there is nothing worth saying.
///
/// A print that ran its whole end sequence needs no message: it finished,
/// the operator watched it finish. A print whose end sequence did **not**
/// run is worth one line naming what did not happen — typically the
/// cooldown, the park, and motors-off — so the operator can decide for
/// themselves.
///
/// It deliberately does not offer to run those commands. A `PRINT_END`
/// macro routinely homes, drops the bed, or moves Z, and none of the
/// envelope or pre-flight analysis that guards a recovery plan applies to
/// an opaque macro body.
#[must_use]
pub fn completion_commands(completion: &Completion) -> Option<(String, String)> {
    let plr_analyzer::RemainingWork::EndSequenceOnly { commands } = &completion.work else {
        return None;
    };
    let name = base_name(&completion.file);
    let listed = commands.join(" ");
    let truncated = if completion.work.commands_truncated() {
        " ..."
    } else {
        ""
    };
    let message = format!(
        "dead-reckoning: print '{name}' is COMPLETE (no extrusion remained after byte {}); \
         no recovery needed. These end-sequence commands did not run: {listed}{truncated}",
        completion.tested_offset,
    );
    Some((
        format!("RESPOND PREFIX=dead-reckoning MSG=\"{message}\""),
        format!("M117 {message}"),
    ))
}

/// The file name part of a path, with quote characters stripped so it is
/// safe inside a `RESPOND ... MSG="..."` argument.
fn base_name(path: &str) -> String {
    path.rsplit(['/', '\\'])
        .next()
        .unwrap_or(path)
        .replace(['"', '\''], "")
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

/// Tri-state presence of an on-disk state file: whether it is absent,
/// present but unreadable/torn, or present and parsed.
///
/// The distinction matters for WAL retention (`crate::retention`): a torn
/// `pending_recovery.json` — produced by exactly the power loss this project
/// exists for — is an *unlocalizable pin*, not the absence of one, so it must
/// hold all evidence rather than let pruning proceed. `.ok()`-style readers
/// that collapse "torn" into "absent" fail open; these do not.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StatePresence<T> {
    /// The file does not exist.
    Absent,
    /// The file exists but could not be read or parsed (torn write, bad
    /// JSON, or an I/O error).
    Unreadable,
    /// The file exists and parsed.
    Present(T),
}

/// Reads [`PENDING_FILE_NAME`] as a tri-state (see [`StatePresence`]).
#[must_use]
pub fn read_pending_presence(wal_dir: &Path) -> StatePresence<PendingRecovery> {
    read_state_presence(&wal_dir.join(PENDING_FILE_NAME))
}

/// Reads [`FRAME_INVALID_FILE_NAME`] as a tri-state (see [`StatePresence`]).
#[must_use]
pub fn read_frame_invalid_presence(wal_dir: &Path) -> StatePresence<FrameInvalid> {
    read_state_presence(&frame_invalid_path(wal_dir))
}

/// Shared tri-state reader: `NotFound` is [`StatePresence::Absent`]; any
/// other I/O error, or a JSON parse failure, is
/// [`StatePresence::Unreadable`] (present but torn); a clean parse is
/// [`StatePresence::Present`].
fn read_state_presence<T: serde::de::DeserializeOwned>(path: &Path) -> StatePresence<T> {
    match std::fs::read_to_string(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => StatePresence::Absent,
        Err(_) => StatePresence::Unreadable,
        Ok(text) => match serde_json::from_str(&text) {
            Ok(value) => StatePresence::Present(value),
            Err(_) => StatePresence::Unreadable,
        },
    }
}

/// Removes any stale state file.
pub fn clear_pending(wal_dir: &Path) {
    let _ = std::fs::remove_file(wal_dir.join(PENDING_FILE_NAME));
}

/// The operator announcement as `(primary, fallback)` G-Code commands
/// (see the module docs for why these two).
#[must_use]
pub fn announcement_commands(pending: &PendingRecovery) -> (String, String) {
    let name = base_name(&pending.file);
    let progress = pending
        .percent
        .map_or(String::new(), |p| format!(", ~{p:.0}% complete"));
    // A prior recovery aborted after declaring the shifted frame: the
    // announcement warns the operator the Z frame is unknown and a fresh
    // dry run is required before resuming. Keyed on the typed flag, never
    // on the crash-class string.
    let frame_note = if pending.frame_invalid {
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

    /// Comment padding of exactly `len` bytes (`len >= 2`).
    fn pad(len: usize) -> String {
        let mut s = String::from(";");
        while s.len() + 1 < len {
            s.push('p');
        }
        s.push('\n');
        assert_eq!(s.len(), len);
        s
    }

    /// A 2000-byte print file with **unfinished work**: an extruding move
    /// begins at byte 500, so the completion gate must announce.
    ///
    /// The context these tests journal reports internal E = 10 in
    /// absolute-E mode, so the move's `E20` is a genuine +10 deposition.
    fn unfinished_gcode(dir: &Path) -> PathBuf {
        const MOVE: &str = "G1 X60 Y60 E20.0 F1800\n";
        let mut text = pad(500);
        text.push_str(MOVE);
        text.push_str(&pad(2_000 - 500 - MOVE.len()));
        assert_eq!(text.len(), 2_000);
        let path = dir.join("part.gcode");
        std::fs::write(&path, text).unwrap();
        path
    }

    /// A 2000-byte print file whose remainder from byte 500 is nothing
    /// but a footer: an end sequence and a config block.
    fn finished_gcode(dir: &Path) -> PathBuf {
        const FOOTER: &str = "M107\nM104 S0\nM140 S0\nG1 E9.2 F2100\nM84\n";
        let mut text = pad(500);
        text.push_str(FOOTER);
        text.push_str(&pad(2_000 - 500 - FOOTER.len()));
        assert_eq!(text.len(), 2_000);
        let path = dir.join("part.gcode");
        std::fs::write(&path, text).unwrap();
        path
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
            // Detection classifies the previous session from paths,
            // positions and markers; the print-time axis plays no part.
            print_time: None,
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

    /// Like [`context`] but in **relative-E mode** (`M83`) with the
    /// extruder at internal E = 0 — the shape Klipper reports for a print
    /// sliced with `use_relative_e_distances = 1`, which is what both
    /// real-footer fixtures are.
    ///
    /// The E mode matters and is not cosmetic: the anchor context seeds the
    /// replay, and replaying a relative-E file in absolute mode turns the
    /// footer's `E-.04571` wipe into a positive E target and therefore into
    /// apparent deposition. See
    /// `a_wrong_e_mode_in_the_anchor_context_announces` for the direction
    /// that mistake fails in.
    fn context_relative_e(mono_ns: u64, file_path: &str, file_position: u64) -> Context {
        let mut ctx = context(mono_ns, file_path, file_position);
        ctx.gcode.absolute_extrude = false;
        ctx.gcode.position = vec![50.0, 50.0, 0.2, 0.0];
        ctx.gcode.gcode_position = vec![50.0, 50.0, 0.2, 0.0];
        ctx
    }

    fn write_wal(dir: &Path, records: &[WalRecord]) {
        write_segment(dir, 1, &SegmentHeader::new(1, 1), records);
    }

    /// Writes segment `index` with an explicit header, so tests can give a
    /// segment a real clock epoch (and give two segments *different* epochs,
    /// which is what a reboot looks like on disk).
    fn write_segment(dir: &Path, index: u64, header: &SegmentHeader, records: &[WalRecord]) {
        let mut writer = WalWriter::create(Vec::new(), header).unwrap();
        for r in records {
            writer.append(r).unwrap();
        }
        std::fs::write(
            dir.join(crate::scan::segment_file_name(index)),
            writer.into_inner(),
        )
        .unwrap();
    }

    #[test]
    fn unclean_print_yields_pending_with_percent() {
        let dir = temp_dir("pending");
        let gcode_path = unfinished_gcode(&dir);
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

    /// `detect` itself must not touch the Z-frame interlock.
    ///
    /// This covers the *classifier* only, and it is not the important site:
    /// the automatic clear this branch removed lived in `daemon.rs`'s
    /// `Detection::Clean` arm, and a clear reintroduced there leaves this
    /// test passing. `daemon::tests::boot_detection_never_clears_the_frame_interlock`
    /// is the one that guards the boot path, mutation-verified at the site
    /// the code was deleted from.
    #[test]
    fn a_clean_end_leaves_the_frame_interlock_alone() {
        use super::{read_frame_invalid, write_frame_invalid, FrameInvalid};
        let dir = temp_dir("clean-keeps-interlock");
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
        write_frame_invalid(
            &dir,
            &FrameInvalid {
                detected_wall_ns: 1,
                step_id: 7,
                phase: "shifted-frame".to_owned(),
                reason: "shifted-frame-declared".to_owned(),
            },
        )
        .unwrap();
        assert_eq!(detect(&dir, &dir.join("hb"), 1), Detection::Clean);
        assert!(
            read_frame_invalid(&dir).is_some(),
            "detection must never clear the interlock"
        );
    }

    #[test]
    fn missing_wal_and_missing_context_are_nothing() {
        let dir = temp_dir("nothing");
        let Detection::Nothing(no_offer) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected nothing");
        };
        assert!(
            no_offer.reason.contains("no WAL segments"),
            "{}",
            no_offer.reason
        );
        // An unreadable WAL is us failing to derive an answer, so any
        // existing offer must survive it (D4).
        assert!(no_offer.preserve_pending);
        // Unclean but contextless: unanalyzable, not pending — and
        // likewise inconclusive.
        write_wal(&dir, &[WalRecord::Heartbeat(heartbeat(1_000_000_000, 1.0))]);
        let Detection::Nothing(no_offer) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected nothing");
        };
        assert!(
            no_offer.reason.contains("unanalyzable"),
            "{}",
            no_offer.reason
        );
        assert!(no_offer.preserve_pending);
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
        let gcode_path = unfinished_gcode(&dir);
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
        assert!(!pending.frame_invalid);
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
            pending.frame_invalid,
            "the typed flag is what consumers use"
        );
        assert!(
            pending.crash_class.contains(FRAME_INVALID_NOTE),
            "the wizard-facing sentinel rides along: {}",
            pending.crash_class
        );
        // The announcement keys on the FLAG, not the string: clearing the
        // sentinel out of the crash class must not silence the warning.
        let (primary, _) = announcement_commands(&PendingRecovery {
            crash_class: "HostDeathOrPowerLoss".to_owned(),
            ..pending.clone()
        });
        assert!(primary.contains("Z frame is UNKNOWN"), "{primary}");
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
        assert!(!pending.frame_invalid);
        // A pending file written before the flag existed still loads,
        // defaulting to "no interlock".
        let legacy = serde_json::json!({
            "detected_wall_ns": 1, "file": "/g/x.gcode", "file_position": 5,
            "file_size": null, "percent": null, "crash_class": "HostDeathOrPowerLoss",
        });
        let decoded: PendingRecovery = serde_json::from_value(legacy).unwrap();
        assert!(!decoded.frame_invalid);
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
            frame_invalid: false,
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

    // -----------------------------------------------------------------
    // The completion gate (D1) and its refusal to suppress without proof
    // -----------------------------------------------------------------

    /// The whole point: an unclean tail plus a print file whose remainder
    /// is only a footer is a FINISHED print, not a recovery.
    #[test]
    fn a_finished_print_is_complete_not_pending() {
        let dir = temp_dir("complete");
        let gcode_path = finished_gcode(&dir);
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(1_000_000_000, gcode_path.to_str().unwrap(), 500)),
            ],
        );
        let detection = detect(&dir, &dir.join("hb"), 1);
        let Detection::Complete(completion) = detection else {
            panic!("expected Complete, got {detection:?}");
        };
        assert_eq!(completion.tested_offset, 500);
        assert_eq!(completion.file_size, 2_000);
        assert_eq!(completion.trailing_bytes, 1_500);
        // 1500 of 2000 bytes remain -- a "75% complete" print by any
        // percentage measure, and yet there is nothing left to print.
        let plr_analyzer::RemainingWork::EndSequenceOnly { commands } = &completion.work else {
            panic!("expected EndSequenceOnly, got {:?}", completion.work);
        };
        assert_eq!(commands, &["M107", "M104", "M140", "G1", "M84"]);
        // And the operator gets told what did not run, with no offer to
        // run it.
        let (primary, fallback) = super::completion_commands(&completion).expect("message");
        assert!(primary.contains("is COMPLETE"), "{primary}");
        assert!(primary.contains("M84"), "{primary}");
        assert!(!primary.to_lowercase().contains("resume"), "{primary}");
        assert!(fallback.starts_with("M117 "), "{fallback}");
    }

    /// Builds a WAL whose print file's remainder from byte 500 is nothing but
    /// comments — the shape of a stop inside a trailing config block, and
    /// equally the shape of a truncated file whose surviving tail is comments.
    ///
    /// `journaled_size` is what Klipper reported for the file; `None` models a
    /// pre-change WAL, or a printer that never reported one.
    fn comment_only_remainder(tag: &str, journaled_size: Option<u64>) -> PathBuf {
        let dir = temp_dir(tag);
        let gcode_path = dir.join("part.gcode");
        let mut text = pad(500);
        text.push_str(&pad(1_500));
        std::fs::write(&gcode_path, &text).unwrap();
        let mut ctx = context(1_000_000_000, gcode_path.to_str().unwrap(), 500);
        if let Some(vsd) = &mut ctx.virtual_sdcard {
            vsd.file_size = journaled_size;
        }
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(ctx),
            ],
        );
        dir
    }

    /// A remainder with no commands at all (mid-config-block) is complete and
    /// worth no message — **when the file's identity is confirmed.** The
    /// recorder journals the size on every WAL this version writes, so this is
    /// the ordinary case.
    #[test]
    fn a_stop_inside_the_config_block_is_complete_and_silent() {
        let dir = comment_only_remainder("complete-silent", Some(2_000));
        let Detection::Complete(completion) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected Complete");
        };
        assert_eq!(completion.work, plr_analyzer::RemainingWork::Nothing);
        assert!(super::completion_commands(&completion).is_none());
    }

    /// **Comments alone are not proof.** The identical remainder, with no
    /// journaled size, must announce: comments cannot tell a trailing config
    /// block from the debris of a truncated file, and a single real slicer
    /// comment line already exceeds `MIN_TRUSTWORTHY_REMAINDER_BYTES` (404 B
    /// in the Orca footer, 553 B in the Prusa one), so the remainder-size
    /// floor does not cover this.
    ///
    /// The harm being prevented: a print dies with work remaining, its file
    /// then loses its tail, and the surviving comments read as "finished" —
    /// suppressing the offer the wizard reads. When the size *was* journaled
    /// that shape is already caught as `FileChanged`; this closes the case
    /// where it was not.
    #[test]
    fn a_comment_only_remainder_without_a_journaled_size_announces() {
        let dir = comment_only_remainder("comment-unverified", None);
        let detection = detect(&dir, &dir.join("hb"), 1);
        assert!(
            matches!(detection, Detection::Pending(_)),
            "comments with no size evidence must announce: {detection:?}"
        );
        // The refusal explains itself, and does not deny the offsets.
        let refusal = super::GateRefusal::UnverifiedCommentOnlyRemainder { remainder: 1_500 };
        assert!(refusal
            .reason()
            .is_some_and(|r| r.contains("comments only")));
        assert!(!refusal.invalidates_offsets());
    }

    /// The slicer fixtures, end to end through `detect`: a stop at the
    /// last depositing line is a completion, one move earlier is not.
    ///
    /// `trailing_bytes` is asserted **exactly**, per fixture, so the number
    /// a percentage would have mistaken for progress is pinned to measured
    /// reality rather than to a threshold that could drift.
    #[test]
    fn the_footer_fixtures_gate_correctly_end_to_end() {
        let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/synthetic");
        for (name, last_move, trailing) in [
            // Real footers (PrusaSlicer 2.9.3, OrcaSlicer 2.3.1).
            (
                "prusa_real_footer.gcode",
                "G1 X117.121 Y105.942 E.03577",
                13_962_u64,
            ),
            (
                "orca_real_footer.gcode",
                "G1 X111.453 Y115.5 E.03577",
                17_744,
            ),
            // Fully synthetic.
            ("prusa_footer_complete.gcode", "G1 X70 Y30 E4.9768", 12_783),
            (
                "orca_footer_complete.gcode",
                "G1 X55 Y55 E0.7465\nEXCLUDE_OBJECT_END",
                12_684,
            ),
        ] {
            let marker = "; THE LAST DEPOSITING LINE IS ABOVE THIS COMMENT";
            let bytes = std::fs::read(fixtures.join(name)).unwrap();
            let text = String::from_utf8(bytes.clone()).unwrap();
            let done_at = text.find(marker).expect("marker") as u64;
            let died_at = text.find(last_move).expect("last move") as u64;
            for (offset, expect_complete) in [(done_at, true), (died_at, false)] {
                let dir = temp_dir("fixture-gate");
                let gcode_path = dir.join(name);
                std::fs::write(&gcode_path, &bytes).unwrap();
                write_wal(
                    &dir,
                    &[
                        WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                        // The replay starts AT this offset, i.e. past the
                        // body's own `M82`/`M83`, so the extruder frame
                        // comes entirely from the context — exactly as it
                        // does after a real power loss. Both real-footer
                        // fixtures are relative-E (matching their committed
                        // `use_relative_e_distances = 1`), and a relative
                        // frame is also correct-or-harmless for the two
                        // synthetic fixtures, whose footers retract under
                        // either reading.
                        WalRecord::Context(context_relative_e(
                            1_000_000_000,
                            gcode_path.to_str().unwrap(),
                            offset,
                        )),
                    ],
                );
                let detection = detect(&dir, &dir.join("hb"), 1);
                assert_eq!(
                    matches!(detection, Detection::Complete(_)),
                    expect_complete,
                    "{name} at byte {offset} (of {}): {detection:?}",
                    bytes.len()
                );
                if let Detection::Complete(c) = detection {
                    // The measured distance a percentage would have
                    // mistaken for progress.
                    assert_eq!(c.trailing_bytes, trailing, "{name}");
                    assert_eq!(c.file_size, bytes.len() as u64, "{name}");
                }
            }
        }
    }

    /// **A wrong E mode in the anchor context announces.**
    ///
    /// The anchor context seeds the replay's extruder frame. If it claims
    /// absolute E for a file that is relative (or vice versa), the footer's
    /// retract/wipe values are misread. This pins the *direction* that
    /// failure takes: misread values look like extrusion, so the gate
    /// announces a recovery it did not need to — the recoverable mistake,
    /// never a suppressed offer.
    #[test]
    fn a_wrong_e_mode_in_the_anchor_context_announces() {
        let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/synthetic");
        let name = "prusa_real_footer.gcode";
        let bytes = std::fs::read(fixtures.join(name)).unwrap();
        let text = String::from_utf8(bytes.clone()).unwrap();
        let done_at = text
            .find("; THE LAST DEPOSITING LINE IS ABOVE THIS COMMENT")
            .expect("marker") as u64;
        let dir = temp_dir("wrong-e-mode");
        let gcode_path = dir.join(name);
        std::fs::write(&gcode_path, &bytes).unwrap();
        // `context` claims absolute E; the fixture is relative.
        let mut wrong = context(1_000_000_000, gcode_path.to_str().unwrap(), done_at);
        wrong.gcode.position = vec![50.0, 50.0, 0.2, 0.0];
        wrong.gcode.gcode_position = vec![50.0, 50.0, 0.2, 0.0];
        assert!(wrong.gcode.absolute_extrude, "the wrong mode, on purpose");
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(wrong),
            ],
        );
        assert!(
            matches!(detect(&dir, &dir.join("hb"), 1), Detection::Pending(_)),
            "a misread E frame must fail towards announcing"
        );
    }

    /// Positive proof or nothing: every way of failing to answer must
    /// still announce.
    #[test]
    fn the_gate_never_suppresses_without_proof() {
        // 1. The print file cannot be read.
        let dir = temp_dir("gate-unreadable");
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(1_000_000_000, "/nonexistent/plrd-gate.gcode", 500)),
            ],
        );
        assert!(
            matches!(detect(&dir, &dir.join("hb"), 1), Detection::Pending(_)),
            "an unreadable print file must announce"
        );

        // 2. The remainder does not replay (a G2 with no offsets is a
        //    hard state-machine error, not a warning).
        let dir = temp_dir("gate-unparseable");
        let gcode_path = dir.join("part.gcode");
        let mut text = pad(500);
        text.push_str("G2 X5 Y5\n");
        text.push_str(&pad(2_000 - 500 - 9));
        std::fs::write(&gcode_path, &text).unwrap();
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(1_000_000_000, gcode_path.to_str().unwrap(), 500)),
            ],
        );
        assert!(
            matches!(detect(&dir, &dir.join("hb"), 1), Detection::Pending(_)),
            "an unparseable remainder must announce"
        );

        // 3. The context offset is past the end of the file: wrong file.
        let dir = temp_dir("gate-wrong-file");
        let gcode_path = finished_gcode(&dir);
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(
                    1_000_000_000,
                    gcode_path.to_str().unwrap(),
                    9_000_000,
                )),
            ],
        );
        assert!(
            matches!(detect(&dir, &dir.join("hb"), 1), Detection::Pending(_)),
            "an offset past EOF must announce"
        );
    }

    /// **The print file must still be the file we were printing.** A path
    /// is not an identity: the file can be truncated or re-sliced under the
    /// same name, and then `file_position` indexes into content we never saw.
    #[test]
    fn a_file_whose_size_no_longer_matches_the_wal_announces() {
        let dir = temp_dir("gate-identity");
        let gcode_path = finished_gcode(&dir); // 2000 bytes, footer from 500
        let path = gcode_path.to_str().unwrap().to_owned();
        // The size Klipper reported matches: suppression is allowed.
        let mut ctx = context_relative_e(1_000_000_000, &path, 500);
        if let Some(vsd) = &mut ctx.virtual_sdcard {
            vsd.file_size = Some(2_000);
        }
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(ctx.clone()),
            ],
        );
        assert!(matches!(
            detect(&dir, &dir.join("hb"), 1),
            Detection::Complete(_)
        ));
        // A re-slice under the same name: same path, different size.
        if let Some(vsd) = &mut ctx.virtual_sdcard {
            vsd.file_size = Some(2_048);
        }
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(ctx),
            ],
        );
        assert!(
            matches!(detect(&dir, &dir.join("hb"), 1), Detection::Pending(_)),
            "a size mismatch must announce"
        );
    }

    /// **Exactly one refusal invalidates the WAL's offsets.**
    ///
    /// `FileChanged` means the byte offsets no longer address the file, so
    /// `pipeline` turns it into `NotPossible` rather than planning a resume
    /// from them. The other three are ordinary "cannot suppress" answers and
    /// must let planning continue — widening this predicate would deny a
    /// recoverable print, which is the expensive direction.
    ///
    /// Pinned per-variant so both a deletion and a widening fail here; the
    /// wiring from this predicate to `NotPossible` is pinned end-to-end by
    /// `pipeline::e2e_tests::a_re_sliced_file_is_not_reported_complete_by_the_recover_path`.
    #[test]
    fn only_a_changed_file_invalidates_the_offsets() {
        use super::GateRefusal;
        assert!(GateRefusal::FileChanged {
            journaled: 512_004,
            on_disk: 12_004,
        }
        .invalidates_offsets());
        for refusal in [
            GateRefusal::WorkRemains,
            GateRefusal::RemainderTooSmall {
                remainder: 3,
                minimum: super::MIN_TRUSTWORTHY_REMAINDER_BYTES,
            },
            GateRefusal::Untrusted(plr_analyzer::WorkUnknown::ReplayFailed { offset: 42 }),
            GateRefusal::Untrusted(plr_analyzer::WorkUnknown::UnintelligibleRemainder {
                offset: 42,
                commands: 1,
            }),
        ] {
            assert!(
                !refusal.invalidates_offsets(),
                "{refusal:?} must not deny a recoverable print"
            );
        }
        // And only `WorkRemains` is silent; every other refusal is reported.
        assert_eq!(GateRefusal::WorkRemains.reason(), None);
        for refusal in [
            GateRefusal::FileChanged {
                journaled: 1,
                on_disk: 2,
            },
            GateRefusal::RemainderTooSmall {
                remainder: 3,
                minimum: 128,
            },
            GateRefusal::Untrusted(plr_analyzer::WorkUnknown::ReplayFailed { offset: 42 }),
        ] {
            assert!(
                refusal.reason().is_some(),
                "{refusal:?} must explain itself"
            );
        }
    }

    /// **A remainder below the floor is not evidence.** Parameterised over
    /// the sizes that used to pass: one byte of remainder is exactly as much
    /// evidence as zero, and the old `> 0` test accepted 1, 3, 40 and 400.
    ///
    /// The operator case: a 20 MB print dies at 40 %, the WAL is intact with
    /// its floor context at ~8 MB, and the *file* then loses its tail to an
    /// unclean unmount. With `> 0` the gate examined the few dozen surviving
    /// bytes, found no extrusion, announced COMPLETE and cleared the pending
    /// offer the wizard reads.
    #[test]
    fn a_remainder_below_the_floor_is_not_proof_of_completion() {
        for remainder in [0_usize, 1, 3, 40, 400] {
            let dir = temp_dir("gate-small-remainder");
            let gcode_path = dir.join("part.gcode");
            // 500 bytes of comment padding, then a short remainder. Garbage
            // for the non-empty cases, so this covers the truncated-tail
            // shape rather than a tidy footer.
            let mut text = pad(500);
            text.push_str(&"x".repeat(remainder));
            std::fs::write(&gcode_path, &text).unwrap();
            write_wal(
                &dir,
                &[
                    WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                    WalRecord::Context(context_relative_e(
                        1_000_000_000,
                        gcode_path.to_str().unwrap(),
                        500,
                    )),
                ],
            );
            let detection = detect(&dir, &dir.join("hb"), 1);
            assert!(
                matches!(detection, Detection::Pending(_)),
                "a {remainder}-byte remainder must announce, got {detection:?}"
            );
        }
    }

    /// The floor is a floor, not a ban: a remainder at or above it, holding a
    /// real end sequence, still suppresses.
    #[test]
    fn a_remainder_at_the_floor_still_suppresses() {
        let dir = temp_dir("gate-floor-ok");
        let gcode_path = dir.join("part.gcode");
        let mut text = pad(500);
        let footer = "M107\nM104 S0\nM140 S0\nG1 E9.2 F2100\nM84\n";
        text.push_str(footer);
        // Pad the remainder to exactly the floor with comment bytes.
        let floor = usize::try_from(super::MIN_TRUSTWORTHY_REMAINDER_BYTES).expect("fits");
        text.push_str(&pad(floor - footer.len()));
        std::fs::write(&gcode_path, &text).unwrap();
        assert_eq!(text.len() - 500, floor);
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context_relative_e(
                    1_000_000_000,
                    gcode_path.to_str().unwrap(),
                    500,
                )),
            ],
        );
        let detection = detect(&dir, &dir.join("hb"), 1);
        assert!(
            matches!(detection, Detection::Complete(_)),
            "a remainder exactly at the floor must still suppress: {detection:?}"
        );
    }

    /// The old shape, kept: a file truncated at the stop offset.
    #[test]
    fn an_empty_remainder_is_not_proof_of_completion() {
        let dir = temp_dir("gate-truncated");
        let gcode_path = dir.join("part.gcode");
        // Truncated exactly at the stop offset.
        std::fs::write(&gcode_path, pad(500)).unwrap();
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context_relative_e(
                    1_000_000_000,
                    gcode_path.to_str().unwrap(),
                    500,
                )),
            ],
        );
        assert!(
            matches!(detect(&dir, &dir.join("hb"), 1), Detection::Pending(_)),
            "a file truncated at the stop offset must announce"
        );
        // A zero-length file likewise.
        std::fs::write(&gcode_path, b"").unwrap();
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context_relative_e(
                    1_000_000_000,
                    gcode_path.to_str().unwrap(),
                    0,
                )),
            ],
        );
        assert!(matches!(
            detect(&dir, &dir.join("hb"), 1),
            Detection::Pending(_)
        ));
    }

    /// The byte cap and the offset check on the tail reader itself.
    #[test]
    fn the_tail_reader_is_bounded_and_honest() {
        let dir = temp_dir("gate-cap");
        let gcode_path = finished_gcode(&dir);
        assert_eq!(
            super::read_capped_tail(&gcode_path, 0).unwrap().len(),
            2_000
        );
        assert_eq!(
            super::read_capped_tail(&gcode_path, 1_900).unwrap().len(),
            100
        );
        let err = super::read_capped_tail(&gcode_path, 2_001).unwrap_err();
        assert!(err.contains("exceeds"), "{err}");
        let err = super::read_capped_tail(Path::new("/nonexistent/x.gcode"), 0).unwrap_err();
        assert!(err.contains("unreadable"), "{err}");
        // A tail past the cap refuses rather than allocating.
        let big = dir.join("big.gcode");
        std::fs::write(&big, b"G1\n").unwrap();
        assert!(super::read_capped_tail(&big, 0).is_ok());
        assert_eq!(super::MAX_GATE_TAIL_BYTES, 64 * 1024 * 1024);
    }

    /// A cancelled object's deposition does not count as work -- but only
    /// once the exclusion report is conclusive.
    #[test]
    fn excluded_work_counts_until_the_report_is_conclusive() {
        use plr_wal::{ExcludeObjectDef, ExcludeState};
        const BODY: &str =
            "EXCLUDE_OBJECT_START NAME=part_a\nG1 X60 Y60 E20.0 F1800\nEXCLUDE_OBJECT_END\n";
        for (torn, expect_complete) in [(false, true), (true, false)] {
            let dir = temp_dir("gate-exclude");
            let gcode_path = dir.join("part.gcode");
            let mut text = pad(500);
            text.push_str(BODY);
            text.push_str(&pad(2_000 - 500 - BODY.len()));
            std::fs::write(&gcode_path, &text).unwrap();
            let mut ctx = context(1_000_000_000, gcode_path.to_str().unwrap(), 500);
            ctx.exclude = Some(Box::new(ExcludeState {
                definitions: Some(vec![ExcludeObjectDef::name_only("PART_A")]),
                excluded: vec!["PART_A".to_owned()],
                current: None,
            }));
            let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
            writer
                .append(&WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)))
                .unwrap();
            writer.append(&WalRecord::Context(ctx)).unwrap();
            let mut bytes = writer.into_inner();
            if torn {
                // A torn tail makes the journaled excluded set
                // inconclusive: a cancellation may have been lost.
                bytes.extend_from_slice(&[0xFF; 8]);
            }
            std::fs::write(dir.join("wal-000001.plr"), bytes).unwrap();
            let detection = detect(&dir, &dir.join("hb"), 1);
            assert_eq!(
                matches!(detection, Detection::Complete(_)),
                expect_complete,
                "torn={torn}: {detection:?}"
            );
        }
    }

    // -----------------------------------------------------------------
    // D3: the recorder stopped, not the print
    // -----------------------------------------------------------------

    /// **D3.** A graceful daemon stop journals
    /// `MarkerKind::RecorderStopped`, and reconstruction surfaces it as
    /// `recorder_stopped_tail`. Detection then says "the print's fate is
    /// unknown": no announcement, and no retraction of an existing offer.
    #[test]
    fn a_recorder_stopped_marker_suppresses_the_offer_without_retracting_it() {
        let dir = temp_dir("recorder-stopped");
        let gcode_path = unfinished_gcode(&dir);
        let records = [
            WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
            WalRecord::Context(context(1_000_000_000, gcode_path.to_str().unwrap(), 500)),
        ];
        // Without the marker this is a genuine mid-print death.
        write_wal(&dir, &records);
        assert!(matches!(
            detect(&dir, &dir.join("hb"), 1),
            Detection::Pending(_)
        ));

        // With it, the print's fate is unknown.
        let mut with_marker = records.to_vec();
        with_marker.push(WalRecord::Marker(Marker {
            mono_ns: 2_000_000_000,
            kind: MarkerKind::RecorderStopped,
        }));
        write_wal(&dir, &with_marker);
        let Detection::Nothing(no_offer) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected Nothing");
        };
        assert!(
            no_offer.reason.contains("stopped on purpose"),
            "{}",
            no_offer.reason
        );
        assert!(
            no_offer.preserve_pending,
            "an unknown fate must not retract an existing offer"
        );

        // ...but it is NOT a clean shutdown: the reconstruction still
        // reports a recovery, so `plrd recover` can still resume.
        let merged = crate::scan::load_merged(&dir).expect("merged");
        let inputs = plr_reconstruct::ReconstructInputs {
            scan: &merged,
            heartbeat: None,
            file_tail: None,
            receive_seq: None,
        };
        let outcome =
            plr_reconstruct::reconstruct(&inputs, &crate::convert::reconstruct_config(None))
                .expect("reconstruct");
        let plr_reconstruct::Reconstruction::Recovery(recovery) = outcome else {
            panic!("a graceful recorder stop must NOT read as a clean shutdown");
        };
        assert_eq!(recovery.timeline.recorder_stopped_tail, Some(2_000_000_000));
        assert!(recovery
            .timeline
            .notes
            .iter()
            .any(|n| matches!(n, plr_reconstruct::IngestNote::RecorderStopped { .. })));
    }

    /// Motion after the marker means the recorder came back and kept
    /// working, so the marker no longer describes how the log ends.
    #[test]
    fn a_recorder_stopped_marker_followed_by_motion_is_not_tail_evidence() {
        let dir = temp_dir("recorder-stopped-stale");
        let gcode_path = unfinished_gcode(&dir);
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(context(1_000_000_000, gcode_path.to_str().unwrap(), 500)),
                WalRecord::Marker(Marker {
                    mono_ns: 2_000_000_000,
                    kind: MarkerKind::RecorderStopped,
                }),
                WalRecord::StepperRange(plr_wal::StepperRange {
                    mono_ns: 3_000_000_000,
                    stepper: "stepper_z".to_owned(),
                    first_clock: 1_000,
                    last_clock: 2_000,
                    first_step_time: 1.0,
                    last_step_time: 2.0,
                    start_position: 0.2,
                    start_mcu_position: 100,
                    step_distance: 0.0025,
                    steps: vec![plr_wal::StepChunk {
                        interval: 5_000,
                        count: 10,
                        add: 0,
                    }],
                }),
            ],
        );
        assert!(
            matches!(detect(&dir, &dir.join("hb"), 1), Detection::Pending(_)),
            "a superseded recorder-stopped marker must not suppress"
        );
    }

    /// **The journaled print state is positive evidence on its own.** Even
    /// with no `CleanShutdown` marker anywhere, a context recording
    /// `print_stats.state == "cancelled"` settles the question.
    #[test]
    fn a_journaled_finished_print_state_settles_the_question() {
        for (state, settled) in [
            ("complete", true),
            ("cancelled", true),
            // `standby` is NOT proof on the read side: `PrintStats.reset()`
            // sets it on every klippy re-init, so a `FIRMWARE_RESTART` after
            // a recoverable death journals it for a print that did not end
            // on purpose. See `PrintState::is_conclusive_end`.
            ("standby", false),
            // `error` is what recovery exists for.
            ("error", false),
            ("printing", false),
            ("paused", false),
            // A state this version does not know must never suppress.
            ("hibernating", false),
        ] {
            let dir = temp_dir("journaled-state");
            let gcode_path = unfinished_gcode(&dir);
            let mut ctx = context(1_000_000_000, gcode_path.to_str().unwrap(), 500);
            ctx.print_state = Some(state.to_owned());
            write_wal(
                &dir,
                &[
                    WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                    WalRecord::Context(ctx),
                ],
            );
            let detection = detect(&dir, &dir.join("hb"), 1);
            if settled {
                let Detection::Nothing(no_offer) = detection else {
                    panic!("{state}: expected Nothing, got {detection:?}");
                };
                assert!(
                    no_offer.reason.contains("ended on purpose"),
                    "{}",
                    no_offer.reason
                );
                assert!(
                    !no_offer.preserve_pending,
                    "{state}: a deliberate end retracts a stale offer"
                );
            } else {
                assert!(
                    matches!(detection, Detection::Pending(_)),
                    "{state}: expected Pending, got {detection:?}"
                );
            }
        }
    }

    /// **The `FIRMWARE_RESTART` shape.** An MCU shutdown kills a print
    /// mid-layer — fully recoverable — and the operator runs the standard
    /// `FIRMWARE_RESTART`. `PrintStats.reset()` then journals `standby` in a
    /// context whose `virtual_sdcard` is empty, while the print file comes
    /// from a pre-restart context: state and file describe different worlds.
    ///
    /// Reading the newest state regardless of which context carried it would
    /// call that "ended on purpose" and **delete** `pending_recovery.json` —
    /// and because the wizard reads that file rather than re-running
    /// detection, the operator would never be told a resumable print exists.
    #[test]
    fn a_restart_baseline_cannot_forge_a_deliberate_end() {
        let dir = temp_dir("restart-baseline");
        let gcode_path = unfinished_gcode(&dir);
        let path = gcode_path.to_str().unwrap().to_owned();
        // Mid-print, printing.
        let mut printing = context(1_000_000_000, &path, 500);
        printing.print_state = Some("printing".to_owned());
        // Post-FIRMWARE_RESTART baseline: no file, `standby`.
        let mut restarted = context(2_000_000_000, &path, 0);
        restarted.virtual_sdcard = None;
        restarted.print_state = Some("standby".to_owned());
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(printing),
                WalRecord::Context(restarted),
            ],
        );
        let detection = detect(&dir, &dir.join("hb"), 1);
        assert!(
            matches!(detection, Detection::Pending(_)),
            "a restart baseline must not suppress a recoverable print: {detection:?}"
        );
        // And the state IS still read when it belongs to the file: the same
        // WAL with `cancelled` on the file-naming context does suppress.
        let mut cancelled = context(2_000_000_000, &path, 500);
        cancelled.print_state = Some("cancelled".to_owned());
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(cancelled),
            ],
        );
        let Detection::Nothing(no_offer) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected Nothing");
        };
        assert!(!no_offer.preserve_pending);
    }

    /// **The shape that distinguishes the file-match guard from its absence.**
    ///
    /// `virtual_sdcard.do_cancel` clears the loaded file and calls
    /// `note_cancel()` in the same g-code command
    /// (`klippy/extras/virtual_sdcard.py`: `self.current_file = None` then
    /// `self.print_stats.note_cancel()`), so Klipper really does journal a
    /// **conclusive** `cancelled` in a context that no longer names any file.
    ///
    /// That context describes the cancel of *some* print; it is not evidence
    /// about the file the newest file-naming context named. A reverse scan for
    /// the newest `Some(state)` — the naive form — would take `cancelled`
    /// from it, call the print deliberately ended, and delete
    /// `pending_recovery.json`, which is the file the wizard reads.
    ///
    /// `a_restart_baseline_cannot_forge_a_deliberate_end` does not cover
    /// this: its baseline carries `standby`, which `is_conclusive_end`
    /// already excludes, so the file match is never reached. This test is the
    /// one that fails if `last_print_state_for` loses its file predicate.
    #[test]
    fn a_conclusive_cancel_on_a_fileless_context_does_not_suppress() {
        let dir = temp_dir("fileless-cancel");
        let gcode_path = unfinished_gcode(&dir);
        let path = gcode_path.to_str().unwrap().to_owned();
        // Mid-print: the file is named, and no print_state was observed yet
        // (a printer whose `print_stats` only reached us at the cancel).
        let printing = context(1_000_000_000, &path, 500);
        assert_eq!(printing.print_state, None);
        // `do_cancel`: file cleared and `cancelled` journaled together. This
        // is the newest context, and it names no file.
        let mut cancelled = context(2_000_000_000, &path, 0);
        cancelled.virtual_sdcard = None;
        cancelled.print_state = Some("cancelled".to_owned());
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(printing),
                WalRecord::Context(cancelled),
            ],
        );
        // The state does not describe the file we would be offering, so it
        // cannot settle the question.
        assert_eq!(super::last_print_state_for(&timeline_of(&dir), &path), None);
        let detection = detect(&dir, &dir.join("hb"), 1);
        assert!(
            matches!(detection, Detection::Pending(_)),
            "a fileless conclusive state must not delete the offer: {detection:?}"
        );
    }

    /// The reconstruction's timeline for a WAL directory, for tests that want
    /// to assert on a helper rather than only on the end-to-end verdict.
    fn timeline_of(dir: &Path) -> plr_reconstruct::WalTimeline {
        let merged = crate::scan::load_merged(dir).expect("merged");
        plr_reconstruct::ingest(&merged, None)
    }

    /// A context with no `print_state` at all (a printer without
    /// `[print_stats]`, or a pre-change WAL) must not be read as anything:
    /// `None` means "not observed", never "no print".
    #[test]
    fn an_unobserved_print_state_decides_nothing() {
        let dir = temp_dir("no-journaled-state");
        let gcode_path = unfinished_gcode(&dir);
        let ctx = context(1_000_000_000, gcode_path.to_str().unwrap(), 500);
        assert_eq!(ctx.print_state, None, "the helper must not set it");
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(ctx),
            ],
        );
        assert!(matches!(
            detect(&dir, &dir.join("hb"), 1),
            Detection::Pending(_)
        ));
    }

    // -----------------------------------------------------------------
    // D4: which verdicts clear the pending file
    // -----------------------------------------------------------------

    #[test]
    fn a_settled_nothing_is_distinguishable_from_an_inconclusive_one() {
        let dir = temp_dir("nothing-clears");
        // A print-less unclean stop is INCONCLUSIVE, not settled: the
        // context naming the print may simply have rotated out of the
        // segments detection reads.
        write_wal(
            &dir,
            &[
                WalRecord::Heartbeat(heartbeat(1_000_000_000, 42.0)),
                WalRecord::Context(Context {
                    virtual_sdcard: None,
                    ..context(1_000_000_000, "unused", 0)
                }),
            ],
        );
        let Detection::Nothing(no_offer) = detect(&dir, &dir.join("hb"), 1) else {
            panic!("expected Nothing");
        };
        assert!(
            no_offer.reason.contains("no print context"),
            "{}",
            no_offer.reason
        );
        assert!(
            no_offer.preserve_pending,
            "an absent context is indistinguishable from a rotated-out one"
        );
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
            frame_invalid: false,
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
