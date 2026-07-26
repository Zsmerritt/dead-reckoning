//! Epoch partitioning: isolating the single crash epoch from a WAL
//! record stream that may span multiple boots and firmware sessions.
//!
//! # Why reconstruction must partition
//!
//! A recovery scan is the concatenation of every WAL segment in the
//! directory (`plrd::scan::merge_scans`). After a power failure *every*
//! real capture spans at least one reboot boundary, and long-lived
//! printers accumulate firmware restarts too. Two independent clocks
//! break across those boundaries, and the downstream stages
//! ([`crate::window`], [`crate::stopset`]) trust both clocks globally:
//!
//! * **Host-monotonic epoch (reboot).** A reboot resets
//!   `CLOCK_MONOTONIC` to ~0 and a fresh `plrd` process opens a new
//!   segment. Every record carries a `mono_ns` stamped from
//!   `now_mono_ns()` at append time (`plrd::client` reads messages
//!   serially, so within one boot `mono_ns` is non-decreasing in append
//!   order). Merging across a reboot therefore scrambles "newest": the
//!   heartbeat with the largest raw `mono_ns` can belong to an *older*
//!   boot, so `t_a` and the anchor context can come from different
//!   epochs.
//! * **Print-time epoch (firmware restart).** A klippy/firmware restart
//!   resets Klipper's `print_time` axis to ~0 while the host clock keeps
//!   running. `plrd::client` journals a [`MarkerKind::SocketLost`] the
//!   instant the API socket drops (immediate durability) and calls
//!   `Recorder::reset_session`, which clears `latest_print_time` and the
//!   append frontier (`plrd::convert::Recorder::reset_session`). The
//!   pre-restart idle session can hold trapq rows at `print_time`
//!   *larger* than the actual print's (an idle klippy that ran for a day
//!   accumulates `print_time`), so a `t_b`/`wal_eval_end` taken as a raw
//!   `max` over the merged stream is poisoned by the older session.
//!
//! Both defects are real in the reference capture: the full-directory
//! reconstruction produced a 102,000 s stop window (a pre-restart idle
//! row at `print_time ≈ 104,492 s` set `wal_eval_end`) over a bed-sized
//! XY region.
//!
//! # The delimiters
//!
//! | Boundary | Signal |
//! |----------|--------|
//! | Reboot | a **new segment** (frame offset restarts low in the merged stream) whose `mono_ns` regressed ≥ [`REBOOT_MONO_REGRESSION_NS`] |
//! | Firmware restart | a [`MarkerKind::SocketLost`] marker (`plrd::client::run_client` journals it immediate-sync, then `reset_session`) |
//!
//! A reboot always opens a fresh WAL segment (a new process), so in the
//! merged record stream it coincides with the per-segment frame offset
//! restarting low. Gating the reboot test on that offset regression means
//! it can only fire at a real segment boundary — never on a within-segment
//! `mono_ns` step, which the writer never produces in append order but a
//! hand-built record stream can. The firmware-restart delimiter is
//! intra-segment and does not consult the offset.
//!
//! ## Why there is no print-time "backstop" — and what that costs
//!
//! An earlier revision added a second firmware-restart signal: a
//! `print_time` regression, meant to catch a `SocketLost` marker lost to
//! a torn tail. It was **deleted** because it cannot be specified safely
//! and buys nothing:
//!
//! * **It has no sound threshold.** The obvious "reset" quantity to test
//!   is the newest heartbeat's `print_time`, but that field is
//!   `latest_print_time` — a running *max* that folds in
//!   `toolhead.print_time`, the *planning* frontier
//!   (`plrd::convert` on the status path; [`crate::window`] documents it
//!   as planned-ahead). When Klipper plans through a dwell the frontier
//!   jumps several seconds and the batched trapq rows that fill the gap
//!   arrive *afterwards*, starting at pre-jump times. Comparing that
//!   running max against a lagging row is a structurally unbounded
//!   cross-source "regression" that no constant threshold survives: on
//!   the reference capture it false-fired eight times, every time
//!   re-exceeding the old max on the very next row, with zero true
//!   positives.
//! * **A lost marker coincides with the crash anyway.** `SocketLost` is
//!   written through the marker path, which — unlike droppable `Context`
//!   records under WAL backpressure — is **never dropped** (that
//!   asymmetry is the whole reason [`MarkerKind::ExclusionUpdateLost`]
//!   exists). The only way a real restart's `SocketLost` is absent is a
//!   torn tail *at* that marker; and a torn tail can only be the **newest
//!   segment's** (rotation fsyncs a segment before opening its
//!   successor). Records cannot follow the tear, so there is no
//!   post-restart session to isolate — the durable log simply ends in the
//!   pre-restart session, which is already the newest epoch this module
//!   selects. See the widened hazard pin in the tests.
//!
//! # The safety asymmetry
//!
//! A **missed** boundary poisons the window with another epoch's
//! evidence — the bug this module fixes; the window inflates but with
//! meaningless states the machine was never in during *this* crash. With
//! the backstop gone, both remaining delimiters fire only on hard,
//! producer-guaranteed evidence (a real new segment with a real clock
//! reset; a durable marker), so a **spurious** split cannot arise from
//! reading normal within-session behaviour — the class of over-partition
//! that could have dropped the *newest* crash evidence (a frontier-jump
//! landing after the last context, making the tail a "newer
//! non-printing" fragment) is removed at the source, not merely bounded.
//!
//! # What is selected
//!
//! [`select_crash_epoch`] returns the **newest epoch that was printing**
//! (holds a [`plr_wal::Context`] whose `virtual_sdcard` names a
//! non-empty file). That skips a post-crash idle boot (segment written
//! after power was restored but before the next print) while still
//! isolating the crash from every older epoch. Evidence from other
//! epochs is not down-weighted — it is not evidence about this crash at
//! all, so it is removed before ingestion.

use plr_wal::{MarkerKind, RecoveryScan, ScanEnd, ScannedRecord, WalRecord};

/// A backwards jump in `mono_ns` of at least this many nanoseconds marks
/// a reboot. Within one boot `mono_ns` is stamped monotonically at
/// append time, so any genuine regression is the full previous uptime
/// (hours) — orders of magnitude above this floor. The floor exists only
/// to refuse to split on a hypothetical sub-second reordering that the
/// producer does not actually create; it never misses a reboot.
pub const REBOOT_MONO_REGRESSION_NS: u64 = 1_000_000_000;

/// Why two adjacent epochs are separated.
#[derive(Debug, Clone, PartialEq)]
pub enum EpochBoundaryKind {
    /// `mono_ns` regressed: the host rebooted (or the monotonic clock
    /// was otherwise reset) between the two records.
    HostReboot {
        /// Running maximum `mono_ns` of the epoch that ended.
        last_mono_ns: u64,
        /// `mono_ns` of the first record of the new epoch.
        next_mono_ns: u64,
    },
    /// A [`MarkerKind::SocketLost`] marker ended the prior epoch: the
    /// Klipper API socket dropped and the session (and its `print_time`
    /// axis) was reset.
    FirmwareRestart {
        /// `mono_ns` of the `SocketLost` marker.
        socket_lost_mono_ns: u64,
    },
}

/// One epoch: a maximal run of `scan.records` sharing a host-monotonic
/// boot and a Klipper print-time session.
#[derive(Debug, Clone, PartialEq)]
pub struct EpochSpan {
    /// Inclusive start index into `scan.records`.
    pub start: usize,
    /// Exclusive end index into `scan.records`.
    pub end: usize,
    /// Why this epoch is separated from its predecessor; `None` for the
    /// first epoch.
    pub boundary_before: Option<EpochBoundaryKind>,
    /// The epoch holds a context whose `virtual_sdcard` names a
    /// non-empty print file — i.e. it was printing.
    pub printing: bool,
    /// The epoch holds at least one context record.
    pub has_context: bool,
    /// The epoch holds at least one motion record (trapq segment or
    /// stepper range).
    pub has_motion: bool,
    /// Smallest `mono_ns` in the epoch.
    pub min_mono_ns: u64,
    /// Largest `mono_ns` in the epoch.
    pub max_mono_ns: u64,
}

impl EpochSpan {
    /// Number of records in the epoch.
    #[must_use]
    pub fn len(&self) -> usize {
        self.end - self.start
    }

    /// Whether the epoch is empty (never true for partitions this module
    /// produces; present for completeness).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.start == self.end
    }

    /// `true` when `mono_ns` falls inside this epoch's monotonic span.
    /// Used to admit out-of-band inputs (the heartbeat file, the
    /// receive-seq sidecar) only when they belong to the crash epoch.
    #[must_use]
    pub fn contains_mono(&self, mono_ns: u64) -> bool {
        self.min_mono_ns <= mono_ns && mono_ns <= self.max_mono_ns
    }
}

/// The crash epoch plus a summary of what partitioning discarded.
#[derive(Debug, Clone, PartialEq)]
pub struct CrashEpochSelection {
    /// Every epoch found, oldest first.
    pub epochs: Vec<EpochSpan>,
    /// Index into [`Self::epochs`] of the selected crash epoch;
    /// `None` only for an empty scan.
    pub selected: Option<usize>,
}

impl CrashEpochSelection {
    /// The selected crash epoch, if any.
    #[must_use]
    pub fn crash_epoch(&self) -> Option<&EpochSpan> {
        self.selected.map(|i| &self.epochs[i])
    }

    /// `true` when more than one epoch was present, so partitioning
    /// actually discarded records.
    #[must_use]
    pub fn partitioned(&self) -> bool {
        self.epochs.len() > 1
    }

    /// Count of epochs older than the selected one (their evidence was
    /// discarded as belonging to earlier boots/sessions).
    #[must_use]
    pub fn discarded_older(&self) -> usize {
        self.selected.unwrap_or(0)
    }

    /// Count of epochs newer than the selected one (e.g. a post-crash
    /// idle boot whose segment was written before the next print).
    #[must_use]
    pub fn discarded_newer(&self) -> usize {
        match self.selected {
            Some(i) => self.epochs.len() - i - 1,
            None => 0,
        }
    }

    /// The boundary kinds separating the epochs (oldest first). Reporting
    /// only.
    #[must_use]
    pub fn boundaries(&self) -> Vec<EpochBoundaryKind> {
        self.epochs
            .iter()
            .filter_map(|e| e.boundary_before.clone())
            .collect()
    }

    /// Builds the recovery scan narrowed to the crash epoch.
    ///
    /// The merged scan's tail metadata (`end`, `truncation_offset`)
    /// describes only the **newest** segment (`merge_scans` keeps the last
    /// segment's and discards the rest), so when the crash epoch is *not*
    /// the newest partition that metadata belongs to a *later* epoch and
    /// cannot be attributed to the crash epoch. The crash epoch's own tail
    /// state is not recoverable after the merge, so `end` is set to
    /// `CleanEof` as a neutral placeholder. This is safe, not merely
    /// convenient: whether the crash epoch's last segment ended torn or
    /// clean is not load-bearing here — the crash is evidenced by the
    /// succeeding epoch boundary (reboot/restart), and both tear states
    /// map to the same [`crate::window::CrashClass::HostDeathOrPowerLoss`]
    /// handling; `torn_tail` only annotates the report. The newest
    /// partition keeps the original `end`, so a genuine power-loss torn
    /// tail is preserved where it *is* the crash evidence.
    #[must_use]
    pub fn narrow(&self, scan: &RecoveryScan) -> RecoveryScan {
        let Some(i) = self.selected else {
            return scan.clone();
        };
        let epoch = &self.epochs[i];
        let is_newest = i + 1 == self.epochs.len();
        RecoveryScan {
            header: scan.header.clone(),
            records: scan.records[epoch.start..epoch.end].to_vec(),
            // `truncation_offset` only drives resume of the newest
            // segment; for a superseded epoch it is not consulted.
            truncation_offset: scan.truncation_offset,
            end: if is_newest {
                scan.end.clone()
            } else {
                ScanEnd::CleanEof
            },
        }
    }
}

/// `true` when the record is a context that names a non-empty print
/// file — the signal that the epoch was actively printing.
fn is_printing_context(record: &WalRecord) -> bool {
    matches!(
        record,
        WalRecord::Context(c)
            if c.virtual_sdcard.as_ref().is_some_and(|v| !v.file_path.is_empty())
    )
}

/// `true` when the record is motion (trapq segment or stepper range).
fn is_motion(record: &WalRecord) -> bool {
    matches!(
        record,
        WalRecord::TrapqSegment(_) | WalRecord::StepperRange(_)
    )
}

/// Partitions a record stream into epochs, oldest first. Total: never
/// panics; a stream with no boundary yields exactly one epoch, an empty
/// stream yields none.
#[must_use]
pub fn partition(records: &[ScannedRecord]) -> Vec<EpochSpan> {
    let mut epochs: Vec<EpochSpan> = Vec::new();
    if records.is_empty() {
        return epochs;
    }

    // Mutable accumulators for the epoch under construction.
    let mut start = 0usize;
    let mut boundary_before: Option<EpochBoundaryKind> = None;
    let mut printing = false;
    let mut has_context = false;
    let mut has_motion = false;
    let mut min_mono = u64::MAX;
    let mut max_mono = 0u64;
    // A `SocketLost` seen in the current epoch, pending the first record
    // of the next session that closes the epoch at that record.
    let mut pending_socket_lost: Option<u64> = None;
    // Frame offset of the previous record. A reboot always opens a NEW
    // segment (fresh process), which in the merged stream shows as the
    // frame offset restarting at the segment-header length — so a reboot
    // is only credible where the offset did NOT advance. Frame offsets
    // strictly increase within a segment (every frame has positive
    // length), so `offset <= prev` is false within a segment and true at
    // any segment boundary, even one following a single-record segment
    // (both offsets equal the header length). This refuses to read a
    // within-segment mono step (which a real writer never produces, but a
    // hand-built fixture can) as a boot boundary.
    let mut prev_offset: Option<u64> = None;

    for (idx, scanned) in records.iter().enumerate() {
        let record = &scanned.record;
        let mono = record.mono_ns();
        let new_segment = prev_offset.is_some_and(|prev| scanned.offset <= prev);

        // Decide whether a NEW epoch begins at `idx`. A reboot (a hard
        // clock reset at a segment boundary) takes precedence over a
        // pending socket-loss restart: the new boot's session starts
        // fresh regardless. Both fire only on producer-guaranteed
        // evidence — a real new segment with a real clock reset, or a
        // durable marker.
        let boundary = if idx > start && new_segment && mono + REBOOT_MONO_REGRESSION_NS <= max_mono
        {
            Some(EpochBoundaryKind::HostReboot {
                last_mono_ns: max_mono,
                next_mono_ns: mono,
            })
        } else {
            pending_socket_lost.map(|socket_lost_mono_ns| EpochBoundaryKind::FirmwareRestart {
                socket_lost_mono_ns,
            })
        };

        if let Some(kind) = boundary {
            epochs.push(EpochSpan {
                start,
                end: idx,
                boundary_before,
                printing,
                has_context,
                has_motion,
                min_mono_ns: min_mono,
                max_mono_ns: max_mono,
            });
            // Reset accumulators for the new epoch.
            start = idx;
            boundary_before = Some(kind);
            printing = false;
            has_context = false;
            has_motion = false;
            min_mono = u64::MAX;
            max_mono = 0;
            pending_socket_lost = None;
        }

        // Fold the record into the current epoch.
        min_mono = min_mono.min(mono);
        max_mono = max_mono.max(mono);
        printing |= is_printing_context(record);
        has_context |= matches!(record, WalRecord::Context(_));
        has_motion |= is_motion(record);
        if matches!(
            record,
            WalRecord::Marker(m) if m.kind == MarkerKind::SocketLost
        ) {
            // The marker belongs to the epoch that just lost its socket;
            // the *next* record opens the fresh session. A trailing
            // `SocketLost` (no record after) therefore stays in this
            // epoch, preserving the socket-lost-tail classification.
            pending_socket_lost = Some(mono);
        }
        prev_offset = Some(scanned.offset);
    }

    epochs.push(EpochSpan {
        start,
        end: records.len(),
        boundary_before,
        printing,
        has_context,
        has_motion,
        min_mono_ns: min_mono,
        max_mono_ns: max_mono,
    });
    epochs
}

/// Partitions `scan` and selects the crash epoch: the newest epoch that
/// was printing, falling back to the newest with any context, then the
/// newest with any record. The fallbacks keep single-epoch and
/// heartbeat-only scans behaving exactly as before partitioning existed,
/// so the downstream stages still surface their own typed errors
/// (`NoContext`, `NoHeartbeat`).
#[must_use]
pub fn select_crash_epoch(scan: &RecoveryScan) -> CrashEpochSelection {
    let epochs = partition(&scan.records);
    let selected = newest_by(&epochs, |e| e.printing)
        .or_else(|| newest_by(&epochs, |e| e.has_context))
        .or_else(|| epochs.len().checked_sub(1));
    CrashEpochSelection { epochs, selected }
}

/// Index of the newest (last) epoch satisfying `pred`.
fn newest_by(epochs: &[EpochSpan], pred: impl Fn(&EpochSpan) -> bool) -> Option<usize> {
    epochs.iter().rposition(pred)
}

#[cfg(test)]
mod tests {
    use plr_wal::{Marker, MarkerKind, RecoveryScan, ScanEnd, ScannedRecord, WalRecord};

    use super::{partition, select_crash_epoch, EpochBoundaryKind, REBOOT_MONO_REGRESSION_NS};
    use crate::testutil::{
        context_at, heartbeat_at, scan_of_segments, stepper_range, trapq_segment,
    };

    /// Builds a single-segment scan at synthetic offsets (mirrors
    /// `testutil::scan_of` but local so this module's tests are
    /// self-contained about tail metadata). Use [`scan_of_segments`] when a
    /// reboot (segment) boundary is under test.
    fn scan(records: Vec<WalRecord>) -> RecoveryScan {
        scan_ending(records, ScanEnd::CleanEof)
    }

    fn scan_ending(records: Vec<WalRecord>, end: ScanEnd) -> RecoveryScan {
        let records = records
            .into_iter()
            .enumerate()
            .map(|(i, record)| ScannedRecord {
                offset: 32 + (i as u64) * 64,
                record,
            })
            .collect();
        RecoveryScan {
            header: None,
            records,
            truncation_offset: 0,
            end,
        }
    }

    fn marker(mono_ns: u64, kind: MarkerKind) -> WalRecord {
        WalRecord::Marker(Marker { mono_ns, kind })
    }

    const S: u64 = 1_000_000_000; // one second in ns

    #[test]
    fn single_epoch_stream_is_not_partitioned() {
        let sel = select_crash_epoch(&scan(vec![
            WalRecord::Heartbeat(heartbeat_at(10 * S, 10.0)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 10 * S)),
            WalRecord::Context(context_at(11 * S, 128)),
        ]));
        assert_eq!(sel.epochs.len(), 1);
        assert!(!sel.partitioned());
        assert_eq!(sel.selected, Some(0));
        assert_eq!(sel.discarded_older(), 0);
        assert_eq!(sel.discarded_newer(), 0);
    }

    #[test]
    fn reboot_splits_on_mono_regression_and_selects_newest_print() {
        // Old boot printed at high mono (one segment); new boot printed at
        // low mono (the next segment, offsets restart low).
        let full = scan_of_segments(vec![
            vec![
                WalRecord::Heartbeat(heartbeat_at(50_000 * S, 100.0)),
                WalRecord::Context(context_at(50_000 * S, 100)),
                WalRecord::TrapqSegment(trapq_segment("toolhead", 100.0, 0.5, 50_000 * S)),
            ],
            vec![
                WalRecord::Heartbeat(heartbeat_at(20 * S, 5.0)),
                WalRecord::Context(context_at(20 * S, 40)),
                WalRecord::TrapqSegment(trapq_segment("toolhead", 5.0, 0.5, 20 * S)),
            ],
        ]);
        let sel = select_crash_epoch(&full);
        assert_eq!(sel.epochs.len(), 2);
        assert_eq!(sel.selected, Some(1));
        assert_eq!(sel.discarded_older(), 1);
        assert!(matches!(
            sel.epochs[1].boundary_before,
            Some(EpochBoundaryKind::HostReboot { .. })
        ));
        // The narrowed scan holds only the new boot's three records.
        let narrowed = sel.narrow(&full);
        assert_eq!(narrowed.records.len(), 3);
        for r in &narrowed.records {
            assert!(r.record.mono_ns() < 1000 * S, "old-boot record leaked");
        }
    }

    #[test]
    fn reboot_after_a_single_record_segment_is_still_caught() {
        // Edge: the segment before the reboot held exactly one record, so
        // its only frame offset equals the next segment's first (the
        // header length). `offset <= prev` still flags the boundary.
        let sel = select_crash_epoch(&scan_of_segments(vec![
            vec![WalRecord::Heartbeat(heartbeat_at(50_000 * S, 100.0))],
            vec![
                WalRecord::Heartbeat(heartbeat_at(20 * S, 5.0)),
                WalRecord::Context(context_at(20 * S, 40)),
            ],
        ]));
        assert_eq!(
            sel.epochs.len(),
            2,
            "reboot after a 1-record segment must split"
        );
        assert!(matches!(
            sel.epochs[1].boundary_before,
            Some(EpochBoundaryKind::HostReboot { .. })
        ));
    }

    #[test]
    fn a_within_segment_mono_step_is_not_a_reboot() {
        // Same segment (offsets increase), but mono jumps far backwards
        // between records. A real writer never does this in append order;
        // it must NOT be read as a reboot without a new segment.
        let sel = select_crash_epoch(&scan(vec![
            WalRecord::Heartbeat(heartbeat_at(50_000 * S, 100.0)),
            WalRecord::Context(context_at(50_000 * S, 100)),
            // mono collapses within the same segment:
            WalRecord::Context(context_at(20 * S, 40)),
        ]));
        assert_eq!(
            sel.epochs.len(),
            1,
            "within-segment mono step is not a reboot"
        );
    }

    #[test]
    fn firmware_restart_splits_on_socket_lost_marker() {
        // One boot, a klippy restart delimited by SocketLost/Resubscribed.
        // Pre-restart idle print_time is HUGE; the print is small.
        let sel = select_crash_epoch(&scan(vec![
            WalRecord::Heartbeat(heartbeat_at(100 * S, 104_000.0)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 104_000.0, 0.5, 100 * S)),
            WalRecord::Context(context_at(100 * S, 25_000)),
            marker(110 * S, MarkerKind::SocketLost),
            marker(120 * S, MarkerKind::Resubscribed),
            // New session: print_time reset near zero.
            WalRecord::Heartbeat(heartbeat_at(130 * S, 5.0)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 5.0, 0.5, 130 * S)),
            WalRecord::Context(context_at(130 * S, 200)),
        ]));
        assert_eq!(sel.epochs.len(), 2);
        assert_eq!(sel.selected, Some(1));
        assert!(matches!(
            sel.epochs[1].boundary_before,
            Some(EpochBoundaryKind::FirmwareRestart { .. })
        ));
        // The Resubscribed marker opens the new epoch.
        assert_eq!(sel.epochs[1].start, 4);
    }

    #[test]
    fn a_planning_frontier_jump_declares_no_boundary() {
        // The exact shape that made the deleted print-time backstop
        // false-fire on the reference capture (probe: 2549.739 -> 2542.960,
        // then re-exceed): Klipper plans through a dwell, so the heartbeat's
        // latest_print_time (a running max folding in the toolhead PLANNING
        // frontier) jumps far ahead; the batched trapq rows that fill the
        // gap arrive AFTERWARDS, starting at pre-jump times, then the next
        // row re-passes the old max. One session, one segment: the
        // partition must see exactly ONE epoch.
        let sel = select_crash_epoch(&scan(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 2549.7, 0.04, 100 * S)),
            // Heartbeat carries the jumped planning frontier.
            WalRecord::Heartbeat(heartbeat_at(101 * S, 2549.739)),
            // Batched rows fill the planned gap, starting BEFORE the jump.
            WalRecord::TrapqSegment(trapq_segment("toolhead", 2542.940, 0.02, 102 * S)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 2542.960, 7.84, 102 * S)),
            WalRecord::Context(context_at(102 * S, 200)),
        ]));
        assert_eq!(
            sel.epochs.len(),
            1,
            "a planning-frontier jump is not a restart"
        );
        assert!(!sel.partitioned());
    }

    #[test]
    fn post_crash_idle_boot_is_skipped_for_the_printing_epoch() {
        // Crash epoch (printing) then a fresh idle boot with no print
        // file: the printing epoch must be selected, not the newest.
        let sel = select_crash_epoch(&scan_of_segments(vec![
            // Segment 0: the crashed print.
            vec![
                WalRecord::Heartbeat(heartbeat_at(60_000 * S, 200.0)),
                WalRecord::Context(context_at(60_000 * S, 5_000)),
                WalRecord::TrapqSegment(trapq_segment("toolhead", 200.0, 0.5, 60_000 * S)),
            ],
            // Segment 1: post-crash reboot, idle (context names no file).
            vec![
                WalRecord::Heartbeat(heartbeat_at(30 * S, 1.0)),
                WalRecord::Context({
                    let mut c = context_at(30 * S, 0);
                    c.virtual_sdcard = None;
                    c
                }),
            ],
        ]));
        assert_eq!(sel.epochs.len(), 2);
        assert_eq!(sel.selected, Some(0), "must pick the printing epoch");
        assert_eq!(sel.discarded_newer(), 1);
        assert!(sel.epochs[0].printing);
        assert!(!sel.epochs[1].printing);
    }

    #[test]
    fn trailing_socket_lost_stays_in_its_epoch() {
        // A klippy shutdown at the tail: SocketLost with nothing after.
        // It must NOT spawn a new (empty) epoch — the marker is the
        // classification signal for THIS epoch.
        let sel = select_crash_epoch(&scan(vec![
            WalRecord::Heartbeat(heartbeat_at(100 * S, 10.0)),
            WalRecord::Context(context_at(100 * S, 200)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 100 * S)),
            marker(110 * S, MarkerKind::SocketLost),
        ]));
        assert_eq!(sel.epochs.len(), 1);
        assert_eq!(sel.epochs[0].end, 4);
    }

    #[test]
    fn narrow_resets_tail_when_a_later_epoch_supersedes_it() {
        // Selected epoch is not the newest -> end becomes CleanEof even
        // if the original scan ended torn (the torn tail belongs to the
        // later epoch, not the crash epoch).
        let mut full = scan_of_segments(vec![
            vec![
                WalRecord::Heartbeat(heartbeat_at(60_000 * S, 200.0)),
                WalRecord::Context(context_at(60_000 * S, 5_000)),
                WalRecord::TrapqSegment(trapq_segment("toolhead", 200.0, 0.5, 60_000 * S)),
            ],
            vec![
                WalRecord::Heartbeat(heartbeat_at(30 * S, 1.0)),
                WalRecord::Context({
                    let mut c = context_at(30 * S, 0);
                    c.virtual_sdcard = None;
                    c
                }),
            ],
        ]);
        full.end = ScanEnd::TruncatedPayload;
        let sel = select_crash_epoch(&full);
        assert_eq!(sel.selected, Some(0));
        let narrowed = sel.narrow(&full);
        assert_eq!(narrowed.end, ScanEnd::CleanEof);
        assert_eq!(narrowed.records.len(), 3);
    }

    #[test]
    fn newest_epoch_keeps_a_torn_tail() {
        let full = scan_ending(
            vec![
                WalRecord::Heartbeat(heartbeat_at(100 * S, 10.0)),
                WalRecord::Context(context_at(100 * S, 200)),
                WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 100 * S)),
            ],
            ScanEnd::TruncatedPayload,
        );
        let sel = select_crash_epoch(&full);
        let narrowed = sel.narrow(&full);
        assert_eq!(narrowed.end, ScanEnd::TruncatedPayload);
    }

    #[test]
    fn empty_scan_selects_nothing_and_narrows_to_itself() {
        let sel = select_crash_epoch(&scan(vec![]));
        assert!(sel.epochs.is_empty());
        assert_eq!(sel.selected, None);
        assert_eq!(sel.narrow(&scan(vec![])).records.len(), 0);
    }

    #[test]
    fn heartbeat_only_scan_falls_back_to_the_whole_stream() {
        // No context anywhere: reconstruction's NoContext error must
        // still be reachable, so we must not narrow away the heartbeat.
        let sel = select_crash_epoch(&scan(vec![WalRecord::Heartbeat(heartbeat_at(
            10 * S,
            10.0,
        ))]));
        assert_eq!(sel.epochs.len(), 1);
        assert_eq!(sel.selected, Some(0));
    }

    #[test]
    fn reboot_threshold_ignores_sub_second_noise_but_catches_real_resets() {
        // A new segment whose mono regressed by LESS than the floor is not
        // a reboot (e.g. an ordinary same-boot segment rotation with clock
        // jitter): the two segments stay one epoch.
        let below = select_crash_epoch(&scan_of_segments(vec![
            vec![
                WalRecord::Heartbeat(heartbeat_at(100 * S, 10.0)),
                WalRecord::Context(context_at(100 * S, 10)),
            ],
            vec![WalRecord::Context(context_at(
                100 * S - (REBOOT_MONO_REGRESSION_NS - 1),
                10,
            ))],
        ]));
        assert_eq!(below.epochs.len(), 1);
        // A new segment whose mono regressed by the floor splits.
        let at = select_crash_epoch(&scan_of_segments(vec![
            vec![
                WalRecord::Heartbeat(heartbeat_at(100 * S, 10.0)),
                WalRecord::Context(context_at(100 * S, 10)),
            ],
            vec![WalRecord::Context(context_at(
                100 * S - REBOOT_MONO_REGRESSION_NS,
                10,
            ))],
        ]));
        assert_eq!(at.epochs.len(), 2);
    }

    #[test]
    fn stepper_only_motion_counts_toward_the_epoch_but_not_printing() {
        // A homing move after reboot (stepper motion, no print file) must
        // not be picked over an older printing epoch.
        let sel = select_crash_epoch(&scan_of_segments(vec![
            vec![
                WalRecord::Heartbeat(heartbeat_at(60_000 * S, 200.0)),
                WalRecord::Context(context_at(60_000 * S, 5_000)),
                WalRecord::TrapqSegment(trapq_segment("toolhead", 200.0, 0.5, 60_000 * S)),
            ],
            vec![
                WalRecord::Heartbeat(heartbeat_at(30 * S, 0.5)),
                WalRecord::StepperRange(stepper_range("stepper_z", 1.0, 30 * S)),
            ],
        ]));
        // The stepper's segment regressed mono -> new epoch, motion but not
        // printing.
        assert_eq!(sel.epochs.len(), 2);
        assert!(sel.epochs[1].has_motion);
        assert!(!sel.epochs[1].printing);
        assert_eq!(sel.selected, Some(0));
    }

    #[test]
    fn partition_is_total_on_marker_only_streams() {
        let epochs = partition(
            &scan(vec![
                marker(1, MarkerKind::Resubscribed),
                marker(2, MarkerKind::Unknown),
            ])
            .records,
        );
        assert_eq!(epochs.len(), 1);
        assert!(!epochs[0].printing);
    }
}
