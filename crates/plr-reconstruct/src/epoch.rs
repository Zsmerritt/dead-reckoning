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
//! # The delimiters (and which is authoritative)
//!
//! | Boundary | Primary signal | Backstop |
//! |----------|----------------|----------|
//! | Reboot   | a **new segment** (frame offset restarts low in the merged stream) whose `mono_ns` regressed ≥ [`REBOOT_MONO_REGRESSION_NS`] | — (a reboot journals no marker: the daemon died, and the next process starts with `lost_after_subscribe == false`, so no `Resubscribed` either — `plrd::client::run_client`) |
//! | Firmware restart | [`MarkerKind::SocketLost`] marker (`plrd::client::run_client` journals it with `SyncPolicy` immediate, then `reset_session`) | `print_time` regression ≥ [`PRINT_TIME_RESET_MIN_REGRESSION_S`] in a motion/heartbeat row (catches a torn or lost marker) |
//!
//! A reboot always opens a fresh WAL segment (a new process), so in the
//! merged record stream it coincides with the per-segment frame offset
//! restarting low. Gating the reboot test on that offset regression means
//! it can only fire at a real segment boundary — never on a within-segment
//! `mono_ns` step, which the writer never produces in append order but a
//! hand-built record stream can. Firmware-restart delimiters, by
//! contrast, are intra-segment and do not consult the offset.
//!
//! The `SocketLost` marker is the reliable firmware-restart delimiter
//! because it is produced by exactly the code path that resets the
//! print-time axis and is journaled durably before any post-restart
//! record. The `print_time`-regression backstop exists only for the case
//! where that marker was itself lost to a torn tail.
//!
//! # The safety asymmetry
//!
//! A **missed** boundary poisons the window with another epoch's
//! evidence — the bug this module fixes; the window inflates but with
//! meaningless states the machine was never in during *this* crash. A
//! **spurious** boundary (over-partition) merely drops genuine
//! crash-epoch evidence from the head of the epoch, which widens the
//! forward-simulated set — the safe direction. Every threshold here is
//! therefore chosen to never miss a real boundary, accepting that it may
//! occasionally split one session in two.
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

/// A drop in `print_time` of at least this many seconds, with the host
/// clock continuous and no delimiting `SocketLost` marker, is treated as
/// a firmware restart whose marker was lost. A real restart resets
/// `print_time` to ~0, a drop of the entire accumulated print time
/// (many seconds at least). Ordinary within-session non-monotonicity is
/// bounded by Klipper's lookahead reordering across the toolhead and
/// extruder queues (sub-second); this threshold sits well above it so
/// the backstop cannot fire on a single healthy session.
pub const PRINT_TIME_RESET_MIN_REGRESSION_S: f64 = 5.0;

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
    /// `print_time` regressed across motion/heartbeat rows with the host
    /// clock continuous and no `SocketLost` marker: a firmware restart
    /// whose delimiting marker was lost. Backstop delimiter.
    PrintTimeReset {
        /// Running maximum `print_time` of the epoch that ended.
        last_print_time: f64,
        /// `print_time` of the first record of the new epoch.
        next_print_time: f64,
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

    /// Builds the recovery scan narrowed to the crash epoch. When the
    /// crash epoch is *not* the newest partition (a later epoch, e.g. a
    /// post-crash boot, exists), the scan's tail metadata is reset to a
    /// clean end: an epoch that a later epoch supersedes was durably
    /// closed before the next one began (the same invariant
    /// `plrd::scan` enforces — a non-newest segment must end cleanly),
    /// so its crash is evidenced by the following reboot, not by a torn
    /// tail. The newest partition keeps the original end (a genuine torn
    /// tail is preserved).
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

/// The `print_time`-domain frontier value a record carries, if any, for
/// the firmware-restart backstop.
///
/// Only the signals the producer maintains AS the print-time frontier
/// count: trapq segment end times (each planned move) and heartbeat
/// `print_time` (`plrd::convert`'s `latest_print_time`, which
/// `reset_session` zeroes on a restart). Stepper `last_step_time`
/// deliberately does **not** — a stepper dump reports already-committed
/// steps and lags the frontier by the batching + step-generation delay,
/// so treating a lagging dump as a print-time high-water would read
/// ordinary lag as a reset. `Context::print_time` is not yet written by
/// the producer but is consulted so the delimiter stays correct if it
/// ever is.
fn record_print_time(record: &WalRecord) -> Option<f64> {
    let value = match record {
        WalRecord::TrapqSegment(t) => t.end_time(),
        WalRecord::Heartbeat(h) => h.print_time,
        WalRecord::Context(c) => return c.print_time.filter(|v| v.is_finite()),
        WalRecord::StepperRange(_) | WalRecord::Marker(_) => return None,
    };
    value.is_finite().then_some(value)
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
    let mut max_print_time = f64::NEG_INFINITY;
    // A `SocketLost` seen in the current epoch, pending the first record
    // of the next session that closes the epoch at that record.
    let mut pending_socket_lost: Option<u64> = None;
    // Frame offset of the previous record. A reboot always opens a NEW
    // segment (fresh process), which in the merged stream shows as the
    // frame offset restarting low — so a reboot is only credible where
    // the offset regressed. This refuses to read a within-segment
    // mono step (which a real writer never produces, but a hand-built
    // fixture can) as a boot boundary.
    let mut prev_offset: Option<u64> = None;

    for (idx, scanned) in records.iter().enumerate() {
        let record = &scanned.record;
        let mono = record.mono_ns();
        let pt = record_print_time(record);
        let new_segment = prev_offset.is_some_and(|prev| scanned.offset < prev);

        // Decide whether a NEW epoch begins at `idx`. Precedence: reboot
        // (a hard clock reset at a segment boundary) first, then a pending
        // socket-loss restart, then the print-time backstop. A reboot
        // subsumes the other two: the new boot's session and print_time
        // start fresh regardless.
        let boundary = if idx > start && new_segment && mono + REBOOT_MONO_REGRESSION_NS <= max_mono
        {
            Some(EpochBoundaryKind::HostReboot {
                last_mono_ns: max_mono,
                next_mono_ns: mono,
            })
        } else if let Some(socket_lost_mono_ns) = pending_socket_lost {
            Some(EpochBoundaryKind::FirmwareRestart {
                socket_lost_mono_ns,
            })
        } else if let Some(pt) = pt {
            (max_print_time.is_finite() && pt + PRINT_TIME_RESET_MIN_REGRESSION_S <= max_print_time)
                .then_some(EpochBoundaryKind::PrintTimeReset {
                    last_print_time: max_print_time,
                    next_print_time: pt,
                })
        } else {
            None
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
            max_print_time = f64::NEG_INFINITY;
            pending_socket_lost = None;
        }

        // Fold the record into the current epoch.
        min_mono = min_mono.min(mono);
        max_mono = max_mono.max(mono);
        if let Some(pt) = pt {
            max_print_time = max_print_time.max(pt);
        }
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

    use super::{
        partition, select_crash_epoch, EpochBoundaryKind, PRINT_TIME_RESET_MIN_REGRESSION_S,
        REBOOT_MONO_REGRESSION_NS,
    };
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
    fn firmware_restart_backstop_fires_without_a_marker() {
        // A torn/lost SocketLost: only a print_time regression remains,
        // the host clock is continuous. The backstop MUST still split.
        let drop = PRINT_TIME_RESET_MIN_REGRESSION_S + 1.0;
        let sel = select_crash_epoch(&scan(vec![
            WalRecord::Heartbeat(heartbeat_at(100 * S, 100.0)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 100.0, 0.5, 100 * S)),
            WalRecord::Context(context_at(100 * S, 25_000)),
            // print_time collapses; mono keeps climbing; no marker.
            WalRecord::TrapqSegment(trapq_segment("toolhead", 100.0 - drop, 0.5, 110 * S)),
            WalRecord::Context(context_at(110 * S, 200)),
        ]));
        assert_eq!(sel.epochs.len(), 2);
        assert_eq!(sel.selected, Some(1));
        assert!(matches!(
            sel.epochs[1].boundary_before,
            Some(EpochBoundaryKind::PrintTimeReset { .. })
        ));
    }

    #[test]
    fn a_healthy_session_reordering_does_not_false_split() {
        // Sub-threshold print_time non-monotonicity (queue interleaving)
        // must NOT be read as a restart.
        let jitter = PRINT_TIME_RESET_MIN_REGRESSION_S - 1.0;
        let sel = select_crash_epoch(&scan(vec![
            WalRecord::Heartbeat(heartbeat_at(100 * S, 100.0)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 100.0, 0.5, 100 * S)),
            WalRecord::TrapqSegment(trapq_segment("extruder", 100.0 - jitter, 0.5, 101 * S)),
            WalRecord::Context(context_at(101 * S, 200)),
        ]));
        assert_eq!(sel.epochs.len(), 1, "queue jitter must not split");
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
