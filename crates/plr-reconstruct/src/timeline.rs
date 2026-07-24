//! WAL ingestion: turning a raw recovery scan plus the heartbeat-file
//! recovery into a validated, queryable [`WalTimeline`].
//!
//! Ingestion is **total**: any combination of records (including
//! hand-built scans with hostile values) produces a timeline plus
//! [`IngestNote`]s describing every anomaly. Missing *prerequisites*
//! (no heartbeat, no context) are not errors here — they become typed
//! errors in the stages that actually need them
//! ([`crate::window::compute_stop_window`],
//! [`crate::stopset::compute_stop_set`]), because a clean-shutdown WAL
//! legitimately needs neither.
//!
//! # What ingestion validates
//!
//! * Records with non-finite floats are dropped (the WAL writer refuses
//!   them, so their presence means the scan was not produced by the
//!   writer) and noted.
//! * Trapq segments are grouped by queue and kept ordered by
//!   `print_time`; out-of-order input is stably sorted and noted. Dwell
//!   gaps between segments are **preserved** — consumers must treat time
//!   discontinuities as legitimate (heating, waiting, homing).
//! * Stepper ranges are checked for per-stepper clock regressions
//!   (noted, not repaired: `t_b` takes a max, so regressions cannot
//!   corrupt the window).
//! * Lifecycle markers are interpreted: a terminal
//!   [`MarkerKind::CleanShutdown`] flags the timeline as needing no
//!   recovery; a tail [`MarkerKind::SocketLost`] without a subsequent
//!   `Resubscribed` is classification evidence; subscription gaps are
//!   noted for honest degradation.

use plr_wal::{
    Context, Heartbeat, HeartbeatRecovery, Marker, MarkerKind, RecoveryScan, ScanEnd, StepperRange,
    TrapqSegment, WalRecord,
};

/// One observation made during ingestion. Notes never abort ingestion;
/// they exist so downstream reporting can be honest about data quality.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IngestNote {
    /// Segments of `queue` arrived out of `print_time` order and were
    /// stably re-sorted.
    ReorderedTrapq {
        /// The affected motion queue.
        queue: String,
    },
    /// A stepper's dump ranges went backwards in `last_clock` between
    /// consecutive records — reordered or restarted MCU data.
    RegressedStepperClock {
        /// The affected stepper.
        stepper: String,
    },
    /// A record carrying NaN/infinity was dropped. The WAL writer
    /// refuses such records, so the scan did not come from it.
    NonFiniteRecordSkipped {
        /// Byte offset of the offending frame.
        wal_offset: u64,
    },
    /// A known observation gap: motion between the bounds was not
    /// captured, so WAL-derived candidates inside it may be missing.
    SubscriptionGap {
        /// Host-monotonic start of the gap (ns).
        start_mono_ns: u64,
        /// Host-monotonic end of the gap (ns).
        end_mono_ns: u64,
    },
    /// The Klipper API socket dropped at this time; motion after it was
    /// not observed until a `Resubscribed` marker (if any).
    SocketLoss {
        /// Host-monotonic time of the drop (ns).
        mono_ns: u64,
    },
    /// A `CleanShutdown` marker exists but motion records follow it, so
    /// it does not end the log and recovery proceeds.
    StaleCleanShutdownMarker,
    /// A marker written by a newer format revision was preserved as
    /// opaque and ignored.
    UnknownMarker {
        /// Host-monotonic time of the marker (ns).
        mono_ns: u64,
    },
    /// The scan ended in a way power loss does not produce (CRC
    /// mismatch of previously-durable bytes, foreign format, ...);
    /// treat the whole recovery with suspicion.
    UnexpectedScanEnd,
    /// The heartbeat file's other slot was torn — the expected state
    /// after power loss mid-rewrite, recorded for completeness.
    TornHeartbeatSlot,
    /// More than one distinct extruder queue contributed segments; the
    /// E interval unions them all.
    MultipleExtruderQueues,
}

/// A validated, grouped view of everything the durable WAL prefix says.
#[derive(Debug, Clone, PartialEq)]
pub struct WalTimeline {
    /// Toolhead (`queue == "toolhead"`) trapq segments, ordered by
    /// `print_time`, dwell gaps preserved.
    pub toolhead_segments: Vec<TrapqSegment>,
    /// Extruder (`queue` starting with `"extruder"`) trapq segments,
    /// ordered by `print_time`. The filament position rides in the X
    /// slot (see [`TrapqSegment`]).
    pub extruder_segments: Vec<TrapqSegment>,
    /// Segments of any other queue (manual steppers, ...), ordered by
    /// `print_time`; carried for completeness, unused by the stop set.
    pub other_segments: Vec<TrapqSegment>,
    /// All committed-step ranges, in append order.
    pub stepper_ranges: Vec<StepperRange>,
    /// All context snapshots, in append order (the last one is the
    /// anchor for forward simulation).
    pub contexts: Vec<Context>,
    /// All lifecycle markers, in append order.
    pub markers: Vec<Marker>,
    /// The newest finite heartbeat across the heartbeat file and every
    /// WAL heartbeat record (by `mono_ns`). `None` when no finite
    /// heartbeat exists anywhere.
    pub heartbeat: Option<Heartbeat>,
    /// `true` when a [`MarkerKind::CleanShutdown`] marker ends the log
    /// (no motion records after it): the print ended on purpose and no
    /// recovery is needed.
    pub clean_shutdown: bool,
    /// Set when the log's tail (after the last motion record) contains a
    /// [`MarkerKind::SocketLost`] with no later `Resubscribed`: the
    /// daemon outlived Klipper's API socket — classification evidence
    /// for a klippy shutdown with power retained.
    pub socket_lost_tail: Option<u64>,
    /// `mono_ns` of the newest motion record (trapq segment or stepper
    /// range); `None` when the WAL holds no motion at all.
    pub last_motion_mono_ns: u64,
    /// Whether any motion record exists (disambiguates
    /// `last_motion_mono_ns == 0`).
    pub has_motion: bool,
    /// Why the recovery scan stopped (propagated for classification).
    pub scan_end: ScanEnd,
    /// Everything unusual observed during ingestion.
    pub notes: Vec<IngestNote>,
}

impl WalTimeline {
    /// End of durable trapq knowledge: the maximum segment end time
    /// across the toolhead and extruder queues, or `None` when no finite
    /// segment exists. Rows are journaled as Klipper *plans* moves
    /// (up to ~1 s ahead of execution), so this generally exceeds the
    /// committed boundary `t_b` — see
    /// [`crate::stopset`] for why the stop set evaluates out to it.
    #[must_use]
    pub fn trapq_end_time(&self) -> Option<f64> {
        let end = self
            .toolhead_segments
            .iter()
            .chain(&self.extruder_segments)
            .map(TrapqSegment::end_time)
            .fold(f64::NEG_INFINITY, f64::max);
        end.is_finite().then_some(end)
    }
}

/// Builds a [`WalTimeline`] from a recovery scan and (optionally) the
/// recovered heartbeat file. Total: never fails, never panics; every
/// anomaly becomes an [`IngestNote`].
#[must_use]
#[allow(clippy::too_many_lines)] // one linear classification pass; splitting it would scatter the invariants
pub fn ingest(scan: &RecoveryScan, heartbeat: Option<&HeartbeatRecovery>) -> WalTimeline {
    let mut notes = Vec::new();
    let mut toolhead: Vec<TrapqSegment> = Vec::new();
    let mut extruder: Vec<TrapqSegment> = Vec::new();
    let mut other: Vec<TrapqSegment> = Vec::new();
    let mut stepper_ranges: Vec<StepperRange> = Vec::new();
    let mut contexts: Vec<Context> = Vec::new();
    let mut markers: Vec<Marker> = Vec::new();
    let mut wal_heartbeats: Vec<Heartbeat> = Vec::new();

    // Indices (in scan order) used to reason about what "ends" the log.
    let mut last_motion_idx: Option<usize> = None;
    let mut last_motion_mono: u64 = 0;
    let mut clean_marker_idx: Option<usize> = None;
    let mut socket_lost: Option<(usize, u64)> = None;
    let mut resubscribed_idx: Option<usize> = None;

    for (idx, scanned) in scan.records.iter().enumerate() {
        if !scanned.record.values_are_finite() {
            notes.push(IngestNote::NonFiniteRecordSkipped {
                wal_offset: scanned.offset,
            });
            continue;
        }
        match &scanned.record {
            WalRecord::TrapqSegment(seg) => {
                last_motion_idx = Some(idx);
                last_motion_mono = last_motion_mono.max(seg.mono_ns);
                if seg.queue == "toolhead" {
                    toolhead.push(seg.clone());
                } else if seg.queue.starts_with("extruder") {
                    extruder.push(seg.clone());
                } else {
                    other.push(seg.clone());
                }
            }
            WalRecord::StepperRange(range) => {
                last_motion_idx = Some(idx);
                last_motion_mono = last_motion_mono.max(range.mono_ns);
                stepper_ranges.push(range.clone());
            }
            WalRecord::Context(ctx) => contexts.push(ctx.clone()),
            WalRecord::Heartbeat(hb) => wal_heartbeats.push(*hb),
            WalRecord::Marker(marker) => {
                match marker.kind {
                    MarkerKind::CleanShutdown => clean_marker_idx = Some(idx),
                    MarkerKind::SocketLost => {
                        socket_lost = Some((idx, marker.mono_ns));
                        notes.push(IngestNote::SocketLoss {
                            mono_ns: marker.mono_ns,
                        });
                    }
                    MarkerKind::Resubscribed => resubscribed_idx = Some(idx),
                    MarkerKind::SubscriptionGap {
                        start_mono_ns,
                        end_mono_ns,
                    } => notes.push(IngestNote::SubscriptionGap {
                        start_mono_ns,
                        end_mono_ns,
                    }),
                    MarkerKind::Unknown => notes.push(IngestNote::UnknownMarker {
                        mono_ns: marker.mono_ns,
                    }),
                }
                markers.push(marker.clone());
            }
        }
    }

    for (queue, segments) in [
        ("toolhead", &mut toolhead),
        ("extruder", &mut extruder),
        ("other", &mut other),
    ] {
        ensure_sorted(queue, segments, &mut notes);
    }
    note_extruder_queue_mix(&extruder, &mut notes);
    note_stepper_clock_regressions(&stepper_ranges, &mut notes);

    // A clean shutdown only counts when nothing moved after the marker.
    let clean_shutdown = match (clean_marker_idx, last_motion_idx) {
        (Some(clean), Some(motion)) => {
            if clean > motion {
                true
            } else {
                notes.push(IngestNote::StaleCleanShutdownMarker);
                false
            }
        }
        (Some(_), None) => true,
        (None, _) => false,
    };

    // SocketLost is tail evidence only if nothing moved after it and no
    // Resubscribed follows it.
    let socket_lost_tail = socket_lost
        .filter(|(idx, _)| last_motion_idx.is_none_or(|m| *idx > m))
        .filter(|(idx, _)| resubscribed_idx.is_none_or(|r| r < *idx))
        .map(|(_, mono)| mono);

    if !scan.end.is_expected_after_power_loss() {
        notes.push(IngestNote::UnexpectedScanEnd);
    }
    if heartbeat.is_some_and(|hb| hb.torn.is_some()) {
        notes.push(IngestNote::TornHeartbeatSlot);
    }

    let best_heartbeat = heartbeat
        .map(|recovery| recovery.heartbeat)
        .into_iter()
        .chain(wal_heartbeats)
        .filter(Heartbeat::values_are_finite)
        .max_by_key(|hb| hb.mono_ns);

    WalTimeline {
        toolhead_segments: toolhead,
        extruder_segments: extruder,
        other_segments: other,
        stepper_ranges,
        contexts,
        markers,
        heartbeat: best_heartbeat,
        clean_shutdown,
        socket_lost_tail,
        last_motion_mono_ns: last_motion_mono,
        has_motion: last_motion_idx.is_some(),
        scan_end: scan.end.clone(),
        notes,
    }
}

/// Stable-sorts `segments` by `print_time` if (and only if) they are out
/// of order, noting the repair. NaN-free by construction (non-finite
/// records were dropped), but the comparator is total regardless.
fn ensure_sorted(queue: &str, segments: &mut [TrapqSegment], notes: &mut Vec<IngestNote>) {
    let ordered = segments
        .windows(2)
        .all(|pair| pair[0].print_time <= pair[1].print_time);
    if !ordered {
        segments.sort_by(|a, b| {
            a.print_time
                .partial_cmp(&b.print_time)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        notes.push(IngestNote::ReorderedTrapq {
            queue: queue.to_owned(),
        });
    }
}

/// Notes when more than one distinct extruder queue name is present.
fn note_extruder_queue_mix(extruder: &[TrapqSegment], notes: &mut Vec<IngestNote>) {
    let first = extruder.first().map(|seg| seg.queue.as_str());
    if extruder
        .iter()
        .any(|seg| first.is_some_and(|name| seg.queue != name))
    {
        notes.push(IngestNote::MultipleExtruderQueues);
    }
}

/// Notes per-stepper `last_clock` regressions between consecutive dump
/// ranges (append order).
fn note_stepper_clock_regressions(ranges: &[StepperRange], notes: &mut Vec<IngestNote>) {
    let mut flagged: Vec<&str> = Vec::new();
    for (i, range) in ranges.iter().enumerate() {
        let regressed = ranges[..i]
            .iter()
            .rev()
            .find(|prev| prev.stepper == range.stepper)
            .is_some_and(|prev| range.last_clock < prev.last_clock);
        if regressed && !flagged.contains(&range.stepper.as_str()) {
            flagged.push(&range.stepper);
            notes.push(IngestNote::RegressedStepperClock {
                stepper: range.stepper.clone(),
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use plr_wal::{
        HeartbeatRecovery, Marker, MarkerKind, RecoveryScan, ScanEnd, ScannedRecord, SlotError,
        SlotId, WalRecord,
    };

    use super::{ingest, IngestNote};
    use crate::testutil::{context_at, heartbeat_at, scan_of, stepper_range, trapq_segment};

    fn marker(mono_ns: u64, kind: MarkerKind) -> WalRecord {
        WalRecord::Marker(Marker { mono_ns, kind })
    }

    #[test]
    fn groups_records_by_kind_and_queue() {
        let scan = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 1_000)),
            WalRecord::TrapqSegment(trapq_segment("extruder", 10.0, 0.5, 1_001)),
            WalRecord::TrapqSegment(trapq_segment("manual_stepper lift", 10.0, 0.5, 1_002)),
            WalRecord::StepperRange(stepper_range("stepper_z", 10.2, 2_000)),
            WalRecord::Context(context_at(3_000, 128)),
            WalRecord::Heartbeat(heartbeat_at(4_000, 10.4)),
        ]);
        let timeline = ingest(&scan, None);
        assert_eq!(timeline.toolhead_segments.len(), 1);
        assert_eq!(timeline.extruder_segments.len(), 1);
        assert_eq!(timeline.other_segments.len(), 1);
        assert_eq!(timeline.stepper_ranges.len(), 1);
        assert_eq!(timeline.contexts.len(), 1);
        assert_eq!(timeline.heartbeat.map(|hb| hb.mono_ns), Some(4_000));
        assert!(timeline.has_motion);
        assert_eq!(timeline.last_motion_mono_ns, 2_000);
        assert!(!timeline.clean_shutdown);
        assert!(timeline.notes.is_empty());
    }

    #[test]
    fn trapq_end_time_spans_toolhead_and_extruder() {
        let scan = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 1)),
            WalRecord::TrapqSegment(trapq_segment("extruder", 11.0, 0.75, 2)),
        ]);
        let timeline = ingest(&scan, None);
        let end = timeline.trapq_end_time().unwrap();
        assert!((end - 11.75).abs() < 1e-12);

        let empty = ingest(&scan_of(vec![]), None);
        assert_eq!(empty.trapq_end_time(), None);
        assert!(!empty.has_motion);
    }

    #[test]
    fn out_of_order_segments_are_sorted_and_noted() {
        let scan = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 12.0, 0.5, 2)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 1)),
        ]);
        let timeline = ingest(&scan, None);
        assert!(
            timeline.toolhead_segments[0].print_time < timeline.toolhead_segments[1].print_time
        );
        assert!(timeline.notes.contains(&IngestNote::ReorderedTrapq {
            queue: "toolhead".to_owned()
        }));
    }

    #[test]
    fn non_finite_records_are_dropped_and_noted() {
        let mut seg = trapq_segment("toolhead", 10.0, 0.5, 1);
        seg.start_z = f64::NAN;
        let scan = scan_of(vec![WalRecord::TrapqSegment(seg)]);
        let timeline = ingest(&scan, None);
        assert!(timeline.toolhead_segments.is_empty());
        assert!(matches!(
            timeline.notes.as_slice(),
            [IngestNote::NonFiniteRecordSkipped { .. }]
        ));
        // A dropped record is not motion evidence.
        assert!(!timeline.has_motion);
    }

    #[test]
    fn clean_shutdown_requires_no_motion_after_marker() {
        let clean_end = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 1)),
            marker(2, MarkerKind::CleanShutdown),
        ]);
        assert!(ingest(&clean_end, None).clean_shutdown);

        let stale = scan_of(vec![
            marker(1, MarkerKind::CleanShutdown),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 2)),
        ]);
        let timeline = ingest(&stale, None);
        assert!(!timeline.clean_shutdown);
        assert!(timeline
            .notes
            .contains(&IngestNote::StaleCleanShutdownMarker));

        let no_motion = scan_of(vec![marker(1, MarkerKind::CleanShutdown)]);
        assert!(ingest(&no_motion, None).clean_shutdown);
    }

    #[test]
    fn socket_lost_tail_needs_no_motion_and_no_resubscribe_after() {
        let tail = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 1)),
            marker(9, MarkerKind::SocketLost),
        ]);
        assert_eq!(ingest(&tail, None).socket_lost_tail, Some(9));

        let resubscribed = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 1)),
            marker(9, MarkerKind::SocketLost),
            marker(10, MarkerKind::Resubscribed),
        ]);
        assert_eq!(ingest(&resubscribed, None).socket_lost_tail, None);

        let motion_after = scan_of(vec![
            marker(9, MarkerKind::SocketLost),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 10)),
        ]);
        assert_eq!(ingest(&motion_after, None).socket_lost_tail, None);

        // An *earlier* Resubscribed does not clear a later SocketLost.
        let lost_again = scan_of(vec![
            marker(5, MarkerKind::SocketLost),
            marker(6, MarkerKind::Resubscribed),
            marker(9, MarkerKind::SocketLost),
        ]);
        assert_eq!(ingest(&lost_again, None).socket_lost_tail, Some(9));
    }

    #[test]
    fn subscription_gap_and_unknown_markers_are_noted() {
        let scan = scan_of(vec![
            marker(
                1,
                MarkerKind::SubscriptionGap {
                    start_mono_ns: 100,
                    end_mono_ns: 200,
                },
            ),
            marker(2, MarkerKind::Unknown),
        ]);
        let timeline = ingest(&scan, None);
        assert!(timeline.notes.contains(&IngestNote::SubscriptionGap {
            start_mono_ns: 100,
            end_mono_ns: 200
        }));
        assert!(timeline
            .notes
            .contains(&IngestNote::UnknownMarker { mono_ns: 2 }));
        assert_eq!(timeline.markers.len(), 2);
    }

    #[test]
    fn heartbeat_selection_prefers_newest_finite_across_sources() {
        let file_recovery = HeartbeatRecovery {
            heartbeat: heartbeat_at(5_000, 12.0),
            slot: SlotId::A,
            torn: Some((SlotId::B, SlotError::CrcMismatch)),
        };
        let scan = scan_of(vec![
            WalRecord::Heartbeat(heartbeat_at(4_000, 11.0)),
            WalRecord::Heartbeat(heartbeat_at(6_000, 13.0)),
        ]);
        let timeline = ingest(&scan, Some(&file_recovery));
        // WAL heartbeat at 6_000 is newer than the file's 5_000.
        assert_eq!(timeline.heartbeat.map(|hb| hb.mono_ns), Some(6_000));
        assert!(timeline.notes.contains(&IngestNote::TornHeartbeatSlot));

        // Non-finite heartbeats are excluded from selection.
        let mut bad = heartbeat_at(9_000, 14.0);
        bad.print_time = f64::NAN;
        let scan = scan_of(vec![WalRecord::Heartbeat(bad)]);
        let timeline = ingest(&scan, Some(&file_recovery));
        assert_eq!(timeline.heartbeat.map(|hb| hb.mono_ns), Some(5_000));
    }

    #[test]
    fn stepper_clock_regression_is_noted_once_per_stepper() {
        let scan = scan_of(vec![
            WalRecord::StepperRange(stepper_range("stepper_z", 10.0, 1)),
            WalRecord::StepperRange(stepper_range("stepper_z", 9.0, 2)),
            WalRecord::StepperRange(stepper_range("stepper_z", 8.0, 3)),
            WalRecord::StepperRange(stepper_range("stepper_x", 10.0, 4)),
        ]);
        let timeline = ingest(&scan, None);
        let regressions = timeline
            .notes
            .iter()
            .filter(|n| matches!(n, IngestNote::RegressedStepperClock { .. }))
            .count();
        assert_eq!(regressions, 1);
    }

    #[test]
    fn multiple_extruder_queues_are_noted() {
        let scan = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("extruder", 10.0, 0.5, 1)),
            WalRecord::TrapqSegment(trapq_segment("extruder1", 10.5, 0.5, 2)),
        ]);
        let timeline = ingest(&scan, None);
        assert!(timeline.notes.contains(&IngestNote::MultipleExtruderQueues));
    }

    #[test]
    fn unexpected_scan_end_is_noted() {
        let mut scan = scan_of(vec![]);
        scan.end = ScanEnd::FrameCrcMismatch;
        let timeline = ingest(&scan, None);
        assert!(timeline.notes.contains(&IngestNote::UnexpectedScanEnd));
        assert_eq!(timeline.scan_end, ScanEnd::FrameCrcMismatch);
    }

    #[test]
    fn empty_scan_yields_empty_timeline() {
        let timeline = ingest(&scan_of(vec![]), None);
        assert!(timeline.toolhead_segments.is_empty());
        assert!(timeline.contexts.is_empty());
        assert_eq!(timeline.heartbeat, None);
        assert!(!timeline.clean_shutdown);
        assert!(!timeline.has_motion);
    }

    #[test]
    fn scan_offsets_do_not_matter_but_order_does() {
        // Ingest works purely off record order, not byte offsets.
        let records = vec![
            ScannedRecord {
                offset: 999,
                record: marker(1, MarkerKind::CleanShutdown),
            },
            ScannedRecord {
                offset: 32,
                record: WalRecord::TrapqSegment(trapq_segment("toolhead", 1.0, 0.1, 2)),
            },
        ];
        let scan = RecoveryScan {
            header: None,
            records,
            truncation_offset: 0,
            end: ScanEnd::CleanEof,
        };
        let timeline = ingest(&scan, None);
        assert!(!timeline.clean_shutdown, "motion after the marker");
    }
}
