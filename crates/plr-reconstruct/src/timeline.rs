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
//!   `Resubscribed` is classification evidence; subscription gaps and
//!   [`MarkerKind::ExclusionUpdateLost`] are noted for honest
//!   degradation; a tail [`MarkerKind::RecorderStopped`] records that the
//!   *daemon* stopped on purpose, which changes nothing about the
//!   reconstruction and only licenses suppressing an announcement (see
//!   [`WalTimeline::recorder_stopped_tail`]). Markers are kept in append
//!   order in
//!   [`WalTimeline::markers`] so later stages can ask *when* an event
//!   happened relative to other records.
//! * Every finite heartbeat is kept, sorted, in
//!   [`WalTimeline::heartbeats`] — not just the newest — because the
//!   *continuity* of that stream is the only proof the WAL writer was
//!   running across a span in which it journaled nothing else.

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
    /// The daemon journaled that a `Context` carrying an exclude-object
    /// **change** was dropped under WAL backpressure: an operator
    /// cancellation may be missing from the log. See
    /// [`crate::exclude`] for how this defeats conclusiveness.
    ExclusionUpdateLost {
        /// Host-monotonic time of the dropped update (ns).
        mono_ns: u64,
    },
    /// The daemon journaled that it was shutting down gracefully. The
    /// print's fate after this instant is unknown; see
    /// [`WalTimeline::recorder_stopped_tail`] for exactly what that
    /// licenses (suppressing an announcement, nothing more).
    RecorderStopped {
        /// Host-monotonic time of the graceful stop (ns).
        mono_ns: u64,
    },
    /// The recorder declared it was entering a reduced-cadence idle
    /// regime ([`plr_wal::MarkerKind::RecordingQuiescent`]). A sparse
    /// heartbeat-record stream after this point is deliberate, not a
    /// stalled recorder.
    ///
    /// Purely a data-quality note: it changes no reconstruction verdict.
    /// The recorder only enters the regime when no recoverable print is in
    /// progress (a print keeps full cadence through its dwells and pauses),
    /// so a quiet span never overlaps a stop-window coverage span — the
    /// liveness reasoning in [`crate::exclude`] is never applied across
    /// one. Recorded so `plrd scan` can report an idle tail honestly and so
    /// the "quiet never overlaps a coverage span" invariant is checkable in
    /// the log rather than assumed.
    RecordingQuiescent {
        /// Host-monotonic time the idle regime began (ns).
        mono_ns: u64,
    },
    /// A GPIO edge signalled that the DC rail began failing at this
    /// host-monotonic time ([`plr_wal::MarkerKind::PowerFailing`]). See
    /// [`WalTimeline::power_failing_tail`] for what a *tail* occurrence
    /// licenses; a non-tail one (motion follows it) is a spurious edge and
    /// this note is the only trace it leaves.
    PowerFailing {
        /// Host-monotonic time the power-fail edge was journaled (ns).
        mono_ns: u64,
    },
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
    /// Every finite heartbeat sample (heartbeat file plus WAL records),
    /// sorted ascending by `mono_ns` and deduplicated on that key.
    ///
    /// Heartbeats are written on the WAL writer's own timer,
    /// independently of context records, which makes them the only
    /// evidence that the **WAL writer thread** was running and able to
    /// append across a span in which it journaled nothing else. See
    /// [`crate::exclude`] for exactly what that does and does not
    /// prove, and for the markers that cover the rest.
    pub heartbeats: Vec<Heartbeat>,
    /// `true` when a [`MarkerKind::CleanShutdown`] marker ends the log
    /// (no motion records after it): the print ended on purpose and no
    /// recovery is needed.
    pub clean_shutdown: bool,
    /// Set when the log's tail (after the last motion record) contains a
    /// [`MarkerKind::SocketLost`] with no later `Resubscribed`: the
    /// daemon outlived Klipper's API socket — classification evidence
    /// for a klippy shutdown with power retained.
    pub socket_lost_tail: Option<u64>,
    /// `mono_ns` of a [`MarkerKind::RecorderStopped`] that ends the log
    /// (no motion record after it): **the recorder stopped on purpose, so
    /// this log says nothing about how the print ended.**
    ///
    /// Deliberately *not* folded into [`clean_shutdown`](Self::clean_shutdown)
    /// and deliberately *not* a [`crate::window::CrashClass`]: a graceful
    /// daemon stop is not a deliberate end of the *print*, and the print
    /// may well have died afterwards with nothing left running to record
    /// it. The reconstruction is therefore computed exactly as it would be
    /// without this marker, and every downstream consumer keeps its full
    /// ability to plan and execute a recovery.
    ///
    /// What it licenses is one thing only: **suppressing an unsolicited
    /// announcement.** A daemon that finds this at the end of the previous
    /// session's log must not tell the operator "your print died" — the
    /// honest statement is "the recorder stopped; the print's fate is
    /// unknown" — and it must not retract a pending-recovery offer either,
    /// because it has learned nothing that contradicts one.
    pub recorder_stopped_tail: Option<u64>,
    /// `mono_ns` of the newest motion record (trapq segment or stepper
    /// range); `None` when the WAL holds no motion at all.
    pub last_motion_mono_ns: Option<u64>,
    /// Why the recovery scan stopped (propagated for classification).
    pub scan_end: ScanEnd,
    /// Everything unusual observed during ingestion.
    pub notes: Vec<IngestNote>,
}

impl WalTimeline {
    /// `mono_ns` of a **tail** [`MarkerKind::PowerFailing`] — a
    /// hold-up-backed GPIO power-fail edge with no motion recorded after
    /// it — or `None`.
    ///
    /// This is the exact-T fact the rest of the reconstruction is built to
    /// infer indirectly: the physical cut lies in
    /// `[t, t + hold_up_margin]` on the host-monotonic axis, where `t` is
    /// the returned value. The marker time is a *lower* bound on the cut
    /// (power was still up when it was journaled, so motion can continue
    /// for the hold-up margin afterward), so it can only ever tighten an
    /// *upper* bound on the stop, never exclude the true stop. The frontier
    /// cap ([`crate::stopset::compute_stop_set`]) consumes it as an
    /// independent, tighter upper bound on `t_cut`; classification
    /// ([`crate::window`]) reads it as decisive power-loss evidence.
    ///
    /// # Tail semantics, and why a spurious edge is dropped
    ///
    /// A `PowerFailing` marker counts only when **no motion record
    /// postdates it** — the same "tail marker" rule
    /// [`recorder_stopped_tail`](Self::recorder_stopped_tail) uses, applied
    /// here on the monotonic axis: the marker qualifies iff its `mono_ns`
    /// is at or after the newest motion record's
    /// ([`last_motion_mono_ns`](Self::last_motion_mono_ns)). An EMI blip
    /// that passes the watcher's debounce and journals a false marker while
    /// the machine keeps printing leaves motion with a *later* `mono_ns`,
    /// so this returns `None` and the false marker is discarded — it can
    /// never narrow the window. The [`IngestNote::PowerFailing`] note still
    /// records that the edge fired, so a spurious signal is visible in
    /// `plrd scan` without being load-bearing.
    ///
    /// # Why a method, not a field
    ///
    /// [`WalTimeline`] exposes public fields and is struct-literal
    /// constructed outside this crate, so adding a field would be a
    /// breaking change; the value is derived on demand from the markers and
    /// the newest-motion timestamp instead, both of which ingest already
    /// stores.
    #[must_use]
    pub fn power_failing_tail(&self) -> Option<u64> {
        self.markers
            .iter()
            .filter(|m| m.kind == MarkerKind::PowerFailing)
            .map(|m| m.mono_ns)
            .filter(|&mono| self.last_motion_mono_ns.is_none_or(|last| mono >= last))
            .max()
    }

    /// End of durable trapq knowledge: the maximum segment end time
    /// across the toolhead and extruder queues, or `None` when no finite
    /// segment exists. Rows are journaled as Klipper *plans* moves
    /// (up to ~1 s ahead of execution), so this generally exceeds the
    /// committed boundary `t_b` — see
    /// [`crate::stopset`] for why the stop set evaluates out to it.
    #[must_use]
    pub fn trapq_end_time(&self) -> Option<f64> {
        self.trapq_end_time_journaled_by(u64::MAX)
    }

    /// End of durable trapq knowledge counting **only rows journaled at
    /// or before `mono_ns`**.
    ///
    /// Trapq rows are journaled when the daemon receives them, i.e. when
    /// Klipper *plans* the move, which is also when the g-code line
    /// producing it has been processed. So for a [`Context`] snapshot
    /// with capture time `t` and processing frontier `F`, this value at
    /// `mono_ns = t` is an upper bound on the execution time of all
    /// motion up to `F` — and, because every line up to `F` was
    /// processed by `t` (and so had its row journaled by `t`, modulo
    /// batching delay that only lowers this value), it is also a **sound
    /// lower bound** on when the machine finishes the motion preceding
    /// `F`.
    ///
    /// [`crate::stopset`] uses that to place the forward extension's
    /// first simulated move on the print-time axis: a frontier stalled
    /// on one long move can sit seconds *behind* the snapshot's own
    /// capture time, and sizing the extension horizon from the capture
    /// time instead under-simulates by exactly that much.
    #[must_use]
    pub fn trapq_end_time_journaled_by(&self, mono_ns: u64) -> Option<f64> {
        let end = self
            .toolhead_segments
            .iter()
            .chain(&self.extruder_segments)
            .filter(|seg| seg.mono_ns <= mono_ns)
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
    let mut last_motion_mono: Option<u64> = None;
    let mut clean_marker_idx: Option<usize> = None;
    let mut socket_lost: Option<(usize, u64)> = None;
    let mut resubscribed_idx: Option<usize> = None;
    let mut recorder_stopped: Option<(usize, u64)> = None;

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
                last_motion_mono = Some(last_motion_mono.unwrap_or(0).max(seg.mono_ns));
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
                last_motion_mono = Some(last_motion_mono.unwrap_or(0).max(range.mono_ns));
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
                    MarkerKind::ExclusionUpdateLost => {
                        notes.push(IngestNote::ExclusionUpdateLost {
                            mono_ns: marker.mono_ns,
                        });
                    }
                    // Recorded, never allowed to change the
                    // reconstruction: see `recorder_stopped_tail`.
                    MarkerKind::RecorderStopped => {
                        recorder_stopped = Some((idx, marker.mono_ns));
                        notes.push(IngestNote::RecorderStopped {
                            mono_ns: marker.mono_ns,
                        });
                    }
                    // A cadence declaration only: never touches any
                    // reconstruction verdict — see `IngestNote::RecordingQuiescent`.
                    MarkerKind::RecordingQuiescent => {
                        notes.push(IngestNote::RecordingQuiescent {
                            mono_ns: marker.mono_ns,
                        });
                    }
                    // Recorded here; promoted to `power_failing_tail` below
                    // only if no motion follows it (a spurious edge with
                    // motion after is discarded — see the field docs).
                    // Noted here; whether it counts as a *tail* fact is
                    // derived on demand by `WalTimeline::power_failing_tail`
                    // (it must be a method, not a field — `WalTimeline` has
                    // public fields and out-of-crate struct literals, so a
                    // new field would be a breaking change).
                    MarkerKind::PowerFailing => {
                        notes.push(IngestNote::PowerFailing {
                            mono_ns: marker.mono_ns,
                        });
                    }
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

    // RecorderStopped is tail evidence on the same terms: motion after it
    // means the recorder came back and kept working, so the marker no
    // longer describes how this log ends.
    let recorder_stopped_tail = recorder_stopped
        .filter(|(idx, _)| last_motion_idx.is_none_or(|m| *idx > m))
        .map(|(_, mono)| mono);

    if !scan.end.is_expected_after_power_loss() {
        notes.push(IngestNote::UnexpectedScanEnd);
    }
    if heartbeat.is_some_and(|hb| hb.torn.is_some()) {
        notes.push(IngestNote::TornHeartbeatSlot);
    }

    let mut all_heartbeats: Vec<Heartbeat> = heartbeat
        .map(|recovery| recovery.heartbeat)
        .into_iter()
        .chain(wal_heartbeats)
        .filter(Heartbeat::values_are_finite)
        .collect();
    all_heartbeats.sort_by_key(|hb| hb.mono_ns);
    all_heartbeats.dedup_by_key(|hb| hb.mono_ns);
    let best_heartbeat = all_heartbeats.last().copied();

    WalTimeline {
        toolhead_segments: toolhead,
        extruder_segments: extruder,
        other_segments: other,
        stepper_ranges,
        contexts,
        markers,
        heartbeat: best_heartbeat,
        heartbeats: all_heartbeats,
        clean_shutdown,
        socket_lost_tail,
        recorder_stopped_tail,
        last_motion_mono_ns: last_motion_mono,
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
        assert_eq!(timeline.last_motion_mono_ns, Some(2_000));
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
        assert_eq!(empty.last_motion_mono_ns, None);
    }

    #[test]
    fn trapq_end_time_can_be_restricted_to_rows_journaled_by_a_time() {
        // Two rows: one journaled at mono 1 ending at pt 10.5, one
        // journaled at mono 100 ending at pt 20.5. Restricting to
        // mono <= 1 must ignore the later row entirely — this is what
        // places a stalled processing frontier on the print-time axis
        // (see crate::stopset::extension_start_time).
        let scan = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 1)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 20.0, 0.5, 100)),
        ]);
        let timeline = ingest(&scan, None);
        let all = timeline.trapq_end_time().unwrap();
        assert!((all - 20.5).abs() < 1e-12);
        let early = timeline.trapq_end_time_journaled_by(1).unwrap();
        assert!((early - 10.5).abs() < 1e-12);
        assert!((timeline.trapq_end_time_journaled_by(99).unwrap() - 10.5).abs() < 1e-12);
        assert!((timeline.trapq_end_time_journaled_by(100).unwrap() - 20.5).abs() < 1e-12);
        assert_eq!(timeline.trapq_end_time_journaled_by(0), None);
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
        assert_eq!(timeline.last_motion_mono_ns, None);
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

    /// `RecorderStopped` is tail evidence on the same terms as
    /// `SocketLost`, and it must NOT make the log read as clean: a
    /// graceful daemon stop says nothing about how the print ended.
    #[test]
    fn recorder_stopped_is_tail_evidence_but_never_a_clean_shutdown() {
        let tail = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 1.0, 0.5, 1)),
            marker(9, MarkerKind::RecorderStopped),
        ]);
        let timeline = ingest(&tail, None);
        assert_eq!(timeline.recorder_stopped_tail, Some(9));
        assert!(
            !timeline.clean_shutdown,
            "a recorder stop is not a deliberate END OF PRINT"
        );
        assert!(timeline
            .notes
            .iter()
            .any(|n| matches!(n, IngestNote::RecorderStopped { mono_ns: 9 })));

        // Motion after it means the recorder came back: no longer tail
        // evidence.
        let superseded = scan_of(vec![
            marker(9, MarkerKind::RecorderStopped),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 2.0, 0.5, 10)),
        ]);
        assert_eq!(ingest(&superseded, None).recorder_stopped_tail, None);

        // It does not disturb the socket-loss classification either.
        let both = scan_of(vec![
            marker(8, MarkerKind::SocketLost),
            marker(9, MarkerKind::RecorderStopped),
        ]);
        let timeline = ingest(&both, None);
        assert_eq!(timeline.socket_lost_tail, Some(8));
        assert_eq!(timeline.recorder_stopped_tail, Some(9));
    }

    /// A genuine tail `PowerFailing` marker (no motion after it) is the
    /// exact-T fact; a spurious one (motion postdates it) is dropped as
    /// a tail fact but still noted. This is the honest-wide-never-narrow
    /// property the reviewer named.
    #[test]
    fn power_failing_tail_is_a_tail_fact_and_a_spurious_edge_is_dropped() {
        // Genuine: edge at mono 9, newest motion at mono 1 → tail.
        let genuine = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 1.0, 0.5, 1)),
            marker(9, MarkerKind::PowerFailing),
        ]);
        let timeline = ingest(&genuine, None);
        assert_eq!(timeline.power_failing_tail(), Some(9));
        // It is a data-quality NOTE regardless, so a spurious one is still
        // visible in `plrd scan`.
        assert!(timeline
            .notes
            .iter()
            .any(|n| matches!(n, IngestNote::PowerFailing { mono_ns: 9 })));
        // ...and it is not a clean shutdown.
        assert!(!timeline.clean_shutdown);

        // Spurious: an EMI blip journals the marker at mono 5, then the
        // machine keeps printing (motion at mono 10). Motion postdates the
        // marker, so it is NOT a tail fact — dropped, never narrowing.
        let spurious = scan_of(vec![
            marker(5, MarkerKind::PowerFailing),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 2.0, 0.5, 10)),
        ]);
        let timeline = ingest(&spurious, None);
        assert_eq!(
            timeline.power_failing_tail(),
            None,
            "a false marker with motion after it must not be a tail fact"
        );
        // But the edge is still noted (the contradiction is visible).
        assert!(timeline
            .notes
            .iter()
            .any(|n| matches!(n, IngestNote::PowerFailing { mono_ns: 5 })));

        // No motion at all: an edge at mono 9 is still a tail fact.
        let no_motion = scan_of(vec![marker(9, MarkerKind::PowerFailing)]);
        assert_eq!(ingest(&no_motion, None).power_failing_tail(), Some(9));

        // No power-fail marker: None.
        let none = scan_of(vec![WalRecord::TrapqSegment(trapq_segment(
            "toolhead", 1.0, 0.5, 1,
        ))]);
        assert_eq!(ingest(&none, None).power_failing_tail(), None);
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

    /// The idle-throttle marker is a pure data-quality note: it is
    /// recorded, but it must not read as a clean shutdown, must not become
    /// recorder-stopped tail evidence, and must not touch the crash
    /// classification. It is the recorded fact that a following sparse
    /// heartbeat stream is deliberate — nothing more.
    #[test]
    fn recording_quiescent_marker_is_a_benign_note() {
        let scan = scan_of(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 1.0, 0.5, 1)),
            marker(9, MarkerKind::RecordingQuiescent),
        ]);
        let timeline = ingest(&scan, None);
        assert!(timeline
            .notes
            .contains(&IngestNote::RecordingQuiescent { mono_ns: 9 }));
        assert!(
            !timeline.clean_shutdown,
            "an idle-cadence declaration is not a print end"
        );
        assert_eq!(
            timeline.recorder_stopped_tail, None,
            "it is not a recorder stop"
        );
        assert_eq!(timeline.socket_lost_tail, None);
        // The marker is preserved verbatim for the forensic report.
        assert_eq!(timeline.markers.len(), 1);
        assert_eq!(timeline.markers[0].kind, MarkerKind::RecordingQuiescent);
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
        assert_eq!(timeline.last_motion_mono_ns, None);
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
