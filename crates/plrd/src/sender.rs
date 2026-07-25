//! The channel between the async socket side and the sync WAL thread,
//! and the drop policy applied when it fills.
//!
//! # Why a bounded channel with drops
//!
//! Klipper disconnects clients whose socket send buffer stays blocked
//! ("Closing unresponsive client", `klippy/webhooks.py`), so the socket
//! reader must **never** block on the WAL thread. If the disk stalls and
//! the bounded channel fills, motion records are dropped — and the drop
//! itself is journaled: the first successful send after a drop is
//! preceded by a [`MarkerKind::SubscriptionGap`] marker covering
//! `[first_drop, resume]`, so reconstruction sees an honest observation
//! gap instead of silently missing motion.
//!
//! Lifecycle markers ([`WalSender::marker`]) are never dropped: when the
//! channel is full they queue in an in-process outbox and flush before
//! any later send. They are rare (order: a handful per print), so the
//! outbox is bounded by event count, not by time.
//!
//! # Exclude-object changes: droppable record, undroppable evidence
//!
//! A `Context` carrying an operator's object cancellation is still a
//! record, and records are droppable — making it block would hand
//! Klipper a reason to disconnect us, which is worse. What is *not*
//! droppable is the knowledge that it was lost: when such a context is
//! dropped, [`WalSender`] queues a
//! [`MarkerKind::ExclusionUpdateLost`] marker in the same never-dropped
//! outbox. Reconstruction then has hard evidence and refuses to treat
//! the surviving excluded set as authoritative, instead of silently
//! resuming a part the operator had cancelled.
//!
//! Detection is local to this module: the sender remembers the excluded
//! name set it last successfully handed over, and treats a context as
//! carrying an exclusion change when the set differs or the context
//! re-journals definitions. At most one loss marker is queued per
//! lost-then-recovered episode, so a long stall cannot grow the outbox.

use std::collections::VecDeque;
use std::sync::mpsc::{SyncSender, TrySendError};

use plr_wal::{Marker, MarkerKind, WalRecord};

/// When the WAL thread must make an appended record durable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncPolicy {
    /// Durable at the next batch-cadence `fdatasync` (default 0.5 s).
    /// Motion records: their loss window is what reconstruction is built
    /// to bound.
    Batched,
    /// `fdatasync` immediately after the append. Markers and context
    /// records: rare, small, and each one changes what recovery may do.
    Immediate,
}

/// Heartbeat payload computed on the async side; the WAL thread owns the
/// sequence number, timestamps, and `wal_offset`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeartbeatData {
    /// Latest print time known from motion data (seconds).
    pub print_time: f64,
    /// Host-monotonic time (ns) of the `estimated_print_time` sample.
    pub est_sample_mono_ns: u64,
    /// Klipper `estimated_print_time` at that instant (seconds).
    pub est_sample_print_time: f64,
}

/// Commands consumed by the WAL thread.
// `Append` dwarfs the other variants, but it is also >99% of the traffic
// on this channel: boxing the record would add a heap allocation to the
// hot path to shrink the rare variants, a strict loss. Deliberate.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum WalCmd {
    /// Append a record with the given durability.
    Append {
        /// The record to journal.
        record: WalRecord,
        /// When it must become durable.
        sync: SyncPolicy,
    },
    /// Update (or clear, with `None`) the data the 10 Hz heartbeat is
    /// built from. `None` pauses heartbeats: no liveness claim without a
    /// live correlation sample.
    Heartbeat(Option<HeartbeatData>),
    /// Persist a widened `receive_seq` observation to the sidecar file.
    ReceiveSeq {
        /// Host-monotonic time (ns) of the observation.
        mono_ns: u64,
        /// The widened, non-decreasing counter value.
        widened: u64,
    },
    /// Final `fdatasync` and clean thread exit.
    Shutdown,
}

/// The WAL thread hung up (it only does so on fatal I/O errors); the
/// daemon cannot continue.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("WAL writer thread is gone; durability cannot be guaranteed")]
pub struct WalGone;

/// Async-side handle implementing the drop policy described in the
/// module docs.
#[derive(Debug)]
pub struct WalSender {
    tx: SyncSender<WalCmd>,
    /// `mono_ns` of the first record dropped since the last successful
    /// record send; open observation gap.
    gap_start: Option<u64>,
    /// Markers waiting for channel space. Never dropped.
    outbox: VecDeque<Marker>,
    /// Excluded-object name set of the last exclude-bearing context that
    /// was successfully handed to the WAL thread.
    last_excluded: Option<Vec<String>>,
    /// A [`MarkerKind::ExclusionUpdateLost`] has been queued and no
    /// exclude-bearing context has landed since; suppresses duplicates
    /// while the channel stays jammed.
    exclusion_loss_pending: bool,
}

impl WalSender {
    /// Wraps the sending half of the WAL channel.
    #[must_use]
    pub fn new(tx: SyncSender<WalCmd>) -> Self {
        Self {
            tx,
            gap_start: None,
            outbox: VecDeque::new(),
            last_excluded: None,
            exclusion_loss_pending: false,
        }
    }

    /// Offers a record. On a full channel the record is dropped and the
    /// gap is tracked; the caller is never blocked.
    ///
    /// When the dropped record carried an exclude-object **change**, a
    /// [`MarkerKind::ExclusionUpdateLost`] marker is additionally queued
    /// in the never-dropped outbox — see the module docs.
    pub fn record(
        &mut self,
        record: WalRecord,
        sync: SyncPolicy,
        mono_ns: u64,
    ) -> Result<(), WalGone> {
        let exclusion = exclude_fingerprint(&record);
        let changes_exclusion = exclusion.as_ref().is_some_and(|(excluded, redefines)| {
            *redefines || self.last_excluded.as_ref() != Some(excluded)
        });

        self.flush_outbox()?;
        if !self.outbox.is_empty() {
            // Markers are still queued; preserve marker-before-record
            // ordering by treating the channel as full for this record.
            self.drop_record(mono_ns, changes_exclusion)?;
            return Ok(());
        }
        if self.gap_start.is_some() && !self.close_gap(mono_ns)? {
            // No room for the gap marker means no room for the record
            // either; the gap keeps extending.
            self.drop_record(mono_ns, changes_exclusion)?;
            return Ok(());
        }
        if self.try_send(WalCmd::Append { record, sync })? {
            if let Some((excluded, _)) = exclusion {
                self.last_excluded = Some(excluded);
                self.exclusion_loss_pending = false;
            }
        } else {
            self.drop_record(mono_ns, changes_exclusion)?;
        }
        Ok(())
    }

    /// Journals a lifecycle marker. Never dropped: queued in the outbox
    /// when the channel is full and flushed before later traffic.
    pub fn marker(&mut self, marker: Marker) -> Result<(), WalGone> {
        self.flush_outbox()?;
        if !self.outbox.is_empty() || !self.send_marker(&marker)? {
            self.outbox.push_back(marker);
        }
        Ok(())
    }

    /// Updates (or pauses, with `None`) heartbeat data. Droppable: a
    /// stale heartbeat sample is conservative, and fresher data follows
    /// within one status refresh.
    pub fn heartbeat_data(&mut self, data: Option<HeartbeatData>) -> Result<(), WalGone> {
        self.try_send(WalCmd::Heartbeat(data)).map(|_| ())
    }

    /// Persists a `receive_seq` observation. Droppable: the next
    /// observation (~1 Hz) supersedes it, and a missing observation only
    /// widens the reconstruction window (safe direction).
    pub fn receive_seq(&mut self, mono_ns: u64, widened: u64) -> Result<(), WalGone> {
        self.try_send(WalCmd::ReceiveSeq { mono_ns, widened })
            .map(|_| ())
    }

    /// Requests a final sync and thread exit. Blocking is acceptable
    /// here: the socket is no longer being served.
    pub fn shutdown(self) {
        // A send error means the thread already exited; nothing to do.
        let _ = self.tx.send(WalCmd::Shutdown);
    }

    /// `true` if the command was accepted, `false` on a full channel.
    fn try_send(&mut self, cmd: WalCmd) -> Result<bool, WalGone> {
        match self.tx.try_send(cmd) {
            Ok(()) => Ok(true),
            Err(TrySendError::Full(_)) => Ok(false),
            Err(TrySendError::Disconnected(_)) => Err(WalGone),
        }
    }

    fn send_marker(&mut self, marker: &Marker) -> Result<bool, WalGone> {
        self.try_send(WalCmd::Append {
            record: WalRecord::Marker(marker.clone()),
            sync: SyncPolicy::Immediate,
        })
    }

    fn flush_outbox(&mut self) -> Result<(), WalGone> {
        while let Some(front) = self.outbox.front().cloned() {
            if self.send_marker(&front)? {
                self.outbox.pop_front();
            } else {
                break;
            }
        }
        Ok(())
    }

    /// Records that a record was dropped: opens/extends the observation
    /// gap and, when the record carried an exclusion change, queues the
    /// undroppable evidence marker.
    fn drop_record(&mut self, mono_ns: u64, changes_exclusion: bool) -> Result<(), WalGone> {
        self.note_drop(mono_ns);
        if changes_exclusion && !self.exclusion_loss_pending {
            self.exclusion_loss_pending = true;
            self.outbox.push_back(Marker {
                mono_ns,
                kind: MarkerKind::ExclusionUpdateLost,
            });
            // Best-effort immediate delivery; it stays queued otherwise.
            self.flush_outbox()?;
        }
        Ok(())
    }

    fn note_drop(&mut self, mono_ns: u64) {
        if self.gap_start.is_none() {
            self.gap_start = Some(mono_ns);
        }
    }

    /// Emits the gap-closing marker; `true` when it was accepted.
    fn close_gap(&mut self, mono_ns: u64) -> Result<bool, WalGone> {
        let Some(start) = self.gap_start else {
            return Ok(true);
        };
        let gap = Marker {
            mono_ns,
            kind: MarkerKind::SubscriptionGap {
                start_mono_ns: start,
                end_mono_ns: mono_ns,
            },
        };
        if self.send_marker(&gap)? {
            self.gap_start = None;
            Ok(true)
        } else {
            Ok(false)
        }
    }
}

/// The exclude-object fingerprint of a record: `(excluded names,
/// re-journals definitions)`, or `None` for records that carry no
/// exclude state at all.
fn exclude_fingerprint(record: &WalRecord) -> Option<(Vec<String>, bool)> {
    let WalRecord::Context(context) = record else {
        return None;
    };
    let exclude = context.exclude.as_deref()?;
    Some((exclude.excluded.clone(), exclude.definitions.is_some()))
}

#[cfg(test)]
mod tests {
    use super::{HeartbeatData, SyncPolicy, WalCmd, WalGone, WalSender};
    use plr_wal::{Marker, MarkerKind, WalRecord};
    use std::sync::mpsc::{sync_channel, Receiver};

    fn test_marker(mono_ns: u64, kind: MarkerKind) -> Marker {
        Marker { mono_ns, kind }
    }

    fn motion_record(mono_ns: u64) -> WalRecord {
        // A Marker doubles as a compact stand-in for any record in these
        // channel-policy tests; the policy is kind-agnostic except for
        // the explicit `marker()` path.
        WalRecord::Marker(test_marker(mono_ns, MarkerKind::Resubscribed))
    }

    fn drain(rx: &Receiver<WalCmd>) -> Vec<WalCmd> {
        let mut out = Vec::new();
        while let Ok(cmd) = rx.try_recv() {
            out.push(cmd);
        }
        out
    }

    #[test]
    fn records_pass_through_with_their_sync_policy() {
        let (tx, rx) = sync_channel(8);
        let mut sender = WalSender::new(tx);
        sender
            .record(motion_record(1), SyncPolicy::Batched, 1)
            .unwrap();
        sender
            .record(motion_record(2), SyncPolicy::Immediate, 2)
            .unwrap();
        let cmds = drain(&rx);
        assert_eq!(cmds.len(), 2);
        assert!(
            matches!(&cmds[0], WalCmd::Append { sync: SyncPolicy::Batched, record } if record.mono_ns() == 1)
        );
        assert!(matches!(
            &cmds[1],
            WalCmd::Append {
                sync: SyncPolicy::Immediate,
                ..
            }
        ));
    }

    #[test]
    fn full_channel_drops_records_and_journals_one_gap() {
        let (tx, rx) = sync_channel(2);
        let mut sender = WalSender::new(tx);
        // Fill the channel.
        sender
            .record(motion_record(10), SyncPolicy::Batched, 10)
            .unwrap();
        sender
            .record(motion_record(11), SyncPolicy::Batched, 11)
            .unwrap();
        // These three are dropped; the gap starts at the first drop.
        for t in [12, 13, 14] {
            sender
                .record(motion_record(t), SyncPolicy::Batched, t)
                .unwrap();
        }
        // Consumer drains; the next record is preceded by the gap marker.
        let first = drain(&rx);
        assert_eq!(first.len(), 2);
        sender
            .record(motion_record(20), SyncPolicy::Batched, 20)
            .unwrap();
        let cmds = drain(&rx);
        assert_eq!(cmds.len(), 2);
        let WalCmd::Append {
            record: WalRecord::Marker(gap),
            sync,
        } = &cmds[0]
        else {
            panic!("expected gap marker first, got {:?}", cmds[0]);
        };
        assert_eq!(*sync, SyncPolicy::Immediate);
        assert_eq!(
            gap.kind,
            MarkerKind::SubscriptionGap {
                start_mono_ns: 12,
                end_mono_ns: 20,
            }
        );
        let WalCmd::Append { record, .. } = &cmds[1] else {
            panic!("expected the record after the gap marker");
        };
        assert_eq!(record.mono_ns(), 20);
    }

    #[test]
    fn gap_marker_needs_room_before_the_record_is_accepted() {
        let (tx, rx) = sync_channel(2);
        let mut sender = WalSender::new(tx);
        sender
            .record(motion_record(1), SyncPolicy::Batched, 1)
            .unwrap();
        sender
            .record(motion_record(2), SyncPolicy::Batched, 2)
            .unwrap();
        sender
            .record(motion_record(3), SyncPolicy::Batched, 3)
            .unwrap(); // dropped, gap opens at 3
                       // Free exactly one slot: only the gap marker fits; the record is
                       // dropped again and the gap re-opens at its timestamp.
        assert!(matches!(rx.try_recv(), Ok(WalCmd::Append { .. })));
        sender
            .record(motion_record(4), SyncPolicy::Batched, 4)
            .unwrap();
        let cmds = drain(&rx);
        // Old record (2) + gap marker; record 4 was dropped.
        assert_eq!(cmds.len(), 2);
        let WalCmd::Append {
            record: WalRecord::Marker(gap),
            ..
        } = &cmds[1]
        else {
            panic!("expected gap marker");
        };
        assert_eq!(
            gap.kind,
            MarkerKind::SubscriptionGap {
                start_mono_ns: 3,
                end_mono_ns: 4,
            }
        );
        // The re-opened gap (record 4) closes on the next success.
        sender
            .record(motion_record(9), SyncPolicy::Batched, 9)
            .unwrap();
        let cmds = drain(&rx);
        assert_eq!(cmds.len(), 2);
        let WalCmd::Append {
            record: WalRecord::Marker(gap),
            ..
        } = &cmds[0]
        else {
            panic!("expected second gap marker");
        };
        assert_eq!(
            gap.kind,
            MarkerKind::SubscriptionGap {
                start_mono_ns: 4,
                end_mono_ns: 9,
            }
        );
    }

    #[test]
    fn markers_are_never_dropped() {
        let (tx, rx) = sync_channel(1);
        let mut sender = WalSender::new(tx);
        sender
            .record(motion_record(1), SyncPolicy::Batched, 1)
            .unwrap(); // fills the channel
        sender
            .marker(test_marker(2, MarkerKind::SocketLost))
            .unwrap(); // queued in the outbox
        sender
            .marker(test_marker(3, MarkerKind::Resubscribed))
            .unwrap(); // queued behind it
        assert!(matches!(rx.try_recv(), Ok(WalCmd::Append { .. })));
        // Any later call flushes the outbox in order, before new traffic.
        sender
            .record(motion_record(9), SyncPolicy::Batched, 9)
            .unwrap();
        let cmds = drain(&rx);
        // Channel capacity 1: only the first outbox marker fit; record 9
        // was counted as a drop (ordering preserved).
        assert_eq!(cmds.len(), 1);
        let WalCmd::Append {
            record: WalRecord::Marker(m),
            ..
        } = &cmds[0]
        else {
            panic!("expected marker");
        };
        assert_eq!(m.kind, MarkerKind::SocketLost);
        // Next flush delivers the second marker.
        sender
            .marker(test_marker(4, MarkerKind::CleanShutdown))
            .unwrap();
        let cmds = drain(&rx);
        assert_eq!(cmds.len(), 1);
        let WalCmd::Append {
            record: WalRecord::Marker(m),
            ..
        } = &cmds[0]
        else {
            panic!("expected marker");
        };
        assert_eq!(m.kind, MarkerKind::Resubscribed);
    }

    #[test]
    fn heartbeat_and_receive_seq_are_droppable() {
        let (tx, rx) = sync_channel(1);
        let mut sender = WalSender::new(tx);
        let data = HeartbeatData {
            print_time: 1.0,
            est_sample_mono_ns: 5,
            est_sample_print_time: 0.9,
        };
        sender.heartbeat_data(Some(data)).unwrap();
        // Channel now full: both are silently dropped, no gap opens.
        sender.heartbeat_data(Some(data)).unwrap();
        sender.receive_seq(6, 100).unwrap();
        let cmds = drain(&rx);
        assert_eq!(cmds.len(), 1);
        assert!(matches!(cmds[0], WalCmd::Heartbeat(Some(_))));
        // No gap marker on the next record: hb/seq drops are not gaps.
        sender
            .record(motion_record(7), SyncPolicy::Batched, 7)
            .unwrap();
        let cmds = drain(&rx);
        assert_eq!(cmds.len(), 1);
        assert!(matches!(&cmds[0], WalCmd::Append { record, .. } if record.mono_ns() == 7));
    }

    #[test]
    fn disconnected_channel_reports_wal_gone() {
        let (tx, rx) = sync_channel(1);
        drop(rx);
        let mut sender = WalSender::new(tx);
        assert_eq!(
            sender.record(motion_record(1), SyncPolicy::Batched, 1),
            Err(WalGone)
        );
        assert_eq!(sender.heartbeat_data(None), Err(WalGone));
        assert_eq!(sender.receive_seq(1, 1), Err(WalGone));
        assert_eq!(
            sender.marker(test_marker(1, MarkerKind::SocketLost)),
            Err(WalGone)
        );
        assert!(WalGone.to_string().contains("durability"));
    }

    /// A `Context` record carrying the given excluded-name set.
    fn exclusion_context(mono_ns: u64, excluded: &[&str], redefine: bool) -> WalRecord {
        WalRecord::Context(plr_wal::Context {
            mono_ns,
            print_state: None,
            virtual_sdcard: None,
            gcode: plr_wal::GcodeState {
                speed_factor: 1.0,
                speed: 1_500.0,
                extrude_factor: 1.0,
                absolute_coordinates: true,
                absolute_extrude: true,
                homing_origin: vec![0.0; 4],
                position: vec![0.0; 4],
                gcode_position: vec![0.0; 4],
            },
            transforms: plr_wal::TransformObservations {
                bed_mesh_active: false,
                bed_mesh_profile: None,
                z_thermal_adjust_enabled: None,
                z_thermal_adjust_offset: None,
                skew_active: false,
                skew_profile: None,
            },
            heaters: Vec::new(),
            fans: Vec::new(),
            exclude: Some(Box::new(plr_wal::ExcludeState {
                definitions: redefine.then(|| vec![plr_wal::ExcludeObjectDef::name_only("PART_A")]),
                excluded: excluded.iter().map(|s| (*s).to_owned()).collect(),
                current: None,
            })),
        })
    }

    /// Every marker kind that reached the channel, in order.
    fn marker_kinds(cmds: &[WalCmd]) -> Vec<MarkerKind> {
        cmds.iter()
            .filter_map(|cmd| match cmd {
                WalCmd::Append {
                    record: WalRecord::Marker(m),
                    ..
                } => Some(m.kind.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn dropping_an_exclusion_change_journals_undroppable_evidence() {
        // The operator cancels a part while the WAL thread is stalled.
        // The context itself is droppable (blocking would get us
        // disconnected by Klipper), but the *fact* that it was lost is
        // not: a marker must reach the log.
        let (tx, rx) = sync_channel(1);
        let mut sender = WalSender::new(tx);
        sender
            .record(motion_record(1), SyncPolicy::Batched, 1)
            .unwrap(); // fills the channel
        sender
            .record(
                exclusion_context(2, &["PART_A"], false),
                SyncPolicy::Immediate,
                2,
            )
            .unwrap(); // dropped -> loss marker queued in the outbox

        // Drain and let the outbox flush.
        assert!(matches!(rx.try_recv(), Ok(WalCmd::Append { .. })));
        sender
            .record(motion_record(9), SyncPolicy::Batched, 9)
            .unwrap();
        let cmds = drain(&rx);
        assert_eq!(
            marker_kinds(&cmds).first(),
            Some(&MarkerKind::ExclusionUpdateLost),
            "the loss marker must lead, got {cmds:?}"
        );
    }

    #[test]
    fn exclusion_loss_marker_is_emitted_once_per_episode() {
        let (tx, rx) = sync_channel(1);
        let mut sender = WalSender::new(tx);
        sender
            .record(motion_record(1), SyncPolicy::Batched, 1)
            .unwrap();
        // A long stall: many exclusion-bearing contexts are dropped. The
        // outbox must not grow without bound.
        for t in 2..40 {
            sender
                .record(
                    exclusion_context(t, &["PART_A"], false),
                    SyncPolicy::Immediate,
                    t,
                )
                .unwrap();
        }
        assert!(matches!(rx.try_recv(), Ok(WalCmd::Append { .. })));
        sender
            .record(motion_record(50), SyncPolicy::Batched, 50)
            .unwrap();
        let cmds = drain(&rx);
        assert_eq!(
            marker_kinds(&cmds)
                .iter()
                .filter(|k| **k == MarkerKind::ExclusionUpdateLost)
                .count(),
            1,
            "one marker per lost-then-recovered episode"
        );
    }

    #[test]
    fn a_landed_exclusion_context_re_arms_the_loss_detector() {
        let (tx, rx) = sync_channel(4);
        let mut sender = WalSender::new(tx);
        // First episode: fill, drop an exclusion change, recover.
        for t in 1..=4 {
            sender
                .record(motion_record(t), SyncPolicy::Batched, t)
                .unwrap();
        }
        sender
            .record(
                exclusion_context(5, &["PART_A"], false),
                SyncPolicy::Immediate,
                5,
            )
            .unwrap(); // dropped
        drain(&rx);
        sender
            .record(
                exclusion_context(6, &["PART_A"], false),
                SyncPolicy::Immediate,
                6,
            )
            .unwrap(); // lands (with the queued marker + gap marker)
        let cmds = drain(&rx);
        assert!(marker_kinds(&cmds).contains(&MarkerKind::ExclusionUpdateLost));

        // Second episode with a *different* excluded set must produce a
        // fresh marker.
        for t in 10..=13 {
            sender
                .record(motion_record(t), SyncPolicy::Batched, t)
                .unwrap();
        }
        sender
            .record(
                exclusion_context(14, &["PART_A", "PART_B"], false),
                SyncPolicy::Immediate,
                14,
            )
            .unwrap(); // dropped
        drain(&rx);
        sender
            .record(motion_record(20), SyncPolicy::Batched, 20)
            .unwrap();
        let cmds = drain(&rx);
        assert!(
            marker_kinds(&cmds).contains(&MarkerKind::ExclusionUpdateLost),
            "a later, different exclusion change must be flagged too"
        );
    }

    #[test]
    fn dropping_an_unchanged_exclusion_context_is_not_a_loss() {
        // Every context re-states the excluded set (it is cheap), so most
        // dropped contexts carry no *change*. Flagging those would cry
        // wolf on every disk hiccup.
        let (tx, rx) = sync_channel(2);
        let mut sender = WalSender::new(tx);
        sender
            .record(
                exclusion_context(1, &["PART_A"], false),
                SyncPolicy::Immediate,
                1,
            )
            .unwrap(); // lands, arms last_excluded
        sender
            .record(motion_record(2), SyncPolicy::Batched, 2)
            .unwrap(); // fills
        sender
            .record(
                exclusion_context(3, &["PART_A"], false),
                SyncPolicy::Immediate,
                3,
            )
            .unwrap(); // dropped, but nothing changed
        drain(&rx);
        sender
            .record(motion_record(9), SyncPolicy::Batched, 9)
            .unwrap();
        let cmds = drain(&rx);
        assert!(
            !marker_kinds(&cmds).contains(&MarkerKind::ExclusionUpdateLost),
            "an unchanged excluded set is not a lost update"
        );
        // The ordinary observation gap is still journaled.
        assert!(marker_kinds(&cmds)
            .iter()
            .any(|k| matches!(k, MarkerKind::SubscriptionGap { .. })));
    }

    #[test]
    fn dropping_a_definition_refresh_counts_as_a_loss() {
        // Re-journaled definitions carry the geometry recovery needs; a
        // dropped one leaves the excluded set unmatched to any outline.
        let (tx, rx) = sync_channel(1);
        let mut sender = WalSender::new(tx);
        sender
            .record(motion_record(1), SyncPolicy::Batched, 1)
            .unwrap();
        sender
            .record(exclusion_context(2, &[], true), SyncPolicy::Immediate, 2)
            .unwrap(); // dropped; empty set but definitions present
        assert!(matches!(rx.try_recv(), Ok(WalCmd::Append { .. })));
        sender
            .record(motion_record(9), SyncPolicy::Batched, 9)
            .unwrap();
        let cmds = drain(&rx);
        assert!(marker_kinds(&cmds).contains(&MarkerKind::ExclusionUpdateLost));
    }

    #[test]
    fn exclusion_contexts_never_block_the_caller() {
        // The non-blocking property is deliberate: Klipper closes clients
        // whose socket stays blocked. Even with a full channel and a
        // full outbox, every call returns immediately.
        let (tx, _rx) = sync_channel(1);
        let mut sender = WalSender::new(tx);
        for t in 0..100 {
            sender
                .record(
                    exclusion_context(t, &["PART_A"], t % 2 == 0),
                    SyncPolicy::Immediate,
                    t,
                )
                .unwrap();
        }
    }

    #[test]
    fn shutdown_is_delivered() {
        let (tx, rx) = sync_channel(4);
        let sender = WalSender::new(tx);
        sender.shutdown();
        assert_eq!(drain(&rx), vec![WalCmd::Shutdown]);
    }
}
