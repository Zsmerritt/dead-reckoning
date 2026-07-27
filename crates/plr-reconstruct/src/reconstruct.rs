//! The top-level orchestration: scan + heartbeat + file tail in,
//! [`Reconstruction`] out.

use plr_wal::{HeartbeatRecovery, RecoveryScan};

use crate::config::ReconstructConfig;
use crate::epoch::{select_crash_epoch, EpochSpan};
use crate::error::ReconstructError;
use crate::exclude::{resolve_exclusions, ExclusionInputs, ExclusionReport};
use crate::stopset::{compute_stop_set, FileTail, PossibleStopSet};
use crate::timeline::{ingest, WalTimeline};
use crate::window::{compute_stop_window, ReceiveSeqObservation, StopWindow};

/// Everything the daemon hands the reconstruction engine after an
/// unclean stop. All inputs are borrowed; this crate does no I/O.
#[derive(Debug, Clone, Copy)]
pub struct ReconstructInputs<'a> {
    /// The recovery scan of the durable WAL prefix
    /// ([`plr_wal::scan`]).
    pub scan: &'a RecoveryScan,
    /// The recovered heartbeat file, when one validated
    /// ([`plr_wal::recover_heartbeat`]). WAL heartbeat records are also
    /// consulted; the newest finite sample wins.
    pub heartbeat: Option<&'a HeartbeatRecovery>,
    /// Bytes of the printed file covering the last context's
    /// `file_position` (the whole file with `base_offset == 0` is
    /// fine). `None` disables the forward extension — which voids the
    /// containment guarantee for true power loss and is flagged as
    /// [`crate::stopset::Degradation::extension_unavailable`].
    pub file_tail: Option<FileTail<'a>>,
    /// The newest durable widened `receive_seq` observation, if the
    /// daemon persisted one. Applied strictly as a time bound on `t_b`.
    pub receive_seq: Option<ReceiveSeqObservation>,
}

/// A full recovery reconstruction: timeline, window, stop set, and the
/// excluded-object picture.
#[derive(Debug, Clone, PartialEq)]
pub struct RecoveryReconstruction {
    /// The ingested, validated WAL timeline.
    ///
    /// This — and every other field below — is scoped to the **crash
    /// epoch** alone: [`reconstruct`] partitions the merged record stream
    /// into boot/firmware epochs (see [`crate::epoch`]) and discards
    /// every other epoch before ingestion. When evidence from other
    /// epochs was discarded, [`crate::stopset::Degradation::cross_epoch_evidence_discarded`]
    /// on [`Self::stop_set`] is set. Recompute the full partition with
    /// [`crate::select_crash_epoch`] over the same scan for the details
    /// (how many epochs, which boundaries).
    pub timeline: WalTimeline,
    /// The stop window `[t_a, t_b]` and crash class.
    pub window: StopWindow,
    /// The possible-stop set.
    pub stop_set: PossibleStopSet,
    /// Which objects the operator cancelled, and how much that answer
    /// can be trusted.
    ///
    /// **Gate any automatic resume on
    /// [`ExclusionReport::is_conclusive`]**, which is true only when
    /// nothing the log records as lost postdates the newest exclusion
    /// observation and that observation is fresh. When it is false,
    /// [`ExclusionReport::confirmation`] carries the per-object payload
    /// the operator prompt must be built from.
    pub exclusions: ExclusionReport,
}

/// Outcome of [`reconstruct`].
#[derive(Debug, Clone, PartialEq)]
pub enum Reconstruction {
    /// The WAL ends with a clean-shutdown marker: the print ended on
    /// purpose and **no recovery is needed**. Reported distinctly so
    /// callers never probe or resume after a deliberate stop. The
    /// timeline is included for reporting.
    CleanShutdown(Box<WalTimeline>),
    /// An unclean stop: the possible-stop set bounds where the machine
    /// actually is.
    Recovery(Box<RecoveryReconstruction>),
}

/// Runs the full pipeline: validate config → ingest → clean-shutdown
/// short-circuit → stop window → possible-stop set.
///
/// Errors are limited to unusable prerequisites (see
/// [`ReconstructError`]); every data-quality problem short of that
/// degrades honestly inside the result.
pub fn reconstruct(
    inputs: &ReconstructInputs<'_>,
    config: &ReconstructConfig,
) -> Result<Reconstruction, ReconstructError> {
    config.validate()?;

    // Isolate the crash epoch BEFORE anything reads the two global
    // clocks. Every later stage (`t_a`, `t_b`, `wal_eval_end`, the anchor
    // context, heartbeat coverage, the forward extension) then operates
    // within one boot and one firmware session. See [`crate::epoch`].
    let selection = select_crash_epoch(inputs.scan);
    let scan = selection.narrow(inputs.scan);

    // Out-of-band inputs (the heartbeat file, the receive-seq sidecar)
    // are rewritten in place and belong to whichever process last wrote
    // them — after a reboot, a *later* boot. Admit each only when its
    // host-monotonic timestamp belongs to the crash epoch, so `t_a` and
    // the `t_b` time bound cannot be taken from another epoch's clock.
    // The heartbeat *records* inside the narrowed scan already carry the
    // crash epoch's liveness; the file only ever adds one fresher sample.
    let epoch = selection.crash_epoch();
    let is_newest = selection
        .selected
        .is_some_and(|i| i + 1 == selection.epochs.len());
    let heartbeat = inputs
        .heartbeat
        .filter(|hb| epoch_admits(epoch, is_newest, hb.heartbeat.mono_ns));
    let receive_seq = inputs
        .receive_seq
        .filter(|obs| epoch_admits(epoch, is_newest, obs.mono_ns));

    let timeline = ingest(&scan, heartbeat);
    if timeline.clean_shutdown {
        return Ok(Reconstruction::CleanShutdown(Box::new(timeline)));
    }
    let window = compute_stop_window(&timeline, receive_seq.as_ref(), config)?;
    let mut stop_set = compute_stop_set(&timeline, &window, inputs.file_tail.as_ref(), config)?;
    // Record, honestly, that this reconstruction covers a single epoch of
    // a multi-epoch log — other epochs' evidence was removed, not merged.
    // Informational (like `extension_start_unanchored`): the result is
    // more accurate for it, not less certain, so it does not move
    // `confidence`.
    stop_set.degradation.cross_epoch_evidence_discarded = selection.partitioned();
    let exclusions = resolve_exclusions(
        &timeline,
        &ExclusionInputs {
            window: Some(&window),
            stop_end_print_time: Some(stop_set.wal_eval_end),
            file: inputs.file_tail.as_ref(),
        },
        config,
    );
    Ok(Reconstruction::Recovery(Box::new(RecoveryReconstruction {
        timeline,
        window,
        stop_set,
        exclusions,
    })))
}

/// Whether an out-of-band input stamped at host-monotonic `mono_ns`
/// belongs to the crash epoch. For the newest epoch (the current boot,
/// nothing wrote the sidecars afterwards) the file may be marginally
/// fresher than the last WAL record, so only the lower bound applies.
/// For a superseded epoch a later boot overwrote the file, so the
/// timestamp must fall strictly within the epoch's observed span.
/// With no epoch (empty scan) nothing is admitted.
///
/// This is a `mono_ns`-interval test, not a proof of epoch *identity*:
/// `CLOCK_MONOTONIC` resets each boot, so a *different* boot whose uptime
/// happens to land inside the crash epoch's span would also pass. That is
/// only reachable for a superseded epoch (a later boot ran at least as
/// long as the crash boot had by the crash) and merely re-admits a
/// heartbeat/receive-seq reading close in uptime — the interval bounds it
/// near the crash epoch either way. Closing it fully would require
/// threading a per-segment epoch identity (e.g. `created_wall_ns`) into
/// the sidecars, which the WAL format does not carry; documented here
/// rather than papered over.
fn epoch_admits(epoch: Option<&EpochSpan>, is_newest: bool, mono_ns: u64) -> bool {
    match epoch {
        Some(e) if is_newest => mono_ns >= e.min_mono_ns,
        Some(e) => e.contains_mono(mono_ns),
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use plr_wal::{ExcludeObjectDef, ExcludeState, Marker, MarkerKind, WalRecord};

    use super::{reconstruct, ReconstructInputs, Reconstruction};
    use crate::config::ReconstructConfig;
    use crate::epoch::{select_crash_epoch, EpochBoundaryKind};
    use crate::error::ReconstructError;
    use crate::exclude::{
        ExclusionDiagnostic, ExclusionFreshness, ExclusionProvenance, UncertaintyCause,
    };
    use crate::reconstruct::RecoveryReconstruction;
    use crate::stopset::FileTail;
    use crate::testutil::{
        context_at, heartbeat_at, scan_of, scan_of_segments, stepper_range, trapq_segment,
    };
    use crate::window::CrashClass;

    fn inputs(scan: &plr_wal::RecoveryScan) -> ReconstructInputs<'_> {
        ReconstructInputs {
            scan,
            heartbeat: None,
            file_tail: None,
            receive_seq: None,
        }
    }

    const S: u64 = 1_000_000_000; // one second in ns

    /// Reconstructs an unclean stop or panics with the error/outcome.
    fn recover(scan: &plr_wal::RecoveryScan) -> RecoveryReconstruction {
        match reconstruct(&inputs(scan), &ReconstructConfig::default()) {
            Ok(Reconstruction::Recovery(r)) => *r,
            other => panic!("expected Recovery, got {other:?}"),
        }
    }

    /// The crash-epoch records of a two-boot log: an OLD boot that printed
    /// far into `print_time` at high `mono_ns`, then (after `boundary`) the
    /// crashed print on a fresh low clock. The old boot's Z stepper and
    /// trapq sit at `print_time = poison`, which a global `max` would take
    /// as `t_b`/`wal_eval_end`.
    fn crash_epoch_records() -> Vec<WalRecord> {
        vec![
            WalRecord::Heartbeat(heartbeat_at(20 * S, 10.0)),
            WalRecord::Context(context_at(20 * S, 500)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 10.0, 0.5, 20 * S)),
            WalRecord::StepperRange(stepper_range("stepper_z", 10.3, 20 * S)),
            WalRecord::Heartbeat(heartbeat_at(21 * S, 10.5)),
        ]
    }

    /// Asserts the reconstruction of `full` (a multi-epoch log) is
    /// bit-identical, in every window/stop-set number, to reconstructing
    /// `crash_epoch_records()` alone — i.e. the older epoch contributed
    /// nothing.
    fn assert_matches_isolated(full: &plr_wal::RecoveryScan) -> RecoveryReconstruction {
        let got = recover(full);
        let mut iso = recover(&scan_of(crash_epoch_records()));
        assert_eq!(
            got.window, iso.window,
            "window must come from the crash epoch alone"
        );
        // The only legitimate difference: `got` partitioned a multi-epoch
        // log and flagged it; the isolated single-epoch input did not.
        // Everything else in the stop set must be bit-identical.
        assert!(got.stop_set.degradation.cross_epoch_evidence_discarded);
        assert!(!iso.stop_set.degradation.cross_epoch_evidence_discarded);
        iso.stop_set.degradation.cross_epoch_evidence_discarded = true;
        assert_eq!(
            got.stop_set, iso.stop_set,
            "stop set must come from the crash epoch alone"
        );
        // Positive proof the poison is gone: the isolated t_b is ~10.3,
        // nowhere near the 60_000 planted in the older epoch.
        assert!(got.window.t_b < 100.0, "t_b poisoned: {}", got.window.t_b);
        assert!(
            got.stop_set.wal_eval_end < 100.0,
            "wal_eval_end poisoned: {}",
            got.stop_set.wal_eval_end
        );
        got
    }

    #[test]
    fn reboot_isolates_the_crash_from_an_older_boot() {
        // Reboot shape: mono reset + file_position reset, across a segment
        // boundary (a reboot always opens a new segment). The old boot
        // printed to print_time 60_000 at mono 40_000 s; the new boot
        // (the crash) restarts the monotonic clock near zero.
        let old_boot = vec![
            WalRecord::Heartbeat(heartbeat_at(40_000 * S, 60_000.0)),
            WalRecord::Context(context_at(40_000 * S, 900_000)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 60_000.0, 0.5, 40_000 * S)),
            WalRecord::StepperRange(stepper_range("stepper_z", 60_000.0, 40_000 * S)),
        ];
        let full = scan_of_segments(vec![old_boot, crash_epoch_records()]);
        let got = assert_matches_isolated(&full);
        assert!(got.stop_set.degradation.cross_epoch_evidence_discarded);
        let sel = select_crash_epoch(&full);
        assert!(sel.partitioned());
        assert_eq!(sel.discarded_older(), 1);
        assert!(matches!(
            sel.crash_epoch().unwrap().boundary_before,
            Some(EpochBoundaryKind::HostReboot { .. })
        ));
    }

    #[test]
    fn firmware_restart_isolates_the_crash_from_the_pre_restart_session() {
        // Firmware-restart shape: SocketLost/Resubscribed + print_time
        // reset, mono CONTINUOUS. The pre-restart idle session sat at
        // print_time 60_000 (a long-idle klippy); the print restarts near
        // zero on the SAME boot clock (mono keeps climbing).
        let mut records = vec![
            WalRecord::Heartbeat(heartbeat_at(10 * S, 60_000.0)),
            WalRecord::Context(context_at(10 * S, 25_000)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 60_000.0, 0.5, 10 * S)),
            WalRecord::StepperRange(stepper_range("stepper_z", 60_000.0, 10 * S)),
            WalRecord::Marker(Marker {
                mono_ns: 12 * S,
                kind: MarkerKind::SocketLost,
            }),
            WalRecord::Marker(Marker {
                mono_ns: 15 * S,
                kind: MarkerKind::Resubscribed,
            }),
        ];
        // The crash print runs later on the same boot; shift its mono up
        // so the host clock is monotonic across the restart.
        for r in crash_epoch_records() {
            records.push(shift_mono(r, 100 * S));
        }
        let full = scan_of(records);
        let got = recover(&full);
        // Same isolation, but the crash epoch here carries a +100 s mono
        // offset, so compare the print-time numbers directly.
        assert!(got.window.t_b < 100.0, "t_b poisoned: {}", got.window.t_b);
        assert!(
            got.stop_set.wal_eval_end < 100.0,
            "wal_eval_end poisoned: {}",
            got.stop_set.wal_eval_end
        );
        assert!(got.stop_set.degradation.cross_epoch_evidence_discarded);
        assert!(matches!(
            select_crash_epoch(&full)
                .crash_epoch()
                .unwrap()
                .boundary_before,
            Some(EpochBoundaryKind::FirmwareRestart { .. })
        ));
    }

    #[test]
    fn out_of_epoch_heartbeat_file_cannot_set_t_a() {
        // A post-crash idle boot rewrote the heartbeat file, so its newest
        // sample belongs to a LATER epoch. It must not become t_a for the
        // crash epoch (which is not the newest partition here).
        use plr_wal::{HeartbeatRecovery, SlotId};
        // Post-crash idle boot is a NEW segment (offsets restart), mono
        // reset, context names no file.
        let idle_boot = vec![
            WalRecord::Heartbeat(heartbeat_at(5 * S, 1.0)),
            WalRecord::Context({
                let mut c = context_at(5 * S, 0);
                c.virtual_sdcard = None;
                c
            }),
        ];
        let scan = scan_of_segments(vec![crash_epoch_records(), idle_boot]);
        // The file heartbeat is from the idle boot (mono 6 s, print_time
        // 99 999) — if admitted it would blow t_a up to ~100 000.
        let file = HeartbeatRecovery {
            heartbeat: heartbeat_at(6 * S, 99_999.0),
            slot: SlotId::A,
            torn: None,
        };
        let out = reconstruct(
            &ReconstructInputs {
                scan: &scan,
                heartbeat: Some(&file),
                file_tail: None,
                receive_seq: None,
            },
            &ReconstructConfig::default(),
        )
        .unwrap();
        let Reconstruction::Recovery(r) = out else {
            panic!("expected Recovery");
        };
        assert!(
            r.window.t_a < 100.0,
            "t_a took a later epoch's file heartbeat: {}",
            r.window.t_a
        );
    }

    /// HAZARD PIN: a same-boot firmware restart with NO `SocketLost`
    /// marker (mono continuous, `print_time` reset to ~0) is NOT
    /// partitioned on the print-time axis — there is no print-time
    /// delimiter at all. This is the deliberate cost of deleting the
    /// print-time backstop (which had zero true positives and false-fired
    /// on the reference capture; see `crate::epoch` docs).
    ///
    /// Why it is safe rather than merely bounded: `SocketLost` rides the
    /// marker path, which is never dropped under WAL backpressure (unlike
    /// `Context` records — the reason `ExclusionUpdateLost` exists). The
    /// only way it is absent for a real restart is a torn tail AT the
    /// marker, and a torn tail can only be the newest segment's (rotation
    /// fsyncs before opening a successor). No durable record can follow
    /// the tear, so there is no post-restart session to isolate — the log
    /// ends in the pre-restart session, already the newest epoch. This
    /// synthetic input (records continuing past a marker-less reset) is a
    /// shape the writer cannot produce; it is pinned so that if a future
    /// print-time delimiter is added, this assertion must be INVERTED and
    /// the argument above re-examined.
    #[test]
    fn hazard_marker_less_restart_is_not_partitioned() {
        let mut records = vec![
            WalRecord::Heartbeat(heartbeat_at(10 * S, 60_000.0)),
            WalRecord::TrapqSegment(trapq_segment("toolhead", 60_000.0, 0.2, 10 * S)),
            WalRecord::Context(context_at(10 * S, 25_000)),
        ];
        // Restart resets print_time to ~0, NO marker, mono continuous.
        records.push(WalRecord::Heartbeat(heartbeat_at(11 * S, 0.1)));
        records.push(WalRecord::Context(context_at(11 * S, 100)));
        records.push(WalRecord::TrapqSegment(trapq_segment(
            "toolhead",
            0.1,
            0.2,
            11 * S,
        )));
        let got = recover(&scan_of(records));
        assert!(
            !got.stop_set.degradation.cross_epoch_evidence_discarded,
            "a marker-less restart has no print-time delimiter (see doc)"
        );
    }

    /// Shifts a record's `mono_ns` forward by `delta` (test helper for
    /// building same-boot, monotonic-clock multi-session logs).
    fn shift_mono(record: WalRecord, delta: u64) -> WalRecord {
        match record {
            WalRecord::Heartbeat(mut h) => {
                h.mono_ns += delta;
                h.est_sample_mono_ns += delta;
                WalRecord::Heartbeat(h)
            }
            WalRecord::Context(mut c) => {
                c.mono_ns += delta;
                WalRecord::Context(c)
            }
            WalRecord::TrapqSegment(mut t) => {
                t.mono_ns += delta;
                WalRecord::TrapqSegment(t)
            }
            WalRecord::StepperRange(mut s) => {
                s.mono_ns += delta;
                WalRecord::StepperRange(s)
            }
            WalRecord::Marker(mut m) => {
                m.mono_ns += delta;
                WalRecord::Marker(m)
            }
        }
    }

    /// Edge case (c): the frontier cap must read only crash-epoch data.
    /// An older boot carrying a giant frontier is partitioned away before
    /// ingestion, so the cap — which reads the anchor context, `t_a`, and
    /// the heartbeat stream — can never take it. If it leaked, the window
    /// high end would blow out to (or past) the giant offset.
    #[test]
    fn frontier_cap_reads_only_crash_epoch_frontier() {
        use std::fmt::Write as _;
        let mut text = String::new();
        for i in 1..=80 {
            let _ = writeln!(text, "G1 X{} Y50 F3000", 50 + 5 * i);
        }
        // Old boot at a high mono/print time with a 900_000-byte frontier.
        let old_boot = vec![
            WalRecord::Heartbeat(heartbeat_at(40_000 * S, 60_000.0)),
            WalRecord::Context(context_at(40_000 * S, 900_000)),
            WalRecord::StepperRange(stepper_range("stepper_z", 60_000.0, 40_000 * S)),
        ];
        // Crash boot: fresh low clock, frontier at offset 0, dense
        // heartbeats so the cap's tail-continuity guard passes.
        let crash = vec![
            WalRecord::Heartbeat(heartbeat_at(20 * S, 10.0)),
            WalRecord::Heartbeat(heartbeat_at(21 * S, 10.5)),
            WalRecord::StepperRange(stepper_range("stepper_z", 10.3, 21 * S)),
            WalRecord::Context(context_at(21 * S, 0)),
        ];
        let full = scan_of_segments(vec![old_boot, crash]);
        let out = reconstruct(
            &ReconstructInputs {
                scan: &full,
                heartbeat: None,
                file_tail: Some(FileTail {
                    base_offset: 0,
                    bytes: text.as_bytes(),
                }),
                receive_seq: None,
            },
            &ReconstructConfig::default(),
        )
        .unwrap();
        let Reconstruction::Recovery(r) = out else {
            panic!("expected Recovery");
        };
        assert!(r.stop_set.degradation.cross_epoch_evidence_discarded);
        let fw = r.stop_set.file_window.unwrap();
        assert!(
            fw.end <= text.len() as u64,
            "cap leaked a cross-epoch frontier: end {} exceeds the crash file {}",
            fw.end,
            text.len()
        );
        // The cap actually applied (narrowed below EOF): proof it ran on
        // the crash frontier, not that it merely defaulted.
        assert!(r
            .stop_set
            .extension
            .as_ref()
            .unwrap()
            .frontier_cap
            .is_some());
        assert!(fw.end < text.len() as u64);
    }

    #[test]
    fn clean_shutdown_reports_distinctly_without_prerequisites() {
        // No heartbeat, no context: a clean shutdown still reports
        // cleanly instead of erroring, because no recovery is needed.
        let scan = scan_of(vec![WalRecord::Marker(Marker {
            mono_ns: 1,
            kind: MarkerKind::CleanShutdown,
        })]);
        let outcome = reconstruct(&inputs(&scan), &ReconstructConfig::default()).unwrap();
        let Reconstruction::CleanShutdown(timeline) = outcome else {
            panic!("expected CleanShutdown, got {outcome:?}");
        };
        assert!(timeline.clean_shutdown);
    }

    #[test]
    fn invalid_config_is_rejected_before_any_work() {
        let scan = scan_of(vec![]);
        let config = ReconstructConfig {
            extension_horizon: f64::NAN,
            ..ReconstructConfig::default()
        };
        assert!(matches!(
            reconstruct(&inputs(&scan), &config),
            Err(ReconstructError::InvalidConfig { .. })
        ));
    }

    #[test]
    fn missing_prerequisites_surface_as_typed_errors() {
        // Empty WAL: no heartbeat.
        let scan = scan_of(vec![]);
        assert_eq!(
            reconstruct(&inputs(&scan), &ReconstructConfig::default()),
            Err(ReconstructError::NoHeartbeat)
        );
        // Heartbeat but no context.
        let scan = scan_of(vec![WalRecord::Heartbeat(heartbeat_at(
            1_000_000_000,
            10.0,
        ))]);
        assert_eq!(
            reconstruct(&inputs(&scan), &ReconstructConfig::default()),
            Err(ReconstructError::NoContext)
        );
    }

    #[test]
    fn minimal_unclean_stop_produces_a_recovery() {
        let scan = scan_of(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::Context(context_at(1_000_000_000, 0)),
        ]);
        let outcome = reconstruct(&inputs(&scan), &ReconstructConfig::default()).unwrap();
        let Reconstruction::Recovery(recovery) = outcome else {
            panic!("expected Recovery, got {outcome:?}");
        };
        assert!(matches!(
            recovery.window.class,
            CrashClass::HostDeathOrPowerLoss { .. }
        ));
        assert!(recovery.stop_set.degradation.extension_unavailable);
        // With no exclude state and no file, the exclusion picture is
        // honestly unknown rather than "nothing was cancelled".
        assert_eq!(
            recovery.exclusions.provenance(),
            ExclusionProvenance::Unknown
        );
        assert!(recovery.exclusions.requires_operator_confirmation());
    }

    #[test]
    fn recovery_carries_the_journaled_exclusion_picture() {
        let mut context = context_at(1_000_000_000, 0);
        context.exclude = Some(Box::new(ExcludeState {
            definitions: Some(vec![ExcludeObjectDef::name_only("PART_A")]),
            excluded: vec!["PART_A".to_owned()],
            current: None,
        }));
        let scan = scan_of(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::Context(context),
        ]);
        let outcome = reconstruct(&inputs(&scan), &ReconstructConfig::default()).unwrap();
        let Reconstruction::Recovery(recovery) = outcome else {
            panic!("expected Recovery");
        };
        assert_eq!(
            recovery.exclusions.provenance(),
            ExclusionProvenance::Journaled
        );
        assert!(recovery.exclusions.is_excluded("PART_A"));
        // The stop window is derived from this very context, the scan
        // ended cleanly and no marker postdates it: nothing to ask.
        assert!(
            recovery.exclusions.is_conclusive(),
            "{:?}",
            recovery.exclusions.uncertainty_causes()
        );
        assert!(matches!(
            recovery.exclusions.freshness(),
            ExclusionFreshness::Known { .. }
        ));
    }

    #[test]
    fn a_torn_log_makes_a_journaled_exclusion_inconclusive() {
        // The normal shape of a power-loss WAL: the tail did not end
        // cleanly, so a cancellation may have been written and lost.
        let mut context = context_at(1_000_000_000, 0);
        context.exclude = Some(Box::new(ExcludeState {
            definitions: Some(vec![ExcludeObjectDef::name_only("PART_A")]),
            excluded: Vec::new(),
            current: None,
        }));
        let mut scan = scan_of(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::Context(context),
        ]);
        scan.end = plr_wal::ScanEnd::TruncatedPayload;
        let outcome = reconstruct(&inputs(&scan), &ReconstructConfig::default()).unwrap();
        let Reconstruction::Recovery(recovery) = outcome else {
            panic!("expected Recovery");
        };
        assert_eq!(
            recovery.exclusions.provenance(),
            ExclusionProvenance::Journaled
        );
        assert!(recovery.exclusions.requires_operator_confirmation());
        let confirmation = recovery.exclusions.confirmation().expect("prompt");
        assert!(confirmation
            .causes
            .iter()
            .any(|c| matches!(c, UncertaintyCause::LogTailIncomplete { .. })));
        // The prompt payload lists every object, not a yes/no.
        assert_eq!(confirmation.objects.len(), 1);
        assert_eq!(confirmation.objects[0].name, "PART_A");
    }

    #[test]
    fn recovery_flags_a_lost_cancellation_record() {
        // The WAL predates the exclude field (or the module was never
        // observed) but the file defines objects: a resume would print
        // all of them.
        let scan = scan_of(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::Context(context_at(1_000_000_000, 0)),
        ]);
        let file = b"EXCLUDE_OBJECT_DEFINE NAME=part_a\nEXCLUDE_OBJECT_DEFINE NAME=part_b\nG1 X1\n";
        let inputs = ReconstructInputs {
            scan: &scan,
            heartbeat: None,
            file_tail: Some(FileTail {
                base_offset: 0,
                bytes: file,
            }),
            receive_seq: None,
        };
        let outcome = reconstruct(&inputs, &ReconstructConfig::default()).unwrap();
        let Reconstruction::Recovery(recovery) = outcome else {
            panic!("expected Recovery");
        };
        assert_eq!(
            recovery.exclusions.provenance(),
            ExclusionProvenance::RecordLost
        );
        assert_eq!(
            recovery.exclusions.diagnostics(),
            [ExclusionDiagnostic::ExclusionStateUncertain {
                cause: UncertaintyCause::NoRecord,
                at_risk: vec!["PART_A".to_owned(), "PART_B".to_owned()],
            }]
        );
        assert!(recovery.exclusions.requires_operator_confirmation());
    }
}
