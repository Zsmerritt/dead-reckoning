//! The top-level orchestration: scan + heartbeat + file tail in,
//! [`Reconstruction`] out.

use plr_wal::{HeartbeatRecovery, RecoveryScan};

use crate::config::ReconstructConfig;
use crate::error::ReconstructError;
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

/// A full recovery reconstruction: timeline, window, and stop set.
#[derive(Debug, Clone, PartialEq)]
pub struct RecoveryReconstruction {
    /// The ingested, validated WAL timeline.
    pub timeline: WalTimeline,
    /// The stop window `[t_a, t_b]` and crash class.
    pub window: StopWindow,
    /// The possible-stop set.
    pub stop_set: PossibleStopSet,
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
    let timeline = ingest(inputs.scan, inputs.heartbeat);
    if timeline.clean_shutdown {
        return Ok(Reconstruction::CleanShutdown(Box::new(timeline)));
    }
    let window = compute_stop_window(&timeline, inputs.receive_seq.as_ref(), config)?;
    let stop_set = compute_stop_set(&timeline, &window, inputs.file_tail.as_ref(), config)?;
    Ok(Reconstruction::Recovery(Box::new(RecoveryReconstruction {
        timeline,
        window,
        stop_set,
    })))
}

#[cfg(test)]
mod tests {
    use plr_wal::{Marker, MarkerKind, WalRecord};

    use super::{reconstruct, ReconstructInputs, Reconstruction};
    use crate::config::ReconstructConfig;
    use crate::error::ReconstructError;
    use crate::testutil::{context_at, heartbeat_at, scan_of};
    use crate::window::CrashClass;

    fn inputs(scan: &plr_wal::RecoveryScan) -> ReconstructInputs<'_> {
        ReconstructInputs {
            scan,
            heartbeat: None,
            file_tail: None,
            receive_seq: None,
        }
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
    }
}
