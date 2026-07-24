//! Typed failures of recovery planning.
//!
//! Every invalid input — hostile numbers included — surfaces as a
//! variant here. Nothing in this crate panics on any input (enforced by
//! the totality property tests in `tests/properties.rs`).

use serde::Serialize;

use crate::envelope::{PROBE_SPEED_MAX, PROBE_SPEED_MIN};
use crate::machine::PrereqFailure;
use crate::preflight::PlanRejection;

/// Failures of recovery planning.
///
/// These are *errors* (invalid inputs, unsatisfied prerequisites);
/// defined-but-degraded outcomes such as a declined contact zone are not
/// errors — they surface as [`crate::build::PlanOutcome::ManualFallback`].
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize)]
pub enum RecoveryError {
    /// A numeric input was NaN or infinite. The planner refuses to emit
    /// any plan containing a non-finite number.
    #[error("non-finite value in {field}")]
    NonFinite {
        /// Which input carried the non-finite value.
        field: &'static str,
    },
    /// A WAL-context value was finite but outside its valid domain
    /// (e.g. a zero or negative extrude factor).
    #[error("WAL context value out of domain: {field}")]
    InvalidContext {
        /// Which context field was out of domain.
        field: &'static str,
    },
    /// The requested probe speed is outside the validated
    /// [[`PROBE_SPEED_MIN`], [`PROBE_SPEED_MAX`]] mm/s band (see
    /// [`crate::envelope`] for why the band is hard).
    #[error(
        "probe speed {speed} mm/s outside the validated \
         [{PROBE_SPEED_MIN}, {PROBE_SPEED_MAX}] mm/s band"
    )]
    ProbeSpeedOutOfRange {
        /// The rejected speed, mm/s.
        speed: f64,
    },
    /// A [`crate::build::PlanConfig`] field was non-finite or out of
    /// range.
    #[error("invalid plan config field {field}")]
    InvalidPlanConfig {
        /// Name of the offending field.
        field: &'static str,
    },
    /// One or more machine prerequisites failed
    /// ([`crate::machine::validate_machine`]). Recovery must not be
    /// attempted on this machine until every failure is resolved.
    #[error("machine prerequisites failed: {} check(s) failed", failures.len())]
    MachineRejected {
        /// Every failed check (all are reported, not just the first).
        failures: Vec<PrereqFailure>,
    },
    /// The WAL timeline holds no context snapshot: there is no
    /// interpreter state to restore from.
    #[error("the WAL timeline holds no context snapshot")]
    NoContext,
    /// The WAL context has no `virtual_sdcard` state: no file print was
    /// active, so there is nothing to resume.
    #[error("no virtual_sdcard state in the WAL context; nothing to resume")]
    NoVirtualSd,
    /// The print file is not at the top level of the `virtual_sdcard`
    /// root. `M23` cannot select files in subdirectories, so the resume
    /// plan cannot be built.
    #[error("print file {path} is not at the virtual_sdcard top level; M23 cannot select it")]
    FileNotTopLevel {
        /// The offending absolute path.
        path: String,
    },
    /// The possible-stop set has no trusted Z candidate
    /// ([`plr_reconstruct::PossibleStopSet::z_span`] returned `None`):
    /// the probe envelope cannot be sized.
    #[error("possible-stop set has no trusted Z span; cannot size the probe envelope")]
    NoZSpan,
    /// The contact outcome was `Candidates` but the list was empty
    /// (defensive: `plr-analyzer` documents the list as non-empty).
    #[error("contact outcome carried an empty candidate list")]
    NoProbeCandidates,
    /// No print nozzle temperature could be established from the WAL or
    /// the file scan: resuming with a cold or unknown-temperature
    /// nozzle is refused.
    #[error("no nozzle print temperature in the WAL or the file scan; refusing a cold resume")]
    NoNozzleTarget,
    /// A name that must be embedded in a command contained characters
    /// that would corrupt the command line.
    #[error("invalid {field} name {name:?}: cannot be embedded in a command")]
    InvalidName {
        /// Which input the name came from.
        field: &'static str,
        /// The offending name.
        name: String,
    },
    /// The whole-itinerary pre-flight ([`crate::preflight`]) found one
    /// or more commanded coordinates outside the machine's bounds or
    /// disagreeing with the selected contact zone. Every violation is
    /// listed.
    #[error(transparent)]
    ItineraryRejected(#[from] PlanRejection),
}

#[cfg(test)]
mod tests {
    use super::RecoveryError;

    #[test]
    fn displays_carry_the_relevant_detail() {
        let e = RecoveryError::ProbeSpeedOutOfRange { speed: 3.5 };
        let msg = e.to_string();
        assert!(msg.contains("3.5"));
        assert!(msg.contains("[1, 2]"));

        let e = RecoveryError::MachineRejected { failures: vec![] };
        assert!(e.to_string().contains("0 check(s)"));

        let e = RecoveryError::FileNotTopLevel {
            path: "/x/sub/f.gcode".to_owned(),
        };
        assert!(e.to_string().contains("/x/sub/f.gcode"));

        let e = RecoveryError::InvalidName {
            field: "exclude_object",
            name: "a b".to_owned(),
        };
        assert!(e.to_string().contains("exclude_object"));
    }

    #[test]
    fn errors_serialize() {
        let e = RecoveryError::NonFinite { field: "gap" };
        let json = serde_json::to_string(&e).unwrap();
        assert!(json.contains("NonFinite"));
    }
}
