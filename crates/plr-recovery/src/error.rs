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
    /// The probe temperature band is too narrow to hold
    /// [`crate::build::PROBE_TEMP_HEADROOM`] below the contact ceiling.
    /// Probing AT the ceiling is refused by the Klipper plugin on any PID
    /// overshoot, which wedges the recovery permanently (see the
    /// constant's docs), so the config is refused up front instead.
    #[error(
        "probe temperature band too narrow: probe_temp_min {probe_temp_min} C leaves no room \
         below the contact ceiling {ceiling} C for the required {headroom} C headroom. \
         Lower probe_temp_min, or raise probe_temp_max / max_probe_nozzle_temp \
         (the ceiling is min(probe_temp_max, max_probe_nozzle_temp))"
    )]
    ProbeTempHeadroomUnavailable {
        /// The configured lower bound of the probing band, °C.
        probe_temp_min: f64,
        /// The effective contact ceiling `min(probe_temp_max,
        /// max_probe_nozzle_temp)`, °C.
        ceiling: f64,
        /// The required headroom, °C
        /// ([`crate::build::PROBE_TEMP_HEADROOM`]).
        headroom: f64,
    },
    /// `drag_nozzle_temp` is negative, or sits within
    /// [`crate::build::PROBE_TEMP_HEADROOM`] of the contact ceiling —
    /// which would let the plan command a drag temperature the Klipper
    /// plugin's ceiling gate then refuses, aborting after the Z frame is
    /// declared and wedging the recovery. `0` (the cold-drag opt-out) is
    /// always accepted.
    #[error(
        "drag_nozzle_temp {drag_nozzle_temp} C is outside [0, {ceiling} - {headroom}]: it must \
         leave {headroom} C of headroom below the contact ceiling {ceiling} C \
         (= min(probe_temp_max, max_probe_nozzle_temp)), or be exactly 0 to opt out of \
         heating for the drag. Lower drag_nozzle_temp, or raise probe_temp_max / \
         max_probe_nozzle_temp"
    )]
    DragTempOutOfRange {
        /// The rejected drag hold temperature, °C.
        drag_nozzle_temp: f64,
        /// The effective contact ceiling, °C.
        ceiling: f64,
        /// The required headroom, °C.
        headroom: f64,
    },
    /// `purge_macro` names a `[gcode_macro ...]` that does not exist on
    /// the machine. Planning refuses rather than silently substituting
    /// the built-in purge: the operator asked for specific behaviour, and
    /// quietly extruding filament at a different place and rate is not an
    /// acceptable substitute (unlike the clean-nozzle macro, which
    /// degrades safely to asking the operator).
    #[error(
        "purge_macro names {name:?} but no [gcode_macro {name}] exists in the printer config; \
         refusing to substitute the built-in purge. Add the macro, correct purge_macro, or \
         unset purge_macro to use the built-in purge (or set purge_enable = False)"
    )]
    PurgeMacroMissing {
        /// The configured macro name that was not found.
        name: String,
    },
    /// `purge_z` is negative. The generated recovery file runs in the
    /// TRUE frame, whose zero is the BED SURFACE, so a negative purge Z
    /// drives the nozzle into the bed at print temperature and extrudes
    /// into it. The Z rail's `position_min` is deliberately below the bed
    /// in this design (it gives the shifted-frame probe envelope room), so
    /// it is not a usable floor for this key — hence an explicit refusal.
    #[error(
        "purge_z {purge_z} mm is below the bed. In the recovery file's true frame Z=0 IS the \
         bed surface, so a negative purge_z extrudes into the bed. Set purge_z >= 0 (and at \
         or above the resume Z to clear the part), or unset it to purge at the parked height"
    )]
    PurgeZBelowBed {
        /// The rejected purge Z, mm.
        purge_z: f64,
    },
    /// A NONZERO `drag_nozzle_temp` below
    /// [`crate::build::DRAG_TEMP_FLOOR`]. Such a target makes the
    /// blocking `M109` wait for a passive cooldown, which on an enclosed
    /// or heated-chamber machine can exceed the executor's 15-minute
    /// timeout — or never converge, if chamber ambient is above the
    /// target. `0` (the cold-drag opt-out, which emits no wait) is
    /// exempt.
    #[error(
        "drag_nozzle_temp {drag_nozzle_temp} C is below the {floor} C floor. A nonzero drag \
         temperature makes the plan WAIT (M109) for the nozzle to settle, and on a PID hotend \
         that includes waiting to COOL — on an enclosed or heated-chamber printer a target at \
         or below chamber ambient may never be reached, burning the full 15-minute step \
         timeout on every retry. Raise it to at least {floor}, or set drag_nozzle_temp = 0 \
         for a deliberate cold drag (no heating and no wait at all)"
    )]
    DragTempBelowFloor {
        /// The rejected drag temperature, °C.
        drag_nozzle_temp: f64,
        /// The refusal floor, °C.
        floor: f64,
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
