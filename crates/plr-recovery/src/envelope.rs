//! Probe-envelope arithmetic and the shifted-frame declaration (design
//! doc §6).
//!
//! # The envelope
//!
//! ```text
//! envelope = expected_gap + POST_TRIGGER_TRAVEL_S × probe_speed + margin
//! ```
//!
//! * `expected_gap` — the Z span of the possible-stop set (plus a sag
//!   allowance), i.e. how far apart the plausible nozzle heights are;
//! * `POST_TRIGGER_TRAVEL_S × probe_speed` — Klipper keeps stepping for
//!   about 0.15 s after a probe trigger while the drip-move flush
//!   horizon drains, so the toolhead travels `0.15 s × speed` beyond
//!   the trigger point before halting;
//! * `margin` — configured slack on top.
//!
//! # The shifted frame
//!
//! The recovery plan declares `SET_KINEMATIC_POSITION
//! Z = position_min + envelope` before probing. The probing move then
//! targets `position_min`, so **Klipper's own rail-limit checking
//! structurally bounds the descent**: even with a faulty or
//! disconnected probe the toolhead may reach but never pass
//! `position_min`. No trust in the probe is required for the descent
//! bound — only for the measurement.
//!
//! # The speed band
//!
//! Probe speed is hard-capped to `[1, 2]` mm/s:
//!
//! * **Upper bound 2 mm/s** — post-trigger travel is
//!   `0.15 s × speed`; at 1 mm/s that is ~0.15 mm of indentation into
//!   the (warm) part, which the true-Z arithmetic absorbs. Faster
//!   probing deepens the indentation proportionally and starts to mar
//!   the part and bias the nozzle-as-stylus datum.
//! * **Lower bound 1 mm/s** — the safety analysis behind the envelope
//!   formula (trigger latency, drip-flush horizon, indentation depth)
//!   was validated for the `[1, 2]` band only; slower speeds also make
//!   the full-envelope descent take arbitrarily long while the hot
//!   nozzle rests near the part.
//!
//! Speeds outside the band are rejected as
//! [`RecoveryError::ProbeSpeedOutOfRange`], never clamped: clamping
//! would silently substitute a speed the caller did not ask for.

use serde::{Deserialize, Serialize};

use crate::error::RecoveryError;

/// Seconds of post-trigger travel Klipper performs after a probe
/// triggers (drip-move flush horizon, `klippy/toolhead.py` drip moves).
pub const POST_TRIGGER_TRAVEL_S: f64 = 0.15;

/// Minimum accepted probe speed, mm/s (see module docs).
pub const PROBE_SPEED_MIN: f64 = 1.0;

/// Maximum accepted probe speed, mm/s (see module docs).
pub const PROBE_SPEED_MAX: f64 = 2.0;

/// Inputs to the envelope formula.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EnvelopeParams {
    /// Expected nozzle-to-part gap uncertainty, mm: the Z span of the
    /// possible-stop set plus a sag allowance. Must be finite and
    /// non-negative.
    pub expected_gap: f64,
    /// Probe speed, mm/s. Must lie in
    /// [[`PROBE_SPEED_MIN`], [`PROBE_SPEED_MAX`]].
    pub probe_speed: f64,
    /// Additional slack, mm. Must be finite and non-negative.
    pub margin: f64,
}

/// The computed envelope and the shifted-frame declaration derived from
/// it. Constructed only by [`compute_envelope`]; every field is finite.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Envelope {
    /// The validated inputs.
    pub params: EnvelopeParams,
    /// `expected_gap + POST_TRIGGER_TRAVEL_S * probe_speed + margin`,
    /// mm.
    pub envelope: f64,
    /// The Z rail's `position_min` (fallback: `[printer]
    /// minimum_z_position`), mm — the anchor of the shifted frame.
    pub position_min: f64,
    /// `position_min + envelope`: the value declared via
    /// `SET_KINEMATIC_POSITION Z=...` before probing.
    pub shifted_declare_z: f64,
}

/// Computes the probe envelope (see module docs for the formula and
/// the speed band).
///
/// # Errors
///
/// * [`RecoveryError::ProbeSpeedOutOfRange`] — speed outside `[1, 2]`
///   mm/s (a NaN speed also lands here: it fails the band check);
/// * [`RecoveryError::NonFinite`] — any other non-finite input or a
///   non-finite result;
/// * [`RecoveryError::InvalidPlanConfig`] — negative gap or margin.
pub fn compute_envelope(
    params: EnvelopeParams,
    position_min: f64,
) -> Result<Envelope, RecoveryError> {
    // The band check subsumes the finiteness check for the speed: NaN
    // fails every comparison, infinities are outside the band.
    if !(params.probe_speed >= PROBE_SPEED_MIN && params.probe_speed <= PROBE_SPEED_MAX) {
        return Err(RecoveryError::ProbeSpeedOutOfRange {
            speed: params.probe_speed,
        });
    }
    if !params.expected_gap.is_finite() {
        return Err(RecoveryError::NonFinite {
            field: "expected_gap",
        });
    }
    if params.expected_gap < 0.0 {
        return Err(RecoveryError::InvalidPlanConfig {
            field: "expected_gap",
        });
    }
    if !params.margin.is_finite() {
        return Err(RecoveryError::NonFinite { field: "margin" });
    }
    if params.margin < 0.0 {
        return Err(RecoveryError::InvalidPlanConfig { field: "margin" });
    }
    if !position_min.is_finite() {
        return Err(RecoveryError::NonFinite {
            field: "position_min",
        });
    }
    let envelope = params.expected_gap + POST_TRIGGER_TRAVEL_S * params.probe_speed + params.margin;
    let shifted_declare_z = position_min + envelope;
    if !envelope.is_finite() || !shifted_declare_z.is_finite() {
        return Err(RecoveryError::NonFinite { field: "envelope" });
    }
    Ok(Envelope {
        params,
        envelope,
        position_min,
        shifted_declare_z,
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)] // exact arithmetic is intentional here

    use super::{compute_envelope, EnvelopeParams, POST_TRIGGER_TRAVEL_S};
    use crate::error::RecoveryError;

    fn params(gap: f64, speed: f64, margin: f64) -> EnvelopeParams {
        EnvelopeParams {
            expected_gap: gap,
            probe_speed: speed,
            margin,
        }
    }

    #[test]
    fn formula_matches_the_design_doc() {
        let e = compute_envelope(params(0.5, 1.0, 0.3), -2.0).unwrap();
        assert_eq!(e.envelope, 0.5 + POST_TRIGGER_TRAVEL_S + 0.3);
        assert_eq!(e.shifted_declare_z, -2.0 + e.envelope);
        assert_eq!(e.position_min, -2.0);
    }

    #[test]
    fn speed_band_is_hard() {
        for bad in [0.0, 0.99, 2.01, 100.0, -1.0, f64::NAN, f64::INFINITY] {
            let err = compute_envelope(params(0.5, bad, 0.3), 0.0).unwrap_err();
            assert!(
                matches!(err, RecoveryError::ProbeSpeedOutOfRange { .. }),
                "speed {bad} must be rejected as out of range, got {err:?}"
            );
        }
        for ok in [1.0, 1.5, 2.0] {
            assert!(compute_envelope(params(0.5, ok, 0.3), 0.0).is_ok());
        }
    }

    #[test]
    fn hostile_inputs_are_typed_errors() {
        assert!(matches!(
            compute_envelope(params(f64::NAN, 1.0, 0.3), 0.0),
            Err(RecoveryError::NonFinite {
                field: "expected_gap"
            })
        ));
        assert!(matches!(
            compute_envelope(params(0.5, 1.0, f64::INFINITY), 0.0),
            Err(RecoveryError::NonFinite { field: "margin" })
        ));
        assert!(matches!(
            compute_envelope(params(0.5, 1.0, 0.3), f64::NAN),
            Err(RecoveryError::NonFinite {
                field: "position_min"
            })
        ));
        assert!(matches!(
            compute_envelope(params(-0.1, 1.0, 0.3), 0.0),
            Err(RecoveryError::InvalidPlanConfig {
                field: "expected_gap"
            })
        ));
        assert!(matches!(
            compute_envelope(params(0.5, 1.0, -0.1), 0.0),
            Err(RecoveryError::InvalidPlanConfig { field: "margin" })
        ));
        // Overflow to infinity in the sum is caught.
        assert!(matches!(
            compute_envelope(params(f64::MAX, 1.0, f64::MAX), 0.0),
            Err(RecoveryError::NonFinite { field: "envelope" })
        ));
    }
}
