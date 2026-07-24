//! Probe-envelope arithmetic and the shifted-frame declaration (design
//! doc §6).
//!
//! # The envelope
//!
//! ```text
//! envelope = expected_gap + overshoot + margin
//! ```
//!
//! * `expected_gap` — the Z span of the possible-stop set (plus a sag
//!   allowance), i.e. how far apart the plausible nozzle heights are;
//! * `overshoot` — how far *below the true surface* the descent can end
//!   before the halt is observed. Its form depends on the probe method
//!   (see [`OvershootTerm`]);
//! * `margin` — configured slack on top.
//!
//! ## Overshoot per probe method
//!
//! * **Continuous descent** (`PROBE`: Tap / load-cell) — Klipper keeps
//!   stepping for about 0.15 s after a probe trigger while the
//!   drip-move flush horizon drains, so the toolhead travels
//!   `0.15 s × probe_speed` beyond the trigger point before halting
//!   ([`OvershootTerm::PostTriggerTravel`]).
//! * **ADXL drag staircase** (`PLR_DRAG_PROBE`) — the descent is a
//!   sequence of **fixed-Z** XY drag passes with a bounded Z decrement
//!   of `drag_z_step` between passes. A pass never moves in Z, so there
//!   is no speed-proportional post-trigger travel at all: the only way
//!   the nozzle can end up below the true surface is the staircase
//!   decrement itself. The first contacting pass sits at most
//!   `drag_z_step` below the last non-contacting one, so the overshoot
//!   is exactly `drag_z_step` ([`OvershootTerm::DragStep`]).
//!
//!   **First-pass clearance, by construction.** `PLR_DRAG_PROBE`
//!   treats contact on the very first pass as a typed FAILURE: its
//!   `trigger_z` is the last *clean* pass, so with no clean pass there
//!   is no datum. The plan must therefore start the staircase at
//!   least one `drag_z_step` above the highest plausible surface —
//!   and it does, arithmetically:
//!
//!   ```text
//!   staircase start   = shifted_declare_z
//!                     = position_min + expected_gap + drag_z_step + margin
//!   highest plausible
//!   surface (shifted) = position_min + expected_gap
//!   clearance         = drag_z_step + margin  >=  drag_z_step
//!   ```
//!
//!   (`margin >= 0` is validated), so even against the worst-case
//!   highest surface the first pass clears it by a full `drag_z_step`
//!   plus the margin, and the first *possible* contact pass always has
//!   a clean pass above it. Pinned by the
//!   `drag_start_clears_the_highest_surface` property test.
//!
//! # The shifted frame
//!
//! The recovery plan declares `SET_KINEMATIC_POSITION
//! Z = position_min + envelope` before probing. The probing move then
//! targets `position_min`, so **Klipper's own rail-limit checking
//! structurally bounds the descent**: even with a faulty or
//! disconnected probe (or a dead accelerometer) the toolhead may reach
//! but never pass `position_min`. No trust in the probe is required for
//! the descent bound — only for the measurement.
//!
//! # The speed band (continuous descent only)
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
//!
//! The drag staircase has no descent speed (passes are fixed-Z), so
//! [`OvershootTerm::DragStep`] carries no speed. Its `drag_z_step` must
//! be finite and strictly positive here (totality); the tighter
//! operational band `(0, 0.2]` mm — the FIXED `[plr]` schema band
//! shared with the Klipper plugin — is enforced by
//! [`crate::build::PlanConfig::validate`], which owns all drag
//! tunables.

use serde::{Deserialize, Serialize};

use crate::error::RecoveryError;

/// Seconds of post-trigger travel Klipper performs after a probe
/// triggers (drip-move flush horizon, `klippy/toolhead.py` drip moves).
pub const POST_TRIGGER_TRAVEL_S: f64 = 0.15;

/// Minimum accepted probe speed, mm/s (see module docs).
pub const PROBE_SPEED_MIN: f64 = 1.0;

/// Maximum accepted probe speed, mm/s (see module docs).
pub const PROBE_SPEED_MAX: f64 = 2.0;

/// The overshoot term of the envelope formula: how far below the true
/// surface the descent can end (see the module docs for the
/// derivations).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum OvershootTerm {
    /// Continuous `PROBE` descent: `POST_TRIGGER_TRAVEL_S × probe_speed`
    /// of drip-move travel past the trigger.
    PostTriggerTravel {
        /// Probe speed, mm/s. Must lie in
        /// [[`PROBE_SPEED_MIN`], [`PROBE_SPEED_MAX`]].
        probe_speed: f64,
    },
    /// ADXL drag staircase: passes are fixed-Z, so the bounded
    /// staircase decrement is the entire overshoot.
    DragStep {
        /// Z decrement between drag passes, mm. Must be finite and
        /// strictly positive.
        drag_z_step: f64,
    },
}

impl OvershootTerm {
    /// The overshoot value in mm, after validation.
    fn validate(self) -> Result<f64, RecoveryError> {
        match self {
            OvershootTerm::PostTriggerTravel { probe_speed } => {
                // The band check subsumes the finiteness check: NaN is
                // not contained in any range, infinities are out of
                // band.
                if !(PROBE_SPEED_MIN..=PROBE_SPEED_MAX).contains(&probe_speed) {
                    return Err(RecoveryError::ProbeSpeedOutOfRange { speed: probe_speed });
                }
                Ok(POST_TRIGGER_TRAVEL_S * probe_speed)
            }
            OvershootTerm::DragStep { drag_z_step } => {
                if !drag_z_step.is_finite() {
                    return Err(RecoveryError::NonFinite {
                        field: "drag_z_step",
                    });
                }
                if drag_z_step <= 0.0 {
                    return Err(RecoveryError::InvalidPlanConfig {
                        field: "drag_z_step",
                    });
                }
                Ok(drag_z_step)
            }
        }
    }
}

/// Inputs to the envelope formula.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EnvelopeParams {
    /// Expected nozzle-to-part gap uncertainty, mm: the Z span of the
    /// possible-stop set plus a sag allowance. Must be finite and
    /// non-negative.
    pub expected_gap: f64,
    /// The probe-method-specific overshoot term.
    pub overshoot: OvershootTerm,
    /// Additional slack, mm. Must be finite and non-negative.
    pub margin: f64,
}

/// The computed envelope and the shifted-frame declaration derived from
/// it. Constructed only by [`compute_envelope`]; every field is finite.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Envelope {
    /// The validated inputs.
    pub params: EnvelopeParams,
    /// `expected_gap + overshoot + margin`, mm (see [`OvershootTerm`]
    /// for the overshoot value per probe method).
    pub envelope: f64,
    /// The Z rail's `position_min` (fallback: `[printer]
    /// minimum_z_position`), mm — the anchor of the shifted frame.
    pub position_min: f64,
    /// `position_min + envelope`: the value declared via
    /// `SET_KINEMATIC_POSITION Z=...` before probing.
    pub shifted_declare_z: f64,
}

/// Computes the probe envelope (see module docs for the formula, the
/// speed band, and the per-method overshoot derivation).
///
/// # Errors
///
/// * [`RecoveryError::ProbeSpeedOutOfRange`] — post-trigger speed
///   outside `[1, 2]` mm/s (a NaN speed also lands here: it fails the
///   band check);
/// * [`RecoveryError::NonFinite`] — any other non-finite input or a
///   non-finite result;
/// * [`RecoveryError::InvalidPlanConfig`] — negative gap or margin, or
///   a non-positive `drag_z_step`.
pub fn compute_envelope(
    params: EnvelopeParams,
    position_min: f64,
) -> Result<Envelope, RecoveryError> {
    let overshoot = params.overshoot.validate()?;
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
    let envelope = params.expected_gap + overshoot + params.margin;
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

    use super::{compute_envelope, EnvelopeParams, OvershootTerm, POST_TRIGGER_TRAVEL_S};
    use crate::error::RecoveryError;

    fn params(gap: f64, speed: f64, margin: f64) -> EnvelopeParams {
        EnvelopeParams {
            expected_gap: gap,
            overshoot: OvershootTerm::PostTriggerTravel { probe_speed: speed },
            margin,
        }
    }

    fn drag_params(gap: f64, z_step: f64, margin: f64) -> EnvelopeParams {
        EnvelopeParams {
            expected_gap: gap,
            overshoot: OvershootTerm::DragStep {
                drag_z_step: z_step,
            },
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
    fn drag_overshoot_is_exactly_the_z_step() {
        // Fixed-Z passes have no speed-proportional travel: the
        // envelope grows by drag_z_step, not by 0.15 × anything.
        let e = compute_envelope(drag_params(0.5, 0.05, 0.3), -2.0).unwrap();
        assert_eq!(e.envelope, 0.5 + 0.05 + 0.3);
        assert_eq!(e.shifted_declare_z, -2.0 + e.envelope);
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
    fn drag_z_step_must_be_finite_and_positive() {
        for (bad, non_finite) in [
            (0.0, false),
            (-0.05, false),
            (f64::NAN, true),
            (f64::INFINITY, true),
        ] {
            let err = compute_envelope(drag_params(0.5, bad, 0.3), 0.0).unwrap_err();
            if non_finite {
                assert!(
                    matches!(
                        err,
                        RecoveryError::NonFinite {
                            field: "drag_z_step"
                        }
                    ),
                    "{bad}: {err:?}"
                );
            } else {
                assert!(
                    matches!(
                        err,
                        RecoveryError::InvalidPlanConfig {
                            field: "drag_z_step"
                        }
                    ),
                    "{bad}: {err:?}"
                );
            }
        }
        assert!(compute_envelope(drag_params(0.5, 0.05, 0.3), 0.0).is_ok());
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
