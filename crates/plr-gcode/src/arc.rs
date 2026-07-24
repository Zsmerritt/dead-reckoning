//! G2/G3 arc decomposition, a faithful port of Klipper's
//! `klippy/extras/gcode_arcs.py` (`ArcSupport.planArc`, itself derived
//! from Marlin's `plan_arc`).
//!
//! Semantics replicated from the reference source:
//!
//! * chord length is `resolution` mm (`mm_per_arc_segment`, Klipper
//!   default 1.0, `gcode_arcs.py:31`); segment count is
//!   `max(1, floor(travel / resolution))` (line 140);
//! * I/J center-offset form only. The R (radius) form is parsed but
//!   **rejected with an error**, exactly as Klipper does
//!   (gcode_arcs.py:72-73) — decomposing it here would invent behavior
//!   the real firmware never executed;
//! * G17/G18/G19 planes are all supported with the same axis mapping as
//!   gcode_arcs.py:79-87 (the remaining axis travels helically);
//! * a zero angular travel with matching planar endpoints emits a full
//!   circle (lines 125-130);
//! * E is distributed linearly across segments; in absolute-extrude mode
//!   each segment receives an accumulated absolute target, in relative
//!   mode each segment receives the same per-segment delta
//!   (lines 146-176). When the E delta is zero no segment carries an E
//!   word at all (`if e_per_move:`, line 173);
//! * F, when present, is attached to every segment (lines 177-178).
//!
//! Divergences: Klipper happily produces unbounded segment counts; this
//! port refuses more than [`MAX_ARC_SEGMENTS`] to bound recovery-time
//! memory (a sane sliced file stays far below it). Non-finite inputs
//! are rejected with [`ArcError::NonFiniteInput`] instead of producing
//! a garbage decomposition (see the variant docs). A successful return
//! always contains at least one chord, matching Klipper's
//! `max(1., ...)` segment rule.

use serde::{Deserialize, Serialize};

/// Upper bound on the number of chords a single arc may decompose into.
pub const MAX_ARC_SEGMENTS: u32 = 1_000_000;

/// Errors raised while validating or decomposing a G2/G3 command.
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize, Deserialize)]
pub enum ArcError {
    /// Klipper: "G2/G3 does not support relative move mode"
    /// (gcode_arcs.py:62-63).
    #[error("G2/G3 does not support relative move mode")]
    RelativeMode,
    /// Klipper: "G2/G3 does not support R moves" (gcode_arcs.py:72-73).
    #[error("G2/G3 does not support R moves")]
    RadiusForm,
    /// Klipper: "G2/G3 requires IJ, IK or JK parameters"
    /// (gcode_arcs.py:89-90).
    #[error("G2/G3 requires IJ, IK or JK parameters")]
    MissingOffsets,
    /// The configured arc resolution is not a positive finite number.
    #[error("arc resolution must be a positive finite number, got {value}")]
    InvalidResolution {
        /// The rejected resolution value.
        value: f64,
    },
    /// An input coordinate, offset, or word is `nan`/`inf`.
    ///
    /// Divergence by necessity from Klipper: `CPython`'s `float('nan')`
    /// would propagate into garbage chords that this port would then
    /// return as a *successful* decomposition — silently wrong, which
    /// this crate forbids. Unreachable via [`crate::state::GcodeState`]
    /// (its parameter parsing already rejects non-finite values), but
    /// `plan_arc` is public API and must be total on its own.
    #[error("non-finite arc input: {field}")]
    NonFiniteInput {
        /// Which input field was non-finite (`current`, `target`,
        /// `offset`, `e_param`, or `f_param`).
        field: String,
    },
    /// The arc would decompose into more than [`MAX_ARC_SEGMENTS`]
    /// chords (divergence from Klipper, which has no cap).
    #[error("arc decomposes into {segments} segments (cap {MAX_ARC_SEGMENTS})")]
    TooManySegments {
        /// The computed (uncapped) segment count.
        segments: f64,
    },
}

/// Arc plane selected by G17/G18/G19 (gcode_arcs.py:16-18, 51-58).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ArcPlane {
    /// G17 — XY plane, Z helical (the default, `gcode_arcs.py:43`).
    #[default]
    Xy,
    /// G18 — XZ plane, Y helical.
    Xz,
    /// G19 — YZ plane, X helical.
    Yz,
}

/// One of the three coordinate axes, used for panic-free plane mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    fn of4(self, p: [f64; 4]) -> f64 {
        match self {
            Axis::X => p[0],
            Axis::Y => p[1],
            Axis::Z => p[2],
        }
    }

    fn of3(self, p: [f64; 3]) -> f64 {
        match self {
            Axis::X => p[0],
            Axis::Y => p[1],
            Axis::Z => p[2],
        }
    }

    fn set3(self, p: &mut [f64; 3], v: f64) {
        match self {
            Axis::X => p[0] = v,
            Axis::Y => p[1] = v,
            Axis::Z => p[2] = v,
        }
    }
}

impl ArcPlane {
    /// `(alpha, beta, helical)` axis mapping (gcode_arcs.py:79-87).
    fn axes(self) -> (Axis, Axis, Axis) {
        match self {
            ArcPlane::Xy => (Axis::X, Axis::Y, Axis::Z),
            ArcPlane::Xz => (Axis::X, Axis::Z, Axis::Y),
            ArcPlane::Yz => (Axis::Y, Axis::Z, Axis::X),
        }
    }
}

/// Inputs to [`plan_arc`], all in g-code coordinates.
#[derive(Debug, Clone, PartialEq)]
pub struct ArcRequest {
    /// Current g-code position (X, Y, Z, E) — Klipper's
    /// `gcode_position` from `gcode_move.get_status()`.
    pub current: [f64; 4],
    /// Target XYZ in g-code coordinates (absent words default to the
    /// current position, as in gcode_arcs.py:68-70).
    pub target: [f64; 3],
    /// Planar center offset: `(I, J)` for G17, `(I, K)` for G18,
    /// `(J, K)` for G19 (gcode_arcs.py:78-87).
    pub offset: (f64, f64),
    /// Arc plane (G17/G18/G19).
    pub plane: ArcPlane,
    /// True for G2 (clockwise), false for G3.
    pub clockwise: bool,
    /// Current absolute/relative extrude mode (M82/M83).
    pub absolute_extrude: bool,
    /// The E word, if present (g-code scale, before extrude-factor).
    pub e_param: Option<f64>,
    /// The F word, if present (mm/min).
    pub f_param: Option<f64>,
    /// Chord length in mm (`mm_per_arc_segment`); must be positive.
    pub resolution: f64,
}

/// One chord of a decomposed arc, expressed as the G1 parameters Klipper
/// would synthesize (gcode_arcs.py:171-180). Feed through the normal G1
/// path: `target` values are absolute g-code coordinates, `e` follows
/// the current extrude mode and is *not* yet extrude-factor scaled.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ArcSegment {
    /// Absolute XYZ target of this chord in g-code coordinates.
    pub target: [f64; 3],
    /// E word for this chord, if the arc extrudes.
    pub e: Option<f64>,
    /// F word (mm/min), replicated on every chord when given.
    pub f: Option<f64>,
}

/// Reject non-finite inputs up front (see [`ArcError::NonFiniteInput`]):
/// NaN poisons the segment-count arithmetic and would otherwise yield a
/// successful garbage decomposition. After this check, `Ok` from
/// [`plan_arc`] guarantees at least one chord (Klipper's `max(1, ...)`
/// rule).
fn reject_non_finite(req: &ArcRequest) -> Result<(), ArcError> {
    let field: &'static str = if !req.current.iter().all(|v| v.is_finite()) {
        "current"
    } else if !req.target.iter().all(|v| v.is_finite()) {
        "target"
    } else if !(req.offset.0.is_finite() && req.offset.1.is_finite()) {
        "offset"
    } else if req.e_param.is_some_and(|v| !v.is_finite()) {
        "e_param"
    } else if req.f_param.is_some_and(|v| !v.is_finite()) {
        "f_param"
    } else {
        return Ok(());
    };
    Err(ArcError::NonFiniteInput {
        field: field.to_string(),
    })
}

/// Decompose an arc into chords, replicating `planArc`
/// (gcode_arcs.py:104-180) operation for operation.
///
/// Mode validation (absolute-coordinates requirement, R rejection,
/// offset-presence check) is the caller's job, mirroring Klipper's
/// `_cmd_inner`; [`crate::state::GcodeState`] performs it before calling
/// here. This function validates what `planArc` itself relies on:
/// resolution and input finiteness. A successful return always contains
/// at least one chord.
#[allow(clippy::similar_names)] // r_p/r_q etc. mirror the Klipper source names.
pub fn plan_arc(req: &ArcRequest) -> Result<Vec<ArcSegment>, ArcError> {
    if req.resolution <= 0.0 || !req.resolution.is_finite() {
        return Err(ArcError::InvalidResolution {
            value: req.resolution,
        });
    }
    reject_non_finite(req)?;
    let (alpha, beta, helical) = req.plane.axes();

    // Radius vector from center to current location (lines 109-111).
    let r_p = -req.offset.0;
    let r_q = -req.offset.1;

    // Determine angular travel (lines 113-123). Note Python
    // `math.atan2(y, x)` maps to Rust `y.atan2(x)`.
    let center_p = alpha.of4(req.current) - r_p;
    let center_q = beta.of4(req.current) - r_q;
    let rt_alpha = alpha.of3(req.target) - center_p;
    let rt_beta = beta.of3(req.target) - center_q;
    let mut angular_travel = (r_p * rt_beta - r_q * rt_alpha).atan2(r_p * rt_alpha + r_q * rt_beta);
    if angular_travel < 0.0 {
        angular_travel += 2.0 * std::f64::consts::PI;
    }
    if req.clockwise {
        angular_travel -= 2.0 * std::f64::consts::PI;
    }

    // Full circle when rotation is zero and the planar target equals the
    // planar start (lines 125-130); exact float equality as in Klipper.
    #[allow(clippy::float_cmp)]
    let full_circle = angular_travel == 0.0
        && alpha.of4(req.current) == alpha.of3(req.target)
        && beta.of4(req.current) == beta.of3(req.target);
    if full_circle {
        angular_travel = 2.0 * std::f64::consts::PI;
    }

    // Determine number of segments (lines 132-140).
    let linear_travel = helical.of3(req.target) - helical.of4(req.current);
    let radius = r_p.hypot(r_q);
    let flat_mm = radius * angular_travel;
    let mm_of_travel = if linear_travel == 0.0 {
        flat_mm.abs()
    } else {
        flat_mm.hypot(linear_travel)
    };
    let segments_f = (mm_of_travel / req.resolution).floor().max(1.0);
    if segments_f > f64::from(MAX_ARC_SEGMENTS) {
        return Err(ArcError::TooManySegments {
            segments: segments_f,
        });
    }
    // Bounded above by MAX_ARC_SEGMENTS and below by 1, so the cast is
    // exact.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let seg_count = segments_f as u32;

    // Generate coordinates (lines 142-180).
    let theta_per_segment = angular_travel / segments_f;
    let linear_per_segment = linear_travel / segments_f;

    let (mut e_base, e_per_move) = match req.e_param {
        None => (0.0, 0.0),
        Some(as_e) => {
            let base = if req.absolute_extrude {
                req.current[3]
            } else {
                0.0
            };
            (base, (as_e - base) / segments_f)
        }
    };

    let mut out = Vec::with_capacity(seg_count as usize);
    for i in 1..=seg_count {
        let fi = f64::from(i);
        let dist_helical = fi * linear_per_segment;
        let c_theta = fi * theta_per_segment;
        let cos_ti = c_theta.cos();
        let sin_ti = c_theta.sin();
        let r_pi = -req.offset.0 * cos_ti + req.offset.1 * sin_ti;
        let r_qi = -req.offset.0 * sin_ti - req.offset.1 * cos_ti;

        let mut c = [0.0_f64; 3];
        alpha.set3(&mut c, center_p + r_pi);
        beta.set3(&mut c, center_q + r_qi);
        helical.set3(&mut c, helical.of4(req.current) + dist_helical);
        if i == seg_count {
            c = req.target;
        }
        let e = if e_per_move == 0.0 {
            None
        } else {
            let v = e_base + e_per_move;
            if req.absolute_extrude {
                e_base = v;
            }
            Some(v)
        };
        out.push(ArcSegment {
            target: c,
            e,
            f: req.f_param,
        });
    }
    Ok(out)
}

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::cast_precision_loss)] // exactness assertions; small counts
mod tests {
    use super::*;

    fn xy_request() -> ArcRequest {
        ArcRequest {
            current: [10.0, 0.0, 0.4, 5.0],
            target: [0.0, 10.0, 0.4],
            offset: (-10.0, 0.0), // center at origin, R=10
            plane: ArcPlane::Xy,
            clockwise: false,
            absolute_extrude: true,
            e_param: None,
            f_param: None,
            resolution: 1.0,
        }
    }

    #[test]
    fn quarter_circle_segment_count_matches_klipper_formula() {
        // Arc length = 10 * pi/2 = 15.7079..., resolution 1.0 =>
        // floor(15.707) = 15 segments.
        let segs = plan_arc(&xy_request()).unwrap();
        assert_eq!(segs.len(), 15);
        // Endpoint is exactly the requested target (planArc line 169-170).
        assert_eq!(segs.last().unwrap().target, [0.0, 10.0, 0.4]);
        // Intermediate points lie on the circle.
        for s in &segs {
            let r = s.target[0].hypot(s.target[1]);
            assert!((r - 10.0).abs() < 1e-9, "off-circle point {:?}", s.target);
        }
        // No E, no F.
        assert!(segs.iter().all(|s| s.e.is_none() && s.f.is_none()));
    }

    #[test]
    fn clockwise_reverses_direction() {
        let mut req = xy_request();
        req.clockwise = true;
        let segs = plan_arc(&req).unwrap();
        // Clockwise from (10,0) to (0,10) is the long way: 3/4 turn,
        // 47 segments (floor(10 * 3pi/2) = floor(47.12)).
        assert_eq!(segs.len(), 47);
        // First segment heads into negative Y (clockwise).
        assert!(segs.first().unwrap().target[1] < 0.0);
        assert_eq!(segs.last().unwrap().target, [0.0, 10.0, 0.4]);
    }

    #[test]
    fn full_circle_when_target_equals_current() {
        let mut req = xy_request();
        req.target = [10.0, 0.0, 0.4];
        let segs = plan_arc(&req).unwrap();
        // Circumference 2*pi*10 = 62.83 -> 62 segments.
        assert_eq!(segs.len(), 62);
        assert_eq!(segs.last().unwrap().target, [10.0, 0.0, 0.4]);
    }

    #[test]
    fn helical_z_ramps_linearly() {
        let mut req = xy_request();
        req.target = [0.0, 10.0, 2.4]; // +2.0 in Z across the arc
        let segs = plan_arc(&req).unwrap();
        // mm_of_travel = hypot(15.707, 2.0) = 15.83 -> 15 segments.
        assert_eq!(segs.len(), 15);
        let mut prev_z = req.current[2];
        for s in &segs {
            assert!(s.target[2] >= prev_z, "Z must ramp monotonically");
            prev_z = s.target[2];
        }
        assert_eq!(segs.last().unwrap().target[2], 2.4);
    }

    #[test]
    fn absolute_e_accumulates_relative_e_repeats() {
        let mut req = xy_request();
        req.e_param = Some(8.0); // from E=5.0 -> 3.0 of extrusion
        let segs = plan_arc(&req).unwrap();
        let n = segs.len() as f64;
        let e_per = (8.0 - 5.0) / n;
        let first_e = segs.first().unwrap().e.unwrap();
        assert!((first_e - (5.0 + e_per)).abs() < 1e-12);
        let last_e = segs.last().unwrap().e.unwrap();
        assert!(
            (last_e - 8.0).abs() < 1e-9,
            "absolute E converges to target"
        );

        req.absolute_extrude = false;
        req.e_param = Some(3.0); // relative: 3.0 total
        let segs = plan_arc(&req).unwrap();
        let per = 3.0 / segs.len() as f64;
        for s in &segs {
            assert!((s.e.unwrap() - per).abs() < 1e-12, "same delta each chord");
        }
        let total: f64 = segs.iter().map(|s| s.e.unwrap()).sum();
        assert!((total - 3.0).abs() < 1e-9, "relative E conserved");
    }

    #[test]
    fn zero_e_delta_emits_no_e_words() {
        let mut req = xy_request();
        req.e_param = Some(5.0); // equals current E -> zero delta
        let segs = plan_arc(&req).unwrap();
        assert!(segs.iter().all(|s| s.e.is_none()));
    }

    #[test]
    fn f_word_replicated_on_every_chord() {
        let mut req = xy_request();
        req.f_param = Some(1800.0);
        let segs = plan_arc(&req).unwrap();
        assert!(segs.iter().all(|s| s.f == Some(1800.0)));
    }

    #[test]
    fn tiny_arc_is_single_chord() {
        let mut req = xy_request();
        req.current = [0.0, 0.0, 0.0, 0.0];
        req.target = [0.02, 0.02, 0.0];
        req.offset = (0.02, 0.0);
        let segs = plan_arc(&req).unwrap();
        assert_eq!(segs.len(), 1);
        assert_eq!(segs.first().unwrap().target, [0.02, 0.02, 0.0]);
    }

    #[test]
    fn xz_plane_mapping() {
        // G18: alpha=X, beta=Z, helical=Y (gcode_arcs.py:80-83).
        let req = ArcRequest {
            current: [10.0, 0.0, 0.0, 0.0],
            target: [0.0, 0.0, 10.0],
            offset: (-10.0, 0.0), // (I, K): center at x=0,z=0
            plane: ArcPlane::Xz,
            clockwise: false,
            absolute_extrude: true,
            e_param: None,
            f_param: None,
            resolution: 1.0,
        };
        let segs = plan_arc(&req).unwrap();
        assert_eq!(segs.len(), 15);
        for s in &segs {
            let r = s.target[0].hypot(s.target[2]);
            assert!((r - 10.0).abs() < 1e-9);
            assert!(s.target[1].abs() < 1e-12, "Y is helical and unchanged");
        }
        assert_eq!(segs.last().unwrap().target, [0.0, 0.0, 10.0]);
    }

    #[test]
    fn yz_plane_mapping() {
        // G19: alpha=Y, beta=Z, helical=X (gcode_arcs.py:84-87).
        let req = ArcRequest {
            current: [0.0, 10.0, 0.0, 0.0],
            target: [0.0, 0.0, 10.0],
            offset: (-10.0, 0.0), // (J, K)
            plane: ArcPlane::Yz,
            clockwise: false,
            absolute_extrude: true,
            e_param: None,
            f_param: None,
            resolution: 1.0,
        };
        let segs = plan_arc(&req).unwrap();
        for s in &segs {
            let r = s.target[1].hypot(s.target[2]);
            assert!((r - 10.0).abs() < 1e-9);
        }
        assert_eq!(segs.last().unwrap().target, [0.0, 0.0, 10.0]);
    }

    #[test]
    fn invalid_resolution_rejected() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut req = xy_request();
            req.resolution = bad;
            assert!(matches!(
                plan_arc(&req),
                Err(ArcError::InvalidResolution { .. })
            ));
        }
    }

    #[test]
    fn non_finite_inputs_rejected_per_field() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let cases: Vec<(&str, ArcRequest)> = vec![
                ("current", {
                    let mut r = xy_request();
                    r.current[0] = bad;
                    r
                }),
                ("current", {
                    let mut r = xy_request();
                    r.current[3] = bad; // E component counts too
                    r
                }),
                ("target", {
                    let mut r = xy_request();
                    r.target[2] = bad;
                    r
                }),
                ("offset", {
                    let mut r = xy_request();
                    r.offset.0 = bad;
                    r
                }),
                ("offset", {
                    let mut r = xy_request();
                    r.offset.1 = bad;
                    r
                }),
                ("e_param", {
                    let mut r = xy_request();
                    r.e_param = Some(bad);
                    r
                }),
                ("f_param", {
                    let mut r = xy_request();
                    r.f_param = Some(bad);
                    r
                }),
            ];
            for (field, req) in cases {
                match plan_arc(&req) {
                    Err(ArcError::NonFiniteInput { field: f }) => {
                        assert_eq!(f, field, "wrong field for {bad}");
                    }
                    other => panic!("{field}={bad}: expected NonFiniteInput, got {other:?}"),
                }
            }
        }
    }

    #[test]
    fn huge_finite_inputs_stay_klipper_faithful() {
        // Finite-but-overflowing geometry (radius -> inf, angular 0)
        // lands in CPython's max(1., floor(nan)) == 1.0 path: a single
        // chord snapped to the target. Klipper does the same; this is
        // not rejected.
        let mut req = xy_request();
        req.offset = (1e308, 1e308);
        req.target = [10.0, 0.0, 0.4];
        req.current = [10.0, 0.0, 0.4, 0.0];
        let segs = plan_arc(&req).unwrap();
        assert_eq!(segs.len(), 1);
        assert_eq!(segs.first().unwrap().target, [10.0, 0.0, 0.4]);
    }

    #[test]
    fn segment_cap_enforced() {
        let mut req = xy_request();
        req.resolution = 1e-9;
        assert!(matches!(
            plan_arc(&req),
            Err(ArcError::TooManySegments { .. })
        ));
    }
}
