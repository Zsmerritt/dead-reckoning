//! Contact-zone selection for the part-referenced Z probe.
//!
//! After a crash the recovery flow probes the *printed part* to
//! re-reference Z. The probe must touch plastic that (a) already exists
//! — layer N−1, where N is the resume layer — and (b) will be printed
//! over when layer N resumes, so the probe mark is buried. This module
//! computes ranked probe points satisfying both, or declines with a
//! typed reason when no safe point exists.
//!
//! # Geometry / tolerance model
//!
//! Paths are extrusion **centerlines**; widths are unknown at this
//! level. Two rules follow:
//!
//! * a probe point is *on* N−1 plastic only if it lies exactly on an
//!   N−1 extrusion segment (points are sampled along the segment at
//!   parameters [`SAMPLE_TS`], midpoint first, never at endpoints —
//!   endpoints are direction changes where beads bunch up or thin
//!   out);
//! * a probe point is *covered by layer N* when it lies within
//!   [`ContactConfig::coverage_tolerance`] of any layer-N extrusion
//!   centerline (any feature class — coverage is about burying the
//!   mark, not about class). The default 0.25 mm is half of a typical
//!   0.4–0.5 mm extrusion width: a point that close to a centerline
//!   receives plastic from that very bead.
//!
//! # Ranking
//!
//! Candidates are ranked class-first — internal/sparse infill, then
//! solid infill, then inner wall ([`FeatureClass::probe_rank`]); outer
//! walls, surfaces, bridges, gap fill, skirts and support are **never**
//! candidates — then by larger distance from the crash XY, then by
//! longer host segment, then by file order.

use plr_gcode::ByteSpan;
use serde::{Deserialize, Serialize};

use crate::geom;
use crate::model::{FeatureClass, Layer, LayerModel, XySegment};

/// Sample parameters along a host segment, tried in order until one
/// passes the exclusion and coverage filters. Midpoint first, then
/// symmetric interior points; endpoints (0 and 1) are never sampled.
pub const SAMPLE_TS: [f64; 5] = [0.5, 0.35, 0.65, 0.25, 0.75];

/// Tunable parameters for [`select_contact_zone`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactConfig {
    /// No candidate may lie within this XY distance of the crash point
    /// (blob/scar risk), mm.
    pub exclusion_radius: f64,
    /// Minimum host-segment length, mm. Short segments give the probe
    /// tip too little landing tolerance.
    pub min_segment_length: f64,
    /// Maximum distance from a candidate to a layer-N extrusion
    /// centerline for the point to count as covered, mm (see module
    /// docs).
    pub coverage_tolerance: f64,
    /// Maximum number of candidates returned.
    pub max_candidates: usize,
    /// Spiral fraction ([`LayerModel::spiral_fraction`]) at or above
    /// which the model is treated as vase-mode printing.
    pub spiral_threshold: f64,
}

impl Default for ContactConfig {
    /// `exclusion_radius` 5 mm (crash blobs are centimeters at worst,
    /// millimeters typically), `min_segment_length` 2 mm,
    /// `coverage_tolerance` 0.25 mm (half a typical extrusion width),
    /// `max_candidates` 5, `spiral_threshold` 0.5.
    fn default() -> Self {
        Self {
            exclusion_radius: 5.0,
            min_segment_length: 2.0,
            coverage_tolerance: 0.25,
            max_candidates: 5,
            spiral_threshold: 0.5,
        }
    }
}

/// A ranked probe candidate on a layer-N−1 extrusion segment.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeCandidate {
    /// Probe XY (internal frame), exactly on the host segment.
    pub point: [f64; 2],
    /// Top-of-N−1 Z at the host segment (internal frame) — the height
    /// the probe should find plastic at.
    pub z: f64,
    /// Feature class of the host path.
    pub class: FeatureClass,
    /// Byte span of the host segment's source line.
    pub host_span: ByteSpan,
    /// XY length of the host segment.
    pub host_length: f64,
    /// XY distance from the crash point.
    pub distance_from_crash: f64,
    /// The [`SAMPLE_TS`] parameter that produced the point.
    pub sample_t: f64,
}

/// Why probing is declined even though the inputs were valid. All of
/// these signal "fall back to per-layer / manual recovery".
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DeclineReason {
    /// The model prints as a continuous spiral (vase mode): there is no
    /// flat N−1 top to probe.
    VaseMode {
        /// Observed fraction of extruding moves that changed Z.
        spiral_fraction: f64,
    },
    /// Layer N−1 consists only of outer-wall/surface extrusions
    /// (single-wall print): every reachable path is visible.
    SingleWall,
    /// Layer N−1 has no probe-eligible feature class at all (e.g. only
    /// support or skirt plastic).
    NoEligiblePaths,
    /// Eligible paths exist, but every sampled point failed the
    /// exclusion-radius, minimum-length or layer-N-coverage filters.
    NoSafeZone {
        /// Number of eligible segments that were considered.
        segments_considered: usize,
    },
}

/// Successful selector outcome.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ContactOutcome {
    /// Ranked, non-empty probe candidates.
    Candidates(Vec<ProbeCandidate>),
    /// Probing is declined; the recovery flow must degrade.
    Declined(DeclineReason),
}

/// Selector failures (invalid usage, not geometric outcomes).
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize, Deserialize)]
pub enum ContactError {
    /// A parameter was non-finite or out of range.
    #[error("invalid contact-zone parameter {param}")]
    InvalidParams {
        /// Name of the offending parameter.
        param: &'static str,
    },
    /// The resume layer or its predecessor is not in the model.
    #[error("resume layer {resume_layer} out of range (model has {layers} layers, N-1 required)")]
    LayerOutOfRange {
        /// Requested resume layer N.
        resume_layer: u32,
        /// Number of layers in the model.
        layers: usize,
    },
    /// The file carries no `;TYPE:` annotations at all. v1 refuses to
    /// infer feature classes geometrically.
    #[error("no ;TYPE: annotations in the file; refusing to classify geometrically")]
    NoTypeAnnotations,
    /// Layer N−1 deposits plastic but none of it is under a `;TYPE:`
    /// annotation.
    #[error(
        "layer {layer} has no ;TYPE:-annotated deposition; refusing to classify geometrically"
    )]
    UnannotatedLayer {
        /// Index of the unannotated layer (N−1).
        layer: u32,
    },
}

/// Validate config and crash point.
fn validate(crash_xy: [f64; 2], config: &ContactConfig) -> Result<(), ContactError> {
    if !crash_xy[0].is_finite() || !crash_xy[1].is_finite() {
        return Err(ContactError::InvalidParams { param: "crash_xy" });
    }
    if !config.exclusion_radius.is_finite() || config.exclusion_radius < 0.0 {
        return Err(ContactError::InvalidParams {
            param: "exclusion_radius",
        });
    }
    if !config.min_segment_length.is_finite() || config.min_segment_length <= 0.0 {
        return Err(ContactError::InvalidParams {
            param: "min_segment_length",
        });
    }
    if !config.coverage_tolerance.is_finite() || config.coverage_tolerance < 0.0 {
        return Err(ContactError::InvalidParams {
            param: "coverage_tolerance",
        });
    }
    if config.max_candidates == 0 {
        return Err(ContactError::InvalidParams {
            param: "max_candidates",
        });
    }
    if !config.spiral_threshold.is_finite() || config.spiral_threshold <= 0.0 {
        return Err(ContactError::InvalidParams {
            param: "spiral_threshold",
        });
    }
    Ok(())
}

/// True when `point` lies within `tolerance` of any extrusion
/// centerline of `layer`.
fn covered_by(point: [f64; 2], layer: &Layer, tolerance: f64) -> bool {
    layer
        .paths
        .iter()
        .flat_map(|path| path.segments.iter())
        .any(|seg| geom::point_seg_distance(point, seg.start, seg.end) <= tolerance)
}

/// All-finite check for a segment (defends against models constructed
/// by hand or via overflowed relative moves).
fn segment_finite(seg: &XySegment) -> bool {
    seg.start
        .iter()
        .chain(seg.end.iter())
        .all(|v| v.is_finite())
        && seg.z.is_finite()
}

/// First sampled point on `seg` that clears both the exclusion radius
/// and layer-N coverage.
fn sample_segment(
    seg: &XySegment,
    class: FeatureClass,
    crash_xy: [f64; 2],
    layer_n: &Layer,
    config: &ContactConfig,
) -> Option<ProbeCandidate> {
    let length = seg.length();
    for t in SAMPLE_TS {
        let point = seg.point_at(t);
        let distance_from_crash = geom::point_distance(point, crash_xy);
        if distance_from_crash <= config.exclusion_radius {
            continue;
        }
        if !covered_by(point, layer_n, config.coverage_tolerance) {
            continue;
        }
        return Some(ProbeCandidate {
            point,
            z: seg.z,
            class,
            host_span: seg.span,
            host_length: length,
            distance_from_crash,
            sample_t: t,
        });
    }
    None
}

/// Rank order: class rank ascending, distance from crash descending,
/// host length descending, file order ascending.
fn rank_cmp(a: &ProbeCandidate, b: &ProbeCandidate) -> std::cmp::Ordering {
    let rank_a = a.class.probe_rank().unwrap_or(u8::MAX);
    let rank_b = b.class.probe_rank().unwrap_or(u8::MAX);
    rank_a
        .cmp(&rank_b)
        .then_with(|| b.distance_from_crash.total_cmp(&a.distance_from_crash))
        .then_with(|| b.host_length.total_cmp(&a.host_length))
        .then_with(|| a.host_span.start.cmp(&b.host_span.start))
}

/// Select ranked probe candidates for resuming at layer `resume_layer`
/// (N), probing on layer N−1. See the module docs for the geometry
/// model and ranking.
///
/// Returns [`ContactOutcome::Declined`] for vase-mode/single-wall
/// prints and for geometrically empty safe zones; returns an error for
/// invalid parameters, out-of-range layers, and missing `;TYPE:`
/// annotations (v1 never infers classes geometrically).
pub fn select_contact_zone(
    model: &LayerModel,
    resume_layer: u32,
    crash_xy: [f64; 2],
    config: &ContactConfig,
) -> Result<ContactOutcome, ContactError> {
    validate(crash_xy, config)?;
    let Some(prev_index) = resume_layer.checked_sub(1) else {
        return Err(ContactError::LayerOutOfRange {
            resume_layer,
            layers: model.layers.len(),
        });
    };
    let (Some(prev), Some(current)) = (model.layer(prev_index), model.layer(resume_layer)) else {
        return Err(ContactError::LayerOutOfRange {
            resume_layer,
            layers: model.layers.len(),
        });
    };
    if !model.annotated {
        return Err(ContactError::NoTypeAnnotations);
    }
    if !prev.paths.iter().any(|p| p.type_name.is_some()) {
        return Err(ContactError::UnannotatedLayer { layer: prev.index });
    }
    let spiral_fraction = model.spiral_fraction();
    if spiral_fraction >= config.spiral_threshold {
        return Ok(ContactOutcome::Declined(DeclineReason::VaseMode {
            spiral_fraction,
        }));
    }
    // Probe-eligible host segments of N-1 (annotated classes only).
    let eligible: Vec<(FeatureClass, &XySegment)> = prev
        .paths
        .iter()
        .filter(|path| path.type_name.is_some() && path.class.probe_eligible())
        .flat_map(|path| path.segments.iter().map(move |seg| (path.class, seg)))
        .collect();
    if eligible.is_empty() {
        let only_walls = prev
            .paths
            .iter()
            .filter(|path| path.type_name.is_some())
            .all(|path| matches!(path.class, FeatureClass::OuterWall | FeatureClass::Surface));
        return Ok(ContactOutcome::Declined(if only_walls {
            DeclineReason::SingleWall
        } else {
            DeclineReason::NoEligiblePaths
        }));
    }
    let mut candidates: Vec<ProbeCandidate> = Vec::new();
    let mut segments_considered = 0_usize;
    for (class, seg) in &eligible {
        if !segment_finite(seg) || seg.length() < config.min_segment_length {
            continue;
        }
        segments_considered += 1;
        if let Some(candidate) = sample_segment(seg, *class, crash_xy, current, config) {
            candidates.push(candidate);
        }
    }
    if candidates.is_empty() {
        return Ok(ContactOutcome::Declined(DeclineReason::NoSafeZone {
            segments_considered,
        }));
    }
    candidates.sort_by(rank_cmp);
    candidates.truncate(config.max_candidates);
    Ok(ContactOutcome::Candidates(candidates))
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact geometry on integer coordinates
mod tests {
    use super::*;
    use crate::model::{build_layer_model, ModelConfig};
    use plr_gcode::GcodeState;

    fn model_of(text: &str) -> LayerModel {
        build_layer_model(
            GcodeState::new(),
            text.as_bytes(),
            0,
            &ModelConfig::default(),
        )
    }

    /// Two layers; N-1 has inner wall + one long sparse-infill line,
    /// N covers the region with a perpendicular infill line plus the
    /// same inner wall.
    const TWO_LAYER: &str = "G90\nM83\nG92 E0\nG1 Z0.2 F7200\n\
        ;TYPE:Outer wall\nG1 X0 Y0 F9000\nG1 X40 Y0 E1 F1800\nG1 X40 Y40 E1\nG1 X0 Y40 E1\nG1 X0 Y0 E1\n\
        ;TYPE:Inner wall\nG1 X2 Y2 F9000\nG1 X38 Y2 E1 F1800\nG1 X38 Y38 E1\nG1 X2 Y38 E1\nG1 X2 Y2 E1\n\
        ;TYPE:Sparse infill\nG1 X4 Y20 F9000\nG1 X36 Y20 E1 F1800\n\
        G1 Z0.4 F7200\n\
        ;TYPE:Inner wall\nG1 X2 Y2 F9000\nG1 X38 Y2 E1 F1800\nG1 X38 Y38 E1\nG1 X2 Y38 E1\nG1 X2 Y2 E1\n\
        ;TYPE:Sparse infill\nG1 X20 Y4 F9000\nG1 X20 Y36 E1 F1800\n";

    #[test]
    fn selects_and_ranks_infill_over_inner_wall() {
        let m = model_of(TWO_LAYER);
        assert_eq!(m.layers.len(), 2);
        let out = select_contact_zone(&m, 1, [100.0, 100.0], &ContactConfig::default())
            .expect("selection");
        let ContactOutcome::Candidates(cands) = out else {
            panic!("expected candidates, got {out:?}");
        };
        // Infill midpoint (20,20) sits exactly on N's perpendicular
        // infill line x=20 -> covered, and ranks first by class.
        let first = &cands[0];
        assert_eq!(first.class, FeatureClass::InternalInfill);
        assert_eq!(first.point, [20.0, 20.0]);
        assert_eq!(first.z, 0.2);
        assert_eq!(first.sample_t, 0.5);
        assert_eq!(first.host_length, 32.0);
        // Inner-wall candidates follow (covered by N's identical inner
        // wall); outer wall never appears.
        assert!(cands
            .iter()
            .skip(1)
            .all(|c| c.class == FeatureClass::InnerWall));
        assert!(cands.iter().all(|c| c.class != FeatureClass::OuterWall));
        assert!(cands.len() <= ContactConfig::default().max_candidates);
        // Every candidate lies exactly on an N-1 segment and is covered
        // by an N centerline.
        let prev = &m.layers[0];
        let cur = &m.layers[1];
        for c in &cands {
            let on_prev = prev
                .paths
                .iter()
                .flat_map(|p| p.segments.iter())
                .any(|s| geom::point_seg_distance(c.point, s.start, s.end) <= 1e-9);
            assert!(on_prev, "candidate {:?} not on an N-1 segment", c.point);
            assert!(covered_by(c.point, cur, 0.25));
        }
    }

    #[test]
    fn exclusion_radius_rejects_and_falls_back_along_segment() {
        let m = model_of(TWO_LAYER);
        // Crash exactly at the infill midpoint: t=0.5 is excluded, and
        // the fallback samples are off the covering line x=20, so the
        // infill segment yields no candidate; inner wall remains.
        let out =
            select_contact_zone(&m, 1, [20.0, 20.0], &ContactConfig::default()).expect("selection");
        let ContactOutcome::Candidates(cands) = out else {
            panic!("expected candidates, got {out:?}");
        };
        assert!(cands.iter().all(|c| c.class == FeatureClass::InnerWall));
        for c in &cands {
            assert!(
                c.distance_from_crash > ContactConfig::default().exclusion_radius,
                "candidate {:?} within exclusion radius",
                c.point
            );
        }
        // A crash near one wall shifts candidates to samples away from
        // it rather than dropping the wall entirely.
        let out =
            select_contact_zone(&m, 1, [20.0, 2.0], &ContactConfig::default()).expect("selection");
        let ContactOutcome::Candidates(cands) = out else {
            panic!("expected candidates, got {out:?}");
        };
        assert!(cands
            .iter()
            .all(|c| geom::point_distance(c.point, [20.0, 2.0]) > 5.0));
    }

    #[test]
    fn coverage_filter_drops_uncovered_points() {
        // N-1 has infill at y=20; N only prints a wall far away at
        // y=100..: nothing covers the infill -> NoSafeZone.
        let text = "G90\nM83\nG1 Z0.2 F7200\n\
            ;TYPE:Sparse infill\nG1 X0 Y20 F9000\nG1 X40 Y20 E1 F1800\n\
            G1 Z0.4 F7200\n;TYPE:Outer wall\nG1 X0 Y100 F9000\nG1 X40 Y100 E1 F1800\n";
        let m = model_of(text);
        let out =
            select_contact_zone(&m, 1, [200.0, 200.0], &ContactConfig::default()).expect("select");
        assert_eq!(
            out,
            ContactOutcome::Declined(DeclineReason::NoSafeZone {
                segments_considered: 1
            })
        );
    }

    #[test]
    fn min_segment_length_filters_short_hosts() {
        // 1 mm infill snippet is below the 2 mm default.
        let text = "G90\nM83\nG1 Z0.2 F7200\n\
            ;TYPE:Sparse infill\nG1 X0 Y20 F9000\nG1 X1 Y20 E0.1 F1800\n\
            G1 Z0.4 F7200\n;TYPE:Sparse infill\nG1 X0 Y20 F9000\nG1 X1 Y20 E0.1 F1800\n";
        let m = model_of(text);
        let out =
            select_contact_zone(&m, 1, [200.0, 200.0], &ContactConfig::default()).expect("select");
        assert_eq!(
            out,
            ContactOutcome::Declined(DeclineReason::NoSafeZone {
                segments_considered: 0
            })
        );
    }

    #[test]
    fn single_wall_declined() {
        let text = "G90\nM83\nG1 Z0.2 F7200\n\
            ;TYPE:External perimeter\nG1 X0 Y0 F9000\nG1 X40 Y0 E1 F1800\nG1 X40 Y40 E1\n\
            G1 Z0.4 F7200\nG1 X0 Y40 E1\nG1 X0 Y0 E1\n";
        let m = model_of(text);
        let out =
            select_contact_zone(&m, 1, [200.0, 200.0], &ContactConfig::default()).expect("select");
        assert_eq!(out, ContactOutcome::Declined(DeclineReason::SingleWall));
    }

    #[test]
    fn support_only_layer_is_no_eligible_paths_not_single_wall() {
        let text = "G90\nM83\nG1 Z0.2 F7200\n\
            ;TYPE:Support material\nG1 X0 Y0 F9000\nG1 X40 Y0 E1 F1800\n\
            G1 Z0.4 F7200\n;TYPE:Support material\nG1 X40 Y40 E1 F1800\n";
        let m = model_of(text);
        let out =
            select_contact_zone(&m, 1, [200.0, 200.0], &ContactConfig::default()).expect("select");
        assert_eq!(
            out,
            ContactOutcome::Declined(DeclineReason::NoEligiblePaths)
        );
    }

    #[test]
    fn vase_mode_declined() {
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y0 F9000\n;TYPE:External perimeter\nG91\n\
            G1 X1 Z0.02 E0.05 F1800\nG1 X1 Z0.02 E0.05\nG1 X1 Z0.02 E0.05\nG1 X1 Z0.02 E0.05\n\
            G1 X1 Z0.02 E0.05\nG1 X1 Z0.02 E0.05\nG1 X1 Z0.02 E0.05\nG1 X1 Z0.02 E0.05\n";
        let m = model_of(text);
        // The ramp crosses z_epsilon repeatedly -> at least two layers.
        assert!(m.layers.len() >= 2, "got {} layers", m.layers.len());
        let out =
            select_contact_zone(&m, 1, [200.0, 200.0], &ContactConfig::default()).expect("select");
        let ContactOutcome::Declined(DeclineReason::VaseMode { spiral_fraction }) = out else {
            panic!("expected VaseMode, got {out:?}");
        };
        assert_eq!(spiral_fraction, 1.0);
    }

    #[test]
    fn missing_annotations_is_a_typed_error() {
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X0 Y0 F9000\nG1 X40 Y0 E1 F1800\n\
            G1 Z0.4 F7200\nG1 X40 Y40 E1 F1800\n";
        let m = model_of(text);
        assert_eq!(
            select_contact_zone(&m, 1, [0.0, 0.0], &ContactConfig::default()).unwrap_err(),
            ContactError::NoTypeAnnotations
        );
    }

    #[test]
    fn unannotated_previous_layer_is_a_typed_error() {
        // The file has annotations (so it passes the global check) but
        // layer 0's deposition happened before the first ;TYPE:.
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X0 Y0 F9000\nG1 X40 Y0 E1 F1800\n\
            G1 Z0.4 F7200\n;TYPE:Sparse infill\nG1 X40 Y40 E1 F1800\n";
        let m = model_of(text);
        assert!(m.annotated);
        assert_eq!(
            select_contact_zone(&m, 1, [0.0, 0.0], &ContactConfig::default()).unwrap_err(),
            ContactError::UnannotatedLayer { layer: 0 }
        );
    }

    #[test]
    fn layer_out_of_range_errors() {
        let m = model_of(TWO_LAYER);
        let cfg = ContactConfig::default();
        assert_eq!(
            select_contact_zone(&m, 0, [0.0, 0.0], &cfg).unwrap_err(),
            ContactError::LayerOutOfRange {
                resume_layer: 0,
                layers: 2
            }
        );
        assert_eq!(
            select_contact_zone(&m, 2, [0.0, 0.0], &cfg).unwrap_err(),
            ContactError::LayerOutOfRange {
                resume_layer: 2,
                layers: 2
            }
        );
    }

    #[test]
    fn invalid_params_are_typed_errors() {
        let m = model_of(TWO_LAYER);
        let cases: Vec<(ContactConfig, [f64; 2], &str)> = vec![
            (ContactConfig::default(), [f64::NAN, 0.0], "crash_xy"),
            (
                ContactConfig {
                    exclusion_radius: -1.0,
                    ..ContactConfig::default()
                },
                [0.0, 0.0],
                "exclusion_radius",
            ),
            (
                ContactConfig {
                    min_segment_length: 0.0,
                    ..ContactConfig::default()
                },
                [0.0, 0.0],
                "min_segment_length",
            ),
            (
                ContactConfig {
                    coverage_tolerance: f64::INFINITY,
                    ..ContactConfig::default()
                },
                [0.0, 0.0],
                "coverage_tolerance",
            ),
            (
                ContactConfig {
                    max_candidates: 0,
                    ..ContactConfig::default()
                },
                [0.0, 0.0],
                "max_candidates",
            ),
            (
                ContactConfig {
                    spiral_threshold: 0.0,
                    ..ContactConfig::default()
                },
                [0.0, 0.0],
                "spiral_threshold",
            ),
        ];
        for (cfg, crash, param) in cases {
            assert_eq!(
                select_contact_zone(&m, 1, crash, &cfg).unwrap_err(),
                ContactError::InvalidParams { param },
                "{param}"
            );
        }
    }

    #[test]
    fn max_candidates_truncates_after_ranking() {
        let m = model_of(TWO_LAYER);
        let cfg = ContactConfig {
            max_candidates: 1,
            ..ContactConfig::default()
        };
        let out = select_contact_zone(&m, 1, [100.0, 100.0], &cfg).expect("select");
        let ContactOutcome::Candidates(cands) = out else {
            panic!("expected candidates");
        };
        assert_eq!(cands.len(), 1);
        // Truncation happens after ranking: the survivor is the infill.
        assert_eq!(cands[0].class, FeatureClass::InternalInfill);
    }

    #[test]
    fn same_class_prefers_distance_then_length() {
        // Two eligible infill lines; crash near one of them.
        let text = "G90\nM83\nG1 Z0.2 F7200\n\
            ;TYPE:Sparse infill\nG1 X0 Y10 F9000\nG1 X40 Y10 E1 F1800\n\
            G1 X0 Y30 F9000\nG1 X30 Y30 E1 F1800\n\
            G1 Z0.4 F7200\n;TYPE:Sparse infill\nG1 X0 Y10 F9000\nG1 X40 Y10 E1 F1800\n\
            G1 X0 Y30 F9000\nG1 X30 Y30 E1 F1800\n";
        let m = model_of(text);
        let out =
            select_contact_zone(&m, 1, [20.0, 0.0], &ContactConfig::default()).expect("select");
        let ContactOutcome::Candidates(cands) = out else {
            panic!("expected candidates");
        };
        assert_eq!(cands.len(), 2);
        // The farther line (y=30) ranks first.
        assert_eq!(cands[0].point[1], 30.0);
        assert_eq!(cands[1].point[1], 10.0);
        assert!(cands[0].distance_from_crash > cands[1].distance_from_crash);
    }

    #[test]
    fn outcome_serializes() {
        let m = model_of(TWO_LAYER);
        // Crash at (20,100): every candidate distance is an integer, so
        // the JSON float round-trip is exact (serde_json's default
        // float parsing is lossy in the 17th digit otherwise).
        let out =
            select_contact_zone(&m, 1, [20.0, 100.0], &ContactConfig::default()).expect("select");
        let json = serde_json::to_string(&out).expect("serialize");
        let back: ContactOutcome = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(out, back);
        let err_json =
            serde_json::to_string(&ContactError::NoTypeAnnotations).expect("serialize error");
        assert!(err_json.contains("NoTypeAnnotations"));
    }
}
