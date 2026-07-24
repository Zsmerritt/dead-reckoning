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
//! # Structural filtering
//!
//! Coverage and class say the probe mark will be buried and invisible.
//! They say nothing about whether the *part* survives the touch. With
//! [`ContactConfig::structural_checks_enabled`] (the default) every
//! surviving sample is additionally put through
//! [`crate::structure::StructuralAnalysis`] — bed-adhesion footprint,
//! tipping aspect ratio, feature width, edge margin, and for
//! [`ContactMode::Drag`] a clear lateral run — and excluded when any
//! hard criterion fails. When nothing survives the decline is
//! [`DeclineReason::NoStructurallySafePoint`], which carries the
//! measured-versus-required numbers and the largest clear run the layer
//! actually offers, so a guided-jog UI can tell an operator where to go.
//!
//! # Ranking
//!
//! Candidates are ranked class-first — internal/sparse infill, then
//! solid infill, then inner wall ([`FeatureClass::probe_rank`]); outer
//! walls, surfaces, bridges, gap fill, skirts and support are **never**
//! candidates — then by descending structural score, then by larger
//! distance from the crash XY, then by longer host segment, then by file
//! order. With structural checks disabled every score is 0, so the
//! historic class → distance → length → file order remains exactly.

use plr_gcode::ByteSpan;
use serde::{Deserialize, Serialize};

use crate::geom;
use crate::model::{FeatureClass, Layer, LayerModel, XySegment};
use crate::structure::{
    ClearRun, ContactMode, CriterionCheck, CriterionUnit, InvalidInput, StructuralAnalysis,
    StructuralAssessment, StructuralCriterion, StructuralOutcome, StructuralVerdict,
};

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
    /// Run the structural checks ([`crate::structure`]) and exclude
    /// candidates that fail them. The escape hatch: turning this off
    /// restores pure surface-quality selection and is the documented
    /// answer when the geometric assumptions do not hold (a large Z
    /// offset that hides the bed layer, an exotic extrusion width, a
    /// model built from a mid-file window).
    pub structural_checks_enabled: bool,
    /// How the nozzle will touch the part. Drags are gated on a clear
    /// lateral run; taps are not.
    pub contact_mode: ContactMode,
    /// Minimum layer-0 footprint area holding the probed feature to the
    /// bed, mm². See [`crate::structure`] for the physical reasoning.
    pub min_bed_contact_area: f64,
    /// Maximum `height / sqrt(footprint_area)` before a feature is
    /// treated as tippable (dimensionless).
    pub max_aspect_ratio: f64,
    /// Minimum material all round the contact point, mm — also the
    /// half-width of the swath a drag must keep clear.
    pub min_edge_margin: f64,
    /// Minimum narrowest bounding-box dimension of the hosting island,
    /// mm. Thinner features are treated as fins.
    pub min_feature_width: f64,
    /// Nominal extrusion width used to turn centerlines into material,
    /// mm. Drives the area estimate, the raster resolution and the
    /// clearance field.
    pub extrusion_width: f64,
    /// Maximum distance between sampled points of two extrusions for
    /// them to belong to the same island, mm.
    pub island_link_tolerance: f64,
}

impl Default for ContactConfig {
    /// Selection: `exclusion_radius` 5 mm (crash blobs are centimeters
    /// at worst, millimeters typically), `min_segment_length` 2 mm,
    /// `coverage_tolerance` 0.25 mm (half a typical extrusion width),
    /// `max_candidates` 5, `spiral_threshold` 0.5.
    ///
    /// Structure: `structural_checks_enabled` true (a safety check is
    /// worthless opt-in), `contact_mode` [`ContactMode::Tap`],
    /// `min_bed_contact_area` 100 mm² (a 10 × 10 mm first-layer patch —
    /// see [`crate::structure`] for the force argument),
    /// `max_aspect_ratio` 3.0 (the "three times as tall as it is wide"
    /// tipping rule of thumb), `min_edge_margin` 3.0 mm (several nozzle
    /// diameters plus post-crash positional uncertainty),
    /// `min_feature_width` 5.0 mm (about a dozen 0.4 mm extrusions side
    /// by side), `extrusion_width` 0.45 mm (a typical 0.4 mm nozzle line
    /// width) and `island_link_tolerance` 0.6 mm (1.33 extrusion widths:
    /// beads that close cannot be separate material).
    fn default() -> Self {
        Self {
            exclusion_radius: 5.0,
            min_segment_length: 2.0,
            coverage_tolerance: 0.25,
            max_candidates: 5,
            spiral_threshold: 0.5,
            structural_checks_enabled: true,
            contact_mode: ContactMode::Tap,
            min_bed_contact_area: 100.0,
            max_aspect_ratio: 3.0,
            min_edge_margin: 3.0,
            min_feature_width: 5.0,
            extrusion_width: 0.45,
            island_link_tolerance: 0.6,
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
    /// Points that satisfy coverage and exclusion exist, but every one
    /// of them would damage the part. Actionable on purpose: the payload
    /// carries what was measured, what was needed, and the best lateral
    /// run the layer actually offers, so a guided-jog UI can tell an
    /// operator where to go instead of "none found".
    NoStructurallySafePoint {
        /// Structurally evaluated candidate points.
        candidates_evaluated: usize,
        /// The best-ranked rejected candidates (at most
        /// [`ContactConfig::max_candidates`]), each with its failing
        /// criteria and measured-versus-required values.
        rejected: Vec<RejectedCandidate>,
        /// The run length the configured [`ContactMode::Drag`] needed,
        /// mm; `None` for [`ContactMode::Tap`].
        required_run_length: Option<f64>,
        /// The longest clear run found anywhere among the evaluated
        /// candidate points, over [`crate::structure::RUN_DIRECTIONS`]
        /// evenly spaced directions. `None` for
        /// [`ContactMode::Tap`] and when no candidate sat on material.
        largest_clear_run: Option<ClearRun>,
    },
}

/// A candidate that passed coverage and exclusion but failed a
/// structural criterion.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RejectedCandidate {
    /// Probe XY (internal frame) that was evaluated.
    pub point: [f64; 2],
    /// Top-of-N−1 Z at the host segment.
    pub z: f64,
    /// Feature class of the host path.
    pub class: FeatureClass,
    /// XY distance from the crash point.
    pub distance_from_crash: f64,
    /// Index of the layer-N−1 island the point sits on, or `usize::MAX`
    /// when the analysis could not place the point on any island.
    pub island: usize,
    /// [`StructuralVerdict::score`] of the rejected point. Rejections are
    /// listed by descending score, so the head of the list is the set of
    /// near-misses an operator has the best chance of rescuing.
    pub score: f64,
    /// The most severe failing criterion.
    pub primary: StructuralCriterion,
    /// Every failing criterion, in decreasing severity, with its
    /// measured and required values.
    pub failures: Vec<CriterionCheck>,
    /// One-line human-readable summary of the rejection.
    pub summary: String,
}

/// Successful selector outcome.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ContactOutcome {
    /// Ranked, non-empty probe candidates.
    Candidates(Vec<ProbeCandidate>),
    /// Probing is declined; the recovery flow must degrade.
    Declined(DeclineReason),
}

/// [`select_contact_zone`]'s outcome plus the structural evidence behind
/// it.
///
/// [`ProbeCandidate`] deliberately stays a pure geometry record — the
/// recovery planner consumes it verbatim — so the verdicts travel
/// alongside it, index-aligned with `ContactOutcome::Candidates`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactSelection {
    /// The selection itself.
    pub outcome: ContactOutcome,
    /// Structural verdict for each returned candidate, in the same
    /// order. Empty when the selection declined or when
    /// [`ContactConfig::structural_checks_enabled`] is false.
    pub verdicts: Vec<StructuralVerdict>,
    /// Candidates excluded by structural filtering, highest structural
    /// score first (the near-misses), capped at
    /// [`ContactConfig::max_candidates`].
    pub rejected: Vec<RejectedCandidate>,
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
    validate_config(config)
}

/// Validate every [`ContactConfig`] field, naming the offending one.
///
/// Shared with [`crate::structure::StructuralAnalysis::build`] so the
/// standalone structural API refuses exactly the same configurations the
/// selector does.
pub(crate) fn validate_config(config: &ContactConfig) -> Result<(), ContactError> {
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
    for (value, param, allow_zero) in [
        (config.min_bed_contact_area, "min_bed_contact_area", false),
        (config.max_aspect_ratio, "max_aspect_ratio", false),
        (config.min_edge_margin, "min_edge_margin", true),
        (config.min_feature_width, "min_feature_width", false),
        (config.extrusion_width, "extrusion_width", false),
        (config.island_link_tolerance, "island_link_tolerance", false),
    ] {
        let bad = !value.is_finite() || value < 0.0 || (!allow_zero && value <= 0.0);
        if bad {
            return Err(ContactError::InvalidParams { param });
        }
    }
    if let ContactMode::Drag {
        direction,
        run_length,
    } = &config.contact_mode
    {
        let norm = direction[0] * direction[0] + direction[1] * direction[1];
        if !norm.is_finite() || norm <= 0.0 {
            return Err(ContactError::InvalidParams {
                param: "contact_mode.direction",
            });
        }
        if !run_length.is_finite() || *run_length <= 0.0 {
            return Err(ContactError::InvalidParams {
                param: "contact_mode.run_length",
            });
        }
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

/// A candidate with its structural score attached for ranking.
struct Scored {
    candidate: ProbeCandidate,
    /// [`StructuralVerdict::score`], or 0 when structural checks are
    /// disabled — a constant score leaves the historic ordering
    /// (class → distance → length → file order) untouched.
    score: f64,
    verdict: Option<StructuralVerdict>,
}

/// Rank order: class rank ascending, structural score descending,
/// distance from crash descending, host length descending, file order
/// ascending.
fn rank_cmp(a: &Scored, b: &Scored) -> std::cmp::Ordering {
    let rank_a = a.candidate.class.probe_rank().unwrap_or(u8::MAX);
    let rank_b = b.candidate.class.probe_rank().unwrap_or(u8::MAX);
    rank_a
        .cmp(&rank_b)
        .then_with(|| b.score.total_cmp(&a.score))
        .then_with(|| {
            b.candidate
                .distance_from_crash
                .total_cmp(&a.candidate.distance_from_crash)
        })
        .then_with(|| b.candidate.host_length.total_cmp(&a.candidate.host_length))
        .then_with(|| {
            a.candidate
                .host_span
                .start
                .cmp(&b.candidate.host_span.start)
        })
}

/// Rank order for rejected candidates: descending structural score,
/// then class, then distance from crash, then position.
///
/// Score first, unlike the accepted list: the payload exists so an
/// operator can be told what to fix, and the near-misses — high score,
/// one failing criterion — are the actionable ones. Ranking these by
/// distance from the crash would surface the hopeless edge-of-the-plate
/// samples instead.
fn reject_cmp(a: &RejectedCandidate, b: &RejectedCandidate) -> std::cmp::Ordering {
    b.score
        .total_cmp(&a.score)
        .then_with(|| {
            a.class
                .probe_rank()
                .unwrap_or(u8::MAX)
                .cmp(&b.class.probe_rank().unwrap_or(u8::MAX))
        })
        .then_with(|| b.distance_from_crash.total_cmp(&a.distance_from_crash))
        .then_with(|| a.point[0].total_cmp(&b.point[0]))
        .then_with(|| a.point[1].total_cmp(&b.point[1]))
}

/// Turn a failing verdict into a [`RejectedCandidate`].
fn rejection(candidate: &ProbeCandidate, verdict: &StructuralVerdict) -> Option<RejectedCandidate> {
    let StructuralOutcome::Unsafe { primary } = verdict.outcome() else {
        return None;
    };
    Some(RejectedCandidate {
        point: candidate.point,
        z: candidate.z,
        class: candidate.class,
        distance_from_crash: candidate.distance_from_crash,
        island: verdict.island.index,
        score: verdict.score,
        primary,
        failures: verdict.failures().into_iter().cloned().collect(),
        summary: verdict.summary.clone(),
    })
}

/// Structurally classify one candidate, returning either its verdict
/// (safe) or its rejection.
///
/// A point that the analysis cannot place on any island is rejected with
/// an edge-margin failure of zero: it is, literally, entirely margin-less
/// material. That cannot happen for selector-generated candidates (they
/// lie exactly on an N−1 centerline) but a hand-built model can produce
/// it, and silently accepting it would be the one unsafe default here.
fn classify(
    analysis: &StructuralAnalysis,
    candidate: &ProbeCandidate,
    mode: &ContactMode,
) -> Result<StructuralVerdict, RejectedCandidate> {
    match analysis.assess(candidate.point, mode) {
        StructuralAssessment::Evaluated(verdict) => match rejection(candidate, &verdict) {
            Some(rejected) => Err(rejected),
            None => Ok(*verdict),
        },
        StructuralAssessment::OffMaterial { distance, .. } => Err(off_material_rejection(
            candidate,
            distance,
            "the point is not on any island of the layer",
        )),
        StructuralAssessment::InvalidPoint { param } => Err(off_material_rejection(
            candidate,
            0.0,
            match param {
                InvalidInput::Point => "the point is not a finite coordinate",
                InvalidInput::DragDirection => "the drag direction is zero or non-finite",
                InvalidInput::DragRunLength => "the drag run length is not a positive number",
            },
        )),
    }
}

/// Rejection record for a candidate the structural analysis could not
/// place on material.
fn off_material_rejection(
    candidate: &ProbeCandidate,
    distance: f64,
    why: &str,
) -> RejectedCandidate {
    let check = CriterionCheck {
        criterion: StructuralCriterion::EdgeMargin,
        passed: false,
        measured: 0.0,
        threshold: 0.0,
        unit: CriterionUnit::Millimetres,
        reason: format!("{why} (nearest material {distance:.2} mm away)"),
    };
    RejectedCandidate {
        point: candidate.point,
        z: candidate.z,
        class: candidate.class,
        distance_from_crash: candidate.distance_from_crash,
        island: usize::MAX,
        score: 0.0,
        primary: StructuralCriterion::EdgeMargin,
        failures: vec![check],
        summary: format!("unsafe (edge_margin): {why}"),
    }
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
    select_contact_zone_detailed(model, resume_layer, crash_xy, config)
        .map(|selection| selection.outcome)
}

/// [`select_contact_zone`] plus the structural evidence: the verdict for
/// every returned candidate and the structurally rejected ones.
///
/// This is the entry point a UI wants; `select_contact_zone` is the
/// narrow view the recovery planner consumes.
pub fn select_contact_zone_detailed(
    model: &LayerModel,
    resume_layer: u32,
    crash_xy: [f64; 2],
    config: &ContactConfig,
) -> Result<ContactSelection, ContactError> {
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
    let declined = |reason: DeclineReason| {
        Ok(ContactSelection {
            outcome: ContactOutcome::Declined(reason),
            verdicts: Vec::new(),
            rejected: Vec::new(),
        })
    };
    if !model.annotated {
        return Err(ContactError::NoTypeAnnotations);
    }
    if !prev.paths.iter().any(|p| p.type_name.is_some()) {
        return Err(ContactError::UnannotatedLayer { layer: prev.index });
    }
    let spiral_fraction = model.spiral_fraction();
    if spiral_fraction >= config.spiral_threshold {
        return declined(DeclineReason::VaseMode { spiral_fraction });
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
        return declined(if only_walls {
            DeclineReason::SingleWall
        } else {
            DeclineReason::NoEligiblePaths
        });
    }
    let mut sampled: Vec<ProbeCandidate> = Vec::new();
    let mut segments_considered = 0_usize;
    for (class, seg) in &eligible {
        if !segment_finite(seg) || seg.length() < config.min_segment_length {
            continue;
        }
        segments_considered += 1;
        if let Some(candidate) = sample_segment(seg, *class, crash_xy, current, config) {
            sampled.push(candidate);
        }
    }
    if sampled.is_empty() {
        return declined(DeclineReason::NoSafeZone {
            segments_considered,
        });
    }
    if !config.structural_checks_enabled {
        let mut scored: Vec<Scored> = sampled
            .into_iter()
            .map(|candidate| Scored {
                candidate,
                score: 0.0,
                verdict: None,
            })
            .collect();
        scored.sort_by(rank_cmp);
        scored.truncate(config.max_candidates);
        return Ok(ContactSelection {
            outcome: ContactOutcome::Candidates(scored.into_iter().map(|s| s.candidate).collect()),
            verdicts: Vec::new(),
            rejected: Vec::new(),
        });
    }
    structural_selection(model, prev_index, sampled, config)
}

/// Structural filtering, ranking and the actionable decline.
fn structural_selection(
    model: &LayerModel,
    prev_index: u32,
    sampled: Vec<ProbeCandidate>,
    config: &ContactConfig,
) -> Result<ContactSelection, ContactError> {
    let analysis = StructuralAnalysis::build(model, prev_index, config)?;
    let mode = &config.contact_mode;
    let evaluated = sampled.len();
    let mut accepted: Vec<Scored> = Vec::new();
    let mut rejected: Vec<RejectedCandidate> = Vec::new();
    let mut best_run: Option<ClearRun> = None;
    for candidate in sampled {
        match classify(&analysis, &candidate, mode) {
            Ok(verdict) => accepted.push(Scored {
                candidate,
                score: verdict.score,
                verdict: Some(verdict),
            }),
            Err(rejection) => {
                if matches!(mode, ContactMode::Drag { .. }) && rejection.island != usize::MAX {
                    let run = analysis.largest_clear_run(rejection.island, candidate.point);
                    if let Some(run) = run {
                        if best_run.as_ref().is_none_or(|b| run.length > b.length) {
                            best_run = Some(run);
                        }
                    }
                }
                rejected.push(rejection);
            }
        }
    }
    rejected.sort_by(reject_cmp);
    rejected.truncate(config.max_candidates);
    if accepted.is_empty() {
        let required_run_length = match mode {
            ContactMode::Drag { run_length, .. } => Some(*run_length),
            ContactMode::Tap => None,
        };
        return Ok(ContactSelection {
            outcome: ContactOutcome::Declined(DeclineReason::NoStructurallySafePoint {
                candidates_evaluated: evaluated,
                rejected: rejected.clone(),
                required_run_length,
                largest_clear_run: best_run,
            }),
            verdicts: Vec::new(),
            rejected,
        });
    }
    accepted.sort_by(rank_cmp);
    accepted.truncate(config.max_candidates);
    let mut candidates = Vec::with_capacity(accepted.len());
    let mut verdicts = Vec::with_capacity(accepted.len());
    for scored in accepted {
        candidates.push(scored.candidate);
        if let Some(verdict) = scored.verdict {
            verdicts.push(verdict);
        }
    }
    Ok(ContactSelection {
        outcome: ContactOutcome::Candidates(candidates),
        verdicts,
        rejected,
    })
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact geometry on integer coordinates
mod tests {
    use super::*;
    use crate::model::{build_layer_model, ModelConfig};
    use plr_gcode::GcodeState;
    use std::fmt::Write as _;

    /// The pre-structural selector semantics these tests pin: surface
    /// quality and coverage only. The toy geometries below are a couple
    /// of square centimetres of sparse plastic and genuinely fail the
    /// 100 mm² bed-contact bar, so structural filtering is switched off
    /// here and exercised by its own tests further down.
    fn geometric() -> ContactConfig {
        ContactConfig {
            structural_checks_enabled: false,
            ..ContactConfig::default()
        }
    }

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
        let out = select_contact_zone(&m, 1, [100.0, 100.0], &geometric()).expect("selection");
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
        let out = select_contact_zone(&m, 1, [20.0, 20.0], &geometric()).expect("selection");
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
        let out = select_contact_zone(&m, 1, [20.0, 2.0], &geometric()).expect("selection");
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
        let out = select_contact_zone(&m, 1, [200.0, 200.0], &geometric()).expect("select");
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
        let out = select_contact_zone(&m, 1, [200.0, 200.0], &geometric()).expect("select");
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
        let out = select_contact_zone(&m, 1, [200.0, 200.0], &geometric()).expect("select");
        assert_eq!(out, ContactOutcome::Declined(DeclineReason::SingleWall));
    }

    #[test]
    fn support_only_layer_is_no_eligible_paths_not_single_wall() {
        let text = "G90\nM83\nG1 Z0.2 F7200\n\
            ;TYPE:Support material\nG1 X0 Y0 F9000\nG1 X40 Y0 E1 F1800\n\
            G1 Z0.4 F7200\n;TYPE:Support material\nG1 X40 Y40 E1 F1800\n";
        let m = model_of(text);
        let out = select_contact_zone(&m, 1, [200.0, 200.0], &geometric()).expect("select");
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
        let out = select_contact_zone(&m, 1, [200.0, 200.0], &geometric()).expect("select");
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
            select_contact_zone(&m, 1, [0.0, 0.0], &geometric()).unwrap_err(),
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
            select_contact_zone(&m, 1, [0.0, 0.0], &geometric()).unwrap_err(),
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
            ..geometric()
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
        let out = select_contact_zone(&m, 1, [20.0, 0.0], &geometric()).expect("select");
        let ContactOutcome::Candidates(cands) = out else {
            panic!("expected candidates");
        };
        assert_eq!(cands.len(), 2);
        // The farther line (y=30) ranks first.
        assert_eq!(cands[0].point[1], 30.0);
        assert_eq!(cands[1].point[1], 10.0);
        assert!(cands[0].distance_from_crash > cands[1].distance_from_crash);
    }

    /// Scan lines filling a rectangle at the 0.4 mm production line
    /// spacing, so the region rasterizes as solid material.
    fn solid(x_lo: f64, x_hi: f64, y_lo: f64, y_hi: f64) -> String {
        let mut out = String::new();
        for row in 0_i32.. {
            let y = f64::from(row).mul_add(0.4, y_lo);
            if y > y_hi + 1e-9 {
                break;
            }
            let _ = writeln!(out, "G1 X{x_lo} Y{y} F9000\nG1 X{x_hi} Y{y} E1 F1800");
        }
        out
    }

    /// Two identical annotated layers of `body` — identical hatches so
    /// every N−1 point is covered by N and the coverage filter never
    /// interferes with what these tests are about.
    fn stacked(body: &str) -> LayerModel {
        model_of(&format!(
            "G90\nM83\nG92 E0\nG1 Z0.2 F7200\n;TYPE:Internal solid infill\n{body}\
             G1 Z0.4 F7200\n;TYPE:Internal solid infill\n{body}"
        ))
    }

    fn candidates_of(outcome: &ContactOutcome) -> &[ProbeCandidate] {
        match outcome {
            ContactOutcome::Candidates(c) => c,
            ContactOutcome::Declined(reason) => panic!("expected candidates, declined: {reason:?}"),
        }
    }

    #[test]
    fn structural_filtering_accepts_a_plate_and_reports_verdicts() {
        let model = stacked(&solid(0.0, 20.0, 0.0, 20.0));
        let selection =
            select_contact_zone_detailed(&model, 1, [40.0, 40.0], &ContactConfig::default())
                .expect("selection");
        let candidates = candidates_of(&selection.outcome);
        assert!(!candidates.is_empty());
        // One verdict per returned candidate, in the same order, each
        // safe and on the plate's single island.
        assert_eq!(selection.verdicts.len(), candidates.len());
        for (candidate, verdict) in candidates.iter().zip(&selection.verdicts) {
            assert_eq!(verdict.point, candidate.point);
            assert_eq!(verdict.outcome(), StructuralOutcome::Safe);
            assert_eq!(verdict.island.index, 0);
            assert_eq!(verdict.layer, 0);
        }
        // Ranking is by descending structural score within a class.
        for pair in selection.verdicts.windows(2) {
            assert!(
                pair[0].score >= pair[1].score,
                "verdicts not score-ordered: {} then {}",
                pair[0].score,
                pair[1].score
            );
        }
        // The plain entry point agrees with the detailed one.
        let plain = select_contact_zone(&model, 1, [40.0, 40.0], &ContactConfig::default())
            .expect("selection");
        assert_eq!(plain, selection.outcome);
    }

    #[test]
    fn structural_score_outranks_distance_from_the_crash() {
        // Every candidate is a scan-line midpoint at x = 10, so distance
        // from a crash at (10,10) is |y − 10| while the edge margin is
        // the distance to the nearest plate edge. The candidate with the
        // better margin must win even though it is closer to the crash.
        let model = stacked(&solid(0.0, 20.0, 0.0, 20.0));
        let config = ContactConfig {
            exclusion_radius: 3.0,
            max_candidates: 3,
            ..ContactConfig::default()
        };
        let selection =
            select_contact_zone_detailed(&model, 1, [10.0, 10.0], &config).expect("selection");
        let candidates = candidates_of(&selection.outcome);
        let best = candidates[0].point;
        let best_margin = best[1].min(20.0 - best[1]);
        assert!(best_margin > 5.0, "best candidate {best:?} hugs an edge");
        // Without the structural stage the same crash ranks the most
        // distant point first, which is exactly the edge-hugging one.
        let geometric_cfg = ContactConfig {
            structural_checks_enabled: false,
            ..config
        };
        let plain =
            select_contact_zone(&model, 1, [10.0, 10.0], &geometric_cfg).expect("selection");
        let first = candidates_of(&plain)[0].point;
        assert!(first[1].min(20.0 - first[1]) < best_margin);
    }

    #[test]
    fn a_pillar_declines_with_actionable_numbers() {
        // 2.4 mm square column, two layers: fails feature width, edge
        // margin and bed adhesion at once.
        let model = stacked(&solid(0.0, 2.4, 0.0, 2.4));
        let selection =
            select_contact_zone_detailed(&model, 1, [40.0, 40.0], &ContactConfig::default())
                .expect("selection");
        let ContactOutcome::Declined(DeclineReason::NoStructurallySafePoint {
            candidates_evaluated,
            rejected,
            required_run_length,
            largest_clear_run,
        }) = &selection.outcome
        else {
            panic!(
                "expected NoStructurallySafePoint, got {:?}",
                selection.outcome
            );
        };
        assert!(*candidates_evaluated > 0);
        assert!(!rejected.is_empty());
        assert!(rejected.len() <= ContactConfig::default().max_candidates);
        assert_eq!(*required_run_length, None, "tap mode needs no run");
        assert_eq!(*largest_clear_run, None);
        // Every rejection names a criterion and carries the numbers that
        // let a UI say "needs X, measured Y".
        for reject in rejected {
            assert!(!reject.failures.is_empty());
            assert!(reject
                .failures
                .iter()
                .any(|c| c.criterion == reject.primary));
            for check in &reject.failures {
                assert!(!check.passed);
                assert!(check.measured.is_finite() && check.threshold.is_finite());
                assert!(!check.reason.is_empty());
            }
            assert!(reject.summary.starts_with("unsafe ("));
            assert_eq!(reject.island, 0);
        }
        let criteria: Vec<StructuralCriterion> =
            rejected[0].failures.iter().map(|c| c.criterion).collect();
        assert!(criteria.contains(&StructuralCriterion::BedAdhesion));
        assert!(criteria.contains(&StructuralCriterion::FeatureWidth));
        // The rejected list also travels on the selection itself.
        assert_eq!(&selection.rejected, rejected);
        assert!(selection.verdicts.is_empty());
    }

    #[test]
    fn an_impossible_drag_reports_the_run_it_could_offer() {
        // A 20 mm plate cannot host a 30 mm drag at a 3 mm margin. The
        // material spans -0.225..20.225, so the region with 3 mm of
        // clearance is a ~14.45 mm square and no run can exceed its
        // 20.4 mm diagonal; the reported run must be a real, reproducible
        // measurement inside that bound.
        let model = stacked(&solid(0.0, 20.0, 0.0, 20.0));
        let config = ContactConfig {
            contact_mode: ContactMode::Drag {
                direction: [1.0, 0.0],
                run_length: 30.0,
            },
            ..ContactConfig::default()
        };
        let selection =
            select_contact_zone_detailed(&model, 1, [40.0, 40.0], &config).expect("selection");
        let ContactOutcome::Declined(DeclineReason::NoStructurallySafePoint {
            rejected,
            required_run_length,
            largest_clear_run,
            ..
        }) = &selection.outcome
        else {
            panic!(
                "expected NoStructurallySafePoint, got {:?}",
                selection.outcome
            );
        };
        assert_eq!(*required_run_length, Some(30.0));
        let run = largest_clear_run.as_ref().expect("a clear run");
        assert!(run.length > 0.0 && run.length < 30.0, "run {}", run.length);
        assert!(
            run.length <= 20.5,
            "run {} exceeds the clear region",
            run.length
        );
        assert_eq!(run.margin, 3.0);
        assert_eq!(run.island, 0);
        assert!((run.direction[0].hypot(run.direction[1]) - 1.0).abs() < 1e-12);
        // The reported run is reproducible: measuring again from the
        // same start and direction gives exactly the same length, and it
        // starts on one of the rejected candidates.
        let analysis = StructuralAnalysis::build(&model, 0, &config).expect("analysis");
        assert_eq!(
            analysis.clear_run(run.island, run.start, run.direction, 30.0),
            run.length
        );
        // The start is a point on the island (it comes from the whole
        // evaluated set, which is wider than the truncated `rejected`
        // list the payload carries).
        assert_eq!(analysis.island_at(run.start), Some(run.island));
        // It is the best available: no listed candidate's own straight
        // run beats it.
        let straight = rejected
            .iter()
            .flat_map(|r| r.failures.iter())
            .filter(|c| c.criterion == StructuralCriterion::DragRun)
            .map(|c| c.measured)
            .fold(0.0_f64, f64::max);
        assert!(run.length >= straight, "{} < {straight}", run.length);
        // The plate is structurally sound, so nothing is rejected for
        // adhesion, tipping or width: only the run and — for the scan
        // lines that hug an edge — the margin.
        assert!(rejected
            .iter()
            .any(|r| r.primary == StructuralCriterion::DragRun));
        for reject in rejected {
            assert!(
                matches!(
                    reject.primary,
                    StructuralCriterion::DragRun | StructuralCriterion::EdgeMargin
                ),
                "unexpected failure {:?}",
                reject.primary
            );
            for check in &reject.failures {
                if check.criterion == StructuralCriterion::DragRun {
                    assert!(check.measured < 30.0);
                    assert_eq!(check.threshold, 30.0);
                }
            }
        }
        // A run the plate can host is accepted at the same point.
        let ok = ContactConfig {
            contact_mode: ContactMode::Drag {
                direction: [1.0, 0.0],
                run_length: 4.0,
            },
            ..ContactConfig::default()
        };
        let selection =
            select_contact_zone_detailed(&model, 1, [40.0, 40.0], &ok).expect("selection");
        assert!(!candidates_of(&selection.outcome).is_empty());
        assert!(selection
            .verdicts
            .iter()
            .all(|v| v.centroid_alignment.is_some()));
    }

    #[test]
    fn the_escape_hatch_restores_pure_geometric_selection() {
        // The pillar that declines under structural checks still yields
        // candidates when they are turned off.
        let model = stacked(&solid(0.0, 2.4, 0.0, 2.4));
        let selection =
            select_contact_zone_detailed(&model, 1, [40.0, 40.0], &geometric()).expect("selection");
        assert!(!candidates_of(&selection.outcome).is_empty());
        assert!(selection.verdicts.is_empty());
        assert!(selection.rejected.is_empty());
    }

    #[test]
    fn new_config_fields_are_validated_by_name() {
        let model = stacked(&solid(0.0, 20.0, 0.0, 20.0));
        let base = ContactConfig::default();
        let cases: Vec<(ContactConfig, &str)> = vec![
            (
                ContactConfig {
                    min_bed_contact_area: 0.0,
                    ..base.clone()
                },
                "min_bed_contact_area",
            ),
            (
                ContactConfig {
                    max_aspect_ratio: f64::NAN,
                    ..base.clone()
                },
                "max_aspect_ratio",
            ),
            (
                ContactConfig {
                    min_edge_margin: -0.1,
                    ..base.clone()
                },
                "min_edge_margin",
            ),
            (
                ContactConfig {
                    min_feature_width: 0.0,
                    ..base.clone()
                },
                "min_feature_width",
            ),
            (
                ContactConfig {
                    extrusion_width: -0.4,
                    ..base.clone()
                },
                "extrusion_width",
            ),
            (
                ContactConfig {
                    island_link_tolerance: f64::INFINITY,
                    ..base.clone()
                },
                "island_link_tolerance",
            ),
            (
                ContactConfig {
                    contact_mode: ContactMode::Drag {
                        direction: [0.0, 0.0],
                        run_length: 5.0,
                    },
                    ..base.clone()
                },
                "contact_mode.direction",
            ),
            (
                ContactConfig {
                    contact_mode: ContactMode::Drag {
                        direction: [1.0, 0.0],
                        run_length: -1.0,
                    },
                    ..base.clone()
                },
                "contact_mode.run_length",
            ),
        ];
        for (config, param) in cases {
            assert_eq!(
                select_contact_zone(&model, 1, [0.0, 0.0], &config).unwrap_err(),
                ContactError::InvalidParams { param },
                "{param}"
            );
        }
        // A zero edge margin is legal — it disables the margin gate
        // without disabling the rest.
        let permissive = ContactConfig {
            min_edge_margin: 0.0,
            ..base
        };
        assert!(select_contact_zone(&model, 1, [0.0, 0.0], &permissive).is_ok());
    }

    #[test]
    fn structural_declines_survive_a_json_round_trip() {
        let model = stacked(&solid(0.0, 2.4, 0.0, 2.4));
        let outcome = select_contact_zone(&model, 1, [40.0, 40.0], &ContactConfig::default())
            .expect("select");
        let json = serde_json::to_string(&outcome).expect("serialize");
        assert!(json.contains("NoStructurallySafePoint"));
        let back: ContactOutcome = serde_json::from_str(&json).expect("deserialize");
        let (
            ContactOutcome::Declined(DeclineReason::NoStructurallySafePoint { rejected, .. }),
            ContactOutcome::Declined(DeclineReason::NoStructurallySafePoint {
                rejected: back_rejected,
                ..
            }),
        ) = (&outcome, &back)
        else {
            panic!("expected a structural decline both ways");
        };
        assert_eq!(rejected.len(), back_rejected.len());
        assert_eq!(rejected[0].primary, back_rejected[0].primary);
        assert_eq!(rejected[0].summary, back_rejected[0].summary);
    }

    #[test]
    fn outcome_serializes() {
        let m = model_of(TWO_LAYER);
        // Crash at (20,100): every candidate distance is an integer, so
        // the JSON float round-trip is exact (serde_json's default
        // float parsing is lossy in the 17th digit otherwise).
        let out = select_contact_zone(&m, 1, [20.0, 100.0], &geometric()).expect("select");
        let json = serde_json::to_string(&out).expect("serialize");
        let back: ContactOutcome = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(out, back);
        let err_json =
            serde_json::to_string(&ContactError::NoTypeAnnotations).expect("serialize error");
        assert!(err_json.contains("NoTypeAnnotations"));
    }
}
