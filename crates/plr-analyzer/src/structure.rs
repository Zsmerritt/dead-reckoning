//! Structural safety for contact-point selection: can the part survive
//! being touched here?
//!
//! [`crate::contact`] answers *"where is there buried plastic to probe?"*
//! — a question purely about surface quality and coverage. It has no
//! notion of whether the feature under the nozzle can take the load. A
//! 6 mm tall pillar standing on a 4 mm² footprint tips over when a
//! nozzle drags across it; a 1 mm fin snaps. This module supplies the
//! missing half.
//!
//! # Islands
//!
//! An **island** is a connected component of deposited material within
//! one layer. Segments are linked when any sampled point of one lies
//! within [`ContactConfig::island_link_tolerance`] of a sampled point of
//! the other; components are accumulated with a disjoint-set union over
//! segment indices, and the candidate pairs come from a uniform spatial
//! hash whose cell equals the link tolerance (so every point within
//! tolerance is in the queried 3 × 3 cell neighborhood).
//!
//! Complexity: with `S` sample points per layer the build is
//! `O(S · k)`, where `k` is the number of samples in a 3 × 3
//! neighborhood, capped at [`NEIGHBOR_SCAN_CAP`] per sample — so linear
//! in `S` with a bounded constant, never the `O(n²)` of all-pairs.
//! Sampling itself is bounded: spacing starts at half the link
//! tolerance and is stretched so a layer never produces more than
//! [`MAX_SAMPLES_PER_LAYER`] points, and no more than
//! [`MAX_SEGMENTS_PER_LAYER`] segments are considered.
//!
//! **Every cap fails safe in the same direction**: coarser sampling,
//! dropped segments and a truncated neighbor scan can only *miss* links,
//! which splits one island into several smaller ones. Smaller islands
//! mean smaller footprints, which mean more refusals — never fewer.
//!
//! Connectivity is by point proximity, not by intersection: two beads
//! that cross in mid-span without a sample landing within tolerance are
//! not linked. At the default 0.6 mm tolerance and 0.3 mm sampling this
//! effectively cannot happen for real extrusion widths, and when it does
//! it under-connects — again the safe direction.
//!
//! # Area
//!
//! Island area is a **strict lower bound** computed by [`crate::raster`]:
//! the union of per-segment capsules discretized onto a grid, counting
//! only cells that provably lie entirely inside the union. See that
//! module for the error analysis. Sparse infill therefore contributes
//! only the plastic actually deposited — not the outline it encloses —
//! which is exactly right for a bed-adhesion question.
//!
//! # Bed-adhesion footprint
//!
//! A candidate on layer N−1 is only as well anchored as the chain of
//! material that carries it to the bed. [`FootprintTrace`] walks the
//! layer stack downward, mapping each island to the islands of the layer
//! below whose material lies within the link tolerance, and accumulates
//! the union until layer 0. What it reports:
//!
//! * `bed_area` — total area of the layer-0 islands reached.
//! * `weakest_link_area` — the smallest inter-layer connection area seen
//!   on the way down. A wide top plate on a narrow neck is held by the
//!   neck, not by the base, and this is the term that says so.
//! * `effective_area` — `min(bed_area, weakest_link_area)`, the number
//!   the adhesion and tipping criteria actually use.
//!
//! Honest limits, all resolved conservatively:
//!
//! * **Bridges and overhangs.** A bridge island is anchored at its ends,
//!   so the trace follows it correctly, but the *strength* of a sliver
//!   of overlap is not modeled — a 0.5 mm² touch traces the same as full
//!   support. `weakest_link_area` is the mitigation: that sliver becomes
//!   the effective area and normally fails adhesion outright.
//! * **Unsupported islands.** When no island of the layer below overlaps,
//!   the trace stops with [`TraceStatus::Broken`] and the effective area
//!   is zero, so every load-bearing criterion fails.
//! * **Mid-file windows.** [`crate::model::build_layer_model`] can model
//!   a byte window that does not start at the bed, in which case model
//!   layer 0 is not the first layer and its area says nothing about
//!   adhesion. The analysis detects this heuristically — layer 0 sitting
//!   above [`BED_LAYER_Z_MAX`] — and reports
//!   [`TraceStatus::BedLayerMissing`], again with zero effective area.
//!   A print whose true first layer is above 1 mm in the internal frame
//!   (a large Z offset, say) is refused rather than guessed at;
//!   [`ContactConfig::structural_checks_enabled`] is the documented
//!   escape hatch.
//! * **Support material** counts as island material like any other
//!   deposition. Excluding it would break the trace for every supported
//!   part and produce false refusals; including it is the choice that
//!   keeps the analysis honest about what is physically under the
//!   nozzle.

// Justification for the module-wide casting allows: the values converted
// here are sample/segment counts bounded by MAX_SAMPLES_PER_LAYER (4e5)
// and grid coordinates bounded by the raster budget, all far inside
// f64's exactly representable integer range; the reverse conversions are
// range-checked before the cast. Per-site allows would be noise.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::contact::{ContactConfig, ContactError};
use crate::geom;
use crate::model::{Layer, LayerModel, XySegment};
use crate::raster::Raster;

/// Deposition Z (internal frame) above which model layer 0 cannot
/// plausibly be the bed layer, mm. First layers are 0.1–0.4 mm; 1.0 mm
/// leaves generous room for Z offsets before the analysis refuses to
/// treat layer 0 as the footprint.
pub const BED_LAYER_Z_MAX: f64 = 1.0;

/// Maximum extrusion segments considered per layer. Excess segments are
/// dropped, which under-connects and under-measures — the safe way to
/// fail on pathological input.
pub const MAX_SEGMENTS_PER_LAYER: usize = 200_000;

/// Maximum sample points generated per layer. The sample spacing is
/// stretched to respect this, which can only miss links.
pub const MAX_SAMPLES_PER_LAYER: usize = 400_000;

/// Maximum sample points per single segment.
const MAX_SAMPLES_PER_SEGMENT: usize = 4096;

/// Maximum neighbor samples examined per sample when linking. Hitting
/// the cap can only miss links.
pub const NEIGHBOR_SCAN_CAP: usize = 64;

/// Number of evenly spaced directions probed by
/// [`StructuralAnalysis::largest_clear_run`].
pub const RUN_DIRECTIONS: usize = 16;

/// Maximum march steps taken when measuring one clear run.
const MAX_RUN_STEPS: usize = 8192;

/// Footprint area substituted when the true area is zero, mm², so the
/// tipping ratio stays finite (and therefore serializable) instead of
/// becoming an infinity.
const AREA_FLOOR: f64 = 1e-6;

/// Spatial-hash coordinates are clamped to this magnitude before any
/// cast: the neighbourhood scan adds +/-1 to them.
const CELL_INDEX_LIMIT: f64 = (i64::MAX / 4) as f64;

/// An axis-aligned bounding box in XY, mm.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    /// Lower corner.
    pub min: [f64; 2],
    /// Upper corner.
    pub max: [f64; 2],
}

impl BoundingBox {
    /// Extent along X.
    #[must_use]
    pub fn width(&self) -> f64 {
        self.max[0] - self.min[0]
    }

    /// Extent along Y.
    #[must_use]
    pub fn height(&self) -> f64 {
        self.max[1] - self.min[1]
    }

    /// Smaller of the two extents — the slenderness measure used by the
    /// feature-width criterion.
    ///
    /// Axis-aligned, so this **over-states** the true width of a feature
    /// that runs diagonally: a 2 mm fin from (0,0) to (20,20) reports
    /// 20 mm, not 2 mm. It is the one place the structural analysis errs
    /// optimistic; see [`StructuralCriterion::FeatureWidth`].
    #[must_use]
    pub fn min_dimension(&self) -> f64 {
        self.width().min(self.height())
    }

    /// Diagonal length.
    #[must_use]
    pub fn diagonal(&self) -> f64 {
        let (w, h) = (self.width(), self.height());
        (w * w + h * h).sqrt()
    }

    /// True when `point` lies in the box grown by `slack` on all sides.
    fn contains(&self, point: [f64; 2], slack: f64) -> bool {
        point[0] >= self.min[0] - slack
            && point[0] <= self.max[0] + slack
            && point[1] >= self.min[1] - slack
            && point[1] <= self.max[1] + slack
    }
}

/// One connected component of deposited material within a layer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Island {
    /// Layer the island belongs to.
    pub layer: u32,
    /// Index of the island within its layer's island list.
    pub index: usize,
    /// Bounding box of the island's extrusion centerlines.
    pub bbox: BoundingBox,
    /// Strict **lower bound** on the deposited material area, mm² (see
    /// the module docs and [`crate::raster`]).
    pub area: f64,
    /// Path-length-weighted centroid of the island's centerlines.
    pub centroid: [f64; 2],
    /// Number of extrusion segments in the island.
    pub segment_count: usize,
    /// Total XY length of the island's extrusion centerlines, mm.
    pub path_length: f64,
    /// The area estimate was computed on a coarsened or truncated grid
    /// (very large or very dense island), so it under-reports by more
    /// than the usual margin.
    pub estimate_degraded: bool,
}

/// How the nozzle intends to touch the part.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ContactMode {
    /// Straight-down tap: force is normal to the layer, resisted by the
    /// whole footprint. Gentle, but still able to snap a slender feature.
    Tap,
    /// Lateral drag: the nozzle travels while in contact. The load is a
    /// shear/moment about the bed contact, which is what tips pillars and
    /// peels edges.
    Drag {
        /// Drag direction in XY (need not be normalized; must be
        /// non-zero and finite).
        direction: [f64; 2],
        /// Length of the lateral run, mm. Any lead-in or lead-out the
        /// executor adds must be included by the caller.
        run_length: f64,
    },
}

impl Default for ContactMode {
    /// [`ContactMode::Tap`] — the gentler of the two.
    fn default() -> Self {
        Self::Tap
    }
}

/// The structural criteria, in decreasing severity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum StructuralCriterion {
    /// Layer-0 footprint area holding the feature to the bed.
    BedAdhesion,
    /// Height-to-footprint aspect ratio (tipping moment).
    Tipping,
    /// Narrowest bounding-box dimension of the island (slenderness).
    ///
    /// Measured on the axis-aligned bounding box, which **over-states**
    /// the width of a diagonal fin — the only non-conservative estimate
    /// in this module. See [`ContactConfig::min_feature_width`].
    FeatureWidth,
    /// Distance from the contact point to the island's boundary.
    EdgeMargin,
    /// Clear lateral run available for a drag.
    DragRun,
}

impl StructuralCriterion {
    /// Short stable identifier, for UIs and logs.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::BedAdhesion => "bed_adhesion",
            Self::Tipping => "tipping",
            Self::FeatureWidth => "feature_width",
            Self::EdgeMargin => "edge_margin",
            Self::DragRun => "drag_run",
        }
    }
}

/// Unit of a [`CriterionCheck`]'s measured and threshold values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CriterionUnit {
    /// Millimetres.
    Millimetres,
    /// Square millimetres.
    SquareMillimetres,
    /// Dimensionless ratio.
    Ratio,
}

impl CriterionUnit {
    /// Short symbol, for rendering.
    #[must_use]
    pub fn symbol(self) -> &'static str {
        match self {
            Self::Millimetres => "mm",
            Self::SquareMillimetres => "mm^2",
            Self::Ratio => "",
        }
    }
}

/// One criterion's outcome: what was measured, what was required, and
/// why it matters. Never a bare boolean.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CriterionCheck {
    /// Which criterion this is.
    pub criterion: StructuralCriterion,
    /// Whether the measured value satisfies the threshold.
    pub passed: bool,
    /// The measured value, in the criterion's own unit.
    pub measured: f64,
    /// The configured threshold, same unit.
    pub threshold: f64,
    /// Unit of `measured`/`threshold`, for rendering.
    pub unit: CriterionUnit,
    /// Human-readable explanation, suitable for an operator.
    pub reason: String,
}

/// Whether a point/mode combination cleared every hard criterion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StructuralOutcome {
    /// Every hard criterion passed.
    Safe,
    /// At least one criterion failed; `primary` is the most severe.
    Unsafe {
        /// The most severe failing criterion.
        primary: StructuralCriterion,
    },
}

/// How far down the layer stack an island could be traced.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum TraceStatus {
    /// The analysed layer *is* the bed layer; the island is its own
    /// footprint.
    OnBed,
    /// Traced through overlapping material at every layer down to 0.
    Anchored,
    /// No island of the layer below overlapped: unsupported in the model.
    Broken {
        /// The layer whose support could not be found.
        layer: u32,
    },
    /// Model layer 0 is too high to be the bed layer, so the model window
    /// does not start at the bed and the footprint is unknowable.
    BedLayerMissing {
        /// Deposition Z of model layer 0, mm (internal frame).
        layer_zero_z: f64,
    },
}

impl TraceStatus {
    /// True when the trace produced a usable footprint.
    fn resolved(self) -> bool {
        matches!(self, Self::OnBed | Self::Anchored)
    }
}

/// The bed-adhesion footprint of one island of the analysed layer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FootprintTrace {
    /// Index of the analysed-layer island this trace starts from.
    pub island: usize,
    /// Outcome of walking the stack down.
    pub status: TraceStatus,
    /// Total material area of the bed-layer islands reached, mm².
    pub bed_area: f64,
    /// Indices (into [`StructuralAnalysis::bed_islands`]) of the reached
    /// bed-layer islands.
    pub bed_island_indices: Vec<usize>,
    /// Smallest inter-layer connection area seen on the way down, mm².
    pub weakest_link_area: f64,
    /// `min(bed_area, weakest_link_area)`, or 0 when unresolved — the
    /// area the adhesion and tipping criteria use.
    pub effective_area: f64,
    /// Lowest model layer the trace reached.
    pub reached_layer: u32,
}

/// A clear lateral run: where it starts, which way it goes, how long it
/// is. Every point of the run is on one island at or above the required
/// edge margin.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ClearRun {
    /// Start XY of the run (internal frame).
    pub start: [f64; 2],
    /// Unit direction of the run.
    pub direction: [f64; 2],
    /// Clear length available from `start` along `direction`, mm.
    pub length: f64,
    /// Island the run lies on.
    pub island: usize,
    /// Edge margin the run was measured at, mm.
    pub margin: f64,
}

/// The full structural verdict for one point and contact mode.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StructuralVerdict {
    /// The evaluated point.
    pub point: [f64; 2],
    /// The evaluated contact mode.
    pub mode: ContactMode,
    /// Layer the point was evaluated on.
    pub layer: u32,
    /// The island the point sits on.
    pub island: Island,
    /// Bed-adhesion trace of that island.
    pub footprint: FootprintTrace,
    /// Every criterion evaluated, in decreasing severity.
    pub checks: Vec<CriterionCheck>,
    /// Structural desirability in `[0, 1]`, higher is better. A ranking
    /// aid, not a gate: a verdict can score well and still be unsafe.
    pub score: f64,
    /// For [`ContactMode::Drag`]: cosine of the angle between the drag
    /// direction and the direction to the island centroid. `1` pushes
    /// straight into the mass, `-1` levers off the far edge. Exposed as
    /// a score input, never as a gate.
    pub centroid_alignment: Option<f64>,
    /// One-line human-readable summary.
    pub summary: String,
}

impl StructuralVerdict {
    /// Whether every hard criterion passed, and if not, which failed
    /// most severely.
    #[must_use]
    pub fn outcome(&self) -> StructuralOutcome {
        self.checks
            .iter()
            .find(|c| !c.passed)
            .map_or(StructuralOutcome::Safe, |c| StructuralOutcome::Unsafe {
                primary: c.criterion,
            })
    }

    /// The failing checks, in decreasing severity.
    #[must_use]
    pub fn failures(&self) -> Vec<&CriterionCheck> {
        self.checks.iter().filter(|c| !c.passed).collect()
    }

    /// Look one criterion up.
    #[must_use]
    pub fn check(&self, criterion: StructuralCriterion) -> Option<&CriterionCheck> {
        self.checks.iter().find(|c| c.criterion == criterion)
    }
}

/// Result of assessing an arbitrary XY — including one an operator
/// jogged to by hand.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum StructuralAssessment {
    /// The point is on island material; the verdict follows.
    Evaluated(Box<StructuralVerdict>),
    /// No island of the layer covers the point.
    OffMaterial {
        /// Distance to the nearest sampled material point, mm.
        distance: f64,
        /// That nearest material point — where to jog to.
        nearest_point: Option<[f64; 2]>,
        /// Island the nearest point belongs to.
        nearest_island: Option<usize>,
    },
    /// The requested point or mode was not finite/usable.
    InvalidPoint {
        /// Which input was rejected.
        param: InvalidInput,
    },
}

/// Which input [`StructuralAnalysis::assess`] refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum InvalidInput {
    /// The XY was not finite.
    Point,
    /// The drag direction was zero or non-finite.
    DragDirection,
    /// The drag run length was not a positive finite number.
    DragRunLength,
}

/// Disjoint-set union over segment indices (path halving).
struct Dsu {
    parent: Vec<usize>,
}

impl Dsu {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[rb] = ra;
        }
    }
}

/// One layer's deposition, sampled and partitioned into islands.
struct LayerShape<'m> {
    segments: Vec<&'m XySegment>,
    members: Vec<Vec<usize>>,
    samples: Vec<[f64; 2]>,
    sample_island: Vec<usize>,
    grid: HashMap<(i64, i64), Vec<u32>>,
    cell: f64,
    spacing: f64,
    bboxes: Vec<BoundingBox>,
    lengths: Vec<f64>,
    centroids: Vec<[f64; 2]>,
}

/// Grid coordinates of `p` at cell size `cell`.
///
/// Clamped well inside `i64`'s range: callers add a +/-1 neighbourhood
/// offset, and a coordinate large enough to overflow that is
/// astronomically far outside any print volume anyway.
fn cell_coords(p: [f64; 2], cell: f64) -> (i64, i64) {
    let fi = (p[0] / cell)
        .floor()
        .clamp(-CELL_INDEX_LIMIT, CELL_INDEX_LIMIT);
    let fj = (p[1] / cell)
        .floor()
        .clamp(-CELL_INDEX_LIMIT, CELL_INDEX_LIMIT);
    (fi as i64, fj as i64)
}

/// True when both endpoints and Z of a segment are finite.
fn segment_finite(seg: &XySegment) -> bool {
    seg.start
        .iter()
        .chain(seg.end.iter())
        .all(|v| v.is_finite())
}

impl<'m> LayerShape<'m> {
    /// Sample `layer`'s deposition and union it into islands.
    fn build(layer: &'m Layer, tolerance: f64) -> Self {
        let segments: Vec<&XySegment> = layer
            .paths
            .iter()
            .flat_map(|p| p.segments.iter())
            .filter(|s| segment_finite(s))
            .take(MAX_SEGMENTS_PER_LAYER)
            .collect();
        let total: f64 = segments.iter().map(|s| s.length()).sum();
        let mut spacing = tolerance / 2.0;
        if total.is_finite() && total > 0.0 {
            spacing = spacing.max(total / MAX_SAMPLES_PER_LAYER as f64);
        }
        let mut shape = Self {
            segments,
            members: Vec::new(),
            samples: Vec::new(),
            sample_island: Vec::new(),
            grid: HashMap::new(),
            cell: tolerance,
            spacing,
            bboxes: Vec::new(),
            lengths: Vec::new(),
            centroids: Vec::new(),
        };
        let mut dsu = Dsu::new(shape.segments.len());
        shape.sample_and_link(&mut dsu, tolerance);
        shape.collect_components(&mut dsu);
        shape
    }

    /// Emit sample points, hash them, and union segments that touch.
    fn sample_and_link(&mut self, dsu: &mut Dsu, tolerance: f64) {
        let mut owner: Vec<usize> = Vec::new();
        for (index, seg) in self.segments.iter().enumerate() {
            let steps =
                ((seg.length() / self.spacing).ceil() as usize).clamp(1, MAX_SAMPLES_PER_SEGMENT);
            for k in 0..=steps {
                let p = seg.point_at(k as f64 / steps as f64);
                if !p[0].is_finite() || !p[1].is_finite() {
                    continue;
                }
                let (ci, cj) = cell_coords(p, self.cell);
                let mut scanned = 0_usize;
                for dj in -1..=1_i64 {
                    for di in -1..=1_i64 {
                        let Some(bucket) = self.grid.get(&(ci + di, cj + dj)) else {
                            continue;
                        };
                        for &other in bucket {
                            if scanned >= NEIGHBOR_SCAN_CAP {
                                break;
                            }
                            scanned += 1;
                            let other = other as usize;
                            if geom::point_distance(p, self.samples[other]) <= tolerance {
                                dsu.union(index, owner[other]);
                            }
                        }
                    }
                }
                let id = u32::try_from(self.samples.len()).unwrap_or(u32::MAX);
                if id == u32::MAX {
                    continue;
                }
                self.samples.push(p);
                owner.push(index);
                self.grid.entry((ci, cj)).or_default().push(id);
            }
        }
        self.sample_island = owner;
    }

    /// Turn the union-find state into island member lists and per-island
    /// geometry, rewriting `sample_island` from segment to island ids.
    fn collect_components(&mut self, dsu: &mut Dsu) {
        let mut root_to_island: HashMap<usize, usize> = HashMap::new();
        let mut island_of_segment = vec![0_usize; self.segments.len()];
        for (index, owner) in island_of_segment.iter_mut().enumerate() {
            let root = dsu.find(index);
            let next = root_to_island.len();
            let island = *root_to_island.entry(root).or_insert(next);
            *owner = island;
            if self.members.len() <= island {
                self.members.resize(island + 1, Vec::new());
            }
            self.members[island].push(index);
        }
        for owner in &mut self.sample_island {
            *owner = island_of_segment[*owner];
        }
        for members in &self.members {
            let mut bbox = BoundingBox {
                min: [f64::INFINITY; 2],
                max: [f64::NEG_INFINITY; 2],
            };
            let (mut length, mut acc) = (0.0_f64, [0.0_f64; 2]);
            for &index in members {
                let seg = self.segments[index];
                for p in [seg.start, seg.end] {
                    bbox.min[0] = bbox.min[0].min(p[0]);
                    bbox.min[1] = bbox.min[1].min(p[1]);
                    bbox.max[0] = bbox.max[0].max(p[0]);
                    bbox.max[1] = bbox.max[1].max(p[1]);
                }
                let len = seg.length();
                let mid = seg.point_at(0.5);
                length += len;
                acc[0] += mid[0] * len;
                acc[1] += mid[1] * len;
            }
            let centroid = if length > 0.0 {
                [acc[0] / length, acc[1] / length]
            } else {
                // Zero-length island (a retract-in-place artifact): the
                // bounding-box center is the only meaningful centroid.
                [
                    (bbox.min[0] + bbox.max[0]) * 0.5,
                    (bbox.min[1] + bbox.max[1]) * 0.5,
                ]
            };
            self.bboxes.push(bbox);
            self.lengths.push(length);
            self.centroids.push(centroid);
        }
    }

    fn island_count(&self) -> usize {
        self.members.len()
    }

    /// Islands whose sampled material lies within `tolerance` of `p`.
    fn hits(&self, p: [f64; 2], tolerance: f64, out: &mut Vec<usize>) {
        out.clear();
        if !p[0].is_finite() || !p[1].is_finite() {
            return;
        }
        let (ci, cj) = cell_coords(p, self.cell);
        for dj in -1..=1_i64 {
            for di in -1..=1_i64 {
                let Some(bucket) = self.grid.get(&(ci + di, cj + dj)) else {
                    continue;
                };
                for &sample in bucket.iter().take(NEIGHBOR_SCAN_CAP) {
                    let sample = sample as usize;
                    if geom::point_distance(p, self.samples[sample]) <= tolerance {
                        let island = self.sample_island[sample];
                        if !out.contains(&island) {
                            out.push(island);
                        }
                    }
                }
            }
        }
    }
}

/// Build [`Island`] descriptors and rasters for every island of `shape`.
fn materialize(shape: &LayerShape, layer: u32, half_width: f64) -> (Vec<Island>, Vec<Raster>) {
    let mut islands = Vec::with_capacity(shape.island_count());
    let mut rasters = Vec::with_capacity(shape.island_count());
    for (index, members) in shape.members.iter().enumerate() {
        let segments: Vec<&XySegment> = members.iter().map(|&s| shape.segments[s]).collect();
        let bbox = shape.bboxes[index];
        let raster = Raster::build(&segments, bbox.min, bbox.max, half_width);
        islands.push(Island {
            layer,
            index,
            bbox,
            area: raster.area(),
            centroid: shape.centroids[index],
            segment_count: members.len(),
            path_length: shape.lengths[index],
            estimate_degraded: raster.coarsened() || raster.truncated(),
        });
        rasters.push(raster);
    }
    (islands, rasters)
}

/// Correspond one layer pair and fold the result into the running
/// footprint groups.
fn step_down(
    upper: &LayerShape,
    lower: &LayerShape,
    lower_layer: u32,
    config: &ContactConfig,
    state: (&mut [Vec<usize>], &mut [f64], &mut [Option<u32>]),
) {
    let (groups, weakest, broken) = state;
    let tolerance = config.island_link_tolerance;
    let mut corr: Vec<Vec<usize>> = vec![Vec::new(); upper.island_count()];
    let mut linked = vec![0_usize; upper.island_count()];
    let mut hits: Vec<usize> = Vec::new();
    for (index, point) in upper.samples.iter().enumerate() {
        let owner = upper.sample_island[index];
        lower.hits(*point, tolerance, &mut hits);
        if hits.is_empty() {
            continue;
        }
        linked[owner] += 1;
        for &island in &hits {
            if !corr[owner].contains(&island) {
                corr[owner].push(island);
            }
        }
    }
    for (group_index, group) in groups.iter_mut().enumerate() {
        if broken[group_index].is_some() {
            continue;
        }
        let mut count = 0_usize;
        let mut next: Vec<usize> = Vec::new();
        for &owner in group.iter() {
            count += linked.get(owner).copied().unwrap_or(0);
            for &island in corr.get(owner).into_iter().flatten() {
                if !next.contains(&island) {
                    next.push(island);
                }
            }
        }
        // Each linked sample stands for `spacing` mm of bead at the
        // configured width: a crude but honest cross-section estimate.
        let area = count as f64 * upper.spacing * config.extrusion_width;
        weakest[group_index] = weakest[group_index].min(area);
        if next.is_empty() {
            broken[group_index] = Some(lower_layer + 1);
        } else {
            next.sort_unstable();
            *group = next;
        }
    }
}

/// Trace every island of `top` down to the bed layer.
fn trace_stack<'m>(
    model: &'m LayerModel,
    top_layer: u32,
    top: LayerShape<'m>,
    config: &ContactConfig,
) -> (Vec<FootprintTrace>, Vec<Island>) {
    let half_width = config.extrusion_width / 2.0;
    let count = top.island_count();
    if top_layer == 0 {
        let (bed, _) = materialize(&top, 0, half_width);
        let traces = bed
            .iter()
            .map(|island| FootprintTrace {
                island: island.index,
                status: TraceStatus::OnBed,
                bed_area: island.area,
                bed_island_indices: vec![island.index],
                weakest_link_area: island.area,
                effective_area: island.area,
                reached_layer: 0,
            })
            .collect();
        return (traces, bed);
    }
    let zero_z = model.layers.first().map_or(f64::MAX, |l| l.z);
    // NaN must be treated as "not a bed layer", so this is deliberately
    // the negation of a `<=` rather than a `>`.
    if !matches!(
        zero_z.partial_cmp(&BED_LAYER_Z_MAX),
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
    ) {
        let status = TraceStatus::BedLayerMissing {
            layer_zero_z: finite_or(zero_z, f64::MAX),
        };
        return (unresolved_traces(count, status, top_layer), Vec::new());
    }
    let mut groups: Vec<Vec<usize>> = (0..count).map(|i| vec![i]).collect();
    let mut weakest = vec![f64::INFINITY; count];
    let mut broken: Vec<Option<u32>> = vec![None; count];
    let mut upper = top;
    let mut layer = top_layer;
    while layer > 0 {
        layer -= 1;
        let Some(next) = model.layer(layer) else {
            // Unreachable for a well-formed model: `LayerModel` layers
            // are contiguous 0..len and `top_layer` was resolved from
            // it. Breaking out of the loop here would leave `upper`
            // pointing at some layer above 0 and then label it the bed
            // layer — reporting an optimistic footprint for a feature
            // whose real footprint was never seen, precisely the failure
            // the design exists to exclude. Refuse instead.
            return (
                unresolved_traces(
                    count,
                    TraceStatus::BedLayerMissing {
                        layer_zero_z: finite_or(zero_z, f64::MAX),
                    },
                    top_layer,
                ),
                Vec::new(),
            );
        };
        let lower = LayerShape::build(next, config.island_link_tolerance);
        step_down(
            &upper,
            &lower,
            layer,
            config,
            (&mut groups, &mut weakest, &mut broken),
        );
        upper = lower;
    }
    let (bed, _) = materialize(&upper, 0, half_width);
    let traces = (0..count)
        .map(|island| assemble_trace(island, &groups, &weakest, &broken, &bed))
        .collect();
    (traces, bed)
}

/// Zero-footprint traces for every island of the analysed layer, used
/// whenever the bed layer could not be established at all. Every
/// load-bearing criterion fails on these.
fn unresolved_traces(count: usize, status: TraceStatus, reached_layer: u32) -> Vec<FootprintTrace> {
    (0..count)
        .map(|island| FootprintTrace {
            island,
            status,
            bed_area: 0.0,
            bed_island_indices: Vec::new(),
            weakest_link_area: 0.0,
            effective_area: 0.0,
            reached_layer,
        })
        .collect()
}

/// Turn the accumulated trace state for one island into a
/// [`FootprintTrace`].
fn assemble_trace(
    island: usize,
    groups: &[Vec<usize>],
    weakest: &[f64],
    broken: &[Option<u32>],
    bed: &[Island],
) -> FootprintTrace {
    let status =
        broken[island].map_or(TraceStatus::Anchored, |layer| TraceStatus::Broken { layer });
    let (bed_area, indices) = if status.resolved() {
        let indices: Vec<usize> = groups[island]
            .iter()
            .copied()
            .filter(|i| *i < bed.len())
            .collect();
        (indices.iter().map(|&i| bed[i].area).sum::<f64>(), indices)
    } else {
        (0.0, Vec::new())
    };
    let link = finite_or(weakest[island], bed_area);
    let effective = if status.resolved() {
        bed_area.min(link)
    } else {
        0.0
    };
    FootprintTrace {
        island,
        status,
        bed_area,
        bed_island_indices: indices,
        weakest_link_area: link,
        effective_area: effective,
        reached_layer: match status {
            TraceStatus::Broken { layer } => layer,
            _ => 0,
        },
    }
}

/// Normalize `direction`, or `None` when it is zero or non-finite.
fn unit(direction: [f64; 2]) -> Option<[f64; 2]> {
    let length = (direction[0] * direction[0] + direction[1] * direction[1]).sqrt();
    if !length.is_finite() || length <= 0.0 {
        return None;
    }
    Some([direction[0] / length, direction[1] / length])
}

/// Clamp to `[0, 1]`, mapping NaN to the worst score.
fn clamp01(value: f64) -> f64 {
    if value.is_nan() {
        0.0
    } else {
        value.clamp(0.0, 1.0)
    }
}

/// `value` when finite, `fallback` otherwise.
fn finite_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        fallback
    }
}

/// Cosine of the angle between a drag direction and the direction from
/// the contact point to the island centroid.
fn alignment_of(point: [f64; 2], direction: [f64; 2], centroid: [f64; 2]) -> f64 {
    let to_centroid = [centroid[0] - point[0], centroid[1] - point[1]];
    unit(to_centroid).map_or(0.0, |u| direction[0] * u[0] + direction[1] * u[1])
}

/// Islands, footprints and verdicts for one layer, built once and
/// queried many times.
///
/// A guided-jog UI builds this once for layer N-1 and calls
/// [`StructuralAnalysis::assess`] for every XY the operator jogs to;
/// rebuilding per point would re-walk the whole layer stack.
pub struct StructuralAnalysis {
    layer: u32,
    layer_z: f64,
    islands: Vec<Island>,
    rasters: Vec<Raster>,
    bed_islands: Vec<Island>,
    traces: Vec<FootprintTrace>,
    samples: Vec<[f64; 2]>,
    sample_island: Vec<usize>,
    config: ContactConfig,
}

impl std::fmt::Debug for StructuralAnalysis {
    /// The rasters and sample cloud are megabytes of grid; only the
    /// summary is printed.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StructuralAnalysis")
            .field("layer", &self.layer)
            .field("layer_z", &self.layer_z)
            .field("islands", &self.islands.len())
            .field("bed_islands", &self.bed_islands.len())
            .finish_non_exhaustive()
    }
}

impl StructuralAnalysis {
    /// Analyse `layer` of `model`: its islands, and the bed-adhesion
    /// footprint of each.
    ///
    /// Cost is one pass over every layer from `layer` down to 0 (the
    /// footprint trace) plus one raster per island of `layer` and of
    /// layer 0.
    pub fn build(
        model: &LayerModel,
        layer: u32,
        config: &ContactConfig,
    ) -> Result<Self, ContactError> {
        crate::contact::validate_config(config)?;
        let Some(top) = model.layer(layer) else {
            return Err(ContactError::LayerOutOfRange {
                resume_layer: layer,
                layers: model.layers.len(),
            });
        };
        let half_width = config.extrusion_width / 2.0;
        let shape = LayerShape::build(top, config.island_link_tolerance);
        let (islands, rasters) = materialize(&shape, layer, half_width);
        let samples = shape.samples.clone();
        let sample_island = shape.sample_island.clone();
        let (traces, bed_islands) = trace_stack(model, layer, shape, config);
        Ok(Self {
            layer,
            layer_z: top.z,
            islands,
            rasters,
            bed_islands,
            traces,
            samples,
            sample_island,
            config: config.clone(),
        })
    }

    /// The analysed layer index.
    #[must_use]
    pub fn layer(&self) -> u32 {
        self.layer
    }

    /// Deposition Z of the analysed layer, mm (internal frame).
    #[must_use]
    pub fn layer_z(&self) -> f64 {
        self.layer_z
    }

    /// Islands of the analysed layer.
    #[must_use]
    pub fn islands(&self) -> &[Island] {
        &self.islands
    }

    /// Islands of the bed layer (model layer 0); empty when the model
    /// window does not start at the bed.
    #[must_use]
    pub fn bed_islands(&self) -> &[Island] {
        &self.bed_islands
    }

    /// Footprint trace of island `index` of the analysed layer.
    #[must_use]
    pub fn footprint(&self, index: usize) -> Option<&FootprintTrace> {
        self.traces.get(index)
    }

    /// Index of the island whose material covers `point`.
    #[must_use]
    pub fn island_at(&self, point: [f64; 2]) -> Option<usize> {
        if !point[0].is_finite() || !point[1].is_finite() {
            return None;
        }
        let slack = self.config.extrusion_width;
        self.islands.iter().position(|island| {
            island.bbox.contains(point, slack) && self.rasters[island.index].is_material(point)
        })
    }

    /// Nearest sampled material point to `point`, with its island.
    fn nearest(&self, point: [f64; 2]) -> Option<(f64, [f64; 2], usize)> {
        let mut best: Option<(f64, [f64; 2], usize)> = None;
        for (index, sample) in self.samples.iter().enumerate() {
            let distance = geom::point_distance(point, *sample);
            if !distance.is_finite() {
                continue;
            }
            if best.is_none_or(|(current, _, _)| distance < current) {
                best = Some((distance, *sample, self.sample_island[index]));
            }
        }
        best
    }

    /// Structural verdict for an arbitrary XY on the analysed layer --
    /// the entry point a manual-jog UI validates a hand-picked point
    /// with.
    #[must_use]
    pub fn assess(&self, point: [f64; 2], mode: &ContactMode) -> StructuralAssessment {
        if !point[0].is_finite() || !point[1].is_finite() {
            return StructuralAssessment::InvalidPoint {
                param: InvalidInput::Point,
            };
        }
        if let ContactMode::Drag {
            direction,
            run_length,
        } = mode
        {
            if unit(*direction).is_none() {
                return StructuralAssessment::InvalidPoint {
                    param: InvalidInput::DragDirection,
                };
            }
            if !run_length.is_finite() || *run_length <= 0.0 {
                return StructuralAssessment::InvalidPoint {
                    param: InvalidInput::DragRunLength,
                };
            }
        }
        let Some(index) = self.island_at(point) else {
            let nearest = self.nearest(point);
            return StructuralAssessment::OffMaterial {
                distance: nearest.map_or(f64::MAX, |(d, _, _)| d),
                nearest_point: nearest.map(|(_, p, _)| p),
                nearest_island: nearest.map(|(_, _, i)| i),
            };
        };
        StructuralAssessment::Evaluated(Box::new(self.verdict(index, point, mode)))
    }

    /// Clear lateral run from `from` along `direction` on `island`, up to
    /// `limit` mm: the distance over which every point stays on that
    /// island at or above [`ContactConfig::min_edge_margin`].
    ///
    /// Requiring that margin as a *radius* at every point of the run is
    /// what covers the pass geometry: a clear disc of radius `margin`
    /// around each point means a swath `2 * margin` wide is material.
    ///
    /// The march step is the raster cell (0.075 mm by default), so the
    /// result is quantized downward -- conservative.
    #[must_use]
    pub fn clear_run(&self, island: usize, from: [f64; 2], direction: [f64; 2], limit: f64) -> f64 {
        let Some(raster) = self.rasters.get(island) else {
            return 0.0;
        };
        let Some(direction) = unit(direction) else {
            return 0.0;
        };
        if !limit.is_finite() || limit <= 0.0 || !from[0].is_finite() || !from[1].is_finite() {
            return 0.0;
        }
        let margin = self.config.min_edge_margin;
        let step = raster.cell().max(0.05);
        let steps = ((limit / step).ceil() as usize).min(MAX_RUN_STEPS);
        let mut reached = 0.0;
        for k in 0..=steps {
            let distance = (k as f64 * step).min(limit);
            let point = [
                from[0] + direction[0] * distance,
                from[1] + direction[1] * distance,
            ];
            if !raster.is_material(point) || raster.clearance_at(point) < margin {
                return reached;
            }
            reached = distance;
            if distance >= limit {
                break;
            }
        }
        reached
    }

    /// Longest clear run from `from` on `island`, over
    /// [`RUN_DIRECTIONS`] evenly spaced directions.
    #[must_use]
    pub fn largest_clear_run(&self, island: usize, from: [f64; 2]) -> Option<ClearRun> {
        let bbox = self.islands.get(island)?.bbox;
        let limit = finite_or(bbox.diagonal(), 1.0).max(1.0);
        let mut best: Option<ClearRun> = None;
        for step in 0..RUN_DIRECTIONS {
            let angle = std::f64::consts::TAU * step as f64 / RUN_DIRECTIONS as f64;
            let direction = [angle.cos(), angle.sin()];
            let length = self.clear_run(island, from, direction, limit);
            if best.as_ref().is_none_or(|current| length > current.length) {
                best = Some(ClearRun {
                    start: from,
                    direction,
                    length,
                    island,
                    margin: self.config.min_edge_margin,
                });
            }
        }
        best
    }

    /// Evaluate every criterion for a point known to be on `index`.
    fn verdict(&self, index: usize, point: [f64; 2], mode: &ContactMode) -> StructuralVerdict {
        let island = self.islands[index].clone();
        let footprint = self.traces[index].clone();
        let config = &self.config;
        let area = footprint.effective_area;
        let min_dimension = island.bbox.min_dimension();
        let clearance = self.rasters[index].clearance_at(point);
        let height = finite_or(self.layer_z, f64::MAX).max(0.0);
        let aspect = finite_or(height / area.max(AREA_FLOOR).sqrt(), f64::MAX);
        let mut checks = vec![
            adhesion_check(area, config.min_bed_contact_area, footprint.status),
            tipping_check(aspect, config.max_aspect_ratio, height, area),
            width_check(min_dimension, config.min_feature_width),
            margin_check(clearance, config.min_edge_margin),
        ];
        let mut centroid_alignment = None;
        if let ContactMode::Drag {
            direction,
            run_length,
        } = mode
        {
            let direction = unit(*direction).unwrap_or([1.0, 0.0]);
            let run = self.clear_run(index, point, direction, *run_length);
            checks.push(run_check(run, *run_length));
            centroid_alignment = Some(alignment_of(point, direction, island.centroid));
        }
        let score = score_of(&checks, centroid_alignment);
        let summary = summarize(&checks, &island);
        StructuralVerdict {
            point,
            mode: mode.clone(),
            layer: self.layer,
            island,
            footprint,
            checks,
            score,
            centroid_alignment,
            summary,
        }
    }
}

/// Bed-adhesion criterion.
///
/// Threshold reasoning ([`ContactConfig::min_bed_contact_area`]): a
/// 10 x 10 mm first-layer patch of PLA on a clean textured PEI sheet
/// holds on the order of tens of newtons in shear -- two orders of
/// magnitude above the fraction of a newton a probe tap applies, and
/// comfortably above the few newtons a drag can develop. Below that the
/// margin stops being a margin.
fn adhesion_check(area: f64, threshold: f64, status: TraceStatus) -> CriterionCheck {
    let passed = area >= threshold;
    let reason = match status {
        TraceStatus::BedLayerMissing { layer_zero_z } => format!(
            "the modelled window does not start at the bed (layer 0 is at Z{layer_zero_z:.2}), \
             so the bed footprint is unknown; refusing rather than guessing"
        ),
        TraceStatus::Broken { layer } => format!(
            "the feature could not be traced to the bed: no supporting material below layer \
             {layer}, so nothing is known to hold it down"
        ),
        TraceStatus::OnBed | TraceStatus::Anchored if passed => format!(
            "{area:.1} mm^2 of bed contact holds this feature ({threshold:.1} mm^2 required)"
        ),
        TraceStatus::OnBed | TraceStatus::Anchored => format!(
            "only {area:.1} mm^2 of bed contact holds this feature ({threshold:.1} mm^2 \
             required); it can shear off the bed"
        ),
    };
    CriterionCheck {
        criterion: StructuralCriterion::BedAdhesion,
        passed,
        measured: area,
        threshold,
        unit: CriterionUnit::SquareMillimetres,
        reason,
    }
}

/// Tipping criterion.
///
/// Threshold reasoning ([`ContactConfig::max_aspect_ratio`]): a lateral
/// force `F` at height `z` applies a moment `F*z` about the bed contact,
/// resisted by the adhesion acting over a lever arm on the order of the
/// footprint's own width, `sqrt(area)`. The dimensionless group
/// `z / sqrt(area)` is therefore the tipping number, and 3.0 is the
/// familiar "three times as tall as it is wide" rule of thumb for a part
/// that starts wanting to come off the plate. This is a heuristic, not a
/// simulation: it ignores material stiffness, adhesion chemistry, infill
/// density and the actual force magnitude.
fn tipping_check(aspect: f64, threshold: f64, height: f64, area: f64) -> CriterionCheck {
    let passed = aspect <= threshold;
    let reason = if passed {
        format!(
            "{height:.1} mm tall on a {area:.1} mm^2 footprint: aspect {aspect:.2} \
             (limit {threshold:.2})"
        )
    } else {
        format!(
            "{height:.1} mm tall on only {area:.1} mm^2 of footprint: aspect {aspect:.2} exceeds \
             the {threshold:.2} tipping limit; lateral load would lever it off the bed"
        )
    };
    CriterionCheck {
        criterion: StructuralCriterion::Tipping,
        passed,
        measured: aspect,
        threshold,
        unit: CriterionUnit::Ratio,
        reason,
    }
}

/// Slenderness criterion.
///
/// Threshold reasoning ([`ContactConfig::min_feature_width`]): 5 mm is
/// about a dozen 0.4 mm extrusions side by side. Below that a feature is
/// a fin -- its section modulus falls with the square of the width, and
/// the layer bond, not the bulk material, carries the bending stress.
/// Measured on the island's bounding box, so a diagonal fin is judged by
/// its enclosing box rather than its true width: an over-estimate of
/// width, and the one place this module is *not* conservative, called
/// out here on purpose.
fn width_check(min_dimension: f64, threshold: f64) -> CriterionCheck {
    let passed = min_dimension >= threshold;
    let reason = if passed {
        format!("island is {min_dimension:.1} mm across at its narrowest (min {threshold:.1} mm)")
    } else {
        format!(
            "island is only {min_dimension:.1} mm across at its narrowest ({threshold:.1} mm \
             required); a fin this thin snaps rather than resists"
        )
    };
    CriterionCheck {
        criterion: StructuralCriterion::FeatureWidth,
        passed,
        measured: min_dimension,
        threshold,
        unit: CriterionUnit::Millimetres,
        reason,
    }
}

/// Edge-margin criterion.
///
/// Threshold reasoning ([`ContactConfig::min_edge_margin`]): 3 mm is
/// several nozzle diameters plus the positional uncertainty a
/// post-crash frame carries. A nozzle that reaches the edge of the
/// material loses the reading *and* applies a peeling force at exactly
/// the place adhesion is weakest.
fn margin_check(clearance: f64, threshold: f64) -> CriterionCheck {
    let passed = clearance >= threshold;
    let reason = if passed {
        format!("{clearance:.1} mm of material all round the contact point (min {threshold:.1} mm)")
    } else {
        format!(
            "only {clearance:.1} mm of material around the contact point ({threshold:.1} mm \
             required); the nozzle would run off the edge and peel it"
        )
    };
    CriterionCheck {
        criterion: StructuralCriterion::EdgeMargin,
        passed,
        measured: clearance,
        threshold,
        unit: CriterionUnit::Millimetres,
        reason,
    }
}

/// Drag-run criterion: the whole lateral run must stay on one island at
/// the required margin.
fn run_check(available: f64, required: f64) -> CriterionCheck {
    let passed = available >= required;
    let reason = if passed {
        format!("{available:.1} mm of clear run available ({required:.1} mm needed)")
    } else {
        format!(
            "only {available:.1} mm of clear run before the material runs out or narrows \
             ({required:.1} mm needed)"
        )
    };
    CriterionCheck {
        criterion: StructuralCriterion::DragRun,
        passed,
        measured: available,
        threshold: required,
        unit: CriterionUnit::Millimetres,
        reason,
    }
}

/// Structural desirability in `[0, 1]`: the mean of one normalized term
/// per criterion, each saturating at twice its threshold, plus the
/// centroid-alignment term for drags. Ranking only -- never a gate.
fn score_of(checks: &[CriterionCheck], alignment: Option<f64>) -> f64 {
    let mut terms: Vec<f64> = Vec::with_capacity(checks.len() + 1);
    for check in checks {
        let term = match check.criterion {
            StructuralCriterion::Tipping => {
                clamp01((2.0 * check.threshold - check.measured) / (2.0 * check.threshold))
            }
            _ => clamp01(check.measured / (2.0 * check.threshold)),
        };
        terms.push(term);
    }
    if let Some(alignment) = alignment {
        terms.push(clamp01(f64::midpoint(1.0, alignment)));
    }
    if terms.is_empty() {
        0.0
    } else {
        terms.iter().sum::<f64>() / terms.len() as f64
    }
}

/// One-line operator summary of a verdict.
fn summarize(checks: &[CriterionCheck], island: &Island) -> String {
    match checks.iter().find(|c| !c.passed) {
        Some(check) => format!("unsafe ({}): {}", check.criterion.name(), check.reason),
        None => format!(
            "safe: island {} of layer {}, {:.1} mm^2 of plastic, {:.1} mm across",
            island.index,
            island.layer,
            island.area,
            island.bbox.min_dimension()
        ),
    }
}

/// Structural verdict for one arbitrary XY, building the analysis on the
/// spot.
///
/// Convenience for one-shot validation; a UI validating many points on
/// the same layer should build a [`StructuralAnalysis`] once and call
/// [`StructuralAnalysis::assess`] instead.
pub fn assess_contact_point(
    model: &LayerModel,
    layer: u32,
    point: [f64; 2],
    mode: &ContactMode,
    config: &ContactConfig,
) -> Result<StructuralAssessment, ContactError> {
    let analysis = StructuralAnalysis::build(model, layer, config)?;
    Ok(analysis.assess(point, mode))
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact geometry on constructed coordinates
mod tests {
    use super::*;
    use crate::model::{build_layer_model, ModelConfig};
    use plr_gcode::{ByteSpan, GcodeState};
    use std::fmt::Write as _;

    fn model_of(text: &str) -> LayerModel {
        build_layer_model(
            GcodeState::new(),
            text.as_bytes(),
            0,
            &ModelConfig::default(),
        )
    }

    /// Scan lines filling `x_lo..x_hi` by `y_lo..y_hi` at 0.4 mm.
    fn rect(x_lo: f64, x_hi: f64, y_lo: f64, y_hi: f64) -> String {
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

    /// A two-layer model whose layers both carry `body`.
    fn stacked(body: &str) -> LayerModel {
        let text = format!(
            "G90\nM83\nG92 E0\nG1 Z0.2 F7200\n;TYPE:Internal solid infill\n{body}\
             G1 Z0.4 F7200\n;TYPE:Internal solid infill\n{body}"
        );
        model_of(&text)
    }

    fn analyse(model: &LayerModel, layer: u32) -> StructuralAnalysis {
        StructuralAnalysis::build(model, layer, &ContactConfig::default()).expect("analysis")
    }

    fn verdict_at(
        analysis: &StructuralAnalysis,
        point: [f64; 2],
        mode: &ContactMode,
    ) -> StructuralVerdict {
        match analysis.assess(point, mode) {
            StructuralAssessment::Evaluated(verdict) => *verdict,
            other => panic!("expected a verdict, got {other:?}"),
        }
    }

    #[test]
    fn bounding_box_geometry() {
        let bbox = BoundingBox {
            min: [1.0, 2.0],
            max: [5.0, 8.0],
        };
        assert_eq!(bbox.width(), 4.0);
        assert_eq!(bbox.height(), 6.0);
        assert_eq!(bbox.min_dimension(), 4.0);
        assert!((bbox.diagonal() - 52.0_f64.sqrt()).abs() < 1e-12);
        assert!(bbox.contains([1.0, 2.0], 0.0));
        assert!(!bbox.contains([0.9, 2.0], 0.0));
        assert!(bbox.contains([0.9, 2.0], 0.2));
        assert!(!bbox.contains([f64::NAN, 2.0], 1.0));
    }

    #[test]
    fn criterion_names_and_units_are_stable() {
        assert_eq!(StructuralCriterion::BedAdhesion.name(), "bed_adhesion");
        assert_eq!(StructuralCriterion::Tipping.name(), "tipping");
        assert_eq!(StructuralCriterion::FeatureWidth.name(), "feature_width");
        assert_eq!(StructuralCriterion::EdgeMargin.name(), "edge_margin");
        assert_eq!(StructuralCriterion::DragRun.name(), "drag_run");
        assert_eq!(CriterionUnit::Millimetres.symbol(), "mm");
        assert_eq!(CriterionUnit::SquareMillimetres.symbol(), "mm^2");
        assert_eq!(CriterionUnit::Ratio.symbol(), "");
        assert_eq!(ContactMode::default(), ContactMode::Tap);
    }

    /// Every criterion at, just under and just over its threshold. The
    /// thresholds are inclusive on the safe side: measuring exactly the
    /// required value passes.
    #[test]
    fn criteria_boundaries_are_inclusive_on_the_safe_side() {
        let anchored = TraceStatus::Anchored;
        assert!(adhesion_check(100.0, 100.0, anchored).passed);
        assert!(!adhesion_check(99.999, 100.0, anchored).passed);
        assert!(adhesion_check(100.001, 100.0, anchored).passed);
        assert!(tipping_check(3.0, 3.0, 6.0, 4.0).passed);
        assert!(!tipping_check(3.001, 3.0, 6.0, 4.0).passed);
        assert!(width_check(5.0, 5.0).passed);
        assert!(!width_check(4.999, 5.0).passed);
        assert!(margin_check(3.0, 3.0).passed);
        assert!(!margin_check(2.999, 3.0).passed);
        assert!(run_check(12.0, 12.0).passed);
        assert!(!run_check(11.999, 12.0).passed);
        // NaN never satisfies a threshold.
        assert!(!adhesion_check(f64::NAN, 100.0, anchored).passed);
        assert!(!tipping_check(f64::NAN, 3.0, 1.0, 1.0).passed);
    }

    #[test]
    fn unresolved_traces_explain_themselves() {
        let broken = adhesion_check(0.0, 100.0, TraceStatus::Broken { layer: 4 });
        assert!(!broken.passed);
        assert!(broken.reason.contains("below layer 4"), "{}", broken.reason);
        let missing = adhesion_check(
            0.0,
            100.0,
            TraceStatus::BedLayerMissing {
                layer_zero_z: 12.25,
            },
        );
        assert!(missing.reason.contains("Z12.25"), "{}", missing.reason);
        assert_eq!(broken.unit, CriterionUnit::SquareMillimetres);
    }

    #[test]
    fn islands_split_on_a_gap_and_merge_across_the_line_spacing() {
        // Two 4 mm squares 4 mm apart: two islands. Their scan lines are
        // 0.4 mm apart, well inside the 0.6 mm link tolerance, so each
        // square is one island rather than eleven.
        let body = format!("{}{}", rect(0.0, 4.0, 0.0, 4.0), rect(8.0, 12.0, 0.0, 4.0));
        let model = stacked(&body);
        let analysis = analyse(&model, 1);
        assert_eq!(analysis.islands().len(), 2);
        let first = &analysis.islands()[0];
        assert_eq!(first.bbox.min, [0.0, 0.0]);
        assert_eq!(first.bbox.max, [4.0, 4.0]);
        assert_eq!(first.segment_count, 11);
        assert_eq!(first.path_length, 44.0);
        assert!((first.centroid[0] - 2.0).abs() < 1e-9);
        assert!((first.centroid[1] - 2.0).abs() < 1e-9);
        assert!(!first.estimate_degraded);
        assert_eq!(analysis.islands()[1].bbox.min, [8.0, 0.0]);
        assert_eq!(analysis.layer(), 1);
        assert_eq!(analysis.layer_z(), 0.4);
    }

    #[test]
    fn link_tolerance_is_the_split_boundary() {
        // Two parallel beads exactly `island_link_tolerance` apart are
        // one island; 1 % further apart they are two.
        let text = |gap: f64| {
            format!(
                "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Internal solid infill\n\
                 G1 X0 Y0 F9000\nG1 X10 Y0 E1 F1800\n\
                 G1 X0 Y{gap} F9000\nG1 X10 Y{gap} E1 F1800\n"
            )
        };
        let joined = model_of(&text(0.6));
        assert_eq!(analyse(&joined, 0).islands().len(), 1);
        let split = model_of(&text(0.606));
        assert_eq!(analyse(&split, 0).islands().len(), 2);
    }

    #[test]
    fn layer_zero_islands_are_their_own_footprint() {
        let model = stacked(&rect(0.0, 20.0, 0.0, 20.0));
        let analysis = analyse(&model, 0);
        let trace = analysis.footprint(0).expect("trace");
        assert_eq!(trace.status, TraceStatus::OnBed);
        assert_eq!(trace.reached_layer, 0);
        assert_eq!(trace.bed_island_indices, vec![0]);
        assert_eq!(trace.effective_area, analysis.islands()[0].area);
        assert_eq!(trace.weakest_link_area, trace.bed_area);
        assert_eq!(analysis.bed_islands().len(), 1);
    }

    #[test]
    fn an_unsupported_island_breaks_the_trace() {
        // Layer 1 carries a second square with nothing below it.
        let base = rect(0.0, 8.0, 0.0, 8.0);
        let floating = rect(60.0, 68.0, 60.0, 68.0);
        let text = format!(
            "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Internal solid infill\n{base}\
             G1 Z0.4 F7200\n;TYPE:Internal solid infill\n{base}{floating}"
        );
        let model = model_of(&text);
        let analysis = analyse(&model, 1);
        assert_eq!(analysis.islands().len(), 2);
        let supported = analysis.footprint(0).expect("trace 0");
        assert_eq!(supported.status, TraceStatus::Anchored);
        assert!(supported.effective_area > 0.0);
        let orphan = analysis.footprint(1).expect("trace 1");
        assert_eq!(orphan.status, TraceStatus::Broken { layer: 1 });
        assert_eq!(orphan.effective_area, 0.0);
        assert_eq!(orphan.bed_area, 0.0);
        assert!(orphan.bed_island_indices.is_empty());
        assert_eq!(orphan.reached_layer, 1);
        // ... and the verdict there refuses on adhesion.
        let verdict = verdict_at(&analysis, [64.0, 64.0], &ContactMode::Tap);
        assert_eq!(
            verdict.outcome(),
            StructuralOutcome::Unsafe {
                primary: StructuralCriterion::BedAdhesion
            }
        );
        assert!(verdict.summary.contains("could not be traced to the bed"));
    }

    #[test]
    fn a_window_that_does_not_start_at_the_bed_is_refused() {
        let body = rect(0.0, 20.0, 0.0, 20.0);
        let text = format!(
            "G90\nM83\nG1 Z2.0 F7200\n;TYPE:Internal solid infill\n{body}\
             G1 Z2.2 F7200\n;TYPE:Internal solid infill\n{body}"
        );
        let model = model_of(&text);
        let analysis = analyse(&model, 1);
        let trace = analysis.footprint(0).expect("trace");
        assert_eq!(
            trace.status,
            TraceStatus::BedLayerMissing { layer_zero_z: 2.0 }
        );
        assert_eq!(trace.effective_area, 0.0);
        assert!(analysis.bed_islands().is_empty());
        let verdict = verdict_at(&analysis, [10.0, 10.0], &ContactMode::Tap);
        assert!(verdict.summary.contains("does not start at the bed"));
        // The same geometry starting at a normal first-layer Z traces.
        let ok = stacked(&body);
        assert_eq!(
            analyse(&ok, 1).footprint(0).expect("trace").status,
            TraceStatus::Anchored
        );
    }

    #[test]
    fn off_material_points_report_where_to_jog() {
        let model = stacked(&rect(0.0, 10.0, 0.0, 10.0));
        let analysis = analyse(&model, 1);
        let StructuralAssessment::OffMaterial {
            distance,
            nearest_point,
            nearest_island,
        } = analysis.assess([30.0, 5.0], &ContactMode::Tap)
        else {
            panic!("expected OffMaterial");
        };
        assert!((distance - 20.0).abs() < 0.31, "distance {distance}");
        assert_eq!(nearest_island, Some(0));
        let nearest = nearest_point.expect("a nearest point");
        assert!(nearest[0] <= 10.0 + 1e-9 && nearest[1] >= -1e-9);
        assert_eq!(analysis.island_at([30.0, 5.0]), None);
        assert_eq!(analysis.island_at([5.0, 5.0]), Some(0));
    }

    #[test]
    fn invalid_points_and_modes_are_typed() {
        let model = stacked(&rect(0.0, 10.0, 0.0, 10.0));
        let analysis = analyse(&model, 1);
        assert_eq!(
            analysis.assess([f64::NAN, 1.0], &ContactMode::Tap),
            StructuralAssessment::InvalidPoint {
                param: InvalidInput::Point
            }
        );
        assert_eq!(
            analysis.assess(
                [5.0, 5.0],
                &ContactMode::Drag {
                    direction: [0.0, 0.0],
                    run_length: 5.0
                }
            ),
            StructuralAssessment::InvalidPoint {
                param: InvalidInput::DragDirection
            }
        );
        assert_eq!(
            analysis.assess(
                [5.0, 5.0],
                &ContactMode::Drag {
                    direction: [1.0, 0.0],
                    run_length: 0.0
                }
            ),
            StructuralAssessment::InvalidPoint {
                param: InvalidInput::DragRunLength
            }
        );
        assert_eq!(analysis.island_at([f64::INFINITY, 1.0]), None);
    }

    #[test]
    fn clear_run_refuses_unusable_arguments() {
        let model = stacked(&rect(0.0, 20.0, 0.0, 20.0));
        let analysis = analyse(&model, 1);
        assert_eq!(analysis.clear_run(9, [10.0, 10.0], [1.0, 0.0], 5.0), 0.0);
        assert_eq!(analysis.clear_run(0, [10.0, 10.0], [0.0, 0.0], 5.0), 0.0);
        assert_eq!(analysis.clear_run(0, [10.0, 10.0], [1.0, 0.0], 0.0), 0.0);
        assert_eq!(
            analysis.clear_run(0, [f64::NAN, 10.0], [1.0, 0.0], 5.0),
            0.0
        );
        assert_eq!(analysis.largest_clear_run(9, [10.0, 10.0]), None);
        // A run that starts off the material is zero, not negative.
        assert_eq!(analysis.clear_run(0, [50.0, 50.0], [1.0, 0.0], 5.0), 0.0);
    }

    /// The clear run from the centre of a 20 x 20 plate at a 3 mm
    /// margin: material spans -0.225..20.225, so the region with 3 mm of
    /// clearance is about 2.8..17.2 in both axes and the longest run
    /// from (10,10) is the 45-degree diagonal, 7.2 * sqrt(2) ~ 10.2 mm,
    /// minus the raster's conservative slack.
    #[test]
    fn largest_clear_run_is_the_diagonal_of_a_square_plate() {
        let model = stacked(&rect(0.0, 20.0, 0.0, 20.0));
        let analysis = analyse(&model, 1);
        let run = analysis
            .largest_clear_run(0, [10.0, 10.0])
            .expect("a clear run");
        assert!((9.5..=10.3).contains(&run.length), "length {}", run.length);
        assert!(
            (run.direction[0].abs() - run.direction[1].abs()).abs() < 1e-9,
            "expected a diagonal, got {:?}",
            run.direction
        );
        assert_eq!(run.island, 0);
        assert_eq!(run.margin, 3.0);
        assert_eq!(run.start, [10.0, 10.0]);
        // Straight along an axis it is the 7.2 mm half-width, not the
        // 10.2 mm diagonal.
        let axial = analysis.clear_run(0, [10.0, 10.0], [1.0, 0.0], 20.0);
        assert!((6.8..=7.3).contains(&axial), "axial {axial}");
    }

    #[test]
    fn centroid_alignment_scores_drags_toward_the_mass() {
        let model = stacked(&rect(0.0, 20.0, 0.0, 20.0));
        let analysis = analyse(&model, 1);
        let toward = verdict_at(
            &analysis,
            [4.0, 10.0],
            &ContactMode::Drag {
                direction: [1.0, 0.0],
                run_length: 1.0,
            },
        );
        let away = verdict_at(
            &analysis,
            [4.0, 10.0],
            &ContactMode::Drag {
                direction: [-1.0, 0.0],
                run_length: 1.0,
            },
        );
        assert_eq!(toward.centroid_alignment, Some(1.0));
        assert_eq!(away.centroid_alignment, Some(-1.0));
        assert!(toward.score > away.score);
        // Alignment is a score input, never a gate: both are safe.
        assert_eq!(toward.outcome(), StructuralOutcome::Safe);
        assert_eq!(away.outcome(), StructuralOutcome::Safe);
        // Standing exactly on the centroid has no preferred direction.
        assert_eq!(alignment_of([1.0, 1.0], [1.0, 0.0], [1.0, 1.0]), 0.0);
    }

    #[test]
    fn verdict_accessors_and_score_range() {
        let model = stacked(&rect(0.0, 20.0, 0.0, 20.0));
        let analysis = analyse(&model, 1);
        let verdict = verdict_at(&analysis, [10.0, 10.0], &ContactMode::Tap);
        assert_eq!(verdict.outcome(), StructuralOutcome::Safe);
        assert!(verdict.failures().is_empty());
        assert_eq!(verdict.checks.len(), 4);
        assert!((0.0..=1.0).contains(&verdict.score));
        assert!(verdict.centroid_alignment.is_none());
        assert_eq!(verdict.layer, 1);
        assert_eq!(verdict.point, [10.0, 10.0]);
        let adhesion = verdict
            .check(StructuralCriterion::BedAdhesion)
            .expect("adhesion check");
        assert!(adhesion.passed && adhesion.threshold == 100.0);
        assert_eq!(verdict.check(StructuralCriterion::DragRun), None);
        assert!(verdict.summary.starts_with("safe:"));
        // Every reported number is finite, so the payload round-trips
        // through JSON (serde_json cannot encode an infinity).
        for check in &verdict.checks {
            assert!(check.measured.is_finite() && check.threshold.is_finite());
        }
        let json = serde_json::to_string(&verdict).expect("serialize");
        let back: StructuralVerdict = serde_json::from_str(&json).expect("deserialize");
        // serde_json's float printing is lossy in the last digit, so the
        // structure is compared exactly and the reals to 1e-9.
        assert_eq!(back.checks.len(), verdict.checks.len());
        assert_eq!(back.summary, verdict.summary);
        assert_eq!(back.footprint.status, verdict.footprint.status);
        assert!((back.score - verdict.score).abs() < 1e-9);
        assert!((back.footprint.effective_area - verdict.footprint.effective_area).abs() < 1e-9);
    }

    #[test]
    fn scores_and_clamps_behave_at_the_extremes() {
        assert_eq!(clamp01(f64::NAN), 0.0);
        assert_eq!(clamp01(-1.0), 0.0);
        assert_eq!(clamp01(7.0), 1.0);
        assert_eq!(finite_or(f64::INFINITY, 4.0), 4.0);
        assert_eq!(finite_or(2.0, 4.0), 2.0);
        assert_eq!(unit([0.0, 0.0]), None);
        assert_eq!(unit([f64::NAN, 1.0]), None);
        assert_eq!(unit([0.0, -3.0]), Some([0.0, -1.0]));
        assert_eq!(score_of(&[], None), 0.0);
        // A failing tipping ratio scores zero, not a negative number.
        let checks = vec![tipping_check(99.0, 3.0, 99.0, 1.0)];
        assert_eq!(score_of(&checks, None), 0.0);
    }

    #[test]
    fn the_free_helper_matches_the_reusable_analysis() {
        let model = stacked(&rect(0.0, 20.0, 0.0, 20.0));
        let config = ContactConfig::default();
        let direct = assess_contact_point(&model, 1, [10.0, 10.0], &ContactMode::Tap, &config)
            .expect("assessment");
        let reused = analyse(&model, 1).assess([10.0, 10.0], &ContactMode::Tap);
        assert_eq!(direct, reused);
        assert_eq!(
            assess_contact_point(&model, 9, [0.0, 0.0], &ContactMode::Tap, &config).unwrap_err(),
            ContactError::LayerOutOfRange {
                resume_layer: 9,
                layers: 2
            }
        );
        let bad = ContactConfig {
            min_feature_width: -1.0,
            ..ContactConfig::default()
        };
        assert_eq!(
            StructuralAnalysis::build(&model, 1, &bad).unwrap_err(),
            ContactError::InvalidParams {
                param: "min_feature_width"
            }
        );
    }

    #[test]
    fn debug_is_a_summary_not_a_grid_dump() {
        let model = stacked(&rect(0.0, 10.0, 0.0, 10.0));
        let text = format!("{:?}", analyse(&model, 1));
        assert!(text.contains("StructuralAnalysis"));
        assert!(text.contains("islands: 1"));
        assert!(text.len() < 200, "debug output is a grid dump: {text}");
    }

    #[test]
    fn degenerate_geometry_produces_islands_without_panicking() {
        // A hand-built model with a zero-length extrusion (a retract
        // artefact the g-code path cannot emit but a caller can hand a
        // model carrying): the island has no path length at all and the
        // centroid falls back to the bounding-box centre.
        let mut model = model_of(
            "G90
M83
G1 Z0.2 F7200
;TYPE:Internal solid infill
G1 X5 Y5 E1 F1800
",
        );
        let degenerate = XySegment {
            start: [40.0, 40.0],
            end: [40.0, 40.0],
            z: 0.2,
            e_start: 0.0,
            e_end: 1.0,
            span: ByteSpan { start: 90, end: 91 },
            arc: None,
        };
        let nonfinite = XySegment {
            start: [f64::NAN, 0.0],
            end: [1.0, 1.0],
            ..degenerate.clone()
        };
        model.layers[0].paths[0].segments.push(degenerate);
        model.layers[0].paths[0].segments.push(nonfinite);
        let analysis = analyse(&model, 0);
        // The non-finite segment is filtered; the degenerate one is its
        // own island.
        assert_eq!(analysis.islands().len(), 2);
        let point = &analysis.islands()[1];
        assert_eq!(point.path_length, 0.0);
        assert_eq!(point.centroid, [40.0, 40.0]);
        assert_eq!(point.segment_count, 1);
        assert!(point.area >= 0.0);
        assert_eq!(
            analysis
                .islands()
                .iter()
                .map(|i| i.segment_count)
                .sum::<usize>(),
            2
        );
    }

    #[test]
    fn island_areas_stay_below_the_capsule_bound() {
        // The area estimator must never over-report: a 10 x 10 patch of
        // 26 scan lines is at most 26 capsules' worth of plastic.
        let model = stacked(&rect(0.0, 10.0, 0.0, 10.0));
        let analysis = analyse(&model, 1);
        let island = &analysis.islands()[0];
        let capsules = 26.0 * (10.0 * 0.45 + std::f64::consts::PI * 0.225 * 0.225);
        assert!(island.area <= capsules, "{} > {capsules}", island.area);
        assert!(island.area > 0.75 * 10.0 * 10.4, "{}", island.area);
    }
}
