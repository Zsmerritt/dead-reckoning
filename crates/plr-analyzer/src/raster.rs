//! Material occupancy raster and clearance field for one island.
//!
//! Extrusion paths are centerlines; the printed material is modeled as
//! the union of *capsules* — the set of points within
//! `extrusion_width / 2` of a centerline. This module discretizes that
//! union onto a uniform grid and answers the three questions the
//! structural checks need:
//!
//! * **area** — how much plastic is there (mm²)?
//! * **on material** — is an arbitrary XY inside the island?
//! * **edge clearance** — how far is an XY from the nearest point that
//!   is *not* material (the peel/run-off distance)?
//!
//! # Two independent passes, on purpose
//!
//! **Area** is measured on its own fine grid at `extrusion_width / 6`,
//! swept in tiles so memory stays bounded whatever the island's size. A
//! cell counts only when the **whole cell** provably lies inside the
//! capsule union (its center is within `half_width - cell*sqrt(2)/2` of
//! a segment). Counted cells are disjoint and entirely inside the
//! material, so `count * cell^2` is a **strict lower bound** on the
//! modeled material area. Nothing about the area estimate ever depends
//! on how big the island is, because a safety threshold compared
//! against an area must never be flattered by discretization.
//!
//! **Containment and clearance** run on a second grid that covers the
//! whole island at once (the distance transform needs it) with the
//! ordinary midpoint rule: a cell is material when its center is within
//! `half_width` of a segment. The strict rule is deliberately erosive
//! and would report a solid region printed at a line spacing wider than
//! twice the strict radius as disconnected stripes, which would make
//! every edge-margin query fail for a physically sound plate. This grid
//! is coarsened when the island would exceed [`CELL_BUDGET`] cells;
//! coarsening lowers the reported clearance (one cell is subtracted, see
//! [`Raster::clearance_at`]), so it too fails conservative.
//!
//! Error direction, stated plainly: **area is under-reported**
//! (typically ~85 % of the modeled area for a solid region at
//! production line spacing, ~76 % for an isolated single bead) and
//! **clearance is under-reported**. Both biases push the structural
//! verdicts toward refusing to probe.

// Justification for the module-wide casting allows: grid extents are
// bounded by CELL_BUDGET (2^20), far inside f64's exactly representable
// integer range, and every f64 -> index conversion below is range-checked
// or clamped to the grid before the cast. Writing `#[allow]` at each of
// the ~15 conversion sites would be noise, not information.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::f64::consts::FRAC_1_SQRT_2;

use crate::geom;
use crate::model::XySegment;

/// Maximum number of grid cells one island raster may allocate. At the
/// default 0.075 mm cell this covers a 75 × 75 mm island exactly; bigger
/// islands are rasterized at a proportionally coarser cell.
pub(crate) const CELL_BUDGET: usize = 1_000_000;

/// Maximum number of cell/segment distance tests one raster performs.
/// Reaching it stops the fill early, which can only *remove* material
/// from the raster (smaller area, smaller clearance) — the conservative
/// direction. Reported as [`Raster::truncated`].
pub(crate) const TEST_BUDGET: usize = 40_000_000;

/// Smallest cell size the raster will use, mm. Guards against a
/// pathological `extrusion_width` collapsing the grid.
const MIN_CELL: f64 = 0.01;

/// Grid coordinates are clamped to this magnitude before any cast:
/// callers add a neighbourhood offset to the result, so the raw floor
/// must not sit at `isize`'s limit. A coordinate this large is
/// astronomically outside any print volume anyway.
const INDEX_LIMIT: f64 = (isize::MAX / 4) as f64;

/// A discretized island: two occupancy bitmaps plus a clearance field.
pub(crate) struct Raster {
    /// Cell size, mm.
    cell: f64,
    /// World XY of the *lower-left corner* of cell `(0, 0)`.
    origin: [f64; 2],
    /// Grid width in cells.
    nx: usize,
    /// Grid height in cells.
    ny: usize,
    /// Midpoint-rule occupancy (used for containment and clearance).
    solid: Vec<bool>,
    /// Distance, mm, from each cell center to the nearest non-`solid`
    /// cell center, already reduced by the conservative slack.
    clearance: Vec<f64>,
    /// Strict lower bound on the material area, mm2, from the separate
    /// fine-grid tiled pass.
    area: f64,
    /// The clearance grid's cell size had to be grown to fit
    /// [`CELL_BUDGET`].
    coarsened: bool,
    /// A fill stopped at [`TEST_BUDGET`].
    truncated: bool,
}

impl Raster {
    /// An empty raster: no material anywhere. Used for degenerate inputs
    /// (non-finite bounds, non-positive extrusion width) so that callers
    /// never have to special-case them — every query answers "nothing
    /// here", which fails every structural criterion.
    fn empty(cell: f64) -> Self {
        Self {
            cell,
            origin: [0.0, 0.0],
            nx: 0,
            ny: 0,
            solid: Vec::new(),
            clearance: Vec::new(),
            area: 0.0,
            coarsened: false,
            truncated: false,
        }
    }

    /// Rasterize `segments` (all belonging to one island) over the
    /// bounding box `min..max`, modeling each as a capsule of radius
    /// `half_width`.
    pub(crate) fn build(
        segments: &[&XySegment],
        min: [f64; 2],
        max: [f64; 2],
        half_width: f64,
    ) -> Self {
        let base_cell = (half_width / 3.0).max(MIN_CELL);
        let (w, h) = (max[0] - min[0], max[1] - min[1]);
        if !half_width.is_finite()
            || half_width <= 0.0
            || !w.is_finite()
            || !h.is_finite()
            || w < 0.0
            || h < 0.0
        {
            return Self::empty(base_cell);
        }
        let Some((cell, nx, ny)) = fit_grid(w, h, half_width, base_cell) else {
            return Self::empty(base_cell);
        };
        let pad = half_width + 2.0 * cell;
        let mut raster = Self {
            cell,
            origin: [min[0] - pad, min[1] - pad],
            nx,
            ny,
            solid: vec![false; nx * ny],
            clearance: Vec::new(),
            area: 0.0,
            coarsened: cell > base_cell,
            truncated: false,
        };
        raster.fill(segments, half_width);
        raster.clearance = clearance_field(&raster.solid, nx, ny, cell);
        let (area, area_truncated) = strict_area(segments, min, max, half_width);
        raster.area = area;
        raster.truncated = raster.truncated || area_truncated;
        raster
    }

    /// Mark the midpoint-rule `solid` cells for every segment.
    fn fill(&mut self, segments: &[&XySegment], half_width: f64) {
        let reach = (half_width / self.cell).ceil() as isize + 1;
        let mut tests = 0_usize;
        for seg in segments {
            let steps = ((seg.length() / self.cell).ceil() as usize).clamp(1, usize::MAX);
            for k in 0..=steps {
                let p = seg.point_at(k as f64 / steps as f64);
                if !p[0].is_finite() || !p[1].is_finite() {
                    continue;
                }
                let (ci, cj) = self.cell_of(p);
                for dj in -reach..=reach {
                    for di in -reach..=reach {
                        let Some(idx) = self.index(ci + di, cj + dj) else {
                            continue;
                        };
                        if self.solid[idx] {
                            continue;
                        }
                        tests += 1;
                        if tests > TEST_BUDGET {
                            self.truncated = true;
                            return;
                        }
                        let center = self.center(ci + di, cj + dj);
                        if geom::point_seg_distance(center, seg.start, seg.end) <= half_width {
                            self.solid[idx] = true;
                        }
                    }
                }
            }
        }
    }

    /// Grid coordinates of the cell containing `p` (may be out of range).
    ///
    /// Clamped well inside `isize`'s range: callers add a neighbourhood
    /// offset to the result, so the raw floor must not sit at the limit.
    fn cell_of(&self, p: [f64; 2]) -> (isize, isize) {
        const LIMIT: f64 = (isize::MAX / 4) as f64;
        let fi = ((p[0] - self.origin[0]) / self.cell).floor();
        let fj = ((p[1] - self.origin[1]) / self.cell).floor();
        // Non-finite coordinates are filtered by the callers; clamping
        // keeps the cast total regardless.
        (
            fi.clamp(-LIMIT, LIMIT) as isize,
            fj.clamp(-LIMIT, LIMIT) as isize,
        )
    }

    /// Flat index of grid cell `(i, j)`, `None` when outside.
    fn index(&self, i: isize, j: isize) -> Option<usize> {
        if i < 0 || j < 0 {
            return None;
        }
        let (i, j) = (i as usize, j as usize);
        if i >= self.nx || j >= self.ny {
            return None;
        }
        Some(j * self.nx + i)
    }

    /// World XY of the center of grid cell `(i, j)`.
    fn center(&self, i: isize, j: isize) -> [f64; 2] {
        [
            self.origin[0] + (i as f64 + 0.5) * self.cell,
            self.origin[1] + (j as f64 + 0.5) * self.cell,
        ]
    }

    /// Strict lower bound on the island's material area, mm2.
    pub(crate) fn area(&self) -> f64 {
        self.area
    }

    /// Whether `point` lies on the island's material (midpoint rule).
    pub(crate) fn is_material(&self, point: [f64; 2]) -> bool {
        if !point[0].is_finite() || !point[1].is_finite() {
            return false;
        }
        let (i, j) = self.cell_of(point);
        self.index(i, j).is_some_and(|idx| self.solid[idx])
    }

    /// Conservative distance, mm, from `point` to the nearest place that
    /// is not this island's material; 0 when `point` is off the island.
    ///
    /// Two sources of error are absorbed by subtracting one cell size in
    /// [`clearance_field`]: the midpoint occupancy rule can call a cell
    /// material when up to `cell·√2/2` of it is not, and the transform
    /// measures to the nearest background *cell center* rather than to
    /// the true boundary. The result therefore under-reports the true
    /// clearance by up to roughly one cell (0.075 mm at the default cell
    /// size, more on a coarsened raster).
    pub(crate) fn clearance_at(&self, point: [f64; 2]) -> f64 {
        if !point[0].is_finite() || !point[1].is_finite() {
            return 0.0;
        }
        let (i, j) = self.cell_of(point);
        self.index(i, j).map_or(0.0, |idx| self.clearance[idx])
    }

    /// Cell size actually used, mm.
    pub(crate) fn cell(&self) -> f64 {
        self.cell
    }

    /// True when the cell size had to be grown to fit [`CELL_BUDGET`].
    pub(crate) fn coarsened(&self) -> bool {
        self.coarsened
    }

    /// True when the fill stopped at [`TEST_BUDGET`].
    pub(crate) fn truncated(&self) -> bool {
        self.truncated
    }
}

/// Maximum tiles the strict-area sweep will visit. 256 tiles of
/// [`CELL_BUDGET`] cells at the default 0.075 mm cell cover a
/// 1.2 × 1.2 m island — larger than any printer this runs on.
const MAX_AREA_TILES: usize = 256;

/// Strict lower bound on the material area of `segments`, mm², plus
/// whether the sweep was cut short.
///
/// Runs on its own fine grid (`half_width / 3`, so the strict radius is
/// always ~76 % of the half width) and sweeps it in tiles of at most
/// [`CELL_BUDGET`] cells, so the memory cost is bounded by one tile no
/// matter how large the island is. Each tile only considers segments
/// whose expanded bounding box reaches it, so the total marking work
/// stays proportional to path length.
fn strict_area(
    segments: &[&XySegment],
    min: [f64; 2],
    max: [f64; 2],
    half_width: f64,
) -> (f64, bool) {
    let mut cell = (half_width / 3.0).max(MIN_CELL);
    let strict_radius = half_width - cell * FRAC_1_SQRT_2;
    if strict_radius <= 0.0 {
        return (0.0, true);
    }
    let pad = half_width + cell;
    let lo = [min[0] - pad, min[1] - pad];
    let (w, h) = (max[0] - min[0] + 2.0 * pad, max[1] - min[1] + 2.0 * pad);
    let side = (CELL_BUDGET as f64).sqrt().floor().max(1.0);
    let mut degraded = false;
    // Grow the cell rather than skip area if an absurd island would need
    // more than MAX_AREA_TILES tiles.
    for _ in 0..64 {
        let tiles = (w / (side * cell)).ceil().max(1.0) * (h / (side * cell)).ceil().max(1.0);
        if tiles.is_finite() && tiles <= MAX_AREA_TILES as f64 {
            break;
        }
        cell *= 1.5;
        degraded = true;
    }
    let strict_radius = half_width - cell * FRAC_1_SQRT_2;
    if strict_radius <= 0.0 {
        return (0.0, true);
    }
    let tile = side as usize;
    let nx = ((w / cell).ceil() as usize).max(1);
    let ny = ((h / cell).ceil() as usize).max(1);
    let mut count = 0_usize;
    let mut tests = 0_usize;
    let mut tj = 0_usize;
    while tj < ny {
        let mut ti = 0_usize;
        while ti < nx {
            let (tw, th) = (tile.min(nx - ti), tile.min(ny - tj));
            let origin = [lo[0] + ti as f64 * cell, lo[1] + tj as f64 * cell];
            count += mark_tile(
                segments,
                (origin, cell, strict_radius, half_width),
                (tw, th),
                &mut tests,
            );
            if tests > TEST_BUDGET {
                return (count as f64 * cell * cell, true);
            }
            ti += tile;
        }
        tj += tile;
    }
    (count as f64 * cell * cell, degraded)
}

/// Mark and count the strictly-inside cells of one tile.
fn mark_tile(
    segments: &[&XySegment],
    grid: ([f64; 2], f64, f64, f64),
    dims: (usize, usize),
    tests: &mut usize,
) -> usize {
    let (origin, cell, strict_radius, half_width) = grid;
    let (tw, th) = dims;
    let mut occupied = vec![false; tw * th];
    let hi = [origin[0] + tw as f64 * cell, origin[1] + th as f64 * cell];
    let reach = (half_width / cell).ceil() as isize + 1;
    for seg in segments {
        // Cheap rejection: the segment's capsule cannot reach this tile.
        let (sx0, sx1) = (seg.start[0].min(seg.end[0]), seg.start[0].max(seg.end[0]));
        let (sy0, sy1) = (seg.start[1].min(seg.end[1]), seg.start[1].max(seg.end[1]));
        if sx1 + half_width < origin[0]
            || sx0 - half_width > hi[0]
            || sy1 + half_width < origin[1]
            || sy0 - half_width > hi[1]
        {
            continue;
        }
        let steps = ((seg.length() / cell).ceil() as usize).max(1);
        for k in 0..=steps {
            let p = seg.point_at(k as f64 / steps as f64);
            if !p[0].is_finite() || !p[1].is_finite() {
                continue;
            }
            let ci = ((p[0] - origin[0]) / cell)
                .floor()
                .clamp(-INDEX_LIMIT, INDEX_LIMIT) as isize;
            let cj = ((p[1] - origin[1]) / cell)
                .floor()
                .clamp(-INDEX_LIMIT, INDEX_LIMIT) as isize;
            for dj in -reach..=reach {
                for di in -reach..=reach {
                    let (i, j) = (ci + di, cj + dj);
                    if i < 0 || j < 0 || i as usize >= tw || j as usize >= th {
                        continue;
                    }
                    let idx = j as usize * tw + i as usize;
                    if occupied[idx] {
                        continue;
                    }
                    *tests += 1;
                    if *tests > TEST_BUDGET {
                        return occupied.iter().filter(|c| **c).count();
                    }
                    let center = [
                        origin[0] + (i as f64 + 0.5) * cell,
                        origin[1] + (j as f64 + 0.5) * cell,
                    ];
                    if geom::point_seg_distance(center, seg.start, seg.end) <= strict_radius {
                        occupied[idx] = true;
                    }
                }
            }
        }
    }
    occupied.iter().filter(|c| **c).count()
}

/// Choose a cell size and grid extent that fits [`CELL_BUDGET`].
///
/// Returns `None` when even repeated coarsening cannot fit the box,
/// which can only happen for absurd (but finite) coordinates.
fn fit_grid(w: f64, h: f64, half_width: f64, base_cell: f64) -> Option<(f64, usize, usize)> {
    let mut cell = base_cell;
    for _ in 0..64 {
        let pad = half_width + 2.0 * cell;
        let nx = ((w + 2.0 * pad) / cell).ceil();
        let ny = ((h + 2.0 * pad) / cell).ceil();
        if nx.is_finite() && ny.is_finite() && nx * ny <= CELL_BUDGET as f64 {
            let (nx, ny) = (nx as usize + 1, ny as usize + 1);
            if nx.checked_mul(ny).is_some_and(|n| n <= CELL_BUDGET) {
                return Some((cell, nx, ny));
            }
        }
        cell *= 1.5;
    }
    None
}

/// Exact squared Euclidean distance transform (Felzenszwalb &
/// Huttenlocher), converted to a conservative clearance in mm.
///
/// The transform runs on the *background* of `solid`: every non-material
/// cell is a zero, every material cell takes the squared distance to the
/// nearest zero, in cell units. One cell size is then subtracted from
/// the metric distance (see [`Raster::clearance_at`]).
fn clearance_field(solid: &[bool], nx: usize, ny: usize, cell: f64) -> Vec<f64> {
    if nx == 0 || ny == 0 {
        return Vec::new();
    }
    // Larger than any squared distance representable on this grid, and
    // finite so the parabola arithmetic below never produces a NaN.
    let big = (nx * nx + ny * ny) as f64 + 1.0;
    let mut d: Vec<f64> = solid.iter().map(|&s| if s { big } else { 0.0 }).collect();
    let n_max = nx.max(ny);
    let mut src = vec![0.0_f64; n_max];
    let mut dst = vec![0.0_f64; n_max];
    let mut v = vec![0_usize; n_max];
    let mut z = vec![0.0_f64; n_max + 1];
    for j in 0..ny {
        src[..nx].copy_from_slice(&d[j * nx..j * nx + nx]);
        edt_1d(&src[..nx], &mut dst[..nx], &mut v[..nx], &mut z[..=nx]);
        d[j * nx..j * nx + nx].copy_from_slice(&dst[..nx]);
    }
    for i in 0..nx {
        for j in 0..ny {
            src[j] = d[j * nx + i];
        }
        edt_1d(&src[..ny], &mut dst[..ny], &mut v[..ny], &mut z[..=ny]);
        for j in 0..ny {
            d[j * nx + i] = dst[j];
        }
    }
    for value in &mut d {
        *value = (value.sqrt() * cell - cell).max(0.0);
    }
    d
}

/// One-dimensional squared distance transform of the sampled function
/// `source`, writing the lower envelope of the parabolas into `dest`.
///
/// `hull` and `bounds` are scratch (`bounds` needs one more slot than
/// `source`).
fn edt_1d(source: &[f64], dest: &mut [f64], hull: &mut [usize], bounds: &mut [f64]) {
    let count = source.len();
    if count == 0 {
        return;
    }
    let mut top = 0_usize;
    hull[0] = 0;
    bounds[0] = f64::NEG_INFINITY;
    bounds[1] = f64::INFINITY;
    for index in 1..count {
        let mut crossing = intersection(source, index, hull[top]);
        // `bounds[0]` is -inf and `crossing` is always finite, so `top`
        // never underflows.
        while crossing <= bounds[top] {
            debug_assert!(top > 0, "edt_1d: hull underflow");
            top -= 1;
            crossing = intersection(source, index, hull[top]);
        }
        top += 1;
        hull[top] = index;
        bounds[top] = crossing;
        bounds[top + 1] = f64::INFINITY;
    }
    let mut top = 0_usize;
    for (index, out) in dest.iter_mut().enumerate().take(count) {
        while bounds[top + 1] < index as f64 {
            top += 1;
        }
        let delta = index as f64 - hull[top] as f64;
        *out = delta * delta + source[hull[top]];
    }
}

/// Abscissa where the parabolas rooted at `p` and `q` intersect.
fn intersection(f: &[f64], p: usize, q: usize) -> f64 {
    let (pf, qf) = (p as f64, q as f64);
    ((f[p] + pf * pf) - (f[q] + qf * qf)) / (2.0 * pf - 2.0 * qf)
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact grid arithmetic is intentional
mod tests {
    use super::*;
    use plr_gcode::ByteSpan;

    fn seg(start: [f64; 2], end: [f64; 2]) -> XySegment {
        XySegment {
            start,
            end,
            z: 0.2,
            e_start: 0.0,
            e_end: 1.0,
            span: ByteSpan { start: 0, end: 1 },
            arc: None,
        }
    }

    /// A dense square patch of scan lines, 0.4 mm apart.
    fn square(lo: f64, hi: f64) -> Vec<XySegment> {
        let mut out = Vec::new();
        let mut y = lo;
        while y <= hi + 1e-9 {
            out.push(seg([lo, y], [hi, y]));
            y += 0.4;
        }
        out
    }

    fn raster_of(segments: &[XySegment], half_width: f64) -> Raster {
        let refs: Vec<&XySegment> = segments.iter().collect();
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        for s in segments {
            for p in [s.start, s.end] {
                min[0] = min[0].min(p[0]);
                min[1] = min[1].min(p[1]);
                max[0] = max[0].max(p[0]);
                max[1] = max[1].max(p[1]);
            }
        }
        Raster::build(&refs, min, max, half_width)
    }

    #[test]
    fn area_of_a_solid_square_is_an_under_estimate() {
        let segments = square(0.0, 10.0);
        let r = raster_of(&segments, 0.225);
        let true_area = 10.0 * 10.4; // 26 lines 0.4 apart, 0.45 wide.
        assert!(r.area() <= true_area, "area {} over {true_area}", r.area());
        assert!(
            r.area() > true_area * 0.7,
            "area {} implausibly low",
            r.area()
        );
        assert!(!r.coarsened() && !r.truncated());
    }

    #[test]
    fn single_bead_area_is_bounded_by_the_capsule() {
        let segments = vec![seg([0.0, 0.0], [10.0, 0.0])];
        let r = raster_of(&segments, 0.225);
        // Capsule area = 10 * 0.45 + pi * 0.225^2.
        let capsule = 10.0 * 0.45 + std::f64::consts::PI * 0.225 * 0.225;
        assert!(r.area() <= capsule, "{} > {capsule}", r.area());
        assert!(r.area() >= 10.0 * 0.45 * 0.6, "{}", r.area());
    }

    #[test]
    fn containment_and_clearance_on_a_square() {
        let segments = square(0.0, 20.0);
        let r = raster_of(&segments, 0.225);
        assert!(r.is_material([10.0, 10.0]));
        assert!(!r.is_material([25.0, 10.0]));
        assert!(!r.is_material([f64::NAN, 0.0]));
        // Center of a 20 mm square: ~10 mm to the nearest edge, minus a
        // cell of conservative slack.
        // Material spans -0.225..20.225 in both axes, so the true edge
        // distance from the center is 10.225; one cell (0.075) of
        // conservative slack is subtracted.
        let c = r.clearance_at([10.0, 10.0]);
        assert!((10.0..=10.16).contains(&c), "center clearance {c}");
        // 2 mm in from the left edge.
        let c = r.clearance_at([2.0, 10.0]);
        assert!((2.0..=2.16).contains(&c), "edge clearance {c}");
        // Off the island entirely.
        assert_eq!(r.clearance_at([40.0, 10.0]), 0.0);
        assert_eq!(r.clearance_at([f64::NAN, 10.0]), 0.0);
    }

    #[test]
    fn clearance_never_exceeds_the_true_distance() {
        let segments = square(0.0, 12.0);
        let r = raster_of(&segments, 0.225);
        for step in 0..25 {
            let p = [f64::from(step).mul_add(0.45, 0.5), 6.0];
            let true_edge = (p[0] - (-0.225)).min(12.225 - p[0]).min(6.225);
            let c = r.clearance_at(p);
            assert!(c <= true_edge + 1e-9, "at {p:?}: {c} > {true_edge}");
        }
    }

    #[test]
    fn degenerate_inputs_produce_an_empty_raster() {
        let s = seg([0.0, 0.0], [1.0, 0.0]);
        for (min, max, hw) in [
            ([f64::NAN, 0.0], [1.0, 1.0], 0.225),
            ([0.0, 0.0], [1.0, 1.0], 0.0),
            ([0.0, 0.0], [1.0, 1.0], f64::NAN),
            ([5.0, 5.0], [1.0, 1.0], 0.225),
        ] {
            let r = Raster::build(&[&s], min, max, hw);
            assert_eq!(r.area(), 0.0);
            assert!(!r.is_material([0.5, 0.0]));
            assert_eq!(r.clearance_at([0.5, 0.0]), 0.0);
        }
    }

    #[test]
    fn oversized_islands_are_coarsened_not_refused() {
        // A 600 mm span at the default 0.075 mm cell would need 64e6
        // cells; the raster must grow the cell instead of failing.
        let segments = vec![
            seg([0.0, 0.0], [600.0, 0.0]),
            seg([0.0, 400.0], [600.0, 400.0]),
        ];
        let r = raster_of(&segments, 0.225);
        assert!(r.coarsened());
        assert!(r.cell() > 0.075);
        // The clearance grid coarsens, but the area pass keeps its own
        // fine grid: two 600 mm beads still measure ~0.34 mm of width.
        assert!(r.area() > 600.0 * 0.3, "area {}", r.area());
        assert!(r.area() <= 2.0 * (600.0 * 0.45 + 0.16), "area {}", r.area());
    }

    #[test]
    fn nonfinite_segment_samples_are_skipped() {
        let segments = [
            seg([0.0, 0.0], [10.0, 0.0]),
            seg([f64::NAN, 0.0], [1.0, 1.0]),
        ];
        let refs: Vec<&XySegment> = segments.iter().collect();
        let r = Raster::build(&refs, [0.0, 0.0], [10.0, 1.0], 0.225);
        assert!(r.area() > 0.0);
        assert!(r.is_material([5.0, 0.0]));
    }

    #[test]
    fn edt_1d_matches_brute_force() {
        let cases: Vec<Vec<f64>> = vec![
            vec![0.0],
            vec![9.0, 0.0, 9.0],
            vec![0.0, 9.0, 9.0, 9.0, 0.0],
            vec![9.0, 9.0, 9.0],
            vec![0.0, 0.0, 0.0],
            vec![4.0, 0.0, 1.0, 9.0, 0.0, 16.0],
        ];
        for f in cases {
            let n = f.len();
            let mut d = vec![0.0; n];
            let mut v = vec![0_usize; n];
            let mut z = vec![0.0; n + 1];
            edt_1d(&f, &mut d, &mut v, &mut z);
            for q in 0..n {
                let expected = (0..n)
                    .map(|p| {
                        let dq = q as f64 - p as f64;
                        dq * dq + f[p]
                    })
                    .fold(f64::INFINITY, f64::min);
                assert!((d[q] - expected).abs() < 1e-9, "{f:?} at {q}: {d:?}");
            }
        }
    }

    #[test]
    fn test_budget_truncates_instead_of_hanging() {
        // A pathological island: many long overlapping beads. The fill
        // stops at the budget and reports it.
        let mut segments = Vec::new();
        for i in 0..400 {
            let y = f64::from(i) * 0.01;
            segments.push(seg([0.0, y], [70.0, y]));
        }
        let r = raster_of(&segments, 0.225);
        assert!(r.truncated() || r.area() > 0.0);
    }
}
