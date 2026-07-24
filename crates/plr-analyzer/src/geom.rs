//! Small 2D geometry kernel shared by the line matcher and the
//! contact-zone selector.
//!
//! All functions are total: any `f64` input (including NaN and
//! infinities) produces a defined `f64`/`bool` result and never panics.
//! NaN inputs propagate into NaN distances (or `false` predicates),
//! which every caller treats as "no match" — comparisons against a
//! tolerance are always of the form `d <= tol`, false for NaN.

/// Euclidean distance between two points.
pub(crate) fn point_distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    (dx * dx + dy * dy).sqrt()
}

/// Parameter `t` in `[0, 1]` of the point on segment `a..b` closest to
/// `p`. A degenerate (zero-length) segment yields `t = 0.5`.
pub(crate) fn closest_point_t(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let len2 = dx * dx + dy * dy;
    if len2 > 0.0 {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.5
    }
}

/// Linear interpolation along segment `a..b`.
pub(crate) fn lerp2(a: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

/// Distance from point `p` to segment `a..b`.
pub(crate) fn point_seg_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let t = closest_point_t(p, a, b);
    point_distance(p, lerp2(a, b, t))
}

/// True when `p` lies inside (or on the boundary of) the axis-aligned
/// box `lo..hi`. NaN anywhere yields `false`.
fn point_in_box(p: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> bool {
    p[0] >= lo[0] && p[0] <= hi[0] && p[1] >= lo[1] && p[1] <= hi[1]
}

/// Distance from point `p` to the axis-aligned box `lo..hi` (0 inside).
fn point_box_distance(p: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> f64 {
    let dx = (lo[0] - p[0]).max(0.0).max(p[0] - hi[0]);
    let dy = (lo[1] - p[1]).max(0.0).max(p[1] - hi[1]);
    (dx * dx + dy * dy).sqrt()
}

/// Signed area orientation of `c` relative to the directed line `a..b`.
fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Proper (strictly crossing) intersection test for segments `p1..p2`
/// and `q1..q2`. Degenerate touch/collinear cases return `false`; the
/// caller compensates with corner/endpoint distance fallbacks, which
/// report ~0 for exactly those cases, so `seg_box_distance` stays
/// correct.
fn proper_intersect(p1: [f64; 2], p2: [f64; 2], q1: [f64; 2], q2: [f64; 2]) -> bool {
    let p_first = orient(q1, q2, p1);
    let p_second = orient(q1, q2, p2);
    let q_first = orient(p1, p2, q1);
    let q_second = orient(p1, p2, q2);
    ((p_first > 0.0 && p_second < 0.0) || (p_first < 0.0 && p_second > 0.0))
        && ((q_first > 0.0 && q_second < 0.0) || (q_first < 0.0 && q_second > 0.0))
}

/// Minimum distance between segment `a..b` and the axis-aligned box
/// `lo..hi`. Returns 0 when they touch or overlap.
///
/// Exactness: proper crossings and containment are exact; touch and
/// collinear-overlap cases are resolved through the corner/endpoint
/// distance minimum, which is 0 for those configurations.
pub(crate) fn seg_box_distance(a: [f64; 2], b: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> f64 {
    // `f64::max` ignores NaN, so a NaN coordinate could otherwise be
    // silently treated as "inside the box"; reject it up front.
    let coords = [a[0], a[1], b[0], b[1], lo[0], lo[1], hi[0], hi[1]];
    if coords.iter().any(|c| c.is_nan()) {
        return f64::NAN;
    }
    if point_in_box(a, lo, hi) || point_in_box(b, lo, hi) {
        return 0.0;
    }
    let corners = [
        [lo[0], lo[1]],
        [hi[0], lo[1]],
        [hi[0], hi[1]],
        [lo[0], hi[1]],
    ];
    let edges = [
        (corners[0], corners[1]),
        (corners[1], corners[2]),
        (corners[2], corners[3]),
        (corners[3], corners[0]),
    ];
    for (edge_a, edge_b) in edges {
        if proper_intersect(a, b, edge_a, edge_b) {
            return 0.0;
        }
    }
    let mut best = point_box_distance(a, lo, hi).min(point_box_distance(b, lo, hi));
    for corner in corners {
        best = best.min(point_seg_distance(corner, a, b));
    }
    best
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact-zero geometry results are intentional
mod tests {
    use super::*;

    #[test]
    fn point_seg_basics() {
        // Perpendicular foot inside the segment.
        assert_eq!(point_seg_distance([5.0, 3.0], [0.0, 0.0], [10.0, 0.0]), 3.0);
        // Beyond an endpoint: distance to the endpoint.
        assert_eq!(
            point_seg_distance([13.0, 4.0], [0.0, 0.0], [10.0, 0.0]),
            5.0
        );
        // Degenerate segment: distance to the point.
        assert_eq!(point_seg_distance([3.0, 4.0], [0.0, 0.0], [0.0, 0.0]), 5.0);
        // On the segment.
        assert_eq!(point_seg_distance([4.0, 0.0], [0.0, 0.0], [10.0, 0.0]), 0.0);
    }

    #[test]
    fn closest_t_degenerate_is_midpoint() {
        assert_eq!(closest_point_t([1.0, 1.0], [2.0, 2.0], [2.0, 2.0]), 0.5);
        assert_eq!(closest_point_t([5.0, 9.0], [0.0, 0.0], [10.0, 0.0]), 0.5);
        assert_eq!(closest_point_t([-5.0, 0.0], [0.0, 0.0], [10.0, 0.0]), 0.0);
        assert_eq!(closest_point_t([15.0, 0.0], [0.0, 0.0], [10.0, 0.0]), 1.0);
    }

    #[test]
    fn seg_box_endpoint_inside() {
        let d = seg_box_distance([1.0, 1.0], [9.0, 9.0], [0.0, 0.0], [2.0, 2.0]);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn seg_box_crossing_through() {
        // Both endpoints outside, segment passes through the box.
        let d = seg_box_distance([-5.0, 1.0], [5.0, 1.0], [0.0, 0.0], [2.0, 2.0]);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn seg_box_through_corner_exactly() {
        // Passes exactly through corner (2,2): caught by the corner
        // fallback, not the proper-intersection test.
        let d = seg_box_distance([1.0, 3.0], [3.0, 1.0], [0.0, 0.0], [2.0, 2.0]);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn seg_box_collinear_with_edge() {
        let d = seg_box_distance([-1.0, 2.0], [5.0, 2.0], [0.0, 0.0], [2.0, 2.0]);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn seg_box_disjoint_distances() {
        // Closest feature: box corner (2,2) to the segment.
        let d = seg_box_distance([0.0, 6.0], [6.0, 0.0], [0.0, 0.0], [2.0, 2.0]);
        assert!((d - std::f64::consts::SQRT_2).abs() < 1e-12, "got {d}");
        // Closest feature: segment endpoint to the box.
        let d = seg_box_distance([5.0, 1.0], [9.0, 1.0], [0.0, 0.0], [2.0, 2.0]);
        assert_eq!(d, 3.0);
    }

    #[test]
    fn nan_inputs_never_match() {
        let d = seg_box_distance([f64::NAN, 0.0], [1.0, f64::NAN], [0.0, 0.0], [10.0, 10.0]);
        assert!(d.is_nan());
        // A NaN distance fails `d <= tol` checks (incomparable).
        assert!(d.partial_cmp(&0.5).is_none());
        assert!(point_seg_distance([f64::NAN; 2], [0.0; 2], [1.0; 2]).is_nan());
    }
}
