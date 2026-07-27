//! Line matching in simulated-move space.
//!
//! After a crash the reconstruction pipeline knows *where* the machine
//! stopped (position/E intervals in Klipper-internal coordinates) but
//! not *which file line* it was executing. [`match_stop_point`] answers
//! that by filtering the [`LayerModel`]'s simulated move stream — the
//! same arc-chord-expanded, state-replayed space the machine actually
//! executed — against the evidence, and reporting every line that is
//! consistent with it.
//!
//! # Ambiguity policy
//!
//! Ambiguity degrades **granularity**, never correctness:
//!
//! * exactly one consistent line → [`MatchConfidence::UniqueLine`];
//! * several (up to [`MatchConfig::ambiguity_limit`]) → an honest
//!   [`MatchConfidence::AmbiguousWindow`] listing every line —
//!   symmetric or repeated geometry is *never* collapsed into a fake
//!   unique match by score;
//! * more than the limit but all within one layer →
//!   [`MatchConfidence::LayerOnly`];
//! * more than the limit across layers → [`MatchError::Inconclusive`];
//! * no consistent line at all → [`MatchError::NoMatch`], unless the
//!   trusted Z evidence pins a unique layer, which degrades to
//!   [`MatchConfidence::LayerOnly`] (Z is exact in the replay; XY/E
//!   evidence may simply be too coarse).
//!
//! Moves whose relevant axes are G28-unknown are excluded from
//! candidacy (they cannot be trusted geometrically) and counted in
//! [`MatchResult::skipped_unknown`].
//!
//! # Arcs
//!
//! Candidates are found per chord, but a chord candidate reports the
//! byte offset of its **source G2/G3 line** (all chords share that
//! span), because `M26` must target a file line. Multiple matching
//! chords of one arc collapse into a single candidate (the
//! best-agreeing chord), so an arc can never outvote a straight line
//! into false ambiguity.

use plr_gcode::{ArcSegmentInfo, ByteSpan};
use serde::{Deserialize, Serialize};

use crate::geom;
use crate::model::{LayerModel, MoveKind, SimMove};

/// A closed interval `[min, max]` of plausible values for one quantity.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Interval {
    /// Lower bound (inclusive).
    pub min: f64,
    /// Upper bound (inclusive).
    pub max: f64,
}

impl Interval {
    /// True when both bounds are finite and ordered.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.min.is_finite() && self.max.is_finite() && self.min <= self.max
    }

    /// Midpoint of the interval.
    #[must_use]
    pub fn midpoint(&self) -> f64 {
        (self.min + self.max) * 0.5
    }

    /// Distance between this interval and the range `[lo, hi]`
    /// (0 when they overlap).
    fn gap_to_range(&self, lo: f64, hi: f64) -> f64 {
        if hi < self.min {
            self.min - hi
        } else if lo > self.max {
            lo - self.max
        } else {
            0.0
        }
    }
}

/// Byte-offset window of the file to search.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ByteWindow {
    /// First byte offset considered (typically the last durable WAL
    /// file offset).
    pub start: u64,
    /// One past the last byte considered; `None` = to the end of the
    /// modeled window.
    pub end: Option<u64>,
}

/// Stop evidence delivered by the reconstruction pipeline.
///
/// # Field mapping (contract for `plr-reconstruct`)
///
/// Every value is in **Klipper-internal toolhead coordinates** — the
/// frame of `GcodeState::last_position`, which is byte-identical to the
/// trapq/WAL position frame. Raw WAL numbers therefore map directly;
/// no G92/M220/M221 arithmetic is needed on the evidence side, because
/// the matcher replays those commands while simulating the file.
///
/// * `x`, `y` — bounding intervals of the plausible toolhead XY at
///   power loss (WAL position ± reconstruction uncertainty);
/// * `e` — interval of the plausible internal extruder position (WAL E
///   frame; compared against simulated internal E, *not* the file's E
///   words). `None` disables the E constraint;
/// * `z_candidates` — plausible internal Z values (e.g. from the WAL
///   tail or `plr_gcode::scan_z_events` reasoning). Empty disables the
///   Z constraint;
/// * `window` — byte range to search: last durable file offset through
///   the forward-simulation horizon.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StopEvidence {
    /// Plausible internal X interval.
    pub x: Interval,
    /// Plausible internal Y interval.
    pub y: Interval,
    /// Plausible internal E interval (`None` = unconstrained).
    pub e: Option<Interval>,
    /// Plausible internal Z values (empty = unconstrained).
    pub z_candidates: Vec<f64>,
    /// Byte window to search.
    pub window: ByteWindow,
}

/// Tolerances and limits for [`match_stop_point`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MatchConfig {
    /// Maximum XY distance between a move segment and the evidence box
    /// for the move to remain a candidate, mm.
    pub xy_tolerance: f64,
    /// Maximum gap between a move's internal-E range and the evidence E
    /// interval, mm of filament.
    pub e_tolerance: f64,
    /// Maximum |simulated Z − candidate Z| for the Z constraint, mm.
    pub z_tolerance: f64,
    /// More candidate lines than this degrade the answer to layer
    /// granularity. Must be at least 1.
    pub ambiguity_limit: usize,
}

impl Default for MatchConfig {
    /// `xy_tolerance` 0.5 mm (about one extrusion width of
    /// reconstruction slack), `e_tolerance` 1.0 mm filament,
    /// `z_tolerance` 0.05 mm (well under any layer height),
    /// `ambiguity_limit` 8 lines.
    fn default() -> Self {
        Self {
            xy_tolerance: 0.5,
            e_tolerance: 1.0,
            z_tolerance: 0.05,
            ambiguity_limit: 8,
        }
    }
}

/// One file line consistent with the stop evidence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MatchCandidate {
    /// `span.start` of the source line — a line boundary, safe for
    /// `M26 S<byte>`. For arc chords this is the G2/G3 line.
    pub offset: u64,
    /// Full span of the source line.
    pub span: ByteSpan,
    /// Simulated internal position (X, Y, Z, E) at the matched point on
    /// the move. Chord-linear for arcs (exact at chord boundaries,
    /// within one chord sagitta elsewhere).
    pub position: [f64; 4],
    /// E agreement in `[0, 1]`: 1.0 when the move's internal-E range
    /// overlaps the evidence interval, decaying linearly to 0 at
    /// [`MatchConfig::e_tolerance`] gap. 1.0 when no E evidence was
    /// supplied.
    pub e_agreement: f64,
    /// XY distance between the move segment and the evidence box (0
    /// when they intersect).
    pub xy_distance: f64,
    /// Layer active at the move (`None` before the first deposition).
    pub layer: Option<u32>,
    /// Deposition classification of the matched move.
    pub kind: MoveKind,
    /// The matching chord, when the source line is an arc.
    pub arc: Option<ArcSegmentInfo>,
}

/// How precisely the stop point was located.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum MatchConfidence {
    /// Exactly one line is consistent with the evidence.
    UniqueLine {
        /// Line-boundary byte offset of that line.
        offset: u64,
    },
    /// Several lines are consistent; all of them are listed.
    AmbiguousWindow {
        /// Line-boundary byte offsets, ascending.
        offsets: Vec<u64>,
    },
    /// Only the layer could be established.
    LayerOnly {
        /// Index of that layer in the [`LayerModel`].
        layer: u32,
    },
}

/// Result of [`match_stop_point`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MatchResult {
    /// Consistent lines, ranked: E agreement descending, then XY
    /// distance ascending, then offset ascending. One entry per source
    /// line (arc chords collapsed).
    pub candidates: Vec<MatchCandidate>,
    /// Granularity of the answer.
    pub confidence: MatchConfidence,
    /// Moves excluded because a required axis was G28-unknown.
    pub skipped_unknown: usize,
}

/// Matching failures. All are defined outcomes — the matcher never
/// panics on any input.
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize, Deserialize)]
pub enum MatchError {
    /// An evidence field was non-finite or an interval was inverted.
    #[error("invalid stop evidence: field {field} is non-finite or inverted")]
    InvalidEvidence {
        /// Name of the offending field.
        field: &'static str,
    },
    /// A tolerance was non-finite/negative, or `ambiguity_limit` was 0.
    #[error("invalid matcher config: field {field}")]
    InvalidConfig {
        /// Name of the offending field.
        field: &'static str,
    },
    /// No simulated move is consistent with the evidence and Z pins no
    /// unique layer.
    #[error("no simulated move is consistent with the stop evidence")]
    NoMatch,
    /// Too many consistent lines spread across several layers — not
    /// even layer granularity can be reported honestly.
    #[error("{lines} candidate lines across layers {layers:?}; below layer granularity")]
    Inconclusive {
        /// Number of consistent lines.
        lines: usize,
        /// Distinct layers they belong to, ascending.
        layers: Vec<u32>,
    },
}

/// Validate the evidence and config, returning the offending field.
///
/// `pub(crate)` so the parallel preview builder ([`crate::preview`]) can
/// reject the same non-finite / inverted inputs `match_stop_point` does,
/// without a second, subtly-different predicate.
pub(crate) fn validate(evidence: &StopEvidence, config: &MatchConfig) -> Result<(), MatchError> {
    let tolerances = [
        ("xy_tolerance", config.xy_tolerance),
        ("e_tolerance", config.e_tolerance),
        ("z_tolerance", config.z_tolerance),
    ];
    for (field, value) in tolerances {
        if !value.is_finite() || value < 0.0 {
            return Err(MatchError::InvalidConfig { field });
        }
    }
    if config.ambiguity_limit == 0 {
        return Err(MatchError::InvalidConfig {
            field: "ambiguity_limit",
        });
    }
    if !evidence.x.is_valid() {
        return Err(MatchError::InvalidEvidence { field: "x" });
    }
    if !evidence.y.is_valid() {
        return Err(MatchError::InvalidEvidence { field: "y" });
    }
    if let Some(e) = &evidence.e {
        if !e.is_valid() {
            return Err(MatchError::InvalidEvidence { field: "e" });
        }
    }
    if evidence.z_candidates.iter().any(|z| !z.is_finite()) {
        return Err(MatchError::InvalidEvidence {
            field: "z_candidates",
        });
    }
    if let Some(end) = evidence.window.end {
        if end < evidence.window.start {
            return Err(MatchError::InvalidEvidence { field: "window" });
        }
    }
    Ok(())
}

/// True when the move's byte span overlaps the search window.
///
/// `pub(crate)` so [`crate::preview`] scopes its nudge domain to the
/// same in-window predicate the matcher uses.
pub(crate) fn in_window(mv: &SimMove, window: ByteWindow) -> bool {
    mv.span.end > window.start && window.end.is_none_or(|end| mv.span.start < end)
}

/// True when every axis the evidence constrains is position-known on
/// both endpoints of the move.
fn axes_known(mv: &SimMove, evidence: &StopEvidence) -> bool {
    let mut required = vec![0_usize, 1];
    if !evidence.z_candidates.is_empty() {
        required.push(2);
    }
    if evidence.e.is_some() {
        required.push(3);
    }
    required.into_iter().all(|axis| {
        mv.start_known.get(axis).copied().unwrap_or(false)
            && mv.end_known.get(axis).copied().unwrap_or(false)
    })
}

/// Evaluate one move against the evidence; `None` when inconsistent.
fn evaluate(mv: &SimMove, evidence: &StopEvidence, config: &MatchConfig) -> Option<MatchCandidate> {
    // Defensive: overflowed (infinite) simulated coordinates cannot be
    // matched meaningfully.
    if !mv.start.iter().chain(mv.end.iter()).all(|v| v.is_finite()) {
        return None;
    }
    // Z constraint (exact replay, tight tolerance).
    if !evidence.z_candidates.is_empty() {
        let z_lo = mv.start[2].min(mv.end[2]) - config.z_tolerance;
        let z_hi = mv.start[2].max(mv.end[2]) + config.z_tolerance;
        if !evidence
            .z_candidates
            .iter()
            .any(|z| *z >= z_lo && *z <= z_hi)
        {
            return None;
        }
    }
    // XY constraint: segment-to-box distance.
    let seg_a = [mv.start[0], mv.start[1]];
    let seg_b = [mv.end[0], mv.end[1]];
    let box_lo = [evidence.x.min, evidence.y.min];
    let box_hi = [evidence.x.max, evidence.y.max];
    // The move endpoints and box bounds are finite here, so the
    // distance is a number and the comparison is total.
    let xy_distance = geom::seg_box_distance(seg_a, seg_b, box_lo, box_hi);
    if xy_distance > config.xy_tolerance {
        return None;
    }
    // E constraint and agreement.
    let e_lo = mv.start[3].min(mv.end[3]);
    let e_hi = mv.start[3].max(mv.end[3]);
    let e_agreement = match &evidence.e {
        None => 1.0,
        Some(interval) => {
            let gap = interval.gap_to_range(e_lo, e_hi);
            if gap > config.e_tolerance {
                return None;
            }
            if gap <= 0.0 {
                1.0
            } else {
                // gap > 0 implies e_tolerance > 0 here.
                1.0 - gap / config.e_tolerance
            }
        }
    };
    // Matched point on the move: at the evidence-E position when E
    // varies along the move, else nearest to the XY box center.
    let e_delta = mv.end[3] - mv.start[3];
    let t = match &evidence.e {
        Some(interval) if e_delta.abs() > 0.0 => {
            let target = interval.midpoint().clamp(e_lo, e_hi);
            ((target - mv.start[3]) / e_delta).clamp(0.0, 1.0)
        }
        _ => {
            let center = [
                (evidence.x.min + evidence.x.max) * 0.5,
                (evidence.y.min + evidence.y.max) * 0.5,
            ];
            geom::closest_point_t(center, seg_a, seg_b)
        }
    };
    let position = [
        mv.start[0] + (mv.end[0] - mv.start[0]) * t,
        mv.start[1] + (mv.end[1] - mv.start[1]) * t,
        mv.start[2] + (mv.end[2] - mv.start[2]) * t,
        mv.start[3] + e_delta * t,
    ];
    Some(MatchCandidate {
        offset: mv.span.start,
        span: mv.span,
        position,
        e_agreement,
        xy_distance,
        layer: mv.layer,
        kind: mv.kind,
        arc: mv.arc,
    })
}

/// True when `a` ranks better than `b` (used both for per-line chord
/// collapsing and for the final ordering).
fn ranks_better(a: &MatchCandidate, b: &MatchCandidate) -> bool {
    match b.e_agreement.total_cmp(&a.e_agreement) {
        std::cmp::Ordering::Less => false,
        std::cmp::Ordering::Greater => true,
        std::cmp::Ordering::Equal => match a.xy_distance.total_cmp(&b.xy_distance) {
            std::cmp::Ordering::Less => true,
            std::cmp::Ordering::Greater => false,
            std::cmp::Ordering::Equal => a.offset < b.offset,
        },
    }
}

/// Evaluate every in-window, position-known move against the evidence,
/// returning the ranked candidate list (one entry per source line, best
/// chord kept) and the count of moves skipped for G28-unknown axes.
///
/// This is the shared evaluate path behind both consumers:
/// [`match_stop_point`] runs the confidence ladder on the output, and
/// [`crate::preview::build_preview`] seeds its representative set from it.
/// Extracted verbatim from `match_stop_point`'s former body so there is
/// exactly one evaluate loop — no second predicate that could drift.
///
/// Assumes the evidence and config were already validated (both callers
/// call [`validate`] first); it never re-validates and never fails.
pub(crate) fn collect_candidates(
    model: &LayerModel,
    evidence: &StopEvidence,
    config: &MatchConfig,
) -> (Vec<MatchCandidate>, usize) {
    let mut skipped_unknown = 0_usize;
    // One candidate per source line, best chord kept.
    let mut by_line: std::collections::BTreeMap<u64, MatchCandidate> =
        std::collections::BTreeMap::new();
    for mv in &model.moves {
        if !in_window(mv, evidence.window) {
            continue;
        }
        if !axes_known(mv, evidence) {
            skipped_unknown += 1;
            continue;
        }
        if let Some(candidate) = evaluate(mv, evidence, config) {
            match by_line.get_mut(&candidate.offset) {
                Some(existing) => {
                    if ranks_better(&candidate, existing) {
                        *existing = candidate;
                    }
                }
                None => {
                    by_line.insert(candidate.offset, candidate);
                }
            }
        }
    }
    let mut candidates: Vec<MatchCandidate> = by_line.into_values().collect();
    candidates.sort_by(|a, b| {
        if ranks_better(a, b) {
            std::cmp::Ordering::Less
        } else if ranks_better(b, a) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
    (candidates, skipped_unknown)
}

/// Match the stop evidence against the model's simulated move stream.
/// See the module docs for the ambiguity policy.
pub fn match_stop_point(
    model: &LayerModel,
    evidence: &StopEvidence,
    config: &MatchConfig,
) -> Result<MatchResult, MatchError> {
    validate(evidence, config)?;
    let (candidates, skipped_unknown) = collect_candidates(model, evidence, config);
    let confidence = match candidates.len() {
        0 => {
            // Z is exact in the replay; if it pins a unique trusted
            // layer, degrade to layer granularity instead of failing.
            let layers: Vec<u32> = model
                .layers
                .iter()
                .filter(|layer| {
                    layer.z_known
                        && layer.span.end > evidence.window.start
                        && evidence.window.end.is_none_or(|end| layer.span.start < end)
                        && evidence
                            .z_candidates
                            .iter()
                            .any(|z| (z - layer.z).abs() <= config.z_tolerance)
                })
                .map(|layer| layer.index)
                .collect();
            match layers.as_slice() {
                [layer] => MatchConfidence::LayerOnly { layer: *layer },
                _ => return Err(MatchError::NoMatch),
            }
        }
        1 => match candidates.first() {
            Some(single) => MatchConfidence::UniqueLine {
                offset: single.offset,
            },
            None => return Err(MatchError::NoMatch),
        },
        n if n <= config.ambiguity_limit => {
            let mut offsets: Vec<u64> = candidates.iter().map(|c| c.offset).collect();
            offsets.sort_unstable();
            MatchConfidence::AmbiguousWindow { offsets }
        }
        n => {
            let mut layers: Vec<u32> = Vec::new();
            let mut any_unlayered = false;
            for candidate in &candidates {
                match candidate.layer {
                    Some(layer) => {
                        if !layers.contains(&layer) {
                            layers.push(layer);
                        }
                    }
                    None => any_unlayered = true,
                }
            }
            layers.sort_unstable();
            match (layers.as_slice(), any_unlayered) {
                ([layer], false) => MatchConfidence::LayerOnly { layer: *layer },
                _ => {
                    return Err(MatchError::Inconclusive { lines: n, layers });
                }
            }
        }
    };
    Ok(MatchResult {
        candidates,
        confidence,
        skipped_unknown,
    })
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact replay equality is intentional
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

    fn whole_file() -> ByteWindow {
        ByteWindow {
            start: 0,
            end: None,
        }
    }

    fn iv(min: f64, max: f64) -> Interval {
        Interval { min, max }
    }

    const SIMPLE: &str = "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\n\
                          G1 X40 Y10 E1.0 F1800\nG1 X40 Y40 E1.0\nG1 X10 Y40 E1.0\n";

    #[test]
    fn unique_line_match() {
        let m = model_of(SIMPLE);
        let evidence = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: Some(iv(0.4, 0.6)),
            z_candidates: vec![0.2],
            window: whole_file(),
        };
        let r = match_stop_point(&m, &evidence, &MatchConfig::default()).expect("match");
        assert_eq!(r.candidates.len(), 1);
        let c = &r.candidates[0];
        assert_eq!(
            r.confidence,
            MatchConfidence::UniqueLine { offset: c.offset }
        );
        assert_eq!(c.offset, SIMPLE.find("G1 X40 Y10").unwrap() as u64);
        assert_eq!(c.e_agreement, 1.0);
        assert_eq!(c.kind, MoveKind::Extrusion);
        // Matched point interpolated at the evidence E midpoint (0.5):
        // halfway along the 30 mm extrusion.
        assert!((c.position[0] - 25.0).abs() < 1e-9);
        assert_eq!(c.position[1], 10.0);
        assert_eq!(c.position[2], 0.2);
        assert!((c.position[3] - 0.5).abs() < 1e-9);
    }

    #[test]
    fn window_excludes_lines() {
        let m = model_of(SIMPLE);
        let target = SIMPLE.find("G1 X40 Y10").unwrap() as u64;
        let evidence = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: None,
            z_candidates: vec![],
            // Window begins after the matching line ends.
            window: ByteWindow {
                start: SIMPLE.find("G1 X40 Y40").unwrap() as u64,
                end: None,
            },
        };
        let err = match_stop_point(&m, &evidence, &MatchConfig::default()).unwrap_err();
        assert_eq!(err, MatchError::NoMatch);
        // And an end bound before the line also excludes it.
        let evidence2 = StopEvidence {
            window: ByteWindow {
                start: 0,
                end: Some(target),
            },
            ..evidence
        };
        let err2 = match_stop_point(&m, &evidence2, &MatchConfig::default()).unwrap_err();
        assert_eq!(err2, MatchError::NoMatch);
    }

    #[test]
    fn e_interval_disambiguates_retraced_geometry() {
        // Same segment extruded twice (relative E).
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\n\
                    G1 X40 Y10 E0.5 F1800\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n";
        let m = model_of(text);
        let base = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: None,
            z_candidates: vec![0.2],
            window: whole_file(),
        };
        // Without E evidence: honest ambiguity (2 extrusions + 1
        // retrace travel share the geometry).
        let r = match_stop_point(&m, &base, &MatchConfig::default()).expect("match");
        let MatchConfidence::AmbiguousWindow { offsets } = &r.confidence else {
            panic!("expected ambiguity, got {:?}", r.confidence);
        };
        assert_eq!(offsets.len(), 3);
        assert!(offsets.windows(2).all(|w| w[0] < w[1]), "offsets ascending");
        // With a tight E interval mid-first-extrusion: unique.
        let tight = StopEvidence {
            e: Some(iv(0.2, 0.3)),
            ..base
        };
        let cfg = MatchConfig {
            e_tolerance: 0.05,
            ..MatchConfig::default()
        };
        let r = match_stop_point(&m, &tight, &cfg).expect("match");
        assert_eq!(
            r.confidence,
            MatchConfidence::UniqueLine {
                offset: text.find("G1 X40 Y10 E0.5").unwrap() as u64
            }
        );
    }

    #[test]
    fn arc_reports_source_line_and_collapses_chords() {
        let text = "G90\nM82\nG1 X10 Y0 Z0.4 F6000\nG3 X0 Y10 I-10 E3 F1800\n";
        let m = model_of(text);
        // Loose XY box near 45 degrees on the arc: several chords pass
        // the XY test, but they must collapse to one candidate on the
        // G3 line.
        let evidence = StopEvidence {
            x: iv(6.5, 7.6),
            y: iv(6.5, 7.6),
            e: Some(iv(1.4, 1.6)),
            z_candidates: vec![0.4],
            window: whole_file(),
        };
        let r = match_stop_point(&m, &evidence, &MatchConfig::default()).expect("match");
        assert_eq!(r.candidates.len(), 1);
        let c = &r.candidates[0];
        assert_eq!(
            r.confidence,
            MatchConfidence::UniqueLine { offset: c.offset }
        );
        assert_eq!(c.offset, text.find("G3").unwrap() as u64);
        assert!(c.arc.is_some(), "must identify the matching chord");
        // Matched position sits on the arc (radius 10 within chord
        // sagitta) at the E midpoint.
        let r_matched = (c.position[0] * c.position[0] + c.position[1] * c.position[1]).sqrt();
        assert!((r_matched - 10.0).abs() < 0.05, "got radius {r_matched}");
        assert!((c.position[3] - 1.5).abs() < 0.11);
    }

    #[test]
    fn layer_only_when_over_ambiguity_limit_in_one_layer() {
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\n\
                    G1 X40 Y10 E0.5 F1800\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
                    G1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n";
        let m = model_of(text);
        let evidence = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let cfg = MatchConfig {
            ambiguity_limit: 2,
            ..MatchConfig::default()
        };
        let r = match_stop_point(&m, &evidence, &cfg).expect("match");
        assert_eq!(r.confidence, MatchConfidence::LayerOnly { layer: 0 });
        assert!(r.candidates.len() > 2, "candidates still reported");
    }

    #[test]
    fn inconclusive_when_over_limit_across_layers() {
        // The same XY segment re-extruded on two layers.
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\n\
                    G1 X40 Y10 E0.5 F1800\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
                    G1 Z0.4 F7200\nG1 X10 Y10 F9000\n\
                    G1 X40 Y10 E0.5 F1800\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n";
        let m = model_of(text);
        let evidence = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let cfg = MatchConfig {
            ambiguity_limit: 2,
            ..MatchConfig::default()
        };
        let err = match_stop_point(&m, &evidence, &cfg).unwrap_err();
        let MatchError::Inconclusive { lines, layers } = err else {
            panic!("expected Inconclusive, got {err:?}");
        };
        assert!(lines > 2);
        assert_eq!(layers, vec![0, 1]);
    }

    #[test]
    fn z_pins_layer_when_geometry_matches_nothing() {
        let m = model_of(SIMPLE);
        let evidence = StopEvidence {
            // Far away from any path.
            x: iv(500.0, 501.0),
            y: iv(500.0, 501.0),
            e: None,
            z_candidates: vec![0.2],
            window: whole_file(),
        };
        let r = match_stop_point(&m, &evidence, &MatchConfig::default()).expect("layer fallback");
        assert_eq!(r.confidence, MatchConfidence::LayerOnly { layer: 0 });
        assert!(r.candidates.is_empty());
        // Without Z evidence the same box is a hard NoMatch.
        let no_z = StopEvidence {
            z_candidates: vec![],
            ..evidence
        };
        assert_eq!(
            match_stop_point(&m, &no_z, &MatchConfig::default()).unwrap_err(),
            MatchError::NoMatch
        );
    }

    #[test]
    fn unknown_axes_are_skipped_not_matched() {
        // After G28 Z the z-only move has unknown Z; with Z evidence it
        // must be skipped, leaving the known extrusion as unique.
        let text = "G90\nM82\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\nG1 X30 Y10 E1.0 F1800\n\
                    G28 Z\nG91\nG1 Z2 F7200\nG90\nG1 Z0.4 F7200\n";
        let m = model_of(text);
        let with_z = StopEvidence {
            x: iv(29.5, 30.5),
            y: iv(9.5, 10.5),
            e: Some(iv(0.9, 1.1)),
            z_candidates: vec![0.2],
            window: whole_file(),
        };
        let r = match_stop_point(&m, &with_z, &MatchConfig::default()).expect("match");
        assert!(r.skipped_unknown > 0, "unknown-Z move must be skipped");
        assert_eq!(
            r.confidence,
            MatchConfidence::UniqueLine {
                offset: text.find("G1 X30 Y10").unwrap() as u64
            }
        );
        // Without the Z constraint the unknown-Z hop is eligible again
        // (XY is still known) and the result is honestly ambiguous.
        let without_z = StopEvidence {
            z_candidates: vec![],
            ..with_z
        };
        let r = match_stop_point(&m, &without_z, &MatchConfig::default()).expect("match");
        assert!(matches!(
            r.confidence,
            MatchConfidence::AmbiguousWindow { .. }
        ));
    }

    #[test]
    fn non_finite_evidence_is_a_typed_error() {
        let m = model_of(SIMPLE);
        let good = StopEvidence {
            x: iv(0.0, 1.0),
            y: iv(0.0, 1.0),
            e: Some(iv(0.0, 1.0)),
            z_candidates: vec![0.2],
            window: whole_file(),
        };
        let cases: Vec<(StopEvidence, &str)> = vec![
            (
                StopEvidence {
                    x: iv(f64::NAN, 1.0),
                    ..good.clone()
                },
                "x",
            ),
            (
                StopEvidence {
                    y: iv(2.0, 1.0),
                    ..good.clone()
                },
                "y",
            ),
            (
                StopEvidence {
                    e: Some(iv(0.0, f64::INFINITY)),
                    ..good.clone()
                },
                "e",
            ),
            (
                StopEvidence {
                    z_candidates: vec![0.2, f64::NAN],
                    ..good.clone()
                },
                "z_candidates",
            ),
            (
                StopEvidence {
                    window: ByteWindow {
                        start: 10,
                        end: Some(5),
                    },
                    ..good.clone()
                },
                "window",
            ),
        ];
        for (evidence, field) in cases {
            let err = match_stop_point(&m, &evidence, &MatchConfig::default()).unwrap_err();
            assert_eq!(err, MatchError::InvalidEvidence { field }, "{field}");
        }
    }

    #[test]
    fn invalid_config_is_a_typed_error() {
        let m = model_of(SIMPLE);
        let evidence = StopEvidence {
            x: iv(0.0, 1.0),
            y: iv(0.0, 1.0),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let bad_tol = MatchConfig {
            xy_tolerance: f64::NAN,
            ..MatchConfig::default()
        };
        assert_eq!(
            match_stop_point(&m, &evidence, &bad_tol).unwrap_err(),
            MatchError::InvalidConfig {
                field: "xy_tolerance"
            }
        );
        let bad_limit = MatchConfig {
            ambiguity_limit: 0,
            ..MatchConfig::default()
        };
        assert_eq!(
            match_stop_point(&m, &evidence, &bad_limit).unwrap_err(),
            MatchError::InvalidConfig {
                field: "ambiguity_limit"
            }
        );
        let neg = MatchConfig {
            e_tolerance: -1.0,
            ..MatchConfig::default()
        };
        assert_eq!(
            match_stop_point(&m, &evidence, &neg).unwrap_err(),
            MatchError::InvalidConfig {
                field: "e_tolerance"
            }
        );
        let bad_z = MatchConfig {
            z_tolerance: f64::INFINITY,
            ..MatchConfig::default()
        };
        assert_eq!(
            match_stop_point(&m, &evidence, &bad_z).unwrap_err(),
            MatchError::InvalidConfig {
                field: "z_tolerance"
            }
        );
    }

    #[test]
    fn matcher_survives_mid_file_state_changes() {
        // G92/M220/M221 between the target and the window start must
        // not break internal-frame E matching.
        let text = "G90\nM82\nG92 E0\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\n\
                    G1 X40 Y10 E2.0 F1800\nM220 S150\nM221 S80\nG92 E0\n\
                    G1 X40 Y40 E2.0 F1800\n";
        let m = model_of(text);
        // Internal E on the second extrusion runs 2.0 -> 3.6 (words
        // scaled by the M221 factor 0.8 on top of the G92 rebase).
        let evidence = StopEvidence {
            x: iv(39.9, 40.1),
            y: iv(24.0, 26.0),
            e: Some(iv(2.7, 2.9)),
            z_candidates: vec![0.2],
            window: whole_file(),
        };
        let cfg = MatchConfig {
            e_tolerance: 0.1,
            ..MatchConfig::default()
        };
        let r = match_stop_point(&m, &evidence, &cfg).expect("match");
        assert_eq!(
            r.confidence,
            MatchConfidence::UniqueLine {
                offset: text.find("G1 X40 Y40").unwrap() as u64
            }
        );
        let c = &r.candidates[0];
        // t = (2.8 - 2.0) / 1.6 = 0.5 -> halfway up the segment.
        assert!((c.position[1] - 25.0).abs() < 1e-9);
    }

    #[test]
    fn e_agreement_decays_with_gap() {
        let m = model_of(SIMPLE);
        // Evidence E just past the first extrusion's range [0, 1].
        let evidence = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: Some(iv(1.5, 1.6)),
            z_candidates: vec![],
            window: whole_file(),
        };
        let r = match_stop_point(&m, &evidence, &MatchConfig::default()).expect("match");
        let c = &r.candidates[0];
        assert!((c.e_agreement - 0.5).abs() < 1e-9, "gap 0.5 of tol 1.0");
        // Matched point clamps to the segment end nearest the evidence.
        assert_eq!(c.position[3], 1.0);
    }

    #[test]
    fn empty_model_never_matches() {
        let m = model_of("");
        let evidence = StopEvidence {
            x: iv(0.0, 1.0),
            y: iv(0.0, 1.0),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        assert_eq!(
            match_stop_point(&m, &evidence, &MatchConfig::default()).unwrap_err(),
            MatchError::NoMatch
        );
    }

    #[test]
    fn interval_helpers() {
        assert!(iv(1.0, 1.0).is_valid());
        assert!(!iv(2.0, 1.0).is_valid());
        assert!(!iv(f64::NEG_INFINITY, 0.0).is_valid());
        assert_eq!(iv(1.0, 3.0).midpoint(), 2.0);
        assert_eq!(iv(5.0, 6.0).gap_to_range(0.0, 4.0), 1.0);
        assert_eq!(iv(5.0, 6.0).gap_to_range(7.0, 9.0), 1.0);
        assert_eq!(iv(5.0, 6.0).gap_to_range(0.0, 5.0), 0.0);
    }

    #[test]
    fn result_serializes() {
        let m = model_of(SIMPLE);
        let evidence = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: Some(iv(0.4, 0.6)),
            z_candidates: vec![0.2],
            window: whole_file(),
        };
        let r = match_stop_point(&m, &evidence, &MatchConfig::default()).expect("match");
        let json = serde_json::to_string(&r).expect("serialize");
        let back: MatchResult = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(r, back);
        let err_json = serde_json::to_string(&MatchError::NoMatch).expect("serialize error");
        assert!(err_json.contains("NoMatch"));
    }

    // NOTE on the `collect_candidates` extraction's faithfulness: there is
    // deliberately no "collect_candidates == match_stop_point's candidates"
    // test. Since `match_stop_point` now *calls* `collect_candidates` and
    // exposes its result verbatim (`result.candidates`/`skipped_unknown`),
    // such a test compares f(x) to itself — a tautology that survives real
    // mutations (e.g. dropping the `skipped_unknown` increment breaks both
    // sides equally). Faithfulness rests instead on two honest witnesses: the
    // reviewed byte-for-byte diff showing the loop body was moved unchanged
    // and the ladder left intact, and the full pre-existing matcher suite
    // above running green unchanged (its ordering and skipped-count tests —
    // e.g. `e_interval_disambiguates_retraced_geometry` — bite the same
    // mutations directly).
}
