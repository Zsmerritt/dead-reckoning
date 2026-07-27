//! Layer model: stream a g-code byte window into per-layer deposition
//! geometry, in simulated-move space.
//!
//! [`build_layer_model`] replays the bytes through
//! [`plr_gcode::GcodeState`] (the same state machine as
//! `plr_gcode::simulate`, without the timing model this crate does not
//! need), so every recorded move — including each chord of a G2/G3 arc
//! decomposition — carries Klipper-internal coordinates and the byte
//! span of its source line.
//!
//! # Layer semantics
//!
//! Layer boundaries are **geometric**: a new layer starts when a
//! depositing move (positive E displacement) lands on a Z that differs
//! from the current layer's Z by more than [`ModelConfig::z_epsilon`].
//! Slicer annotations never move a boundary — `;LAYER_CHANGE` may be
//! absent (or lie) while the geometry cannot — but they **classify**:
//! `;TYPE:` names label the extrusion paths, and the most recent `;Z:`
//! annotation is attached to the next layer as advisory metadata.
//! Z-hops and travels never open a layer (they deposit nothing);
//! vase-mode spiral ramps open a layer each time the accumulated ramp
//! exceeds `z_epsilon`, and are additionally counted per layer as
//! [`Layer::spiral_moves`] so consumers can detect spiral printing.
//!
//! A layer's [`Layer::span`] covers its first through last *depositing*
//! source line (leading travel and annotation lines are excluded —
//! deposition is what defines the layer geometrically). Non-depositing
//! moves are still recorded in [`LayerModel::moves`] and tagged with the
//! layer that was active when they executed.
//!
//! # Totality
//!
//! Building never panics and never fails: on arbitrary bytes the parser
//! is total, and the first line the state machine rejects stops the
//! model with [`ModelStop::LineError`] (everything before it is kept).

use plr_gcode::{Annotation, ArcSegmentInfo, ByteSpan, GcodeState, LineIter, StateError};
use serde::{Deserialize, Serialize};

/// Coarse classification of a slicer `;TYPE:` feature name, ordered by
/// probe preference.
///
/// Only the first three classes are probe-eligible (see
/// [`FeatureClass::probe_rank`]); everything else is either
/// surface-visible (a probe scar would show), mechanically unsuitable
/// (bridges, gap fill, support) or not part of the printed part at all
/// (skirt, wipe tower).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FeatureClass {
    /// Sparse/internal infill — best probe target (internal, tolerant).
    InternalInfill,
    /// Solid (but not top-visible) infill.
    SolidInfill,
    /// Internal perimeter / inner wall.
    InnerWall,
    /// External perimeter / outer wall — never probed.
    OuterWall,
    /// Visible surfaces: top/bottom surface, skin, ironing.
    Surface,
    /// Bridges and overhangs — unsupported plastic, never probed.
    Bridge,
    /// Gap fill — thin features, mechanically unsuitable.
    GapFill,
    /// Skirt/brim — on the bed, not on the part.
    SkirtBrim,
    /// Support structures.
    Support,
    /// Anything unrecognized (custom g-code, wipe/prime towers, ...).
    Other,
}

impl FeatureClass {
    /// Probe-preference rank: `Some(0)` best. `None` means the class
    /// must never be probed.
    #[must_use]
    pub fn probe_rank(self) -> Option<u8> {
        match self {
            Self::InternalInfill => Some(0),
            Self::SolidInfill => Some(1),
            Self::InnerWall => Some(2),
            Self::OuterWall
            | Self::Surface
            | Self::Bridge
            | Self::GapFill
            | Self::SkirtBrim
            | Self::Support
            | Self::Other => None,
        }
    }

    /// True when this class may host a probe point.
    #[must_use]
    pub fn probe_eligible(self) -> bool {
        self.probe_rank().is_some()
    }
}

/// Classify a slicer `;TYPE:` feature name (`PrusaSlicer`,
/// `OrcaSlicer` / `BambuStudio`, and Cura vocabularies).
///
/// Matching is case-insensitive and substring-based, with visibility
/// and risk checked before the broader infill/wall buckets so that
/// e.g. `Top solid infill` lands in [`FeatureClass::Surface`], not
/// [`FeatureClass::SolidInfill`]. Unknown names map to
/// [`FeatureClass::Other`] — v1 never geometrically infers a class.
#[must_use]
pub fn classify_feature_type(name: &str) -> FeatureClass {
    let n = name.trim().to_ascii_lowercase();
    if n.contains("bridge") || n.contains("overhang") {
        return FeatureClass::Bridge;
    }
    if n.contains("top")
        || n.contains("bottom")
        || n.contains("ironing")
        || n.contains("skin")
        || n.contains("surface")
    {
        return FeatureClass::Surface;
    }
    if n.contains("gap") {
        return FeatureClass::GapFill;
    }
    if n.contains("support") {
        return FeatureClass::Support;
    }
    if n.contains("skirt") || n.contains("brim") {
        return FeatureClass::SkirtBrim;
    }
    if n.contains("wipe tower") || n.contains("prime tower") {
        return FeatureClass::Other;
    }
    if n.contains("external perimeter") || n.contains("outer wall") || n.contains("wall-outer") {
        return FeatureClass::OuterWall;
    }
    if n.contains("solid infill") {
        return FeatureClass::SolidInfill;
    }
    if n.contains("infill") || n.contains("sparse") || n == "fill" {
        return FeatureClass::InternalInfill;
    }
    if n.contains("perimeter") || n.contains("inner wall") || n.contains("wall-inner") {
        return FeatureClass::InnerWall;
    }
    FeatureClass::Other
}

/// What a recorded move does, from the deposition point of view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MoveKind {
    /// XYZ motion with positive E displacement — deposits plastic.
    Extrusion,
    /// XYZ motion with zero or negative E displacement (travel, wipe).
    Travel,
    /// No XYZ motion, E only (retract or unretract/prime).
    ExtrudeOnly,
}

/// One simulated move, in execution order, with Klipper-internal
/// coordinates (the frame `GcodeState::last_position` lives in — the
/// same frame trapq/WAL positions use).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimMove {
    /// Position before the move (X, Y, Z, E).
    pub start: [f64; 4],
    /// Position after the move, toolhead-snap corrected (extrude-only
    /// moves never move XYZ — `PlannedMove::kinematic_end`).
    pub end: [f64; 4],
    /// Byte span of the source line. For arc chords this is the span of
    /// the G2/G3 line, shared by every chord.
    pub span: ByteSpan,
    /// Deposition classification.
    pub kind: MoveKind,
    /// Layer active when the move executed (`None` before the first
    /// deposition of the file/window).
    pub layer: Option<u32>,
    /// `exclude_object` object active when the move executed, as
    /// Klipper stores it (upper-cased), or `None` outside any
    /// `EXCLUDE_OBJECT_START`/`EXCLUDE_OBJECT_END` bracket — which is
    /// also what a file with no object annotations at all yields.
    ///
    /// Deposition attributed to no object (skirt, brim, prime line,
    /// wipe tower) must be treated as **work that cannot be cancelled**:
    /// `None` means "not attributable", never "excluded".
    ///
    /// `#[serde(default)]` so a serialized model written before this
    /// field existed still decodes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub object: Option<String>,
    /// Set when the move is one chord of an arc decomposition.
    pub arc: Option<ArcSegmentInfo>,
    /// Per-axis reliability of `start` (false after G28 until an
    /// absolute move re-establishes the axis).
    pub start_known: [bool; 4],
    /// Per-axis reliability of `end`.
    pub end_known: [bool; 4],
}

impl SimMove {
    /// E displacement of the move.
    #[must_use]
    pub fn e_delta(&self) -> f64 {
        self.end[3] - self.start[3]
    }
}

/// One XY extrusion segment of a typed path (chord geometry only —
/// extrusion widths are unknown at this level).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct XySegment {
    /// XY start point.
    pub start: [f64; 2],
    /// XY end point.
    pub end: [f64; 2],
    /// Deposition Z (internal coordinates) at the segment's end.
    pub z: f64,
    /// Internal E before the segment.
    pub e_start: f64,
    /// Internal E after the segment.
    pub e_end: f64,
    /// Byte span of the source line (the G2/G3 line for arc chords).
    pub span: ByteSpan,
    /// Set when the segment is one chord of an arc decomposition.
    pub arc: Option<ArcSegmentInfo>,
}

impl XySegment {
    /// XY length of the segment.
    #[must_use]
    pub fn length(&self) -> f64 {
        crate::geom::point_distance(self.start, self.end)
    }

    /// Point at parameter `t` (0 = start, 1 = end) along the segment.
    #[must_use]
    pub fn point_at(&self, t: f64) -> [f64; 2] {
        crate::geom::lerp2(self.start, self.end, t)
    }
}

/// A contiguous run of extrusion segments under one `;TYPE:` block.
///
/// `type_name` is `None` for deposition that occurred before any
/// `;TYPE:` annotation — such paths are never probe-eligible (v1
/// refuses to classify geometrically) but still count as deposition
/// coverage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TypedPath {
    /// Classification of `type_name` ([`FeatureClass::Other`] when
    /// unannotated or unrecognized).
    pub class: FeatureClass,
    /// The verbatim `;TYPE:` name, `None` when deposition happened
    /// without an active annotation.
    pub type_name: Option<String>,
    /// Extrusion segments in execution order.
    pub segments: Vec<XySegment>,
}

/// One geometric layer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Layer {
    /// 0-based geometric layer ordinal within the modeled window.
    pub index: u32,
    /// Deposition Z of the layer's first extrusion (internal frame).
    pub z: f64,
    /// False when `z` derives from a G28-unknown position; such a layer
    /// must not be trusted for Z decisions.
    pub z_known: bool,
    /// Byte span from the first through the last depositing source line
    /// of the layer.
    pub span: ByteSpan,
    /// The most recent `;Z:` annotation seen before the layer opened
    /// (advisory only; geometric `z` wins).
    pub annotation_z: Option<f64>,
    /// Typed extrusion paths in execution order (one entry per
    /// contiguous `;TYPE:` block; a type recurring later in the layer
    /// opens a new entry).
    pub paths: Vec<TypedPath>,
    /// Number of extrusion moves in the layer (always at least 1).
    pub extrusion_moves: u32,
    /// Extrusion moves that also changed Z (spiral / vase / helical).
    pub spiral_moves: u32,
}

/// Why model building stopped consuming input.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ModelStop {
    /// All input consumed.
    EndOfInput,
    /// A line failed to apply; the model covers everything before it.
    LineError {
        /// Byte offset of the offending line.
        offset: u64,
        /// The state-machine error.
        error: StateError,
    },
}

/// Configuration for [`build_layer_model`].
///
/// Deliberately not `Copy`: configs are passed by reference throughout
/// the crate, and future fields must not silently change the calling
/// convention.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelConfig {
    /// Deposition-Z change that opens a new layer, mm. Non-finite
    /// values behave as "never" (everything lands in one layer) because
    /// the comparison `|dz| > z_epsilon` is false for NaN/inf.
    pub z_epsilon: f64,
}

impl Default for ModelConfig {
    /// `z_epsilon` = 0.05 mm: above float/rounding noise, below half of
    /// any practical layer height (min ~0.04 is rare; typical 0.1-0.3).
    fn default() -> Self {
        Self { z_epsilon: 0.05 }
    }
}

/// The per-layer deposition model of a g-code byte window.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LayerModel {
    /// Geometric layers in deposition order.
    pub layers: Vec<Layer>,
    /// Every simulated move (travel, extrusion, retract) in execution
    /// order — the matcher's search space.
    pub moves: Vec<SimMove>,
    /// True when at least one `;TYPE:` annotation was seen.
    pub annotated: bool,
    /// Why building stopped.
    pub stop: ModelStop,
    /// Lines successfully applied.
    pub lines_consumed: usize,
}

/// Which geometric layer(s) a byte window can contain — the capped
/// offset window mapped through the layer spans, independently of any
/// XY/E/Z evidence.
///
/// This is the answer to "which layer(s) is the stop in?" on geometry the
/// matcher cannot separate: on a prismatic part (e.g. a benchy smokestack)
/// consecutive layers are byte-distinct but XY-identical, so the offset
/// window — not the XY evidence — is what pins the layer. Reporting only;
/// it never feeds the matcher's ladder.
///
/// # Attribution rule
///
/// A layer is "active" from its first depositing line until the *next*
/// layer's first depositing line — a total partition of the modeled bytes
/// by [`Layer::span`] start (the non-depositing gap between two layers'
/// deposition — the Z-lift, travel and `;LAYER_CHANGE` — is attributed to
/// the layer still in progress, the one whose deposition most recently
/// began). [`Self::layers`] lists every partition cell the window touches.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WindowLayers {
    /// Geometric layer ordinals the window can contain, ascending; each
    /// indexes [`LayerModel::layers`]. Empty when the model has no layers,
    /// or when the window lies wholly before the first deposition (then
    /// [`Self::before_first`] is set).
    pub layers: Vec<u32>,
    /// The window reaches into the pre-first-deposition preamble (leading
    /// travel / heat-up / skirt setup before any layer opened). A stop
    /// there is "before layer 0"; nothing has been deposited yet.
    pub before_first: bool,
}

impl WindowLayers {
    /// True when the window can be in exactly one layer and nowhere else
    /// (no boundary straddle, no preamble) — the unambiguous case.
    #[must_use]
    pub fn is_single_layer(&self) -> bool {
        self.layers.len() == 1 && !self.before_first
    }

    /// Cross-check the geometric attribution against a slicer
    /// `current_layer` mark (Part 3). The mark is an **upper bound** on the
    /// physically-printing layer — the slicer sets it at the layer-change
    /// line's parse time, which leads execution — so the attribution is
    /// consistent iff at least one attributed layer is `<= mark`, or the
    /// window is in the preamble (before layer 0, trivially `<=` anything).
    /// A geometric attribution lying entirely *above* the mark is the one
    /// physically impossible case and returns `false` (evidence to flag,
    /// never a reason to override geometry).
    #[must_use]
    pub fn mark_is_consistent(&self, current_layer: u32) -> bool {
        self.before_first || self.layers.iter().any(|&l| l <= current_layer)
    }

    /// A one-line human summary for the scan/pipeline narration, e.g.
    /// "layer 12", "layers 12–15", "before layer 0 (preamble)", or
    /// "before layer 0, then layers 0–1". Never panics.
    #[must_use]
    pub fn describe(&self) -> String {
        let span = match self.layers.as_slice() {
            [] => None,
            [only] => Some(format!("layer {only}")),
            [first, .., last] => Some(format!("layers {first}\u{2013}{last}")),
        };
        match (self.before_first, span) {
            (false, Some(s)) => s,
            (true, None) => "before layer 0 (preamble)".to_owned(),
            (true, Some(s)) => format!("before layer 0, then {s}"),
            (false, None) => "no modeled layer".to_owned(),
        }
    }
}

impl LayerModel {
    /// Look up a layer by index.
    #[must_use]
    pub fn layer(&self, index: u32) -> Option<&Layer> {
        self.layers.get(index as usize)
    }

    /// Map a byte window `[start, end)` onto the geometric layer(s) it can
    /// contain (see [`WindowLayers`]). `end = None` means "to the end of
    /// the modeled window". The window is intersected against a total
    /// partition of the bytes by [`Layer::span`] start: layer `i` owns
    /// `[layers[i].span.start, layers[i+1].span.start)`, the last layer
    /// owns everything from its start onward, and everything before the
    /// first layer's deposition is the preamble.
    ///
    /// Reporting only — never consulted by [`crate::match_stop_point`], so
    /// the matcher's ladder is unaffected. Total: never panics, and an
    /// inverted or empty window simply yields no layers.
    #[must_use]
    pub fn layers_in_window(&self, start: u64, end: Option<u64>) -> WindowLayers {
        // A window is [start, end_excl). Treat `end = None` and any end
        // below start as "just the single offset `start`" so a degenerate
        // window still attributes the offset it names rather than nothing.
        let end_excl = end
            .filter(|&e| e > start)
            .unwrap_or(start.saturating_add(1));
        let mut layers = Vec::new();
        let mut before_first = false;
        if self.layers.is_empty() {
            return WindowLayers {
                layers,
                before_first: true,
            };
        }
        // Preamble cell: [0, layers[0].span.start).
        if start < self.layers[0].span.start {
            before_first = true;
        }
        for (i, layer) in self.layers.iter().enumerate() {
            let cell_start = layer.span.start;
            // The cell ends where the next layer's deposition begins; the
            // last layer's cell runs to the end of the modeled window.
            let cell_end = self
                .layers
                .get(i + 1)
                .map_or(u64::MAX, |next| next.span.start);
            // Half-open overlap of [start, end_excl) with [cell_start, cell_end).
            if start < cell_end && cell_start < end_excl {
                // `index` is authoritative (it may differ from `i` only if
                // a caller mutated `layers`; use the stored ordinal).
                layers.push(layer.index);
            }
        }
        WindowLayers {
            layers,
            before_first,
        }
    }

    /// Fraction of extrusion moves that also changed Z (0 when the
    /// model contains no extrusion). Near 1.0 indicates vase-mode /
    /// spiral printing.
    #[must_use]
    pub fn spiral_fraction(&self) -> f64 {
        let (mut spiral, mut total) = (0_u32, 0_u32);
        for layer in &self.layers {
            spiral = spiral.saturating_add(layer.spiral_moves);
            total = total.saturating_add(layer.extrusion_moves);
        }
        if total == 0 {
            0.0
        } else {
            f64::from(spiral) / f64::from(total)
        }
    }

    /// First depositing move whose source line starts at or after
    /// `offset` — the resume-file selection rule ("first safe
    /// deposition line at or after the stop point"). The returned
    /// move's `span.start` is a line boundary, safe for `M26`.
    #[must_use]
    pub fn first_deposition_at_or_after(&self, offset: u64) -> Option<&SimMove> {
        self.moves
            .iter()
            .find(|m| m.kind == MoveKind::Extrusion && m.span.start >= offset)
    }
}

/// Build a [`LayerModel`] from `data` (a byte window of the print file
/// beginning at stream offset `base_offset`), replaying from `state` —
/// pass a WAL-reconstructed [`GcodeState`] to model mid-file windows,
/// or [`GcodeState::new`] for a whole file.
///
/// Total: never panics and never fails; see the module docs.
#[must_use]
pub fn build_layer_model(
    state: GcodeState,
    data: &[u8],
    base_offset: u64,
    config: &ModelConfig,
) -> LayerModel {
    let mut st = state;
    let mut model = LayerModel {
        layers: Vec::new(),
        moves: Vec::new(),
        annotated: false,
        stop: ModelStop::EndOfInput,
        lines_consumed: 0,
    };
    let mut current_type: Option<String> = None;
    let mut current_object: Option<String> = None;
    let mut pending_annotation_z: Option<f64> = None;
    for line in LineIter::new(data, base_offset) {
        match line.comment().and_then(plr_gcode::Comment::annotation) {
            Some(Annotation::FeatureType(name)) => {
                model.annotated = true;
                current_type = Some(name);
            }
            Some(Annotation::Z(z)) => pending_annotation_z = Some(z),
            Some(Annotation::LayerChange | Annotation::Layer(_)) | None => {}
        }
        apply_object_bracket(&line, &mut current_object);
        match st.apply(&line) {
            Err(error) => {
                model.stop = ModelStop::LineError {
                    offset: line.span.start,
                    error,
                };
                break;
            }
            Ok(outcome) => {
                model.lines_consumed += 1;
                for planned in &outcome.moves {
                    record_move(
                        &mut model,
                        planned,
                        current_type.as_deref(),
                        current_object.as_deref(),
                        &mut pending_annotation_z,
                        config,
                    );
                }
            }
        }
    }
    model
}

/// Tracks the `exclude_object` bracket around a line.
///
/// `EXCLUDE_OBJECT_START NAME=<name>` opens an object and
/// `EXCLUDE_OBJECT_END` closes it, mirroring
/// `klippy/extras/exclude_object.py` (`cmd_EXCLUDE_OBJECT_START` /
/// `cmd_EXCLUDE_OBJECT_END`). Names are upper-cased exactly as Klipper
/// does (`name.upper()` in both `cmd_EXCLUDE_OBJECT_DEFINE` and
/// `cmd_EXCLUDE_OBJECT_START`), so they compare directly against the
/// journaled `excluded_objects` list.
///
/// `EXCLUDE_OBJECT_END` closes whatever is open regardless of its
/// optional `NAME=`: Klipper's handler ignores a mismatched name beyond
/// logging, and a nesting disagreement must not leave an object open
/// forever. A `START` with no usable `NAME=` closes the previous bracket
/// without opening a new one — the following deposition is then
/// unattributed, which is the conservative answer (see
/// [`SimMove::object`]).
///
/// This is bracket *tracking* only; the state machine still treats both
/// commands as no-ops, so replay stays byte-exact.
fn apply_object_bracket(line: &plr_gcode::Line, current: &mut Option<String>) {
    let Some(command) = line.command() else {
        return;
    };
    match command.name.as_str() {
        "EXCLUDE_OBJECT_START" => {
            *current = command
                .get("NAME")
                .map(str::trim)
                .filter(|name| !name.is_empty())
                .map(str::to_uppercase);
        }
        "EXCLUDE_OBJECT_END" => *current = None,
        _ => {}
    }
}

/// Record one planned move into the model, opening a layer when a
/// deposition lands beyond `z_epsilon` of the current layer's Z.
fn record_move(
    model: &mut LayerModel,
    planned: &plr_gcode::PlannedMove,
    current_type: Option<&str>,
    current_object: Option<&str>,
    pending_annotation_z: &mut Option<f64>,
    config: &ModelConfig,
) {
    let end = planned.kinematic_end();
    let e_delta = end[3] - planned.start[3];
    let kind = if planned.is_extrude_only() {
        MoveKind::ExtrudeOnly
    } else if e_delta > 0.0 {
        MoveKind::Extrusion
    } else {
        MoveKind::Travel
    };
    if kind == MoveKind::Extrusion {
        let deposition_z = end[2];
        let opens_layer = match model.layers.last() {
            None => true,
            Some(layer) => (deposition_z - layer.z).abs() > config.z_epsilon,
        };
        if opens_layer {
            let index = u32::try_from(model.layers.len()).unwrap_or(u32::MAX);
            model.layers.push(Layer {
                index,
                z: deposition_z,
                z_known: planned.end_known[2],
                span: planned.span,
                annotation_z: pending_annotation_z.take(),
                paths: Vec::new(),
                extrusion_moves: 0,
                spiral_moves: 0,
            });
        }
        if let Some(layer) = model.layers.last_mut() {
            layer.extrusion_moves = layer.extrusion_moves.saturating_add(1);
            if (end[2] - planned.start[2]).abs() > 0.0 {
                layer.spiral_moves = layer.spiral_moves.saturating_add(1);
            }
            layer.span.end = layer.span.end.max(planned.span.end);
            let segment = XySegment {
                start: [planned.start[0], planned.start[1]],
                end: [end[0], end[1]],
                z: deposition_z,
                e_start: planned.start[3],
                e_end: end[3],
                span: planned.span,
                arc: planned.arc_segment,
            };
            match layer.paths.last_mut() {
                Some(path) if path.type_name.as_deref() == current_type => {
                    path.segments.push(segment);
                }
                _ => layer.paths.push(TypedPath {
                    class: current_type.map_or(FeatureClass::Other, classify_feature_type),
                    type_name: current_type.map(str::to_string),
                    segments: vec![segment],
                }),
            }
        }
    }
    model.moves.push(SimMove {
        start: planned.start,
        end,
        span: planned.span,
        kind,
        layer: model.layers.last().map(|l| l.index),
        object: current_object.map(str::to_owned),
        arc: planned.arc_segment,
        start_known: planned.start_known,
        end_known: planned.end_known,
    });
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact replay equality is intentional
mod tests {
    use super::*;

    fn model_of(text: &str) -> LayerModel {
        build_layer_model(
            GcodeState::new(),
            text.as_bytes(),
            0,
            &ModelConfig::default(),
        )
    }

    #[test]
    fn classification_table() {
        // (name, class) pairs across the three slicer vocabularies.
        let cases = [
            ("Internal infill", FeatureClass::InternalInfill),
            ("Sparse infill", FeatureClass::InternalInfill),
            ("FILL", FeatureClass::InternalInfill),
            ("Solid infill", FeatureClass::SolidInfill),
            ("Internal solid infill", FeatureClass::SolidInfill),
            ("Top solid infill", FeatureClass::Surface),
            ("Top surface", FeatureClass::Surface),
            ("Bottom surface", FeatureClass::Surface),
            ("Ironing", FeatureClass::Surface),
            ("SKIN", FeatureClass::Surface),
            ("Perimeter", FeatureClass::InnerWall),
            ("Inner wall", FeatureClass::InnerWall),
            ("WALL-INNER", FeatureClass::InnerWall),
            ("External perimeter", FeatureClass::OuterWall),
            ("Outer wall", FeatureClass::OuterWall),
            ("WALL-OUTER", FeatureClass::OuterWall),
            ("Overhang perimeter", FeatureClass::Bridge),
            ("Overhang wall", FeatureClass::Bridge),
            ("Bridge infill", FeatureClass::Bridge),
            ("Internal bridge infill", FeatureClass::Bridge),
            ("Gap fill", FeatureClass::GapFill),
            ("Gap infill", FeatureClass::GapFill),
            ("Skirt/Brim", FeatureClass::SkirtBrim),
            ("Skirt", FeatureClass::SkirtBrim),
            ("Brim", FeatureClass::SkirtBrim),
            ("Support material", FeatureClass::Support),
            ("Support material interface", FeatureClass::Support),
            ("Support interface", FeatureClass::Support),
            ("Wipe tower", FeatureClass::Other),
            ("Prime tower", FeatureClass::Other),
            ("Custom", FeatureClass::Other),
            ("", FeatureClass::Other),
        ];
        for (name, expected) in cases {
            assert_eq!(classify_feature_type(name), expected, "for {name:?}");
        }
    }

    #[test]
    fn probe_rank_ordering() {
        assert_eq!(FeatureClass::InternalInfill.probe_rank(), Some(0));
        assert_eq!(FeatureClass::SolidInfill.probe_rank(), Some(1));
        assert_eq!(FeatureClass::InnerWall.probe_rank(), Some(2));
        for class in [
            FeatureClass::OuterWall,
            FeatureClass::Surface,
            FeatureClass::Bridge,
            FeatureClass::GapFill,
            FeatureClass::SkirtBrim,
            FeatureClass::Support,
            FeatureClass::Other,
        ] {
            assert_eq!(class.probe_rank(), None, "{class:?}");
            assert!(!class.probe_eligible());
        }
        assert!(FeatureClass::InternalInfill.probe_eligible());
    }

    /// A three-layer geometric model whose layer deposition spans are
    /// known, so window→layer attribution can be checked at exact offsets.
    fn three_layer_model() -> LayerModel {
        // Each layer: a Z move, a travel, then one depositing move.
        let m = model_of(
            "G90\nM83\n\
             G1 Z0.2 F7200\nG1 X10 Y10 F9000\nG1 X20 Y10 E1 F1800\n\
             G1 Z0.4 F7200\nG1 X10 Y10 F9000\nG1 X20 Y10 E1\n\
             G1 Z0.6 F7200\nG1 X10 Y10 F9000\nG1 X20 Y10 E1\n",
        );
        assert_eq!(m.layers.len(), 3, "fixture must have three layers");
        m
    }

    #[test]
    fn window_inside_one_layer_attributes_that_layer_only() {
        let m = three_layer_model();
        // A window strictly inside layer 1's deposition span.
        let l1 = &m.layers[1];
        let start = l1.span.start;
        let end = l1.span.end; // exclusive-ish; still inside cell 1
        let wl = m.layers_in_window(start, Some(end));
        assert_eq!(wl.layers, vec![1]);
        assert!(!wl.before_first);
        assert!(wl.is_single_layer());
        assert_eq!(wl.describe(), "layer 1");
    }

    #[test]
    fn window_straddling_two_layers_reports_both() {
        let m = three_layer_model();
        // From inside layer 0 to inside layer 2 → 0,1,2.
        let start = m.layers[0].span.start;
        let end = m.layers[2].span.end;
        let wl = m.layers_in_window(start, Some(end));
        assert_eq!(wl.layers, vec![0, 1, 2]);
        assert!(!wl.is_single_layer());
        assert_eq!(wl.describe(), "layers 0\u{2013}2");
    }

    #[test]
    fn a_stop_in_the_layer_change_gap_stays_on_the_layer_in_progress() {
        let m = three_layer_model();
        // The gap between layer 0's last deposition and layer 1's first
        // deposition (the Z0.4 lift + travel) is owned by layer 0, the
        // layer whose deposition most recently began.
        let gap_offset = m.layers[0].span.end; // just past layer 0's deposit
        assert!(gap_offset < m.layers[1].span.start, "there is a real gap");
        let wl = m.layers_in_window(gap_offset, Some(gap_offset + 1));
        assert_eq!(
            wl.layers,
            vec![0],
            "gap byte belongs to the layer in progress"
        );
    }

    #[test]
    fn a_window_in_the_preamble_reports_before_first() {
        let m = three_layer_model();
        // Offset 0 is the G90/M83 preamble, before any deposition.
        let wl = m.layers_in_window(0, Some(1));
        assert!(wl.before_first);
        assert!(wl.layers.is_empty());
        assert_eq!(wl.describe(), "before layer 0 (preamble)");
        // A window from the preamble into layer 0 flags both.
        let into_l0 = m.layers_in_window(0, Some(m.layers[0].span.start + 1));
        assert!(into_l0.before_first);
        assert_eq!(into_l0.layers, vec![0]);
        assert_eq!(into_l0.describe(), "before layer 0, then layer 0");
    }

    #[test]
    fn slicer_mark_cross_check_respects_the_upper_bound_direction() {
        // Geometry says the stop is in layers 3..5. The slicer mark is an
        // UPPER bound on the physical layer (parse leads execution), so:
        //  - mark 5 (== max): consistent (some layer <= 5).
        //  - mark 4 (inside): consistent.
        //  - mark 3 (== min): consistent.
        //  - mark 2 (below all): INCONSISTENT — geometry above the upper
        //    bound is the one physically impossible case.
        let wl = WindowLayers {
            layers: vec![3, 4, 5],
            before_first: false,
        };
        assert!(wl.mark_is_consistent(5));
        assert!(wl.mark_is_consistent(4));
        assert!(wl.mark_is_consistent(3));
        assert!(
            !wl.mark_is_consistent(2),
            "mark below every layer is impossible"
        );
        // Preamble is trivially consistent with any mark (before layer 0).
        let pre = WindowLayers {
            layers: vec![],
            before_first: true,
        };
        assert!(pre.mark_is_consistent(0));
    }

    #[test]
    fn degenerate_and_empty_windows_are_total() {
        let m = three_layer_model();
        // end == start (degenerate): attribute the single named offset.
        let at = m.layers[2].span.start;
        assert_eq!(m.layers_in_window(at, Some(at)).layers, vec![2]);
        // end == None: from the offset to the end of the model → last layer.
        assert_eq!(m.layers_in_window(at, None).layers, vec![2]);
        // A window past every deposition still lands on the last layer.
        assert_eq!(m.layers_in_window(u64::MAX - 1, None).layers, vec![2]);
        // An empty model: everything is "before first", no layers, no panic.
        let empty = model_of("G90\nM83\nG1 X10 Y10 F9000\n");
        assert!(empty.layers.is_empty());
        let wl = empty.layers_in_window(0, Some(100));
        assert!(wl.before_first);
        assert!(wl.layers.is_empty());
        assert_eq!(wl.describe(), "before layer 0 (preamble)");
    }

    #[test]
    fn layers_split_geometrically_not_by_annotation() {
        // No LAYER_CHANGE comments at all; two Z planes -> two layers.
        let m = model_of(
            "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y10 F9000\nG1 X20 Y10 E1 F1800\n\
             G1 Z0.4 F7200\nG1 X10 Y10 E1\n",
        );
        assert_eq!(m.layers.len(), 2);
        assert_eq!(m.layers[0].z, 0.2);
        assert_eq!(m.layers[1].z, 0.4);
        assert_eq!(m.layers[0].index, 0);
        assert_eq!(m.layers[1].index, 1);
        assert!(m.layers.iter().all(|l| l.z_known));
        assert!(!m.annotated);
    }

    #[test]
    fn z_hop_does_not_open_a_layer() {
        let m = model_of(
            "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y0 E1 F1800\n\
             G1 E-0.8 F2100\nG1 Z0.6 F7200\nG1 X50 Y0 F9000\nG1 Z0.2 F7200\n\
             G1 E0.8 F2100\nG1 X60 Y0 E1 F1800\n",
        );
        assert_eq!(m.layers.len(), 1, "hop travel must not open a layer");
        assert_eq!(m.layers[0].extrusion_moves, 2);
        assert_eq!(m.layers[0].spiral_moves, 0);
        // The travels and retracts are still in the move stream.
        assert!(m.moves.iter().any(|mv| mv.kind == MoveKind::Travel));
        assert!(m.moves.iter().any(|mv| mv.kind == MoveKind::ExtrudeOnly));
    }

    #[test]
    fn type_blocks_group_paths_and_annotations_classify() {
        let m = model_of(
            "G90\nM83\nG1 Z0.2 F7200\n\
             ;TYPE:Inner wall\nG1 X10 Y0 E1 F1800\nG1 X10 Y10 E1\n\
             ;TYPE:Sparse infill\nG1 X0 Y10 E1\n\
             ;TYPE:Inner wall\nG1 X0 Y0 E1\n",
        );
        assert!(m.annotated);
        assert_eq!(m.layers.len(), 1);
        let paths = &m.layers[0].paths;
        assert_eq!(paths.len(), 3, "recurring type opens a new block");
        assert_eq!(paths[0].class, FeatureClass::InnerWall);
        assert_eq!(paths[0].segments.len(), 2);
        assert_eq!(paths[1].class, FeatureClass::InternalInfill);
        assert_eq!(paths[2].class, FeatureClass::InnerWall);
        assert_eq!(paths[0].type_name.as_deref(), Some("Inner wall"));
    }

    #[test]
    fn unannotated_deposition_is_kept_but_unnamed() {
        let m = model_of("G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y0 E1 F1800\n");
        assert!(!m.annotated);
        let path = &m.layers[0].paths[0];
        assert_eq!(path.type_name, None);
        assert_eq!(path.class, FeatureClass::Other);
    }

    #[test]
    fn annotation_z_attaches_to_next_layer() {
        let m = model_of(
            "G90\nM83\n;Z:0.2\nG1 Z0.2 F7200\nG1 X10 Y0 E1 F1800\n\
             ;LAYER_CHANGE\n;Z:0.4\nG1 Z0.4 F7200\nG1 X0 Y0 E1\n",
        );
        assert_eq!(m.layers.len(), 2);
        assert_eq!(m.layers[0].annotation_z, Some(0.2));
        assert_eq!(m.layers[1].annotation_z, Some(0.4));
    }

    #[test]
    fn spiral_moves_counted() {
        let m = model_of(
            "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y0 F9000\nG91\n\
             G1 X1 Z0.01 E0.05 F1800\nG1 X1 Z0.01 E0.05\nG1 X1 Z0.01 E0.05\n",
        );
        assert_eq!(m.layers.len(), 1);
        assert_eq!(m.layers[0].spiral_moves, 3);
        assert_eq!(m.spiral_fraction(), 1.0);
    }

    #[test]
    fn spiral_fraction_zero_without_extrusion() {
        let m = model_of("G90\nG1 X10 F9000\n");
        assert_eq!(m.spiral_fraction(), 0.0);
        assert!(m.layers.is_empty());
        assert_eq!(m.moves.len(), 1);
        assert_eq!(m.moves[0].layer, None);
    }

    #[test]
    fn arc_chords_expand_with_shared_source_span() {
        let m = model_of("G90\nM82\nG1 X10 Y0 Z0.4 F6000\nG3 X0 Y10 I-10 E3 F1800\n");
        assert_eq!(m.layers.len(), 1);
        let segs = &m.layers[0].paths[0].segments;
        assert_eq!(segs.len(), 15, "quarter circle at 1 mm resolution");
        let first_span = segs[0].span;
        assert!(segs.iter().all(|s| s.span == first_span));
        assert!(segs.iter().all(|s| s.arc.is_some()));
        // Chords chain: each start is the previous end.
        for pair in segs.windows(2) {
            assert_eq!(pair[0].end, pair[1].start);
        }
    }

    #[test]
    fn line_error_stops_and_keeps_prefix() {
        let text = "G90\nG1 Z0.2 F7200\nG1 X10 Y0 E1 F1800\nG20\nG1 X20 Y0 E1\n";
        let m = model_of(text);
        assert_eq!(m.layers.len(), 1);
        assert_eq!(m.layers[0].extrusion_moves, 1);
        let ModelStop::LineError { offset, error } = &m.stop else {
            panic!("expected LineError, got {:?}", m.stop);
        };
        assert_eq!(*offset, text.find("G20").unwrap() as u64);
        assert!(matches!(error, StateError::InchesUnsupported));
    }

    #[test]
    fn empty_and_garbage_inputs_are_sane() {
        let empty = model_of("");
        assert!(empty.layers.is_empty() && empty.moves.is_empty());
        assert_eq!(empty.stop, ModelStop::EndOfInput);
        let garbage = build_layer_model(
            GcodeState::new(),
            &[0xff, 0xfe, b'\n', b'@', b'!', b'\n'],
            0,
            &ModelConfig::default(),
        );
        assert!(garbage.layers.is_empty());
    }

    #[test]
    fn base_offset_shifts_spans() {
        let text = "G1 X10 Y0 E1 F1800\n";
        let m = build_layer_model(
            GcodeState::new(),
            text.as_bytes(),
            1000,
            &ModelConfig::default(),
        );
        assert_eq!(m.moves.len(), 1);
        assert_eq!(m.moves[0].span.start, 1000);
        assert_eq!(m.moves[0].span.end, 1000 + text.len() as u64);
    }

    #[test]
    fn first_deposition_at_or_after_selects_next_extrusion() {
        let text =
            "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y0 E1 F1800\nG1 X20 Y0 F9000\nG1 X30 Y0 E1 F1800\n";
        let m = model_of(text);
        let first = m.first_deposition_at_or_after(0).expect("has deposition");
        assert_eq!(first.start[0], 0.0);
        // After the first extrusion line: skips the travel, lands on the
        // second extrusion.
        let after = m
            .first_deposition_at_or_after(first.span.end)
            .expect("second deposition");
        assert_eq!(after.end[0], 30.0);
        assert_eq!(m.first_deposition_at_or_after(u64::MAX), None);
        // The offsets are line boundaries by construction.
        assert_eq!(after.span.start, text.find("G1 X30").unwrap() as u64);
    }

    #[test]
    fn g28_marks_layer_z_unknown() {
        let m = model_of("G90\nM83\nG28\nG91\nG1 Z0.2 F7200\nG1 X10 Y0 E1 F1800\n");
        assert_eq!(m.layers.len(), 1);
        assert!(!m.layers[0].z_known);
    }

    #[test]
    fn nonfinite_z_epsilon_keeps_single_layer() {
        let cfg = ModelConfig {
            z_epsilon: f64::NAN,
        };
        let m = build_layer_model(
            GcodeState::new(),
            b"G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y0 E1 F1800\nG1 Z5 F7200\nG1 X0 Y0 E1\n",
            0,
            &cfg,
        );
        assert_eq!(m.layers.len(), 1, "NaN epsilon never opens layers");
    }

    #[test]
    fn model_serializes() {
        let m = model_of("G90\nM83\nG1 Z0.2 F7200\n;TYPE:Inner wall\nG1 X10 Y0 E1 F1800\n");
        let json = serde_json::to_string(&m).expect("serialize");
        let back: LayerModel = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(m, back);
    }
}
