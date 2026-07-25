//! G-code path analysis for power-loss recovery: line matching in
//! simulated-move space and contact-zone selection for the
//! part-referenced Z probe. Pure logic, cross-platform by construction
//! (fixture file I/O lives in the tests only).
//!
//! Built on `plr-gcode`: every analysis here replays the byte-exact
//! parser and the Klipper-faithful [`plr_gcode::GcodeState`], so
//! positions, extruder values and arc chords live in the same
//! Klipper-internal frame the WAL/trapq data uses.
//!
//! # The three components
//!
//! * [`model::build_layer_model`] — streams a byte window into a
//!   [`model::LayerModel`]: geometric layers (annotations classify,
//!   geometry decides), per-`;TYPE:` extrusion polylines with arc-chord
//!   expansion, and the full simulated move stream.
//! * [`matcher::match_stop_point`] — answers *"where in the file did we
//!   stop?"* from reconstruction evidence
//!   ([`matcher::StopEvidence`] — the input contract for
//!   `plr-reconstruct`), with honest
//!   [`matcher::MatchConfidence`] granularity: unique line, ambiguous
//!   window, or layer-only. Ambiguity never degrades Z correctness.
//! * [`contact::select_contact_zone`] — answers *"where do we probe?"*:
//!   ranked probe points on layer N−1 plastic that layer N will bury,
//!   never on visible or excluded surfaces, with typed
//!   [`contact::DeclineReason`]s (vase mode, single wall, no safe
//!   zone) and typed refusals when `;TYPE:` annotations are missing.
//! * [`structure`] — answers *"will the part survive being touched
//!   there?"*: connected-component islands, bed-adhesion footprints
//!   traced down the layer stack, and a per-criterion
//!   [`structure::StructuralVerdict`] (adhesion, tipping, feature
//!   width, edge margin, drag run). The selector filters on it; a
//!   manual-jog UI validates hand-picked points against the same
//!   [`structure::StructuralAnalysis`].
//! * [`work::remaining_work`] — answers *"is there anything left to
//!   print?"*: the completion gate. A finished print stops ~14.5 KB
//!   short of EOF because of the slicer's config-block footer, so no
//!   percentage can decide this; only a content test can. It may only
//!   ever answer "complete" on positive proof — every way of not
//!   knowing is a [`work::WorkUnknown`].
//!
//! # Totality
//!
//! No public function panics on any input: model building is total on
//! arbitrary bytes, and the matcher/selector validate every numeric
//! input, returning typed errors for NaN/infinite evidence
//! (property-tested in `tests/properties.rs`).

mod geom;
mod raster;

pub mod contact;
pub mod matcher;
pub mod model;
pub mod structure;
pub mod work;

pub use contact::{
    select_contact_zone, select_contact_zone_detailed, ContactConfig, ContactError, ContactOutcome,
    ContactSelection, DeclineReason, ProbeCandidate, RejectedCandidate, SAMPLE_TS,
};
pub use matcher::{
    match_stop_point, ByteWindow, Interval, MatchCandidate, MatchConfidence, MatchConfig,
    MatchError, MatchResult, StopEvidence,
};
pub use model::{
    build_layer_model, classify_feature_type, FeatureClass, Layer, LayerModel, ModelConfig,
    ModelStop, MoveKind, SimMove, TypedPath, XySegment,
};
pub use structure::{
    assess_contact_point, BoundingBox, ClearRun, ContactMode, CriterionCheck, FootprintTrace,
    Island, StructuralAnalysis, StructuralAssessment, StructuralCriterion, StructuralOutcome,
    StructuralVerdict, TraceStatus,
};
pub use work::{
    remaining_work, AnchorFrame, ExclusionOracle, ModeAxis, RemainingWork, WorkUnknown,
    MAX_NAMED_COMMANDS,
};
