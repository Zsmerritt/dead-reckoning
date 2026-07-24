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
//!
//! # Totality
//!
//! No public function panics on any input: model building is total on
//! arbitrary bytes, and the matcher/selector validate every numeric
//! input, returning typed errors for NaN/infinite evidence
//! (property-tested in `tests/properties.rs`).

mod geom;

pub mod contact;
pub mod matcher;
pub mod model;

pub use contact::{
    select_contact_zone, ContactConfig, ContactError, ContactOutcome, DeclineReason,
    ProbeCandidate, SAMPLE_TS,
};
pub use matcher::{
    match_stop_point, ByteWindow, Interval, MatchCandidate, MatchConfidence, MatchConfig,
    MatchError, MatchResult, StopEvidence,
};
pub use model::{
    build_layer_model, classify_feature_type, FeatureClass, Layer, LayerModel, ModelConfig,
    ModelStop, MoveKind, SimMove, TypedPath, XySegment,
};
