//! G-code parsing, `gcode_move` state simulation, arc decomposition, and
//! forward motion simulation for power-loss recovery.
//!
//! After a power loss the WAL's durable tail can end *before* the
//! machine actually stopped. This crate forward-simulates the g-code
//! from the last recorded file offset, reproducing Klipper's coordinate
//! bookkeeping and arc decomposition so the reconstruction engine can
//! reason about what the machine did next.
//!
//! # Design contract
//!
//! * **Z is safety-critical and exact.** The sequence of Z-touching
//!   moves — z-hops, layer changes, spiral steps — produced by
//!   [`sim::scan_z_events`] comes from a byte-faithful replay of
//!   Klipper's `gcode_move` semantics and involves no timing model.
//! * **XY/E timing is approximate.** [`sim::simulate`] attaches
//!   trapezoidal timestamps whose accuracy limits are documented in
//!   [`sim`]'s module docs; they bound line-match granularity only.
//! * **Byte offsets are line-boundary exact.** Every parsed
//!   [`parse::Line`] carries its [`parse::ByteSpan`]; `span.end` of a
//!   line is the offset of the next line, suitable for `M26 S<byte>`.
//! * **E-frame fidelity.** trapq/WAL E is Klipper-internal; matching it
//!   to file E requires replaying G92/M220/M221/modes/offsets. The
//!   [`state::GcodeState`] arithmetic for these matches
//!   `klippy/extras/gcode_move.py` exactly (unit-tested per claim).
//! * **Totality.** Parsing never panics on any input, including
//!   non-UTF-8 bytes (property-tested); the state machine returns
//!   errors, never panics.
//!
//! # Klipper reference
//!
//! Semantics are grounded in the Klipper source tree (line references in
//! each module): `klippy/gcode.py` (tokenization),
//! `klippy/extras/gcode_move.py` (coordinate state),
//! `klippy/extras/gcode_arcs.py` (G2/G3), `klippy/toolhead.py`
//! (kinematic limits and lookahead), `klippy/extras/homing.py` (G28).
//!
//! Known divergences, all documented at their site: lossy UTF-8
//! decoding, rejection of non-finite parameter values, an upper bound
//! on arc segment count, and the timing-model simplifications listed in
//! [`sim`].

pub mod arc;
pub mod parse;
pub mod sim;
pub mod state;

pub use arc::{plan_arc, ArcError, ArcPlane, ArcRequest, ArcSegment, MAX_ARC_SEGMENTS};
pub use parse::{
    parse_line, Annotation, ByteSpan, Command, CommandParams, Comment, Line, LineBody, LineIter,
};
pub use sim::{
    scan_z_events, simulate, z_event_of, SimConfig, Simulation, StopReason, TimedMove, ZEvent,
    ZScan, ZScanConfig,
};
pub use state::{
    ApplyOutcome, ArcSegmentInfo, Disposition, GcodeState, PlannedMove, SavedGcodeState,
    StateError, DEFAULT_ARC_RESOLUTION,
};
