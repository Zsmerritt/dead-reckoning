//! The g-code coordinate state machine, mirroring Klipper's
//! `klippy/extras/gcode_move.py` (`GCodeMove`) attribute for attribute.
//!
//! Coordinate frames (all lengths mm, speeds mm/s internally):
//!
//! * `last_position` — Klipper-internal ("toolhead") coordinates, the
//!   frame trapq/WAL positions live in;
//! * `base_position` — offset such that
//!   `gcode = last - base` (with E additionally divided by
//!   `extrude_factor`), see `_get_gcode_position` (gcode_move.py:94-97);
//! * `homing_position` — the `SET_GCODE_OFFSET` origin.
//!
//! Semantics that later crates depend on (each covered by a targeted
//! unit test below, with the justifying source lines):
//!
//! * G1 E words are multiplied by `extrude_factor` before being applied,
//!   and E is treated absolutely only when *both* G90 and M82 modes are
//!   active (gcode_move.py:134-151);
//! * G92 shifts `base_position` only — `offset` is scaled by
//!   `extrude_factor` for E, and a bare G92 rebases all four axes to the
//!   current position (gcode_move.py:181-190);
//! * M220 rescales the internal speed so the *g-code* speed is
//!   unchanged; it never touches positions (gcode_move.py:195-199);
//! * M221 rewrites `base_position[3]` so the *g-code* E position is
//!   unchanged by the factor switch (gcode_move.py:200-206);
//! * `SET_GCODE_OFFSET` accumulates into both `homing_position` and
//!   `base_position`, optionally emitting a compensating move
//!   (gcode_move.py:207-226).
//!
//! Position knowledge: Klipper reads the real position back from the
//! toolhead after homing; a file-side simulation cannot. After G28 the
//! homed axes are marked *unknown* (`position_known` false) while
//! `base_position` is set to `homing_position` exactly as Klipper's
//! `_handle_home_rails_end` does (gcode_move.py:79-82). The next
//! *absolute* move on such an axis fully restores knowledge (the target
//! is `value + base_position`, both known); relative moves keep the axis
//! unknown. Consumers must treat positions with a false flag as
//! unreliable.
//!
//! Deliberate safety divergence from Klipper: non-finite parameter
//! values (`nan`/`inf`, which `CPython`'s `float()` accepts) are rejected
//! with [`StateError::NonFiniteParam`] instead of being propagated into
//! positions. Klipper would poison its position state and error out at
//! the toolhead layer; either way the print is lost, and refusing early
//! keeps the safety-critical Z reconstruction sound.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::arc::{plan_arc, ArcError, ArcPlane, ArcRequest};
use crate::parse::{ByteSpan, Command, Line, LineBody};

/// Klipper's default arc chord length in mm (`gcode_arcs.py:31`).
pub const DEFAULT_ARC_RESOLUTION: f64 = 1.0;
/// Moves with less XYZ travel than this are extrude-only
/// (toolhead.py:26).
pub const MIN_KINEMATIC_MOVE: f64 = 0.000_000_001;
/// Acceleration Klipper assigns to extrude-only moves (toolhead.py:35).
pub const EXTRUDE_ONLY_ACCEL: f64 = 99_999_999.9;

/// Errors surfaced while applying a line to the state machine. Klipper
/// responds `!!` and aborts a printing file on these; the simulator
/// stops at the offending line.
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize, Deserialize)]
pub enum StateError {
    /// A parameter value failed to parse as a number.
    #[error("unable to parse parameter {key}={value:?} of {command}")]
    InvalidParam {
        /// Command name.
        command: String,
        /// Parameter key.
        key: String,
        /// Offending raw value.
        value: String,
    },
    /// A parameter parsed to `nan`/`inf` (safety divergence; see module
    /// docs).
    #[error("non-finite value for parameter {key} of {command}")]
    NonFiniteParam {
        /// Command name.
        command: String,
        /// Parameter key.
        key: String,
    },
    /// `F` word not strictly positive (gcode_move.py:152-156).
    #[error("invalid speed F{value}")]
    InvalidSpeed {
        /// The rejected feedrate (g-code mm/min scale).
        value: f64,
    },
    /// A parameter violated an `above=` bound of the Klipper handler.
    #[error("parameter {key} of {command} must be above {min}")]
    ParamNotAbove {
        /// Command name.
        command: String,
        /// Parameter key.
        key: String,
        /// Exclusive lower bound.
        min: f64,
    },
    /// G20: "Machine does not support G20 (inches) command"
    /// (gcode_move.py:163-165).
    #[error("machine does not support G20 (inches) command")]
    InchesUnsupported,
    /// Extended command whose parameters failed shlex parsing; Klipper
    /// raises "Malformed command" (gcode.py:275-277).
    #[error("malformed parameters for extended command {command}")]
    MalformedExtended {
        /// Command name.
        command: String,
    },
    /// `RESTORE_GCODE_STATE` with an unsaved name (gcode_move.py:243-244).
    #[error("unknown g-code state: {name}")]
    UnknownSavedState {
        /// The requested state name.
        name: String,
    },
    /// Arc validation/decomposition failure.
    #[error(transparent)]
    Arc(#[from] ArcError),
}

/// Identifies which chord of an arc a [`PlannedMove`] came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArcSegmentInfo {
    /// 1-based chord index.
    pub index: u32,
    /// Total chord count of the arc.
    pub count: u32,
}

/// A single motion produced by applying a line: Klipper-internal start
/// and end coordinates plus the requested speed.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlannedMove {
    /// Position before the move (X, Y, Z, E), internal coordinates.
    pub start: [f64; 4],
    /// Position after the move (X, Y, Z, E), internal coordinates.
    pub end: [f64; 4],
    /// Requested speed in mm/s (already speed-factor scaled — Klipper's
    /// internal `self.speed`).
    pub speed: f64,
    /// M204 acceleration override active for this move, mm/s^2
    /// (`None` = use the simulator's configured `max_accel`).
    pub accel_override: Option<f64>,
    /// Span of the source line that produced this move.
    pub span: ByteSpan,
    /// Set when this move is one chord of a G2/G3 decomposition.
    pub arc_segment: Option<ArcSegmentInfo>,
    /// Per-axis reliability of `start` (see module docs on G28).
    pub start_known: [bool; 4],
    /// Per-axis reliability of `end`.
    pub end_known: [bool; 4],
}

impl PlannedMove {
    /// Per-axis displacement.
    #[must_use]
    pub fn axes_delta(&self) -> [f64; 4] {
        [
            self.end[0] - self.start[0],
            self.end[1] - self.start[1],
            self.end[2] - self.start[2],
            self.end[3] - self.start[3],
        ]
    }

    /// Euclidean XYZ travel distance.
    #[must_use]
    pub fn xyz_distance(&self) -> f64 {
        let d = self.axes_delta();
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    }

    /// True when the toolhead classifies this as an extrude-only move
    /// (XYZ travel below [`MIN_KINEMATIC_MOVE`], toolhead.py:26-37).
    #[must_use]
    pub fn is_extrude_only(&self) -> bool {
        self.xyz_distance() < MIN_KINEMATIC_MOVE
    }

    /// The distance the kinematics actually plan over: XYZ distance, or
    /// |dE| for extrude-only moves (toolhead.py:25-31).
    #[must_use]
    pub fn kinematic_distance(&self) -> f64 {
        if self.is_extrude_only() {
            (self.end[3] - self.start[3]).abs()
        } else {
            self.xyz_distance()
        }
    }

    /// The end position after the toolhead's extrude-only XYZ snap
    /// (toolhead.py:28-29): extrude-only moves do not move XYZ at all.
    #[must_use]
    pub fn kinematic_end(&self) -> [f64; 4] {
        if self.is_extrude_only() {
            [self.start[0], self.start[1], self.start[2], self.end[3]]
        } else {
            self.end
        }
    }

    /// True when the move extrudes forward (positive E displacement).
    #[must_use]
    pub fn extrudes(&self) -> bool {
        self.end[3] - self.start[3] > 0.0
    }
}

/// How a line was handled by [`GcodeState::apply`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum Disposition {
    /// Blank or comment-only line.
    Blank,
    /// G0/G1 (possibly producing no motion, e.g. an F-only line).
    Move,
    /// G2/G3, decomposed into `segments` chords.
    Arc {
        /// Chord count of the decomposition.
        segments: u32,
    },
    /// A known state-manipulation command (modes, factors, offsets,
    /// save/restore, M204, G21, read-only queries).
    State,
    /// G28; `axes[i]` true when axis i (X, Y, Z) was homed.
    Homing {
        /// Homed axes.
        axes: [bool; 3],
    },
    /// Unknown command — state unchanged, annotated for the caller.
    PassThrough,
}

/// Result of applying one line.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ApplyOutcome {
    /// Motions produced by the line, in execution order.
    pub moves: Vec<PlannedMove>,
    /// Classification of what the line did.
    pub disposition: Disposition,
}

impl ApplyOutcome {
    fn empty(disposition: Disposition) -> Self {
        Self {
            moves: Vec::new(),
            disposition,
        }
    }
}

/// Snapshot stored by `SAVE_GCODE_STATE` (gcode_move.py:228-238), plus
/// the position-knowledge flags this crate tracks.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SavedGcodeState {
    /// Saved G90/G91 mode.
    pub absolute_coord: bool,
    /// Saved M82/M83 mode.
    pub absolute_extrude: bool,
    /// Saved `base_position`.
    pub base_position: [f64; 4],
    /// Saved `last_position`.
    pub last_position: [f64; 4],
    /// Saved `homing_position`.
    pub homing_position: [f64; 4],
    /// Saved internal speed (mm/s).
    pub speed: f64,
    /// Saved speed factor.
    pub speed_factor: f64,
    /// Saved extrude factor.
    pub extrude_factor: f64,
    /// Saved `base_position` reliability flags.
    pub base_known: [bool; 4],
    /// Saved `last_position` reliability flags.
    pub position_known: [bool; 4],
}

/// Mirror of Klipper `GCodeMove` state (gcode_move.py:29-36) plus arc
/// plane/resolution (`gcode_arcs.py:31,43`), the M204 override, and
/// position-knowledge tracking.
///
/// Fields are public because this is a faithful mirror of Klipper's
/// plain attributes; reconstruction code initializes them from WAL data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GcodeState {
    /// G90 (true) / G91 (false).
    pub absolute_coord: bool,
    /// M82 (true) / M83 (false).
    pub absolute_extrude: bool,
    /// `gcode = last - base` offset (E additionally factor-scaled).
    pub base_position: [f64; 4],
    /// Current internal position.
    pub last_position: [f64; 4],
    /// `SET_GCODE_OFFSET` origin.
    pub homing_position: [f64; 4],
    /// Internal speed, mm/s (Klipper default 25.0).
    pub speed: f64,
    /// Speed factor in per-minute units (default 1/60 = 100%).
    pub speed_factor: f64,
    /// M221 extrude factor (default 1.0).
    pub extrude_factor: f64,
    /// Per-axis reliability of `last_position` (false after G28 until an
    /// absolute move re-establishes the axis).
    pub position_known: [bool; 4],
    /// Per-axis reliability of `base_position`.
    pub base_known: [bool; 4],
    /// Arc plane selected by G17/G18/G19.
    pub arc_plane: ArcPlane,
    /// Arc chord length, mm (`[gcode_arcs] resolution`).
    pub arc_resolution: f64,
    /// M204-set acceleration, mm/s^2 (None until the file sets one).
    pub accel_override: Option<f64>,
    /// `SAVE_GCODE_STATE` snapshots by name.
    pub saved_states: BTreeMap<String, SavedGcodeState>,
}

impl Default for GcodeState {
    fn default() -> Self {
        Self {
            absolute_coord: true,
            absolute_extrude: true,
            base_position: [0.0; 4],
            last_position: [0.0; 4],
            homing_position: [0.0; 4],
            speed: 25.0,
            speed_factor: 1.0 / 60.0,
            extrude_factor: 1.0,
            // Reconstruction starts from a WAL-known state, so positions
            // default to "known" (Klipper itself starts unhomed).
            position_known: [true; 4],
            base_known: [true; 4],
            arc_plane: ArcPlane::default(),
            arc_resolution: DEFAULT_ARC_RESOLUTION,
            accel_override: None,
            saved_states: BTreeMap::new(),
        }
    }
}

impl GcodeState {
    /// Fresh state with Klipper's power-on defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Current g-code position (`_get_gcode_position`,
    /// gcode_move.py:94-97).
    #[must_use]
    pub fn gcode_position(&self) -> [f64; 4] {
        [
            self.last_position[0] - self.base_position[0],
            self.last_position[1] - self.base_position[1],
            self.last_position[2] - self.base_position[2],
            (self.last_position[3] - self.base_position[3]) / self.extrude_factor,
        ]
    }

    /// Current g-code-frame speed in mm/s (`_get_gcode_speed`,
    /// gcode_move.py:98-99).
    #[must_use]
    pub fn gcode_speed(&self) -> f64 {
        self.speed / self.speed_factor
    }

    /// The M220 override as a fraction (1.0 = 100%), matching
    /// `_get_gcode_speed_override` (gcode_move.py:100-101).
    #[must_use]
    pub fn speed_factor_override(&self) -> f64 {
        self.speed_factor * 60.0
    }

    /// Apply one parsed line, mutating the state and returning any
    /// motions plus a disposition. Unknown commands are annotated
    /// [`Disposition::PassThrough`] and leave the state untouched.
    pub fn apply(&mut self, line: &Line) -> Result<ApplyOutcome, StateError> {
        let command = match &line.body {
            LineBody::Blank | LineBody::Comment(_) => {
                return Ok(ApplyOutcome::empty(Disposition::Blank))
            }
            LineBody::Command { command, .. } => command,
        };
        let span = line.span;
        match command.name.as_str() {
            "G0" | "G1" => self.cmd_g1(command, span),
            "G2" => self.cmd_arc(command, span, true),
            "G3" => self.cmd_arc(command, span, false),
            "G17" => {
                self.arc_plane = ArcPlane::Xy;
                Ok(ApplyOutcome::empty(Disposition::State))
            }
            "G18" => {
                self.arc_plane = ArcPlane::Xz;
                Ok(ApplyOutcome::empty(Disposition::State))
            }
            "G19" => {
                self.arc_plane = ArcPlane::Yz;
                Ok(ApplyOutcome::empty(Disposition::State))
            }
            "G20" => Err(StateError::InchesUnsupported),
            // G21 is a no-op (gcode_move.py:166-168); M114/GET_POSITION
            // are read-only queries.
            "G21" | "M114" | "GET_POSITION" => Ok(ApplyOutcome::empty(Disposition::State)),
            "G28" => Ok(self.cmd_g28(command)),
            "G90" => {
                self.absolute_coord = true;
                Ok(ApplyOutcome::empty(Disposition::State))
            }
            "G91" => {
                self.absolute_coord = false;
                Ok(ApplyOutcome::empty(Disposition::State))
            }
            "G92" => self.cmd_g92(command),
            "M82" => {
                self.absolute_extrude = true;
                Ok(ApplyOutcome::empty(Disposition::State))
            }
            "M83" => {
                self.absolute_extrude = false;
                Ok(ApplyOutcome::empty(Disposition::State))
            }
            "M204" => self.cmd_m204(command),
            "M220" => self.cmd_m220(command),
            "M221" => self.cmd_m221(command),
            "SET_GCODE_OFFSET" => self.cmd_set_gcode_offset(command, span),
            "SAVE_GCODE_STATE" => self.cmd_save_gcode_state(command),
            "RESTORE_GCODE_STATE" => self.cmd_restore_gcode_state(command, span),
            _ => Ok(ApplyOutcome::empty(Disposition::PassThrough)),
        }
    }

    // --- G0/G1 (gcode_move.py:134-161) ---

    fn cmd_g1(&mut self, cmd: &Command, span: ByteSpan) -> Result<ApplyOutcome, StateError> {
        // Parse in Klipper's axis_map order (X, Y, Z, E), then F, so
        // error precedence matches; then apply atomically (Klipper's
        // partial updates are rolled back by its command_error handler,
        // so the net effect is identical).
        let word_x = get_float(cmd, "X")?;
        let word_y = get_float(cmd, "Y")?;
        let word_z = get_float(cmd, "Z")?;
        let word_e = get_float(cmd, "E")?;
        let word_f = get_float(cmd, "F")?;
        if let Some(fv) = word_f {
            if fv <= 0.0 {
                return Err(StateError::InvalidSpeed { value: fv });
            }
        }
        let mv = self.apply_g1_values([word_x, word_y, word_z, word_e], word_f, span, None);
        Ok(ApplyOutcome {
            moves: mv.into_iter().collect(),
            disposition: Disposition::Move,
        })
    }

    /// Core of `cmd_G1` (gcode_move.py:134-161) operating on pre-parsed
    /// values; also the entry point for synthesized arc chords, which
    /// Klipper feeds through `cmd_G1` (gcode_arcs.py:171-180).
    /// `feed` must already be validated (> 0).
    fn apply_g1_values(
        &mut self,
        values: [Option<f64>; 4],
        feed: Option<f64>,
        span: ByteSpan,
        arc_segment: Option<ArcSegmentInfo>,
    ) -> Option<PlannedMove> {
        let start = self.last_position;
        let start_known = self.position_known;
        let [vx, vy, vz, ve] = values;
        if let Some(v) = vx {
            self.apply_axis(0, v, self.absolute_coord);
        }
        if let Some(v) = vy {
            self.apply_axis(1, v, self.absolute_coord);
        }
        if let Some(v) = vz {
            self.apply_axis(2, v, self.absolute_coord);
        }
        if let Some(v) = ve {
            // E value is extrude-factor scaled, and absolute only when
            // both G90 and M82 are active (gcode_move.py:142-145).
            let scaled = v * self.extrude_factor;
            let absolute = self.absolute_coord && self.absolute_extrude;
            self.apply_axis(3, scaled, absolute);
        }
        if let Some(fv) = feed {
            self.speed = fv * self.speed_factor;
        }
        self.emit_move(start, start_known, self.speed, span, arc_segment)
    }

    /// Apply one axis word: absolute values are relative to
    /// `base_position`, relative values add to `last_position`
    /// (gcode_move.py:146-151). `axis` is always 0..=3; out-of-range
    /// indices are unreachable and degrade to a no-op rather than panic.
    fn apply_axis(&mut self, axis: usize, value: f64, absolute: bool) {
        let base = self.base_position.get(axis).copied().unwrap_or(0.0);
        let base_known = self.base_known.get(axis).copied().unwrap_or(false);
        if absolute {
            if let Some(slot) = self.last_position.get_mut(axis) {
                *slot = value + base;
            }
            if let Some(kslot) = self.position_known.get_mut(axis) {
                *kslot = base_known;
            }
        } else if let Some(slot) = self.last_position.get_mut(axis) {
            *slot += value;
        }
    }

    /// Emit a [`PlannedMove`] unless the toolhead would drop it as
    /// zero-distance (toolhead.py Move ctor + `ToolHead.move`'s
    /// `if not move.move_d` guard).
    #[allow(clippy::float_cmp)] // exact zero-drop check mirrors Klipper
    fn emit_move(
        &self,
        start: [f64; 4],
        start_known: [bool; 4],
        speed: f64,
        span: ByteSpan,
        arc_segment: Option<ArcSegmentInfo>,
    ) -> Option<PlannedMove> {
        let end = self.last_position;
        let dx = end[0] - start[0];
        let dy = end[1] - start[1];
        let dz = end[2] - start[2];
        let xyz_d = (dx * dx + dy * dy + dz * dz).sqrt();
        let kin_d = if xyz_d < MIN_KINEMATIC_MOVE {
            (end[3] - start[3]).abs()
        } else {
            xyz_d
        };
        if kin_d == 0.0 {
            return None;
        }
        Some(PlannedMove {
            start,
            end,
            speed,
            accel_override: self.accel_override,
            span,
            arc_segment,
            start_known,
            end_known: self.position_known,
        })
    }

    // --- G2/G3 (gcode_arcs.py:60-94) ---

    fn cmd_arc(
        &mut self,
        cmd: &Command,
        span: ByteSpan,
        clockwise: bool,
    ) -> Result<ApplyOutcome, StateError> {
        if !self.absolute_coord {
            return Err(ArcError::RelativeMode.into());
        }
        let current = self.gcode_position();
        // Target words default to the current g-code position
        // (gcode_arcs.py:68-70).
        let target_x = get_float(cmd, "X")?.unwrap_or(current[0]);
        let target_y = get_float(cmd, "Y")?.unwrap_or(current[1]);
        let target_z = get_float(cmd, "Z")?.unwrap_or(current[2]);
        if get_float(cmd, "R")?.is_some() {
            return Err(ArcError::RadiusForm.into());
        }
        let word_i = get_float(cmd, "I")?.unwrap_or(0.0);
        let word_j = get_float(cmd, "J")?.unwrap_or(0.0);
        let offset = match self.arc_plane {
            ArcPlane::Xy => (word_i, word_j),
            ArcPlane::Xz => {
                let word_k = get_float(cmd, "K")?.unwrap_or(0.0);
                (word_i, word_k)
            }
            ArcPlane::Yz => {
                let word_k = get_float(cmd, "K")?.unwrap_or(0.0);
                (word_j, word_k)
            }
        };
        // Truthiness check as in `if not (asPlanar[0] or asPlanar[1])`
        // (gcode_arcs.py:89-90).
        if offset.0 == 0.0 && offset.1 == 0.0 {
            return Err(ArcError::MissingOffsets.into());
        }
        let word_e = get_float(cmd, "E")?;
        let word_f = get_float(cmd, "F")?;
        if let Some(fv) = word_f {
            // The synthesized G1s would fail this on the first chord
            // (gcode_move.py:152-156); check up front so no partial
            // state change occurs.
            if fv <= 0.0 {
                return Err(StateError::InvalidSpeed { value: fv });
            }
        }
        let segments = plan_arc(&ArcRequest {
            current,
            target: [target_x, target_y, target_z],
            offset,
            plane: self.arc_plane,
            clockwise,
            absolute_extrude: self.absolute_extrude,
            e_param: word_e,
            f_param: word_f,
            resolution: self.arc_resolution,
        })?;
        let count = u32::try_from(segments.len()).unwrap_or(u32::MAX);
        let mut moves = Vec::new();
        for (idx, seg) in segments.iter().enumerate() {
            let info = ArcSegmentInfo {
                index: u32::try_from(idx).unwrap_or(u32::MAX).saturating_add(1),
                count,
            };
            let mv = self.apply_g1_values(
                [
                    Some(seg.target[0]),
                    Some(seg.target[1]),
                    Some(seg.target[2]),
                    seg.e,
                ],
                seg.f,
                span,
                Some(info),
            );
            if let Some(m) = mv {
                moves.push(m);
            }
        }
        Ok(ApplyOutcome {
            moves,
            disposition: Disposition::Arc { segments: count },
        })
    }

    // --- G28 (extras/homing.py:337-343 + gcode_move.py:79-82) ---

    fn cmd_g28(&mut self, cmd: &Command) -> ApplyOutcome {
        let mut axes = [
            cmd.get("X").is_some(),
            cmd.get("Y").is_some(),
            cmd.get("Z").is_some(),
        ];
        if axes == [false; 3] {
            axes = [true; 3];
        }
        for (idx, homed) in axes.iter().enumerate() {
            if *homed {
                // `_handle_home_rails_end`: base_position[axis] =
                // homing_position[axis]; the real landing position is
                // config-dependent, so mark it unknown.
                let homing = self.homing_position.get(idx).copied().unwrap_or(0.0);
                if let Some(b) = self.base_position.get_mut(idx) {
                    *b = homing;
                }
                if let Some(bk) = self.base_known.get_mut(idx) {
                    *bk = true;
                }
                if let Some(pk) = self.position_known.get_mut(idx) {
                    *pk = false;
                }
            }
        }
        ApplyOutcome::empty(Disposition::Homing { axes })
    }

    // --- G92 (gcode_move.py:181-190) ---

    fn cmd_g92(&mut self, cmd: &Command) -> Result<ApplyOutcome, StateError> {
        let offsets = [
            get_float(cmd, "X")?,
            get_float(cmd, "Y")?,
            get_float(cmd, "Z")?,
            get_float(cmd, "E")?,
        ];
        if offsets == [None; 4] {
            // Bare G92 rebases every axis (gcode_move.py:189-190).
            self.base_position = self.last_position;
            self.base_known = self.position_known;
        } else {
            for (idx, off) in offsets.iter().enumerate() {
                if let Some(mut o) = *off {
                    if idx == 3 {
                        o *= self.extrude_factor;
                    }
                    let last = self.last_position.get(idx).copied().unwrap_or(0.0);
                    if let Some(b) = self.base_position.get_mut(idx) {
                        *b = last - o;
                    }
                    let known = self.position_known.get(idx).copied().unwrap_or(false);
                    if let Some(bk) = self.base_known.get_mut(idx) {
                        *bk = known;
                    }
                }
            }
        }
        Ok(ApplyOutcome::empty(Disposition::State))
    }

    // --- M204 (toolhead.py:590-602) ---

    fn cmd_m204(&mut self, cmd: &Command) -> Result<ApplyOutcome, StateError> {
        let s = get_float_above(cmd, "S", 0.0)?;
        if let Some(accel) = s {
            self.accel_override = Some(accel);
        } else {
            let p = get_float_above(cmd, "P", 0.0)?;
            let t = get_float_above(cmd, "T", 0.0)?;
            if let (Some(p), Some(t)) = (p, t) {
                self.accel_override = Some(p.min(t));
            }
            // Otherwise Klipper only responds "Invalid M204" and
            // changes nothing (toolhead.py:597-600).
        }
        Ok(ApplyOutcome::empty(Disposition::State))
    }

    // --- M220 (gcode_move.py:195-199) ---

    fn cmd_m220(&mut self, cmd: &Command) -> Result<ApplyOutcome, StateError> {
        let s = get_float_above(cmd, "S", 0.0)?.unwrap_or(100.0);
        let value = s / (60.0 * 100.0);
        self.speed = self.gcode_speed() * value;
        self.speed_factor = value;
        Ok(ApplyOutcome::empty(Disposition::State))
    }

    // --- M221 (gcode_move.py:200-206) ---

    fn cmd_m221(&mut self, cmd: &Command) -> Result<ApplyOutcome, StateError> {
        let s = get_float_above(cmd, "S", 0.0)?.unwrap_or(100.0);
        let new_extrude_factor = s / 100.0;
        let last_e_pos = self.last_position[3];
        let e_value = (last_e_pos - self.base_position[3]) / self.extrude_factor;
        self.base_position[3] = last_e_pos - e_value * new_extrude_factor;
        self.extrude_factor = new_extrude_factor;
        // base[3] was recomputed from last[3]; it is only as reliable as
        // both inputs.
        self.base_known[3] = self.base_known[3] && self.position_known[3];
        Ok(ApplyOutcome::empty(Disposition::State))
    }

    // --- SET_GCODE_OFFSET (gcode_move.py:207-226) ---

    fn cmd_set_gcode_offset(
        &mut self,
        cmd: &Command,
        span: ByteSpan,
    ) -> Result<ApplyOutcome, StateError> {
        ensure_wellformed_extended(cmd)?;
        let mut move_delta = [0.0_f64; 4];
        for (pos, axis) in ["X", "Y", "Z", "E"].iter().enumerate() {
            let homing = self.homing_position.get(pos).copied().unwrap_or(0.0);
            let offset = if let Some(o) = get_float(cmd, axis)? {
                o
            } else {
                let adjust_key = format!("{axis}_ADJUST");
                let Some(adj) = get_float(cmd, &adjust_key)? else {
                    continue;
                };
                adj + homing
            };
            let delta = offset - homing;
            if let Some(d) = move_delta.get_mut(pos) {
                *d = delta;
            }
            if let Some(b) = self.base_position.get_mut(pos) {
                *b += delta;
            }
            if let Some(h) = self.homing_position.get_mut(pos) {
                *h = offset;
            }
        }
        let mut moves = Vec::new();
        if get_int(cmd, "MOVE")?.unwrap_or(0) != 0 {
            let speed = get_float_above(cmd, "MOVE_SPEED", 0.0)?.unwrap_or(self.speed);
            let start = self.last_position;
            let start_known = self.position_known;
            for (slot, d) in self.last_position.iter_mut().zip(move_delta) {
                *slot += d;
            }
            if let Some(m) = self.emit_move(start, start_known, speed, span, None) {
                moves.push(m);
            }
        }
        Ok(ApplyOutcome {
            moves,
            disposition: Disposition::State,
        })
    }

    // --- SAVE_GCODE_STATE (gcode_move.py:228-238) ---

    fn cmd_save_gcode_state(&mut self, cmd: &Command) -> Result<ApplyOutcome, StateError> {
        ensure_wellformed_extended(cmd)?;
        let name = cmd.get("NAME").unwrap_or("default").to_string();
        self.saved_states.insert(
            name,
            SavedGcodeState {
                absolute_coord: self.absolute_coord,
                absolute_extrude: self.absolute_extrude,
                base_position: self.base_position,
                last_position: self.last_position,
                homing_position: self.homing_position,
                speed: self.speed,
                speed_factor: self.speed_factor,
                extrude_factor: self.extrude_factor,
                base_known: self.base_known,
                position_known: self.position_known,
            },
        );
        Ok(ApplyOutcome::empty(Disposition::State))
    }

    // --- RESTORE_GCODE_STATE (gcode_move.py:240-260) ---

    fn cmd_restore_gcode_state(
        &mut self,
        cmd: &Command,
        span: ByteSpan,
    ) -> Result<ApplyOutcome, StateError> {
        ensure_wellformed_extended(cmd)?;
        let name = cmd.get("NAME").unwrap_or("default").to_string();
        let Some(state) = self.saved_states.get(&name).cloned() else {
            return Err(StateError::UnknownSavedState { name });
        };
        self.absolute_coord = state.absolute_coord;
        self.absolute_extrude = state.absolute_extrude;
        self.base_position = state.base_position;
        self.homing_position = state.homing_position;
        self.speed = state.speed;
        self.speed_factor = state.speed_factor;
        self.extrude_factor = state.extrude_factor;
        self.base_known = state.base_known;
        // Restore the relative E position (gcode_move.py:253-255).
        let e_diff = self.last_position[3] - state.last_position[3];
        self.base_position[3] += e_diff;
        self.base_known[3] =
            state.base_known[3] && self.position_known[3] && state.position_known[3];
        let mut moves = Vec::new();
        if get_int(cmd, "MOVE")?.unwrap_or(0) != 0 {
            let speed = get_float_above(cmd, "MOVE_SPEED", 0.0)?.unwrap_or(self.speed);
            let start = self.last_position;
            let start_known = self.position_known;
            self.last_position[0] = state.last_position[0];
            self.last_position[1] = state.last_position[1];
            self.last_position[2] = state.last_position[2];
            self.position_known[0] = state.position_known[0];
            self.position_known[1] = state.position_known[1];
            self.position_known[2] = state.position_known[2];
            if let Some(m) = self.emit_move(start, start_known, speed, span, None) {
                moves.push(m);
            }
        }
        Ok(ApplyOutcome {
            moves,
            disposition: Disposition::State,
        })
    }
}

/// Reject extended commands whose shlex parse failed; Klipper raises
/// "Malformed command" before the handler runs (gcode.py:275-277).
fn ensure_wellformed_extended(cmd: &Command) -> Result<(), StateError> {
    if cmd.is_malformed_extended() {
        return Err(StateError::MalformedExtended {
            command: cmd.name.clone(),
        });
    }
    Ok(())
}

/// `gcmd.get_float(key, None)`: absent → `None`; unparseable → error.
/// Non-finite values are rejected (safety divergence; module docs).
fn get_float(cmd: &Command, key: &str) -> Result<Option<f64>, StateError> {
    match cmd.get(key) {
        None => Ok(None),
        Some(raw) => {
            let v: f64 = raw.trim().parse().map_err(|_| StateError::InvalidParam {
                command: cmd.name.clone(),
                key: key.to_string(),
                value: raw.to_string(),
            })?;
            if !v.is_finite() {
                return Err(StateError::NonFiniteParam {
                    command: cmd.name.clone(),
                    key: key.to_string(),
                });
            }
            Ok(Some(v))
        }
    }
}

/// `gcmd.get_float(key, ..., above=min)` (gcode.py:84-86): the parsed
/// value must be strictly greater than `min`.
fn get_float_above(cmd: &Command, key: &str, min: f64) -> Result<Option<f64>, StateError> {
    match get_float(cmd, key)? {
        None => Ok(None),
        Some(v) => {
            if v <= min {
                return Err(StateError::ParamNotAbove {
                    command: cmd.name.clone(),
                    key: key.to_string(),
                    min,
                });
            }
            Ok(Some(v))
        }
    }
}

/// `gcmd.get_int(key, None)`.
fn get_int(cmd: &Command, key: &str) -> Result<Option<i64>, StateError> {
    match cmd.get(key) {
        None => Ok(None),
        Some(raw) => raw
            .trim()
            .parse::<i64>()
            .map(Some)
            .map_err(|_| StateError::InvalidParam {
                command: cmd.name.clone(),
                key: key.to_string(),
                value: raw.to_string(),
            }),
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact equivalence with Klipper arithmetic is the point
mod tests {
    use super::*;
    use crate::parse::{parse_line, ByteSpan};

    fn line(s: &str) -> Line {
        parse_line(
            s.as_bytes(),
            ByteSpan {
                start: 0,
                end: s.len() as u64,
            },
        )
    }

    fn apply(state: &mut GcodeState, s: &str) -> ApplyOutcome {
        state
            .apply(&line(s))
            .unwrap_or_else(|e| panic!("apply {s:?}: {e}"))
    }

    fn apply_err(state: &mut GcodeState, s: &str) -> StateError {
        state.apply(&line(s)).expect_err("expected an error")
    }

    #[test]
    fn defaults_match_klipper_init() {
        // gcode_move.py:29-36.
        let s = GcodeState::new();
        assert!(s.absolute_coord && s.absolute_extrude);
        assert_eq!(s.speed, 25.0);
        assert_eq!(s.speed_factor, 1.0 / 60.0);
        assert_eq!(s.extrude_factor, 1.0);
        assert_eq!(s.speed_factor_override(), 1.0);
    }

    #[test]
    fn g1_absolute_uses_base_position() {
        // gcode_move.py:149-151: absolute value lands at v + base.
        let mut s = GcodeState::new();
        s.base_position = [1.0, 2.0, 3.0, 0.0];
        let out = apply(&mut s, "G1 X10 Y20 Z0.4");
        assert_eq!(s.last_position, [11.0, 22.0, 3.4, 0.0]);
        assert_eq!(out.moves.len(), 1);
        assert_eq!(out.disposition, Disposition::Move);
    }

    #[test]
    fn g1_relative_adds() {
        // gcode_move.py:146-148.
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X10");
        apply(&mut s, "G91");
        apply(&mut s, "G1 X-3 Z1");
        assert_eq!(s.last_position[0], 7.0);
        assert_eq!(s.last_position[2], 1.0);
    }

    #[test]
    fn g0_is_g1() {
        // gcode.py registration: G0 -> cmd_G1 (gcode_move.py:23).
        let mut s = GcodeState::new();
        let out = apply(&mut s, "G0 X5");
        assert_eq!(out.disposition, Disposition::Move);
        assert_eq!(s.last_position[0], 5.0);
    }

    #[test]
    fn e_scaled_by_extrude_factor_and_mode() {
        // gcode_move.py:142-145: E is factor-scaled; absolute only when
        // both absolute_coord and absolute_extrude hold.
        let mut s = GcodeState::new();
        apply(&mut s, "M221 S200"); // extrude_factor = 2.0
        apply(&mut s, "M83");
        apply(&mut s, "G1 E5"); // relative: += 5 * 2.0
        assert_eq!(s.last_position[3], 10.0);
        apply(&mut s, "M82");
        apply(&mut s, "G1 E5"); // absolute: = 5*2.0 + base[3]
        assert_eq!(s.last_position[3], 10.0 + s.base_position[3]);
    }

    #[test]
    fn m82_with_g91_is_still_relative_e() {
        // gcode_move.py:141-145: `absolute_coord` gates E as well.
        let mut s = GcodeState::new();
        apply(&mut s, "M82");
        apply(&mut s, "G91");
        apply(&mut s, "G1 E2");
        apply(&mut s, "G1 E2");
        assert_eq!(s.last_position[3], 4.0, "E must accumulate under G91");
    }

    #[test]
    fn f_word_scales_by_speed_factor_and_persists() {
        // gcode_move.py:152-157: internal speed = F * speed_factor.
        let mut s = GcodeState::new();
        apply(&mut s, "G1 F9000");
        assert_eq!(s.speed, 9000.0 / 60.0);
        assert_eq!(s.gcode_speed(), 9000.0);
        // Speed persists across moves.
        let out = apply(&mut s, "G1 X10");
        assert_eq!(out.moves[0].speed, 150.0);
    }

    #[test]
    fn f_only_line_moves_nothing() {
        let mut s = GcodeState::new();
        let out = apply(&mut s, "G1 F1200");
        assert!(out.moves.is_empty());
        assert_eq!(out.disposition, Disposition::Move);
    }

    #[test]
    fn invalid_speed_rejected() {
        // gcode_move.py:152-156.
        let mut s = GcodeState::new();
        assert!(matches!(
            apply_err(&mut s, "G1 X1 F0"),
            StateError::InvalidSpeed { .. }
        ));
        assert!(matches!(
            apply_err(&mut s, "G1 F-100"),
            StateError::InvalidSpeed { .. }
        ));
        // State unchanged by the failed line.
        assert_eq!(s.last_position, [0.0; 4]);
    }

    #[test]
    fn unparseable_and_nonfinite_params_rejected() {
        let mut s = GcodeState::new();
        // Letters in a traditional value merge into the key run under
        // Klipper's tokenizer, so an unparseable traditional value must
        // be non-alphabetic.
        assert!(matches!(
            apply_err(&mut s, "G1 X..5"),
            StateError::InvalidParam { .. }
        ));
        // Non-finite values can only arrive via extended commands
        // (traditional values cannot contain letters at all).
        assert!(matches!(
            apply_err(&mut s, "SET_GCODE_OFFSET Z=nan"),
            StateError::NonFiniteParam { .. }
        ));
        assert!(matches!(
            apply_err(&mut s, "SET_GCODE_OFFSET Z=inf"),
            StateError::NonFiniteParam { .. }
        ));
        // And "G1 Xnan" merges into key "XNAN": the X word simply never
        // exists, matching Klipper.
        let out = apply(&mut s, "G1 Xnan");
        assert!(out.moves.is_empty());
        assert_eq!(s.last_position, [0.0; 4]);
    }

    #[test]
    fn g92_shifts_base_position_only() {
        // gcode_move.py:181-188.
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X30 Y40 E5");
        let last_before = s.last_position;
        apply(&mut s, "G92 E0");
        assert_eq!(s.last_position, last_before, "G92 must not move anything");
        assert_eq!(s.base_position[3], 5.0);
        assert_eq!(s.gcode_position()[3], 0.0);
        // XY bases untouched.
        assert_eq!(s.base_position[0], 0.0);
    }

    #[test]
    fn g92_e_offset_scaled_by_extrude_factor() {
        // gcode_move.py:186-187: `offset *= self.extrude_factor`.
        let mut s = GcodeState::new();
        apply(&mut s, "G1 E10");
        apply(&mut s, "M221 S50"); // factor 0.5
        apply(&mut s, "G92 E4");
        // base[3] = last[3] - 4 * 0.5 = 10 - 2 = 8.
        assert_eq!(s.base_position[3], 8.0);
        // And the g-code E position now reads 4.
        assert_eq!(s.gcode_position()[3], 4.0);
    }

    #[test]
    fn bare_g92_rebases_all_axes() {
        // gcode_move.py:189-190.
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X7 Y8 Z9 E10");
        apply(&mut s, "G92");
        assert_eq!(s.base_position, s.last_position);
        assert_eq!(s.gcode_position(), [0.0; 4]);
    }

    #[test]
    fn m220_preserves_gcode_speed_and_positions() {
        // gcode_move.py:195-199.
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X10 F6000");
        let before = s.clone();
        apply(&mut s, "M220 S150");
        assert_eq!(s.last_position, before.last_position);
        assert_eq!(s.base_position, before.base_position);
        assert_eq!(s.speed_factor, 1.5 / 60.0);
        // Internal speed rescaled so g-code speed is preserved.
        assert_eq!(s.gcode_speed(), before.gcode_speed());
        assert_eq!(s.speed, before.speed * 1.5);
        // The next F word lands under the new factor.
        apply(&mut s, "G1 F6000");
        assert_eq!(s.speed, 6000.0 * 1.5 / 60.0);
    }

    #[test]
    fn m220_default_is_100() {
        let mut s = GcodeState::new();
        apply(&mut s, "M220 S50");
        apply(&mut s, "M220");
        assert_eq!(s.speed_factor, 1.0 / 60.0);
    }

    #[test]
    fn m221_preserves_gcode_e_position() {
        // gcode_move.py:200-206: e_value = (last - base) / old_factor;
        // base = last - e_value * new_factor.
        let mut s = GcodeState::new();
        apply(&mut s, "G1 E10");
        let e_gcode_before = s.gcode_position()[3];
        apply(&mut s, "M221 S73");
        assert_eq!(s.extrude_factor, 0.73);
        assert_eq!(
            s.gcode_position()[3],
            e_gcode_before,
            "M221 must not change the g-code E reading"
        );
        assert_eq!(s.base_position[3], 10.0 - 10.0 * 0.73);
        // And positions are untouched.
        assert_eq!(s.last_position[3], 10.0);
    }

    #[test]
    fn m221_then_moves_then_g92_e0_matches_klipper_composition() {
        // Composite check used by the E-frame matcher: M221, extrusion,
        // G92 E0 chained together stay consistent with the closed form.
        let mut s = GcodeState::new();
        apply(&mut s, "M221 S95");
        apply(&mut s, "G1 E10");
        assert_eq!(s.last_position[3], 9.5);
        apply(&mut s, "G92 E0");
        assert_eq!(s.base_position[3], 9.5);
        apply(&mut s, "G1 E2");
        assert_eq!(s.last_position[3], 2.0f64.mul_add(0.95, 9.5));
        // Same float rounding Klipper's division would produce.
        assert!((s.gcode_position()[3] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn factor_bounds_enforced() {
        let mut s = GcodeState::new();
        assert!(matches!(
            apply_err(&mut s, "M220 S0"),
            StateError::ParamNotAbove { .. }
        ));
        assert!(matches!(
            apply_err(&mut s, "M221 S-5"),
            StateError::ParamNotAbove { .. }
        ));
    }

    #[test]
    fn set_gcode_offset_accumulates_and_adjusts() {
        // gcode_move.py:207-220.
        let mut s = GcodeState::new();
        apply(&mut s, "SET_GCODE_OFFSET Z=0.2");
        assert_eq!(s.homing_position[2], 0.2);
        assert_eq!(s.base_position[2], 0.2);
        // _ADJUST is relative to the current offset.
        apply(&mut s, "SET_GCODE_OFFSET Z_ADJUST=-0.05");
        assert!((s.homing_position[2] - 0.15).abs() < 1e-12);
        assert!((s.base_position[2] - 0.15).abs() < 1e-12);
        // Positions unchanged without MOVE=1.
        assert_eq!(s.last_position, [0.0; 4]);
    }

    #[test]
    fn set_gcode_offset_move_emits_compensating_move() {
        // gcode_move.py:221-226.
        let mut s = GcodeState::new();
        let out = apply(&mut s, "SET_GCODE_OFFSET Z=0.3 MOVE=1 MOVE_SPEED=10");
        assert_eq!(out.moves.len(), 1);
        let m = &out.moves[0];
        assert_eq!(m.end[2] - m.start[2], 0.3);
        // MOVE_SPEED is raw mm/s, not speed-factor scaled.
        assert_eq!(m.speed, 10.0);
        // The persistent speed is untouched.
        assert_eq!(s.speed, 25.0);
        assert_eq!(s.last_position[2], 0.3);
    }

    #[test]
    fn save_restore_gcode_state_round_trip() {
        // gcode_move.py:228-260.
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X10 Y10 Z1 E5 F3000");
        apply(&mut s, "SAVE_GCODE_STATE NAME=pause");
        apply(&mut s, "G91");
        apply(&mut s, "M83");
        apply(&mut s, "G1 X5 E-2");
        apply(&mut s, "M220 S200");
        let out = apply(&mut s, "RESTORE_GCODE_STATE NAME=pause MOVE=1");
        assert!(s.absolute_coord && s.absolute_extrude);
        assert_eq!(s.speed_factor, 1.0 / 60.0);
        // XYZ restored by the compensating move; E is rebased, not moved
        // (gcode_move.py:253-255).
        assert_eq!(s.last_position[0], 10.0);
        assert_eq!(s.last_position[3], 3.0, "E stays where it was");
        assert_eq!(s.gcode_position()[3], 5.0, "but reads as the saved E");
        assert_eq!(out.moves.len(), 1);
    }

    #[test]
    fn restore_unknown_state_errors() {
        let mut s = GcodeState::new();
        assert!(matches!(
            apply_err(&mut s, "RESTORE_GCODE_STATE NAME=nope"),
            StateError::UnknownSavedState { .. }
        ));
    }

    #[test]
    fn g28_marks_axes_unknown_and_rebases() {
        let mut s = GcodeState::new();
        apply(&mut s, "SET_GCODE_OFFSET Z=0.2");
        apply(&mut s, "G1 X50 Y50 Z5");
        let out = apply(&mut s, "G28 Z");
        assert_eq!(
            out.disposition,
            Disposition::Homing {
                axes: [false, false, true]
            }
        );
        assert!(!s.position_known[2]);
        assert!(s.position_known[0] && s.position_known[1]);
        // base_position[2] = homing_position[2] (gcode_move.py:79-82).
        assert_eq!(s.base_position[2], 0.2);
        // An absolute Z move restores knowledge.
        apply(&mut s, "G1 Z1");
        assert!(s.position_known[2]);
        assert_eq!(s.last_position[2], 1.2);
        // A bare G28 homes everything.
        let out = apply(&mut s, "G28");
        assert_eq!(out.disposition, Disposition::Homing { axes: [true; 3] });
        assert_eq!(s.position_known, [false, false, false, true]);
    }

    #[test]
    fn relative_move_after_g28_stays_unknown() {
        let mut s = GcodeState::new();
        apply(&mut s, "G28 X");
        apply(&mut s, "G91");
        apply(&mut s, "G1 X5");
        assert!(!s.position_known[0]);
        apply(&mut s, "G90");
        apply(&mut s, "G1 X5");
        assert!(s.position_known[0]);
    }

    #[test]
    fn g20_errors_g21_noop() {
        let mut s = GcodeState::new();
        assert!(matches!(
            apply_err(&mut s, "G20"),
            StateError::InchesUnsupported
        ));
        let out = apply(&mut s, "G21");
        assert_eq!(out.disposition, Disposition::State);
    }

    #[test]
    fn m204_variants() {
        // toolhead.py:590-601.
        let mut s = GcodeState::new();
        apply(&mut s, "M204 S1500");
        assert_eq!(s.accel_override, Some(1500.0));
        apply(&mut s, "M204 P2000 T1000");
        assert_eq!(s.accel_override, Some(1000.0));
        // P without T: ignored, no change.
        apply(&mut s, "M204 P900");
        assert_eq!(s.accel_override, Some(1000.0));
        assert!(matches!(
            apply_err(&mut s, "M204 S-1"),
            StateError::ParamNotAbove { .. }
        ));
        // Moves carry the override.
        let out = apply(&mut s, "G1 X10");
        assert_eq!(out.moves[0].accel_override, Some(1000.0));
    }

    #[test]
    fn unknown_commands_pass_through_unchanged() {
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X10 E1");
        let before = s.clone();
        for cmdline in [
            "M104 S210",
            "M106 S255",
            "M117 Hello World",
            "EXCLUDE_OBJECT_START NAME=x",
            "G29",
            "M900 K0.05",
        ] {
            let out = apply(&mut s, cmdline);
            assert_eq!(out.disposition, Disposition::PassThrough, "{cmdline}");
            assert!(out.moves.is_empty());
        }
        assert_eq!(s, before, "pass-through must not mutate state");
    }

    #[test]
    fn blank_and_comment_lines_are_noops() {
        let mut s = GcodeState::new();
        for l in ["", "   ", "; comment", ";TYPE:Skirt"] {
            let out = apply(&mut s, l);
            assert_eq!(out.disposition, Disposition::Blank);
        }
    }

    #[test]
    fn arc_requires_absolute_mode_and_offsets() {
        // gcode_arcs.py:62-63, 72-73, 89-90.
        let mut s = GcodeState::new();
        apply(&mut s, "G91");
        assert!(matches!(
            apply_err(&mut s, "G2 X1 Y1 I1"),
            StateError::Arc(ArcError::RelativeMode)
        ));
        apply(&mut s, "G90");
        assert!(matches!(
            apply_err(&mut s, "G2 X1 Y1 R5"),
            StateError::Arc(ArcError::RadiusForm)
        ));
        assert!(matches!(
            apply_err(&mut s, "G2 X1 Y1"),
            StateError::Arc(ArcError::MissingOffsets)
        ));
        assert!(matches!(
            apply_err(&mut s, "G2 X1 Y1 I0 J0"),
            StateError::Arc(ArcError::MissingOffsets)
        ));
    }

    #[test]
    fn arc_decomposes_and_advances_state() {
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X10 Y0 Z0.4 F3000");
        let out = apply(&mut s, "G3 X0 Y10 I-10 E3 F1800");
        let Disposition::Arc { segments } = out.disposition else {
            panic!("expected arc disposition");
        };
        assert_eq!(segments, 15);
        assert_eq!(out.moves.len(), 15);
        // Final XYZ is exactly the target (planArc line 169-170); E
        // accumulates per-chord as Klipper does, converging within
        // float rounding.
        assert_eq!(s.last_position[0], 0.0);
        assert_eq!(s.last_position[1], 10.0);
        assert!((s.last_position[3] - 3.0).abs() < 1e-9);
        // F was applied through the chords.
        assert_eq!(s.speed, 1800.0 / 60.0);
        // Chord bookkeeping.
        let first = &out.moves[0];
        assert_eq!(
            first.arc_segment,
            Some(ArcSegmentInfo {
                index: 1,
                count: 15
            })
        );
        // All chords share the source span.
        assert!(out.moves.iter().all(|m| m.span == first.span));
    }

    #[test]
    fn arc_e_respects_extrude_factor_via_g1_path() {
        // Chorded E goes through the normal G1 scaling
        // (gcode_arcs.py:179-180 -> gcode_move.py:142-143).
        let mut s = GcodeState::new();
        apply(&mut s, "M221 S50");
        apply(&mut s, "G1 X10 Y0");
        apply(&mut s, "G2 X-10 Y0 I-10 E4");
        // g-code E reads 4; internal E is 2.
        assert!((s.gcode_position()[3] - 4.0).abs() < 1e-9);
        assert!((s.last_position[3] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn arc_plane_selection_applies() {
        let mut s = GcodeState::new();
        apply(&mut s, "G18");
        assert_eq!(s.arc_plane, ArcPlane::Xz);
        apply(&mut s, "G1 X10 Y0 Z0");
        // XZ-plane arc: I/K offsets; Y helical.
        let out = apply(&mut s, "G2 X0 Z10 I-10 K0");
        assert!(matches!(out.disposition, Disposition::Arc { .. }));
        assert_eq!(s.last_position[2], 10.0);
        apply(&mut s, "G19");
        assert_eq!(s.arc_plane, ArcPlane::Yz);
        apply(&mut s, "G17");
        assert_eq!(s.arc_plane, ArcPlane::Xy);
    }

    #[test]
    fn zero_distance_moves_are_dropped() {
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X10");
        let out = apply(&mut s, "G1 X10"); // same position
        assert!(out.moves.is_empty());
        // Sub-threshold XYZ with no E is also dropped (toolhead rule).
        let out = apply(&mut s, "G91");
        assert!(out.moves.is_empty());
        let out = apply(&mut s, "G1 X0.0000000001");
        assert!(out.moves.is_empty());
    }

    #[test]
    fn extrude_only_move_classification() {
        let mut s = GcodeState::new();
        let out = apply(&mut s, "G1 E-0.8 F2100");
        assert_eq!(out.moves.len(), 1);
        let m = &out.moves[0];
        assert!(m.is_extrude_only());
        assert!(!m.extrudes());
        assert_eq!(m.kinematic_distance(), 0.8);
        assert_eq!(m.kinematic_end(), [0.0, 0.0, 0.0, -0.8]);
    }

    #[test]
    fn malformed_extended_rejected_for_known_commands() {
        let mut s = GcodeState::new();
        assert!(matches!(
            apply_err(&mut s, "SET_GCODE_OFFSET Z=\"unterminated"),
            StateError::MalformedExtended { .. }
        ));
        // Unknown commands with malformed params still pass through
        // (Klipper only shlex-parses registered extended handlers).
        let out = apply(&mut s, "SOME_MACRO Z=\"unterminated");
        assert_eq!(out.disposition, Disposition::PassThrough);
    }

    #[test]
    fn get_int_error_path() {
        let mut s = GcodeState::new();
        assert!(matches!(
            apply_err(&mut s, "SET_GCODE_OFFSET Z=1 MOVE=1.5"),
            StateError::InvalidParam { .. }
        ));
    }

    #[test]
    fn planned_move_helpers() {
        let mut s = GcodeState::new();
        let out = apply(&mut s, "G1 X3 Y4 E2 F6000");
        let m = &out.moves[0];
        assert_eq!(m.axes_delta(), [3.0, 4.0, 0.0, 2.0]);
        assert_eq!(m.xyz_distance(), 5.0);
        assert!(!m.is_extrude_only());
        assert!(m.extrudes());
        assert_eq!(m.kinematic_end(), m.end);
    }

    #[test]
    fn state_serializes() {
        let mut s = GcodeState::new();
        apply(&mut s, "G1 X10 E2");
        apply(&mut s, "SAVE_GCODE_STATE NAME=a");
        let json = serde_json::to_string(&s).unwrap();
        let back: GcodeState = serde_json::from_str(&json).unwrap();
        assert_eq!(s, back);
    }
}
