//! Typed WAL records.
//!
//! Field shapes mirror what Klipper actually emits over its API socket so
//! that the daemon can journal subscription payloads without lossy
//! translation:
//!
//! - [`TrapqSegment`] mirrors `motion_report` `dump_trapq` rows
//!   (`klippy/extras/motion_report.py`, `DumpTrapQ._process_batch`):
//!   `time, duration, start_velocity, acceleration, start_position,
//!   direction` per move, sourced from the C `struct pull_move`.
//! - [`StepperRange`] mirrors `dump_stepper` batches
//!   (`DumpStepper._process_batch`): `(interval, count, add)` step chunks
//!   plus the clock/position framing fields.
//! - [`Context`] snapshots `virtual_sdcard`, `gcode_move` status, active
//!   move-transform observations, heater/fan targets, and the
//!   `exclude_object` cancellation state ([`ExcludeState`]).
//!
//! Every record carries a host-monotonic capture timestamp (`mono_ns`).
//! Cross-domain time correlation (`print_time` ↔ monotonic ↔ wall clock)
//! is carried by [`Heartbeat`], which pairs a monotonic sample with
//! Klipper's `estimated_print_time` at that sample.

use serde::{Deserialize, Serialize};

/// One trapezoidal move segment extracted from a Klipper trapq.
///
/// A segment describes motion over `[print_time, print_time + duration]`:
///
/// ```text
/// dist(t) = (start_velocity + 0.5 * acceleration * t) * t
/// pos(t)  = start + direction_ratio * dist(t)      (per axis)
/// ```
///
/// where `t` is seconds since `print_time`. This is exactly the evaluation
/// Klipper's `motion_report.DumpTrapQ.get_trapq_position` performs.
///
/// Segments are **not** assumed contiguous: dwells (heating, waiting,
/// homing) appear as gaps between one segment's end time and the next
/// segment's `print_time`. Consumers must treat time discontinuities as
/// legitimate.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrapqSegment {
    /// Host-monotonic time (nanoseconds) when the daemon captured this row.
    pub mono_ns: u64,
    /// Which motion queue this segment came from: `"toolhead"`,
    /// `"extruder"`, `"extruder1"`, `"manual_stepper …"`, etc. (the mux key
    /// of the `motion_report/dump_trapq` endpoint).
    pub queue: String,
    /// Segment start, in Klipper print time (seconds).
    pub print_time: f64,
    /// Segment duration in seconds (`move_t` in Klipper's `pull_move`).
    pub duration: f64,
    /// Velocity at `print_time`, in mm/s along the move direction.
    pub start_velocity: f64,
    /// Constant acceleration over the segment, mm/s².
    pub acceleration: f64,
    /// Start position, X axis (mm). For extruder queues Klipper stores the
    /// filament position in the X slot and zeros Y/Z.
    pub start_x: f64,
    /// Start position, Y axis (mm).
    pub start_y: f64,
    /// Start position, Z axis (mm).
    pub start_z: f64,
    /// Unit direction ratio, X component (`x_r` in `pull_move`).
    pub x_r: f64,
    /// Unit direction ratio, Y component.
    pub y_r: f64,
    /// Unit direction ratio, Z component.
    pub z_r: f64,
}

impl TrapqSegment {
    /// Print time at which this segment ends (`print_time + duration`).
    #[must_use]
    pub fn end_time(&self) -> f64 {
        self.print_time + self.duration
    }

    /// Position at `print_time` (seconds, Klipper print-time domain),
    /// clamped to the segment bounds — the same clamping
    /// `motion_report.get_trapq_position` applies.
    ///
    /// Returns `[x, y, z]` (for extruder queues, the filament position is
    /// in the first component).
    #[must_use]
    pub fn position_at(&self, print_time: f64) -> [f64; 3] {
        let t = clamp_elapsed(print_time - self.print_time, self.duration);
        let dist = (self.start_velocity + 0.5 * self.acceleration * t) * t;
        [
            self.start_x + self.x_r * dist,
            self.start_y + self.y_r * dist,
            self.start_z + self.z_r * dist,
        ]
    }

    /// Velocity magnitude at `print_time`, clamped to the segment bounds.
    #[must_use]
    pub fn velocity_at(&self, print_time: f64) -> f64 {
        let t = clamp_elapsed(print_time - self.print_time, self.duration);
        self.start_velocity + self.acceleration * t
    }

    /// `true` when every float field is finite (no NaN / infinity).
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        [
            self.print_time,
            self.duration,
            self.start_velocity,
            self.acceleration,
            self.start_x,
            self.start_y,
            self.start_z,
            self.x_r,
            self.y_r,
            self.z_r,
        ]
        .iter()
        .all(|value| value.is_finite())
    }
}

/// Clamps an elapsed-time offset into `[0, max(duration, 0)]` without ever
/// panicking (unlike `f64::clamp`, which panics when `min > max`).
fn clamp_elapsed(elapsed: f64, duration: f64) -> f64 {
    elapsed.max(0.0).min(duration.max(0.0))
}

/// One compressed step chunk from a `dump_stepper` batch: `|count|`
/// steps, the first after `interval` clock ticks, each subsequent
/// interval increasing by `add`.
///
/// Field widths and signs match the emitting source, Klipper's host-side
/// step history — `struct pull_history_steps` in
/// `klippy/chelper/stepcompress.h` declares `step_count`, `interval` and
/// `add` all as signed C `int` (i32). These are **not** the unsigned MCU
/// `queue_step` wire widths (`interval=%u count=%hu add=%hi`): the
/// history negates `count` for reverse-direction steps
/// (`klippy/chelper/stepcompress.c:372`) and stores the u32 interval in
/// an `int`, so all three can be negative in a real dump (every Z
/// lift/lower yields negative counts).
///
/// # WAL format compatibility
///
/// Chunks are serialized as JSON integers, so widening `u32/u16/i16` to
/// `i32` keeps every previously written record readable: all values the
/// old recorder could produce lie inside `i32` (positive wire values are
/// bounded by the C `int` the dump emits, and the old daemon-side
/// saturation bounds were subsets of `i32`). The reverse is not true —
/// records written after this change may contain negative `interval` /
/// `count` values that a pre-change reader would reject; readers must be
/// upgraded together with the recorder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct StepChunk {
    /// Clock ticks before the first step of this chunk, as the raw
    /// signed value from the dump; negative values are wrapped `u32`
    /// tick counts (reinterpret as `u32` to recover the ticks).
    pub interval: i32,
    /// Signed step count: `|count|` steps, negative when stepping in the
    /// reverse direction; `0` marks a `set_position` row.
    pub count: i32,
    /// Signed tick delta added to `interval` after every step.
    pub add: i32,
}

/// A batch of committed steps for one stepper, as emitted by
/// `motion_report/dump_stepper` (`DumpStepper._process_batch`).
///
/// The `[first_clock, last_clock]` range covers motion the MCU has already
/// been told to execute; `last_clock` across all steppers is the source of
/// truth for the committed-motion boundary `t_b`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StepperRange {
    /// Host-monotonic time (nanoseconds) when the daemon captured this
    /// batch.
    pub mono_ns: u64,
    /// Stepper name, e.g. `"stepper_x"` (the mux key of the
    /// `motion_report/dump_stepper` endpoint).
    pub stepper: String,
    /// Raw MCU clock of the first step in the batch (64-bit extended
    /// ticks).
    pub first_clock: u64,
    /// Raw MCU clock of the last step in the batch.
    pub last_clock: u64,
    /// `first_clock` converted to print time by Klipper (seconds).
    pub first_step_time: f64,
    /// `last_clock` converted to print time by Klipper (seconds).
    pub last_step_time: f64,
    /// Commanded axis position at `first_clock`, in mm
    /// (`mcu_to_commanded_position` of `start_mcu_position`).
    pub start_position: f64,
    /// Raw MCU step counter at `first_clock`.
    pub start_mcu_position: i64,
    /// Distance of one full step, mm (`get_step_dist`).
    pub step_distance: f64,
    /// The step chunks, in execution order.
    pub steps: Vec<StepChunk>,
}

impl StepperRange {
    /// `true` when every float field is finite (no NaN / infinity).
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        [
            self.first_step_time,
            self.last_step_time,
            self.start_position,
            self.step_distance,
        ]
        .iter()
        .all(|value| value.is_finite())
    }
}

/// `virtual_sdcard` progress: which file is printing and how far the
/// reader has advanced (fields from `virtual_sdcard.get_status`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct VirtualSdState {
    /// Absolute path of the file being printed.
    pub file_path: String,
    /// Byte offset the G-code reader has reached in that file.
    pub file_position: u64,
}

/// Snapshot of `gcode_move.get_status`: everything needed to re-enter the
/// G-code interpreter state on resume.
///
/// Coordinate vectors are variable-length (`X, Y, Z, E`, then any extra
/// axes) to match Klipper's `Coord` with extra-axis support.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GcodeState {
    /// `M220` speed factor as a multiplier (Klipper reports 1.0 = 100%).
    pub speed_factor: f64,
    /// Current requested feed rate as `get_status` emits it:
    /// `speed / speed_factor`, i.e. the raw G-code `F` value in
    /// **mm/min at 100% factor** — NOT mm/s (Klipper's Status Reference
    /// documents this field as mm/s; that is known-wrong). Recover the
    /// internal mm/s speed with `speed * speed_factor / 60.0`, exactly
    /// as `plr-reconstruct`'s anchor conversion does.
    pub speed: f64,
    /// `M221` extrude factor as a multiplier.
    pub extrude_factor: f64,
    /// `G90`/`G91` state: `true` when coordinates are absolute.
    pub absolute_coordinates: bool,
    /// `M82`/`M83` state: `true` when extrusion is absolute.
    pub absolute_extrude: bool,
    /// `SET_GCODE_OFFSET` / `G92`-derived homing origin, per axis (mm).
    pub homing_origin: Vec<f64>,
    /// Internal (post-transform) position, per axis (mm).
    pub position: Vec<f64>,
    /// G-code space position (what the file's coordinates refer to), per
    /// axis (mm).
    pub gcode_position: Vec<f64>,
}

impl GcodeState {
    /// `true` when every float field is finite (no NaN / infinity).
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        [self.speed_factor, self.speed, self.extrude_factor]
            .iter()
            .all(|value| value.is_finite())
            && self
                .homing_origin
                .iter()
                .chain(&self.position)
                .chain(&self.gcode_position)
                .all(|value| value.is_finite())
    }
}

/// Observations about move transforms active between `gcode_move` and the
/// toolhead. These are captured so recovery can decide whether replayed
/// coordinates need the same transforms re-armed before resuming.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TransformObservations {
    /// `true` when `bed_mesh` reports a loaded `mesh_matrix` (mesh leveling
    /// is actively transforming moves).
    pub bed_mesh_active: bool,
    /// Name of the loaded bed-mesh profile, if any.
    pub bed_mesh_profile: Option<String>,
    /// `z_thermal_adjust` enabled state, if the module is configured.
    pub z_thermal_adjust_enabled: Option<bool>,
    /// Current Z offset applied by `z_thermal_adjust` (mm), if configured.
    pub z_thermal_adjust_offset: Option<f64>,
    /// `true` when `skew_correction` has an active profile.
    pub skew_active: bool,
    /// Name of the active skew profile, if any.
    pub skew_profile: Option<String>,
}

impl TransformObservations {
    /// `true` when every float field is finite (no NaN / infinity).
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        self.z_thermal_adjust_offset.is_none_or(f64::is_finite)
    }
}

/// A heater setpoint at capture time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeaterTarget {
    /// Heater name, e.g. `"extruder"`, `"heater_bed"`.
    pub name: String,
    /// Target temperature, °C. `0.0` means off.
    pub target: f64,
}

/// A fan speed setting at capture time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FanTarget {
    /// Fan name, e.g. `"fan"`, `"fan_generic exhaust"`.
    pub name: String,
    /// Requested speed in `[0.0, 1.0]`.
    pub speed: f64,
}

/// Maximum number of outline vertices journaled verbatim for one
/// excluded-object candidate (see [`ExcludeObjectDef::polygon`]).
///
/// Slicers emit `EXCLUDE_OBJECT_DEFINE POLYGON=` as a convex hull or a
/// simplified footprint: PrusaSlicer/SuperSlicer/OrcaSlicer produce on
/// the order of 4–40 points per object. 128 leaves an order of magnitude
/// of headroom while bounding the payload — at ~40 JSON bytes per
/// `[x, y]` pair a capped outline costs ≲5 KB, and definitions are
/// journaled only when they change (see [`ExcludeState::definitions`]),
/// not in every [`Context`].
///
/// Outlines above the cap are **not** silently shortened: they are
/// replaced by their axis-aligned bounding box and flagged
/// [`PolygonFidelity::BoundingBox`].
pub const MAX_POLYGON_POINTS: usize = 128;

/// How faithfully [`ExcludeObjectDef::polygon`] represents the outline
/// Klipper reported.
///
/// Serialized internally tagged (`{"kind": "Exact"}`) so new variants
/// can be added without breaking the wire format; decoders built against
/// this version map unrecognized kinds to [`PolygonFidelity::Unknown`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum PolygonFidelity {
    /// Klipper reported no outline for this object, so
    /// [`ExcludeObjectDef::polygon`] is empty. `POLYGON=` is optional on
    /// `EXCLUDE_OBJECT_DEFINE`, and `EXCLUDE_OBJECT_START NAME=` auto-
    /// defines a name-only object
    /// (`klippy/extras/exclude_object.py`, `cmd_EXCLUDE_OBJECT_START`).
    #[default]
    Absent,
    /// [`ExcludeObjectDef::polygon`] is exactly the outline Klipper
    /// reported.
    Exact,
    /// The outline exceeded [`MAX_POLYGON_POINTS`];
    /// [`ExcludeObjectDef::polygon`] holds its axis-aligned bounding box
    /// (4 points, counter-clockwise from the min corner).
    ///
    /// A bounding box is a **superset** of the true outline, which is
    /// the conservative direction for the question this geometry exists
    /// to answer ("is this point on an object the operator cancelled?"):
    /// it over-reports contact with a cancelled part rather than
    /// under-reporting it.
    BoundingBox {
        /// Vertex count of the outline Klipper reported.
        source_points: u32,
    },
    /// Klipper reported an outline that cannot be used: a non-finite or
    /// malformed coordinate, or fewer than three points.
    /// [`ExcludeObjectDef::polygon`] is empty and point-in-object
    /// queries cannot answer for this object.
    Unusable {
        /// Vertex count of the outline Klipper reported.
        source_points: u32,
    },
    /// A fidelity written by a newer format revision; preserved as
    /// opaque. Never written by this version except when round-tripping.
    #[serde(other)]
    Unknown,
}

impl PolygonFidelity {
    /// `true` for [`PolygonFidelity::Absent`] (the serialization
    /// default, omitted from the JSON payload).
    #[must_use]
    pub const fn is_absent(&self) -> bool {
        matches!(self, Self::Absent)
    }

    /// `true` when the stored outline is not a verbatim copy of what
    /// Klipper reported — the caller must surface this rather than treat
    /// the geometry as exact.
    #[must_use]
    pub const fn is_degraded(self) -> bool {
        matches!(
            self,
            Self::BoundingBox { .. } | Self::Unusable { .. } | Self::Unknown
        )
    }
}

/// One object defined for the running print, as
/// `exclude_object.get_status()["objects"]` reports it
/// (`klippy/extras/exclude_object.py`, `cmd_EXCLUDE_OBJECT_DEFINE`
/// builds `{"name": ..., "center": [...], "polygon": [[...]]}`, with
/// `center` and `polygon` both optional).
///
/// Klipper upper-cases every object name it stores (`name.upper()` in
/// `cmd_EXCLUDE_OBJECT_DEFINE` and `cmd_EXCLUDE_OBJECT_START`); the
/// recorder journals names in that normalized form, and
/// [`ExcludeState::excluded`] uses the same normalization, so the two
/// can be compared directly.
///
/// Klipper also folds any *other* `KEY=VALUE` parameters of
/// `EXCLUDE_OBJECT_DEFINE` into the object dict (`obj.update(parameters)`);
/// those are slicer-specific metadata that recovery has no use for and
/// are deliberately not journaled.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExcludeObjectDef {
    /// Object name, upper-cased exactly as Klipper stores it.
    pub name: String,
    /// Object centre `[x, y]` in G-code coordinates (mm), when Klipper
    /// supplied a finite `CENTER=`. Klipper parses `CENTER=x,y` as
    /// `json.loads('[%s]' % center)`, so it may carry more than two
    /// components; only X and Y are journaled.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 2]>,
    /// Object outline as `[x, y]` points (mm), in Klipper's order.
    /// Empty when [`fidelity`](Self::fidelity) is
    /// [`Absent`](PolygonFidelity::Absent) or
    /// [`Unusable`](PolygonFidelity::Unusable); the bounding box when it
    /// is [`BoundingBox`](PolygonFidelity::BoundingBox).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub polygon: Vec<[f64; 2]>,
    /// What [`polygon`](Self::polygon) actually is. Always report this
    /// alongside any geometric answer derived from the outline.
    #[serde(default, skip_serializing_if = "PolygonFidelity::is_absent")]
    pub fidelity: PolygonFidelity,
}

impl ExcludeObjectDef {
    /// A name-only definition, as `EXCLUDE_OBJECT_START NAME=` produces.
    #[must_use]
    pub fn name_only(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            center: None,
            polygon: Vec::new(),
            fidelity: PolygonFidelity::Absent,
        }
    }

    /// Builds a definition from raw Klipper geometry, applying the
    /// normalization rules that keep the record honest and writable.
    ///
    /// `polygon` mirrors what a reader of Klipper's `objects` list (or
    /// of an `EXCLUDE_OBJECT_DEFINE POLYGON=` line) can produce:
    ///
    /// * `None` — no outline was supplied →
    ///   [`PolygonFidelity::Absent`].
    /// * `Some(Err(n))` — an outline of `n` points was supplied but is
    ///   unusable (a point with fewer than two components, or a
    ///   non-finite coordinate) → [`PolygonFidelity::Unusable`], empty
    ///   `polygon`. Points are never dropped individually: a partial
    ///   ring encloses a different region than the real one.
    /// * `Some(Ok(points))` with fewer than three points → also
    ///   [`PolygonFidelity::Unusable`]; a ring needs three vertices.
    /// * `Some(Ok(points))` above [`MAX_POLYGON_POINTS`] → the
    ///   axis-aligned bounding box, [`PolygonFidelity::BoundingBox`].
    /// * otherwise verbatim, [`PolygonFidelity::Exact`].
    ///
    /// A non-finite `center` must already have been rejected by the
    /// caller (pass `None`).
    #[must_use]
    pub fn normalized(
        name: String,
        center: Option<[f64; 2]>,
        polygon: Option<Result<Vec<[f64; 2]>, usize>>,
    ) -> Self {
        let (polygon, fidelity) = normalize_polygon(polygon);
        Self {
            name,
            center: center.filter(|[x, y]| x.is_finite() && y.is_finite()),
            polygon,
            fidelity,
        }
    }

    /// `true` when every stored coordinate is finite.
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        self.center
            .iter()
            .flatten()
            .chain(self.polygon.iter().flatten())
            .all(|value| value.is_finite())
    }
}

/// Applies the outline rules documented on [`ExcludeObjectDef::normalized`].
fn normalize_polygon(
    polygon: Option<Result<Vec<[f64; 2]>, usize>>,
) -> (Vec<[f64; 2]>, PolygonFidelity) {
    let Some(polygon) = polygon else {
        return (Vec::new(), PolygonFidelity::Absent);
    };
    let source_points = match &polygon {
        Ok(points) => points.len(),
        Err(reported) => *reported,
    };
    let source_points = u32::try_from(source_points).unwrap_or(u32::MAX);
    let Ok(points) = polygon else {
        return (Vec::new(), PolygonFidelity::Unusable { source_points });
    };
    if points.iter().flatten().any(|value| !value.is_finite()) || points.len() < 3 {
        return (Vec::new(), PolygonFidelity::Unusable { source_points });
    }
    if points.len() > MAX_POLYGON_POINTS {
        return (
            bounding_box(&points),
            PolygonFidelity::BoundingBox { source_points },
        );
    }
    (points, PolygonFidelity::Exact)
}

/// The axis-aligned bounding box of `points` as a four-point ring,
/// counter-clockwise from the minimum corner. `points` is non-empty and
/// all-finite by construction (see [`normalize_polygon`]).
fn bounding_box(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut min = [f64::INFINITY; 2];
    let mut max = [f64::NEG_INFINITY; 2];
    for point in points {
        for axis in 0..2 {
            min[axis] = min[axis].min(point[axis]);
            max[axis] = max[axis].max(point[axis]);
        }
    }
    vec![
        [min[0], min[1]],
        [max[0], min[1]],
        [max[0], max[1]],
        [min[0], max[1]],
    ]
}

/// The `exclude_object` cancellation state at capture time
/// (`klippy/extras/exclude_object.py`, `ExcludeObject.get_status`:
/// `objects`, `excluded_objects`, `current_object`).
///
/// # Why this is journaled at all
///
/// Klipper's copy is per-print runtime state held only in RAM
/// (`_reset_state`, and `_reset_file` on `virtual_sdcard:reset_file`).
/// An operator cancels an object because it detached, warped, or turned
/// into spaghetti; a power loss destroys Klipper's record of that, and a
/// resume that un-excludes the object drives the nozzle back into the
/// debris. The cancellation must therefore outlive the power loss.
///
/// # Payload strategy: definitions once, excluded set always
///
/// [`excluded`](Self::excluded) and [`current`](Self::current) are short
/// name lists and ride along in **every** [`Context`] that carries
/// exclude state, so the newest surviving context always yields the
/// complete excluded set even after a torn tail.
///
/// [`definitions`](Self::definitions) can be large (polygons) and is
/// therefore journaled **only when it changes** — in practice once, at
/// the top of the print when the slicer's `EXCLUDE_OBJECT_DEFINE` block
/// is processed, plus once more for each object that
/// `EXCLUDE_OBJECT_START` auto-defines and once after any
/// re-subscription. `None` means "unchanged since the previous context
/// that carried definitions"; `Some(vec![])` means "Klipper knows no
/// objects" (what `EXCLUDE_OBJECT_DEFINE RESET=1` produces). Because the
/// WAL is recovered as a durable *prefix*, a definitions record written
/// at the top of the print always survives a later power loss.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExcludeState {
    /// Object definitions, present only in the context that first
    /// observed them or observed them change. `None` = carry the
    /// previous context's definitions forward.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub definitions: Option<Vec<ExcludeObjectDef>>,
    /// Names of the objects the operator has cancelled, in Klipper's
    /// sorted order (`_exclude_object` keeps `excluded_objects` sorted).
    /// Authoritative and complete as of this context.
    pub excluded: Vec<String>,
    /// Name of the object currently being printed
    /// (`EXCLUDE_OBJECT_START`/`END`), `None` between objects — Klipper
    /// reports JSON `null` there.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub current: Option<String>,
}

impl ExcludeState {
    /// `true` when every coordinate in every carried definition is
    /// finite.
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        self.definitions
            .iter()
            .flatten()
            .all(ExcludeObjectDef::values_are_finite)
    }
}

/// Print-context snapshot: everything beyond raw motion needed to rebuild
/// a resumable state (file position, interpreter state, transforms,
/// thermal targets, cancelled objects).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Context {
    /// Host-monotonic time (nanoseconds) when this snapshot was taken.
    pub mono_ns: u64,
    /// `virtual_sdcard` file/position, when a file print is active.
    pub virtual_sdcard: Option<VirtualSdState>,
    /// `gcode_move` interpreter state.
    pub gcode: GcodeState,
    /// Active move-transform observations.
    pub transforms: TransformObservations,
    /// Heater targets at capture time.
    pub heaters: Vec<HeaterTarget>,
    /// Fan targets at capture time.
    pub fans: Vec<FanTarget>,
    /// `exclude_object` cancellation state, when the printer runs the
    /// module and the daemon has observed it. `None` means "not
    /// observed", **not** "nothing excluded" — see [`ExcludeState`] and
    /// the provenance handling in `plr-reconstruct`.
    ///
    /// Boxed deliberately: [`ExcludeState`] is the largest thing a
    /// `Context` carries, yet most records in a log are trapq segments
    /// and most contexts carry no exclude state at all. Inlining it
    /// would widen every [`WalRecord`] moved through the recorder's hot
    /// path by the size of a definition list. Reach the payload with
    /// `ctx.exclude.as_deref()`.
    ///
    /// # WAL format compatibility
    ///
    /// This field was added after the format's first records were
    /// written. It is `#[serde(default)]`, so every pre-change `Context`
    /// payload still decodes (yielding `None`), and
    /// `skip_serializing_if` keeps it out of the JSON entirely when
    /// absent, so a `Context` without exclude state serializes
    /// byte-identically to the pre-change format and a pre-change reader
    /// is unaffected. Readers that predate the field also tolerate its
    /// presence: `Context` does not use `deny_unknown_fields`, so an old
    /// decoder ignores the key.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclude: Option<Box<ExcludeState>>,
}

impl Context {
    /// `true` when every float field is finite (no NaN / infinity).
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        self.gcode.values_are_finite()
            && self.transforms.values_are_finite()
            && self.heaters.iter().all(|h| h.target.is_finite())
            && self.fans.iter().all(|f| f.speed.is_finite())
            && self
                .exclude
                .as_deref()
                .is_none_or(ExcludeState::values_are_finite)
    }
}

/// What kind of lifecycle event a [`Marker`] records.
///
/// Serialized internally tagged (`{"kind": "SocketLost"}`), so new
/// variants can be added without breaking the wire format; decoders built
/// against this version map unrecognized kinds to [`MarkerKind::Unknown`]
/// instead of failing.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum MarkerKind {
    /// The print ended or was cancelled cleanly; the WAL ends on purpose.
    CleanShutdown,
    /// The Klipper API socket dropped (e.g. `RESTART`); subscriptions are
    /// dead and motion data after this point was not observed.
    SocketLost,
    /// The daemon reconnected and re-established its subscriptions.
    Resubscribed,
    /// A known observation gap: data between the bounds was not captured.
    SubscriptionGap {
        /// Host-monotonic time (nanoseconds) when the gap began.
        start_mono_ns: u64,
        /// Host-monotonic time (nanoseconds) when the gap ended.
        end_mono_ns: u64,
    },
    /// A [`Context`] carrying an **exclude-object change** was dropped
    /// before it reached the log (WAL channel backpressure).
    ///
    /// The socket reader must never block — Klipper disconnects
    /// unresponsive clients — so context records are droppable. Markers
    /// are not: this one exists so the loss of a cancellation leaves
    /// hard evidence in the log even though the cancellation itself did
    /// not make it. Reconstruction must refuse to call the excluded set
    /// authoritative when this marker postdates the newest journaled
    /// exclude state.
    ExclusionUpdateLost,
    /// A marker kind written by a newer format revision; preserved as
    /// opaque. Never written by this version except when round-tripping.
    #[serde(other)]
    Unknown,
}

/// A durable lifecycle marker in the log.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Marker {
    /// Host-monotonic time (nanoseconds) when the event was observed.
    pub mono_ns: u64,
    /// The event itself.
    pub kind: MarkerKind,
}

/// Liveness + clock-correlation sample, written at ~10 Hz.
///
/// Proves "the daemon was alive and Klipper was executing at time `t_a`"
/// and carries the sample pair needed to correlate Klipper print time with
/// host clocks. Appears both as a WAL record and as the fixed-layout slot
/// in the heartbeat file (see [`crate::heartbeat`]).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Heartbeat {
    /// Monotonically increasing heartbeat counter (wrapping).
    pub sequence: u64,
    /// Host-monotonic time (nanoseconds) when this heartbeat was taken.
    pub mono_ns: u64,
    /// Wall-clock time (nanoseconds since the Unix epoch) at the same
    /// instant, for post-mortem human correlation. Not monotonic; NTP may
    /// step it.
    pub wall_ns: u64,
    /// Latest print time known from motion data (seconds).
    pub print_time: f64,
    /// Host-monotonic time (nanoseconds) at which
    /// [`est_sample_print_time`](Self::est_sample_print_time) was sampled.
    pub est_sample_mono_ns: u64,
    /// Klipper `estimated_print_time` at
    /// [`est_sample_mono_ns`](Self::est_sample_mono_ns) (seconds). The
    /// (monotonic, estimated print time) pair anchors the
    /// print-time ↔ host-time correlation.
    pub est_sample_print_time: f64,
    /// WAL append offset (bytes) at the moment this heartbeat was taken:
    /// everything before this offset was already handed to the OS.
    pub wal_offset: u64,
}

impl Heartbeat {
    /// `true` when every float field is finite (no NaN / infinity).
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        self.print_time.is_finite() && self.est_sample_print_time.is_finite()
    }
}

/// Stable one-byte tags identifying each record kind in the frame header.
///
/// The numeric values are part of the on-disk format; never renumber.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RecordKind {
    /// [`TrapqSegment`]
    TrapqSegment = 1,
    /// [`StepperRange`]
    StepperRange = 2,
    /// [`Context`]
    Context = 3,
    /// [`Marker`]
    Marker = 4,
    /// [`Heartbeat`]
    Heartbeat = 5,
}

impl RecordKind {
    /// The on-disk tag byte for this kind.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Maps an on-disk tag byte back to a kind; `None` for tags this
    /// version does not know.
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::TrapqSegment),
            2 => Some(Self::StepperRange),
            3 => Some(Self::Context),
            4 => Some(Self::Marker),
            5 => Some(Self::Heartbeat),
            _ => None,
        }
    }
}

/// Any record that can appear in the append-only log.
///
/// Serialized internally tagged: the payload JSON carries a `"type"` field
/// naming the variant, duplicated (deliberately) by the one-byte kind tag
/// in the binary frame header so frames can be classified without parsing
/// JSON.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum WalRecord {
    /// One trapq move segment.
    TrapqSegment(TrapqSegment),
    /// One batch of committed steps.
    StepperRange(StepperRange),
    /// A print-context snapshot.
    Context(Context),
    /// A lifecycle marker.
    Marker(Marker),
    /// A liveness/clock-correlation sample.
    Heartbeat(Heartbeat),
}

impl WalRecord {
    /// The frame-header tag for this record.
    #[must_use]
    pub const fn kind(&self) -> RecordKind {
        match self {
            Self::TrapqSegment(_) => RecordKind::TrapqSegment,
            Self::StepperRange(_) => RecordKind::StepperRange,
            Self::Context(_) => RecordKind::Context,
            Self::Marker(_) => RecordKind::Marker,
            Self::Heartbeat(_) => RecordKind::Heartbeat,
        }
    }

    /// The host-monotonic capture timestamp carried by every record.
    #[must_use]
    pub const fn mono_ns(&self) -> u64 {
        match self {
            Self::TrapqSegment(r) => r.mono_ns,
            Self::StepperRange(r) => r.mono_ns,
            Self::Context(r) => r.mono_ns,
            Self::Marker(r) => r.mono_ns,
            Self::Heartbeat(r) => r.mono_ns,
        }
    }

    /// `true` when every float in the record is finite.
    ///
    /// JSON cannot round-trip NaN / infinity (`serde_json` serializes them
    /// as `null`), so the WAL writer refuses records where this is
    /// `false`.
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        match self {
            Self::TrapqSegment(r) => r.values_are_finite(),
            Self::StepperRange(r) => r.values_are_finite(),
            Self::Context(r) => r.values_are_finite(),
            Self::Marker(_) => true,
            Self::Heartbeat(r) => r.values_are_finite(),
        }
    }
}

/// Realistic sample values shared by unit tests across the crate.
#[cfg(test)]
pub(crate) mod samples {
    use super::{
        Context, ExcludeObjectDef, ExcludeState, FanTarget, GcodeState, Heartbeat, HeaterTarget,
        Marker, MarkerKind, PolygonFidelity, StepChunk, StepperRange, TransformObservations,
        TrapqSegment, VirtualSdState,
    };

    /// The exclude state of a two-object plate with one object
    /// cancelled, in the shape `exclude_object.get_status` reports.
    pub(crate) fn sample_exclude() -> ExcludeState {
        ExcludeState {
            definitions: Some(vec![
                ExcludeObjectDef {
                    name: "CUBE_ID_0_COPY_0".into(),
                    center: Some([100.0, 100.0]),
                    polygon: vec![[90.0, 90.0], [110.0, 90.0], [110.0, 110.0], [90.0, 110.0]],
                    fidelity: PolygonFidelity::Exact,
                },
                ExcludeObjectDef::name_only("CUBE_ID_1_COPY_0"),
            ]),
            excluded: vec!["CUBE_ID_1_COPY_0".into()],
            current: Some("CUBE_ID_0_COPY_0".into()),
        }
    }

    pub(crate) fn sample_trapq() -> TrapqSegment {
        TrapqSegment {
            mono_ns: 1_111,
            queue: "toolhead".into(),
            print_time: 12.5,
            duration: 0.075,
            start_velocity: 40.0,
            acceleration: -1500.0,
            start_x: 10.0,
            start_y: 20.0,
            start_z: 0.4,
            x_r: 0.6,
            y_r: 0.8,
            z_r: 0.0,
        }
    }

    pub(crate) fn sample_stepper() -> StepperRange {
        StepperRange {
            mono_ns: 2_222,
            stepper: "stepper_x".into(),
            first_clock: 84_000_000,
            last_clock: 84_500_000,
            first_step_time: 12.0,
            last_step_time: 12.007,
            start_position: 9.98,
            start_mcu_position: -1_204,
            step_distance: 0.0025,
            steps: vec![
                StepChunk {
                    interval: 5_000,
                    count: 12,
                    add: -3,
                },
                // Reverse-direction chunk: negative count (the sign is
                // direction, stepcompress.c:372) — every Z lift emits
                // these, so the WAL round-trip must preserve them.
                StepChunk {
                    interval: 4_964,
                    count: -40,
                    add: 0,
                },
                // First-step-after-idle chunk: wrapped u32 interval.
                StepChunk {
                    interval: -2_136_919_700,
                    count: 1,
                    add: 0,
                },
            ],
        }
    }

    pub(crate) fn sample_context() -> Context {
        Context {
            mono_ns: 3_333,
            virtual_sdcard: Some(VirtualSdState {
                file_path: "/home/pi/gcodes/benchy.gcode".into(),
                file_position: 123_456,
            }),
            gcode: GcodeState {
                speed_factor: 1.0,
                speed: 150.0,
                extrude_factor: 0.95,
                absolute_coordinates: true,
                absolute_extrude: false,
                homing_origin: vec![0.0, 0.0, -0.12, 0.0],
                position: vec![10.0, 20.0, 0.4, 512.7],
                gcode_position: vec![10.0, 20.0, 0.52, 512.7],
            },
            transforms: TransformObservations {
                bed_mesh_active: true,
                bed_mesh_profile: Some("default".into()),
                z_thermal_adjust_enabled: Some(true),
                z_thermal_adjust_offset: Some(0.013),
                skew_active: false,
                skew_profile: None,
            },
            heaters: vec![
                HeaterTarget {
                    name: "extruder".into(),
                    target: 215.0,
                },
                HeaterTarget {
                    name: "heater_bed".into(),
                    target: 60.0,
                },
            ],
            fans: vec![FanTarget {
                name: "fan".into(),
                speed: 1.0,
            }],
            exclude: Some(Box::new(sample_exclude())),
        }
    }

    /// A context with no exclude-object observation — the shape every
    /// pre-change WAL carries.
    pub(crate) fn sample_context_without_exclude() -> Context {
        Context {
            exclude: None,
            ..sample_context()
        }
    }

    pub(crate) fn sample_marker() -> Marker {
        Marker {
            mono_ns: 4_444,
            kind: MarkerKind::SubscriptionGap {
                start_mono_ns: 4_000,
                end_mono_ns: 4_400,
            },
        }
    }

    pub(crate) fn sample_heartbeat() -> Heartbeat {
        Heartbeat {
            sequence: 42,
            mono_ns: 5_555,
            wall_ns: 1_760_000_000_000_000_000,
            print_time: 12.625,
            est_sample_mono_ns: 5_500,
            est_sample_print_time: 12.61,
            wal_offset: 8_192,
        }
    }
}

#[cfg(test)]
mod tests {
    // Exact float comparison is the property under test in the round-trip
    // assertions: serde_json must reproduce every finite f64 bit pattern.
    #![allow(clippy::float_cmp)]

    use super::samples::{
        sample_context, sample_context_without_exclude, sample_exclude, sample_heartbeat,
        sample_marker, sample_stepper, sample_trapq,
    };
    use super::{
        Context, ExcludeObjectDef, ExcludeState, Marker, MarkerKind, PolygonFidelity, RecordKind,
        WalRecord, MAX_POLYGON_POINTS,
    };

    /// A `Context` payload exactly as the recorder wrote it **before**
    /// the `exclude` field existed. Pinned verbatim: it is the
    /// backward-compatibility contract, not a value to regenerate.
    const PRE_EXCLUDE_CONTEXT_JSON: &str = concat!(
        r#"{"type":"Context","mono_ns":3333,"#,
        r#""virtual_sdcard":{"file_path":"/home/pi/gcodes/benchy.gcode","file_position":123456},"#,
        r#""gcode":{"speed_factor":1.0,"speed":150.0,"extrude_factor":0.95,"#,
        r#""absolute_coordinates":true,"absolute_extrude":false,"#,
        r#""homing_origin":[0.0,0.0,-0.12,0.0],"position":[10.0,20.0,0.4,512.7],"#,
        r#""gcode_position":[10.0,20.0,0.52,512.7]},"#,
        r#""transforms":{"bed_mesh_active":true,"bed_mesh_profile":"default","#,
        r#""z_thermal_adjust_enabled":true,"z_thermal_adjust_offset":0.013,"#,
        r#""skew_active":false,"skew_profile":null},"#,
        r#""heaters":[{"name":"extruder","target":215.0},{"name":"heater_bed","target":60.0}],"#,
        r#""fans":[{"name":"fan","speed":1.0}]}"#,
    );

    fn roundtrip(record: &WalRecord) -> WalRecord {
        let json = serde_json::to_vec(record).unwrap();
        serde_json::from_slice(&json).unwrap()
    }

    #[test]
    fn trapq_segment_roundtrips() {
        let record = WalRecord::TrapqSegment(sample_trapq());
        assert_eq!(roundtrip(&record), record);
    }

    #[test]
    fn stepper_range_roundtrips() {
        let record = WalRecord::StepperRange(sample_stepper());
        assert_eq!(roundtrip(&record), record);
    }

    #[test]
    fn context_roundtrips() {
        let record = WalRecord::Context(sample_context());
        assert_eq!(roundtrip(&record), record);
    }

    #[test]
    fn marker_roundtrips_every_kind() {
        for kind in [
            MarkerKind::CleanShutdown,
            MarkerKind::SocketLost,
            MarkerKind::Resubscribed,
            MarkerKind::SubscriptionGap {
                start_mono_ns: 1,
                end_mono_ns: 2,
            },
            MarkerKind::ExclusionUpdateLost,
            MarkerKind::Unknown,
        ] {
            let record = WalRecord::Marker(Marker { mono_ns: 9, kind });
            assert_eq!(roundtrip(&record), record);
        }
    }

    #[test]
    fn heartbeat_roundtrips() {
        let record = WalRecord::Heartbeat(sample_heartbeat());
        assert_eq!(roundtrip(&record), record);
    }

    #[test]
    fn pre_exclude_context_fixture_still_decodes() {
        // Backward compatibility: a Context payload written before the
        // `exclude` field existed must decode unchanged, with the new
        // field defaulting to None (never to "nothing was excluded").
        let record: WalRecord = serde_json::from_str(PRE_EXCLUDE_CONTEXT_JSON).unwrap();
        assert_eq!(record, WalRecord::Context(sample_context_without_exclude()));
        let WalRecord::Context(ctx) = &record else {
            panic!("variant changed");
        };
        assert_eq!(ctx.exclude, None);
        assert!(record.values_are_finite());
    }

    #[test]
    fn context_without_exclude_serializes_byte_identically_to_pre_change_format() {
        // Forward compatibility for pre-change *readers*: when no
        // exclude state was observed the key is omitted entirely, so the
        // bytes are exactly what the old recorder produced.
        let json =
            serde_json::to_string(&WalRecord::Context(sample_context_without_exclude())).unwrap();
        assert_eq!(json, PRE_EXCLUDE_CONTEXT_JSON);
    }

    #[test]
    fn exclude_state_roundtrips_through_the_context_record() {
        let record = WalRecord::Context(sample_context());
        assert_eq!(roundtrip(&record), record);
        let json = serde_json::to_string(&record).unwrap();
        assert!(json.contains(r#""exclude":{"#), "{json}");
        assert!(
            json.contains(r#""excluded":["CUBE_ID_1_COPY_0"]"#),
            "{json}"
        );
        // The name-only definition omits center/polygon/fidelity.
        assert!(
            json.contains(r#"{"name":"CUBE_ID_1_COPY_0"}"#),
            "name-only definitions must stay compact: {json}"
        );
    }

    #[test]
    fn absent_definitions_are_distinct_from_empty_definitions() {
        // `None` = "unchanged, carry forward"; `Some(vec![])` = "Klipper
        // knows no objects" (EXCLUDE_OBJECT_DEFINE RESET=1). The wire
        // format must keep them apart.
        let carried = ExcludeState {
            definitions: None,
            excluded: vec!["A".into()],
            current: None,
        };
        let reset = ExcludeState {
            definitions: Some(Vec::new()),
            excluded: Vec::new(),
            current: None,
        };
        let carried_json = serde_json::to_string(&carried).unwrap();
        let reset_json = serde_json::to_string(&reset).unwrap();
        assert_eq!(carried_json, r#"{"excluded":["A"]}"#);
        assert_eq!(reset_json, r#"{"definitions":[],"excluded":[]}"#);
        assert_eq!(
            serde_json::from_str::<ExcludeState>(&carried_json).unwrap(),
            carried
        );
        assert_eq!(
            serde_json::from_str::<ExcludeState>(&reset_json).unwrap(),
            reset
        );
    }

    #[test]
    fn polygon_fidelity_roundtrips_and_degrades_unknown_kinds() {
        for fidelity in [
            PolygonFidelity::Absent,
            PolygonFidelity::Exact,
            PolygonFidelity::BoundingBox { source_points: 900 },
            PolygonFidelity::Unusable { source_points: 2 },
            PolygonFidelity::Unknown,
        ] {
            let def = ExcludeObjectDef {
                name: "A".into(),
                center: None,
                polygon: Vec::new(),
                fidelity,
            };
            let json = serde_json::to_string(&def).unwrap();
            assert_eq!(
                serde_json::from_str::<ExcludeObjectDef>(&json).unwrap(),
                def
            );
        }
        // A fidelity written by a newer revision decodes as Unknown
        // rather than failing the whole record.
        let def: ExcludeObjectDef =
            serde_json::from_str(r#"{"name":"A","fidelity":{"kind":"Voxelized","levels":3}}"#)
                .unwrap();
        assert_eq!(def.fidelity, PolygonFidelity::Unknown);
        assert!(def.fidelity.is_degraded());
        assert!(!def.fidelity.is_absent());
        assert!(!PolygonFidelity::Exact.is_degraded());
        assert!(PolygonFidelity::Absent.is_absent());
        assert!(PolygonFidelity::default().is_absent());
        assert!(PolygonFidelity::BoundingBox { source_points: 1 }.is_degraded());
        assert!(PolygonFidelity::Unusable { source_points: 1 }.is_degraded());
    }

    #[test]
    fn exclude_geometry_participates_in_the_finiteness_gate() {
        // The WAL writer refuses non-finite records; a hostile polygon
        // must be caught there rather than silently becoming JSON null.
        let mut ctx = sample_context();
        let defs = ctx.exclude.as_mut().unwrap().definitions.as_mut().unwrap();
        defs[0].polygon[2][1] = f64::NAN;
        assert!(!WalRecord::Context(ctx).values_are_finite());

        let mut ctx = sample_context();
        ctx.exclude.as_mut().unwrap().definitions.as_mut().unwrap()[0].center =
            Some([1.0, f64::INFINITY]);
        assert!(!WalRecord::Context(ctx).values_are_finite());

        // Carried-forward definitions (None) have nothing to check.
        let ctx = Context {
            exclude: Some(Box::new(ExcludeState {
                definitions: None,
                excluded: vec!["A".into()],
                current: None,
            })),
            ..sample_context_without_exclude()
        };
        assert!(WalRecord::Context(ctx).values_are_finite());
        assert!(sample_exclude().values_are_finite());
        assert!(ExcludeObjectDef::name_only("A").values_are_finite());
    }

    #[test]
    fn normalized_applies_every_outline_rule() {
        // Absent outline.
        let def = ExcludeObjectDef::normalized("A".into(), Some([1.0, 2.0]), None);
        assert_eq!(def.fidelity, PolygonFidelity::Absent);
        assert_eq!(def.center, Some([1.0, 2.0]));
        assert!(def.polygon.is_empty());

        // Verbatim outline.
        let square = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let def = ExcludeObjectDef::normalized("A".into(), None, Some(Ok(square.clone())));
        assert_eq!(def.fidelity, PolygonFidelity::Exact);
        assert_eq!(def.polygon, square);

        // Reported-malformed outline: the ring is discarded whole.
        let def = ExcludeObjectDef::normalized("A".into(), None, Some(Err(7)));
        assert_eq!(def.fidelity, PolygonFidelity::Unusable { source_points: 7 });
        assert!(def.polygon.is_empty());

        // Fewer than three vertices is not a ring.
        let def =
            ExcludeObjectDef::normalized("A".into(), None, Some(Ok(vec![[0.0, 0.0], [1.0, 1.0]])));
        assert_eq!(def.fidelity, PolygonFidelity::Unusable { source_points: 2 });

        // A non-finite coordinate that slipped through.
        let def = ExcludeObjectDef::normalized(
            "A".into(),
            None,
            Some(Ok(vec![[0.0, 0.0], [1.0, f64::NAN], [2.0, 2.0]])),
        );
        assert_eq!(def.fidelity, PolygonFidelity::Unusable { source_points: 3 });
        assert!(def.values_are_finite());

        // A non-finite centre is rejected rather than journaled.
        let def = ExcludeObjectDef::normalized("A".into(), Some([f64::INFINITY, 0.0]), None);
        assert_eq!(def.center, None);

        // Over-long outlines become their bounding box.
        #[allow(clippy::cast_precision_loss)]
        let long: Vec<[f64; 2]> = (0..=MAX_POLYGON_POINTS)
            .map(|i| [i as f64, -(i as f64)])
            .collect();
        let expected = u32::try_from(long.len()).unwrap();
        #[allow(clippy::cast_precision_loss)]
        let max = MAX_POLYGON_POINTS as f64;
        let def = ExcludeObjectDef::normalized("A".into(), None, Some(Ok(long)));
        assert_eq!(
            def.fidelity,
            PolygonFidelity::BoundingBox {
                source_points: expected
            }
        );
        assert_eq!(
            def.polygon,
            vec![[0.0, -max], [max, -max], [max, 0.0], [0.0, 0.0]]
        );
        assert!(def.values_are_finite());
    }

    #[test]
    fn unknown_marker_kind_decodes_as_unknown() {
        // A marker written by a future format revision must not fail to
        // decode; it degrades to MarkerKind::Unknown.
        let json = r#"{"mono_ns": 7, "kind": {"kind": "PowerBrownout", "volts": 10.9}}"#;
        let marker: Marker = serde_json::from_str(json).unwrap();
        assert_eq!(marker.kind, MarkerKind::Unknown);
        assert_eq!(marker.mono_ns, 7);
    }

    #[test]
    fn json_float_roundtrip_is_ulp_exact() {
        // Regression: without serde_json's `float_roundtrip` feature this
        // value parses back one ULP off (-918209536388.2688). Positions
        // and velocities must survive the WAL bit-for-bit.
        let mut seg = sample_trapq();
        seg.start_y = -918_209_536_388.268_9;
        let record = WalRecord::TrapqSegment(seg);
        let WalRecord::TrapqSegment(decoded) = roundtrip(&record) else {
            panic!("variant changed in roundtrip");
        };
        assert_eq!(
            decoded.start_y.to_bits(),
            (-918_209_536_388.268_9_f64).to_bits()
        );
    }

    #[test]
    fn record_json_carries_type_tag() {
        let json = serde_json::to_string(&WalRecord::Marker(sample_marker())).unwrap();
        assert!(json.contains(r#""type":"Marker""#), "{json}");
    }

    #[test]
    fn kind_tags_are_stable_and_reversible() {
        let records = [
            WalRecord::TrapqSegment(sample_trapq()),
            WalRecord::StepperRange(sample_stepper()),
            WalRecord::Context(sample_context()),
            WalRecord::Marker(sample_marker()),
            WalRecord::Heartbeat(sample_heartbeat()),
        ];
        // On-disk values, pinned: renumbering is a format break.
        let expected_tags = [1_u8, 2, 3, 4, 5];
        for (record, expected) in records.iter().zip(expected_tags) {
            assert_eq!(record.kind().as_u8(), expected);
            assert_eq!(RecordKind::from_u8(expected), Some(record.kind()));
        }
        assert_eq!(RecordKind::from_u8(0), None);
        assert_eq!(RecordKind::from_u8(6), None);
        assert_eq!(RecordKind::from_u8(255), None);
    }

    #[test]
    fn mono_ns_is_exposed_for_every_variant() {
        assert_eq!(WalRecord::TrapqSegment(sample_trapq()).mono_ns(), 1_111);
        assert_eq!(WalRecord::StepperRange(sample_stepper()).mono_ns(), 2_222);
        assert_eq!(WalRecord::Context(sample_context()).mono_ns(), 3_333);
        assert_eq!(WalRecord::Marker(sample_marker()).mono_ns(), 4_444);
        assert_eq!(WalRecord::Heartbeat(sample_heartbeat()).mono_ns(), 5_555);
    }

    #[test]
    fn trapq_position_evaluation_matches_klipper_math() {
        let seg = sample_trapq();
        // At t = 0: start position exactly.
        assert_eq!(seg.position_at(seg.print_time), [10.0, 20.0, 0.4]);
        assert_eq!(seg.velocity_at(seg.print_time), 40.0);
        // At t = 0.05: dist = (40 + 0.5*(-1500)*0.05) * 0.05 = 0.125 mm.
        let t = seg.print_time + 0.05;
        let dist = (40.0 + 0.5 * (-1500.0) * 0.05) * 0.05;
        let pos = seg.position_at(t);
        assert!((pos[0] - (10.0 + 0.6 * dist)).abs() < 1e-12);
        assert!((pos[1] - (20.0 + 0.8 * dist)).abs() < 1e-12);
        assert_eq!(pos[2], 0.4);
        // `t - print_time` re-derives 0.05 with rounding error; compare
        // with a tolerance rather than exactly.
        assert!((seg.velocity_at(t) - (40.0 - 1500.0 * 0.05)).abs() < 1e-9);
        // Before the segment: clamped to the start.
        assert_eq!(seg.position_at(seg.print_time - 5.0), [10.0, 20.0, 0.4]);
        // After the segment: clamped to the end (`end_time()` re-derives
        // the elapsed time with rounding error, hence the tolerance).
        let clamped = seg.position_at(seg.print_time + 100.0);
        let at_end = seg.position_at(seg.end_time());
        for (c, e) in clamped.iter().zip(at_end) {
            assert!((c - e).abs() < 1e-9);
        }
        assert!((seg.end_time() - 12.575).abs() < 1e-12);
    }

    #[test]
    fn position_evaluation_never_panics_on_degenerate_segments() {
        let mut seg = sample_trapq();
        seg.duration = -1.0; // corrupt/degenerate: must clamp, not panic
        assert_eq!(seg.position_at(seg.print_time + 1.0), [10.0, 20.0, 0.4]);
        seg.duration = f64::NAN;
        let _ = seg.position_at(seg.print_time + 1.0);
        let _ = seg.velocity_at(seg.print_time + 1.0);
    }

    #[test]
    fn finiteness_checks_catch_every_float_field() {
        assert!(WalRecord::TrapqSegment(sample_trapq()).values_are_finite());
        assert!(WalRecord::StepperRange(sample_stepper()).values_are_finite());
        assert!(WalRecord::Context(sample_context()).values_are_finite());
        assert!(WalRecord::Marker(sample_marker()).values_are_finite());
        assert!(WalRecord::Heartbeat(sample_heartbeat()).values_are_finite());

        let mut trapq = sample_trapq();
        trapq.y_r = f64::NAN;
        assert!(!WalRecord::TrapqSegment(trapq).values_are_finite());

        let mut stepper = sample_stepper();
        stepper.step_distance = f64::INFINITY;
        assert!(!WalRecord::StepperRange(stepper).values_are_finite());

        let mut context = sample_context();
        context.gcode.position[1] = f64::NEG_INFINITY;
        assert!(!WalRecord::Context(context).values_are_finite());

        let mut context = sample_context();
        context.transforms.z_thermal_adjust_offset = Some(f64::NAN);
        assert!(!WalRecord::Context(context).values_are_finite());

        let mut context = sample_context();
        context.heaters[0].target = f64::NAN;
        assert!(!WalRecord::Context(context).values_are_finite());

        let mut context = sample_context();
        context.fans[0].speed = f64::NAN;
        assert!(!WalRecord::Context(context).values_are_finite());

        let mut heartbeat = sample_heartbeat();
        heartbeat.est_sample_print_time = f64::NAN;
        assert!(!WalRecord::Heartbeat(heartbeat).values_are_finite());
    }
}
