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
//!   move-transform observations, and heater/fan targets.
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

/// One compressed step chunk as Klipper's MCU `queue_step` command encodes
/// it: `count` steps, the first after `interval` clock ticks, each
/// subsequent interval increasing by `add`.
///
/// Field widths match the MCU protocol (`queue_step oid=%c interval=%u
/// count=%hu add=%hi`): interval is a 32-bit tick count, count 16-bit,
/// add a signed 16-bit per-step interval delta.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct StepChunk {
    /// Clock ticks before the first step of this chunk.
    pub interval: u32,
    /// Number of steps in this chunk.
    pub count: u16,
    /// Signed tick delta added to `interval` after every step.
    pub add: i16,
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
    /// Current requested feed rate, mm/s.
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

/// Print-context snapshot: everything beyond raw motion needed to rebuild
/// a resumable state (file position, interpreter state, transforms,
/// thermal targets).
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
}

impl Context {
    /// `true` when every float field is finite (no NaN / infinity).
    #[must_use]
    pub fn values_are_finite(&self) -> bool {
        self.gcode.values_are_finite()
            && self.transforms.values_are_finite()
            && self.heaters.iter().all(|h| h.target.is_finite())
            && self.fans.iter().all(|f| f.speed.is_finite())
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
        Context, FanTarget, GcodeState, Heartbeat, HeaterTarget, Marker, MarkerKind, StepChunk,
        StepperRange, TransformObservations, TrapqSegment, VirtualSdState,
    };

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
                StepChunk {
                    interval: 4_964,
                    count: 40,
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
        sample_context, sample_heartbeat, sample_marker, sample_stepper, sample_trapq,
    };
    use super::{Marker, MarkerKind, RecordKind, WalRecord};

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
    fn unknown_marker_kind_decodes_as_unknown() {
        // A marker written by a future format revision must not fail to
        // decode; it degrades to MarkerKind::Unknown.
        let json = r#"{"mono_ns": 7, "kind": {"kind": "PowerBrownout", "volts": 10.9}}"#;
        let marker: Marker = serde_json::from_str(json).unwrap();
        assert_eq!(marker.kind, MarkerKind::Unknown);
        assert_eq!(marker.mono_ns, 7);
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
