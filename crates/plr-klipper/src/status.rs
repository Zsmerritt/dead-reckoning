//! Typed payloads for `objects/query` results and `objects/subscribe`
//! updates.
//!
//! # Diff semantics
//!
//! The initial subscribe/query response carries every requested field, but
//! each later subscription update carries **only the fields whose value
//! changed** since the previous refresh (`klippy/webhooks.py`,
//! `QueryStatusHelper._do_query`: a field is re-sent only when
//! `rd != lres.get(ri)`). Every field of every struct here is therefore an
//! `Option`: `None` means "not present in this message", not "no value".
//! Consumers must merge updates onto their own snapshot.
//!
//! Fields that can legitimately be JSON `null` (for example
//! `virtual_sdcard.file_path`) are modeled as `Option<Option<T>>`:
//! the outer `Option` is presence in the message, the inner one is the
//! value itself.
//!
//! Updates are generated at most every 0.25 s
//! (`klippy/webhooks.py`, `SUBSCRIPTION_REFRESH_TIME`).
//!
//! Unknown fields are tolerated (ignored) everywhere for forward
//! compatibility; unknown *objects* stay accessible through
//! [`Status::get`].

// `Option<Option<T>>` is deliberate here: the outer layer is diff
// presence, the inner one is JSON nullability, and serde maps this
// pattern directly (absent / null / value). A custom tri-state enum would
// need hand-written `Deserialize` impls for no added clarity.
#![allow(clippy::option_option)]

use std::collections::BTreeMap;

use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value};

use crate::error::MessageError;

/// Deserializes a field so that JSON `null` and an absent key are
/// distinguishable when combined with `#[serde(default)]`: absent keeps
/// the default (`None`), `null` becomes `Some(None)`, a value becomes
/// `Some(Some(v))`.
fn double_option<'de, T, D>(deserializer: D) -> Result<Option<Option<T>>, D::Error>
where
    T: Deserialize<'de>,
    D: Deserializer<'de>,
{
    Option::<T>::deserialize(deserializer).map(Some)
}

/// An `objects/subscribe` update or `objects/query`/`objects/subscribe`
/// response body: `{"eventtime": ..., "status": {...}}`
/// (`klippy/webhooks.py`, `QueryStatusHelper._do_query` builds
/// `{'eventtime': eventtime, 'status': cquery}`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct StatusUpdate {
    /// Host-side monotonic reactor time (seconds) at which the statuses
    /// were sampled. This is the time axis correlated against
    /// `toolhead.estimated_print_time` by
    /// [`ClockCorrelator`](crate::clock::ClockCorrelator).
    pub eventtime: f64,
    /// Per-object status diffs, keyed by object name (for example
    /// `"toolhead"` or `"heater_generic chamber"`).
    pub status: Status,
}

/// The `status` dictionary: object name to that object's (partial)
/// `get_status()` result.
#[derive(Debug, Clone, PartialEq, Default, Deserialize)]
#[serde(transparent)]
pub struct Status(pub Map<String, Value>);

impl Status {
    /// Whether the update contains an entry for `object`.
    #[must_use]
    pub fn contains(&self, object: &str) -> bool {
        self.0.contains_key(object)
    }

    /// Parses the status entry for `object` as `T`. Returns `Ok(None)`
    /// when the object is absent from this update.
    pub fn get<T: serde::de::DeserializeOwned>(
        &self,
        object: &str,
    ) -> Result<Option<T>, MessageError> {
        match self.0.get(object) {
            None => Ok(None),
            Some(value) => serde_json::from_value(value.clone())
                .map(Some)
                .map_err(|source| MessageError::Payload {
                    context: format!("status object {object}"),
                    source,
                }),
        }
    }

    /// The `toolhead` object.
    pub fn toolhead(&self) -> Result<Option<ToolheadStatus>, MessageError> {
        self.get("toolhead")
    }

    /// The `gcode_move` object.
    pub fn gcode_move(&self) -> Result<Option<GcodeMoveStatus>, MessageError> {
        self.get("gcode_move")
    }

    /// The `virtual_sdcard` object.
    pub fn virtual_sdcard(&self) -> Result<Option<VirtualSdcardStatus>, MessageError> {
        self.get("virtual_sdcard")
    }

    /// The primary `mcu` object. Secondary MCUs appear under
    /// `"mcu <name>"`; fetch them with [`Status::get`].
    pub fn mcu(&self) -> Result<Option<McuStatus>, MessageError> {
        self.get("mcu")
    }

    /// The `bed_mesh` object.
    pub fn bed_mesh(&self) -> Result<Option<BedMeshStatus>, MessageError> {
        self.get("bed_mesh")
    }

    /// The `exclude_object` object.
    pub fn exclude_object(&self) -> Result<Option<ExcludeObjectStatus>, MessageError> {
        self.get("exclude_object")
    }

    /// The `z_thermal_adjust` object.
    pub fn z_thermal_adjust(&self) -> Result<Option<ZThermalAdjustStatus>, MessageError> {
        self.get("z_thermal_adjust")
    }

    /// The `skew_correction` object. See [`SkewCorrectionStatus`] for why
    /// this must not be trusted for state reconstruction.
    pub fn skew_correction(&self) -> Result<Option<SkewCorrectionStatus>, MessageError> {
        self.get("skew_correction")
    }

    /// The `idle_timeout` object.
    pub fn idle_timeout(&self) -> Result<Option<IdleTimeoutStatus>, MessageError> {
        self.get("idle_timeout")
    }

    /// The `webhooks` object (printer state).
    pub fn webhooks(&self) -> Result<Option<WebhooksStatus>, MessageError> {
        self.get("webhooks")
    }

    /// The `probe` object (also serves `[bltouch]` and other probe types
    /// that register the shared probe status helper).
    pub fn probe(&self) -> Result<Option<ProbeStatus>, MessageError> {
        self.get("probe")
    }

    /// A heater object by name: `"extruder"`, `"heater_bed"`,
    /// `"heater_generic <name>"`, ... All expose `temperature`, `target`
    /// and `power` (`klippy/extras/heaters.py`, `Heater.get_status`).
    pub fn heater(&self, name: &str) -> Result<Option<HeaterStatus>, MessageError> {
        self.get(name)
    }

    /// A fan object by name: `"fan"`, `"heater_fan <name>"`,
    /// `"fan_generic <name>"`, ...
    pub fn fan(&self, name: &str) -> Result<Option<FanStatus>, MessageError> {
        self.get(name)
    }
}

/// `toolhead` status (`klippy/toolhead.py`, `ToolHead.get_status`, merged
/// with the kinematics status, e.g. `klippy/kinematics/cartesian.py`,
/// `CartKinematics.get_status`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ToolheadStatus {
    /// Last scheduled `print_time` (primary-MCU seconds). Advances only
    /// while moves are queued.
    pub print_time: Option<f64>,
    /// The MCU clock mapped to `print_time` seconds at this update's
    /// `eventtime` (`klippy/clocksync.py`,
    /// `ClockSync.estimated_print_time`). Paired with `eventtime`, this is
    /// the clock-correlation sample consumed by
    /// [`ClockCorrelator`](crate::clock::ClockCorrelator).
    pub estimated_print_time: Option<f64>,
    /// Homed axes as a lowercase string subset of `"xyz"` (empty when
    /// nothing is homed).
    pub homed_axes: Option<String>,
    /// Commanded position `[x, y, z, e, ...]` (a `Coord` tuple on the
    /// wire; extra axes append further components).
    pub position: Option<Vec<f64>>,
    /// Name of the active extruder object.
    pub extruder: Option<String>,
    /// Configured maximum velocity, mm/s.
    pub max_velocity: Option<f64>,
    /// Configured maximum acceleration, mm/s².
    pub max_accel: Option<f64>,
}

/// `gcode_move` status (`klippy/extras/gcode_move.py`,
/// `GCodeMove.get_status`). All positions are `Coord` tuples serialized
/// as arrays `[x, y, z, e, ...]`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct GcodeMoveStatus {
    /// Current speed factor (M220), 1.0 = 100%.
    pub speed_factor: Option<f64>,
    /// Current G-Code speed in mm/s.
    pub speed: Option<f64>,
    /// Current extrude factor (M221), 1.0 = 100%.
    pub extrude_factor: Option<f64>,
    /// True when in G90 absolute-coordinate mode.
    pub absolute_coordinates: Option<bool>,
    /// True when in M82 absolute-extrude mode.
    pub absolute_extrude: Option<bool>,
    /// Homing origin (G92 offset plus `SET_GCODE_OFFSET`), in mm.
    pub homing_origin: Option<Vec<f64>>,
    /// Last commanded position in toolhead coordinates.
    pub position: Option<Vec<f64>>,
    /// Last commanded position in G-Code coordinates (offsets and
    /// scaling removed).
    pub gcode_position: Option<Vec<f64>>,
    /// Axis letter to `position` index map (newer Klipper only).
    pub axis_map: Option<BTreeMap<String, u32>>,
}

/// `virtual_sdcard` status (`klippy/extras/virtual_sdcard.py`,
/// `VirtualSD.get_status`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct VirtualSdcardStatus {
    /// Absolute path of the currently loaded file; JSON `null` when no
    /// file is loaded (`VirtualSD.file_path` returns `None`).
    #[serde(default, deserialize_with = "double_option")]
    pub file_path: Option<Option<String>>,
    /// Fraction of the file already read, `0.0..=1.0`.
    pub progress: Option<f64>,
    /// True while the virtual SD work timer is running (printing).
    pub is_active: Option<bool>,
    /// Current read offset into the file, in bytes.
    pub file_position: Option<u64>,
    /// Total file size in bytes (0 when no file is loaded).
    pub file_size: Option<u64>,
}

/// `mcu` status (`klippy/mcu.py`, `MCUStatsHelper`: `get_status` returns
/// `mcu_version`, `mcu_build_versions`, `mcu_constants`, and — once stats
/// have run — `last_stats`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct McuStatus {
    /// MCU firmware version string.
    pub mcu_version: Option<String>,
    /// Toolchain versions the firmware was built with.
    pub mcu_build_versions: Option<String>,
    /// Firmware compile-time constants (`msgparser.get_constants()`),
    /// mixed ints and strings. Notably contains `CLOCK_FREQ`.
    pub mcu_constants: Option<Map<String, Value>>,
    /// Most recent one-second statistics sample. Absent until the first
    /// stats tick after connect (`MCUStatsHelper.stats` populates it).
    pub last_stats: Option<McuLastStats>,
}

impl McuStatus {
    /// The MCU clock frequency in Hz from `mcu_constants["CLOCK_FREQ"]`,
    /// if present. This is the divisor for
    /// [`McuClock`](crate::clock::McuClock) tick conversion.
    #[must_use]
    pub fn clock_freq(&self) -> Option<f64> {
        self.mcu_constants.as_ref()?.get("CLOCK_FREQ")?.as_f64()
    }
}

/// The `mcu.last_stats` dictionary (`klippy/mcu.py`,
/// `MCUStatsHelper.stats`): the one-second stats line parsed into a dict,
/// values becoming floats when they contain a `.` and ints otherwise.
/// Sources: the load line built in `MCUStatsHelper.stats`, the serial
/// stats (`klippy/chelper/serialqueue.c`, `serialqueue_get_stats`), and
/// the clocksync stats (`klippy/clocksync.py`, `ClockSync.stats`).
///
/// Refresh rate: ~1 Hz (Klipper's stats timer). Subscribing to
/// `mcu.last_stats` therefore yields about one update per second.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct McuLastStats {
    /// Last acknowledged message-block sequence number. **Wraps at 32
    /// bits**: the counter is 64-bit in the host C code, but
    /// `serialqueue_get_stats` formats it with `%u` after an `(int)` cast
    /// (`klippy/chelper/serialqueue.c`), so the exposed value is the low
    /// 32 bits only. Widen it with
    /// [`ReceiveSeqWidener`](crate::clock::ReceiveSeqWidener). An advance
    /// of this counter proves the MCU acknowledged commands up to the
    /// corresponding block — the WAL's evidence that motion commands
    /// reached the MCU.
    pub receive_seq: Option<u64>,
    /// Last sent message-block sequence number (same 32-bit truncation).
    pub send_seq: Option<u64>,
    /// Sequence number of the last retransmit request (same truncation).
    pub retransmit_seq: Option<u64>,
    /// Total bytes written to the serial device.
    pub bytes_write: Option<u64>,
    /// Total bytes read from the serial device.
    pub bytes_read: Option<u64>,
    /// Total bytes retransmitted.
    pub bytes_retransmit: Option<u64>,
    /// Total invalid bytes received.
    pub bytes_invalid: Option<u64>,
    /// Smoothed round-trip time, seconds.
    pub srtt: Option<f64>,
    /// Round-trip time variance, seconds.
    pub rttvar: Option<f64>,
    /// Retransmission timeout, seconds.
    pub rto: Option<f64>,
    /// Estimated MCU clock frequency from clocksync regression
    /// (`klippy/clocksync.py`, `ClockSync.stats`).
    pub freq: Option<u64>,
    /// Fraction of the last second the MCU spent awake
    /// (`MCUStatsHelper.stats` load line).
    pub mcu_awake: Option<f64>,
    /// Average task time on the MCU, seconds.
    pub mcu_task_avg: Option<f64>,
    /// Task time standard deviation on the MCU, seconds.
    pub mcu_task_stddev: Option<f64>,
}

/// Status of a single heater — `extruder`, `heater_bed`,
/// `heater_generic <name>` (`klippy/extras/heaters.py`,
/// `Heater.get_status`; the `extruder` object merges these keys into its
/// own status).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct HeaterStatus {
    /// Smoothed temperature in °C, rounded to 2 decimals.
    pub temperature: Option<f64>,
    /// Target temperature in °C (0 when off).
    pub target: Option<f64>,
    /// Last PWM duty cycle, `0.0..=1.0`.
    pub power: Option<f64>,
}

/// Status of a fan — `fan`, `heater_fan <name>`, `fan_generic <name>`
/// (`klippy/extras/fan.py`, `Fan.get_status`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct FanStatus {
    /// Requested fan speed, `0.0..=1.0`.
    pub speed: Option<f64>,
    /// Measured RPM; JSON `null` when the fan has no tachometer
    /// (`FanTachometer.get_status`).
    #[serde(default, deserialize_with = "double_option")]
    pub rpm: Option<Option<f64>>,
}

/// `bed_mesh` status (`klippy/extras/bed_mesh.py`,
/// `BedMesh.update_status`).
///
/// # Gating "mesh active"
///
/// Use [`BedMeshStatus::mesh_active`] (matrix non-empty), **not**
/// `profile_name`: with no mesh loaded Klipper reports the baseline
/// `{"profile_name": "", ..., "mesh_matrix": [[]]}`, but adaptive meshes
/// (`BED_MESH_CALIBRATE ADAPTIVE=1`) are active while `profile_name` is
/// still `""` — the name only reflects a *saved* profile.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct BedMeshStatus {
    /// Name of the loaded mesh profile; `""` for no profile **or** an
    /// unsaved (e.g. adaptive) mesh. Do not gate on this.
    pub profile_name: Option<String>,
    /// Minimum `(x, y)` of the mesh area.
    pub mesh_min: Option<(f64, f64)>,
    /// Maximum `(x, y)` of the mesh area.
    pub mesh_max: Option<(f64, f64)>,
    /// Raw probed Z values; `[[]]` when no mesh is active.
    pub probed_matrix: Option<Vec<Vec<f64>>>,
    /// Interpolated mesh Z values; `[[]]` when no mesh is active.
    pub mesh_matrix: Option<Vec<Vec<f64>>>,
    /// Saved profiles keyed by name.
    pub profiles: Option<Map<String, Value>>,
}

impl BedMeshStatus {
    /// Whether a mesh is currently applied: `mesh_matrix` present with at
    /// least one non-empty row. `None` when this update does not carry
    /// `mesh_matrix` (diff semantics — state unknown from this message
    /// alone).
    #[must_use]
    pub fn mesh_active(&self) -> Option<bool> {
        self.mesh_matrix
            .as_ref()
            .map(|m| m.iter().any(|row| !row.is_empty()))
    }
}

/// `exclude_object` status (`klippy/extras/exclude_object.py`,
/// `ExcludeObject.get_status` returns exactly
/// `{"objects": [...], "excluded_objects": [...], "current_object": ...}`).
///
/// # Lifetime of the state (why it must be journaled)
///
/// Every field here is per-print runtime state held only in Klipper's
/// memory. `_reset_state` clears all three at construction, and
/// `_reset_file` clears them again on the `virtual_sdcard:reset_file`
/// event and on `EXCLUDE_OBJECT_DEFINE RESET=1`. Nothing persists it, so
/// a power loss erases the operator's cancellations.
///
/// # Command semantics (all name comparisons are upper-cased)
///
/// * `EXCLUDE_OBJECT_DEFINE NAME=<n> [CENTER=x,y] [POLYGON=[[x,y],...]]`
///   appends `{"name": n.upper(), ...}` to `objects`, kept sorted by
///   name (`_add_object_definition`). `CENTER` is parsed as
///   `json.loads('[%s]' % center)` and `POLYGON` as `json.loads(polygon)`.
///   Any other `KEY=VALUE` pairs are merged into the object dict
///   verbatim (`obj.update(parameters)`).
/// * `EXCLUDE_OBJECT_DEFINE RESET=1` calls `_reset_file()`: `objects`,
///   `excluded_objects` **and** `current_object` are all cleared.
/// * `EXCLUDE_OBJECT_START NAME=<n>` sets `current_object` to
///   `n.upper()` and, if that name is unknown, auto-defines a name-only
///   object `{"name": n}` — so `objects` can grow mid-print.
/// * `EXCLUDE_OBJECT_END` sets `current_object` back to `None`.
/// * `EXCLUDE_OBJECT NAME=<n>` adds `n.upper()` to `excluded_objects`
///   (kept sorted); `EXCLUDE_OBJECT CURRENT=1` excludes
///   `current_object`, erroring when there is none.
/// * `EXCLUDE_OBJECT RESET=1` clears **all** exclusions;
///   `EXCLUDE_OBJECT RESET=1 NAME=<n>` un-excludes just that object.
///   Note the Python truthiness: `RESET=0` is a non-empty string and so
///   still resets.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ExcludeObjectStatus {
    /// All objects defined for this print, sorted by name
    /// (`EXCLUDE_OBJECT_DEFINE`, plus auto-definitions from
    /// `EXCLUDE_OBJECT_START`).
    pub objects: Option<Vec<ExcludeObjectDefinition>>,
    /// Names of objects currently excluded, sorted.
    pub excluded_objects: Option<Vec<String>>,
    /// Name of the object being printed; JSON `null` between objects.
    #[serde(default, deserialize_with = "double_option")]
    pub current_object: Option<Option<String>>,
}

/// One defined printable object (`klippy/extras/exclude_object.py`,
/// `cmd_EXCLUDE_OBJECT_DEFINE` builds `{"name": ..., "center": ...,
/// "polygon": ...}` with `center`/`polygon` optional).
///
/// Slicer-specific extra parameters that Klipper merges into the same
/// dict are ignored (no `deny_unknown_fields`): recovery has no use for
/// them.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ExcludeObjectDefinition {
    /// Object name (upper-cased by Klipper).
    pub name: String,
    /// Optional centre point. Klipper parses `CENTER=` as
    /// `json.loads('[%s]' % center)`, so the component count is whatever
    /// the slicer emitted — usually `[x, y]`, sometimes `[x, y, z]`.
    pub center: Option<Vec<f64>>,
    /// Optional outline polygon of `[x, y]` points, straight from
    /// `json.loads(POLYGON)`. Element shape is not validated by Klipper.
    pub polygon: Option<Vec<Vec<f64>>>,
}

impl ExcludeObjectDefinition {
    /// The centre as a finite `[x, y]` pair, or `None` when Klipper
    /// supplied no centre, fewer than two components, or a non-finite
    /// coordinate.
    #[must_use]
    pub fn center_xy(&self) -> Option<[f64; 2]> {
        let center = self.center.as_ref()?;
        let (&x, &y) = (center.first()?, center.get(1)?);
        (x.is_finite() && y.is_finite()).then_some([x, y])
    }

    /// The outline as finite `[x, y]` pairs.
    ///
    /// * `None` — Klipper supplied no `polygon` at all.
    /// * `Some(Err(count))` — a `polygon` was supplied but is unusable:
    ///   some point has fewer than two components or a non-finite
    ///   coordinate. `count` is the number of points reported.
    /// * `Some(Ok(points))` — every point converted.
    ///
    /// Malformed points are never silently dropped: one bad point
    /// invalidates the whole outline, because a partial ring changes
    /// which region it encloses.
    #[must_use]
    pub fn polygon_xy(&self) -> Option<Result<Vec<[f64; 2]>, usize>> {
        let polygon = self.polygon.as_ref()?;
        let mut points = Vec::with_capacity(polygon.len());
        for point in polygon {
            match (point.first(), point.get(1)) {
                (Some(&x), Some(&y)) if x.is_finite() && y.is_finite() => points.push([x, y]),
                _ => return Some(Err(polygon.len())),
            }
        }
        Some(Ok(points))
    }
}

/// The `exclude_object` state accumulated from Klipper's diff-style
/// status updates.
///
/// [`ExcludeObjectStatus`] is a *diff* (see the module docs): a field is
/// re-sent only when it changed. Consumers that need the full picture —
/// the WAL recorder, above all — must merge updates onto a snapshot,
/// which is what this type does.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ExcludeObjectSnapshot {
    /// All objects Klipper currently knows, in its order (sorted by
    /// name).
    pub objects: Vec<ExcludeObjectDefinition>,
    /// Names of the objects the operator has cancelled, in Klipper's
    /// order (sorted).
    pub excluded_objects: Vec<String>,
    /// The object currently being printed, `None` between objects.
    pub current_object: Option<String>,
}

/// Which parts of an [`ExcludeObjectSnapshot`] one merge actually
/// changed. Callers decide what is worth journaling: `current` flips at
/// every `EXCLUDE_OBJECT_START`/`END` (once per object per layer) while
/// `definitions` and `excluded` change only on operator or slicer
/// action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ExcludeObjectChange {
    /// The object definition list changed.
    pub definitions: bool,
    /// The excluded-name set changed — the safety-critical one.
    pub excluded: bool,
    /// The currently-printing object changed.
    pub current: bool,
}

impl ExcludeObjectChange {
    /// `true` when the merge changed anything at all.
    #[must_use]
    pub const fn any(&self) -> bool {
        self.definitions || self.excluded || self.current
    }
}

impl ExcludeObjectSnapshot {
    /// An empty snapshot, matching Klipper's `_reset_state`.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Merges one status diff, reporting what changed. Fields absent
    /// from the diff are left untouched.
    pub fn merge(&mut self, status: &ExcludeObjectStatus) -> ExcludeObjectChange {
        let mut change = ExcludeObjectChange::default();
        if let Some(objects) = &status.objects {
            change.definitions = *objects != self.objects;
            self.objects.clone_from(objects);
        }
        if let Some(excluded) = &status.excluded_objects {
            change.excluded = *excluded != self.excluded_objects;
            self.excluded_objects.clone_from(excluded);
        }
        if let Some(current) = &status.current_object {
            // Outer Option: presence in the diff. Inner: JSON null,
            // which Klipper emits between objects.
            change.current = *current != self.current_object;
            self.current_object.clone_from(current);
        }
        change
    }

    /// `true` when `name` is currently excluded. Comparison is
    /// case-insensitive because Klipper stores upper-cased names
    /// (`name.upper()` in `cmd_EXCLUDE_OBJECT`).
    #[must_use]
    pub fn is_excluded(&self, name: &str) -> bool {
        self.excluded_objects
            .iter()
            .any(|excluded| excluded.eq_ignore_ascii_case(name))
    }

    /// The definition of `name`, if Klipper knows it (case-insensitive).
    #[must_use]
    pub fn definition(&self, name: &str) -> Option<&ExcludeObjectDefinition> {
        self.objects
            .iter()
            .find(|object| object.name.eq_ignore_ascii_case(name))
    }
}

/// `z_thermal_adjust` status (`klippy/extras/z_thermal_adjust.py`,
/// `ZThermalAdjuster.get_status`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ZThermalAdjustStatus {
    /// Smoothed sensor temperature, °C.
    pub temperature: Option<f64>,
    /// Minimum measured temperature, °C.
    pub measured_min_temp: Option<f64>,
    /// Maximum measured temperature, °C.
    pub measured_max_temp: Option<f64>,
    /// Currently applied Z adjustment in mm. This offset is applied
    /// outside the G-Code coordinate system, so reconstructing true
    /// toolhead Z requires it.
    pub current_z_adjust: Option<f64>,
    /// Reference temperature the adjustment is relative to, °C.
    pub z_adjust_ref_temperature: Option<f64>,
    /// Whether adjustment is enabled (`SET_Z_THERMAL_ADJUST ENABLE=`).
    pub enabled: Option<bool>,
}

/// `skew_correction` status (`klippy/extras/skew_correction.py`,
/// `PrinterSkew.get_status`).
///
/// # Reliability warning
///
/// This status is **unreliable for reconstructing skew state** and must
/// not be used as a source of truth:
///
/// * `SET_SKEW XY=... ` (and `SET_SKEW CLEAR=1`) change the active skew
///   factors without touching `current_profile_name`
///   (`cmd_SET_SKEW` never writes it).
/// * `SKEW_PROFILE LOAD=<name>` sets `current_profile_name` *before*
///   checking that the profile exists, so a failed load still updates the
///   reported name (`cmd_SKEW_PROFILE`).
/// * The active skew factors themselves are not exposed at all.
///
/// Recovery must instead replay the skew-related G-Code commands from the
/// job stream.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct SkewCorrectionStatus {
    /// Name of the last profile referenced by `SKEW_PROFILE LOAD`; `""`
    /// initially. See the type-level reliability warning.
    pub current_profile_name: Option<String>,
}

/// `idle_timeout` status (`klippy/extras/idle_timeout.py`,
/// `IdleTimeout.get_status`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct IdleTimeoutStatus {
    /// `"Idle"`, `"Ready"`, or `"Printing"`.
    pub state: Option<String>,
    /// Seconds spent in the current print, 0 when not printing.
    pub printing_time: Option<f64>,
    /// Configured idle timeout in seconds.
    pub idle_timeout: Option<f64>,
}

/// `webhooks` status (`klippy/webhooks.py`, `WebHooks.get_status`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct WebhooksStatus {
    /// Printer state: `"ready"`, `"startup"`, `"shutdown"`, or `"error"`.
    pub state: Option<String>,
    /// Human-readable state message.
    pub state_message: Option<String>,
}

/// `probe` status (`klippy/extras/probe.py`,
/// `ProbeCommandHelper.get_status`).
///
/// # Raw trigger Z versus bed-relative Z
///
/// Klipper builds probe results as `bed_z = test_z - z_offset`, where
/// `test_z` is the **raw toolhead Z at trigger** and `z_offset` is the
/// configured probe offset (`klippy/extras/manual_probe.py`,
/// `create_probe_result`). Consequently:
///
/// * [`last_probe_position`](Self::last_probe_position)`[2]` is `bed_z` —
///   the **bed-relative** height, i.e. the raw trigger Z with `z_offset`
///   already subtracted (`ProbeCommandHelper.cmd_PROBE` stores
///   `(bed_x, bed_y, bed_z)`).
/// * [`last_z_result`](Self::last_z_result) is `bed_z + z_offset`, which
///   equals `test_z` — the **raw trigger Z** in toolhead coordinates
///   (marked "Deprecated" in the Klipper source but still emitted).
///
/// Power-loss recovery needs the raw trigger Z (`last_z_result`);
/// recovering it from `last_probe_position` additionally requires the
/// configured `z_offset`. Both fields are modeled so the recorder can
/// store both and let analysis choose.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ProbeStatus {
    /// Config section name (e.g. `"probe"`, `"bltouch"`).
    pub name: Option<String>,
    /// Result of the last `QUERY_PROBE` (true = triggered).
    pub last_query: Option<bool>,
    /// `(bed_x, bed_y, bed_z)` of the last `PROBE` — z is bed-relative
    /// (see type-level docs). A `Coord` tuple on the wire, so it may
    /// carry a fourth (e) component.
    pub last_probe_position: Option<Vec<f64>>,
    /// Raw toolhead Z at the last probe trigger (see type-level docs).
    pub last_z_result: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::{Status, StatusUpdate};
    use serde_json::json;

    fn parse(v: serde_json::Value) -> StatusUpdate {
        serde_json::from_value(v).unwrap()
    }

    #[test]
    fn full_subscribe_response_round_trip() {
        // Shape per docs/API_Server.md "objects/subscribe" example.
        let update = parse(json!({
            "eventtime": 3_052_153.382_083_195,
            "status": {
                "webhooks": {"state": "ready", "state_message": "Printer is ready"},
                "toolhead": {"position": [0.0, 0.0, 0.0, 0.0]},
            }
        }));
        assert!((update.eventtime - 3_052_153.382_083_195).abs() < 1e-9);
        let th = update.status.toolhead().unwrap().unwrap();
        assert_eq!(th.position, Some(vec![0.0, 0.0, 0.0, 0.0]));
        assert_eq!(th.print_time, None); // diff semantics: not present
        let wh = update.status.webhooks().unwrap().unwrap();
        assert_eq!(wh.state.as_deref(), Some("ready"));
    }

    #[test]
    fn absent_object_is_ok_none() {
        let update = parse(json!({"eventtime": 1.0, "status": {}}));
        assert!(update.status.toolhead().unwrap().is_none());
        assert!(!update.status.contains("toolhead"));
    }

    #[test]
    fn wrong_shape_object_is_a_payload_error() {
        let status = Status(
            json!({"toolhead": {"position": "not-a-list"}})
                .as_object()
                .unwrap()
                .clone(),
        );
        let err = status.toolhead().unwrap_err();
        assert!(err.to_string().contains("status object toolhead"));
    }
}
