//! Typed `motion_report` dump batch payloads.
//!
//! Batching model (`klippy/extras/bulk_sensor.py`): batches are assembled
//! every `BATCH_INTERVAL = 0.500` seconds and pushed to every subscriber.
//! There is **no throttling for slow clients** — if a client stops reading
//! and its send buffer stays blocked for too long, Klipper closes the
//! connection outright (`klippy/webhooks.py`, `ServerSocket.stats`:
//! "Closing unresponsive client"). A consumer must therefore drain the
//! socket promptly and treat reconnection as a normal event.
//!
//! Empty batches are not transmitted (`BatchBulkHelper._proc_batch` skips
//! falsy batch results), so `data` is normally non-empty; the types
//! nevertheless accept an empty `data` array.

use serde::Deserialize;

/// One `motion_report/dump_trapq` batch
/// (`klippy/extras/motion_report.py`, `DumpTrapQ._process_batch` returns
/// `{"data": [...]}`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct TrapqBatch {
    /// Moves in ascending `time` order.
    pub data: Vec<TrapqMove>,
}

/// One trapezoidal move segment from a `dump_trapq` batch.
///
/// Position along the move at relative time `t ∈ [0, duration]` is
/// `start_position + direction * (start_velocity*t + 0.5*acceleration*t²)`
/// (`klippy/extras/motion_report.py`, `DumpTrapQ.get_trapq_position`).
///
/// Two wire encodings exist and both are accepted:
///
/// * Current Klipper (header `time, duration, start_velocity,
///   acceleration, start_position, direction`;
///   `klippy/extras/motion_report.py`, `DumpTrapQ.__init__` /
///   `_process_batch`): `[time, duration, start_v, accel,
///   [start_x, start_y, start_z], [x_r, y_r, z_r]]`.
/// * Older Klipper (pre-2022 header `time, duration, start_v,
///   acceleration, start_x, start_y, start_z, x_r, y_r, z_r`): the same
///   ten values as one flat row.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(from = "TrapqRow")]
pub struct TrapqMove {
    /// Segment start, in primary-MCU `print_time` seconds.
    pub time: f64,
    /// Segment duration in seconds (`move_t`).
    pub duration: f64,
    /// Velocity at segment start, mm/s (`start_v`).
    pub start_velocity: f64,
    /// Constant acceleration over the segment, mm/s² (`accel`).
    pub acceleration: f64,
    /// Cartesian start position `[start_x, start_y, start_z]` in mm. For
    /// extruder trapqs only the first component is meaningful (the other
    /// axes are zero).
    pub start_position: [f64; 3],
    /// Unit direction vector `[x_r, y_r, z_r]`.
    pub direction: [f64; 3],
}

/// Wire-shape helper for [`TrapqMove`]; see its docs for the two forms.
#[derive(Deserialize)]
#[serde(untagged)]
enum TrapqRow {
    Nested(f64, f64, f64, f64, [f64; 3], [f64; 3]),
    Flat(f64, f64, f64, f64, f64, f64, f64, f64, f64, f64),
}

impl From<TrapqRow> for TrapqMove {
    fn from(row: TrapqRow) -> Self {
        match row {
            TrapqRow::Nested(
                time,
                duration,
                start_velocity,
                acceleration,
                start_position,
                direction,
            ) => TrapqMove {
                time,
                duration,
                start_velocity,
                acceleration,
                start_position,
                direction,
            },
            TrapqRow::Flat(
                time,
                duration,
                start_velocity,
                acceleration,
                sx,
                sy,
                sz,
                xr,
                yr,
                zr,
            ) => TrapqMove {
                time,
                duration,
                start_velocity,
                acceleration,
                start_position: [sx, sy, sz],
                direction: [xr, yr, zr],
            },
        }
    }
}

impl TrapqMove {
    /// Position at absolute `print_time` seconds, clamped to the segment
    /// (`klippy/extras/motion_report.py`, `DumpTrapQ.get_trapq_position`).
    /// Total for any finite or non-finite input; a NaN input propagates
    /// NaN, never panics.
    #[must_use]
    pub fn position_at(&self, print_time: f64) -> [f64; 3] {
        let t = (print_time - self.time).clamp(0.0, self.duration.max(0.0));
        let dist = (self.start_velocity + 0.5 * self.acceleration * t) * t;
        [
            self.start_position[0] + self.direction[0] * dist,
            self.start_position[1] + self.direction[1] * dist,
            self.start_position[2] + self.direction[2] * dist,
        ]
    }
}

/// One `motion_report/dump_stepper` batch
/// (`klippy/extras/motion_report.py`, `DumpStepper._process_batch` returns
/// exactly these keys).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct StepperBatch {
    /// `queue_step` rows `[interval, count, add]`, oldest first.
    pub data: Vec<StepperStep>,
    /// Commanded position (mm) at `first_clock`
    /// (`mcu_to_commanded_position(start_mcu_position)`).
    pub start_position: f64,
    /// Raw MCU step counter at `first_clock`.
    pub start_mcu_position: i64,
    /// Distance per step in mm (`get_step_dist`).
    pub step_distance: f64,
    /// MCU clock tick of the first step. Already widened to 64 bits on the
    /// Klipper host side (see [`crate::clock::McuClock`]).
    pub first_clock: u64,
    /// `first_clock` converted to `print_time` seconds by Klipper
    /// (`clock_to_print_time`).
    pub first_step_time: f64,
    /// MCU clock tick of the last step (host-widened, 64-bit).
    pub last_clock: u64,
    /// `last_clock` converted to `print_time` seconds by Klipper.
    pub last_step_time: f64,
}

/// One `queue_step` history row: `|count|` steps starting at `interval`
/// ticks after the previous step, with the interval increasing by `add`
/// ticks after each step (header `('interval', 'count', 'add')`,
/// `klippy/extras/motion_report.py`, `DumpStepper.__init__`).
///
/// # Signedness — these are the C history fields, not the MCU wire fields
///
/// Rows come from the host-side step history, `struct pull_history_steps`
/// (`klippy/chelper/stepcompress.h:8-12`), whose `step_count`, `interval`
/// and `add` are all **signed C `int` (i32)** — not the unsigned MCU
/// `queue_step` wire widths (`struct step_move`,
/// `klippy/chelper/stepcompress.c:59-63`). All three fields can appear
/// negative in the JSON.
///
/// * [`count`](Self::count): **the sign encodes step direction** —
///   `hs->step_count = sc->sdir ? move->count : -move->count`
///   (`klippy/chelper/stepcompress.c:372`), so a negative count means
///   `|count|` steps in the reverse direction (`start_position` decreases
///   by `|count|`, see `stepcompress.c:373` and
///   `stepcompress_find_past_position`, `stepcompress.c:611-613`). Any Z
///   lift/lower produces negative-count rows. The magnitude fits `u16`
///   (`move->count` is `uint16_t`). `count == 0` is a `set_position`
///   marker row (all-zero `history_steps` appended by
///   `stepcompress_set_last_position`, `stepcompress.c:580-585`);
///   Klipper ends a batch after one (`DumpStepper._process_batch`).
/// * [`interval`](Self::interval): the unsigned 32-bit tick count
///   `move->interval` stored into a C `int` (`stepcompress.c:370`), so
///   intervals of 2³¹ ticks or more (a first step after a long idle)
///   wrap negative on the wire. Recover the tick count with
///   [`interval_ticks`](Self::interval_ticks).
/// * [`add`](Self::add): genuinely signed and small (`move->add` is
///   `int16_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(from = "(i32, i32, i32)")]
pub struct StepperStep {
    /// Ticks from the previous step to the first step of this row, as the
    /// raw signed value Klipper emits. Negative values are wrapped `u32`
    /// tick counts — use [`interval_ticks`](Self::interval_ticks).
    pub interval: i32,
    /// Signed step count: `|count|` steps, negative = reverse direction
    /// (see type-level docs). `0` marks a `set_position` row.
    pub count: i32,
    /// Signed per-step interval adjustment in ticks.
    pub add: i32,
}

impl From<(i32, i32, i32)> for StepperStep {
    fn from((interval, count, add): (i32, i32, i32)) -> Self {
        StepperStep {
            interval,
            count,
            add,
        }
    }
}

impl StepperStep {
    /// The interval as the unsigned tick count the MCU actually executes:
    /// the raw value reinterpreted as `u32` (two's complement), undoing
    /// the `uint32_t` → C `int` narrowing in
    /// `klippy/chelper/stepcompress.c:370`.
    #[must_use]
    pub fn interval_ticks(&self) -> u32 {
        self.interval.cast_unsigned()
    }

    /// Number of steps in this row, direction stripped (`|count|`).
    #[must_use]
    pub fn steps(&self) -> u32 {
        self.count.unsigned_abs()
    }

    /// True for a `set_position` marker row (`count == 0`; see type-level
    /// docs). Marker rows carry no motion.
    #[must_use]
    pub fn is_set_position_marker(&self) -> bool {
        self.count == 0
    }
}

#[cfg(test)]
// Exact float comparison is intended here: the values are parsed from
// JSON or computed with the exact arithmetic the test controls.
#[allow(clippy::float_cmp)]
mod tests {
    use super::{StepperBatch, StepperStep, TrapqBatch, TrapqMove};

    #[test]
    fn parses_nested_trapq_rows() {
        // Shape per docs/API_Server.md "motion_report/dump_trapq" example
        // and motion_report.py DumpTrapQ._process_batch.
        let json = r#"{"data": [
            [4.05, 1.0, 0.0, 0.0, [300.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            [5.054, 0.001, 0.0, 3000.0, [300.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]
        ]}"#;
        let batch: TrapqBatch = serde_json::from_str(json).unwrap();
        assert_eq!(batch.data.len(), 2);
        assert_eq!(batch.data[1].acceleration, 3000.0);
        assert_eq!(batch.data[1].direction, [-1.0, 0.0, 0.0]);
    }

    #[test]
    fn parses_flat_trapq_rows() {
        // Older Klipper flat row form (header time, duration, start_v,
        // acceleration, start_x, start_y, start_z, x_r, y_r, z_r).
        let json = r#"{"data": [
            [12.5, 0.25, 40.0, -1500.0, 10.0, 20.0, 0.3, 1.0, 0.0, 0.0]
        ]}"#;
        let batch: TrapqBatch = serde_json::from_str(json).unwrap();
        let m = &batch.data[0];
        assert_eq!(
            *m,
            TrapqMove {
                time: 12.5,
                duration: 0.25,
                start_velocity: 40.0,
                acceleration: -1500.0,
                start_position: [10.0, 20.0, 0.3],
                direction: [1.0, 0.0, 0.0],
            }
        );
    }

    #[test]
    fn rejects_malformed_trapq_rows() {
        for json in [
            r#"{"data": [[1.0, 2.0]]}"#,
            r#"{"data": [["a", 1, 2, 3, [0,0,0], [0,0,0]]]}"#,
            r#"{"data": [[1, 2, 3, 4, [0,0], [0,0,0]]]}"#,
            r#"{"data": 7}"#,
        ] {
            assert!(serde_json::from_str::<TrapqBatch>(json).is_err());
        }
    }

    #[test]
    fn position_at_reproduces_trapq_kinematics() {
        let m = TrapqMove {
            time: 10.0,
            duration: 2.0,
            start_velocity: 5.0,
            acceleration: 2.0,
            start_position: [1.0, 2.0, 3.0],
            direction: [1.0, 0.0, 0.0],
        };
        // Before the segment: clamped to start.
        assert_eq!(m.position_at(9.0), [1.0, 2.0, 3.0]);
        // Mid segment, t = 1: dist = (5 + 0.5*2*1)*1 = 6.
        assert_eq!(m.position_at(11.0), [7.0, 2.0, 3.0]);
        // Past the end: clamped to t = 2: dist = (5 + 2)*2 = 14.
        assert_eq!(m.position_at(100.0), [15.0, 2.0, 3.0]);
        // NaN propagates without panicking.
        assert!(m.position_at(f64::NAN)[0].is_nan());
    }

    #[test]
    fn position_at_handles_negative_duration() {
        let m = TrapqMove {
            time: 0.0,
            duration: -1.0,
            start_velocity: 1.0,
            acceleration: 0.0,
            start_position: [0.0; 3],
            direction: [1.0, 0.0, 0.0],
        };
        assert_eq!(m.position_at(5.0), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn parses_stepper_batch() {
        // Shape per motion_report.py DumpStepper._process_batch; clock
        // values chosen to exceed 2^32 (host-widened 64-bit clocks).
        // Rows include real captured values from a Trident-class triple-Z
        // machine: negative counts are reverse-direction steps
        // (stepcompress.c:372), and the large negative interval is a
        // wrapped u32 tick count (stepcompress.c:70,370).
        let json = r#"{
            "data": [[-2136919700, 1, 0], [10000, 976, 0], [9855, -5, 187],
                     [7457, -40, 0], [12000, -1, 0], [0, 0, 0]],
            "start_position": 12.7,
            "start_mcu_position": -3175,
            "step_distance": 0.0025,
            "first_clock": 5000000000,
            "first_step_time": 27.7777,
            "last_clock": 5009862855,
            "last_step_time": 27.8325
        }"#;
        let batch: StepperBatch = serde_json::from_str(json).unwrap();
        assert_eq!(batch.data.len(), 6);
        // Wrapped interval: -2136919700 as u32 = 2^32 - 2136919700.
        assert_eq!(
            batch.data[0],
            StepperStep {
                interval: -2_136_919_700,
                count: 1,
                add: 0,
            }
        );
        assert_eq!(batch.data[0].interval_ticks(), 2_158_047_596);
        assert!(!batch.data[0].is_set_position_marker());
        // Reverse-direction rows: |count| steps, direction in the sign.
        assert_eq!(
            batch.data[2],
            StepperStep {
                interval: 9855,
                count: -5,
                add: 187,
            }
        );
        assert_eq!(batch.data[2].steps(), 5);
        assert_eq!(batch.data[3].count, -40);
        assert_eq!(batch.data[3].steps(), 40);
        assert_eq!(batch.data[4].count, -1);
        // Positive interval passes through interval_ticks unchanged.
        assert_eq!(batch.data[1].interval_ticks(), 10_000);
        // set_position marker row: all-zero (stepcompress.c:580-585).
        assert!(batch.data[5].is_set_position_marker());
        assert_eq!(batch.data[5].steps(), 0);
        assert_eq!(batch.start_mcu_position, -3175);
        assert!(batch.first_clock > u64::from(u32::MAX));
    }

    #[test]
    fn stepper_batch_requires_documented_keys() {
        assert!(serde_json::from_str::<StepperBatch>(r#"{"data": []}"#).is_err());
    }

    #[test]
    fn stepper_row_bounds_match_the_c_int_fields() {
        // The full i32 range parses (the C fields are plain int)...
        let row: StepperStep =
            serde_json::from_str("[-2147483648, 2147483647, -2147483648]").unwrap();
        assert_eq!(row.interval, i32::MIN);
        assert_eq!(row.interval_ticks(), 1 << 31);
        assert_eq!(row.count, i32::MAX);
        // ...and values outside i32 cannot come from the history structs.
        assert!(serde_json::from_str::<StepperStep>("[2147483648, 0, 0]").is_err());
        assert!(serde_json::from_str::<StepperStep>("[0, -2147483649, 0]").is_err());
    }

    #[test]
    fn rejects_malformed_stepper_rows() {
        for row in [r"[1, 2]", r"[1, 2, 3, 4]", r"[1.5, 2, 3]", r#"["x", 2, 3]"#] {
            assert!(serde_json::from_str::<StepperStep>(row).is_err(), "{row}");
        }
    }
}
