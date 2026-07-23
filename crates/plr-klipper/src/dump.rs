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

/// One `queue_step` command: `count` steps starting at `interval` ticks
/// after the previous step, with the interval increasing by `add` ticks
/// after each step (header `('interval', 'count', 'add')`,
/// `klippy/extras/motion_report.py`, `DumpStepper.__init__`).
///
/// A row with `count == 0` is a `set_position` marker; Klipper ends a
/// batch after one (`DumpStepper._process_batch`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(from = "(u64, u64, i64)")]
pub struct StepperStep {
    /// Ticks from the previous step to the first step of this row.
    pub interval: u64,
    /// Number of steps.
    pub count: u64,
    /// Signed per-step interval adjustment in ticks.
    pub add: i64,
}

impl From<(u64, u64, i64)> for StepperStep {
    fn from((interval, count, add): (u64, u64, i64)) -> Self {
        StepperStep {
            interval,
            count,
            add,
        }
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
        let json = r#"{
            "data": [[7457, 1, 0], [10000, 976, 0], [9855, 5, 187], [0, 0, 0]],
            "start_position": 12.7,
            "start_mcu_position": -3175,
            "step_distance": 0.0025,
            "first_clock": 5000000000,
            "first_step_time": 27.7777,
            "last_clock": 5009862855,
            "last_step_time": 27.8325
        }"#;
        let batch: StepperBatch = serde_json::from_str(json).unwrap();
        assert_eq!(batch.data.len(), 4);
        assert_eq!(
            batch.data[2],
            StepperStep {
                interval: 9855,
                count: 5,
                add: 187,
            }
        );
        // set_position marker row.
        assert_eq!(batch.data[3].count, 0);
        assert_eq!(batch.start_mcu_position, -3175);
        assert!(batch.first_clock > u64::from(u32::MAX));
    }

    #[test]
    fn stepper_batch_requires_documented_keys() {
        assert!(serde_json::from_str::<StepperBatch>(r#"{"data": []}"#).is_err());
    }

    #[test]
    fn rejects_malformed_stepper_rows() {
        for row in [r"[1, 2]", r"[1, 2, 3, 4]", r"[-1, 2, 3]", r#"["x", 2, 3]"#] {
            assert!(serde_json::from_str::<StepperStep>(row).is_err(), "{row}");
        }
    }
}
