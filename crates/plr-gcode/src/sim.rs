//! Forward motion simulation with approximate timing, plus the exact
//! Z-event scan.
//!
//! # Timing model and its accuracy (read before trusting timestamps)
//!
//! Per-move trapezoidal velocity profiles with junction speeds computed
//! by a port of Klipper's `Move.calc_junction` (toolhead.py:66-99: the
//! junction-deviation formula derived from `square_corner_velocity`,
//! including both centripetal caps), followed by a single
//! backward/forward reachability pass equivalent to Klipper's
//! non-lazy `LookAheadQueue.flush` (toolhead.py:135-180) **minus**:
//!
//! * `minimum_cruise_ratio` smoothing (`peak_cruise_v2` propagation,
//!   toolhead.py:150-166; default 0.5). On chains of short moves that
//!   cannot reach cruise speed, real Klipper caps the cruise velocity
//!   lower than this model does, so **this model tends to
//!   underestimate durations** there;
//! * per-axis kinematic limits (`max_z_velocity`/`max_z_accel`),
//!   extruder velocity/accel limits, pressure advance, and input-shaper
//!   effects — all unmodeled, again biasing toward underestimation;
//! * lazy flushing: the simulation horizon always ends in a full stop,
//!   so the last few moves of the window decelerate artificially; the
//!   window also *starts* from zero velocity, while the real machine
//!   was typically mid-motion at the resume offset (overestimates the
//!   first move's duration by up to one accel ramp);
//! * dwell and synchronization commands (G4, M400) pass through
//!   untimed, as do heater waits (M109/M190) — another source of
//!   underestimation if they occur inside the window.
//!
//! The bias direction is the safe one for windowing: because simulated
//! durations are generally lower bounds, a `max_duration` of 2 s covers
//! at least ~2 s of real machine motion. Expect XY timestamps within
//! roughly ±20% for typical sliced files; **do not** use them for
//! anything that needs exact time. The Z *sequence* (which moves touch
//! Z, in what order, to what values) is exact — it comes from the same
//! [`GcodeState`] replay as [`scan_z_events`], independent of timing.
//!
//! `SET_VELOCITY_LIMIT` is not interpreted (it passes through); M204 is.
//!
//! # Stop conditions
//!
//! Input consumption stops at end of input, at `max_lines`, when the
//! accumulated conservative lower-bound time (`move_d / cruise_v`,
//! Klipper's `min_move_t`, toolhead.py:41) reaches `max_duration`, or at
//! the first erroring line ([`StopReason::LineError`] — the moves before
//! it are still returned). A motionless infinite input is bounded only
//! by `max_lines`; keep it set.

use serde::{Deserialize, Serialize};

use crate::parse::Line;
use crate::state::{ApplyOutcome, GcodeState, PlannedMove, StateError, EXTRUDE_ONLY_ACCEL};

/// Klipper's `Move.next_junction_v2` initial value (toolhead.py:48),
/// participating in the junction min.
const NEXT_JUNCTION_V2: f64 = 999_999_999.9;

/// Kinematic limits and stop conditions for [`simulate`].
///
/// All limits must be positive finite numbers; non-positive values do
/// not panic but yield meaningless (infinite/NaN) timestamps.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimConfig {
    /// Toolhead `max_velocity`, mm/s.
    pub max_velocity: f64,
    /// Toolhead `max_accel`, mm/s^2 (overridden per-move by M204).
    pub max_accel: f64,
    /// `square_corner_velocity`, mm/s (junction deviation is derived as
    /// `scv^2 * (sqrt(2)-1) / accel`, toolhead.py:534-536).
    pub square_corner_velocity: f64,
    /// Stop once the conservative lower-bound simulated time reaches
    /// this many seconds (`None` = unbounded).
    pub max_duration: Option<f64>,
    /// Stop after consuming this many input lines (`None` = unbounded).
    pub max_lines: Option<usize>,
}

impl Default for SimConfig {
    /// Klipper-typical limits and the design-doc 2 s / 20 000-line
    /// horizon.
    fn default() -> Self {
        Self {
            max_velocity: 300.0,
            max_accel: 3000.0,
            square_corner_velocity: 5.0,
            max_duration: Some(2.0),
            max_lines: Some(20_000),
        }
    }
}

/// Why the simulation (or scan) stopped consuming input.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum StopReason {
    /// The line iterator was exhausted.
    EndOfInput,
    /// The `max_duration` budget was reached.
    DurationReached,
    /// The `max_lines` budget was reached.
    LineBudget,
    /// The `max_events` budget was reached (Z scan only).
    EventBudget,
    /// A line failed to apply; everything before it is still reported.
    LineError {
        /// Byte offset of the offending line.
        offset: u64,
        /// The error.
        error: StateError,
    },
}

/// One move with trapezoid timing attached. All velocities mm/s, times
/// seconds.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimedMove {
    /// The underlying move.
    pub planned: PlannedMove,
    /// Simulation time at which the move starts.
    pub start_time: f64,
    /// Velocity entering the move.
    pub start_v: f64,
    /// Cruise (peak) velocity.
    pub cruise_v: f64,
    /// Velocity leaving the move.
    pub end_v: f64,
    /// Acceleration-phase duration.
    pub accel_t: f64,
    /// Cruise-phase duration.
    pub cruise_t: f64,
    /// Deceleration-phase duration.
    pub decel_t: f64,
}

impl TimedMove {
    /// Total duration of the move.
    #[must_use]
    pub fn duration(&self) -> f64 {
        self.accel_t + self.cruise_t + self.decel_t
    }

    /// Simulation time at which the move ends.
    #[must_use]
    pub fn end_time(&self) -> f64 {
        self.start_time + self.duration()
    }
}

/// Result of [`simulate`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Simulation {
    /// Timed moves in execution order.
    pub moves: Vec<TimedMove>,
    /// Why input consumption stopped.
    pub stop: StopReason,
    /// Number of lines successfully applied.
    pub lines_consumed: usize,
    /// Total simulated time across all returned moves.
    pub total_time: f64,
    /// `span.end` of the last successfully applied line — a valid
    /// line-boundary resume offset (`None` if nothing was consumed).
    pub resume_offset: Option<u64>,
}

/// Forward-simulate lines against `state` (mutating it), producing timed
/// moves. See the module docs for the timing model's accuracy limits.
pub fn simulate<'a, I>(state: &mut GcodeState, lines: I, config: &SimConfig) -> Simulation
where
    I: IntoIterator<Item = &'a Line>,
{
    let mut planned: Vec<PlannedMove> = Vec::new();
    let mut lines_consumed = 0_usize;
    let mut lower_bound_time = 0.0_f64;
    let mut stop = StopReason::EndOfInput;
    let mut resume_offset = None;
    for line in lines {
        if let Some(max) = config.max_lines {
            if lines_consumed >= max {
                stop = StopReason::LineBudget;
                break;
            }
        }
        match state.apply(line) {
            Err(error) => {
                stop = StopReason::LineError {
                    offset: line.span.start,
                    error,
                };
                break;
            }
            Ok(ApplyOutcome { moves, .. }) => {
                lines_consumed += 1;
                resume_offset = Some(line.span.end);
                for m in moves {
                    lower_bound_time += min_move_time(&m, config);
                    planned.push(m);
                }
            }
        }
        if let Some(max) = config.max_duration {
            if lower_bound_time >= max {
                stop = StopReason::DurationReached;
                break;
            }
        }
    }
    let (moves, total_time) = plan_times(&planned, config);
    Simulation {
        moves,
        stop,
        lines_consumed,
        total_time,
        resume_offset,
    }
}

/// Klipper's `min_move_t` (toolhead.py:41): distance over capped cruise
/// velocity. A strict lower bound on the move's real duration.
fn min_move_time(m: &PlannedMove, config: &SimConfig) -> f64 {
    let d = m.kinematic_distance();
    let v = if m.is_extrude_only() {
        m.speed
    } else {
        m.speed.min(config.max_velocity)
    };
    if v > 0.0 {
        d / v
    } else {
        0.0
    }
}

/// Per-move kinematic quantities, mirroring `Move.__init__`
/// (toolhead.py:14-51) without the `minimum_cruise_ratio` fields.
struct KMove {
    move_d: f64,
    axes_r: [f64; 3],
    accel: f64,
    junction_deviation: f64,
    max_cruise_v2: f64,
    delta_v2: f64,
    max_start_v2: f64,
    is_kinematic: bool,
}

impl KMove {
    fn new(pm: &PlannedMove, config: &SimConfig) -> Self {
        let d = pm.axes_delta();
        let xyz_d = pm.xyz_distance();
        let (move_d, axes_r, accel, velocity, is_kinematic) =
            if xyz_d < crate::state::MIN_KINEMATIC_MOVE {
                // Extrude-only move (toolhead.py:26-37): distance |dE|,
                // huge accel, speed not clamped by max_velocity.
                (
                    d[3].abs(),
                    [0.0, 0.0, 0.0],
                    EXTRUDE_ONLY_ACCEL,
                    pm.speed,
                    false,
                )
            } else {
                let inv = 1.0 / xyz_d;
                (
                    xyz_d,
                    [d[0] * inv, d[1] * inv, d[2] * inv],
                    pm.accel_override.unwrap_or(config.max_accel),
                    pm.speed.min(config.max_velocity),
                    true,
                )
            };
        // toolhead.py:534-536; per-move accel matches Klipper's global
        // junction_deviation recomputed at each M204.
        let scv2 = config.square_corner_velocity * config.square_corner_velocity;
        let junction_deviation = scv2 * (std::f64::consts::SQRT_2 - 1.0) / accel;
        Self {
            move_d,
            axes_r,
            accel,
            junction_deviation,
            max_cruise_v2: velocity * velocity,
            delta_v2: 2.0 * move_d * accel,
            max_start_v2: 0.0,
            is_kinematic,
        }
    }

    /// Port of `Move.calc_junction` (toolhead.py:66-97), without extra
    /// axes and without the `minimum_cruise_ratio` bookkeeping.
    fn calc_junction(&mut self, prev: &KMove) {
        if !self.is_kinematic || !prev.is_kinematic {
            // Junction with an extrude-only move forces a full stop
            // (max_start_v2 stays 0), as in Klipper.
            return;
        }
        let mut max_start_v2 = self
            .max_cruise_v2
            .min(prev.max_cruise_v2)
            .min(NEXT_JUNCTION_V2)
            .min(prev.max_start_v2 + prev.delta_v2);
        let junction_cos_theta = -(self.axes_r[0] * prev.axes_r[0]
            + self.axes_r[1] * prev.axes_r[1]
            + self.axes_r[2] * prev.axes_r[2]);
        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta)).max(0.0).sqrt();
        let cos_theta_d2 = (0.5 * (1.0 + junction_cos_theta)).max(0.0).sqrt();
        let one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;
        if one_minus_sin_theta_d2 > 0.0 && cos_theta_d2 > 0.0 {
            let r_jd = sin_theta_d2 / one_minus_sin_theta_d2;
            let move_jd_v2 = r_jd * self.junction_deviation * self.accel;
            let pmove_jd_v2 = r_jd * prev.junction_deviation * prev.accel;
            // Approximated circle must contact moves no further than
            // mid-move (toolhead.py:89-95).
            let quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2;
            let move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2;
            let pmove_centripetal_v2 = prev.delta_v2 * quarter_tan_theta_d2;
            max_start_v2 = max_start_v2
                .min(move_jd_v2)
                .min(pmove_jd_v2)
                .min(move_centripetal_v2)
                .min(pmove_centripetal_v2);
        }
        self.max_start_v2 = max_start_v2;
    }
}

/// Backward/forward pass assigning trapezoid profiles, equivalent to
/// `LookAheadQueue.flush(lazy=False)` + `Move.set_junction`
/// (toolhead.py:100-114, 135-180) minus `minimum_cruise_ratio` (see
/// module docs). The final move decelerates to a stop.
fn plan_times(moves: &[PlannedMove], config: &SimConfig) -> (Vec<TimedMove>, f64) {
    let mut kmoves: Vec<KMove> = Vec::with_capacity(moves.len());
    for pm in moves {
        let mut km = KMove::new(pm, config);
        if let Some(prev) = kmoves.last() {
            km.calc_junction(prev);
        }
        kmoves.push(km);
    }
    // Backward pass: max junction speeds assuming a stop after the last
    // move (toolhead.py:140-169).
    let mut plan_rev: Vec<(f64, f64, f64)> = Vec::with_capacity(kmoves.len());
    let mut next_start_v2 = 0.0_f64;
    for km in kmoves.iter().rev() {
        let reachable_start_v2 = next_start_v2 + km.delta_v2;
        let start_v2 = km.max_start_v2.min(reachable_start_v2);
        let cruise_v2 = km.max_cruise_v2.min((start_v2 + reachable_start_v2) * 0.5);
        plan_rev.push((start_v2, cruise_v2, next_start_v2));
        next_start_v2 = start_v2;
    }
    plan_rev.reverse();
    // Forward pass: times per `Move.set_junction` (toolhead.py:100-114).
    let mut out = Vec::with_capacity(kmoves.len());
    let mut t = 0.0_f64;
    for ((km, pm), (start_v2, cruise_v2, end_v2)) in kmoves.iter().zip(moves).zip(plan_rev) {
        let start_v2 = start_v2.min(cruise_v2);
        let end_v2 = end_v2.min(cruise_v2);
        let half_inv_accel = 0.5 / km.accel;
        let accel_d = (cruise_v2 - start_v2) * half_inv_accel;
        let decel_d = (cruise_v2 - end_v2) * half_inv_accel;
        let cruise_d = km.move_d - accel_d - decel_d;
        let start_v = start_v2.sqrt();
        let cruise_v = cruise_v2.sqrt();
        let end_v = end_v2.sqrt();
        // Rounding can push cruise_d a hair negative; clamp phase times
        // at zero (divergence: Klipper carries the tiny negative).
        let accel_t = (accel_d / ((start_v + cruise_v) * 0.5)).max(0.0);
        let cruise_t = (cruise_d / cruise_v).max(0.0);
        let decel_t = (decel_d / ((end_v + cruise_v) * 0.5)).max(0.0);
        let tm = TimedMove {
            planned: pm.clone(),
            start_time: t,
            start_v,
            cruise_v,
            end_v,
            accel_t,
            cruise_t,
            decel_t,
        };
        t += tm.duration();
        out.push(tm);
    }
    (out, t)
}

/// Stop conditions for [`scan_z_events`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ZScanConfig {
    /// Stop after consuming this many input lines (`None` = unbounded).
    pub max_lines: Option<usize>,
    /// Stop after collecting this many Z events (`None` = unbounded).
    pub max_events: Option<usize>,
}

impl Default for ZScanConfig {
    fn default() -> Self {
        Self {
            max_lines: Some(20_000),
            max_events: None,
        }
    }
}

/// One upcoming Z-touching move: a z-hop up/down, a layer change, or a
/// spiral (vase-mode / helical-arc) step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ZEvent {
    /// Span of the source line (`span.start` is the `M26`-safe offset of
    /// the line producing this Z motion).
    pub span: crate::parse::ByteSpan,
    /// Z before the move (internal coordinates).
    pub z_from: f64,
    /// Z after the move (internal coordinates).
    pub z_to: f64,
    /// True when the move extrudes while changing Z (spiral/vase or
    /// helical arc chord) — distinguishes layer changes and z-hops
    /// (non-extruding) from spiral motion.
    pub extruding: bool,
    /// False when Z knowledge was lost (G28) and not yet re-established;
    /// such values must not be trusted.
    pub z_known: bool,
    /// Set when the event is one chord of an arc.
    pub arc_segment: Option<crate::state::ArcSegmentInfo>,
}

/// Result of [`scan_z_events`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ZScan {
    /// Z events in execution order.
    pub events: Vec<ZEvent>,
    /// Why the scan stopped.
    pub stop: StopReason,
    /// Number of lines successfully applied.
    pub lines_consumed: usize,
}

/// Exact enumeration of upcoming Z-touching moves.
///
/// This is the safety-critical path: it replays the same
/// [`GcodeState::apply`] as [`simulate`] but involves **no timing
/// model** — the sequence of (offset, `z_from`, `z_to`) is exact for any
/// input the state machine handles. A move counts as Z-touching when
/// its toolhead-effective Z changes at all (extrude-only moves never
/// move Z: the toolhead snaps their XYZ, toolhead.py:28-29).
pub fn scan_z_events<'a, I>(state: &mut GcodeState, lines: I, config: &ZScanConfig) -> ZScan
where
    I: IntoIterator<Item = &'a Line>,
{
    let mut events: Vec<ZEvent> = Vec::new();
    let mut lines_consumed = 0_usize;
    let mut stop = StopReason::EndOfInput;
    'outer: for line in lines {
        if let Some(max) = config.max_lines {
            if lines_consumed >= max {
                stop = StopReason::LineBudget;
                break;
            }
        }
        match state.apply(line) {
            Err(error) => {
                stop = StopReason::LineError {
                    offset: line.span.start,
                    error,
                };
                break;
            }
            Ok(outcome) => {
                lines_consumed += 1;
                for m in &outcome.moves {
                    if let Some(ev) = z_event_of(m) {
                        if let Some(max) = config.max_events {
                            if events.len() >= max {
                                stop = StopReason::EventBudget;
                                break 'outer;
                            }
                        }
                        events.push(ev);
                    }
                }
            }
        }
    }
    ZScan {
        events,
        stop,
        lines_consumed,
    }
}

/// The Z event a single move produces, if any. Exact-equality Z change
/// on the kinematic (snap-corrected) endpoint.
#[must_use]
#[allow(clippy::float_cmp)] // any Z change at all is an event, exactly
pub fn z_event_of(m: &PlannedMove) -> Option<ZEvent> {
    let end = m.kinematic_end();
    if end[2] == m.start[2] {
        return None;
    }
    Some(ZEvent {
        span: m.span,
        z_from: m.start[2],
        z_to: end[2],
        extruding: m.extrudes(),
        z_known: m.start_known[2] && m.end_known[2],
        arc_segment: m.arc_segment,
    })
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::parse::LineIter;

    fn lines(text: &str) -> Vec<Line> {
        LineIter::new(text.as_bytes(), 0).collect()
    }

    fn sim(text: &str, config: &SimConfig) -> Simulation {
        let mut state = GcodeState::new();
        simulate(&mut state, &lines(text), config)
    }

    fn unbounded() -> SimConfig {
        SimConfig {
            max_duration: None,
            max_lines: None,
            ..SimConfig::default()
        }
    }

    #[test]
    fn single_move_trapezoid_exact() {
        // 100 mm at 60 mm/s, accel 3000: accel_d = decel_d = 0.6 mm,
        // accel_t = decel_t = 0.02 s, cruise_t = 98.8/60.
        let s = sim("G1 F3600\nG1 X100\n", &unbounded());
        assert_eq!(s.moves.len(), 1);
        let m = &s.moves[0];
        assert_eq!(m.start_v, 0.0);
        assert_eq!(m.cruise_v, 60.0);
        assert_eq!(m.end_v, 0.0);
        assert!((m.accel_t - 0.02).abs() < 1e-12);
        assert!((m.decel_t - 0.02).abs() < 1e-12);
        assert!((m.cruise_t - 98.8 / 60.0).abs() < 1e-12);
        assert!((s.total_time - m.duration()).abs() < 1e-12);
        assert_eq!(m.start_time, 0.0);
        assert_eq!(s.stop, StopReason::EndOfInput);
    }

    #[test]
    fn short_move_is_accel_limited_triangle() {
        // 1 mm at requested 300 mm/s, accel 3000: cannot reach cruise;
        // cruise_v2 = (0 + 2*d*a)/2 = d*a = 3000 -> cruise_v = 54.77.
        let s = sim("G1 F18000\nG1 X1\n", &unbounded());
        let m = &s.moves[0];
        assert!((m.cruise_v - 3000.0_f64.sqrt()).abs() < 1e-9);
        assert_eq!(m.cruise_t, 0.0);
    }

    #[test]
    fn max_velocity_caps_cruise() {
        let mut cfg = unbounded();
        cfg.max_velocity = 100.0;
        let s = sim("G1 F60000\nG1 X500\n", &cfg);
        assert_eq!(s.moves[0].cruise_v, 100.0);
    }

    #[test]
    fn straight_chain_keeps_speed_through_junction() {
        // Two collinear 50 mm moves: junction speed = cruise speed; no
        // mid-path deceleration.
        let s = sim("G1 F6000\nG1 X50\nG1 X100\n", &unbounded());
        assert_eq!(s.moves.len(), 2);
        assert_eq!(s.moves[0].end_v, 100.0);
        assert_eq!(s.moves[1].start_v, 100.0);
    }

    #[test]
    fn right_angle_corner_slows_to_scv() {
        // 90-degree corner: junction-deviation formula yields exactly
        // scv^2 (R_jd * jd * accel = scv^2 at theta=90).
        let s = sim("G1 F6000\nG1 X50\nG1 Y50\n", &unbounded());
        let junction_v = s.moves[0].end_v;
        assert!((junction_v - 5.0).abs() < 1e-9, "got {junction_v}");
        assert_eq!(s.moves[1].start_v, junction_v);
    }

    #[test]
    fn reversal_stops_completely() {
        // 180-degree reversal: junction speed 0.
        let s = sim("G1 F6000\nG1 X50\nG1 X0\n", &unbounded());
        assert_eq!(s.moves[0].end_v, 0.0);
        assert_eq!(s.moves[1].start_v, 0.0);
    }

    #[test]
    fn extrude_only_move_forces_stop_and_uses_own_timing() {
        // Retract between two travel moves: junctions with a
        // non-kinematic move keep max_start_v2 = 0 (toolhead.py:67-68).
        let s = sim(
            "G1 F6000\nG1 X50\nG1 E-0.8 F3000\nG1 X100 F6000\n",
            &unbounded(),
        );
        assert_eq!(s.moves.len(), 3);
        assert_eq!(s.moves[0].end_v, 0.0);
        assert_eq!(s.moves[1].start_v, 0.0);
        assert_eq!(s.moves[1].end_v, 0.0);
        // Extrude-only: 0.8 mm at 50 mm/s with huge accel ~= 0.016 s.
        assert!((s.moves[1].duration() - 0.8 / 50.0).abs() < 1e-4);
        assert_eq!(s.moves[2].start_v, 0.0);
    }

    #[test]
    fn m204_override_applies_to_following_moves() {
        let mut cfg = unbounded();
        cfg.max_accel = 3000.0;
        let a = sim("G1 F3600\nG1 X100\n", &cfg);
        let b = sim("M204 S500\nG1 F3600\nG1 X100\n", &cfg);
        // Lower accel -> longer accel phase.
        assert!(b.moves[0].accel_t > a.moves[0].accel_t * 2.0);
    }

    #[test]
    fn duration_budget_stops_consumption() {
        use std::fmt::Write as _;
        let mut text = String::from("G1 F6000\n");
        for i in 1..=100 {
            let _ = writeln!(text, "G1 X{}", i * 10);
        }
        let cfg = SimConfig {
            max_duration: Some(0.5),
            max_lines: None,
            ..SimConfig::default()
        };
        let s = sim(&text, &cfg);
        assert_eq!(s.stop, StopReason::DurationReached);
        // The lower bound guarantees >= the requested horizon.
        assert!(s.total_time >= 0.5);
        assert!(s.lines_consumed < 101);
    }

    #[test]
    fn line_budget_stops_consumption() {
        let cfg = SimConfig {
            max_duration: None,
            max_lines: Some(2),
            ..SimConfig::default()
        };
        let s = sim("G1 X1\nG1 X2\nG1 X3\n", &cfg);
        assert_eq!(s.stop, StopReason::LineBudget);
        assert_eq!(s.lines_consumed, 2);
        assert_eq!(s.moves.len(), 2);
    }

    #[test]
    fn zero_line_budget_consumes_nothing() {
        let cfg = SimConfig {
            max_duration: None,
            max_lines: Some(0),
            ..SimConfig::default()
        };
        let s = sim("G1 X1\n", &cfg);
        assert_eq!(s.stop, StopReason::LineBudget);
        assert_eq!(s.resume_offset, None);
    }

    #[test]
    fn error_line_reports_offset_and_keeps_prior_moves() {
        let s = sim("G1 X10\nG20\nG1 X20\n", &unbounded());
        assert_eq!(s.moves.len(), 1);
        let StopReason::LineError { offset, error } = &s.stop else {
            panic!("expected LineError, got {:?}", s.stop);
        };
        assert_eq!(*offset, 7);
        assert!(matches!(error, StateError::InchesUnsupported));
    }

    #[test]
    fn resume_offset_is_line_boundary() {
        let text = "G1 X10\nG1 X20\nG1 X30\n";
        let cfg = SimConfig {
            max_duration: None,
            max_lines: Some(2),
            ..SimConfig::default()
        };
        let s = sim(text, &cfg);
        assert_eq!(s.resume_offset, Some(14), "end of second line");
    }

    #[test]
    fn timestamps_are_monotone_and_contiguous() {
        let s = sim(
            "G1 F9000\nG1 X20 Y5\nG1 X40 Y-3 E2\nG1 Y50\nG1 E-1\nG1 X0 Y0\n",
            &unbounded(),
        );
        let mut t = 0.0;
        for m in &s.moves {
            assert_eq!(m.start_time, t);
            assert!(m.duration() > 0.0);
            t = m.end_time();
        }
        assert_eq!(s.total_time, t);
    }

    #[test]
    fn z_scan_basic_hop_sequence() {
        let text =
            "G1 Z0.2 F7200\nG1 X10 E1\nG1 E-0.8\nG1 Z0.6\nG1 X50\nG1 Z0.2\nG1 E0.8\nG1 X60 E2\n";
        let mut state = GcodeState::new();
        let scan = scan_z_events(&mut state, &lines(text), &ZScanConfig::default());
        assert_eq!(scan.stop, StopReason::EndOfInput);
        let zs: Vec<(f64, f64, bool)> = scan
            .events
            .iter()
            .map(|e| (e.z_from, e.z_to, e.extruding))
            .collect();
        assert_eq!(
            zs,
            vec![(0.0, 0.2, false), (0.2, 0.6, false), (0.6, 0.2, false),]
        );
        assert!(scan.events.iter().all(|e| e.z_known));
        // Offsets point at the producing lines.
        assert_eq!(scan.events[0].span.start, 0);
    }

    #[test]
    fn z_scan_spiral_flags_extruding() {
        let text = "G1 X10 F3000\nG91\nG1 X1 Z0.01 E0.05\nG1 X1 Z0.01 E0.05\n";
        let mut state = GcodeState::new();
        let scan = scan_z_events(&mut state, &lines(text), &ZScanConfig::default());
        assert_eq!(scan.events.len(), 2);
        assert!(scan.events.iter().all(|e| e.extruding));
    }

    #[test]
    fn z_scan_event_budget() {
        let text = "G91\nG1 Z1\nG1 Z1\nG1 Z1\n";
        let mut state = GcodeState::new();
        let cfg = ZScanConfig {
            max_lines: None,
            max_events: Some(2),
        };
        let scan = scan_z_events(&mut state, &lines(text), &cfg);
        assert_eq!(scan.stop, StopReason::EventBudget);
        assert_eq!(scan.events.len(), 2);
    }

    #[test]
    fn z_scan_unknown_z_flagged() {
        let text = "G28\nG91\nG1 Z1\n";
        let mut state = GcodeState::new();
        let scan = scan_z_events(&mut state, &lines(text), &ZScanConfig::default());
        assert_eq!(scan.events.len(), 1);
        assert!(!scan.events[0].z_known);
    }

    #[test]
    fn z_scan_matches_simulation_z_sequence() {
        let text = "G1 Z0.2 F7200\nG1 X10 E1 F1800\nG2 X-10 Y0 I-10 Z1.2 E4\nG1 E-0.5\nG1 Z2\n";
        let ls = lines(text);
        let mut s1 = GcodeState::new();
        let simres = simulate(&mut s1, &ls, &unbounded());
        let sim_events: Vec<Option<ZEvent>> = simres
            .moves
            .iter()
            .map(|tm| z_event_of(&tm.planned))
            .collect();
        let sim_events: Vec<ZEvent> = sim_events.into_iter().flatten().collect();
        let mut s2 = GcodeState::new();
        let scan = scan_z_events(
            &mut s2,
            &ls,
            &ZScanConfig {
                max_lines: None,
                max_events: None,
            },
        );
        assert_eq!(scan.events, sim_events);
        // The helical arc contributes extruding Z events.
        assert!(scan.events.iter().any(|e| e.extruding));
    }

    #[test]
    fn extrude_only_never_produces_z_event() {
        // Even with accumulated sub-nm Z drift, an extrude-only move's
        // kinematic end snaps XYZ (toolhead.py:28-29).
        let mut state = GcodeState::new();
        let ls = lines("G91\nG1 Z0.0000000001 E1 F300\n");
        let scan = scan_z_events(&mut state, &ls, &ZScanConfig::default());
        assert!(scan.events.is_empty());
    }

    #[test]
    fn empty_input() {
        let s = sim("", &SimConfig::default());
        assert_eq!(s.stop, StopReason::EndOfInput);
        assert!(s.moves.is_empty());
        assert_eq!(s.total_time, 0.0);
        assert_eq!(s.resume_offset, None);
    }
}
