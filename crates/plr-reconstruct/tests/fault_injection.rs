//! Fault-injection property test — the E4 acceptance test in miniature.
//!
//! Pipeline per proptest case:
//!
//! 1. Generate a small synthetic print (moves at layer Z, z-hops, layer
//!    changes, dwells, retracts, `G92 E0` rebasing) and render it to
//!    g-code text.
//! 2. Build the **true motion timeline** by replaying the rendered text
//!    through `plr_gcode::GcodeState` (so positions are byte-exact
//!    against what reconstruction will parse) with a constant-velocity
//!    timing model (each move takes `kinematic_distance / capped_speed`,
//!    exactly the simulator's documented per-line lower bound, so the
//!    horizon math is exercised at its tightest).
//! 3. Synthesize the daemon's WAL with **honest batching**: trapq rows
//!    appear at their processing time (which *leads* execution by a
//!    per-scenario lookahead, like real Klipper's read-ahead), Z-stepper
//!    dump ranges cover only motion committed by each 0.5 s flush tick
//!    (with a per-scenario 0.1–0.7 s step-generation lead), context
//!    snapshots record the processing frontier at each flush, and
//!    heartbeats tick at 10 Hz.
//! 4. Cut power at a random time **between** batch flushes: bytes
//!    appended after the last flush survive only as a random-length
//!    prefix (torn tail through `plr_wal`'s real writer + truncation);
//!    the heartbeat file keeps its last two slots, optionally with the
//!    newest torn.
//! 5. Run [`plr_reconstruct::reconstruct`] on the scanned bytes and
//!    assert **containment of the true stop state**: the Z candidate
//!    list contains the true Z, the XY region the true XY, both E
//!    intervals the true E, and the offset window the true file offset —
//!    for every cut point, hop state, layer change, and dwell.

use plr_gcode::{GcodeState, Line, LineIter, PlannedMove};
use plr_reconstruct::{
    reconstruct, CrashClass, FileTail, ReceiveSeqObservation, ReconstructConfig, ReconstructInputs,
    Reconstruction,
};
use plr_wal::heartbeat::{HEARTBEAT_FILE_LEN, HEARTBEAT_SLOT_LEN};
use plr_wal::{
    encode_slot, recover_heartbeat, scan, slot_for_sequence, Context, GcodeState as WalGcodeState,
    Heartbeat, HeartbeatRecovery, SegmentHeader, StepChunk, StepperRange, TransformObservations,
    TrapqSegment, VirtualSdState, WalRecord, WalWriter,
};
use proptest::prelude::*;
use proptest::test_runner::{FileFailurePersistence, Reason};

/// Print-time origin of the synthetic job.
const T0: f64 = 100.0;
/// Toolhead `max_velocity` used by both truth and reconstruction.
const MAX_VELOCITY: f64 = 300.0;
/// Daemon flush/fsync period (dump batching), seconds.
const FLUSH_PERIOD: f64 = 0.5;
/// Heartbeat period, seconds.
const HB_PERIOD: f64 = 0.1;
/// Primary MCU clock frequency.
const MCU_FREQ: f64 = 180_000_000.0;
/// Layer height and z-hop height of the synthetic job.
const LAYER_STEP: f64 = 0.2;
const HOP: f64 = 0.4;
/// File path recorded in contexts.
const FILE_PATH: &str = "/gcodes/fault-injection.gcode";

// ---------------------------------------------------------------------
// Scenario generation
// ---------------------------------------------------------------------

#[derive(Debug, Clone)]
enum Op {
    /// XY move at the current level; extrudes when not hopped.
    Extrude { dx: f64, dy: f64 },
    /// XY travel move (never extrudes).
    Travel { dx: f64, dy: f64 },
    /// Z-hop up (no-op if already hopped).
    HopUp,
    /// Z-hop down (no-op if not hopped).
    HopDown,
    /// Advance to the next layer (un-hops first).
    LayerChange,
    /// `G4 P<ms>` dwell.
    Dwell { ms: u32 },
    /// Absolute-E retract of 0.8 mm.
    Retract,
    /// Absolute-E unretract of 0.8 mm.
    Unretract,
    /// `G92 E0` rebase (rate-limited by rendered motion; see `render`).
    G92E0,
}

fn op_strategy() -> impl Strategy<Value = Op> {
    let delta = -15.0..15.0_f64;
    prop_oneof![
        4 => (delta.clone(), delta.clone()).prop_map(|(dx, dy)| Op::Extrude { dx, dy }),
        2 => (delta.clone(), delta).prop_map(|(dx, dy)| Op::Travel { dx, dy }),
        1 => Just(Op::HopUp),
        1 => Just(Op::HopDown),
        1 => Just(Op::LayerChange),
        1 => (50u32..2_500).prop_map(|ms| Op::Dwell { ms }),
        1 => Just(Op::Retract),
        1 => Just(Op::Unretract),
        1 => Just(Op::G92E0),
    ]
}

// Independent orthogonal fault switches, deliberately bools.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone)]
struct Scenario {
    ops: Vec<Op>,
    /// How far g-code processing leads execution, seconds.
    lookahead: f64,
    /// How far step commits lead execution, seconds.
    step_lead: f64,
    /// Cut position as a fraction of the job duration.
    cut_frac: f64,
    /// Fraction of the unsynced tail bytes that survive the cut.
    torn_frac: f64,
    /// Tear the newest heartbeat slot.
    tear_heartbeat: bool,
    /// Provide a receive-seq observation.
    with_receive_seq: bool,
    /// Provide the MCU frequency to reconstruction.
    with_mcu_freq: bool,
    /// Write heartbeat `print_time` naively (may run ahead of "now"),
    /// exercising the reconstructor's estimate clamp on `t_a`.
    naive_heartbeat_print_time: bool,
}

fn scenario_strategy() -> impl Strategy<Value = Scenario> {
    (
        proptest::collection::vec(op_strategy(), 6..40),
        0.2..1.5_f64,
        0.1..0.7_f64,
        0.05..0.95_f64,
        0.0..1.0_f64,
        any::<bool>(),
        any::<bool>(),
        any::<bool>(),
        any::<bool>(),
    )
        .prop_map(
            |(
                ops,
                lookahead,
                step_lead,
                cut_frac,
                torn_frac,
                tear_heartbeat,
                with_receive_seq,
                with_mcu_freq,
                naive_heartbeat_print_time,
            )| Scenario {
                ops,
                lookahead,
                step_lead,
                cut_frac,
                torn_frac,
                tear_heartbeat,
                with_receive_seq,
                with_mcu_freq,
                naive_heartbeat_print_time,
            },
        )
}

/// Renders ops to g-code text. Keeps XY inside the bed, pairs hops, and
/// rate-limits `G92 E0` so consecutive E-frame shifts are separated by
/// at least ~35 mm (0.7 s) of motion — the documented recoverability
/// condition for WAL-side file-frame E (real slicers rebase at most
/// once per layer).
fn render(ops: &[Op]) -> String {
    use std::fmt::Write as _;
    fn step(v: f64, dv: f64) -> f64 {
        let next = v + dv;
        if (20.0..=180.0).contains(&next) {
            next
        } else {
            v - dv
        }
    }
    let mut text = String::from("G90\nM82\nG92 E0\nG1 X50.000 Y50.000 Z0.200 F3000\n");
    let (mut x, mut y) = (50.0_f64, 50.0_f64);
    let mut layer = LAYER_STEP;
    let mut hopped = false;
    let mut e = 0.0_f64; // file-frame absolute E
    let mut mm_since_g92 = 0.0_f64;
    for op in ops {
        match op {
            Op::Extrude { dx, dy } => {
                let (nx, ny) = (step(x, *dx), step(y, *dy));
                let dist = ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
                if dist < 0.5 {
                    continue;
                }
                mm_since_g92 += dist;
                if hopped {
                    let _ = writeln!(text, "G1 X{nx:.3} Y{ny:.3}");
                } else {
                    e += 0.04 * dist;
                    let _ = writeln!(text, "G1 X{nx:.3} Y{ny:.3} E{e:.5}");
                }
                (x, y) = (nx, ny);
            }
            Op::Travel { dx, dy } => {
                let (nx, ny) = (step(x, *dx), step(y, *dy));
                let dist = ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
                if dist < 0.5 {
                    continue;
                }
                mm_since_g92 += dist;
                let _ = writeln!(text, "G1 X{nx:.3} Y{ny:.3}");
                (x, y) = (nx, ny);
            }
            Op::HopUp => {
                if !hopped {
                    hopped = true;
                    let z = layer + HOP;
                    let _ = writeln!(text, "G1 Z{z:.3}");
                }
            }
            Op::HopDown => {
                if hopped {
                    hopped = false;
                    let _ = writeln!(text, "G1 Z{layer:.3}");
                }
            }
            Op::LayerChange => {
                hopped = false;
                layer += LAYER_STEP;
                let _ = writeln!(text, "G1 Z{layer:.3}");
            }
            Op::Dwell { ms } => {
                let _ = writeln!(text, "G4 P{ms}");
            }
            Op::Retract => {
                e -= 0.8;
                let _ = writeln!(text, "G1 E{e:.5}");
            }
            Op::Unretract => {
                e += 0.8;
                let _ = writeln!(text, "G1 E{e:.5}");
            }
            Op::G92E0 => {
                if mm_since_g92 >= 35.0 {
                    mm_since_g92 = 0.0;
                    e = 0.0;
                    text.push_str("G92 E0\n");
                }
            }
        }
    }
    // A generous tail of further printing so every cut point has real
    // motion beyond it (the extension must have something to simulate).
    for i in 0..12 {
        let tx = 40.0 + 10.0 * f64::from(i % 5);
        let dist_guess = 10.0;
        e += 0.04 * dist_guess;
        let _ = writeln!(text, "G1 X{tx:.3} Y{:.3} E{e:.5}", 30.0 + f64::from(i));
    }
    text
}

// ---------------------------------------------------------------------
// Truth timeline
// ---------------------------------------------------------------------

/// One executed move with constant-velocity timing.
#[derive(Debug, Clone)]
struct TruthMove {
    start_pt: f64,
    end_pt: f64,
    m: PlannedMove,
    /// File-frame E before/after the move.
    file_e_start: f64,
    file_e_end: f64,
    /// Processing-complete time of the producing line.
    p_time: f64,
}

/// Per-line execution/processing bookkeeping.
#[derive(Debug, Clone)]
struct LineInfo {
    span_start: u64,
    span_end: u64,
    /// End of the line's execution interval (equals the previous line's
    /// end for instant lines).
    exec_end: f64,
    /// Processing-complete time (monotone).
    p_time: f64,
    /// Interpreter state after the line.
    snapshot: GcodeState,
}

#[derive(Debug)]
struct Truth {
    infos: Vec<LineInfo>,
    moves: Vec<TruthMove>,
    total_end: f64,
}

/// Dwell seconds of a `G4` line, if it is one.
fn dwell_secs(line: &Line) -> Option<f64> {
    let cmd = line.command()?;
    if cmd.name != "G4" {
        return None;
    }
    let ms: f64 = cmd.get("P")?.parse().ok()?;
    Some(ms / 1000.0)
}

/// The per-move speed both the truth model and the simulator's
/// lower-bound accounting use.
fn move_speed(m: &PlannedMove) -> f64 {
    if m.is_extrude_only() {
        m.speed
    } else {
        m.speed.min(MAX_VELOCITY)
    }
}

/// Replays the rendered text into a constant-velocity truth timeline.
/// Processing (`p_time`) leads execution by `lookahead` for move lines,
/// blocks on dwells, and is instant for state-only lines.
fn build_truth(text: &str, lookahead: f64) -> Truth {
    let lines: Vec<Line> = LineIter::new(text.as_bytes(), 0).collect();
    let mut state = GcodeState::new();
    let mut infos = Vec::with_capacity(lines.len());
    let mut moves = Vec::new();
    let mut cursor = T0;
    let mut p_prev = T0 - lookahead - 1.0;
    for line in &lines {
        let file_e_before = state.gcode_position()[3];
        let outcome = state
            .apply(line)
            .expect("generator emits only supported lines");
        let mut p_time = p_prev;
        if let Some(secs) = dwell_secs(line) {
            cursor += secs;
            // G4 blocks the interpreter until the dwell completes.
            p_time = cursor.max(p_prev);
        }
        // Generator lines are non-arc: at most one move per line, so the
        // line's processing time is computed from its (single) move end.
        let mut pending: Vec<(f64, f64, PlannedMove)> = Vec::new();
        for m in outcome.moves {
            let v = move_speed(&m);
            let dur = if v > 0.0 {
                m.kinematic_distance() / v
            } else {
                0.0
            };
            let start_pt = cursor;
            cursor += dur;
            p_time = (cursor - lookahead).max(p_prev);
            pending.push((start_pt, cursor, m));
        }
        for (start_pt, end_pt, m) in pending {
            moves.push(TruthMove {
                start_pt,
                end_pt,
                m,
                file_e_start: file_e_before,
                file_e_end: state.gcode_position()[3],
                p_time,
            });
        }
        p_prev = p_time;
        infos.push(LineInfo {
            span_start: line.span.start,
            span_end: line.span.end,
            exec_end: cursor,
            p_time,
            snapshot: state.clone(),
        });
    }
    Truth {
        infos,
        moves,
        total_end: cursor,
    }
}

/// The true machine state at print time `t`: position (XYZE internal),
/// file-frame E, and the resume-correct file offset.
fn true_state_at(truth: &Truth, t: f64) -> ([f64; 4], f64, u64) {
    // The file offset comes from the line whose execution interval
    // covers `t` (dwell gaps are G4 intervals, so coverage is total up
    // to total_end).
    let offset = truth
        .infos
        .iter()
        .find(|info| info.exec_end > t)
        .map_or_else(
            || truth.infos.last().map_or(0, |i| i.span_end),
            |info| info.span_start,
        );
    // Position: interpolate the covering move, else hold the last
    // completed line's snapshot.
    for tm in &truth.moves {
        if t >= tm.start_pt && t < tm.end_pt {
            let frac = (t - tm.start_pt) / (tm.end_pt - tm.start_pt);
            let end = tm.m.kinematic_end();
            let mut pos = [0.0; 4];
            for axis in 0..4 {
                pos[axis] = tm.m.start[axis] + (end[axis] - tm.m.start[axis]) * frac;
            }
            let file_e = tm.file_e_start + (tm.file_e_end - tm.file_e_start) * frac;
            return (pos, file_e, offset);
        }
    }
    let snapshot = truth
        .infos
        .iter()
        .rev()
        .find(|info| info.exec_end <= t)
        .map_or_else(GcodeState::new, |info| info.snapshot.clone());
    let pos = snapshot.last_position;
    let file_e = snapshot.gcode_position()[3];
    (pos, file_e, offset)
}

// ---------------------------------------------------------------------
// WAL synthesis
// ---------------------------------------------------------------------

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn pt_to_mono_ns(pt: f64) -> u64 {
    (pt * 1e9).max(0.0) as u64
}

fn wal_gcode_state(state: &GcodeState) -> WalGcodeState {
    WalGcodeState {
        speed_factor: state.speed_factor * 60.0,
        speed: state.speed / state.speed_factor,
        extrude_factor: state.extrude_factor,
        absolute_coordinates: state.absolute_coord,
        absolute_extrude: state.absolute_extrude,
        homing_origin: state.homing_position.to_vec(),
        position: state.last_position.to_vec(),
        gcode_position: state.gcode_position().to_vec(),
    }
}

fn context_record(pt: f64, file_position: u64, state: &GcodeState) -> WalRecord {
    WalRecord::Context(Context {
        mono_ns: pt_to_mono_ns(pt),
        // Faithful to the real recorder: Klipper samples
        // `toolhead.print_time` and `virtual_sdcard.file_position` in one
        // `_do_query` pass, so a context at print time `pt` reports `pt` as
        // its trapq append frontier. Setting it here is what makes this
        // containment suite exercise the coverage-certified band path
        // rather than the pre-change `Uncertifiable` fallback.
        print_time: Some(pt),
        virtual_sdcard: Some(VirtualSdState {
            file_path: FILE_PATH.to_owned(),
            file_position,
            file_size: None,
        }),
        gcode: wal_gcode_state(state),
        transforms: TransformObservations {
            bed_mesh_active: false,
            bed_mesh_profile: None,
            z_thermal_adjust_enabled: None,
            z_thermal_adjust_offset: None,
            skew_active: false,
            skew_profile: None,
        },
        heaters: Vec::new(),
        fans: Vec::new(),
        exclude: None,
        print_state: None,
        current_layer: None,
        total_layer: None,
    })
}

/// Trapq rows for one truth move, in the shape Klipper's
/// `motion_report` emits (toolhead row for XYZ motion; extruder row for
/// E motion with the filament position in the X slot).
fn trapq_rows(tm: &TruthMove) -> Vec<WalRecord> {
    let mut rows = Vec::new();
    let dur = tm.end_pt - tm.start_pt;
    if dur <= 0.0 {
        return rows;
    }
    let mono_ns = pt_to_mono_ns(tm.p_time);
    let end = tm.m.kinematic_end();
    let d = [
        end[0] - tm.m.start[0],
        end[1] - tm.m.start[1],
        end[2] - tm.m.start[2],
        end[3] - tm.m.start[3],
    ];
    let xyz = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if xyz > 0.0 {
        rows.push(WalRecord::TrapqSegment(TrapqSegment {
            mono_ns,
            queue: "toolhead".to_owned(),
            print_time: tm.start_pt,
            duration: dur,
            start_velocity: xyz / dur,
            acceleration: 0.0,
            start_x: tm.m.start[0],
            start_y: tm.m.start[1],
            start_z: tm.m.start[2],
            x_r: d[0] / xyz,
            y_r: d[1] / xyz,
            z_r: d[2] / xyz,
        }));
    }
    if d[3] != 0.0 {
        rows.push(WalRecord::TrapqSegment(TrapqSegment {
            mono_ns,
            queue: "extruder".to_owned(),
            print_time: tm.start_pt,
            duration: dur,
            start_velocity: d[3].abs() / dur,
            acceleration: 0.0,
            start_x: tm.m.start[3],
            start_y: 0.0,
            start_z: 0.0,
            x_r: d[3].signum(),
            y_r: 0.0,
            z_r: 0.0,
        }));
    }
    rows
}

/// Z-stepper dump range covering committed Z motion in
/// `(committed_before, committed_now]`, if any.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn z_range_record(
    truth: &Truth,
    committed_before: f64,
    committed_now: f64,
    tick: f64,
) -> Option<WalRecord> {
    let mut first: Option<f64> = None;
    let mut last: Option<f64> = None;
    let mut start_z = 0.0;
    for tm in &truth.moves {
        let dz = tm.m.kinematic_end()[2] - tm.m.start[2];
        if dz == 0.0 {
            continue;
        }
        let lo = tm.start_pt.max(committed_before);
        let hi = tm.end_pt.min(committed_now);
        if lo >= hi {
            continue;
        }
        if first.is_none() {
            first = Some(lo);
            let frac = (lo - tm.start_pt) / (tm.end_pt - tm.start_pt);
            start_z = tm.m.start[2] + dz * frac;
        }
        last = Some(hi);
    }
    let (first, last) = (first?, last?);
    let first_clock = (first * MCU_FREQ).round() as u64;
    let last_clock = (last * MCU_FREQ).round() as u64;
    Some(WalRecord::StepperRange(StepperRange {
        mono_ns: pt_to_mono_ns(tick),
        stepper: "stepper_z".to_owned(),
        first_clock,
        last_clock,
        first_step_time: first_clock as f64 / MCU_FREQ,
        last_step_time: last_clock as f64 / MCU_FREQ,
        start_position: start_z,
        start_mcu_position: 0,
        step_distance: 0.0025,
        steps: vec![StepChunk {
            interval: 4_000,
            count: 16,
            add: 0,
        }],
    }))
}

/// The whole daemon-observed event stream (time, record), time-sorted.
// Heartbeat counts stay tiny (hundreds); the u64 -> f64 tick math is exact.
#[allow(clippy::cast_precision_loss)]
fn synthesize_events(truth: &Truth, scenario: &Scenario) -> Vec<(f64, WalRecord)> {
    let mut events: Vec<(f64, WalRecord)> = Vec::new();
    // Trapq rows appear when the daemon receives them: at processing time.
    for tm in &truth.moves {
        for row in trapq_rows(tm) {
            events.push((tm.p_time.max(T0 - 2.0), row));
        }
    }
    // Flush ticks: contexts (processing frontier) and committed Z ranges.
    let horizon_end = truth.total_end + 2.0;
    let mut committed_before = T0;
    let mut k = 0u32;
    loop {
        let tick = FLUSH_PERIOD.mul_add(f64::from(k), T0);
        if tick > horizon_end {
            break;
        }
        // Frontier: last line processed by this tick.
        let frontier = truth.infos.iter().rev().find(|info| info.p_time <= tick);
        let (fp, state) = frontier.map_or_else(
            || (0, GcodeState::new()),
            |info| (info.span_end, info.snapshot.clone()),
        );
        events.push((tick, context_record(tick, fp, &state)));
        // Committed frontier: bounded by step-generation lead and by
        // what has been planned at all.
        let planned = frontier.map_or(T0, |info| info.exec_end);
        let committed_now = (tick + scenario.step_lead)
            .min(planned)
            .min(truth.total_end)
            .max(committed_before);
        if let Some(record) = z_range_record(truth, committed_before, committed_now, tick) {
            events.push((tick, record));
        }
        committed_before = committed_now;
        k += 1;
    }
    // Heartbeats at 10 Hz.
    let mut n = 0u64;
    loop {
        let tick = HB_PERIOD.mul_add(n as f64, T0);
        if tick > horizon_end {
            break;
        }
        let received_end = truth
            .moves
            .iter()
            .filter(|tm| tm.p_time <= tick)
            .map(|tm| tm.end_pt)
            .fold(T0, f64::max);
        let print_time = if scenario.naive_heartbeat_print_time {
            received_end
        } else {
            received_end.min(tick)
        };
        events.push((
            tick,
            WalRecord::Heartbeat(Heartbeat {
                sequence: n,
                mono_ns: pt_to_mono_ns(tick),
                wall_ns: 1_700_000_000_000_000_000,
                print_time,
                est_sample_mono_ns: pt_to_mono_ns(tick),
                est_sample_print_time: tick,
                wal_offset: 0,
            }),
        ));
        n += 1;
    }
    events.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    events
}

/// Appends events through the real WAL writer, cutting at `cut`:
/// everything up to the last flush tick is durable; later bytes survive
/// only as a `torn_frac` prefix (a torn tail).
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn synthesize_wal(events: &[(f64, WalRecord)], cut: f64, torn_frac: f64) -> Vec<u8> {
    let flushes = ((cut - T0) / FLUSH_PERIOD).floor().max(0.0);
    let last_flush = FLUSH_PERIOD.mul_add(flushes, T0);
    let mut writer = WalWriter::create(
        Vec::new(),
        &SegmentHeader::new(1_700_000_000_000_000_000, 1),
    )
    .expect("writing to a Vec cannot fail");
    let mut durable_len: Option<usize> = None;
    for (time, record) in events {
        if *time > cut {
            break;
        }
        if *time > last_flush && durable_len.is_none() {
            durable_len = Some(usize::try_from(writer.offset()).expect("offset fits usize"));
        }
        writer.append(record).expect("append to Vec cannot fail");
    }
    let bytes = writer.into_inner();
    let durable = durable_len.unwrap_or(bytes.len());
    let tail = bytes.len() - durable;
    let keep = durable + (tail as f64 * torn_frac) as usize;
    bytes[..keep.min(bytes.len())].to_vec()
}

/// Builds the 128-byte heartbeat file holding the last two heartbeats
/// before the cut, optionally tearing the newest slot.
fn synthesize_heartbeat_file(events: &[(f64, WalRecord)], cut: f64, tear_newest: bool) -> Vec<u8> {
    let mut latest: Option<Heartbeat> = None;
    let mut previous: Option<Heartbeat> = None;
    for (time, record) in events {
        if *time > cut {
            break;
        }
        if let WalRecord::Heartbeat(hb) = record {
            previous = latest;
            latest = Some(*hb);
        }
    }
    let mut file = vec![0_u8; HEARTBEAT_FILE_LEN];
    let mut place = |hb: &Heartbeat| {
        let offset = slot_for_sequence(hb.sequence).offset();
        file[offset..offset + HEARTBEAT_SLOT_LEN].copy_from_slice(&encode_slot(hb));
    };
    if let Some(hb) = &previous {
        place(hb);
    }
    if let Some(hb) = &latest {
        place(hb);
    }
    if tear_newest {
        if let (Some(hb), Some(_)) = (&latest, &previous) {
            // Corrupt a payload byte: CRC mismatch, recovery falls back
            // to the previous slot.
            let offset = slot_for_sequence(hb.sequence).offset();
            file[offset + 20] ^= 0xFF;
        }
    }
    file
}

// ---------------------------------------------------------------------
// The property
// ---------------------------------------------------------------------

// One linear assertion battery per case; splitting it would obscure the
// property being asserted.
#[allow(clippy::too_many_lines)]
fn run_case(scenario: &Scenario) -> Result<(), TestCaseError> {
    let text = render(&scenario.ops);
    let truth = build_truth(&text, scenario.lookahead);
    prop_assume!(truth.total_end > T0 + 2.0);
    let cut = (truth.total_end - T0).mul_add(scenario.cut_frac, T0);

    let events = synthesize_events(&truth, scenario);
    let wal_bytes = synthesize_wal(&events, cut, scenario.torn_frac);
    let hb_file = synthesize_heartbeat_file(&events, cut, scenario.tear_heartbeat);

    let recovery_scan = scan(&wal_bytes);
    let heartbeat: Option<HeartbeatRecovery> = recover_heartbeat(&hb_file).ok();

    let obs = scenario.with_receive_seq.then(|| {
        let acks = ((cut - T0) / 1.0).floor().max(0.0);
        // Small non-negative integer by construction.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let acks_int = acks as u64;
        ReceiveSeqObservation {
            mono_ns: pt_to_mono_ns(acks + T0),
            widened_seq: 4_000_000_000 + acks_int,
        }
    });
    let inputs = ReconstructInputs {
        scan: &recovery_scan,
        heartbeat: heartbeat.as_ref(),
        file_tail: Some(FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        }),
        receive_seq: obs,
    };
    let config = ReconstructConfig {
        mcu_freq: scenario.with_mcu_freq.then_some(MCU_FREQ),
        ..ReconstructConfig::default()
    };

    let outcome = reconstruct(&inputs, &config)
        .map_err(|e| TestCaseError::fail(Reason::from(format!("reconstruct failed: {e}"))))?;
    let Reconstruction::Recovery(recovery) = outcome else {
        return Err(TestCaseError::fail(Reason::from(
            "unclean stop misread as clean",
        )));
    };
    prop_assert!(
        recovery.window.class != CrashClass::CleanShutdown,
        "unclean stop classified clean"
    );

    let (pos, file_e, offset) = true_state_at(&truth, cut);
    let set = &recovery.stop_set;

    // t_a must never exceed the true stop time.
    prop_assert!(
        recovery.window.t_a <= cut + 1e-6,
        "t_a {} exceeds the cut {}",
        recovery.window.t_a,
        cut
    );

    // Z containment: the exact candidate enumeration must contain the
    // true Z.
    prop_assert!(
        set.contains_z(pos[2], 1e-6),
        "true z {} not in candidates {:?} (cut {}, window {:?})",
        pos[2],
        set.z_candidates,
        cut,
        (recovery.window.t_a, recovery.window.t_b)
    );
    // The z-span (probe envelope sizing) must cover it too: all
    // generator candidates are z_known.
    let span = set
        .z_span()
        .ok_or_else(|| TestCaseError::fail(Reason::from("no trusted z span")))?;
    prop_assert!(span.contains(pos[2], 1e-6));

    // XY containment.
    let xy = set
        .xy
        .ok_or_else(|| TestCaseError::fail(Reason::from("no xy region")))?;
    prop_assert!(
        xy.contains(pos[0], pos[1], 1e-6),
        "true xy ({}, {}) outside region {:?} at cut {}",
        pos[0],
        pos[1],
        xy,
        cut
    );

    // E containment, both frames.
    let e_internal = set
        .e_internal
        .ok_or_else(|| TestCaseError::fail(Reason::from("no internal E interval")))?;
    prop_assert!(
        e_internal.contains(pos[3], 1e-6),
        "true internal E {} outside {:?} at cut {}",
        pos[3],
        e_internal,
        cut
    );
    let e_file = set
        .e_file
        .ok_or_else(|| TestCaseError::fail(Reason::from("no file E interval")))?;
    prop_assert!(
        e_file.contains(file_e, 1e-4),
        "true file E {} outside {:?} at cut {}",
        file_e,
        e_file,
        cut
    );

    // File-offset containment. The floor is only guaranteed when the
    // WAL held a context old enough (the crate flags the fallback).
    let window = set
        .file_window
        .ok_or_else(|| TestCaseError::fail(Reason::from("no offset window")))?;
    prop_assert!(
        offset <= window.end,
        "true offset {} beyond window end {} at cut {}",
        offset,
        window.end,
        cut
    );
    if !set.degradation.offset_floor_uncertain {
        prop_assert!(
            window.start <= offset,
            "window floor {} beyond true offset {} at cut {}",
            window.start,
            offset,
            cut
        );
    }
    Ok(())
}

proptest! {
    // Case count comes from ProptestConfig::default(), so
    // PROPTEST_CASES can crank it up for soak runs (256 by default).
    // Shrunk counterexamples persist to the checked-in file
    // tests/fault_injection.proptest-regressions: the SourceParallel
    // default cannot locate lib.rs/main.rs from an integration test
    // and only works via a warning-emitting fallback, so WithSource
    // pins the exact same path explicitly.
    #![proptest_config(ProptestConfig {
        max_shrink_iters: 400,
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::default()
    })]

    /// The whole point of the crate: the possible-stop set ALWAYS
    /// contains the true stop state, across randomized cut points,
    /// hop/no-hop, layer changes, dwells, torn tails, torn heartbeat
    /// slots, and missing receive-seq / MCU-frequency inputs.
    #[test]
    fn possible_stop_set_always_contains_true_state(scenario in scenario_strategy()) {
        run_case(&scenario)?;
    }
}

#[test]
fn clean_shutdown_marker_reports_distinctly() {
    let scenario = Scenario {
        ops: vec![
            Op::Extrude { dx: 10.0, dy: 5.0 },
            Op::HopUp,
            Op::Travel { dx: -8.0, dy: 3.0 },
            Op::HopDown,
        ],
        lookahead: 0.5,
        step_lead: 0.4,
        cut_frac: 0.5,
        torn_frac: 1.0,
        tear_heartbeat: false,
        with_receive_seq: false,
        with_mcu_freq: true,
        naive_heartbeat_print_time: false,
    };
    let text = render(&scenario.ops);
    let truth = build_truth(&text, scenario.lookahead);
    let mut events = synthesize_events(&truth, &scenario);
    let end = truth.total_end + 1.0;
    events.push((
        end,
        WalRecord::Marker(plr_wal::Marker {
            mono_ns: pt_to_mono_ns(end),
            kind: plr_wal::MarkerKind::CleanShutdown,
        }),
    ));
    // Cut well after the marker: everything durable.
    let wal_bytes = synthesize_wal(&events, end + 10.0, 1.0);
    let hb_file = synthesize_heartbeat_file(&events, end + 10.0, false);
    let recovery_scan = scan(&wal_bytes);
    let heartbeat = recover_heartbeat(&hb_file).ok();
    let outcome = reconstruct(
        &ReconstructInputs {
            scan: &recovery_scan,
            heartbeat: heartbeat.as_ref(),
            file_tail: Some(FileTail {
                base_offset: 0,
                bytes: text.as_bytes(),
            }),
            receive_seq: None,
        },
        &ReconstructConfig {
            mcu_freq: Some(MCU_FREQ),
            ..ReconstructConfig::default()
        },
    )
    .expect("clean shutdown reconstructs");
    assert!(
        matches!(outcome, Reconstruction::CleanShutdown(_)),
        "clean shutdown not reported distinctly: {outcome:?}"
    );
}
