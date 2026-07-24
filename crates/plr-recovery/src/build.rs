//! The recovery-plan builder: turns validated machine state,
//! reconstruction output, analyzer results and WAL context into the
//! strictly ordered §8 plan.
//!
//! Pure logic: the output is data ([`crate::plan::RecoveryPlan`]); the
//! daemon executes it. Every numeric input is validated finite before
//! it can reach a command string.
//!
//! # Ordering deviations, documented
//!
//! * The restore step (§8.9) begins with a bounded **relative Z lift**
//!   off the part: after `PROBE SAMPLES=1` the nozzle rests pressed
//!   against layer N−1 plastic, and the restore step's print
//!   temperature verification polls for minutes — a nozzle dwelling at
//!   print temperature against the part would melt a divot. The lift
//!   moves in the safe direction (away from the part) inside the
//!   already-declared true frame.
//! * The final feedrate (`G1 F<raw>`) is emitted in the restore step
//!   (§8.9) **and re-asserted at the end of the entry step**, because
//!   the entry moves carry their own `F` words which would otherwise
//!   overwrite the restored feedrate just before `M24`.
//! * `G92 E` and the absolute/relative mode restores run at the end of
//!   the entry step rather than in §8.9, because the entry moves
//!   themselves must run in absolute XYZ / relative E regardless of the
//!   file's modes, and the prime move changes E after any earlier
//!   `G92 E` would have run.

use std::fmt::Write as _;

use plr_analyzer::{
    ContactOutcome, DeclineReason, FeatureClass, LayerModel, MatchConfidence, MatchResult,
    ProbeCandidate,
};
use plr_reconstruct::Reconstruction;
use plr_wal::Context;
use serde::{Deserialize, Serialize};

use crate::envelope::{compute_envelope, Envelope, EnvelopeParams, OvershootTerm};
use crate::error::RecoveryError;
use crate::machine::{validate_machine, MachineConfig, ProbeKind, ValidatedMachine};
use crate::plan::{
    fmt_num, AbortReason, FailureAction, Phase, PlanWarning, Predicate, RecoveryPlan, RecoveryStep,
    RuntimeComputation, TriggerSource, TrueZFormula, Verification, RESTORE_ACCEL_PLACEHOLDER,
    TRUE_Z_PLACEHOLDER,
};
use crate::preflight::{preflight_itinerary, ItineraryBounds};
use crate::preheat::{derive_preheat, FileTemps};

/// Tunables of the plan builder. [`PlanConfig::default`] is the
/// design-doc configuration; [`PlanConfig::validate`] enforces every
/// documented bound.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanConfig {
    /// Probe speed, mm/s. Hard-capped to `[1, 2]`
    /// (see [`crate::envelope`]).
    pub probe_speed: f64,
    /// Envelope margin, mm.
    pub margin: f64,
    /// Sag allowance added to the stop-set Z span, mm.
    pub sag_allowance: f64,
    /// `SET_IDLE_TIMEOUT` value, seconds. Must be at least 600: the
    /// default idle timeout runs `M84`, and any motor-off clears ALL
    /// homed state — a later naive `G28` would crash the bed into the
    /// nozzle.
    pub idle_timeout_s: f64,
    /// Nozzle temperature commanded for probing, °C. Must lie inside
    /// the `[probe_temp_min, probe_temp_max]` band.
    pub probe_nozzle_temp: f64,
    /// Lower bound of the warm-but-below-ooze probing band, °C
    /// (design doc: 140).
    pub probe_temp_min: f64,
    /// Upper bound of the probing band, °C (design doc: 160).
    pub probe_temp_max: f64,
    /// Allowed deviation when verifying restored print temperatures,
    /// °C.
    pub temp_epsilon: f64,
    /// XY travel feedrate for the probe approach, mm/min.
    pub travel_feed: f64,
    /// Feedrate of the speed-limited entry moves, mm/min. Capped at
    /// 1800 (30 mm/s): the first moves near the part are deliberately
    /// slow.
    pub entry_feed: f64,
    /// Entry hop above the resume Z, mm.
    pub entry_hop: f64,
    /// Prime (unretract) length before resuming, mm of filament.
    pub prime_mm: f64,
    /// Prime feedrate, mm/min.
    pub prime_feed: f64,
    /// Allowed XY deviation when verifying positions, mm.
    pub xy_epsilon: f64,
    /// Allowed Z deviation when verifying declarations, mm.
    pub z_epsilon: f64,
    /// ADXL drag: XY speed of each fixed-Z drag pass, mm/s. Hard band
    /// `(0, 100]` — the FIXED `[plr]` schema band shared with the
    /// Klipper plugin (`klippy_plugin/plr/tunables.py`); plrd and the
    /// plugin must accept exactly the same values or a config the
    /// console accepted would be refused at recover time. The pass
    /// speed does not affect the descent bound: passes are fixed-Z and
    /// the envelope's overshoot is `drag_z_step` alone. Only consulted
    /// when the probe kind is `AdxlDrag`, but always validated (an
    /// invalid config is invalid regardless of which branch would read
    /// it).
    pub drag_speed: f64,
    /// ADXL drag: Z decrement between passes, mm. Hard band
    /// `(0, 0.2]` (the shared `[plr]` schema band): this value IS the
    /// descent overshoot — the first contacting pass sits at most
    /// `drag_z_step` below the true surface — so it is capped at
    /// layer-height scale (and grows the envelope one-for-one); it
    /// must be strictly positive for the staircase to make progress.
    pub drag_z_step: f64,
    /// ADXL drag: unitless 0–100 sensitivity knob, mapped by the
    /// plugin onto a detection threshold over the calibrated noise
    /// floor. Hard band `[0, 100]` (the shared `[plr]` schema band);
    /// plrd passes it verbatim to `PLR_DRAG_PROBE` — the knob's
    /// mapping is the plugin's contract, the descent bound never
    /// depends on it (the shifted frame limits travel structurally).
    pub drag_sensitivity: f64,
    /// Consensus touch: number of agreeing samples the median is taken
    /// over (`PLR_TOUCH SAMPLES=`). Hard band `[3, 7]`, integral: fewer
    /// than three cannot form a consensus, more than seven wears the
    /// part for no statistical gain (Cartographer's touch default is 3,
    /// `interfaces/configuration.py`). Tap / load-cell only; ignored on
    /// the drag path, but always validated.
    pub touch_samples: f64,
    /// Consensus touch: the acceptable spread (max − min, mm) of the
    /// agreeing subset (`PLR_TOUCH SAMPLE_RANGE=`). Default 0.010; a
    /// **hard cap of 0.015 mm** — values above it are REFUSED, never
    /// clamped, mirroring Cartographer's `sample_range` option ceiling
    /// (`interfaces/configuration.py:248`,
    /// `default=0.010, min=0.001, max=0.015`): a wider consensus band
    /// admits touches too noisy to trust as a Z datum.
    pub touch_sample_range: f64,
    /// Consensus touch: retract distance between/after touches
    /// (`PLR_TOUCH RETRACT=`, mm). Minimum 1.0 — the nozzle must clear
    /// the surface between touches so each descent starts clean
    /// (Cartographer retracts to `retract_distance` before and after
    /// each probe, `touch_mode.py:257-279`). Tap / load-cell only.
    pub touch_retract: f64,
    /// Consensus touch: the `max_accel` clamp applied around the touch
    /// phase (`PLR_TOUCH TOUCH_ACCEL=` and the plan-level
    /// `SET_VELOCITY_LIMIT` clamp). Hard band `[50, 1000]` mm/s²;
    /// Cartographer's `TOUCH_ACCEL` is 100 (`touch_mode.py:31`). A
    /// gentle accel keeps the tap from over-driving the nozzle into the
    /// part before the trigger is observed.
    pub touch_accel: f64,
    /// Use the legacy single-sample `PROBE SAMPLES=1` step instead of
    /// the consensus `PLR_TOUCH` sequence on Tap / load-cell machines.
    /// Set by the legacy `/etc/plrd.conf [machine]` path, where the
    /// Klipper plugin (and therefore its `PLR_TOUCH` command) may not
    /// be loaded — the stock `PROBE` is all that can be assumed. The
    /// `[plr]` path leaves it `false` (plugin present ⇒ consensus
    /// touch). Ignored on the drag path (drag always uses
    /// `PLR_DRAG_PROBE`).
    pub legacy_single_probe: bool,
}

impl Default for PlanConfig {
    fn default() -> Self {
        Self {
            probe_speed: 1.0,
            margin: 0.5,
            sag_allowance: 0.2,
            idle_timeout_s: 86_400.0,
            probe_nozzle_temp: 150.0,
            probe_temp_min: 140.0,
            probe_temp_max: 160.0,
            temp_epsilon: 3.0,
            travel_feed: 6_000.0,
            entry_feed: 1_200.0,
            entry_hop: 1.0,
            prime_mm: 0.4,
            prime_feed: 1_800.0,
            xy_epsilon: 0.25,
            z_epsilon: 0.05,
            // The [plr] schema defaults (klippy_plugin/plr/tunables.py).
            drag_speed: 20.0,
            drag_z_step: 0.05,
            drag_sensitivity: 30.0,
            // Consensus-touch defaults (mirror Cartographer's touch
            // config; see the field docs).
            touch_samples: 3.0,
            touch_sample_range: 0.010,
            touch_retract: 2.0,
            touch_accel: 100.0,
            legacy_single_probe: false,
        }
    }
}

impl PlanConfig {
    /// Validates every field (finiteness and documented bounds).
    ///
    /// # Errors
    ///
    /// [`RecoveryError::InvalidPlanConfig`] naming the first offending
    /// field. The probe speed band is enforced later by
    /// [`compute_envelope`].
    pub fn validate(&self) -> Result<(), RecoveryError> {
        let checks: [(&'static str, f64, bool); 21] = [
            ("margin", self.margin, self.margin >= 0.0),
            (
                "sag_allowance",
                self.sag_allowance,
                self.sag_allowance >= 0.0,
            ),
            (
                "idle_timeout_s",
                self.idle_timeout_s,
                self.idle_timeout_s >= 600.0,
            ),
            (
                "probe_temp_min",
                self.probe_temp_min,
                self.probe_temp_min >= 140.0,
            ),
            (
                "probe_temp_max",
                self.probe_temp_max,
                self.probe_temp_max <= 160.0 && self.probe_temp_max > self.probe_temp_min,
            ),
            (
                "probe_nozzle_temp",
                self.probe_nozzle_temp,
                self.probe_nozzle_temp >= self.probe_temp_min
                    && self.probe_nozzle_temp <= self.probe_temp_max,
            ),
            ("temp_epsilon", self.temp_epsilon, self.temp_epsilon > 0.0),
            ("travel_feed", self.travel_feed, self.travel_feed > 0.0),
            (
                "entry_feed",
                self.entry_feed,
                self.entry_feed > 0.0 && self.entry_feed <= 1_800.0,
            ),
            ("entry_hop", self.entry_hop, self.entry_hop >= 0.0),
            ("prime_mm", self.prime_mm, self.prime_mm >= 0.0),
            ("prime_feed", self.prime_feed, self.prime_feed > 0.0),
            ("xy_epsilon", self.xy_epsilon, self.xy_epsilon > 0.0),
            ("z_epsilon", self.z_epsilon, self.z_epsilon > 0.0),
            // Drag tunable bands: the FIXED [plr] schema shared with
            // the Klipper plugin (see the field docs).
            (
                "drag_speed",
                self.drag_speed,
                self.drag_speed > 0.0 && self.drag_speed <= 100.0,
            ),
            (
                "drag_z_step",
                self.drag_z_step,
                self.drag_z_step > 0.0 && self.drag_z_step <= 0.2,
            ),
            (
                "drag_sensitivity",
                self.drag_sensitivity,
                self.drag_sensitivity >= 0.0 && self.drag_sensitivity <= 100.0,
            ),
            // Consensus-touch tunables (see the field docs).
            (
                "touch_samples",
                self.touch_samples,
                self.touch_samples >= 3.0
                    && self.touch_samples <= 7.0
                    && self.touch_samples.fract() == 0.0,
            ),
            (
                // Hard cap 0.015 (Cartographer configuration.py:248);
                // above it is REFUSED, not clamped.
                "touch_sample_range",
                self.touch_sample_range,
                self.touch_sample_range > 0.0 && self.touch_sample_range <= 0.015,
            ),
            (
                "touch_retract",
                self.touch_retract,
                self.touch_retract >= 1.0,
            ),
            (
                "touch_accel",
                self.touch_accel,
                self.touch_accel >= 50.0 && self.touch_accel <= 1000.0,
            ),
        ];
        for (field, value, in_range) in checks {
            if !value.is_finite() {
                return Err(RecoveryError::NonFinite { field });
            }
            if !in_range {
                return Err(RecoveryError::InvalidPlanConfig { field });
            }
        }
        // probe_speed: non-finite values fall out of the band check in
        // compute_envelope; nothing to do here.
        Ok(())
    }
}

/// One `exclude_object` definition to restore after `M23` (whose
/// `virtual_sdcard:reset_file` tears down exclude-object state).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExcludeObjectDef {
    /// Object name as the slicer defined it.
    pub name: String,
    /// Optional `CENTER=x,y`.
    pub center: Option<[f64; 2]>,
    /// Optional `POLYGON=[[x,y],...]` outline.
    pub polygon: Vec<[f64; 2]>,
    /// `true` when the object was already excluded before the crash
    /// (re-excluded after redefinition).
    pub currently_excluded: bool,
}

/// Why the planner degraded to manual recovery. These are defined,
/// safe outcomes — not errors.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FallbackReason {
    /// The contact-zone selector declined to probe
    /// (vase mode, single wall, no safe zone, ...).
    ContactDeclined(DeclineReason),
    /// The stop-point match established only a layer; automatic resume
    /// at line granularity is refused.
    MatchTooCoarse {
        /// The layer the match established.
        layer: u32,
    },
    /// No depositing move exists at or after the matched offset.
    NoResumeDeposition,
    /// The resume move's position is G28-unknown or non-finite.
    ResumePositionUnknown,
}

/// Outcome of [`plan_recovery`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PlanOutcome {
    /// The WAL ended with a clean shutdown: no recovery, no plan.
    NoRecoveryNeeded,
    /// The full §8 recovery plan.
    Plan(Box<RecoveryPlan>),
    /// Automatic recovery is declined; the operator must recover
    /// manually (typed reason attached).
    ManualFallback {
        /// Why automation declined.
        reason: FallbackReason,
    },
}

/// The selected resume point in the print file.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ResumeTarget {
    /// Line-boundary byte offset for `M26 S` (the start of the resume
    /// line, per the `plr-gcode` span contract).
    pub offset: u64,
    /// Klipper-internal position `[x, y, z, e]` at the resume move's
    /// start.
    pub position: [f64; 4],
    /// Layer index of the resume move, when known.
    pub layer: Option<u32>,
    /// `true` when the resume line is infill (internal or solid) —
    /// the preferred, seam-hiding case.
    pub on_infill: bool,
}

/// Selects the resume target from the match result (design doc §8,
/// step 12): the **latest** plausible stop offset (skip-forward is the
/// conservative direction — resuming earlier would double-extrude over
/// printed geometry), then the first depositing move at or after it.
///
/// # Errors
///
/// A typed [`FallbackReason`] when the match is too coarse or no safe
/// deposition exists — the caller degrades to manual recovery.
pub fn select_resume_target(
    model: &LayerModel,
    result: &MatchResult,
) -> Result<ResumeTarget, FallbackReason> {
    let base = match &result.confidence {
        MatchConfidence::UniqueLine { offset } => *offset,
        MatchConfidence::AmbiguousWindow { offsets } => offsets
            .iter()
            .max()
            .copied()
            .ok_or(FallbackReason::NoResumeDeposition)?,
        MatchConfidence::LayerOnly { layer } => {
            return Err(FallbackReason::MatchTooCoarse { layer: *layer })
        }
    };
    let mv = model
        .first_deposition_at_or_after(base)
        .ok_or(FallbackReason::NoResumeDeposition)?;
    let xyz_known = mv.start_known[0] && mv.start_known[1] && mv.start_known[2];
    if !xyz_known || !mv.start.iter().all(|v| v.is_finite()) {
        return Err(FallbackReason::ResumePositionUnknown);
    }
    let on_infill = mv
        .layer
        .and_then(|idx| model.layer(idx))
        .is_some_and(|layer| {
            layer.paths.iter().any(|p| {
                matches!(
                    p.class,
                    FeatureClass::InternalInfill | FeatureClass::SolidInfill
                ) && p.segments.iter().any(|s| s.span.start == mv.span.start)
            })
        });
    Ok(ResumeTarget {
        offset: mv.span.start,
        position: mv.start,
        layer: mv.layer,
        on_infill,
    })
}

/// Everything [`plan_recovery`] consumes. All borrowed; no I/O.
#[derive(Debug, Clone, Copy)]
pub struct PlanInputs<'a> {
    /// The machine snapshot (validated internally).
    pub machine: &'a MachineConfig,
    /// The reconstruction outcome (clean shutdown short-circuits).
    pub reconstruction: &'a Reconstruction,
    /// The contact-zone selection.
    pub contact: &'a ContactOutcome,
    /// The stop-point match.
    pub match_result: &'a MatchResult,
    /// The layer model the match was computed against. Must be built
    /// by replaying from the WAL-reconstructed g-code state
    /// ([`plr_analyzer::build_layer_model`] with the seeded state), so
    /// its internal coordinate frame is the WAL frame — the entry-move
    /// arithmetic converts model positions to file coordinates using
    /// the WAL homing origin.
    pub model: &'a LayerModel,
    /// File temperature scan ([`crate::preheat::scan_file_temps`]).
    pub file_temps: FileTemps,
    /// Exclude-object definitions to restore after `M23`.
    pub exclude_objects: &'a [ExcludeObjectDef],
}

/// Validated numeric view of the WAL g-code state.
struct GcodeNumbers {
    origin: [f64; 4],
    speed_factor: f64,
    extrude_factor: f64,
    speed_raw: f64,
    absolute_coordinates: bool,
    absolute_extrude: bool,
}

/// Extracts and validates the g-code state numbers.
fn validate_gcode_state(g: &plr_wal::GcodeState) -> Result<GcodeNumbers, RecoveryError> {
    if g.homing_origin.len() < 4 || g.position.len() < 4 {
        return Err(RecoveryError::InvalidContext {
            field: "gcode coordinate vectors",
        });
    }
    if !(g.speed_factor.is_finite() && g.speed_factor > 0.0) {
        return Err(RecoveryError::InvalidContext {
            field: "speed_factor",
        });
    }
    if !(g.extrude_factor.is_finite() && g.extrude_factor > 0.0) {
        return Err(RecoveryError::InvalidContext {
            field: "extrude_factor",
        });
    }
    // WAL `speed` is the raw commanded F value in mm/min (Klipper's
    // status `speed` with M220 divided out). Emitted verbatim as
    // `G1 F{speed}`; M220 is restored separately. Treating this as
    // mm/s would run the resume 60x slow.
    if !(g.speed.is_finite() && g.speed > 0.0) {
        return Err(RecoveryError::InvalidContext { field: "speed" });
    }
    let mut origin = [0.0; 4];
    for (slot, value) in origin.iter_mut().zip(g.homing_origin.iter()) {
        *slot = *value;
    }
    Ok(GcodeNumbers {
        origin,
        speed_factor: g.speed_factor,
        extrude_factor: g.extrude_factor,
        speed_raw: g.speed,
        absolute_coordinates: g.absolute_coordinates,
        absolute_extrude: g.absolute_extrude,
    })
}

/// Validates that the print file sits at the `virtual_sdcard` top level
/// (`M23` cannot select subdirectory files) and returns the bare name.
fn top_level_file_name(path: &str, root: &str) -> Result<String, RecoveryError> {
    let root_trimmed = root.trim_end_matches(['/', '\\']);
    let not_top_level = || RecoveryError::FileNotTopLevel {
        path: path.to_owned(),
    };
    let rest = path.strip_prefix(root_trimmed).ok_or_else(not_top_level)?;
    if !rest.starts_with(['/', '\\']) {
        return Err(not_top_level());
    }
    let name = rest.trim_start_matches(['/', '\\']);
    if name.is_empty() || name.contains(['/', '\\']) {
        return Err(not_top_level());
    }
    Ok(name.to_owned())
}

/// Rejects names that cannot be safely embedded in a command line.
fn validate_command_name(field: &'static str, name: &str) -> Result<(), RecoveryError> {
    let bad = name.is_empty()
        || name
            .chars()
            .any(|c| c.is_whitespace() || c.is_control() || matches!(c, '=' | '"' | '\''));
    if bad {
        return Err(RecoveryError::InvalidName {
            field,
            name: name.to_owned(),
        });
    }
    Ok(())
}

/// Validates exclude-object definitions.
fn validate_excludes(excludes: &[ExcludeObjectDef]) -> Result<(), RecoveryError> {
    for def in excludes {
        validate_command_name("exclude_object", &def.name)?;
        let center_finite = def.center.is_none_or(|c| c.iter().all(|v| v.is_finite()));
        let polygon_finite = def.polygon.iter().all(|p| p.iter().all(|v| v.is_finite()));
        if !center_finite || !polygon_finite {
            return Err(RecoveryError::NonFinite {
                field: "exclude_object geometry",
            });
        }
    }
    Ok(())
}

/// Restore commands for the WAL fan targets, with warnings for shapes
/// that cannot be restored. `heater_fan`/`controller_fan` are
/// Klipper-automatic and skipped silently.
fn fan_commands(fans: &[(String, f64)]) -> (Vec<String>, Vec<PlanWarning>) {
    let mut commands = Vec::new();
    let mut warnings = Vec::new();
    for (name, speed) in fans {
        if name == "fan" {
            commands.push(format!("M106 S{}", fmt_num((speed * 255.0).round())));
        } else if let Some(suffix) = name.strip_prefix("fan_generic ") {
            if validate_command_name("fan", suffix).is_ok() {
                commands.push(format!(
                    "SET_FAN_SPEED FAN={suffix} SPEED={}",
                    fmt_num(*speed)
                ));
            } else {
                warnings.push(PlanWarning::UnrestorableFan { name: name.clone() });
            }
        } else if name.starts_with("heater_fan") || name.starts_with("controller_fan") {
            // Automatic fans: Klipper drives them; nothing to restore.
        } else {
            warnings.push(PlanWarning::UnrestorableFan { name: name.clone() });
        }
    }
    (commands, warnings)
}

/// Everything the step constructors need, computed once.
struct Ctx<'a> {
    cfg: &'a PlanConfig,
    machine: &'a ValidatedMachine,
    context: &'a Context,
    gcode: GcodeNumbers,
    envelope: Envelope,
    candidate: &'a ProbeCandidate,
    formula: TrueZFormula,
    resume: ResumeTarget,
    nozzle_print: f64,
    bed: Option<f64>,
    other_heaters: &'a [(String, f64)],
    fan_cmds: &'a [String],
    file_name: &'a str,
    file_path: &'a str,
    excludes: &'a [ExcludeObjectDef],
}

fn step(
    phase: Phase,
    summary: &str,
    commands: Vec<String>,
    pre_verify: Vec<Verification>,
    verify: Vec<Verification>,
    compute: Option<RuntimeComputation>,
    reason: AbortReason,
) -> RecoveryStep {
    RecoveryStep {
        id: 0, // renumbered by the builder
        phase,
        summary: summary.to_owned(),
        commands,
        pre_verify,
        verify,
        compute,
        cleanup_commands: Vec::new(),
        on_failure: FailureAction::Abort { reason },
    }
}

fn step_idle_timeout(ctx: &Ctx<'_>) -> RecoveryStep {
    let timeout = ctx.cfg.idle_timeout_s;
    step(
        Phase::IdleTimeout,
        "disarm the idle timeout FIRST (its default M84 would clear all homed state)",
        vec![format!("SET_IDLE_TIMEOUT TIMEOUT={}", fmt_num(timeout))],
        vec![],
        vec![Verification::new(
            "idle_timeout",
            "idle_timeout",
            Predicate::NumWithin {
                expected: timeout,
                epsilon: 0.5,
            },
        )],
        None,
        AbortReason::IdleTimeoutNotApplied,
    )
}

fn step_stepper_enable(ctx: &Ctx<'_>) -> RecoveryStep {
    let mut commands = Vec::new();
    let mut verify = Vec::new();
    for name in &ctx.machine.z_stepper_names {
        commands.push(format!("SET_STEPPER_ENABLE STEPPER={name} ENABLE=1"));
        verify.push(Verification::new(
            "stepper_enable",
            &format!("steppers.{name}"),
            Predicate::BoolTrue,
        ));
    }
    step(
        Phase::StepperEnable,
        "energize the Z steppers (enabling never touches homed state; there is no M17)",
        commands,
        vec![],
        verify,
        None,
        AbortReason::StepperEnableFailed,
    )
}

fn step_preheat(ctx: &Ctx<'_>) -> RecoveryStep {
    let mut commands = Vec::new();
    let mut verify = Vec::new();
    if let Some(bed) = ctx.bed {
        commands.push(format!("M140 S{}", fmt_num(bed)));
        verify.push(Verification::new(
            "heater_bed",
            "temperature",
            Predicate::NumAtLeast {
                min: bed - ctx.cfg.temp_epsilon,
            },
        ));
    }
    for (name, target) in ctx.other_heaters {
        let short = name.rsplit(' ').next().unwrap_or(name);
        if validate_command_name("heater", short).is_ok() {
            commands.push(format!(
                "SET_HEATER_TEMPERATURE HEATER={short} TARGET={}",
                fmt_num(*target)
            ));
        }
    }
    commands.push(format!("M104 S{}", fmt_num(ctx.cfg.probe_nozzle_temp)));
    verify.push(Verification::new(
        "extruder",
        "temperature",
        Predicate::TempWithin {
            min: ctx.cfg.probe_temp_min,
            max: ctx.cfg.probe_temp_max,
        },
    ));
    step(
        Phase::Preheat,
        "bed to target; nozzle to the warm-but-below-ooze probing band",
        commands,
        vec![],
        verify,
        None,
        AbortReason::PreheatFailed,
    )
}

fn step_home_xy() -> RecoveryStep {
    step(
        Phase::HomeXy,
        "home XY only (never bare G28, never Z: the bed rises into a fixed gantry)",
        vec!["G28 X Y".to_owned()],
        vec![],
        vec![
            Verification::new(
                "toolhead",
                "homed_axes",
                Predicate::Contains {
                    needle: "x".to_owned(),
                },
            ),
            Verification::new(
                "toolhead",
                "homed_axes",
                Predicate::Contains {
                    needle: "y".to_owned(),
                },
            ),
        ],
        None,
        AbortReason::HomingFailed,
    )
}

fn step_transform_freeze() -> RecoveryStep {
    step(
        Phase::TransformFreeze,
        "freeze z_thermal_adjust at its last value (disabling freezes, not zeroes)",
        vec!["SET_Z_THERMAL_ADJUST ENABLE=0".to_owned()],
        vec![],
        vec![
            Verification::new("z_thermal_adjust", "enabled", Predicate::BoolFalse),
            Verification::new(
                "z_thermal_adjust",
                "current_z_adjust",
                Predicate::FinitePresent,
            ),
        ],
        None,
        AbortReason::TransformFreezeFailed,
    )
}

fn step_shifted_frame(ctx: &Ctx<'_>) -> RecoveryStep {
    let z = ctx.envelope.shifted_declare_z;
    step(
        Phase::ShiftedFrame,
        "declare the shifted frame: Klipper's rail limit now structurally bounds the descent",
        vec![format!("SET_KINEMATIC_POSITION Z={}", fmt_num(z))],
        vec![],
        vec![
            Verification::new(
                "toolhead",
                "homed_axes",
                Predicate::Contains {
                    needle: "z".to_owned(),
                },
            ),
            Verification::new(
                "toolhead",
                "position.2",
                Predicate::NumWithin {
                    expected: z,
                    epsilon: ctx.cfg.z_epsilon,
                },
            ),
        ],
        None,
        AbortReason::ShiftedFrameNotDeclared,
    )
}

fn step_probe_approach(ctx: &Ctx<'_>) -> RecoveryStep {
    let [px, py] = ctx.candidate.point;
    step(
        Phase::ProbeApproach,
        "XY travel to the selected contact point (no Z motion)",
        vec![
            "G90".to_owned(),
            format!(
                "G0 X{} Y{} F{}",
                fmt_num(px),
                fmt_num(py),
                fmt_num(ctx.cfg.travel_feed)
            ),
        ],
        vec![],
        vec![
            Verification::new(
                "toolhead",
                "position.0",
                Predicate::NumWithin {
                    expected: px,
                    epsilon: ctx.cfg.xy_epsilon,
                },
            ),
            Verification::new(
                "toolhead",
                "position.1",
                Predicate::NumWithin {
                    expected: py,
                    epsilon: ctx.cfg.xy_epsilon,
                },
            ),
        ],
        None,
        AbortReason::ApproachFailed,
    )
}

/// `true` when this plan probes Tap / load-cell machines with the
/// consensus `PLR_TOUCH` sequence rather than the legacy single `PROBE`.
/// Drag machines are never touch-consensus (they run `PLR_DRAG_PROBE`).
fn uses_consensus_touch(machine: &ValidatedMachine, cfg: &PlanConfig) -> bool {
    matches!(machine.probe.kind, ProbeKind::Tap | ProbeKind::LoadCell) && !cfg.legacy_single_probe
}

fn step_accel_clamp(ctx: &Ctx<'_>) -> RecoveryStep {
    // Record the pre-clamp max_accel (RecordMaxAccel reads it BEFORE the
    // SET_VELOCITY_LIMIT below runs), clamp it to the touch accel, and
    // declare the abort cleanup that restores it — the plan-level
    // `finally` (Cartographer touch_mode.py:262-274). The success-path
    // restore is the separate AccelRestore step.
    let mut s = step(
        Phase::AccelClamp,
        "clamp max_accel to the touch accel around the consensus touch (restored after / on abort)",
        vec![format!(
            "SET_VELOCITY_LIMIT ACCEL={}",
            fmt_num(ctx.cfg.touch_accel)
        )],
        vec![],
        vec![Verification::new(
            "toolhead",
            "max_accel",
            Predicate::NumWithin {
                expected: ctx.cfg.touch_accel,
                epsilon: 1.0,
            },
        )],
        Some(RuntimeComputation::RecordMaxAccel),
        AbortReason::AccelClampFailed,
    );
    s.cleanup_commands = vec![format!(
        "SET_VELOCITY_LIMIT ACCEL={RESTORE_ACCEL_PLACEHOLDER}"
    )];
    s
}

fn step_accel_restore() -> RecoveryStep {
    step(
        Phase::AccelRestore,
        "restore the pre-clamp max_accel on the success path",
        vec![format!(
            "SET_VELOCITY_LIMIT ACCEL={RESTORE_ACCEL_PLACEHOLDER}"
        )],
        vec![],
        vec![Verification::new(
            "toolhead",
            "max_accel",
            Predicate::NumWithinComputed { epsilon: 1.0 },
        )],
        // No compute: the daemon reuses the pre-clamp accel the
        // accel-clamp step already recorded (it must NOT re-read
        // max_accel here — that would read the clamped value). Both the
        // {restore_accel} substitution and the NumWithinComputed check
        // resolve against that stored value.
        None,
        AbortReason::AccelRestoreFailed,
    )
}

fn step_probe(ctx: &Ctx<'_>) -> RecoveryStep {
    // Trigger readback location per probe method. The consensus and
    // drag results live on the plugin's own `plr` status object.
    let (trigger_object, trigger_field) = match ctx.formula.trigger_source {
        TriggerSource::RawLastZResult => ("probe", "last_z_result"),
        TriggerSource::BedZPlusOffset { .. } => ("probe", "last_probe_position.2"),
        TriggerSource::DragResult => ("plr", "last_drag_result.trigger_z"),
        TriggerSource::TouchResult { .. } => ("plr", "last_touch_result.median_z"),
    };
    // Command + summary + method-specific post-verifications.
    let consensus = uses_consensus_touch(ctx.machine, ctx.cfg);
    let (summary, command, mut verify) = match &ctx.machine.probe.kind {
        ProbeKind::Tap | ProbeKind::LoadCell if consensus => (
            "consensus multi-touch (PLR_TOUCH: sliding-window best subset; median is the trigger Z)",
            format!(
                "PLR_TOUCH SAMPLES={} SAMPLE_RANGE={} SPEED={} RETRACT={} TOUCH_ACCEL={}",
                fmt_num(ctx.cfg.touch_samples),
                fmt_num(ctx.cfg.touch_sample_range),
                fmt_num(ctx.cfg.probe_speed),
                fmt_num(ctx.cfg.touch_retract),
                fmt_num(ctx.cfg.touch_accel),
            ),
            vec![
                // The consensus actually converged: the reported spread
                // is inside the band and enough samples agreed.
                Verification::new(
                    "plr",
                    "last_touch_result.range",
                    Predicate::NumAtMost {
                        max: ctx.cfg.touch_sample_range,
                    },
                ),
                Verification::new(
                    "plr",
                    "last_touch_result.samples_used",
                    Predicate::NumAtLeast {
                        min: ctx.cfg.touch_samples,
                    },
                ),
            ],
        ),
        ProbeKind::Tap | ProbeKind::LoadCell => (
            "single-sample probe (SAMPLES=1: the toolhead rests exactly at the halt position)",
            format!(
                "PROBE PROBE_SPEED={} SAMPLES=1",
                fmt_num(ctx.cfg.probe_speed)
            ),
            vec![],
        ),
        ProbeKind::AdxlDrag { chip } => (
            "ADXL drag probe (bounded fixed-Z staircase; the accelerometer hears the contact)",
            format!(
                // CHIP is ALWAYS double-quoted: klippy's extended-command
                // parser shlex-parses quoted values on every ingress path
                // plrd uses (gcode.py `_get_extended_params` 145-151 +
                // posix shlex 266-281), so spaced section names like
                // `adxl345 bed` arrive intact — and quoting a space-free
                // name is equally valid. Names quoting cannot carry are
                // refused by validation (`machine::chip_embeddable`).
                "PLR_DRAG_PROBE CHIP=\"{chip}\" SPEED={} Z_STEP={} SENSITIVITY={}",
                fmt_num(ctx.cfg.drag_speed),
                fmt_num(ctx.cfg.drag_z_step),
                fmt_num(ctx.cfg.drag_sensitivity),
            ),
            vec![],
        ),
    };
    // The trigger readback is present and finite for every method.
    verify.push(Verification::new(
        trigger_object,
        trigger_field,
        Predicate::FinitePresent,
    ));
    step(
        Phase::Probe,
        summary,
        vec![command],
        probe_pre_verify(ctx),
        verify,
        None,
        AbortReason::ProbeNoTrigger,
    )
}

/// The mandatory, daemon-enforced probe pre-verifications: the nozzle
/// temperature interlock (both CURRENT temperature and the extruder
/// TARGET inside the probe band — the `max(current, target)` guard from
/// Cartographer `touch_mode.py:299-303`, so a nozzle commanded to print
/// temperature while transiently cool still refuses) and XYZ homed.
fn probe_pre_verify(ctx: &Ctx<'_>) -> Vec<Verification> {
    let homed = |axis: &str| {
        Verification::new(
            "toolhead",
            "homed_axes",
            Predicate::Contains {
                needle: axis.to_owned(),
            },
        )
    };
    vec![
        Verification::new(
            "extruder",
            "temperature",
            Predicate::TempWithin {
                min: ctx.cfg.probe_temp_min,
                max: ctx.cfg.probe_temp_max,
            },
        ),
        Verification::new(
            "extruder",
            "target",
            Predicate::NumAtMost {
                max: ctx.cfg.probe_temp_max,
            },
        ),
        homed("x"),
        homed("y"),
        homed("z"),
    ]
}

fn step_true_z_declare(ctx: &Ctx<'_>) -> RecoveryStep {
    step(
        Phase::TrueZDeclare,
        "true-Z arithmetic and kinematic re-declaration (never a gcode offset)",
        vec![format!("SET_KINEMATIC_POSITION Z={TRUE_Z_PLACEHOLDER}")],
        vec![],
        vec![Verification::new(
            "toolhead",
            "position.2",
            Predicate::NumWithinComputed {
                epsilon: ctx.cfg.z_epsilon,
            },
        )],
        Some(RuntimeComputation::TrueZ(ctx.formula)),
        AbortReason::TrueZDeclareFailed,
    )
}

fn step_mesh_load(profile: &str) -> RecoveryStep {
    step(
        Phase::MeshLoad,
        "restore the bed mesh (not auto-loaded after restart; gated on the WAL mesh_matrix)",
        vec![format!("BED_MESH_PROFILE LOAD={profile}")],
        vec![],
        vec![Verification::new(
            "bed_mesh",
            "mesh_matrix",
            Predicate::NonEmptyMatrix,
        )],
        None,
        AbortReason::MeshLoadFailed,
    )
}

fn step_final_declare(ctx: &Ctx<'_>) -> RecoveryStep {
    step(
        Phase::FinalDeclare,
        "final true-frame declaration after all transforms are in place",
        vec![format!("SET_KINEMATIC_POSITION Z={TRUE_Z_PLACEHOLDER}")],
        vec![],
        vec![
            Verification::new(
                "toolhead",
                "homed_axes",
                Predicate::Contains {
                    needle: "z".to_owned(),
                },
            ),
            Verification::new(
                "toolhead",
                "position.2",
                Predicate::NumWithinComputed {
                    epsilon: ctx.cfg.z_epsilon,
                },
            ),
        ],
        Some(RuntimeComputation::TrueZ(ctx.formula)),
        AbortReason::FinalDeclareFailed,
    )
}

fn step_restore_frame(ctx: &Ctx<'_>) -> RecoveryStep {
    let g = &ctx.gcode;
    // After the probe the nozzle rests pressed against layer N−1. It
    // must lift off *before* the print temperature is restored — the
    // temperature verification below polls for minutes, and a nozzle
    // dwelling at print temperature against plastic melts a divot. The
    // lift is a bounded relative move in the safe direction (away from
    // the part), never less than 0.5 mm.
    let lift = ctx.cfg.entry_hop.max(0.5);
    let mut commands = vec![
        "G91".to_owned(),
        format!("G1 Z{} F{}", fmt_num(lift), fmt_num(ctx.cfg.entry_feed)),
        "G90".to_owned(),
        format!(
            "SET_GCODE_OFFSET X={} Y={} Z={}",
            fmt_num(g.origin[0]),
            fmt_num(g.origin[1]),
            fmt_num(g.origin[2])
        ),
        format!("M220 S{}", fmt_num(g.speed_factor * 100.0)),
        format!("M221 S{}", fmt_num(g.extrude_factor * 100.0)),
    ];
    let transforms = &ctx.context.transforms;
    if transforms.skew_active {
        if let Some(profile) = transforms.skew_profile.as_deref().filter(|p| !p.is_empty()) {
            commands.push(format!("SKEW_PROFILE LOAD={profile}"));
        }
    }
    commands.push(format!("M104 S{}", fmt_num(ctx.nozzle_print)));
    if let Some(bed) = ctx.bed {
        commands.push(format!("M140 S{}", fmt_num(bed)));
    }
    commands.extend_from_slice(ctx.fan_cmds);
    // Raw F value in mm/min from the WAL; M220 restored above.
    commands.push(format!("G1 F{}", fmt_num(g.speed_raw)));
    let mut verify = vec![
        Verification::new(
            "gcode_move",
            "speed_factor",
            Predicate::NumWithin {
                expected: g.speed_factor,
                epsilon: 0.01,
            },
        ),
        Verification::new(
            "gcode_move",
            "extrude_factor",
            Predicate::NumWithin {
                expected: g.extrude_factor,
                epsilon: 0.01,
            },
        ),
        Verification::new(
            "extruder",
            "temperature",
            Predicate::TempWithin {
                min: ctx.nozzle_print - ctx.cfg.temp_epsilon,
                max: ctx.nozzle_print + ctx.cfg.temp_epsilon,
            },
        ),
    ];
    if let Some(bed) = ctx.bed {
        verify.push(Verification::new(
            "heater_bed",
            "temperature",
            Predicate::NumAtLeast {
                min: bed - ctx.cfg.temp_epsilon,
            },
        ));
    }
    step(
        Phase::RestoreFrame,
        "lift off the part, then replay offsets, factors, skew, print temperatures, fans, feedrate",
        commands,
        vec![],
        verify,
        None,
        AbortReason::RestoreFailed,
    )
}

fn step_entry(ctx: &Ctx<'_>) -> Result<RecoveryStep, RecoveryError> {
    let g = &ctx.gcode;
    let [internal_x, internal_y, internal_z, internal_e] = ctx.resume.position;
    let gcode_x = internal_x - g.origin[0];
    let gcode_y = internal_y - g.origin[1];
    let gcode_z = internal_z - g.origin[2];
    let entry_z = gcode_z + ctx.cfg.entry_hop;
    let file_e = (internal_e - g.origin[3]) / g.extrude_factor;
    for (field, v) in [
        ("entry_x", gcode_x),
        ("entry_y", gcode_y),
        ("entry_z", entry_z),
        ("resume_z", gcode_z),
        ("file_e", file_e),
    ] {
        if !v.is_finite() {
            return Err(RecoveryError::NonFinite { field });
        }
    }
    let feed = fmt_num(ctx.cfg.entry_feed);
    let mut commands = vec![
        "G90".to_owned(),
        "M83".to_owned(),
        format!("G0 Z{} F{feed}", fmt_num(entry_z)),
        format!("G0 X{} Y{} F{feed}", fmt_num(gcode_x), fmt_num(gcode_y)),
        format!("G1 Z{} F{feed}", fmt_num(gcode_z)),
    ];
    if ctx.cfg.prime_mm > 0.0 {
        commands.push(format!(
            "G1 E{} F{}",
            fmt_num(ctx.cfg.prime_mm),
            fmt_num(ctx.cfg.prime_feed)
        ));
    }
    commands.push(format!("G92 E{}", fmt_num(file_e)));
    commands.push(if g.absolute_extrude { "M82" } else { "M83" }.to_owned());
    commands.push(if g.absolute_coordinates { "G90" } else { "G91" }.to_owned());
    // Re-assert the file feedrate: the entry moves' F words overwrote
    // the §8.9 restore.
    commands.push(format!("G1 F{}", fmt_num(g.speed_raw)));
    Ok(step(
        Phase::Entry,
        "enter from above the part interior, speed-limited; prime; final E frame and modes",
        commands,
        vec![],
        vec![
            Verification::new(
                "toolhead",
                "position.0",
                Predicate::NumWithin {
                    expected: internal_x,
                    epsilon: ctx.cfg.xy_epsilon,
                },
            ),
            Verification::new(
                "toolhead",
                "position.1",
                Predicate::NumWithin {
                    expected: internal_y,
                    epsilon: ctx.cfg.xy_epsilon,
                },
            ),
        ],
        None,
        AbortReason::EntryFailed,
    ))
}

fn step_file_select(ctx: &Ctx<'_>) -> RecoveryStep {
    let mut commands = vec![format!("M23 {}", ctx.file_name)];
    for def in ctx.excludes {
        let mut cmd = format!("EXCLUDE_OBJECT_DEFINE NAME={}", def.name);
        if let Some([cx, cy]) = def.center {
            let _ = write!(cmd, " CENTER={},{}", fmt_num(cx), fmt_num(cy));
        }
        if !def.polygon.is_empty() {
            let points: Vec<String> = def
                .polygon
                .iter()
                .map(|[x, y]| format!("[{},{}]", fmt_num(*x), fmt_num(*y)))
                .collect();
            let _ = write!(cmd, " POLYGON=[{}]", points.join(","));
        }
        commands.push(cmd);
    }
    for def in ctx.excludes.iter().filter(|d| d.currently_excluded) {
        commands.push(format!("EXCLUDE_OBJECT NAME={}", def.name));
    }
    #[allow(clippy::cast_precision_loss)] // verification tolerance is 0.5
    let offset_f = ctx.resume.offset as f64;
    commands.push(format!("M26 S{}", ctx.resume.offset));
    step(
        Phase::FileSelect,
        "select the file (top level only), restore exclude-object state, seek to the line boundary",
        commands,
        vec![],
        vec![
            Verification::new(
                "virtual_sdcard",
                "file_path",
                Predicate::Equals {
                    value: ctx.file_path.to_owned(),
                },
            ),
            Verification::new(
                "virtual_sdcard",
                "file_position",
                Predicate::NumWithin {
                    expected: offset_f,
                    epsilon: 0.5,
                },
            ),
        ],
        None,
        AbortReason::FileSelectFailed,
    )
}

fn step_resume_start() -> RecoveryStep {
    step(
        Phase::ResumeStart,
        "start playback",
        vec!["M24".to_owned()],
        vec![],
        vec![
            Verification::new("virtual_sdcard", "is_active", Predicate::BoolTrue),
            Verification::new(
                "idle_timeout",
                "state",
                Predicate::Equals {
                    value: "Printing".to_owned(),
                },
            ),
        ],
        None,
        AbortReason::ResumeStartFailed,
    )
}

/// The per-probe-kind trigger source and true-Z formula (design doc
/// §8, step 6).
fn true_z_formula(
    machine: &ValidatedMachine,
    cfg: &PlanConfig,
    candidate: &ProbeCandidate,
    context: &Context,
) -> TrueZFormula {
    // The consensus path (both Tap and load-cell) reads the plugin's
    // consensus median; the legacy single-`PROBE` path keeps the
    // per-probe-type stock readback.
    let trigger_source = if uses_consensus_touch(machine, cfg) {
        // The plugin reports the consensus median in the
        // z_offset-subtracted bed-probing frame (both Tap and load-cell
        // touch through a klippy probe session), so carry the configured
        // offset to add back.
        TriggerSource::TouchResult {
            z_offset: machine.probe.z_offset,
        }
    } else {
        match &machine.probe.kind {
            ProbeKind::Tap => TriggerSource::RawLastZResult,
            ProbeKind::LoadCell => TriggerSource::BedZPlusOffset {
                z_offset: machine.probe.z_offset,
            },
            ProbeKind::AdxlDrag { .. } => TriggerSource::DragResult,
        }
    };
    TrueZFormula {
        z_prev_top: candidate.z,
        trigger_source,
        frozen_z_adjust: context.transforms.z_thermal_adjust_offset,
    }
}

/// Appends the non-fatal planning observations.
/// The noise floor is speed-specific: when the calibration speed is
/// known (optional `[plr]` `noise_floor_speed`) and the plan's
/// `drag_speed` strays more than 20% from it, warn — never refuse —
/// so the operator re-runs `PLR_NOISE_TEST` at the current speed.
/// Non-finite or non-positive recorded speeds are ignored (tolerant:
/// the key is forward-looking metadata, not a gate). Contact-probe
/// machines never check it.
fn noise_floor_speed_warning(
    machine: &ValidatedMachine,
    noise_floor_speed: Option<f64>,
    drag_speed: f64,
) -> Option<PlanWarning> {
    let ProbeKind::AdxlDrag { .. } = &machine.probe.kind else {
        return None;
    };
    let calibrated_at = noise_floor_speed?;
    let usable = calibrated_at.is_finite() && calibrated_at > 0.0;
    if usable && (drag_speed - calibrated_at).abs() > 0.2 * calibrated_at {
        return Some(PlanWarning::NoiseFloorSpeedMismatch {
            calibrated_at,
            drag_speed,
        });
    }
    None
}

fn collect_warnings(
    context: &Context,
    bed_unknown: bool,
    on_infill: bool,
    warnings: &mut Vec<PlanWarning>,
) {
    let transforms = &context.transforms;
    if transforms.bed_mesh_active
        && transforms
            .bed_mesh_profile
            .as_deref()
            .is_none_or(str::is_empty)
    {
        warnings.push(PlanWarning::AdaptiveMeshNotRestorable);
    }
    if transforms.skew_active && transforms.skew_profile.as_deref().is_none_or(str::is_empty) {
        warnings.push(PlanWarning::SkewProfileUnknown);
    }
    if bed_unknown {
        warnings.push(PlanWarning::NoBedTarget);
    }
    if !on_infill {
        warnings.push(PlanWarning::ResumeNotOnInfill);
    }
}

/// Assembles the steps in strict §8 order and renumbers them.
fn build_steps(ctx: &Ctx<'_>) -> Result<Vec<RecoveryStep>, RecoveryError> {
    let transforms = &ctx.context.transforms;
    let mut steps = vec![
        step_idle_timeout(ctx),
        step_stepper_enable(ctx),
        step_preheat(ctx),
        step_home_xy(),
    ];
    if transforms.z_thermal_adjust_enabled.is_some() {
        steps.push(step_transform_freeze());
    }
    steps.push(step_shifted_frame(ctx));
    steps.push(step_probe_approach(ctx));
    // The accel clamp/restore wrap only the consensus PLR_TOUCH phase
    // (Cartographer clamps around z_probing_move). Drag passes are
    // fixed-Z with the plugin owning its own motion profile, and the
    // legacy single-PROBE path predates the plugin, so neither gets the
    // plan-level clamp.
    let clamp = uses_consensus_touch(ctx.machine, ctx.cfg);
    if clamp {
        steps.push(step_accel_clamp(ctx));
    }
    steps.push(step_probe(ctx));
    if clamp {
        steps.push(step_accel_restore());
    }
    steps.push(step_true_z_declare(ctx));
    if transforms.bed_mesh_active {
        if let Some(profile) = transforms
            .bed_mesh_profile
            .as_deref()
            .filter(|p| !p.is_empty())
        {
            validate_command_name("bed_mesh profile", profile)?;
            steps.push(step_mesh_load(profile));
        }
    }
    steps.push(step_final_declare(ctx));
    steps.push(step_restore_frame(ctx));
    steps.push(step_entry(ctx)?);
    steps.push(step_file_select(ctx));
    steps.push(step_resume_start());
    for (index, s) in steps.iter_mut().enumerate() {
        s.id = u32::try_from(index + 1).unwrap_or(u32::MAX);
    }
    Ok(steps)
}

/// Builds the strictly ordered recovery plan (design doc §8), or a
/// typed non-plan outcome.
///
/// # Errors
///
/// [`RecoveryError`] on invalid inputs or failed machine
/// prerequisites. Degraded-but-defined situations (declined contact
/// zone, too-coarse match) return
/// [`PlanOutcome::ManualFallback`] instead of an error; a clean
/// shutdown returns [`PlanOutcome::NoRecoveryNeeded`].
pub fn plan_recovery(
    inputs: &PlanInputs<'_>,
    config: &PlanConfig,
) -> Result<PlanOutcome, RecoveryError> {
    config.validate()?;
    let machine = validate_machine(inputs.machine).map_err(|r| RecoveryError::MachineRejected {
        failures: r.failures,
    })?;
    let recovery = match inputs.reconstruction {
        Reconstruction::CleanShutdown(_) => return Ok(PlanOutcome::NoRecoveryNeeded),
        Reconstruction::Recovery(recovery) => recovery,
    };
    let context = recovery
        .timeline
        .contexts
        .last()
        .ok_or(RecoveryError::NoContext)?;
    if !context.values_are_finite() {
        return Err(RecoveryError::NonFinite { field: "context" });
    }
    let gcode = validate_gcode_state(&context.gcode)?;
    let vsd = context
        .virtual_sdcard
        .as_ref()
        .ok_or(RecoveryError::NoVirtualSd)?;
    let file_name = top_level_file_name(&vsd.file_path, &machine.sdcard_root)?;

    let envelope = size_envelope(&machine, config, recovery)?;

    let candidate = match inputs.contact {
        ContactOutcome::Declined(reason) => {
            return Ok(PlanOutcome::ManualFallback {
                reason: FallbackReason::ContactDeclined(reason.clone()),
            })
        }
        ContactOutcome::Candidates(candidates) => {
            candidates.first().ok_or(RecoveryError::NoProbeCandidates)?
        }
    };
    if !(candidate.point.iter().all(|v| v.is_finite()) && candidate.z.is_finite()) {
        return Err(RecoveryError::NonFinite {
            field: "contact_candidate",
        });
    }

    let resume = match select_resume_target(inputs.model, inputs.match_result) {
        Ok(resume) => resume,
        Err(reason) => return Ok(PlanOutcome::ManualFallback { reason }),
    };

    let preheat = derive_preheat(context, &inputs.file_temps);
    let nozzle_print = preheat.nozzle.ok_or(RecoveryError::NoNozzleTarget)?;
    validate_excludes(inputs.exclude_objects)?;

    let formula = true_z_formula(&machine, config, candidate, context);
    let (fan_cmds, mut warnings) = fan_commands(&preheat.fans);
    collect_warnings(
        context,
        preheat.bed.is_none(),
        resume.on_infill,
        &mut warnings,
    );
    warnings.extend(noise_floor_speed_warning(
        &machine,
        inputs.machine.noise_floor_speed,
        config.drag_speed,
    ));

    let ctx = Ctx {
        cfg: config,
        machine: &machine,
        context,
        gcode,
        envelope,
        candidate,
        formula,
        resume,
        nozzle_print,
        bed: preheat.bed,
        other_heaters: &preheat.other_heaters,
        fan_cmds: &fan_cmds,
        file_name: &file_name,
        file_path: &vsd.file_path,
        excludes: inputs.exclude_objects,
    };
    let steps = build_steps(&ctx)?;
    let plan = RecoveryPlan {
        steps,
        envelope,
        resume_file: file_name,
        resume_offset: resume.offset,
        warnings,
    };
    run_preflight(&plan, &machine, candidate.point)?;
    Ok(PlanOutcome::Plan(Box::new(plan)))
}

/// Sizes the probe envelope for the recovery. Overshoot per probe
/// method (see `envelope` for the derivations): continuous `PROBE`
/// descents overshoot by 0.15 s of drip-move travel past the trigger;
/// the drag staircase's passes are fixed-Z, so its only overshoot is the
/// bounded Z decrement itself.
fn size_envelope(
    machine: &ValidatedMachine,
    config: &PlanConfig,
    recovery: &plr_reconstruct::RecoveryReconstruction,
) -> Result<Envelope, RecoveryError> {
    let z_span = recovery.stop_set.z_span().ok_or(RecoveryError::NoZSpan)?;
    let overshoot = match &machine.probe.kind {
        ProbeKind::Tap | ProbeKind::LoadCell => OvershootTerm::PostTriggerTravel {
            probe_speed: config.probe_speed,
        },
        ProbeKind::AdxlDrag { .. } => OvershootTerm::DragStep {
            drag_z_step: config.drag_z_step,
        },
    };
    compute_envelope(
        EnvelopeParams {
            expected_gap: z_span.width() + config.sag_allowance,
            overshoot,
            margin: config.margin,
        },
        machine.z_position_min,
    )
}

/// Whole-itinerary pre-flight: every commanded coordinate the plan will
/// emit is checked against the machine's known axis limits and the
/// analyzer's already-selected contact point BEFORE the plan is returned
/// (Cartographer validates the whole itinerary up front,
/// `axis_twist_compensation.py:87-111`). Aggregates every violation.
fn run_preflight(
    plan: &RecoveryPlan,
    machine: &ValidatedMachine,
    contact_point: [f64; 2],
) -> Result<(), RecoveryError> {
    let bounds = ItineraryBounds {
        x: machine.axis_limits.x,
        y: machine.axis_limits.y,
        z_max: machine.axis_limits.z_max,
        position_min: machine.z_position_min,
        contact_point,
    };
    preflight_itinerary(plan, &bounds)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{
        top_level_file_name, validate_command_name, validate_excludes, ExcludeObjectDef, PlanConfig,
    };
    use crate::error::RecoveryError;

    fn check(
        field: &'static str,
        set: impl Fn(&mut PlanConfig, f64),
        ok_values: &[f64],
        bad_values: &[f64],
    ) {
        for &v in ok_values {
            let mut config = PlanConfig::default();
            set(&mut config, v);
            assert!(config.validate().is_ok(), "{field} = {v} must pass");
        }
        for &v in bad_values {
            let mut config = PlanConfig::default();
            set(&mut config, v);
            assert!(
                matches!(
                    config.validate(),
                    Err(RecoveryError::InvalidPlanConfig { field: f }) if f == field
                ),
                "{field} = {v} must be rejected"
            );
        }
    }

    #[test]
    fn drag_tunable_bands_are_hard() {
        assert!(PlanConfig::default().validate().is_ok());
        // The bands mirror the [plr] schema in
        // klippy_plugin/plr/tunables.py, including its defaults.
        check(
            "drag_speed",
            |c, v| c.drag_speed = v,
            &[0.5, 20.0, 100.0],
            &[0.0, -1.0, 100.01],
        );
        check(
            "drag_z_step",
            |c, v| c.drag_z_step = v,
            &[0.005, 0.05, 0.2],
            &[0.0, -0.01, 0.201, 1.0],
        );
        check(
            "drag_sensitivity",
            |c, v| c.drag_sensitivity = v,
            &[0.0, 30.0, 100.0],
            &[-0.01, 100.5, -3.0],
        );
    }

    #[test]
    fn touch_tunable_bands_are_hard() {
        assert!(PlanConfig::default().validate().is_ok());
        check(
            "touch_samples",
            |c, v| c.touch_samples = v,
            &[3.0, 5.0, 7.0],
            &[2.0, 8.0, 3.5, 0.0],
        );
        // The hard cap: 0.015 passes, anything above is REFUSED (never
        // clamped), per Cartographer configuration.py:248.
        check(
            "touch_sample_range",
            |c, v| c.touch_sample_range = v,
            &[0.001, 0.010, 0.015],
            &[0.0, -0.001, 0.0151, 0.02, 1.0],
        );
        check(
            "touch_retract",
            |c, v| c.touch_retract = v,
            &[1.0, 2.0, 10.0],
            &[0.999, 0.0, -1.0],
        );
        check(
            "touch_accel",
            |c, v| c.touch_accel = v,
            &[50.0, 100.0, 1000.0],
            &[49.0, 1000.1, 0.0, -5.0],
        );
    }

    #[test]
    fn top_level_names_are_extracted() {
        assert_eq!(
            top_level_file_name("/home/pi/gcodes/part.gcode", "/home/pi/gcodes").unwrap(),
            "part.gcode"
        );
        assert_eq!(
            top_level_file_name("/home/pi/gcodes/part.gcode", "/home/pi/gcodes/").unwrap(),
            "part.gcode"
        );
    }

    #[test]
    fn subdirectories_and_foreign_paths_are_refused() {
        for path in [
            "/home/pi/gcodes/sub/part.gcode",
            "/home/pi/gcodes/",
            "/other/part.gcode",
            "/home/pi/gcodesx/part.gcode", // prefix but not a directory boundary
        ] {
            assert!(
                matches!(
                    top_level_file_name(path, "/home/pi/gcodes"),
                    Err(RecoveryError::FileNotTopLevel { .. })
                ),
                "{path} must be refused"
            );
        }
    }

    #[test]
    fn command_names_reject_injection_shapes() {
        assert!(validate_command_name("t", "part_1").is_ok());
        for bad in ["", "a b", "a=b", "a\"b", "a'b", "a\nb"] {
            assert!(validate_command_name("t", bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn exclude_geometry_must_be_finite() {
        let def = ExcludeObjectDef {
            name: "cube".to_owned(),
            center: Some([f64::NAN, 0.0]),
            polygon: vec![],
            currently_excluded: false,
        };
        assert!(validate_excludes(&[def]).is_err());
        let def = ExcludeObjectDef {
            name: "cube".to_owned(),
            center: None,
            polygon: vec![[0.0, f64::INFINITY]],
            currently_excluded: false,
        };
        assert!(validate_excludes(&[def]).is_err());
    }
}
