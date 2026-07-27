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
use std::time::Duration;

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
    RuntimeComputation, TriggerSource, TrueZFormula, Verification, MACHINE_ACCEL_PLACEHOLDER,
    PARK_Z_PLACEHOLDER, RESTORE_ACCEL_PLACEHOLDER, TRUE_Z_PLACEHOLDER,
};
use crate::preflight::{preflight_itinerary, ItineraryBounds};
use crate::preheat::{derive_preheat, FileTemps};

/// Tunables of the plan builder. [`PlanConfig::default`] is the
/// design-doc configuration; [`PlanConfig::validate`] enforces every
/// documented bound.
///
/// The `struct_excessive_bools` allow is deliberate: this type is a flat
/// mirror of the operator's `[plr]` section, one field per documented
/// config key. The lint's usual remedy — collapsing related booleans into
/// an enum or a sub-struct — would put a layer of translation between
/// what the operator wrote in `printer.cfg` and what the planner reads,
/// which is precisely the seam a config bug hides in.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[allow(clippy::struct_excessive_bools)]
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
    /// FROZEN `[plr]` key `max_probe_nozzle_temp` — the hard ceiling, °C,
    /// for ANY contact operation. Range `[80, 160]`. Clamps the probe
    /// temperature band: the effective band is
    /// `[probe_temp_min, min(probe_temp_max, max_probe_nozzle_temp)]`; a
    /// config whose clamped band is empty is REFUSED. The probe/drag
    /// `pre_verify` predicates (current AND target) use this ceiling.
    pub max_probe_nozzle_temp: f64,
    /// FROZEN `[plr]` key `reheat_park_x` — nozzle park X while reheating
    /// to print temperatures. `None` computes a park point outside the
    /// analyzer's part bounding box (+ margin), clamped inside the axis
    /// limits, with a plan warning recommending explicit configuration.
    pub reheat_park_x: Option<f64>,
    /// FROZEN `[plr]` key `reheat_park_y` — nozzle park Y (see
    /// [`Self::reheat_park_x`]).
    pub reheat_park_y: Option<f64>,
    /// FROZEN `[plr]` key `reheat_park_delta_z` — Z lift above the
    /// current (post-probe true-frame) Z for parking, mm. Range
    /// `(0, 10]`. Klipper's own rail limit clamps the resulting absolute
    /// Z below `position_max`.
    pub reheat_park_delta_z: f64,
    /// FROZEN `[plr]` key `pre_home_z_lift` — added to the believed Z
    /// before XY homing, mm. Range `(0, 20]`. Clamped against `z_max`
    /// when known.
    pub pre_home_z_lift: f64,
    /// FROZEN `[plr]` key `purge_enable` — whether the recovery file
    /// purges before resuming. `false` means no purge of ANY kind.
    pub purge_enable: bool,
    /// FROZEN `[plr]` key `purge_amount` — built-in purge extrusion
    /// length, mm. Range `[0, 100]`.
    pub purge_amount: f64,
    /// FROZEN `[plr]` key `purge_macro` — when set, the macro OWNS the
    /// purge entirely (its own positioning, amount and speed).
    ///
    /// If it is set but the macro does not exist on the machine, planning
    /// is REFUSED ([`RecoveryError::PurgeMacroMissing`]) — never silently
    /// downgraded to the built-in purge. See the precedence table on
    /// [`resolve_purge`].
    pub purge_macro: Option<String>,
    /// FROZEN `[plr]` key `purge_x` — X of the built-in purge, mm.
    /// `None` defaults to the reheat park point's X.
    pub purge_x: Option<f64>,
    /// FROZEN `[plr]` key `purge_y` — Y of the built-in purge, mm.
    /// `None` defaults to the reheat park point's Y.
    pub purge_y: Option<f64>,
    /// FROZEN `[plr]` key `purge_z` — ABSOLUTE Z of the built-in purge,
    /// mm. `None` keeps the elevated park Z already in effect; setting it
    /// lets an operator purge low over a defined spot instead of dropping
    /// filament from mid-air.
    ///
    /// **Must be `>= 0`.** This is the ONE raw operator-chosen absolute Z
    /// in the generated file, and the file runs in the TRUE frame, where
    /// `Z = 0` is the bed surface — so a negative value drives the nozzle
    /// into the bed at print temperature and extrudes into it. It cannot
    /// be validated against the Z RAIL's `position_min`, which this design
    /// deliberately places BELOW the bed (typically −2 mm) so the
    /// shifted-frame probe envelope has room; that check would accept
    /// `purge_z = -1.9`. [`PlanConfig::validate`] refuses negatives
    /// outright, and the builder warns when the value sits below the
    /// resume Z (i.e. below the part's current top).
    pub purge_z: Option<f64>,
    /// FROZEN `[plr]` key `purge_retract` — filament retracted after the
    /// built-in purge, mm, to help break the string. Range `[0, 10]`;
    /// `0` disables it.
    pub purge_retract: f64,
    /// FROZEN `[plr]` key `clean_nozzle_macro` — the macro the
    /// clean-nozzle step calls when it exists (default `CLEAN_NOZZLE`).
    pub clean_nozzle_macro: String,
    /// FROZEN `[plr]` key `drag_nozzle_temp` — the nozzle temperature, °C,
    /// the ADXL drag path heats to AND HOLDS FOR before dragging.
    ///
    /// Closes a real asymmetry: the touch path commands a probe
    /// temperature and effectively holds for it (its band `pre_verify`
    /// polls until satisfied), while the drag path historically carried
    /// only an upper ceiling — so a drag probe could reference at any
    /// temperature from ambient upward, and two runs of the same machine
    /// could take their Z reference at different thermal states. Nozzle
    /// thermal expansion moves the effective reference by tens of microns
    /// across a 100 °C swing, so that inconsistency is a systematic Z
    /// error, not noise.
    ///
    /// Default 145.0 — deliberately equal to the touch path's commanded
    /// probe temperature, so both oracles reference at the same thermal
    /// state.
    ///
    /// **`0` is an explicit opt-out**: "do not heat for dragging, and do
    /// not hold". A cold drag is legitimate (see the drag path's
    /// temperature gate), and an operator who wants one must not be left
    /// waiting for the nozzle to cool to ambient — which it may never
    /// reach. At `0` the plan emits no drag `M104`, no `M109`, and no
    /// [`Phase::ProbeTempHold`] step at all; the contact ceiling still
    /// applies.
    ///
    /// Valid range `[0, clamped_probe_max - PROBE_TEMP_HEADROOM]`: the
    /// same headroom rule as [`Self::probe_nozzle_temp`], so plrd can
    /// never command a temperature the plugin's ceiling gate would then
    /// refuse (the finding-9 interlock).
    pub drag_nozzle_temp: f64,
    /// FROZEN `[plr]` key `purge_speed` — built-in purge extrusion
    /// feedrate, mm/min. Range `(0, 3000]`; deliberately slow by default.
    pub purge_speed: f64,
    /// `[plr]` key `recovery_accel` — one `max_accel` value, mm/s²,
    /// applied for the WHOLE recovery and restored on completion and on
    /// abort alike. `None` (the default) leaves the machine's own
    /// acceleration alone.
    ///
    /// Range [[`ACCEL_MIN`], [`ACCEL_MAX`]]; an out-of-band value is
    /// refused with [`RecoveryError::AccelOutOfRange`], never clamped.
    pub recovery_accel: Option<f64>,
    /// `[plr]` key `accel_home` — `max_accel` for the XY homing step
    /// ([`Phase::HomeXy`]), mm/s². `None` inherits whatever is in force.
    pub accel_home: Option<f64>,
    /// `[plr]` key `accel_travel` — `max_accel` for the long XY travel to
    /// the contact point ([`Phase::ProbeApproach`]), mm/s².
    pub accel_travel: Option<f64>,
    /// `[plr]` key `accel_probe` — `max_accel` for the contact step
    /// ([`Phase::Probe`]) on the ADXL-drag and legacy single-`PROBE`
    /// paths, mm/s².
    ///
    /// **Ignored on the consensus-touch path**, where
    /// [`Self::touch_accel`] owns the contact acceleration through the
    /// existing [`Phase::AccelClamp`] step — two settings fighting over
    /// the same number during the one motion that must not be
    /// over-driven is not a feature. The plan says so out loud with
    /// [`PlanWarning::AccelProbeIgnoredOnTouchPath`] rather than
    /// swallowing the key.
    pub accel_probe: Option<f64>,
    /// `[plr]` key `accel_entry` — `max_accel` for the moves made in
    /// close proximity to printed geometry: the operator Z-confirmation
    /// standoff ([`Phase::ZConfirmStandoff`]) and the lift-and-park
    /// before the reheat ([`Phase::ParkForReheat`]), mm/s².
    pub accel_entry: Option<f64>,
    /// `[plr]` key `confirm_z_before_resume` (default `false`) — after
    /// the true-Z declaration, lift to the entry standoff and PAUSE,
    /// reporting the believed Z and how it was derived, until the
    /// operator confirms over the control socket.
    ///
    /// Adds the [`Phase::ZConfirmStandoff`] step; leaving it `false`
    /// produces a plan byte-identical to one built before the key
    /// existed.
    pub confirm_z_before_resume: bool,
    /// `[plr]` key `debug_confirm_each_step` (default `false`) — pause
    /// before every step, reporting that step's commands and
    /// verifications. Carried onto
    /// [`RecoveryPlan::debug_confirm_each_step`]; changes no command.
    pub debug_confirm_each_step: bool,
    /// `[plr]` key `UNSAFE_allow_purge_z_below_bed` (see
    /// [`crate::diagnosis::UNSAFE_PURGE_Z_BELOW_BED`]) — permits a
    /// `purge_z` below the bed surface, which is otherwise a
    /// [`crate::diagnosis::Tier::Hard`] refusal. Setting it raises
    /// [`PlanWarning::UnsafeOverrideActive`].
    pub unsafe_allow_purge_z_below_bed: bool,
    /// `[plr]` key `confirm_timeout_s` — how long a confirm-point waits
    /// for an operator's answer before aborting cleanly, seconds. `None`
    /// leaves the daemon's [`CONFIRM_TIMEOUT_DEFAULT_S`] default.
    ///
    /// Range [[`CONFIRM_TIMEOUT_MIN_S`], [`CONFIRM_TIMEOUT_MAX_S`]]; an
    /// out-of-band value is refused with
    /// [`RecoveryError::ConfirmTimeoutOutOfRange`], never clamped.
    pub confirm_timeout_s: Option<f64>,
    /// `[plr]` key `gcode_barrier_timeout_s` — how long the recovery will
    /// wait for Klipper's g-code mutex before refusing, seconds. `None`
    /// leaves the daemon's [`GCODE_BARRIER_TIMEOUT_DEFAULT_S`] default.
    ///
    /// The knob exists because the *right* value depends on the operator's
    /// own macros: a printer whose `PLR_RECOVER` sits in a macro that then
    /// does thirty seconds of real work needs a longer wait, and the only
    /// alternatives without this key are editing the macro or rebuilding
    /// the daemon. Raising it trades promptness for tolerance; it never
    /// weakens the check, because the wait ends either way — in exclusive
    /// access or in a refusal.
    ///
    /// Range [[`GCODE_BARRIER_TIMEOUT_MIN_S`],
    /// [`GCODE_BARRIER_TIMEOUT_MAX_S`]]; an out-of-band value is refused
    /// with [`RecoveryError::GcodeBarrierTimeoutOutOfRange`], never
    /// clamped.
    pub gcode_barrier_timeout_s: Option<f64>,
    /// `[plr]` key `resume_candidate_policy` (`first`|`mid`|`last`|`ask`,
    /// default `ask`) — which stop in an ambiguous candidate set becomes
    /// the resume point, and whether the interactive preview runs (design
    /// `docs/design/resume-preview.md` §3). The enum's own explicit-choice
    /// parse (in `plrd`'s `plrcfg.rs`, like `probe_method`) refuses any
    /// spelling outside the four; nothing to range-check here.
    #[serde(default, skip_serializing_if = "ResumePolicy::is_default")]
    pub resume_candidate_policy: ResumePolicy,
    /// `[plr]` key `preview_standoff` — the height, mm, the single XY-hover
    /// plane sits above the highest preview stop (design §E.1). `None`
    /// derives it from [`Self::entry_hop`] (the ruled default), so the
    /// hover plane clears geometry by the same safe standoff the Z-confirm
    /// lift uses. Resolve with [`Self::preview_standoff_mm`].
    ///
    /// Range `[0.0, ∞)` when set (a negative standoff would lower the
    /// hover plane INTO the part); refused with [`RecoveryError::NonFinite`]
    /// / [`RecoveryError::InvalidPlanConfig`], never clamped.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preview_standoff: Option<f64>,
    /// `[plr]` key `preview_nozzle_temp` — the nozzle target, °C, commanded
    /// on preview entry so a nozzle hovering over the part for minutes of
    /// deliberation cannot ooze onto it (design §E.2). `None` derives it
    /// from the plan's probe hold temperature (the ruling: probe temps are
    /// non-oozing by design and the nozzle is already there, so
    /// reheat-from-probe beats reheat-from-cold; on a cold-drag machine the
    /// hold is `None` and this resolves to `0`). `0` is a legal explicit
    /// value (cool fully, extra caution). Resolve with
    /// [`Self::preview_nozzle_temp_c`].
    ///
    /// Range `[0.0, max_probe_nozzle_temp]` when set — a preview target
    /// above the probe ceiling would defeat the non-ooze purpose; refused
    /// with [`RecoveryError::InvalidPlanConfig`], never clamped.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preview_nozzle_temp: Option<f64>,
}

/// Default [`PlanConfig::confirm_timeout_s`].
///
/// Long enough to walk to the printer, look at the nozzle, and walk back
/// — which is exactly what a Z-height confirmation asks for.
///
/// **This is the single definition of that default.** It used to be
/// written out twice in two units in two crates — `600.0` here and
/// `Duration::from_mins(10)` in `plrd`'s executor — which is not a
/// duplication but a latent divergence: the number this crate *documents*
/// to the operator (and quotes back at them in
/// [`crate::diagnosis`]'s `confirm_timeout_out_of_range` fix text) would
/// have silently stopped describing the number the daemon *enforces*, and
/// nothing on the operator's side could reveal the difference. `plrd`'s
/// `executor::DEFAULT_CONFIRM_TIMEOUT` is now this constant, and
/// [`CONFIRM_TIMEOUT_DEFAULT_S`] is derived from it.
pub const CONFIRM_TIMEOUT_DEFAULT: Duration = Duration::from_mins(10);

/// [`CONFIRM_TIMEOUT_DEFAULT`] in seconds — the units of the `[plr]`
/// `confirm_timeout_s` key and of the band below. Derived, never written
/// out a second time.
// A whole number of seconds in the hundreds is exact in f64; the band
// (30..=3600 s) cannot reach a magnitude where u64 -> f64 loses anything.
#[allow(clippy::cast_precision_loss)]
pub const CONFIRM_TIMEOUT_DEFAULT_S: f64 = CONFIRM_TIMEOUT_DEFAULT.as_secs() as f64;

/// Lower bound for [`PlanConfig::confirm_timeout_s`], seconds.
///
/// Below half a minute a pause cannot survive the walk it exists to
/// permit: the operator would be looking at the nozzle when the recovery
/// gave up on them.
pub const CONFIRM_TIMEOUT_MIN_S: f64 = 30.0;

/// Upper bound for [`PlanConfig::confirm_timeout_s`], seconds.
///
/// An hour. Past that the bound stops being a bound: an abandoned
/// recovery would sit paused with the heaters on and the toolhead over
/// the part for as long as the number says.
pub const CONFIRM_TIMEOUT_MAX_S: f64 = 3_600.0;

/// Default [`PlanConfig::gcode_barrier_timeout_s`].
///
/// Generous enough to absorb the ordinary case the barrier exists for — a
/// `[gcode_macro]` that called `PLR_RECOVER` and has a handful of commands
/// left, or a console command already in flight — and short enough that
/// the operator gets a diagnosis instead of a hang.
///
/// Single definition, same reasoning as [`CONFIRM_TIMEOUT_DEFAULT`]:
/// `plrd`'s `executor::DEFAULT_GCODE_BARRIER_TIMEOUT` is this constant.
pub const GCODE_BARRIER_TIMEOUT_DEFAULT: Duration = Duration::from_secs(30);

/// [`GCODE_BARRIER_TIMEOUT_DEFAULT`] in seconds — the units of the `[plr]`
/// key and of the band below. Derived, never written out a second time.
// Whole seconds in the tens; the band (5..=600 s) cannot reach a magnitude
// where u64 -> f64 loses anything.
#[allow(clippy::cast_precision_loss)]
pub const GCODE_BARRIER_TIMEOUT_DEFAULT_S: f64 = GCODE_BARRIER_TIMEOUT_DEFAULT.as_secs() as f64;

/// Lower bound for [`PlanConfig::gcode_barrier_timeout_s`], seconds.
///
/// Five seconds. Below that the barrier stops distinguishing "somebody
/// else holds the mutex" from "this printer is slow": a busy Klipper host
/// can take a second or more to turn a queued script around, and a barrier
/// that expires inside that window would refuse healthy recoveries while
/// telling the operator something false about why.
pub const GCODE_BARRIER_TIMEOUT_MIN_S: f64 = 5.0;

/// Upper bound for [`PlanConfig::gcode_barrier_timeout_s`], seconds.
///
/// Ten minutes. Past that the bound stops being a bound: the operator is
/// staring at a console that has said nothing for longer than they will
/// wait, and the honest answer — that something else owns the printer — was
/// available minutes earlier.
pub const GCODE_BARRIER_TIMEOUT_MAX_S: f64 = 600.0;

/// Lower bound, mm/s², for every acceleration override
/// ([`PlanConfig::recovery_accel`] and the per-phase `accel_*` keys).
///
/// Below this a recovery move takes longer to accelerate than the
/// executor's own verification timeouts allow for; the recovery would
/// abort on a timeout while the toolhead was still doing exactly what it
/// was told. (`touch_accel` has its own, lower floor of 50: that clamp
/// covers a single short contact motion, not the whole recovery.)
pub const ACCEL_MIN: f64 = 50.0;

/// Upper bound, mm/s², for every acceleration override.
///
/// Recovery moves happen next to printed geometry, in a Z frame
/// established by one contact measurement. 20 000 mm/s² is already far
/// beyond what any of these machines run; past it the value is not a
/// tuning choice but a typo, and honouring a typo here means an impact.
pub const ACCEL_MAX: f64 = 20_000.0;

/// Headroom, °C, between the nozzle temperature the plan COMMANDS for
/// probing and the hard contact ceiling it verifies against.
///
/// The Klipper plugin refuses `PLR_TOUCH` / `PLR_DRAG_PROBE` when
/// `max(current, target) > max_probe_nozzle_temp`. If the plan commanded
/// the ceiling exactly, ordinary PID overshoot (150.4 °C under a 150 °C
/// ceiling is unremarkable) would trip that refusal — and because the
/// probe runs AFTER the shifted-frame declare, the abort invalidates the Z
/// frame, a re-execute is refused, and the fresh dry run regenerates an
/// identical plan that fails identically: the recovery wedges permanently
/// with the nozzle parked over the part.
///
/// The plan therefore targets `clamped_probe_max - PROBE_TEMP_HEADROOM`
/// (145 °C under the default 150 °C ceiling) while still VERIFYING against
/// the full band up to the ceiling — the target leaves room, the band does
/// not tighten. [`PlanConfig::validate`] refuses any config where the
/// headroom cannot be honored.
pub const PROBE_TEMP_HEADROOM: f64 = 5.0;

/// Tolerance, °C, added to the contact ceiling when checking the MEASURED
/// nozzle temperature before a contact operation.
///
/// Mirrors the Klipper plugin's `MAX_TOUCH_TEMPERATURE_EPSILON` (itself
/// following Cartographer's `MAX_TOUCH_TEMPERATURE_EPSILON = 2`,
/// `probe/touch_mode.py:34`). The two sides must agree on the identical
/// boundary, not merely be ordered: without this tolerance the probe step
/// gates measured temperature at exactly the ceiling, and a measured
/// overshoot aborts at [`Phase::Probe`] — which sits AFTER
/// [`Phase::ShiftedFrame`], so the abort invalidates the Z frame, refuses
/// re-execution, and regenerates an identical plan. The wedge would then
/// be plrd's own doing rather than the plugin's.
///
/// The asymmetry is deliberate and load-bearing:
///
/// * the **measured** (`extruder.temperature`) bound is
///   `clamped_probe_max + PROBE_TEMP_MEASURED_TOLERANCE` — measurement
///   noise and PID overshoot around a legitimately-commanded temperature
///   are forgiven;
/// * the **target** (`extruder.target`) bound stays exactly
///   `clamped_probe_max` — commanding a hotter nozzle is never forgiven.
///
/// The preheat step's own band is deliberately NOT widened: it runs
/// before any frame declaration, so a refusal there is a clean early
/// abort with nothing to unwind.
pub const PROBE_TEMP_MEASURED_TOLERANCE: f64 = 2.0;

/// Refusal floor, °C, for a NONZERO [`PlanConfig::drag_nozzle_temp`].
///
/// A nonzero drag temperature makes the plan emit a blocking `M109`, and
/// on a PID hotend that waits in BOTH directions — so a low target means
/// waiting for a PASSIVE COOLDOWN. On the enclosed / heated-chamber
/// machines this project targets, 30–60 °C is at or below chamber
/// ambient: the nozzle can take longer than the executor's 15-minute
/// `temp_timeout` to get there, and if ambient EXCEEDS the target it can
/// never converge at all — every retry burns the full timeout before
/// aborting.
///
/// So sub-50 °C targets are refused, and the honest way to ask for a cold
/// drag is `drag_nozzle_temp = 0`: the documented opt-out, which emits no
/// heat command and no wait at all.
pub const DRAG_TEMP_FLOOR: f64 = 50.0;

/// Half-width, °C, of the band the [`Phase::ProbeTempHold`] step verifies
/// the held nozzle temperature landed inside.
///
/// Deliberately a named constant, not another config knob: `M109` already
/// blocks natively until Klipper is satisfied, so this verification is
/// belt-and-braces confirmation that the temperature really landed — not
/// a control parameter an operator should be tuning. ±5 °C comfortably
/// covers steady-state PID ripple on a settled hotend while still
/// catching a heater that never converged (or converged somewhere else).
pub const PROBE_HOLD_BAND: f64 = 5.0;

impl PlanConfig {
    /// The effective probe-temperature ceiling: `probe_temp_max` clamped
    /// down to `max_probe_nozzle_temp`. Both the current-temperature band
    /// and the extruder-target interlock use this as their upper bound.
    #[must_use]
    pub fn clamped_probe_max(&self) -> f64 {
        self.probe_temp_max.min(self.max_probe_nozzle_temp)
    }

    /// The nozzle temperature the plan actually COMMANDS for probing:
    /// the configured `probe_nozzle_temp` pulled into
    /// `[probe_temp_min, clamped_probe_max - PROBE_TEMP_HEADROOM]`.
    ///
    /// This is what the `M104` carries; the verification band's upper
    /// bound stays at [`Self::clamped_probe_max`] (see
    /// [`PROBE_TEMP_HEADROOM`] for why the two differ).
    #[must_use]
    pub fn commanded_probe_temp(&self) -> f64 {
        let ceiling = self.clamped_probe_max() - PROBE_TEMP_HEADROOM;
        self.probe_nozzle_temp.clamp(self.probe_temp_min, ceiling)
    }

    /// The nozzle temperature this machine heats to AND HOLDS FOR before
    /// the probe, or `None` when no hold applies.
    ///
    /// * ADXL drag → [`Self::drag_nozzle_temp`], or `None` when that is
    ///   `0` (the documented cold-drag opt-out: never wait for the nozzle
    ///   to cool to ambient).
    /// * Tap / load-cell → [`Self::commanded_probe_temp`].
    #[must_use]
    pub fn probe_hold_target(&self, kind: &ProbeKind) -> Option<f64> {
        match kind {
            ProbeKind::AdxlDrag { .. } => {
                (self.drag_nozzle_temp > 0.0).then_some(self.drag_nozzle_temp)
            }
            ProbeKind::Tap | ProbeKind::LoadCell => Some(self.commanded_probe_temp()),
        }
    }

    /// The resolved preview hover-plane standoff, mm (design §E.1): the
    /// operator's [`Self::preview_standoff`] when set, else [`Self::entry_hop`]
    /// (the ruled default). Never negative for a validated config.
    #[must_use]
    pub fn preview_standoff_mm(&self) -> f64 {
        self.preview_standoff.unwrap_or(self.entry_hop)
    }

    /// The resolved preview cool-down nozzle target, °C (design §E.2,
    /// ruled): the operator's [`Self::preview_nozzle_temp`] when set, else
    /// the probe hold temperature the nozzle is already at when preview
    /// begins (post-probe), or `0` on a cold-drag machine that holds no
    /// probe temperature. `0` therefore means "cool fully" whether it was
    /// the explicit choice or the cold-drag fall-through — the safe
    /// direction either way.
    #[must_use]
    pub fn preview_nozzle_temp_c(&self, kind: &ProbeKind) -> f64 {
        self.preview_nozzle_temp
            .unwrap_or_else(|| self.probe_hold_target(kind).unwrap_or(0.0))
    }
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
            // FROZEN [plr] recovery-UX keys (see the field docs).
            max_probe_nozzle_temp: 150.0,
            reheat_park_x: None,
            reheat_park_y: None,
            reheat_park_delta_z: 2.0,
            pre_home_z_lift: 5.0,
            purge_enable: true,
            purge_amount: 5.0,
            purge_macro: None,
            purge_x: None,
            purge_y: None,
            purge_z: None,
            purge_retract: 0.0,
            clean_nozzle_macro: "CLEAN_NOZZLE".to_owned(),
            // Matches commanded_probe_temp() under the default ceiling, so
            // drag and touch reference at the same thermal state.
            drag_nozzle_temp: 145.0,
            purge_speed: 300.0,
            // Acceleration overrides and confirm-points are OFF by
            // default: an unconfigured machine gets exactly the plan it
            // got before these keys existed.
            recovery_accel: None,
            accel_home: None,
            accel_travel: None,
            accel_probe: None,
            accel_entry: None,
            confirm_z_before_resume: false,
            debug_confirm_each_step: false,
            unsafe_allow_purge_z_below_bed: false,
            confirm_timeout_s: None,
            gcode_barrier_timeout_s: None,
            // Preview keys OFF/derived by default: an unconfigured machine
            // gets `ask` (the ruled default — which resolves as `last`
            // skip-forward until increment 3's dialog lands), a hover
            // standoff equal to `entry_hop`, and a preview cool target
            // equal to whatever the probe holds at.
            resume_candidate_policy: ResumePolicy::Ask,
            preview_standoff: None,
            preview_nozzle_temp: None,
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
    #[allow(clippy::too_many_lines)] // one flat table of field-bound checks
    pub fn validate(&self) -> Result<(), RecoveryError> {
        let checks: [(&'static str, f64, bool); 27] = [
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
            // FROZEN [plr] recovery-UX bands.
            (
                "max_probe_nozzle_temp",
                self.max_probe_nozzle_temp,
                self.max_probe_nozzle_temp >= 80.0 && self.max_probe_nozzle_temp <= 160.0,
            ),
            (
                "reheat_park_delta_z",
                self.reheat_park_delta_z,
                self.reheat_park_delta_z > 0.0 && self.reheat_park_delta_z <= 10.0,
            ),
            (
                "pre_home_z_lift",
                self.pre_home_z_lift,
                self.pre_home_z_lift > 0.0 && self.pre_home_z_lift <= 20.0,
            ),
            (
                "purge_amount",
                self.purge_amount,
                self.purge_amount >= 0.0 && self.purge_amount <= 100.0,
            ),
            (
                "purge_speed",
                self.purge_speed,
                self.purge_speed > 0.0 && self.purge_speed <= 3000.0,
            ),
            (
                "purge_retract",
                self.purge_retract,
                self.purge_retract >= 0.0 && self.purge_retract <= 10.0,
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
        // The probe temperature band clamps to the ceiling; refuse a
        // config whose clamped band is empty.
        let clamped_max = self.clamped_probe_max();
        if clamped_max <= self.probe_temp_min {
            return Err(RecoveryError::InvalidPlanConfig {
                field: "max_probe_nozzle_temp",
            });
        }
        // The commanded probe temperature must be able to sit at least
        // PROBE_TEMP_HEADROOM below the ceiling (see the constant's docs:
        // targeting the ceiling wedges the recovery on PID overshoot).
        // When the band is too narrow to hold that headroom, refuse with
        // the offending bounds named rather than silently probing at the
        // ceiling.
        if self.probe_temp_min > clamped_max - PROBE_TEMP_HEADROOM {
            return Err(RecoveryError::ProbeTempHeadroomUnavailable {
                probe_temp_min: self.probe_temp_min,
                ceiling: clamped_max,
                headroom: PROBE_TEMP_HEADROOM,
            });
        }
        // Finiteness is machine-independent: a NaN is wrong wherever it
        // is read. The BAND check is not — it moved to
        // `validate_for_probe`, because it only bites on the drag path.
        if !self.drag_nozzle_temp.is_finite() {
            return Err(RecoveryError::NonFinite {
                field: "drag_nozzle_temp",
            });
        }
        // Acceleration overrides: finite and inside the shared band, or
        // absent. Refused with their own typed error so the diagnosis can
        // carry measured/expected as numbers (see `AccelOutOfRange`).
        for (key, value) in [
            ("recovery_accel", self.recovery_accel),
            ("accel_home", self.accel_home),
            ("accel_travel", self.accel_travel),
            ("accel_probe", self.accel_probe),
            ("accel_entry", self.accel_entry),
        ] {
            if let Some(v) = value {
                if !v.is_finite() || !(ACCEL_MIN..=ACCEL_MAX).contains(&v) {
                    return Err(RecoveryError::AccelOutOfRange {
                        key,
                        value: v,
                        min: ACCEL_MIN,
                        max: ACCEL_MAX,
                    });
                }
            }
        }
        // A nonzero drag target below DRAG_TEMP_FLOOR is NOT refused here.
        // Its worst case is an M109 waiting for a cooldown that never
        // converges, bounded by the executor's step timeout and aborting
        // BEFORE the shifted-frame declare — wasted time and a clean
        // abort, not damage or an unknowable frame. That is the
        // Confirmable tier's job, so the builder raises
        // `PlanWarning::DragTempBelowFloor` (on drag machines, where the
        // key is actually read) and the operator gets an explanation and a
        // button instead of a config edit.
        //
        // Confirm-point timeout: bounded on both sides, refused rather
        // than clamped, for the reasons on the two constants.
        if let Some(seconds) = self.confirm_timeout_s {
            if !seconds.is_finite()
                || !(CONFIRM_TIMEOUT_MIN_S..=CONFIRM_TIMEOUT_MAX_S).contains(&seconds)
            {
                return Err(RecoveryError::ConfirmTimeoutOutOfRange {
                    value: seconds,
                    min: CONFIRM_TIMEOUT_MIN_S,
                    max: CONFIRM_TIMEOUT_MAX_S,
                });
            }
        }
        // G-code mutex barrier budget: same treatment, same reasoning.
        if let Some(seconds) = self.gcode_barrier_timeout_s {
            if !seconds.is_finite()
                || !(GCODE_BARRIER_TIMEOUT_MIN_S..=GCODE_BARRIER_TIMEOUT_MAX_S).contains(&seconds)
            {
                return Err(RecoveryError::GcodeBarrierTimeoutOutOfRange {
                    value: seconds,
                    min: GCODE_BARRIER_TIMEOUT_MIN_S,
                    max: GCODE_BARRIER_TIMEOUT_MAX_S,
                });
            }
        }
        // purge_z is the only raw operator-chosen ABSOLUTE Z in the
        // generated file, and that file runs in the TRUE frame where zero
        // is the bed surface. The Z rail's position_min is NOT a usable
        // floor here — this design puts it below the bed on purpose (see
        // the field docs) — so negatives are refused outright.
        //
        // The UNSAFE_ escape hatch applies here too: the consequence is
        // confined to the purge blob and the operator can see the geometry
        // involved, so a deliberate printer.cfg edit may permit it.
        if let Some(z) = self.purge_z {
            if z.is_finite() && z < 0.0 && !self.unsafe_allow_purge_z_below_bed {
                return Err(RecoveryError::PurgeZBelowBed { purge_z: z });
            }
        }
        // Optional reheat park coordinates: finite when present (axis
        // bounds are checked by the whole-itinerary pre-flight).
        for (field, v) in [
            ("reheat_park_x", self.reheat_park_x),
            ("reheat_park_y", self.reheat_park_y),
            ("purge_x", self.purge_x),
            ("purge_y", self.purge_y),
            ("purge_z", self.purge_z),
        ] {
            if let Some(v) = v {
                if !v.is_finite() {
                    return Err(RecoveryError::NonFinite { field });
                }
            }
        }
        // Preview hover-plane standoff: non-negative and finite when set.
        // A negative standoff would lower the single hover plane BELOW the
        // highest stop — INTO the part — which is the exact never-descend
        // guarantee §E.1 exists to make structural, so it is refused (a
        // violating input: `preview_standoff = -1.0`), never clamped.
        if let Some(s) = self.preview_standoff {
            if !s.is_finite() {
                return Err(RecoveryError::NonFinite {
                    field: "preview_standoff",
                });
            }
            if s < 0.0 {
                return Err(RecoveryError::InvalidPlanConfig {
                    field: "preview_standoff",
                });
            }
        }
        // Preview cool-down nozzle target: finite, in
        // `[0, max_probe_nozzle_temp]` when set. A target ABOVE the probe
        // ceiling would be a hot, oozing nozzle over the part for minutes —
        // defeating the whole purpose of the cool-down (a violating input:
        // `preview_nozzle_temp = 250` on a machine with the default 150 °C
        // ceiling). Refused, never clamped; `0` (cool fully) stays legal.
        if let Some(t) = self.preview_nozzle_temp {
            if !t.is_finite() {
                return Err(RecoveryError::NonFinite {
                    field: "preview_nozzle_temp",
                });
            }
            if t < 0.0 || t > self.max_probe_nozzle_temp {
                return Err(RecoveryError::InvalidPlanConfig {
                    field: "preview_nozzle_temp",
                });
            }
        }
        // probe_speed: non-finite values fall out of the band check in
        // compute_envelope; nothing to do here.
        Ok(())
    }

    /// The checks that only apply to a particular probe path.
    ///
    /// [`Self::validate`] is machine-independent by design, so it cannot
    /// know whether `drag_nozzle_temp` will ever be commanded. On a Tap
    /// or load-cell machine it never is — the plan emits no drag `M104`
    /// and no drag `M109` — so refusing a recovery over its value would
    /// be refusing over a setting this machine does not read. That is
    /// exactly the pointless obstruction the diagnosis framework exists
    /// to remove, and it is the same gating
    /// [`PlanWarning::DragTempBelowFloor`] already uses.
    ///
    /// [`plan_recovery`] calls this immediately after
    /// [`Self::validate`], once the machine snapshot is known.
    ///
    /// # Errors
    ///
    /// [`RecoveryError::DragTempOutOfRange`] when a drag machine's hold
    /// temperature does not leave [`PROBE_TEMP_HEADROOM`] below the
    /// contact ceiling — an interlock that must stay Hard, because the
    /// plugin's own ceiling gate would refuse the drag AFTER the Z frame
    /// is declared.
    pub fn validate_for_probe(&self, kind: &ProbeKind) -> Result<(), RecoveryError> {
        match kind {
            ProbeKind::Tap | ProbeKind::LoadCell => Ok(()),
            ProbeKind::AdxlDrag { .. } => {
                let clamped_max = self.clamped_probe_max();
                if self.drag_nozzle_temp < 0.0
                    || self.drag_nozzle_temp > clamped_max - PROBE_TEMP_HEADROOM
                {
                    return Err(RecoveryError::DragTempOutOfRange {
                        drag_nozzle_temp: self.drag_nozzle_temp,
                        ceiling: clamped_max,
                        headroom: PROBE_TEMP_HEADROOM,
                    });
                }
                Ok(())
            }
        }
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

/// Which stop in an ambiguous candidate set becomes the resume point
/// (design `docs/design/resume-preview.md` §3).
///
/// The policy only bites when a *set* exists ([`MatchConfidence::
/// AmbiguousWindow`]): a [`MatchConfidence::UniqueLine`] has one line and
/// ignores the policy, and [`MatchConfidence::LayerOnly`] is always too
/// coarse to resume automatically.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResumePolicy {
    /// Minimum-offset candidate — re-prints geometry that may already
    /// exist (the nozzle plows the existing wall). Kept, warned plainly.
    First,
    /// Lower-median-offset candidate (by execution-order file offset; the
    /// same convention as [`plr_analyzer::median_index`], shared so `mid`
    /// picks the same stop over an offset list or a preview set). May
    /// re-print; warned like `First`.
    Mid,
    /// Maximum-offset candidate — skip-forward, the safe default: never
    /// double-prints, bounded sub-line void. Byte-identical to the
    /// historical selector.
    Last,
    /// Operator picks via the interactive preview. The preview plan does
    /// not exist yet (increment 2 wires the routing), so `Ask` currently
    /// resolves exactly as [`ResumePolicy::Last`] — a headless setup gets
    /// today's safe skip-forward, never a regression.
    Ask,
}

impl Default for ResumePolicy {
    /// The ruled default is `Ask` (interactive preview).
    fn default() -> Self {
        Self::Ask
    }
}

impl ResumePolicy {
    /// `true` when this is the default (`Ask`). Used by
    /// [`PlanConfig`]'s `skip_serializing_if` so a config that does not set
    /// `resume_candidate_policy` serializes byte-identically to one built
    /// before the key existed.
    #[must_use]
    pub fn is_default(&self) -> bool {
        matches!(self, ResumePolicy::Ask)
    }
}

/// Resolve a base offset to a concrete [`ResumeTarget`]: the first
/// depositing move at or after `base`, with its infill classification.
/// The shared tail behind both [`select_resume_target`] and
/// [`select_resume_target_with_policy`] — one predicate, no drift.
fn resolve_resume_from_offset(
    model: &LayerModel,
    base: u64,
) -> Result<ResumeTarget, FallbackReason> {
    let mv = model
        .first_deposition_at_or_after(base)
        .ok_or(FallbackReason::NoResumeDeposition)?;
    // The trusted-position gate is the shared `SimMove::start_position_known`
    // predicate (X/Y/Z known + all-finite start), the single predicate the
    // preview builder also applies to its nudge domain and resume baking, so
    // a line this resolver refuses preview can neither hover nor bake.
    if !mv.start_position_known() {
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

/// Selects the resume target from the match result (design doc §8,
/// step 12): the **latest** plausible stop offset (skip-forward is the
/// conservative direction — resuming earlier would double-extrude over
/// printed geometry), then the first depositing move at or after it.
///
/// This is the historical, policy-free selector — equivalent to
/// [`select_resume_target_with_policy`] with [`ResumePolicy::Last`], and
/// kept as the byte-identical default-safe path (regression-pinned).
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
    resolve_resume_from_offset(model, base)
}

/// The offset a policy selects from an ambiguous candidate offset list.
/// `None` only when the list is empty. `Last`/`Ask` return the maximum —
/// byte-identical to [`select_resume_target`]'s `offsets.iter().max()`.
fn select_offset(offsets: &[u64], policy: ResumePolicy) -> Option<u64> {
    if offsets.is_empty() {
        return None;
    }
    let mut sorted = offsets.to_vec();
    sorted.sort_unstable();
    let chosen = match policy {
        ResumePolicy::First => sorted[0],
        ResumePolicy::Mid => sorted[plr_analyzer::median_index(sorted.len())],
        ResumePolicy::Last | ResumePolicy::Ask => sorted[sorted.len() - 1],
    };
    Some(chosen)
}

/// Policy-aware resume selection (design §3): for an ambiguous set,
/// `First`/`Mid`/`Last` pick the min / lower-median / max offset; `Ask`
/// resolves as `Last` until the preview plan exists (increment 2). A
/// `UniqueLine` ignores the policy (one line); `LayerOnly` is always too
/// coarse.
///
/// # Byte-identity guarantee
///
/// For every [`MatchConfidence::AmbiguousWindow`],
/// `select_resume_target_with_policy(m, r, ResumePolicy::Last)` equals
/// `select_resume_target(m, r)` exactly — the default-safe automatic path
/// is unchanged. (Regression-pinned and mutation-proven in the tests.)
///
/// # Errors
///
/// A typed [`FallbackReason`], as [`select_resume_target`].
pub fn select_resume_target_with_policy(
    model: &LayerModel,
    result: &MatchResult,
    policy: ResumePolicy,
) -> Result<ResumeTarget, FallbackReason> {
    let base = match &result.confidence {
        MatchConfidence::UniqueLine { offset } => *offset,
        MatchConfidence::AmbiguousWindow { offsets } => {
            select_offset(offsets, policy).ok_or(FallbackReason::NoResumeDeposition)?
        }
        MatchConfidence::LayerOnly { layer } => {
            return Err(FallbackReason::MatchTooCoarse { layer: *layer })
        }
    };
    resolve_resume_from_offset(model, base)
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
    /// `true` when a `[gcode_macro <clean_nozzle_macro>]` section exists
    /// in the running printer config (the daemon resolves this from the
    /// `configfile` sections it already queries). When `false` the
    /// clean-nozzle step carries no command and the plan sets
    /// [`RecoveryPlan::requires_clean_nozzle_confirmation`].
    pub clean_nozzle_macro_present: bool,
    /// `true` when the configured `purge_macro` exists as a
    /// `[gcode_macro <purge_macro>]` section (only consulted when
    /// `purge_macro` is set).
    ///
    /// When `purge_macro` is set and this is `false`, planning is
    /// **REFUSED** ([`RecoveryError::PurgeMacroMissing`]) — it does NOT
    /// fall back to the built-in purge. See the precedence table on
    /// [`resolve_purge`] for the full mapping and why this asymmetry with
    /// `clean_nozzle_macro` (which degrades to asking the operator) is
    /// deliberate.
    pub purge_macro_present: bool,
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
    bed: Option<f64>,
    other_heaters: &'a [(String, f64)],
    fan_cmds: &'a [String],
    excludes: &'a [ExcludeObjectDef],
    /// Upper bound of the possible-stop-set Z (the conservative believed
    /// value declared before XY homing).
    believed_z: f64,
    /// The reheat park point `[x, y]` (configured or computed).
    park: [f64; 2],
    /// Whether the clean-nozzle macro exists on the machine.
    clean_nozzle_present: bool,
    /// The generated recovery file's top-level name (the `M23` target).
    recovery_file_name: &'a str,
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

fn step_immediate_bed_heat(ctx: &Ctx<'_>) -> RecoveryStep {
    // The FIRST heating action, non-blocking: bed heating is the long
    // pole, so its M140 goes out before any motion; the nozzle is nudged
    // toward the (clamped) probe temperature at the same time. Neither
    // command WAITS here — convergence is gated later at the probe's
    // pre_verify. The verifications only confirm the TARGETS were set.
    let mut commands = Vec::new();
    let mut verify = Vec::new();
    if let Some(bed) = ctx.bed {
        commands.push(format!("M140 S{}", fmt_num(bed)));
        verify.push(Verification::new(
            "heater_bed",
            "target",
            Predicate::NumWithin {
                expected: bed,
                epsilon: 1.0,
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
    // The COMMANDED probe temperature sits at least PROBE_TEMP_HEADROOM
    // below the contact ceiling the probe step verifies against: the
    // plugin refuses contact when max(current, target) exceeds the
    // ceiling, and a target ON the ceiling trips that refusal on ordinary
    // PID overshoot — wedging the recovery (see PROBE_TEMP_HEADROOM).
    //
    // Method-aware: a drag machine heads for `drag_nozzle_temp` (the
    // temperature it will later HOLD for), a touch/load-cell machine for
    // the commanded probe temp. A drag machine that opted out
    // (`drag_nozzle_temp = 0`) gets NO nozzle command at all here — the
    // cold-drag path must not be nudged warm behind the operator's back.
    //
    // These verifications stay as they are — confirming the TARGETS were
    // accepted, not that they were reached. Convergence is now the
    // explicit job of the ProbeTempHold step's blocking M109; making this
    // step also wait would double the heat-up serialization for no gain,
    // and it runs before any frame declaration where a stall is merely a
    // clean early abort.
    if let Some(target) = ctx.cfg.probe_hold_target(&ctx.machine.probe.kind) {
        commands.push(format!("M104 S{}", fmt_num(target)));
        verify.push(Verification::new(
            "extruder",
            "target",
            Predicate::NumWithin {
                expected: target,
                epsilon: 1.0,
            },
        ));
    }
    step(
        Phase::ImmediateBedHeat,
        "FIRST heating action (non-blocking): bed toward target (the long pole) + nozzle toward the clamped probe temp",
        commands,
        vec![],
        verify,
        None,
        AbortReason::ImmediateBedHeatFailed,
    )
}

fn step_believed_z_declare(ctx: &Ctx<'_>) -> RecoveryStep {
    // Declare the conservative believed Z (the upper bound of the
    // possible-stop set) then lift by pre_home_z_lift so the following XY
    // homing moves cannot drag the nozzle across the part. The lift is
    // clamped so the commanded absolute Z never exceeds z_max when known.
    let believed = ctx.believed_z;
    let lift = clamp_lift(
        ctx.cfg.pre_home_z_lift,
        believed,
        ctx.machine.axis_limits.z_max,
    );
    let target = believed + lift;
    step(
        Phase::BelievedZDeclare,
        "declare the conservative believed Z (possible-stop upper bound) then lift before XY homing",
        vec![
            format!("SET_KINEMATIC_POSITION Z={}", fmt_num(believed)),
            "G91".to_owned(),
            format!("G1 Z{} F{}", fmt_num(lift), fmt_num(ctx.cfg.travel_feed)),
            "G90".to_owned(),
        ],
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
                    expected: target,
                    epsilon: ctx.cfg.z_epsilon,
                },
            ),
        ],
        None,
        AbortReason::BelievedZDeclareFailed,
    )
}

/// The probe-temperature HOLD step, or `None` on a drag machine that has
/// opted out (`drag_nozzle_temp = 0`).
///
/// `M109` blocks natively until Klipper is satisfied the heater settled,
/// so this step is where the recovery actually WAITS for temperature —
/// closing the asymmetry where the drag path had only an upper ceiling
/// and could therefore reference at any temperature from ambient upward.
/// The band verification below is belt-and-braces: `M109` already
/// returned, so a failure here means the reading disagrees with what
/// Klipper concluded.
fn step_probe_temp_hold(ctx: &Ctx<'_>) -> Option<RecoveryStep> {
    let target = ctx.cfg.probe_hold_target(&ctx.machine.probe.kind)?;
    let summary = match &ctx.machine.probe.kind {
        ProbeKind::AdxlDrag { .. } => {
            "heat to the drag temperature and HOLD (M109 blocks; a PID hotend also waits to COOL)"
        }
        ProbeKind::Tap | ProbeKind::LoadCell => {
            "heat to the probe temperature and HOLD (M109 blocks; a PID hotend also waits to COOL)"
        }
    };
    Some(step(
        Phase::ProbeTempHold,
        summary,
        vec![format!("M109 S{}", fmt_num(target))],
        vec![],
        vec![Verification::new(
            "extruder",
            "temperature",
            Predicate::TempWithin {
                min: target - PROBE_HOLD_BAND,
                max: target + PROBE_HOLD_BAND,
            },
        )],
        None,
        AbortReason::ProbeTempHoldFailed,
    ))
}

fn step_clean_nozzle(ctx: &Ctx<'_>) -> RecoveryStep {
    // When the operator's clean-nozzle macro exists, call it (verify:
    // none — macro semantics are unknowable). When it does not, emit NO
    // command; the plan-level requires_clean_nozzle_confirmation flag
    // tells the wizard/CLI to obtain the operator's confirmation instead.
    let commands = if ctx.clean_nozzle_present {
        vec![ctx.cfg.clean_nozzle_macro.clone()]
    } else {
        vec![]
    };
    let summary = if ctx.clean_nozzle_present {
        "call the clean-nozzle macro (semantics unknowable; not verified)"
    } else {
        "no clean-nozzle macro: the operator must confirm the nozzle is clean (no command)"
    };
    step(
        Phase::CleanNozzle,
        summary,
        commands,
        vec![],
        vec![],
        None,
        AbortReason::CleanNozzleFailed,
    )
}

/// Clamps a Z lift so `base + lift` never exceeds `z_max` (when known),
/// never returning a negative lift.
fn clamp_lift(lift: f64, base: f64, z_max: Option<f64>) -> f64 {
    match z_max {
        Some(zm) if base + lift > zm => (zm - base).max(0.0),
        _ => lift,
    }
}

fn step_home_xy(ctx: &Ctx<'_>) -> RecoveryStep {
    let mut commands = accel_prefix(ctx.cfg.accel_home);
    commands.push("G28 X Y".to_owned());
    step(
        Phase::HomeXy,
        "home XY only (never bare G28, never Z: the bed rises into a fixed gantry)",
        commands,
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
    let mut commands = accel_prefix(ctx.cfg.accel_travel);
    commands.push("G90".to_owned());
    commands.push(format!(
        "G0 X{} Y{} F{}",
        fmt_num(px),
        fmt_num(py),
        fmt_num(ctx.cfg.travel_feed)
    ));
    step(
        Phase::ProbeApproach,
        "XY travel to the selected contact point (no Z motion)",
        commands,
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
    // `accel_probe` applies to the drag and legacy single-PROBE paths
    // only; on the consensus path `touch_accel` owns the contact accel
    // through the AccelClamp step (the plan warns rather than fights).
    let mut commands = if consensus {
        Vec::new()
    } else {
        accel_prefix(ctx.cfg.accel_probe)
    };
    commands.push(command);
    step(
        Phase::Probe,
        summary,
        commands,
        probe_pre_verify(ctx),
        verify,
        None,
        AbortReason::ProbeNoTrigger,
    )
}

/// The mandatory, daemon-enforced probe pre-verifications: the nozzle
/// temperature interlock (both CURRENT temperature and the extruder
/// TARGET at or below the clamped ceiling — the `max(current, target)`
/// guard from Cartographer `touch_mode.py:299-303`, so a nozzle commanded
/// to print temperature while transiently cool still refuses) and XYZ
/// homed.
///
/// Touch / `PROBE` machines keep the warm band `[probe_temp_min,
/// ceiling]` (a below-ooze warmth protects the tip on contact). The drag
/// path has NO warm minimum — a bare ceiling: cold dragging is fine,
/// while a hot nozzle melts the part and corrupts the accelerometer
/// readings — so its current-temperature predicate is `NumAtMost` the
/// ceiling, not a band.
fn probe_pre_verify(ctx: &Ctx<'_>) -> Vec<Verification> {
    let ceiling = ctx.cfg.clamped_probe_max();
    // The MEASURED bound carries the plugin's tolerance so both sides
    // refuse at the identical boundary; the TARGET bound does not (see
    // PROBE_TEMP_MEASURED_TOLERANCE for why the asymmetry is the point).
    let measured_max = ceiling + PROBE_TEMP_MEASURED_TOLERANCE;
    let homed = |axis: &str| {
        Verification::new(
            "toolhead",
            "homed_axes",
            Predicate::Contains {
                needle: axis.to_owned(),
            },
        )
    };
    let current_temp = match &ctx.machine.probe.kind {
        ProbeKind::AdxlDrag { .. } => Verification::new(
            "extruder",
            "temperature",
            Predicate::NumAtMost { max: measured_max },
        ),
        ProbeKind::Tap | ProbeKind::LoadCell => Verification::new(
            "extruder",
            "temperature",
            Predicate::TempWithin {
                min: ctx.cfg.probe_temp_min,
                max: measured_max,
            },
        ),
    };
    vec![
        current_temp,
        Verification::new("extruder", "target", Predicate::NumAtMost { max: ceiling }),
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

/// `true` when any acceleration override is configured, i.e. when the
/// [`Phase::RecoveryAccel`] / [`Phase::RecoveryAccelRestore`] pair must
/// exist so whatever the plan sets is put back afterwards.
fn uses_accel_overrides(cfg: &PlanConfig) -> bool {
    cfg.recovery_accel.is_some()
        || cfg.accel_home.is_some()
        || cfg.accel_travel.is_some()
        || cfg.accel_probe.is_some()
        || cfg.accel_entry.is_some()
}

/// The `SET_VELOCITY_LIMIT` a per-phase override prepends to its step, if
/// any. Set-and-leave: the authoritative put-back is the single
/// [`Phase::RecoveryAccelRestore`] step (success) or the
/// [`Phase::RecoveryAccel`] step's cleanup (abort), so no phase has to
/// know what the next one wants.
fn accel_prefix(value: Option<f64>) -> Vec<String> {
    value
        .map(|v| format!("SET_VELOCITY_LIMIT ACCEL={}", fmt_num(v)))
        .into_iter()
        .collect()
}

/// Records the machine's own `max_accel` and, when `recovery_accel` is
/// set, clamps acceleration for the whole recovery. Declares the abort
/// cleanup that puts the machine value back on ANY later failure — the
/// same `cleanup_commands` mechanism the touch clamp already uses.
fn step_recovery_accel(ctx: &Ctx<'_>) -> RecoveryStep {
    let commands = accel_prefix(ctx.cfg.recovery_accel);
    let verify = ctx
        .cfg
        .recovery_accel
        .map(|v| {
            vec![Verification::new(
                "toolhead",
                "max_accel",
                Predicate::NumWithin {
                    expected: v,
                    epsilon: 1.0,
                },
            )]
        })
        .unwrap_or_default();
    let summary = if ctx.cfg.recovery_accel.is_some() {
        "record the machine's max_accel and clamp it for the whole recovery (restored after / on abort)"
    } else {
        "record the machine's max_accel so the per-phase overrides can be put back (restored after / on abort)"
    };
    let mut s = step(
        Phase::RecoveryAccel,
        summary,
        commands,
        vec![],
        verify,
        Some(RuntimeComputation::RecordMachineAccel),
        AbortReason::RecoveryAccelFailed,
    );
    s.cleanup_commands = vec![format!(
        "SET_VELOCITY_LIMIT ACCEL={MACHINE_ACCEL_PLACEHOLDER}"
    )];
    s
}

/// Puts the machine's own `max_accel` back on the success path, before
/// the recovery file is selected — the resumed print must run at the
/// machine's acceleration, not at a recovery override.
fn step_recovery_accel_restore() -> RecoveryStep {
    step(
        Phase::RecoveryAccelRestore,
        "restore the machine's own max_accel before the recovery file starts",
        vec![format!(
            "SET_VELOCITY_LIMIT ACCEL={MACHINE_ACCEL_PLACEHOLDER}"
        )],
        vec![],
        // Deliberately unverified: the value being restored is the
        // machine slot, which no Predicate can name (NumWithinComputed
        // reads the PHASE slot). A SET_VELOCITY_LIMIT that fails is a
        // command error and aborts anyway — and the abort then runs the
        // RecoveryAccel cleanup, which restores the same value. The
        // guarantee is the cleanup, not a readback.
        vec![],
        None,
        AbortReason::RecoveryAccelRestoreFailed,
    )
}

/// The operator Z-confirmation standoff (`confirm_z_before_resume`).
///
/// Reuses the entry-hop distance — the documented safe standoff above the
/// resume point — through the SAME rail-clamped `ParkZ` arithmetic the
/// reheat park uses, so the move is `min(current_Z + entry_hop, z_max)`
/// and, because [`crate::park_z_at`] never clamps below the current Z,
/// **cannot descend**. Driving toward the bed to "show" the operator
/// where Z is would be a new unreviewed descent; this lifts instead and
/// lets the daemon do the explaining.
fn step_z_confirm_standoff(ctx: &Ctx<'_>) -> RecoveryStep {
    let mut commands = accel_prefix(ctx.cfg.accel_entry);
    commands.push("G90".to_owned());
    commands.push(format!(
        "G1 Z{PARK_Z_PLACEHOLDER} F{}",
        fmt_num(ctx.cfg.entry_feed)
    ));
    step(
        Phase::ZConfirmStandoff,
        "lift to the entry standoff for the operator Z confirmation (never descends)",
        commands,
        vec![],
        vec![Verification::new(
            "toolhead",
            "position.2",
            Predicate::NumWithinComputed {
                epsilon: ctx.cfg.z_epsilon,
            },
        )],
        Some(RuntimeComputation::ParkZ {
            delta_z: ctx.cfg.entry_hop,
            z_max: ctx.machine.axis_limits.z_max,
        }),
        AbortReason::ZConfirmStandoffFailed,
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

fn step_park_for_reheat(ctx: &Ctx<'_>) -> RecoveryStep {
    // Park the nozzle away from the part before the recovery file reheats
    // to print temperature: a nozzle dwelling at print temperature
    // pressed against layer N−1 plastic melts a divot.
    //
    // The lift is an ABSOLUTE move to a runtime-computed, rail-clamped
    // height (`RuntimeComputation::ParkZ` → min(current + delta, z_max)),
    // not a blind relative `G1 Z<delta>`: Klipper does not clamp an
    // out-of-range move, it raises "Move out of range"
    // (kinematics/cartesian.py:105) — which here would abort AFTER the
    // probe established the Z reference and force a full re-run. Then an
    // absolute travel to the reheat park XY.
    let [px, py] = ctx.park;
    let mut commands = accel_prefix(ctx.cfg.accel_entry);
    commands.extend([
        "G90".to_owned(),
        format!("G1 Z{PARK_Z_PLACEHOLDER} F{}", fmt_num(ctx.cfg.entry_feed)),
        format!(
            "G0 X{} Y{} F{}",
            fmt_num(px),
            fmt_num(py),
            fmt_num(ctx.cfg.travel_feed)
        ),
    ]);
    step(
        Phase::ParkForReheat,
        "lift off the part (rail-clamped) and park at the reheat XY (the file reheats to print temp here)",
        commands,
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
            // The lift landed at the computed clamped height.
            Verification::new(
                "toolhead",
                "position.2",
                Predicate::NumWithinComputed {
                    epsilon: ctx.cfg.z_epsilon,
                },
            ),
        ],
        Some(RuntimeComputation::ParkZ {
            delta_z: ctx.cfg.reheat_park_delta_z,
            z_max: ctx.machine.axis_limits.z_max,
        }),
        AbortReason::ParkForReheatFailed,
    )
}

fn step_restore_frame(ctx: &Ctx<'_>) -> RecoveryStep {
    let g = &ctx.gcode;
    // Replay the frame state: offsets, speed/extrude factors, skew, and
    // fans. Print temperatures and the file feedrate are restored INSIDE
    // the recovery file (the reheat is gated there), so they are absent
    // here.
    let mut commands = vec![
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
    commands.extend_from_slice(ctx.fan_cmds);
    let verify = vec![
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
    ];
    step(
        Phase::RestoreFrame,
        "replay offsets, speed/extrude factors, skew, fans (print temps reheat in the file)",
        commands,
        vec![],
        verify,
        None,
        AbortReason::RestoreFailed,
    )
}

/// The entry-move commands (travel above the part interior, descend,
/// prime, restore E frame / modes / feedrate). These relocate INTO the
/// generated recovery file (section e); the plan carries them only via
/// the [`crate::resume_file::RecoveryFileSpec`]. The file has already
/// re-homed XY and heated, so these run from home XY / park Z.
///
/// # Errors
///
/// [`RecoveryError::NonFinite`] on any non-finite derived coordinate.
fn build_entry_commands(ctx: &Ctx<'_>) -> Result<Vec<String>, RecoveryError> {
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
    // Re-assert the file feedrate: the entry moves' F words overwrote it.
    commands.push(format!("G1 F{}", fmt_num(g.speed_raw)));
    Ok(commands)
}

fn step_recovery_file_select(ctx: &Ctx<'_>) -> RecoveryStep {
    // Select the GENERATED recovery file and start it. No M26: the
    // recovery file already begins at the resume boundary (its verbatim
    // tail). Exclude-object state is restored between M23 (which resets
    // it) and M24.
    let mut commands = vec![format!("M23 {}", ctx.recovery_file_name)];
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
    commands.push("M24".to_owned());
    step(
        Phase::RecoveryFileSelect,
        "select the generated recovery file (M23), restore exclude-object state, start it (M24)",
        commands,
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
        AbortReason::RecoveryFileSelectFailed,
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

/// Warnings about the configuration itself: an `UNSAFE_` escape hatch in
/// force, a sub-floor drag temperature, and keys this machine's probe
/// path or config mode cannot honour.
///
/// An override that fires silently is not an escape hatch, it is a booby
/// trap: whoever set it may not be the person standing at the printer
/// now. The `UNSAFE_` warning is
/// [`crate::diagnosis::Tier::Advisory`] — the deliberate `printer.cfg`
/// edit WAS the confirmation, so demanding a second one at the worst
/// possible moment would defeat the design.
fn config_warnings(machine: &ValidatedMachine, cfg: &PlanConfig) -> Vec<PlanWarning> {
    let mut out = Vec::new();
    if cfg.unsafe_allow_purge_z_below_bed && cfg.purge_z.is_some_and(|z| z.is_finite() && z < 0.0) {
        out.push(PlanWarning::UnsafeOverrideActive {
            key: crate::diagnosis::UNSAFE_PURGE_Z_BELOW_BED.to_owned(),
            permitted: "purge_z_below_bed".to_owned(),
        });
    }
    // Only on a machine that actually drags: on Tap / load-cell the key is
    // never read, and pausing a recovery to ask about an inert setting is
    // precisely the pointless obstruction this framework removes.
    if matches!(machine.probe.kind, ProbeKind::AdxlDrag { .. })
        && cfg.drag_nozzle_temp > 0.0
        && cfg.drag_nozzle_temp < DRAG_TEMP_FLOOR
    {
        out.push(PlanWarning::DragTempBelowFloor {
            drag_nozzle_temp: cfg.drag_nozzle_temp,
            floor: DRAG_TEMP_FLOOR,
        });
    }
    if let Some(accel_probe) = cfg.accel_probe {
        if uses_consensus_touch(machine, cfg) {
            out.push(PlanWarning::AccelProbeIgnoredOnTouchPath { accel_probe });
        }
    }
    // The generated file can only carry an accel clamp if it can also name
    // the value to restore afterwards (see `entry_accel_pair`).
    if let Some(accel_entry) = cfg.accel_entry {
        if machine.max_accel.is_none() {
            out.push(PlanWarning::AccelEntryNotAppliedToFile { accel_entry });
        }
    }
    out
}

/// The `(clamp, restore)` acceleration pair the generated recovery file
/// wraps its entry moves in, or `None` when it must emit neither.
///
/// The entry moves were relocated INTO the recovery file, so they — not
/// the plan's phases — are the motion that actually descends toward the
/// part, and they are the single place a low acceleration matters most.
/// The file has no runtime-placeholder machinery, so both values must be
/// literals known now: the clamp is `accel_entry`, and the restore is the
/// machine's own configured `max_accel`.
///
/// Both or neither. A clamp the file cannot undo would leave the printer
/// at the recovery acceleration for the entire remaining print, which is
/// a worse outcome than not clamping at all — so an unknown `max_accel`
/// skips the pair and says so ([`PlanWarning::AccelEntryNotAppliedToFile`]).
fn entry_accel_pair(machine: &ValidatedMachine, cfg: &PlanConfig) -> Option<(f64, f64)> {
    Some((cfg.accel_entry?, machine.max_accel?))
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

/// Assembles the steps in the strict recovery-UX order and renumbers
/// them.
fn build_steps(ctx: &Ctx<'_>) -> Result<Vec<RecoveryStep>, RecoveryError> {
    let transforms = &ctx.context.transforms;
    // 1–5: idle timeout, stepper enable, the FIRST heating action
    // (non-blocking bed + nozzle), the believed-Z declare + pre-home
    // lift, XY homing (now after the lift), and the clean-nozzle step.
    let mut steps = vec![step_idle_timeout(ctx), step_stepper_enable(ctx)];
    // 1c: the acceleration frame, before the first motion of any kind, so
    // the value it records is genuinely the machine's own and so every
    // later step (and every abort cleanup) has something to restore to.
    if uses_accel_overrides(ctx.cfg) {
        steps.push(step_recovery_accel(ctx));
    }
    steps.push(step_immediate_bed_heat(ctx));
    steps.push(step_believed_z_declare(ctx));
    steps.push(step_home_xy(ctx));
    // The temperature HOLD goes between homing and the clean: reaching
    // temperature first means heat-up ooze is wiped by the clean rather
    // than deposited during the probe. Absent on an opted-out cold drag.
    steps.extend(step_probe_temp_hold(ctx));
    steps.push(step_clean_nozzle(ctx));
    // 6: the probe envelope machinery (unchanged content).
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
    // 7: true-Z declare, the optional operator Z confirmation, mesh,
    // final declare.
    steps.push(step_true_z_declare(ctx));
    if ctx.cfg.confirm_z_before_resume {
        steps.push(step_z_confirm_standoff(ctx));
    }
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
    // 8: park for reheat, then the frame restore (offsets/factors/skew/
    // fans). 9: select and start the generated recovery file.
    steps.push(step_park_for_reheat(ctx));
    steps.push(step_restore_frame(ctx));
    if uses_accel_overrides(ctx.cfg) {
        steps.push(step_recovery_accel_restore());
    }
    steps.push(step_recovery_file_select(ctx));
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
#[allow(clippy::too_many_lines)] // linear input validation + assembly
pub fn plan_recovery(
    inputs: &PlanInputs<'_>,
    config: &PlanConfig,
) -> Result<PlanOutcome, RecoveryError> {
    config.validate()?;
    let machine = validate_machine(inputs.machine).map_err(|r| RecoveryError::MachineRejected {
        failures: r.failures,
    })?;
    // Probe-path-specific config checks, now that the machine is known.
    config.validate_for_probe(&machine.probe.kind)?;
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
    warnings.extend(config_warnings(&machine, config));

    // The conservative believed Z: the upper bound of the possible-stop
    // set (declared before XY homing so the homing moves clear the part).
    let z_span = recovery.stop_set.z_span().ok_or(RecoveryError::NoZSpan)?;
    let believed_z = z_span.hi;
    if !believed_z.is_finite() {
        return Err(RecoveryError::NonFinite {
            field: "believed_z",
        });
    }

    // The reheat park point: configured (cross-checked against the part
    // footprint) or computed on a side that verifiably clears it.
    let park_choice = reheat_park(config, inputs.model, &machine.axis_limits);
    let park = park_choice.point;
    if !park.iter().all(|v| v.is_finite()) {
        return Err(RecoveryError::NonFinite {
            field: "reheat_park",
        });
    }
    warnings.extend(park_choice.warning);

    // The generated recovery file's name (collision resolution is the
    // daemon's job; the builder emits the plain desired name).
    let recovery_name = crate::resume_file::recovery_file_name(&file_name, &|_| false);

    let ctx = Ctx {
        cfg: config,
        machine: &machine,
        context,
        gcode,
        envelope,
        candidate,
        formula,
        resume,
        bed: preheat.bed,
        other_heaters: &preheat.other_heaters,
        fan_cmds: &fan_cmds,
        excludes: inputs.exclude_objects,
        believed_z,
        park,
        clean_nozzle_present: inputs.clean_nozzle_macro_present,
        recovery_file_name: &recovery_name,
    };
    let steps = build_steps(&ctx)?;

    // The recovery-file spec: the entry moves relocate into the file, and
    // the print-temperature reheat / purge live there behind the heating
    // gate. Built from the same ctx so the derivation is shared.
    let entry_commands = build_entry_commands(&ctx)?;
    // Purge precedence (see `resolve_purge`): disabled / macro-owned /
    // REFUSE on a missing macro / built-in at the resolved location.
    let purge = resolve_purge(config, inputs.purge_macro_present, park)?;
    // A built-in purge that lands on printed geometry warns (never
    // refuses — a sacrificial area is a legitimate target).
    if let Some(point) = purge.as_ref().and_then(crate::PurgePlan::built_in_point) {
        if part_bbox(inputs.model).is_some_and(|bb| inside_bbox(point, bb)) {
            warnings.push(PlanWarning::PurgeInsidePart {
                point,
                configured: config.purge_x.is_some() || config.purge_y.is_some(),
                // Carried so the warning can distinguish "drops filament
                // from the parked height" from "DESCENDS into the part".
                purge_z: config.purge_z,
            });
        }
    }
    // A purge Z below the resume Z is below the top of what is already
    // printed. Not a refusal (the purge point may be over bare bed, where
    // a low Z is exactly right) but the operator should see it.
    if let Some(z) = config.purge_z {
        // Both in the file's g-code frame (the frame the recovery file's
        // own absolute moves run in), so they are directly comparable.
        let resume_z = resume.position[2] - ctx.gcode.origin[2];
        if resume_z.is_finite() && z < resume_z {
            warnings.push(PlanWarning::PurgeZBelowResume {
                purge_z: z,
                resume_z,
            });
        }
    }
    let recovery_file = crate::resume_file::RecoveryFileSpec {
        name: recovery_name.clone(),
        source_name: file_name.clone(),
        plan_id: format!("plr-{}", resume.offset),
        tail_offset: resume.offset,
        bed: preheat.bed,
        nozzle: nozzle_print,
        purge,
        park,
        park_feed: config.travel_feed,
        // The purge descent is a near-part move: slow entry feedrate.
        descend_feed: config.entry_feed,
        entry_commands,
        // The entry moves are the near-part descent, so `accel_entry`
        // belongs here rather than only on the plan's phases.
        entry_accel: entry_accel_pair(&machine, config),
        header_cap: RECOVERY_HEADER_CAP,
    };

    let plan = RecoveryPlan {
        steps,
        envelope,
        resume_file: recovery_name,
        resume_offset: resume.offset,
        requires_clean_nozzle_confirmation: !inputs.clean_nozzle_macro_present,
        recovery_file,
        debug_confirm_each_step: config.debug_confirm_each_step,
        confirm_timeout_s: config.confirm_timeout_s,
        gcode_barrier_timeout_s: config.gcode_barrier_timeout_s,
        warnings,
    };
    run_preflight(&plan, &machine, candidate.point)?;
    Ok(PlanOutcome::Plan(Box::new(plan)))
}

/// Pre-flights the GENERATED RECOVERY FILE's own absolute coordinates —
/// the re-park travel, the purge, and the entry moves that used to be the
/// plan's `Entry` step — against the same axis limits the plan itinerary
/// is checked against.
///
/// The file is played back by Klipper with no per-step verification, so an
/// out-of-range coordinate there would surface only as a mid-recovery
/// "Move out of range" AFTER the probe established the Z reference. The
/// daemon calls this on the same build path that generates the file, with
/// the identical bounds, and refuses the recovery on any violation.
///
/// # Errors
///
/// [`RecoveryError::ItineraryRejected`] listing every violation.
pub fn preflight_generated_file(
    file: &crate::resume_file::GeneratedRecoveryFile,
    machine: &MachineConfig,
    contact_point: [f64; 2],
) -> Result<(), RecoveryError> {
    let validated = validate_machine(machine).map_err(|r| RecoveryError::MachineRejected {
        failures: r.failures,
    })?;
    let bounds = ItineraryBounds {
        x: validated.axis_limits.x,
        y: validated.axis_limits.y,
        z_max: validated.axis_limits.z_max,
        position_min: validated.z_position_min,
        contact_point,
    };
    crate::preflight::preflight_recovery_file(file, &bounds)?;
    Ok(())
}

/// Cap on leading comment lines the recovery file copies from the
/// original (slicer metadata header).
const RECOVERY_HEADER_CAP: usize = 200;

/// Resolves the `[plr]` purge precedence table into a [`PurgePlan`].
///
/// | `purge_enable` | `purge_macro` | macro exists | result |
/// |---|---|---|---|
/// | `false` | *(any)* | *(any)* | `None` — no purge of any kind |
/// | `true` | set | yes | [`PurgePlan::Macro`] — the macro owns everything |
/// | `true` | set | **no** | **REFUSE** ([`RecoveryError::PurgeMacroMissing`]) |
/// | `true` | unset | — | [`PurgePlan::BuiltIn`] at the resolved location |
///
/// # Why a missing purge macro REFUSES rather than degrading
///
/// The clean-nozzle path degrades to asking the operator, and that is
/// safe: a human confirming a clean tip is a real substitute for a macro
/// that wipes it. A purge has no equivalent human fallback — substituting
/// the built-in would extrude filament at a location and rate the
/// operator never asked for, which is precisely how a nozzle ends up
/// purging somewhere unintended. So the asymmetry is deliberate: the
/// clean-nozzle macro degrades, the purge macro refuses.
///
/// # Errors
///
/// [`RecoveryError::PurgeMacroMissing`] when `purge_macro` names a macro
/// that does not exist on the machine.
fn resolve_purge(
    config: &PlanConfig,
    purge_macro_present: bool,
    park: [f64; 2],
) -> Result<Option<crate::PurgePlan>, RecoveryError> {
    if !config.purge_enable {
        return Ok(None);
    }
    let configured_macro = config
        .purge_macro
        .as_deref()
        .map(str::trim)
        .filter(|m| !m.is_empty());
    if let Some(name) = configured_macro {
        if !purge_macro_present {
            return Err(RecoveryError::PurgeMacroMissing {
                name: name.to_owned(),
            });
        }
        return Ok(Some(crate::PurgePlan::Macro {
            call: name.to_owned(),
        }));
    }
    // Built-in: each coordinate defaults to the already-computed,
    // part-clear, bounds-checked park point.
    Ok(Some(crate::PurgePlan::BuiltIn {
        point: [
            config.purge_x.unwrap_or(park[0]),
            config.purge_y.unwrap_or(park[1]),
        ],
        z: config.purge_z,
        amount: config.purge_amount,
        speed: config.purge_speed,
        retract: config.purge_retract,
        travel_feed: config.travel_feed,
    }))
}

/// Margin, mm, by which a computed park point clears the part's XY
/// bounding box.
const PART_MARGIN: f64 = 10.0;

/// The part's XY bounding box `[min_x, min_y, max_x, max_y]` from the
/// model's deposition segments, or `None` when the model has no
/// finite-coordinate deposition.
fn part_bbox(model: &LayerModel) -> Option<[f64; 4]> {
    let mut bbox: Option<[f64; 4]> = None;
    for layer in &model.layers {
        for path in &layer.paths {
            for seg in &path.segments {
                for p in [seg.start, seg.end] {
                    if !p.iter().all(|v| v.is_finite()) {
                        continue;
                    }
                    bbox = Some(match bbox {
                        None => [p[0], p[1], p[0], p[1]],
                        Some([mnx, mny, mxx, mxy]) => {
                            [mnx.min(p[0]), mny.min(p[1]), mxx.max(p[0]), mxy.max(p[1])]
                        }
                    });
                }
            }
        }
    }
    bbox
}

/// `true` when `[x, y]` lies inside (or on) the part's bounding box.
fn inside_bbox(point: [f64; 2], bbox: [f64; 4]) -> bool {
    let [mnx, mny, mxx, mxy] = bbox;
    point[0] >= mnx && point[0] <= mxx && point[1] >= mny && point[1] <= mxy
}

/// Outcome of choosing the reheat park point.
struct ParkChoice {
    point: [f64; 2],
    warning: Option<PlanWarning>,
}

/// The reheat park point.
///
/// * **Configured** `(reheat_park_x, reheat_park_y)` (both set) is used
///   verbatim — validated finite by [`PlanConfig::validate`] and
///   bounds-checked by the pre-flight — but is still cross-checked
///   against the part footprint: a park point the operator placed INSIDE
///   the part is the same hazard as a computed one, so it warns.
/// * **Computed** otherwise: each side of the bounding box is tried in
///   turn (+X, −X, +Y, −Y), clamped into the known axis limits, and the
///   first candidate that actually lands OUTSIDE the footprint wins.
///   Clamping can pull a candidate back into the part on a machine whose
///   travel barely exceeds the print, which is exactly why every
///   candidate is re-checked after clamping instead of assuming the
///   +X side is clear.
/// * When no candidate clears the part, the honest
///   [`PlanWarning::ReheatParkInsidePart`] is emitted rather than
///   claiming a clearance that does not exist.
fn reheat_park(
    config: &PlanConfig,
    model: &LayerModel,
    limits: &crate::machine::AxisLimits,
) -> ParkChoice {
    let bbox = part_bbox(model);
    let clamp = |mut px: f64, mut py: f64| {
        if let Some((lo, hi)) = limits.x {
            px = px.clamp(lo, hi);
        }
        if let Some((lo, hi)) = limits.y {
            py = py.clamp(lo, hi);
        }
        [px, py]
    };

    // Configured: honored as-is, but cross-checked against the footprint.
    if let (Some(x), Some(y)) = (config.reheat_park_x, config.reheat_park_y) {
        let point = [x, y];
        let warning =
            bbox.filter(|bb| inside_bbox(point, *bb))
                .map(|_| PlanWarning::ReheatParkInsidePart {
                    point,
                    configured: true,
                });
        return ParkChoice { point, warning };
    }

    // No footprint known: park at a modest corner offset. There is
    // nothing to verify the point against, so say exactly that rather
    // than claiming a clearance that was never checked.
    let Some(bb) = bbox else {
        let point = clamp(PART_MARGIN, PART_MARGIN);
        return ParkChoice {
            point,
            warning: Some(PlanWarning::ReheatParkUnverified { point }),
        };
    };
    let [mnx, mny, mxx, mxy] = bb;
    let (cx, cy) = ((mnx + mxx) * 0.5, (mny + mxy) * 0.5);
    // Candidate park points, one per side of the footprint.
    let candidates = [
        [mxx + PART_MARGIN, cy],
        [mnx - PART_MARGIN, cy],
        [cx, mxy + PART_MARGIN],
        [cx, mny - PART_MARGIN],
    ];
    for candidate in candidates {
        let point = clamp(candidate[0], candidate[1]);
        // Re-check AFTER clamping: the clamp may have pulled the point
        // back over the part on a tight machine.
        if !inside_bbox(point, bb) {
            return ParkChoice {
                point,
                warning: Some(PlanWarning::ReheatParkComputed { point }),
            };
        }
    }
    // Every side clamps back inside the footprint: the part occupies the
    // reachable bed. Park at the least-bad candidate and say so honestly.
    let point = clamp(mxx + PART_MARGIN, cy);
    ParkChoice {
        point,
        warning: Some(PlanWarning::ReheatParkInsidePart {
            point,
            configured: false,
        }),
    }
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
    fn preview_keys_are_refused_not_clamped() {
        assert!(PlanConfig::default().validate().is_ok());
        // preview_standoff: non-negative when set. A negative standoff
        // would lower the single hover plane INTO the part — the exact
        // never-descend guarantee this refusal makes structural.
        check(
            "preview_standoff",
            |c, v| c.preview_standoff = Some(v),
            &[0.0, 1.0, 25.0],
            &[-0.01, -1.0],
        );
        // preview_nozzle_temp: [0, max_probe_nozzle_temp] when set. The
        // upper bad value (250 > the default 150 °C ceiling) is a hot,
        // oozing nozzle over the part — the fired-guard input.
        check(
            "preview_nozzle_temp",
            |c, v| c.preview_nozzle_temp = Some(v),
            &[0.0, 100.0, 150.0],
            &[-1.0, 150.01, 250.0],
        );
        // Non-finite is the other refusal (NonFinite, not InvalidPlanConfig).
        for field in ["preview_standoff", "preview_nozzle_temp"] {
            let mut c = PlanConfig::default();
            if field == "preview_standoff" {
                c.preview_standoff = Some(f64::NAN);
            } else {
                c.preview_nozzle_temp = Some(f64::INFINITY);
            }
            assert!(
                matches!(c.validate(), Err(RecoveryError::NonFinite { field: f }) if f == field),
                "{field} non-finite must be refused"
            );
        }
    }

    #[test]
    fn preview_resolvers_apply_the_ruled_defaults() {
        use crate::machine::ProbeKind;
        let mut c = PlanConfig::default();
        // preview_standoff unset -> entry_hop; set -> the value.
        assert!((c.preview_standoff_mm() - c.entry_hop).abs() < 1e-9);
        c.preview_standoff = Some(3.5);
        assert!((c.preview_standoff_mm() - 3.5).abs() < 1e-9);
        // preview_nozzle_temp unset -> the probe hold temp; a tap machine
        // holds at commanded_probe_temp, a cold-drag machine at 0.
        let mut c = PlanConfig::default();
        assert!(
            (c.preview_nozzle_temp_c(&ProbeKind::Tap) - c.commanded_probe_temp()).abs() < 1e-9,
            "tap preview default follows the probe hold temp"
        );
        c.drag_nozzle_temp = 0.0;
        assert!(
            c.preview_nozzle_temp_c(&ProbeKind::AdxlDrag {
                chip: String::new()
            }) < 1e-9,
            "cold-drag preview default is 0 (nothing to hold)"
        );
        // An explicit 0 stays 0 (extra-caution cool) regardless of probe.
        c.preview_nozzle_temp = Some(0.0);
        assert!(c.preview_nozzle_temp_c(&ProbeKind::Tap) < 1e-9);
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
