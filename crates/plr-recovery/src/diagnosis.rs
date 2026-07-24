//! The one shape every refusal, warning, and step failure takes.
//!
//! # Why this type exists
//!
//! Recovering a print after a power loss is the moment an operator is
//! least able to absorb a bare error string. Hours of machine time and a
//! spool of filament are already committed; a message like
//! `invalid plan config field purge_z` is, at that moment, indisputably
//! true and completely useless. Every failure in this system therefore
//! answers three questions in a fixed order:
//!
//! 1. **what** happened, in one line;
//! 2. **why** it matters — the physical or logical consequence, in
//!    operator language, not implementation language;
//! 3. **what to change** — the exact `[plr]` config key or command.
//!
//! Making that structural rather than a thing authors remember is the
//! point of [`Diagnose`]: every mapping is an exhaustive `match` with
//! **no catch-all arm**, so a new error variant fails to compile until
//! somebody writes its diagnosis.
//!
//! # The three tiers
//!
//! [`Tier`] decides what the daemon *does*, and the difference between
//! the tiers is only ever how the escape hatch is reached:
//!
//! * [`Tier::Advisory`] — proceeds by default and warns loudly. No
//!   escape hatch is needed because nothing is being refused.
//! * [`Tier::Confirmable`] — stops, explains, and offers "continue
//!   anyway" over the control socket. The escape hatch is a **click**,
//!   taken while looking at the explanation.
//! * [`Tier::Hard`] — refuses. The only escape is a pre-set
//!   `UNSAFE_`-prefixed `[plr]` config key
//!   ([`Diagnosis::override_key`]), edited in `printer.cfg` while calm,
//!   never a runtime button. Cartographer's `UNSAFE_` / `EXPERIMENTAL_`
//!   option naming is the precedent.
//!
//! A Hard diagnosis whose `override_key` is `None` cannot be permitted
//! by any configuration at all; see [`Diagnosis::override_key`] for the
//! full list and the reasoning.
//!
//! # Numbers are typed, never baked into prose
//!
//! [`Diagnosis::measured`] and [`Diagnosis::expected`] carry the numbers
//! as data so a client can render a gauge, compare against its own
//! bounds, or localize the units — none of which is possible once the
//! number has been `format!`ed into a sentence. The prose still mentions
//! the values (an operator reading a log needs them there), but the
//! prose is not the source of truth.

use serde::Serialize;

use crate::plan::fmt_num;

/// The `[plr]` boolean that permits an otherwise-Hard `purge_z` below
/// the bed surface.
///
/// Klipper lowercases option names in `configfile.settings`, so the
/// daemon looks the key up case-insensitively; the operator writes it in
/// `printer.cfg` exactly as spelled here, and the screaming prefix is
/// the point — it is meant to be uncomfortable to type.
pub const UNSAFE_PURGE_Z_BELOW_BED: &str = "UNSAFE_allow_purge_z_below_bed";

// There is deliberately exactly ONE `UNSAFE_` key. The tier boundary is
// "Hard = physical damage or an unknowable machine state", and
// `purge_z_below_bed` is the only refusal whose consequence is a nozzle
// driven into the bed. Anything whose worst case is a bounded wait ending
// in a clean abort belongs in the Confirmable tier, where the operator
// gets an explanation and a button — which is what the whole diagnosis
// framework exists to provide.

/// What the daemon does with a [`Diagnosis`] (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Tier {
    /// Proceeds by default; warns loudly.
    Advisory,
    /// Stops, explains, and offers "continue anyway" over the control
    /// socket.
    Confirmable,
    /// Refuses. Only a pre-set `UNSAFE_` `[plr]` key can permit it, and
    /// only when [`Diagnosis::override_key`] names one.
    Hard,
}

impl Tier {
    /// Stable machine-readable tag (the JSON wire value).
    #[must_use]
    pub fn tag(self) -> &'static str {
        match self {
            Tier::Advisory => "advisory",
            Tier::Confirmable => "confirmable",
            Tier::Hard => "hard",
        }
    }

    /// `true` when execution must stop and ask rather than proceed.
    #[must_use]
    pub fn stops_execution(self) -> bool {
        matches!(self, Tier::Confirmable | Tier::Hard)
    }
}

/// A measured quantity, typed rather than baked into prose.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Measured {
    /// What was measured (`"purge_z"`, `"extruder.temperature"`).
    pub quantity: String,
    /// The observed value.
    pub value: f64,
    /// Its unit (`"mm"`, `"C"`, `"mm/s"`, `""` when dimensionless).
    pub unit: &'static str,
}

/// The band a [`Measured`] value should have fallen in. At least one of
/// `min`/`max` is set; both set means a closed band.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Expected {
    /// What the band applies to (usually the same as the measurement).
    pub quantity: String,
    /// Inclusive lower bound, when there is one.
    pub min: Option<f64>,
    /// Inclusive upper bound, when there is one.
    pub max: Option<f64>,
    /// Its unit.
    pub unit: &'static str,
}

impl Expected {
    fn render(&self) -> String {
        match (self.min, self.max) {
            (Some(lo), Some(hi)) => format!("[{}, {}]", fmt_num(lo), fmt_num(hi)),
            (Some(lo), None) => format!(">= {}", fmt_num(lo)),
            (None, Some(hi)) => format!("<= {}", fmt_num(hi)),
            (None, None) => "(unbounded)".to_owned(),
        }
    }
}

/// One failure, warning, or refusal — explained.
///
/// Produced by [`Diagnose::diagnosis`] for every typed failure in the
/// system, rendered by [`Diagnosis::one_line`] /
/// [`Diagnosis::full`], and serialized verbatim into control-socket
/// responses so a client renders every diagnosis exactly one way.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Diagnosis {
    /// Stable machine-readable identifier (`"purge_z_below_bed"`).
    /// Clients branch on this and nothing else; the prose may be
    /// reworded, the code may not.
    pub code: &'static str,
    /// What the daemon does about it.
    pub tier: Tier,
    /// One line: what happened.
    pub what: String,
    /// The physical or logical consequence, in operator language.
    pub why: String,
    /// The exact config key or command to change.
    pub suggested_fix: String,
    /// The measured value, when a number is involved.
    pub measured: Option<Measured>,
    /// The band the measurement should have fallen in.
    pub expected: Option<Expected>,
    /// For [`Tier::Hard`]: the `UNSAFE_`-prefixed `[plr]` key that
    /// would permit this anyway, or `None` when **nothing** may permit
    /// it.
    ///
    /// **Exactly one** Hard diagnosis carries an override:
    /// [`UNSAFE_PURGE_Z_BELOW_BED`] for `purge_z_below_bed`. Its
    /// consequence is confined to the purge blob, and the operator can
    /// see the geometry involved — so a deliberate edit may permit it.
    ///
    /// Every other Hard diagnosis is `None`, because permitting it would
    /// mean commanding motion against an unknown or invalid Z frame
    /// (`no_trusted_z_span`, `itinerary_out_of_bounds`,
    /// `non_finite_input`, the whole machine-prerequisite family), or
    /// commanding a temperature the Klipper plugin's own ceiling gate
    /// would refuse *after* the frame is declared — wedging the recovery
    /// in exactly the state it must never be left in.
    ///
    /// The set is small on purpose. A failure whose worst case is a
    /// bounded wait ending in a clean abort is not Hard at all: it is
    /// [`Tier::Confirmable`], where the operator gets the same
    /// explanation and a button instead of a config edit.
    pub override_key: Option<&'static str>,
}

impl Diagnosis {
    /// A diagnosis with no numbers and no override.
    #[must_use]
    pub fn new(
        code: &'static str,
        tier: Tier,
        what: impl Into<String>,
        why: impl Into<String>,
        suggested_fix: impl Into<String>,
    ) -> Self {
        Self {
            code,
            tier,
            what: what.into(),
            why: why.into(),
            suggested_fix: suggested_fix.into(),
            measured: None,
            expected: None,
            override_key: None,
        }
    }

    /// Attaches the measured value.
    #[must_use]
    pub fn measured(mut self, quantity: impl Into<String>, value: f64, unit: &'static str) -> Self {
        self.measured = Some(Measured {
            quantity: quantity.into(),
            value,
            unit,
        });
        self
    }

    /// Attaches the band the measurement should have fallen in.
    #[must_use]
    pub fn expected(
        mut self,
        quantity: impl Into<String>,
        min: Option<f64>,
        max: Option<f64>,
        unit: &'static str,
    ) -> Self {
        self.expected = Some(Expected {
            quantity: quantity.into(),
            min,
            max,
            unit,
        });
        self
    }

    /// Names the `UNSAFE_` key that would permit this Hard refusal.
    #[must_use]
    pub fn overridable_by(mut self, key: &'static str) -> Self {
        self.override_key = Some(key);
        self
    }

    /// The compact form: one line, for a log or a status list.
    #[must_use]
    pub fn one_line(&self) -> String {
        use std::fmt::Write as _;
        // Writing into a String is infallible; the results are discarded.
        let mut s = format!("[{}] {}: {}", self.tier.tag(), self.code, self.what);
        if let Some(m) = &self.measured {
            let _ = write!(
                s,
                " (measured {} = {}{})",
                m.quantity,
                fmt_num(m.value),
                unit_suffix(m.unit)
            );
        }
        s
    }

    /// The full form: the three-part message an operator reads.
    #[must_use]
    pub fn full(&self) -> String {
        use std::fmt::Write as _;
        let mut s = String::new();
        let _ = writeln!(
            s,
            "{} [{}] {}",
            tier_banner(self.tier),
            self.code,
            self.what
        );
        let _ = writeln!(s, "  why:      {}", self.why);
        if let Some(m) = &self.measured {
            let _ = writeln!(
                s,
                "  measured: {} = {}{}",
                m.quantity,
                fmt_num(m.value),
                unit_suffix(m.unit)
            );
        }
        if let Some(e) = &self.expected {
            let _ = writeln!(
                s,
                "  expected: {} {}{}",
                e.quantity,
                e.render(),
                unit_suffix(e.unit)
            );
        }
        let _ = writeln!(s, "  fix:      {}", self.suggested_fix);
        match (self.tier, self.override_key) {
            (Tier::Hard, Some(key)) => {
                let _ = writeln!(
                    s,
                    "  override: set `{key} = True` in printer.cfg's [plr] section and restart \
                     klippy. There is deliberately no runtime button for this — the only way \
                     past it is an edit made while nothing is at stake."
                );
            }
            (Tier::Hard, None) => {
                s.push_str("  override: NONE. No configuration permits this; it must be fixed.\n");
            }
            (Tier::Confirmable, _) => {
                s.push_str(
                    "  override: answer `continue` to the confirmation to proceed anyway.\n",
                );
            }
            (Tier::Advisory, _) => {}
        }
        s
    }
}

fn unit_suffix(unit: &'static str) -> String {
    if unit.is_empty() {
        String::new()
    } else {
        format!(" {unit}")
    }
}

fn tier_banner(tier: Tier) -> &'static str {
    match tier {
        Tier::Advisory => "WARNING",
        Tier::Confirmable => "STOPPED (confirmable)",
        Tier::Hard => "REFUSED",
    }
}

/// Every typed failure in the system produces exactly one
/// [`Diagnosis`].
///
/// The implementations below are deliberately written as exhaustive
/// matches with **no catch-all arm** (see the module docs): adding a
/// variant to any of the mapped enums is a compile error until its
/// diagnosis is written.
pub trait Diagnose {
    /// This value's diagnosis.
    fn diagnosis(&self) -> Diagnosis;
}

// --- RecoveryError -----------------------------------------------------------

impl Diagnose for crate::error::RecoveryError {
    #[allow(clippy::too_many_lines)] // one arm per variant, by design
    fn diagnosis(&self) -> Diagnosis {
        use crate::error::RecoveryError as E;
        match self {
            E::NonFinite { field } => Diagnosis::new(
                "non_finite_input",
                Tier::Hard,
                format!("the value for `{field}` is not a real number (NaN or infinity)"),
                "A non-finite number cannot be turned into a coordinate. If it reached a \
                 move it would be sent to Klipper as `nan`, and the resulting motion is \
                 undefined — which, on a machine whose bed rises into a fixed nozzle, means \
                 an undefined collision.",
                format!(
                    "This is corrupt input rather than a setting: re-run `plrd recover` \
                     without `--execute` to regenerate the plan. If `{field}` names a \
                     `[plr]` key, correct it in printer.cfg; if it names WAL data, the \
                     crash record is damaged and manual recovery is required."
                ),
            ),
            E::InvalidContext { field } => Diagnosis::new(
                "wal_context_out_of_domain",
                Tier::Hard,
                format!("the recorded printer state has an impossible `{field}`"),
                "The interpreter state captured at the moment of the power loss is the \
                 only record of what the printer was doing. A value outside its domain \
                 (a zero extrude factor, a negative speed) means the snapshot cannot be \
                 replayed faithfully, and a resume built on it would extrude or move by \
                 the wrong amount.",
                "Nothing in the configuration can fix this — the recorded state itself is \
                 unusable. Recover manually, and check `plrd status` for WAL corruption \
                 (a `SubscriptionGap` marker near the crash points at a disk stall)."
                    .to_owned(),
            ),
            E::ProbeSpeedOutOfRange { speed } => Diagnosis::new(
                "probe_speed_out_of_range",
                Tier::Hard,
                format!(
                    "probe_speed is {} mm/s, outside the validated band",
                    fmt_num(*speed)
                ),
                "The probe envelope's overshoot term is only valid inside this band: it is \
                 derived from how far the toolhead keeps travelling after the trigger is \
                 seen. Probe faster and the nozzle overshoots further than the envelope \
                 allows for, which is the one number standing between the descent and the \
                 part.",
                format!(
                    "Set `probe_speed` in printer.cfg's [plr] section to a value in \
                     [{}, {}] mm/s (1 mm/s is the conservative default).",
                    fmt_num(crate::envelope::PROBE_SPEED_MIN),
                    fmt_num(crate::envelope::PROBE_SPEED_MAX)
                ),
            )
            .measured("probe_speed", *speed, "mm/s")
            .expected(
                "probe_speed",
                Some(crate::envelope::PROBE_SPEED_MIN),
                Some(crate::envelope::PROBE_SPEED_MAX),
                "mm/s",
            ),
            E::InvalidPlanConfig { field } => Diagnosis::new(
                "plan_config_out_of_range",
                Tier::Hard,
                format!("the `{field}` setting is outside its permitted range"),
                "Every planning tunable has a hard band that plrd and the Klipper plugin \
                 both enforce. Values outside it are refused rather than clamped: silently \
                 substituting a different number than the operator asked for is how a \
                 recovery ends up doing something nobody chose.",
                format!(
                    "Correct `{field}` in printer.cfg's [plr] section. The permitted band \
                     for every key is listed in deploy/plrd.conf.example."
                ),
            ),
            E::AccelOutOfRange {
                key,
                value,
                min,
                max,
            } => Diagnosis::new(
                "accel_out_of_range",
                Tier::Hard,
                format!(
                    "the acceleration override `{key}` is {} mm/s^2, outside the permitted band",
                    fmt_num(*value)
                ),
                "Recovery moves happen next to printed geometry with a Z frame that was \
                 just established by a single contact measurement. An absurdly high \
                 acceleration turns a small position error into an impact; an absurdly low \
                 one makes the recovery outlast the plan's own step timeouts.",
                format!(
                    "Set `{key}` in printer.cfg's [plr] section to a value in [{}, {}] \
                     mm/s^2, or remove the key entirely to leave the machine's own \
                     acceleration alone.",
                    fmt_num(*min),
                    fmt_num(*max)
                ),
            )
            .measured((*key).to_string(), *value, "mm/s^2")
            .expected((*key).to_string(), Some(*min), Some(*max), "mm/s^2"),
            E::ProbeTempHeadroomUnavailable {
                probe_temp_min,
                ceiling,
                headroom,
            } => Diagnosis::new(
                "probe_temp_headroom_unavailable",
                Tier::Hard,
                format!(
                    "the probing temperature band is too narrow: probe_temp_min {} C leaves \
                     no room below the {} C contact ceiling for the required {} C of headroom",
                    fmt_num(*probe_temp_min),
                    fmt_num(*ceiling),
                    fmt_num(*headroom)
                ),
                "The Klipper plugin refuses any contact operation when the nozzle is at or \
                 above the contact ceiling. A PID hotend routinely overshoots its target by \
                 a degree or two, so aiming AT the ceiling means the probe is refused on a \
                 normal overshoot — and that refusal lands after the Z frame has already \
                 been declared, which wedges the recovery. plrd therefore refuses the \
                 configuration up front instead.",
                "Lower `probe_temp_min`, or raise `probe_temp_max` / \
                 `max_probe_nozzle_temp`, in printer.cfg's [plr] section. The ceiling is \
                 min(probe_temp_max, max_probe_nozzle_temp) and the commanded target must \
                 sit at least the headroom below it."
                    .to_owned(),
            )
            .measured("probe_temp_min", *probe_temp_min, "C")
            .expected("probe_temp_min", None, Some(ceiling - headroom), "C"),
            E::DragTempOutOfRange {
                drag_nozzle_temp,
                ceiling,
                headroom,
            } => Diagnosis::new(
                "drag_temp_out_of_range",
                Tier::Hard,
                format!(
                    "drag_nozzle_temp {} C does not leave {} C of headroom below the {} C \
                     contact ceiling",
                    fmt_num(*drag_nozzle_temp),
                    fmt_num(*headroom),
                    fmt_num(*ceiling)
                ),
                "The plan would command this temperature and then hold for it, and the \
                 Klipper plugin's ceiling gate would refuse the drag when it arrived — \
                 after the shifted Z frame has been declared. That is the one failure mode \
                 that leaves the printer in an ambiguous frame with nothing left to do but \
                 a manual recovery.",
                "Lower `drag_nozzle_temp`, or raise `probe_temp_max` / \
                 `max_probe_nozzle_temp`, in printer.cfg's [plr] section. Setting \
                 `drag_nozzle_temp = 0` opts out of heating for the drag entirely (a \
                 deliberate cold drag: no heating and no wait)."
                    .to_owned(),
            )
            .measured("drag_nozzle_temp", *drag_nozzle_temp, "C")
            .expected("drag_nozzle_temp", Some(0.0), Some(ceiling - headroom), "C"),
            E::PurgeMacroMissing { name } => Diagnosis::new(
                "purge_macro_missing",
                Tier::Hard,
                format!("purge_macro names {name:?}, but no [gcode_macro {name}] exists"),
                "The operator asked for a specific purge routine, with its own positioning \
                 and extrusion. Quietly substituting the built-in purge would extrude \
                 filament at a different place, height and rate than was asked for — over \
                 a part that is already half printed.",
                format!(
                    "Add `[gcode_macro {name}]` to printer.cfg, correct the `purge_macro` \
                     value in the [plr] section, unset `purge_macro` to use the built-in \
                     purge, or set `purge_enable = False` to skip purging entirely."
                ),
            ),
            E::PurgeZBelowBed { purge_z } => Diagnosis::new(
                "purge_z_below_bed",
                Tier::Hard,
                format!(
                    "purge_z is {} mm, which is below the bed surface",
                    fmt_num(*purge_z)
                ),
                "The generated recovery file runs in the TRUE frame, where Z = 0 IS the \
                 bed surface. A negative purge_z therefore drives a nozzle at full print \
                 temperature into the bed and extrudes into it. The Z rail's position_min \
                 is deliberately BELOW the bed in this design (it gives the shifted-frame \
                 probe envelope room), so Klipper's own rail limit will not stop this one.",
                "Set `purge_z` in printer.cfg's [plr] section to at least 0 — and, to clear \
                 what is already printed, at or above the resume Z. Unsetting it purges at \
                 the parked height instead."
                    .to_owned(),
            )
            .measured("purge_z", *purge_z, "mm")
            .expected("purge_z", Some(0.0), None, "mm")
            .overridable_by(UNSAFE_PURGE_Z_BELOW_BED),
            E::ConfirmTimeoutOutOfRange { value, min, max } => Diagnosis::new(
                "confirm_timeout_out_of_range",
                Tier::Hard,
                format!(
                    "confirm_timeout_s is {} s, outside the permitted band",
                    fmt_num(*value)
                ),
                "This is how long a confirmation pause waits before giving up and aborting. \
                 Set too short, a pause that asks you to walk to the printer and look at the \
                 nozzle expires before you get there; set too long (or to zero, or to \
                 infinity), an abandoned recovery sits paused with the heaters on and the \
                 toolhead over the part indefinitely.",
                format!(
                    "Set `confirm_timeout_s` in printer.cfg's [plr] section to a value in \
                     [{}, {}] seconds, or remove the key to use the {} s default.",
                    fmt_num(*min),
                    fmt_num(*max),
                    fmt_num(crate::build::CONFIRM_TIMEOUT_DEFAULT_S)
                ),
            )
            .measured("confirm_timeout_s", *value, "s")
            .expected("confirm_timeout_s", Some(*min), Some(*max), "s"),
            E::MachineRejected { failures } => Diagnosis::new(
                "machine_prerequisites_failed",
                Tier::Hard,
                format!("{} machine prerequisite check(s) failed", failures.len()),
                "These are the structural assumptions the whole recovery rests on — that \
                 Z can be force-moved, that the leadscrews held the bed where it was, that \
                 there is exactly one probe and it does not move on activation. Recovery \
                 without them is not a degraded recovery; it is an uncontrolled one.",
                "Fix each listed prerequisite (every failure carries its own diagnosis), \
                 then re-run `plrd recover` without `--execute` to re-validate."
                    .to_owned(),
            ),
            E::NoContext => Diagnosis::new(
                "no_wal_context",
                Tier::Hard,
                "the crash record holds no interpreter state snapshot",
                "Without a context snapshot there is no record of offsets, factors, \
                 temperatures or position at the moment of the loss. Nothing can be \
                 restored because nothing was captured.",
                "This WAL cannot be recovered from. Check that plrd was running and \
                 recording before the loss (`plrd status` shows segment count and \
                 heartbeat age); recover this print manually."
                    .to_owned(),
            ),
            E::NoVirtualSd => Diagnosis::new(
                "no_virtual_sdcard_state",
                Tier::Hard,
                "no virtual_sdcard print was active in the crash record",
                "A resume needs a file and a byte offset to resume FROM. The recorded \
                 state shows no file print in progress, so there is no print to pick up.",
                "Nothing to do — this is the correct answer for a machine that was idle, \
                 or was printing over a serial/streaming host rather than from the \
                 virtual SD card (which this design cannot resume)."
                    .to_owned(),
            ),
            E::FileNotTopLevel { path } => Diagnosis::new(
                "print_file_not_top_level",
                Tier::Hard,
                format!("the print file {path} is not at the virtual_sdcard top level"),
                "The resume selects the generated recovery file with `M23`, and `M23` \
                 cannot name a file inside a subdirectory. The recovery file could be \
                 written, but the printer would never be able to select it.",
                "Move the print file (and re-slice/re-upload if necessary) to the top \
                 level of the virtual_sdcard root, then re-run the recovery."
                    .to_owned(),
            ),
            E::NoZSpan => Diagnosis::new(
                "no_trusted_z_span",
                Tier::Hard,
                "the reconstruction found no trusted Z candidate, so the probe envelope \
                 cannot be sized",
                "The probe envelope is what structurally bounds the descent: it is sized \
                 from the range of Z values the reconstruction is willing to vouch for. \
                 With no trusted span there is no bound, and a descent with no bound on a \
                 machine that never re-homes Z is exactly the thing this system exists to \
                 avoid.",
                "No configuration change helps: the recorded step history does not pin Z \
                 closely enough. Recover manually. If this recurs, check that every Z \
                 stepper is listed in plrd's `z_steppers` and lives on the primary MCU."
                    .to_owned(),
            ),
            E::NoProbeCandidates => Diagnosis::new(
                "no_probe_candidates",
                Tier::Hard,
                "the contact analysis returned a candidate list that was empty",
                "There is nowhere on the part the analyzer is willing to touch: every \
                 candidate zone was excluded. Probing anyway would mean picking a point \
                 the analysis specifically declined.",
                "Reduce `exclusion_radius` in printer.cfg's [plr] section if the crash \
                 point excluded the only flat area, or recover manually — the geometry \
                 near the stop may simply offer no safe contact zone."
                    .to_owned(),
            ),
            E::NoNozzleTarget => Diagnosis::new(
                "no_nozzle_target",
                Tier::Hard,
                "no print nozzle temperature could be established from the crash record \
                 or the sliced file",
                "Resuming means extruding. With no known print temperature the resume \
                 would either run the nozzle cold — jamming the extruder and grinding the \
                 filament — or guess a temperature that may be wrong for the material.",
                "Ensure the sliced file sets its temperatures with `M104`/`M109` in the \
                 header (most slicers do). If it uses only a start macro parameter, \
                 recover manually and preheat by hand."
                    .to_owned(),
            ),
            E::InvalidName { field, name } => Diagnosis::new(
                "invalid_embedded_name",
                Tier::Hard,
                format!("the {field} name {name:?} cannot be embedded in a command"),
                "The name has to be written into a G-code command line. Characters like \
                 quotes, backslashes or newlines would terminate or split that line, so \
                 the printer would receive a different command than the plan describes — \
                 and the plan is the only thing that has been reviewed.",
                format!(
                    "Rename the {field} in printer.cfg to a name without quotes, \
                     backslashes or control characters."
                ),
            ),
            E::ItineraryRejected(rejection) => rejection.diagnosis(),
        }
    }
}

// --- PlanRejection -----------------------------------------------------------

impl Diagnose for crate::preflight::PlanRejection {
    fn diagnosis(&self) -> Diagnosis {
        use crate::preflight::PlanRejection as R;
        match self {
            R::ItineraryOutOfBounds { violations } => {
                let first = violations.first();
                let d = Diagnosis::new(
                    "itinerary_out_of_bounds",
                    Tier::Hard,
                    format!(
                        "{} commanded coordinate(s) in the plan fall outside the machine's \
                         limits or disagree with the selected contact point",
                        violations.len()
                    ),
                    "The whole-itinerary pre-flight walks every coordinate the plan would \
                     command and checks it against the machine's own travel limits before \
                     anything is sent. A violation means the plan itself is wrong — either \
                     it would drive an axis past its rail, or its probe approach does not \
                     land on the contact point the analysis chose. Executing it would mean \
                     trusting arithmetic that has already been shown to disagree with the \
                     machine.",
                    "This is a planning fault, not a setting: re-run `plrd recover` without \
                     `--execute` to regenerate. If it repeats, the machine's axis limits in \
                     printer.cfg most likely do not match the physical machine — check \
                     `position_min`/`position_max` on every rail."
                        .to_owned(),
                );
                match first {
                    Some(v) => d
                        .measured(format!("step {} axis {}", v.step_id, v.axis), v.value, "mm")
                        .expected(
                            format!("step {} axis {}", v.step_id, v.axis),
                            v.min,
                            v.max,
                            "mm",
                        ),
                    None => d,
                }
            }
        }
    }
}

// --- PrereqFailure -----------------------------------------------------------

impl Diagnose for crate::machine::PrereqFailure {
    #[allow(clippy::too_many_lines)] // one arm per variant, by design
    fn diagnosis(&self) -> Diagnosis {
        use crate::machine::PrereqFailure as P;
        match self {
            P::ForceMoveDisabled => Diagnosis::new(
                "force_move_disabled",
                Tier::Hard,
                "[force_move] is missing, or enable_force_move is not True",
                "Every Z motion in this recovery happens inside a frame plrd declares \
                 explicitly, because Z must never be re-homed on a machine whose bed rises \
                 into a fixed nozzle. Declaring that frame requires \
                 `SET_KINEMATIC_POSITION`, which Klipper only provides when force_move is \
                 enabled.",
                "Add to printer.cfg:\n    [force_move]\n    enable_force_move: True\nthen \
                 restart klippy."
                    .to_owned(),
            ),
            P::ZNotSelfLocking => Diagnosis::new(
                "z_not_self_locking",
                Tier::Hard,
                "self-locking Z leadscrews have not been attested",
                "The entire method assumes the bed stayed exactly where the power loss \
                 left it. That is true of self-locking leadscrews and false of belted Z \
                 or a counterweighted gantry, where the bed sags the instant the motors \
                 de-energize — and a sagged bed means the probe measures the wrong \
                 surface.",
                "Run `PLR_SETUP` on the printer and attest self-locking Z when prompted \
                 (it autosaves `self_locking_z` into the [plr] section). Do NOT attest it \
                 on a belted or counterweighted Z axis."
                    .to_owned(),
            ),
            P::NoZSteppers => Diagnosis::new(
                "no_z_steppers",
                Tier::Hard,
                "the machine snapshot lists no Z steppers",
                "Committed Z step history is how the bed's true position at the moment of \
                 the loss is reconstructed. With no Z stepper recorded there is no history, \
                 and therefore no starting estimate for the probe envelope.",
                "List every Z stepper in plrd's `z_steppers` setting (e.g. \
                 `z_steppers = stepper_z, stepper_z1`) and restart plrd."
                    .to_owned(),
            ),
            P::ZStepperOffPrimaryMcu { stepper, mcu } => Diagnosis::new(
                "z_stepper_off_primary_mcu",
                Tier::Hard,
                format!("Z stepper {stepper} is wired to MCU {mcu}, not the primary MCU"),
                "Step counts from a secondary MCU arrive on a different clock domain. \
                 Mixing them into the position reconstruction introduces an unbounded \
                 timing error in exactly the axis that must never be guessed.",
                format!(
                    "Move {stepper} onto the primary [mcu], or recover manually on this \
                     machine. Multi-MCU Z is not supported by this design."
                ),
            ),
            P::NoTypeAnnotations => Diagnosis::new(
                "no_type_annotations",
                Tier::Hard,
                "the sliced file carries no `;TYPE:` annotations",
                "The resume point is chosen by feature type: resuming mid-perimeter leaves \
                 a visible scar and a weak seam, while an infill start is nearly \
                 invisible. Without `;TYPE:` comments there is no way to tell one from the \
                 other, so every candidate resume point is equally blind.",
                "Enable feature-type comments in the slicer (PrusaSlicer/SuperSlicer: \
                 \"Label objects\" / verbose G-code; OrcaSlicer/Cura emit them by default) \
                 and re-slice. Already-printing files cannot be fixed retroactively."
                    .to_owned(),
            ),
            P::NoProbe => Diagnosis::new(
                "no_probe_configured",
                Tier::Hard,
                "no Tap-style [probe] or [load_cell_probe] is configured",
                "The recovery re-establishes Z by touching the part with the nozzle \
                 itself. That needs a probe whose trigger point IS the nozzle tip; an \
                 offset inductive or optical probe measures the bed, not the printed \
                 surface, and cannot answer the question being asked.",
                "Configure a nozzle-contact probe ([probe] on a Tap-style machine, \
                 [load_cell_probe] on a load-cell machine), or set `probe_method = \
                 adxl_drag` in the [plr] section to use accelerometer drag detection \
                 instead."
                    .to_owned(),
            ),
            P::MultipleProbes { count } => Diagnosis::new(
                "multiple_probes",
                Tier::Hard,
                format!("{count} probe objects are listed; exactly one is required"),
                "Klipper allows exactly one probe object. A snapshot showing several means \
                 the configuration plrd read does not describe the machine that is running, \
                 and every offset derived from it is suspect.",
                "Leave exactly one of [probe] / [load_cell_probe] / [bltouch] in \
                 printer.cfg and restart klippy."
                    .to_owned(),
            )
            .measured(
                "probe objects",
                f64::from(u32::try_from(*count).unwrap_or(u32::MAX)),
                "",
            )
            .expected("probe objects", Some(1.0), Some(1.0), ""),
            P::ProbeZOffsetNonFinite => Diagnosis::new(
                "probe_z_offset_non_finite",
                Tier::Hard,
                "the probe's z_offset is not a finite number",
                "The true-Z arithmetic subtracts (or adds back) this offset to convert the \
                 probe's reading into a raw toolhead height. A non-finite offset makes the \
                 whole Z reference undefined.",
                "Set a numeric `z_offset` on the probe section in printer.cfg (run the \
                 machine's normal probe calibration if it has never been set)."
                    .to_owned(),
            ),
            P::ProbeActivateGcodeMoves => Diagnosis::new(
                "probe_activate_gcode_moves",
                Tier::Hard,
                "the probe's activate_gcode is neither empty nor verified move-free",
                "activate_gcode runs immediately before the probe descends — inside the \
                 shifted frame, with the nozzle already close to the part. Any motion in \
                 it happens outside the plan plrd reviewed and the envelope it sized.",
                "Empty `activate_gcode` on the probe section in printer.cfg, or reduce it \
                 to commands that provably do not move the toolhead."
                    .to_owned(),
            ),
            P::ProbeDeactivateGcodeMoves => Diagnosis::new(
                "probe_deactivate_gcode_moves",
                Tier::Hard,
                "the probe's deactivate_gcode is neither empty nor verified move-free",
                "deactivate_gcode runs immediately after the trigger, while the toolhead is \
                 resting at the halt position that the true-Z arithmetic is about to read. \
                 A move there silently corrupts the Z reference for the entire resume.",
                "Empty `deactivate_gcode` on the probe section in printer.cfg, or reduce \
                 it to commands that provably do not move the toolhead."
                    .to_owned(),
            ),
            P::PositionMinUnknown => Diagnosis::new(
                "z_position_min_unknown",
                Tier::Hard,
                "the Z rail's position_min (or [printer] minimum_z_position) is unknown",
                "The shifted frame is declared so that Klipper's own rail-limit check \
                 bounds the descent even if the probe never triggers. That backstop needs \
                 position_min; without it the descent has no structural floor at all.",
                "Set `position_min` on the Z rail (e.g. `position_min: -2`) in \
                 printer.cfg, or `minimum_z_position` in the [printer] section, and \
                 restart klippy."
                    .to_owned(),
            ),
            P::PositionMinNonFinite => Diagnosis::new(
                "z_position_min_non_finite",
                Tier::Hard,
                "the Z rail's position_min is not a finite number",
                "position_min is the structural floor of the probe descent. A non-finite \
                 floor is no floor.",
                "Set a numeric `position_min` on the Z rail in printer.cfg and restart \
                 klippy."
                    .to_owned(),
            ),
            P::ConfigNeverValidated => Diagnosis::new(
                "config_never_validated",
                Tier::Hard,
                "the machine prerequisites have never been validated against a config",
                "plrd will not drive a machine it has never inspected. The validation pass \
                 is what establishes that force_move exists, that Z is self-locking, and \
                 that the probe is usable — none of which can be assumed.",
                "Adopt the [plr] section in printer.cfg (preferred: the snapshot is then \
                 re-read live on every run), or run the legacy validation and set \
                 `validated_config_hash` in /etc/plrd.conf to the hash printed by \
                 `plrd recover`."
                    .to_owned(),
            ),
            P::ConfigChangedSinceValidation { validated, current } => Diagnosis::new(
                "config_changed_since_validation",
                Tier::Hard,
                format!(
                    "printer.cfg changed since validation (validated {validated}, running \
                     {current})"
                ),
                "The prerequisites were checked against a different configuration than the \
                 one now running. Any of the assumptions could have been edited away since \
                 — force_move disabled, the probe swapped, the Z rail's limits moved — and \
                 the recovery would proceed on stale evidence.",
                "Re-run `plrd recover` without `--execute` to re-validate, then set \
                 `validated_config_hash` in /etc/plrd.conf to the hash it prints. Adopting \
                 the [plr] section in printer.cfg removes this step permanently: the \
                 snapshot is then read from the live config every run."
                    .to_owned(),
            ),
            P::SdcardRootUnknown => Diagnosis::new(
                "sdcard_root_unknown",
                Tier::Hard,
                "the [virtual_sdcard] root directory is unknown",
                "The generated recovery file has to be written into that directory, and \
                 the print file has to be proven to sit at its top level so `M23` can \
                 select it. Neither check can run without the path.",
                "Set `path:` on the [virtual_sdcard] section in printer.cfg (or \
                 `virtual_sdcard_root` in /etc/plrd.conf on the legacy path) and restart."
                    .to_owned(),
            ),
            P::AccelChipInvalid { chip } => Diagnosis::new(
                "accel_chip_invalid",
                Tier::Hard,
                format!("accel_chip {chip:?} cannot be embedded in a quoted command argument"),
                "The chip name is written into `PLR_DRAG_PROBE CHIP=\"...\"`. Double \
                 quotes, backslashes and control characters cannot survive that quoting, \
                 so the plugin would receive a different chip name than the plan names — \
                 or a broken command line.",
                "Set `accel_chip` in printer.cfg's [plr] section to the accelerometer's \
                 section name exactly (e.g. `adxl345` or `adxl345 bed`), without quotes or \
                 backslashes."
                    .to_owned(),
            ),
            P::NoiseFloorMissing => Diagnosis::new(
                "noise_floor_missing",
                Tier::Hard,
                "the ADXL noise floor has not been calibrated",
                "Drag contact detection works by watching for accelerometer energy above \
                 the machine's own baseline vibration. Without a measured baseline there \
                 is no threshold, so the probe would either trigger on nothing or never \
                 trigger at all — and it is descending toward the part while it decides.",
                "Run `PLR_NOISE_TEST` on the printer (it autosaves the `noise_floor_*` \
                 values into the [plr] section), then retry the recovery."
                    .to_owned(),
            ),
            P::NoiseFloorInvalid { value } => Diagnosis::new(
                "noise_floor_invalid",
                Tier::Hard,
                format!(
                    "the calibrated ADXL noise floor {} is not a finite positive number",
                    fmt_num(*value)
                ),
                "The detection threshold is a multiple of this number. Zero, negative or \
                 non-finite makes the threshold meaningless, which means the drag probe's \
                 trigger point is meaningless.",
                "Re-run `PLR_NOISE_TEST` on the printer to re-measure the noise floor. If \
                 it keeps producing a bad value, check the accelerometer wiring and that \
                 the machine is genuinely idle during the test."
                    .to_owned(),
            )
            .measured("noise_floor", *value, "")
            .expected("noise_floor", Some(f64::MIN_POSITIVE), None, ""),
        }
    }
}

// --- PlanWarning -------------------------------------------------------------

impl Diagnose for crate::plan::PlanWarning {
    #[allow(clippy::too_many_lines)] // one arm per variant, by design
    fn diagnosis(&self) -> Diagnosis {
        use crate::plan::PlanWarning as W;
        match self {
            W::AdaptiveMeshNotRestorable => Diagnosis::new(
                "adaptive_mesh_not_restorable",
                Tier::Advisory,
                "an active bed mesh has no saved profile name and will not be restored",
                "Adaptive meshes are generated per print and never named, so there is \
                 nothing to load back. The resume runs without mesh compensation, which \
                 on a bed with real warp shows up as inconsistent first-layer adhesion \
                 for the remainder of the print.",
                "For future prints, either use a saved bed-mesh profile instead of an \
                 adaptive mesh, or accept this warning — the resume itself is unaffected."
                    .to_owned(),
            ),
            W::SkewProfileUnknown => Diagnosis::new(
                "skew_profile_unknown",
                Tier::Advisory,
                "skew correction was active but no profile name was recorded; skew is not \
                 restored",
                "The remainder of the print runs uncorrected, so its geometry will differ \
                 slightly from the part printed before the loss. Dimensionally this shows \
                 as a small step at the resume layer.",
                "Load the skew profile by hand after the resume starts \
                 (`SKEW_PROFILE LOAD=<name>`), or accept the warning."
                    .to_owned(),
            ),
            W::NoBedTarget => Diagnosis::new(
                "no_bed_target",
                Tier::Advisory,
                "no bed temperature was found in the crash record or the file; the bed is \
                 left unheated",
                "A cold bed under a part that is already stuck to it is usually fine — but \
                 if the part had cooled and released, the resume will print onto something \
                 that is no longer held down.",
                "Set the bed temperature by hand before continuing if the part has cooled, \
                 or check that the sliced file sets `M140`/`M190` in its header."
                    .to_owned(),
            ),
            W::ReheatParkComputed { point } => Diagnosis::new(
                "reheat_park_computed",
                Tier::Advisory,
                format!(
                    "no reheat_park_x/y was configured; parking at the computed ({}, {}), \
                     verified clear of the part",
                    fmt_num(point[0]),
                    fmt_num(point[1])
                ),
                "The nozzle reheats to full print temperature somewhere before resuming. \
                 The computed point was checked against the part's bounding box and is \
                 clear of it, so the reheat will not melt printed geometry — but the \
                 choice was plrd's, not the operator's.",
                "Set `reheat_park_x` and `reheat_park_y` in printer.cfg's [plr] section to \
                 a spot you have chosen deliberately (a purge bucket or a bare corner of \
                 the bed)."
                    .to_owned(),
            ),
            W::ReheatParkUnverified { point } => Diagnosis::new(
                "reheat_park_unverified",
                // Confirmable, and deliberately STRICTER than the
                // better-informed `reheat_park_inside_part`: there plrd
                // knows the park lands on the part; here it knows
                // nothing at all. Less knowledge must not buy less
                // friction — that would make ignorance the cheaper path.
                Tier::Confirmable,
                format!(
                    "no reheat_park_x/y was configured and no part geometry is available; \
                     parking at ({}, {}) UNVERIFIED",
                    fmt_num(point[0]),
                    fmt_num(point[1])
                ),
                "Without part geometry the park point could not be checked against \
                 anything. If it happens to sit over printed material, the nozzle will \
                 dwell there at print temperature while it heats and may melt a divot. \
                 plrd is not saying the point is bad — it is saying nobody knows, and you \
                 are the only one who can look.",
                "If you can see that this spot is clear of the part, continue. Otherwise \
                 set `reheat_park_x` and `reheat_park_y` in printer.cfg's [plr] section to \
                 a spot you know is clear, then re-run the recovery."
                    .to_owned(),
            ),
            W::PurgeInsidePart {
                point,
                configured,
                purge_z,
            } => {
                // With an explicit low purge_z this is not "drops filament on
                // the part" but a commanded DESCENT into printed geometry:
                // that earns a confirmation rather than a warning.
                let tier = if purge_z.is_some() {
                    Tier::Confirmable
                } else {
                    Tier::Advisory
                };
                let source = if *configured {
                    "the configured purge point"
                } else {
                    "the purge point (defaulted to the reheat park point)"
                };
                let d = Diagnosis::new(
                    "purge_inside_part",
                    tier,
                    format!(
                        "{source} ({}, {}) lies inside the part's footprint",
                        fmt_num(point[0]),
                        fmt_num(point[1])
                    ),
                    match purge_z {
                        Some(z) => format!(
                            "The recovery file DESCENDS to Z {} at this point before \
                             extruding. Inside the footprint that is not a cosmetic \
                             problem — it is a commanded collision with printed geometry, \
                             at print temperature.",
                            fmt_num(*z)
                        ),
                        None => "The purge deposits filament onto printed geometry. That \
                                 is legitimate when purging onto a prime tower or a \
                                 sacrificial skirt, and a ruined surface otherwise."
                            .to_owned(),
                    },
                    "Set `purge_x` / `purge_y` in printer.cfg's [plr] section to a spot \
                     clear of the part, raise `purge_z` above the part's current top, or \
                     set `purge_enable = False` to skip purging."
                        .to_owned(),
                );
                match purge_z {
                    Some(z) => d.measured("purge_z", *z, "mm"),
                    None => d,
                }
            }
            W::PurgeZBelowResume { purge_z, resume_z } => Diagnosis::new(
                "purge_z_below_resume",
                Tier::Confirmable,
                format!(
                    "purge_z {} mm sits below the resume Z {} mm",
                    fmt_num(*purge_z),
                    fmt_num(*resume_z)
                ),
                "purge_z is below the top of what has already been printed, so the descent \
                 to it may drive the nozzle into the part. This is correct and deliberate \
                 when the purge point is a bare patch of bed — and a collision when it is \
                 not.",
                format!(
                    "If the purge point is over bare bed, continue. Otherwise raise \
                     `purge_z` above {} mm in printer.cfg's [plr] section, or move \
                     `purge_x`/`purge_y` clear of the part.",
                    fmt_num(*resume_z)
                ),
            )
            .measured("purge_z", *purge_z, "mm")
            .expected("purge_z", Some(*resume_z), None, "mm"),
            W::ReheatParkInsidePart { point, configured } => Diagnosis::new(
                "reheat_park_inside_part",
                // Deliberately configured there: the operator already made
                // this call. Computed there: no clear side existed once the
                // travel limits were applied, which nobody chose.
                if *configured {
                    Tier::Advisory
                } else {
                    Tier::Confirmable
                },
                format!(
                    "the reheat park point ({}, {}) lies inside the part's footprint",
                    fmt_num(point[0]),
                    fmt_num(point[1])
                ),
                "The nozzle heats to full print temperature while parked here, directly \
                 over printed geometry. A nozzle dwelling at print temperature against \
                 plastic melts a divot into it.",
                if *configured {
                    "You configured this point explicitly. If that was deliberate \
                     (a purge bucket inside the footprint, say), continue; otherwise move \
                     `reheat_park_x`/`reheat_park_y` in printer.cfg's [plr] section."
                        .to_owned()
                } else {
                    "No side of the part stayed clear once clamped into the machine's \
                     travel limits. Set `reheat_park_x`/`reheat_park_y` in printer.cfg's \
                     [plr] section to a spot you know is clear."
                        .to_owned()
                },
            ),
            W::ResumeNotOnInfill => Diagnosis::new(
                "resume_not_on_infill",
                Tier::Advisory,
                "the resume point is not on infill",
                "Resuming mid-perimeter leaves a visible seam and a locally weaker wall, \
                 because the restart blob and the small positioning error land on the \
                 outside of the part instead of inside it. The print will finish; it will \
                 show where it was interrupted.",
                "Nothing to change now. For future prints, slicers that emit richer \
                 `;TYPE:` annotations give the matcher more infill starts to choose from."
                    .to_owned(),
            ),
            W::UnrestorableFan { name } => Diagnosis::new(
                "unrestorable_fan",
                Tier::Advisory,
                format!("fan {name:?} has an unrecognized name shape and is not restored"),
                "That fan comes back at whatever its power-on default is instead of the \
                 speed it was running at. For a part-cooling fan on a bridging or \
                 overhang-heavy section, that difference is visible in the finished part.",
                format!(
                    "Set {name:?} by hand after the resume starts (`SET_FAN_SPEED`), or \
                     rename it to a standard Klipper fan section name."
                ),
            ),
            W::NoiseFloorSpeedMismatch {
                calibrated_at,
                drag_speed,
            } => Diagnosis::new(
                "noise_floor_speed_mismatch",
                Tier::Confirmable,
                format!(
                    "the ADXL noise floor was calibrated at {} mm/s but drag_speed is now \
                     {} mm/s (more than 20% apart)",
                    fmt_num(*calibrated_at),
                    fmt_num(*drag_speed)
                ),
                "The noise floor is speed-specific: faster passes excite more baseline \
                 vibration. A floor measured at a different speed makes the contact \
                 threshold either too sensitive (triggering in clear air, giving a Z \
                 reference that is too high) or too dull (missing the contact and \
                 continuing to descend). The sensitivity knob usually absorbs a difference \
                 this size, which is why this asks rather than refuses.",
                format!(
                    "Re-run `PLR_NOISE_TEST` at the current drag speed, or set \
                     `drag_speed = {}` in printer.cfg's [plr] section to match the \
                     calibration.",
                    fmt_num(*calibrated_at)
                ),
            )
            .measured("drag_speed", *drag_speed, "mm/s")
            .expected(
                "drag_speed",
                Some(calibrated_at * 0.8),
                Some(calibrated_at * 1.2),
                "mm/s",
            ),
            W::DragTempBelowFloor {
                drag_nozzle_temp,
                floor,
            } => Diagnosis::new(
                "drag_temp_below_floor",
                // Confirmable, not Hard: trace the consequence. A sub-floor
                // target makes the drag path's M109 wait for a cooldown that
                // may never converge, and the executor's existing 15-minute
                // step timeout then aborts — BEFORE the shifted-frame
                // declare, so the abort is clean and the frame stays valid.
                // The cost is wasted time, not a collision or an unknowable
                // machine state, and that is exactly the profile this tier
                // exists for: explain it and offer the button.
                Tier::Confirmable,
                format!(
                    "drag_nozzle_temp {} C is below the {} C floor",
                    fmt_num(*drag_nozzle_temp),
                    fmt_num(*floor)
                ),
                "A nonzero drag temperature makes the plan WAIT for the nozzle to settle, \
                 and on a PID hotend that includes waiting to COOL. On an enclosed or \
                 heated-chamber printer a target at or below chamber ambient may never be \
                 reached at all, burning the full 15-minute step timeout on every attempt. \
                 On an open-frame machine in a cold room it is perfectly reachable — which \
                 is why this asks rather than refuses. The wait happens before the Z frame \
                 is declared, so if it does time out the abort is clean and you can simply \
                 retry.",
                format!(
                    "If this machine really can reach {} C, continue. Otherwise raise \
                     `drag_nozzle_temp` to at least {} C in printer.cfg's [plr] section, or \
                     set it to exactly 0 for a deliberate cold drag (no heating and no wait \
                     at all).",
                    fmt_num(*drag_nozzle_temp),
                    fmt_num(*floor)
                ),
            )
            .measured("drag_nozzle_temp", *drag_nozzle_temp, "C")
            .expected("drag_nozzle_temp", Some(*floor), None, "C"),
            W::AccelEntryNotAppliedToFile { accel_entry } => Diagnosis::new(
                "accel_entry_not_applied_to_file",
                Tier::Advisory,
                format!(
                    "accel_entry ({} mm/s^2) is not applied to the recovery file's entry \
                     moves: the machine's own max_accel is unknown",
                    fmt_num(*accel_entry)
                ),
                "The recovery file has no runtime machinery, so a clamp written into it must \
                 name the value to restore afterwards as a literal. Without the machine's \
                 configured max_accel there is nothing to restore TO, and a clamp with no \
                 restore would leave the printer at the recovery acceleration for the whole \
                 remainder of the print. Skipping the clamp is the lesser harm; the \
                 plan-level moves still honour accel_entry.",
                "Adopt the [plr] section in printer.cfg so plrd reads the live config \
                 (which carries [printer] max_accel). On the legacy /etc/plrd.conf \
                 [machine] path this key cannot reach the generated file."
                    .to_owned(),
            )
            .measured("accel_entry", *accel_entry, "mm/s^2"),
            W::UnsafeOverrideActive { key, permitted } => Diagnosis::new(
                "unsafe_override_active",
                Tier::Advisory,
                format!("the UNSAFE override `{key}` is set, permitting `{permitted}`"),
                "A refusal that exists to prevent physical damage has been switched off in \
                 printer.cfg. That was a deliberate edit made while nothing was at stake, \
                 which is exactly how this escape hatch is meant to be used — but it is \
                 still in force right now, during a recovery, and it is worth knowing that \
                 before the nozzle moves.",
                format!(
                    "If this is no longer wanted, remove `{key}` from printer.cfg's [plr] \
                     section and restart klippy. The recovery proceeds either way."
                ),
            ),
            W::AccelProbeIgnoredOnTouchPath { accel_probe } => Diagnosis::new(
                "accel_probe_ignored_on_touch_path",
                Tier::Advisory,
                format!(
                    "accel_probe ({} mm/s^2) is ignored on the consensus-touch path",
                    fmt_num(*accel_probe)
                ),
                "On a Tap or load-cell machine the contact acceleration is owned by \
                 `touch_accel`, which the plan clamps with SET_VELOCITY_LIMIT around the \
                 touch and restores afterwards. Honouring `accel_probe` as well would mean \
                 two settings fighting over the same number during the one motion that \
                 must not be over-driven.",
                format!(
                    "Set `touch_accel` instead of `accel_probe` in printer.cfg's [plr] \
                     section on this machine (`accel_probe` applies to the ADXL drag and \
                     legacy single-PROBE paths). The plan is running at touch_accel, not \
                     {} mm/s^2.",
                    fmt_num(*accel_probe)
                ),
            )
            .measured("accel_probe", *accel_probe, "mm/s^2"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Diagnose, Diagnosis, Tier};
    use crate::error::RecoveryError;

    #[test]
    fn tier_tags_and_stop_semantics_are_stable() {
        assert_eq!(Tier::Advisory.tag(), "advisory");
        assert_eq!(Tier::Confirmable.tag(), "confirmable");
        assert_eq!(Tier::Hard.tag(), "hard");
        assert!(!Tier::Advisory.stops_execution());
        assert!(Tier::Confirmable.stops_execution());
        assert!(Tier::Hard.stops_execution());
    }

    #[test]
    fn one_line_and_full_render_every_part() {
        let d = RecoveryError::PurgeZBelowBed { purge_z: -0.5 }.diagnosis();
        let one = d.one_line();
        assert!(one.contains("[hard]"), "{one}");
        assert!(one.contains("purge_z_below_bed"), "{one}");
        assert!(one.contains("-0.5"), "{one}");
        let full = d.full();
        assert!(full.contains("REFUSED"), "{full}");
        assert!(full.contains("why:"), "{full}");
        assert!(full.contains("measured:"), "{full}");
        assert!(full.contains("expected:"), "{full}");
        assert!(full.contains("fix:"), "{full}");
        assert!(full.contains(super::UNSAFE_PURGE_Z_BELOW_BED), "{full}");
    }

    #[test]
    fn a_hard_diagnosis_without_an_override_says_so() {
        let d = RecoveryError::NoZSpan.diagnosis();
        assert_eq!(d.override_key, None);
        assert!(d.full().contains("override: NONE"), "{}", d.full());
    }

    #[test]
    fn a_confirmable_diagnosis_offers_continue() {
        let d = crate::plan::PlanWarning::PurgeZBelowResume {
            purge_z: 0.1,
            resume_z: 0.6,
        }
        .diagnosis();
        assert_eq!(d.tier, Tier::Confirmable);
        assert!(d.full().contains("STOPPED (confirmable)"), "{}", d.full());
        assert!(d.full().contains("answer `continue`"), "{}", d.full());
    }

    #[test]
    fn an_advisory_diagnosis_offers_no_override_line() {
        let d = crate::plan::PlanWarning::ResumeNotOnInfill.diagnosis();
        assert_eq!(d.tier, Tier::Advisory);
        assert!(!d.full().contains("override:"), "{}", d.full());
        assert!(d.full().contains("WARNING"), "{}", d.full());
    }

    #[test]
    fn diagnoses_serialize_with_the_frozen_field_names() {
        let d = RecoveryError::PurgeZBelowBed { purge_z: -1.0 }.diagnosis();
        let v = serde_json::to_value(&d).unwrap();
        for key in [
            "code",
            "tier",
            "what",
            "why",
            "suggested_fix",
            "measured",
            "expected",
            "override_key",
        ] {
            assert!(v.get(key).is_some(), "missing {key} in {v}");
        }
        assert_eq!(v["tier"], serde_json::json!("hard"));
        assert_eq!(v["measured"]["quantity"], serde_json::json!("purge_z"));
        assert_eq!(v["measured"]["unit"], serde_json::json!("mm"));
        assert_eq!(v["expected"]["min"], serde_json::json!(0.0));
        assert_eq!(v["expected"]["max"], serde_json::Value::Null);
    }

    #[test]
    fn unbounded_expected_bands_render() {
        let d = Diagnosis::new("x", Tier::Advisory, "w", "y", "f").expected("q", None, None, "mm");
        assert!(d.full().contains("(unbounded)"), "{}", d.full());
        let d =
            Diagnosis::new("x", Tier::Advisory, "w", "y", "f").expected("q", None, Some(2.0), "");
        assert!(d.full().contains("<= 2"), "{}", d.full());
        let d =
            Diagnosis::new("x", Tier::Advisory, "w", "y", "f").expected("q", Some(1.0), None, "");
        assert!(d.full().contains(">= 1"), "{}", d.full());
    }
}
