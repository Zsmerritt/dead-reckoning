//! Machine configuration sourced from Klipper's `[plr]` printer.cfg
//! section, via the `configfile` status object.
//!
//! # Where the values come from
//!
//! Klipper's `configfile` status object (`klippy/configfile.py`,
//! `PrinterConfig.get_status`) exposes:
//!
//! * `settings` — `ConfigValidate.get_status`: every option every
//!   module *accessed* while loading the config, as
//!   `{section: {option: typed_value}}` with section/option names
//!   lowercased (`ConfigValidate._build_status_settings` builds it from
//!   the access-tracking map). Values carry the parsed type
//!   (`getfloat` → number, `getboolean` → bool, `get` → string), and —
//!   crucially — the autosave (`SAVE_CONFIG`) block is merged into the
//!   file config before access tracking starts
//!   (`ConfigAutoSave.load_main_config`), so autosaved options such as
//!   `[plr] self_locking_z` appear here as ordinary settings after a
//!   restart.
//! * `config` — the raw, file-only, **string-valued** file config as
//!   `{section: {option: value}}`: exactly the options the operator
//!   wrote (autosave block merged in), with NO default-injected keys and
//!   NO access filtering. Used for the **calibration fingerprint** and
//!   nothing else (see the fingerprinting section below).
//!
//! Machine ASSEMBLY (Z steppers, probe `z_offset`, axis limits, …) reads
//! `settings` — it needs the typed, default-resolved view. The
//! calibration FINGERPRINT reads `config` — it must hash the same
//! file-only key-set the plugin hashes on the Python side
//! (`config.get_prefix_options`, file options only). These are the two
//! distinct inputs; conflating them was a real bug (see below).
//!
//! The daemon queries `configfile` (plus the plugin's `plr` status
//! object) over the Klipper API socket at recover time, so the machine
//! snapshot is **the running config**, every run — which is exactly why
//! the crc32c config-hash blessing of the legacy `/etc/plrd.conf`
//! `[machine]` path is unnecessary in `[plr]` mode (see
//! [`LIVE_CONFIG_HASH`]).
//!
//! # Defense in depth
//!
//! The plugin's `PLR_SETUP` shows the operator the same derivations
//! (primary-MCU Z, empty `activate_gcode`, single probe); plrd
//! re-derives every one of them independently from the settings here.
//! A plugin bug cannot bless a machine the daemon would refuse.
//!
//! # Calibration fingerprint: why `config`, not `settings`
//!
//! The plugin stamps each calibration value-group with a CRC-32 over the
//! calibration-relevant config slice, computed on the Python side from
//! the raw FILE options (`ConfigWrapper.get_prefix_options("")`,
//! klippy/configfile.py — file-written options only). plrd re-derives
//! the identical hash here — but it MUST hash `config`, not `settings`:
//! `settings` is the access-tracked, **default-resolved** view, so it
//! carries keys the operator never wrote (e.g. `[adxl345] axes_map` /
//! `rate` defaulted by klippy/extras/adxl345.py, and `[probe]` defaults),
//! a superset of the file. Hashing `settings` would diverge from the
//! plugin's file-only hash on essentially every real machine —
//! `CalTier::Invalid`, the noise floor nulled, and recovery spuriously
//! refused on a correctly-calibrated printer. Hashing `config` feeds both
//! sides the identical file-only, string-valued key-set. Because the
//! inputs are now identical, the value normalization
//! (`normalize_value`) is only defensive robustness (both sides already
//! see the same strings), documented at its definition.

use std::collections::BTreeMap;

use plr_recovery::{MachineConfig, PlanConfig, ProbeConfig, ProbeKind, ResumePolicy, ZStepper};
use serde_json::{Map, Value};

/// Sentinel used for both `config_hash` and `validated_config_hash` in
/// `[plr]` mode: the machine snapshot is read from the live config on
/// every run, so the change-detection check is satisfied by
/// construction and the equality check passes trivially. The legacy
/// `[machine]` path keeps its real crc32c blessing.
pub const LIVE_CONFIG_HASH: &str = "live:[plr]";

/// The name of the primary MCU in Klipper (`[mcu]`).
const PRIMARY_MCU: &str = "mcu";

/// The prefix the plugin's `PLR_NOISE_TEST` autosaves the whole
/// noise-floor calibration group under (`klippy_plugin/plr/noise_test.py`).
/// Parsing sweeps by prefix rather than by name because the concrete
/// measurement names are the plugin's to choose — see
/// [`NOISE_FLOOR_METADATA_KEYS`] for the one thing the sweep must know.
const NOISE_FLOOR_PREFIX: &str = "noise_floor_";

/// Drag speed the noise floor was measured at, mm/s — METADATA.
const NOISE_FLOOR_SPEED: &str = "noise_floor_speed";

/// Nozzle/chamber temperature the noise floor was measured at, °C —
/// METADATA.
const NOISE_FLOOR_TEMP: &str = "noise_floor_temp";

/// Name of the klippy sensor object the temperature covariate reads —
/// METADATA, and a STRING.
const NOISE_FLOOR_TEMP_SENSOR: &str = "noise_floor_temp_sensor";

/// The `noise_floor_*` options that describe a noise-floor calibration
/// rather than *being* one.
///
/// **This is the one place a `noise_floor_*` key gets classified.** Every
/// other option carrying the [`NOISE_FLOOR_PREFIX`] is treated as a
/// MEASUREMENT: swept by prefix, required to be a number, and entered into
/// [`PlrSettings::noise_floor`] — where a non-empty map is precisely what
/// "this machine has a calibrated noise floor" means
/// ([`PlrSettings::representative_noise_floor`]). Adding a
/// `noise_floor_*` key on the plugin side therefore needs exactly one
/// decision here, and getting it wrong breaks in one of two ways that both
/// showed up in the plugin audit:
///
/// * **Metadata swept in as a measurement** makes an UNCALIBRATED machine
///   look calibrated. `noise_floor_temp` is only the temperature the
///   baseline was captured at (staged by `PLR_NOISE_TEST` only when
///   `noise_floor_temp_sensor` is configured), so a machine carrying it
///   alone had "a noise floor" that was really a thermometer reading —
///   and the `NoiseFloorMissing` refusal that would have told the
///   operator to run `PLR_NOISE_TEST` never fired.
/// * **A non-numeric option swept in at all** refuses the whole `[plr]`
///   section. `noise_floor_temp_sensor` is a klippy sensor *name*
///   (`config.get`, `klippy_plugin/plr/plugin.py`), so the sweep's
///   `as_f64` demand made a DOCUMENTED, operator-facing key
///   (`klippy_plugin/README.md`, `docs/install.md`) fail parsing with
///   `[plr] noise_floor_temp_sensor is not a number` — costing the
///   operator recovery entirely for configuring the temperature
///   covariate.
///
/// Classification, not typing, is the guard: a plausible temperature
/// (40 °C) is a perfectly valid noise-floor *number*, so no range check
/// downstream could have caught the first case. The metadata plrd itself
/// consumes is read by name afterwards ([`NOISE_FLOOR_SPEED`]); the rest
/// belongs to the plugin's drag oracle
/// (`klippy_plugin/plr/drag_probe.py`) and plrd only needs to not choke
/// on it.
const NOISE_FLOOR_METADATA_KEYS: [&str; 3] =
    [NOISE_FLOOR_SPEED, NOISE_FLOOR_TEMP, NOISE_FLOOR_TEMP_SENSOR];

/// The typed `[plr]` section, parsed from `configfile.settings.plr`.
///
/// `probe_method` is required; every other key falls back to the same
/// default the plugin's config parsing uses, so a settings map from an
/// older plugin still reads coherently. Wrong-typed values are hard
/// errors — a `[plr]` section that cannot be read faithfully must
/// refuse, never guess.
///
/// The `struct_excessive_bools` allow is deliberate: this is a flat
/// mirror of the operator's `[plr]` section, one field per documented
/// key. Collapsing related booleans into enums would insert a layer of
/// translation between what the operator wrote and what plrd reads —
/// exactly the seam a config bug hides in.
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::struct_excessive_bools)]
pub struct PlrSettings {
    /// `tap`, `load_cell`, or `adxl_drag`.
    pub probe_method: String,
    /// Accelerometer chip for `adxl_drag`; may be empty.
    pub accel_chip: String,
    /// The plugin's view of the WAL directory (informational here; the
    /// daemon's own recorder path comes from /etc/plrd.conf).
    pub wal_dir: String,
    /// The control-socket path the plugin connects to. Must match the
    /// daemon's `control_socket` in /etc/plrd.conf.
    pub control_socket: String,
    /// Probe speed, mm/s (continuous descent methods).
    pub probe_speed: f64,
    /// Envelope margin, mm.
    pub envelope_margin: f64,
    /// Sag allowance, mm.
    pub sag_allowance: f64,
    /// Drag pass XY speed, mm/s.
    pub drag_speed: f64,
    /// Drag staircase Z decrement, mm.
    pub drag_z_step: f64,
    /// Drag contact threshold as a multiple of the noise floor.
    pub drag_sensitivity: f64,
    /// Consensus touch: number of agreeing samples (`PLR_TOUCH SAMPLES`).
    pub touch_samples: f64,
    /// Consensus touch: acceptable sample spread, mm (hard cap 0.015 in
    /// [`PlanConfig::validate`]).
    pub touch_sample_range: f64,
    /// Consensus touch: retract distance between/after touches, mm.
    pub touch_retract: f64,
    /// Consensus touch: `max_accel` clamp around the touch, mm/s².
    pub touch_accel: f64,
    /// Contact-selection exclusion radius around the crash point, mm.
    pub exclusion_radius: f64,
    /// Entry-move feedrate, mm/min.
    pub entry_feedrate: f64,
    /// FROZEN `max_probe_nozzle_temp` — hard ceiling for contact ops, °C.
    pub max_probe_nozzle_temp: f64,
    /// FROZEN `reheat_park_x` — nozzle park X while reheating (optional).
    pub reheat_park_x: Option<f64>,
    /// FROZEN `reheat_park_y` — nozzle park Y while reheating (optional).
    pub reheat_park_y: Option<f64>,
    /// FROZEN `reheat_park_delta_z` — Z lift above current Z for parking.
    pub reheat_park_delta_z: f64,
    /// FROZEN `pre_home_z_lift` — Z lift before XY homing, mm.
    pub pre_home_z_lift: f64,
    /// FROZEN `purge_enable` — whether the recovery file purges.
    pub purge_enable: bool,
    /// FROZEN `purge_amount` — built-in purge extrusion length, mm.
    pub purge_amount: f64,
    /// FROZEN `purge_macro` — optional macro that OWNS the purge.
    pub purge_macro: Option<String>,
    /// FROZEN `purge_x` — built-in purge X, mm (default: park point).
    pub purge_x: Option<f64>,
    /// FROZEN `purge_y` — built-in purge Y, mm (default: park point).
    pub purge_y: Option<f64>,
    /// FROZEN `purge_z` — built-in purge absolute Z, mm (default: park Z).
    pub purge_z: Option<f64>,
    /// FROZEN `purge_speed` — built-in purge extrusion feedrate, mm/min.
    pub purge_speed: f64,
    /// FROZEN `purge_retract` — retract after the built-in purge, mm.
    pub purge_retract: f64,
    /// FROZEN `clean_nozzle_macro` — clean-nozzle macro name.
    pub clean_nozzle_macro: String,
    /// FROZEN `drag_nozzle_temp` — nozzle temperature the ADXL drag path
    /// heats to AND HOLDS for, °C. `0` opts out (cold drag, no wait).
    pub drag_nozzle_temp: f64,
    /// `recovery_accel` — one `max_accel` for the whole recovery, mm/s²
    /// (`None` leaves the machine's own value alone).
    pub recovery_accel: Option<f64>,
    /// `accel_home` — `max_accel` for the XY homing step, mm/s².
    pub accel_home: Option<f64>,
    /// `accel_travel` — `max_accel` for the probe-approach travel, mm/s².
    pub accel_travel: Option<f64>,
    /// `accel_probe` — `max_accel` for the contact step on the drag /
    /// legacy single-`PROBE` paths, mm/s² (ignored on the consensus-touch
    /// path, where `touch_accel` owns it).
    pub accel_probe: Option<f64>,
    /// `accel_entry` — `max_accel` for the moves made close to printed
    /// geometry (Z-confirm standoff, lift-and-park), mm/s².
    pub accel_entry: Option<f64>,
    /// `confirm_z_before_resume` — pause after the true-Z declaration and
    /// ask the operator to confirm the believed Z (default `false`).
    pub confirm_z_before_resume: bool,
    /// `debug_confirm_each_step` — pause before every step (default
    /// `false`).
    pub debug_confirm_each_step: bool,
    /// `UNSAFE_allow_purge_z_below_bed` — permits a `purge_z` below the
    /// bed surface (default `false`).
    pub unsafe_allow_purge_z_below_bed: bool,
    /// `confirm_timeout_s` — how long a confirm-point waits for an
    /// answer before aborting cleanly, seconds. `None` uses the daemon
    /// default.
    pub confirm_timeout_s: Option<f64>,
    /// `gcode_barrier_timeout_s` — how long execution waits for
    /// Klipper's g-code mutex before refusing, seconds. `None` uses the
    /// daemon default.
    pub gcode_barrier_timeout_s: Option<f64>,
    /// `resume_candidate_policy` — `first`|`mid`|`last`|`ask` (default
    /// `ask`): which stop in an ambiguous candidate set becomes the resume
    /// point, and whether the interactive preview runs. Parsed with the
    /// same explicit-choice check as [`Self::probe_method`] — an unknown
    /// spelling refuses at parse time rather than being guessed.
    pub resume_candidate_policy: ResumePolicy,
    /// `preview_standoff` — the XY-hover plane's standoff above the highest
    /// preview stop, mm. `None` (absent or wrong-typed, tolerant) derives
    /// it from `entry_hop`. Range-checked (non-negative) by
    /// [`PlanConfig::validate`], never clamped here.
    pub preview_standoff: Option<f64>,
    /// `preview_nozzle_temp` — the nozzle target commanded on preview entry
    /// so it cannot ooze during deliberation, °C. `None` (absent or
    /// wrong-typed, tolerant) derives it from the plan's probe hold
    /// temperature. Range-checked by [`PlanConfig::validate`].
    pub preview_nozzle_temp: Option<f64>,
    /// Operator attestation autosaved by the plugin's `PLR_SETUP`.
    pub self_locking_z: bool,
    /// Autosaved probe resolution, mm; `None` before first calibration.
    pub probe_resolution: Option<f64>,
    /// Every autosaved `noise_floor_*` **measurement** (the plugin's
    /// `PLR_NOISE_TEST` writes `noise_floor_rms` /
    /// `noise_floor_still_rms` / `noise_floor_peak`). Non-empty means
    /// the noise floor is calibrated.
    ///
    /// The `noise_floor_*` options that are metadata *about* the
    /// measurements — [`NOISE_FLOOR_METADATA_KEYS`] — are excluded by
    /// name and never appear here: metadata must never make an
    /// uncalibrated machine look calibrated.
    pub noise_floor: BTreeMap<String, f64>,
    /// OPTIONAL: drag speed the noise floor was measured at, mm/s
    /// ([`NOISE_FLOOR_SPEED`] — metadata, staged by the plugin's
    /// `PLR_NOISE_TEST`; absent — a calibration from before the key
    /// existed — means no speed check, tolerant back-compat).
    ///
    /// The other two metadata keys are not mirrored into a field because
    /// plrd consumes neither: `noise_floor_temp` /
    /// `noise_floor_temp_sensor` feed the plugin's own drag oracle
    /// (`klippy_plugin/plr/drag_probe.py`), which applies the
    /// temperature covariate on the Klipper side.
    pub noise_floor_speed: Option<f64>,
    /// Per-group calibration fingerprint stamp for the noise-floor group
    /// (`cal_fingerprint_noise_floor`, staged by `PLR_NOISE_TEST` /
    /// `PLR_DRAG_CALIBRATE` — `klippy_plugin/plr/calibration_meta.py`).
    /// `None` for a pre-stamping (legacy) calibration.
    pub cal_fingerprint_noise_floor: Option<String>,
    /// Per-group calibration fingerprint stamp for the `probe_resolution`
    /// group (`cal_fingerprint_probe_resolution`, staged by
    /// `PLR_PROBE_TEST`). `None` for a legacy calibration.
    pub cal_fingerprint_probe_resolution: Option<String>,
    /// The plugin version the calibration was staged under
    /// (`cal_plugin_version`); recorded for forensics, not gated here.
    pub cal_plugin_version: Option<String>,
    /// The Klipper version the calibration was staged under
    /// (`cal_klipper_version`); recorded for forensics, not gated here.
    pub cal_klipper_version: Option<String>,
}

/// Reads a required-or-defaulted f64 option.
fn opt_f64(section: &Map<String, Value>, key: &str, default: f64) -> Result<f64, String> {
    match section.get(key) {
        None => Ok(default),
        Some(v) => v
            .as_f64()
            .ok_or_else(|| format!("[plr] {key} is not a number: {v}")),
    }
}

/// Reads an OPTIONAL number STRICTLY: absent is `None`, wrong-typed is a
/// hard error.
///
/// The strict direction is right for a calibration value plrd actually
/// CONSUMES (`noise_floor_speed` feeds the speed-mismatch warning): a
/// value plrd cannot read faithfully must refuse, never be silently
/// dropped as if the calibration had never recorded it. It cannot fire on
/// a booting printer anyway — the plugin parses the same option with
/// `getfloat` (`klippy_plugin/plr/plugin.py`), so a non-numeric value
/// never leaves klippy.
fn strict_number(section: &Map<String, Value>, key: &str) -> Result<Option<f64>, String> {
    match section.get(key) {
        None => Ok(None),
        Some(v) => v
            .as_f64()
            .map(Some)
            .ok_or_else(|| format!("[plr] {key} is not a number: {v}")),
    }
}

/// Reads an OPTIONAL string stamp tolerantly: absent OR wrong-typed both
/// yield `None` (never an error). A calibration stamp is metadata — a
/// malformed one degrades the calibration to legacy/unstamped, it must not
/// make an otherwise-valid `[plr]` section refuse.
fn opt_stamp(section: &Map<String, Value>, key: &str) -> Option<String> {
    section.get(key).and_then(Value::as_str).map(str::to_owned)
}

/// Reads a defaulted boolean option (tolerant: absent → default).
fn opt_bool(section: &Map<String, Value>, key: &str, default: bool) -> Result<bool, String> {
    match section.get(key) {
        None => Ok(default),
        Some(v) => v
            .as_bool()
            .ok_or_else(|| format!("[plr] {key} is not a boolean: {v}")),
    }
}

/// Reads an OPTIONAL float tolerantly: absent OR wrong-typed both yield
/// `None` (these FROZEN keys are parsed tolerantly).
fn opt_opt_f64(section: &Map<String, Value>, key: &str) -> Option<f64> {
    section.get(key).and_then(Value::as_f64)
}

/// Reads an `UNSAFE_`-prefixed boolean, tolerating either case.
///
/// # Why both spellings are accepted — and why the lowercase one is what
/// actually arrives
///
/// The documented spelling is `UNSAFE_allow_purge_z_below_bed`, because
/// the screaming prefix is the whole point of the naming. What reaches
/// plrd is `unsafe_allow_purge_z_below_bed`, whatever the operator typed:
/// klippy parses printer.cfg with a `configparser.RawConfigParser` and
/// does NOT override `optionxform` (`klippy/configfile.py:170-176`), so
/// option names are LOWERCASED on read; and `access_tracking` — the map
/// `configfile.settings` is built from — is keyed lowercased regardless
/// (`klippy/configfile.py:34,47`).
/// The lowercase fallback below is therefore not belt-and-braces, it is
/// the branch that carries every real machine; the exact-case lookup is
/// the belt. Pinned from the Klipper side by
/// `klippy_plugin/tests/test_daemon_keys.py::
/// test_unsafe_override_reaches_the_daemon_in_any_casing`, which asserts
/// the documented, lowercase, and mangled-case spellings all land as the
/// same lowercase option.
///
/// The plugin declares this key (like every other option plrd consumes),
/// so a `[plr]` section that sets it boots and the value arrives:
/// klippy's `check_unused` refuses to start only for an option NO module
/// claimed, and that whole class of gap is now guarded on the plugin side
/// by `test_klippy_accepts_every_key_plrd_consumes`, which derives its
/// expected key set from this very file.
///
/// Tolerant like the other FROZEN keys: absent OR wrong-typed is
/// `false`. That direction is deliberate — a malformed override must
/// fail CLOSED (the refusal stays in force), never open.
fn unsafe_flag(section: &Map<String, Value>, key: &str) -> bool {
    let lower = key.to_ascii_lowercase();
    section
        .get(key)
        .or_else(|| section.get(lower.as_str()))
        .and_then(Value::as_bool)
        == Some(true)
}

/// Maps the `resume_candidate_policy` string to its [`ResumePolicy`] with
/// the same explicit-choice discipline as `probe_method`: a known spelling
/// → its variant, anything else → a hard refusal naming the four legal
/// values. A silent guess would let a typo
/// (`resume_candidate_policy = lastt`) resolve to the interactive default
/// on a headless printer — exactly the automatic-vs-manual divergence this
/// key exists to remove.
///
/// The value itself is read by `opt_str(plr, "resume_candidate_policy",
/// "ask")` at the call site so the boot-guard test's key extractor
/// (`test_daemon_keys.py`, which greps for `opt_str(plr, "…"`) sees the
/// key — routing the read through a bespoke helper would hide it and
/// defeat the guard.
fn resume_policy_from(s: &str) -> Result<ResumePolicy, String> {
    match s {
        "first" => Ok(ResumePolicy::First),
        "mid" => Ok(ResumePolicy::Mid),
        "last" => Ok(ResumePolicy::Last),
        "ask" => Ok(ResumePolicy::Ask),
        other => Err(format!(
            "[plr] resume_candidate_policy {other:?} is not first, mid, last, or ask"
        )),
    }
}

/// Reads a defaulted string option.
fn opt_str(section: &Map<String, Value>, key: &str, default: &str) -> Result<String, String> {
    match section.get(key) {
        None => Ok(default.to_owned()),
        Some(v) => v
            .as_str()
            .map(str::to_owned)
            .ok_or_else(|| format!("[plr] {key} is not a string: {v}")),
    }
}

impl PlrSettings {
    /// Parses `configfile.settings.plr`. See the type docs for the
    /// required/optional split.
    pub fn parse(plr: &Map<String, Value>) -> Result<Self, String> {
        let probe_method = plr
            .get("probe_method")
            .ok_or_else(|| "[plr] probe_method is missing".to_owned())?
            .as_str()
            .ok_or_else(|| "[plr] probe_method is not a string".to_owned())?
            .to_owned();
        if !matches!(probe_method.as_str(), "tap" | "load_cell" | "adxl_drag") {
            return Err(format!(
                "[plr] probe_method {probe_method:?} is not tap, load_cell, or adxl_drag"
            ));
        }
        // Sweep the noise-floor MEASUREMENTS by prefix, skipping exactly
        // the keys classified as metadata (NOISE_FLOOR_METADATA_KEYS —
        // read there for why both halves of this matter). Anything else
        // carrying the prefix is a measurement and must be a number: an
        // unreadable measurement cannot be quietly dropped, because a
        // machine whose only remaining measurement was dropped would then
        // look uncalibrated in a way no message explains.
        let mut noise_floor = BTreeMap::new();
        for (key, value) in plr {
            if !key.starts_with(NOISE_FLOOR_PREFIX)
                || NOISE_FLOOR_METADATA_KEYS.contains(&key.as_str())
            {
                continue;
            }
            let number = value
                .as_f64()
                .ok_or_else(|| format!("[plr] {key} is not a number: {value}"))?;
            noise_floor.insert(key.clone(), number);
        }
        // The one metadata key plrd consumes, read by name.
        let noise_floor_speed = strict_number(plr, NOISE_FLOOR_SPEED)?;
        let probe_resolution = match plr.get("probe_resolution") {
            None => None,
            Some(v) => Some(
                v.as_f64()
                    .ok_or_else(|| format!("[plr] probe_resolution is not a number: {v}"))?,
            ),
        };
        let self_locking_z = match plr.get("self_locking_z") {
            None => false,
            Some(v) => v
                .as_bool()
                .ok_or_else(|| format!("[plr] self_locking_z is not a boolean: {v}"))?,
        };
        // Numeric tunables default to the plan builder's defaults so an
        // older plugin (fewer keys) still yields the documented
        // behavior.
        let d = PlanConfig::default();
        Ok(Self {
            probe_method,
            accel_chip: opt_str(plr, "accel_chip", "")?,
            wal_dir: opt_str(plr, "wal_dir", "/var/lib/plrd/wal")?,
            control_socket: opt_str(plr, "control_socket", "/var/lib/plrd/plrd.sock")?,
            probe_speed: opt_f64(plr, "probe_speed", d.probe_speed)?,
            envelope_margin: opt_f64(plr, "envelope_margin", d.margin)?,
            sag_allowance: opt_f64(plr, "sag_allowance", d.sag_allowance)?,
            drag_speed: opt_f64(plr, "drag_speed", d.drag_speed)?,
            drag_z_step: opt_f64(plr, "drag_z_step", d.drag_z_step)?,
            drag_sensitivity: opt_f64(plr, "drag_sensitivity", d.drag_sensitivity)?,
            touch_samples: opt_f64(plr, "touch_samples", d.touch_samples)?,
            touch_sample_range: opt_f64(plr, "touch_sample_range", d.touch_sample_range)?,
            touch_retract: opt_f64(plr, "touch_retract", d.touch_retract)?,
            touch_accel: opt_f64(plr, "touch_accel", d.touch_accel)?,
            exclusion_radius: opt_f64(plr, "exclusion_radius", 5.0)?,
            entry_feedrate: opt_f64(plr, "entry_feedrate", d.entry_feed)?,
            // FROZEN recovery-UX keys — parsed tolerantly (defaults from
            // the plan builder; optional floats absent-or-wrong-typed →
            // None). Out-of-band values are refused later by
            // PlanConfig::validate, not clamped here.
            max_probe_nozzle_temp: opt_f64(plr, "max_probe_nozzle_temp", d.max_probe_nozzle_temp)?,
            reheat_park_x: opt_opt_f64(plr, "reheat_park_x"),
            reheat_park_y: opt_opt_f64(plr, "reheat_park_y"),
            reheat_park_delta_z: opt_f64(plr, "reheat_park_delta_z", d.reheat_park_delta_z)?,
            pre_home_z_lift: opt_f64(plr, "pre_home_z_lift", d.pre_home_z_lift)?,
            purge_enable: opt_bool(plr, "purge_enable", d.purge_enable)?,
            purge_amount: opt_f64(plr, "purge_amount", d.purge_amount)?,
            purge_macro: match plr.get("purge_macro").and_then(Value::as_str) {
                Some(s) if !s.trim().is_empty() => Some(s.trim().to_owned()),
                _ => None,
            },
            purge_x: opt_opt_f64(plr, "purge_x"),
            purge_y: opt_opt_f64(plr, "purge_y"),
            purge_z: opt_opt_f64(plr, "purge_z"),
            purge_speed: opt_f64(plr, "purge_speed", d.purge_speed)?,
            purge_retract: opt_f64(plr, "purge_retract", d.purge_retract)?,
            clean_nozzle_macro: opt_str(plr, "clean_nozzle_macro", &d.clean_nozzle_macro)?,
            drag_nozzle_temp: opt_f64(plr, "drag_nozzle_temp", d.drag_nozzle_temp)?,
            // Acceleration overrides: optional, parsed tolerantly, and
            // range-checked by PlanConfig::validate (which refuses with a
            // typed diagnosis rather than clamping).
            recovery_accel: opt_opt_f64(plr, "recovery_accel"),
            accel_home: opt_opt_f64(plr, "accel_home"),
            accel_travel: opt_opt_f64(plr, "accel_travel"),
            accel_probe: opt_opt_f64(plr, "accel_probe"),
            accel_entry: opt_opt_f64(plr, "accel_entry"),
            confirm_z_before_resume: opt_bool(plr, "confirm_z_before_resume", false)?,
            debug_confirm_each_step: opt_bool(plr, "debug_confirm_each_step", false)?,
            unsafe_allow_purge_z_below_bed: unsafe_flag(
                plr,
                plr_recovery::UNSAFE_PURGE_Z_BELOW_BED,
            ),
            confirm_timeout_s: opt_opt_f64(plr, "confirm_timeout_s"),
            gcode_barrier_timeout_s: opt_opt_f64(plr, "gcode_barrier_timeout_s"),
            // Resume-preview keys. The policy is an explicit-choice enum
            // (refused, not guessed, on an unknown spelling); the two
            // numeric derived-default keys are tolerant Option floats (like
            // the FROZEN recovery-UX keys) with their bands enforced by
            // PlanConfig::validate.
            resume_candidate_policy: resume_policy_from(&opt_str(
                plr,
                "resume_candidate_policy",
                "ask",
            )?)?,
            preview_standoff: opt_opt_f64(plr, "preview_standoff"),
            preview_nozzle_temp: opt_opt_f64(plr, "preview_nozzle_temp"),
            self_locking_z,
            probe_resolution,
            noise_floor,
            noise_floor_speed,
            // Calibration stamps: parsed tolerantly — a wrong-typed or
            // absent stamp is `None`, never a hard error. A malformed
            // stamp degrades to "unstamped/legacy"; it must not make an
            // otherwise-parseable [plr] section refuse.
            cal_fingerprint_noise_floor: opt_stamp(plr, "cal_fingerprint_noise_floor"),
            cal_fingerprint_probe_resolution: opt_stamp(plr, "cal_fingerprint_probe_resolution"),
            cal_plugin_version: opt_stamp(plr, "cal_plugin_version"),
            cal_klipper_version: opt_stamp(plr, "cal_klipper_version"),
        })
    }

    /// The representative calibrated noise floor, if any:
    /// `noise_floor_rms` when present (the primary calibration value),
    /// else the first `noise_floor_*` key in sorted order. The exact
    /// key set firms up with the plugin's `PLR_NOISE_TEST`; validation
    /// only needs "calibrated, finite, positive".
    ///
    /// That "first key in sorted order" fallback is why
    /// [`NOISE_FLOOR_METADATA_KEYS`] has to be exact: any metadata that
    /// reached the map could be handed out here as the noise floor.
    #[must_use]
    pub fn representative_noise_floor(&self) -> Option<f64> {
        self.noise_floor
            .get("noise_floor_rms")
            .or_else(|| self.noise_floor.values().next())
            .copied()
    }

    /// The plan-builder tunables carried by `[plr]`. Out-of-band values
    /// are *not* clamped here: `plan_recovery` validates the config and
    /// refuses, which is the honest failure.
    #[must_use]
    pub fn plan_config(&self) -> PlanConfig {
        PlanConfig {
            probe_speed: self.probe_speed,
            margin: self.envelope_margin,
            sag_allowance: self.sag_allowance,
            entry_feed: self.entry_feedrate,
            drag_speed: self.drag_speed,
            drag_z_step: self.drag_z_step,
            drag_sensitivity: self.drag_sensitivity,
            touch_samples: self.touch_samples,
            touch_sample_range: self.touch_sample_range,
            touch_retract: self.touch_retract,
            touch_accel: self.touch_accel,
            // FROZEN recovery-UX keys.
            max_probe_nozzle_temp: self.max_probe_nozzle_temp,
            reheat_park_x: self.reheat_park_x,
            reheat_park_y: self.reheat_park_y,
            reheat_park_delta_z: self.reheat_park_delta_z,
            pre_home_z_lift: self.pre_home_z_lift,
            purge_enable: self.purge_enable,
            purge_amount: self.purge_amount,
            purge_macro: self.purge_macro.clone(),
            purge_x: self.purge_x,
            purge_y: self.purge_y,
            purge_z: self.purge_z,
            purge_speed: self.purge_speed,
            purge_retract: self.purge_retract,
            clean_nozzle_macro: self.clean_nozzle_macro.clone(),
            drag_nozzle_temp: self.drag_nozzle_temp,
            recovery_accel: self.recovery_accel,
            accel_home: self.accel_home,
            accel_travel: self.accel_travel,
            accel_probe: self.accel_probe,
            accel_entry: self.accel_entry,
            confirm_z_before_resume: self.confirm_z_before_resume,
            debug_confirm_each_step: self.debug_confirm_each_step,
            unsafe_allow_purge_z_below_bed: self.unsafe_allow_purge_z_below_bed,
            confirm_timeout_s: self.confirm_timeout_s,
            gcode_barrier_timeout_s: self.gcode_barrier_timeout_s,
            resume_candidate_policy: self.resume_candidate_policy,
            preview_standoff: self.preview_standoff,
            preview_nozzle_temp: self.preview_nozzle_temp,
            // [plr] mode: the plugin (and its PLR_TOUCH command) is
            // present, so the consensus touch is used.
            legacy_single_probe: false,
            ..PlanConfig::default()
        }
    }
}

/// The set of `[gcode_macro <NAME>]` macro names present in the running
/// config, uppercased. Klippy's config sections are `[gcode_macro NAME]`;
/// `configfile.settings`/`config` lowercase the section name to
/// `gcode_macro name` (`klippy/extras/gcode_macro.py` registers the
/// object under the lowercased section name). plrd uppercases them back so a
/// case-insensitive `macro_present(name)` check matches the operator's
/// `clean_nozzle_macro` / `purge_macro` value.
#[must_use]
pub fn gcode_macro_names(sections: &Map<String, Value>) -> std::collections::BTreeSet<String> {
    sections
        .keys()
        .filter_map(|name| name.strip_prefix("gcode_macro "))
        .map(|n| n.trim().to_ascii_uppercase())
        .collect()
}

// --- calibration fingerprinting --------------------------------------------
//
// A byte-for-byte port of `klippy_plugin/plr/calibration_meta.py`: the plugin
// stamps each persisted calibration value-group with a CRC-32 fingerprint of
// the calibration-relevant config slice, and plrd re-derives the same
// fingerprint here (defense in depth). The Python side reads raw config
// strings; this side reads Klipper's typed `configfile.settings` — the
// numeric normalization below is what makes the two agree. Pinned by the
// shared literal-hash fixtures in the test module (identical hex to the
// python suite).

/// The two independently-fingerprinted calibration value-groups.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CalGroup {
    /// The `noise_floor_*` measurements (and the derived `drag_sensitivity`):
    /// depends on the `stepper_z*` kinematics and the accel-chip section.
    NoiseFloor,
    /// The `probe_resolution` measurement: depends on the `stepper_z*`
    /// kinematics and the active touch-probe section (NOT the accel chip).
    ProbeResolution,
}

/// The three-tier classification of a value-group's stamp (plrd checks the
/// fingerprint only; the plugin additionally checks plugin-version
/// regression at load time).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CalTier {
    /// Stamp present and matches the recomputed fingerprint.
    Valid,
    /// No stamp (a pre-stamping calibration): accepted, cross-check skipped.
    Legacy,
    /// Stamp present but the recomputed fingerprint differs: the value is
    /// treated as absent.
    Invalid,
}

/// CRC-32 (IEEE 802.3, reflected, poly `0xEDB88320`, init/xorout all-ones) —
/// reproduces Python's `zlib.crc32` byte-for-byte. Eight lowercase hex digits.
fn crc32_hex(data: &[u8]) -> String {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    format!("{:08x}", !crc)
}

/// The canonical decimal string for a finite `f64`: integer-valued numbers
/// without a decimal point (`-2.0` -> `"-2"`), others via the shortest
/// round-tripping `Display`. `None` for a non-finite value (caller keeps the
/// original text) — mirrors `calibration_meta._canonical_number`.
fn canonical_number(value: f64) -> Option<String> {
    if !value.is_finite() {
        return None;
    }
    #[allow(clippy::cast_possible_truncation)]
    if value.abs() < 1e15 && value.fract() == 0.0 {
        Some(format!("{}", value as i64))
    } else {
        Some(format!("{value}"))
    }
}

/// Whitespace-collapse then numeric-canonicalize a string value (mirrors the
/// Python side operating on raw config strings).
fn normalize_str(text: &str) -> String {
    let collapsed = text.split_whitespace().collect::<Vec<_>>().join(" ");
    match collapsed.parse::<f64>() {
        Ok(number) => canonical_number(number).unwrap_or(collapsed),
        Err(_) => collapsed,
    }
}

/// Normalize a config value to the canonical text the Python side derives from
/// the raw config string. The fingerprint input is `configfile.config` (raw,
/// string-valued), so in practice every value is already a `String` and the
/// numeric/bool arms are only defensive robustness (harmless: a `String`
/// `"-2"` and a `Number` `-2.0` both canonicalize to `-2`).
fn normalize_value(value: &Value) -> String {
    match value {
        Value::String(text) => normalize_str(text),
        Value::Number(number) => number
            .as_f64()
            .and_then(canonical_number)
            .unwrap_or_else(|| number.to_string()),
        Value::Bool(flag) => (if *flag { "true" } else { "false" }).to_owned(),
        Value::Null => String::new(),
        other => other.to_string(),
    }
}

/// The canonical serialization of the relevant config slice: sorted section
/// names, sorted option keys, normalized values, then a synthetic `[plr]`
/// block with only the selected hardware-selection keys. See
/// `calibration_meta._canonical_string` for the exact grammar.
fn canonical_string(
    sections: &Map<String, Value>,
    section_names: &[String],
    plr_keys: &[&str],
) -> String {
    let mut names: Vec<&String> = section_names.iter().collect();
    names.sort();
    let mut parts: Vec<String> = Vec::new();
    for name in names {
        let Some(body) = sections.get(name.as_str()).and_then(Value::as_object) else {
            continue;
        };
        parts.push(format!("[{name}]"));
        let mut keys: Vec<&String> = body.keys().collect();
        keys.sort();
        for key in keys {
            parts.push(format!("{key}={}", normalize_value(&body[key])));
        }
    }
    parts.push("[plr]".to_owned());
    if let Some(plr) = sections.get("plr").and_then(Value::as_object) {
        let mut selected: Vec<&str> = plr_keys.to_vec();
        selected.sort_unstable();
        for key in selected {
            if let Some(value) = plr.get(key) {
                parts.push(format!("{key}={}", normalize_value(value)));
            }
        }
    }
    parts.join("\n")
}

/// The low-level fingerprint over an explicit section/key selection — the
/// surface the cross-language literal-hash fixtures pin. Test-only: the
/// production path always goes through [`compute_fingerprint`] (group-derived
/// section selection).
#[cfg(test)]
pub(crate) fn fingerprint(
    sections: &Map<String, Value>,
    section_names: &[&str],
    plr_keys: &[&str],
) -> String {
    let names: Vec<String> = section_names
        .iter()
        .map(|name| (*name).to_owned())
        .collect();
    crc32_hex(canonical_string(sections, &names, plr_keys).as_bytes())
}

/// The touch-probe section backing each descending probe method (`None` for
/// `adxl_drag`, which has no touch-probe section).
fn probe_section_for(method: &str) -> Option<&'static str> {
    match method {
        "tap" => Some("probe"),
        "load_cell" => Some("load_cell_probe"),
        _ => None,
    }
}

/// The section names that feed `group`'s fingerprint, derived from the
/// `[plr]` hardware selection. `config` is the raw file-only config map.
fn relevant_section_names(
    config: &Map<String, Value>,
    plr: &PlrSettings,
    group: CalGroup,
) -> Vec<String> {
    let mut names: Vec<String> = config
        .keys()
        .filter(|name| is_z_stepper(name))
        .cloned()
        .collect();
    match group {
        CalGroup::ProbeResolution => {
            if let Some(section) = probe_section_for(&plr.probe_method) {
                names.push(section.to_owned());
            }
        }
        CalGroup::NoiseFloor => {
            if !plr.accel_chip.is_empty() {
                names.push(plr.accel_chip.clone());
            }
        }
    }
    names
}

/// The `[plr]` hardware-selection keys folded into `group`'s fingerprint.
fn plr_keys(group: CalGroup) -> &'static [&'static str] {
    match group {
        CalGroup::ProbeResolution => &["probe_method"],
        CalGroup::NoiseFloor => &["accel_chip", "probe_method"],
    }
}

/// Recompute `group`'s fingerprint from the raw file-only `configfile.config`
/// map.
///
/// The `config` parameter MUST be `configfile.config` (raw, file-only), NEVER
/// `configfile.settings` (which carries default-injected keys the plugin's
/// file-only hash never sees). This is a load-bearing precondition — feeding
/// `settings` would spuriously invalidate correctly-calibrated machines; the
/// module docs and the `fingerprint_uses_config_not_settings` test pin it.
pub(crate) fn compute_fingerprint(
    config: &Map<String, Value>,
    plr: &PlrSettings,
    group: CalGroup,
) -> String {
    crc32_hex(
        canonical_string(
            config,
            &relevant_section_names(config, plr, group),
            plr_keys(group),
        )
        .as_bytes(),
    )
}

/// Classify `group`: `(tier, stored_stamp, recomputed_fingerprint)`. `config`
/// is the raw file-only `configfile.config` map (see [`compute_fingerprint`]).
pub(crate) fn validate_group(
    config: &Map<String, Value>,
    plr: &PlrSettings,
    group: CalGroup,
) -> (CalTier, Option<String>, String) {
    let stored = match group {
        CalGroup::NoiseFloor => plr.cal_fingerprint_noise_floor.clone(),
        CalGroup::ProbeResolution => plr.cal_fingerprint_probe_resolution.clone(),
    };
    let current = compute_fingerprint(config, plr, group);
    let tier = match &stored {
        None => CalTier::Legacy,
        Some(stamp) if *stamp == current => CalTier::Valid,
        Some(_) => CalTier::Invalid,
    };
    (tier, stored, current)
}

/// The effective noise floor after the fingerprint cross-check (ports
/// `calibration_meta.validate_group` + treat-as-absent gating): a stamp
/// mismatch treats the calibrated floor as absent so the existing
/// `NoiseFloorMissing` refusal fires; an absent stamp is a legacy calibration,
/// accepted with a note. plrd checks the fingerprint only — plugin-version
/// regression is the plugin's (authoritative) load-time check.
fn gated_noise_floor(
    config: &Map<String, Value>,
    plr: &PlrSettings,
    notes: &mut Vec<String>,
) -> Option<f64> {
    let representative = plr.representative_noise_floor();
    if representative.is_none() {
        return representative;
    }
    let (tier, stored, current) = validate_group(config, plr, CalGroup::NoiseFloor);
    match tier {
        CalTier::Invalid => {
            notes.push(format!(
                "noise-floor calibration fingerprint mismatch (staged {}, \
                 recomputed {current}) — treating the noise floor as \
                 uncalibrated; re-run PLR_NOISE_TEST",
                stored.as_deref().unwrap_or("<none>")
            ));
            None
        }
        CalTier::Legacy => {
            notes.push(
                "noise-floor calibration predates fingerprint stamping (legacy) \
                 — accepted without a fingerprint cross-check; re-run \
                 PLR_NOISE_TEST to stamp it"
                    .to_owned(),
            );
            representative
        }
        CalTier::Valid => representative,
    }
}

/// `probe_resolution` is not consumed by machine validation, but a stale one
/// is worth flagging so the operator re-runs `PLR_PROBE_TEST`.
fn note_stale_probe_resolution(
    config: &Map<String, Value>,
    plr: &PlrSettings,
    notes: &mut Vec<String>,
) {
    if plr.probe_resolution.is_none() {
        return;
    }
    let (tier, stored, current) = validate_group(config, plr, CalGroup::ProbeResolution);
    if tier == CalTier::Invalid {
        notes.push(format!(
            "probe_resolution calibration fingerprint mismatch (staged {}, \
             recomputed {current}) — ignore the stored probe_resolution; re-run \
             PLR_PROBE_TEST",
            stored.as_deref().unwrap_or("<none>")
        ));
    }
}

/// The MCU a config pin lives on: strip the `!`/`^`/`~`/`*` modifier
/// prefixes, then everything before a `:` names the MCU; a bare pin is
/// on the primary (`klippy/pins.py`, `parse_pin`/`lookup_pin` — pins
/// are written `[chip_name:]pin` with optional modifier prefixes).
fn pin_mcu(pin: &str) -> String {
    let trimmed = pin.trim_start_matches(['!', '^', '~', '*']);
    match trimmed.split_once(':') {
        Some((mcu, _)) => mcu.trim().to_owned(),
        None => PRIMARY_MCU.to_owned(),
    }
}

/// `true` when a section name is a Z stepper: `stepper_z` or
/// `stepper_z<N>`.
fn is_z_stepper(section: &str) -> bool {
    section
        .strip_prefix("stepper_z")
        .is_some_and(|suffix| suffix.is_empty() || suffix.bytes().all(|b| b.is_ascii_digit()))
}

/// A gcode-shaped option (e.g. `activate_gcode`) is "verified no-move"
/// here only when it is empty: plrd does not simulate arbitrary gcode,
/// so anything non-empty must be attested through the legacy path or
/// emptied. Missing counts as empty (Klipper defaults these to "").
fn gcode_option_empty(section: &Map<String, Value>, key: &str) -> bool {
    section
        .get(key)
        .and_then(Value::as_str)
        .is_none_or(|s| s.trim().is_empty())
}

/// Known axis travel limits from the Klipper stepper sections; absent
/// values leave that axis unconstrained ("where known").
fn axis_limits_from(settings: &Map<String, Value>) -> plr_recovery::AxisLimits {
    let section = |name: &str| settings.get(name).and_then(Value::as_object);
    let pair = |name: &str| -> Option<(f64, f64)> {
        let s = section(name)?;
        let lo = s.get("position_min").and_then(Value::as_f64)?;
        let hi = s.get("position_max").and_then(Value::as_f64)?;
        Some((lo, hi))
    };
    plr_recovery::AxisLimits {
        x: pair("stepper_x"),
        y: pair("stepper_y"),
        z_max: section("stepper_z")
            .and_then(|s| s.get("position_max"))
            .and_then(Value::as_f64),
    }
}

/// One probe-ish config section mapped onto [`ProbeConfig`].
fn probe_from_section(section: &Map<String, Value>, kind: ProbeKind) -> ProbeConfig {
    ProbeConfig {
        kind,
        // A missing z_offset is represented as NaN so validation
        // reports ProbeZOffsetNonFinite instead of inventing a zero.
        z_offset: section
            .get("z_offset")
            .and_then(Value::as_f64)
            .unwrap_or(f64::NAN),
        activate_gcode_no_move: gcode_option_empty(section, "activate_gcode"),
        deactivate_gcode_no_move: gcode_option_empty(section, "deactivate_gcode"),
    }
}

/// Assembles the recovery `MachineConfig` from the Klipper config snapshot
/// (the `[plr]` section already parsed). Returns the config plus
/// human-readable assembly notes.
///
/// Two distinct inputs:
/// * `settings` — `configfile.settings` (typed, default-resolved): every
///   machine-ASSEMBLY value (Z steppers, probe `z_offset`, axis limits, …).
/// * `config` — `configfile.config` (raw, file-only): the calibration
///   FINGERPRINT input ONLY. It must be the file-only map so the recomputed
///   fingerprint matches the plugin's file-only Python hash; passing
///   `settings` here would spuriously invalidate a correctly-calibrated
///   machine (module docs).
///
/// Total by design: missing cross-check data becomes the
/// `MachineConfig` shape that *fails validation* (empty probe list,
/// `None` `position_min`, ...) rather than an error here — so the
/// operator sees every problem in one `validate_machine` report.
pub fn machine_from_settings(
    settings: &Map<String, Value>,
    config: &Map<String, Value>,
    plr: &PlrSettings,
    type_annotations_present: bool,
) -> (MachineConfig, Vec<String>) {
    let mut notes = Vec::new();
    let section = |name: &str| settings.get(name).and_then(Value::as_object);

    // Z steppers with their MCUs, derived from each section's step_pin.
    let mut z_steppers: Vec<ZStepper> = Vec::new();
    for (name, value) in settings {
        if !is_z_stepper(name) {
            continue;
        }
        let mcu = value
            .as_object()
            .and_then(|s| s.get("step_pin"))
            .and_then(Value::as_str)
            .map_or_else(
                || {
                    notes.push(format!(
                        "section [{name}] has no step_pin; assuming {PRIMARY_MCU}"
                    ));
                    PRIMARY_MCU.to_owned()
                },
                pin_mcu,
            );
        z_steppers.push(ZStepper {
            name: name.clone(),
            mcu,
        });
    }

    // position_min: the Z rail's, falling back to [printer]
    // minimum_z_position.
    let z_position_min = section("stepper_z")
        .and_then(|s| s.get("position_min"))
        .and_then(Value::as_f64)
        .or_else(|| {
            section("printer")
                .and_then(|s| s.get("minimum_z_position"))
                .and_then(Value::as_f64)
        });

    // Known axis travel limits for the whole-itinerary pre-flight
    // (plr_recovery::preflight), read from the Klipper stepper sections.
    let axis_limits = axis_limits_from(settings);

    // Probes, per the [plr] probe_method (authoritative — see below).
    let probes = match plr.probe_method.as_str() {
        "adxl_drag" => {
            // The nozzle is the stylus: no z_offset, no probe gcode. A
            // coexisting [probe] section (kept for ordinary bed
            // leveling) is deliberately NOT counted as a second probe:
            // probe_method selects what recovery uses.
            vec![ProbeConfig {
                kind: ProbeKind::AdxlDrag {
                    chip: plr.accel_chip.clone(),
                },
                z_offset: 0.0,
                activate_gcode_no_move: true,
                deactivate_gcode_no_move: true,
            }]
        }
        method => {
            // The method's own section first (validation reads
            // probes[0]); the other traditional section, if ALSO
            // present, is appended so the single-probe check reports
            // the inconsistency.
            let tap = ("probe", ProbeKind::Tap);
            let load_cell = ("load_cell_probe", ProbeKind::LoadCell);
            let ordered = if method == "load_cell" {
                [load_cell, tap]
            } else {
                [tap, load_cell]
            };
            ordered
                .into_iter()
                .filter_map(|(name, kind)| section(name).map(|s| probe_from_section(s, kind)))
                .collect()
        }
    };

    // Calibration fingerprint defense-in-depth (see `gated_noise_floor`).
    // The fingerprint reads `config` (raw file-only), NOT `settings`.
    let noise_floor = gated_noise_floor(config, plr, &mut notes);
    note_stale_probe_resolution(config, plr, &mut notes);

    let virtual_sdcard_root = section("virtual_sdcard")
        .and_then(|s| s.get("path"))
        .and_then(Value::as_str)
        .map(str::to_owned);
    if virtual_sdcard_root
        .as_deref()
        .is_some_and(|r| r.starts_with('~'))
    {
        notes.push(
            "[virtual_sdcard] path uses `~`; plrd cannot expand another user's home — \
             use an absolute path or recovery will refuse the file as non-top-level"
                .to_owned(),
        );
    }

    let machine = MachineConfig {
        force_move_enabled: section("force_move")
            .and_then(|s| s.get("enable_force_move"))
            .and_then(Value::as_bool)
            .unwrap_or(false),
        z_self_locking_attested: plr.self_locking_z,
        z_steppers,
        primary_mcu: PRIMARY_MCU.to_owned(),
        type_annotations_present,
        probes,
        z_position_min,
        // Live-config mode: change detection is satisfied by
        // construction (module docs); both sides carry the sentinel so
        // the equality check passes trivially.
        config_hash: LIVE_CONFIG_HASH.to_owned(),
        validated_config_hash: Some(LIVE_CONFIG_HASH.to_owned()),
        virtual_sdcard_root,
        noise_floor,
        noise_floor_speed: plr.noise_floor_speed,
        axis_limits,
        // `[printer] max_accel` is a required Klipper option, so it is
        // present here on every real machine. It exists in the snapshot
        // for exactly one purpose: the generated recovery file restores
        // it as a literal after clamping its entry moves to
        // `accel_entry` (the file has no runtime-placeholder machinery).
        max_accel: section("printer")
            .and_then(|s| s.get("max_accel"))
            .and_then(Value::as_f64),
    };
    (machine, notes)
}

/// What one live query of the Klipper API socket returned.
#[derive(Debug, Clone)]
pub struct KlippySnapshot {
    /// `configfile.settings`: `{section: {option: typed value}}` — the
    /// access-tracked, default-resolved view. Used for machine ASSEMBLY
    /// (typed values), never for the calibration fingerprint.
    pub settings: Map<String, Value>,
    /// `configfile.config`: `{section: {option: string value}}` — the
    /// raw, file-only config (autosave merged, no defaults). This is the
    /// calibration FINGERPRINT input, so it matches the plugin's
    /// file-only Python hash byte-for-byte. Empty when the query result
    /// omits it (older klippy / a stub) — a legacy calibration then still
    /// legacy-accepts (empty config only matters when a stamp is present,
    /// in which case a mismatch conservatively refuses, never a
    /// false-accept).
    pub config: Map<String, Value>,
    /// The plugin's `plr` status object, when the plugin is loaded
    /// (`{method, configured, attested, probe_resolution,
    /// daemon_alive}` plus `last_drag_result` after a drag probe).
    pub plr_object: Option<Map<String, Value>>,
}

impl KlippySnapshot {
    /// Extracts the snapshot from an `objects/query` result. (Off-Unix
    /// only tests reach this: the stub query cannot produce a result.)
    #[cfg_attr(not(unix), allow(dead_code))]
    pub fn from_query_result(result: &Value) -> Result<Self, String> {
        let status = result
            .get("status")
            .and_then(Value::as_object)
            .ok_or_else(|| "objects/query result has no status map".to_owned())?;
        let configfile = status.get("configfile");
        let settings = configfile
            .and_then(|c| c.get("settings"))
            .and_then(Value::as_object)
            .cloned()
            .ok_or_else(|| {
                "configfile.settings absent from the query result (klippy still starting?)"
                    .to_owned()
            })?;
        // The raw file-only config (klippy `PrinterConfig.get_status` ->
        // `config`): the calibration-fingerprint input. Tolerant of
        // absence so machine ASSEMBLY (which uses `settings`) still works
        // against an older klippy that omits it.
        let config = configfile
            .and_then(|c| c.get("config"))
            .and_then(Value::as_object)
            .cloned()
            .unwrap_or_default();
        let plr_object = status
            .get("plr")
            .and_then(Value::as_object)
            .filter(|m| !m.is_empty())
            .cloned();
        Ok(Self {
            settings,
            config,
            plr_object,
        })
    }

    /// The `[plr]` settings section, when the config has one.
    #[must_use]
    pub fn plr_section(&self) -> Option<&Map<String, Value>> {
        self.settings.get("plr").and_then(Value::as_object)
    }
}

/// Queries `configfile` + `plr` over the Klipper API socket (Unix
/// stream, `0x03`-framed JSON — the same wire `client.rs` records
/// from, but as a one-shot blocking call: recover-time code is
/// synchronous and must not entangle itself with the recorder's
/// session).
#[cfg(unix)]
pub fn query_klippy_snapshot(
    socket: &std::path::Path,
    timeout: std::time::Duration,
) -> Result<KlippySnapshot, String> {
    use std::io::{Read as _, Write as _};

    use plr_klipper::{classify, FrameEvent, FrameSplitter, Inbound, Request, SubscriptionObjects};

    let mut objects = SubscriptionObjects::new();
    objects.insert("configfile".to_owned(), None);
    objects.insert("plr".to_owned(), None);
    let frame = Request::ObjectsQuery { objects }
        .to_frame(1)
        .map_err(|e| format!("cannot encode objects/query: {e}"))?;

    let mut stream = std::os::unix::net::UnixStream::connect(socket)
        .map_err(|e| format!("cannot connect to klippy at {}: {e}", socket.display()))?;
    stream
        .set_read_timeout(Some(timeout))
        .and_then(|()| stream.set_write_timeout(Some(timeout)))
        .map_err(|e| format!("cannot set socket timeouts: {e}"))?;
    stream
        .write_all(&frame)
        .map_err(|e| format!("cannot send objects/query to klippy: {e}"))?;

    let deadline = std::time::Instant::now() + timeout;
    let mut splitter = FrameSplitter::new();
    let mut buf = vec![0_u8; 64 * 1024];
    loop {
        if std::time::Instant::now() >= deadline {
            return Err(format!(
                "klippy at {} did not answer objects/query within {timeout:?}",
                socket.display()
            ));
        }
        let n = stream
            .read(&mut buf)
            .map_err(|e| format!("read from klippy failed: {e}"))?;
        if n == 0 {
            return Err("klippy closed the socket mid-query".to_owned());
        }
        for event in splitter.feed(&buf[..n]) {
            let FrameEvent::Frame(frame) = event else {
                continue; // oversized frames cannot be ours
            };
            match classify(&frame) {
                // Subscription noise from other clients does not reach
                // this fresh connection; only our reply matters.
                Ok(Inbound::Response { id: 1, result }) => {
                    return KlippySnapshot::from_query_result(&result);
                }
                Ok(Inbound::Error { id: 1, error }) => {
                    return Err(format!(
                        "klippy rejected objects/query: {:?}",
                        error.message
                    ));
                }
                Ok(_) | Err(_) => {}
            }
        }
    }
}

/// Non-Unix stub: there is no Unix-socket transport, so the snapshot is
/// always unavailable and callers take their documented
/// klippy-unreachable path (legacy fallback or refusal).
#[cfg(not(unix))]
pub fn query_klippy_snapshot(
    socket: &std::path::Path,
    _timeout: std::time::Duration,
) -> Result<KlippySnapshot, String> {
    Err(format!(
        "cannot query klippy at {}: unix sockets are unsupported on this platform",
        socket.display()
    ))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::{machine_from_settings, pin_mcu, KlippySnapshot, PlrSettings, LIVE_CONFIG_HASH};
    use plr_recovery::{validate_machine, PrereqFailure, ProbeKind};
    use serde_json::{json, Map, Value};

    /// A realistic `configfile` status body for a `[plr]`-commissioned
    /// tap machine. Shape per `klippy/configfile.py`:
    /// `PrinterConfig.get_status` returns `config` (raw strings),
    /// `settings` (typed, access-tracked, **autosave-merged** — note
    /// `self_locking_z`/`probe_resolution` present as ordinary
    /// settings), `warnings`, and the save-pending fields.
    pub(crate) fn configfile_status(plr_overrides: &[(&str, Value)]) -> Value {
        let mut plr = json!({
            "probe_method": "tap",
            "accel_chip": "",
            "wal_dir": "/var/lib/plrd/wal",
            "control_socket": "/var/lib/plrd/plrd.sock",
            "probe_speed": 1.0,
            "envelope_margin": 0.5,
            "sag_allowance": 0.2,
            "drag_speed": 20.0,
            "drag_z_step": 0.05,
            "drag_sensitivity": 30.0,
            "exclusion_radius": 5.0,
            "entry_feedrate": 1200.0,
            // SAVE_CONFIG-persisted: merged into settings after
            // restart (ConfigAutoSave.load_main_config).
            "self_locking_z": true,
            "probe_resolution": 0.0125
        });
        for (key, value) in plr_overrides {
            plr[*key] = value.clone();
        }
        // The raw, file-only `config` block (klippy `PrinterConfig.get_status`
        // -> `config`): the calibration-fingerprint input. It carries the
        // SAME file-written keys as `settings` for the fingerprint-relevant
        // sections but as raw STRINGS (as klippy's raw config does) — and
        // deliberately NONE of the default-injected keys `settings` gains
        // (modeled per-test, e.g. `settings_default_keys_do_not_move_the_...`).
        // `[plr]` in `config` carries only the fingerprint-relevant hardware
        // keys (the plugin's file-options view), mirrored from the overrides.
        let config_plr = json!({
            "probe_method": plr["probe_method"].clone(),
            "accel_chip": plr["accel_chip"].clone()
        });
        json!({
            "config": {
                "plr": config_plr,
                "stepper_z": {
                    "step_pin": "PB0",
                    "position_min": "-2",
                    "position_max": "250"
                },
                "stepper_z1": {"step_pin": "!PB3"},
                "probe": {
                    "z_offset": "-0.1",
                    "activate_gcode": "",
                    "deactivate_gcode": ""
                },
                "printer": {"kinematics": "corexy"}
            },
            "warnings": [],
            "save_config_pending": false,
            "save_config_pending_items": {},
            "settings": {
                "plr": plr,
                "printer": {
                    "kinematics": "corexy",
                    "max_velocity": 300.0,
                    "minimum_z_position": -3.0
                },
                "force_move": {"enable_force_move": true},
                "stepper_z": {
                    "step_pin": "PB0",
                    "position_min": -2.0,
                    "position_max": 250.0
                },
                "stepper_z1": {"step_pin": "!PB3"},
                "probe": {
                    "z_offset": -0.1,
                    "activate_gcode": "",
                    "deactivate_gcode": ""
                },
                "virtual_sdcard": {"path": "/home/pi/printer_data/gcodes"}
            }
        })
    }

    // Test-helper ergonomics: callers hand over the fixture values.
    #[allow(clippy::needless_pass_by_value)]
    pub(crate) fn query_result(configfile: Value, plr_object: Value) -> Value {
        json!({
            "eventtime": 100.5,
            "status": {"configfile": configfile, "plr": plr_object}
        })
    }

    /// A minimal fake klippy API socket (Unix only): serves every
    /// connection the same `objects/query` response, over the real
    /// 0x03-framed wire. The accept thread is detached; it dies with
    /// the process.
    #[cfg(unix)]
    pub(crate) fn spawn_fake_klippy(tag: &str, response: Value) -> std::path::PathBuf {
        use std::io::{Read as _, Write as _};
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-fake-klippy-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("klippy.sock");
        let listener = std::os::unix::net::UnixListener::bind(&path).unwrap();
        std::thread::spawn(move || {
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else { return };
                let response = response.clone();
                std::thread::spawn(move || {
                    let mut buf = Vec::new();
                    let mut chunk = [0_u8; 4096];
                    loop {
                        match stream.read(&mut chunk) {
                            Ok(0) | Err(_) => return,
                            Ok(n) => buf.extend_from_slice(&chunk[..n]),
                        }
                        if let Some(pos) = buf.iter().position(|&b| b == 0x03) {
                            let request: Value =
                                serde_json::from_slice(&buf[..pos]).unwrap_or(Value::Null);
                            let id = request.get("id").cloned().unwrap_or(Value::Null);
                            let mut reply =
                                serde_json::to_vec(&json!({"id": id, "result": response})).unwrap();
                            reply.push(0x03);
                            let _ = stream.write_all(&reply);
                            return;
                        }
                    }
                });
            }
        });
        path
    }

    #[cfg(unix)]
    #[test]
    fn live_query_round_trips_over_a_real_socket() {
        let response = query_result(configfile_status(&[]), plr_object());
        let path = spawn_fake_klippy("query", response);
        let snapshot =
            super::query_klippy_snapshot(&path, std::time::Duration::from_secs(5)).unwrap();
        assert!(snapshot.plr_section().is_some());
        assert!(snapshot.plr_object.is_some());
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        assert_eq!(plr.probe_method, "tap");
    }

    #[cfg(unix)]
    #[test]
    fn live_query_reports_unreachable_and_dead_sockets() {
        let err = super::query_klippy_snapshot(
            std::path::Path::new("/nonexistent-plrd/klippy.sock"),
            std::time::Duration::from_millis(200),
        )
        .unwrap_err();
        assert!(err.contains("cannot connect"), "{err}");
    }

    pub(crate) fn plr_object() -> Value {
        json!({
            "probe_method": "tap",
            "configured": true,
            "attested": true,
            "probe_resolution": 0.0125,
            "daemon_alive": true
        })
    }

    fn parse_fixture(overrides: &[(&str, Value)]) -> (KlippySnapshot, PlrSettings) {
        let result = query_result(configfile_status(overrides), plr_object());
        let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        (snapshot, plr)
    }

    #[test]
    fn full_fixture_parses_and_validates() {
        let (snapshot, plr) = parse_fixture(&[]);
        assert_eq!(plr.probe_method, "tap");
        assert!(plr.self_locking_z);
        assert_eq!(plr.probe_resolution, Some(0.0125));
        assert!(plr.noise_floor.is_empty());
        assert_eq!(plr.control_socket, "/var/lib/plrd/plrd.sock");

        let (machine, notes) =
            machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert!(notes.is_empty(), "{notes:?}");
        assert!(machine.force_move_enabled);
        assert!(machine.z_self_locking_attested);
        // Both Z steppers discovered; MCUs derived from step_pin
        // (modifier prefixes stripped, bare pin = primary MCU).
        assert_eq!(machine.z_steppers.len(), 2);
        assert!(machine.z_steppers.iter().all(|s| s.mcu == "mcu"));
        assert_eq!(machine.probes.len(), 1);
        assert_eq!(machine.probes[0].kind, ProbeKind::Tap);
        assert!((machine.probes[0].z_offset - (-0.1)).abs() < 1e-12);
        assert!(machine.probes[0].activate_gcode_no_move);
        // The rail's own position_min wins over [printer].
        assert_eq!(machine.z_position_min, Some(-2.0));
        assert_eq!(
            machine.virtual_sdcard_root.as_deref(),
            Some("/home/pi/printer_data/gcodes")
        );
        // Live-config mode: change detection satisfied by construction.
        assert_eq!(machine.config_hash, LIVE_CONFIG_HASH);
        assert_eq!(
            machine.validated_config_hash.as_deref(),
            Some(LIVE_CONFIG_HASH)
        );
        assert!(validate_machine(&machine).is_ok());
    }

    #[test]
    fn plan_config_carries_the_plr_tunables() {
        let (_, plr) = parse_fixture(&[
            ("probe_speed", json!(1.5)),
            ("envelope_margin", json!(0.8)),
            ("sag_allowance", json!(0.3)),
            ("entry_feedrate", json!(900.0)),
            ("drag_z_step", json!(0.08)),
        ]);
        let config = plr.plan_config();
        assert!((config.probe_speed - 1.5).abs() < 1e-12);
        assert!((config.margin - 0.8).abs() < 1e-12);
        assert!((config.sag_allowance - 0.3).abs() < 1e-12);
        assert!((config.entry_feed - 900.0).abs() < 1e-12);
        assert!((config.drag_z_step - 0.08).abs() < 1e-12);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn plan_config_carries_the_frozen_recovery_ux_keys() {
        let (_, plr) = parse_fixture(&[
            ("max_probe_nozzle_temp", json!(155.0)),
            ("reheat_park_x", json!(12.5)),
            ("reheat_park_y", json!(30.0)),
            ("reheat_park_delta_z", json!(3.0)),
            ("pre_home_z_lift", json!(8.0)),
            ("purge_enable", json!(false)),
            ("purge_amount", json!(7.0)),
            ("purge_macro", json!("MY_PURGE")),
            ("clean_nozzle_macro", json!("WIPE_NOZZLE")),
        ]);
        assert!((plr.max_probe_nozzle_temp - 155.0).abs() < 1e-12);
        assert!((plr.reheat_park_x.unwrap() - 12.5).abs() < 1e-12);
        assert!((plr.reheat_park_y.unwrap() - 30.0).abs() < 1e-12);
        assert!(!plr.purge_enable);
        assert_eq!(plr.purge_macro.as_deref(), Some("MY_PURGE"));
        assert_eq!(plr.clean_nozzle_macro, "WIPE_NOZZLE");
        let config = plr.plan_config();
        assert!((config.max_probe_nozzle_temp - 155.0).abs() < 1e-12);
        assert!((config.reheat_park_x.unwrap() - 12.5).abs() < 1e-12);
        assert!((config.reheat_park_delta_z - 3.0).abs() < 1e-12);
        assert!((config.pre_home_z_lift - 8.0).abs() < 1e-12);
        assert!(!config.purge_enable);
        assert_eq!(config.purge_macro.as_deref(), Some("MY_PURGE"));
        assert_eq!(config.clean_nozzle_macro, "WIPE_NOZZLE");
        assert!(config.validate().is_ok());
    }

    #[test]
    fn frozen_keys_default_and_parse_tolerantly() {
        // Absent keys default to the plan-builder defaults; optional park
        // coordinates absent → None.
        let (_, plr) = parse_fixture(&[]);
        assert!((plr.max_probe_nozzle_temp - 150.0).abs() < 1e-12);
        assert_eq!(plr.reheat_park_x, None);
        assert_eq!(plr.reheat_park_y, None);
        assert!(plr.purge_enable);
        assert_eq!(plr.purge_macro, None);
        assert_eq!(plr.clean_nozzle_macro, "CLEAN_NOZZLE");
        // A wrong-typed OPTIONAL park coordinate is tolerated as None.
        let (_, plr) = parse_fixture(&[("reheat_park_x", json!("oops"))]);
        assert_eq!(plr.reheat_park_x, None);
        // An empty purge_macro string is treated as unset.
        let (_, plr) = parse_fixture(&[("purge_macro", json!("  "))]);
        assert_eq!(plr.purge_macro, None);
    }

    #[test]
    fn purge_and_drag_temp_keys_parse_and_carry_through() {
        let (_, plr) = parse_fixture(&[
            ("drag_nozzle_temp", json!(120.0)),
            ("purge_x", json!(150.0)),
            ("purge_y", json!(12.0)),
            ("purge_z", json!(0.8)),
            ("purge_speed", json!(240.0)),
            ("purge_retract", json!(2.0)),
            ("purge_amount", json!(9.0)),
        ]);
        assert!((plr.drag_nozzle_temp - 120.0).abs() < 1e-12);
        assert!((plr.purge_x.unwrap() - 150.0).abs() < 1e-12);
        assert!((plr.purge_y.unwrap() - 12.0).abs() < 1e-12);
        assert!((plr.purge_z.unwrap() - 0.8).abs() < 1e-12);
        let config = plr.plan_config();
        assert!((config.drag_nozzle_temp - 120.0).abs() < 1e-12);
        assert!((config.purge_speed - 240.0).abs() < 1e-12);
        assert!((config.purge_retract - 2.0).abs() < 1e-12);
        assert!((config.purge_amount - 9.0).abs() < 1e-12);
        assert_eq!(config.purge_x, plr.purge_x);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn purge_and_drag_temp_keys_default_tolerantly() {
        let (_, plr) = parse_fixture(&[]);
        // Defaults match the plan builder's.
        let d = plr_recovery::PlanConfig::default();
        assert!((plr.drag_nozzle_temp - d.drag_nozzle_temp).abs() < 1e-12);
        assert!((plr.purge_speed - d.purge_speed).abs() < 1e-12);
        assert!((plr.purge_retract - d.purge_retract).abs() < 1e-12);
        assert_eq!(plr.purge_x, None);
        assert_eq!(plr.purge_y, None);
        assert_eq!(plr.purge_z, None);
        // Wrong-typed OPTIONAL coordinates degrade to None, never error.
        let (_, plr) = parse_fixture(&[("purge_x", json!("oops"))]);
        assert_eq!(plr.purge_x, None);
        // The cold-drag opt-out rides through.
        let (_, plr) = parse_fixture(&[("drag_nozzle_temp", json!(0.0))]);
        assert!((plr.drag_nozzle_temp - 0.0).abs() < 1e-12);
        assert!(plr.plan_config().validate().is_ok());
    }

    #[test]
    fn gcode_macro_names_are_uppercased_from_lowercased_sections() {
        use super::gcode_macro_names;
        let sections = json!({
            "gcode_macro clean_nozzle": {"gcode": "..."},
            "gcode_macro my_purge": {"gcode": "..."},
            "stepper_z": {"step_pin": "PB0"},
        });
        let names = gcode_macro_names(sections.as_object().unwrap());
        assert!(names.contains("CLEAN_NOZZLE"));
        assert!(names.contains("MY_PURGE"));
        assert!(!names.contains("STEPPER_Z"));
        assert_eq!(names.len(), 2);
    }

    #[test]
    fn adxl_drag_machine_builds_the_drag_probe() {
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
            ("noise_floor_peak", json!(410.0)),
            ("noise_floor_speed", json!(20.0)),
        ]);
        assert_eq!(plr.noise_floor.get("noise_floor_rms").copied(), Some(118.0));
        assert_eq!(plr.representative_noise_floor(), Some(118.0));
        // The calibration speed is metadata, carried separately —
        // never part of the calibrated-floor map.
        assert_eq!(plr.noise_floor_speed, Some(20.0));
        assert!(!plr.noise_floor.contains_key("noise_floor_speed"));
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.noise_floor_speed, Some(20.0));
        // probe_method is authoritative: the coexisting [probe]
        // section does not create a second probe object.
        assert_eq!(machine.probes.len(), 1);
        assert_eq!(
            machine.probes[0].kind,
            ProbeKind::AdxlDrag {
                chip: "adxl345".to_owned()
            }
        );
        assert_eq!(machine.noise_floor, Some(118.0));
        assert!(validate_machine(&machine).is_ok());
    }

    #[test]
    fn uncalibrated_drag_machine_fails_validation_with_the_hint() {
        // A recorded calibration SPEED alone is not a calibration:
        // the machine must still refuse drag.
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_speed", json!(20.0)),
        ]);
        assert!(plr.noise_floor.is_empty());
        assert_eq!(plr.noise_floor_speed, Some(20.0));
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        let rejection = validate_machine(&machine).unwrap_err();
        assert!(rejection
            .failures
            .contains(&PrereqFailure::NoiseFloorMissing));
    }

    /// The temperature covariate must not cost the operator recovery.
    ///
    /// `noise_floor_temp_sensor` is a klippy sensor NAME (a string) and is
    /// documented for operators (`klippy_plugin/README.md`,
    /// `docs/install.md`). Before [`super::NOISE_FLOOR_METADATA_KEYS`]
    /// existed, the `noise_floor_*` prefix sweep demanded a number from it
    /// and the whole `[plr]` section failed to parse with
    /// `[plr] noise_floor_temp_sensor is not a number: "temperature_sensor
    /// chamber"` — i.e. configuring a documented feature disabled recovery
    /// entirely.
    #[test]
    fn temp_sensor_metadata_parses_alongside_real_measurements() {
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
            ("noise_floor_still_rms", json!(31.0)),
            ("noise_floor_speed", json!(20.0)),
            ("noise_floor_temp", json!(40.0)),
            (
                "noise_floor_temp_sensor",
                json!("temperature_sensor chamber"),
            ),
        ]);
        // Only the measurements are in the calibrated-floor map.
        assert_eq!(
            plr.noise_floor.keys().collect::<Vec<_>>(),
            vec!["noise_floor_rms", "noise_floor_still_rms"]
        );
        assert_eq!(plr.representative_noise_floor(), Some(118.0));
        assert_eq!(plr.noise_floor_speed, Some(20.0));
        // ... and the machine is fully calibrated, temperature covariate
        // and all.
        let (machine, notes) =
            machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.noise_floor, Some(118.0));
        assert_eq!(machine.noise_floor_speed, Some(20.0));
        assert!(validate_machine(&machine).is_ok(), "{notes:?}");
    }

    /// A recorded TEMPERATURE is not a noise floor.
    ///
    /// `PLR_NOISE_TEST` stages `noise_floor_temp` only as the temperature
    /// the baseline was captured at. A machine carrying it alone has never
    /// measured a floor, so it must fall through to the same
    /// `NoiseFloorMissing` refusal an untouched machine gets — not present
    /// 40 °C as a calibration. Nothing downstream could catch this: 40.0
    /// is a perfectly valid noise-floor number.
    #[test]
    fn temperature_only_is_not_a_calibrated_noise_floor() {
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_temp", json!(40.0)),
            ("noise_floor_temp_sensor", json!("extruder")),
        ]);
        assert!(plr.noise_floor.is_empty());
        assert_eq!(plr.representative_noise_floor(), None);
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.noise_floor, None);
        let rejection = validate_machine(&machine).unwrap_err();
        assert_eq!(
            rejection.failures,
            vec![PrereqFailure::NoiseFloorMissing],
            "temperature-only must read as uncalibrated"
        );
        assert!(
            rejection.failures[0]
                .to_string()
                .contains("run PLR_NOISE_TEST first"),
            "{}",
            rejection.failures[0]
        );
    }

    /// The sweep still types every UNCLASSIFIED `noise_floor_*` key: a
    /// measurement plrd cannot read is a hard error, metadata or not.
    #[test]
    fn a_non_numeric_measurement_still_refuses_the_section() {
        for (key, value) in [
            ("noise_floor_rms", json!("loud")),
            // A key the plugin has not written yet: unclassified means
            // measurement, so it is typed like one.
            ("noise_floor_p95", json!("loud")),
        ] {
            let result = query_result(
                configfile_status(&[("probe_method", json!("adxl_drag")), (key, value)]),
                plr_object(),
            );
            let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
            let err = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap_err();
            assert_eq!(err, format!("[plr] {key} is not a number: \"loud\""));
        }
    }

    /// Every classified key must actually be reachable by the sweep it is
    /// meant to exempt: an entry without the prefix would be a silent
    /// no-op, and the key it names would go back to being swept as a
    /// measurement.
    #[test]
    fn every_metadata_key_carries_the_noise_floor_prefix() {
        for key in super::NOISE_FLOOR_METADATA_KEYS {
            assert!(
                key.starts_with(super::NOISE_FLOOR_PREFIX),
                "{key} cannot be exempted from a sweep it never reaches"
            );
        }
    }

    #[test]
    fn missing_cross_check_sections_fail_validation_not_parsing() {
        // Strip every cross-check section: assembly stays total and
        // validation lists every gap.
        let (snapshot, plr) = parse_fixture(&[]);
        let mut settings = Map::new();
        settings.insert("plr".to_owned(), snapshot.settings["plr"].clone());
        // No stamps and no noise floor here, so the fingerprint input is
        // immaterial (probe_resolution is legacy, noise floor absent);
        // reuse `settings` as the config map.
        let (machine, _) = machine_from_settings(&settings, &settings, &plr, false);
        let rejection = validate_machine(&machine).unwrap_err();
        let f = &rejection.failures;
        assert!(f.contains(&PrereqFailure::ForceMoveDisabled));
        assert!(f.contains(&PrereqFailure::NoZSteppers));
        assert!(f.contains(&PrereqFailure::NoProbe));
        assert!(f.contains(&PrereqFailure::PositionMinUnknown));
        assert!(f.contains(&PrereqFailure::SdcardRootUnknown));
        assert!(f.contains(&PrereqFailure::NoTypeAnnotations));
    }

    #[test]
    fn secondary_mcu_z_and_nonempty_activate_gcode_are_detected() {
        let result = query_result(configfile_status(&[]), plr_object());
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        snapshot.settings["stepper_z1"]["step_pin"] = json!("^!z_board:PA1");
        snapshot.settings["probe"]["activate_gcode"] = json!("G1 Z5\n");
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        let rejection = validate_machine(&machine).unwrap_err();
        assert!(rejection.failures.iter().any(|f| matches!(
            f,
            PrereqFailure::ZStepperOffPrimaryMcu { stepper, mcu }
                if stepper == "stepper_z1" && mcu == "z_board"
        )));
        assert!(rejection
            .failures
            .contains(&PrereqFailure::ProbeActivateGcodeMoves));
    }

    #[test]
    fn both_probe_sections_present_is_a_multiple_probe_refusal() {
        let result = query_result(configfile_status(&[]), plr_object());
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        snapshot.settings.insert(
            "load_cell_probe".to_owned(),
            json!({"z_offset": -0.15, "activate_gcode": ""}),
        );
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        // The method's section leads; the stray one still counts.
        assert_eq!(machine.probes[0].kind, ProbeKind::Tap);
        let rejection = validate_machine(&machine).unwrap_err();
        assert!(rejection
            .failures
            .contains(&PrereqFailure::MultipleProbes { count: 2 }));
    }

    #[test]
    fn load_cell_method_reads_its_own_section_first() {
        let result = query_result(
            configfile_status(&[("probe_method", json!("load_cell"))]),
            plr_object(),
        );
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        let settings = &mut snapshot.settings;
        let probe = settings.remove("probe").unwrap();
        let mut lc = probe.as_object().unwrap().clone();
        lc.insert("z_offset".to_owned(), json!(-0.15));
        settings.insert("load_cell_probe".to_owned(), Value::Object(lc));
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.probes.len(), 1);
        assert_eq!(machine.probes[0].kind, ProbeKind::LoadCell);
        assert!((machine.probes[0].z_offset - (-0.15)).abs() < 1e-12);
    }

    #[test]
    fn tilde_sdcard_root_is_noted() {
        let result = query_result(configfile_status(&[]), plr_object());
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        snapshot.settings["virtual_sdcard"]["path"] = json!("~/printer_data/gcodes");
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (_, notes) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert!(notes.iter().any(|n| n.contains('~')), "{notes:?}");
    }

    #[test]
    fn parse_rejects_bad_types_and_methods() {
        for (key, value, needle) in [
            ("probe_method", json!(7), "not a string"),
            ("probe_method", json!("laser"), "not tap, load_cell"),
            ("probe_speed", json!("fast"), "not a number"),
            ("self_locking_z", json!("yes"), "not a boolean"),
            ("accel_chip", json!(3), "not a string"),
            ("noise_floor_rms", json!("loud"), "not a number"),
            ("noise_floor_speed", json!("fast"), "not a number"),
            ("probe_resolution", json!(false), "not a number"),
        ] {
            let result = query_result(configfile_status(&[(key, value)]), plr_object());
            let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
            let err = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap_err();
            assert!(err.contains(needle), "{key}: {err}");
        }
        // A missing probe_method is required.
        let result = query_result(configfile_status(&[]), plr_object());
        let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        let mut section = snapshot.plr_section().unwrap().clone();
        section.remove("probe_method");
        assert!(PlrSettings::parse(&section)
            .unwrap_err()
            .contains("probe_method is missing"));
    }

    #[test]
    fn touch_tunables_parse_and_carry_into_plan_config() {
        let (_, plr) = parse_fixture(&[
            ("touch_samples", json!(5.0)),
            ("touch_sample_range", json!(0.012)),
            ("touch_retract", json!(3.0)),
            ("touch_accel", json!(250.0)),
        ]);
        assert!((plr.touch_samples - 5.0).abs() < 1e-12);
        assert!((plr.touch_sample_range - 0.012).abs() < 1e-12);
        assert!((plr.touch_retract - 3.0).abs() < 1e-12);
        assert!((plr.touch_accel - 250.0).abs() < 1e-12);
        let config = plr.plan_config();
        assert!((config.touch_samples - 5.0).abs() < 1e-12);
        assert!((config.touch_sample_range - 0.012).abs() < 1e-12);
        // [plr] mode uses the consensus touch (plugin present).
        assert!(!config.legacy_single_probe);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn touch_defaults_apply_when_keys_absent() {
        let (_, plr) = parse_fixture(&[]);
        let d = plr_recovery::PlanConfig::default();
        assert!((plr.touch_samples - d.touch_samples).abs() < 1e-12);
        assert!((plr.touch_sample_range - d.touch_sample_range).abs() < 1e-12);
        assert!((plr.touch_retract - d.touch_retract).abs() < 1e-12);
        assert!((plr.touch_accel - d.touch_accel).abs() < 1e-12);
    }

    #[test]
    fn touch_sample_range_hard_cap_is_refused_at_plan_config() {
        // A [plr] section is parsed tolerantly (values are not clamped),
        // but the resulting plan config REFUSES a sample range above the
        // 0.015 hard cap (Cartographer configuration.py:248).
        let (_, plr) = parse_fixture(&[("touch_sample_range", json!(0.02))]);
        assert!((plr.touch_sample_range - 0.02).abs() < 1e-12);
        let err = plr.plan_config().validate().unwrap_err();
        assert!(
            matches!(
                err,
                plr_recovery::RecoveryError::InvalidPlanConfig {
                    field: "touch_sample_range"
                }
            ),
            "{err:?}"
        );
        // The 0.015 boundary itself is accepted.
        let (_, ok) = parse_fixture(&[("touch_sample_range", json!(0.015))]);
        assert!(ok.plan_config().validate().is_ok());
    }

    #[test]
    fn axis_limits_come_from_the_stepper_sections() {
        // The fixture has stepper_z position_min/max but no stepper_x/y,
        // so z_max is known and x/y are unknown ("where known").
        let (snapshot, plr) = parse_fixture(&[]);
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.axis_limits.z_max, Some(250.0));
        assert_eq!(machine.axis_limits.x, None);
        assert_eq!(machine.axis_limits.y, None);

        // Add stepper_x/stepper_y sections: both limit pairs appear.
        let result = query_result(configfile_status(&[]), plr_object());
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        snapshot.settings.insert(
            "stepper_x".to_owned(),
            json!({"position_min": 0.0, "position_max": 235.0}),
        );
        snapshot.settings.insert(
            "stepper_y".to_owned(),
            json!({"position_min": -1.0, "position_max": 235.0}),
        );
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.axis_limits.x, Some((0.0, 235.0)));
        assert_eq!(machine.axis_limits.y, Some((-1.0, 235.0)));
    }

    #[test]
    fn missing_optional_keys_fall_back_to_plan_defaults() {
        let minimal: Map<String, Value> =
            json!({"probe_method": "tap"}).as_object().unwrap().clone();
        let plr = PlrSettings::parse(&minimal).unwrap();
        assert!(!plr.self_locking_z);
        assert_eq!(plr.probe_resolution, None);
        let d = plr_recovery::PlanConfig::default();
        assert!((plr.probe_speed - d.probe_speed).abs() < 1e-12);
        assert!((plr.drag_z_step - d.drag_z_step).abs() < 1e-12);
        assert!(plr.plan_config().validate().is_ok());
    }

    #[test]
    fn snapshot_extraction_handles_absent_pieces() {
        // No status at all.
        assert!(KlippySnapshot::from_query_result(&json!({})).is_err());
        // configfile absent (klippy still starting).
        let err = KlippySnapshot::from_query_result(&json!({"status": {"plr": {}}})).unwrap_err();
        assert!(err.contains("configfile.settings"), "{err}");
        // Plugin not loaded: klippy maps unknown objects to {}.
        let result = json!({"status": {
            "configfile": {"settings": {"printer": {}}},
            "plr": {}
        }});
        let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        assert!(snapshot.plr_object.is_none());
        assert!(snapshot.plr_section().is_none());
    }

    #[test]
    fn pin_mcu_strips_modifiers_and_prefixes() {
        assert_eq!(pin_mcu("PB0"), "mcu");
        assert_eq!(pin_mcu("!PB0"), "mcu");
        assert_eq!(pin_mcu("^!PB0"), "mcu");
        assert_eq!(pin_mcu("z_board:PA1"), "z_board");
        assert_eq!(pin_mcu("~*aux: PA1"), "aux");
    }

    // --- calibration fingerprinting -------------------------------------
    use super::{compute_fingerprint, fingerprint, validate_group, CalGroup, CalTier};

    /// The noise-floor fingerprint of the file-only slice
    /// `{stepper_z: {step_pin: PB0, position_min: -2}, adxl345: {cs_pin: PB1},
    /// plr: {probe_method: adxl_drag, accel_chip: adxl345}}`. Shared,
    /// byte-identical cross-language constant — the python suite asserts the
    /// SAME hex (`test_calibration_meta.py` `SHARED_ADXL_NOISE_FLOOR_HEX`),
    /// proving
    /// plrd's config-based recompute matches the plugin's file-options hash.
    const SHARED_ADXL_NOISE_FLOOR_HEX: &str = "d5bd905a";

    /// A `{key: string}` JSON object for a fixture section.
    fn obj(pairs: &[(&str, &str)]) -> Value {
        let mut map = Map::new();
        for (key, value) in pairs {
            map.insert((*key).to_owned(), json!(value));
        }
        Value::Object(map)
    }

    #[test]
    fn crc32_reproduces_zlib_reference_vectors() {
        assert_eq!(super::crc32_hex(b"123456789"), "cbf43926");
        assert_eq!(super::crc32_hex(b""), "00000000");
    }

    /// The three shared literal fixtures: the SAME expected hex the python
    /// suite asserts (`klippy_plugin/tests/test_calibration_meta.py`
    /// `SHARED_FIXTURES`). A byte-identical fingerprint across languages is
    /// the cross-language contract.
    #[test]
    fn fingerprint_matches_python_shared_fixtures() {
        // F1: stepper_z + probe, plr_keys=[probe_method] -> ca910c12.
        let mut f1 = Map::new();
        f1.insert(
            "stepper_z".to_owned(),
            obj(&[
                ("step_pin", "PF11"),
                ("dir_pin", "!PH1"),
                ("position_min", "-2"),
                ("position_max", "250"),
            ]),
        );
        f1.insert(
            "probe".to_owned(),
            obj(&[("z_offset", "0.5"), ("pin", "^PA1")]),
        );
        f1.insert(
            "plr".to_owned(),
            obj(&[("probe_method", "tap"), ("accel_chip", "adxl345")]),
        );
        assert_eq!(
            fingerprint(&f1, &["stepper_z", "probe"], &["probe_method"]),
            "ca910c12"
        );

        // F3: two Z steppers + accel chip, plr_keys=[accel_chip,probe_method].
        let mut f3 = Map::new();
        f3.insert(
            "stepper_z".to_owned(),
            obj(&[("step_pin", "PF11"), ("position_min", "-2")]),
        );
        f3.insert("stepper_z1".to_owned(), obj(&[("step_pin", "PG0")]));
        f3.insert(
            "adxl345".to_owned(),
            obj(&[("cs_pin", "PB1"), ("axes_map", "x,y,z")]),
        );
        f3.insert(
            "plr".to_owned(),
            obj(&[("probe_method", "adxl_drag"), ("accel_chip", "adxl345")]),
        );
        assert_eq!(
            fingerprint(
                &f3,
                &["stepper_z", "stepper_z1", "adxl345"],
                &["accel_chip", "probe_method"]
            ),
            "cecd3842"
        );

        // F4: numeric canonicalization (-2.0 -> -2), plr_keys=[probe_method].
        let mut f4 = Map::new();
        f4.insert(
            "stepper_z".to_owned(),
            obj(&[("position_min", "-2.0"), ("microsteps", "16")]),
        );
        f4.insert("plr".to_owned(), obj(&[("probe_method", "tap")]));
        assert_eq!(
            fingerprint(&f4, &["stepper_z"], &["probe_method"]),
            "404202d1"
        );
    }

    #[test]
    fn fingerprint_normalizes_typed_numbers_like_python_strings() {
        // A typed JSON number (as Klipper's getfloat yields) and the raw
        // string "-2" canonicalize identically, so plrd's recompute matches
        // the plugin's stamp on the same machine.
        let mut typed = Map::new();
        typed.insert("stepper_z".to_owned(), json!({ "position_min": -2.0 }));
        typed.insert("plr".to_owned(), json!({ "probe_method": "tap" }));
        let mut text = Map::new();
        text.insert("stepper_z".to_owned(), obj(&[("position_min", "-2")]));
        text.insert("plr".to_owned(), obj(&[("probe_method", "tap")]));
        assert_eq!(
            fingerprint(&typed, &["stepper_z"], &["probe_method"]),
            fingerprint(&text, &["stepper_z"], &["probe_method"])
        );
    }

    #[test]
    fn parse_tolerates_absent_and_wrongtyped_stamps() {
        // Absent -> None.
        let (_, plr) = parse_fixture(&[]);
        assert!(plr.cal_fingerprint_noise_floor.is_none());
        assert!(plr.cal_fingerprint_probe_resolution.is_none());
        assert!(plr.cal_plugin_version.is_none());
        assert!(plr.cal_klipper_version.is_none());
        // Wrong-typed -> None (tolerant), NOT a parse error.
        let (_, plr) = parse_fixture(&[
            ("cal_fingerprint_noise_floor", json!(12345)),
            ("cal_plugin_version", json!(3.0)),
            ("cal_klipper_version", json!(false)),
        ]);
        assert!(plr.cal_fingerprint_noise_floor.is_none());
        assert!(plr.cal_plugin_version.is_none());
        assert!(plr.cal_klipper_version.is_none());
        // A well-formed string stamp parses through.
        let (_, plr) = parse_fixture(&[("cal_plugin_version", json!("0.3.0"))]);
        assert_eq!(plr.cal_plugin_version.as_deref(), Some("0.3.0"));
    }

    #[test]
    fn noise_floor_fingerprint_mismatch_is_treated_as_missing() {
        // A drag machine with a calibrated floor but a STALE stamp: the floor
        // is treated as absent, the existing NoiseFloorMissing refusal fires,
        // and a note names the mismatch.
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
            ("cal_fingerprint_noise_floor", json!("deadbeef")),
        ]);
        let (machine, notes) =
            machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.noise_floor, None);
        let rejection = validate_machine(&machine).unwrap_err();
        assert!(rejection
            .failures
            .contains(&PrereqFailure::NoiseFloorMissing));
        assert!(
            notes.iter().any(|n| n.contains("fingerprint mismatch")),
            "{notes:?}"
        );
    }

    #[test]
    fn noise_floor_fingerprint_match_is_accepted() {
        // Recompute the true fingerprint, stamp it, and confirm the floor is
        // kept and validation passes (no mismatch note).
        let (snapshot, base) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
        ]);
        let fp = compute_fingerprint(&snapshot.config, &base, CalGroup::NoiseFloor);
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
            ("cal_fingerprint_noise_floor", json!(fp)),
        ]);
        assert_eq!(
            validate_group(&snapshot.config, &plr, CalGroup::NoiseFloor).0,
            CalTier::Valid
        );
        let (machine, notes) =
            machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.noise_floor, Some(118.0));
        assert!(validate_machine(&machine).is_ok());
        assert!(!notes.iter().any(|n| n.contains("mismatch")), "{notes:?}");
    }

    #[test]
    fn legacy_noise_floor_without_stamp_is_accepted_with_note() {
        // No stamp at all: the floor is kept (legacy back-compat) and a note
        // records that it was accepted without a fingerprint cross-check.
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
        ]);
        assert_eq!(
            validate_group(&snapshot.config, &plr, CalGroup::NoiseFloor).0,
            CalTier::Legacy
        );
        let (machine, notes) =
            machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.noise_floor, Some(118.0));
        assert!(validate_machine(&machine).is_ok());
        assert!(notes.iter().any(|n| n.contains("legacy")), "{notes:?}");
    }

    #[test]
    fn a_changed_z_stepper_pin_invalidates_a_stamped_floor() {
        // Stamp the true fingerprint, then mutate a Z stepper pin: the
        // recomputed fingerprint diverges and the floor is treated as missing.
        let (snapshot, base) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
        ]);
        let fp = compute_fingerprint(&snapshot.config, &base, CalGroup::NoiseFloor);
        let result = query_result(
            configfile_status(&[
                ("probe_method", json!("adxl_drag")),
                ("accel_chip", json!("adxl345")),
                ("noise_floor_rms", json!(118.0)),
                ("cal_fingerprint_noise_floor", json!(fp)),
            ]),
            plr_object(),
        );
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        // Unchanged config still validates VALID...
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        assert_eq!(
            validate_group(&snapshot.config, &plr, CalGroup::NoiseFloor).0,
            CalTier::Valid
        );
        // ...but changing the Z stepper step_pin (in the file config, the
        // fingerprint input) invalidates it.
        snapshot.config["stepper_z"]["step_pin"] = json!("PF99");
        assert_eq!(
            validate_group(&snapshot.config, &plr, CalGroup::NoiseFloor).0,
            CalTier::Invalid
        );
        let (machine, _) = machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        assert_eq!(machine.noise_floor, None);
    }

    #[test]
    fn irrelevant_section_change_does_not_move_the_fingerprint() {
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
        ]);
        let before = compute_fingerprint(&snapshot.config, &plr, CalGroup::NoiseFloor);
        let mut config = snapshot.config.clone();
        config.insert("fan".to_owned(), json!({ "pin": "PA8" }));
        config.insert("display".to_owned(), json!({ "lcd_type": "st7920" }));
        let after = compute_fingerprint(&config, &plr, CalGroup::NoiseFloor);
        assert_eq!(before, after);
    }

    /// The decisive regression test: `settings` carries default-injected keys
    /// (`[adxl345] axes_map` / `rate` — klippy/extras/adxl345.py defaults)
    /// that the operator never wrote, so they are ABSENT from `config`. The
    /// fingerprint MUST hash `config` (file-only) and MUST match the plugin's
    /// file-only Python hash; hashing `settings` would diverge and spuriously
    /// invalidate a correctly-calibrated machine.
    #[test]
    fn settings_default_keys_do_not_move_the_config_fingerprint() {
        let file_only = json!({
            "stepper_z": {"step_pin": "PB0", "position_min": "-2"},
            "adxl345": {"cs_pin": "PB1"},
            "plr": {"probe_method": "adxl_drag", "accel_chip": "adxl345"}
        });
        let file_only = file_only.as_object().unwrap().clone();
        // `settings` view: the same, PLUS klippy's default-injected keys.
        let mut with_defaults = file_only.clone();
        with_defaults.insert(
            "adxl345".to_owned(),
            json!({"cs_pin": "PB1", "axes_map": "x,y,z", "rate": 3200}),
        );
        with_defaults["stepper_z"]["microsteps"] = json!(16);

        let plr = PlrSettings::parse(file_only.get("plr").unwrap().as_object().unwrap()).unwrap();
        // The config (file-only) hash is what the plugin's file-options hash
        // equals — pinned as a shared cross-language constant (asserted in
        // klippy_plugin/tests/test_calibration_meta.py::SHARED_FIXTURES).
        let config_fp = compute_fingerprint(&file_only, &plr, CalGroup::NoiseFloor);
        assert_eq!(config_fp, SHARED_ADXL_NOISE_FLOOR_HEX);
        // Hashing the default-injected `settings` view would DIVERGE — proving
        // why the fingerprint input must be `config`, not `settings`.
        let settings_fp = compute_fingerprint(&with_defaults, &plr, CalGroup::NoiseFloor);
        assert_ne!(settings_fp, config_fp);
    }

    /// Structural guard that the extraction site feeds the fingerprint the
    /// file-only `config`, never `settings`: build a snapshot whose two views
    /// diverge on a fingerprint-relevant key and assert the gating used the
    /// `config` hash (floor kept), not the `settings` hash (which would null
    /// it). This fails if `machine_from_settings` ever passes `settings` to
    /// the fingerprint.
    #[test]
    fn fingerprint_uses_config_not_settings() {
        // Stamp the CONFIG (file-only) fingerprint.
        let (base_snapshot, base_plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
        ]);
        let config_fp = compute_fingerprint(&base_snapshot.config, &base_plr, CalGroup::NoiseFloor);
        let result = query_result(
            configfile_status(&[
                ("probe_method", json!("adxl_drag")),
                ("accel_chip", json!("adxl345")),
                ("noise_floor_rms", json!(118.0)),
                ("cal_fingerprint_noise_floor", json!(config_fp)),
            ]),
            plr_object(),
        );
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        // Inject a default-only key into `settings` (absent from `config`): if
        // the fingerprint ever read `settings`, this would flip it to Invalid.
        snapshot.settings["stepper_z"]["microsteps"] = json!(16);
        snapshot
            .settings
            .insert("adxl345".to_owned(), json!({"axes_map": "x,y,z"}));
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (machine, notes) =
            machine_from_settings(&snapshot.settings, &snapshot.config, &plr, true);
        // config hash matches -> floor kept, no mismatch note.
        assert_eq!(machine.noise_floor, Some(118.0));
        assert!(!notes.iter().any(|n| n.contains("mismatch")), "{notes:?}");
    }

    #[test]
    fn confirm_point_and_accel_keys_default_off_and_parse_when_present() {
        // Absent: every one of them off / None. A machine that never
        // heard of these keys plans exactly as it did before.
        let (_, plr) = parse_fixture(&[]);
        assert!(!plr.confirm_z_before_resume);
        assert!(!plr.debug_confirm_each_step);
        assert!(!plr.unsafe_allow_purge_z_below_bed);
        assert_eq!(plr.confirm_timeout_s, None);
        assert_eq!(plr.gcode_barrier_timeout_s, None);
        for value in [
            plr.recovery_accel,
            plr.accel_home,
            plr.accel_travel,
            plr.accel_probe,
            plr.accel_entry,
        ] {
            assert_eq!(value, None);
        }
        let config = plr.plan_config();
        assert!(!config.confirm_z_before_resume);
        assert!(!config.debug_confirm_each_step);
        assert_eq!(config.recovery_accel, None);

        // Present: carried through to the plan config verbatim (range
        // checking is PlanConfig::validate's job — it refuses rather than
        // clamping, so clamping here would hide the refusal).
        let (_, plr) = parse_fixture(&[
            ("confirm_z_before_resume", json!(true)),
            ("debug_confirm_each_step", json!(true)),
            ("recovery_accel", json!(1500.0)),
            ("accel_home", json!(1000.0)),
            ("accel_travel", json!(3000.0)),
            ("accel_probe", json!(400.0)),
            ("accel_entry", json!(600.0)),
            ("confirm_timeout_s", json!(120.0)),
            ("gcode_barrier_timeout_s", json!(45.0)),
        ]);
        let config = plr.plan_config();
        assert!(config.confirm_z_before_resume);
        assert!(config.debug_confirm_each_step);
        assert_eq!(config.recovery_accel, Some(1500.0));
        assert_eq!(config.accel_home, Some(1000.0));
        assert_eq!(config.accel_travel, Some(3000.0));
        assert_eq!(config.accel_probe, Some(400.0));
        assert_eq!(config.accel_entry, Some(600.0));
        assert_eq!(config.confirm_timeout_s, Some(120.0));
        assert_eq!(config.gcode_barrier_timeout_s, Some(45.0));

        // Out-of-band values are parsed, then REFUSED by validation with
        // the typed diagnosis — never silently clamped into range.
        let (_, plr) = parse_fixture(&[("recovery_accel", json!(2.0))]);
        let error = plr.plan_config().validate().unwrap_err();
        assert!(matches!(
            error,
            plr_recovery::RecoveryError::AccelOutOfRange {
                key: "recovery_accel",
                ..
            }
        ));
    }

    #[test]
    fn unsafe_overrides_are_read_case_insensitively_and_fail_closed() {
        use plr_recovery::UNSAFE_PURGE_Z_BELOW_BED;
        // The operator writes the screaming spelling in printer.cfg;
        // klippy lowercases option names before they reach
        // configfile.settings. Both must be honoured, or a careful edit
        // silently does nothing.
        for key in [
            UNSAFE_PURGE_Z_BELOW_BED,
            &UNSAFE_PURGE_Z_BELOW_BED.to_ascii_lowercase(),
        ] {
            let (_, plr) = parse_fixture(&[(key, json!(true))]);
            assert!(plr.unsafe_allow_purge_z_below_bed, "{key}");
            assert!(plr.plan_config().unsafe_allow_purge_z_below_bed, "{key}");
        }

        // Fail CLOSED: anything that is not literally `true` leaves the
        // refusal in force. A malformed override must never open a gate.
        for bad in [json!(false), json!("true"), json!(1), json!(null)] {
            let (_, plr) = parse_fixture(&[(UNSAFE_PURGE_Z_BELOW_BED, bad.clone())]);
            assert!(
                !plr.unsafe_allow_purge_z_below_bed,
                "{bad} must not enable the override"
            );
        }
    }
}
