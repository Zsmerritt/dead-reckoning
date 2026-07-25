//! Machine-prerequisite validation (design doc §1).
//!
//! Recovery on a moving-bed-Z machine is only safe when the machine
//! satisfies a set of structural prerequisites. This module validates a
//! [`MachineConfig`] snapshot — assembled by the daemon from the
//! Klipper config and operator attestations — and refuses recovery with
//! **every** failed check listed (not just the first), so the operator
//! can fix the machine in one pass.
//!
//! The checks, mapped to the design doc:
//!
//! * `[force_move]` present with `enable_force_move: True`;
//! * self-locking Z leadscrews (operator attestation — software cannot
//!   observe this);
//! * every Z stepper on the primary MCU (multi-MCU Z is refused: the
//!   shifted-frame bound relies on single-MCU step accounting);
//! * slicer `;TYPE:` annotations present (contact-zone selection
//!   refuses to classify geometry without them);
//! * exactly one Tap-style `[probe]` or `[load_cell_probe]` (Klipper
//!   allows a single probe object);
//! * probe `activate_gcode`/`deactivate_gcode` empty or verified
//!   no-move (a moving activate g-code would break the halt-position
//!   arithmetic);
//! * the Z rail's `position_min` known (fallback `[printer]
//!   minimum_z_position`) — it anchors the probe envelope;
//! * config-change detection: the running config hash must equal the
//!   hash the prerequisites were last validated against.

use serde::{Deserialize, Serialize};

/// Which kind of nozzle-contact probe the machine carries.
///
/// Not `Copy`: [`ProbeKind::AdxlDrag`] carries the accelerometer chip
/// name, which is intrinsic probe identity (validation requires it
/// non-empty and command-embeddable), so it lives here rather than as a
/// separate optional field that could drift out of sync with the kind.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProbeKind {
    /// Tap-style `[probe]` (nozzle-actuated switch).
    Tap,
    /// `[load_cell_probe]`.
    LoadCell,
    /// ADXL drag probing: the nozzle is dragged across the part in
    /// fixed-Z XY passes while an accelerometer listens for contact
    /// (`PLR_DRAG_PROBE`). Requires a calibrated noise floor
    /// ([`MachineConfig::noise_floor`], autosaved by `PLR_NOISE_TEST`).
    AdxlDrag {
        /// The accelerometer chip name (the `[plr]` `accel_chip`
        /// setting), embedded double-quoted in the `PLR_DRAG_PROBE`
        /// command (`CHIP="<chip>"` — spaced section names such as
        /// `adxl345 bed` are supported; see `chip_embeddable`).
        chip: String,
    },
}

/// One probe object from the Klipper config.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeConfig {
    /// Probe kind.
    pub kind: ProbeKind,
    /// Configured `z_offset`, mm. For nozzle-as-stylus recovery this
    /// must be handled explicitly per probe type (see
    /// [`crate::plan::TriggerSource`]).
    pub z_offset: f64,
    /// `true` when `activate_gcode` is empty or has been verified to
    /// command no motion.
    pub activate_gcode_no_move: bool,
    /// `true` when `deactivate_gcode` is empty or has been verified to
    /// command no motion.
    pub deactivate_gcode_no_move: bool,
}

/// Known travel limits of the printer's axes, used by the
/// whole-itinerary pre-flight ([`crate::preflight`]) to reject a plan
/// that would command a coordinate outside the machine. Every field is
/// optional: the legacy `/etc/plrd.conf [machine]` path knows none of
/// them (its checks are skipped — "where known"), while the `[plr]`
/// live-config path reads them from the Klipper stepper sections. `x`
/// and `y` are `(min, max)` pairs; `z_max` complements the Z rail's
/// `position_min` (already carried separately as the envelope anchor).
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct AxisLimits {
    /// `(position_min, position_max)` of the X axis, mm.
    pub x: Option<(f64, f64)>,
    /// `(position_min, position_max)` of the Y axis, mm.
    pub y: Option<(f64, f64)>,
    /// The Z rail's `position_max`, mm.
    pub z_max: Option<f64>,
}

/// One Z stepper and the MCU its step pin lives on.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ZStepper {
    /// Config section name, e.g. `"stepper_z"`, `"stepper_z1"`.
    pub name: String,
    /// MCU name the stepper is wired to (`"mcu"` for the primary).
    pub mcu: String,
}

/// Snapshot of everything prerequisite validation needs, assembled by
/// the daemon from the Klipper config and operator attestations.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachineConfig {
    /// `[force_move]` present with `enable_force_move: True`.
    pub force_move_enabled: bool,
    /// Operator attestation that the Z leadscrews are self-locking
    /// (the bed cannot back-drive under gravity when unpowered).
    pub z_self_locking_attested: bool,
    /// Every Z stepper with its MCU.
    pub z_steppers: Vec<ZStepper>,
    /// Name of the primary MCU (usually `"mcu"`).
    pub primary_mcu: String,
    /// `true` when the sliced file carries `;TYPE:` annotations.
    pub type_annotations_present: bool,
    /// Every probe object in the config. Must be exactly one.
    pub probes: Vec<ProbeConfig>,
    /// The Z rail's `position_min` (fallback `[printer]
    /// minimum_z_position`), mm. `None` when neither is configured.
    pub z_position_min: Option<f64>,
    /// Hash of the running Klipper config.
    ///
    /// In `[plr]` mode (machine config sourced live from Klipper's
    /// `configfile` settings) the change-detection role of this hash is
    /// satisfied by construction — the values *are* the running config,
    /// every run — so the daemon sets `config_hash` and
    /// `validated_config_hash` to the same sentinel and this check
    /// passes trivially. The field is kept so the legacy
    /// `/etc/plrd.conf [machine]` path retains real change detection.
    pub config_hash: String,
    /// Hash of the config these prerequisites were last validated
    /// against; `None` when never validated.
    pub validated_config_hash: Option<String>,
    /// Root directory of `[virtual_sdcard]`; `None` when unknown.
    pub virtual_sdcard_root: Option<String>,
    /// Calibrated ADXL noise floor (the representative RMS the
    /// plugin's `PLR_NOISE_TEST` autosaves into `[plr]` as
    /// `noise_floor_rms` / `noise_floor_still_rms` / `noise_floor_peak`).
    /// Required — finite and positive — when the probe kind is
    /// [`ProbeKind::AdxlDrag`]; ignored otherwise.
    ///
    /// A **measurement** only. The `noise_floor_*` options that merely
    /// describe a calibration — the speed and temperature it was taken at,
    /// and the temperature sensor's name — must never land here: they are
    /// classified out where the `[plr]` section is parsed
    /// (`crates/plrd/src/plrcfg.rs`, `NOISE_FLOOR_METADATA_KEYS`).
    /// Nothing at this layer can tell the difference — a recorded 40 °C is
    /// a perfectly valid noise-floor number — so metadata reaching this
    /// field silently retires the [`PrereqFailure::NoiseFloorMissing`]
    /// refusal instead of tripping anything
    /// (`a_metadata_only_calibration_must_arrive_as_missing` pins the
    /// consequence).
    #[serde(default)]
    pub noise_floor: Option<f64>,
    /// Drag speed the noise floor was measured at, mm/s (the OPTIONAL
    /// `[plr]` `noise_floor_speed` autosave, staged by the plugin's
    /// `PLR_NOISE_TEST` alongside the `noise_floor_*` measurements —
    /// `klippy_plugin/plr/noise_test.py`). Metadata, so it is carried
    /// separately from [`Self::noise_floor`] and never substitutes for
    /// it. The noise floor is
    /// speed-specific, so when this is present and differs from the
    /// plan's `drag_speed` by more than 20% the plan carries
    /// [`crate::plan::PlanWarning::NoiseFloorSpeedMismatch`] — a
    /// warning, never a refusal. `None` — a calibration from before
    /// the key existed — checks nothing (tolerant back-compat until
    /// the operator re-runs `PLR_NOISE_TEST`).
    #[serde(default)]
    pub noise_floor_speed: Option<f64>,
    /// Known axis travel limits for the whole-itinerary pre-flight
    /// ([`crate::preflight`]). Default (all `None`) is the honest
    /// "unknown" the legacy path carries; the `[plr]` path fills in what
    /// the Klipper stepper sections expose.
    #[serde(default)]
    pub axis_limits: AxisLimits,
    /// The machine's own configured `max_accel` (`[printer] max_accel`),
    /// mm/s². `None` when unknown — the legacy `/etc/plrd.conf [machine]`
    /// path cannot see the running Klipper config.
    ///
    /// Read only so the generated recovery file can RESTORE it as a
    /// literal after clamping its entry moves to `accel_entry`: that file
    /// has no runtime-placeholder machinery, so a clamp it cannot undo
    /// would outlive the recovery and govern the entire remaining print.
    #[serde(default)]
    pub max_accel: Option<f64>,
}

/// One failed prerequisite check.
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize, Deserialize)]
pub enum PrereqFailure {
    /// `[force_move]` missing or `enable_force_move` not `True`.
    #[error("[force_move] with enable_force_move: True is required")]
    ForceMoveDisabled,
    /// The operator has not attested self-locking Z leadscrews.
    #[error("self-locking Z leadscrews are not attested")]
    ZNotSelfLocking,
    /// No Z steppers were listed at all.
    #[error("no Z steppers in the machine snapshot")]
    NoZSteppers,
    /// A Z stepper lives on a secondary MCU.
    #[error("Z stepper {stepper} is on MCU {mcu}, not the primary MCU")]
    ZStepperOffPrimaryMcu {
        /// The offending stepper.
        stepper: String,
        /// The MCU it is wired to.
        mcu: String,
    },
    /// The sliced file carries no `;TYPE:` annotations.
    #[error("slicer ;TYPE: annotations are required")]
    NoTypeAnnotations,
    /// No Tap-style `[probe]` / `[load_cell_probe]` configured.
    #[error("a Tap-style [probe] or [load_cell_probe] is required")]
    NoProbe,
    /// More than one probe object was listed (Klipper allows one; a
    /// multi-probe snapshot is inconsistent and refused).
    #[error("{count} probe objects listed; exactly one is required")]
    MultipleProbes {
        /// How many probes were listed.
        count: usize,
    },
    /// The probe `z_offset` was NaN or infinite.
    #[error("probe z_offset is not finite")]
    ProbeZOffsetNonFinite,
    /// `activate_gcode` is neither empty nor verified no-move.
    #[error("probe activate_gcode is not verified move-free")]
    ProbeActivateGcodeMoves,
    /// `deactivate_gcode` is neither empty nor verified no-move.
    #[error("probe deactivate_gcode is not verified move-free")]
    ProbeDeactivateGcodeMoves,
    /// Neither the Z rail's `position_min` nor `[printer]
    /// minimum_z_position` is known.
    #[error("Z position_min (or [printer] minimum_z_position) is unknown")]
    PositionMinUnknown,
    /// `position_min` was NaN or infinite.
    #[error("Z position_min is not finite")]
    PositionMinNonFinite,
    /// The prerequisites were never validated against any config.
    #[error("machine prerequisites have never been validated")]
    ConfigNeverValidated,
    /// The running config hash differs from the validated one:
    /// re-validation is required before recovery.
    #[error("config changed since validation (validated {validated}, running {current})")]
    ConfigChangedSinceValidation {
        /// Hash the prerequisites were validated against.
        validated: String,
        /// Hash of the running config.
        current: String,
    },
    /// The `[virtual_sdcard]` root is unknown; the `M23` top-level
    /// check cannot run.
    #[error("[virtual_sdcard] root directory is unknown")]
    SdcardRootUnknown,
    /// The ADXL drag probe has no usable accelerometer chip name:
    /// empty, or containing characters that cannot survive the quoted
    /// `PLR_DRAG_PROBE CHIP="<chip>"` embedding (double quotes,
    /// backslashes, control characters — see `chip_embeddable`).
    #[error(
        "accel_chip {chip:?} is empty or cannot be embedded in a quoted \
         command argument (double quotes, backslashes, and control \
         characters are not representable)"
    )]
    AccelChipInvalid {
        /// The offending chip name.
        chip: String,
    },
    /// ADXL drag probing without a calibrated noise floor is refused:
    /// contact detection thresholds against it.
    #[error("ADXL noise floor is not calibrated; run PLR_NOISE_TEST first")]
    NoiseFloorMissing,
    /// The calibrated noise floor is non-finite or non-positive.
    #[error("ADXL noise floor {value} is not a finite positive number; re-run PLR_NOISE_TEST")]
    NoiseFloorInvalid {
        /// The rejected value.
        value: f64,
    },
}

/// The values recovery planning actually consumes, extracted from a
/// [`MachineConfig`] that passed every check. Obtain via
/// [`validate_machine`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ValidatedMachine {
    /// The single configured probe.
    pub probe: ProbeConfig,
    /// The Z rail's `position_min`, mm (finite).
    pub z_position_min: f64,
    /// Names of every Z stepper (all on the primary MCU).
    pub z_stepper_names: Vec<String>,
    /// Root directory of `[virtual_sdcard]`.
    pub sdcard_root: String,
    /// Known axis travel limits (carried through for the
    /// whole-itinerary pre-flight; all `None` when unknown).
    pub axis_limits: AxisLimits,
    /// The machine's own `[printer] max_accel`, mm/s², when known and
    /// usable (finite and positive). See
    /// [`MachineConfig::max_accel`].
    pub max_accel: Option<f64>,
}

/// All prerequisite failures of one validation pass.
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize, Deserialize)]
#[error("machine prerequisites failed: {} check(s) failed", failures.len())]
pub struct MachineRejection {
    /// Every failed check.
    pub failures: Vec<PrereqFailure>,
}

/// `true` when a chip name can be embedded as a **quoted** command
/// argument (`PLR_DRAG_PROBE CHIP="<chip>"`).
///
/// The plan always emits the CHIP value double-quoted: klippy's
/// extended-command parser handles quoted values on every ingress path
/// plrd uses (`klippy/gcode.py`, `_get_extended_params` at 145-151
/// dispatches lines containing a quote to shlex, and 266-281 parses
/// them posix-style — console, macros, and the `run_script` API path
/// all arrive intact). Spaced section names like `adxl345 bed`
/// (Klipper's `[adxl345 bed]` chips) are therefore fully supported,
/// and quoting a space-free name is equally valid.
///
/// What quoting canNOT carry, and is refused here:
///
/// * a **double quote** in the name — it terminates the shlex quoted
///   region, so the value cannot round-trip;
/// * a **backslash** — inside posix double quotes shlex treats it as
///   an escape character, so the name would arrive altered;
/// * **control characters** — a newline would break the daemon's
///   line-framed transcript/command stream before klippy ever saw it;
/// * the empty string.
fn chip_embeddable(chip: &str) -> bool {
    !chip.is_empty()
        && !chip
            .chars()
            .any(|c| c.is_control() || matches!(c, '"' | '\\'))
}

/// Validates every machine prerequisite (design doc §1), collecting
/// **all** failures.
///
/// # Errors
///
/// [`MachineRejection`] listing every failed check.
pub fn validate_machine(config: &MachineConfig) -> Result<ValidatedMachine, MachineRejection> {
    let mut failures = Vec::new();

    if !config.force_move_enabled {
        failures.push(PrereqFailure::ForceMoveDisabled);
    }
    if !config.z_self_locking_attested {
        failures.push(PrereqFailure::ZNotSelfLocking);
    }
    if config.z_steppers.is_empty() {
        failures.push(PrereqFailure::NoZSteppers);
    }
    for stepper in &config.z_steppers {
        if stepper.mcu != config.primary_mcu {
            failures.push(PrereqFailure::ZStepperOffPrimaryMcu {
                stepper: stepper.name.clone(),
                mcu: stepper.mcu.clone(),
            });
        }
    }
    if !config.type_annotations_present {
        failures.push(PrereqFailure::NoTypeAnnotations);
    }
    match config.probes.len() {
        0 => failures.push(PrereqFailure::NoProbe),
        1 => {}
        count => failures.push(PrereqFailure::MultipleProbes { count }),
    }
    if let Some(probe) = config.probes.first() {
        if !probe.z_offset.is_finite() {
            failures.push(PrereqFailure::ProbeZOffsetNonFinite);
        }
        if !probe.activate_gcode_no_move {
            failures.push(PrereqFailure::ProbeActivateGcodeMoves);
        }
        if !probe.deactivate_gcode_no_move {
            failures.push(PrereqFailure::ProbeDeactivateGcodeMoves);
        }
        if let ProbeKind::AdxlDrag { chip } = &probe.kind {
            if !chip_embeddable(chip) {
                failures.push(PrereqFailure::AccelChipInvalid { chip: chip.clone() });
            }
            match config.noise_floor {
                None => failures.push(PrereqFailure::NoiseFloorMissing),
                Some(value) if !(value.is_finite() && value > 0.0) => {
                    failures.push(PrereqFailure::NoiseFloorInvalid { value });
                }
                Some(_) => {}
            }
        }
    }
    match config.z_position_min {
        None => failures.push(PrereqFailure::PositionMinUnknown),
        Some(v) if !v.is_finite() => failures.push(PrereqFailure::PositionMinNonFinite),
        Some(_) => {}
    }
    match config.validated_config_hash.as_deref() {
        None => failures.push(PrereqFailure::ConfigNeverValidated),
        Some(validated) if validated != config.config_hash => {
            failures.push(PrereqFailure::ConfigChangedSinceValidation {
                validated: validated.to_owned(),
                current: config.config_hash.clone(),
            });
        }
        Some(_) => {}
    }
    if config.virtual_sdcard_root.is_none() {
        failures.push(PrereqFailure::SdcardRootUnknown);
    }

    if !failures.is_empty() {
        return Err(MachineRejection { failures });
    }
    // All `unwrap_or` fallbacks below are unreachable (the checks above
    // guarantee presence) but keep this path panic-free by construction.
    Ok(ValidatedMachine {
        probe: config.probes.first().cloned().unwrap_or(ProbeConfig {
            kind: ProbeKind::Tap,
            z_offset: 0.0,
            activate_gcode_no_move: true,
            deactivate_gcode_no_move: true,
        }),
        z_position_min: config.z_position_min.unwrap_or(0.0),
        z_stepper_names: config.z_steppers.iter().map(|s| s.name.clone()).collect(),
        sdcard_root: config.virtual_sdcard_root.clone().unwrap_or_default(),
        axis_limits: config.axis_limits,
        // Carried through as-is: a missing max_accel is not a
        // prerequisite failure, it only costs the file-level entry clamp
        // (which the plan warns about).
        max_accel: config.max_accel.filter(|v| v.is_finite() && *v > 0.0),
    })
}

#[cfg(test)]
mod tests {
    use super::{validate_machine, MachineConfig, PrereqFailure, ProbeConfig, ProbeKind, ZStepper};

    /// A config that passes every check.
    pub(crate) fn good_config() -> MachineConfig {
        MachineConfig {
            force_move_enabled: true,
            z_self_locking_attested: true,
            z_steppers: vec![
                ZStepper {
                    name: "stepper_z".to_owned(),
                    mcu: "mcu".to_owned(),
                },
                ZStepper {
                    name: "stepper_z1".to_owned(),
                    mcu: "mcu".to_owned(),
                },
            ],
            primary_mcu: "mcu".to_owned(),
            type_annotations_present: true,
            probes: vec![ProbeConfig {
                kind: ProbeKind::Tap,
                z_offset: -0.1,
                activate_gcode_no_move: true,
                deactivate_gcode_no_move: true,
            }],
            z_position_min: Some(-2.0),
            config_hash: "abc".to_owned(),
            validated_config_hash: Some("abc".to_owned()),
            virtual_sdcard_root: Some("/home/pi/gcodes".to_owned()),
            noise_floor: None,
            noise_floor_speed: None,
            axis_limits: super::AxisLimits::default(),
            max_accel: Some(3_000.0),
        }
    }

    #[test]
    fn good_config_validates_and_extracts() {
        let v = validate_machine(&good_config()).unwrap();
        assert_eq!(v.z_stepper_names, vec!["stepper_z", "stepper_z1"]);
        assert!((v.z_position_min - (-2.0)).abs() < 1e-12);
        assert_eq!(v.probe.kind, ProbeKind::Tap);
        assert_eq!(v.sdcard_root, "/home/pi/gcodes");
    }

    #[test]
    fn every_failure_is_collected_not_just_the_first() {
        let config = MachineConfig {
            force_move_enabled: false,
            z_self_locking_attested: false,
            z_steppers: vec![],
            primary_mcu: "mcu".to_owned(),
            type_annotations_present: false,
            probes: vec![],
            z_position_min: None,
            config_hash: "abc".to_owned(),
            validated_config_hash: None,
            virtual_sdcard_root: None,
            noise_floor: None,
            noise_floor_speed: None,
            axis_limits: super::AxisLimits::default(),
            max_accel: None,
        };
        let rejection = validate_machine(&config).unwrap_err();
        let f = &rejection.failures;
        assert!(f.contains(&PrereqFailure::ForceMoveDisabled));
        assert!(f.contains(&PrereqFailure::ZNotSelfLocking));
        assert!(f.contains(&PrereqFailure::NoZSteppers));
        assert!(f.contains(&PrereqFailure::NoTypeAnnotations));
        assert!(f.contains(&PrereqFailure::NoProbe));
        assert!(f.contains(&PrereqFailure::PositionMinUnknown));
        assert!(f.contains(&PrereqFailure::ConfigNeverValidated));
        assert!(f.contains(&PrereqFailure::SdcardRootUnknown));
        assert_eq!(f.len(), 8);
        assert!(rejection.to_string().contains("8 check(s)"));
    }

    #[test]
    fn secondary_mcu_z_stepper_is_refused() {
        let mut config = good_config();
        config.z_steppers[1].mcu = "mcu2".to_owned();
        let rejection = validate_machine(&config).unwrap_err();
        assert_eq!(
            rejection.failures,
            vec![PrereqFailure::ZStepperOffPrimaryMcu {
                stepper: "stepper_z1".to_owned(),
                mcu: "mcu2".to_owned(),
            }]
        );
    }

    #[test]
    fn multiple_probes_are_refused() {
        let mut config = good_config();
        config.probes.push(ProbeConfig {
            kind: ProbeKind::LoadCell,
            z_offset: 0.0,
            activate_gcode_no_move: true,
            deactivate_gcode_no_move: true,
        });
        let rejection = validate_machine(&config).unwrap_err();
        assert_eq!(
            rejection.failures,
            vec![PrereqFailure::MultipleProbes { count: 2 }]
        );
    }

    #[test]
    fn moving_probe_gcode_and_bad_offset_are_refused() {
        let mut config = good_config();
        config.probes[0].z_offset = f64::NAN;
        config.probes[0].activate_gcode_no_move = false;
        config.probes[0].deactivate_gcode_no_move = false;
        let rejection = validate_machine(&config).unwrap_err();
        assert_eq!(
            rejection.failures,
            vec![
                PrereqFailure::ProbeZOffsetNonFinite,
                PrereqFailure::ProbeActivateGcodeMoves,
                PrereqFailure::ProbeDeactivateGcodeMoves,
            ]
        );
    }

    #[test]
    fn config_hash_mismatch_requires_revalidation() {
        let mut config = good_config();
        config.validated_config_hash = Some("old".to_owned());
        let rejection = validate_machine(&config).unwrap_err();
        assert_eq!(
            rejection.failures,
            vec![PrereqFailure::ConfigChangedSinceValidation {
                validated: "old".to_owned(),
                current: "abc".to_owned(),
            }]
        );
    }

    #[test]
    fn non_finite_position_min_is_refused() {
        let mut config = good_config();
        config.z_position_min = Some(f64::INFINITY);
        let rejection = validate_machine(&config).unwrap_err();
        assert_eq!(
            rejection.failures,
            vec![PrereqFailure::PositionMinNonFinite]
        );
    }

    /// A drag config: the good config with the probe swapped for an
    /// ADXL drag probe and a calibrated noise floor.
    fn drag_config() -> MachineConfig {
        let mut config = good_config();
        config.probes = vec![ProbeConfig {
            kind: ProbeKind::AdxlDrag {
                chip: "adxl345".to_owned(),
            },
            z_offset: 0.0,
            activate_gcode_no_move: true,
            deactivate_gcode_no_move: true,
        }];
        config.noise_floor = Some(120.0);
        config
    }

    #[test]
    fn calibrated_drag_config_validates() {
        let v = validate_machine(&drag_config()).unwrap();
        assert_eq!(
            v.probe.kind,
            ProbeKind::AdxlDrag {
                chip: "adxl345".to_owned()
            }
        );
    }

    #[test]
    fn drag_without_noise_floor_demands_plr_noise_test() {
        let mut config = drag_config();
        config.noise_floor = None;
        let rejection = validate_machine(&config).unwrap_err();
        assert_eq!(rejection.failures, vec![PrereqFailure::NoiseFloorMissing]);
        assert!(
            rejection.failures[0]
                .to_string()
                .contains("run PLR_NOISE_TEST first"),
            "{}",
            rejection.failures[0]
        );
    }

    /// Why the measurement/metadata split has to happen upstream.
    ///
    /// A machine whose `[plr]` section records only calibration METADATA
    /// (the speed, the temperature, the sensor name) has never measured a
    /// floor, so it must arrive here as `None` and get the
    /// "run `PLR_NOISE_TEST` first" refusal. This layer cannot enforce that
    /// itself: the second half shows a plausible temperature (40 °C)
    /// validating as a noise floor, because as a NUMBER it is entirely
    /// legitimate. Classification is therefore the only guard, and it
    /// lives at the parse boundary (`crates/plrd/src/plrcfg.rs`,
    /// `NOISE_FLOOR_METADATA_KEYS`).
    #[test]
    fn a_metadata_only_calibration_must_arrive_as_missing() {
        let mut config = drag_config();
        // Metadata recorded, no measurement: the honest shape.
        config.noise_floor = None;
        config.noise_floor_speed = Some(20.0);
        let rejection = validate_machine(&config).unwrap_err();
        assert_eq!(rejection.failures, vec![PrereqFailure::NoiseFloorMissing]);
        // The shape that would have slipped through unnoticed.
        config.noise_floor = Some(40.0);
        assert!(validate_machine(&config).is_ok());
    }

    #[test]
    fn drag_with_invalid_noise_floor_is_refused() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut config = drag_config();
            config.noise_floor = Some(bad);
            let rejection = validate_machine(&config).unwrap_err();
            assert!(
                matches!(
                    rejection.failures.as_slice(),
                    [PrereqFailure::NoiseFloorInvalid { .. }]
                ),
                "noise floor {bad}: {rejection:?}"
            );
        }
    }

    #[test]
    fn drag_chip_must_be_quote_embeddable() {
        // Only names the quoted CHIP="..." transport cannot carry are
        // refused: double quotes, backslashes, control chars, empty.
        for bad in ["", "a\"b", "a\\b", "a\nb", "a\tb\x07"] {
            let mut config = drag_config();
            config.probes[0].kind = ProbeKind::AdxlDrag {
                chip: bad.to_owned(),
            };
            let rejection = validate_machine(&config).unwrap_err();
            assert!(
                rejection
                    .failures
                    .iter()
                    .any(|f| matches!(f, PrereqFailure::AccelChipInvalid { .. })),
                "chip {bad:?}: {rejection:?}"
            );
        }
        // Bare names AND spaced Klipper section names pass (klippy's
        // shlex path parses the quoted value; see chip_embeddable).
        for ok in [
            "adxl345",
            "lis2dw",
            "adxl345_bed",
            "adxl345 bed",
            "a'b",
            "a=b",
        ] {
            let mut config = drag_config();
            config.probes[0].kind = ProbeKind::AdxlDrag {
                chip: ok.to_owned(),
            };
            assert!(validate_machine(&config).is_ok(), "chip {ok:?}");
        }
    }

    #[test]
    fn noise_floor_is_irrelevant_to_contact_probes() {
        // A tap machine with no noise floor stays valid: the field
        // gates only the drag method.
        let config = good_config();
        assert!(config.noise_floor.is_none());
        assert!(validate_machine(&config).is_ok());
    }

    #[test]
    fn failures_render_and_serialize() {
        let f = PrereqFailure::ZStepperOffPrimaryMcu {
            stepper: "stepper_z1".to_owned(),
            mcu: "mcu2".to_owned(),
        };
        assert!(f.to_string().contains("stepper_z1"));
        let json = serde_json::to_string(&f).unwrap();
        let back: PrereqFailure = serde_json::from_str(&json).unwrap();
        assert_eq!(back, f);
    }
}
