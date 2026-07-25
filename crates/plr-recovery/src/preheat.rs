//! Preheat-target derivation (design doc §8, step 2): WAL heater/fan
//! targets, cross-checked against a backward temperature scan of the
//! print file.
//!
//! The WAL context records the heater targets that were actually active
//! at capture time — the primary source. The file scan is the fallback:
//! the *last* `M104`/`M109` (nozzle) and `M140`/`M190` (bed) at or
//! before the stop offset. "Backward scan from the stop offset" is
//! implemented as a forward pass keeping the last value seen before the
//! offset — identical result, and it reuses the byte-exact
//! [`plr_gcode::LineIter`] parser.

use plr_gcode::LineIter;
use plr_wal::Context;
use serde::{Deserialize, Serialize};

/// Temperature targets recovered from the print file.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct FileTemps {
    /// Last `M104`/`M109` S (or R) value before the stop offset, °C.
    pub nozzle: Option<f64>,
    /// Last `M140`/`M190` S (or R) value before the stop offset, °C.
    pub bed: Option<f64>,
}

/// Scans `bytes` (a window of the print file whose first byte sits at
/// stream offset `base_offset`) for the last nozzle/bed temperature
/// command strictly before `stop_offset`.
///
/// Non-finite or non-positive values are ignored (a hostile file cannot
/// inject NaN into the plan); `M104 S0` style "heater off" commands
/// therefore also clear nothing — the WAL remains the authority for
/// "off". Total: never panics on any input bytes.
#[must_use]
pub fn scan_file_temps(bytes: &[u8], base_offset: u64, stop_offset: u64) -> FileTemps {
    let mut temps = FileTemps::default();
    for line in LineIter::new(bytes, base_offset) {
        if line.span.start >= stop_offset {
            break;
        }
        let Some(command) = line.command() else {
            continue;
        };
        let value = command
            .get("S")
            .or_else(|| command.get("R"))
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|v| v.is_finite() && *v > 0.0);
        let Some(value) = value else { continue };
        match command.name.as_str() {
            "M104" | "M109" => temps.nozzle = Some(value),
            "M140" | "M190" => temps.bed = Some(value),
            _ => {}
        }
    }
    temps
}

/// Merged preheat targets: WAL first, file-scan fallback.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PreheatTargets {
    /// Print nozzle target, °C (`None` when neither source knows one).
    pub nozzle: Option<f64>,
    /// Bed target, °C.
    pub bed: Option<f64>,
    /// Heaters other than `extruder`/`heater_bed` with a positive
    /// target, `(name, °C)`, from the WAL only.
    pub other_heaters: Vec<(String, f64)>,
    /// Fan targets `(name, speed in [0, 1])`, from the WAL only.
    pub fans: Vec<(String, f64)>,
}

/// `Some(target)` when finite and positive.
fn usable(target: f64) -> Option<f64> {
    (target.is_finite() && target > 0.0).then_some(target)
}

/// Derives the preheat targets from the WAL context, falling back to
/// the file scan where the WAL is silent. Non-finite WAL values are
/// discarded (never propagated into a plan).
#[must_use]
pub fn derive_preheat(context: &Context, file: &FileTemps) -> PreheatTargets {
    let mut nozzle = None;
    let mut bed = None;
    let mut other_heaters = Vec::new();
    for heater in &context.heaters {
        let Some(target) = usable(heater.target) else {
            continue;
        };
        match heater.name.as_str() {
            "extruder" => nozzle = Some(target),
            "heater_bed" => bed = Some(target),
            _ => other_heaters.push((heater.name.clone(), target)),
        }
    }
    let fans = context
        .fans
        .iter()
        .filter(|f| f.speed.is_finite() && f.speed > 0.0)
        .map(|f| (f.name.clone(), f.speed.clamp(0.0, 1.0)))
        .collect();
    PreheatTargets {
        nozzle: nozzle.or_else(|| file.nozzle.and_then(usable)),
        bed: bed.or_else(|| file.bed.and_then(usable)),
        other_heaters,
        fans,
    }
}

#[cfg(test)]
mod tests {
    use plr_wal::{Context, FanTarget, GcodeState, HeaterTarget, TransformObservations};

    use super::{derive_preheat, scan_file_temps, FileTemps};

    fn context(heaters: Vec<HeaterTarget>, fans: Vec<FanTarget>) -> Context {
        Context {
            mono_ns: 0,
            virtual_sdcard: None,
            gcode: GcodeState {
                speed_factor: 1.0,
                speed: 3000.0,
                extrude_factor: 1.0,
                absolute_coordinates: true,
                absolute_extrude: true,
                homing_origin: vec![0.0; 4],
                position: vec![0.0; 4],
                gcode_position: vec![0.0; 4],
            },
            transforms: TransformObservations {
                bed_mesh_active: false,
                bed_mesh_profile: None,
                z_thermal_adjust_enabled: None,
                z_thermal_adjust_offset: None,
                skew_active: false,
                skew_profile: None,
            },
            heaters,
            fans,
            exclude: None,
            print_state: None,
        }
    }

    #[test]
    fn file_scan_keeps_the_last_value_before_the_stop() {
        let file = b"M104 S200\nM140 S60\nG1 X1\nM104 S215\nM109 S230\nG1 X2\nM104 S250\n";
        // Stop before the final M104 S250: nozzle is the M109's 230.
        let stop = file.len() as u64 - "M104 S250\n".len() as u64;
        let temps = scan_file_temps(file, 0, stop);
        assert_eq!(temps.nozzle, Some(230.0));
        assert_eq!(temps.bed, Some(60.0));
        // Stop at 0: nothing seen.
        let temps = scan_file_temps(file, 0, 0);
        assert_eq!(temps, FileTemps::default());
    }

    #[test]
    fn file_scan_respects_base_offset_and_r_param() {
        let bytes = b"M190 R55\nM104 S210\n";
        let temps = scan_file_temps(bytes, 1000, 2000);
        assert_eq!(temps.bed, Some(55.0));
        assert_eq!(temps.nozzle, Some(210.0));
        // stop_offset below the window start sees nothing.
        let temps = scan_file_temps(bytes, 1000, 1000);
        assert_eq!(temps, FileTemps::default());
    }

    #[test]
    fn file_scan_discards_hostile_and_off_values() {
        let file = b"M104 SNaN\nM104 Sinf\nM104 S0\nM140 S-5\nM104 S\nM104\n";
        let temps = scan_file_temps(file, 0, u64::MAX);
        assert_eq!(temps, FileTemps::default());
    }

    #[test]
    fn wal_targets_win_over_file_scan() {
        let ctx = context(
            vec![
                HeaterTarget {
                    name: "extruder".to_owned(),
                    target: 205.0,
                },
                HeaterTarget {
                    name: "heater_bed".to_owned(),
                    target: 61.0,
                },
                HeaterTarget {
                    name: "heater_generic chamber".to_owned(),
                    target: 40.0,
                },
            ],
            vec![FanTarget {
                name: "fan".to_owned(),
                speed: 0.8,
            }],
        );
        let file = FileTemps {
            nozzle: Some(250.0),
            bed: Some(90.0),
        };
        let targets = derive_preheat(&ctx, &file);
        assert_eq!(targets.nozzle, Some(205.0));
        assert_eq!(targets.bed, Some(61.0));
        assert_eq!(
            targets.other_heaters,
            vec![("heater_generic chamber".to_owned(), 40.0)]
        );
        assert_eq!(targets.fans, vec![("fan".to_owned(), 0.8)]);
    }

    #[test]
    fn file_scan_fills_wal_silence_and_hostile_wal_is_discarded() {
        let ctx = context(
            vec![HeaterTarget {
                name: "extruder".to_owned(),
                target: f64::NAN,
            }],
            vec![FanTarget {
                name: "fan".to_owned(),
                speed: f64::INFINITY,
            }],
        );
        let file = FileTemps {
            nozzle: Some(210.0),
            bed: None,
        };
        let targets = derive_preheat(&ctx, &file);
        assert_eq!(targets.nozzle, Some(210.0));
        assert_eq!(targets.bed, None);
        assert!(targets.fans.is_empty());
    }

    #[test]
    fn zero_targets_mean_off() {
        let ctx = context(
            vec![HeaterTarget {
                name: "extruder".to_owned(),
                target: 0.0,
            }],
            vec![FanTarget {
                name: "fan".to_owned(),
                speed: 0.0,
            }],
        );
        let targets = derive_preheat(&ctx, &FileTemps::default());
        assert_eq!(targets.nozzle, None);
        assert!(targets.fans.is_empty());
    }
}
