//! Shared scenario fixtures for the integration tests: a synthetic
//! two-layer print, a WAL context, a possible-stop set and a machine
//! snapshot, all built through the sibling crates' public APIs.
#![allow(dead_code)] // each test binary uses a subset

use plr_analyzer::{
    build_layer_model, ContactOutcome, FeatureClass, LayerModel, MatchConfidence, MatchResult,
    ModelConfig, ProbeCandidate,
};
use plr_gcode::ByteSpan;
use plr_klipper::ClockCorrelator;
use plr_reconstruct::{
    CrashClass, Degradation, ExclusionReport, Interval, PossibleStopSet, Provenance,
    Reconstruction, RecoveryReconstruction, StopWindow, TbSource, WalTimeline, ZCandidate, ZKind,
};
use plr_recovery::{MachineConfig, ProbeConfig, ProbeKind, ZStepper};
use plr_wal::{
    Context, FanTarget, GcodeState, HeaterTarget, ScanEnd, TransformObservations, VirtualSdState,
};

/// The synthetic two-layer print used by every scenario. Layer 0 at
/// Z 0.2, layer 1 at Z 0.4, all deposition annotated as internal
/// infill.
pub const MODEL_TEXT: &str = "\
G90
M83
G1 Z0.2 F7200
;TYPE:Internal infill
G1 X10 Y10 E1 F1800
G1 X30 Y10 E1
G1 X30 Y30 E1
G1 Z0.4 F7200
;TYPE:Internal infill
G1 X10 Y10 E1 F1800
G1 X30 Y10 E1
G1 X30 Y30 E1
";

/// Byte offset of the `n`-th (0-based) occurrence of `needle` in
/// [`MODEL_TEXT`].
pub fn offset_of(needle: &str, n: usize) -> u64 {
    let mut from = 0usize;
    let mut count = 0usize;
    while let Some(pos) = MODEL_TEXT[from..].find(needle) {
        let abs = from + pos;
        if count == n {
            return abs as u64;
        }
        count += 1;
        from = abs + needle.len();
    }
    panic!("{needle:?} occurrence {n} not found in MODEL_TEXT");
}

/// The layer model of [`MODEL_TEXT`].
pub fn model() -> LayerModel {
    build_layer_model(
        plr_gcode::GcodeState::new(),
        MODEL_TEXT.as_bytes(),
        0,
        &ModelConfig::default(),
    )
}

/// A unique-line match at the given byte offset.
pub fn match_at(offset: u64) -> MatchResult {
    MatchResult {
        candidates: vec![],
        confidence: MatchConfidence::UniqueLine { offset },
        skipped_unknown: 0,
    }
}

/// A single ranked probe candidate on layer N−1 plastic at `z`.
pub fn contact_at(z: f64) -> ContactOutcome {
    ContactOutcome::Candidates(vec![ProbeCandidate {
        point: [20.0, 10.0],
        z,
        class: FeatureClass::InternalInfill,
        host_span: ByteSpan { start: 0, end: 1 },
        host_length: 20.0,
        distance_from_crash: 15.0,
        sample_t: 0.5,
    }])
}

/// Default transform observations: no mesh, no skew, no
/// `z_thermal_adjust`.
pub fn plain_transforms() -> TransformObservations {
    TransformObservations {
        bed_mesh_active: false,
        bed_mesh_profile: None,
        z_thermal_adjust_enabled: None,
        z_thermal_adjust_offset: None,
        skew_active: false,
        skew_profile: None,
    }
}

/// The WAL context of the crashed print: `/tmp/part.gcode`, nozzle
/// 210 °C, bed 60 °C, part fan 50%, relative E, 1800 mm/min raw F.
pub fn wal_context(transforms: TransformObservations) -> Context {
    Context {
        mono_ns: 5_000_000_000,
        // These fixtures exercise plan building from a given stop set, not
        // trapq coverage certification, so the append frontier is not
        // observed here.
        print_time: None,
        virtual_sdcard: Some(VirtualSdState {
            file_path: "/tmp/part.gcode".to_owned(),
            file_position: 60,
            file_size: None,
        }),
        gcode: GcodeState {
            speed_factor: 1.0,
            speed: 1_800.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: false,
            homing_origin: vec![0.0, 0.0, 0.05, 0.0],
            position: vec![30.0, 10.0, 0.45, 12.0],
            gcode_position: vec![30.0, 10.0, 0.4, 12.0],
        },
        transforms,
        heaters: vec![
            HeaterTarget {
                name: "extruder".to_owned(),
                target: 210.0,
            },
            HeaterTarget {
                name: "heater_bed".to_owned(),
                target: 60.0,
            },
        ],
        fans: vec![FanTarget {
            name: "fan".to_owned(),
            speed: 0.5,
        }],
        exclude: None,
        print_state: None,
    }
}

/// A minimal valid WAL timeline holding one context.
pub fn timeline(context: Context, clean_shutdown: bool) -> WalTimeline {
    WalTimeline {
        toolhead_segments: vec![],
        extruder_segments: vec![],
        other_segments: vec![],
        stepper_ranges: vec![],
        contexts: vec![context],
        markers: vec![],
        heartbeat: None,
        heartbeats: vec![],
        clean_shutdown,
        socket_lost_tail: None,
        recorder_stopped_tail: None,
        last_motion_mono_ns: None,
        scan_end: ScanEnd::CleanEof,
        notes: vec![],
    }
}

/// A stop window classifying the crash as host-death-or-power-loss.
pub fn stop_window() -> StopWindow {
    StopWindow {
        t_a: 10.0,
        t_b: 10.5,
        t_b_source: TbSource::HeartbeatOnly,
        class: CrashClass::HostDeathOrPowerLoss { torn_tail: true },
        correlation: ClockCorrelator::new(),
        anomalies: vec![],
    }
}

/// A possible-stop set whose trusted Z candidates are the given
/// plateaus.
pub fn stop_set(z_plateaus: &[f64]) -> PossibleStopSet {
    PossibleStopSet {
        t_a: 10.0,
        wal_eval_end: 10.5,
        z_candidates: z_plateaus
            .iter()
            .map(|&z| ZCandidate {
                z: Interval::from_pair(z, z),
                provenance: Provenance::Wal,
                z_known: true,
                kind: ZKind::Plateau,
            })
            .collect(),
        xy: None,
        e_internal: None,
        e_file: None,
        file_window: None,
        extension: None,
        degradation: Degradation::default(),
    }
}

/// A recovery reconstruction from the given stop set and context.
pub fn recovery(set: PossibleStopSet, context: Context) -> Reconstruction {
    Reconstruction::Recovery(Box::new(RecoveryReconstruction {
        timeline: timeline(context, false),
        window: stop_window(),
        stop_set: set,
        exclusions: ExclusionReport::unknown(),
    }))
}

/// A clean-shutdown reconstruction.
pub fn clean_shutdown() -> Reconstruction {
    Reconstruction::CleanShutdown(Box::new(timeline(wal_context(plain_transforms()), true)))
}

/// A machine snapshot passing every prerequisite, with a Tap probe and
/// `/tmp` as the `virtual_sdcard` root.
pub fn machine_tap() -> MachineConfig {
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
        config_hash: "cfg-v1".to_owned(),
        validated_config_hash: Some("cfg-v1".to_owned()),
        virtual_sdcard_root: Some("/tmp".to_owned()),
        noise_floor: None,
        noise_floor_speed: None,
        axis_limits: plr_recovery::AxisLimits::default(),
        // `[printer] max_accel`, as the [plr] path reads from the live
        // config: the value the generated file restores after clamping
        // its entry moves.
        max_accel: Some(3_000.0),
    }
}

/// The Tap machine with the probe swapped for a load cell.
pub fn machine_load_cell() -> MachineConfig {
    let mut machine = machine_tap();
    machine.probes = vec![ProbeConfig {
        kind: ProbeKind::LoadCell,
        z_offset: -0.15,
        activate_gcode_no_move: true,
        deactivate_gcode_no_move: true,
    }];
    machine
}

/// The Tap machine with the probe swapped for the ADXL drag method
/// (chip `adxl345`) and a calibrated noise floor.
pub fn machine_adxl_drag() -> MachineConfig {
    let mut machine = machine_tap();
    machine.probes = vec![ProbeConfig {
        kind: ProbeKind::AdxlDrag {
            chip: "adxl345".to_owned(),
        },
        // The nozzle is the stylus: there is no probe z_offset.
        z_offset: 0.0,
        activate_gcode_no_move: true,
        deactivate_gcode_no_move: true,
    }];
    machine.noise_floor = Some(120.0);
    machine
}
