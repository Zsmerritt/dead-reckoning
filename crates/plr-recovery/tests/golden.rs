//! Golden-plan scenario tests: whole plans for fixture scenarios,
//! including the §-mapped step order, the probe-type `z_offset` split,
//! typed fallbacks and the clean-shutdown short-circuit.

mod common;

use plr_analyzer::{ContactOutcome, DeclineReason, MatchConfidence};
use plr_recovery::{
    plan_recovery, select_resume_target, ExcludeObjectDef, FallbackReason, FileTemps,
    OvershootTerm, Phase, PlanConfig, PlanInputs, PlanOutcome, RecoveryError, RecoveryPlan,
    RuntimeComputation, TriggerSource,
};

use common::{
    clean_shutdown, contact_at, machine_adxl_drag, machine_load_cell, machine_tap, match_at, model,
    offset_of, plain_transforms, recovery, stop_set, wal_context,
};

/// The layer-1 resume line every scenario matches at.
fn resume_offset() -> u64 {
    offset_of("G1 X10 Y10 E1 F1800", 1)
}

/// Builds a plan for the given machine/transforms, panicking on any
/// non-plan outcome.
fn build_plan(
    machine: &plr_recovery::MachineConfig,
    transforms: plr_wal::TransformObservations,
) -> RecoveryPlan {
    build_plan_with(machine, transforms, &PlanConfig::default())
}

/// [`build_plan`] with an explicit plan config.
fn build_plan_with(
    machine: &plr_recovery::MachineConfig,
    transforms: plr_wal::TransformObservations,
    config: &PlanConfig,
) -> RecoveryPlan {
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(transforms));
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine,
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    match plan_recovery(&inputs, config) {
        Ok(PlanOutcome::Plan(plan)) => *plan,
        other => panic!("expected a plan, got {other:?}"),
    }
}

#[test]
fn clean_shutdown_produces_no_plan() {
    let reconstruction = clean_shutdown();
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine: &machine_tap(),
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    assert_eq!(
        plan_recovery(&inputs, &PlanConfig::default()).unwrap(),
        PlanOutcome::NoRecoveryNeeded
    );
}

#[test]
#[allow(clippy::too_many_lines)] // one comprehensive golden assertion body
fn normal_tap_recovery_matches_the_golden_plan() {
    let plan = build_plan(&machine_tap(), plain_transforms());

    // Strict recovery-UX phase order, exact (no z_thermal_adjust, no
    // mesh here). The default config uses the consensus PLR_TOUCH path,
    // so the accel clamp/restore wrap the probe.
    let phases: Vec<Phase> = plan.steps.iter().map(|s| s.phase).collect();
    assert_eq!(
        phases,
        vec![
            Phase::IdleTimeout,
            Phase::StepperEnable,
            Phase::ImmediateBedHeat,
            Phase::BelievedZDeclare,
            Phase::HomeXy,
            Phase::CleanNozzle,
            Phase::ShiftedFrame,
            Phase::ProbeApproach,
            Phase::AccelClamp,
            Phase::Probe,
            Phase::AccelRestore,
            Phase::TrueZDeclare,
            Phase::FinalDeclare,
            Phase::ParkForReheat,
            Phase::RestoreFrame,
            Phase::RecoveryFileSelect,
        ]
    );
    // Ids are 1-based and sequential.
    for (index, step) in plan.steps.iter().enumerate() {
        assert_eq!(step.id as usize, index + 1);
    }

    // Accel clamp precedes the probe; restore follows on success; the
    // clamp declares an abort cleanup.
    assert!(plan.accel_clamp_precedes_probe());
    assert!(plan.accel_restore_follows_probe());
    assert!(plan.accel_clamp_declares_cleanup());
    // The consensus touch command carries every tunable explicitly.
    let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe");
    assert_eq!(
        probe.commands,
        vec!["PLR_TOUCH SAMPLES=3 SAMPLE_RANGE=0.01 SPEED=1 RETRACT=2 TOUCH_ACCEL=100"]
    );
    // The extruder-target interlock is present alongside the current
    // temperature band.
    assert!(probe
        .pre_verify
        .iter()
        .any(|v| v.object == "extruder" && v.field == "target"));
    // Envelope: gap (0 span + 0.2 sag) + 0.15*1.0 + 0.5 margin = 0.85,
    // anchored at position_min -2.0.
    assert!((plan.envelope.envelope - 0.85).abs() < 1e-12);
    assert!((plan.envelope.shifted_declare_z - (-1.15)).abs() < 1e-12);

    // Invariant accessors all hold (old and new ordering guarantees).
    assert!(plan.idle_timeout_first());
    assert!(plan.steppers_enabled_before_motion());
    assert!(plan.temp_verify_precedes_probe());
    assert!(plan.z_thermal_freeze_precedes_shifted_declare());
    assert!(plan.probe_step_precedes_mesh_load());
    assert!(plan.mesh_load_precedes_final_declare());
    assert!(plan.no_g28_after_shifted_declare());
    assert!(plan.bed_heat_precedes_motion());
    assert!(plan.believed_z_precedes_home_xy());
    assert!(plan.clean_nozzle_between_home_and_shifted());
    assert!(plan.park_precedes_restore());
    assert!(plan.recovery_file_select_last());
    assert_eq!(plan.resume_offset, resume_offset());
    // The plan now selects the GENERATED recovery file, not the original.
    assert_eq!(plan.resume_file, "part_RECOVERY.gcode");
    assert_eq!(plan.recovery_file.source_name, "part.gcode");
    assert_eq!(plan.recovery_file.name, "part_RECOVERY.gcode");
    assert_eq!(plan.recovery_file.tail_offset, resume_offset());
    // The default fixture has the clean-nozzle macro present, so no
    // confirmation is required and the step calls the macro.
    assert!(!plan.requires_clean_nozzle_confirmation);
    let clean = plan
        .steps_in_phase(Phase::CleanNozzle)
        .next()
        .expect("clean-nozzle step");
    assert_eq!(clean.commands, vec!["CLEAN_NOZZLE"]);

    // The park step lifts off the part (bounded relative Z, safe
    // direction) then travels to the reheat park XY: the print-temp
    // reheat (in the file) must not dwell against the plastic. No print
    // temperatures appear in the plan (they move into the recovery file).
    let park = plan
        .steps_in_phase(Phase::ParkForReheat)
        .next()
        .expect("park step");
    // The lift is an ABSOLUTE move to a runtime-computed, rail-clamped
    // height — never a blind relative lift Klipper would reject with
    // "Move out of range" after the probe established the reference.
    assert_eq!(park.commands[0], "G90");
    assert!(park.commands[1].starts_with("G1 Z{park_z}"));
    assert!(park.commands[2].starts_with("G0 X"));
    assert!(matches!(
        park.compute,
        Some(RuntimeComputation::ParkZ { delta_z, .. }) if (delta_z - 2.0).abs() < 1e-12
    ));
    // The blocking heat waits (M109/M190) are the recovery file's
    // heating gate — never in the plan. (The plan's ImmediateBedHeat
    // does a NON-blocking M104 toward the probe temp, which is fine.)
    assert!(
        !plan
            .steps
            .iter()
            .flat_map(|s| s.commands.iter())
            .any(|c| c.starts_with("M109") || c.starts_with("M190")),
        "blocking heat waits must live in the recovery file, not the plan"
    );
    // The recovery file itself carries the print-temperature reheat
    // behind the heating gate.
    let file = plr_recovery::build_recovery_file(
        &plan.recovery_file,
        common::MODEL_TEXT.as_bytes(),
        "TEST-TS",
    );
    assert!(plr_recovery::verify_heating_gate(&file).is_ok());
    let file_text = file.preamble_text().into_owned();
    assert!(file_text.contains("M109 S210"));
    assert!(file_text.contains("M190 S60"));

    // The consensus Tap probe reads the plugin's consensus median.
    let probe_declare = plan
        .steps_in_phase(Phase::TrueZDeclare)
        .next()
        .expect("true-z step");
    let Some(RuntimeComputation::TrueZ(formula)) = probe_declare.compute else {
        panic!("true-z step must carry the formula");
    };
    // Consensus median is in the z_offset-subtracted bed frame; the
    // formula carries the machine's probe z_offset (-0.1 for the tap
    // fixture) to add back.
    assert_eq!(
        formula.trigger_source,
        TriggerSource::TouchResult { z_offset: -0.1 }
    );
    assert!((formula.z_prev_top - 0.4).abs() < 1e-12);

    // Golden snapshot of the rendered form. Regenerate with
    // PLR_BLESS=1 after intentional changes.
    let rendered = plan.render();
    let golden_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden/normal_tap.txt");
    if std::env::var("PLR_BLESS").is_ok() {
        std::fs::write(golden_path, &rendered).expect("write golden");
    }
    let golden = std::fs::read_to_string(golden_path).expect("golden file (run with PLR_BLESS=1)");
    assert_eq!(rendered, golden.replace("\r\n", "\n"));
}

#[test]
fn adxl_drag_recovery_matches_the_golden_plan() {
    let plan = build_plan(&machine_adxl_drag(), plain_transforms());

    // Identical phase skeleton to the tap plan: the probe method
    // changes the probe step's command and readback, nothing else.
    let phases: Vec<Phase> = plan.steps.iter().map(|s| s.phase).collect();
    assert_eq!(
        phases,
        vec![
            Phase::IdleTimeout,
            Phase::StepperEnable,
            Phase::ImmediateBedHeat,
            Phase::BelievedZDeclare,
            Phase::HomeXy,
            Phase::CleanNozzle,
            Phase::ShiftedFrame,
            Phase::ProbeApproach,
            Phase::Probe,
            Phase::TrueZDeclare,
            Phase::FinalDeclare,
            Phase::ParkForReheat,
            Phase::RestoreFrame,
            Phase::RecoveryFileSelect,
        ]
    );

    // Envelope: gap (0 span + 0.2 sag) + drag_z_step 0.05 + margin 0.5
    // = 0.75 — no speed-proportional term (passes are fixed-Z).
    assert_eq!(
        plan.envelope.params.overshoot,
        OvershootTerm::DragStep { drag_z_step: 0.05 }
    );
    assert!((plan.envelope.envelope - 0.75).abs() < 1e-12);
    assert!((plan.envelope.shifted_declare_z - (-1.25)).abs() < 1e-12);

    // The probe step emits PLR_DRAG_PROBE with every tunable as an
    // explicit, auditable argument, and reads the plugin's drag result.
    let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe");
    assert_eq!(
        probe.commands,
        vec!["PLR_DRAG_PROBE CHIP=\"adxl345\" SPEED=20 Z_STEP=0.05 SENSITIVITY=30"]
    );
    assert!(probe
        .verify
        .iter()
        .any(|v| v.object == "plr" && v.field == "last_drag_result.trigger_z"));
    // The temperature interlock (current AND target) is
    // method-independent; the drag path gets no accel-clamp steps.
    assert!(plan.temp_verify_precedes_probe());
    assert!(probe
        .pre_verify
        .iter()
        .any(|v| v.object == "extruder" && v.field == "target"));
    assert!(plan.first_index(Phase::AccelClamp).is_none());
    assert!(plan.accel_clamp_precedes_probe());
    assert!(plan.accel_restore_follows_probe());

    // The true-Z formula reads the drag trigger source.
    let declare = plan
        .steps_in_phase(Phase::TrueZDeclare)
        .next()
        .expect("true-z step");
    let Some(RuntimeComputation::TrueZ(formula)) = declare.compute else {
        panic!("true-z step must carry the formula");
    };
    assert_eq!(formula.trigger_source, TriggerSource::DragResult);

    // The drag path has NO warm minimum: cold dragging is fine, hot
    // dragging melts the part — so the current-temperature gate is a
    // bare ceiling (NumAtMost), not a warm band.
    assert!(probe.pre_verify.iter().any(|v| v.object == "extruder"
        && v.field == "temperature"
        && matches!(v.predicate, plr_recovery::Predicate::NumAtMost { .. })));

    // Every structural invariant holds for the drag variant too.
    assert!(plan.idle_timeout_first());
    assert!(plan.steppers_enabled_before_motion());
    assert!(plan.no_g28_after_shifted_declare());
    assert!(plan.bed_heat_precedes_motion());
    assert!(plan.believed_z_precedes_home_xy());
    assert!(plan.recovery_file_select_last());

    // Golden snapshot of the rendered form. Regenerate with
    // PLR_BLESS=1 after intentional changes.
    let rendered = plan.render();
    let golden_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden/adxl_drag.txt");
    if std::env::var("PLR_BLESS").is_ok() {
        std::fs::write(golden_path, &rendered).expect("write golden");
    }
    let golden = std::fs::read_to_string(golden_path).expect("golden file (run with PLR_BLESS=1)");
    assert_eq!(rendered, golden.replace("\r\n", "\n"));
}

#[test]
fn quoted_chip_carries_spaced_section_names() {
    // A spaced Klipper accel section name rides intact inside the
    // quoted CHIP value (klippy shlex-parses quoted extended-command
    // values; see machine::chip_embeddable).
    let mut machine = machine_adxl_drag();
    machine.probes[0].kind = plr_recovery::ProbeKind::AdxlDrag {
        chip: "adxl345 bed".to_owned(),
    };
    let plan = build_plan(&machine, plain_transforms());
    let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe");
    assert_eq!(
        probe.commands,
        vec!["PLR_DRAG_PROBE CHIP=\"adxl345 bed\" SPEED=20 Z_STEP=0.05 SENSITIVITY=30"]
    );
}

#[test]
fn noise_floor_speed_mismatch_warns_but_never_refuses() {
    // Calibrated at 10 mm/s, planned at 20 mm/s (default): 100% apart,
    // far beyond the 20% band -> warning, plan still produced.
    let mut machine = machine_adxl_drag();
    machine.noise_floor_speed = Some(10.0);
    let plan = build_plan(&machine, plain_transforms());
    assert!(
        plan.warnings.iter().any(|w| matches!(
            w,
            plr_recovery::PlanWarning::NoiseFloorSpeedMismatch {
                calibrated_at,
                drag_speed,
            } if (*calibrated_at - 10.0).abs() < 1e-12 && (*drag_speed - 20.0).abs() < 1e-12
        )),
        "{:?}",
        plan.warnings
    );
    // The rendered plan tells the operator what to do.
    assert!(
        plan.render().contains("re-run PLR_NOISE_TEST"),
        "{}",
        plan.render()
    );

    // Within 20% of the calibration speed: no warning.
    let mut machine = machine_adxl_drag();
    machine.noise_floor_speed = Some(18.0); // |20-18| = 2 <= 3.6
    let plan = build_plan(&machine, plain_transforms());
    assert!(
        !plan
            .warnings
            .iter()
            .any(|w| matches!(w, plr_recovery::PlanWarning::NoiseFloorSpeedMismatch { .. })),
        "{:?}",
        plan.warnings
    );

    // No recorded speed (today's plugin): nothing to check.
    let plan = build_plan(&machine_adxl_drag(), plain_transforms());
    assert!(!plan
        .warnings
        .iter()
        .any(|w| matches!(w, plr_recovery::PlanWarning::NoiseFloorSpeedMismatch { .. })));

    // A tap machine never checks it, even with a stray recorded speed.
    let mut machine = machine_tap();
    machine.noise_floor_speed = Some(1.0);
    let plan = build_plan(&machine, plain_transforms());
    assert!(!plan
        .warnings
        .iter()
        .any(|w| matches!(w, plr_recovery::PlanWarning::NoiseFloorSpeedMismatch { .. })));
}

#[test]
fn drag_without_noise_floor_is_rejected_with_the_calibration_hint() {
    let mut machine = machine_adxl_drag();
    machine.noise_floor = None;
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine: &machine,
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    let Err(RecoveryError::MachineRejected { failures }) =
        plan_recovery(&inputs, &PlanConfig::default())
    else {
        panic!("expected MachineRejected");
    };
    assert!(
        failures
            .iter()
            .any(|f| f.to_string().contains("run PLR_NOISE_TEST first")),
        "{failures:?}"
    );
}

#[test]
fn consensus_load_cell_reads_the_touch_median() {
    // Default config: the load-cell machine goes through PLR_TOUCH just
    // like tap (a load cell natively supports repeated cheap touches —
    // repeated force sampling with a retract between is its native
    // calibration mode), so it reads the consensus median.
    let plan = build_plan(&machine_load_cell(), plain_transforms());
    let declare = plan
        .steps_in_phase(Phase::TrueZDeclare)
        .next()
        .expect("true-z step");
    let Some(RuntimeComputation::TrueZ(formula)) = declare.compute else {
        panic!("true-z step must carry the formula");
    };
    // Load-cell fixture z_offset is -0.15; the consensus median needs
    // it added back too.
    assert_eq!(
        formula.trigger_source,
        TriggerSource::TouchResult { z_offset: -0.15 }
    );
    let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe");
    assert!(probe.commands[0].starts_with("PLR_TOUCH "));
    assert!(probe
        .verify
        .iter()
        .any(|v| v.object == "plr" && v.field == "last_touch_result.median_z"));
}

#[test]
fn legacy_single_probe_preserves_the_per_probe_readback() {
    // The legacy single-PROBE path (plugin/PLR_TOUCH may be absent)
    // keeps the stock per-probe-type readback and z_offset arithmetic,
    // and emits no accel-clamp steps.
    let legacy = PlanConfig {
        legacy_single_probe: true,
        ..PlanConfig::default()
    };
    let build = |machine: &plr_recovery::MachineConfig| {
        let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
        let contact = contact_at(0.4);
        let match_result = match_at(resume_offset());
        let model = model();
        let inputs = PlanInputs {
            machine,
            reconstruction: &reconstruction,
            contact: &contact,
            match_result: &match_result,
            model: &model,
            file_temps: FileTemps::default(),
            exclude_objects: &[],
            clean_nozzle_macro_present: true,
            purge_macro_present: false,
        };
        match plan_recovery(&inputs, &legacy) {
            Ok(PlanOutcome::Plan(plan)) => *plan,
            other => panic!("expected a plan, got {other:?}"),
        }
    };
    // Load cell: bed_z + z_offset, PROBE, no accel clamp.
    let lc = build(&machine_load_cell());
    assert!(lc.first_index(Phase::AccelClamp).is_none());
    let lc_declare = lc.steps_in_phase(Phase::TrueZDeclare).next().unwrap();
    let Some(RuntimeComputation::TrueZ(formula)) = lc_declare.compute else {
        panic!("formula");
    };
    assert_eq!(
        formula.trigger_source,
        TriggerSource::BedZPlusOffset { z_offset: -0.15 }
    );
    let lc_probe = lc.steps_in_phase(Phase::Probe).next().unwrap();
    assert_eq!(lc_probe.commands, vec!["PROBE PROBE_SPEED=1 SAMPLES=1"]);
    assert!(lc_probe
        .verify
        .iter()
        .any(|v| v.object == "probe" && v.field == "last_probe_position.2"));
    // Tap: raw last_z_result, PROBE.
    let tap = build(&machine_tap());
    let tap_probe = tap.steps_in_phase(Phase::Probe).next().unwrap();
    assert!(tap_probe
        .verify
        .iter()
        .any(|v| v.object == "probe" && v.field == "last_z_result"));
    // The target interlock is method- and path-independent.
    assert!(tap_probe
        .pre_verify
        .iter()
        .any(|v| v.object == "extruder" && v.field == "target"));
}

#[test]
fn hop_ambiguity_widens_the_envelope() {
    let narrow = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let wide = recovery(stop_set(&[0.4, 0.6]), wal_context(plain_transforms()));
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let machine = machine_tap();
    let mut envelopes = Vec::new();
    for reconstruction in [&narrow, &wide] {
        let inputs = PlanInputs {
            machine: &machine,
            reconstruction,
            contact: &contact,
            match_result: &match_result,
            model: &model,
            file_temps: FileTemps::default(),
            exclude_objects: &[],
            clean_nozzle_macro_present: true,
            purge_macro_present: false,
        };
        match plan_recovery(&inputs, &PlanConfig::default()).unwrap() {
            PlanOutcome::Plan(plan) => envelopes.push(plan.envelope.envelope),
            other => panic!("expected plan, got {other:?}"),
        }
    }
    // The 0.2 mm hop span widens the envelope by exactly 0.2 mm.
    assert!((envelopes[1] - envelopes[0] - 0.2).abs() < 1e-12);
}

#[test]
fn declined_contact_zone_degrades_to_typed_manual_fallback() {
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let contact = ContactOutcome::Declined(DeclineReason::VaseMode {
        spiral_fraction: 0.9,
    });
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine: &machine_tap(),
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    let outcome = plan_recovery(&inputs, &PlanConfig::default()).unwrap();
    assert_eq!(
        outcome,
        PlanOutcome::ManualFallback {
            reason: FallbackReason::ContactDeclined(DeclineReason::VaseMode {
                spiral_fraction: 0.9
            })
        }
    );
}

#[test]
fn layer_only_match_degrades_to_typed_manual_fallback() {
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let contact = contact_at(0.4);
    let match_result = plr_analyzer::MatchResult {
        candidates: vec![],
        confidence: MatchConfidence::LayerOnly { layer: 1 },
        skipped_unknown: 0,
    };
    let model = model();
    let inputs = PlanInputs {
        machine: &machine_tap(),
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    let outcome = plan_recovery(&inputs, &PlanConfig::default()).unwrap();
    assert_eq!(
        outcome,
        PlanOutcome::ManualFallback {
            reason: FallbackReason::MatchTooCoarse { layer: 1 }
        }
    );
}

#[test]
fn z_thermal_and_mesh_steps_appear_in_strict_order() {
    let mut transforms = plain_transforms();
    transforms.z_thermal_adjust_enabled = Some(true);
    transforms.z_thermal_adjust_offset = Some(0.03);
    transforms.bed_mesh_active = true;
    transforms.bed_mesh_profile = Some("default".to_owned());
    let plan = build_plan(&machine_tap(), transforms);

    let freeze = plan.first_index(Phase::TransformFreeze).expect("freeze");
    let shifted = plan.first_index(Phase::ShiftedFrame).expect("shifted");
    let probe = plan.first_index(Phase::Probe).expect("probe");
    let mesh = plan.first_index(Phase::MeshLoad).expect("mesh");
    let declare = plan.first_index(Phase::FinalDeclare).expect("final");
    assert!(freeze < shifted);
    assert!(probe < mesh);
    assert!(mesh < declare);
    assert!(plan.z_thermal_freeze_precedes_shifted_declare());
    assert!(plan.probe_step_precedes_mesh_load());
    assert!(plan.mesh_load_precedes_final_declare());

    // The frozen adjust value rides in the formula.
    let true_z = plan
        .steps_in_phase(Phase::TrueZDeclare)
        .next()
        .expect("true-z");
    let Some(RuntimeComputation::TrueZ(formula)) = true_z.compute else {
        panic!("formula expected");
    };
    assert_eq!(formula.frozen_z_adjust, Some(0.03));

    // The mesh command loads the named profile.
    let mesh_step = plan.steps_in_phase(Phase::MeshLoad).next().expect("mesh");
    assert_eq!(mesh_step.commands, vec!["BED_MESH_PROFILE LOAD=default"]);
}

#[test]
fn adaptive_mesh_is_not_restorable_and_warns() {
    let mut transforms = plain_transforms();
    transforms.bed_mesh_active = true;
    transforms.bed_mesh_profile = Some(String::new()); // adaptive: empty name
    let plan = build_plan(&machine_tap(), transforms);
    assert!(plan.first_index(Phase::MeshLoad).is_none());
    assert!(plan
        .warnings
        .contains(&plr_recovery::PlanWarning::AdaptiveMeshNotRestorable));
}

#[test]
fn subdirectory_print_file_is_a_typed_error() {
    let mut context = wal_context(plain_transforms());
    context.virtual_sdcard.as_mut().unwrap().file_path = "/tmp/sub/part.gcode".to_owned();
    let reconstruction = recovery(stop_set(&[0.4]), context);
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine: &machine_tap(),
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    assert!(matches!(
        plan_recovery(&inputs, &PlanConfig::default()),
        Err(RecoveryError::FileNotTopLevel { .. })
    ));
}

#[test]
fn machine_rejection_lists_every_failure() {
    let mut machine = machine_tap();
    machine.force_move_enabled = false;
    machine.validated_config_hash = Some("stale".to_owned());
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine: &machine,
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    let Err(RecoveryError::MachineRejected { failures }) =
        plan_recovery(&inputs, &PlanConfig::default())
    else {
        panic!("expected MachineRejected");
    };
    assert_eq!(failures.len(), 2);
}

#[test]
fn exclude_objects_are_restored_between_m23_and_m26() {
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let excludes = [ExcludeObjectDef {
        name: "cube_1".to_owned(),
        center: Some([50.0, 50.0]),
        polygon: vec![[40.0, 40.0], [60.0, 40.0], [60.0, 60.0], [40.0, 60.0]],
        currently_excluded: true,
    }];
    let inputs = PlanInputs {
        machine: &machine_tap(),
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &excludes,
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    let PlanOutcome::Plan(plan) = plan_recovery(&inputs, &PlanConfig::default()).unwrap() else {
        panic!("expected plan");
    };
    let select = plan
        .steps_in_phase(Phase::RecoveryFileSelect)
        .next()
        .expect("recovery-file-select");
    let commands = &select.commands;
    // The recovery file is selected (not the original), exclude-object
    // state is restored between M23 and M24, and there is no M26.
    let m23 = commands.iter().position(|c| c.starts_with("M23")).unwrap();
    assert!(commands[m23].contains("_RECOVERY.gcode"));
    let define = commands
        .iter()
        .position(|c| c.starts_with("EXCLUDE_OBJECT_DEFINE NAME=cube_1"))
        .unwrap();
    let exclude = commands
        .iter()
        .position(|c| c == "EXCLUDE_OBJECT NAME=cube_1")
        .unwrap();
    let m24 = commands.iter().position(|c| c == "M24").unwrap();
    assert!(m23 < define && define < exclude && exclude < m24);
    assert!(!commands.iter().any(|c| c.starts_with("M26")));
    assert!(commands[define].contains("CENTER=50,50"));
    assert!(commands[define].contains("POLYGON=[[40,40],[60,40],[60,60],[40,60]]"));
}

#[test]
fn recovery_file_matches_the_golden_and_holds_the_heating_gate() {
    // The full generated recovery file for the normal_tap scenario: the
    // header, the temps-at-park block, the re-home, the built-in purge,
    // the entry moves, then the verbatim original tail.
    let plan = build_plan(&machine_tap(), plain_transforms());
    let file =
        plr_recovery::build_recovery_file(&plan.recovery_file, common::MODEL_TEXT.as_bytes(), "TS");

    // Heating gate: no XY move precedes the blocking waits; G28 X Y then
    // entry follow.
    assert!(plr_recovery::verify_heating_gate(&file).is_ok());

    // The verbatim tail is byte-identical to the original from the
    // matched offset.
    assert_eq!(
        file.tail_bytes(),
        &common::MODEL_TEXT.as_bytes()[usize::try_from(plan.resume_offset).unwrap()..]
    );

    // Golden snapshot of the full file content.
    let golden_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/golden/recovery_file.gcode"
    );
    if std::env::var("PLR_BLESS").is_ok() {
        std::fs::write(golden_path, &file.content).expect("write golden");
    }
    let golden = std::fs::read_to_string(golden_path).expect("golden file (run with PLR_BLESS=1)");
    assert_eq!(
        String::from_utf8(file.content).expect("ASCII fixture"),
        golden.replace("\r\n", "\n")
    );
}

#[test]
fn no_clean_nozzle_macro_requires_confirmation_and_emits_no_command() {
    // With the clean-nozzle macro ABSENT the step carries no command and
    // the plan flags that the operator must confirm the nozzle is clean.
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine: &machine_tap(),
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: false,
        purge_macro_present: false,
    };
    let PlanOutcome::Plan(plan) = plan_recovery(&inputs, &PlanConfig::default()).unwrap() else {
        panic!("expected plan");
    };
    assert!(plan.requires_clean_nozzle_confirmation);
    let clean = plan
        .steps_in_phase(Phase::CleanNozzle)
        .next()
        .expect("clean-nozzle step");
    assert!(clean.commands.is_empty());
    assert!(plan
        .render()
        .contains("confirm the nozzle is clean before executing"));
}

/// Finding 3 regression: the ENTRY coordinates left the plan for the
/// generated file, and must still be bounds-checked. On main these were
/// `Phase::Entry` commands the itinerary pre-flight walked; the
/// equivalent guarantee is now `preflight_recovery_file` over the file's
/// preamble, run on the same build path.
#[test]
fn generated_file_entry_coordinates_are_bounds_checked() {
    use plr_recovery::{
        preflight_recovery_file, ItineraryBounds, PlanRejection, ViolationKind,
        RECOVERY_FILE_STEP_ID,
    };
    let plan = build_plan(&machine_tap(), plain_transforms());
    let file =
        plr_recovery::build_recovery_file(&plan.recovery_file, common::MODEL_TEXT.as_bytes(), "TS");
    let bounds = ItineraryBounds {
        x: Some((0.0, 200.0)),
        y: Some((0.0, 200.0)),
        z_max: Some(250.0),
        position_min: plan.envelope.position_min,
        contact_point: [20.0, 10.0],
    };
    // The generated file passes with roomy limits.
    assert!(preflight_recovery_file(&file, &bounds).is_ok());

    // entry_z = resume_z + entry_hop must be checked against z_max: a
    // machine whose Z travel ends below the entry hop is caught HERE, at
    // plan time, instead of dying mid-file on "Move out of range".
    let tight_z = ItineraryBounds {
        z_max: Some(0.5), // entry emits G0 Z1.35 / G1 Z0.35
        ..bounds
    };
    let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
        preflight_recovery_file(&file, &tight_z)
    else {
        panic!("an entry Z above z_max must be rejected");
    };
    assert!(
        violations
            .iter()
            .any(|v| v.axis == 'Z' && v.kind == ViolationKind::AxisLimit && v.value > 1.0),
        "the entry hop Z must be named: {violations:?}"
    );
    assert!(violations
        .iter()
        .all(|v| v.step_id == RECOVERY_FILE_STEP_ID));

    // The entry XY is checked too (it is an absolute travel in the file).
    let tight_xy = ItineraryBounds {
        x: Some((0.0, 5.0)),
        ..bounds
    };
    let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
        preflight_recovery_file(&file, &tight_xy)
    else {
        panic!("an entry X beyond the limit must be rejected");
    };
    assert!(violations.iter().any(|v| v.axis == 'X'));

    // Aggregation: several bad axes are reported together, not first-fail.
    let tight_all = ItineraryBounds {
        x: Some((0.0, 5.0)),
        y: Some((0.0, 5.0)),
        z_max: Some(0.5),
        ..bounds
    };
    let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
        preflight_recovery_file(&file, &tight_all)
    else {
        panic!("expected rejection");
    };
    assert!(violations.iter().any(|v| v.axis == 'X'));
    assert!(violations.iter().any(|v| v.axis == 'Y'));
    assert!(violations.iter().any(|v| v.axis == 'Z'));
}

/// Finding 3: an out-of-bounds entry coordinate must make the whole plan
/// build fail, not merely be detectable by a test that remembers to look.
#[test]
fn a_resume_near_z_max_is_refused_at_plan_time() {
    // z_max just above the resume Z but below resume_z + entry_hop: the
    // file's entry hop would be out of range mid-recovery.
    let mut machine = machine_tap();
    machine.axis_limits = plr_recovery::AxisLimits {
        x: Some((0.0, 200.0)),
        y: Some((0.0, 200.0)),
        z_max: Some(0.5),
    };
    let plan = build_plan(&machine, plain_transforms());
    let file =
        plr_recovery::build_recovery_file(&plan.recovery_file, common::MODEL_TEXT.as_bytes(), "TS");
    let err = plr_recovery::preflight_generated_file(&file, &machine, [20.0, 10.0])
        .expect_err("entry hop above z_max must be refused");
    assert!(
        matches!(err, RecoveryError::ItineraryRejected(_)),
        "{err:?}"
    );
}

#[test]
fn configured_reheat_park_is_used_without_a_warning() {
    let config = PlanConfig {
        reheat_park_x: Some(5.0),
        reheat_park_y: Some(7.0),
        ..PlanConfig::default()
    };
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine: &machine_tap(),
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    let PlanOutcome::Plan(plan) = plan_recovery(&inputs, &config).unwrap() else {
        panic!("expected plan");
    };
    let park = plan
        .steps_in_phase(Phase::ParkForReheat)
        .next()
        .expect("park step");
    assert!(park.commands.iter().any(|c| c == "G0 X5 Y7 F6000"));
    assert!(!plan
        .warnings
        .iter()
        .any(|w| matches!(w, plr_recovery::PlanWarning::ReheatParkComputed { .. })));
}

/// Finding 7 regression: the park warning must assert only what it
/// checked. A CONFIGURED park point inside the part footprint warns
/// honestly instead of being waved through.
#[test]
fn a_configured_park_inside_the_part_warns() {
    // The fixture part spans X 10..30, Y 10..30; park at its centre.
    let config = PlanConfig {
        reheat_park_x: Some(20.0),
        reheat_park_y: Some(20.0),
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_tap(), plain_transforms(), &config);
    let warning = plan
        .warnings
        .iter()
        .find_map(|w| match w {
            plr_recovery::PlanWarning::ReheatParkInsidePart { point, configured } => {
                Some((*point, *configured))
            }
            _ => None,
        })
        .expect("a park inside the part must warn");
    assert!(warning.1, "the warning must say it was configured");
    assert!((warning.0[0] - 20.0).abs() < 1e-12);
    let rendered = plan.render();
    assert!(
        rendered.contains("INSIDE the part bounding box"),
        "{rendered}"
    );
    assert!(
        rendered.contains("configured via reheat_park_x/y"),
        "{rendered}"
    );
}

/// Finding 7 regression: a COMPUTED park point is only described as
/// "clear of the part" when it verifiably is. With axis limits that clamp
/// every side back over the footprint, the honest
/// `ReheatParkInsidePart` warning is emitted instead.
#[test]
fn a_computed_park_that_clamps_back_inside_the_part_warns_honestly() {
    // Travel limits inside the part footprint (part spans X 10..30,
    // Y 10..30): every candidate side clamps back over the part. The
    // limits still contain the analyzer's contact point (20, 10), so the
    // plan itself is valid — only the park has nowhere clear to go.
    let mut machine = machine_tap();
    machine.axis_limits = plr_recovery::AxisLimits {
        x: Some((12.0, 28.0)),
        y: Some((10.0, 28.0)),
        z_max: Some(250.0),
    };
    let plan = build_plan(&machine, plain_transforms());
    assert!(
        plan.warnings.iter().any(|w| matches!(
            w,
            plr_recovery::PlanWarning::ReheatParkInsidePart {
                configured: false,
                ..
            }
        )),
        "no side clears the part; the warning must say so: {:?}",
        plan.warnings
    );
    // ...and it must NOT claim the point is clear of the part.
    assert!(!plan
        .warnings
        .iter()
        .any(|w| matches!(w, plr_recovery::PlanWarning::ReheatParkComputed { .. })));
}

/// Finding 7: when a side DOES clear the part, the computed warning is
/// used and the point is genuinely outside the footprint.
#[test]
fn a_computed_park_that_clears_the_part_says_so_truthfully() {
    let mut machine = machine_tap();
    machine.axis_limits = plr_recovery::AxisLimits {
        x: Some((0.0, 200.0)),
        y: Some((0.0, 200.0)),
        z_max: Some(250.0),
    };
    let plan = build_plan(&machine, plain_transforms());
    let point = plan
        .warnings
        .iter()
        .find_map(|w| match w {
            plr_recovery::PlanWarning::ReheatParkComputed { point } => Some(*point),
            _ => None,
        })
        .expect("a clear computed park must use the computed warning");
    // The fixture part spans X 10..30, Y 10..30: verify the claim.
    let inside = point[0] >= 10.0 && point[0] <= 30.0 && point[1] >= 10.0 && point[1] <= 30.0;
    assert!(!inside, "computed park {point:?} is inside the part bbox");
    assert!(plan.render().contains("verified clear of the part"));
}

#[test]
fn out_of_bounds_configured_park_is_refused_by_preflight() {
    // A configured park outside the known axis limits is caught by the
    // whole-itinerary pre-flight (the park step's absolute G0).
    let mut machine = machine_tap();
    machine.axis_limits = plr_recovery::AxisLimits {
        x: Some((0.0, 200.0)),
        y: Some((0.0, 200.0)),
        z_max: Some(250.0),
    };
    let config = PlanConfig {
        reheat_park_x: Some(9_999.0),
        reheat_park_y: Some(10.0),
        ..PlanConfig::default()
    };
    let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
    let contact = contact_at(0.4);
    let match_result = match_at(resume_offset());
    let model = model();
    let inputs = PlanInputs {
        machine: &machine,
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &[],
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    assert!(matches!(
        plan_recovery(&inputs, &config),
        Err(RecoveryError::ItineraryRejected(_))
    ));
}

#[test]
fn empty_clamped_temp_band_is_refused() {
    // A ceiling at/below probe_temp_min empties the clamped band.
    let config = PlanConfig {
        max_probe_nozzle_temp: 140.0, // == probe_temp_min → empty band
        ..PlanConfig::default()
    };
    assert!(matches!(
        config.validate(),
        Err(RecoveryError::InvalidPlanConfig {
            field: "max_probe_nozzle_temp"
        })
    ));
    // A band too narrow to hold the headroom below the ceiling is
    // refused, naming both bounds and the headroom (finding 9).
    let config = PlanConfig {
        max_probe_nozzle_temp: 143.0, // ceiling 143, min 140 -> only 3 C
        ..PlanConfig::default()
    };
    let err = config.validate().unwrap_err();
    assert!(
        matches!(
            err,
            RecoveryError::ProbeTempHeadroomUnavailable { headroom, .. }
                if (headroom - plr_recovery::PROBE_TEMP_HEADROOM).abs() < 1e-12
        ),
        "{err:?}"
    );
    let msg = err.to_string();
    assert!(msg.contains("140"), "{msg}");
    assert!(msg.contains("143"), "{msg}");
    assert!(msg.contains("max_probe_nozzle_temp"), "{msg}");
}

/// Finding 9 (cross-component interlock): the plan must NEVER command a
/// probe nozzle target at the plugin's contact ceiling. The plugin
/// refuses `PLR_TOUCH`/`PLR_DRAG_PROBE` when `max(current, target)`
/// exceeds `max_probe_nozzle_temp`; a target ON the ceiling trips that on
/// ordinary PID overshoot, and because the probe runs after the
/// shifted-frame declare the abort wedges the recovery permanently.
#[test]
fn probe_target_leaves_headroom_below_the_contact_ceiling() {
    let headroom = plr_recovery::PROBE_TEMP_HEADROOM;

    // Default config: the emitted M104 sits at least `headroom` below the
    // ceiling (145 under the default 150).
    let plan = build_plan(&machine_tap(), plain_transforms());
    let heat = plan
        .steps_in_phase(Phase::ImmediateBedHeat)
        .next()
        .expect("immediate-bed-heat step");
    let m104 = heat
        .commands
        .iter()
        .find(|c| c.starts_with("M104 S"))
        .expect("nozzle preheat command");
    let emitted: f64 = m104.trim_start_matches("M104 S").parse().expect("number");
    let ceiling = PlanConfig::default().clamped_probe_max();
    assert!(
        emitted + headroom <= ceiling + 1e-9,
        "emitted probe target {emitted} must be >= {headroom} C below the ceiling {ceiling}"
    );
    assert!(
        (emitted - 145.0).abs() < 1e-12,
        "expected 145, got {emitted}"
    );

    // An operator asking for the ceiling (or above it) is pulled down to
    // the headroom, never emitted at the ceiling.
    for asked in [150.0, 155.0, 160.0] {
        let config = PlanConfig {
            probe_nozzle_temp: asked,
            probe_temp_max: 160.0,
            max_probe_nozzle_temp: 160.0,
            ..PlanConfig::default()
        };
        config.validate().expect("config is valid");
        let commanded = config.commanded_probe_temp();
        assert!(
            commanded + headroom <= config.clamped_probe_max() + 1e-9,
            "asked {asked}: commanded {commanded} must leave {headroom} C of headroom"
        );
    }

    // The VERIFICATION band is not tightened by the headroom: it stays at
    // the ceiling (plus the measured tolerance, see the next test).
    let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe");
    let band_max = probe
        .pre_verify
        .iter()
        .find_map(|v| match v.predicate {
            plr_recovery::Predicate::TempWithin { max, .. } if v.field == "temperature" => {
                Some(max)
            }
            _ => None,
        })
        .expect("temperature band");
    assert!(
        band_max >= ceiling,
        "band max {band_max} must not tighten below the ceiling {ceiling}"
    );
}

/// Finding 10 (cross-language contract): the Probe step's MEASURED
/// temperature bound must equal `max_probe_nozzle_temp +
/// PROBE_TEMP_MEASURED_TOLERANCE`, and its TARGET bound exactly
/// `max_probe_nozzle_temp`.
///
/// The tolerance mirrors the Klipper plugin's
/// `MAX_TOUCH_TEMPERATURE_EPSILON` (Cartographer
/// `probe/touch_mode.py:34`, value 2). plrd and the plugin must refuse at
/// the IDENTICAL boundary — if either side changes its tolerance, this
/// test is what fails first. The asymmetry is deliberate: measured
/// overshoot is forgiven, a hotter commanded target is not.
#[test]
fn probe_temperature_bounds_stay_in_lockstep_with_the_plugin() {
    // Keep in sync with klippy_plugin's MAX_TOUCH_TEMPERATURE_EPSILON.
    const PLUGIN_MAX_TOUCH_TEMPERATURE_EPSILON: f64 = 2.0;
    assert!(
        (plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE - PLUGIN_MAX_TOUCH_TEMPERATURE_EPSILON).abs()
            < 1e-12,
        "plrd's measured tolerance must equal the plugin's MAX_TOUCH_TEMPERATURE_EPSILON"
    );

    let ceiling = PlanConfig::default().max_probe_nozzle_temp;
    // Both probe paths carry the same bounds (the drag path's measured
    // predicate is a bare ceiling, the touch path's a band).
    for machine in [machine_tap(), machine_adxl_drag()] {
        let plan = build_plan(&machine, plain_transforms());
        let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe");

        let measured_max = probe
            .pre_verify
            .iter()
            .filter(|v| v.object == "extruder" && v.field == "temperature")
            .find_map(|v| match v.predicate {
                plr_recovery::Predicate::TempWithin { max, .. }
                | plr_recovery::Predicate::NumAtMost { max } => Some(max),
                _ => None,
            })
            .expect("measured temperature bound");
        assert!(
            (measured_max - (ceiling + PLUGIN_MAX_TOUCH_TEMPERATURE_EPSILON)).abs() < 1e-12,
            "measured bound {measured_max} must be the ceiling {ceiling} + {PLUGIN_MAX_TOUCH_TEMPERATURE_EPSILON}"
        );

        let target_max = probe
            .pre_verify
            .iter()
            .filter(|v| v.object == "extruder" && v.field == "target")
            .find_map(|v| match v.predicate {
                plr_recovery::Predicate::NumAtMost { max } => Some(max),
                _ => None,
            })
            .expect("target bound");
        assert!(
            (target_max - ceiling).abs() < 1e-12,
            "target bound {target_max} must stay exactly at the ceiling {ceiling} \
             (commanding a hotter nozzle is never forgiven)"
        );
        // The asymmetry itself.
        assert!(
            measured_max > target_max,
            "the measured bound must be the forgiving one"
        );
    }
}

#[test]
fn resume_target_is_the_layer_1_infill_line() {
    let model = model();
    let target = select_resume_target(&model, &match_at(resume_offset())).unwrap();
    assert_eq!(target.offset, resume_offset());
    assert_eq!(target.layer, Some(1));
    assert!(target.on_infill);
    assert!((target.position[2] - 0.4).abs() < 1e-9);

    // Ambiguity resolves to the latest offset (skip-forward).
    let ambiguous = plr_analyzer::MatchResult {
        candidates: vec![],
        confidence: MatchConfidence::AmbiguousWindow {
            offsets: vec![offset_of("G1 X10 Y10 E1 F1800", 0), resume_offset()],
        },
        skipped_unknown: 0,
    };
    let target = select_resume_target(&model, &ambiguous).unwrap();
    assert_eq!(target.offset, resume_offset());
}

#[test]
fn itinerary_preflight_catches_a_corrupted_probe_site() {
    use plr_recovery::{preflight_itinerary, ItineraryBounds, PlanRejection, ViolationKind};
    let mut plan = build_plan(&machine_tap(), plain_transforms());
    // The contact point the analyzer selected (common::contact_at).
    let bounds = ItineraryBounds {
        x: Some((0.0, 200.0)),
        y: Some((0.0, 200.0)),
        z_max: Some(250.0),
        position_min: plan.envelope.position_min,
        contact_point: [20.0, 10.0],
    };
    // A clean plan passes.
    assert!(preflight_itinerary(&plan, &bounds).is_ok());
    // Corrupt the probe-approach travel target to a different, and
    // out-of-limits, coordinate.
    let approach = plan
        .steps
        .iter_mut()
        .find(|s| s.phase == Phase::ProbeApproach)
        .expect("approach");
    approach.commands = vec!["G90".to_owned(), "G0 X999 Y10 F6000".to_owned()];
    let Err(PlanRejection::ItineraryOutOfBounds { violations }) =
        preflight_itinerary(&plan, &bounds)
    else {
        panic!("expected rejection");
    };
    // Both the contact mismatch and the axis-limit breach are reported.
    assert!(violations
        .iter()
        .any(|v| v.kind == ViolationKind::ContactMismatch && v.axis == 'X'));
    assert!(violations
        .iter()
        .any(|v| v.kind == ViolationKind::AxisLimit && v.axis == 'X'));
}

#[test]
fn plans_round_trip_through_serde() {
    let plan = build_plan(&machine_tap(), plain_transforms());
    let json = serde_json::to_string(&plan).unwrap();
    let back: RecoveryPlan = serde_json::from_str(&json).unwrap();
    assert_eq!(back, plan);
}
