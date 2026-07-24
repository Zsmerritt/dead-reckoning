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
    };
    match plan_recovery(&inputs, &PlanConfig::default()) {
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
    };
    assert_eq!(
        plan_recovery(&inputs, &PlanConfig::default()).unwrap(),
        PlanOutcome::NoRecoveryNeeded
    );
}

#[test]
fn normal_tap_recovery_matches_the_golden_plan() {
    let plan = build_plan(&machine_tap(), plain_transforms());

    // §8 phase order, exact (no z_thermal_adjust, no mesh here).
    let phases: Vec<Phase> = plan.steps.iter().map(|s| s.phase).collect();
    assert_eq!(
        phases,
        vec![
            Phase::IdleTimeout,
            Phase::StepperEnable,
            Phase::Preheat,
            Phase::HomeXy,
            Phase::ShiftedFrame,
            Phase::ProbeApproach,
            Phase::Probe,
            Phase::TrueZDeclare,
            Phase::FinalDeclare,
            Phase::RestoreFrame,
            Phase::Entry,
            Phase::FileSelect,
            Phase::ResumeStart,
        ]
    );
    // Ids are 1-based and sequential.
    for (index, step) in plan.steps.iter().enumerate() {
        assert_eq!(step.id as usize, index + 1);
    }
    // Envelope: gap (0 span + 0.2 sag) + 0.15*1.0 + 0.5 margin = 0.85,
    // anchored at position_min -2.0.
    assert!((plan.envelope.envelope - 0.85).abs() < 1e-12);
    assert!((plan.envelope.shifted_declare_z - (-1.15)).abs() < 1e-12);

    // Invariant accessors all hold.
    assert!(plan.idle_timeout_first());
    assert!(plan.steppers_enabled_before_motion());
    assert!(plan.temp_verify_precedes_probe());
    assert!(plan.z_thermal_freeze_precedes_shifted_declare());
    assert!(plan.probe_step_precedes_mesh_load());
    assert!(plan.mesh_load_precedes_final_declare());
    assert!(plan.no_g28_after_shifted_declare());
    assert_eq!(plan.m26_offset(), Some(plan.resume_offset));
    assert_eq!(plan.resume_offset, resume_offset());
    assert_eq!(plan.resume_file, "part.gcode");

    // The restore step lifts off the part (bounded relative Z, safe
    // direction) BEFORE any print-temperature command: the nozzle must
    // not dwell at print temperature pressed against the plastic.
    let restore = plan
        .steps_in_phase(Phase::RestoreFrame)
        .next()
        .expect("restore step");
    assert_eq!(restore.commands[0], "G91");
    assert!(restore.commands[1].starts_with("G1 Z"));
    assert_eq!(restore.commands[2], "G90");
    let first_heat = restore
        .commands
        .iter()
        .position(|c| c.starts_with("M104") || c.starts_with("M140"))
        .expect("restore sets print temps");
    assert!(
        first_heat > 2,
        "lift must precede print-temperature restore"
    );

    // The Tap probe uses the raw trigger Z.
    let probe_declare = plan
        .steps_in_phase(Phase::TrueZDeclare)
        .next()
        .expect("true-z step");
    let Some(RuntimeComputation::TrueZ(formula)) = probe_declare.compute else {
        panic!("true-z step must carry the formula");
    };
    assert_eq!(formula.trigger_source, TriggerSource::RawLastZResult);
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
            Phase::Preheat,
            Phase::HomeXy,
            Phase::ShiftedFrame,
            Phase::ProbeApproach,
            Phase::Probe,
            Phase::TrueZDeclare,
            Phase::FinalDeclare,
            Phase::RestoreFrame,
            Phase::Entry,
            Phase::FileSelect,
            Phase::ResumeStart,
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
        vec!["PLR_DRAG_PROBE CHIP=adxl345 SPEED=5 Z_STEP=0.05 SENSITIVITY=3"]
    );
    assert!(probe
        .verify
        .iter()
        .any(|v| v.object == "plr" && v.field == "last_drag_result.trigger_z"));
    // The temperature interlock is method-independent.
    assert!(plan.temp_verify_precedes_probe());

    // The true-Z formula reads the drag trigger source.
    let declare = plan
        .steps_in_phase(Phase::TrueZDeclare)
        .next()
        .expect("true-z step");
    let Some(RuntimeComputation::TrueZ(formula)) = declare.compute else {
        panic!("true-z step must carry the formula");
    };
    assert_eq!(formula.trigger_source, TriggerSource::DragResult);

    // Every structural invariant holds for the drag variant too.
    assert!(plan.idle_timeout_first());
    assert!(plan.steppers_enabled_before_motion());
    assert!(plan.no_g28_after_shifted_declare());
    assert_eq!(plan.m26_offset(), Some(plan.resume_offset));

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
fn load_cell_probe_adds_z_offset_back() {
    let plan = build_plan(&machine_load_cell(), plain_transforms());
    let declare = plan
        .steps_in_phase(Phase::TrueZDeclare)
        .next()
        .expect("true-z step");
    let Some(RuntimeComputation::TrueZ(formula)) = declare.compute else {
        panic!("true-z step must carry the formula");
    };
    assert_eq!(
        formula.trigger_source,
        TriggerSource::BedZPlusOffset { z_offset: -0.15 }
    );
    // The probe verification reads the bed_z field for this probe type.
    let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe");
    assert!(probe
        .verify
        .iter()
        .any(|v| v.object == "probe" && v.field == "last_probe_position.2"));

    // The tap plan reads the raw field instead.
    let tap_plan = build_plan(&machine_tap(), plain_transforms());
    let tap_probe = tap_plan.steps_in_phase(Phase::Probe).next().expect("probe");
    assert!(tap_probe
        .verify
        .iter()
        .any(|v| v.object == "probe" && v.field == "last_z_result"));
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
    };
    let PlanOutcome::Plan(plan) = plan_recovery(&inputs, &PlanConfig::default()).unwrap() else {
        panic!("expected plan");
    };
    let select = plan
        .steps_in_phase(Phase::FileSelect)
        .next()
        .expect("file-select");
    let commands = &select.commands;
    let m23 = commands.iter().position(|c| c.starts_with("M23")).unwrap();
    let define = commands
        .iter()
        .position(|c| c.starts_with("EXCLUDE_OBJECT_DEFINE NAME=cube_1"))
        .unwrap();
    let exclude = commands
        .iter()
        .position(|c| c == "EXCLUDE_OBJECT NAME=cube_1")
        .unwrap();
    let m26 = commands
        .iter()
        .position(|c| c.starts_with("M26 S"))
        .unwrap();
    assert!(m23 < define && define < exclude && exclude < m26);
    assert!(commands[define].contains("CENTER=50,50"));
    assert!(commands[define].contains("POLYGON=[[40,40],[60,40],[60,60],[40,60]]"));
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
fn plans_round_trip_through_serde() {
    let plan = build_plan(&machine_tap(), plain_transforms());
    let json = serde_json::to_string(&plan).unwrap();
    let back: RecoveryPlan = serde_json::from_str(&json).unwrap();
    assert_eq!(back, plan);
}
