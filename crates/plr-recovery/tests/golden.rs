//! Golden-plan scenario tests: whole plans for fixture scenarios,
//! including the §-mapped step order, the probe-type `z_offset` split,
//! typed fallbacks and the clean-shutdown short-circuit.

mod common;

use plr_analyzer::{ContactOutcome, DeclineReason, MatchConfidence};
use plr_recovery::{
    plan_recovery, select_resume_target, select_resume_target_with_policy, Diagnose,
    ExcludeObjectDef, FallbackReason, FileTemps, OvershootTerm, Phase, PlanConfig, PlanInputs,
    PlanOutcome, PlanWarning, RecoveryError, RecoveryPlan, ResumePolicy, RuntimeComputation, Tier,
    TriggerSource,
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
            Phase::ProbeTempHold,
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
    // The PRINT-temperature waits belong to the recovery file's heating
    // gate, never to the plan: no M190 anywhere (the bed is only ever
    // nudged non-blocking here), and the only blocking M109 is the
    // probe-temperature hold — which waits for the PROBE temp, not the
    // print temp.
    assert!(
        !plan
            .steps
            .iter()
            .flat_map(|s| s.commands.iter())
            .any(|c| c.starts_with("M190")),
        "the blocking bed wait belongs to the recovery file, not the plan"
    );
    let m109_phases: Vec<Phase> = plan
        .steps
        .iter()
        .filter(|s| s.commands.iter().any(|c| c.starts_with("M109")))
        .map(|s| s.phase)
        .collect();
    assert_eq!(
        m109_phases,
        vec![Phase::ProbeTempHold],
        "the only blocking nozzle wait in the plan is the probe-temp hold"
    );
    let hold = plan
        .steps_in_phase(Phase::ProbeTempHold)
        .next()
        .expect("hold step");
    assert_eq!(hold.commands, vec!["M109 S145"]);
    // The recovery file itself carries the print-temperature reheat
    // behind the heating gate.
    let file = plr_recovery::build_recovery_file(
        &plan.recovery_file,
        common::MODEL_TEXT.as_bytes(),
        "TEST-TS",
    );
    assert!(plr_recovery::verify_heating_gate(&file, &plan.recovery_file).is_ok());
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
            Phase::ProbeTempHold,
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
    assert!(plr_recovery::verify_heating_gate(&file, &plan.recovery_file).is_ok());

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

/// Reads `MAX_TOUCH_TEMPERATURE_EPSILON` out of the Klipper plugin's
/// `setup_checks.py`, so the cross-language pin compares against the
/// PLUGIN's real value rather than a local copy of it.
///
/// `Err(reason)` distinguishes the two ways this can be unavailable —
/// file unreadable vs. constant missing from it — so the failure message
/// states which, instead of guessing.
fn plugin_touch_temperature_epsilon() -> Result<f64, String> {
    const CONST_NAME: &str = "MAX_TOUCH_TEMPERATURE_EPSILON";
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../klippy_plugin/plr/setup_checks.py");
    let text = std::fs::read_to_string(&path)
        .map_err(|e| format!("{} could not be read: {e}", path.display()))?;
    // `MAX_TOUCH_TEMPERATURE_EPSILON = 2.0` (tolerate spacing/comments).
    let line = text
        .lines()
        .find(|l| l.trim_start().starts_with(CONST_NAME))
        .ok_or_else(|| {
            format!(
                "{} exists but defines no {CONST_NAME} — the plugin's contact temperature \
                 gate must keep this constant for plrd to pin against",
                path.display()
            )
        })?;
    let rhs = line
        .split('=')
        .nth(1)
        .ok_or_else(|| format!("{CONST_NAME} line has no assignment: {line:?}"))?;
    let value = rhs
        .split('#')
        .next()
        .unwrap_or("")
        .trim()
        .trim_end_matches(|c: char| !(c.is_ascii_digit() || c == '.'));
    value
        .parse::<f64>()
        .map_err(|e| format!("{CONST_NAME} value {value:?} is not a number: {e}"))
}

/// The cross-language tolerance pin must be VERIFIABLE, not merely
/// satisfied.
///
/// `probe_temperature_bounds_stay_in_lockstep_with_the_plugin` asserts the
/// pin whenever the plugin constant is readable — but if the constant ever
/// becomes unreadable (the file moves, is renamed, or the definition is
/// deleted) that assertion silently has nothing to do, and a gate that
/// cannot be seen to have not-run is the same class of problem as a guard
/// that cannot fire. This test closes that hole by FAILING whenever the
/// pin cannot be evaluated at all.
///
/// Together the two tests enforce the contract from both directions: this
/// one proves the plugin's `MAX_TOUCH_TEMPERATURE_EPSILON` is still
/// reachable, and the other proves it still agrees with
/// [`plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE`].
#[test]
fn plugin_tolerance_pin_is_live() {
    match plugin_touch_temperature_epsilon() {
        Ok(plugin_epsilon) => assert!(
            (plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE - plugin_epsilon).abs() < 1e-12,
            "TOLERANCE DIVERGENCE: plrd {} vs plugin {plugin_epsilon}",
            plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE
        ),
        Err(reason) => panic!(
            "the cross-language tolerance pin is NOT verifiable: {reason}. \
             plr-recovery's PROBE_TEMP_MEASURED_TOLERANCE is {}, and with the plugin's \
             constant unreadable NOTHING checks the two against each other — \
             `probe_temperature_bounds_stay_in_lockstep_with_the_plugin` silently has \
             nothing to compare and would pass. Restore the constant (or fix this \
             test's path to it) rather than deleting this test.",
            plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE
        ),
    }
}

/// Finding 10 (cross-language contract): the Probe step's MEASURED
/// temperature bound must equal `max_probe_nozzle_temp +
/// PROBE_TEMP_MEASURED_TOLERANCE`, and its TARGET bound exactly
/// `max_probe_nozzle_temp`.
///
/// The tolerance mirrors the Klipper plugin's
/// `MAX_TOUCH_TEMPERATURE_EPSILON` in `klippy_plugin/plr/setup_checks.py`
/// (itself following Cartographer `probe/touch_mode.py:34`). plrd and the
/// plugin must refuse at the IDENTICAL boundary, so this test READS the
/// plugin's constant from the repo rather than restating it — a local
/// copy compared to a local copy would stay green while the two sides
/// diverged. The asymmetry is deliberate: measured overshoot is forgiven,
/// a hotter commanded target is not.
#[test]
fn probe_temperature_bounds_stay_in_lockstep_with_the_plugin() {
    // The plugin defines the constant on main, so this pin is LIVE: it
    // compares against the plugin's real value and fails on divergence.
    // `plugin_tolerance_pin_is_live` (above) separately guarantees the
    // constant stays READABLE, so this `if let` can never silently become
    // a no-op.
    if let Ok(plugin_epsilon) = plugin_touch_temperature_epsilon() {
        assert!(
            (plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE - plugin_epsilon).abs() < 1e-12,
            "TOLERANCE DIVERGENCE: plr-recovery's PROBE_TEMP_MEASURED_TOLERANCE is {} \
             (crates/plr-recovery/src/build.rs) but the plugin's \
             MAX_TOUCH_TEMPERATURE_EPSILON is {plugin_epsilon} \
             (klippy_plugin/plr/setup_checks.py). Both sides gate contact on the same \
             ceiling and MUST use the same tolerance, or one will refuse where the other \
             allows. Update whichever is wrong.",
            plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE
        );
    }

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
            (measured_max - (ceiling + plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE)).abs() < 1e-12,
            "measured bound {measured_max} must be the ceiling {ceiling} + {}",
            plr_recovery::PROBE_TEMP_MEASURED_TOLERANCE
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

// ---- drag temperature hold (item 5) ---------------------------------

/// The hold phase exists on BOTH probe paths and sits between homing and
/// the clean, so heat-up ooze is wiped rather than deposited.
#[test]
fn the_probe_temp_hold_precedes_the_clean_on_both_paths() {
    for machine in [machine_tap(), machine_adxl_drag()] {
        let plan = build_plan(&machine, plain_transforms());
        assert!(
            plan.probe_temp_hold_precedes_clean_nozzle(),
            "hold must sit between HomeXy and CleanNozzle"
        );
        let hold = plan.first_index(Phase::ProbeTempHold).expect("hold step");
        let home = plan.first_index(Phase::HomeXy).expect("home");
        let clean = plan.first_index(Phase::CleanNozzle).expect("clean");
        assert!(home < hold && hold < clean);
        // The hold blocks natively and confirms the band afterwards.
        let step = &plan.steps[hold];
        assert!(step.commands[0].starts_with("M109 S"));
        assert!(step.verify.iter().any(|v| v.object == "extruder"
            && v.field == "temperature"
            && matches!(v.predicate, plr_recovery::Predicate::TempWithin { .. })));
    }
}

/// A drag machine commands AND holds `drag_nozzle_temp` — the early
/// non-blocking `M104` and the blocking `M109` both carry it.
#[test]
fn a_drag_machine_commands_and_holds_the_drag_temperature() {
    let config = PlanConfig {
        drag_nozzle_temp: 120.0,
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_adxl_drag(), plain_transforms(), &config);
    let heat = plan
        .steps_in_phase(Phase::ImmediateBedHeat)
        .next()
        .expect("heat step");
    assert!(
        heat.commands.iter().any(|c| c == "M104 S120"),
        "the early non-blocking heat must target the DRAG temp: {:?}",
        heat.commands
    );
    let hold = plan
        .steps_in_phase(Phase::ProbeTempHold)
        .next()
        .expect("hold step");
    assert_eq!(hold.commands, vec!["M109 S120"]);
    // The band is the documented constant, not a knob.
    assert!(hold.verify.iter().any(|v| matches!(
        v.predicate,
        plr_recovery::Predicate::TempWithin { min, max }
            if (min - (120.0 - plr_recovery::PROBE_HOLD_BAND)).abs() < 1e-12
                && (max - (120.0 + plr_recovery::PROBE_HOLD_BAND)).abs() < 1e-12
    )));
    // A touch machine is unaffected by drag_nozzle_temp.
    let tap = build_plan_with(&machine_tap(), plain_transforms(), &config);
    let tap_hold = tap
        .steps_in_phase(Phase::ProbeTempHold)
        .next()
        .expect("hold");
    assert_eq!(tap_hold.commands, vec!["M109 S145"]);
}

/// `drag_nozzle_temp = 0` is the cold-drag opt-out: no `M104`, no `M109`,
/// no hold phase at all — and the contact CEILING still gates the probe.
#[test]
fn a_zero_drag_temperature_opts_out_of_heating_entirely() {
    let config = PlanConfig {
        drag_nozzle_temp: 0.0,
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_adxl_drag(), plain_transforms(), &config);
    assert!(
        plan.first_index(Phase::ProbeTempHold).is_none(),
        "no hold phase on an opted-out cold drag"
    );
    assert!(
        plan.probe_temp_hold_precedes_clean_nozzle(),
        "vacuously true"
    );
    let all: Vec<&String> = plan.steps.iter().flat_map(|s| s.commands.iter()).collect();
    assert!(
        !all.iter().any(|c| c.starts_with("M104")),
        "a cold drag must not be nudged warm: {all:?}"
    );
    assert!(!all.iter().any(|c| c.starts_with("M109")));
    // The ceiling gate is still on the probe step.
    let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe");
    assert!(probe.pre_verify.iter().any(|v| v.object == "extruder"
        && v.field == "temperature"
        && matches!(v.predicate, plr_recovery::Predicate::NumAtMost { .. })));
    assert!(probe
        .pre_verify
        .iter()
        .any(|v| v.object == "extruder" && v.field == "target"));
    // A touch machine still holds even with drag_nozzle_temp = 0.
    let tap = build_plan_with(&machine_tap(), plain_transforms(), &config);
    assert!(tap.first_index(Phase::ProbeTempHold).is_some());
}

/// `drag_nozzle_temp` obeys the same headroom interlock as the probe
/// temp — but only on the machine that reads it.
///
/// The interlock stays HARD: the plugin's ceiling gate would refuse the
/// drag AFTER the Z frame is declared, which wedges the recovery. What
/// changed is that it is now gated on the probe path, exactly like its
/// sibling `DragTempBelowFloor` warning — a Tap machine never commands
/// this temperature, so refusing its recovery over the value would be
/// refusing over a setting the machine does not read.
#[test]
fn an_out_of_range_drag_temperature_is_refused_on_the_drag_path_only() {
    use plr_recovery::ProbeKind;
    let drag = ProbeKind::AdxlDrag {
        chip: "adxl345".to_owned(),
    };
    // Above ceiling - headroom (150 - 5 = 145), and negative.
    for bad in [146.0, 150.0, 200.0, -1.0] {
        let config = PlanConfig {
            drag_nozzle_temp: bad,
            ..PlanConfig::default()
        };
        // Machine-independent validation no longer decides this...
        config
            .validate()
            .unwrap_or_else(|e| panic!("{bad} is not a machine-independent fault: {e:?}"));
        // ...the drag path does.
        let err = config.validate_for_probe(&drag).unwrap_err();
        assert!(
            matches!(err, RecoveryError::DragTempOutOfRange { .. }),
            "{bad}: {err:?}"
        );
        assert_eq!(err.diagnosis().tier, Tier::Hard);
        assert_eq!(err.diagnosis().override_key, None);
        let msg = err.to_string();
        assert!(msg.contains("drag_nozzle_temp"), "{msg}");
        // And a Tap / load-cell machine, which never commands it, is
        // untouched.
        for inert in [ProbeKind::Tap, ProbeKind::LoadCell] {
            config
                .validate_for_probe(&inert)
                .unwrap_or_else(|e| panic!("{bad} must not refuse on {inert:?}: {e:?}"));
        }
        // End to end: the drag machine refuses to plan, the tap machine
        // plans happily.
        let inputs_err = plan_for(&machine_adxl_drag(), &config).unwrap_err();
        assert!(matches!(
            inputs_err,
            RecoveryError::DragTempOutOfRange { .. }
        ));
        assert!(plan_for(&machine_tap(), &config).is_ok());
    }
    // 0 (the cold-drag opt-out) and the boundary are accepted everywhere.
    for ok in [0.0, 100.0, 145.0] {
        let config = PlanConfig {
            drag_nozzle_temp: ok,
            ..PlanConfig::default()
        };
        config.validate().expect("machine-independent");
        config.validate_for_probe(&drag).expect("drag path");
    }
}

/// `plan_recovery` for a machine + config, discarding the plan.
fn plan_for(
    machine: &plr_recovery::MachineConfig,
    config: &PlanConfig,
) -> Result<(), RecoveryError> {
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
    plan_recovery(&inputs, config).map(|_| ())
}

// ---- purge precedence (item 6) --------------------------------------

/// Path 1: `purge_enable = false` emits no purge of any kind.
#[test]
fn purge_disabled_emits_nothing() {
    let config = PlanConfig {
        purge_enable: false,
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_tap(), plain_transforms(), &config);
    assert!(plan.recovery_file.purge.is_none());
    let file =
        plr_recovery::build_recovery_file(&plan.recovery_file, common::MODEL_TEXT.as_bytes(), "TS");
    let text = file.preamble_text().into_owned();
    // `G92 E0` is purge-only (the entry's own E-frame reset is `G92 E<n>`
    // with the file's E value), so its absence proves no purge ran. The
    // entry prime `G1 E0.4` is a different thing and legitimately stays.
    assert!(!text.contains("G92 E0"), "{text}");
    assert!(!text.contains("F300"), "no purge extrusion: {text}");
    assert!(plr_recovery::verify_heating_gate(&file, &plan.recovery_file).is_ok());
}

/// Path 2: a present macro owns the purge entirely.
#[test]
fn purge_macro_present_owns_the_purge() {
    let config = PlanConfig {
        purge_macro: Some("MY_PURGE".to_owned()),
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
        purge_macro_present: true,
    };
    let PlanOutcome::Plan(plan) = plan_recovery(&inputs, &config).unwrap() else {
        panic!("expected plan");
    };
    assert!(matches!(
        plan.recovery_file.purge,
        Some(plr_recovery::PurgePlan::Macro { ref call }) if call == "MY_PURGE"
    ));
    let file =
        plr_recovery::build_recovery_file(&plan.recovery_file, common::MODEL_TEXT.as_bytes(), "TS");
    let text = file.preamble_text().into_owned();
    assert!(text.contains("MY_PURGE"));
    assert!(!text.contains("G92 E0"), "the macro owns everything");
}

/// Path 3: a MISSING purge macro REFUSES to plan — never a silent
/// downgrade to the built-in purge.
#[test]
fn a_missing_purge_macro_refuses_to_plan() {
    let config = PlanConfig {
        purge_macro: Some("NOPE".to_owned()),
        ..PlanConfig::default()
    };
    // build_plan_with passes purge_macro_present: false.
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
    let err = plan_recovery(&inputs, &config).unwrap_err();
    assert!(
        matches!(&err, RecoveryError::PurgeMacroMissing { name } if name == "NOPE"),
        "{err:?}"
    );
    // Zero commands emitted: there is no plan at all.
    let msg = err.to_string();
    assert!(msg.contains("NOPE"), "{msg}");
    assert!(
        msg.contains("refusing to substitute the built-in purge"),
        "{msg}"
    );
}

/// Path 4: the built-in purge defaults to the park point and honors
/// every explicit knob.
#[test]
fn the_built_in_purge_defaults_to_the_park_and_honors_knobs() {
    // Defaults: purge point == reheat park point.
    let plan = build_plan(&machine_tap(), plain_transforms());
    let park = plan.recovery_file.park;
    assert!(matches!(
        plan.recovery_file.purge,
        Some(plr_recovery::PurgePlan::BuiltIn { point, .. })
            if point.iter().zip(park).all(|(a, b)| (a - b).abs() < 1e-12)
    ));

    // Explicit knobs.
    let config = PlanConfig {
        purge_x: Some(150.0),
        purge_y: Some(12.0),
        purge_z: Some(0.8),
        purge_amount: 9.0,
        purge_speed: 240.0,
        purge_retract: 2.0,
        ..PlanConfig::default()
    };
    let mut machine = machine_tap();
    machine.axis_limits = plr_recovery::AxisLimits {
        x: Some((0.0, 200.0)),
        y: Some((0.0, 200.0)),
        z_max: Some(250.0),
    };
    let plan = build_plan_with(&machine, plain_transforms(), &config);
    assert!(matches!(
        plan.recovery_file.purge,
        Some(plr_recovery::PurgePlan::BuiltIn {
            point, z: Some(z), amount, speed, retract, ..
        }) if (point[0] - 150.0).abs() < 1e-12 && (point[1] - 12.0).abs() < 1e-12
            && (z - 0.8).abs() < 1e-12
            && (amount - 9.0).abs() < 1e-12
            && (speed - 240.0).abs() < 1e-12
            && (retract - 2.0).abs() < 1e-12
    ));
    let file =
        plr_recovery::build_recovery_file(&plan.recovery_file, common::MODEL_TEXT.as_bytes(), "TS");
    let text = file.preamble_text().into_owned();
    assert!(text.contains("G0 X150 Y12"));
    assert!(text.contains("G1 Z0.8"));
    assert!(text.contains("G1 E9 F240"));
    assert!(text.contains("G1 E-2 F240"));
    assert!(plr_recovery::verify_heating_gate(&file, &plan.recovery_file).is_ok());
}

/// Every purge coordinate flows through the generated-file pre-flight.
#[test]
fn an_out_of_bounds_purge_point_is_refused_at_plan_time() {
    let mut machine = machine_tap();
    machine.axis_limits = plr_recovery::AxisLimits {
        x: Some((0.0, 200.0)),
        y: Some((0.0, 200.0)),
        z_max: Some(250.0),
    };
    let config = PlanConfig {
        purge_x: Some(9_999.0),
        purge_y: Some(12.0),
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine, plain_transforms(), &config);
    let file =
        plr_recovery::build_recovery_file(&plan.recovery_file, common::MODEL_TEXT.as_bytes(), "TS");
    let err = plr_recovery::preflight_generated_file(&file, &machine, [20.0, 10.0])
        .expect_err("an out-of-range purge X must be refused");
    assert!(
        matches!(err, RecoveryError::ItineraryRejected(_)),
        "{err:?}"
    );
}

/// A purge landing on printed geometry warns (never refuses — a
/// sacrificial area is legitimate).
#[test]
fn a_purge_inside_the_part_warns() {
    // The fixture part spans X 10..30, Y 10..30.
    let config = PlanConfig {
        purge_x: Some(20.0),
        purge_y: Some(20.0),
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_tap(), plain_transforms(), &config);
    assert!(
        plan.warnings.iter().any(|w| matches!(
            w,
            plr_recovery::PlanWarning::PurgeInsidePart {
                configured: true,
                ..
            }
        )),
        "{:?}",
        plan.warnings
    );
    let rendered = plan.render();
    assert!(rendered.contains("built-in purge point"), "{rendered}");
    assert!(
        rendered.contains("configured via purge_x/purge_y"),
        "{rendered}"
    );
}

/// A golden for the fully-configured built-in purge, so the RETRACT and
/// the `M83` that makes it correct are both visible in a checked-in
/// artifact (the default golden has `purge_retract = 0`).
#[test]
fn a_configured_purge_matches_its_golden() {
    let config = PlanConfig {
        purge_x: Some(150.0),
        purge_y: Some(12.0),
        purge_z: Some(5.0),
        purge_amount: 9.0,
        purge_speed: 240.0,
        purge_retract: 2.0,
        ..PlanConfig::default()
    };
    let mut machine = machine_tap();
    machine.axis_limits = plr_recovery::AxisLimits {
        x: Some((0.0, 200.0)),
        y: Some((0.0, 200.0)),
        z_max: Some(250.0),
    };
    let plan = build_plan_with(&machine, plain_transforms(), &config);
    let file =
        plr_recovery::build_recovery_file(&plan.recovery_file, common::MODEL_TEXT.as_bytes(), "TS");
    assert!(plr_recovery::verify_heating_gate(&file, &plan.recovery_file).is_ok());

    let golden_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/golden/recovery_file_purge.gcode"
    );
    if std::env::var("PLR_BLESS").is_ok() {
        std::fs::write(golden_path, &file.content).expect("write golden");
    }
    let golden = std::fs::read_to_string(golden_path).expect("golden file (run with PLR_BLESS=1)");
    assert_eq!(
        String::from_utf8(file.content).expect("ASCII fixture"),
        golden.replace("\r\n", "\n")
    );
    // The retract is present, and the M83 that makes its magnitude
    // correct precedes it.
    assert!(golden.contains("G1 E-2 F240"));
    let m83 = golden.find("M83").expect("relative E");
    assert!(m83 < golden.find("G1 E-2 F240").unwrap());
}

/// Item 4 regression: `purge_z` is the one raw operator-chosen absolute Z
/// in the generated file, and the file runs in the TRUE frame where Z=0
/// is the bed. A negative value must be REFUSED — the Z rail's
/// `position_min` is deliberately below the bed (goldens use −2), so the
/// rail check accepts `-1.9` and cannot be the floor here.
#[test]
fn a_negative_purge_z_is_refused() {
    for bad in [-0.001, -1.9, -50.0] {
        let config = PlanConfig {
            purge_z: Some(bad),
            ..PlanConfig::default()
        };
        let err = config.validate().unwrap_err();
        assert!(
            matches!(err, RecoveryError::PurgeZBelowBed { purge_z } if (purge_z - bad).abs() < 1e-12),
            "purge_z {bad} must be refused, got {err:?}"
        );
        let msg = err.to_string();
        assert!(msg.contains("bed"), "{msg}");
    }
    // The rail floor really is below the bed in the fixture, which is why
    // the rail check alone would have accepted -1.9.
    assert!(machine_tap().z_position_min.unwrap() < 0.0);
    // Zero and positive are accepted.
    for ok in [0.0, 0.2, 50.0] {
        assert!(PlanConfig {
            purge_z: Some(ok),
            ..PlanConfig::default()
        }
        .validate()
        .is_ok());
    }
}

/// Item 4 (second half): a `purge_z` below the resume Z warns — the
/// descent may drive the nozzle into the part, but a purge over bare bed
/// legitimately wants a low Z, so this is a warning not a refusal.
#[test]
fn a_purge_z_below_the_resume_z_warns() {
    // The fixture resumes at Z 0.4 in the WAL frame with a 0.05 origin,
    // so the file-frame resume Z is 0.35.
    let config = PlanConfig {
        purge_z: Some(0.1),
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_tap(), plain_transforms(), &config);
    assert!(
        plan.warnings.iter().any(|w| matches!(
            w,
            plr_recovery::PlanWarning::PurgeZBelowResume { purge_z, .. }
                if (purge_z - 0.1).abs() < 1e-12
        )),
        "{:?}",
        plan.warnings
    );
    assert!(plan.render().contains("below the resume Z"));
    // Above the resume Z: no such warning.
    let config = PlanConfig {
        purge_z: Some(5.0),
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_tap(), plain_transforms(), &config);
    assert!(!plan
        .warnings
        .iter()
        .any(|w| matches!(w, plr_recovery::PlanWarning::PurgeZBelowResume { .. })));
}

/// Item 5 regression: with `purge_z` set, a purge inside the footprint is
/// a COLLISION (the file descends to that Z over the part), and the
/// warning must say so rather than describing a cosmetic blob.
#[test]
fn a_purge_inside_the_part_with_a_low_z_warns_about_collision() {
    let config = PlanConfig {
        purge_x: Some(20.0),
        purge_y: Some(20.0),
        purge_z: Some(0.2),
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_tap(), plain_transforms(), &config);
    let rendered = plan.render();
    assert!(
        plan.warnings.iter().any(|w| matches!(
            w,
            plr_recovery::PlanWarning::PurgeInsidePart {
                purge_z: Some(_),
                ..
            }
        )),
        "{:?}",
        plan.warnings
    );
    assert!(rendered.contains("COLLISION RISK"), "{rendered}");
    assert!(rendered.contains("descends to that Z"), "{rendered}");

    // WITHOUT purge_z the same point is the milder deposit warning.
    let config = PlanConfig {
        purge_x: Some(20.0),
        purge_y: Some(20.0),
        ..PlanConfig::default()
    };
    let plan = build_plan_with(&machine_tap(), plain_transforms(), &config);
    let rendered = plan.render();
    assert!(!rendered.contains("COLLISION RISK"), "{rendered}");
    assert!(rendered.contains("drop filament onto"), "{rendered}");
}

/// Item 8, re-tiered: a nonzero `drag_nozzle_temp` below the floor
/// CONFIRMS rather than refuses.
///
/// Trace the consequence: the drag path's `M109` waits for a cooldown
/// that on an enclosed machine may never converge, the executor's step
/// timeout bounds that wait, and the abort lands BEFORE the shifted-frame
/// declare — wasted time and a clean abort, not damage or an unknowable
/// frame. That is the Confirmable tier, so the operator gets the
/// explanation and a button. `0` (the cold-drag opt-out) stays silent.
#[test]
fn a_sub_floor_drag_temperature_confirms_rather_than_refusing() {
    for bad in [1.0, 30.0, 49.9] {
        let config = PlanConfig {
            drag_nozzle_temp: bad,
            ..PlanConfig::default()
        };
        // No longer a planning refusal at all.
        config
            .validate()
            .unwrap_or_else(|e| panic!("drag_nozzle_temp {bad} must not refuse: {e:?}"));
        // On a DRAG machine it raises the Confirmable diagnosis...
        let plan = build_plan_with(&machine_adxl_drag(), plain_transforms(), &config);
        let warning = plan
            .warnings
            .iter()
            .find(|w| matches!(w, PlanWarning::DragTempBelowFloor { .. }))
            .unwrap_or_else(|| panic!("drag_nozzle_temp {bad} must warn"));
        let d = warning.diagnosis();
        assert_eq!(d.code, "drag_temp_below_floor");
        assert_eq!(d.tier, Tier::Confirmable);
        assert_eq!(
            d.override_key, None,
            "a Confirmable diagnosis never names an UNSAFE_ key"
        );
        assert!(d.why.contains("chamber"), "{}", d.why);
        assert!(d.suggested_fix.contains("continue"), "{}", d.suggested_fix);
        assert!(
            (d.expected.as_ref().unwrap().min.unwrap() - plr_recovery::DRAG_TEMP_FLOOR).abs()
                < 1e-12
        );
        // ...and on a TAP machine it does not: the key is never read
        // there, and pausing a recovery over an inert setting is the
        // obstruction this framework exists to remove.
        let tap = build_plan_with(&machine_tap(), plain_transforms(), &config);
        assert!(!tap
            .warnings
            .iter()
            .any(|w| matches!(w, PlanWarning::DragTempBelowFloor { .. })));
    }
    // The opt-out and the floor itself are silent on the drag path too.
    for ok in [0.0, plr_recovery::DRAG_TEMP_FLOOR, 145.0] {
        let config = PlanConfig {
            drag_nozzle_temp: ok,
            ..PlanConfig::default()
        };
        config.validate().expect("must be accepted");
        let plan = build_plan_with(&machine_adxl_drag(), plain_transforms(), &config);
        assert!(
            !plan
                .warnings
                .iter()
                .any(|w| matches!(w, PlanWarning::DragTempBelowFloor { .. })),
            "{ok} must not warn"
        );
    }
}

/// Range refusals for each new purge knob.
#[test]
fn purge_knob_ranges_are_hard() {
    type Mutate = fn(&mut PlanConfig);
    let cases: [(&str, Mutate); 4] = [
        ("purge_amount", |c| c.purge_amount = 101.0),
        ("purge_speed", |c| c.purge_speed = 3001.0),
        ("purge_speed", |c| c.purge_speed = 0.0),
        ("purge_retract", |c| c.purge_retract = 10.1),
    ];
    for (field, set) in cases {
        let mut config = PlanConfig::default();
        set(&mut config);
        assert!(
            matches!(
                config.validate(),
                Err(RecoveryError::InvalidPlanConfig { field: f }) if f == field
            ),
            "{field} must be refused"
        );
    }
    // Boundaries accepted.
    let config = PlanConfig {
        purge_amount: 100.0,
        purge_speed: 3000.0,
        purge_retract: 10.0,
        ..PlanConfig::default()
    };
    assert!(config.validate().is_ok());
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
fn resume_policy_selects_first_mid_last() {
    let model = model();
    // Three ascending real extrusion-line offsets on layer 0.
    let o_first = offset_of("G1 X10 Y10 E1 F1800", 0);
    let o_mid = offset_of("G1 X30 Y10 E1", 0);
    let o_last = offset_of("G1 X30 Y30 E1", 0);
    assert!(o_first < o_mid && o_mid < o_last, "offsets ascending");
    let ambiguous = plr_analyzer::MatchResult {
        candidates: vec![],
        confidence: MatchConfidence::AmbiguousWindow {
            // Deliberately unsorted to prove the selector sorts.
            offsets: vec![o_mid, o_last, o_first],
        },
        skipped_unknown: 0,
    };

    // Byte-identity pin: Last == the historical policy-free selector, and
    // == an independently computed skip-forward from offsets.max().
    let today = select_resume_target(&model, &ambiguous).unwrap();
    let last = select_resume_target_with_policy(&model, &ambiguous, ResumePolicy::Last).unwrap();
    assert_eq!(
        last, today,
        "Last must be byte-identical to select_resume_target"
    );
    let expected_last = model
        .first_deposition_at_or_after(o_last)
        .unwrap()
        .span
        .start;
    assert_eq!(last.offset, expected_last);
    assert_eq!(last.offset, o_last);

    // Ask resolves as Last until the preview plan lands (increment 2).
    assert_eq!(
        select_resume_target_with_policy(&model, &ambiguous, ResumePolicy::Ask).unwrap(),
        today
    );

    // First = min offset; Mid = lower-median (index 1 of 3) = o_mid.
    let first = select_resume_target_with_policy(&model, &ambiguous, ResumePolicy::First).unwrap();
    assert_eq!(first.offset, o_first);
    let mid = select_resume_target_with_policy(&model, &ambiguous, ResumePolicy::Mid).unwrap();
    assert_eq!(mid.offset, o_mid);

    // Mutation proof: the three extremes are distinct, so a selector that
    // returned the wrong offset (min for Last, etc.) is caught here.
    assert_ne!(first.offset, last.offset);
    assert_ne!(mid.offset, last.offset);
    assert_ne!(first.offset, mid.offset);

    // Even-count lower median: 4 offsets -> index (4-1)/2 = 1 (2nd).
    let four = plr_analyzer::MatchResult {
        candidates: vec![],
        confidence: MatchConfidence::AmbiguousWindow {
            offsets: vec![o_first, o_mid, o_last, o_last + 1],
        },
        skipped_unknown: 0,
    };
    // The 2nd-smallest offset is o_mid; first_deposition_at_or_after lands
    // there (o_mid is itself a deposition line).
    assert_eq!(
        select_resume_target_with_policy(&model, &four, ResumePolicy::Mid)
            .unwrap()
            .offset,
        o_mid
    );

    // UniqueLine ignores the policy entirely.
    let unique = match_at(resume_offset());
    for policy in [
        ResumePolicy::First,
        ResumePolicy::Mid,
        ResumePolicy::Last,
        ResumePolicy::Ask,
    ] {
        assert_eq!(
            select_resume_target_with_policy(&model, &unique, policy)
                .unwrap()
                .offset,
            resume_offset()
        );
    }
}

/// Cross-crate byte-identity pin (MAJOR-2 / MINOR-1): the preview default
/// cursor (`last_index`) and the `mid` anchor COMMIT exactly the resume
/// `select_resume_target_with_policy` commits for `Last`/`Mid` — even when
/// the max matcher candidate is a TRAVEL line the extrusion-only preview
/// stop set cannot hold. The divergence the reviewer found is confined to
/// these anchors; `select_resume_target_with_policy` itself stays
/// byte-identical to the historical selector (pinned above).
#[test]
fn preview_anchors_match_recovery_policy_resume_byte_for_byte() {
    use plr_analyzer::{
        build_layer_model, build_preview, match_stop_point, ByteWindow, Interval, MatchConfig,
        ModelConfig, PreviewBounds, PreviewOutcome, StopEvidence,
    };

    // Crash-during-wipe: the latest matcher candidate (T3) is a travel.
    // A  (candidate ext):  X10Y10 -> X40Y10 crosses the box.
    // B  (non-cand ext):   X40Y10 -> X40Y60 leaves the box.
    // T1 (non-cand travel):X40Y60 -> X40Y10.
    // T2 (candidate travel):X40Y10 -> X25Y10 ends in the box.
    // T3 (candidate travel):X25Y10 -> X10Y40 starts in the box.
    // C  (non-cand ext):   X10Y40 -> X10Y60.
    let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        G1 X10 Y10 F9000\n\
        G1 X40 Y10 E0.5 F1800\n\
        G1 X40 Y60 E0.5 F1800\n\
        G1 X40 Y10 F9000\n\
        G1 X25 Y10 F9000\n\
        G1 X10 Y40 F9000\n\
        G1 X10 Y60 E0.5 F1800\n";
    let m = build_layer_model(
        plr_gcode::GcodeState::new(),
        text.as_bytes(),
        0,
        &ModelConfig::default(),
    );
    let evidence = StopEvidence {
        x: Interval {
            min: 24.0,
            max: 26.0,
        },
        y: Interval {
            min: 9.8,
            max: 10.2,
        },
        e: None,
        z_candidates: vec![],
        window: ByteWindow {
            start: 0,
            end: None,
        },
    };
    let cfg = MatchConfig::default();
    let result = match_stop_point(&m, &evidence, &cfg).expect("match");
    let MatchConfidence::AmbiguousWindow { offsets } = &result.confidence else {
        panic!("expected AmbiguousWindow, got {:?}", result.confidence);
    };
    let a_off = text.find("G1 X40 Y10 E0.5").unwrap() as u64;
    assert_ne!(
        *offsets.iter().max().unwrap(),
        a_off,
        "the max candidate is a travel line, not the extrusion A"
    );

    let PreviewOutcome::Preview(set) =
        build_preview(&m, &evidence, &cfg, None, &PreviewBounds::default())
    else {
        panic!("expected a preview");
    };

    // The load-bearing equalities: each anchor's committed resume equals the
    // recovery policy's resume for that policy.
    let last = select_resume_target_with_policy(&m, &result, ResumePolicy::Last).unwrap();
    assert_eq!(
        set.stops[set.last_index as usize].resume_offset, last.offset,
        "default cursor commits select_resume_target_with_policy(Last)'s resume"
    );
    let mid = select_resume_target_with_policy(&m, &result, ResumePolicy::Mid).unwrap();
    assert_eq!(
        set.stops[set.mid_index as usize].resume_offset, mid.offset,
        "mid anchor commits select_resume_target_with_policy(Mid)'s resume"
    );
    // Last is still byte-identical to the historical selector (skip-forward
    // resume unchanged), and the previewed default stop is the PREDECESSOR of
    // the resume line, not the max EXTRUSION candidate (the pre-fix bug).
    assert_eq!(last, select_resume_target(&m, &result).unwrap());
    assert!(
        set.stops[set.last_index as usize].offset < last.offset,
        "the default stop precedes the resume it commits"
    );
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

// --- Confirm-point and acceleration steps ------------------------------------
//
// The load-bearing claim in this section is INERTNESS: a machine that
// configures none of these keys must get byte-for-byte the plan it got
// before they existed. Every test below therefore compares against the
// default-config plan rather than merely asserting the new steps are
// absent — "absent" would still permit a whitespace change in a summary
// somewhere.

/// The three phases these features introduce.
const NEW_PHASES: [Phase; 3] = [
    Phase::RecoveryAccel,
    Phase::ZConfirmStandoff,
    Phase::RecoveryAccelRestore,
];

#[test]
fn confirm_points_and_accel_overrides_are_inert_when_unset() {
    let baseline = build_plan(&machine_tap(), plain_transforms());
    for phase in NEW_PHASES {
        assert!(
            baseline.first_index(phase).is_none(),
            "{phase:?} must not exist without its config key"
        );
    }
    assert!(!baseline.debug_confirm_each_step);
    // Explicitly writing the defaults changes nothing at all — the plan
    // is identical, and so is its rendering and its JSON.
    let explicit = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
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
            ..PlanConfig::default()
        },
    );
    assert_eq!(explicit, baseline);
    assert_eq!(explicit.render(), baseline.render());
    assert_eq!(
        serde_json::to_string(&explicit).unwrap(),
        serde_json::to_string(&baseline).unwrap(),
        "a disabled feature must not even appear in the serialized plan"
    );
}

#[test]
fn confirm_z_before_resume_adds_a_standoff_that_cannot_descend() {
    let plan = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            confirm_z_before_resume: true,
            ..PlanConfig::default()
        },
    );
    let index = plan
        .first_index(Phase::ZConfirmStandoff)
        .expect("standoff step");
    let true_z = plan.first_index(Phase::TrueZDeclare).expect("true-z");
    assert!(
        true_z < index,
        "the confirmation must come AFTER Z is established"
    );
    let step = &plan.steps[index];
    // It reuses the rail-clamped park arithmetic with the entry hop as
    // the delta — `park_z_at` clamps down to the rail but never below the
    // current Z, so the move is structurally incapable of descending.
    let Some(RuntimeComputation::ParkZ { delta_z, .. }) = step.compute else {
        panic!(
            "expected the rail-clamped ParkZ computation, got {:?}",
            step.compute
        );
    };
    assert!((delta_z - PlanConfig::default().entry_hop).abs() < 1e-12);
    assert!(step.commands.iter().any(|c| c == "G90"));
    assert!(
        step.commands.iter().any(|c| c.starts_with("G1 Z{park_z}")),
        "{:?}",
        step.commands
    );
    // No descent is expressible: the only Z word is the clamped
    // placeholder, never a literal.
    assert!(
        !step
            .commands
            .iter()
            .any(|c| c.contains('Z') && !c.contains("{park_z}")),
        "{:?}",
        step.commands
    );
}

#[test]
fn debug_confirm_each_step_rides_on_the_plan_and_changes_no_command() {
    let baseline = build_plan(&machine_tap(), plain_transforms());
    let plan = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            debug_confirm_each_step: true,
            ..PlanConfig::default()
        },
    );
    assert!(plan.debug_confirm_each_step);
    // Every command is untouched: this is a pause, not a plan change.
    let commands = |p: &RecoveryPlan| -> Vec<String> {
        p.steps.iter().flat_map(|s| s.commands.clone()).collect()
    };
    assert_eq!(commands(&plan), commands(&baseline));
    assert!(plan.render().contains("pauses before EVERY step"));
}

#[test]
fn a_recovery_accel_override_clamps_early_and_restores_on_both_paths() {
    let plan = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            recovery_accel: Some(2_500.0),
            ..PlanConfig::default()
        },
    );
    let clamp = plan.first_index(Phase::RecoveryAccel).expect("clamp step");
    let restore = plan
        .first_index(Phase::RecoveryAccelRestore)
        .expect("restore step");
    // Before every motion, and after everything that moves.
    let first_motion = plan
        .steps
        .iter()
        .position(|s| {
            s.commands
                .iter()
                .any(|c| c.starts_with("G28") || c.starts_with("G0 ") || c.starts_with("G1 "))
        })
        .expect("some motion");
    assert!(clamp < first_motion, "the clamp must precede any motion");
    assert!(clamp < restore);
    // The success path restores before the recovery file is selected: a
    // resumed print must run at the machine's own acceleration.
    let select = plan
        .first_index(Phase::RecoveryFileSelect)
        .expect("file select");
    assert!(restore < select, "restore before M23/M24");
    assert_eq!(
        plan.steps[clamp].commands,
        vec!["SET_VELOCITY_LIMIT ACCEL=2500".to_owned()]
    );
    assert_eq!(
        plan.steps[clamp].compute,
        Some(RuntimeComputation::RecordMachineAccel)
    );
    // The abort path goes through the existing cleanup mechanism.
    assert_eq!(
        plan.steps[clamp].cleanup_commands,
        vec!["SET_VELOCITY_LIMIT ACCEL={machine_accel}".to_owned()]
    );
    assert_eq!(
        plan.steps[restore].commands,
        vec!["SET_VELOCITY_LIMIT ACCEL={machine_accel}".to_owned()]
    );
}

#[test]
fn per_phase_accel_overrides_lead_their_phase_and_still_restore() {
    let plan = build_plan_with(
        &machine_adxl_drag(),
        plain_transforms(),
        &PlanConfig {
            accel_home: Some(1_000.0),
            accel_travel: Some(3_000.0),
            accel_probe: Some(400.0),
            accel_entry: Some(600.0),
            confirm_z_before_resume: true,
            ..PlanConfig::default()
        },
    );
    let leading = |phase: Phase| -> String {
        let i = plan
            .first_index(phase)
            .unwrap_or_else(|| panic!("{phase:?}"));
        plan.steps[i].commands.first().cloned().unwrap_or_default()
    };
    assert_eq!(leading(Phase::HomeXy), "SET_VELOCITY_LIMIT ACCEL=1000");
    assert_eq!(
        leading(Phase::ProbeApproach),
        "SET_VELOCITY_LIMIT ACCEL=3000"
    );
    assert_eq!(leading(Phase::Probe), "SET_VELOCITY_LIMIT ACCEL=400");
    assert_eq!(
        leading(Phase::ZConfirmStandoff),
        "SET_VELOCITY_LIMIT ACCEL=600"
    );
    assert_eq!(
        leading(Phase::ParkForReheat),
        "SET_VELOCITY_LIMIT ACCEL=600"
    );
    // Even with no global `recovery_accel`, the record/restore pair
    // exists — otherwise a per-phase override would simply be left in
    // force on the machine forever.
    let record = plan.first_index(Phase::RecoveryAccel).expect("record step");
    assert!(plan.first_index(Phase::RecoveryAccelRestore).is_some());
    assert!(
        plan.steps[record].commands.is_empty(),
        "with no global override the step only RECORDS: {:?}",
        plan.steps[record].commands
    );
}

#[test]
fn accel_probe_is_ignored_and_announced_on_the_consensus_touch_path() {
    let plan = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            accel_probe: Some(400.0),
            ..PlanConfig::default()
        },
    );
    // touch_accel owns the contact accel there, through the existing
    // AccelClamp step; accel_probe must not also appear on the probe.
    let probe = plan.first_index(Phase::Probe).expect("probe");
    assert!(
        !plan.steps[probe]
            .commands
            .iter()
            .any(|c| c.contains("ACCEL=400")),
        "{:?}",
        plan.steps[probe].commands
    );
    let warning = plan
        .warnings
        .iter()
        .find(|w| matches!(w, PlanWarning::AccelProbeIgnoredOnTouchPath { .. }))
        .expect("the ignored key must be announced, never swallowed");
    let d = warning.diagnosis();
    assert_eq!(d.code, "accel_probe_ignored_on_touch_path");
    assert_eq!(d.tier, Tier::Advisory);
}

#[test]
fn an_unsafe_override_in_force_is_announced_in_the_plan() {
    let plan = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            purge_z: Some(-0.5),
            unsafe_allow_purge_z_below_bed: true,
            ..PlanConfig::default()
        },
    );
    let warning = plan
        .warnings
        .iter()
        .find(|w| matches!(w, PlanWarning::UnsafeOverrideActive { .. }))
        .expect("an override that fires silently is a booby trap");
    let d = warning.diagnosis();
    assert_eq!(d.code, "unsafe_override_active");
    assert_eq!(d.tier, Tier::Advisory);
    assert!(
        d.what.contains("UNSAFE_allow_purge_z_below_bed"),
        "{}",
        d.what
    );
    // A machine that has the key set but no offending value gets no
    // warning: the announcement is about what was permitted, not about
    // what is merely permissible.
    let quiet = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            unsafe_allow_purge_z_below_bed: true,
            ..PlanConfig::default()
        },
    );
    assert!(!quiet
        .warnings
        .iter()
        .any(|w| matches!(w, PlanWarning::UnsafeOverrideActive { .. })));
}

/// `accel_entry` reaches the GENERATED FILE, which is where the entry
/// moves actually live.
///
/// The plan-level phases it also covers are the standoff and the park;
/// the descent toward the part is in the recovery file, so a version of
/// this feature that stopped at the plan would miss the one motion that
/// matters most.
#[test]
fn accel_entry_reaches_the_recovery_file_entry_moves() {
    let plan = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            accel_entry: Some(600.0),
            ..PlanConfig::default()
        },
    );
    // The shared fixture machine reports [printer] max_accel = 3000, so
    // the file can name what to restore to.
    assert_eq!(plan.recovery_file.entry_accel, Some((600.0, 3_000.0)));
    let file = plr_recovery::build_recovery_file(&plan.recovery_file, b"", "TS");
    let text = file.preamble_text().into_owned();
    assert!(text.contains("SET_VELOCITY_LIMIT ACCEL=600"), "{text}");
    assert!(text.contains("SET_VELOCITY_LIMIT ACCEL=3000"), "{text}");
    // Unset: nothing in the file at all.
    let unset = build_plan(&machine_tap(), plain_transforms());
    assert_eq!(unset.recovery_file.entry_accel, None);
    assert!(
        !plr_recovery::build_recovery_file(&unset.recovery_file, b"", "TS")
            .preamble_text()
            .contains("SET_VELOCITY_LIMIT")
    );
}

/// Both or neither: with the machine's own `max_accel` unknown (the
/// legacy `[machine]` path) the file gets no clamp, because a clamp it
/// could not undo would govern the entire remaining print. The plan says
/// so rather than silently dropping the key.
#[test]
fn an_unknown_machine_accel_skips_the_file_clamp_and_says_so() {
    let mut machine = machine_tap();
    machine.max_accel = None;
    let plan = build_plan_with(
        &machine,
        plain_transforms(),
        &PlanConfig {
            accel_entry: Some(600.0),
            ..PlanConfig::default()
        },
    );
    assert_eq!(plan.recovery_file.entry_accel, None);
    let warning = plan
        .warnings
        .iter()
        .find(|w| matches!(w, PlanWarning::AccelEntryNotAppliedToFile { .. }))
        .expect("a dropped key must be announced");
    let d = warning.diagnosis();
    assert_eq!(d.code, "accel_entry_not_applied_to_file");
    assert_eq!(d.tier, Tier::Advisory);
    // The plan-level near-part moves still honour it.
    let park = plan.first_index(Phase::ParkForReheat).expect("park");
    assert_eq!(
        plan.steps[park].commands.first().map(String::as_str),
        Some("SET_VELOCITY_LIMIT ACCEL=600")
    );
    // And a machine that DOES know its accel raises no such warning.
    let known = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            accel_entry: Some(600.0),
            ..PlanConfig::default()
        },
    );
    assert!(!known
        .warnings
        .iter()
        .any(|w| matches!(w, PlanWarning::AccelEntryNotAppliedToFile { .. })));
}

/// `confirm_timeout_s` rides onto the plan so the executor can honour the
/// operator's setting rather than only the daemon's default.
#[test]
fn the_confirm_timeout_rides_onto_the_plan() {
    let plan = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            confirm_timeout_s: Some(120.0),
            ..PlanConfig::default()
        },
    );
    assert_eq!(plan.confirm_timeout_s, Some(120.0));
    // Absent leaves the daemon default in force, and keeps the plan
    // byte-identical to one built before the key existed.
    let baseline = build_plan(&machine_tap(), plain_transforms());
    assert_eq!(baseline.confirm_timeout_s, None);
    assert!(!serde_json::to_string(&baseline)
        .unwrap()
        .contains("confirm_timeout_s"));
}

/// `gcode_barrier_timeout_s` rides onto the plan the same way, so the
/// executor's per-step re-check waits as long as the operator asked rather
/// than as long as the daemon guessed.
#[test]
fn the_gcode_barrier_timeout_rides_onto_the_plan() {
    let plan = build_plan_with(
        &machine_tap(),
        plain_transforms(),
        &PlanConfig {
            gcode_barrier_timeout_s: Some(45.0),
            ..PlanConfig::default()
        },
    );
    assert_eq!(plan.gcode_barrier_timeout_s, Some(45.0));
    let baseline = build_plan(&machine_tap(), plain_transforms());
    assert_eq!(baseline.gcode_barrier_timeout_s, None);
    assert!(!serde_json::to_string(&baseline)
        .unwrap()
        .contains("gcode_barrier_timeout_s"));
}
