//! Property tests: envelope arithmetic, plan ordering invariants over
//! randomized valid inputs, guard-scan totality, and totality on
//! hostile (non-finite) inputs — no panic, no plan carrying a
//! non-finite number, ever.

mod common;

use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

use plr_analyzer::{MatchConfidence, MatchResult};
use plr_gcode::LineIter;
use plr_recovery::{
    compute_envelope, fmt_num, plan_recovery, preflight_itinerary, sanitize_macro_text,
    scan_macro_text, true_z_at_halt, EnvelopeParams, ExcludeObjectDef, FileTemps, GuardOutcome,
    ItineraryBounds, OvershootTerm, Phase, PlanConfig, PlanInputs, PlanOutcome, PlanRejection,
    RecoveryError, RecoveryPlan, TriggerSource, TrueZFormula, ViolationKind,
};

use common::{
    contact_at, machine_adxl_drag, machine_tap, model, offset_of, plain_transforms, recovery,
    stop_set, wal_context, MODEL_TEXT,
};

/// Every generated/rendered number must be finite: check that no token
/// of the rendered plan is `nan`/`inf` (token-exact, so "infill" never
/// false-positives) and that the `fmt_num` non-finite sentinel never
/// appears.
fn assert_rendered_finite(rendered: &str) {
    let lower = rendered.to_ascii_lowercase();
    assert!(!lower.contains("invalid"), "non-finite sentinel in plan");
    for token in lower.split(|c: char| !(c.is_ascii_alphanumeric() || c == '.' || c == '-')) {
        let t = token.trim_start_matches('-');
        assert!(t != "nan" && t != "inf", "non-finite token in plan");
    }
}

/// Z values appearing in a command (`Z=<v>` or `Z<v>` words).
fn z_values(command: &str) -> Vec<f64> {
    command
        .split_whitespace()
        .filter_map(|w| {
            w.strip_prefix("Z=")
                .or_else(|| w.strip_prefix('Z'))
                .and_then(|v| v.parse::<f64>().ok())
        })
        .collect()
}

/// All line-start offsets of the fixture print file.
fn line_starts() -> Vec<u64> {
    LineIter::new(MODEL_TEXT.as_bytes(), 0)
        .map(|line| line.span.start)
        .collect()
}

/// A randomized valid planning scenario.
#[derive(Debug, Clone)]
struct Scenario {
    z_lo: f64,
    hop: f64,
    probe_speed: f64,
    margin: f64,
    sag: f64,
    point: [f64; 2],
    origin_z: f64,
    speed_factor: f64,
    extrude_factor: f64,
    speed_raw: f64,
    nozzle: f64,
    bed: Option<f64>,
    mesh: u8, // 0 none, 1 named, 2 adaptive
    z_thermal: Option<f64>,
    skew: bool,
    match_choice: u8,
    position_min: f64,
    exclude_count: usize,
    /// `true` selects the ADXL drag machine; `false` the tap machine.
    drag: bool,
    drag_z_step: f64,
}

fn scenario_strategy() -> impl Strategy<Value = Scenario> {
    (
        (
            0.2..30.0_f64,
            0.0..0.6_f64,
            1.0..=2.0_f64,
            0.0..2.0_f64,
            0.0..1.0_f64,
            (5.0..200.0_f64, 5.0..200.0_f64),
        ),
        (
            -0.1..0.1_f64,
            0.5..2.0_f64,
            0.5..2.0_f64,
            600.0..30_000.0_f64,
            170.0..300.0_f64,
            proptest::option::of(40.0..110.0_f64),
        ),
        (
            0..3_u8,
            proptest::option::of(-0.1..0.1_f64),
            any::<bool>(),
            0..4_u8,
            -3.0..0.0_f64,
            0..3_usize,
        ),
        (any::<bool>(), 0.005..=0.1_f64),
    )
        .prop_map(
            |(
                (z_lo, hop, probe_speed, margin, sag, (px, py)),
                (origin_z, speed_factor, extrude_factor, speed_raw, nozzle, bed),
                (mesh, z_thermal, skew, match_choice, position_min, exclude_count),
                (drag, drag_z_step),
            )| Scenario {
                z_lo,
                hop,
                probe_speed,
                margin,
                sag,
                point: [px, py],
                origin_z,
                speed_factor,
                extrude_factor,
                speed_raw,
                nozzle,
                bed,
                mesh,
                z_thermal,
                skew,
                match_choice,
                position_min,
                exclude_count,
                drag,
                drag_z_step,
            },
        )
}

/// Builds the plan for a scenario; panics only on outcomes the
/// generator makes impossible.
fn build_scenario(s: &Scenario) -> RecoveryPlan {
    let mut transforms = plain_transforms();
    match s.mesh {
        1 => {
            transforms.bed_mesh_active = true;
            transforms.bed_mesh_profile = Some("default".to_owned());
        }
        2 => {
            transforms.bed_mesh_active = true;
            transforms.bed_mesh_profile = Some(String::new());
        }
        _ => {}
    }
    if let Some(adjust) = s.z_thermal {
        transforms.z_thermal_adjust_enabled = Some(true);
        transforms.z_thermal_adjust_offset = Some(adjust);
    }
    if s.skew {
        transforms.skew_active = true;
        transforms.skew_profile = Some("cal".to_owned());
    }
    let mut context = wal_context(transforms);
    context.gcode.homing_origin[2] = s.origin_z;
    context.gcode.speed_factor = s.speed_factor;
    context.gcode.extrude_factor = s.extrude_factor;
    context.gcode.speed = s.speed_raw;
    context.heaters[0].target = s.nozzle;
    match s.bed {
        Some(bed) => context.heaters[1].target = bed,
        None => context.heaters.truncate(1),
    }

    let reconstruction = recovery(stop_set(&[s.z_lo, s.z_lo + s.hop]), context);

    let mut contact = contact_at((s.z_lo - 0.2).max(0.05));
    if let plr_analyzer::ContactOutcome::Candidates(c) = &mut contact {
        c[0].point = s.point;
    }

    let first_l1 = offset_of("G1 X10 Y10 E1 F1800", 1);
    let first_l0 = offset_of("G1 X10 Y10 E1 F1800", 0);
    let last_l1 = offset_of("G1 X30 Y30 E1", 1);
    let confidence = match s.match_choice {
        0 => MatchConfidence::UniqueLine { offset: first_l1 },
        1 => MatchConfidence::UniqueLine { offset: first_l0 },
        2 => MatchConfidence::AmbiguousWindow {
            offsets: vec![first_l0, first_l1],
        },
        _ => MatchConfidence::UniqueLine { offset: last_l1 },
    };
    let match_result = MatchResult {
        candidates: vec![],
        confidence,
        skipped_unknown: 0,
    };

    let excludes: Vec<ExcludeObjectDef> = (0..s.exclude_count)
        .map(|i| ExcludeObjectDef {
            name: format!("obj{i}"),
            center: Some([10.0 + f64::from(u32::try_from(i).unwrap_or(0)), 20.0]),
            polygon: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            currently_excluded: i % 2 == 0,
        })
        .collect();

    let mut machine = if s.drag {
        machine_adxl_drag()
    } else {
        machine_tap()
    };
    machine.z_position_min = Some(s.position_min);

    let config = PlanConfig {
        probe_speed: s.probe_speed,
        margin: s.margin,
        sag_allowance: s.sag,
        drag_z_step: s.drag_z_step,
        ..PlanConfig::default()
    };

    let model = model();
    let inputs = PlanInputs {
        machine: &machine,
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps: FileTemps::default(),
        exclude_objects: &excludes,
        clean_nozzle_macro_present: true,
        purge_macro_present: false,
    };
    match plan_recovery(&inputs, &config) {
        Ok(PlanOutcome::Plan(plan)) => *plan,
        other => panic!("valid scenario must plan, got {other:?}"),
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::default()
    })]

    /// Envelope formula: monotonic in gap, speed and margin; exact.
    #[test]
    fn envelope_is_monotonic_and_exact(
        gap in 0.0..50.0_f64,
        delta in 0.0..10.0_f64,
        speed in 1.0..=2.0_f64,
        speed_hi in 1.0..=2.0_f64,
        margin in 0.0..5.0_f64,
        margin_delta in 0.0..5.0_f64,
        position_min in -5.0..5.0_f64,
    ) {
        let post = |probe_speed| OvershootTerm::PostTriggerTravel { probe_speed };
        let base = compute_envelope(
            EnvelopeParams { expected_gap: gap, overshoot: post(speed), margin },
            position_min,
        ).unwrap();
        // Exact formula.
        prop_assert!((base.envelope - (gap + 0.15 * speed + margin)).abs() < 1e-12);
        prop_assert!((base.shifted_declare_z - (position_min + base.envelope)).abs() < 1e-12);
        // Monotonic in gap (z_span growth ⇒ envelope growth).
        let wider = compute_envelope(
            EnvelopeParams { expected_gap: gap + delta, overshoot: post(speed), margin },
            position_min,
        ).unwrap();
        prop_assert!(wider.envelope >= base.envelope);
        prop_assert!((wider.envelope - base.envelope - delta).abs() < 1e-9);
        // Monotonic in margin.
        let padded = compute_envelope(
            EnvelopeParams { expected_gap: gap, overshoot: post(speed), margin: margin + margin_delta },
            position_min,
        ).unwrap();
        prop_assert!(padded.envelope >= base.envelope);
        // Monotonic in speed.
        let (lo, hi) = if speed <= speed_hi { (speed, speed_hi) } else { (speed_hi, speed) };
        let e_lo = compute_envelope(
            EnvelopeParams { expected_gap: gap, overshoot: post(lo), margin }, position_min,
        ).unwrap();
        let e_hi = compute_envelope(
            EnvelopeParams { expected_gap: gap, overshoot: post(hi), margin }, position_min,
        ).unwrap();
        prop_assert!(e_hi.envelope >= e_lo.envelope);
    }

    /// Drag envelope: exact (`gap + drag_z_step + margin` — no
    /// speed-proportional term exists for fixed-Z passes), monotonic in
    /// the staircase decrement, and hostile decrements are typed
    /// errors.
    #[test]
    fn drag_envelope_is_exact_and_monotonic(
        gap in 0.0..50.0_f64,
        z_step in 0.001..0.5_f64,
        step_delta in 0.0..0.5_f64,
        margin in 0.0..5.0_f64,
        position_min in -5.0..5.0_f64,
    ) {
        let drag = |drag_z_step| OvershootTerm::DragStep { drag_z_step };
        let base = compute_envelope(
            EnvelopeParams { expected_gap: gap, overshoot: drag(z_step), margin },
            position_min,
        ).unwrap();
        prop_assert!((base.envelope - (gap + z_step + margin)).abs() < 1e-12);
        prop_assert!((base.shifted_declare_z - (position_min + base.envelope)).abs() < 1e-12);
        // Monotonic in the decrement: a coarser staircase needs a
        // larger envelope.
        let coarser = compute_envelope(
            EnvelopeParams { expected_gap: gap, overshoot: drag(z_step + step_delta), margin },
            position_min,
        ).unwrap();
        prop_assert!(coarser.envelope >= base.envelope);
        prop_assert!((coarser.envelope - base.envelope - step_delta).abs() < 1e-9);
    }

    /// First-pass clearance (see the envelope module docs, DragStep
    /// arm): PLR_DRAG_PROBE treats first-pass contact as a typed
    /// failure (no clean pass exists to be the datum), so the
    /// staircase start must clear the highest plausible surface —
    /// `position_min + expected_gap` in the shifted frame — by at
    /// least one `drag_z_step`. Holds by construction for every valid
    /// parameter set.
    #[test]
    fn drag_start_clears_the_highest_surface(
        gap in 0.0..50.0_f64,
        z_step in 0.001..0.5_f64,
        margin in 0.0..5.0_f64,
        position_min in -5.0..5.0_f64,
    ) {
        let e = compute_envelope(
            EnvelopeParams {
                expected_gap: gap,
                overshoot: OvershootTerm::DragStep { drag_z_step: z_step },
                margin,
            },
            position_min,
        ).unwrap();
        let highest_surface = position_min + gap;
        prop_assert!(
            e.shifted_declare_z - highest_surface >= z_step - 1e-12,
            "start {} must clear the highest surface {} by >= drag_z_step {}",
            e.shifted_declare_z, highest_surface, z_step
        );
    }

    /// Non-positive or non-finite drag decrements are typed errors.
    #[test]
    fn drag_z_step_rejection_is_total(bad in prop_oneof![
        -1000.0..=0.0_f64,
        Just(f64::NAN),
        Just(f64::INFINITY),
        Just(f64::NEG_INFINITY),
    ]) {
        let err = compute_envelope(
            EnvelopeParams {
                expected_gap: 1.0,
                overshoot: OvershootTerm::DragStep { drag_z_step: bad },
                margin: 0.5,
            },
            0.0,
        ).unwrap_err();
        let typed = matches!(
            err,
            RecoveryError::NonFinite { field: "drag_z_step" }
                | RecoveryError::InvalidPlanConfig { field: "drag_z_step" }
        );
        prop_assert!(typed);
    }

    /// The speed band is enforced for every out-of-band value.
    #[test]
    fn envelope_speed_cap_is_hard(speed in prop_oneof![
        -1000.0..0.999_f64,
        2.001..1000.0_f64,
        Just(f64::NAN),
        Just(f64::INFINITY),
        Just(f64::NEG_INFINITY),
    ]) {
        let err = compute_envelope(
            EnvelopeParams {
                expected_gap: 1.0,
                overshoot: OvershootTerm::PostTriggerTravel { probe_speed: speed },
                margin: 0.5,
            },
            0.0,
        ).unwrap_err();
        let out_of_range = matches!(err, RecoveryError::ProbeSpeedOutOfRange { .. });
        prop_assert!(out_of_range);
    }

    /// Every ordering invariant holds on every valid randomized plan;
    /// the M26 offset is a line boundary; probe-phase Z values stay
    /// inside the envelope; rendered plans carry only finite numbers.
    #[test]
    fn plan_invariants_hold_on_random_valid_inputs(s in scenario_strategy()) {
        let plan = build_scenario(&s);

        prop_assert!(plan.idle_timeout_first());
        prop_assert!(plan.steppers_enabled_before_motion());
        prop_assert!(plan.temp_verify_precedes_probe());
        prop_assert!(plan.z_thermal_freeze_precedes_shifted_declare());
        prop_assert!(plan.probe_step_precedes_mesh_load());
        prop_assert!(plan.mesh_load_precedes_final_declare());
        prop_assert!(plan.no_g28_after_shifted_declare());
        // New recovery-UX ordering guarantees:
        // bed heat first (M140 before any G28/motion), believed-Z + lift
        // before HomeXy, clean-nozzle between home and shifted frame,
        // park before restore, file-select last.
        prop_assert!(plan.bed_heat_precedes_motion());
        prop_assert!(plan.believed_z_precedes_home_xy());
        prop_assert!(plan.probe_temp_hold_precedes_clean_nozzle());
        prop_assert!(plan.clean_nozzle_between_home_and_shifted());
        prop_assert!(plan.park_precedes_restore());
        prop_assert!(plan.recovery_file_select_last());
        // The hold phase exists on every path here (the generator never
        // opts out of drag heating), and always blocks with an M109.
        let hold = plan
            .steps_in_phase(Phase::ProbeTempHold)
            .next()
            .expect("probe-temp hold step");
        prop_assert!(hold.commands[0].starts_with("M109 S"));
        // Accel clamp precedes the probe, restore follows on success,
        // and the clamp declares an abort cleanup — for every valid
        // plan (vacuously so on the drag path, which has no clamp).
        prop_assert!(plan.accel_clamp_precedes_probe());
        prop_assert!(plan.accel_restore_follows_probe());
        prop_assert!(plan.accel_clamp_declares_cleanup());
        // The accel-clamp steps exist exactly on the consensus-touch
        // (tap/load-cell, non-legacy) path: present for tap, absent for
        // drag.
        prop_assert_eq!(plan.first_index(Phase::AccelClamp).is_some(), !s.drag);
        prop_assert_eq!(plan.first_index(Phase::AccelRestore).is_some(), !s.drag);

        // z_thermal step present exactly when the module is configured.
        prop_assert_eq!(
            plan.first_index(Phase::TransformFreeze).is_some(),
            s.z_thermal.is_some()
        );
        // Mesh step present exactly for a restorable (named) mesh.
        prop_assert_eq!(plan.first_index(Phase::MeshLoad).is_some(), s.mesh == 1);

        // The resume offset (the recovery file's verbatim-tail start) is
        // a line boundary of the original file, and the recovery-file
        // spec agrees with it.
        prop_assert!(line_starts().contains(&plan.resume_offset));
        prop_assert_eq!(plan.recovery_file.tail_offset, plan.resume_offset);
        prop_assert!(plan.resume_file.ends_with("_RECOVERY.gcode"));
        prop_assert_eq!(&plan.recovery_file.name, &plan.resume_file);
        // The clean-nozzle confirmation flag is the negation of macro
        // presence (this generator always sets it present).
        prop_assert!(!plan.requires_clean_nozzle_confirmation);

        // Recovery-file generator over the fixture original: the heating
        // gate always holds, and the verbatim tail is byte-identical to
        // the original from the matched offset.
        let file = plr_recovery::build_recovery_file(
            &plan.recovery_file,
            MODEL_TEXT.as_bytes(),
            "TS",
        );
        prop_assert!(plr_recovery::verify_heating_gate(&file, &plan.recovery_file).is_ok());
        prop_assert_eq!(
            file.tail_bytes(),
            &MODEL_TEXT.as_bytes()[usize::try_from(plan.resume_offset).unwrap()..]
        );

        // Probe-phase Z bound: position_min <= Z <= shifted declare,
        // with the documented slack of 0.5e-5 mm — commands format
        // numbers at 5 decimal places, so a rendered Z may exceed the
        // exact envelope by at most half an ulp of that quantization.
        let slack = 1e-5;
        let lo = plan.envelope.position_min - slack;
        let hi = plan.envelope.shifted_declare_z + slack;
        for phase in [Phase::ShiftedFrame, Phase::ProbeApproach, Phase::Probe] {
            for step in plan.steps_in_phase(phase) {
                for command in &step.commands {
                    for z in z_values(command) {
                        prop_assert!(z >= lo && z <= hi, "Z {z} outside [{lo}, {hi}] in {command}");
                    }
                }
            }
        }

        // Ids strictly sequential from 1.
        for (index, step) in plan.steps.iter().enumerate() {
            prop_assert_eq!(step.id as usize, index + 1);
        }

        // Rendered plan is finite everywhere.
        assert_rendered_finite(&plan.render());

        // Envelope grows with the stop-set z-span (hop).
        prop_assert!(
            (plan.envelope.params.expected_gap - (s.hop + s.sag)).abs() < 1e-9,
            "gap must be span ({}) + sag ({})", s.hop, s.sag
        );

        // The probe method drives the probe step, its readback, and
        // the envelope's overshoot term.
        let probe = plan.steps_in_phase(Phase::Probe).next().expect("probe step");
        if s.drag {
            prop_assert!(probe.commands[0].starts_with("PLR_DRAG_PROBE CHIP=\"adxl345\" "));
            prop_assert!(probe.verify.iter().any(
                |v| v.object == "plr" && v.field == "last_drag_result.trigger_z"
            ));
            prop_assert_eq!(
                plan.envelope.params.overshoot,
                OvershootTerm::DragStep { drag_z_step: s.drag_z_step }
            );
        } else {
            // Tap consensus path: PLR_TOUCH, reading the consensus
            // median off the plr status object.
            prop_assert!(probe.commands[0].starts_with("PLR_TOUCH SAMPLES="));
            prop_assert!(probe.verify.iter().any(
                |v| v.object == "plr" && v.field == "last_touch_result.median_z"
            ));
            prop_assert_eq!(
                plan.envelope.params.overshoot,
                OvershootTerm::PostTriggerTravel { probe_speed: s.probe_speed }
            );
        }
        // The extruder-target temperature interlock is on every probe
        // step (method-independent).
        prop_assert!(probe.pre_verify.iter().any(
            |v| v.object == "extruder" && v.field == "target"
        ));
    }

    /// The whole-itinerary pre-flight passes every validly-generated
    /// plan, but a single corrupted absolute coordinate — the
    /// probe-approach XY displaced off the contact anchor, or an
    /// out-of-limit absolute travel move injected into a step — is
    /// always caught as `ItineraryOutOfBounds` naming that coordinate's
    /// axis. (The acceptance-criteria pre-flight proptest.)
    #[test]
    fn preflight_catches_a_single_corrupted_coordinate(
        s in scenario_strategy(),
        corrupt_axis in 0..2_u8,
        mode in 0..2_u8,
    ) {
        let plan = build_scenario(&s);
        // Generous, KNOWN limits so the axis checks are active; the
        // contact point is the analyzer's selected probe site (s.point,
        // which the approach travels to).
        let bounds = ItineraryBounds {
            x: Some((-10_000.0, 10_000.0)),
            y: Some((-10_000.0, 10_000.0)),
            z_max: Some(1.0e9),
            position_min: plan.envelope.position_min,
            contact_point: s.point,
        };
        // The clean, validly-generated plan passes.
        prop_assert!(preflight_itinerary(&plan, &bounds).is_ok());

        let axis = if corrupt_axis == 0 { 'X' } else { 'Y' };
        let bad = 99_999.0_f64;
        let [px, py] = s.point;
        let mut corrupted = plan.clone();
        let step_id;
        if mode == 0 {
            // Displace ONE probe-approach coordinate off the contact
            // anchor (and beyond the limit); the other stays correct.
            let approach = corrupted
                .steps
                .iter_mut()
                .find(|st| st.phase == Phase::ProbeApproach)
                .expect("approach step");
            step_id = approach.id;
            let (xw, yw) = if corrupt_axis == 0 { (bad, py) } else { (px, bad) };
            approach.commands =
                vec!["G90".to_owned(), format!("G0 X{} Y{} F6000", fmt_num(xw), fmt_num(yw))];
        } else {
            // Inject an out-of-limit ABSOLUTE travel move into the park
            // step (which runs in the absolute frame).
            let park = corrupted
                .steps
                .iter_mut()
                .find(|st| st.phase == Phase::ParkForReheat)
                .expect("park step");
            step_id = park.id;
            park.commands.insert(0, "G90".to_owned());
            park.commands
                .push(format!("G1 {axis}{} F1200", fmt_num(bad)));
        }

        let Err(RecoveryError::ItineraryRejected(PlanRejection::ItineraryOutOfBounds {
            violations,
        })) = preflight_itinerary(&corrupted, &bounds).map_err(RecoveryError::from)
        else {
            prop_assert!(false, "corrupted plan must be rejected");
            unreachable!();
        };
        // A violation names the corrupted axis at the corrupted step.
        prop_assert!(
            violations.iter().any(|v| v.axis == axis
                && v.step_id == step_id
                && matches!(
                    v.kind,
                    ViolationKind::AxisLimit | ViolationKind::ContactMismatch
                )),
            "violations {:?} do not name axis {} at step {}",
            violations,
            axis,
            step_id
        );
    }

    /// Hostile numbers anywhere produce a typed error or a typed
    /// fallback — never a panic, and never a plan containing a
    /// non-finite number.
    #[test]
    fn hostile_inputs_never_panic_and_never_leak(
        s in scenario_strategy(),
        site in 0..12_u8,
        bad in prop_oneof![Just(f64::NAN), Just(f64::INFINITY), Just(f64::NEG_INFINITY)],
    ) {
        let mut transforms = plain_transforms();
        if site == 0 { transforms.z_thermal_adjust_enabled = Some(true); transforms.z_thermal_adjust_offset = Some(bad); }
        let mut context = wal_context(transforms);
        match site {
            1 => context.gcode.speed_factor = bad,
            2 => context.gcode.extrude_factor = bad,
            3 => context.gcode.speed = bad,
            4 => context.gcode.homing_origin[2] = bad,
            5 => context.heaters[0].target = bad,
            6 => context.gcode.position[3] = bad,
            _ => {}
        }
        let z_candidates = if site == 7 { vec![bad] } else { vec![s.z_lo] };
        let reconstruction = recovery(stop_set(&z_candidates), context);
        let mut contact = contact_at(0.3);
        if let plr_analyzer::ContactOutcome::Candidates(c) = &mut contact {
            if site == 8 { c[0].z = bad; }
            if site == 9 { c[0].point[0] = bad; }
        }
        let excludes = if site == 10 {
            vec![ExcludeObjectDef {
                name: "obj".to_owned(),
                center: Some([bad, 0.0]),
                polygon: vec![],
                currently_excluded: false,
            }]
        } else {
            vec![]
        };
        let mut machine = machine_tap();
        if site == 11 { machine.z_position_min = Some(bad); }
        let match_result = MatchResult {
            candidates: vec![],
            confidence: MatchConfidence::UniqueLine {
                offset: offset_of("G1 X10 Y10 E1 F1800", 1),
            },
            skipped_unknown: 0,
        };
        let model = model();
        let inputs = PlanInputs {
            machine: &machine,
            reconstruction: &reconstruction,
            contact: &contact,
            match_result: &match_result,
            model: &model,
            file_temps: FileTemps::default(),
            exclude_objects: &excludes,
            clean_nozzle_macro_present: true,
            purge_macro_present: false,
        };
        // Must not panic; a produced plan must be finite everywhere.
        if let Ok(PlanOutcome::Plan(plan)) = plan_recovery(&inputs, &PlanConfig::default()) {
            assert_rendered_finite(&plan.render());
        }
    }

    /// Hostile plan-config numbers are typed errors, never panics.
    #[test]
    fn hostile_config_is_rejected(
        field in 0..7_u8,
        bad in prop_oneof![Just(f64::NAN), Just(f64::INFINITY), Just(f64::NEG_INFINITY)],
    ) {
        let mut config = PlanConfig::default();
        match field {
            0 => config.margin = bad,
            1 => config.sag_allowance = bad,
            2 => config.probe_speed = bad,
            3 => config.drag_speed = bad,
            4 => config.drag_z_step = bad,
            5 => config.drag_sensitivity = bad,
            _ => config.idle_timeout_s = bad,
        }
        let reconstruction = recovery(stop_set(&[0.4]), wal_context(plain_transforms()));
        let contact = contact_at(0.3);
        let match_result = MatchResult {
            candidates: vec![],
            confidence: MatchConfidence::UniqueLine {
                offset: offset_of("G1 X10 Y10 E1 F1800", 1),
            },
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
        prop_assert!(plan_recovery(&inputs, &config).is_err());
    }

    /// The guard scan is total on arbitrary text, and sanitizing always
    /// yields a clean macro.
    #[test]
    fn guard_scan_total_and_sanitize_clean(text in "[ -~\\n\\t;#{}%]{0,400}") {
        let scan = scan_macro_text(&text);
        for hit in &scan.hits {
            prop_assert!(plr_recovery::GUARDED_COMMANDS.contains(&hit.command.as_str()));
        }
        match sanitize_macro_text(&text) {
            GuardOutcome::Clean { text: out } => {
                prop_assert_eq!(out, text.clone());
                prop_assert!(scan.is_clean());
            }
            GuardOutcome::Stripped { text: out, removed } => {
                prop_assert!(!removed.is_empty());
                prop_assert!(scan_macro_text(&out).is_clean());
                prop_assert_eq!(out.lines().count(), text.lines().count());
            }
        }
    }

    /// The guard scan never panics on fully arbitrary (non-ASCII)
    /// strings either.
    #[test]
    fn guard_scan_total_on_arbitrary_unicode(text in any::<String>()) {
        let _ = scan_macro_text(&text);
        let _ = sanitize_macro_text(&text);
    }

    /// True-Z evaluation is total: Ok implies finite; hostile inputs
    /// are typed errors.
    #[test]
    fn true_z_total(
        prev in any::<f64>(),
        trigger in any::<f64>(),
        halt in any::<f64>(),
        offset in any::<f64>(),
        load_cell in any::<bool>(),
    ) {
        let formula = TrueZFormula {
            z_prev_top: prev,
            trigger_source: if load_cell {
                TriggerSource::BedZPlusOffset { z_offset: offset }
            } else {
                TriggerSource::RawLastZResult
            },
            frozen_z_adjust: None,
        };
        match true_z_at_halt(&formula, trigger, halt) {
            Ok(z) => prop_assert!(z.is_finite()),
            Err(RecoveryError::NonFinite { .. }) => {}
            Err(other) => prop_assert!(false, "unexpected error {other:?}"),
        }
    }

    /// Finding 2 (byte fidelity): for ARBITRARY original bytes —
    /// including non-UTF-8 sequences a lossy decode would rewrite as
    /// `EF BF BD` — the generated tail is byte-identical to
    /// `original[offset..]`, and the tail length matches exactly.
    ///
    /// Varying the BYTES (not just the offset over a fixed ASCII fixture)
    /// is what makes this able to catch a lossy copy at all.
    #[test]
    fn recovery_file_tail_is_byte_verbatim_for_arbitrary_bytes(
        original in proptest::collection::vec(any::<u8>(), 0..600),
        offset_frac in 0.0..1.0_f64,
        bed in proptest::option::of(40.0..110.0_f64),
        nozzle in 170.0..300.0_f64,
        purge_on in any::<bool>(),
        purge_amount in 0.0..60.0_f64,
        purge_retract in 0.0..10.0_f64,
    ) {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let offset = ((original.len() as f64) * offset_frac) as usize;
        let spec = plr_recovery::RecoveryFileSpec {
            name: "p_RECOVERY.gcode".to_owned(),
            source_name: "p.gcode".to_owned(),
            plan_id: "plr-1".to_owned(),
            tail_offset: offset as u64,
            bed,
            nozzle,
            purge: purge_on.then_some(plr_recovery::PurgePlan::BuiltIn {
                point: [180.0, 20.0],
                z: None,
                amount: purge_amount,
                speed: 300.0,
                // NONZERO retracts are the shape the absolute-E blocker
                // got wrong; the generator must handle them for any
                // amount/retract pair.
                retract: purge_retract,
                travel_feed: 6000.0,
            }),
            park: [180.0, 20.0],
            park_feed: 6000.0,
            descend_feed: 1200.0,
            entry_commands: vec!["G90".to_owned(), "G0 X30 Y30 F1200".to_owned()],
            header_cap: 200,
        };
        let file = plr_recovery::build_recovery_file(&spec, &original, "TS");
        // Byte-exact tail, exact length: no transcoding anywhere.
        prop_assert_eq!(file.tail_bytes(), &original[offset..]);
        prop_assert_eq!(file.tail_bytes().len(), original.len() - offset);
        // The whole file is preamble ++ tail, with nothing lost between.
        prop_assert_eq!(file.content.len(), file.tail_start + original.len() - offset);
        // The heating gate holds regardless of what the original carried.
        prop_assert!(plr_recovery::verify_heating_gate(&file, &spec).is_ok());
    }

    /// Finding 9 (cross-component interlock): for ANY valid `[plr]`
    /// temperature configuration, the COMMANDED probe target stays at
    /// least `PROBE_TEMP_HEADROOM` below the contact ceiling the plugin
    /// refuses at — so PID overshoot can never wedge the recovery.
    #[test]
    fn commanded_probe_temp_always_leaves_headroom(
        probe_temp_min in 140.0..=145.0_f64,
        probe_temp_max in 145.001..=160.0_f64,
        max_probe_nozzle_temp in 80.0..=160.0_f64,
        asked in 100.0..=200.0_f64,
    ) {
        let config = PlanConfig {
            probe_temp_min,
            probe_temp_max,
            max_probe_nozzle_temp,
            probe_nozzle_temp: asked,
            ..PlanConfig::default()
        };
        // Only configs the validator ACCEPTS make a claim; the rest are
        // refused up front (which is the other half of the fix).
        if config.validate().is_err() {
            return Ok(());
        }
        let ceiling = config.clamped_probe_max();
        let commanded = config.commanded_probe_temp();
        prop_assert!(
            commanded + plr_recovery::PROBE_TEMP_HEADROOM <= ceiling + 1e-9,
            "commanded {} must be >= {} C below the ceiling {}",
            commanded, plr_recovery::PROBE_TEMP_HEADROOM, ceiling
        );
        // ...and still inside the verification band's lower bound.
        prop_assert!(commanded >= config.probe_temp_min - 1e-9);
    }

    /// `fmt_num` is total and value-faithful for finite inputs.
    #[test]
    fn fmt_num_total_and_faithful(v in any::<f64>()) {
        let s = fmt_num(v);
        if v.is_finite() {
            let parsed: f64 = s.parse().unwrap();
            let tolerance = 1e-5 + v.abs() * 1e-12;
            prop_assert!((parsed - v).abs() <= tolerance, "{v} -> {s} -> {parsed}");
        } else {
            prop_assert_eq!(s, "invalid");
        }
    }

    /// The file temperature scan is total on arbitrary bytes.
    #[test]
    fn temp_scan_total(bytes in proptest::collection::vec(any::<u8>(), 0..600), stop in any::<u64>()) {
        let temps = plr_recovery::scan_file_temps(&bytes, 0, stop);
        prop_assert!(temps.nozzle.is_none_or(f64::is_finite));
        prop_assert!(temps.bed.is_none_or(f64::is_finite));
    }
}
