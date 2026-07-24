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
    compute_envelope, fmt_num, plan_recovery, sanitize_macro_text, scan_macro_text, true_z_at_halt,
    EnvelopeParams, ExcludeObjectDef, FileTemps, GuardOutcome, OvershootTerm, Phase, PlanConfig,
    PlanInputs, PlanOutcome, RecoveryError, RecoveryPlan, TriggerSource, TrueZFormula,
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

        // z_thermal step present exactly when the module is configured.
        prop_assert_eq!(
            plan.first_index(Phase::TransformFreeze).is_some(),
            s.z_thermal.is_some()
        );
        // Mesh step present exactly for a restorable (named) mesh.
        prop_assert_eq!(plan.first_index(Phase::MeshLoad).is_some(), s.mesh == 1);

        // M26 offset: present, equals the plan's resume offset, and is
        // a line boundary of the file.
        let m26 = plan.m26_offset();
        prop_assert_eq!(m26, Some(plan.resume_offset));
        prop_assert!(line_starts().contains(&plan.resume_offset));

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
            prop_assert!(probe.commands[0].starts_with("PLR_DRAG_PROBE CHIP=adxl345 "));
            prop_assert!(probe.verify.iter().any(
                |v| v.object == "plr" && v.field == "last_drag_result.trigger_z"
            ));
            prop_assert_eq!(
                plan.envelope.params.overshoot,
                OvershootTerm::DragStep { drag_z_step: s.drag_z_step }
            );
        } else {
            prop_assert!(probe.commands[0].starts_with("PROBE PROBE_SPEED="));
            prop_assert!(probe.verify.iter().any(
                |v| v.object == "probe" && v.field == "last_z_result"
            ));
            prop_assert_eq!(
                plan.envelope.params.overshoot,
                OvershootTerm::PostTriggerTravel { probe_speed: s.probe_speed }
            );
        }
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
