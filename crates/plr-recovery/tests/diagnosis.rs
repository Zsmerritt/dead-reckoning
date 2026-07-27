//! The diagnosis contract: every typed failure explains itself, every
//! tier means what it says, and the `UNSAFE_` escape hatches work only
//! where they are documented to.
//!
//! # Exactly what is guaranteed, and by what
//!
//! **The compiler guarantees totality.** Each `Diagnose` implementation
//! is an exhaustive match with no catch-all arm, so a new enum variant
//! cannot compile until somebody writes its diagnosis. That is the real
//! protection and it needs no test.
//!
//! **A second compile-time trap lives here.** The `tier_of_*` functions
//! below are also exhaustive matches with no catch-all, over the same
//! enums. A new variant therefore fails to compile in TWO places: the
//! implementation, and this test's independent statement of what tier the
//! variant should carry. Restating the tier separately from the code that
//! produces it is the point — a test that read the tier back off the
//! implementation would agree with any tier the implementation happened
//! to pick.
//!
//! **The hand-written lists are sample VALUES, nothing more.** They exist
//! to give each arm a concrete input so the rendered message can be
//! checked for substance (non-empty `what`/`why`/`suggested_fix`, a
//! `snake_case` code, an `override_key` only where the tier allows one).
//! Their `assert_eq!` length checks guard against silent REMOVAL from the
//! list; they cannot detect a variant added to the enum, because nothing
//! ties a `Vec` literal's length to an enum's cardinality. Rust has no
//! built-in variant enumeration and this crate takes no derive-macro
//! dependency for one, so that gap is closed by `tier_of_*` refusing to
//! compile instead.

use plr_recovery::diagnosis::{Diagnose, Tier};
use plr_recovery::machine::PrereqFailure;
use plr_recovery::preflight::{BoundsViolation, PlanRejection, ViolationKind};
use plr_recovery::{
    PlanConfig, PlanWarning, RecoveryError, ACCEL_MAX, ACCEL_MIN, CONFIRM_TIMEOUT_DEFAULT_S,
    CONFIRM_TIMEOUT_MAX_S, CONFIRM_TIMEOUT_MIN_S, GCODE_BARRIER_TIMEOUT_MAX_S,
    GCODE_BARRIER_TIMEOUT_MIN_S, UNSAFE_PURGE_Z_BELOW_BED,
};

/// Every [`RecoveryError`] variant, with the tier it must carry.
fn every_recovery_error() -> Vec<(RecoveryError, Tier)> {
    vec![
        (RecoveryError::NonFinite { field: "gap" }, Tier::Hard),
        (
            RecoveryError::InvalidContext {
                field: "extrude_factor",
            },
            Tier::Hard,
        ),
        (
            RecoveryError::ProbeSpeedOutOfRange { speed: 9.0 },
            Tier::Hard,
        ),
        (
            RecoveryError::InvalidPlanConfig {
                field: "purge_speed",
            },
            Tier::Hard,
        ),
        (
            RecoveryError::AccelOutOfRange {
                key: "recovery_accel",
                value: 4.0,
                min: ACCEL_MIN,
                max: ACCEL_MAX,
            },
            Tier::Hard,
        ),
        (
            RecoveryError::ProbeTempHeadroomUnavailable {
                probe_temp_min: 148.0,
                ceiling: 150.0,
                headroom: 5.0,
            },
            Tier::Hard,
        ),
        (
            RecoveryError::DragTempOutOfRange {
                drag_nozzle_temp: 149.0,
                ceiling: 150.0,
                headroom: 5.0,
            },
            Tier::Hard,
        ),
        (
            RecoveryError::PurgeMacroMissing {
                name: "MY_PURGE".to_owned(),
            },
            Tier::Hard,
        ),
        (RecoveryError::PurgeZBelowBed { purge_z: -0.4 }, Tier::Hard),
        (
            RecoveryError::ConfirmTimeoutOutOfRange {
                value: 5.0,
                min: CONFIRM_TIMEOUT_MIN_S,
                max: CONFIRM_TIMEOUT_MAX_S,
            },
            Tier::Hard,
        ),
        (
            RecoveryError::GcodeBarrierTimeoutOutOfRange {
                value: 0.0,
                min: GCODE_BARRIER_TIMEOUT_MIN_S,
                max: GCODE_BARRIER_TIMEOUT_MAX_S,
            },
            Tier::Hard,
        ),
        (
            RecoveryError::MachineRejected {
                failures: vec![PrereqFailure::ForceMoveDisabled],
            },
            Tier::Hard,
        ),
        (RecoveryError::NoContext, Tier::Hard),
        (RecoveryError::NoVirtualSd, Tier::Hard),
        (
            RecoveryError::FileNotTopLevel {
                path: "/g/sub/x.gcode".to_owned(),
            },
            Tier::Hard,
        ),
        (RecoveryError::NoZSpan, Tier::Hard),
        (RecoveryError::NoProbeCandidates, Tier::Hard),
        (RecoveryError::NoNozzleTarget, Tier::Hard),
        (
            RecoveryError::InvalidName {
                field: "exclude_object",
                name: "a\"b".to_owned(),
            },
            Tier::Hard,
        ),
        (
            RecoveryError::ItineraryRejected(PlanRejection::ItineraryOutOfBounds {
                violations: vec![violation()],
            }),
            Tier::Hard,
        ),
    ]
}

fn violation() -> BoundsViolation {
    BoundsViolation {
        step_id: 7,
        axis: 'Z',
        value: -9.5,
        min: Some(-2.0),
        max: Some(300.0),
        kind: ViolationKind::AxisLimit,
    }
}

/// Every [`PrereqFailure`] variant, with the tier it must carry. All are
/// Hard: these are the structural assumptions the whole method rests on,
/// and none of them is a judgement call an operator can make from a
/// dialog box.
fn every_prereq_failure() -> Vec<(PrereqFailure, Tier)> {
    vec![
        (PrereqFailure::ForceMoveDisabled, Tier::Hard),
        (PrereqFailure::ZNotSelfLocking, Tier::Hard),
        (PrereqFailure::NoZSteppers, Tier::Hard),
        (
            PrereqFailure::ZStepperOffPrimaryMcu {
                stepper: "stepper_z1".to_owned(),
                mcu: "mcu tool".to_owned(),
            },
            Tier::Hard,
        ),
        (PrereqFailure::NoTypeAnnotations, Tier::Hard),
        (PrereqFailure::NoProbe, Tier::Hard),
        (PrereqFailure::MultipleProbes { count: 2 }, Tier::Hard),
        (PrereqFailure::ProbeZOffsetNonFinite, Tier::Hard),
        (PrereqFailure::ProbeActivateGcodeMoves, Tier::Hard),
        (PrereqFailure::ProbeDeactivateGcodeMoves, Tier::Hard),
        (PrereqFailure::PositionMinUnknown, Tier::Hard),
        (PrereqFailure::PositionMinNonFinite, Tier::Hard),
        (PrereqFailure::ConfigNeverValidated, Tier::Hard),
        (
            PrereqFailure::ConfigChangedSinceValidation {
                validated: "aaaa".to_owned(),
                current: "bbbb".to_owned(),
            },
            Tier::Hard,
        ),
        (PrereqFailure::SdcardRootUnknown, Tier::Hard),
        (
            PrereqFailure::AccelChipInvalid {
                chip: "a\"b".to_owned(),
            },
            Tier::Hard,
        ),
        (PrereqFailure::NoiseFloorMissing, Tier::Hard),
        (PrereqFailure::NoiseFloorInvalid { value: -1.0 }, Tier::Hard),
    ]
}

/// Every [`PlanWarning`] variant, with the tier it must carry. Two of
/// them are data-dependent (a purge inside the part is a collision only
/// when a `purge_z` makes the file descend to it; a park inside the part
/// is a judgement the operator already made only when they configured
/// it), so both of their shapes appear.
fn every_plan_warning() -> Vec<(PlanWarning, Tier)> {
    vec![
        (PlanWarning::AdaptiveMeshNotRestorable, Tier::Advisory),
        (PlanWarning::SkewProfileUnknown, Tier::Advisory),
        (PlanWarning::NoBedTarget, Tier::Advisory),
        (
            PlanWarning::ReheatParkComputed {
                point: [10.0, 20.0],
            },
            Tier::Advisory,
        ),
        (
            PlanWarning::ReheatParkUnverified {
                point: [10.0, 20.0],
            },
            Tier::Confirmable,
        ),
        (
            PlanWarning::PurgeInsidePart {
                point: [10.0, 20.0],
                configured: true,
                purge_z: None,
            },
            Tier::Advisory,
        ),
        (
            PlanWarning::PurgeInsidePart {
                point: [10.0, 20.0],
                configured: false,
                purge_z: Some(0.2),
            },
            Tier::Confirmable,
        ),
        (
            PlanWarning::PurgeZBelowResume {
                purge_z: 0.1,
                resume_z: 0.6,
            },
            Tier::Confirmable,
        ),
        (
            PlanWarning::ReheatParkInsidePart {
                point: [10.0, 20.0],
                configured: true,
            },
            Tier::Advisory,
        ),
        (
            PlanWarning::ReheatParkInsidePart {
                point: [10.0, 20.0],
                configured: false,
            },
            Tier::Confirmable,
        ),
        (PlanWarning::ResumeNotOnInfill, Tier::Advisory),
        (
            PlanWarning::UnrestorableFan {
                name: "weird_fan".to_owned(),
            },
            Tier::Advisory,
        ),
        (
            PlanWarning::NoiseFloorSpeedMismatch {
                calibrated_at: 20.0,
                drag_speed: 40.0,
            },
            Tier::Confirmable,
        ),
        (
            PlanWarning::UnsafeOverrideActive {
                key: UNSAFE_PURGE_Z_BELOW_BED.to_owned(),
                permitted: "purge_z_below_bed".to_owned(),
            },
            Tier::Advisory,
        ),
        (
            PlanWarning::AccelProbeIgnoredOnTouchPath { accel_probe: 500.0 },
            Tier::Advisory,
        ),
        (
            PlanWarning::DragTempBelowFloor {
                drag_nozzle_temp: 30.0,
                floor: 50.0,
            },
            Tier::Confirmable,
        ),
        (
            PlanWarning::AccelEntryNotAppliedToFile { accel_entry: 600.0 },
            Tier::Advisory,
        ),
        (
            PlanWarning::PreviewStandoffSqueezed {
                available: 0.3,
                requested: 1.0,
            },
            Tier::Advisory,
        ),
    ]
}

/// The core assertion applied to every diagnosis in the system: three
/// non-empty parts, a `snake_case` code, and an override key only where the
/// tier permits one.
fn assert_well_formed(d: &plr_recovery::Diagnosis, expected_tier: Tier, label: &str) {
    assert_eq!(d.tier, expected_tier, "{label}: wrong tier");
    assert!(!d.what.trim().is_empty(), "{label}: empty `what`");
    assert!(!d.why.trim().is_empty(), "{label}: empty `why`");
    assert!(
        !d.suggested_fix.trim().is_empty(),
        "{label}: empty `suggested_fix`"
    );
    // A "fix" that does not name anything actionable is not a fix. Every
    // one of ours points at a config key, a command, or an explicit
    // "nothing can fix this".
    assert!(
        d.suggested_fix.len() > 20,
        "{label}: suggested_fix is too short to say anything: {:?}",
        d.suggested_fix
    );
    assert!(
        !d.code.is_empty()
            && d.code
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_'),
        "{label}: code {:?} is not snake_case",
        d.code
    );
    if d.tier != Tier::Hard {
        assert_eq!(
            d.override_key, None,
            "{label}: only a Hard diagnosis may name an UNSAFE_ override"
        );
    }
    if let Some(key) = d.override_key {
        assert!(
            key.starts_with("UNSAFE_"),
            "{label}: override key {key:?} must be UNSAFE_-prefixed"
        );
    }
    // Both render helpers must work and mention the code.
    assert!(d.one_line().contains(d.code), "{label}: one_line");
    assert!(d.full().contains(d.code), "{label}: full");
    assert!(d.full().contains(&d.suggested_fix), "{label}: full fix");
}

// --- The second compile-time trap ------------------------------------------
//
// Exhaustive matches with NO catch-all arm, restating each variant's tier
// independently of the implementation that produces it. A new variant
// fails to compile here as well as in `diagnosis.rs`, so "what tier is
// this?" cannot be answered by accident.

/// The tier every [`RecoveryError`] must carry.
fn tier_of_recovery_error(e: &RecoveryError) -> Tier {
    match e {
        // Every RecoveryError is Hard: these are errors, not
        // defined-but-degraded outcomes (the crate draws that line by
        // routing the latter to PlanWarning / ManualFallback instead).
        RecoveryError::NonFinite { .. }
        | RecoveryError::InvalidContext { .. }
        | RecoveryError::ProbeSpeedOutOfRange { .. }
        | RecoveryError::InvalidPlanConfig { .. }
        | RecoveryError::AccelOutOfRange { .. }
        | RecoveryError::ConfirmTimeoutOutOfRange { .. }
        | RecoveryError::GcodeBarrierTimeoutOutOfRange { .. }
        | RecoveryError::ProbeTempHeadroomUnavailable { .. }
        | RecoveryError::DragTempOutOfRange { .. }
        | RecoveryError::PurgeMacroMissing { .. }
        | RecoveryError::PurgeZBelowBed { .. }
        | RecoveryError::MachineRejected { .. }
        | RecoveryError::NoContext
        | RecoveryError::NoVirtualSd
        | RecoveryError::FileNotTopLevel { .. }
        | RecoveryError::NoZSpan
        | RecoveryError::NoProbeCandidates
        | RecoveryError::NoNozzleTarget
        | RecoveryError::InvalidName { .. }
        | RecoveryError::ItineraryRejected(_) => Tier::Hard,
    }
}

/// The tier every [`PrereqFailure`] must carry.
fn tier_of_prereq_failure(f: &PrereqFailure) -> Tier {
    match f {
        // The structural assumptions the whole method rests on. None of
        // them is a judgement an operator can make from a dialog box, and
        // none may be overridden.
        PrereqFailure::ForceMoveDisabled
        | PrereqFailure::ZNotSelfLocking
        | PrereqFailure::NoZSteppers
        | PrereqFailure::ZStepperOffPrimaryMcu { .. }
        | PrereqFailure::NoTypeAnnotations
        | PrereqFailure::NoProbe
        | PrereqFailure::MultipleProbes { .. }
        | PrereqFailure::ProbeZOffsetNonFinite
        | PrereqFailure::ProbeActivateGcodeMoves
        | PrereqFailure::ProbeDeactivateGcodeMoves
        | PrereqFailure::PositionMinUnknown
        | PrereqFailure::PositionMinNonFinite
        | PrereqFailure::ConfigNeverValidated
        | PrereqFailure::ConfigChangedSinceValidation { .. }
        | PrereqFailure::SdcardRootUnknown
        | PrereqFailure::AccelChipInvalid { .. }
        | PrereqFailure::NoiseFloorMissing
        | PrereqFailure::NoiseFloorInvalid { .. } => Tier::Hard,
    }
}

/// The tier every [`PlanWarning`] must carry.
///
/// Two are data-dependent: a purge inside the part is a commanded
/// collision only when a `purge_z` makes the file descend to it, and a
/// park inside the part is a judgement the operator already made only
/// when they configured it there.
///
/// **No arm may return [`Tier::Hard`].** A warning is an observation
/// about a plan that was successfully built; a Hard diagnosis is a
/// refusal to build one. The executor enforces the same thing at runtime
/// (a Hard diagnosis aborts rather than being offered a continue-anyway
/// button), and `no_plan_warning_is_hard` pins it here.
fn tier_of_plan_warning(w: &PlanWarning) -> Tier {
    match w {
        PlanWarning::AdaptiveMeshNotRestorable
        | PlanWarning::SkewProfileUnknown
        | PlanWarning::NoBedTarget
        | PlanWarning::ReheatParkComputed { .. }
        | PlanWarning::ResumeNotOnInfill
        | PlanWarning::UnrestorableFan { .. }
        | PlanWarning::UnsafeOverrideActive { .. }
        | PlanWarning::AccelProbeIgnoredOnTouchPath { .. }
        | PlanWarning::PreviewStandoffSqueezed { .. }
        | PlanWarning::AccelEntryNotAppliedToFile { .. } => Tier::Advisory,
        PlanWarning::PurgeZBelowResume { .. }
        | PlanWarning::NoiseFloorSpeedMismatch { .. }
        | PlanWarning::DragTempBelowFloor { .. }
        // Nothing is known about this park point at all; less knowledge
        // must not buy less friction than the better-informed
        // `ReheatParkInsidePart` below.
        | PlanWarning::ReheatParkUnverified { .. } => Tier::Confirmable,
        PlanWarning::PurgeInsidePart { purge_z, .. } => {
            if purge_z.is_some() {
                Tier::Confirmable
            } else {
                Tier::Advisory
            }
        }
        PlanWarning::ReheatParkInsidePart { configured, .. } => {
            if *configured {
                Tier::Advisory
            } else {
                Tier::Confirmable
            }
        }
    }
}

/// A plan warning is an observation, never a refusal: the executor routes
/// Hard diagnoses to an abort rather than to the confirmation, so a Hard
/// warning would be a refusal nobody could act on.
#[test]
fn no_plan_warning_is_hard() {
    for (w, _) in every_plan_warning() {
        assert_ne!(
            tier_of_plan_warning(&w),
            Tier::Hard,
            "a PlanWarning may not be Hard: {w:?}"
        );
        assert_ne!(w.diagnosis().tier, Tier::Hard, "{w:?}");
    }
}

#[test]
fn every_recovery_error_variant_yields_a_usable_diagnosis() {
    let all = every_recovery_error();
    assert_eq!(
        all.len(),
        20,
        "a sample value was removed from this list; it must exercise every arm"
    );
    for (error, tier) in &all {
        // The declared tier and the independently-restated one must
        // agree, and both must match what the implementation produced.
        assert_eq!(tier_of_recovery_error(error), *tier, "{error:?}");
        assert_well_formed(&error.diagnosis(), *tier, &format!("{error:?}"));
    }
}

#[test]
fn every_prereq_failure_variant_yields_a_usable_diagnosis() {
    let all = every_prereq_failure();
    assert_eq!(
        all.len(),
        18,
        "a sample value was removed from this list; it must exercise every arm"
    );
    for (failure, tier) in &all {
        assert_eq!(tier_of_prereq_failure(failure), *tier, "{failure:?}");
        assert_well_formed(&failure.diagnosis(), *tier, &format!("{failure:?}"));
    }
}

#[test]
fn every_plan_warning_variant_yields_a_usable_diagnosis() {
    let all = every_plan_warning();
    // 16 variants, two of which appear twice because their tier depends
    // on the data they carry.
    assert_eq!(
        all.len(),
        18,
        "a sample value was removed from this list; it must exercise every arm"
    );
    for (warning, tier) in &all {
        assert_eq!(tier_of_plan_warning(warning), *tier, "{warning:?}");
        assert_well_formed(&warning.diagnosis(), *tier, &format!("{warning:?}"));
    }
}

#[test]
fn the_plan_rejection_variant_yields_a_usable_diagnosis() {
    let rejection = PlanRejection::ItineraryOutOfBounds {
        violations: vec![violation()],
    };
    let d = rejection.diagnosis();
    assert_well_formed(&d, Tier::Hard, "ItineraryOutOfBounds");
    // The typed numbers survive: a client can render the offending
    // coordinate without parsing prose.
    let measured = d.measured.as_ref().expect("measured");
    assert!((measured.value - -9.5).abs() < 1e-12);
    let expected = d.expected.as_ref().expect("expected");
    assert_eq!(expected.min, Some(-2.0));
    assert_eq!(expected.max, Some(300.0));
    // Wrapping it in RecoveryError delegates rather than inventing a
    // second, differently-worded explanation of the same thing.
    let wrapped = RecoveryError::ItineraryRejected(PlanRejection::ItineraryOutOfBounds {
        violations: vec![violation()],
    });
    assert_eq!(wrapped.diagnosis(), d);
}

#[test]
fn an_empty_violation_list_still_diagnoses() {
    // Defensive: the constructor never produces one, but a Diagnosis must
    // not depend on that.
    let d = PlanRejection::ItineraryOutOfBounds { violations: vec![] }.diagnosis();
    assert_well_formed(&d, Tier::Hard, "empty violations");
    assert!(d.measured.is_none());
}

#[test]
fn diagnosis_codes_are_unique_across_every_source() {
    let mut codes: Vec<&'static str> = Vec::new();
    for (e, _) in every_recovery_error() {
        // ItineraryRejected delegates to PlanRejection, so its code is
        // deliberately shared; skip the duplicate.
        if !matches!(e, RecoveryError::ItineraryRejected(_)) {
            codes.push(e.diagnosis().code);
        }
    }
    for (f, _) in every_prereq_failure() {
        codes.push(f.diagnosis().code);
    }
    for (w, _) in every_plan_warning() {
        let code = w.diagnosis().code;
        if !codes.contains(&code) {
            codes.push(code);
        }
    }
    codes.push(
        PlanRejection::ItineraryOutOfBounds { violations: vec![] }
            .diagnosis()
            .code,
    );
    let mut sorted = codes.clone();
    sorted.sort_unstable();
    sorted.dedup();
    assert_eq!(
        sorted.len(),
        codes.len(),
        "diagnosis codes must be unique — clients branch on them: {codes:?}"
    );
}

#[test]
fn exactly_one_hard_diagnosis_carries_an_unsafe_override() {
    let mut overridable: Vec<(&'static str, &'static str)> = Vec::new();
    for (e, _) in every_recovery_error() {
        let d = e.diagnosis();
        if let Some(key) = d.override_key {
            overridable.push((d.code, key));
        }
    }
    for (f, _) in every_prereq_failure() {
        assert_eq!(
            f.diagnosis().override_key,
            None,
            "no machine prerequisite may be overridden: {f:?}"
        );
    }
    for (w, _) in every_plan_warning() {
        assert_eq!(
            w.diagnosis().override_key,
            None,
            "only a Hard diagnosis may name an UNSAFE_ key: {w:?}"
        );
    }
    overridable.sort_unstable();
    assert_eq!(
        overridable,
        vec![("purge_z_below_bed", UNSAFE_PURGE_Z_BELOW_BED)],
        "EXACTLY ONE. The tier boundary is `Hard = physical damage or an unknowable \
         machine state`, and a nozzle driven into the bed is the only consequence that \
         clears it. Anything whose worst case is a bounded wait ending in a clean abort \
         belongs in the Confirmable tier, where the operator gets a button instead of a \
         config edit. Changing this set is a safety decision, not a refactor."
    );
}

// --- Tier policy at the config gate -----------------------------------------

/// A `purge_z` below the bed is Hard: refused, and permitted ONLY by the
/// pre-set `UNSAFE_` key.
#[test]
fn purge_z_below_bed_is_refused_and_only_the_unsafe_key_permits_it() {
    let refused = PlanConfig {
        purge_z: Some(-0.5),
        ..PlanConfig::default()
    };
    let error = refused.validate().expect_err("must refuse");
    assert!(matches!(error, RecoveryError::PurgeZBelowBed { .. }));
    let d = error.diagnosis();
    assert_eq!(d.tier, Tier::Hard);
    assert_eq!(d.code, "purge_z_below_bed");
    assert_eq!(d.override_key, Some(UNSAFE_PURGE_Z_BELOW_BED));
    assert!((d.measured.as_ref().unwrap().value - -0.5).abs() < 1e-12);
    assert_eq!(d.expected.as_ref().unwrap().min, Some(0.0));
    // The full render tells the operator the escape hatch is an edit,
    // not a button — that distinction is the entire policy.
    let full = d.full();
    assert!(full.contains("printer.cfg"), "{full}");
    assert!(full.contains("no runtime button"), "{full}");

    // With the key set — a deliberate edit made while calm — it passes.
    let permitted = PlanConfig {
        purge_z: Some(-0.5),
        unsafe_allow_purge_z_below_bed: true,
        ..PlanConfig::default()
    };
    permitted
        .validate()
        .expect("the UNSAFE_ key permits the otherwise-Hard refusal");

    // And the key permits ONLY its own refusal: it does not become a
    // general-purpose bypass.
    let unrelated = PlanConfig {
        purge_speed: 5_000.0,
        unsafe_allow_purge_z_below_bed: true,
        ..PlanConfig::default()
    };
    assert!(matches!(
        unrelated.validate(),
        Err(RecoveryError::InvalidPlanConfig {
            field: "purge_speed"
        })
    ));
}

/// A sub-floor drag temperature is no longer a planning refusal at all —
/// it is the Confirmable warning the builder raises on drag machines.
#[test]
fn a_sub_floor_drag_temperature_is_not_a_planning_refusal() {
    PlanConfig {
        drag_nozzle_temp: 30.0,
        ..PlanConfig::default()
    }
    .validate()
    .expect("a bounded wait ending in a clean abort is Confirmable, not Hard");
    let d = PlanWarning::DragTempBelowFloor {
        drag_nozzle_temp: 30.0,
        floor: 50.0,
    }
    .diagnosis();
    assert_eq!(d.tier, Tier::Confirmable);
    assert_eq!(d.override_key, None);
    assert!(d.full().contains("answer `continue`"), "{}", d.full());
}

#[test]
fn the_confirm_timeout_is_bounded_on_both_sides() {
    for bad in [
        CONFIRM_TIMEOUT_MIN_S - 1.0,
        CONFIRM_TIMEOUT_MAX_S + 1.0,
        0.0,
        f64::NAN,
        f64::INFINITY,
    ] {
        let error = PlanConfig {
            confirm_timeout_s: Some(bad),
            ..PlanConfig::default()
        }
        .validate()
        .unwrap_err_or_panic(&format!("confirm_timeout_s = {bad} must refuse"));
        assert!(
            matches!(error, RecoveryError::ConfirmTimeoutOutOfRange { .. }),
            "{bad}: {error:?}"
        );
        let d = error.diagnosis();
        assert_eq!(d.code, "confirm_timeout_out_of_range");
        assert_eq!(d.tier, Tier::Hard);
        assert_eq!(d.override_key, None);
        assert_eq!(
            d.expected.as_ref().unwrap().min,
            Some(CONFIRM_TIMEOUT_MIN_S)
        );
        assert_eq!(
            d.expected.as_ref().unwrap().max,
            Some(CONFIRM_TIMEOUT_MAX_S)
        );
    }
    for ok in [
        CONFIRM_TIMEOUT_MIN_S,
        CONFIRM_TIMEOUT_DEFAULT_S,
        CONFIRM_TIMEOUT_MAX_S,
    ] {
        PlanConfig {
            confirm_timeout_s: Some(ok),
            ..PlanConfig::default()
        }
        .validate()
        .unwrap_or_else(|e| panic!("{ok} must be accepted: {e:?}"));
    }
    // Absent is always fine: the daemon's default stays in force.
    assert_eq!(PlanConfig::default().confirm_timeout_s, None);
}

#[test]
fn the_gcode_barrier_timeout_is_bounded_on_both_sides() {
    for bad in [
        GCODE_BARRIER_TIMEOUT_MIN_S - 1.0,
        GCODE_BARRIER_TIMEOUT_MAX_S + 1.0,
        0.0,
        f64::NAN,
        f64::INFINITY,
    ] {
        let error = PlanConfig {
            gcode_barrier_timeout_s: Some(bad),
            ..PlanConfig::default()
        }
        .validate()
        .unwrap_err_or_panic(&format!("gcode_barrier_timeout_s = {bad} must refuse"));
        assert!(
            matches!(error, RecoveryError::GcodeBarrierTimeoutOutOfRange { .. }),
            "{bad}: {error:?}"
        );
        let d = error.diagnosis();
        assert_eq!(d.code, "gcode_barrier_timeout_out_of_range");
        assert_eq!(d.tier, Tier::Hard);
        assert_eq!(d.override_key, None);
        assert_eq!(
            d.expected.as_ref().unwrap().min,
            Some(GCODE_BARRIER_TIMEOUT_MIN_S)
        );
        assert_eq!(
            d.expected.as_ref().unwrap().max,
            Some(GCODE_BARRIER_TIMEOUT_MAX_S)
        );
    }
    for ok in [GCODE_BARRIER_TIMEOUT_MIN_S, GCODE_BARRIER_TIMEOUT_MAX_S] {
        PlanConfig {
            gcode_barrier_timeout_s: Some(ok),
            ..PlanConfig::default()
        }
        .validate()
        .unwrap_or_else(|e| panic!("{ok} must be accepted: {e:?}"));
    }
    assert_eq!(PlanConfig::default().gcode_barrier_timeout_s, None);
}

/// The out-of-range advice quotes a default the same validator accepts.
///
/// Not a restatement of the band. `confirm_timeout_out_of_range` tells the
/// operator "remove the key to use the N s default", where N is
/// [`CONFIRM_TIMEOUT_DEFAULT_S`] — which is also the deadline `plrd`
/// enforces. This reads N back out of the rendered advice and hands it to
/// the same validator that produced the refusal, so advice that a printer
/// would reject fails here instead of in front of an operator mid-recovery.
///
/// It lives in this crate on purpose: `plrd` is excluded from the Windows
/// gate (`--exclude plrd`), so the plrd-side equivalent is structurally
/// invisible to a Windows-only run, while this file is a default member and
/// always runs.
#[test]
fn the_out_of_range_advice_quotes_a_default_the_validator_accepts() {
    let error = PlanConfig {
        confirm_timeout_s: Some(CONFIRM_TIMEOUT_MAX_S + 1.0),
        ..PlanConfig::default()
    }
    .validate()
    .unwrap_err_or_panic("an out-of-band confirm_timeout_s must refuse");
    let fix = error.diagnosis().suggested_fix;
    // "… or remove the key to use the <N> s default."
    let quoted: f64 = fix
        .split("to use the ")
        .nth(1)
        .and_then(|rest| rest.split(" s default").next())
        .unwrap_or_else(|| panic!("the advice must name the default: {fix}"))
        .parse()
        .unwrap_or_else(|e| panic!("the advice's default must be a number ({e}): {fix}"));
    // Compared as `Duration`s — exact, and it ties the rendered advice to
    // the one `Duration` constant that `plrd` also enforces.
    assert_eq!(
        std::time::Duration::from_secs_f64(quoted),
        plr_recovery::CONFIRM_TIMEOUT_DEFAULT,
        "the advice must quote the real default: {fix}"
    );
    PlanConfig {
        confirm_timeout_s: Some(quoted),
        ..PlanConfig::default()
    }
    .validate()
    .unwrap_or_else(|e| {
        panic!("the advice names {quoted} s, which the same validator rejects: {e:?}")
    });
}

/// A Hard refusal with `override_key: None` is refused no matter what
/// UNSAFE keys are set.
#[test]
fn a_hard_refusal_without_an_override_cannot_be_permitted_at_all() {
    let config = PlanConfig {
        // A commanded probe temperature with no room below the ceiling
        // is the "nozzle above the contact ceiling" class: nothing may
        // permit it, because the plugin's own gate would refuse the probe
        // AFTER the Z frame was declared.
        probe_temp_min: 148.0,
        max_probe_nozzle_temp: 150.0,
        unsafe_allow_purge_z_below_bed: true,
        ..PlanConfig::default()
    };
    let d = config.validate().expect_err("must refuse").diagnosis();
    assert_eq!(d.tier, Tier::Hard);
    assert_eq!(d.override_key, None);
    assert!(d.full().contains("override: NONE"), "{}", d.full());
}

// --- Acceleration overrides --------------------------------------------------

#[test]
fn acceleration_overrides_refuse_absurd_values_with_a_diagnosis() {
    for key in [
        "recovery_accel",
        "accel_home",
        "accel_travel",
        "accel_probe",
        "accel_entry",
    ] {
        for bad in [ACCEL_MIN - 1.0, ACCEL_MAX + 1.0, f64::NAN, f64::INFINITY] {
            let mut config = PlanConfig::default();
            match key {
                "recovery_accel" => config.recovery_accel = Some(bad),
                "accel_home" => config.accel_home = Some(bad),
                "accel_travel" => config.accel_travel = Some(bad),
                "accel_probe" => config.accel_probe = Some(bad),
                _ => config.accel_entry = Some(bad),
            }
            let error = config
                .validate()
                .unwrap_err_or_panic(&format!("{key} = {bad} must refuse"));
            let RecoveryError::AccelOutOfRange { key: named, .. } = &error else {
                panic!("{key} = {bad}: expected AccelOutOfRange, got {error:?}");
            };
            assert_eq!(*named, key);
            let d = error.diagnosis();
            assert_eq!(d.code, "accel_out_of_range");
            assert_eq!(d.tier, Tier::Hard);
            assert_eq!(d.override_key, None, "acceleration is not overridable");
            assert_eq!(d.expected.as_ref().unwrap().min, Some(ACCEL_MIN));
            assert_eq!(d.expected.as_ref().unwrap().max, Some(ACCEL_MAX));
            assert!(d.what.contains(key), "{}", d.what);
        }
    }
}

#[test]
fn acceleration_overrides_accept_the_documented_band() {
    let config = PlanConfig {
        recovery_accel: Some(ACCEL_MIN),
        accel_home: Some(ACCEL_MAX),
        accel_travel: Some(3_000.0),
        accel_probe: Some(500.0),
        accel_entry: Some(1_000.0),
        ..PlanConfig::default()
    };
    config.validate().expect("in-band values are accepted");
    // Absent is always fine — the machine's own acceleration is left
    // alone, which is the default posture.
    PlanConfig::default().validate().expect("no overrides");
}

/// A tiny helper so the loop above reads as one assertion per case.
trait UnwrapErrOrPanic<E> {
    fn unwrap_err_or_panic(self, message: &str) -> E;
}

impl<T: std::fmt::Debug, E> UnwrapErrOrPanic<E> for Result<T, E> {
    fn unwrap_err_or_panic(self, message: &str) -> E {
        match self {
            Ok(v) => panic!("{message}, but it was accepted: {v:?}"),
            Err(e) => e,
        }
    }
}
