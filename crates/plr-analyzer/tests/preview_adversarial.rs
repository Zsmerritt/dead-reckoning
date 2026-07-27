//! Adversarial regression tests for the resume-preview builder
//! (`docs/design/resume-preview.md` §A / §10). Each pins a claim an
//! adversarial reviewer attacked; the shapes were the review's probe
//! shapes, adopted here as committed regressions.

use plr_analyzer::{
    build_layer_model, build_preview, match_stop_point, ByteWindow, ExclusionOracle, Interval,
    LayerModel, MatchConfig, ModelConfig, MoveKind, PreviewBounds, PreviewOutcome, StopEvidence,
};
use plr_gcode::GcodeState;
use std::fmt::Write as _;

fn model_of(text: &str) -> LayerModel {
    build_layer_model(
        GcodeState::new(),
        text.as_bytes(),
        0,
        &ModelConfig::default(),
    )
}

fn whole_file() -> ByteWindow {
    ByteWindow {
        start: 0,
        end: None,
    }
}

fn iv(min: f64, max: f64) -> Interval {
    Interval { min, max }
}

fn wide_box() -> StopEvidence {
    StopEvidence {
        x: iv(0.0, 100.0),
        y: iv(0.0, 100.0),
        e: None,
        z_candidates: vec![],
        window: whole_file(),
    }
}

struct Oracle {
    excluded: Vec<&'static str>,
    conclusive: bool,
}

impl ExclusionOracle for Oracle {
    fn is_conclusive(&self) -> bool {
        self.conclusive
    }
    fn is_excluded(&self, object: &str) -> bool {
        self.excluded.contains(&object)
    }
}

/// MAJOR-1: an in-window extrusion whose start position is UNKNOWN
/// (post-G28, relative mode — the model keeps a *finite* stale value with
/// `start_known == false`) must be neither a hoverable stop nor a baked
/// resume target, mirroring the resolver's `ResumePositionUnknown` refusal.
/// A kept stop whose only following deposition is that untrusted line has
/// no valid resume and is dropped.
#[test]
fn unknown_position_line_is_neither_a_stop_nor_a_resume_target() {
    // A, A2: known extrusions (A's next deposition is A2, so A stays with a
    // valid resume). Then G28 X + relative make U's start-X untrusted; U is
    // A2's only following deposition, so A2 is dropped.
    let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        G1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
        G1 X10 Y12 F9000\nG1 X40 Y12 E0.5 F1800\n\
        G28 X\nG91\nG1 X5 E0.5 F1800\n";
    let m = model_of(text);
    let a_off = text.find("G1 X40 Y10 E0.5").unwrap() as u64;
    let follow_off = text.find("G1 X40 Y12 E0.5").unwrap() as u64;
    let u_off = text.find("G1 X5 E0.5").unwrap() as u64;

    // Ground truth: the model marks U's X unknown yet finite (an is_finite
    // check alone would let it through — the known flag is load-bearing).
    let u = m
        .moves
        .iter()
        .find(|mv| mv.span.start == u_off)
        .expect("U exists");
    assert_eq!(u.kind, MoveKind::Extrusion, "U deposits");
    assert!(!u.start_known[0], "U's X is G28-unknown");
    assert!(
        u.start.iter().all(|v| v.is_finite()),
        "yet U's start is finite"
    );
    assert!(!u.start_position_known(), "so U is not a trusted position");
    // The matcher skips U.
    let r = match_stop_point(&m, &wide_box(), &MatchConfig::default()).unwrap();
    assert!(r.skipped_unknown > 0, "matcher skips the unknown-X move");

    let PreviewOutcome::Preview(set) = build_preview(
        &m,
        &wide_box(),
        &MatchConfig::default(),
        None,
        &PreviewBounds::default(),
    ) else {
        panic!("expected a preview (A is a valid stop)");
    };
    // U is not a hoverable stop, and no kept stop bakes a resume into it.
    assert!(
        set.stops.iter().all(|s| s.offset != u_off),
        "U must not be a stop"
    );
    assert!(
        set.stops.iter().all(|s| s.resume_offset != u_off),
        "no stop may resume into the untrusted line"
    );
    // A2 was dropped (its only following deposition, U, is untrusted — the
    // resolver would refuse a resume there), while A (whose next deposition
    // is the trusted A2) survives. This is the non-vacuous half: a known,
    // matched stop is dropped purely because it has no valid resume.
    assert!(
        set.stops.iter().any(|s| s.offset == a_off),
        "A survives with resume A2"
    );
    assert!(
        set.stops.iter().all(|s| s.offset != follow_off),
        "A2 dropped: no valid resume past it"
    );
    let a = set.stops.iter().find(|s| s.offset == a_off).unwrap();
    assert_eq!(a.resume_offset, follow_off, "A resumes at the trusted A2");
}

/// MAJOR-1, terminal shape: when the untrusted line is the *last*
/// deposition, every earlier stop that would resume into it is dropped and
/// the preview refuses (`NoStops`) — exactly the resolver's outcome
/// (`ResumePositionUnknown` -> manual fallback), the safe direction.
#[test]
fn unknown_terminal_line_refuses_to_no_stops() {
    let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        G1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
        G28 X\nG91\nG1 X5 E0.5 F1800\n";
    let m = model_of(text);
    assert_eq!(
        build_preview(
            &m,
            &wide_box(),
            &MatchConfig::default(),
            None,
            &PreviewBounds::default()
        ),
        PreviewOutcome::NoStops
    );
}

/// MAJOR-2: the max-offset MATCHER candidate can be a TRAVEL line (a wipe
/// crossing the evidence box). Today's skip-forward resumes from the first
/// deposition at/after that travel; the preview's default cursor
/// (`last_index`) must COMMIT that same resume, though the extrusion-only
/// stop set cannot contain the travel.
#[test]
fn default_cursor_matches_skip_forward_when_max_candidate_is_a_travel() {
    // Box: X 24..26, Y 9.8..10.2 (tolerance 0.5).
    // A  (candidate ext):    X10Y10 -> X40Y10 crosses the box.
    // B  (non-cand ext):     X40Y10 -> X40Y60 (leaves the box).
    // T1 (non-cand travel):  X40Y60 -> X40Y10.
    // T2 (candidate travel): X40Y10 -> X25Y10 ends IN the box.
    // T3 (candidate travel): X25Y10 -> X10Y40 leaves the box.
    // C  (non-cand ext):     X10Y40 -> X10Y60.
    let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        G1 X10 Y10 F9000\n\
        G1 X40 Y10 E0.5 F1800\n\
        G1 X40 Y60 E0.5 F1800\n\
        G1 X40 Y10 F9000\n\
        G1 X25 Y10 F9000\n\
        G1 X10 Y40 F9000\n\
        G1 X10 Y60 E0.5 F1800\n";
    let m = model_of(text);
    let a_off = text.find("G1 X40 Y10 E0.5").unwrap() as u64;
    let b_off = text.find("G1 X40 Y60 E0.5").unwrap() as u64;
    let c_off = text.find("G1 X10 Y60 E0.5").unwrap() as u64;
    let evidence = StopEvidence {
        x: iv(24.0, 26.0),
        y: iv(9.8, 10.2),
        e: None,
        z_candidates: vec![],
        window: whole_file(),
    };
    let cfg = MatchConfig::default();
    let r = match_stop_point(&m, &evidence, &cfg).unwrap();
    let plr_analyzer::MatchConfidence::AmbiguousWindow { offsets } = &r.confidence else {
        panic!("expected AmbiguousWindow, got {:?}", r.confidence);
    };
    let max_off = *offsets.iter().max().unwrap();
    assert_ne!(
        max_off, a_off,
        "the max candidate is a travel, not the extrusion A"
    );
    // Today's resolver resume: first deposition at/after the max candidate.
    let today_resume = m.first_deposition_at_or_after(max_off).unwrap().span.start;
    assert_eq!(today_resume, c_off, "today's skip-forward lands on C");

    let PreviewOutcome::Preview(set) =
        build_preview(&m, &evidence, &cfg, None, &PreviewBounds::default())
    else {
        panic!("expected a preview");
    };
    let last = &set.stops[set.last_index as usize];
    assert_eq!(
        last.resume_offset, today_resume,
        "default cursor commits today's skip-forward (C)"
    );
    // The pre-fix bug baked B (resume from the max EXTRUSION candidate A);
    // that re-printed a line the automatic path skips.
    assert_ne!(
        last.resume_offset, b_off,
        "must not resume at B (the pre-fix regression)"
    );
}

/// MINOR-1: `mid` must select over the DISTINCT candidate LINE offsets
/// (matcher population, travels/arcs deduped per line) — not the chord-
/// duplicated extrusion stop set — and commit the resolver's resume from
/// that median, matching `plr-recovery`'s `select_offset` convention.
#[test]
fn mid_matches_the_per_line_median_under_arc_chords() {
    // An early arc (many chords sharing one span) then several straight
    // extrusions — arranged so the per-line median lands on a straight
    // extrusion line while the arc's chords dominate a per-STOP median. This
    // makes the two populations pick DIFFERENT stops, so the fix is
    // observable (not just internally different).
    let text = "G90\nM82\nG92 E0\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        G1 X10 Y10 F9000\nG1 E1 F2100\n\
        G3 X10 Y30 J10 E3 F1800\n\
        G1 X40 Y30 E4 F1800\nG1 X40 Y10 E5 F1800\n\
        G1 X20 Y40 E6 F1800\nG1 X30 Y40 E7 F1800\nG1 X30 Y20 E8 F1800\n";
    let m = model_of(text);
    let arc_off = text.find("G3 ").unwrap() as u64;
    let cfg = MatchConfig {
        ambiguity_limit: 1000,
        ..MatchConfig::default()
    };
    let evidence = StopEvidence {
        x: iv(0.0, 60.0),
        y: iv(0.0, 60.0),
        e: None,
        z_candidates: vec![],
        window: whole_file(),
    };
    // The arc really decomposes into many chords sharing arc_off.
    let arc_chords = m
        .moves
        .iter()
        .filter(|mv| mv.span.start == arc_off && mv.kind == MoveKind::Extrusion)
        .count();
    assert!(arc_chords > 2, "arc decomposes into many chords");

    let r = match_stop_point(&m, &evidence, &cfg).unwrap();
    // `mid`'s selection population is the DISTINCT candidate LINE offsets
    // (arcs deduped to one line — `select_offset`'s population), NOT the
    // chord-duplicated stop set.
    let mut lines: Vec<u64> = r.candidates.iter().map(|c| c.offset).collect();
    lines.sort_unstable();
    lines.dedup();
    let per_line_median = lines[(lines.len() - 1) / 2];

    let PreviewOutcome::Preview(set) =
        build_preview(&m, &evidence, &cfg, None, &PreviewBounds::default())
    else {
        panic!("expected a preview");
    };
    // The chord-duplicated (pre-fix) median over candidate STOPS lands on a
    // DIFFERENT line — the arc — because its many chords dominate. Prove the
    // populations diverge here so the pin is non-vacuous.
    let mut cand_stop_offsets: Vec<u64> = set
        .stops
        .iter()
        .filter(|s| s.is_candidate)
        .map(|s| s.offset)
        .collect();
    cand_stop_offsets.sort_unstable();
    let chord_skewed_median = cand_stop_offsets[(cand_stop_offsets.len() - 1) / 2];
    assert_eq!(
        chord_skewed_median, arc_off,
        "the chord-duplicated median is the arc"
    );
    assert_ne!(
        per_line_median, chord_skewed_median,
        "per-line and chord-skewed medians must differ for this to bite"
    );
    // mid selects the per-line median (a straight extrusion stop), not the
    // chord-skewed arc.
    assert_eq!(
        set.stops[set.mid_index as usize].offset, per_line_median,
        "mid selects the per-line median candidate line, not the chord-skewed arc"
    );
    assert_ne!(
        set.stops[set.mid_index as usize].offset, arc_off,
        "mid must not collapse onto the chord-duplicated arc line (the pre-fix bug)"
    );
}

/// Settled (adopted): the layer bound counts CANDIDATE layers, not window
/// layers — a window spanning 9 layers whose candidates sit in 2 ADMITS.
#[test]
fn nine_window_layers_two_candidate_layers_admits() {
    let mut text = String::from("G90\nM83\n;TYPE:Sparse infill\n");
    for i in 0..9 {
        let z = 0.2 + f64::from(i) * 0.2;
        let y = if i < 2 { 10 } else { 50 };
        write!(
            text,
            "G1 Z{z} F7200\nG1 X10 Y{y} F9000\nG1 X40 Y{y} E0.5 F1800\n"
        )
        .unwrap();
    }
    let m = model_of(&text);
    assert!(m.layers.len() >= 9, "window really spans 9 layers");
    let evidence = StopEvidence {
        x: iv(24.0, 26.0),
        y: iv(9.8, 10.2),
        e: None,
        z_candidates: vec![],
        window: whole_file(),
    };
    let PreviewOutcome::Preview(set) = build_preview(
        &m,
        &evidence,
        &MatchConfig::default(),
        None,
        &PreviewBounds::default(),
    ) else {
        panic!("9 window layers / 2 candidate layers must ADMIT");
    };
    let cand_layers: std::collections::BTreeSet<_> = set
        .stops
        .iter()
        .filter(|s| s.is_candidate)
        .map(|s| s.layer)
        .collect();
    assert_eq!(cand_layers.len(), 2);
    assert!(
        set.stops.iter().any(|s| !s.is_candidate),
        "non-candidate in-window lines stay"
    );
}

/// Settled (adopted), D9 resume side: a kept stop whose next deposition is
/// EXCLUDED, with a later kept deposition, resumes past the excluded line
/// onto the later kept one.
#[test]
fn resume_skips_excluded_deposition_to_next_kept() {
    let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        EXCLUDE_OBJECT_START NAME=PART_A\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
        EXCLUDE_OBJECT_END NAME=PART_A\n\
        EXCLUDE_OBJECT_START NAME=PART_B\nG1 X10 Y30 F9000\nG1 X40 Y30 E0.5 F1800\n\
        EXCLUDE_OBJECT_END NAME=PART_B\n\
        EXCLUDE_OBJECT_START NAME=PART_C\nG1 X10 Y50 F9000\nG1 X40 Y50 E0.5 F1800\n\
        EXCLUDE_OBJECT_END NAME=PART_C\n";
    let m = model_of(text);
    let a_off = text.find("G1 X40 Y10 E0.5").unwrap() as u64;
    let b_off = text.find("G1 X40 Y30 E0.5").unwrap() as u64;
    let c_off = text.find("G1 X40 Y50 E0.5").unwrap() as u64;
    let oracle = Oracle {
        excluded: vec!["PART_B"],
        conclusive: true,
    };
    let PreviewOutcome::Preview(set) = build_preview(
        &m,
        &wide_box(),
        &MatchConfig {
            ambiguity_limit: 100,
            ..MatchConfig::default()
        },
        Some(&oracle),
        &PreviewBounds::default(),
    ) else {
        panic!("expected a preview");
    };
    let a_stop = set
        .stops
        .iter()
        .find(|s| s.offset == a_off)
        .expect("kept PART_A stop");
    assert_ne!(
        a_stop.resume_offset, b_off,
        "must not resume into excluded PART_B"
    );
    assert_eq!(
        a_stop.resume_offset, c_off,
        "resumes at the next kept deposition PART_C"
    );
}

/// Settled (adopted): the nudge domain is extrusion-only (travels and
/// retracts absent) and complete over a known-position file.
#[test]
fn domain_is_extrusion_only_and_complete() {
    let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        G1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\nG1 E-0.8 F2100\nG1 X20 Y20 F9000\n\
        G1 E0.8 F2100\nG1 X40 Y20 E0.5 F1800\n";
    let m = model_of(text);
    let travel_off = text.find("G1 X20 Y20 F9000").unwrap() as u64;
    let retract_off = text.find("G1 E-0.8").unwrap() as u64;
    let PreviewOutcome::Preview(set) = build_preview(
        &m,
        &wide_box(),
        &MatchConfig::default(),
        None,
        &PreviewBounds::default(),
    ) else {
        panic!("expected a preview");
    };
    assert!(
        set.stops.iter().all(|s| s.offset != travel_off),
        "travel excluded"
    );
    assert!(
        set.stops.iter().all(|s| s.offset != retract_off),
        "retract excluded"
    );
    for mv in m.moves.iter().filter(|mv| mv.kind == MoveKind::Extrusion) {
        assert!(
            set.stops.iter().any(|s| s.offset == mv.span.start),
            "extrusion at {} missing from the nudge domain",
            mv.span.start
        );
    }
}

/// Settled (adopted): inconclusive oracle keeps everything not positively
/// excluded, drops the positively-excluded (the wide, safe direction).
#[test]
fn inconclusive_oracle_keeps_all_but_positively_excluded() {
    let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        G1 X5 Y5 F9000\nG1 X60 Y5 E0.5 F1800\n\
        EXCLUDE_OBJECT_START NAME=PART_A\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
        EXCLUDE_OBJECT_END NAME=PART_A\n\
        EXCLUDE_OBJECT_START NAME=PART_B\nG1 X10 Y50 F9000\nG1 X40 Y50 E0.5 F1800\n\
        EXCLUDE_OBJECT_END NAME=PART_B\n";
    let m = model_of(text);
    let skirt_off = text.find("G1 X60 Y5 E0.5").unwrap() as u64;
    let a_off = text.find("G1 X40 Y10 E0.5").unwrap() as u64;
    let b_off = text.find("G1 X40 Y50 E0.5").unwrap() as u64;
    let oracle = Oracle {
        excluded: vec!["PART_B"],
        conclusive: false,
    };
    let PreviewOutcome::Preview(set) = build_preview(
        &m,
        &wide_box(),
        &MatchConfig {
            ambiguity_limit: 100,
            ..MatchConfig::default()
        },
        Some(&oracle),
        &PreviewBounds::default(),
    ) else {
        panic!("expected a preview");
    };
    assert!(
        set.stops.iter().all(|s| s.offset != b_off),
        "positively-excluded PART_B dropped"
    );
    assert!(
        set.stops.iter().any(|s| s.offset == a_off),
        "PART_A kept (not positively excluded)"
    );
    assert!(
        set.stops.iter().any(|s| s.offset == skirt_off),
        "unattributed skirt kept"
    );
}
