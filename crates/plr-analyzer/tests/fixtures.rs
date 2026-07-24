//! Integration tests over the committed fixture corpus
//! (`fixtures/synthetic/`) and any real sliced files dropped into
//! `fixtures/real/`.

#![allow(clippy::float_cmp)] // exact replay equality is intentional

use std::fs;
use std::path::{Path, PathBuf};

use plr_analyzer::{
    assess_contact_point, build_layer_model, match_stop_point, select_contact_zone,
    select_contact_zone_detailed, ByteWindow, ContactConfig, ContactError, ContactMode,
    ContactOutcome, DeclineReason, FeatureClass, Interval, LayerModel, MatchConfidence,
    MatchConfig, ModelConfig, MoveKind, SimMove, StopEvidence, StructuralAnalysis,
    StructuralAssessment, StructuralCriterion, StructuralOutcome, StructuralVerdict, TraceStatus,
};
use plr_gcode::GcodeState;

fn fixtures_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../fixtures")
}

fn gcode_files(dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut files: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case("gcode"))
        })
        .collect();
    files.sort();
    files
}

fn load(name: &str) -> Vec<u8> {
    let path = fixtures_dir().join("synthetic").join(name);
    fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn model_of(data: &[u8]) -> LayerModel {
    build_layer_model(GcodeState::new(), data, 0, &ModelConfig::default())
}

/// Byte offset of the first occurrence of `needle` in `data` — the
/// span.start of that line when the needle starts a line.
fn offset_of(data: &[u8], needle: &str) -> u64 {
    let n = needle.as_bytes();
    data.windows(n.len())
        .position(|w| w == n)
        .unwrap_or_else(|| panic!("needle {needle:?} not found")) as u64
}

fn iv(min: f64, max: f64) -> Interval {
    Interval { min, max }
}

fn whole_file() -> ByteWindow {
    ByteWindow {
        start: 0,
        end: None,
    }
}

/// Every fixture must produce a sane model: no panic, layer indices
/// sequential, deposition inside every layer, and all matcher inputs
/// derived from the model must land on line boundaries.
fn check_model_invariants(model: &LayerModel, len: u64) {
    for (i, layer) in model.layers.iter().enumerate() {
        assert_eq!(layer.index as usize, i);
        assert!(layer.extrusion_moves >= 1);
        assert!(layer.span.start < layer.span.end);
        assert!(layer.span.end <= len);
        assert!(
            !layer.paths.is_empty(),
            "layer {i} has extrusion but no paths"
        );
        let segment_count: usize = layer.paths.iter().map(|p| p.segments.len()).sum();
        assert!(segment_count >= 1);
    }
    let mut prev_start = 0_u64;
    for mv in &model.moves {
        assert!(mv.span.start <= mv.span.end);
        assert!(mv.span.end <= len);
        assert!(
            mv.span.start >= prev_start,
            "moves must be in file order (arcs share a span)"
        );
        prev_start = mv.span.start;
    }
}

#[test]
fn synthetic_corpus_builds_sane_models() {
    let files = gcode_files(&fixtures_dir().join("synthetic"));
    assert!(!files.is_empty(), "synthetic corpus missing");
    for path in files {
        let data = fs::read(&path).expect("read fixture");
        let model = model_of(&data);
        check_model_invariants(&model, data.len() as u64);
        // The selector must never panic at any resume layer, whatever
        // its verdict.
        for n in 0..=u32::try_from(model.layers.len()).unwrap() {
            let _ = select_contact_zone(&model, n, [0.0, 0.0], &ContactConfig::default());
        }
    }
}

#[test]
fn real_corpus_builds_sane_models() {
    for path in gcode_files(&fixtures_dir().join("real")) {
        let data = fs::read(&path).expect("read fixture");
        let model = model_of(&data);
        check_model_invariants(&model, data.len() as u64);
        for n in 0..=u32::try_from(model.layers.len().min(30)).unwrap() {
            let _ = select_contact_zone(&model, n, [0.0, 0.0], &ContactConfig::default());
        }
    }
}

#[test]
fn zhop_fixture_matches_through_a_hop() {
    let data = load("zhop_retract.gcode");
    let model = model_of(&data);
    // Stop mid-travel at hop height Z0.6, X between 55 and 65.
    let evidence = StopEvidence {
        x: iv(55.0, 65.0),
        y: iv(9.5, 10.5),
        e: Some(iv(0.15, 0.25)),
        z_candidates: vec![0.6],
        window: whole_file(),
    };
    let r = match_stop_point(&model, &evidence, &MatchConfig::default()).expect("match");
    assert_eq!(
        r.confidence,
        MatchConfidence::UniqueLine {
            offset: offset_of(&data, "G1 X80 Y10 F9000")
        }
    );
    assert_eq!(r.candidates[0].kind, MoveKind::Travel);
    assert_eq!(r.candidates[0].position[2], 0.6);
    // Stop mid-hop on the Z move itself: Z evidence between the layer
    // planes still finds the hop line uniquely.
    let evidence = StopEvidence {
        x: iv(39.5, 40.5),
        y: iv(9.5, 10.5),
        e: Some(iv(0.15, 0.25)),
        z_candidates: vec![0.4],
        window: whole_file(),
    };
    let r = match_stop_point(&model, &evidence, &MatchConfig::default()).expect("match");
    assert_eq!(
        r.confidence,
        MatchConfidence::UniqueLine {
            offset: offset_of(&data, "G1 Z0.6 F7200")
        }
    );
    // Resume selection: the first deposition at or after the matched
    // hop line skips the travel, un-hop and unretract (extrude-only is
    // not deposition) and lands on the X80 Y40 extrusion.
    let matched = offset_of(&data, "G1 Z0.6 F7200");
    let resume = model
        .first_deposition_at_or_after(matched)
        .expect("resume deposition");
    assert_eq!(resume.span.start, offset_of(&data, "G1 X80 Y40 E1.0"));
}

#[test]
fn arc_fixtures_match_equivalently() {
    let data_ij = load("arcs_ij.gcode");
    let data_pre = load("arcs_prechorded.gcode");
    let model_ij = model_of(&data_ij);
    let model_pre = model_of(&data_pre);
    // Derive the evidence from the IJ model: the 8th chord of the
    // first arc (G3 X0 Y10 I-10 E3), around 45 degrees.
    let g3_offset = offset_of(&data_ij, "G3 X0 Y10");
    let chord: &SimMove = model_ij
        .moves
        .iter()
        .find(|m| m.span.start == g3_offset && m.arc.map(|a| a.index) == Some(8))
        .expect("chord 8 of the quarter arc");
    let mid = [
        (chord.start[0] + chord.end[0]) * 0.5,
        (chord.start[1] + chord.end[1]) * 0.5,
        (chord.start[3] + chord.end[3]) * 0.5,
    ];
    let evidence = StopEvidence {
        x: iv(mid[0] - 0.05, mid[0] + 0.05),
        y: iv(mid[1] - 0.05, mid[1] + 0.05),
        e: Some(iv(mid[2] - 0.02, mid[2] + 0.02)),
        z_candidates: vec![0.4],
        window: whole_file(),
    };
    let cfg = MatchConfig {
        xy_tolerance: 0.1,
        e_tolerance: 0.05,
        ..MatchConfig::default()
    };
    let r_ij = match_stop_point(&model_ij, &evidence, &cfg).expect("ij match");
    let r_pre = match_stop_point(&model_pre, &evidence, &cfg).expect("prechorded match");
    // IJ: unique on the G3 source line, chord identified.
    assert_eq!(
        r_ij.confidence,
        MatchConfidence::UniqueLine { offset: g3_offset }
    );
    assert!(r_ij.candidates[0].arc.is_some());
    // Prechorded: unique on the equivalent G1 chord line, no arc info.
    let MatchConfidence::UniqueLine { offset: pre_offset } = r_pre.confidence else {
        panic!("expected unique match, got {:?}", r_pre.confidence);
    };
    assert!(r_pre.candidates[0].arc.is_none());
    // The prechorded match lands on a plain G1 chord line.
    let pre_line = &data_pre[usize::try_from(pre_offset).unwrap()..];
    assert!(pre_line.starts_with(b"G1 "), "unexpected line at match");
    // Same physical stop point: positions agree to within one chord.
    let p_ij = r_ij.candidates[0].position;
    let p_pre = r_pre.candidates[0].position;
    for axis in 0..3 {
        assert!(
            (p_ij[axis] - p_pre[axis]).abs() < 1.0,
            "axis {axis}: {p_ij:?} vs {p_pre:?}"
        );
    }
}

#[test]
fn vase_fixture_declines_probing() {
    let data = load("vase_mode.gcode");
    let model = model_of(&data);
    assert!(model.layers.len() >= 2, "spiral must span layers");
    assert!(model.spiral_fraction() > 0.9);
    let out = select_contact_zone(&model, 1, [200.0, 200.0], &ContactConfig::default())
        .expect("selection runs");
    assert!(
        matches!(
            out,
            ContactOutcome::Declined(DeclineReason::VaseMode { .. })
        ),
        "got {out:?}"
    );
}

#[test]
fn state_changes_fixture_matches_after_g92_m220_m221() {
    let data = load("state_changes.gcode");
    let model = model_of(&data);
    // Target the extrusion after RESTORE_GCODE_STATE, M204, G92 (bare)
    // and both factor changes: G1 X20 Y20 E2.0 F3000.
    let target = offset_of(&data, "G1 X20 Y20 E2.0");
    let mv = model
        .moves
        .iter()
        .find(|m| m.span.start == target)
        .expect("target move in model");
    let mid = [
        (mv.start[0] + mv.end[0]) * 0.5,
        (mv.start[1] + mv.end[1]) * 0.5,
        (mv.start[3] + mv.end[3]) * 0.5,
    ];
    let evidence = StopEvidence {
        x: iv(mid[0] - 0.2, mid[0] + 0.2),
        y: iv(mid[1] - 0.2, mid[1] + 0.2),
        e: Some(iv(mid[2] - 0.05, mid[2] + 0.05)),
        z_candidates: vec![mv.end[2]],
        window: whole_file(),
    };
    let cfg = MatchConfig {
        e_tolerance: 0.1,
        ..MatchConfig::default()
    };
    let r = match_stop_point(&model, &evidence, &cfg).expect("match");
    assert_eq!(r.confidence, MatchConfidence::UniqueLine { offset: target });
    // The matched position reproduces the simulated midpoint.
    let c = &r.candidates[0];
    assert!((c.position[0] - mid[0]).abs() < 1e-9);
    assert!((c.position[3] - mid[2]).abs() < 1e-9);
}

#[test]
fn g28_fixture_unknown_window_is_skipped_never_trusted() {
    let data = load("g28_midfile.gcode");
    let model = model_of(&data);
    // The relative Z2 hop right after G28 Z has unknown Z. With Z
    // evidence it must be skipped; the known extrusion wins uniquely.
    let evidence = StopEvidence {
        x: iv(29.5, 30.5),
        y: iv(9.5, 10.5),
        e: Some(iv(0.9, 1.1)),
        z_candidates: vec![0.2],
        window: whole_file(),
    };
    let r = match_stop_point(&model, &evidence, &MatchConfig::default()).expect("match");
    assert!(r.skipped_unknown > 0, "unknown-axis moves must be skipped");
    assert_eq!(
        r.confidence,
        MatchConfidence::UniqueLine {
            offset: offset_of(&data, "G1 X30 Y10 E1.0")
        }
    );
    // A window restricted to the unknown region matches nothing rather
    // than trusting unknown positions.
    let unknown_window = StopEvidence {
        window: ByteWindow {
            start: offset_of(&data, "G28 Z"),
            // The mid-file G90 (the header G90 is at offset 0).
            end: Some(offset_of(&data, "G90\nG1 Z0.4")),
        },
        ..evidence
    };
    let err = match_stop_point(&model, &unknown_window, &MatchConfig::default()).unwrap_err();
    assert_eq!(err, plr_analyzer::MatchError::NoMatch);
}

#[test]
fn repeated_infill_fixture_is_honestly_ambiguous() {
    let data = load("repeated_infill.gcode");
    let model = model_of(&data);
    let base = StopEvidence {
        x: iv(29.7, 30.3),
        y: iv(19.7, 20.3),
        e: None,
        z_candidates: vec![0.2],
        window: whole_file(),
    };
    // Without E evidence: every retrace (3 extrusions + 2 travels back)
    // is listed; never a fake unique match.
    let r = match_stop_point(&model, &base, &MatchConfig::default()).expect("match");
    let MatchConfidence::AmbiguousWindow { offsets } = &r.confidence else {
        panic!("expected AmbiguousWindow, got {:?}", r.confidence);
    };
    assert_eq!(offsets.len(), 5);
    let first_ext = offset_of(&data, "G1 X40 Y20 E0.5");
    assert!(offsets.contains(&first_ext));
    // A low ambiguity limit degrades to the layer, never to a line.
    let cfg = MatchConfig {
        ambiguity_limit: 2,
        ..MatchConfig::default()
    };
    let r = match_stop_point(&model, &base, &cfg).expect("match");
    assert_eq!(r.confidence, MatchConfidence::LayerOnly { layer: 0 });
    // A tight internal-E interval mid-first-trace disambiguates.
    let tight = StopEvidence {
        e: Some(iv(0.2, 0.3)),
        ..base
    };
    let cfg = MatchConfig {
        e_tolerance: 0.05,
        ..MatchConfig::default()
    };
    let r = match_stop_point(&model, &tight, &cfg).expect("match");
    assert_eq!(
        r.confidence,
        MatchConfidence::UniqueLine { offset: first_ext }
    );
}

#[test]
fn two_layer_hatch_contact_zone_exact() {
    let data = load("two_layer_hatch.gcode");
    let model = model_of(&data);
    assert_eq!(model.layers.len(), 2);
    // These assertions pin the surface-quality selector: a 20 mm toy
    // square of sparse infill genuinely fails the 100 mm² bed-contact
    // bar, so the structural stage is switched off here and covered by
    // the struct_* fixtures below.
    let cfg = ContactConfig {
        exclusion_radius: 2.0,
        structural_checks_enabled: false,
        ..ContactConfig::default()
    };
    let out = select_contact_zone(&model, 1, [30.0, 30.0], &cfg).expect("selection");
    let ContactOutcome::Candidates(cands) = out else {
        panic!("expected candidates, got {out:?}");
    };
    // Top candidates: the two short diagonals' midpoints (27,33) and
    // (33,27) — sparse infill, covered by layer 2's anti-diagonal,
    // outside the 2 mm exclusion around the crash at the hatch center.
    assert!(cands.len() >= 2);
    assert_eq!(cands[0].class, FeatureClass::InternalInfill);
    assert_eq!(cands[1].class, FeatureClass::InternalInfill);
    let sparse_points: Vec<[f64; 2]> = cands
        .iter()
        .filter(|c| c.class == FeatureClass::InternalInfill)
        .map(|c| c.point)
        .collect();
    assert!(sparse_points.contains(&[27.0, 33.0]));
    assert!(sparse_points.contains(&[33.0, 27.0]));
    // The main diagonal's midpoint (30,30) is the crash itself: it and
    // its fallback samples must not appear.
    assert!(!cands.iter().any(|c| c.point == [30.0, 30.0]));
    // The solid-infill line is not covered by layer 2: absent.
    assert!(cands.iter().all(|c| c.class != FeatureClass::SolidInfill));
    // Never the outer wall, never inside the exclusion radius, always
    // exactly on an N-1 segment.
    let prev = &model.layers[0];
    for c in &cands {
        assert_ne!(c.class, FeatureClass::OuterWall);
        let dx = c.point[0] - 30.0;
        let dy = c.point[1] - 30.0;
        assert!((dx * dx + dy * dy).sqrt() > 2.0);
        assert_eq!(c.z, 0.2);
        let on_prev = prev.paths.iter().flat_map(|p| p.segments.iter()).any(|s| {
            let t = ((c.point[0] - s.start[0]) * (s.end[0] - s.start[0])
                + (c.point[1] - s.start[1]) * (s.end[1] - s.start[1]))
                / ((s.end[0] - s.start[0]).powi(2) + (s.end[1] - s.start[1]).powi(2));
            let proj = [
                s.start[0] + (s.end[0] - s.start[0]) * t.clamp(0.0, 1.0),
                s.start[1] + (s.end[1] - s.start[1]) * t.clamp(0.0, 1.0),
            ];
            let d = ((c.point[0] - proj[0]).powi(2) + (c.point[1] - proj[1]).powi(2)).sqrt();
            d <= 1e-9
        });
        assert!(on_prev, "candidate {:?} not on an N-1 segment", c.point);
    }
    // Inner-wall candidates fill the remaining slots.
    assert!(cands
        .iter()
        .skip(2)
        .all(|c| c.class == FeatureClass::InnerWall));
}

#[test]
fn orca_fixture_contact_zone_uses_inner_wall_under_coverage() {
    // Orca fixture: layer 2 only deposits two outer-wall segments, so
    // only the inner-wall midpoints they retrace are covered; sparse
    // and solid infill of layer 1 are uncovered and must be filtered.
    let data = load("orca_relative_e.gcode");
    let model = model_of(&data);
    assert_eq!(model.layers.len(), 2);
    // Pre-structural semantics (see the note in the hatch test above).
    let cfg = ContactConfig {
        structural_checks_enabled: false,
        ..ContactConfig::default()
    };
    let out = select_contact_zone(&model, 1, [0.0, 0.0], &cfg).expect("selection");
    let ContactOutcome::Candidates(cands) = out else {
        panic!("expected candidates, got {out:?}");
    };
    assert_eq!(cands.len(), 2);
    assert!(cands.iter().all(|c| c.class == FeatureClass::InnerWall));
    let points: Vec<[f64; 2]> = cands.iter().map(|c| c.point).collect();
    assert!(points.contains(&[50.0, 40.0]));
    assert!(points.contains(&[60.0, 50.0]));
    // Farther from the crash ranks first.
    assert_eq!(cands[0].point, [60.0, 50.0]);
    // A crash on the covered wall segment excludes its midpoint and the
    // strict exclusion boundary (distance == radius is still excluded).
    let out = select_contact_zone(&model, 1, [50.0, 40.0], &cfg).expect("selection");
    let ContactOutcome::Candidates(cands) = out else {
        panic!("expected candidates, got {out:?}");
    };
    assert_eq!(cands.len(), 1);
    assert_eq!(cands[0].point, [60.0, 50.0]);
}

#[test]
fn prusa_fixture_absolute_e_matches_in_internal_frame() {
    let data = load("prusa_absolute_e.gcode");
    let model = model_of(&data);
    // Layer at Z0.4 re-zeroes E with G92 E0; internal E keeps
    // accumulating. Target its second perimeter extrusion.
    let target = offset_of(&data, "G1 X50 Y50 E1.2442");
    let mv = model
        .moves
        .iter()
        .find(|m| m.span.start == target)
        .expect("target move");
    // Internal E continues past layer 1's total despite the G92 E0.
    assert!(mv.start[3] > 8.0, "internal E accumulates: {}", mv.start[3]);
    let mid_e = (mv.start[3] + mv.end[3]) * 0.5;
    let evidence = StopEvidence {
        x: iv(49.9, 50.1),
        y: iv(39.0, 41.0),
        e: Some(iv(mid_e - 0.05, mid_e + 0.05)),
        z_candidates: vec![0.4],
        window: whole_file(),
    };
    let cfg = MatchConfig {
        e_tolerance: 0.1,
        ..MatchConfig::default()
    };
    let r = match_stop_point(&model, &evidence, &cfg).expect("match");
    assert_eq!(r.confidence, MatchConfidence::UniqueLine { offset: target });
}

#[test]
fn annotationless_fixtures_refuse_contact_selection() {
    for name in ["zhop_retract.gcode", "g28_midfile.gcode"] {
        let data = load(name);
        let model = model_of(&data);
        if model.layers.len() < 2 {
            continue;
        }
        assert_eq!(
            select_contact_zone(&model, 1, [0.0, 0.0], &ContactConfig::default()).unwrap_err(),
            ContactError::NoTypeAnnotations,
            "{name}"
        );
    }
}

#[test]
fn helical_planes_fixture_spiral_and_annotationless() {
    let data = load("helical_planes.gcode");
    let model = model_of(&data);
    // The only extrusions are helical chords: spiral fraction 1.
    assert_eq!(model.spiral_fraction(), 1.0);
    assert!(!model.annotated);
}

#[test]
fn mixed_endings_fixture_matches_with_crlf_spans() {
    let data = load("mixed_endings.gcode");
    let model = model_of(&data);
    // Stop on the extrusion N12 G1 X60 Y40 E2.0 (CRLF file, N-numbers,
    // checksums). The matched offset must be the line boundary of that
    // exact line.
    let target = offset_of(&data, "N12 G1 X60 Y40");
    let evidence = StopEvidence {
        x: iv(59.5, 60.5),
        y: iv(24.0, 26.0),
        e: Some(iv(1.4, 1.6)),
        z_candidates: vec![0.2],
        window: whole_file(),
    };
    let r = match_stop_point(&model, &evidence, &MatchConfig::default()).expect("match");
    assert_eq!(r.confidence, MatchConfidence::UniqueLine { offset: target });
}

// --- structural safety over the struct_* fixtures ------------------

/// The structural fixtures are generated by
/// `fixtures/synthetic/structural_generator.py`; each file's header
/// states the geometry and the numbers derived from it.
fn analysis_of(name: &str, layer: u32) -> (LayerModel, StructuralAnalysis) {
    let model = model_of(&load(name));
    let analysis = StructuralAnalysis::build(&model, layer, &ContactConfig::default())
        .expect("structural analysis");
    (model, analysis)
}

fn verdict_of(
    analysis: &StructuralAnalysis,
    point: [f64; 2],
    mode: &ContactMode,
) -> StructuralVerdict {
    match analysis.assess(point, mode) {
        StructuralAssessment::Evaluated(verdict) => *verdict,
        other => panic!("expected a verdict at {point:?}, got {other:?}"),
    }
}

fn measured(verdict: &StructuralVerdict, criterion: StructuralCriterion) -> f64 {
    verdict
        .check(criterion)
        .expect("criterion present")
        .measured
}

#[test]
fn flat_plate_fixture_passes_every_criterion() {
    let (model, analysis) = analysis_of("struct_flat_plate.gcode", 1);
    assert_eq!(model.layers.len(), 3);
    assert_eq!(analysis.islands().len(), 1, "a solid plate is one island");
    let island = &analysis.islands()[0];
    assert_eq!(island.segment_count, 51);
    assert_eq!(island.bbox.min, [0.0, 0.0]);
    assert_eq!(island.bbox.max, [20.0, 20.0]);
    // The area estimate is a lower bound on the 20 x 20 outline and
    // stays within the capsule union of 51 beads.
    assert!(
        island.area > 300.0 && island.area < 400.0,
        "{}",
        island.area
    );
    let trace = analysis.footprint(0).expect("trace");
    assert_eq!(trace.status, TraceStatus::Anchored);
    assert_eq!(trace.reached_layer, 0);
    assert_eq!(trace.effective_area, island.area);
    let verdict = verdict_of(&analysis, [10.0, 10.0], &ContactMode::Tap);
    assert_eq!(verdict.outcome(), StructuralOutcome::Safe);
    assert!(measured(&verdict, StructuralCriterion::EdgeMargin) > 9.5);
    assert!(measured(&verdict, StructuralCriterion::Tipping) < 0.1);
    assert_eq!(measured(&verdict, StructuralCriterion::FeatureWidth), 20.0);
    // ... and the selector returns candidates rather than declining.
    let selection =
        select_contact_zone_detailed(&model, 2, [50.0, 50.0], &ContactConfig::default())
            .expect("selection");
    let ContactOutcome::Candidates(candidates) = &selection.outcome else {
        panic!("expected candidates, got {:?}", selection.outcome);
    };
    assert!(!candidates.is_empty());
    assert!(selection
        .verdicts
        .iter()
        .all(|v| v.outcome() == StructuralOutcome::Safe));
}

#[test]
fn tall_pillar_fixture_fails_adhesion_aspect_and_width() {
    // Top layer of the 24-layer, 2.5 x 2.4 mm column: Z9.6.
    let (model, analysis) = analysis_of("struct_tall_pillar.gcode", 23);
    assert_eq!(model.layers.len(), 24);
    assert!((analysis.layer_z() - 9.6).abs() < 1e-9);
    assert_eq!(analysis.islands().len(), 1);
    let verdict = verdict_of(&analysis, [1.25, 1.2], &ContactMode::Tap);
    let failed: Vec<StructuralCriterion> = verdict.failures().iter().map(|c| c.criterion).collect();
    assert!(failed.contains(&StructuralCriterion::BedAdhesion));
    assert!(failed.contains(&StructuralCriterion::Tipping));
    assert!(failed.contains(&StructuralCriterion::FeatureWidth));
    assert_eq!(
        verdict.outcome(),
        StructuralOutcome::Unsafe {
            primary: StructuralCriterion::BedAdhesion
        }
    );
    // The measured numbers are the ones the fixture header claims.
    let area = measured(&verdict, StructuralCriterion::BedAdhesion);
    assert!((5.0..10.0).contains(&area), "footprint {area}");
    let aspect = measured(&verdict, StructuralCriterion::Tipping);
    assert!((aspect - 9.6 / area.sqrt()).abs() < 1e-9);
    assert!(aspect > 3.0, "aspect {aspect}");
    assert_eq!(measured(&verdict, StructuralCriterion::FeatureWidth), 2.4);
    // And the selector declines with that evidence attached.
    let selection =
        select_contact_zone_detailed(&model, 23, [50.0, 50.0], &ContactConfig::default())
            .expect("selection");
    let ContactOutcome::Declined(DeclineReason::NoStructurallySafePoint { rejected, .. }) =
        &selection.outcome
    else {
        panic!("expected a structural decline, got {:?}", selection.outcome);
    };
    assert!(!rejected.is_empty());
    assert!(rejected
        .iter()
        .all(|r| r.primary == StructuralCriterion::BedAdhesion));
}

#[test]
fn isthmus_fixture_taps_safely_where_a_drag_would_not() {
    let (_model, analysis) = analysis_of("struct_isthmus.gcode", 1);
    assert_eq!(analysis.islands().len(), 1, "the isthmus joins the plates");
    assert_eq!(analysis.islands()[0].bbox.max, [34.0, 14.0]);
    // A tap at the centre of the left plate is safe...
    let tap = verdict_of(&analysis, [7.0, 7.0], &ContactMode::Tap);
    assert_eq!(tap.outcome(), StructuralOutcome::Safe);
    assert!(measured(&tap, StructuralCriterion::EdgeMargin) > 6.9);
    // ... while a 12 mm drag from the same point must cross the isthmus,
    // where the material is only ~2 mm from the centreline to the edge.
    // The run ends where the distance to the re-entrant corner at
    // (14, 9.03) falls to the 3 mm margin, x ~ 11.8, so ~4.8 mm.
    let drag = verdict_of(
        &analysis,
        [7.0, 7.0],
        &ContactMode::Drag {
            direction: [1.0, 0.0],
            run_length: 12.0,
        },
    );
    assert_eq!(
        drag.outcome(),
        StructuralOutcome::Unsafe {
            primary: StructuralCriterion::DragRun
        }
    );
    let run = measured(&drag, StructuralCriterion::DragRun);
    assert!((4.4..=5.2).contains(&run), "clear run {run}");
    // Nothing but the run changed between the two modes.
    for criterion in [
        StructuralCriterion::BedAdhesion,
        StructuralCriterion::Tipping,
        StructuralCriterion::FeatureWidth,
        StructuralCriterion::EdgeMargin,
    ] {
        assert!(drag.check(criterion).expect("criterion").passed);
        assert_eq!(measured(&drag, criterion), measured(&tap, criterion));
    }
    // A drag short enough to stay clear of the isthmus is accepted.
    let short = verdict_of(
        &analysis,
        [7.0, 7.0],
        &ContactMode::Drag {
            direction: [1.0, 0.0],
            run_length: 4.0,
        },
    );
    assert_eq!(short.outcome(), StructuralOutcome::Safe);
}

#[test]
fn two_island_fixture_never_drags_across_the_gap() {
    let (_model, analysis) = analysis_of("struct_two_islands.gcode", 1);
    assert_eq!(analysis.islands().len(), 2, "a 4 mm gap splits the plates");
    assert_eq!(analysis.island_at([7.0, 7.0]), Some(0));
    assert_eq!(analysis.island_at([25.0, 7.0]), Some(1));
    assert_eq!(analysis.island_at([16.0, 7.0]), None, "the gap is empty");
    assert_eq!(analysis.islands()[0].bbox.max, [14.0, 14.0]);
    assert_eq!(analysis.islands()[1].bbox.min, [18.0, 0.0]);
    // Each plate is independently anchored to its own bed island.
    let first = analysis.footprint(0).expect("trace 0");
    let second = analysis.footprint(1).expect("trace 1");
    assert_eq!(first.bed_island_indices, vec![0]);
    assert_eq!(second.bed_island_indices, vec![1]);
    assert_ne!(first.bed_island_indices, second.bed_island_indices);
    // A 20 mm drag from the first plate would reach into the second, so
    // it must fail; the measured run stops inside the first plate, well
    // short of the 14 mm gap-crossing distance.
    let drag = verdict_of(
        &analysis,
        [7.0, 7.0],
        &ContactMode::Drag {
            direction: [1.0, 0.0],
            run_length: 20.0,
        },
    );
    assert_eq!(
        drag.outcome(),
        StructuralOutcome::Unsafe {
            primary: StructuralCriterion::DragRun
        }
    );
    let run = measured(&drag, StructuralCriterion::DragRun);
    assert!(run < 7.0, "run {run} left the first plate");
    // Measured directly, the run never reaches the second plate either.
    let reach = analysis.clear_run(0, [7.0, 7.0], [1.0, 0.0], 30.0);
    assert!(7.0 + reach < 18.0, "run reached the second island");
}

#[test]
fn wide_top_small_base_fixture_needs_the_footprint_trace() {
    // Layer 2 is the 20 x 20 flange; layers 0-1 are the 6 x 6 base.
    let (model, analysis) = analysis_of("struct_wide_top_small_base.gcode", 2);
    assert_eq!(model.layers.len(), 4);
    let island = &analysis.islands()[0];
    assert_eq!(island.bbox.max, [20.0, 20.0]);
    // Judged on its own layer the flange looks generous...
    assert!(
        island.area > ContactConfig::default().min_bed_contact_area,
        "layer-2 island area {}",
        island.area
    );
    // ... but what holds it down is the 6 x 6 base.
    let trace = analysis.footprint(0).expect("trace");
    assert_eq!(trace.status, TraceStatus::Anchored);
    assert_eq!(analysis.bed_islands().len(), 1);
    assert_eq!(analysis.bed_islands()[0].bbox.min, [7.0, 7.0]);
    assert_eq!(analysis.bed_islands()[0].bbox.max, [13.0, 13.0]);
    assert!(trace.bed_area < 50.0, "bed area {}", trace.bed_area);
    assert_eq!(
        trace.effective_area,
        trace.bed_area.min(trace.weakest_link_area)
    );
    let verdict = verdict_of(&analysis, [10.0, 10.0], &ContactMode::Tap);
    assert_eq!(
        verdict.outcome(),
        StructuralOutcome::Unsafe {
            primary: StructuralCriterion::BedAdhesion
        }
    );
    assert!(
        verdict
            .check(StructuralCriterion::EdgeMargin)
            .unwrap()
            .passed
    );
    assert!(
        verdict
            .check(StructuralCriterion::FeatureWidth)
            .unwrap()
            .passed
    );
}

#[test]
fn arbitrary_operator_points_are_validated_live() {
    // The manual-jog path: the same verdict for a hand-picked XY, both
    // through the one-shot helper and a reused analysis.
    let data = load("struct_two_islands.gcode");
    let model = model_of(&data);
    let config = ContactConfig::default();
    let one_shot = assess_contact_point(&model, 1, [7.0, 7.0], &ContactMode::Tap, &config)
        .expect("assessment");
    let analysis = StructuralAnalysis::build(&model, 1, &config).expect("analysis");
    assert_eq!(one_shot, analysis.assess([7.0, 7.0], &ContactMode::Tap));
    let StructuralAssessment::Evaluated(verdict) = one_shot else {
        panic!("expected a verdict");
    };
    assert_eq!(verdict.outcome(), StructuralOutcome::Safe);
    // A point in the gap tells the operator how far to jog and to what.
    let StructuralAssessment::OffMaterial {
        distance,
        nearest_point,
        nearest_island,
    } = analysis.assess([16.0, 7.0], &ContactMode::Tap)
    else {
        panic!("expected OffMaterial in the gap");
    };
    assert!((1.5..2.5).contains(&distance), "distance {distance}");
    let nearest = nearest_point.expect("a nearest point");
    assert!(nearest[0] <= 14.0 + 1e-9 || nearest[0] >= 18.0 - 1e-9);
    assert!(nearest_island.is_some());
    // Non-finite input is refused, never guessed at.
    assert_eq!(
        analysis.assess([f64::NAN, 7.0], &ContactMode::Tap),
        StructuralAssessment::InvalidPoint {
            param: plr_analyzer::structure::InvalidInput::Point
        }
    );
    // Out-of-range layers are a typed error, not a panic.
    assert!(assess_contact_point(&model, 99, [7.0, 7.0], &ContactMode::Tap, &config).is_err());
}

#[test]
fn structural_fixtures_survive_every_resume_layer_in_both_modes() {
    // Totality over the corpus: no panic, and every decline is one of
    // the typed reasons, at every layer, tapping and dragging.
    let drag = ContactConfig {
        contact_mode: ContactMode::Drag {
            direction: [1.0, 0.0],
            run_length: 8.0,
        },
        ..ContactConfig::default()
    };
    for name in [
        "struct_flat_plate.gcode",
        "struct_isthmus.gcode",
        "struct_tall_pillar.gcode",
        "struct_two_islands.gcode",
        "struct_wide_top_small_base.gcode",
    ] {
        let model = model_of(&load(name));
        for layer in 0..=u32::try_from(model.layers.len()).unwrap() {
            for config in [&ContactConfig::default(), &drag] {
                let _ = select_contact_zone_detailed(&model, layer, [0.0, 0.0], config);
            }
        }
    }
}
