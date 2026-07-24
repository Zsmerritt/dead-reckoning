//! Property tests: builder totality on arbitrary bytes, matcher
//! totality on arbitrary (including non-finite) evidence, and selector
//! geometric invariants on generated geometry.

#![allow(clippy::float_cmp)] // exact recomputation equality is intentional

use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

use plr_analyzer::{
    build_layer_model, match_stop_point, select_contact_zone, select_contact_zone_detailed,
    ByteWindow, ContactConfig, ContactMode, ContactOutcome, FeatureClass, Interval, Layer,
    LayerModel, MatchConfidence, MatchConfig, MatchError, ModelConfig, ModelStop, StopEvidence,
    StructuralAnalysis, StructuralAssessment as ContactAssessment, StructuralCriterion,
    StructuralOutcome, TypedPath, XySegment,
};
use plr_gcode::{ByteSpan, GcodeState};

/// A small but feature-rich file for matcher totality runs: two
/// layers, retraces, a hop, an arc.
const MATCH_CORPUS: &str = "G90\nM83\nG92 E0\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
    G1 X20 Y20 F9000\nG1 X40 Y20 E0.5 F3000\nG1 X20 Y20 F9000\nG1 X40 Y20 E0.5 F3000\n\
    G1 E-0.8 F2100\nG1 Z0.6 F7200\nG1 X10 Y0 F9000\nG1 Z0.2 F7200\nG1 E0.8 F2100\n\
    G3 X0 Y10 I-10 E2 F1800\nG1 Z0.4 F7200\nG1 X20 Y10 E1 F3000\n";

fn f64_from_bits(bits: u64) -> f64 {
    f64::from_bits(bits)
}

/// Distance from a point to a segment, reference implementation for
/// the invariant checks.
fn point_seg_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let proj = [a[0] + dx * t, a[1] + dy * t];
    ((p[0] - proj[0]).powi(2) + (p[1] - proj[1]).powi(2)).sqrt()
}

/// Build a two-layer model directly from generated segments (fields
/// are public precisely so property tests can construct adversarial
/// geometry the g-code path would never emit).
fn synth_model(
    prev_segments: Vec<XySegment>,
    cover_segments: Vec<XySegment>,
    class: FeatureClass,
) -> LayerModel {
    let prev_count = u32::try_from(prev_segments.len()).unwrap_or(u32::MAX);
    let cover_count = u32::try_from(cover_segments.len()).unwrap_or(u32::MAX);
    LayerModel {
        layers: vec![
            Layer {
                index: 0,
                z: 0.2,
                z_known: true,
                span: ByteSpan { start: 0, end: 100 },
                annotation_z: None,
                paths: vec![TypedPath {
                    class,
                    type_name: Some("Sparse infill".to_string()),
                    segments: prev_segments,
                }],
                extrusion_moves: prev_count.max(1),
                spiral_moves: 0,
            },
            Layer {
                index: 1,
                z: 0.4,
                z_known: true,
                span: ByteSpan {
                    start: 100,
                    end: 200,
                },
                annotation_z: None,
                paths: vec![TypedPath {
                    class: FeatureClass::InternalInfill,
                    type_name: Some("Sparse infill".to_string()),
                    segments: cover_segments,
                }],
                extrusion_moves: cover_count.max(1),
                spiral_moves: 0,
            },
        ],
        moves: Vec::new(),
        annotated: true,
        stop: ModelStop::EndOfInput,
        lines_consumed: 0,
    }
}

fn segment_strategy() -> impl Strategy<Value = XySegment> {
    (
        0.0f64..100.0,
        0.0f64..100.0,
        0.0f64..100.0,
        0.0f64..100.0,
        0u64..10_000,
    )
        .prop_map(|(x0, y0, x1, y1, off)| XySegment {
            start: [x0, y0],
            end: [x1, y1],
            z: 0.2,
            e_start: 0.0,
            e_end: 1.0,
            span: ByteSpan {
                start: off,
                end: off + 1,
            },
            arc: None,
        })
}

proptest! {
    // Persist shrunk counterexamples to the checked-in file
    // tests/properties.proptest-regressions. The default
    // (SourceParallel) cannot locate lib.rs/main.rs from an
    // integration test and only works via a warning-emitting fallback;
    // WithSource pins the exact same path explicitly so regressions
    // are reliably replayed on every run.
    #![proptest_config(ProptestConfig {
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::default()
    })]

    /// The layer-model builder is total on arbitrary bytes and always
    /// produces internally consistent output.
    #[test]
    fn builder_total_on_arbitrary_bytes(
        data in proptest::collection::vec(any::<u8>(), 0..2048),
        base in any::<u32>(),
    ) {
        let base = u64::from(base);
        let model = build_layer_model(
            GcodeState::new(),
            &data,
            base,
            &ModelConfig::default(),
        );
        let end = base + data.len() as u64;
        for (i, layer) in model.layers.iter().enumerate() {
            prop_assert_eq!(layer.index as usize, i);
            prop_assert!(layer.extrusion_moves >= 1);
            prop_assert!(layer.span.start >= base && layer.span.end <= end);
            prop_assert!(!layer.paths.is_empty());
        }
        for mv in &model.moves {
            prop_assert!(mv.span.start >= base && mv.span.end <= end);
            if let Some(layer) = mv.layer {
                prop_assert!((layer as usize) < model.layers.len());
            }
        }
        // Spiral fraction is always a valid fraction.
        let f = model.spiral_fraction();
        prop_assert!((0.0..=1.0).contains(&f));
        // The resume-selection helper never panics and only returns
        // depositing moves at line boundaries.
        if let Some(dep) = model.first_deposition_at_or_after(base) {
            prop_assert!(dep.span.start >= base);
        }
    }

    /// The matcher is total on arbitrary evidence bit patterns:
    /// non-finite or inverted evidence is a typed error, and every Ok
    /// result reports line-boundary offsets consistent with its
    /// confidence.
    #[test]
    fn matcher_total_on_arbitrary_evidence(
        xa in any::<u64>(), xb in any::<u64>(),
        ya in any::<u64>(), yb in any::<u64>(),
        ea in any::<u64>(), eb in any::<u64>(),
        has_e in any::<bool>(),
        zs in proptest::collection::vec(any::<u64>(), 0..4),
        wstart in any::<u64>(),
        wend in proptest::option::of(any::<u64>()),
    ) {
        let model = build_layer_model(
            GcodeState::new(),
            MATCH_CORPUS.as_bytes(),
            0,
            &ModelConfig::default(),
        );
        let evidence = StopEvidence {
            x: Interval { min: f64_from_bits(xa), max: f64_from_bits(xb) },
            y: Interval { min: f64_from_bits(ya), max: f64_from_bits(yb) },
            e: has_e.then(|| Interval {
                min: f64_from_bits(ea),
                max: f64_from_bits(eb),
            }),
            z_candidates: zs.iter().copied().map(f64_from_bits).collect(),
            window: ByteWindow { start: wstart, end: wend },
        };
        // Must never panic; must be a typed error when any field is
        // non-finite.
        let result = match_stop_point(&model, &evidence, &MatchConfig::default());
        let finite = evidence.x.is_valid()
            && evidence.y.is_valid()
            && evidence.e.as_ref().is_none_or(Interval::is_valid)
            && evidence.z_candidates.iter().all(|z| z.is_finite())
            && wend.is_none_or(|end| end >= wstart);
        match result {
            Err(MatchError::InvalidEvidence { .. }) => prop_assert!(!finite),
            Err(MatchError::InvalidConfig { .. }) => prop_assert!(false, "default config valid"),
            Err(MatchError::NoMatch | MatchError::Inconclusive { .. }) => prop_assert!(finite),
            Ok(r) => {
                prop_assert!(finite);
                let line_starts: Vec<u64> =
                    model.moves.iter().map(|m| m.span.start).collect();
                for c in &r.candidates {
                    prop_assert!(line_starts.contains(&c.offset), "not a line boundary");
                    prop_assert!((0.0..=1.0).contains(&c.e_agreement));
                }
                match &r.confidence {
                    MatchConfidence::UniqueLine { offset } => {
                        prop_assert_eq!(r.candidates.len(), 1);
                        prop_assert_eq!(*offset, r.candidates[0].offset);
                    }
                    MatchConfidence::AmbiguousWindow { offsets } => {
                        prop_assert_eq!(offsets.len(), r.candidates.len());
                        prop_assert!(offsets.windows(2).all(|w| w[0] < w[1]));
                    }
                    MatchConfidence::LayerOnly { layer } => {
                        prop_assert!((*layer as usize) < model.layers.len());
                    }
                }
            }
        }
    }

    /// Selector invariants on generated geometry: every candidate lies
    /// exactly on an N-1 host segment, strictly outside the exclusion
    /// radius, and within coverage tolerance of a layer-N segment.
    #[test]
    fn selector_candidates_respect_geometry(
        mut prev in proptest::collection::vec(segment_strategy(), 1..6),
        cover in proptest::collection::vec(segment_strategy(), 0..6),
        crash_x in 0.0f64..100.0,
        crash_y in 0.0f64..100.0,
        radius in 0.0f64..30.0,
    ) {
        // Force UNIQUE spans on the host layer: this test identifies a
        // candidate's host segment by span, and the random strategy can
        // collide two segments on the same offset (found by proptest:
        // a degenerate and a real segment shared a span, so the lookup
        // resolved to the wrong host). Real models cannot collide —
        // each segment comes from a distinct file line.
        for (index, segment) in prev.iter_mut().enumerate() {
            let start = 20_000 + index as u64;
            segment.span = ByteSpan { start, end: start + 1 };
        }
        let model = synth_model(prev.clone(), cover.clone(), FeatureClass::InternalInfill);
        // This property is about the *geometric* selector invariants
        // (on-segment, exclusion radius, coverage). The structural stage
        // would filter most of this random geometry out before those
        // invariants could be observed, and has its own properties
        // below, so it is switched off here.
        let config = ContactConfig {
            exclusion_radius: radius,
            structural_checks_enabled: false,
            ..ContactConfig::default()
        };
        let out = select_contact_zone(&model, 1, [crash_x, crash_y], &config);
        let out = out.expect("valid inputs never error");
        if let ContactOutcome::Candidates(cands) = out {
            prop_assert!(!cands.is_empty());
            prop_assert!(cands.len() <= config.max_candidates);
            for c in &cands {
                // Strictly outside the exclusion radius.
                let d_crash = ((c.point[0] - crash_x).powi(2)
                    + (c.point[1] - crash_y).powi(2))
                .sqrt();
                prop_assert!(d_crash > radius, "candidate inside exclusion radius");
                // Not exact-eq: the selector computes the distance via
                // geom::point_distance, whose operation order may
                // differ from the recomputation above by an ulp
                // (found by proptest).
                prop_assert!(
                    (c.distance_from_crash - d_crash).abs() <= 1e-9,
                    "distance_from_crash {} != recomputed {}",
                    c.distance_from_crash,
                    d_crash
                );
                // Exactly on the host segment (identified by span).
                let host = prev
                    .iter()
                    .find(|s| s.span == c.host_span)
                    .expect("host segment exists");
                prop_assert!(
                    point_seg_dist(c.point, host.start, host.end) <= 1e-9,
                    "candidate off its host segment"
                );
                // The host is long enough.
                prop_assert!(c.host_length >= config.min_segment_length);
                // Covered by layer N within tolerance (allow float slop).
                let covered = cover.iter().any(|s| {
                    point_seg_dist(c.point, s.start, s.end)
                        <= config.coverage_tolerance + 1e-9
                });
                prop_assert!(covered, "candidate not covered by layer N");
                // Sampled strictly inside the segment, never at an end.
                prop_assert!(c.sample_t > 0.0 && c.sample_t < 1.0);
            }
        }
    }

    /// The selector never panics even on degenerate/NaN segments
    /// injected directly into a hand-built model.
    #[test]
    fn selector_total_on_adversarial_segments(
        bits in proptest::collection::vec(any::<u64>(), 4),
        crash_x in 0.0f64..100.0,
    ) {
        let seg = XySegment {
            start: [f64_from_bits(bits[0]), f64_from_bits(bits[1])],
            end: [f64_from_bits(bits[2]), f64_from_bits(bits[3])],
            z: 0.2,
            e_start: 0.0,
            e_end: 1.0,
            span: ByteSpan { start: 0, end: 1 },
            arc: None,
        };
        let model = synth_model(vec![seg.clone()], vec![seg], FeatureClass::InternalInfill);
        let out = select_contact_zone(
            &model,
            1,
            [crash_x, 50.0],
            &ContactConfig::default(),
        );
        // Non-finite host geometry is filtered, never propagated.
        if let Ok(ContactOutcome::Candidates(cands)) = out {
            for c in &cands {
                prop_assert!(c.point[0].is_finite() && c.point[1].is_finite());
            }
        }
    }
}

/// Scan lines filling a square centred on `centre`, at the 0.4 mm
/// production line spacing so the patch rasterizes as solid material.
fn square_segments(centre: [f64; 2], side: f64, z: f64, span_base: u64) -> Vec<XySegment> {
    let (lo, hi) = (centre[0] - side / 2.0, centre[0] + side / 2.0);
    let mut out = Vec::new();
    for row in 0_u32.. {
        let y = f64::from(row).mul_add(0.4, centre[1] - side / 2.0);
        if y > centre[1] + side / 2.0 + 1e-9 {
            break;
        }
        let offset = span_base + u64::from(row);
        out.push(XySegment {
            start: [lo, y],
            end: [hi, y],
            z,
            e_start: 0.0,
            e_end: 1.0,
            span: ByteSpan {
                start: offset,
                end: offset + 1,
            },
            arc: None,
        });
    }
    out
}

/// Whether the bed-adhesion criterion passed at `point`, for a model
/// whose layer 0 is a square of `base_side` carrying a fixed 2 mm plate.
fn adhesion_passes(base_side: f64) -> (bool, f64) {
    let model = synth_model(
        square_segments([50.0, 50.0], base_side, 0.2, 1_000),
        square_segments([50.0, 50.0], 2.0, 0.4, 20_000),
        FeatureClass::InternalInfill,
    );
    let analysis = StructuralAnalysis::build(&model, 1, &ContactConfig::default())
        .expect("valid config and layer");
    match analysis.assess([50.0, 50.0], &ContactMode::Tap) {
        ContactAssessment::Evaluated(verdict) => {
            let check = verdict
                .check(StructuralCriterion::BedAdhesion)
                .expect("adhesion check");
            (check.passed, check.measured)
        }
        other => panic!("expected a verdict, got {other:?}"),
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::default()
    })]

    /// Structural verdicts are total: arbitrary (including non-finite)
    /// geometry, points and drag parameters never panic, and every
    /// number a verdict reports is finite — the decline payload is
    /// serialized as JSON downstream, which cannot encode an infinity.
    #[test]
    fn structural_verdicts_total_on_arbitrary_geometry(
        bits in proptest::collection::vec(any::<u64>(), 8),
        px in any::<u64>(),
        py in any::<u64>(),
        dx in any::<u64>(),
        dy in any::<u64>(),
        run in any::<u64>(),
        drag in any::<bool>(),
    ) {
        let mut prev = square_segments([50.0, 50.0], 8.0, 0.2, 1_000);
        let mut cover = square_segments([50.0, 50.0], 8.0, 0.4, 20_000);
        // Splice adversarial coordinates into both layers.
        prev[0].start = [f64_from_bits(bits[0]), f64_from_bits(bits[1])];
        prev[0].end = [f64_from_bits(bits[2]), f64_from_bits(bits[3])];
        cover[0].start = [f64_from_bits(bits[4]), f64_from_bits(bits[5])];
        cover[0].end = [f64_from_bits(bits[6]), f64_from_bits(bits[7])];
        let model = synth_model(prev, cover, FeatureClass::InternalInfill);
        let analysis = StructuralAnalysis::build(&model, 1, &ContactConfig::default())
            .expect("the default config is valid");
        let mode = if drag {
            ContactMode::Drag {
                direction: [f64_from_bits(dx), f64_from_bits(dy)],
                run_length: f64_from_bits(run),
            }
        } else {
            ContactMode::Tap
        };
        let point = [f64_from_bits(px), f64_from_bits(py)];
        match analysis.assess(point, &mode) {
            ContactAssessment::Evaluated(verdict) => {
                prop_assert!(verdict.score.is_finite());
                prop_assert!((0.0..=1.0).contains(&verdict.score));
                prop_assert!(!verdict.summary.is_empty());
                prop_assert!(verdict.footprint.effective_area.is_finite());
                prop_assert!(verdict.footprint.bed_area.is_finite());
                prop_assert!(verdict.footprint.weakest_link_area.is_finite());
                prop_assert!(verdict.island.area.is_finite());
                for check in &verdict.checks {
                    prop_assert!(check.measured.is_finite(), "{:?}", check);
                    prop_assert!(check.threshold.is_finite(), "{:?}", check);
                    prop_assert!(!check.reason.is_empty());
                }
                if let Some(alignment) = verdict.centroid_alignment {
                    prop_assert!(alignment.is_finite());
                }
                // A safe verdict has no failing check, and vice versa.
                match verdict.outcome() {
                    StructuralOutcome::Safe => prop_assert!(verdict.failures().is_empty()),
                    StructuralOutcome::Unsafe { primary } => {
                        prop_assert!(!verdict.failures().is_empty());
                        prop_assert_eq!(verdict.failures()[0].criterion, primary);
                    }
                }
            }
            ContactAssessment::OffMaterial { distance, .. } => {
                prop_assert!(distance.is_finite());
            }
            ContactAssessment::InvalidPoint { .. } => {
                let bad_point = !point[0].is_finite() || !point[1].is_finite();
                let dragging = matches!(&mode, ContactMode::Drag { .. });
                prop_assert!(bad_point || dragging);
            }
        }
        // The selector never panics on the same model either.
        let _ = select_contact_zone_detailed(&model, 1, [0.0, 0.0], &ContactConfig::default());
    }
}

proptest! {
    // Each case builds two full analyses with their rasters, so the
    // case count is lowered deliberately; the property is structural,
    // not statistical.
    #![proptest_config(ProptestConfig {
        cases: 24,
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::default()
    })]

    /// Monotonicity: shrinking the footprint that holds a feature can
    /// never turn a structural failure into a pass. Concretely, if the
    /// smaller base clears the bed-adhesion bar then the larger one
    /// must too, and it must measure at least as much area.
    #[test]
    fn shrinking_a_footprint_never_turns_a_fail_into_a_pass(
        base in 8.0f64..14.0,
        factor in 0.2f64..0.5,
    ) {
        let small = base * factor;
        let (small_passed, small_area) = adhesion_passes(small);
        let (big_passed, big_area) = adhesion_passes(base);
        prop_assert!(
            big_area >= small_area,
            "shrinking {base} -> {small} grew the measured area: {big_area} < {small_area}"
        );
        prop_assert!(
            !small_passed || big_passed,
            "the smaller {small} mm base passed where the {base} mm one failed"
        );
    }
}
