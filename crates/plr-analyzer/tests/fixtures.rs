//! Integration tests over the committed fixture corpus
//! (`fixtures/synthetic/`) and any real sliced files dropped into
//! `fixtures/real/`.

#![allow(clippy::float_cmp)]
// exact replay equality is intentional
// The density measurements below average small counts (candidates per sampled
// stop point, tens at most) into f64 for reporting. `usize -> f64` cannot lose
// precision until 2^53, so the lint has nothing to catch here; the alternative
// is `u32` casts that would need their own justification.
#![allow(clippy::cast_precision_loss)]

use std::fs;
use std::path::{Path, PathBuf};

use plr_analyzer::{
    assess_contact_point, build_layer_model, match_stop_point, remaining_work, select_contact_zone,
    select_contact_zone_detailed, ByteWindow, ContactConfig, ContactError, ContactMode,
    ContactOutcome, DeclineReason, FeatureClass, Interval, LayerModel, MatchConfidence,
    MatchConfig, ModelConfig, MoveKind, RemainingWork, SimMove, StopEvidence, StructuralAnalysis,
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

/// # This test used to pass vacuously too
///
/// `fixtures/real/` held only a README, so this loop body never executed —
/// the same hole as `plr-gcode`'s `real_corpus_full_pipeline`, in a second
/// place, found only because the first one was being fixed. The corpus
/// assertion is the fix: an empty corpus now fails loudly rather than
/// reporting `ok` while testing nothing.
#[test]
fn real_corpus_builds_sane_models() {
    let files = gcode_files(&fixtures_dir().join("real"));
    assert!(
        !files.is_empty(),
        "fixtures/real/ has no *.gcode: this test would pass vacuously. \
         Regenerate with `python3 fixtures/real/realistic_generator.py`."
    );
    for path in files {
        let data = fs::read(&path).expect("read fixture");
        let model = model_of(&data);
        check_model_invariants(&model, data.len() as u64);
        for n in 0..=u32::try_from(model.layers.len().min(30)).unwrap() {
            let _ = select_contact_zone(&model, n, [0.0, 0.0], &ContactConfig::default());
        }
    }
}

// --- stop-point discrimination at realistic extrusion density -------------
//
// `plrd`'s end-to-end fixture deliberately puts 0.65 mm of E on single lines
// so that stop-point matching is unambiguous, and it sits at exactly 8 of 8
// candidates against `ambiguity_limit`. Both properties make it useless for
// asking how well the matcher actually discriminates on production files,
// where per-line E is 0.028–0.078 mm (measured; see
// `fixtures/real/README.md`) — more than ten times smaller.
//
// These tests ask that question against the realistic corpus.

fn realistic() -> Vec<u8> {
    let path = fixtures_dir().join("real").join("realistic_orca.gcode");
    fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

/// The move at `index` in the model, and evidence naming it exactly: a tight
/// XY box around its endpoint, its Z, and its own E range.
///
/// This is the *best case* for the matcher — evidence as precise as any
/// reconstruction could produce. Whatever ambiguity remains is intrinsic to
/// the file, not a product of loose bounds.
fn evidence_for(model: &LayerModel, index: usize, e_pad: f64) -> (StopEvidence, &SimMove) {
    evidence_for_xy(model, index, e_pad, 0.05)
}

/// [`evidence_for`] with an explicit XY half-width.
///
/// The tight default (0.05 mm) is the best case and deliberately gives XY the
/// strongest possible hand, which *understates* what the E constraint
/// contributes. A real reconstruction reports XY as a bounding region over the
/// whole evaluated window, which can be tens of millimetres — so E's real
/// contribution has to be measured against a loose box too.
fn evidence_for_xy(
    model: &LayerModel,
    index: usize,
    e_pad: f64,
    xy_pad: f64,
) -> (StopEvidence, &SimMove) {
    evidence_full(model, index, e_pad, xy_pad, None)
}

/// Realistic byte-window half-width, bytes.
///
/// **This is the dominant filter, and omitting it makes every other number
/// meaningless.** The real pipeline never matches against a whole file: the
/// window is `[frontier at t_a - max_processing_lead, extension resume
/// offset]`. At the frontier rates measured on real slicer output (~1400 B/s
/// median) a 3 s lead is ~4 KB behind the stop, and the 2 s extension horizon
/// ~3 KB ahead. Measured on `plrd`'s own e2e fixture the window was
/// `[8, 2246]` of a 4262-byte file.
///
/// A `whole_file()` window on a 348 KB corpus admits two orders of magnitude
/// more moves than production ever would, which inflates candidate counts and
/// makes the E constraint look far more load-bearing than it is.
const WINDOW_HALF: u64 = 4_096;

/// [`evidence_for_xy`] with an explicit byte window (`None` = realistic).
fn evidence_full(
    model: &LayerModel,
    index: usize,
    e_pad: f64,
    xy_pad: f64,
    window: Option<ByteWindow>,
) -> (StopEvidence, &SimMove) {
    let mv = &model.moves[index];
    let (ex, ey) = (mv.end[0], mv.end[1]);
    let e_lo = mv.start[3].min(mv.end[3]);
    let e_hi = mv.start[3].max(mv.end[3]);
    (
        StopEvidence {
            x: iv(ex - xy_pad, ex + xy_pad),
            y: iv(ey - xy_pad, ey + xy_pad),
            e: Some(iv(e_lo - e_pad, e_hi + e_pad)),
            z_candidates: vec![mv.end[2]],
            window: window.unwrap_or(ByteWindow {
                start: mv.span.start.saturating_sub(WINDOW_HALF),
                end: Some(mv.span.end + WINDOW_HALF),
            }),
        },
        mv,
    )
}

/// Sample move indices spread across the model, skipping the first layer
/// (which has no predecessor geometry) and any non-depositing move.
fn sample_indices(model: &LayerModel) -> Vec<usize> {
    let n = model.moves.len();
    (0..24)
        .map(|k| n / 6 + k * (n / 40).max(1))
        .filter(|i| *i < n && model.moves[*i].kind == MoveKind::Extrusion)
        .collect()
}

/// **How much work is the E constraint actually doing at realistic density?**
///
/// `MatchConfig::e_tolerance` is a fixed 1.0 mm. At the ~0.05 mm per line a
/// real slicer emits that spans roughly twenty lines, so the E filter should
/// be nearly non-discriminating in production — which would mean per-line
/// matching is carried almost entirely by XY, and that the E evidence this
/// project spends considerable effort sharpening contributes far less than
/// assumed.
///
/// This measures it directly: the same sampled stop points matched with E
/// evidence present and with `e: None`, and the candidate counts compared.
/// If the counts are equal, E changed nothing.
#[test]
fn e_constraint_contribution_at_realistic_density() {
    let data = realistic();
    let model = model_of(&data);
    let config = MatchConfig::default();
    let mut samples = 0_usize;
    let mut with_e = 0_usize;
    let mut without_e = 0_usize;
    let mut identical = 0_usize;
    // Swept across XY looseness, because a tight XY box hides E's value.
    for xy_pad in [0.05_f64, 5.0, 15.0, 30.0] {
        let (mut w, mut wo, mut same, mut n) = (0_usize, 0_usize, 0_usize, 0_usize);
        for i in sample_indices(&model) {
            let (ev, _) = evidence_for_xy(&model, i, 0.0, xy_pad);
            let mut ev_none = ev.clone();
            ev_none.e = None;
            let (Ok(a), Ok(b)) = (
                match_stop_point(&model, &ev, &config),
                match_stop_point(&model, &ev_none, &config),
            ) else {
                continue;
            };
            n += 1;
            w += a.candidates.len();
            wo += b.candidates.len();
            if a.candidates.len() == b.candidates.len() {
                same += 1;
            }
        }
        if n == 0 {
            continue;
        }
        eprintln!(
            "E-CONTRIBUTION xy_pad={xy_pad:>5} n={n:>3} with_E={w:>5} without_E={wo:>5}  E_removes={:>5.1}%  unchanged_in={same}/{n}",
            100.0 * (wo - w) as f64 / wo.max(1) as f64
        );
        if (xy_pad - 0.05).abs() < 1e-9 {
            (samples, with_e, without_e, identical) = (n, w, wo, same);
        }
    }
    assert!(samples >= 8, "too few usable samples ({samples})");
    // Recorded as a measurement, not a threshold: the point is the number,
    // and the assertion only pins the direction (E can never *add*
    // candidates, since it is a filter).
    assert!(
        with_e <= without_e,
        "E evidence added candidates ({with_e} vs {without_e}) — impossible \
         for a filter; the harness is wrong"
    );
    eprintln!(
        "E-CONTRIBUTION samples={samples} candidates_with_E={with_e} \
         candidates_without_E={without_e} unchanged_in={identical}/{samples} \
         e_tolerance={}",
        config.e_tolerance
    );
}

/// **Is `e_tolerance = 1.0 mm` mis-tuned for real files?**
///
/// The companion question to
/// [`e_constraint_contribution_at_realistic_density`]. If a tighter tolerance
/// sharply reduces candidate counts, the constant is leaving discrimination on
/// the table and should arguably be derived from local per-line E density. If
/// it barely moves them, 1.0 mm is fine and XY genuinely carries per-line
/// matching.
///
/// **This measures granularity only, and tightening is not free.**
/// `e_tolerance` absorbs every error in the E evidence — frame reconstruction,
/// the un-evidenced coverage band, pressure advance, the one-line sampling
/// skew. A value below the true error excludes the actual stop line, which is
/// a containment failure and strictly worse than ambiguity. So a low candidate
/// count here is a reason to investigate, never on its own a reason to lower
/// the constant.
#[test]
fn e_tolerance_sweep_at_realistic_density() {
    let data = realistic();
    let model = model_of(&data);
    let tolerances = [1.0_f64, 0.5, 0.2, 0.1, 0.05, 0.01];
    let mut totals = vec![0_usize; tolerances.len()];
    let mut misses = vec![0_usize; tolerances.len()];
    let mut samples = 0_usize;
    for i in sample_indices(&model) {
        let (ev, mv) = evidence_for(&model, i, 0.0);
        let truth = mv.span.start;
        let mut row = Vec::with_capacity(tolerances.len());
        for t in tolerances {
            let config = MatchConfig {
                e_tolerance: t,
                ..MatchConfig::default()
            };
            match match_stop_point(&model, &ev, &config) {
                // Does the candidate set still CONTAIN the true line?
                Ok(r) => row.push((
                    r.candidates.len(),
                    r.candidates.iter().any(|c| c.offset == truth),
                )),
                Err(_) => row.push((usize::MAX, false)),
            }
        }
        if row.iter().any(|(n, _)| *n == usize::MAX) {
            continue;
        }
        samples += 1;
        for (k, (n, hit)) in row.iter().enumerate() {
            totals[k] += n;
            if !hit {
                misses[k] += 1;
            }
        }
    }
    assert!(samples >= 8, "too few usable samples ({samples})");
    let mean: Vec<String> = totals
        .iter()
        .map(|t| format!("{:.2}", *t as f64 / samples as f64))
        .collect();
    eprintln!(
        "E-TOLERANCE samples={samples} tolerances={tolerances:?}\n  \
         mean_candidates=[{}]\n  containment_misses={misses:?}",
        mean.join(", ")
    );
    // At the shipped tolerance the true line must always survive. This is the
    // containment property; the sweep exists to show what tightening costs it.
    assert_eq!(
        misses[0], 0,
        "the true stop line was excluded at the shipped e_tolerance of {} — \
         a containment failure independent of any widening question",
        tolerances[0]
    );
}

/// The Q1 ladder at realistic density: what does widening the E interval
/// actually cost when a line carries 0.05 mm rather than 0.65 mm?
///
/// `plrd`'s fixture went 8 → 9 → 10 → 12 candidates for progressively wider
/// `e_internal`, tipping past `ambiguity_limit` at the first step. The
/// prediction under test is that at realistic density the same widenings cost
/// little or nothing, because `e_tolerance` already dominates them.
#[test]
fn e_widening_cost_at_realistic_density() {
    let data = realistic();
    let model = model_of(&data);
    let config = MatchConfig::default();
    // Widenings in mm, spanning the band widths measured on real files: a
    // 0.65 s coverage lag at 0.05 mm/line over 22-147 lines is 1.1-7.4 mm.
    let pads = [0.0_f64, 0.5, 1.0, 2.0, 4.0, 8.0];
    // Swept across XY looseness, because E's contribution scales with it:
    // measured, E removes 27% of candidates against a 0.05 mm XY box but 73%
    // against a 5 mm one. A tight box makes the band look free when it is not.
    for xy_pad in [0.05_f64, 5.0, 15.0, 30.0] {
        let mut totals = vec![0_usize; pads.len()];
        let mut maxima = vec![0_usize; pads.len()];
        let mut inconclusive = vec![0_usize; pads.len()];
        let mut samples = 0_usize;
        for i in sample_indices(&model) {
            samples += 1;
            for (k, pad) in pads.iter().enumerate() {
                let (ev, _) = evidence_for_xy(&model, i, *pad, xy_pad);
                match match_stop_point(&model, &ev, &config) {
                    Ok(r) => {
                        totals[k] += r.candidates.len();
                        maxima[k] = maxima[k].max(r.candidates.len());
                    }
                    // The outcome that becomes a ManualFallback downstream.
                    Err(_) => inconclusive[k] += 1,
                }
            }
        }
        assert!(samples >= 8, "too few usable samples ({samples})");
        let mean: Vec<String> = totals
            .iter()
            .zip(&inconclusive)
            .map(|(t, bad)| {
                let ok = samples - bad;
                if ok == 0 {
                    "-".to_owned()
                } else {
                    format!("{:.2}", *t as f64 / ok as f64)
                }
            })
            .collect();
        eprintln!(
            "E-WIDENING xy_pad={xy_pad:>5} n={samples} pads={pads:?}
  mean=[{}]
  max={maxima:?}
  inconclusive={inconclusive:?}",
            mean.join(", ")
        );
        // Monotone in the pad: a wider interval can only admit more lines, so
        // it can only push samples toward inconclusive, never away.
        for w in inconclusive.windows(2) {
            assert!(w[0] <= w[1], "inconclusive not monotone: {inconclusive:?}");
        }
        // --- pinned measurements, not hopes -------------------------------
        //
        // The prediction going in was that widening E is free at realistic
        // extrusion density, because `e_tolerance` (1.0 mm) already dwarfs a
        // 0.05 mm line. Measured against a TIGHT XY box that looked true. It
        // is false in the regime that actually ships: the daemon reports XY as
        // a bounding region over the evaluated window, measured at 30-60 mm
        // wide on `plrd`'s own e2e fixture, and there E does most of the
        // filtering (63-81% of candidates removed) so widening it costs real
        // refusals.
        //
        // These assertions pin that, in the hazard-pin style used elsewhere in
        // this project: they assert what was measured, and if one fires the
        // finding has changed and the numbers in this comment plus
        // `plr_reconstruct::stopset`'s module docs must be updated rather than
        // the assertion relaxed.
        if xy_pad < 1.0 {
            // Tight XY: E is a minor filter and widening it is genuinely free.
            assert_eq!(
                inconclusive,
                vec![0; pads.len()],
                "tight-XY regime stopped being refusal-free: {inconclusive:?}"
            );
        } else {
            // Loose (production-shaped) XY: widening E to the top of the
            // measured band range DOES turn resolvable stop points into
            // refusals. If this stops being true, the band may have become
            // affordable -- verify and update the docs, do not delete this.
            assert!(
                *inconclusive.last().unwrap() > inconclusive[0],
                "widening E no longer costs refusals at xy_pad={xy_pad}                  ({inconclusive:?}); if that is a real improvement, update                  stopset's \"Durable extruder coverage\" docs, which cite                  this measurement as the reason the band is unaffordable"
            );
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

// ---------------------------------------------------------------------
// The completion gate over footer-bearing fixtures (D12).
//
// `fixtures/synthetic/` used to contain no file with a footer at all,
// which is precisely why nothing could observe that a functionally
// complete print stops well short of EOF.
//
// Two of the four fixtures carry a REAL slicer footer, verbatim except for
// a scrubbed model name (see `fixtures/synthetic/footer_generator.py` for
// the licensing rationale). They are the primary evidence; the fully
// synthetic pair is kept for the excluded-object path, which needs
// EXCLUDE_OBJECT brackets around deposition.
// ---------------------------------------------------------------------

/// The mode flags of `plr_gcode::GcodeState::new()` — Klipper's own
/// defaults (`G90` + `M82`, both absolute). Every golden here replays a
/// whole file from byte 0 with a fresh state, so these are the flags the
/// replay starts in and what `remaining_work`'s extruder-frame trust check
/// compares against.
///
/// The fixtures declare their real modes in their own header, a few bytes
/// in. For the deep tested offsets these goldens mostly use, the header
/// precedes the offset, so the file has settled both axes and the check has
/// nothing to compare. For a tested offset of 0 the declarations land
/// *after* the offset instead, and what keeps the check quiet there is the
/// materiality narrowing: no move sits in the doubtful span.
fn fresh_state_frame() -> plr_analyzer::AnchorFrame {
    let state = GcodeState::new();
    plr_analyzer::AnchorFrame {
        absolute_coordinates: state.absolute_coord,
        absolute_extrude: state.absolute_extrude,
    }
}

/// Lossless on 64-bit; the fixtures are kilobytes.
fn offset_u64(v: usize) -> u64 {
    u64::try_from(v).expect("fixture offsets fit in u64")
}

/// The comment the fixture generator plants immediately after the last
/// positive-extrusion line, so "the whole footer" is exactly "every byte
/// at or after this".
const LAST_DEPOSITION_MARKER: &str = "; THE LAST DEPOSITING LINE IS ABOVE THIS COMMENT";

/// Byte offset one past the end of the line containing `needle`.
fn offset_after_line(data: &[u8], needle: &str) -> u64 {
    let n = needle.as_bytes();
    let start = data
        .windows(n.len())
        .position(|w| w == n)
        .unwrap_or_else(|| panic!("needle {needle:?} not found"));
    let rest = data.get(start..).unwrap_or_default();
    let nl = rest
        .iter()
        .position(|&b| b == b'\n')
        .map_or(rest.len(), |i| i + 1);
    offset_u64(start + nl)
}

/// `(fixture, total bytes, bytes after the last depositing line)`.
///
/// The two `*_real_footer` entries carry a genuine footer: `PrusaSlicer`
/// 2.9.3 wrote 14,537 bytes over 403 lines after the last extrusion
/// (13,913 as committed — the model name and the footprint outline are
/// substituted for licensing, see `fixtures/synthetic/footer_generator.py`),
/// and `OrcaSlicer` 2.3.1 wrote 17,699 over 560 lines (17,695 committed).
///
/// Those are **measured, not estimated**, and they are the whole argument
/// against a percentage threshold: the gap is a near-constant 14–18 KB
/// regardless of how big the print is, so the same finished print reads as
/// 7% remaining in these small fixtures, ~5% in a 300 KB file and ~0.09%
/// in a 20 MB one. Nothing separates "finished" from "died on the last
/// layer" on that axis.
const FOOTER_FIXTURES: [(&str, u64, u64); 5] = [
    ("prusa_real_footer.gcode", 15_459, 13_962),
    ("orca_real_footer.gcode", 19_162, 17_744),
    ("prusa_footer_complete.gcode", 14_377, 12_783),
    ("orca_footer_complete.gcode", 13_922, 12_684),
    ("cura_footer_complete.gcode", 13_698, 12_780),
];

/// Offsets in `cura_footer_complete.gcode` at which the extruder-frame trust
/// check refuses rather than suppressing.
///
/// **Measured exhaustively**, not sampled: every offset in the file was
/// tested, the refusing set is contiguous `0..1060`, and the part of it at or
/// after the last-deposition marker (918) is `918..1060` — **142 offsets**.
/// `a_footer_that_changes_extrusion_mode_is_still_gated_correctly` re-derives
/// both bounds from the fixture so this constant cannot drift from the
/// measurement.
///
/// Cura's footer switches to relative positioning (`G91` at byte 1071) for
/// its wipe-out move, which contradicts the absolute-coordinates frame a
/// whole-file replay from `GcodeState::new()` starts in. For a tested offset
/// early enough that a *depositing-capable* move still sits between it and
/// that `G91`, the span was classified under a flag the file disagrees with,
/// so the check refuses. Past the last such move the materiality narrowing
/// lets it through — the span does still contain two `ExtrudeOnly` retracts,
/// so the property that holds is "no move whose classification the flag could
/// flip", not "no move at all".
///
/// 142 bytes of a 12,780-byte footer, failing towards **announcing**: a Cura
/// print that died in that window is offered a recovery it did not need,
/// which costs a dry run. Pinned so the cost stays visible.
const CURA_REFUSAL_WINDOW: std::ops::Range<u64> = 918..1_060;

/// The exact placeholder outline `footer_generator.py` substitutes for the
/// real one: a 20x20 mm square with 5 mm-spaced collinear vertices.
///
/// Pinned here so the guard below can assert the polygon **is** this, which
/// is a complete check. Asserting the *absence* of the real coordinates
/// would have meant writing those coordinates down in this file — putting
/// the very content the substitution removes back into the repository and
/// its history.
const PLACEHOLDER_POLYGON: &str = "[100.000,100.000],[105.000,100.000],[110.000,100.000],\
     [115.000,100.000],[120.000,100.000],[120.000,105.000],[120.000,110.000],\
     [120.000,115.000],[120.000,120.000],[115.000,120.000],[110.000,120.000],\
     [105.000,120.000],[100.000,120.000],[100.000,115.000],[100.000,110.000],\
     [100.000,105.000]";

/// The neutral object name the real footers were scrubbed to.
const PLACEHOLDER_NAME: &str = "part_a.stl";

/// **No fixture may carry third-party model content.** The real footers were
/// extracted from prints of a CC BY-ND model, and this repository is public.
/// Two things were substituted: the model file name, and the 52-point
/// footprint outline `PrusaSlicer` embeds in its `; objects_info = {...}`
/// line.
///
/// The assertions are **positive** — the polygon *is* the placeholder, every
/// object name *is* the placeholder — rather than "the real values are
/// absent". That is both stronger (any substitution failure fails here,
/// including one this test's author never thought of) and the only form that
/// does not require the real name and coordinates to be written into this
/// file, which would defeat the point.
///
/// # It checks the WORKING TREE only
///
/// A test can only see the files on disk. It says nothing about git history,
/// and history is where the real licensing exposure lives: a commit that
/// once contained the content puts it in the repository permanently, even if
/// a later commit removes it. **Do not treat a green run here as evidence
/// that a branch is clean.** That is a separate check, on the commits —
/// `git log -p main..HEAD -- fixtures/` — and the branch this test arrived
/// on was squashed to one commit for exactly that reason.
#[test]
fn no_fixture_carries_third_party_model_content() {
    // 1. Every object name in the real-footer fixtures is the placeholder.
    //    Both slicers label object boundaries with `; stop printing object
    //    <name> id:`, and Orca repeats the name on `EXCLUDE_OBJECT_END`.
    let mut names_checked = 0_usize;
    for name in ["prusa_real_footer.gcode", "orca_real_footer.gcode"] {
        let text = String::from_utf8(load(name)).expect("ascii fixture");
        for line in text.lines() {
            if let Some(rest) = line.strip_prefix("; stop printing object ") {
                let object = rest.split(" id:").next().unwrap_or(rest);
                assert_eq!(object, PLACEHOLDER_NAME, "{name}: {line}");
                names_checked += 1;
            }
            if let Some(rest) = line.strip_prefix("EXCLUDE_OBJECT_END NAME=") {
                assert!(rest.starts_with(PLACEHOLDER_NAME), "{name}: {line}");
                names_checked += 1;
            }
        }
    }
    assert!(names_checked >= 3, "only {names_checked} object names seen");

    // 2. The footprint outline IS the placeholder, and the line around it is
    //    still structurally what `PrusaSlicer` emits: same key names, same
    //    nesting, same numeric formatting, so the fixture keeps exercising a
    //    long comma-and-brace-heavy comment.
    let text = String::from_utf8(load("prusa_real_footer.gcode")).expect("ascii");
    let line = text
        .lines()
        .find(|l| l.starts_with("; objects_info = "))
        .expect("objects_info line");
    assert_eq!(
        line,
        format!(
            "; objects_info = {{\"objects\":[{{\"name\":\"{PLACEHOLDER_NAME} id:0 copy 0\",\
             \"polygon\":[{PLACEHOLDER_POLYGON}]}}]}}"
        ),
        "the objects_info line must be exactly the scrubbed form"
    );
    assert!(line.len() > 300, "{} bytes is too short", line.len());

    // 3. No OTHER fixture grew an `objects_info` line carrying an outline.
    let mut scanned = 0_usize;
    for path in gcode_files(&fixtures_dir().join("synthetic")) {
        let text = String::from_utf8(fs::read(&path).expect("readable")).expect("ascii");
        let name = path.file_name().unwrap_or_default().to_string_lossy();
        for line in text.lines() {
            if line.starts_with("; objects_info = ") {
                assert!(
                    line.contains(PLACEHOLDER_POLYGON),
                    "{name} carries an outline that is not the placeholder"
                );
            }
        }
        scanned += 1;
    }
    assert!(scanned >= 5, "only {scanned} fixtures scanned");
}

/// **The corpus must contain a footer that changes extrusion mode.** Cura's
/// stock end g-code switches to relative positioning for its wipe-out move
/// and back (`G91` ... `G90`), and since the effective extruder frame is
/// `absolute_coord && absolute_extrude`, those are exactly the commands
/// `remaining_work`'s trust check reasons about. Without a fixture like this
/// the check's branch is never reached by the goldens, and an over-refusal on
/// ordinary Cura output would go unnoticed.
///
/// The gate must still answer "complete" here: the mode commands sit after
/// the tested offset but agree with the anchor frame these goldens replay
/// from, so there is nothing to contradict.
#[test]
fn a_footer_that_changes_extrusion_mode_is_still_gated_correctly() {
    let data = load("cura_footer_complete.gcode");
    let text = String::from_utf8(data.clone()).expect("ascii fixture");
    let marker = offset_of(&data, LAST_DEPOSITION_MARKER);
    // The mode commands really are in the footer, i.e. after the offset.
    let g91 = offset_of(&data, "G91 ;relative positioning");
    let g90 = offset_of(&data, "G90 ;absolute positioning");
    assert!(g91 > marker && g90 > g91, "{g91} {g90} vs {marker}");
    assert!(text.contains("M82 ;absolute extrusion mode"));

    let model = model_of(&data);

    // At the marker a move still sits between the offset and the `G91`, so
    // the span was classified under a flag the file contradicts: refuse.
    // This is the over-refusal, and it is the safe direction.
    let refused = remaining_work(&model, &data, 0, marker, fresh_state_frame(), None);
    assert_eq!(
        refused,
        Err(plr_analyzer::WorkUnknown::ExtrudeModeContradiction {
            offset: g91,
            axis: plr_analyzer::ModeAxis::Coordinates,
            file_absolute: false,
        }),
        "the coordinate axis is what bites here, not the extrusion axis"
    );
    assert_eq!(marker, CURA_REFUSAL_WINDOW.start, "the window's low end");

    // The window's bounds are re-derived here, exhaustively, so the pinned
    // constant is the measurement rather than a claim about it.
    let refusing: Vec<u64> = (0..offset_u64(data.len()))
        .filter(|off| remaining_work(&model, &data, 0, *off, fresh_state_frame(), None).is_err())
        .collect();
    assert!(
        refusing.windows(2).all(|w| w[1] == w[0] + 1),
        "the refusing set must be contiguous"
    );
    let measured = *refusing.first().expect("some offset refuses")
        ..*refusing.last().expect("some offset refuses") + 1;
    assert_eq!(measured.start, 0, "refusal starts at the file head");
    assert_eq!(
        CURA_REFUSAL_WINDOW,
        marker..measured.end,
        "the pinned window must equal the measurement from the marker on"
    );
    assert_eq!(CURA_REFUSAL_WINDOW.count(), 142);

    // Immediately past the window the check lets it through — including the
    // offsets between the window's end and the `G91` itself, which a sampling
    // stride can miss entirely.
    for offset in [CURA_REFUSAL_WINDOW.end, 1_065, g91, g90] {
        let work = remaining_work(&model, &data, 0, offset, fresh_state_frame(), None)
            .unwrap_or_else(|e| panic!("at {offset}: {e}"));
        assert!(work.is_complete(), "at {offset}: {work:?}");
    }

    // The window is not move-free: it holds two `ExtrudeOnly` retracts. What
    // holds is that none of them deposits.
    let in_window: Vec<MoveKind> = model
        .moves
        .iter()
        .filter(|m| m.span.start >= marker && m.span.start < g91)
        .map(|m| m.kind)
        .collect();
    assert_eq!(
        in_window,
        vec![MoveKind::ExtrudeOnly, MoveKind::ExtrudeOnly]
    );

    // And an anchor that agrees with the footer suppresses from the marker.
    let agreeing = plr_analyzer::AnchorFrame {
        absolute_coordinates: false,
        absolute_extrude: true,
    };
    assert!(remaining_work(&model, &data, 0, marker, agreeing, None)
        .expect("agreeing frame")
        .is_complete());
}

/// The measured distance from the last depositing line to EOF, pinned.
#[test]
fn footer_fixtures_end_far_before_eof() {
    for (name, expected_size, expected_gap) in FOOTER_FIXTURES {
        let data = load(name);
        assert_eq!(offset_u64(data.len()), expected_size, "{name} size changed");
        let model = model_of(&data);
        assert_eq!(
            model.stop,
            plr_analyzer::model::ModelStop::EndOfInput,
            "{name} must replay to the end"
        );
        let last = model
            .moves
            .iter()
            .rfind(|m| m.kind == MoveKind::Extrusion)
            .unwrap_or_else(|| panic!("{name} has no deposition"));
        let marker = offset_of(&data, LAST_DEPOSITION_MARKER);
        assert!(
            last.span.end <= marker,
            "{name}: deposition at {}..{} must precede the marker at {marker}",
            last.span.start,
            last.span.end
        );
        assert_eq!(
            offset_u64(data.len()) - marker,
            expected_gap,
            "{name}: bytes after the last depositing line"
        );
        // The whole tail is a footer: replaying it finds nothing that
        // deposits, anywhere in it — except inside the measured window where
        // Cura's `G91` makes the trust check refuse rather than suppress
        // (see `CURA_REFUSAL_WINDOW`; refusing is the safe direction).
        let cura = name == "cura_footer_complete.gcode";
        for offset in std::iter::once(marker).chain((marker..offset_u64(data.len())).step_by(97)) {
            let got = remaining_work(&model, &data, 0, offset, fresh_state_frame(), None);
            if cura && CURA_REFUSAL_WINDOW.contains(&offset) {
                assert!(
                    matches!(
                        got,
                        Err(plr_analyzer::WorkUnknown::ExtrudeModeContradiction { .. })
                    ),
                    "{name} at {offset}: expected the documented refusal, got {got:?}"
                );
            } else {
                let work = got.unwrap_or_else(|e| panic!("{name} at {offset}: {e}"));
                assert!(work.is_complete(), "{name} at {offset}: {work:?}");
            }
        }
    }
}

/// One line earlier — the true "died on the last move" case — the gate
/// announces. The distinction a percentage cannot make.
#[test]
fn footer_fixtures_still_report_the_final_move_as_work() {
    for (name, last_line) in [
        // The real-footer fixtures end their body on a leading-dot float,
        // which is the form real slicers emit.
        ("prusa_real_footer.gcode", "G1 X117.121 Y105.942 E.03577"),
        ("orca_real_footer.gcode", "G1 X111.453 Y115.5 E.03577"),
        ("prusa_footer_complete.gcode", "G1 X70 Y30 E4.9768"),
        (
            "orca_footer_complete.gcode",
            "G1 X55 Y55 E0.7465\nEXCLUDE_OBJECT_END",
        ),
    ] {
        let data = load(name);
        let model = model_of(&data);
        let offset = offset_of(&data, last_line);
        let work =
            remaining_work(&model, &data, 0, offset, fresh_state_frame(), None).expect("modeled");
        assert!(
            !work.is_complete(),
            "{name}: one move short of done must still be work: {work:?}"
        );
    }
}

/// **Leading-dot floats.** Both `PrusaSlicer` and `OrcaSlicer` write E values
/// with no digit before the decimal point (`E-.64987`, `E.03577`). No
/// fixture in the corpus carried that form before the real footers arrived,
/// so this asserts the parser's behaviour rather than reasoning about it —
/// a positive leading-dot E that failed to classify as
/// [`MoveKind::Extrusion`] would silently hide work in every
/// `PrusaSlicer`/Orca file.
#[test]
fn leading_dot_floats_parse_and_classify() {
    // Positive, negative, and negative-with-XY, all leading-dot.
    let text = "G90\nM83\nG1 Z0.2 F7200\n\
                G1 X10 Y0 E.03577 F1800\n\
                G1 E-.64987 F1800\n\
                G1 X11 Y0 E-.01429\n\
                G1 X12 Y0 E.5\n";
    let data = text.as_bytes();
    let model = model_of(data);
    assert_eq!(model.stop, plr_analyzer::model::ModelStop::EndOfInput);
    let expected = [
        (MoveKind::Travel, 0.0),            // the Z move
        (MoveKind::Extrusion, 0.035_77),    // E.03577 -> DEPOSITS
        (MoveKind::ExtrudeOnly, -0.649_87), // E-.64987 retract
        (MoveKind::Travel, -0.014_29),      // wipe: XY motion, negative E
        (MoveKind::Extrusion, 0.5),         // E.5
    ];
    assert_eq!(model.moves.len(), expected.len());
    for (got, (kind, e_delta)) in model.moves.iter().zip(expected) {
        assert_eq!(got.kind, kind, "at {:?}", got.span);
        // Epsilon, not equality: E accumulates across moves in relative
        // mode, so the *difference* carries one rounding step. The parse
        // itself is exact; that is what is being asserted.
        assert!(
            (got.e_delta() - e_delta).abs() < 1e-12,
            "leading-dot float parsed as {} not {e_delta} at {:?}",
            got.e_delta(),
            got.span
        );
    }
    // And the same forms as they appear in the real footers.
    for name in ["prusa_real_footer.gcode", "orca_real_footer.gcode"] {
        let data = load(name);
        let text = String::from_utf8(data.clone()).expect("ascii fixture");
        assert!(
            text.contains("E-."),
            "{name} must retain the real leading-dot forms"
        );
        assert_eq!(
            model_of(&data).stop,
            plr_analyzer::model::ModelStop::EndOfInput,
            "{name}: leading-dot floats must not stop the replay"
        );
    }
}

/// **The wipe trail is not work.** Both real footers begin with a retract
/// and then a `;WIPE_START`…`;WIPE_END` block of genuine XY moves carrying
/// negative E. So "no motion remaining" would be the wrong test and would
/// announce a recovery for every completed print; "no positive extrusion
/// remaining" is the right one.
#[test]
fn the_real_wipe_trail_is_motion_but_not_work() {
    for (name, wipe_line) in [
        ("prusa_real_footer.gcode", "G1 X117.069 Y105.757 E-.04571"),
        ("orca_real_footer.gcode", "G1 X111.935 Y115.272 E-.08008"),
    ] {
        let data = load(name);
        let model = model_of(&data);
        let wipe_at = offset_of(&data, wipe_line);
        // It really is XY motion, and it really carries negative E.
        let wipe = model
            .moves
            .iter()
            .find(|m| m.span.start == wipe_at)
            .unwrap_or_else(|| panic!("{name}: no move for the wipe line"));
        assert!(
            (wipe.end[0] - wipe.start[0]).abs() + (wipe.end[1] - wipe.start[1]).abs() > 0.0,
            "{name}: the wipe must actually move XY"
        );
        assert!(wipe.e_delta() < 0.0, "{name}: the wipe must retract");
        assert_eq!(wipe.kind, MoveKind::Travel, "{name}");
        // And the gate is unmoved by it.
        let marker = offset_of(&data, LAST_DEPOSITION_MARKER);
        assert!(
            remaining_work(&model, &data, 0, marker, fresh_state_frame(), None)
                .expect("modeled")
                .is_complete()
        );
    }
}

/// The footer's un-run commands are *named*, not offered: this is what an
/// operator gets instead of a bogus recovery. Pinned against the real
/// `PrusaSlicer` end sequence.
#[test]
fn a_real_footer_stop_names_the_end_sequence() {
    let data = load("prusa_real_footer.gcode");
    let model = model_of(&data);
    let offset = offset_of(&data, LAST_DEPOSITION_MARKER);
    let work =
        remaining_work(&model, &data, 0, offset, fresh_state_frame(), None).expect("modeled");
    let RemainingWork::EndSequenceOnly { commands } = &work else {
        panic!("expected EndSequenceOnly, got {work:?}");
    };
    // The real sequence: retract, wipe, fan off, four park moves, bed off,
    // nozzle off, fan off again, motors off.
    assert_eq!(
        commands,
        &["G1", "G1", "G1", "G1", "M107", "G1", "G1", "G1", "G1", "M140", "M104", "M107", "M84"]
            .map(str::to_owned)
    );
    assert!(!work.commands_truncated());
    // Deeper into the config block there is nothing left at all.
    let offset = offset_of(&data, "; prusaslicer_config = begin");
    assert_eq!(
        remaining_work(&model, &data, 0, offset, fresh_state_frame(), None).expect("modeled"),
        RemainingWork::Nothing
    );
}

/// The synthetic pair's end sequence, for the same assertion on a fixture
/// that is free to change.
#[test]
fn a_synthetic_footer_stop_names_the_end_sequence() {
    let data = load("prusa_footer_complete.gcode");
    let model = model_of(&data);
    let offset = offset_after_line(&data, "; --- end gcode ---");
    let work =
        remaining_work(&model, &data, 0, offset, fresh_state_frame(), None).expect("modeled");
    let RemainingWork::EndSequenceOnly { commands } = &work else {
        panic!("expected EndSequenceOnly, got {work:?}");
    };
    assert_eq!(
        commands,
        &["M107", "M104", "M140", "G1", "G4", "M400", "G1", "G1", "M84", "M117"].map(str::to_owned)
    );
}

/// Object attribution comes off the `EXCLUDE_OBJECT_START`/`END` brackets,
/// upper-cased as Klipper stores names.
#[test]
fn footer_fixtures_attribute_deposition_to_objects() {
    let data = load("prusa_footer_complete.gcode");
    let model = model_of(&data);
    let names: std::collections::BTreeSet<&str> = model
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Extrusion)
        .filter_map(|m| m.object.as_deref())
        .collect();
    assert_eq!(
        names.into_iter().collect::<Vec<_>>(),
        vec!["PART_A", "PART_B"]
    );
    let data = load("orca_footer_complete.gcode");
    let model = model_of(&data);
    assert!(model
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Extrusion)
        .all(|m| m.object.as_deref() == Some("BODY1_ID_0_COPY_0")));
    // Orca emits the real command form, and it appears INSIDE the footer:
    // `EXCLUDE_OBJECT_END NAME=part_a.stl_id_0_copy_0`. The bracket closes
    // regardless of the name, which is what keeps a nesting disagreement
    // from leaving an object open forever.
    let data = load("orca_real_footer.gcode");
    let model = model_of(&data);
    assert!(model
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Extrusion)
        .all(|m| m.object.as_deref() == Some("PART_A.STL_ID_0_COPY_0")));
}

/// **`; stop printing object …` is a comment, and stays one.**
///
/// `PrusaSlicer` (and Orca) mark object boundaries with
/// `; printing object …` / `; stop printing object …` comments. Klipper's
/// object-cancellation support needs the `EXCLUDE_OBJECT_*` *commands*, so
/// a preprocessor (Moonraker's `preprocess_cancellation`, or the slicer's
/// own "label objects: firmware" setting) rewrites them — meaning a file in
/// the wild may carry either form, or the comment form alone.
///
/// This asserts the conservative outcome rather than adding a second
/// attribution path: an unconverted file yields **no** object attribution,
/// so its deposition is unattributed, so it always counts as work. A
/// cancelled object on such a file produces a false offer, never a
/// suppressed one.
#[test]
fn the_comment_form_of_object_markers_yields_no_attribution() {
    struct ExcludeEverything;
    impl plr_analyzer::ExclusionOracle for ExcludeEverything {
        fn is_conclusive(&self) -> bool {
            true
        }
        fn is_excluded(&self, _object: &str) -> bool {
            true
        }
    }

    let data = load("prusa_real_footer.gcode");
    let text = String::from_utf8(data.clone()).expect("ascii fixture");
    assert!(
        text.contains("; stop printing object part_a.stl id:0 copy 0"),
        "the fixture must retain the comment form"
    );
    let model = model_of(&data);
    // No EXCLUDE_OBJECT command anywhere, so nothing is attributed.
    assert!(
        !text.contains("EXCLUDE_OBJECT_START"),
        "this fixture is deliberately unconverted"
    );
    assert!(
        model.moves.iter().all(|m| m.object.is_none()),
        "the comment form must not attribute deposition"
    );
    // And unattributed deposition counts as work even when every named
    // object is excluded — the safe direction.
    let body_start = offset_of(&data, "G1 X155 Y95 E1.8663");
    let work = remaining_work(
        &model,
        &data,
        0,
        body_start,
        fresh_state_frame(),
        Some(&ExcludeEverything),
    )
    .expect("modeled");
    assert!(
        !work.is_complete(),
        "unattributed deposition must survive a blanket exclusion: {work:?}"
    );
}

/// If the whole plate is cancelled, the remaining deposition does not
/// count — but only while the exclusion picture is conclusive.
#[test]
fn a_fully_cancelled_plate_is_complete_only_when_conclusive() {
    struct Oracle(bool);
    impl plr_analyzer::ExclusionOracle for Oracle {
        fn is_conclusive(&self) -> bool {
            self.0
        }
        fn is_excluded(&self, object: &str) -> bool {
            object == "PART_A" || object == "PART_B"
        }
    }
    let data = load("prusa_footer_complete.gcode");
    let model = model_of(&data);
    let conclusive = remaining_work(
        &model,
        &data,
        0,
        0,
        fresh_state_frame(),
        Some(&Oracle(true)),
    )
    .expect("modeled");
    assert!(conclusive.is_complete(), "{conclusive:?}");
    let doubtful = remaining_work(
        &model,
        &data,
        0,
        0,
        fresh_state_frame(),
        Some(&Oracle(false)),
    )
    .expect("modeled");
    assert!(
        !doubtful.is_complete(),
        "an inconclusive report must count excluded work as work: {doubtful:?}"
    );
}
