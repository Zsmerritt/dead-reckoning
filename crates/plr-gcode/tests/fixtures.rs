//! Full-pipeline tests over the committed fixture corpus
//! (`fixtures/synthetic/`) and any real sliced files dropped into
//! `fixtures/real/`.

use std::fs;
use std::path::{Path, PathBuf};

use plr_gcode::{
    scan_z_events, simulate, z_event_of, GcodeState, Line, LineIter, SimConfig, StopReason,
    ZScanConfig,
};

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

fn unbounded_sim() -> SimConfig {
    SimConfig {
        max_duration: None,
        max_lines: None,
        ..SimConfig::default()
    }
}

fn unbounded_scan() -> ZScanConfig {
    ZScanConfig {
        max_lines: None,
        max_events: None,
    }
}

/// The full pipeline every fixture (synthetic or real) must survive:
/// spans tile the file, simulation completes without error, and the
/// Z-event scan agrees with the simulation's move stream.
fn run_pipeline(path: &Path) {
    let name = path.display();
    let bytes = fs::read(path).unwrap_or_else(|e| panic!("read {name}: {e}"));
    let lines: Vec<Line> = LineIter::new(&bytes, 0).collect();
    assert!(!lines.is_empty(), "{name}: no lines parsed");

    // Byte spans tile the whole file: every span.start/end is a line
    // boundary usable for M26.
    let mut expect = 0_u64;
    for l in &lines {
        assert_eq!(l.span.start, expect, "{name}: span gap");
        assert!(l.span.end > l.span.start, "{name}: empty span");
        expect = l.span.end;
    }
    assert_eq!(expect, bytes.len() as u64, "{name}: spans do not tile file");

    // Simulation runs to completion without error.
    let mut sim_state = GcodeState::new();
    let sim = simulate(&mut sim_state, &lines, &unbounded_sim());
    assert_eq!(
        sim.stop,
        StopReason::EndOfInput,
        "{name}: simulation did not complete cleanly"
    );
    assert_eq!(sim.lines_consumed, lines.len(), "{name}");
    assert_eq!(sim.resume_offset, Some(bytes.len() as u64), "{name}");
    assert!(sim.total_time.is_finite(), "{name}: non-finite total time");
    assert!(sim.total_time >= 0.0, "{name}");

    // Timestamps are monotone and contiguous.
    let mut t = 0.0_f64;
    for m in &sim.moves {
        assert!(
            (m.start_time - t).abs() < 1e-9,
            "{name}: non-contiguous timestamps"
        );
        assert!(m.duration() >= 0.0, "{name}");
        t = m.end_time();
    }

    // The Z-event scan is consistent with the simulated move stream:
    // identical Z sequence, from an independent state replay.
    let mut scan_state = GcodeState::new();
    let scan = scan_z_events(&mut scan_state, &lines, &unbounded_scan());
    assert_eq!(scan.stop, StopReason::EndOfInput, "{name}");
    let from_sim: Vec<_> = sim
        .moves
        .iter()
        .filter_map(|tm| z_event_of(&tm.planned))
        .collect();
    assert_eq!(scan.events, from_sim, "{name}: Z scan diverges from sim");

    // Both replays agree on the final state.
    assert_eq!(sim_state, scan_state, "{name}: replay states diverged");
}

#[test]
fn synthetic_corpus_full_pipeline() {
    let files = gcode_files(&fixtures_dir().join("synthetic"));
    assert!(
        files.len() >= 10,
        "synthetic fixture corpus missing (found {})",
        files.len()
    );
    for f in &files {
        run_pipeline(f);
    }
}

#[test]
fn real_corpus_full_pipeline() {
    // Auto-discovers any *.gcode dropped into fixtures/real/; passes
    // vacuously while the directory is empty.
    for f in &gcode_files(&fixtures_dir().join("real")) {
        run_pipeline(f);
    }
}

/// The same toolpath expressed with G2/G3 and pre-chorded must produce
/// identical move endpoint sequences (arc decomposition equivalence).
#[test]
fn arcs_ij_equals_prechorded() {
    let dir = fixtures_dir().join("synthetic");
    let endpoints = |file: &str| -> Vec<[f64; 4]> {
        let bytes = fs::read(dir.join(file)).expect("fixture");
        let lines: Vec<Line> = LineIter::new(&bytes, 0).collect();
        let mut state = GcodeState::new();
        let sim = simulate(&mut state, &lines, &unbounded_sim());
        assert_eq!(sim.stop, StopReason::EndOfInput, "{file}");
        sim.moves.iter().map(|m| m.planned.end).collect()
    };
    let native = endpoints("arcs_ij.gcode");
    let chorded = endpoints("arcs_prechorded.gcode");
    assert_eq!(native.len(), chorded.len(), "move counts differ");
    for (i, (a, b)) in native.iter().zip(&chorded).enumerate() {
        for axis in 0..4 {
            assert!(
                (a[axis] - b[axis]).abs() < 1e-9,
                "move {i} axis {axis}: {} vs {}",
                a[axis],
                b[axis]
            );
        }
    }
}

/// Absolute-E and relative-E variants of the same toolpath end at the
/// same internal position.
#[test]
fn absolute_and_relative_e_squares_agree() {
    let dir = fixtures_dir().join("synthetic");
    let final_state = |file: &str| -> GcodeState {
        let bytes = fs::read(dir.join(file)).expect("fixture");
        let lines: Vec<Line> = LineIter::new(&bytes, 0).collect();
        let mut state = GcodeState::new();
        let sim = simulate(&mut state, &lines, &unbounded_sim());
        assert_eq!(sim.stop, StopReason::EndOfInput, "{file}");
        state
    };
    let abs = final_state("abs_e_square.gcode");
    let rel = final_state("rel_e_square.gcode");
    for axis in 0..4 {
        assert!(
            (abs.last_position[axis] - rel.last_position[axis]).abs() < 1e-9,
            "axis {axis}: {} vs {}",
            abs.last_position[axis],
            rel.last_position[axis]
        );
    }
}

/// The z-hop fixture produces exactly the documented hop/layer-change
/// sequence with correct extruding flags.
#[test]
fn zhop_fixture_z_sequence_exact() {
    let bytes = fs::read(fixtures_dir().join("synthetic/zhop_retract.gcode")).expect("fixture");
    let lines: Vec<Line> = LineIter::new(&bytes, 0).collect();
    let mut state = GcodeState::new();
    let scan = scan_z_events(&mut state, &lines, &unbounded_scan());
    let seq: Vec<(f64, f64)> = scan.events.iter().map(|e| (e.z_from, e.z_to)).collect();
    assert_eq!(
        seq,
        vec![
            (0.0, 0.2), // initial layer height
            (0.2, 0.6), // hop up
            (0.6, 0.2), // hop down
            (0.2, 0.6), // hop up
            (0.6, 0.2), // hop down
            (0.2, 0.4), // layer change
        ]
    );
    assert!(scan.events.iter().all(|e| !e.extruding && e.z_known));
    // Each event's offset points at a line whose reparse starts with G1 Z.
    for ev in &scan.events {
        let start = usize::try_from(ev.span.start).expect("offset fits");
        let end = usize::try_from(ev.span.end).expect("offset fits");
        let text = std::str::from_utf8(&bytes[start..end]).expect("utf8");
        assert!(
            text.trim_start().to_uppercase().starts_with("G1 Z"),
            "unexpected line at offset {start}: {text:?}"
        );
    }
}

/// The vase fixture's Z events are all extruding (spiral).
#[test]
fn vase_fixture_is_all_spiral() {
    let bytes = fs::read(fixtures_dir().join("synthetic/vase_mode.gcode")).expect("fixture");
    let lines: Vec<Line> = LineIter::new(&bytes, 0).collect();
    let mut state = GcodeState::new();
    let scan = scan_z_events(&mut state, &lines, &unbounded_scan());
    // One initial layer move plus 16 spiral moves.
    assert_eq!(scan.events.len(), 17);
    assert!(scan.events.iter().skip(1).all(|e| e.extruding));
    assert!(!scan.events[0].extruding);
    // Z is monotone non-decreasing throughout the spiral.
    for w in scan.events.windows(2) {
        assert!(w[1].z_to >= w[0].z_to);
    }
}

/// The G28 fixture flags unknown-Z windows.
#[test]
fn g28_fixture_flags_unknown_z() {
    let bytes = fs::read(fixtures_dir().join("synthetic/g28_midfile.gcode")).expect("fixture");
    let lines: Vec<Line> = LineIter::new(&bytes, 0).collect();
    let mut state = GcodeState::new();
    let scan = scan_z_events(&mut state, &lines, &unbounded_scan());
    assert!(
        scan.events.iter().any(|e| !e.z_known),
        "expected unknown-Z events after G28"
    );
    assert!(
        scan.events.iter().any(|e| e.z_known),
        "expected known-Z events before G28"
    );
    // Positions become known again by the end (absolute moves).
    assert_eq!(state.position_known, [true; 4]);
}

/// Annotations in the slicer-style fixtures are recognized.
#[test]
fn slicer_annotations_recognized() {
    use plr_gcode::Annotation;
    let bytes = fs::read(fixtures_dir().join("synthetic/prusa_absolute_e.gcode")).expect("fixture");
    let lines: Vec<Line> = LineIter::new(&bytes, 0).collect();
    let annotations: Vec<Annotation> = lines
        .iter()
        .filter_map(|l| l.comment().and_then(plr_gcode::Comment::annotation))
        .collect();
    assert!(annotations
        .iter()
        .any(|a| matches!(a, Annotation::FeatureType(t) if t == "External perimeter")));
    assert!(annotations
        .iter()
        .any(|a| matches!(a, Annotation::LayerChange)));
    assert!(annotations
        .iter()
        .any(|a| matches!(a, Annotation::Z(z) if (*z - 0.4).abs() < 1e-12)));
}

/// CRLF fixture: spans tile, line numbers and checksums are stripped,
/// lowercase words parse.
#[test]
fn mixed_endings_fixture_parses() {
    let bytes = fs::read(fixtures_dir().join("synthetic/mixed_endings.gcode")).expect("fixture");
    assert!(
        bytes.windows(2).any(|w| w == b"\r\n"),
        "fixture lost its CRLF endings (check fixtures/.gitattributes)"
    );
    let lines: Vec<Line> = LineIter::new(&bytes, 0).collect();
    let mut state = GcodeState::new();
    let sim = simulate(&mut state, &lines, &unbounded_sim());
    assert_eq!(sim.stop, StopReason::EndOfInput);
    // n4 g1 z0.2 f7200*88 must have parsed as a real Z move.
    assert!((state.last_position[2] - 0.2).abs() < 1e-12);
    assert!((state.last_position[3] - 2.0).abs() < 1e-12);
}
