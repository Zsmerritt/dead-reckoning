//! Property tests: parser totality, serialize/reparse stability, arc
//! chord geometry bounds, and state-machine invariants.

#![allow(clippy::float_cmp)] // exactness properties are intentional

use proptest::prelude::*;

use plr_gcode::{
    parse_line, plan_arc, scan_z_events, simulate, ArcPlane, ArcRequest, ByteSpan, GcodeState,
    Line, LineIter, SimConfig, ZScanConfig,
};

fn parse(s: &str) -> Line {
    parse_line(
        s.as_bytes(),
        ByteSpan {
            start: 0,
            end: s.len() as u64,
        },
    )
}

proptest! {
    /// The parser and state machine are total: no input bytes — valid,
    /// garbage, or non-UTF-8 — may panic, and spans always tile the
    /// buffer.
    #[test]
    fn parser_total_on_arbitrary_bytes(data in proptest::collection::vec(any::<u8>(), 0..2048), base in any::<u32>()) {
        let base = u64::from(base);
        let mut state = GcodeState::new();
        let mut expect = base;
        for line in LineIter::new(&data, base) {
            prop_assert_eq!(line.span.start, expect);
            prop_assert!(line.span.end > line.span.start);
            expect = line.span.end;
            // Applying must never panic (errors are fine).
            let _ = state.apply(&line);
            // Display must never panic either.
            let _ = line.to_string();
        }
        prop_assert_eq!(expect, base + data.len() as u64);
    }

    /// serialize/reparse converges for arbitrary input: by the second
    /// round the representation is a fixpoint. (Exact first-round
    /// stability cannot hold for garbage like `n0n[a`, whose residual
    /// command name `N[` re-tokenizes as a line-number prefix; Klipper
    /// treats both spellings as unknown-command no-ops. First-round
    /// stability for valid lines is covered by the generated-commands
    /// property below and by unit tests.)
    #[test]
    fn serialize_reparse_converges_on_arbitrary_ascii(s in "[ -~]{0,80}") {
        let l1 = parse(&s);
        let l2 = parse(&l1.to_string());
        let out2 = l2.to_string();
        let l3 = parse(&out2);
        prop_assert_eq!(&l3.body, &l2.body, "no fixpoint: {:?} -> {:?} -> {:?}", s, l1.to_string(), out2);
        prop_assert_eq!(out2.clone(), l3.to_string());
    }

    /// serialize/reparse stability on realistic generated command lines.
    #[test]
    fn serialize_reparse_stable_on_generated_commands(
        cmd in prop::sample::select(vec!["G0", "G1", "G92", "M204", "M220", "M221"]),
        x in proptest::option::of(-500.0..500.0_f64),
        y in proptest::option::of(-500.0..500.0_f64),
        z in proptest::option::of(-10.0..10.0_f64),
        e in proptest::option::of(-10.0..10.0_f64),
        comment in proptest::option::of("[ -:<-~][ -~]{0,20}"),
    ) {
        use std::fmt::Write as _;
        let mut s = cmd.to_string();
        for (k, v) in [("X", x), ("Y", y), ("Z", z), ("E", e)] {
            if let Some(v) = v {
                let _ = write!(s, " {k}{v}");
            }
        }
        if let Some(c) = &comment {
            let _ = write!(s, " ;{c}");
        }
        let l1 = parse(&s);
        let l2 = parse(&l1.to_string());
        prop_assert_eq!(&l1.body, &l2.body);
        // Numeric params survive the round trip exactly.
        if let (Some(c1), Some(c2)) = (l1.command(), l2.command()) {
            for key in ["X", "Y", "Z", "E"] {
                prop_assert_eq!(c1.get(key), c2.get(key));
            }
        }
    }

    /// Arc chords: endpoints exact, every chord endpoint on the circle,
    /// radial deviation within the resolution-derived sagitta bound,
    /// and E conserved.
    #[test]
    fn arc_chords_respect_geometry(
        cx in -100.0..100.0_f64,
        cy in -100.0..100.0_f64,
        radius in 0.5..80.0_f64,
        start_angle in 0.0..std::f64::consts::TAU,
        sweep in 0.05..std::f64::consts::TAU,
        clockwise in any::<bool>(),
        resolution in 0.2..3.0_f64,
        e_target in proptest::option::of(0.01..40.0_f64),
        absolute_extrude in any::<bool>(),
    ) {
        let start = [cx + radius * start_angle.cos(), cy + radius * start_angle.sin()];
        let end_angle = if clockwise { start_angle - sweep } else { start_angle + sweep };
        let target = [cx + radius * end_angle.cos(), cy + radius * end_angle.sin(), 0.0];
        let current = [start[0], start[1], 0.0, 1.0];
        let req = ArcRequest {
            current,
            target,
            offset: (cx - start[0], cy - start[1]),
            plane: ArcPlane::Xy,
            clockwise,
            absolute_extrude,
            e_param: e_target,
            f_param: None,
            resolution,
        };
        let segs = plan_arc(&req).expect("valid arc");
        prop_assert!(!segs.is_empty());
        // Endpoint is bit-exact (planArc line 169-170).
        prop_assert_eq!(segs.last().expect("nonempty").target, target);
        // Every chord endpoint (except the snapped final target) lies on
        // the circle.
        for s in &segs[..segs.len() - 1] {
            let r = (s.target[0] - cx).hypot(s.target[1] - cy);
            prop_assert!((r - radius).abs() < 1e-9 * (1.0 + radius), "off-circle: r={r} vs {radius}");
        }
        // Max radial deviation along each chord (sagitta) is bounded by
        // the resolution: chord arc-length < 2*resolution (floor rule),
        // so sagitta <= resolution^2 / (2 * radius) plus float slack.
        let bound = resolution * resolution / (2.0 * radius) + 1e-9;
        let mut prev = [start[0], start[1]];
        for s in &segs {
            let mid = [(prev[0] + s.target[0]) * 0.5, (prev[1] + s.target[1]) * 0.5];
            let r_mid = (mid[0] - cx).hypot(mid[1] - cy);
            prop_assert!(radius - r_mid <= bound, "sagitta {} exceeds bound {bound}", radius - r_mid);
            prop_assert!(r_mid <= radius + 1e-9 * (1.0 + radius));
            prev = [s.target[0], s.target[1]];
        }
        // E is conserved across chords.
        if let Some(e_t) = e_target {
            if absolute_extrude {
                let last_e = segs.last().expect("nonempty").e.expect("extruding arc");
                prop_assert!((last_e - e_t).abs() < 1e-6 * (1.0 + e_t.abs()));
            } else {
                let total: f64 = segs.iter().filter_map(|s| s.e).sum();
                prop_assert!((total - e_t).abs() < 1e-6 * (1.0 + e_t.abs()));
            }
        } else {
            prop_assert!(segs.iter().all(|s| s.e.is_none()));
        }
    }

    /// plan_arc is total over arbitrary f64 bit patterns (NaN and
    /// infinities included): it never panics, and `Ok` always means at
    /// least one chord (Klipper's `max(1, ...)` rule) computed from
    /// all-finite inputs — never a silent garbage decomposition.
    #[test]
    fn plan_arc_total_over_arbitrary_bits(
        bits in proptest::collection::vec(any::<u64>(), 12),
        plane_sel in 0..3_usize,
        clockwise in any::<bool>(),
        absolute_extrude in any::<bool>(),
        has_e in any::<bool>(),
        has_f in any::<bool>(),
    ) {
        let v: Vec<f64> = bits.iter().copied().map(f64::from_bits).collect();
        let plane = [ArcPlane::Xy, ArcPlane::Xz, ArcPlane::Yz][plane_sel];
        let req = ArcRequest {
            current: [v[0], v[1], v[2], v[3]],
            target: [v[4], v[5], v[6]],
            offset: (v[7], v[8]),
            plane,
            clockwise,
            absolute_extrude,
            e_param: has_e.then_some(v[9]),
            f_param: has_f.then_some(v[10]),
            resolution: v[11],
        };
        let inputs_finite = req.current.iter().all(|x| x.is_finite())
            && req.target.iter().all(|x| x.is_finite())
            && req.offset.0.is_finite()
            && req.offset.1.is_finite()
            && req.e_param.is_none_or(f64::is_finite)
            && req.f_param.is_none_or(f64::is_finite);
        if let Ok(segs) = plan_arc(&req) {
            prop_assert!(!segs.is_empty(), "Ok with zero chords");
            prop_assert!(inputs_finite, "Ok despite non-finite input");
            prop_assert!(
                req.resolution.is_finite() && req.resolution > 0.0,
                "Ok despite invalid resolution"
            );
            // The final chord always lands exactly on the target.
            prop_assert_eq!(segs.last().expect("nonempty").target, req.target);
        }
    }

    /// M220 never changes positions; G92 changes base_position only;
    /// M221 preserves the g-code E reading; G90/G91 round-trips.
    #[test]
    fn state_invariants(
        x in -100.0..100.0_f64,
        e in -5.0..50.0_f64,
        m220 in 1.0..500.0_f64,
        m221 in 1.0..500.0_f64,
        g92e in proptest::option::of(-10.0..10.0_f64),
    ) {
        let mut s = GcodeState::new();
        let apply = |s: &mut GcodeState, text: String| {
            s.apply(&parse(&text)).expect("valid command")
        };
        apply(&mut s, format!("G1 X{x} E{e} F3000"));
        let before = s.clone();

        // M220 changes speed bookkeeping only.
        apply(&mut s, format!("M220 S{m220}"));
        prop_assert_eq!(s.last_position, before.last_position);
        prop_assert_eq!(s.base_position, before.base_position);
        prop_assert_eq!(s.homing_position, before.homing_position);
        prop_assert!((s.gcode_speed() - before.gcode_speed()).abs() < 1e-9 * before.gcode_speed());

        // M221 preserves the g-code E reading and every position.
        let e_read = s.gcode_position()[3];
        apply(&mut s, format!("M221 S{m221}"));
        prop_assert_eq!(s.last_position, before.last_position);
        prop_assert!((s.gcode_position()[3] - e_read).abs() < 1e-9 * (1.0 + e_read.abs()));

        // G92 shifts base only; last_position bit-identical.
        let last = s.last_position;
        match g92e {
            Some(v) => { apply(&mut s, format!("G92 E{v}")); }
            None => { apply(&mut s, "G92".to_string()); }
        }
        prop_assert_eq!(s.last_position, last);
        if let Some(v) = g92e {
            // The g-code E reading is now exactly the G92 argument
            // (modulo the factor-scale division round trip).
            prop_assert!((s.gcode_position()[3] - v).abs() < 1e-9 * (1.0 + v.abs()));
        }

        // G91 followed by G90 restores absolute mode.
        apply(&mut s, "G91".to_string());
        prop_assert!(!s.absolute_coord);
        apply(&mut s, "G90".to_string());
        prop_assert!(s.absolute_coord);
    }

    /// Relative/absolute E round trip: a G91+M83 move of +d then -d
    /// returns E and XYZ to the start, for any extrude factor.
    #[test]
    fn relative_round_trip_returns_home(
        dx in -50.0..50.0_f64,
        de in -5.0..5.0_f64,
        factor in 10.0..300.0_f64,
    ) {
        let mut s = GcodeState::new();
        let apply = |s: &mut GcodeState, text: String| {
            s.apply(&parse(&text)).expect("valid command")
        };
        apply(&mut s, "G1 X10 Y10 E3 F3000".to_string());
        apply(&mut s, format!("M221 S{factor}"));
        let before = s.last_position;
        apply(&mut s, "G91".to_string());
        apply(&mut s, "M83".to_string());
        apply(&mut s, format!("G1 X{dx} E{de}"));
        apply(&mut s, format!("G1 X{} E{}", -dx, -de));
        for (axis, (now, was)) in s.last_position.iter().zip(&before).enumerate() {
            prop_assert!(
                (now - was).abs() < 1e-9,
                "axis {} drifted: {} vs {}", axis, now, was
            );
        }
    }

    /// Full pipeline on generated command streams: simulation and Z scan
    /// never panic, and their Z sequences always agree.
    #[test]
    fn pipeline_z_consistency_on_generated_streams(
        cmds in proptest::collection::vec(
            prop::sample::select(vec![
                "G1 X10 Y5 E0.5 F3000",
                "G1 Z0.6 F7200",
                "G1 Z0.2",
                "G1 E-0.8 F2100",
                "G1 E0.8",
                "G91",
                "G90",
                "M83",
                "M82",
                "G92 E0",
                "M220 S150",
                "M221 S95",
                "M204 S4000",
                "G1 X-3 Y2 Z0.01 E0.1 F1200",
                "SET_GCODE_OFFSET Z_ADJUST=0.01",
                ";TYPE:Internal infill",
                "",
                "G28 Z",
                "M117 status",
            ]),
            0..60,
        )
    ) {
        let text = cmds.join("\n");
        let lines: Vec<Line> = LineIter::new(text.as_bytes(), 0).collect();
        let cfg = SimConfig { max_duration: None, max_lines: None, ..SimConfig::default() };
        let mut s1 = GcodeState::new();
        let sim = simulate(&mut s1, &lines, &cfg);
        let mut s2 = GcodeState::new();
        let scan = scan_z_events(
            &mut s2,
            &lines,
            &ZScanConfig { max_lines: None, max_events: None },
        );
        // Both consume the same number of lines (same error behavior).
        prop_assert_eq!(sim.lines_consumed, scan.lines_consumed);
        let from_sim: Vec<_> = sim
            .moves
            .iter()
            .filter_map(|tm| plr_gcode::z_event_of(&tm.planned))
            .collect();
        prop_assert_eq!(scan.events, from_sim);
        prop_assert_eq!(s1, s2);
        // Timing outputs are ordered and non-negative.
        let mut t = 0.0_f64;
        for m in &sim.moves {
            prop_assert!(m.start_time >= t - 1e-12);
            prop_assert!(m.duration() >= 0.0);
            t = m.end_time();
        }
    }
}
