//! Totality property tests: reconstruction never panics, for any input —
//! including hand-built scans full of NaN/infinite floats, hostile
//! orderings, arbitrary scan ends, arbitrary heartbeat files, arbitrary
//! file-tail bytes, and out-of-domain configurations. Errors are fine;
//! panics are not.

use plr_gcode::SimConfig;
use plr_reconstruct::{
    compute_stop_window, ingest, parse_object_definitions, point_in_polygon, reconstruct,
    resolve_exclusions, ExclusionInputs, FileTail, ReceiveSeqObservation, ReconstructConfig,
    ReconstructInputs,
};
use plr_wal::{
    recover_heartbeat, Context, ExcludeObjectDef, ExcludeState, FanTarget, GcodeState, Heartbeat,
    HeaterTarget, Marker, MarkerKind, PolygonFidelity, RecoveryScan, ScanEnd, ScannedRecord,
    StepChunk, StepperRange, TransformObservations, TrapqSegment, VirtualSdState, WalRecord,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// Any f64 bit pattern: finite, subnormal, NaN, infinities.
fn any_f64() -> impl Strategy<Value = f64> {
    prop_oneof![
        4 => -1.0e12..1.0e12_f64,
        1 => Just(f64::NAN),
        1 => Just(f64::INFINITY),
        1 => Just(f64::NEG_INFINITY),
        1 => Just(f64::MIN_POSITIVE),
        1 => any::<u64>().prop_map(f64::from_bits),
    ]
}

fn any_trapq() -> impl Strategy<Value = TrapqSegment> {
    (
        any::<u64>(),
        prop_oneof![
            Just("toolhead".to_owned()),
            Just("extruder".to_owned()),
            Just("extruder1".to_owned()),
            Just("manual_stepper weird".to_owned()),
        ],
        proptest::collection::vec(any_f64(), 10),
    )
        .prop_map(|(mono_ns, queue, f)| TrapqSegment {
            mono_ns,
            queue,
            print_time: f[0],
            duration: f[1],
            start_velocity: f[2],
            acceleration: f[3],
            start_x: f[4],
            start_y: f[5],
            start_z: f[6],
            x_r: f[7],
            y_r: f[8],
            z_r: f[9],
        })
}

fn any_stepper() -> impl Strategy<Value = StepperRange> {
    (
        any::<u64>(),
        prop_oneof![
            Just("stepper_z".to_owned()),
            Just("stepper_z1".to_owned()),
            Just("stepper_x".to_owned()),
        ],
        any::<u64>(),
        any::<u64>(),
        proptest::collection::vec(any_f64(), 4),
        // Full i32 ranges: dump rows are the signed C ints of Klipper's
        // `struct pull_history_steps` (negative count = reverse steps).
        proptest::collection::vec((any::<i32>(), any::<i32>(), any::<i32>()), 0..3),
    )
        .prop_map(
            |(mono_ns, stepper, first_clock, last_clock, f, chunks)| StepperRange {
                mono_ns,
                stepper,
                first_clock,
                last_clock,
                first_step_time: f[0],
                last_step_time: f[1],
                start_position: f[2],
                start_mcu_position: 0,
                step_distance: f[3],
                steps: chunks
                    .into_iter()
                    .map(|(interval, count, add)| StepChunk {
                        interval,
                        count,
                        add,
                    })
                    .collect(),
            },
        )
}

fn any_gcode_state() -> impl Strategy<Value = GcodeState> {
    (
        proptest::collection::vec(any_f64(), 3),
        any::<bool>(),
        any::<bool>(),
        proptest::collection::vec(any_f64(), 0..6),
        proptest::collection::vec(any_f64(), 0..6),
        proptest::collection::vec(any_f64(), 0..6),
    )
        .prop_map(|(f, ac, ae, origin, pos, gpos)| GcodeState {
            speed_factor: f[0],
            speed: f[1],
            extrude_factor: f[2],
            absolute_coordinates: ac,
            absolute_extrude: ae,
            homing_origin: origin,
            position: pos,
            gcode_position: gpos,
        })
}

/// Hostile object outlines: NaN and infinite vertices, degenerate
/// rings, points with the wrong arity already collapsed away, and every
/// fidelity flag — the shapes a corrupt or hand-built WAL can carry.
fn any_polygon() -> impl Strategy<Value = Vec<[f64; 2]>> {
    proptest::collection::vec((any_f64(), any_f64()).prop_map(|(x, y)| [x, y]), 0..6)
}

fn any_exclude_def() -> impl Strategy<Value = ExcludeObjectDef> {
    (
        "[A-Z_0-9]{0,8}",
        proptest::option::of((any_f64(), any_f64()).prop_map(|(x, y)| [x, y])),
        any_polygon(),
        prop_oneof![
            Just(PolygonFidelity::Absent),
            Just(PolygonFidelity::Exact),
            any::<u32>().prop_map(|n| PolygonFidelity::BoundingBox { source_points: n }),
            any::<u32>().prop_map(|n| PolygonFidelity::Unusable { source_points: n }),
            Just(PolygonFidelity::Unknown),
        ],
    )
        .prop_map(|(name, center, polygon, fidelity)| ExcludeObjectDef {
            name,
            center,
            polygon,
            fidelity,
        })
}

fn any_exclude_state() -> impl Strategy<Value = ExcludeState> {
    (
        proptest::option::of(proptest::collection::vec(any_exclude_def(), 0..4)),
        proptest::collection::vec("[A-Z_0-9]{0,8}", 0..4),
        proptest::option::of("[A-Z_0-9]{0,8}"),
    )
        .prop_map(|(definitions, excluded, current)| ExcludeState {
            definitions,
            excluded,
            current,
        })
}

fn any_context() -> impl Strategy<Value = Context> {
    (
        any::<u64>(),
        proptest::option::of((any::<u64>(), "[a-z/.]{0,12}")),
        any_gcode_state(),
        any_f64(),
        proptest::option::of(any_exclude_state().prop_map(Box::new)),
        // Verbatim field: totality must hold for any reported state.
        proptest::option::of(".{0,12}"),
        // Includes NaN/infinity: the coverage certificate compares against
        // this, and a non-finite value must not panic or certify.
        proptest::option::of(any_f64()),
    )
        .prop_map(
            |(mono_ns, vsd, gcode, target, exclude, print_state, print_time)| Context {
                mono_ns,
                print_state,
                print_time,
                virtual_sdcard: vsd.map(|(file_position, file_path)| VirtualSdState {
                    file_path,
                    file_position,
                    file_size: None,
                }),
                gcode,
                transforms: TransformObservations {
                    bed_mesh_active: false,
                    bed_mesh_profile: None,
                    z_thermal_adjust_enabled: Some(true),
                    z_thermal_adjust_offset: Some(target),
                    skew_active: false,
                    skew_profile: None,
                },
                heaters: vec![HeaterTarget {
                    name: "extruder".to_owned(),
                    target,
                }],
                fans: vec![FanTarget {
                    name: "fan".to_owned(),
                    speed: target,
                }],
                exclude,
                current_layer: None,
                total_layer: None,
            },
        )
}

fn any_marker() -> impl Strategy<Value = Marker> {
    (
        any::<u64>(),
        prop_oneof![
            Just(MarkerKind::CleanShutdown),
            Just(MarkerKind::SocketLost),
            Just(MarkerKind::Resubscribed),
            (any::<u64>(), any::<u64>()).prop_map(|(s, e)| MarkerKind::SubscriptionGap {
                start_mono_ns: s,
                end_mono_ns: e,
            }),
            Just(MarkerKind::Unknown),
        ],
    )
        .prop_map(|(mono_ns, kind)| Marker { mono_ns, kind })
}

fn any_heartbeat() -> impl Strategy<Value = Heartbeat> {
    (
        any::<u64>(),
        any::<u64>(),
        any::<u64>(),
        any_f64(),
        any_f64(),
    )
        .prop_map(
            |(sequence, mono_ns, est_sample_mono_ns, print_time, est)| Heartbeat {
                sequence,
                mono_ns,
                wall_ns: 0,
                print_time,
                est_sample_mono_ns,
                est_sample_print_time: est,
                wal_offset: 0,
            },
        )
}

fn any_record() -> impl Strategy<Value = WalRecord> {
    prop_oneof![
        3 => any_trapq().prop_map(WalRecord::TrapqSegment),
        2 => any_stepper().prop_map(WalRecord::StepperRange),
        2 => any_context().prop_map(WalRecord::Context),
        1 => any_marker().prop_map(WalRecord::Marker),
        2 => any_heartbeat().prop_map(WalRecord::Heartbeat),
    ]
}

fn any_scan_end() -> impl Strategy<Value = ScanEnd> {
    prop_oneof![
        Just(ScanEnd::CleanEof),
        Just(ScanEnd::TruncatedFrameHeader),
        Just(ScanEnd::TruncatedPayload),
        Just(ScanEnd::BadFrameMagic),
        Just(ScanEnd::FrameCrcMismatch),
        Just(ScanEnd::SegmentHeaderCrcMismatch),
    ]
}

fn any_config() -> impl Strategy<Value = ReconstructConfig> {
    (
        proptest::option::of(any_f64()),
        any_f64(),
        any_f64(),
        any_f64(),
        proptest::collection::vec(any_f64(), 3),
    )
        .prop_map(|(mcu_freq, lead, horizon, tol, sim)| ReconstructConfig {
            mcu_freq,
            z_stepper_prefix: "stepper_z".to_owned(),
            step_gen_lead: lead,
            quiet_tail_ns: 2_000_000_000,
            max_processing_lead: lead,
            extension_horizon: horizon,
            exclusion_freshness_horizon: horizon,
            // Hostile but in-domain: 0 would fail validation, so the
            // period is any positive tick count and the tolerance any
            // multiplier >= 1.
            heartbeat_period_ns: 1,
            heartbeat_gap_tolerance: 1.0,
            z_merge_tolerance: tol,
            durability_lag_ns: 500_000_000,
            sim: SimConfig {
                max_velocity: sim[0],
                max_accel: sim[1],
                square_corner_velocity: sim[2],
                max_duration: Some(horizon),
                max_lines: Some(500),
            },
        })
}

proptest! {
    // Persist shrunk counterexamples to the checked-in file
    // tests/totality.proptest-regressions (case count stays
    // env-overridable via ProptestConfig::default() / PROPTEST_CASES).
    // The SourceParallel default cannot locate lib.rs/main.rs from an
    // integration test and only works via a warning-emitting fallback;
    // WithSource pins the exact same path explicitly.
    #![proptest_config(ProptestConfig {
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::default()
    })]

    /// `reconstruct` is total: for any records (hostile floats
    /// included), any scan end, any heartbeat-file bytes, any file
    /// tail, and any configuration, it returns a `Result` — it never
    /// panics.
    #[test]
    fn reconstruct_never_panics(
        records in proptest::collection::vec(any_record(), 0..24),
        end in any_scan_end(),
        hb_bytes in proptest::collection::vec(any::<u8>(), 0..160),
        tail in proptest::collection::vec(any::<u8>(), 0..512),
        base_offset in any::<u64>(),
        seq in proptest::option::of((any::<u64>(), any::<u64>())),
        config in any_config(),
    ) {
        let scan = RecoveryScan {
            header: None,
            records: records
                .into_iter()
                .enumerate()
                .map(|(i, record)| ScannedRecord {
                    offset: i as u64 * 97,
                    record,
                })
                .collect(),
            truncation_offset: 0,
            end,
        };
        let heartbeat = recover_heartbeat(&hb_bytes).ok();
        let inputs = ReconstructInputs {
            scan: &scan,
            heartbeat: heartbeat.as_ref(),
            file_tail: Some(FileTail {
                base_offset,
                bytes: &tail,
            }),
            receive_seq: seq.map(|(mono_ns, widened_seq)| ReceiveSeqObservation {
                mono_ns,
                widened_seq,
            }),
            power_fail_edge_mono_ns: None,
        };
        // Any Result is acceptable; a panic fails the test.
        let _ = reconstruct(&inputs, &config);
        // The default config must also never panic on hostile records.
        let _ = reconstruct(&inputs, &ReconstructConfig::default());
    }

    /// The exclusion resolver is total: arbitrary journaled exclude
    /// state (NaN/infinite polygons, degenerate rings, contradictory
    /// fidelity flags) and arbitrary print-file bytes never panic, and
    /// point-in-object lookup answers for arbitrary query points.
    #[test]
    fn exclusion_resolution_never_panics(
        records in proptest::collection::vec(any_record(), 0..12),
        end in any_scan_end(),
        tail in proptest::collection::vec(any::<u8>(), 0..512),
        base_offset in prop_oneof![Just(0_u64), any::<u64>()],
        x in any_f64(),
        y in any_f64(),
        config in any_config(),
    ) {
        let scan = RecoveryScan {
            header: None,
            records: records
                .into_iter()
                .enumerate()
                .map(|(i, record)| ScannedRecord {
                    offset: i as u64 * 97,
                    record,
                })
                .collect(),
            truncation_offset: 0,
            end,
        };
        let timeline = ingest(&scan, None);
        let file = FileTail { base_offset, bytes: &tail };
        // The window is itself derived from hostile records, so it may
        // legitimately fail to compute; both paths must be total.
        let window = compute_stop_window(&timeline, None, &config).ok();
        for file in [None, Some(&file)] {
            for window in [None, window.as_ref()] {
                let inputs = ExclusionInputs {
                    window,
                    stop_end_print_time: Some(x),
                    file,
                };
                let report = resolve_exclusions(&timeline, &inputs, &config);
                // Every query is total and self-consistent.
                let at = report.objects_at(x, y);
                prop_assert!(at.len() <= report.definitions().len());
                prop_assert_eq!(report.object_at(x, y).is_some(), !at.is_empty());
                if let Some(hit) = report.excluded_object_at(x, y) {
                    prop_assert!(report.is_excluded(&hit.name));
                }
                // Conclusiveness and the confirmation payload agree, and
                // a prompt always carries every known object.
                prop_assert_eq!(
                    report.is_conclusive(),
                    report.confirmation().is_none()
                );
                if let Some(confirmation) = report.confirmation() {
                    prop_assert!(!confirmation.causes.is_empty());
                    prop_assert_eq!(
                        confirmation.objects.len(),
                        report.object_states().len()
                    );
                    // Every recorded exclusion is pre-selected.
                    for name in report.excluded() {
                        prop_assert!(confirmation
                            .objects
                            .iter()
                            .any(|o| o.name.eq_ignore_ascii_case(name) && o.preselected()));
                    }
                }
                // The structural invariant: exclusions only ever come
                // from a journaled log.
                if !report.excluded().is_empty() {
                    prop_assert_eq!(
                        report.provenance(),
                        plr_reconstruct::ExclusionProvenance::Journaled
                    );
                }
                let _ = report.geometry_is_complete();
                let _ = report.excluded_definitions();
                let _ = report.freshness();
            }
        }
    }

    /// Point-in-polygon is total over arbitrary rings and query points,
    /// and never claims containment for a non-finite input.
    #[test]
    fn point_in_polygon_never_panics(
        polygon in any_polygon(),
        x in any_f64(),
        y in any_f64(),
    ) {
        let inside = point_in_polygon(x, y, &polygon);
        if !x.is_finite() || !y.is_finite() || polygon.len() < 3 {
            prop_assert!(!inside);
        }
    }

    /// The print-file object scan is total over arbitrary bytes.
    #[test]
    fn object_definition_scan_never_panics(
        bytes in proptest::collection::vec(any::<u8>(), 0..512),
    ) {
        let scan = parse_object_definitions(&bytes);
        prop_assert_eq!(scan.is_empty(), scan.definitions.is_empty() && scan.unparsed_lines == 0);
        prop_assert_eq!(scan.names().len(), scan.definitions.len());
    }
}
