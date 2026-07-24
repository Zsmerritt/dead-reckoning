//! Totality property tests: reconstruction never panics, for any input —
//! including hand-built scans full of NaN/infinite floats, hostile
//! orderings, arbitrary scan ends, arbitrary heartbeat files, arbitrary
//! file-tail bytes, and out-of-domain configurations. Errors are fine;
//! panics are not.

use plr_gcode::SimConfig;
use plr_reconstruct::{
    reconstruct, FileTail, ReceiveSeqObservation, ReconstructConfig, ReconstructInputs,
};
use plr_wal::{
    recover_heartbeat, Context, FanTarget, GcodeState, Heartbeat, HeaterTarget, Marker, MarkerKind,
    RecoveryScan, ScanEnd, ScannedRecord, StepChunk, StepperRange, TransformObservations,
    TrapqSegment, VirtualSdState, WalRecord,
};
use proptest::prelude::*;

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
        proptest::collection::vec((any::<u32>(), any::<u16>(), any::<i16>()), 0..3),
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

fn any_context() -> impl Strategy<Value = Context> {
    (
        any::<u64>(),
        proptest::option::of((any::<u64>(), "[a-z/.]{0,12}")),
        any_gcode_state(),
        any_f64(),
    )
        .prop_map(|(mono_ns, vsd, gcode, target)| Context {
            mono_ns,
            virtual_sdcard: vsd.map(|(file_position, file_path)| VirtualSdState {
                file_path,
                file_position,
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
        })
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
            z_merge_tolerance: tol,
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
        };
        // Any Result is acceptable; a panic fails the test.
        let _ = reconstruct(&inputs, &config);
        // The default config must also never panic on hostile records.
        let _ = reconstruct(&inputs, &ReconstructConfig::default());
    }
}
