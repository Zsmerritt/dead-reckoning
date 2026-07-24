//! Property tests for the WAL formats.
//!
//! The four core properties from the design brief:
//!
//! 1. encode/decode round-trip for arbitrary record values;
//! 2. the decoder never panics on arbitrary byte vectors;
//! 3. prefix property: truncating a valid multi-record log at ANY byte
//!    boundary yields a decoded record list that is a prefix of the full
//!    list — never invented records;
//! 4. single-bit corruption anywhere in a record is detected: the record
//!    is rejected and the prefix before it is preserved.
//!
//! Plus the heartbeat mirror of (1), (2), and (4).

use plr_wal::frame::{FRAME_HEADER_LEN, SEGMENT_HEADER_LEN};
use plr_wal::heartbeat::{HEARTBEAT_FILE_LEN, HEARTBEAT_SLOT_LEN};
use plr_wal::{
    decode_slot, encode_slot, recover_heartbeat, scan, slot_for_sequence, Context, FanTarget,
    GcodeState, Heartbeat, HeaterTarget, Marker, MarkerKind, ScanEnd, SegmentHeader, SlotError,
    SlotId, StepChunk, StepperRange, TransformObservations, TrapqSegment, VirtualSdState,
    WalRecord, WalWriter,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// Finite floats only: the JSON payload encoding refuses NaN/infinity by
/// design (the writer rejects them), so the round-trip domain is finite.
fn finite_f64() -> impl Strategy<Value = f64> {
    prop_oneof![Just(0.0), Just(-0.0), -1.0e12..1.0e12_f64]
}

prop_compose! {
    fn arb_trapq()(
        mono_ns in any::<u64>(),
        queue in ".{0,12}",
        f in prop::array::uniform10(finite_f64()),
    ) -> TrapqSegment {
        TrapqSegment {
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
        }
    }
}

prop_compose! {
    fn arb_step_chunk()(
        interval in any::<u32>(),
        count in any::<u16>(),
        add in any::<i16>(),
    ) -> StepChunk {
        StepChunk { interval, count, add }
    }
}

prop_compose! {
    fn arb_stepper()(
        mono_ns in any::<u64>(),
        stepper in ".{0,12}",
        first_clock in any::<u64>(),
        last_clock in any::<u64>(),
        f in prop::array::uniform4(finite_f64()),
        start_mcu_position in any::<i64>(),
        steps in prop::collection::vec(arb_step_chunk(), 0..16),
    ) -> StepperRange {
        StepperRange {
            mono_ns,
            stepper,
            first_clock,
            last_clock,
            first_step_time: f[0],
            last_step_time: f[1],
            start_position: f[2],
            start_mcu_position,
            step_distance: f[3],
            steps,
        }
    }
}

prop_compose! {
    fn arb_gcode()(
        speed_factor in finite_f64(),
        speed in finite_f64(),
        extrude_factor in finite_f64(),
        absolute_coordinates in any::<bool>(),
        absolute_extrude in any::<bool>(),
        homing_origin in prop::collection::vec(finite_f64(), 0..6),
        position in prop::collection::vec(finite_f64(), 0..6),
        gcode_position in prop::collection::vec(finite_f64(), 0..6),
    ) -> GcodeState {
        GcodeState {
            speed_factor,
            speed,
            extrude_factor,
            absolute_coordinates,
            absolute_extrude,
            homing_origin,
            position,
            gcode_position,
        }
    }
}

prop_compose! {
    fn arb_transforms()(
        bed_mesh_active in any::<bool>(),
        bed_mesh_profile in prop::option::of(".{0,8}"),
        z_thermal_adjust_enabled in prop::option::of(any::<bool>()),
        z_thermal_adjust_offset in prop::option::of(finite_f64()),
        skew_active in any::<bool>(),
        skew_profile in prop::option::of(".{0,8}"),
    ) -> TransformObservations {
        TransformObservations {
            bed_mesh_active,
            bed_mesh_profile,
            z_thermal_adjust_enabled,
            z_thermal_adjust_offset,
            skew_active,
            skew_profile,
        }
    }
}

prop_compose! {
    fn arb_context()(
        mono_ns in any::<u64>(),
        virtual_sdcard in prop::option::of(
            (".{0,20}", any::<u64>()).prop_map(|(file_path, file_position)| VirtualSdState {
                file_path,
                file_position,
            })
        ),
        gcode in arb_gcode(),
        transforms in arb_transforms(),
        heaters in prop::collection::vec(
            (".{0,10}", finite_f64()).prop_map(|(name, target)| HeaterTarget { name, target }),
            0..4,
        ),
        fans in prop::collection::vec(
            (".{0,10}", finite_f64()).prop_map(|(name, speed)| FanTarget { name, speed }),
            0..4,
        ),
    ) -> Context {
        Context { mono_ns, virtual_sdcard, gcode, transforms, heaters, fans }
    }
}

fn arb_marker_kind() -> impl Strategy<Value = MarkerKind> {
    prop_oneof![
        Just(MarkerKind::CleanShutdown),
        Just(MarkerKind::SocketLost),
        Just(MarkerKind::Resubscribed),
        (any::<u64>(), any::<u64>()).prop_map(|(start_mono_ns, end_mono_ns)| {
            MarkerKind::SubscriptionGap {
                start_mono_ns,
                end_mono_ns,
            }
        }),
        Just(MarkerKind::Unknown),
    ]
}

prop_compose! {
    fn arb_marker()(mono_ns in any::<u64>(), kind in arb_marker_kind()) -> Marker {
        Marker { mono_ns, kind }
    }
}

prop_compose! {
    fn arb_heartbeat_finite()(
        sequence in any::<u64>(),
        mono_ns in any::<u64>(),
        wall_ns in any::<u64>(),
        print_time in finite_f64(),
        est_sample_mono_ns in any::<u64>(),
        est_sample_print_time in finite_f64(),
        wal_offset in any::<u64>(),
    ) -> Heartbeat {
        Heartbeat {
            sequence,
            mono_ns,
            wall_ns,
            print_time,
            est_sample_mono_ns,
            est_sample_print_time,
            wal_offset,
        }
    }
}

prop_compose! {
    /// Heartbeats with fully arbitrary float bits (NaN and infinities
    /// included): the binary slot encoding must round-trip them
    /// bit-exactly even though the JSON path refuses them.
    fn arb_heartbeat_any_bits()(
        sequence in any::<u64>(),
        mono_ns in any::<u64>(),
        wall_ns in any::<u64>(),
        print_time_bits in any::<u64>(),
        est_sample_mono_ns in any::<u64>(),
        est_sample_print_time_bits in any::<u64>(),
        wal_offset in any::<u64>(),
    ) -> Heartbeat {
        Heartbeat {
            sequence,
            mono_ns,
            wall_ns,
            print_time: f64::from_bits(print_time_bits),
            est_sample_mono_ns,
            est_sample_print_time: f64::from_bits(est_sample_print_time_bits),
            wal_offset,
        }
    }
}

fn arb_record() -> impl Strategy<Value = WalRecord> {
    prop_oneof![
        arb_trapq().prop_map(WalRecord::TrapqSegment),
        arb_stepper().prop_map(WalRecord::StepperRange),
        arb_context().prop_map(WalRecord::Context),
        arb_marker().prop_map(WalRecord::Marker),
        arb_heartbeat_finite().prop_map(WalRecord::Heartbeat),
    ]
}

/// Builds a log and returns `(bytes, boundaries)` where `boundaries[i]`
/// is the start offset of frame `i` and the final element is the end of
/// the log.
fn build_log(records: &[WalRecord]) -> (Vec<u8>, Vec<u64>) {
    let header = SegmentHeader::new(11, 22);
    let mut writer = WalWriter::create(Vec::new(), &header).unwrap();
    let mut boundaries: Vec<u64> = records.iter().map(|r| writer.append(r).unwrap()).collect();
    boundaries.push(writer.offset());
    (writer.into_inner(), boundaries)
}

fn decoded_records(result: &plr_wal::RecoveryScan) -> Vec<WalRecord> {
    result.records.iter().map(|r| r.record.clone()).collect()
}

/// Field-wise equality with bit-exact float comparison (survives NaN).
fn heartbeat_bits_equal(a: &Heartbeat, b: &Heartbeat) -> bool {
    a.sequence == b.sequence
        && a.mono_ns == b.mono_ns
        && a.wall_ns == b.wall_ns
        && a.print_time.to_bits() == b.print_time.to_bits()
        && a.est_sample_mono_ns == b.est_sample_mono_ns
        && a.est_sample_print_time.to_bits() == b.est_sample_print_time.to_bits()
        && a.wal_offset == b.wal_offset
}

proptest! {
    // Persist shrunk counterexamples to the checked-in file
    // tests/proptests.proptest-regressions. The default
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

    // Property 1: encode/decode round-trip for arbitrary record values.
    #[test]
    fn arbitrary_records_roundtrip_through_writer_and_scan(
        records in prop::collection::vec(arb_record(), 0..5)
    ) {
        let (bytes, boundaries) = build_log(&records);
        let result = scan(&bytes);
        prop_assert_eq!(&result.end, &ScanEnd::CleanEof);
        prop_assert_eq!(decoded_records(&result), records);
        prop_assert_eq!(result.truncation_offset, bytes.len() as u64);
        let scanned_offsets: Vec<u64> = result.records.iter().map(|r| r.offset).collect();
        prop_assert_eq!(scanned_offsets, boundaries[..boundaries.len() - 1].to_vec());
    }

    // Property 2: the decoders never panic on arbitrary bytes.
    #[test]
    fn decoders_never_panic_on_arbitrary_bytes(
        bytes in prop::collection::vec(any::<u8>(), 0..512)
    ) {
        let _ = scan(&bytes);
        let _ = recover_heartbeat(&bytes);
        let _ = decode_slot(&bytes);
        let _ = SegmentHeader::decode(&bytes);
    }

    // Property 2, deeper: a valid segment header followed by garbage
    // exercises the frame parser itself on arbitrary bytes.
    #[test]
    fn frame_parser_never_panics_after_valid_header(
        tail in prop::collection::vec(any::<u8>(), 0..384)
    ) {
        let mut bytes = SegmentHeader::new(3, 4).encode().to_vec();
        bytes.extend_from_slice(&tail);
        let result = scan(&bytes);
        prop_assert!(result.header.is_some());
        prop_assert!(result.truncation_offset >= SEGMENT_HEADER_LEN as u64);
    }

    // Property 3: truncating at ANY byte yields a strict prefix, with
    // the exact truncation offset and a tear-shaped reason.
    #[test]
    fn truncation_at_any_byte_yields_a_prefix(
        (records, cut) in prop::collection::vec(arb_record(), 1..4)
            .prop_flat_map(|records| {
                let (bytes, _) = build_log(&records);
                let len = bytes.len();
                (Just(records), 0..=len)
            })
    ) {
        let (bytes, boundaries) = build_log(&records);
        let result = scan(&bytes[..cut]);
        // Number of frames fully contained in the first `cut` bytes.
        let complete = boundaries[1..]
            .iter()
            .filter(|end| usize::try_from(**end).unwrap() <= cut)
            .count();
        prop_assert_eq!(decoded_records(&result), records[..complete].to_vec());
        if cut < SEGMENT_HEADER_LEN {
            prop_assert_eq!(result.truncation_offset, 0);
            prop_assert_eq!(&result.end, &ScanEnd::TruncatedSegmentHeader { len: cut });
        } else {
            prop_assert_eq!(result.truncation_offset, boundaries[complete]);
            let into_frame = cut - usize::try_from(boundaries[complete]).unwrap();
            let expected = if into_frame == 0 {
                ScanEnd::CleanEof
            } else if into_frame < FRAME_HEADER_LEN {
                ScanEnd::TruncatedFrameHeader
            } else {
                ScanEnd::TruncatedPayload
            };
            prop_assert_eq!(&result.end, &expected);
        }
        prop_assert!(result.end.is_expected_after_power_loss());
    }

    // Property 4: flipping any single bit anywhere in the log is
    // detected — the touched frame is rejected, the prefix before it is
    // preserved intact, and nothing after it is invented.
    //
    // For flips inside a frame's CRC-covered bytes or its stored CRC
    // this is a mathematical guarantee (a CRC detects all single-bit
    // errors). A flip in the length field makes the parser re-frame and
    // compare a CRC read from the wrong place, where detection is
    // "only" overwhelmingly probable (~2^-32 per case); with 256
    // deterministic cases this test cannot realistically observe a
    // collision.
    #[test]
    fn single_bit_corruption_is_detected_and_prefix_preserved(
        (records, byte_index, bit) in prop::collection::vec(arb_record(), 1..4)
            .prop_flat_map(|records| {
                let (bytes, _) = build_log(&records);
                let len = bytes.len();
                (Just(records), 0..len, 0..8_u8)
            })
    ) {
        let (mut bytes, boundaries) = build_log(&records);
        bytes[byte_index] ^= 1_u8 << bit;
        let result = scan(&bytes);
        if byte_index < SEGMENT_HEADER_LEN {
            prop_assert!(result.header.is_none());
            prop_assert!(result.records.is_empty());
            prop_assert_eq!(result.truncation_offset, 0);
        } else {
            let hit = (0..records.len())
                .find(|i| byte_index < usize::try_from(boundaries[i + 1]).unwrap())
                .unwrap();
            prop_assert_eq!(decoded_records(&result), records[..hit].to_vec());
            prop_assert_eq!(result.truncation_offset, boundaries[hit]);
            prop_assert!(result.end != ScanEnd::CleanEof);
        }
    }

    // Heartbeat mirror of property 1, over ALL float bit patterns.
    #[test]
    fn heartbeat_slots_roundtrip_bit_exactly(hb in arb_heartbeat_any_bits()) {
        let decoded = decode_slot(&encode_slot(&hb)).unwrap();
        prop_assert!(heartbeat_bits_equal(&decoded, &hb));
    }

    // Heartbeat mirror of property 4: a single-bit flip anywhere in the
    // file destroys at most the slot it landed in; recovery returns the
    // other slot and reports the tear.
    #[test]
    fn heartbeat_single_bit_corruption_falls_back_to_other_slot(
        hb in arb_heartbeat_any_bits(),
        byte_index in 0..HEARTBEAT_FILE_LEN,
        bit in 0..8_u8,
    ) {
        let next = Heartbeat { sequence: hb.sequence.wrapping_add(1), ..hb };
        let (slot_a, slot_b) = if slot_for_sequence(hb.sequence) == SlotId::A {
            (hb, next)
        } else {
            (next, hb)
        };
        let mut file = Vec::with_capacity(HEARTBEAT_FILE_LEN);
        file.extend_from_slice(&encode_slot(&slot_a));
        file.extend_from_slice(&encode_slot(&slot_b));
        file[byte_index] ^= 1_u8 << bit;

        let corrupted = if byte_index < HEARTBEAT_SLOT_LEN { SlotId::A } else { SlotId::B };
        let survivor = if corrupted == SlotId::A { slot_b } else { slot_a };
        let recovery = recover_heartbeat(&file).unwrap();
        prop_assert_eq!(recovery.slot, corrupted.other());
        prop_assert!(heartbeat_bits_equal(&recovery.heartbeat, &survivor));
        // Single-bit flips always land inside the CRC-covered bytes or
        // the stored CRC, so the tear is always a CRC mismatch.
        prop_assert_eq!(recovery.torn, Some((corrupted, SlotError::CrcMismatch)));
    }

    // Heartbeat mirror of property 2 for two-slot recovery with one
    // valid slot present: recovery never panics and never invents a
    // heartbeat that fails validation.
    #[test]
    fn heartbeat_recovery_with_garbage_second_slot_never_panics(
        hb in arb_heartbeat_any_bits(),
        garbage in prop::collection::vec(any::<u8>(), 0..=HEARTBEAT_SLOT_LEN),
    ) {
        let mut file = encode_slot(&hb).to_vec();
        file.extend_from_slice(&garbage);
        let recovery = recover_heartbeat(&file).unwrap();
        // Slot A is intact, so recovery must never lose it.
        if recovery.slot == SlotId::A {
            prop_assert!(heartbeat_bits_equal(&recovery.heartbeat, &hb));
        } else {
            // Garbage that validated must genuinely decode as slot B.
            let decoded = decode_slot(&garbage).unwrap();
            prop_assert!(heartbeat_bits_equal(&recovery.heartbeat, &decoded));
        }
    }
}
