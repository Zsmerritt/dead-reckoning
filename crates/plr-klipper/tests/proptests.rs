//! Property tests: totality and invariants over adversarial inputs.

use plr_klipper::{
    classify, ClockCorrelator, FrameEvent, FrameSplitter, ReceiveSeqWidener, SeqKind,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// Splits `bytes` into `pieces` chunks at the given fractional cut
/// points, preserving order and content.
fn chunkings(bytes: &[u8], cuts: &[usize]) -> Vec<Vec<u8>> {
    let mut result = Vec::new();
    let mut start = 0;
    let mut cut_points: Vec<usize> = cuts.iter().map(|&c| c % (bytes.len() + 1)).collect();
    cut_points.sort_unstable();
    for cut in cut_points {
        if cut > start {
            result.push(bytes[start..cut].to_vec());
        }
        start = start.max(cut);
    }
    result.push(bytes[start..].to_vec());
    result
}

fn feed_all(splitter: &mut FrameSplitter, chunks: &[Vec<u8>]) -> Vec<FrameEvent> {
    let mut events = Vec::new();
    for chunk in chunks {
        events.extend(splitter.feed(chunk));
    }
    events
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

    /// The splitter is total: arbitrary bytes in arbitrary chunkings
    /// never panic, and buffered data respects the cap.
    #[test]
    fn splitter_total_on_arbitrary_bytes(
        bytes in proptest::collection::vec(any::<u8>(), 0..2048),
        cuts in proptest::collection::vec(any::<usize>(), 0..16),
        cap in 0_usize..64,
    ) {
        let mut splitter = FrameSplitter::with_max_frame_len(cap);
        for chunk in chunkings(&bytes, &cuts) {
            splitter.feed(&chunk);
            prop_assert!(splitter.pending_len() <= cap);
        }
    }

    /// Reassembly: any chunking of the same byte stream yields exactly
    /// the same event sequence as feeding it whole.
    #[test]
    fn splitter_chunking_independent(
        bytes in proptest::collection::vec(any::<u8>(), 0..2048),
        cuts in proptest::collection::vec(any::<usize>(), 0..16),
        cap in prop_oneof![Just(usize::MAX), (0_usize..64)],
    ) {
        let mut whole = FrameSplitter::with_max_frame_len(cap);
        let expected = whole.feed(&bytes);
        let mut chunked = FrameSplitter::with_max_frame_len(cap);
        let got = feed_all(&mut chunked, &chunkings(&bytes, &cuts));
        prop_assert_eq!(got, expected);
        prop_assert_eq!(chunked.pending_len(), whole.pending_len());
        prop_assert_eq!(chunked.is_discarding(), whole.is_discarding());
    }

    /// A stream of valid frames survives any chunking: every frame body
    /// comes back exactly once, in order (frame bodies here contain no
    /// ETX byte by construction).
    #[test]
    fn splitter_reassembles_valid_frame_streams(
        frames in proptest::collection::vec(
            proptest::collection::vec(any::<u8>().prop_filter("no ETX", |&b| b != 0x03), 1..64),
            0..16,
        ),
        cuts in proptest::collection::vec(any::<usize>(), 0..16),
    ) {
        let mut stream = Vec::new();
        for frame in &frames {
            stream.extend_from_slice(frame);
            stream.push(0x03);
        }
        let mut splitter = FrameSplitter::new();
        let events = feed_all(&mut splitter, &chunkings(&stream, &cuts));
        let got: Vec<Vec<u8>> = events
            .into_iter()
            .map(|e| match e {
                FrameEvent::Frame(f) => f,
                FrameEvent::Oversized { .. } => panic!("no frame here exceeds the cap"),
            })
            .collect();
        prop_assert_eq!(got, frames);
        prop_assert_eq!(splitter.pending_len(), 0);
    }

    /// The classifier is total on arbitrary bytes.
    #[test]
    fn classify_total_on_arbitrary_bytes(bytes in proptest::collection::vec(any::<u8>(), 0..512)) {
        let _ = classify(&bytes);
    }

    /// The classifier is total on arbitrary *JSON* (worst case for the
    /// object-shape paths), and typed payload extraction never panics.
    #[test]
    fn classify_total_on_arbitrary_json(value in arbitrary_json(4)) {
        let bytes = serde_json::to_vec(&value).expect("json serializes");
        if let Ok(plr_klipper::Inbound::Notification(n)) = classify(&bytes) {
            let _ = n.status_update();
            let _ = n.trapq_batch();
            let _ = n.stepper_batch();
            let _ = n.gcode_output();
        }
    }

    /// The correlator is total on arbitrary float samples and queries,
    /// and outputs are always finite.
    #[test]
    fn correlator_total_on_arbitrary_floats(
        samples in proptest::collection::vec((arbitrary_f64(), arbitrary_f64()), 0..32),
        queries in proptest::collection::vec(arbitrary_f64(), 0..8),
    ) {
        let mut c = ClockCorrelator::new();
        for (e, p) in samples {
            let _ = c.add_sample(e, p);
            if let Some((se, sp)) = c.latest_sample() {
                prop_assert!(se.is_finite() && sp.is_finite());
            }
        }
        for q in queries {
            if let Some(v) = c.print_time_to_eventtime(q) {
                prop_assert!(v.is_finite());
            }
            if let Some(v) = c.eventtime_to_print_time(q) {
                prop_assert!(v.is_finite());
            }
        }
    }

    /// Round trip: for reasonable magnitudes, converting a print_time to
    /// eventtime and back returns the original within float tolerance.
    #[test]
    fn correlator_round_trip(
        e in -1.0e9_f64..1.0e9,
        p in -1.0e9_f64..1.0e9,
        q in -1.0e9_f64..1.0e9,
    ) {
        let mut c = ClockCorrelator::new();
        prop_assert_eq!(c.add_sample(e, p), plr_klipper::SampleOutcome::Accepted);
        let host = c.print_time_to_eventtime(q).expect("finite");
        let back = c.eventtime_to_print_time(host).expect("finite");
        prop_assert!((back - q).abs() <= 1.0e-6 * q.abs().max(1.0));
    }

    /// The widener exactly reconstructs any true u64 counter sequence
    /// whose per-observation advances stay below 2^31, observed through
    /// the 32-bit truncation — including across wraps.
    #[test]
    fn widener_reconstructs_wrapped_sequences(
        start in any::<u64>(),
        steps in proptest::collection::vec(0_u32..u32::MAX / 2, 1..64),
    ) {
        let mut w = ReceiveSeqWidener::new();
        let mut truth = start;
        let first = w.observe(truth & 0xffff_ffff).widened;
        let anchor = truth;
        for step in steps {
            truth = truth.wrapping_add(u64::from(step));
            let update = w.observe(truth & 0xffff_ffff);
            // Widened value tracks the truth exactly, relative to the
            // first observation.
            prop_assert_eq!(update.widened - first, truth.wrapping_sub(anchor));
            match update.kind {
                SeqKind::Advanced { delta } => prop_assert_eq!(delta, step),
                SeqKind::Unchanged => prop_assert_eq!(step, 0),
                other => prop_assert!(false, "unexpected kind {other:?}"),
            }
        }
    }

    /// The widener output is non-decreasing for arbitrary raw inputs,
    /// including hostile regressions.
    #[test]
    fn widener_monotonic_on_arbitrary_inputs(
        raws in proptest::collection::vec(any::<u64>(), 1..128),
    ) {
        let mut w = ReceiveSeqWidener::new();
        let mut last = 0_u64;
        for raw in raws {
            let update = w.observe(raw);
            prop_assert!(update.widened >= last);
            prop_assert_eq!(w.current(), Some(update.widened));
            last = update.widened;
        }
    }
}

/// Any f64 bit pattern: covers NaN, infinities, subnormals.
fn arbitrary_f64() -> impl Strategy<Value = f64> {
    any::<u64>().prop_map(f64::from_bits)
}

/// Arbitrary JSON values of bounded depth.
fn arbitrary_json(depth: u32) -> impl Strategy<Value = serde_json::Value> {
    let leaf = prop_oneof![
        Just(serde_json::Value::Null),
        any::<bool>().prop_map(serde_json::Value::from),
        any::<i64>().prop_map(serde_json::Value::from),
        (-1.0e15_f64..1.0e15).prop_map(serde_json::Value::from),
        "[a-zA-Z0-9 ]{0,12}".prop_map(serde_json::Value::from),
    ];
    leaf.prop_recursive(depth, 64, 8, |inner| {
        prop_oneof![
            proptest::collection::vec(inner.clone(), 0..8).prop_map(serde_json::Value::from),
            proptest::collection::btree_map("[a-z]{0,6}", inner, 0..8)
                .prop_map(|m| { serde_json::Value::Object(m.into_iter().collect()) }),
        ]
    })
}
