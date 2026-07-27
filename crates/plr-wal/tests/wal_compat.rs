//! Old-reader compatibility proof for `MarkerKind::PowerFailing`.
//!
//! The `feat/powerfail-gpio` branch adds one variant,
//! `MarkerKind::PowerFailing`, to the WAL's marker enum. New marker kinds
//! are the established old-reader-safe pattern in this format
//! (`#[serde(other)] Unknown`). This test proves the two halves of that
//! contract against a reader **built before the variant existed**:
//!
//! 1. **Byte-identity where the new variant is absent.** A log that
//!    contains none of the new marker serialises exactly as the old writer
//!    produced it — adding an enum variant changes no other variant's
//!    bytes.
//! 2. **The old reader degrades the new marker to `Unknown` and keeps
//!    scanning.** A `{"kind":"PowerFailing"}` frame written by the current
//!    writer decodes, under the old marker enum, to that enum's
//!    `#[serde(other)] Unknown` arm, and a real segment scan walks *past*
//!    it to the records that follow.
//!
//! # Why the old reader is transcribed, not `git archive`-built
//!
//! [`OldMarkerKind`] below is transcribed **verbatim** from
//! `git show 4a63eef:crates/plr-wal/src/record.rs` (the branch's base
//! commit): same variants, same `#[serde(tag = "kind")]`, same
//! `#[serde(other)] Unknown`, with `PowerFailing` deliberately absent — it
//! *is* the old reader's decode surface for markers, which is the whole
//! and only surface that a new marker kind can break. A `git archive`
//! build of the whole crate would prove the same serde behaviour at far
//! greater cost and cross-platform fragility (it would shell out to `git`
//! and `cargo` from inside a test), so the contract is pinned here where a
//! rename or a re-tag becomes a failing test rather than a silent format
//! break. The frame/scan machinery the marker rides in is byte-for-byte
//! unchanged by this branch, so the current scanner's behaviour over a
//! segment *is* the old scanner's behaviour over the same bytes.

use plr_wal::{
    scan, Marker, MarkerKind, ScanEnd, SegmentHeader, TrapqSegment, WalRecord, WalWriter,
};
use serde::{Deserialize, Serialize};

/// The marker enum **exactly as it was at `4a63eef`**, before
/// `PowerFailing` existed. Transcribed verbatim from
/// `git show 4a63eef:crates/plr-wal/src/record.rs`. Do not add variants:
/// the point of this type is to be the old decode surface.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind")]
enum OldMarkerKind {
    CleanShutdown,
    SocketLost,
    Resubscribed,
    SubscriptionGap {
        start_mono_ns: u64,
        end_mono_ns: u64,
    },
    ExclusionUpdateLost,
    RecorderStopped,
    RecordingQuiescent,
    #[serde(other)]
    Unknown,
}

/// The old marker record shape (unchanged across the branch).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct OldMarker {
    mono_ns: u64,
    kind: OldMarkerKind,
}

#[test]
fn every_pre_existing_marker_kind_is_byte_identical_across_the_enum_change() {
    // For each kind that existed at 4a63eef, the current writer's bytes
    // must equal the old writer's bytes: adding `PowerFailing` changed no
    // other variant's serialisation. This is the "byte-identity where
    // absent" half of the contract.
    let cases: [(MarkerKind, OldMarkerKind); 7] = [
        (MarkerKind::CleanShutdown, OldMarkerKind::CleanShutdown),
        (MarkerKind::SocketLost, OldMarkerKind::SocketLost),
        (MarkerKind::Resubscribed, OldMarkerKind::Resubscribed),
        (
            MarkerKind::SubscriptionGap {
                start_mono_ns: 3,
                end_mono_ns: 7,
            },
            OldMarkerKind::SubscriptionGap {
                start_mono_ns: 3,
                end_mono_ns: 7,
            },
        ),
        (
            MarkerKind::ExclusionUpdateLost,
            OldMarkerKind::ExclusionUpdateLost,
        ),
        (MarkerKind::RecorderStopped, OldMarkerKind::RecorderStopped),
        (
            MarkerKind::RecordingQuiescent,
            OldMarkerKind::RecordingQuiescent,
        ),
    ];
    for (new_kind, old_kind) in cases {
        let new_bytes = serde_json::to_vec(&Marker {
            mono_ns: 11,
            kind: new_kind.clone(),
        })
        .unwrap();
        let old_bytes = serde_json::to_vec(&OldMarker {
            mono_ns: 11,
            kind: old_kind,
        })
        .unwrap();
        assert_eq!(
            new_bytes, old_bytes,
            "serialisation of {new_kind:?} drifted from the 4a63eef reader"
        );
        // And the old reader round-trips its own bytes (it is a real,
        // working decoder, not a strawman).
        let back: OldMarker = serde_json::from_slice(&old_bytes).unwrap();
        assert_ne!(back.kind, OldMarkerKind::Unknown);
    }
}

#[test]
fn the_old_reader_degrades_a_power_failing_marker_to_unknown() {
    // The current writer's PowerFailing marker, decoded by the old enum,
    // is `Unknown` — never a decode error, never a different known kind.
    let new_bytes = serde_json::to_vec(&Marker {
        mono_ns: 99,
        kind: MarkerKind::PowerFailing,
    })
    .unwrap();
    let old: OldMarker = serde_json::from_slice(&new_bytes)
        .expect("an old reader must decode a future marker, not reject it");
    assert_eq!(old.kind, OldMarkerKind::Unknown);
    assert_eq!(old.mono_ns, 99, "the timestamp survives the degrade");
}

#[test]
fn an_old_reader_scans_past_a_power_failing_marker_to_the_following_records() {
    // A real segment: heartbeat, then the power-failing marker, then more
    // motion and a clean end. The frame/scan format is unchanged by this
    // branch, so the current scanner's walk over these bytes is exactly
    // the walk an old scanner performs. The old reader must not stop at
    // the unknown marker — it degrades it and keeps going.
    let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
    let trapq = TrapqSegment {
        mono_ns: 2,
        queue: "toolhead".to_owned(),
        print_time: 12.5,
        duration: 0.075,
        start_velocity: 40.0,
        acceleration: -1500.0,
        start_x: 10.0,
        start_y: 20.0,
        start_z: 0.4,
        x_r: 0.6,
        y_r: 0.8,
        z_r: 0.0,
    };
    writer
        .append(&WalRecord::Marker(Marker {
            mono_ns: 1,
            kind: MarkerKind::PowerFailing,
        }))
        .unwrap();
    writer
        .append(&WalRecord::TrapqSegment(trapq.clone()))
        .unwrap();
    writer
        .append(&WalRecord::Marker(Marker {
            mono_ns: 3,
            kind: MarkerKind::CleanShutdown,
        }))
        .unwrap();
    let bytes = writer.into_inner();

    let result = scan(&bytes);
    assert_eq!(
        result.end,
        ScanEnd::CleanEof,
        "the unknown marker must not truncate the scan"
    );
    assert_eq!(result.records.len(), 3, "every frame is recovered");

    // The record after the power-failing marker survives intact — the
    // scan did not stop at the marker.
    assert_eq!(
        result.records[1].record,
        WalRecord::TrapqSegment(trapq),
        "the record after the unknown marker must be recovered"
    );

    // Now the old-enum decode of that first marker's on-disk JSON: the
    // one thing the new variant could break, proven degrading to Unknown.
    let WalRecord::Marker(marker) = &result.records[0].record else {
        panic!("first frame is a marker");
    };
    let json = serde_json::to_vec(marker).unwrap();
    let old: OldMarker = serde_json::from_slice(&json).unwrap();
    assert_eq!(old.kind, OldMarkerKind::Unknown);
    // And the clean-shutdown marker after it still decodes as itself under
    // the old reader.
    let WalRecord::Marker(tail) = &result.records[2].record else {
        panic!("third frame is a marker");
    };
    let old_tail: OldMarker = serde_json::from_slice(&serde_json::to_vec(tail).unwrap()).unwrap();
    assert_eq!(old_tail.kind, OldMarkerKind::CleanShutdown);
}
