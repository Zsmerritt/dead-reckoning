//! Fixed-size, torn-write-safe heartbeat slot file.
//!
//! The daemon rewrites a heartbeat in place at ~10 Hz to prove "alive and
//! executing at time `t_a`". Rewriting one region in place can tear under
//! power loss, so the file holds **two alternating slots** (A and B):
//! heartbeat with sequence `n` goes to slot A when `n` is even, slot B
//! when `n` is odd ([`slot_for_sequence`]). A torn write can only destroy
//! the slot being written; the other slot still holds the previous (one
//! tick older) heartbeat. The reader validates both slots and picks the
//! valid one with the newest sequence.
//!
//! The file is exactly [`HEARTBEAT_FILE_LEN`] = 128 bytes — deliberately
//! tiny, because it is rewritten and fdatasync'd (by the daemon, not
//! here) at 10 Hz and write amplification matters.
//!
//! # Slot layout ([`HEARTBEAT_SLOT_LEN`] = 64 bytes, all integers
//! little-endian)
//!
//! ```text
//! offset  size  field
//! 0       4     slot magic + version, b"PHB1" (HEARTBEAT_SLOT_MAGIC)
//! 4       8     sequence, u64 (wrapping heartbeat counter)
//! 12      8     mono_ns, u64 (host-monotonic capture time, ns)
//! 20      8     wall_ns, u64 (wall clock, ns since Unix epoch)
//! 28      8     print_time, f64 as IEEE-754 bits (latest known)
//! 36      8     est_sample_mono_ns, u64 (monotonic time of the
//!               estimated_print_time sample)
//! 44      8     est_sample_print_time, f64 bits (estimated_print_time
//!               at est_sample_mono_ns)
//! 52      8     wal_offset, u64 (WAL append offset at heartbeat time)
//! 60      4     CRC32C over bytes 0..60, u32
//! ```
//!
//! Slot A occupies file bytes `0..64`, slot B bytes `64..128`.
//!
//! Floats are stored as raw IEEE-754 bits, so this encoding (unlike the
//! JSON WAL payloads) round-trips NaN and signed zeros bit-exactly.

use crate::bytes::{le_f64, le_u32, le_u64, write_at};
use crate::crc32c::crc32c;
use crate::record::Heartbeat;

/// Magic + format version opening every heartbeat slot.
pub const HEARTBEAT_SLOT_MAGIC: [u8; 4] = *b"PHB1";

/// Encoded size of one heartbeat slot, in bytes.
pub const HEARTBEAT_SLOT_LEN: usize = 64;

/// Total size of the heartbeat file (two slots), in bytes.
pub const HEARTBEAT_FILE_LEN: usize = 2 * HEARTBEAT_SLOT_LEN;

/// Which of the two alternating slots a heartbeat lives in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlotId {
    /// First slot, file bytes `0..64`; holds even sequence numbers.
    A,
    /// Second slot, file bytes `64..128`; holds odd sequence numbers.
    B,
}

impl SlotId {
    /// Byte offset of this slot within the heartbeat file.
    #[must_use]
    pub const fn offset(self) -> usize {
        match self {
            Self::A => 0,
            Self::B => HEARTBEAT_SLOT_LEN,
        }
    }

    /// The other slot.
    #[must_use]
    pub const fn other(self) -> Self {
        match self {
            Self::A => Self::B,
            Self::B => Self::A,
        }
    }
}

/// The slot a heartbeat with `sequence` must be written to (even → A,
/// odd → B), so consecutive heartbeats always alternate slots.
#[must_use]
pub const fn slot_for_sequence(sequence: u64) -> SlotId {
    if sequence.is_multiple_of(2) {
        SlotId::A
    } else {
        SlotId::B
    }
}

/// Why one slot failed validation (i.e. is torn or foreign).
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum SlotError {
    /// Fewer than [`HEARTBEAT_SLOT_LEN`] bytes were available for the
    /// slot (short file).
    #[error("slot is {len} bytes, shorter than the {HEARTBEAT_SLOT_LEN}-byte layout")]
    TooShort {
        /// Bytes actually present.
        len: usize,
    },
    /// The slot does not start with [`HEARTBEAT_SLOT_MAGIC`].
    #[error("slot magic/version mismatch")]
    BadMagic,
    /// The slot CRC does not match its contents: a torn or corrupt write.
    #[error("slot CRC mismatch (torn write)")]
    CrcMismatch,
}

/// Errors from [`recover_heartbeat`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum HeartbeatError {
    /// The file is larger than [`HEARTBEAT_FILE_LEN`]: not a heartbeat
    /// file this format wrote.
    #[error("heartbeat file is {len} bytes; the format is exactly {HEARTBEAT_FILE_LEN}")]
    OversizedFile {
        /// Actual file length.
        len: usize,
    },
    /// Neither slot validated — no heartbeat is recoverable. Both
    /// per-slot reasons are reported so "brand new file" (both too
    /// short/zeroed) is distinguishable from "double corruption".
    #[error("no valid heartbeat: slot A {slot_a}; slot B {slot_b}")]
    Unrecoverable {
        /// Why slot A was rejected.
        slot_a: SlotError,
        /// Why slot B was rejected.
        slot_b: SlotError,
    },
}

/// Outcome of a successful heartbeat recovery.
#[derive(Debug, Clone, PartialEq)]
pub struct HeartbeatRecovery {
    /// The newest valid heartbeat.
    pub heartbeat: Heartbeat,
    /// Which slot it came from.
    pub slot: SlotId,
    /// The other slot's failure, when it did not validate (`None` when
    /// both slots were intact). A torn other-slot is the expected state
    /// after power loss mid-rewrite.
    pub torn: Option<(SlotId, SlotError)>,
}

/// Encodes one heartbeat into its fixed 64-byte slot layout.
#[must_use]
pub fn encode_slot(heartbeat: &Heartbeat) -> [u8; HEARTBEAT_SLOT_LEN] {
    let mut buf = [0_u8; HEARTBEAT_SLOT_LEN];
    write_at(&mut buf, 0, &HEARTBEAT_SLOT_MAGIC);
    write_at(&mut buf, 4, &heartbeat.sequence.to_le_bytes());
    write_at(&mut buf, 12, &heartbeat.mono_ns.to_le_bytes());
    write_at(&mut buf, 20, &heartbeat.wall_ns.to_le_bytes());
    write_at(&mut buf, 28, &heartbeat.print_time.to_bits().to_le_bytes());
    write_at(&mut buf, 36, &heartbeat.est_sample_mono_ns.to_le_bytes());
    write_at(
        &mut buf,
        44,
        &heartbeat.est_sample_print_time.to_bits().to_le_bytes(),
    );
    write_at(&mut buf, 52, &heartbeat.wal_offset.to_le_bytes());
    let crc = crc32c(&buf[..HEARTBEAT_SLOT_LEN - 4]);
    write_at(&mut buf, HEARTBEAT_SLOT_LEN - 4, &crc.to_le_bytes());
    buf
}

/// Decodes and validates one slot. Never panics on any input.
pub fn decode_slot(bytes: &[u8]) -> Result<Heartbeat, SlotError> {
    if bytes.len() < HEARTBEAT_SLOT_LEN {
        return Err(SlotError::TooShort { len: bytes.len() });
    }
    let slot = bytes
        .get(..HEARTBEAT_SLOT_LEN)
        .ok_or(SlotError::TooShort { len: bytes.len() })?;
    let crc_end = HEARTBEAT_SLOT_LEN - 4;
    let crc_region = slot.get(..crc_end).ok_or(SlotError::CrcMismatch)?;
    let stored = le_u32(slot, crc_end).ok_or(SlotError::CrcMismatch)?;
    if crc32c(crc_region) != stored {
        return Err(SlotError::CrcMismatch);
    }
    // CRC-valid but wrong magic: written by something else entirely.
    if !slot.starts_with(&HEARTBEAT_SLOT_MAGIC) {
        return Err(SlotError::BadMagic);
    }
    // All offsets below are in bounds by construction (len checked above);
    // the ok_or arms are unreachable belt and braces.
    Ok(Heartbeat {
        sequence: le_u64(slot, 4).ok_or(SlotError::CrcMismatch)?,
        mono_ns: le_u64(slot, 12).ok_or(SlotError::CrcMismatch)?,
        wall_ns: le_u64(slot, 20).ok_or(SlotError::CrcMismatch)?,
        print_time: le_f64(slot, 28).ok_or(SlotError::CrcMismatch)?,
        est_sample_mono_ns: le_u64(slot, 36).ok_or(SlotError::CrcMismatch)?,
        est_sample_print_time: le_f64(slot, 44).ok_or(SlotError::CrcMismatch)?,
        wal_offset: le_u64(slot, 52).ok_or(SlotError::CrcMismatch)?,
    })
}

/// `true` when `candidate` is newer than `other` in wrapping (serial
/// number) arithmetic: the forward distance from `other` to `candidate`
/// is in `(0, 2^63)`.
///
/// A u64 counter at 10 Hz cannot wrap in the lifetime of the universe,
/// but the comparison is exact anyway so wrap behavior is defined, not
/// accidental.
const fn sequence_is_newer(candidate: u64, other: u64) -> bool {
    let forward = candidate.wrapping_sub(other);
    forward != 0 && forward < (1 << 63)
}

/// Recovers the newest valid heartbeat from the two-slot file image.
///
/// Handles every post-power-loss state:
///
/// - both slots valid → the newer sequence wins (wrapping comparison;
///   on the impossible equal-sequence tie, slot A deterministically);
/// - one slot torn → the intact slot wins, and the tear is reported in
///   [`HeartbeatRecovery::torn`];
/// - both torn/invalid → [`HeartbeatError::Unrecoverable`] with both
///   per-slot reasons.
///
/// A file shorter than 128 bytes is treated as having torn slot(s), not
/// as an error shape of its own: power loss during initial file creation
/// legitimately produces short files.
pub fn recover_heartbeat(bytes: &[u8]) -> Result<HeartbeatRecovery, HeartbeatError> {
    if bytes.len() > HEARTBEAT_FILE_LEN {
        return Err(HeartbeatError::OversizedFile { len: bytes.len() });
    }
    let slot_a = decode_slot(bytes);
    let slot_b = decode_slot(bytes.get(HEARTBEAT_SLOT_LEN..).unwrap_or(&[]));
    match (slot_a, slot_b) {
        (Ok(a), Ok(b)) => {
            // Tie (equal sequences) cannot be produced by the alternating
            // writer; resolve deterministically to A.
            if sequence_is_newer(b.sequence, a.sequence) {
                Ok(HeartbeatRecovery {
                    heartbeat: b,
                    slot: SlotId::B,
                    torn: None,
                })
            } else {
                Ok(HeartbeatRecovery {
                    heartbeat: a,
                    slot: SlotId::A,
                    torn: None,
                })
            }
        }
        (Ok(a), Err(b_err)) => Ok(HeartbeatRecovery {
            heartbeat: a,
            slot: SlotId::A,
            torn: Some((SlotId::B, b_err)),
        }),
        (Err(a_err), Ok(b)) => Ok(HeartbeatRecovery {
            heartbeat: b,
            slot: SlotId::B,
            torn: Some((SlotId::A, a_err)),
        }),
        (Err(slot_a), Err(slot_b)) => Err(HeartbeatError::Unrecoverable { slot_a, slot_b }),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        decode_slot, encode_slot, recover_heartbeat, sequence_is_newer, slot_for_sequence,
        HeartbeatError, SlotError, SlotId, HEARTBEAT_FILE_LEN, HEARTBEAT_SLOT_LEN,
    };
    use crate::crc32c::crc32c;
    use crate::record::samples::sample_heartbeat;
    use crate::record::Heartbeat;

    fn heartbeat_with_sequence(sequence: u64) -> Heartbeat {
        Heartbeat {
            sequence,
            mono_ns: 1_000_u64.wrapping_add(sequence),
            ..sample_heartbeat()
        }
    }

    /// Builds a file image with `a` in slot A and `b` in slot B.
    fn build_file(a: &Heartbeat, b: &Heartbeat) -> Vec<u8> {
        let mut file = Vec::with_capacity(HEARTBEAT_FILE_LEN);
        file.extend_from_slice(&encode_slot(a));
        file.extend_from_slice(&encode_slot(b));
        file
    }

    #[test]
    fn slot_layout_is_pinned_byte_by_byte() {
        let hb = sample_heartbeat();
        let slot = encode_slot(&hb);
        assert_eq!(&slot[..4], b"PHB1");
        assert_eq!(&slot[4..12], &hb.sequence.to_le_bytes());
        assert_eq!(&slot[12..20], &hb.mono_ns.to_le_bytes());
        assert_eq!(&slot[20..28], &hb.wall_ns.to_le_bytes());
        assert_eq!(&slot[28..36], &hb.print_time.to_bits().to_le_bytes());
        assert_eq!(&slot[36..44], &hb.est_sample_mono_ns.to_le_bytes());
        assert_eq!(
            &slot[44..52],
            &hb.est_sample_print_time.to_bits().to_le_bytes()
        );
        assert_eq!(&slot[52..60], &hb.wal_offset.to_le_bytes());
        assert_eq!(&slot[60..64], &crc32c(&slot[..60]).to_le_bytes());
    }

    #[test]
    fn slot_roundtrips_including_non_finite_floats() {
        let mut hb = sample_heartbeat();
        assert_eq!(decode_slot(&encode_slot(&hb)), Ok(hb));
        // Binary slots are bit-exact even for values JSON would refuse.
        hb.print_time = f64::NAN;
        hb.est_sample_print_time = f64::NEG_INFINITY;
        let decoded = decode_slot(&encode_slot(&hb)).unwrap();
        assert_eq!(decoded.print_time.to_bits(), hb.print_time.to_bits());
        assert_eq!(
            decoded.est_sample_print_time.to_bits(),
            hb.est_sample_print_time.to_bits()
        );
    }

    #[test]
    fn slot_for_sequence_alternates_and_wraps() {
        assert_eq!(slot_for_sequence(0), SlotId::A);
        assert_eq!(slot_for_sequence(1), SlotId::B);
        assert_eq!(slot_for_sequence(2), SlotId::A);
        assert_eq!(slot_for_sequence(u64::MAX), SlotId::B);
        assert_eq!(slot_for_sequence(u64::MAX.wrapping_add(1)), SlotId::A);
        assert_eq!(SlotId::A.offset(), 0);
        assert_eq!(SlotId::B.offset(), HEARTBEAT_SLOT_LEN);
        assert_eq!(SlotId::A.other(), SlotId::B);
        assert_eq!(SlotId::B.other(), SlotId::A);
    }

    #[test]
    fn both_slots_valid_picks_higher_sequence() {
        let older = heartbeat_with_sequence(10);
        let newer = heartbeat_with_sequence(11);
        let file = build_file(&older, &newer);
        let recovery = recover_heartbeat(&file).unwrap();
        assert_eq!(recovery.heartbeat, newer);
        assert_eq!(recovery.slot, SlotId::B);
        assert_eq!(recovery.torn, None);

        // And the mirror image: newer heartbeat in slot A.
        let older = heartbeat_with_sequence(11);
        let newer = heartbeat_with_sequence(12);
        let file = build_file(&newer, &older);
        let recovery = recover_heartbeat(&file).unwrap();
        assert_eq!(recovery.heartbeat, newer);
        assert_eq!(recovery.slot, SlotId::A);
    }

    #[test]
    fn sequence_wrap_prefers_wrapped_zero_over_u64_max() {
        // At wrap, slot B holds u64::MAX (odd) and the next heartbeat,
        // sequence 0, lands in slot A. Naive max() would pick u64::MAX;
        // serial-number comparison must pick 0.
        let pre_wrap = heartbeat_with_sequence(u64::MAX);
        let post_wrap = heartbeat_with_sequence(0);
        assert_eq!(slot_for_sequence(pre_wrap.sequence), SlotId::B);
        assert_eq!(slot_for_sequence(post_wrap.sequence), SlotId::A);
        let file = build_file(&post_wrap, &pre_wrap);
        let recovery = recover_heartbeat(&file).unwrap();
        assert_eq!(recovery.heartbeat, post_wrap);
        assert_eq!(recovery.slot, SlotId::A);
    }

    #[test]
    fn sequence_is_newer_is_a_strict_wrapping_order() {
        assert!(sequence_is_newer(1, 0));
        assert!(!sequence_is_newer(0, 1));
        assert!(!sequence_is_newer(5, 5));
        assert!(sequence_is_newer(0, u64::MAX)); // wrap
        assert!(!sequence_is_newer(u64::MAX, 0));
    }

    #[test]
    fn equal_sequences_resolve_to_slot_a() {
        let hb = heartbeat_with_sequence(4);
        let file = build_file(&hb, &hb);
        let recovery = recover_heartbeat(&file).unwrap();
        assert_eq!(recovery.slot, SlotId::A);
    }

    #[test]
    fn torn_slot_a_recovers_from_slot_b() {
        let a = heartbeat_with_sequence(2);
        let b = heartbeat_with_sequence(1);
        let mut file = build_file(&a, &b);
        file[17] ^= 0x80; // corrupt slot A mid-field
        let recovery = recover_heartbeat(&file).unwrap();
        assert_eq!(recovery.heartbeat, b);
        assert_eq!(recovery.slot, SlotId::B);
        assert_eq!(recovery.torn, Some((SlotId::A, SlotError::CrcMismatch)));
    }

    #[test]
    fn torn_slot_b_recovers_from_slot_a() {
        let a = heartbeat_with_sequence(2);
        let b = heartbeat_with_sequence(3);
        let mut file = build_file(&a, &b);
        file[HEARTBEAT_SLOT_LEN + 60] ^= 0x01; // corrupt slot B's CRC itself
        let recovery = recover_heartbeat(&file).unwrap();
        assert_eq!(recovery.heartbeat, a);
        assert_eq!(recovery.slot, SlotId::A);
        assert_eq!(recovery.torn, Some((SlotId::B, SlotError::CrcMismatch)));
    }

    #[test]
    fn both_torn_reports_unrecoverable_with_both_reasons() {
        let a = heartbeat_with_sequence(2);
        let b = heartbeat_with_sequence(3);
        let mut file = build_file(&a, &b);
        file[5] ^= 0xFF;
        file[HEARTBEAT_SLOT_LEN + 5] ^= 0xFF;
        let err = recover_heartbeat(&file).unwrap_err();
        assert_eq!(
            err,
            HeartbeatError::Unrecoverable {
                slot_a: SlotError::CrcMismatch,
                slot_b: SlotError::CrcMismatch,
            }
        );
        assert!(err.to_string().contains("slot A"));
    }

    #[test]
    fn fresh_zeroed_file_is_unrecoverable_but_distinct() {
        let err = recover_heartbeat(&[0_u8; HEARTBEAT_FILE_LEN]).unwrap_err();
        // All-zero slots fail CRC (crc32c of 60 zero bytes != 0)…
        assert_eq!(
            err,
            HeartbeatError::Unrecoverable {
                slot_a: SlotError::CrcMismatch,
                slot_b: SlotError::CrcMismatch,
            }
        );
        // …while an empty file reports TooShort, so the daemon can tell
        // "never written" from "written and destroyed".
        let err = recover_heartbeat(&[]).unwrap_err();
        assert_eq!(
            err,
            HeartbeatError::Unrecoverable {
                slot_a: SlotError::TooShort { len: 0 },
                slot_b: SlotError::TooShort { len: 0 },
            }
        );
    }

    #[test]
    fn short_file_with_only_slot_a_recovers_slot_a() {
        // Power loss after writing slot A but before the file ever
        // reached full size.
        let a = heartbeat_with_sequence(0);
        let file = encode_slot(&a);
        let recovery = recover_heartbeat(&file).unwrap();
        assert_eq!(recovery.heartbeat, a);
        assert_eq!(recovery.slot, SlotId::A);
        assert_eq!(
            recovery.torn,
            Some((SlotId::B, SlotError::TooShort { len: 0 }))
        );
    }

    #[test]
    fn oversized_file_is_rejected_outright() {
        let file = vec![0_u8; HEARTBEAT_FILE_LEN + 1];
        assert_eq!(
            recover_heartbeat(&file),
            Err(HeartbeatError::OversizedFile {
                len: HEARTBEAT_FILE_LEN + 1
            })
        );
    }

    #[test]
    fn crc_valid_foreign_magic_reports_bad_magic() {
        let hb = heartbeat_with_sequence(6);
        let mut slot = encode_slot(&hb);
        slot[..4].copy_from_slice(b"XXXX");
        let crc = crc32c(&slot[..60]);
        slot[60..].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_slot(&slot), Err(SlotError::BadMagic));
    }

    #[test]
    fn stale_vs_fresh_selection_ignores_which_slot_is_which() {
        // Regardless of physical order, the newest sequence wins.
        for (a_seq, b_seq, expected_slot) in [
            (100_u64, 99_u64, SlotId::A),
            (99, 100, SlotId::B),
            (0, 1, SlotId::B),
            (1, 0, SlotId::A),
        ] {
            let file = build_file(
                &heartbeat_with_sequence(a_seq),
                &heartbeat_with_sequence(b_seq),
            );
            let recovery = recover_heartbeat(&file).unwrap();
            assert_eq!(recovery.slot, expected_slot, "a={a_seq} b={b_seq}");
        }
    }

    #[test]
    fn error_displays_are_meaningful() {
        assert!(SlotError::TooShort { len: 3 }
            .to_string()
            .contains("3 bytes"));
        assert!(SlotError::CrcMismatch.to_string().contains("torn"));
        assert!(HeartbeatError::OversizedFile { len: 300 }
            .to_string()
            .contains("300"));
    }
}
