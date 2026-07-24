//! The `receive_seq` sidecar file: the newest durable widened
//! `receive_seq` observation.
//!
//! # Why a sidecar and not a WAL record
//!
//! `plr-reconstruct` consumes the observation as an *optional* input
//! (`ReceiveSeqObservation`) that tightens the committed-motion boundary
//! `t_b`; the WAL record shapes in `plr-wal` have no field for it and the
//! on-disk format is frozen. A 24-byte rewrite-in-place file updated on
//! every counter advance (~1 Hz from `mcu.last_stats`) is cheaper than a
//! WAL format revision and self-describing for the offline `scan` tool.
//!
//! # Torn writes
//!
//! The file is CRC-guarded and rewritten in place without slots: a torn
//! write simply loses the observation. That is the *safe* direction —
//! reconstruction without a receive-seq bound only widens the possible-
//! stop set, never shrinks it — so single-slot is deliberate.
//!
//! # Layout (24 bytes, little-endian)
//!
//! ```text
//! offset  size  field
//! 0       4     magic + version, b"PSQ1"
//! 4       8     mono_ns, u64 (host-monotonic observation time)
//! 12      8     widened receive_seq, u64
//! 20      4     CRC32C over bytes 0..20
//! ```

use plr_wal::crc32c;

/// Magic + format version opening the sidecar file.
pub const SEQ_FILE_MAGIC: [u8; 4] = *b"PSQ1";

/// Exact file length in bytes.
pub const SEQ_FILE_LEN: usize = 24;

/// Encodes one observation to the fixed layout.
#[must_use]
pub fn encode_seq(mono_ns: u64, widened: u64) -> [u8; SEQ_FILE_LEN] {
    let mut buf = [0_u8; SEQ_FILE_LEN];
    buf[..4].copy_from_slice(&SEQ_FILE_MAGIC);
    buf[4..12].copy_from_slice(&mono_ns.to_le_bytes());
    buf[12..20].copy_from_slice(&widened.to_le_bytes());
    let crc = crc32c(&buf[..20]);
    buf[20..].copy_from_slice(&crc.to_le_bytes());
    buf
}

/// Decodes an observation: `(mono_ns, widened)`. `None` on any invalid
/// input (short file, wrong magic, CRC mismatch) — the observation is
/// optional and its absence is the conservative direction.
#[must_use]
pub fn decode_seq(bytes: &[u8]) -> Option<(u64, u64)> {
    let bytes: &[u8; SEQ_FILE_LEN] = bytes.get(..SEQ_FILE_LEN)?.try_into().ok()?;
    if bytes[..4] != SEQ_FILE_MAGIC {
        return None;
    }
    let stored = u32::from_le_bytes(bytes[20..24].try_into().ok()?);
    if crc32c(&bytes[..20]) != stored {
        return None;
    }
    let mono_ns = u64::from_le_bytes(bytes[4..12].try_into().ok()?);
    let widened = u64::from_le_bytes(bytes[12..20].try_into().ok()?);
    Some((mono_ns, widened))
}

#[cfg(test)]
mod tests {
    use super::{decode_seq, encode_seq, SEQ_FILE_LEN, SEQ_FILE_MAGIC};

    #[test]
    fn round_trips_and_layout_is_pinned() {
        let bytes = encode_seq(123_456_789_000, 0xDEAD_BEEF_CAFE);
        assert_eq!(bytes.len(), SEQ_FILE_LEN);
        assert_eq!(&bytes[..4], &SEQ_FILE_MAGIC);
        assert_eq!(&bytes[4..12], &123_456_789_000_u64.to_le_bytes());
        assert_eq!(&bytes[12..20], &0xDEAD_BEEF_CAFE_u64.to_le_bytes());
        assert_eq!(
            decode_seq(&bytes),
            Some((123_456_789_000, 0xDEAD_BEEF_CAFE))
        );
        // Extremes round-trip.
        assert_eq!(decode_seq(&encode_seq(0, 0)), Some((0, 0)));
        assert_eq!(
            decode_seq(&encode_seq(u64::MAX, u64::MAX)),
            Some((u64::MAX, u64::MAX))
        );
    }

    #[test]
    fn invalid_inputs_decode_to_none() {
        // Empty and short files (fresh creation, torn write).
        assert_eq!(decode_seq(&[]), None);
        assert_eq!(decode_seq(&encode_seq(1, 2)[..23]), None);
        // All zeros (created, never written).
        assert_eq!(decode_seq(&[0_u8; SEQ_FILE_LEN]), None);
        // Flipping any single byte breaks the CRC (or the magic).
        let good = encode_seq(55_555, 77_777);
        for i in 0..SEQ_FILE_LEN {
            let mut bad = good;
            bad[i] ^= 0x01;
            assert_eq!(decode_seq(&bad), None, "byte {i} flip went undetected");
        }
        // Trailing garbage after a valid image is tolerated (read
        // whole-file semantics: only the first 24 bytes are the record).
        let mut long = good.to_vec();
        long.push(0xFF);
        assert_eq!(decode_seq(&long), Some((55_555, 77_777)));
    }
}
