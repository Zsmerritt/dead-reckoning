//! Panic-free little-endian byte accessors.
//!
//! The decoder must never panic on any input, so all reads go through these
//! bounds-checked helpers instead of slice indexing. The writers use
//! [`write_at`] for fixed-layout buffers for the same reason.

/// Reads a little-endian `u32` at byte offset `at`, or `None` if `bytes` is
/// too short.
pub(crate) fn le_u32(bytes: &[u8], at: usize) -> Option<u32> {
    let end = at.checked_add(4)?;
    let slice = bytes.get(at..end)?;
    let array: [u8; 4] = slice.try_into().ok()?;
    Some(u32::from_le_bytes(array))
}

/// Reads a little-endian `u64` at byte offset `at`, or `None` if `bytes` is
/// too short.
pub(crate) fn le_u64(bytes: &[u8], at: usize) -> Option<u64> {
    let end = at.checked_add(8)?;
    let slice = bytes.get(at..end)?;
    let array: [u8; 8] = slice.try_into().ok()?;
    Some(u64::from_le_bytes(array))
}

/// Copies `src` into `buf[at..at + src.len()]`.
///
/// A no-op if the destination range does not fit inside `buf`; callers use
/// compile-time-constant offsets into fixed-size buffers, so the no-op path
/// is unreachable in practice but keeps this helper panic-free by
/// construction.
pub(crate) fn write_at(buf: &mut [u8], at: usize, src: &[u8]) {
    if let Some(dst) = at
        .checked_add(src.len())
        .and_then(|end| buf.get_mut(at..end))
    {
        dst.copy_from_slice(src);
    }
}

#[cfg(test)]
mod tests {
    use super::{le_u32, le_u64, write_at};

    #[test]
    fn le_u32_reads_in_bounds() {
        let bytes = [0x78, 0x56, 0x34, 0x12, 0xAA];
        assert_eq!(le_u32(&bytes, 0), Some(0x1234_5678));
        assert_eq!(le_u32(&bytes, 1), Some(0xAA12_3456));
    }

    #[test]
    fn le_u32_rejects_out_of_bounds() {
        let bytes = [1, 2, 3, 4];
        assert_eq!(le_u32(&bytes, 1), None);
        assert_eq!(le_u32(&bytes, usize::MAX), None); // offset overflow
        assert_eq!(le_u32(&[], 0), None);
    }

    #[test]
    fn le_u64_reads_in_bounds() {
        let bytes = 0xDEAD_BEEF_0BAD_F00D_u64.to_le_bytes();
        assert_eq!(le_u64(&bytes, 0), Some(0xDEAD_BEEF_0BAD_F00D));
    }

    #[test]
    fn le_u64_rejects_out_of_bounds() {
        let bytes = [0_u8; 8];
        assert_eq!(le_u64(&bytes, 1), None);
        assert_eq!(le_u64(&bytes, usize::MAX), None);
    }

    #[test]
    fn write_at_copies_in_bounds() {
        let mut buf = [0_u8; 6];
        write_at(&mut buf, 2, &[0xAB, 0xCD]);
        assert_eq!(buf, [0, 0, 0xAB, 0xCD, 0, 0]);
    }

    #[test]
    fn write_at_ignores_out_of_bounds() {
        let mut buf = [0_u8; 4];
        write_at(&mut buf, 3, &[1, 2]); // would run off the end
        write_at(&mut buf, usize::MAX, &[1]); // offset overflow
        assert_eq!(buf, [0_u8; 4]);
    }

    #[test]
    fn write_at_empty_source_is_noop() {
        let mut buf = [7_u8; 2];
        write_at(&mut buf, 0, &[]);
        assert_eq!(buf, [7, 7]);
    }
}
