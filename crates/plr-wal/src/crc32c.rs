//! CRC32C (Castagnoli) checksum, implemented in-crate.
//!
//! The WAL uses CRC32C for every integrity check: segment headers, record
//! frames, and heartbeat slots. CRC32C is chosen over CRC32 (IEEE) for its
//! better error-detection properties on short messages; it is the checksum
//! specified by iSCSI (RFC 3720), Btrfs, and ext4.
//!
//! This is a small table-driven implementation (1 KiB table, built at
//! compile time) of the *reflected* CRC32C: polynomial `0x1EDC6F41`
//! (reflected form `0x82F63B78`), initial value `0xFFFF_FFFF`, final XOR
//! `0xFFFF_FFFF`, input and output reflected. It matches the published
//! RFC 3720 appendix B.4 test vectors (see the unit tests below).

/// The CRC32C (Castagnoli) polynomial in reflected (LSB-first) form.
///
/// The forward-notation polynomial is `0x1EDC6F41`.
pub const POLYNOMIAL_REFLECTED: u32 = 0x82F6_3B78;

/// Lookup table for one byte of input at a time, generated at compile time.
const TABLE: [u32; 256] = build_table();

const fn build_table() -> [u32; 256] {
    let mut table = [0_u32; 256];
    let mut i: usize = 0;
    while i < 256 {
        // `i < 256`, so the cast to u32 is lossless.
        #[allow(clippy::cast_possible_truncation)]
        let mut crc = i as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 == 1 {
                (crc >> 1) ^ POLYNOMIAL_REFLECTED
            } else {
                crc >> 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Incremental CRC32C state.
///
/// Use this when the checksummed bytes are produced in pieces (e.g. a frame
/// header followed by a payload) and concatenating them first would cost an
/// allocation. Producing the digest of the concatenation of all
/// [`update`](Self::update) inputs, [`finalize`](Self::finalize) is
/// non-destructive and may be called at any point.
#[derive(Debug, Clone)]
pub struct Crc32c {
    state: u32,
}

impl Crc32c {
    /// Creates a fresh CRC32C state (no bytes processed yet).
    #[must_use]
    pub const fn new() -> Self {
        Self { state: 0xFFFF_FFFF }
    }

    /// Feeds `data` into the checksum.
    pub fn update(&mut self, data: &[u8]) {
        let mut crc = self.state;
        for &byte in data {
            let index = (crc ^ u32::from(byte)) & 0xFF;
            crc = (crc >> 8) ^ TABLE[index as usize];
        }
        self.state = crc;
    }

    /// Returns the CRC32C digest of all bytes fed so far.
    #[must_use]
    pub const fn finalize(&self) -> u32 {
        !self.state
    }
}

impl Default for Crc32c {
    fn default() -> Self {
        Self::new()
    }
}

/// Computes the CRC32C digest of `data` in one shot.
#[must_use]
pub fn crc32c(data: &[u8]) -> u32 {
    let mut crc = Crc32c::new();
    crc.update(data);
    crc.finalize()
}

#[cfg(test)]
mod tests {
    use super::{crc32c, Crc32c};

    // Published test vectors from RFC 3720 (iSCSI), appendix B.4
    // "CRC Examples", expressed as native u32 digests. The same values
    // appear in the reference implementations shipped with the `crc32c`
    // and `crc` crates and in the Linux kernel selftests.

    #[test]
    fn compile_time_table_matches_runtime_generation() {
        // TABLE is const-evaluated, so exercise `build_table` at runtime
        // too and pin a couple of well-known entries.
        let runtime = super::build_table();
        assert_eq!(runtime, super::TABLE);
        assert_eq!(runtime[0], 0);
        assert_eq!(runtime[1], 0xF26B_8303);
        assert_eq!(runtime[255], 0xAD7D_5351);
    }

    #[test]
    fn rfc3720_check_value_123456789() {
        // The classic CRC "check" input.
        assert_eq!(crc32c(b"123456789"), 0xE306_9283);
    }

    #[test]
    fn rfc3720_32_bytes_of_zeros() {
        assert_eq!(crc32c(&[0_u8; 32]), 0x8A91_36AA);
    }

    #[test]
    fn rfc3720_32_bytes_of_ones() {
        assert_eq!(crc32c(&[0xFF_u8; 32]), 0x62A8_AB43);
    }

    #[test]
    fn rfc3720_32_bytes_incrementing() {
        let data: Vec<u8> = (0_u8..32).collect();
        assert_eq!(crc32c(&data), 0x46DD_794E);
    }

    #[test]
    fn rfc3720_32_bytes_decrementing() {
        let data: Vec<u8> = (0_u8..32).rev().collect();
        assert_eq!(crc32c(&data), 0x113F_DB5C);
    }

    #[test]
    fn empty_input_digest_is_zero() {
        // CRC32C of the empty string: init ^ final-xor == 0.
        assert_eq!(crc32c(&[]), 0);
    }

    #[test]
    fn incremental_updates_match_one_shot() {
        let data = b"the quick brown fox jumps over the lazy dog";
        for split in 0..=data.len() {
            let (left, right) = data.split_at(split);
            let mut crc = Crc32c::new();
            crc.update(left);
            crc.update(right);
            assert_eq!(crc.finalize(), crc32c(data), "split at {split}");
        }
    }

    #[test]
    fn finalize_is_non_destructive() {
        let mut crc = Crc32c::new();
        crc.update(b"abc");
        let first = crc.finalize();
        assert_eq!(crc.finalize(), first);
        crc.update(b"def");
        assert_eq!(crc.finalize(), crc32c(b"abcdef"));
    }

    #[test]
    fn default_equals_new() {
        assert_eq!(Crc32c::default().finalize(), Crc32c::new().finalize());
    }

    #[test]
    fn single_bit_flip_always_changes_digest() {
        // Any CRC detects all single-bit errors within the checksummed
        // region; spot-check the property since the WAL leans on it.
        let data = b"power loss recovery";
        let base = crc32c(data);
        for byte_index in 0..data.len() {
            for bit in 0..8 {
                let mut corrupted = data.to_vec();
                corrupted[byte_index] ^= 1 << bit;
                assert_ne!(crc32c(&corrupted), base, "byte {byte_index} bit {bit}");
            }
        }
    }
}
