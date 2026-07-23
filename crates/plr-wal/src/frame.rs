//! Append-only log encoding, decoding, and torn-write recovery.
//!
//! # On-disk format
//!
//! A WAL segment is a 32-byte segment header followed by zero or more
//! record frames, back to back. All integers are little-endian.
//!
//! Segment header ([`SEGMENT_HEADER_LEN`] = 32 bytes):
//!
//! ```text
//! offset  size  field
//! 0       8     magic, b"PLR-WAL\0" (SEGMENT_MAGIC)
//! 8       4     format version, u32 (FORMAT_VERSION)
//! 12      8     creation wall-clock time, u64 ns since Unix epoch
//! 20      8     creation host-monotonic time, u64 ns
//! 28      4     CRC32C over bytes 0..28, u32
//! ```
//!
//! Record frame ([`FRAME_HEADER_LEN`] = 8 header bytes +
//! [`FRAME_TRAILER_LEN`] = 4 trailer bytes):
//!
//! ```text
//! offset  size  field
//! 0       2     frame magic, [0xD5, 0xAA] (FRAME_MAGIC)
//! 2       1     payload format tag, u8 (1 = JSON, PAYLOAD_FORMAT_JSON)
//! 3       1     record kind tag, u8 (RecordKind::as_u8)
//! 4       4     payload length, u32 (<= MAX_PAYLOAD_LEN)
//! 8       len   payload (serde_json bytes of a WalRecord)
//! 8+len   4     CRC32C over bytes 0..8+len (header + payload), u32
//! ```
//!
//! JSON payloads are a deliberate choice at this data rate (~1–6 KB/s
//! during motion): the log stays greppable and debuggable post-mortem,
//! while the binary frame supplies the length, type, and integrity
//! guarantees JSON lacks.
//!
//! # Torn writes are the expected case
//!
//! Power loss mid-append leaves a partial frame at the tail. The recovery
//! scan ([`scan`] / [`WalReader`]) therefore yields records until the
//! first invalid frame and reports *where* and *why* the valid prefix
//! ends; a truncated trailing frame is a normal outcome, not an error.
//! The decoder never panics and never allocates based on unvalidated
//! lengths.

use std::io::{Read, Write};

use crate::bytes::{le_u32, le_u64, write_at};
use crate::crc32c::crc32c;
use crate::record::{RecordKind, WalRecord};

/// Magic bytes opening every WAL segment file.
pub const SEGMENT_MAGIC: [u8; 8] = *b"PLR-WAL\0";

/// On-disk format version this crate reads and writes.
pub const FORMAT_VERSION: u32 = 1;

/// Encoded size of the segment header, in bytes.
pub const SEGMENT_HEADER_LEN: usize = 32;

/// Magic bytes opening every record frame. Chosen to be invalid UTF-8 so
/// raw JSON accidentally written at a frame boundary cannot match.
pub const FRAME_MAGIC: [u8; 2] = [0xD5, 0xAA];

/// Size of the fixed frame header (magic + format + kind + length).
pub const FRAME_HEADER_LEN: usize = 8;

/// Size of the frame trailer (the CRC32C), in bytes.
pub const FRAME_TRAILER_LEN: usize = 4;

/// Payload format tag for `serde_json`-encoded [`WalRecord`] payloads.
pub const PAYLOAD_FORMAT_JSON: u8 = 1;

/// Hard cap on the payload length field.
///
/// Real records are hundreds of bytes; anything above this is corruption
/// (or an attack) and the scan stops rather than trusting the length.
/// The cap also bounds what [`scan_read`] buffers per frame.
pub const MAX_PAYLOAD_LEN: u32 = 1 << 20;

/// Errors from the append path ([`WalWriter`]).
///
/// The read/recovery path never fails — it reports how far the log was
/// valid via [`RecoveryScan`] instead.
#[derive(Debug, thiserror::Error)]
pub enum WalError {
    /// The underlying `Write` failed. The stream may now end in a partial
    /// frame; that prefix remains recoverable, but this writer must not
    /// be used for further appends.
    #[error("i/o error while writing WAL bytes: {0}")]
    Io(#[from] std::io::Error),
    /// The record could not be serialized to JSON.
    #[error("failed to serialize record payload: {0}")]
    Serialize(#[from] serde_json::Error),
    /// The serialized payload exceeds [`MAX_PAYLOAD_LEN`].
    #[error("record payload is {len} bytes; the format caps payloads at {max}")]
    PayloadTooLarge {
        /// Serialized payload size.
        len: usize,
        /// The cap ([`MAX_PAYLOAD_LEN`]).
        max: u32,
    },
    /// The record contains NaN or an infinity, which JSON cannot
    /// round-trip (see [`WalRecord::values_are_finite`]).
    #[error("record contains a non-finite float, which JSON cannot round-trip")]
    NonFiniteValue,
    /// [`WalWriter::resume`] was given an offset inside the segment
    /// header, which cannot be a valid append position.
    #[error("resume offset {offset} lies inside the {header}-byte segment header")]
    ResumeOffsetTooSmall {
        /// The offset passed to `resume`.
        offset: u64,
        /// Size of the segment header ([`SEGMENT_HEADER_LEN`]).
        header: usize,
    },
}

/// Decoded segment header.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SegmentHeader {
    /// On-disk format version the segment was written with.
    pub version: u32,
    /// Wall-clock creation time, nanoseconds since the Unix epoch.
    pub created_wall_ns: u64,
    /// Host-monotonic creation time, nanoseconds.
    pub created_mono_ns: u64,
}

impl SegmentHeader {
    /// Builds a header for a new segment at the current
    /// ([`FORMAT_VERSION`]) format.
    #[must_use]
    pub const fn new(created_wall_ns: u64, created_mono_ns: u64) -> Self {
        Self {
            version: FORMAT_VERSION,
            created_wall_ns,
            created_mono_ns,
        }
    }

    /// Encodes the header to its fixed 32-byte layout.
    #[must_use]
    pub fn encode(&self) -> [u8; SEGMENT_HEADER_LEN] {
        let mut buf = [0_u8; SEGMENT_HEADER_LEN];
        write_at(&mut buf, 0, &SEGMENT_MAGIC);
        write_at(&mut buf, 8, &self.version.to_le_bytes());
        write_at(&mut buf, 12, &self.created_wall_ns.to_le_bytes());
        write_at(&mut buf, 20, &self.created_mono_ns.to_le_bytes());
        let crc = crc32c(&buf[..SEGMENT_HEADER_LEN - FRAME_TRAILER_LEN]);
        write_at(
            &mut buf,
            SEGMENT_HEADER_LEN - FRAME_TRAILER_LEN,
            &crc.to_le_bytes(),
        );
        buf
    }

    /// Decodes and validates a segment header from the front of `bytes`.
    ///
    /// Corruption is reported via [`ScanEnd`]; the CRC is checked before
    /// the version field, so a corrupted version byte reports as
    /// [`ScanEnd::SegmentHeaderCrcMismatch`], and
    /// [`ScanEnd::UnsupportedVersion`] means a genuine (intact) foreign
    /// version.
    pub fn decode(bytes: &[u8]) -> Result<Self, ScanEnd> {
        if bytes.len() < SEGMENT_HEADER_LEN {
            return Err(ScanEnd::TruncatedSegmentHeader { len: bytes.len() });
        }
        if !bytes.starts_with(&SEGMENT_MAGIC) {
            return Err(ScanEnd::BadSegmentMagic);
        }
        let crc_end = SEGMENT_HEADER_LEN - FRAME_TRAILER_LEN;
        let crc_region = bytes
            .get(..crc_end)
            .ok_or(ScanEnd::TruncatedSegmentHeader { len: bytes.len() })?;
        let stored =
            le_u32(bytes, crc_end).ok_or(ScanEnd::TruncatedSegmentHeader { len: bytes.len() })?;
        if crc32c(crc_region) != stored {
            return Err(ScanEnd::SegmentHeaderCrcMismatch);
        }
        let version = le_u32(bytes, 8).ok_or(ScanEnd::SegmentHeaderCrcMismatch)?;
        if version != FORMAT_VERSION {
            return Err(ScanEnd::UnsupportedVersion { version });
        }
        let created_wall_ns = le_u64(bytes, 12).ok_or(ScanEnd::SegmentHeaderCrcMismatch)?;
        let created_mono_ns = le_u64(bytes, 20).ok_or(ScanEnd::SegmentHeaderCrcMismatch)?;
        Ok(Self {
            version,
            created_wall_ns,
            created_mono_ns,
        })
    }
}

/// Appends framed records to a caller-supplied [`std::io::Write`].
///
/// # Durability is the caller's problem — loudly
///
/// `WalWriter` **never flushes and never syncs**. [`append`](Self::append)
/// hands the encoded frame to the `Write` and nothing more. The daemon
/// owns buffering policy, `flush`, `fdatasync`/`O_DSYNC`, and decides
/// what "durable" means; nothing in this crate does, and no test here
/// pretends to. Records appended but not yet synced by the caller can and
/// will be lost on power failure — the recovery scan is designed around
/// exactly that outcome.
///
/// # Error handling
///
/// If [`append`](Self::append) returns an error after partially writing a
/// frame, the stream tail is torn. The already-written prefix remains
/// recoverable via [`scan`], but this writer instance must be discarded;
/// the internal offset is only advanced on fully successful appends.
#[derive(Debug)]
pub struct WalWriter<W: Write> {
    inner: W,
    offset: u64,
}

impl<W: Write> WalWriter<W> {
    /// Starts a new segment: writes the 32-byte segment header to `inner`
    /// and returns a writer positioned after it.
    pub fn create(mut inner: W, header: &SegmentHeader) -> Result<Self, WalError> {
        inner.write_all(&header.encode())?;
        Ok(Self {
            inner,
            offset: SEGMENT_HEADER_LEN as u64,
        })
    }

    /// Resumes appending to an existing segment.
    ///
    /// `inner` must already be positioned at `offset` bytes into the
    /// segment — normally the [`RecoveryScan::truncation_offset`] after
    /// the caller truncated the file there. This crate cannot verify the
    /// positioning (it does no I/O); the caller owns that invariant.
    pub fn resume(inner: W, offset: u64) -> Result<Self, WalError> {
        if offset < SEGMENT_HEADER_LEN as u64 {
            return Err(WalError::ResumeOffsetTooSmall {
                offset,
                header: SEGMENT_HEADER_LEN,
            });
        }
        Ok(Self { inner, offset })
    }

    /// Appends one record and returns the byte offset (from the start of
    /// the segment) at which its frame begins.
    ///
    /// Rejects records containing non-finite floats
    /// ([`WalError::NonFiniteValue`]) and payloads over
    /// [`MAX_PAYLOAD_LEN`] ([`WalError::PayloadTooLarge`]) before writing
    /// anything.
    pub fn append(&mut self, record: &WalRecord) -> Result<u64, WalError> {
        if !record.values_are_finite() {
            return Err(WalError::NonFiniteValue);
        }
        let payload = serde_json::to_vec(record)?;
        let payload_len = u32::try_from(payload.len())
            .ok()
            .filter(|len| *len <= MAX_PAYLOAD_LEN)
            .ok_or(WalError::PayloadTooLarge {
                len: payload.len(),
                max: MAX_PAYLOAD_LEN,
            })?;

        let mut frame = Vec::with_capacity(FRAME_HEADER_LEN + payload.len() + FRAME_TRAILER_LEN);
        frame.extend_from_slice(&FRAME_MAGIC);
        frame.push(PAYLOAD_FORMAT_JSON);
        frame.push(record.kind().as_u8());
        frame.extend_from_slice(&payload_len.to_le_bytes());
        frame.extend_from_slice(&payload);
        let crc = crc32c(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());

        self.inner.write_all(&frame)?;
        let start = self.offset;
        // Saturating: overflowing u64 would need a 16-EiB log.
        self.offset = self.offset.saturating_add(frame.len() as u64);
        Ok(start)
    }

    /// The offset at which the next appended frame will begin.
    #[must_use]
    pub const fn offset(&self) -> u64 {
        self.offset
    }

    /// Access to the underlying writer, e.g. for the caller to `flush`
    /// before it syncs. Do not write through this between appends.
    pub fn get_mut(&mut self) -> &mut W {
        &mut self.inner
    }

    /// Consumes the writer, returning the underlying `Write`.
    pub fn into_inner(self) -> W {
        self.inner
    }
}

/// Why a scan stopped where it did.
///
/// [`TruncatedSegmentHeader`](Self::TruncatedSegmentHeader),
/// [`TruncatedFrameHeader`](Self::TruncatedFrameHeader), and
/// [`TruncatedPayload`](Self::TruncatedPayload) are the *expected*
/// outcomes after power loss mid-write (see
/// [`is_expected_after_power_loss`](Self::is_expected_after_power_loss));
/// the remaining variants indicate corruption or a foreign/newer format.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ScanEnd {
    /// The log ended exactly at a frame boundary; nothing was torn.
    #[error("log ends cleanly at a frame boundary")]
    CleanEof,
    /// The file is shorter than a segment header — power loss during
    /// initial segment creation, or not a WAL file at all.
    #[error("file is {len} bytes, shorter than the {SEGMENT_HEADER_LEN}-byte segment header")]
    TruncatedSegmentHeader {
        /// Actual file length.
        len: usize,
    },
    /// The first 8 bytes are not [`SEGMENT_MAGIC`]: not a WAL segment.
    #[error("segment magic mismatch: not a WAL segment file")]
    BadSegmentMagic,
    /// The segment header CRC does not match its contents.
    #[error("segment header CRC mismatch")]
    SegmentHeaderCrcMismatch,
    /// The segment was written by a format version this crate does not
    /// read.
    #[error("unsupported WAL format version {version} (this crate reads {FORMAT_VERSION})")]
    UnsupportedVersion {
        /// Version found in the header.
        version: u32,
    },
    /// Fewer than [`FRAME_HEADER_LEN`] bytes remain: a frame header was
    /// torn mid-write.
    #[error("torn frame header at end of log")]
    TruncatedFrameHeader,
    /// The frame header is intact but the payload + CRC run past the end
    /// of the data: a payload was torn mid-write.
    #[error("torn frame payload at end of log")]
    TruncatedPayload,
    /// The bytes at a frame boundary do not start with [`FRAME_MAGIC`].
    #[error("bad frame magic")]
    BadFrameMagic,
    /// The length field exceeds [`MAX_PAYLOAD_LEN`]; the value is not
    /// trusted and nothing was allocated from it.
    #[error("frame declares an absurd payload length of {len} bytes")]
    OversizedLength {
        /// The untrusted length field value.
        len: u32,
    },
    /// Frame CRC mismatch: the frame (header or payload) is corrupt.
    #[error("frame CRC mismatch")]
    FrameCrcMismatch,
    /// CRC-valid frame with a payload format tag this crate cannot
    /// decode (written by a newer format revision).
    #[error("unsupported payload format tag {tag}")]
    UnsupportedPayloadFormat {
        /// The unknown format tag.
        tag: u8,
    },
    /// CRC-valid frame with a record kind tag this crate does not know
    /// (written by a newer format revision).
    #[error("unknown record kind tag {kind}")]
    UnknownRecordKind {
        /// The unknown kind tag.
        kind: u8,
    },
    /// CRC-valid frame whose JSON payload failed to parse as a record.
    #[error("frame payload is not a decodable record")]
    MalformedPayload,
    /// CRC-valid frame whose decoded record disagrees with the kind tag
    /// in the frame header.
    #[error("frame kind tag {header_kind} disagrees with the decoded payload")]
    RecordKindMismatch {
        /// The kind tag from the frame header.
        header_kind: u8,
    },
}

impl ScanEnd {
    /// `true` for outcomes that a power cut mid-write produces on a
    /// correctly-functioning system: a clean end between frames, or a
    /// truncated header/frame/payload at the tail. `false` for outcomes
    /// that imply corruption of *previously durable* bytes or a foreign
    /// format, which deserve alarm rather than routine recovery.
    ///
    /// Note: on storage that preallocates or reorders writes, a tear can
    /// also surface as trailing garbage
    /// ([`BadFrameMagic`](Self::BadFrameMagic) /
    /// [`FrameCrcMismatch`](Self::FrameCrcMismatch) over a zero-filled
    /// tail); the daemon decides how suspicious to be using the offset
    /// and its own knowledge of the file's allocation.
    #[must_use]
    pub const fn is_expected_after_power_loss(&self) -> bool {
        matches!(
            self,
            Self::CleanEof
                | Self::TruncatedSegmentHeader { .. }
                | Self::TruncatedFrameHeader
                | Self::TruncatedPayload
        )
    }
}

/// One successfully decoded record and where its frame started.
#[derive(Debug, Clone, PartialEq)]
pub struct ScannedRecord {
    /// Byte offset of the frame's first byte, from the segment start.
    pub offset: u64,
    /// The decoded record.
    pub record: WalRecord,
}

/// Result of a full recovery scan: the valid prefix and where/why it
/// ends.
#[derive(Debug, Clone, PartialEq)]
pub struct RecoveryScan {
    /// The segment header, when it validated.
    pub header: Option<SegmentHeader>,
    /// Every record in the valid prefix, in append order.
    pub records: Vec<ScannedRecord>,
    /// Byte offset where the valid prefix ends. To resume appending, the
    /// caller truncates the file here and uses [`WalWriter::resume`].
    pub truncation_offset: u64,
    /// Why the scan stopped.
    pub end: ScanEnd,
}

/// Incremental recovery reader over an in-memory WAL segment.
///
/// Yields records (as an [`Iterator`]) until the first invalid frame,
/// then exposes where and why it stopped. Never panics on any input;
/// never allocates from unvalidated lengths.
#[derive(Debug)]
pub struct WalReader<'a> {
    bytes: &'a [u8],
    pos: usize,
    header: Option<SegmentHeader>,
    end: Option<ScanEnd>,
}

impl<'a> WalReader<'a> {
    /// Wraps `bytes` (a whole segment, starting at the segment header)
    /// and validates the header eagerly.
    #[must_use]
    pub fn new(bytes: &'a [u8]) -> Self {
        match SegmentHeader::decode(bytes) {
            Ok(header) => Self {
                bytes,
                pos: SEGMENT_HEADER_LEN,
                header: Some(header),
                end: None,
            },
            Err(end) => Self {
                bytes,
                pos: 0,
                header: None,
                end: Some(end),
            },
        }
    }

    /// The validated segment header, or `None` if it was invalid.
    #[must_use]
    pub const fn segment_header(&self) -> Option<&SegmentHeader> {
        self.header.as_ref()
    }

    /// Byte offset of the end of the valid prefix read so far.
    #[must_use]
    pub fn position(&self) -> u64 {
        self.pos as u64
    }

    /// Why reading stopped; `None` while records may still remain.
    #[must_use]
    pub const fn end(&self) -> Option<&ScanEnd> {
        self.end.as_ref()
    }

    /// Decodes the next record, or records why the log ends here and
    /// returns `None` (thereafter always `None`).
    pub fn next_record(&mut self) -> Option<ScannedRecord> {
        if self.end.is_some() {
            return None;
        }
        let remaining = self.bytes.get(self.pos..).unwrap_or(&[]);
        if remaining.is_empty() {
            self.end = Some(ScanEnd::CleanEof);
            return None;
        }
        match parse_frame(remaining) {
            Ok((record, frame_len)) => {
                let offset = self.pos as u64;
                self.pos = self.pos.saturating_add(frame_len);
                Some(ScannedRecord { offset, record })
            }
            Err(end) => {
                self.end = Some(end);
                None
            }
        }
    }
}

impl Iterator for WalReader<'_> {
    type Item = ScannedRecord;

    fn next(&mut self) -> Option<Self::Item> {
        self.next_record()
    }
}

/// Parses one frame from the front of `remaining`, returning the record
/// and the total frame length, or the reason the frame is invalid.
fn parse_frame(remaining: &[u8]) -> Result<(WalRecord, usize), ScanEnd> {
    if remaining.len() < FRAME_HEADER_LEN {
        return Err(ScanEnd::TruncatedFrameHeader);
    }
    if !remaining.starts_with(&FRAME_MAGIC) {
        return Err(ScanEnd::BadFrameMagic);
    }
    // Bounds are established above; the ok_or arms are unreachable belt
    // and braces, mapped to the same truncation reason.
    let format_tag = remaining
        .get(2)
        .copied()
        .ok_or(ScanEnd::TruncatedFrameHeader)?;
    let kind_tag = remaining
        .get(3)
        .copied()
        .ok_or(ScanEnd::TruncatedFrameHeader)?;
    let payload_len = le_u32(remaining, 4).ok_or(ScanEnd::TruncatedFrameHeader)?;

    // Validate the length before using it for anything.
    if payload_len > MAX_PAYLOAD_LEN {
        return Err(ScanEnd::OversizedLength { len: payload_len });
    }
    let payload_len_usize =
        usize::try_from(payload_len).map_err(|_| ScanEnd::OversizedLength { len: payload_len })?;
    let crc_region_len = FRAME_HEADER_LEN + payload_len_usize; // <= 8 + 1 MiB: cannot overflow
    let frame_len = crc_region_len + FRAME_TRAILER_LEN;
    if remaining.len() < frame_len {
        return Err(ScanEnd::TruncatedPayload);
    }

    let crc_region = remaining
        .get(..crc_region_len)
        .ok_or(ScanEnd::TruncatedPayload)?;
    let stored_crc = le_u32(remaining, crc_region_len).ok_or(ScanEnd::TruncatedPayload)?;
    if crc32c(crc_region) != stored_crc {
        return Err(ScanEnd::FrameCrcMismatch);
    }

    // From here on the frame is authentic; disagreements are format
    // evolution, not corruption.
    if format_tag != PAYLOAD_FORMAT_JSON {
        return Err(ScanEnd::UnsupportedPayloadFormat { tag: format_tag });
    }
    let Some(expected_kind) = RecordKind::from_u8(kind_tag) else {
        return Err(ScanEnd::UnknownRecordKind { kind: kind_tag });
    };
    let payload = crc_region
        .get(FRAME_HEADER_LEN..)
        .ok_or(ScanEnd::MalformedPayload)?;
    let record: WalRecord =
        serde_json::from_slice(payload).map_err(|_| ScanEnd::MalformedPayload)?;
    if record.kind() != expected_kind {
        return Err(ScanEnd::RecordKindMismatch {
            header_kind: kind_tag,
        });
    }
    Ok((record, frame_len))
}

/// Scans a whole in-memory segment and returns the valid prefix plus the
/// truncation point and reason. Never panics on any input.
#[must_use]
pub fn scan(bytes: &[u8]) -> RecoveryScan {
    let mut reader = WalReader::new(bytes);
    let mut records = Vec::new();
    while let Some(record) = reader.next_record() {
        records.push(record);
    }
    let end = reader.end().cloned().unwrap_or(ScanEnd::CleanEof);
    RecoveryScan {
        header: reader.segment_header().cloned(),
        records,
        truncation_offset: reader.position(),
        end,
    }
}

/// Reads at most `max_len` bytes from `reader` into memory and scans
/// them.
///
/// `max_len` caps the allocation (a WAL segment is a few MB at most; the
/// daemon knows its own rotation size). Bytes past `max_len` are treated
/// exactly like a file truncated there.
pub fn scan_read<R: Read>(reader: R, max_len: u64) -> std::io::Result<RecoveryScan> {
    let mut buf = Vec::new();
    reader.take(max_len).read_to_end(&mut buf)?;
    Ok(scan(&buf))
}

#[cfg(test)]
mod tests {
    use super::{
        scan, scan_read, RecoveryScan, ScanEnd, SegmentHeader, WalError, WalReader, WalWriter,
        FORMAT_VERSION, FRAME_HEADER_LEN, FRAME_MAGIC, FRAME_TRAILER_LEN, MAX_PAYLOAD_LEN,
        PAYLOAD_FORMAT_JSON, SEGMENT_HEADER_LEN, SEGMENT_MAGIC,
    };
    use crate::crc32c::crc32c;
    use crate::record::samples::{
        sample_context, sample_heartbeat, sample_marker, sample_stepper, sample_trapq,
    };
    use crate::record::{Marker, MarkerKind, WalRecord};

    fn sample_records() -> Vec<WalRecord> {
        vec![
            WalRecord::TrapqSegment(sample_trapq()),
            WalRecord::StepperRange(sample_stepper()),
            WalRecord::Context(sample_context()),
            WalRecord::Marker(sample_marker()),
            WalRecord::Heartbeat(sample_heartbeat()),
        ]
    }

    fn sample_header() -> SegmentHeader {
        SegmentHeader::new(1_760_000_000_000_000_000, 987_654_321)
    }

    /// Builds a log of `records`, returning (bytes, per-record offsets,
    /// final offset).
    fn build_log(records: &[WalRecord]) -> (Vec<u8>, Vec<u64>, u64) {
        let mut writer = WalWriter::create(Vec::new(), &sample_header()).unwrap();
        let offsets = records.iter().map(|r| writer.append(r).unwrap()).collect();
        let end = writer.offset();
        (writer.into_inner(), offsets, end)
    }

    fn assert_clean(scan_result: &RecoveryScan, expected: &[WalRecord]) {
        assert_eq!(scan_result.end, ScanEnd::CleanEof);
        assert_eq!(scan_result.header, Some(sample_header()));
        let decoded: Vec<_> = scan_result
            .records
            .iter()
            .map(|r| r.record.clone())
            .collect();
        assert_eq!(decoded, expected);
    }

    #[test]
    fn empty_input_reports_truncated_segment_header() {
        let result = scan(&[]);
        assert_eq!(result.header, None);
        assert!(result.records.is_empty());
        assert_eq!(result.truncation_offset, 0);
        assert_eq!(result.end, ScanEnd::TruncatedSegmentHeader { len: 0 });
        assert!(result.end.is_expected_after_power_loss());
    }

    #[test]
    fn partial_segment_header_reports_truncation() {
        let bytes = sample_header().encode();
        let result = scan(&bytes[..17]);
        assert_eq!(result.end, ScanEnd::TruncatedSegmentHeader { len: 17 });
        assert_eq!(result.truncation_offset, 0);
    }

    #[test]
    fn header_only_log_scans_clean_and_empty() {
        let (bytes, offsets, end) = build_log(&[]);
        assert_eq!(bytes.len(), SEGMENT_HEADER_LEN);
        assert!(offsets.is_empty());
        assert_eq!(end, SEGMENT_HEADER_LEN as u64);
        let result = scan(&bytes);
        assert_clean(&result, &[]);
        assert_eq!(result.truncation_offset, SEGMENT_HEADER_LEN as u64);
    }

    #[test]
    fn bad_segment_magic_is_rejected() {
        let mut bytes = sample_header().encode().to_vec();
        bytes[0] = b'X';
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::BadSegmentMagic);
        assert!(!result.end.is_expected_after_power_loss());
        assert_eq!(result.header, None);
    }

    #[test]
    fn corrupt_segment_header_crc_is_rejected() {
        let mut bytes = sample_header().encode().to_vec();
        bytes[13] ^= 0x01; // flip a bit in created_wall_ns
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::SegmentHeaderCrcMismatch);
    }

    #[test]
    fn foreign_version_with_valid_crc_is_rejected_as_unsupported() {
        let header = SegmentHeader {
            version: FORMAT_VERSION + 7,
            created_wall_ns: 1,
            created_mono_ns: 2,
        };
        let result = scan(&header.encode());
        assert_eq!(
            result.end,
            ScanEnd::UnsupportedVersion {
                version: FORMAT_VERSION + 7
            }
        );
    }

    #[test]
    fn corrupted_version_field_reports_crc_not_version() {
        let mut bytes = sample_header().encode().to_vec();
        bytes[8] ^= 0xFF;
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::SegmentHeaderCrcMismatch);
    }

    #[test]
    fn n_records_roundtrip_through_scan() {
        let records = sample_records();
        let (bytes, offsets, end) = build_log(&records);
        let result = scan(&bytes);
        assert_clean(&result, &records);
        assert_eq!(result.truncation_offset, end);
        assert_eq!(end, bytes.len() as u64);
        let scanned_offsets: Vec<_> = result.records.iter().map(|r| r.offset).collect();
        assert_eq!(scanned_offsets, offsets);
        assert_eq!(offsets[0], SEGMENT_HEADER_LEN as u64);
    }

    #[test]
    fn truncation_mid_frame_header_yields_valid_prefix() {
        let records = sample_records();
        let (bytes, offsets, _) = build_log(&records);
        // Cut 3 bytes into the third record's frame header.
        let cut = usize::try_from(offsets[2]).unwrap() + 3;
        let result = scan(&bytes[..cut]);
        assert_eq!(result.end, ScanEnd::TruncatedFrameHeader);
        assert!(result.end.is_expected_after_power_loss());
        assert_eq!(result.records.len(), 2);
        assert_eq!(result.truncation_offset, offsets[2]);
    }

    #[test]
    fn truncation_mid_payload_yields_valid_prefix() {
        let records = sample_records();
        let (bytes, offsets, _) = build_log(&records);
        // Cut into the second record's payload, past its frame header.
        let cut = usize::try_from(offsets[1]).unwrap() + FRAME_HEADER_LEN + 10;
        let result = scan(&bytes[..cut]);
        assert_eq!(result.end, ScanEnd::TruncatedPayload);
        assert!(result.end.is_expected_after_power_loss());
        assert_eq!(result.records.len(), 1);
        assert_eq!(result.records[0].record, records[0]);
        assert_eq!(result.truncation_offset, offsets[1]);
    }

    #[test]
    fn corrupt_crc_in_middle_preserves_prefix_and_reports_offset() {
        let records = sample_records();
        let (mut bytes, offsets, _) = build_log(&records);
        // Corrupt one payload byte of the third record.
        let victim = usize::try_from(offsets[2]).unwrap() + FRAME_HEADER_LEN + 5;
        bytes[victim] ^= 0x40;
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::FrameCrcMismatch);
        assert!(!result.end.is_expected_after_power_loss());
        assert_eq!(result.records.len(), 2);
        assert_eq!(result.records[0].record, records[0]);
        assert_eq!(result.records[1].record, records[1]);
        assert_eq!(result.truncation_offset, offsets[2]);
    }

    #[test]
    fn garbage_after_valid_records_is_not_invented_into_records() {
        let records = sample_records();
        let (mut bytes, _, end) = build_log(&records);
        bytes.extend_from_slice(&[0xFF; 64]);
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::BadFrameMagic);
        assert_eq!(result.records.len(), records.len());
        assert_eq!(result.truncation_offset, end);
    }

    #[test]
    fn zero_fill_after_valid_records_reports_bad_magic() {
        // Preallocated-file shape: valid prefix, then a zeroed tail.
        let records = sample_records();
        let (mut bytes, _, end) = build_log(&records);
        bytes.extend_from_slice(&[0x00; 256]);
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::BadFrameMagic);
        assert_eq!(result.truncation_offset, end);
    }

    /// Handcrafts a frame with the given header fields and payload,
    /// with a *correct* CRC.
    fn forge_frame(format_tag: u8, kind_tag: u8, payload: &[u8]) -> Vec<u8> {
        let mut frame = Vec::new();
        frame.extend_from_slice(&FRAME_MAGIC);
        frame.push(format_tag);
        frame.push(kind_tag);
        frame.extend_from_slice(&u32::try_from(payload.len()).unwrap().to_le_bytes());
        frame.extend_from_slice(payload);
        let crc = crc32c(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());
        frame
    }

    #[test]
    fn absurd_length_field_is_rejected_without_allocation() {
        let (mut bytes, _, end) = build_log(&sample_records()[..1]);
        // Frame header claiming a u32::MAX-byte payload.
        bytes.extend_from_slice(&FRAME_MAGIC);
        bytes.push(PAYLOAD_FORMAT_JSON);
        bytes.push(1);
        bytes.extend_from_slice(&u32::MAX.to_le_bytes());
        bytes.extend_from_slice(&[0xAB; 32]);
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::OversizedLength { len: u32::MAX });
        assert_eq!(result.records.len(), 1);
        assert_eq!(result.truncation_offset, end);
    }

    #[test]
    fn length_just_over_cap_is_rejected_but_cap_is_allowed_semantics() {
        let (mut bytes, _, _) = build_log(&[]);
        bytes.extend_from_slice(&FRAME_MAGIC);
        bytes.push(PAYLOAD_FORMAT_JSON);
        bytes.push(1);
        bytes.extend_from_slice(&(MAX_PAYLOAD_LEN + 1).to_le_bytes());
        let result = scan(&bytes);
        assert_eq!(
            result.end,
            ScanEnd::OversizedLength {
                len: MAX_PAYLOAD_LEN + 1
            }
        );
    }

    #[test]
    fn unsupported_payload_format_tag_stops_scan() {
        let (mut bytes, _, end) = build_log(&[]);
        let payload = serde_json::to_vec(&WalRecord::Marker(sample_marker())).unwrap();
        bytes.extend_from_slice(&forge_frame(99, 4, &payload));
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::UnsupportedPayloadFormat { tag: 99 });
        assert_eq!(result.truncation_offset, end);
    }

    #[test]
    fn unknown_record_kind_tag_stops_scan() {
        let (mut bytes, _, _) = build_log(&[]);
        let payload = serde_json::to_vec(&WalRecord::Marker(sample_marker())).unwrap();
        bytes.extend_from_slice(&forge_frame(PAYLOAD_FORMAT_JSON, 200, &payload));
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::UnknownRecordKind { kind: 200 });
    }

    #[test]
    fn crc_valid_garbage_json_reports_malformed_payload() {
        let (mut bytes, _, _) = build_log(&[]);
        bytes.extend_from_slice(&forge_frame(PAYLOAD_FORMAT_JSON, 4, b"{\"type\":\"Nope\"}"));
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::MalformedPayload);
    }

    #[test]
    fn kind_tag_payload_disagreement_is_rejected() {
        let (mut bytes, _, _) = build_log(&[]);
        let payload = serde_json::to_vec(&WalRecord::Marker(sample_marker())).unwrap();
        // Header claims Heartbeat (5); payload is a Marker.
        bytes.extend_from_slice(&forge_frame(PAYLOAD_FORMAT_JSON, 5, &payload));
        let result = scan(&bytes);
        assert_eq!(result.end, ScanEnd::RecordKindMismatch { header_kind: 5 });
    }

    #[test]
    fn writer_rejects_non_finite_records_before_writing() {
        let mut writer = WalWriter::create(Vec::new(), &sample_header()).unwrap();
        let mut bad = sample_trapq();
        bad.acceleration = f64::NAN;
        let before = writer.offset();
        let err = writer.append(&WalRecord::TrapqSegment(bad)).unwrap_err();
        assert!(matches!(err, WalError::NonFiniteValue));
        assert_eq!(writer.offset(), before);
        assert_eq!(writer.get_mut().len(), SEGMENT_HEADER_LEN);
    }

    #[test]
    fn writer_rejects_oversized_payloads_before_writing() {
        let mut writer = WalWriter::create(Vec::new(), &sample_header()).unwrap();
        let mut huge = sample_context();
        huge.virtual_sdcard = Some(crate::record::VirtualSdState {
            file_path: "g".repeat(usize::try_from(MAX_PAYLOAD_LEN).unwrap() + 1),
            file_position: 0,
        });
        let err = writer.append(&WalRecord::Context(huge)).unwrap_err();
        match err {
            WalError::PayloadTooLarge { len, max } => {
                assert!(len > usize::try_from(MAX_PAYLOAD_LEN).unwrap());
                assert_eq!(max, MAX_PAYLOAD_LEN);
            }
            other => panic!("wrong error: {other}"),
        }
        assert_eq!(writer.get_mut().len(), SEGMENT_HEADER_LEN);
    }

    #[test]
    fn writer_io_error_does_not_advance_offset() {
        struct FailingWriter;
        impl std::io::Write for FailingWriter {
            fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("disk gone"))
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let mut writer = WalWriter::resume(FailingWriter, SEGMENT_HEADER_LEN as u64).unwrap();
        let before = writer.offset();
        let err = writer
            .append(&WalRecord::Marker(sample_marker()))
            .unwrap_err();
        assert!(matches!(err, WalError::Io(_)));
        assert_eq!(writer.offset(), before);
    }

    #[test]
    fn resume_appends_where_scan_left_off() {
        let records = sample_records();
        let (bytes, _, end) = build_log(&records[..2]);
        let mut writer = WalWriter::resume(bytes, end).unwrap();
        let offset = writer.append(&records[2]).unwrap();
        assert_eq!(offset, end);
        let result = scan(&writer.into_inner());
        let decoded: Vec<_> = result.records.into_iter().map(|r| r.record).collect();
        assert_eq!(decoded, records[..3].to_vec());
        assert_eq!(result.end, ScanEnd::CleanEof);
    }

    #[test]
    fn resume_inside_segment_header_is_rejected() {
        let err = WalWriter::resume(Vec::new(), 5).unwrap_err();
        assert!(matches!(
            err,
            WalError::ResumeOffsetTooSmall { offset: 5, header } if header == SEGMENT_HEADER_LEN
        ));
    }

    #[test]
    fn reader_iterator_and_accessors_agree_with_scan() {
        let records = sample_records();
        let (bytes, _, end) = build_log(&records);
        let mut reader = WalReader::new(&bytes);
        assert_eq!(reader.segment_header(), Some(&sample_header()));
        assert_eq!(reader.end(), None);
        let collected: Vec<_> = reader.by_ref().map(|r| r.record).collect();
        assert_eq!(collected, records);
        assert_eq!(reader.position(), end);
        assert_eq!(reader.end(), Some(&ScanEnd::CleanEof));
        // Exhausted reader stays exhausted.
        assert!(reader.next_record().is_none());
        assert_eq!(reader.end(), Some(&ScanEnd::CleanEof));
    }

    #[test]
    fn scan_read_caps_input_like_truncation() {
        let records = sample_records();
        let (bytes, offsets, _) = build_log(&records);
        let full = scan_read(std::io::Cursor::new(&bytes), u64::MAX).unwrap();
        assert_clean(&full, &records);
        // Cap mid-way through the log: behaves like a truncated file.
        let cap = offsets[3] + 4;
        let capped = scan_read(std::io::Cursor::new(&bytes), cap).unwrap();
        assert_eq!(capped.records.len(), 3);
        assert_eq!(capped.end, ScanEnd::TruncatedFrameHeader);
    }

    #[test]
    fn scan_read_propagates_io_errors() {
        struct FailingReader;
        impl std::io::Read for FailingReader {
            fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("cable chewed"))
            }
        }
        assert!(scan_read(FailingReader, 1024).is_err());
    }

    #[test]
    fn segment_header_encode_layout_is_pinned() {
        let header = sample_header();
        let bytes = header.encode();
        assert_eq!(&bytes[..8], &SEGMENT_MAGIC);
        assert_eq!(&bytes[8..12], &FORMAT_VERSION.to_le_bytes());
        assert_eq!(&bytes[12..20], &header.created_wall_ns.to_le_bytes());
        assert_eq!(&bytes[20..28], &header.created_mono_ns.to_le_bytes());
        assert_eq!(&bytes[28..32], &crc32c(&bytes[..28]).to_le_bytes());
        assert_eq!(SegmentHeader::decode(&bytes), Ok(header));
    }

    #[test]
    fn frame_layout_is_pinned() {
        let record = WalRecord::Marker(Marker {
            mono_ns: 1,
            kind: MarkerKind::CleanShutdown,
        });
        let (bytes, offsets, end) = build_log(std::slice::from_ref(&record));
        let frame = &bytes[usize::try_from(offsets[0]).unwrap()..usize::try_from(end).unwrap()];
        assert_eq!(&frame[..2], &FRAME_MAGIC);
        assert_eq!(frame[2], PAYLOAD_FORMAT_JSON);
        assert_eq!(frame[3], record.kind().as_u8());
        let payload_len = u32::from_le_bytes(frame[4..8].try_into().unwrap());
        let payload = &frame[8..8 + usize::try_from(payload_len).unwrap()];
        assert_eq!(
            serde_json::from_slice::<WalRecord>(payload).unwrap(),
            record
        );
        let crc_end = FRAME_HEADER_LEN + usize::try_from(payload_len).unwrap();
        assert_eq!(
            &frame[crc_end..crc_end + FRAME_TRAILER_LEN],
            &crc32c(&frame[..crc_end]).to_le_bytes()
        );
    }

    #[test]
    fn scan_end_display_messages_are_meaningful() {
        assert_eq!(
            ScanEnd::TruncatedSegmentHeader { len: 3 }.to_string(),
            "file is 3 bytes, shorter than the 32-byte segment header"
        );
        assert!(ScanEnd::OversizedLength { len: 4_294_967_295 }
            .to_string()
            .contains("4294967295"));
        assert!(WalError::NonFiniteValue.to_string().contains("non-finite"));
    }
}
