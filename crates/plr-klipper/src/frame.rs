//! Incremental splitter for Klipper API socket frames.
//!
//! Klipper's Unix API socket exchanges JSON messages terminated by a single
//! `0x03` (ETX) byte (`klippy/webhooks.py`: `ClientConnection.send` appends
//! `b"\x03"`, `ClientConnection.process_received` splits on `b'\x03'`).
//! [`FrameSplitter`] performs the receive-side split incrementally: feed it
//! arbitrary byte chunks as they arrive from the socket and it yields
//! complete frame bodies (without the terminator), regardless of how the
//! chunk boundaries fall.
//!
//! Guarantees:
//!
//! * **Chunking independence** — any partitioning of the same byte stream
//!   yields the same sequence of [`FrameEvent`]s (proptest-enforced).
//! * **Totality** — never panics, for any input bytes.
//! * **Bounded memory** — a frame longer than the configured cap is
//!   discarded (reported as [`FrameEvent::Oversized`]) and the splitter
//!   resynchronizes at the next terminator. Buffered data never exceeds the
//!   cap.

/// The frame terminator byte (ASCII ETX) used by Klipper's API server
/// (`klippy/webhooks.py`).
pub const ETX: u8 = 0x03;

/// Default maximum accepted frame length in bytes (terminator excluded).
///
/// The largest routine payloads on this socket are `bed_mesh` status
/// updates (`probed_matrix` + `mesh_matrix`, tens of kilobytes for dense
/// meshes) and `motion_report` dump batches (a 0.5 s batch is a few
/// kilobytes). 8 MiB is orders of magnitude above anything Klipper emits
/// while still bounding memory if the peer misbehaves.
pub const DEFAULT_MAX_FRAME_LEN: usize = 8 * 1024 * 1024;

/// An event produced by [`FrameSplitter::feed`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameEvent {
    /// A complete frame body (terminator not included). May contain any
    /// bytes; JSON validation happens later in
    /// [`classify`](crate::message::classify).
    Frame(Vec<u8>),
    /// A frame exceeded the length cap and was discarded in its entirety.
    /// Emitted once per oversized frame, when its terminator is reached;
    /// the splitter then resumes with the next frame.
    Oversized {
        /// Total length in bytes of the discarded frame body.
        discarded_len: u64,
    },
}

/// Incremental 0x03-terminated frame splitter.
///
/// Zero-length frames (consecutive terminators) are skipped silently: ETX
/// is a terminator rather than a separator, an empty body cannot be valid
/// JSON, and Klipper never emits one.
#[derive(Debug, Clone)]
pub struct FrameSplitter {
    /// Bytes of the current partial frame. Invariant: `buf.len()` never
    /// exceeds `max_frame_len`.
    buf: Vec<u8>,
    max_frame_len: usize,
    /// True while skipping the remainder of an oversized frame.
    discarding: bool,
    /// Bytes discarded so far from the oversized frame being skipped.
    discarded_len: u64,
}

impl Default for FrameSplitter {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameSplitter {
    /// Creates a splitter with [`DEFAULT_MAX_FRAME_LEN`].
    #[must_use]
    pub fn new() -> Self {
        Self::with_max_frame_len(DEFAULT_MAX_FRAME_LEN)
    }

    /// Creates a splitter that accepts frame bodies up to `max_frame_len`
    /// bytes. Longer frames are discarded and reported as
    /// [`FrameEvent::Oversized`]. A cap of 0 rejects every non-empty frame.
    #[must_use]
    pub fn with_max_frame_len(max_frame_len: usize) -> Self {
        Self {
            buf: Vec::new(),
            max_frame_len,
            discarding: false,
            discarded_len: 0,
        }
    }

    /// Number of bytes currently buffered for an incomplete frame.
    #[must_use]
    pub fn pending_len(&self) -> usize {
        self.buf.len()
    }

    /// True while the splitter is skipping an oversized frame.
    #[must_use]
    pub fn is_discarding(&self) -> bool {
        self.discarding
    }

    /// Feeds one chunk of received bytes and returns every event completed
    /// by it, in stream order.
    pub fn feed(&mut self, chunk: &[u8]) -> Vec<FrameEvent> {
        let mut events = Vec::new();
        let mut rest = chunk;
        while let Some(pos) = rest.iter().position(|&b| b == ETX) {
            let head = &rest[..pos];
            rest = &rest[pos + 1..];
            self.complete_frame(head, &mut events);
        }
        // Trailing bytes with no terminator yet: buffer or keep discarding.
        if self.discarding {
            self.discarded_len = self.discarded_len.saturating_add(rest.len() as u64);
        } else if exceeds(self.buf.len(), rest.len(), self.max_frame_len) {
            self.discarded_len = (self.buf.len() as u64).saturating_add(rest.len() as u64);
            self.buf.clear();
            self.discarding = true;
        } else {
            self.buf.extend_from_slice(rest);
        }
        events
    }

    /// Handles the bytes `head` that complete a frame at a terminator.
    fn complete_frame(&mut self, head: &[u8], events: &mut Vec<FrameEvent>) {
        if self.discarding {
            let total = self.discarded_len.saturating_add(head.len() as u64);
            self.discarding = false;
            self.discarded_len = 0;
            events.push(FrameEvent::Oversized {
                discarded_len: total,
            });
        } else if exceeds(self.buf.len(), head.len(), self.max_frame_len) {
            let total = (self.buf.len() as u64).saturating_add(head.len() as u64);
            self.buf.clear();
            events.push(FrameEvent::Oversized {
                discarded_len: total,
            });
        } else if self.buf.is_empty() && head.is_empty() {
            // Zero-length frame: skip (see type-level docs).
        } else {
            let mut frame = std::mem::take(&mut self.buf);
            frame.extend_from_slice(head);
            events.push(FrameEvent::Frame(frame));
        }
    }
}

/// True when `a + b > cap`, computed without usize overflow.
fn exceeds(a: usize, b: usize, cap: usize) -> bool {
    (a as u64).saturating_add(b as u64) > cap as u64
}

#[cfg(test)]
mod tests {
    use super::{FrameEvent, FrameSplitter, DEFAULT_MAX_FRAME_LEN, ETX};

    fn frames(events: Vec<FrameEvent>) -> Vec<Vec<u8>> {
        events
            .into_iter()
            .map(|e| match e {
                FrameEvent::Frame(f) => f,
                FrameEvent::Oversized { .. } => panic!("unexpected oversized event"),
            })
            .collect()
    }

    #[test]
    fn single_frame_single_chunk() {
        let mut s = FrameSplitter::new();
        let events = s.feed(b"{\"id\":1}\x03");
        assert_eq!(frames(events), vec![b"{\"id\":1}".to_vec()]);
        assert_eq!(s.pending_len(), 0);
    }

    #[test]
    fn default_uses_default_cap() {
        let s = FrameSplitter::default();
        assert_eq!(s.max_frame_len, DEFAULT_MAX_FRAME_LEN);
        assert!(!s.is_discarding());
    }

    #[test]
    fn frame_split_across_one_byte_feeds() {
        let mut s = FrameSplitter::new();
        let msg = b"{\"key\": \"value\"}\x03";
        let mut got = Vec::new();
        for &b in msg {
            got.extend(s.feed(&[b]));
        }
        assert_eq!(frames(got), vec![b"{\"key\": \"value\"}".to_vec()]);
    }

    #[test]
    fn multiple_frames_in_one_chunk() {
        let mut s = FrameSplitter::new();
        let events = s.feed(b"a\x03bb\x03ccc\x03");
        assert_eq!(
            frames(events),
            vec![b"a".to_vec(), b"bb".to_vec(), b"ccc".to_vec()]
        );
    }

    #[test]
    fn partial_frame_carries_across_chunks() {
        let mut s = FrameSplitter::new();
        assert!(s.feed(b"hel").is_empty());
        assert_eq!(s.pending_len(), 3);
        let events = s.feed(b"lo\x03wor");
        assert_eq!(frames(events), vec![b"hello".to_vec()]);
        assert_eq!(s.pending_len(), 3);
        let events = s.feed(b"ld\x03");
        assert_eq!(frames(events), vec![b"world".to_vec()]);
    }

    #[test]
    fn frame_split_mid_utf8_multibyte_sequence() {
        // "héllo" with the two-byte UTF-8 for é split across chunks.
        let mut s = FrameSplitter::new();
        let bytes = "héllo".as_bytes();
        assert!(s.feed(&bytes[..2]).is_empty()); // 'h' + first byte of é
        let mut events = s.feed(&bytes[2..]);
        events.extend(s.feed(&[ETX]));
        assert_eq!(frames(events), vec!["héllo".as_bytes().to_vec()]);
    }

    #[test]
    fn empty_frames_are_skipped() {
        let mut s = FrameSplitter::new();
        let events = s.feed(b"\x03\x03a\x03\x03");
        assert_eq!(frames(events), vec![b"a".to_vec()]);
    }

    #[test]
    fn empty_chunk_is_a_no_op() {
        let mut s = FrameSplitter::new();
        assert!(s.feed(b"").is_empty());
        assert!(s.feed(b"x").is_empty());
        assert!(s.feed(b"").is_empty());
        assert_eq!(frames(s.feed(b"\x03")), vec![b"x".to_vec()]);
    }

    #[test]
    fn oversized_frame_in_one_chunk_is_discarded_with_recovery() {
        let mut s = FrameSplitter::with_max_frame_len(4);
        let events = s.feed(b"toolong\x03ok\x03");
        assert_eq!(
            events,
            vec![
                FrameEvent::Oversized { discarded_len: 7 },
                FrameEvent::Frame(b"ok".to_vec()),
            ]
        );
    }

    #[test]
    fn oversized_frame_streamed_bytewise_is_discarded_with_recovery() {
        let mut s = FrameSplitter::with_max_frame_len(4);
        let mut got = Vec::new();
        for &b in b"abcdefghij\x03ok\x03" {
            got.extend(s.feed(&[b]));
        }
        assert_eq!(
            got,
            vec![
                FrameEvent::Oversized { discarded_len: 10 },
                FrameEvent::Frame(b"ok".to_vec()),
            ]
        );
        assert!(!s.is_discarding());
    }

    #[test]
    fn discard_state_reported_while_skipping() {
        let mut s = FrameSplitter::with_max_frame_len(2);
        assert!(s.feed(b"abcdef").is_empty());
        assert!(s.is_discarding());
        assert_eq!(s.pending_len(), 0);
        let events = s.feed(b"gh\x03");
        assert_eq!(events, vec![FrameEvent::Oversized { discarded_len: 8 }]);
        assert!(!s.is_discarding());
    }

    #[test]
    fn frame_of_exactly_max_len_is_accepted() {
        let mut s = FrameSplitter::with_max_frame_len(4);
        let events = s.feed(b"abcd\x03");
        assert_eq!(frames(events), vec![b"abcd".to_vec()]);
    }

    #[test]
    fn zero_cap_rejects_every_nonempty_frame() {
        let mut s = FrameSplitter::with_max_frame_len(0);
        let events = s.feed(b"a\x03\x03b\x03");
        assert_eq!(
            events,
            vec![
                FrameEvent::Oversized { discarded_len: 1 },
                FrameEvent::Oversized { discarded_len: 1 },
            ]
        );
    }

    #[test]
    fn back_to_back_oversized_frames_each_reported() {
        let mut s = FrameSplitter::with_max_frame_len(1);
        let events = s.feed(b"xx\x03yyy\x03z\x03");
        assert_eq!(
            events,
            vec![
                FrameEvent::Oversized { discarded_len: 2 },
                FrameEvent::Oversized { discarded_len: 3 },
                FrameEvent::Frame(b"z".to_vec()),
            ]
        );
    }
}
