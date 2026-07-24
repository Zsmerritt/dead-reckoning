//! Append-only motion write-ahead log for power-loss recovery: record
//! formats, encoding/decoding, and integrity checking for durable
//! print-state journaling.
//!
//! # Pure logic, no I/O
//!
//! This crate performs **no syscalls and no file I/O**. Every API operates
//! on byte slices, in-memory buffers, or caller-supplied
//! [`std::io::Write`]/[`std::io::Read`] values. Durability — file handles,
//! `fdatasync`, `O_DSYNC`, write ordering — is owned entirely by the `plrd`
//! daemon and is never mocked here.

pub mod crc32c;
pub mod frame;
pub mod heartbeat;
pub mod record;

mod bytes;

pub use crc32c::{crc32c, Crc32c};
pub use frame::{
    scan, scan_read, RecoveryScan, ScanEnd, ScannedRecord, SegmentHeader, WalError, WalReader,
    WalWriter,
};
pub use heartbeat::{
    decode_slot, encode_slot, recover_heartbeat, slot_for_sequence, HeartbeatError,
    HeartbeatRecovery, SlotError, SlotId,
};
pub use record::{
    Context, ExcludeObjectDef, ExcludeState, FanTarget, GcodeState, Heartbeat, HeaterTarget,
    Marker, MarkerKind, PolygonFidelity, RecordKind, StepChunk, StepperRange,
    TransformObservations, TrapqSegment, VirtualSdState, WalRecord, MAX_POLYGON_POINTS,
};
