//! Pure-logic model of Klipper's Unix API socket protocol: wire framing,
//! typed request construction, response/notification classification,
//! typed status and motion-dump payloads, and clock-correlation math.
//!
//! No sockets and no I/O live here — the daemon (`plrd`) owns the
//! transport and feeds received bytes into this crate.
//!
//! Every payload shape is grounded in the Klipper source; each type's
//! documentation cites the file (relative to a Klipper checkout, e.g.
//! `klippy/webhooks.py`) and function that emits it.
//!
//! # Typical receive path
//!
//! 1. Feed socket bytes into [`frame::FrameSplitter`] to recover
//!    `0x03`-terminated frames.
//! 2. Pass each frame to [`message::classify`] to obtain responses,
//!    errors, or notifications.
//! 3. Parse notification payloads with the typed helpers
//!    ([`message::Notification::status_update`],
//!    [`message::Notification::trapq_batch`], ...).
//! 4. Correlate time axes with [`clock::ClockCorrelator`] /
//!    [`clock::McuClock`], and widen `receive_seq` with
//!    [`clock::ReceiveSeqWidener`].

pub mod clock;
pub mod dump;
pub mod error;
pub mod frame;
pub mod message;
pub mod request;
pub mod status;

pub use clock::{ClockCorrelator, McuClock, ReceiveSeqWidener, SampleOutcome, SeqKind, SeqUpdate};
pub use dump::{StepperBatch, StepperStep, TrapqBatch, TrapqMove};
pub use error::{ClockError, EncodeError, MessageError};
pub use frame::{FrameEvent, FrameSplitter, DEFAULT_MAX_FRAME_LEN, ETX};
pub use message::{
    classify, ApiError, DumpHeader, GcodeOutput, Inbound, InfoResponse, Notification,
};
pub use request::{Request, ResponseTemplate, SubscriptionObjects};
pub use status::{
    BedMeshStatus, ExcludeObjectChange, ExcludeObjectDefinition, ExcludeObjectSnapshot,
    ExcludeObjectStatus, FanStatus, GcodeMoveStatus, HeaterStatus, IdleTimeoutStatus, McuLastStats,
    McuStatus, ProbeStatus, SkewCorrectionStatus, Status, StatusUpdate, ToolheadStatus,
    VirtualSdcardStatus, WebhooksStatus, ZThermalAdjustStatus,
};
