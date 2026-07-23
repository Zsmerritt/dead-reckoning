//! Error types for wire decoding, request encoding, and clock math.

use thiserror::Error;

/// Errors from interpreting a received frame or one of its payloads.
#[derive(Debug, Error)]
pub enum MessageError {
    /// The frame body is not valid JSON.
    ///
    /// Klipper itself logs and skips undecodable requests
    /// (`klippy/webhooks.py`, `ClientConnection.process_received`); a client
    /// receiving garbage should likewise skip the frame and continue.
    #[error("frame is not valid JSON: {0}")]
    Json(#[from] serde_json::Error),

    /// The frame parsed as JSON but its top level is not an object.
    ///
    /// Every message Klipper sends is a JSON object (`klippy/webhooks.py`,
    /// `WebRequest.finish` and `ClientConnection.send`).
    #[error("frame is not a JSON object")]
    NotAnObject,

    /// The message carries an `id` that is not a non-negative integer.
    ///
    /// Klipper echoes the client-supplied `id` verbatim; this crate's
    /// [`Request`](crate::request::Request) builder only ever sends `u64`
    /// ids, so anything else on a response is a protocol violation.
    #[error("message id is not a non-negative integer")]
    InvalidId,

    /// The message carries an `id` but neither a `result` nor an `error`
    /// key. Klipper always attaches exactly one of the two
    /// (`klippy/webhooks.py`, `WebRequest.finish`).
    #[error("message with id {id} has neither `result` nor `error`")]
    MissingResultOrError {
        /// The echoed request id.
        id: u64,
    },

    /// A payload did not match the shape the Klipper source emits.
    #[error("payload shape mismatch for {context}: {source}")]
    Payload {
        /// What was being parsed (endpoint or object name).
        context: String,
        /// The underlying serde error.
        #[source]
        source: serde_json::Error,
    },
}

/// Errors from serializing a request to wire bytes.
#[derive(Debug, Error)]
pub enum EncodeError {
    /// JSON serialization failed. Unreachable for the request shapes this
    /// crate builds (string keys, no fallible `Serialize` impls), but
    /// surfaced instead of unwrapped so library code cannot panic.
    #[error("failed to serialize request to JSON: {0}")]
    Json(#[from] serde_json::Error),
}

/// Errors from MCU clock conversions.
#[derive(Debug, Error)]
pub enum ClockError {
    /// The MCU frequency must be finite and strictly positive.
    ///
    /// The value comes from the MCU's `CLOCK_FREQ` constant
    /// (`klippy/clocksync.py`, `ClockSync.connect`), exposed to API clients
    /// via the `mcu` status object's `mcu_constants` field.
    #[error("invalid MCU frequency {0}: must be finite and > 0")]
    InvalidFrequency(f64),
}
