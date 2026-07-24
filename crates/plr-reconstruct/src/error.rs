//! Typed errors for the reconstruction engine.
//!
//! Errors are reserved for **missing or unusable prerequisites**: things
//! the caller must fix (supply a heartbeat, supply the file tail that
//! actually covers the anchor offset) or that make reconstruction
//! meaningless (no context record ever reached the WAL). Everything that
//! merely *degrades* the answer — subscription gaps, torn tails, missing
//! receive-seq observations — is reported inside the result types
//! ([`crate::timeline::IngestNote`], [`crate::window::WindowAnomaly`],
//! [`crate::stopset::Degradation`]) so recovery can proceed honestly.

use thiserror::Error;

/// Why a context snapshot could not seed the forward simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum ContextDefect {
    /// A float field is NaN or infinite. The WAL writer refuses such
    /// records, so this indicates a hand-built or corrupted scan.
    #[error("a float field is NaN or infinite")]
    NonFinite,
    /// A coordinate vector (`position`, `gcode_position`,
    /// `homing_origin`) carries fewer than the four X/Y/Z/E axes.
    #[error("a coordinate vector carries fewer than 4 axes")]
    TooFewAxes,
    /// `extrude_factor` is not a positive finite number; the E-frame
    /// conversion `gcode_e = (internal_e - base_e) / extrude_factor`
    /// would be meaningless.
    #[error("extrude factor is not a positive finite number")]
    BadExtrudeFactor,
    /// The M220 speed factor is not a positive finite number.
    #[error("speed factor is not a positive finite number")]
    BadSpeedFactor,
    /// The reconstructed internal feed rate is not a positive finite
    /// number (Klipper's `gcode_move.speed` is strictly positive).
    #[error("reconstructed internal speed is not a positive finite number")]
    NonPositiveSpeed,
}

/// Errors from stop-window computation and possible-stop-set assembly.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ReconstructError {
    /// Neither the heartbeat file nor any WAL heartbeat record supplied
    /// a finite sample. Without `t_a` there is no proof the machine was
    /// executing at any particular time, and no clock correlation.
    #[error(
        "no usable heartbeat: neither the heartbeat file nor any WAL \
         heartbeat record supplied a finite sample"
    )]
    NoHeartbeat,
    /// The recovered WAL contains no [`plr_wal::Context`] record, so
    /// there is no file offset or g-code interpreter state to anchor
    /// reconstruction to.
    #[error("no context record in the recovered WAL; nothing anchors the file offset")]
    NoContext,
    /// The anchor context snapshot exists but cannot seed the forward
    /// simulation.
    #[error("anchor context snapshot is unusable: {defect}")]
    MalformedContext {
        /// What is wrong with the snapshot.
        defect: ContextDefect,
    },
    /// The supplied file tail does not cover the anchor context's
    /// `file_position`, so the forward simulation cannot start at the
    /// right byte. (An empty coverage — `file_position` exactly at the
    /// tail end — is accepted and simulates zero lines.)
    #[error(
        "file tail (bytes {base_offset}..{tail_end}) does not cover the anchor \
         context's file position {file_position}"
    )]
    FileTailMismatch {
        /// File offset of the first supplied byte.
        base_offset: u64,
        /// File offset one past the last supplied byte.
        tail_end: u64,
        /// The anchor context's `virtual_sdcard` file position.
        file_position: u64,
    },
    /// A configuration value is out of domain (non-finite, non-positive
    /// where a positive value is required, or an empty stepper prefix).
    #[error("invalid configuration: {reason}")]
    InvalidConfig {
        /// Which constraint was violated.
        reason: &'static str,
    },
}
