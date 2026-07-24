//! Recovery executor scaffold: turning a reconstruction into machine
//! action via Moonraker.
//!
//! # Status: scaffold
//!
//! The executor is deliberately not implemented yet. Executing a
//! recovery moves a hot nozzle around a solidified print, so it ships
//! only together with its own safety review; this module pins down the
//! *shape* the daemon and CLI already agree on, so the wiring does not
//! churn when the implementation lands.
//!
//! # Planned flow (documented for reviewers)
//!
//! 1. `plrd scan` (or the daemon after an unclean stop) produces a
//!    [`plr_reconstruct::RecoveryReconstruction`].
//! 2. A human (or a policy gate) approves a [`RecoveryPlan`] derived
//!    from it: probe envelope from the Z candidates, re-arm transforms,
//!    thermal targets, resume line window.
//! 3. [`MoonrakerExecutor`] connects to Moonraker's WebSocket JSON-RPC
//!    (`ws://.../websocket`, `printer.gcode.script` et al.) — the
//!    `tokio-tungstenite` workspace dependency is reserved for exactly
//!    this — and executes the plan step by step, aborting on any
//!    unexpected printer state.

use std::path::PathBuf;

/// One executable recovery step, in execution order.
#[derive(Debug, Clone, PartialEq)]
pub enum RecoveryStep {
    /// Re-heat to journaled targets and wait.
    RestoreThermals {
        /// `(heater name, target °C)` pairs from the last context.
        targets: Vec<(String, f64)>,
    },
    /// Home safe axes / re-establish a trusted Z reference within the
    /// reconstruction's probe envelope.
    ReestablishReference {
        /// Lowest Z the probe may approach, from the stop set.
        z_floor_mm: f64,
    },
    /// Re-arm move transforms (bed mesh profile, skew, gcode offsets).
    RearmTransforms {
        /// G-code commands to replay, in order.
        gcode: Vec<String>,
    },
    /// Resume the print from a file offset.
    ResumeAt {
        /// The file to print.
        file: PathBuf,
        /// Byte offset chosen from the reconstruction's offset window.
        offset: u64,
    },
}

/// A vetted, ordered plan. Construction from a reconstruction lands with
/// the executor implementation.
#[derive(Debug, Clone, PartialEq)]
pub struct RecoveryPlan {
    /// Steps in execution order.
    pub steps: Vec<RecoveryStep>,
}

/// Error surface of the (future) executor.
#[derive(Debug, thiserror::Error)]
pub enum ExecutorError {
    /// The executor is not implemented yet; see the module docs.
    #[error("recovery execution is not implemented yet (scaffold only)")]
    NotImplemented,
}

/// Client for Moonraker's WebSocket JSON-RPC API.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MoonrakerExecutor {
    /// Moonraker WebSocket URL, e.g. `ws://localhost:7125/websocket`.
    pub url: String,
}

impl MoonrakerExecutor {
    /// Creates an executor pointed at a Moonraker instance.
    #[must_use]
    pub fn new(url: String) -> Self {
        Self { url }
    }

    /// Executes a plan. Scaffold: always
    /// [`ExecutorError::NotImplemented`].
    // Scaffold: the implementation will use `self.url`; the signature is
    // pinned now so callers do not churn when it lands.
    #[allow(clippy::unused_self)]
    pub fn execute(&self, _plan: &RecoveryPlan) -> Result<(), ExecutorError> {
        Err(ExecutorError::NotImplemented)
    }
}

#[cfg(test)]
mod tests {
    use super::{ExecutorError, MoonrakerExecutor, RecoveryPlan, RecoveryStep};

    #[test]
    fn scaffold_refuses_to_execute() {
        let executor = MoonrakerExecutor::new("ws://localhost:7125/websocket".to_owned());
        let plan = RecoveryPlan {
            steps: vec![
                RecoveryStep::RestoreThermals {
                    targets: vec![("extruder".to_owned(), 215.0)],
                },
                RecoveryStep::ReestablishReference { z_floor_mm: 4.2 },
                RecoveryStep::RearmTransforms {
                    gcode: vec!["BED_MESH_PROFILE LOAD=default".to_owned()],
                },
                RecoveryStep::ResumeAt {
                    file: "/g/x.gcode".into(),
                    offset: 123_456,
                },
            ],
        };
        let err = executor.execute(&plan).unwrap_err();
        assert!(matches!(err, ExecutorError::NotImplemented));
        assert!(err.to_string().contains("not implemented"));
    }
}
