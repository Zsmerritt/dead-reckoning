//! Reconstruction engine: computes the **possible-stop-state set** of a
//! 3D printer after an unclean stop, from the durable WAL prefix
//! recovered by `plr-wal`, the clock math of `plr-klipper`, and the
//! g-code replay of `plr-gcode`.
//!
//! # The problem
//!
//! Klipper's motion dumps batch at ~0.5 s and step generation runs
//! 0.4–0.7 s ahead of execution, so after a power loss the durable WAL
//! can end *before* the machine actually stopped: the machine may have
//! executed motion the daemon never received. Reconstruction therefore
//! produces a **set** of possible stop states, not a point estimate:
//!
//! ```text
//! possible-stop set = { state(t) : t ∈ [t_a, wal_eval_end] }   (from the WAL)
//!                     ∪ forward-simulated extension            (from the file)
//! ```
//!
//! * `t_a` — the last fsync'd heartbeat: provably alive and executing
//!   ([`window`] documents the exact derivation, including why the
//!   heartbeat's own `print_time` is clamped by the
//!   `estimated_print_time` correlation).
//! * `t_b` — end of committed motion from Z-stepper `dump_stepper`
//!   history (raw MCU clocks via [`plr_klipper::McuClock`]), bracketed
//!   by the widened `receive_seq` counter **as a time bound only**.
//! * The extension simulates the g-code from the last WAL-recorded
//!   context (file offset + interpreter state) for a configurable
//!   horizon (default 2 s of simulated motion, plus catch-up when the
//!   context lags `t_b`).
//!
//! # The guarantee (and its safety asymmetry)
//!
//! **The true stop state is always contained in the set** — enforced in
//! miniature by this crate's fault-injection property test, which
//! synthesizes WALs with honest 0.5 s batch flushing, torn tails, and
//! random power-cut points, and asserts containment of the true Z, XY,
//! E, and file offset for every cut.
//!
//! The Z projection is **exact**: an enumerable candidate list
//! (`{z_layer, z_layer − hop}` plateaus plus short ramp intervals at
//! worst, each with provenance and knowledge flags), because Z sizes
//! the probe envelope downstream and an unexpected Z-touch crashes the
//! nozzle into the print. XY/E timing fidelity, by contrast, only
//! affects line-match granularity, so XY is a bounding region and E a
//! pair of intervals (Klipper-internal frame and file frame — see
//! [`stopset`] for the E-frame replay).
//!
//! Degradations (subscription gaps, missing file tail, unparseable
//! lines) never silently shrink the set; they surface as typed flags in
//! [`stopset::Degradation`] with per-line vs per-layer confidence.
//!
//! # Cancelled (excluded) objects
//!
//! Recovery also has to know what the operator **cancelled** — usually
//! because that object failed — so a resume does not print back into
//! the debris. [`exclude`] resolves that from the WAL's journaled
//! `exclude_object` state, falling back to the print file's
//! `EXCLUDE_OBJECT_DEFINE` block. Crucially it gates on **uncertainty,
//! not on which answer it saw**: [`ExclusionReport::is_conclusive`] is
//! true only when nothing the log records as lost postdates the newest
//! exclusion observation *and* that observation is fresh, so a stale or
//! gap-shadowed excluded set prompts just as a missing one does. Every
//! reason is a named [`UncertaintyCause`], and
//! [`ExclusionReport::confirmation`] hands back the full per-object
//! payload so the prompt is a per-object selection, never a yes/no. The
//! report also answers "which object contains this XY?" from the
//! outlines.
//!
//! # Crash classes
//!
//! [`window::CrashClass`] classifies the stop: clean shutdown (reported
//! distinctly, no recovery), klippy/MCU shutdown with power retained,
//! and host-death-or-power-loss (indistinguishable from the WAL alone;
//! handled identically and conservatively). The klippy.log cross-check
//! mentioned in the design doc is deliberately **not** an input:
//! classification depends only on durable local evidence.
//!
//! # Pipeline
//!
//! ```text
//! plr_wal::scan ──► timeline::ingest ──► window::compute_stop_window ──► stopset::compute_stop_set
//!        heartbeat file ┘                        receive_seq ┘                file tail ┘
//! ```
//!
//! or in one call: [`reconstruct`] with [`ReconstructInputs`].
//!
//! Pure logic: no I/O, no panics on any input (property-tested),
//! `thiserror` errors for missing prerequisites only.

pub mod config;
pub mod error;
pub mod exclude;
pub mod reconstruct;
pub mod stopset;
pub mod timeline;
pub mod window;

pub use config::ReconstructConfig;
pub use error::{ContextDefect, ReconstructError};
pub use exclude::{
    parse_object_definitions, point_in_polygon, resolve_exclusions, ExclusionConfirmation,
    ExclusionDiagnostic, ExclusionFreshness, ExclusionInputs, ExclusionProvenance, ExclusionReport,
    FileObjectScan, ObjectKnowledge, ObjectState, UncertaintyCause, EDGE_TOLERANCE_MM,
};
pub use reconstruct::{reconstruct, ReconstructInputs, Reconstruction, RecoveryReconstruction};
pub use stopset::{
    anchor_state_from_context, compute_stop_set, Confidence, Degradation, ExtensionSummary,
    FileTail, Interval, OffsetWindow, PossibleStopSet, Provenance, XyRegion, ZCandidate, ZKind,
};
pub use timeline::{ingest, IngestNote, WalTimeline};
pub use window::{
    compute_stop_window, CrashClass, ReceiveSeqObservation, ShutdownEvidence, StopWindow, TbSource,
    WindowAnomaly,
};

/// Shared record constructors for this crate's tests.
#[cfg(test)]
pub(crate) mod testutil {
    use plr_wal::{
        Context, GcodeState, Heartbeat, RecoveryScan, ScanEnd, ScannedRecord, StepChunk,
        StepperRange, TransformObservations, TrapqSegment, VirtualSdState, WalRecord,
    };

    use crate::timeline::{ingest, WalTimeline};

    /// A scan holding `records` at synthetic offsets, ending cleanly.
    pub(crate) fn scan_of(records: Vec<WalRecord>) -> RecoveryScan {
        let records: Vec<ScannedRecord> = records
            .into_iter()
            .enumerate()
            .map(|(i, record)| ScannedRecord {
                offset: 32 + (i as u64) * 64,
                record,
            })
            .collect();
        RecoveryScan {
            header: None,
            records,
            truncation_offset: 0,
            end: ScanEnd::CleanEof,
        }
    }

    /// [`ingest`] over a synthetic scan with no heartbeat file.
    pub(crate) fn ingest_records(records: Vec<WalRecord>) -> WalTimeline {
        ingest(&scan_of(records), None)
    }

    /// A heartbeat whose est-sample pair anchors print time at exactly
    /// `mono_ns` (1 s of print time per 1 s of mono time).
    pub(crate) fn heartbeat_at(mono_ns: u64, print_time: f64) -> Heartbeat {
        Heartbeat {
            sequence: 1,
            mono_ns,
            wall_ns: 0,
            print_time,
            est_sample_mono_ns: mono_ns,
            est_sample_print_time: print_time,
            wal_offset: 0,
        }
    }

    /// A generic X-direction trapq segment.
    pub(crate) fn trapq_segment(
        queue: &str,
        print_time: f64,
        duration: f64,
        mono_ns: u64,
    ) -> TrapqSegment {
        trapq_segment_xyz(
            queue,
            print_time,
            duration,
            [10.0, 20.0, 0.2],
            [1.0, 0.0, 0.0],
            25.0,
            mono_ns,
        )
    }

    /// A constant-velocity trapq segment with explicit geometry.
    pub(crate) fn trapq_segment_xyz(
        queue: &str,
        print_time: f64,
        duration: f64,
        start: [f64; 3],
        ratios: [f64; 3],
        velocity: f64,
        mono_ns: u64,
    ) -> TrapqSegment {
        TrapqSegment {
            mono_ns,
            queue: queue.to_owned(),
            print_time,
            duration,
            start_velocity: velocity,
            acceleration: 0.0,
            start_x: start[0],
            start_y: start[1],
            start_z: start[2],
            x_r: ratios[0],
            y_r: ratios[1],
            z_r: ratios[2],
        }
    }

    /// A stepper range whose clocks are consistent with `last_step_time`
    /// at the given MCU frequency.
    pub(crate) fn stepper_range_with_clock(
        stepper: &str,
        last_step_time: f64,
        freq: f64,
        mono_ns: u64,
    ) -> StepperRange {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let last_clock = (last_step_time * freq).round() as u64;
        let first_clock = last_clock.saturating_sub(1_000_000);
        #[allow(clippy::cast_precision_loss)]
        let (first_t, last_t) = (first_clock as f64 / freq, last_clock as f64 / freq);
        StepperRange {
            mono_ns,
            stepper: stepper.to_owned(),
            first_clock,
            last_clock,
            first_step_time: first_t,
            last_step_time: last_t,
            start_position: 0.2,
            start_mcu_position: 100,
            step_distance: 0.0025,
            steps: vec![StepChunk {
                interval: 5_000,
                count: 10,
                add: 0,
            }],
        }
    }

    /// A stepper range with the given reported step time and synthetic
    /// (arbitrary) clocks — for tests that run without an MCU frequency.
    pub(crate) fn stepper_range(stepper: &str, last_step_time: f64, mono_ns: u64) -> StepperRange {
        let mut range = stepper_range_with_clock(stepper, last_step_time, 1_000_000.0, mono_ns);
        range.last_step_time = last_step_time;
        range
    }

    /// A neutral, valid context snapshot at (50, 50, 0.2), internal E
    /// 100, factors 1.0, printing `/tmp/test.gcode` at `file_position`.
    pub(crate) fn context_at(mono_ns: u64, file_position: u64) -> Context {
        context_with_gcode(
            mono_ns,
            file_position,
            GcodeState {
                speed_factor: 1.0,
                speed: 3000.0,
                extrude_factor: 1.0,
                absolute_coordinates: true,
                absolute_extrude: true,
                homing_origin: vec![0.0; 4],
                position: vec![50.0, 50.0, 0.2, 100.0],
                gcode_position: vec![50.0, 50.0, 0.2, 100.0],
            },
        )
    }

    /// A context snapshot with explicit g-code state.
    pub(crate) fn context_with_gcode(
        mono_ns: u64,
        file_position: u64,
        gcode: GcodeState,
    ) -> Context {
        Context {
            mono_ns,
            virtual_sdcard: Some(VirtualSdState {
                file_path: "/tmp/test.gcode".to_owned(),
                file_position,
            }),
            gcode,
            transforms: TransformObservations {
                bed_mesh_active: false,
                bed_mesh_profile: None,
                z_thermal_adjust_enabled: None,
                z_thermal_adjust_offset: None,
                skew_active: false,
                skew_profile: None,
            },
            heaters: Vec::new(),
            fans: Vec::new(),
            exclude: None,
        }
    }
}
