//! Reconstruction configuration.

use plr_gcode::SimConfig;

use crate::error::ReconstructError;

/// Tunables for stop-window computation and possible-stop-set assembly.
///
/// Every default is grounded in Klipper behavior; the doc comment on each
/// field cites the justification. Validate with [`ReconstructConfig::validate`]
/// (the top-level [`crate::reconstruct`] does this for you).
#[derive(Debug, Clone, PartialEq)]
pub struct ReconstructConfig {
    /// Primary-MCU `CLOCK_FREQ` in Hz, used to convert the raw
    /// `last_clock` of stepper dumps to print time via
    /// [`plr_klipper::McuClock`] (`print_time = clock / freq`,
    /// `klippy/clocksync.py`). `None` falls back to the
    /// Klipper-converted `last_step_time` carried by the same record,
    /// with a [`crate::window::WindowAnomaly::NoMcuFrequency`] anomaly —
    /// the raw clock is stamped at transmit time and is the more
    /// trustworthy source, so supply the frequency when known.
    pub mcu_freq: Option<f64>,
    /// Steppers whose name starts with this prefix are treated as Z
    /// steppers for the committed-motion boundary `t_b`. Klipper names
    /// them `stepper_z`, `stepper_z1`, ... for cartesian/corexy
    /// machines.
    pub z_stepper_prefix: String,
    /// How far ahead of execution step generation runs, seconds. Klipper
    /// flushes steps 0.4–0.7 s ahead of the executed print time
    /// (0.1 s during homing); 0.7 is the worst case. Used to widen the
    /// receive-seq time bound: blocks acked at host time `m` cannot
    /// contain steps scheduled later than `print_time(m) + lead`.
    pub step_gen_lead: f64,
    /// Classification threshold, nanoseconds: when the newest heartbeat
    /// postdates the newest motion record by more than this, the daemon
    /// demonstrably outlived motion (klippy/MCU shutdown with power
    /// retained). Must comfortably exceed the 0.5 s dump batching delay
    /// plus one heartbeat period; default 2 s.
    pub quiet_tail_ns: u64,
    /// Upper bound, seconds, on how far g-code *processing* runs ahead of
    /// *execution*. Klipper's `virtual_sdcard` reads ahead until the
    /// lookahead queue is full (`buffer_time_high` = 2.0 s in
    /// `klippy/toolhead.py`) plus step generation lead (≤ 0.7 s);
    /// default 3.0 adds margin. Context snapshots record the processing
    /// frontier, so the *executed* file offset at any time `t` is at
    /// least the frontier recorded at `t - max_processing_lead`. This
    /// bounds the low end of the file-offset candidate window and the
    /// set of E-frame snapshots that can apply to in-window motion.
    pub max_processing_lead: f64,
    /// Forward-simulation horizon, seconds of simulated motion beyond
    /// the committed boundary. The unreceived tail after power loss is
    /// at most ~0.5 s of dump batching plus ~1 s of trapq planning
    /// ahead; 2 s covers it with margin. The effective horizon passed to
    /// the simulator is `extension_horizon + max(0, t_b - t_anchor)` so
    /// a stale context still simulates through the committed span.
    /// Because [`plr_gcode::simulate`]'s per-line time accounting is a
    /// documented *lower bound* on real durations, the horizon covers at
    /// least this much real machine time.
    pub extension_horizon: f64,
    /// Kinematic limits and line budget for the forward simulation. The
    /// `max_duration` field is overridden by the computed horizon; the
    /// `max_lines` budget also caps how many parsed lines are collected
    /// from the file tail.
    pub sim: SimConfig,
    /// Two Z candidates closer than this (mm) with identical
    /// kind/provenance/knowledge merge into one. Covers float noise
    /// between independently-computed copies of the same layer height;
    /// keep well below any physical layer height.
    pub z_merge_tolerance: f64,
}

impl Default for ReconstructConfig {
    fn default() -> Self {
        Self {
            mcu_freq: None,
            z_stepper_prefix: "stepper_z".to_owned(),
            step_gen_lead: 0.7,
            quiet_tail_ns: 2_000_000_000,
            max_processing_lead: 3.0,
            extension_horizon: 2.0,
            sim: SimConfig::default(),
            z_merge_tolerance: 1e-6,
        }
    }
}

impl ReconstructConfig {
    /// Checks every field is in domain. Called by [`crate::reconstruct`];
    /// direct users of the stage functions should call it themselves.
    pub fn validate(&self) -> Result<(), ReconstructError> {
        let err = |reason| Err(ReconstructError::InvalidConfig { reason });
        if let Some(freq) = self.mcu_freq {
            if !freq.is_finite() || freq <= 0.0 {
                return err("mcu_freq must be finite and > 0");
            }
        }
        if self.z_stepper_prefix.is_empty() {
            return err("z_stepper_prefix must be non-empty");
        }
        if !self.step_gen_lead.is_finite() || self.step_gen_lead < 0.0 {
            return err("step_gen_lead must be finite and >= 0");
        }
        if !self.max_processing_lead.is_finite() || self.max_processing_lead < 0.0 {
            return err("max_processing_lead must be finite and >= 0");
        }
        if !self.extension_horizon.is_finite() || self.extension_horizon <= 0.0 {
            return err("extension_horizon must be finite and > 0");
        }
        if !self.z_merge_tolerance.is_finite() || self.z_merge_tolerance < 0.0 {
            return err("z_merge_tolerance must be finite and >= 0");
        }
        if !self.sim.max_velocity.is_finite() || self.sim.max_velocity <= 0.0 {
            return err("sim.max_velocity must be finite and > 0");
        }
        if !self.sim.max_accel.is_finite() || self.sim.max_accel <= 0.0 {
            return err("sim.max_accel must be finite and > 0");
        }
        if !self.sim.square_corner_velocity.is_finite() || self.sim.square_corner_velocity < 0.0 {
            return err("sim.square_corner_velocity must be finite and >= 0");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::ReconstructConfig;
    use crate::error::ReconstructError;

    #[test]
    fn default_config_is_valid() {
        assert_eq!(ReconstructConfig::default().validate(), Ok(()));
    }

    #[test]
    fn each_field_is_domain_checked() {
        let base = ReconstructConfig::default;
        let cases: Vec<ReconstructConfig> = vec![
            ReconstructConfig {
                mcu_freq: Some(0.0),
                ..base()
            },
            ReconstructConfig {
                mcu_freq: Some(f64::NAN),
                ..base()
            },
            ReconstructConfig {
                z_stepper_prefix: String::new(),
                ..base()
            },
            ReconstructConfig {
                step_gen_lead: -0.1,
                ..base()
            },
            ReconstructConfig {
                step_gen_lead: f64::INFINITY,
                ..base()
            },
            ReconstructConfig {
                max_processing_lead: f64::NAN,
                ..base()
            },
            ReconstructConfig {
                extension_horizon: 0.0,
                ..base()
            },
            ReconstructConfig {
                z_merge_tolerance: -1.0,
                ..base()
            },
        ];
        for config in cases {
            assert!(
                matches!(
                    config.validate(),
                    Err(ReconstructError::InvalidConfig { .. })
                ),
                "accepted invalid config: {config:?}"
            );
        }

        let mut config = base();
        config.sim.max_velocity = -1.0;
        assert!(config.validate().is_err());
        let mut config = base();
        config.sim.max_accel = f64::NAN;
        assert!(config.validate().is_err());
        let mut config = base();
        config.sim.square_corner_velocity = -0.5;
        assert!(config.validate().is_err());
    }
}
