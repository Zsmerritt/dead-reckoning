//! Stop-window computation: the print-time interval `[t_a, t_b]` inside
//! which the machine provably kept executing, plus the crash-class
//! verdict.
//!
//! # `t_a` — provably alive and executing
//!
//! `t_a` comes from the newest finite heartbeat (heartbeat file or WAL
//! record, whichever is newer). A heartbeat proves the daemon observed
//! Klipper alive at host-monotonic time `mono_ns`; its
//! (`est_sample_mono_ns`, `est_sample_print_time`) pair anchors the
//! print-time ↔ host-monotonic correlation (unit-slope linear, the same
//! model as [`ClockCorrelator`]). `t_a` is
//! `min(heartbeat.print_time, estimated print time at heartbeat.mono_ns)`:
//! the heartbeat's own `print_time` field is "latest print time known
//! from motion data", which can run *ahead* of execution when trapq rows
//! are planned ahead — taking the minimum keeps `t_a` a sound lower
//! bound on the stop time (an inflated `t_a` would wrongly exclude
//! possible stop states; a deflated one merely widens the window).
//!
//! # `t_b` — end of committed motion (exact derivation chain)
//!
//! 1. Take every [`plr_wal::StepperRange`] whose stepper name starts
//!    with the configured Z prefix (`dump_stepper` history of the
//!    safety-critical axis; clock-stamped at transmit time).
//! 2. For each, convert the raw 64-bit `last_clock` to print time with
//!    [`McuClock`] (`clock / freq`) when the MCU frequency is known;
//!    otherwise fall back to the Klipper-converted `last_step_time`
//!    carried by the record ([`WindowAnomaly::NoMcuFrequency`]). When
//!    both are available they are cross-checked
//!    ([`WindowAnomaly::ClockStepTimeMismatch`]; the *larger* value is
//!    used — widening is the safe direction).
//! 3. `t_b` = the maximum over those ranges ([`TbSource::ZStepper`]).
//!    With no Z-stepper history, fall back to all steppers
//!    ([`TbSource::AnyStepper`], [`WindowAnomaly::NoZStepperHistory`]).
//! 4. The `receive_seq` observation — a bare acked-block counter,
//!    already widened by the caller with
//!    [`plr_klipper::ReceiveSeqWidener`] — is applied **as a time bound
//!    only**: blocks acked at host time `m` cannot contain steps
//!    scheduled beyond `print_time(m) + step_gen_lead`. The bound can
//!    only *widen* the window (`t_b = max(t_b, bound)`); it never
//!    narrows it, because the WAL-committed evidence is already a lower
//!    bound on committed motion.
//! 5. With no stepper history at all, `t_b` degenerates to the
//!    receive-seq bound, or to `t_a`
//!    ([`WindowAnomaly::EmptyStepperHistory`]); the forward-simulated
//!    extension still covers the unobserved tail.
//! 6. If the result precedes `t_a` (stale stepper data), it is clamped
//!    up to `t_a` and reported ([`WindowAnomaly::TbBeforeTa`]): states
//!    before `t_a` are excluded by the liveness proof, so a window
//!    narrower than a point cannot exist.
//!
//! # Crash classification
//!
//! See [`CrashClass`] for the classes and the honesty limits of each
//! discriminator. Classification **never** narrows the possible-stop
//! set; it exists for reporting and for downstream policy (e.g. "was
//! the bed energized while unattended").

use plr_klipper::{ClockCorrelator, McuClock};
use plr_wal::{MarkerKind, ScanEnd, StepperRange};

use crate::config::ReconstructConfig;
use crate::error::ReconstructError;
use crate::timeline::WalTimeline;

/// A widened `receive_seq` observation: the newest acked-block counter
/// reading the daemon durably knows about, with the host-monotonic time
/// it was taken. Produced upstream from `mcu.last_stats` via
/// [`plr_klipper::ReceiveSeqWidener`] (which handles the 32-bit wrap);
/// this crate uses only its **time**, as a bound on how far committed
/// motion can extend past the last stepper dump.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReceiveSeqObservation {
    /// Host-monotonic time (ns) of the observation.
    pub mono_ns: u64,
    /// The widened (64-bit, non-decreasing) counter value, carried for
    /// provenance/reporting only.
    pub widened_seq: u64,
}

/// Where the committed-motion boundary `t_b` came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TbSource {
    /// The newest committed Z-stepper dump range (the designed source).
    ZStepper,
    /// No Z-stepper history; the newest committed range of *any* stepper.
    AnyStepper,
    /// The receive-seq time bound exceeded (or replaced) the stepper
    /// evidence.
    ReceiveSeq,
    /// No stepper history and no receive-seq observation: `t_b` collapsed
    /// to `t_a` and the extension carries the whole burden.
    HeartbeatOnly,
}

/// Evidence for a [`CrashClass::ShutdownPowerRetained`] verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShutdownEvidence {
    /// The log tail holds a `SocketLost` marker with no later
    /// `Resubscribed` and no later motion: the daemon outlived Klipper's
    /// API socket (klippy restart/shutdown).
    SocketLostMarker {
        /// Host-monotonic time (ns) of the drop.
        mono_ns: u64,
    },
    /// The newest heartbeat postdates the newest motion record by more
    /// than the configured quiet-tail threshold: the daemon demonstrably
    /// outlived motion.
    ///
    /// Honesty limit: a power cut during a *dwell* longer than the
    /// threshold also produces this signature. The verdict is then
    /// wrong about the cause but right about the position (motion had
    /// stopped at a WAL-known position either way), and the stop-set
    /// computation runs the forward extension regardless, so
    /// containment is unaffected.
    QuietTail {
        /// Observed heartbeat-after-motion gap, ns.
        quiet_ns: u64,
    },
}

/// The crash-class verdict, per the design doc's four classes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrashClass {
    /// A `CleanShutdown` marker ends the WAL: the print ended or was
    /// cancelled on purpose. **No recovery is needed**;
    /// [`crate::reconstruct`] reports this outcome distinctly and never
    /// builds a stop set for it.
    CleanShutdown,
    /// Klippy and/or the MCU shut down while host power stayed up:
    /// motion halted promptly at a WAL-known position and the daemon
    /// observed the aftermath. (A klippy.log cross-check can corroborate
    /// this class but is **never** a dependency.)
    ShutdownPowerRetained {
        /// What proved the daemon outlived motion.
        evidence: ShutdownEvidence,
    },
    /// The log simply ends: host death (MCU alive) or full power loss.
    /// The two are **indistinguishable from the WAL alone** and are
    /// handled identically — the possible-stop set covers both:
    ///
    /// * Host death, MCU alive: the MCU executes every received step,
    ///   then heater PWM `max_duration` (~3 s) fires an MCU shutdown and
    ///   the steppers are released; any bed sag happens *after* motion
    ///   stopped at a position inside the committed window. Edge case,
    ///   documented deliberately: if all heaters were cold/off at the
    ///   crash, no PWM watchdog ever fires, the MCU idles with drivers
    ///   energized and the bed (if separately powered) **stays
    ///   energized** — downstream recovery policy must not assume the
    ///   machine de-powered itself.
    /// * Power loss: the machine may have executed motion the daemon
    ///   never received (both dump endpoints batch at ~0.5 s), so
    ///   `t_stop ∈ [t_a, t_b ∪ extension]` and the forward-simulated
    ///   extension is load-bearing.
    HostDeathOrPowerLoss {
        /// `true` when the scan ended in a torn frame (power loss
        /// mid-append); `false` when it ended cleanly between frames.
        torn_tail: bool,
    },
}

/// Anything unusual observed while computing the window. Anomalies never
/// abort the computation; each documents the defined fallback taken.
#[derive(Debug, Clone, PartialEq)]
pub enum WindowAnomaly {
    /// The stepper-derived `t_b` preceded `t_a` (stale dump data);
    /// clamped up to `t_a`.
    TbBeforeTa {
        /// The pre-clamp value.
        raw_t_b: f64,
    },
    /// No stepper dump ranges at all; `t_b` fell back per
    /// [`TbSource`].
    EmptyStepperHistory,
    /// No stepper matching the Z prefix; all steppers were used instead.
    NoZStepperHistory,
    /// No MCU frequency configured; `last_step_time` (converted by
    /// Klipper at capture time) was trusted instead of the raw clock.
    NoMcuFrequency,
    /// Clock-derived and Klipper-reported step times disagree by more
    /// than 5 ms for this range; the larger was used.
    ClockStepTimeMismatch {
        /// The stepper whose range disagreed.
        stepper: String,
        /// `last_clock / freq`.
        clock_derived: f64,
        /// The record's `last_step_time`.
        reported: f64,
    },
    /// The heartbeat's `print_time` ran ahead of the estimated print
    /// time at the heartbeat instant (planned-ahead motion data); `t_a`
    /// used the estimate.
    HeartbeatAheadOfEstimate {
        /// The heartbeat's motion-derived print time.
        print_time: f64,
        /// The correlation-derived estimate at the heartbeat instant.
        estimated_now: f64,
    },
    /// No receive-seq observation was supplied; the upper bound relies
    /// on stepper dumps plus the extension alone.
    NoReceiveSeqBound,
    /// The scan ended in a way power loss does not produce; the window
    /// is computed but everything downstream deserves suspicion.
    CorruptScanEnd,
    /// A stepper range had a non-finite reported time and was skipped.
    NonFiniteStepperTime {
        /// The affected stepper.
        stepper: String,
    },
}

/// The computed stop window and crash class.
#[derive(Debug, Clone, PartialEq)]
pub struct StopWindow {
    /// Lower bound: the machine was provably alive and executing at
    /// `t_a` (print time, seconds), so the true stop is at or after it.
    pub t_a: f64,
    /// Upper bound of *committed-motion evidence* (print time, seconds),
    /// `>= t_a`. The true stop can exceed `t_b` by the unreceived tail;
    /// the stop set's forward extension covers that — see
    /// [`crate::stopset`].
    pub t_b: f64,
    /// How `t_b` was derived.
    pub t_b_source: TbSource,
    /// The crash-class verdict.
    pub class: CrashClass,
    /// Print-time ↔ host-monotonic correlation anchored at the
    /// heartbeat's `estimated_print_time` sample. "Eventtime" here is
    /// host-monotonic seconds (`mono_ns / 1e9`).
    pub correlation: ClockCorrelator,
    /// Everything unusual, with the fallback taken for each.
    pub anomalies: Vec<WindowAnomaly>,
}

impl StopWindow {
    /// Maps a host-monotonic timestamp (ns) to print time using the
    /// heartbeat-anchored correlation. `None` only for non-finite
    /// inputs/results.
    #[must_use]
    pub fn mono_ns_to_print_time(&self, mono_ns: u64) -> Option<f64> {
        self.correlation.eventtime_to_print_time(ns_to_s(mono_ns))
    }

    /// Maps a print time to host-monotonic seconds. `None` only for
    /// non-finite inputs/results.
    #[must_use]
    pub fn print_time_to_mono_s(&self, print_time: f64) -> Option<f64> {
        self.correlation.print_time_to_eventtime(print_time)
    }
}

/// Nanoseconds to seconds. The precision loss on the `u64 -> f64` cast
/// is below one microsecond for any uptime under ~285 years, far under
/// the millisecond-scale accuracy of the correlation itself.
#[allow(clippy::cast_precision_loss)]
pub(crate) fn ns_to_s(ns: u64) -> f64 {
    ns as f64 / 1e9
}

/// Computes the stop window and crash class from an ingested timeline.
///
/// Errors only when no usable heartbeat exists
/// ([`ReconstructError::NoHeartbeat`]) — without `t_a` there is neither
/// a liveness proof nor a clock correlation. All other defects degrade
/// into [`WindowAnomaly`]s with defined fallbacks.
pub fn compute_stop_window(
    timeline: &WalTimeline,
    receive_seq: Option<&ReceiveSeqObservation>,
    config: &ReconstructConfig,
) -> Result<StopWindow, ReconstructError> {
    let mut anomalies = Vec::new();
    let heartbeat = timeline.heartbeat.ok_or(ReconstructError::NoHeartbeat)?;

    // Correlation anchor: the heartbeat's estimated_print_time sample.
    // Finite by construction (ingest filters non-finite heartbeats), so
    // the sample is always accepted.
    let mut correlation = ClockCorrelator::new();
    correlation.add_sample(
        ns_to_s(heartbeat.est_sample_mono_ns),
        heartbeat.est_sample_print_time,
    );

    // t_a: min(motion-known print time, estimated now). See module docs.
    let est_now = correlation.eventtime_to_print_time(ns_to_s(heartbeat.mono_ns));
    let t_a = match est_now {
        Some(estimate) if estimate < heartbeat.print_time => {
            anomalies.push(WindowAnomaly::HeartbeatAheadOfEstimate {
                print_time: heartbeat.print_time,
                estimated_now: estimate,
            });
            estimate
        }
        _ => heartbeat.print_time,
    };

    // t_b steps 1-3: committed stepper evidence.
    let mcu_clock = config.mcu_freq.and_then(|freq| McuClock::new(freq).ok());
    if mcu_clock.is_none() {
        anomalies.push(WindowAnomaly::NoMcuFrequency);
    }
    let z_committed = committed_end(
        timeline
            .stepper_ranges
            .iter()
            .filter(|r| r.stepper.starts_with(&config.z_stepper_prefix)),
        mcu_clock.as_ref(),
        &mut anomalies,
    );
    let committed = if z_committed.is_some() {
        z_committed
    } else {
        let any = committed_end(
            timeline.stepper_ranges.iter(),
            mcu_clock.as_ref(),
            &mut anomalies,
        );
        if any.is_some() {
            anomalies.push(WindowAnomaly::NoZStepperHistory);
        }
        any
    };
    let committed_source = if z_committed.is_some() {
        TbSource::ZStepper
    } else {
        TbSource::AnyStepper
    };
    if committed.is_none() {
        anomalies.push(WindowAnomaly::EmptyStepperHistory);
    }

    // Step 4: the receive-seq time bound — widening only.
    let seq_bound = receive_seq
        .and_then(|obs| correlation.eventtime_to_print_time(ns_to_s(obs.mono_ns)))
        .map(|pt| pt + config.step_gen_lead)
        .filter(|bound| bound.is_finite());
    if receive_seq.is_none() {
        anomalies.push(WindowAnomaly::NoReceiveSeqBound);
    }

    let (raw_t_b, t_b_source) = match (committed, seq_bound) {
        (Some(c), Some(b)) if b > c => (b, TbSource::ReceiveSeq),
        (Some(c), _) => (c, committed_source),
        (None, Some(b)) => (b, TbSource::ReceiveSeq),
        (None, None) => (t_a, TbSource::HeartbeatOnly),
    };

    // Step 6: clamp to >= t_a.
    let t_b = if raw_t_b < t_a {
        anomalies.push(WindowAnomaly::TbBeforeTa { raw_t_b });
        t_a
    } else {
        raw_t_b
    };

    if !timeline.scan_end.is_expected_after_power_loss() {
        anomalies.push(WindowAnomaly::CorruptScanEnd);
    }
    let class = classify(timeline, heartbeat.mono_ns, config);

    Ok(StopWindow {
        t_a,
        t_b,
        t_b_source,
        class,
        correlation,
        anomalies,
    })
}

/// Newest committed print time over `ranges`, per the derivation chain
/// in the module docs. `None` when the iterator yields nothing usable.
fn committed_end<'a, I>(
    ranges: I,
    mcu_clock: Option<&McuClock>,
    anomalies: &mut Vec<WindowAnomaly>,
) -> Option<f64>
where
    I: Iterator<Item = &'a StepperRange>,
{
    /// Cross-check tolerance between `last_clock / freq` and the
    /// record's own `last_step_time`; generous against float noise and
    /// clocksync jitter, tight against real disagreement.
    const MISMATCH_TOLERANCE: f64 = 0.005;
    let mut newest: Option<f64> = None;
    for range in ranges {
        let reported = range.last_step_time;
        let time = match mcu_clock {
            Some(clock) => {
                let derived = clock.clock_to_print_time(range.last_clock);
                if reported.is_finite() && (derived - reported).abs() > MISMATCH_TOLERANCE {
                    anomalies.push(WindowAnomaly::ClockStepTimeMismatch {
                        stepper: range.stepper.clone(),
                        clock_derived: derived,
                        reported,
                    });
                    // Widening is the safe direction.
                    derived.max(reported)
                } else {
                    derived
                }
            }
            None => reported,
        };
        if !time.is_finite() {
            anomalies.push(WindowAnomaly::NonFiniteStepperTime {
                stepper: range.stepper.clone(),
            });
            continue;
        }
        newest = Some(newest.map_or(time, |n: f64| n.max(time)));
    }
    newest
}

/// The classification decision tree; precedence documented on
/// [`CrashClass`].
fn classify(
    timeline: &WalTimeline,
    heartbeat_mono_ns: u64,
    config: &ReconstructConfig,
) -> CrashClass {
    if timeline.clean_shutdown {
        return CrashClass::CleanShutdown;
    }
    // A tail power-fail GPIO edge is the cause of death itself: decisive
    // power-loss evidence. It is checked FIRST among the unclean classes —
    // ahead of the socket-loss marker — because asserting
    // `ShutdownPowerRetained` (power was RETAINED) against a hold-up edge
    // that says power was FAILING is a direct contradiction. That race is
    // reachable: klippy's API socket can drop in the same millisecond band
    // as the watcher's own edge (both happen as the rail browns out). When
    // both are present, the power-fail edge wins. The resulting class is the
    // ordinary `HostDeathOrPowerLoss`, which runs the full forward
    // extension, so this only ever corrects a classification — it never
    // narrows the stop set (and it overrides the quiet-tail *inference*
    // below, which would otherwise misread a power cut during a long dwell
    // as a power-retained shutdown). `CrashClass` is deliberately left
    // unchanged (it is matched exhaustively outside this crate); the exact-T
    // fact reaches the *window arithmetic* via the frontier cap
    // (`crate::stopset`), not a new class.
    if timeline.power_failing_tail().is_some() {
        return CrashClass::HostDeathOrPowerLoss {
            torn_tail: matches!(
                timeline.scan_end,
                ScanEnd::TruncatedFrameHeader | ScanEnd::TruncatedPayload
            ),
        };
    }
    if let Some(mono_ns) = timeline.socket_lost_tail {
        return CrashClass::ShutdownPowerRetained {
            evidence: ShutdownEvidence::SocketLostMarker { mono_ns },
        };
    }
    if let Some(last_motion) = timeline.last_motion_mono_ns {
        let quiet_ns = heartbeat_mono_ns.saturating_sub(last_motion);
        if quiet_ns > config.quiet_tail_ns {
            return CrashClass::ShutdownPowerRetained {
                evidence: ShutdownEvidence::QuietTail { quiet_ns },
            };
        }
    }
    CrashClass::HostDeathOrPowerLoss {
        torn_tail: matches!(
            timeline.scan_end,
            ScanEnd::TruncatedFrameHeader | ScanEnd::TruncatedPayload
        ),
    }
}

/// `true` when the marker kind indicates unobserved motion relevant to
/// degradation flags (used by the stop set).
pub(crate) fn is_observation_gap(kind: &MarkerKind) -> bool {
    matches!(
        kind,
        MarkerKind::SocketLost | MarkerKind::SubscriptionGap { .. }
    )
}

#[cfg(test)]
mod tests {
    use plr_wal::{Marker, MarkerKind, ScanEnd, WalRecord};

    use super::{
        compute_stop_window, CrashClass, ReceiveSeqObservation, ShutdownEvidence, TbSource,
        WindowAnomaly,
    };
    use crate::config::ReconstructConfig;
    use crate::error::ReconstructError;
    use crate::testutil::{
        heartbeat_at, ingest_records, scan_of, stepper_range, stepper_range_with_clock,
        trapq_segment,
    };
    use crate::timeline::ingest;

    const FREQ: f64 = 180_000_000.0;

    fn cfg() -> ReconstructConfig {
        ReconstructConfig {
            mcu_freq: Some(FREQ),
            ..ReconstructConfig::default()
        }
    }

    #[test]
    fn no_heartbeat_is_a_typed_error() {
        let timeline = ingest_records(vec![]);
        assert_eq!(
            compute_stop_window(&timeline, None, &cfg()),
            Err(ReconstructError::NoHeartbeat)
        );
    }

    #[test]
    fn t_b_comes_from_newest_z_stepper_clock() {
        // Heartbeat at print time 10.0; Z commits out to 11.5 and 12.25.
        let timeline = ingest_records(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::StepperRange(stepper_range_with_clock("stepper_z", 11.5, FREQ, 1)),
            WalRecord::StepperRange(stepper_range_with_clock("stepper_z1", 12.25, FREQ, 2)),
            WalRecord::StepperRange(stepper_range_with_clock("stepper_x", 14.0, FREQ, 3)),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(window.t_b_source, TbSource::ZStepper);
        assert!((window.t_a - 10.0).abs() < 1e-9);
        // Only Z steppers feed t_b; stepper_x's 14.0 is ignored.
        assert!((window.t_b - 12.25).abs() < 1e-6, "t_b = {}", window.t_b);
        assert!(!window
            .anomalies
            .iter()
            .any(|a| matches!(a, WindowAnomaly::ClockStepTimeMismatch { .. })));
    }

    #[test]
    fn missing_mcu_freq_falls_back_to_reported_step_time() {
        let timeline = ingest_records(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::StepperRange(stepper_range("stepper_z", 11.5, 1)),
        ]);
        let config = ReconstructConfig {
            mcu_freq: None,
            ..ReconstructConfig::default()
        };
        let window = compute_stop_window(&timeline, None, &config).unwrap();
        assert!((window.t_b - 11.5).abs() < 1e-12);
        assert!(window.anomalies.contains(&WindowAnomaly::NoMcuFrequency));
    }

    #[test]
    fn clock_step_time_mismatch_uses_the_larger_and_reports() {
        // Range claims last_step_time 11.5 but the clock says 13.0.
        let mut range = stepper_range_with_clock("stepper_z", 13.0, FREQ, 1);
        range.last_step_time = 11.5;
        let timeline = ingest_records(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::StepperRange(range),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert!((window.t_b - 13.0).abs() < 1e-6);
        assert!(window
            .anomalies
            .iter()
            .any(|a| matches!(a, WindowAnomaly::ClockStepTimeMismatch { .. })));
    }

    #[test]
    fn no_z_history_falls_back_to_any_stepper() {
        let timeline = ingest_records(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::StepperRange(stepper_range_with_clock("stepper_x", 11.0, FREQ, 1)),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(window.t_b_source, TbSource::AnyStepper);
        assert!((window.t_b - 11.0).abs() < 1e-6);
        assert!(window.anomalies.contains(&WindowAnomaly::NoZStepperHistory));
    }

    #[test]
    fn empty_stepper_history_collapses_to_t_a_or_receive_seq() {
        let records = vec![WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0))];
        let timeline = ingest_records(records);

        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(window.t_b_source, TbSource::HeartbeatOnly);
        assert!((window.t_b - window.t_a).abs() < 1e-12);
        assert!(window
            .anomalies
            .contains(&WindowAnomaly::EmptyStepperHistory));

        // With a receive-seq observation 0.5 s after the est sample:
        // bound = 10.5 + 0.7 lead.
        let obs = ReceiveSeqObservation {
            mono_ns: 1_500_000_000,
            widened_seq: 42,
        };
        let window = compute_stop_window(&timeline, Some(&obs), &cfg()).unwrap();
        assert_eq!(window.t_b_source, TbSource::ReceiveSeq);
        assert!((window.t_b - 11.2).abs() < 1e-9, "t_b = {}", window.t_b);
    }

    #[test]
    fn receive_seq_bound_only_ever_widens() {
        let base = vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::StepperRange(stepper_range_with_clock("stepper_z", 13.0, FREQ, 1)),
        ];
        let timeline = ingest_records(base);
        // Bound 10.2 + 0.7 = 10.9 < committed 13.0: ignored.
        let stale = ReceiveSeqObservation {
            mono_ns: 1_200_000_000,
            widened_seq: 7,
        };
        let window = compute_stop_window(&timeline, Some(&stale), &cfg()).unwrap();
        assert_eq!(window.t_b_source, TbSource::ZStepper);
        assert!((window.t_b - 13.0).abs() < 1e-6);
        // Bound 14.0 + 0.7 = 14.7 > 13.0: widens.
        let fresh = ReceiveSeqObservation {
            mono_ns: 5_000_000_000,
            widened_seq: 8,
        };
        let window = compute_stop_window(&timeline, Some(&fresh), &cfg()).unwrap();
        assert_eq!(window.t_b_source, TbSource::ReceiveSeq);
        assert!((window.t_b - 14.7).abs() < 1e-9);
    }

    #[test]
    fn stale_t_b_is_clamped_to_t_a() {
        let timeline = ingest_records(vec![
            WalRecord::Heartbeat(heartbeat_at(10_000_000_000, 20.0)),
            WalRecord::StepperRange(stepper_range_with_clock("stepper_z", 5.0, FREQ, 1)),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert!((window.t_b - window.t_a).abs() < 1e-12);
        assert!(window.anomalies.iter().any(
            |a| matches!(a, WindowAnomaly::TbBeforeTa { raw_t_b } if (raw_t_b - 5.0).abs() < 1e-6)
        ));
    }

    #[test]
    fn inflated_heartbeat_print_time_is_clamped_by_estimate() {
        // Heartbeat claims motion-known print time 15.0, but the est
        // sample says "now" is 10.0 at the same instant.
        let mut hb = heartbeat_at(1_000_000_000, 15.0);
        hb.est_sample_print_time = 10.0;
        hb.est_sample_mono_ns = 1_000_000_000;
        let timeline = ingest_records(vec![WalRecord::Heartbeat(hb)]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert!((window.t_a - 10.0).abs() < 1e-9);
        assert!(window
            .anomalies
            .iter()
            .any(|a| matches!(a, WindowAnomaly::HeartbeatAheadOfEstimate { .. })));
    }

    #[test]
    fn classifies_clean_shutdown() {
        let timeline = ingest_records(vec![
            WalRecord::Heartbeat(heartbeat_at(1_000_000_000, 10.0)),
            WalRecord::Marker(Marker {
                mono_ns: 2_000_000_000,
                kind: MarkerKind::CleanShutdown,
            }),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(window.class, CrashClass::CleanShutdown);
    }

    #[test]
    fn classifies_socket_lost_tail() {
        let timeline = ingest_records(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 9.0, 0.5, 1_000_000_000)),
            WalRecord::Heartbeat(heartbeat_at(1_100_000_000, 10.0)),
            WalRecord::Marker(Marker {
                mono_ns: 1_200_000_000,
                kind: MarkerKind::SocketLost,
            }),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(
            window.class,
            CrashClass::ShutdownPowerRetained {
                evidence: ShutdownEvidence::SocketLostMarker {
                    mono_ns: 1_200_000_000
                }
            }
        );
    }

    #[test]
    fn classifies_quiet_tail() {
        // Motion at mono 1e9, heartbeat at 5e9: 4 s of quiet > 2 s.
        let timeline = ingest_records(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 9.0, 0.5, 1_000_000_000)),
            WalRecord::Heartbeat(heartbeat_at(5_000_000_000, 10.0)),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(
            window.class,
            CrashClass::ShutdownPowerRetained {
                evidence: ShutdownEvidence::QuietTail {
                    quiet_ns: 4_000_000_000
                }
            }
        );
    }

    #[test]
    fn a_tail_power_fail_marker_overrides_the_quiet_tail_inference() {
        // The exact scenario `power_loss_during_long_dwell_classifies_quiet`
        // documents as an ambiguity — motion stopped 4 s before the newest
        // heartbeat — but now with a hold-up GPIO edge at the tail. The
        // quiet-tail INFERENCE would say "power retained"; the edge is the
        // cause of death itself and forces the honest HostDeathOrPowerLoss.
        let timeline = ingest_records(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 9.0, 0.5, 1_000_000_000)),
            WalRecord::Heartbeat(heartbeat_at(5_000_000_000, 10.0)),
            WalRecord::Marker(Marker {
                mono_ns: 5_100_000_000,
                kind: MarkerKind::PowerFailing,
            }),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(
            window.class,
            CrashClass::HostDeathOrPowerLoss { torn_tail: false },
            "a tail power-fail edge is decisive power-loss evidence"
        );
    }

    #[test]
    fn a_neutralized_power_fail_marker_does_not_change_classification() {
        // A false edge that liveness outlived past the margin (motion AND a
        // heartbeat 4 s after a 1 s edge) is neutralized, so classification
        // is exactly what it would be without the marker at all.
        let records = vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 9.0, 0.5, 5_000_000_000)),
            WalRecord::Heartbeat(heartbeat_at(5_000_000_000, 10.0)),
        ];
        let mut with_edge = records.clone();
        with_edge.insert(
            0,
            WalRecord::Marker(Marker {
                mono_ns: 1_000_000_000,
                kind: MarkerKind::PowerFailing,
            }),
        );
        let without = compute_stop_window(&ingest_records(records), None, &cfg())
            .unwrap()
            .class;
        let timeline = ingest_records(with_edge);
        assert_eq!(
            timeline.power_failing_tail(),
            None,
            "liveness 4 s past a 1 s edge must neutralize it"
        );
        let with_ = compute_stop_window(&timeline, None, &cfg()).unwrap().class;
        assert_eq!(
            with_, without,
            "a neutralized edge must not change the class"
        );
        assert_eq!(with_, CrashClass::HostDeathOrPowerLoss { torn_tail: false });
    }

    /// **MINOR precedence fix + race.** A tail `SocketLost` and a tail
    /// `PowerFailing` in the same millisecond band (klippy's socket dropping
    /// as the rail browns out): the power-fail edge WINS. Asserting
    /// `ShutdownPowerRetained` — power RETAINED — against a hold-up edge
    /// saying power was FAILING would be a contradiction.
    #[test]
    fn a_tail_power_fail_edge_beats_a_tail_socket_lost() {
        let timeline = ingest_records(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 9.0, 0.5, 1_000_000_000)),
            WalRecord::Heartbeat(heartbeat_at(1_100_000_000, 10.0)),
            // Socket drops, then the power-fail edge, both after motion.
            WalRecord::Marker(Marker {
                mono_ns: 1_200_000_000,
                kind: MarkerKind::SocketLost,
            }),
            WalRecord::Marker(Marker {
                mono_ns: 1_200_500_000,
                kind: MarkerKind::PowerFailing,
            }),
        ]);
        // Both tail facts are present...
        assert_eq!(timeline.socket_lost_tail, Some(1_200_000_000));
        assert_eq!(timeline.power_failing_tail(), Some(1_200_500_000));
        // ...and the power-fail edge decides the class.
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(
            window.class,
            CrashClass::HostDeathOrPowerLoss { torn_tail: false },
            "a power-fail tail must never yield a power-retained class"
        );
    }

    #[test]
    fn classifies_host_death_or_power_loss_with_tear_flag() {
        let records = vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 9.0, 0.5, 1_000_000_000)),
            WalRecord::Heartbeat(heartbeat_at(1_100_000_000, 10.0)),
        ];
        let mut scan = scan_of(records.clone());
        scan.end = ScanEnd::TruncatedPayload;
        let window = compute_stop_window(&ingest(&scan, None), None, &cfg()).unwrap();
        assert_eq!(
            window.class,
            CrashClass::HostDeathOrPowerLoss { torn_tail: true }
        );

        let clean = ingest_records(records);
        let window = compute_stop_window(&clean, None, &cfg()).unwrap();
        assert_eq!(
            window.class,
            CrashClass::HostDeathOrPowerLoss { torn_tail: false }
        );
    }

    #[test]
    fn power_loss_during_long_dwell_classifies_quiet_but_stays_safe() {
        // Documented ambiguity: dwell > quiet threshold then power cut.
        // The class is ShutdownPowerRetained; containment is preserved
        // by the stop set running the extension for every non-clean
        // class (asserted in stopset/fault-injection tests).
        let timeline = ingest_records(vec![
            WalRecord::TrapqSegment(trapq_segment("toolhead", 9.0, 0.5, 1_000_000_000)),
            WalRecord::Heartbeat(heartbeat_at(9_000_000_000, 9.5)),
        ]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert!(matches!(
            window.class,
            CrashClass::ShutdownPowerRetained {
                evidence: ShutdownEvidence::QuietTail { .. }
            }
        ));
    }

    #[test]
    fn corrupt_scan_end_is_flagged() {
        let mut scan = scan_of(vec![WalRecord::Heartbeat(heartbeat_at(1, 10.0))]);
        scan.end = ScanEnd::FrameCrcMismatch;
        let window = compute_stop_window(&ingest(&scan, None), None, &cfg()).unwrap();
        assert!(window.anomalies.contains(&WindowAnomaly::CorruptScanEnd));
        // CRC corruption of durable bytes is not a "torn tail".
        assert_eq!(
            window.class,
            CrashClass::HostDeathOrPowerLoss { torn_tail: false }
        );
    }

    #[test]
    fn non_finite_stepper_time_is_skipped_without_freq() {
        let mut range = stepper_range("stepper_z", 11.0, 1);
        range.last_step_time = f64::NAN;
        // NaN last_step_time makes the whole record non-finite, so it is
        // dropped at ingest; build the timeline by hand to exercise the
        // window-level guard too.
        let mut timeline = ingest_records(vec![WalRecord::Heartbeat(heartbeat_at(1, 10.0))]);
        timeline.stepper_ranges.push(range);
        let config = ReconstructConfig {
            mcu_freq: None,
            ..ReconstructConfig::default()
        };
        let window = compute_stop_window(&timeline, None, &config).unwrap();
        assert!(window
            .anomalies
            .iter()
            .any(|a| matches!(a, WindowAnomaly::NonFiniteStepperTime { .. })));
        assert_eq!(window.t_b_source, TbSource::HeartbeatOnly);
    }

    #[test]
    fn mono_print_time_round_trip() {
        let timeline = ingest_records(vec![WalRecord::Heartbeat(heartbeat_at(
            2_000_000_000,
            10.0,
        ))]);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let pt = window.mono_ns_to_print_time(3_000_000_000).unwrap();
        assert!((pt - 11.0).abs() < 1e-9);
        let mono_s = window.print_time_to_mono_s(pt).unwrap();
        assert!((mono_s - 3.0).abs() < 1e-9);
    }
}
