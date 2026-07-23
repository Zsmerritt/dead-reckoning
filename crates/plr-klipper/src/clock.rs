//! Clock correlation between Klipper time domains.
//!
//! Three time axes matter to the recorder:
//!
//! * **`print_time`** — primary-MCU seconds; the time axis of trapq moves
//!   and (after conversion) stepper steps.
//! * **host `eventtime`** — Klipper's monotonic reactor clock; every
//!   status update carries one.
//! * **MCU clock ticks** — `first_clock`/`last_clock` in stepper dumps;
//!   related to `print_time` by the MCU frequency
//!   (`klippy/clocksync.py`, `ClockSync.clock_to_print_time`:
//!   `clock / mcu_freq`).
//!
//! [`ClockCorrelator`] links the first two using the
//! (`eventtime`, `toolhead.estimated_print_time`) pairs found in status
//! updates; [`McuClock`] converts the third; [`ReceiveSeqWidener`]
//! reconstructs the 64-bit `receive_seq` counter from its 32-bit-truncated
//! representation in `mcu.last_stats`.

use crate::error::ClockError;

/// Correlates `print_time` with host `eventtime`.
///
/// # Model
///
/// Latest-sample linear with unit slope: from the newest accepted sample
/// `(e, p)`, `eventtime(pt) = e + (pt - p)` and
/// `print_time(et) = p + (et - e)`. The relative frequency error between
/// the MCU clock (which defines `print_time`) and the host monotonic
/// clock is crystal drift, typically below 50 ppm, so with subscription
/// samples arriving every 0.25–1 s the extrapolation error from assuming
/// unit slope is under ~50 µs — far below the ~1 ms noise floor of
/// `estimated_print_time` itself (clocksync regression updated from
/// ~1 Hz `get_clock` round trips, `klippy/clocksync.py`). Expected
/// overall error: single-digit milliseconds worst case, dominated by the
/// upstream estimate, not by this model.
///
/// # Pathological inputs (all defined, none panic)
///
/// * NaN/±∞ in a sample → sample rejected
///   ([`SampleOutcome::RejectedNonFinite`]).
/// * `eventtime` earlier than the newest accepted sample → rejected
///   ([`SampleOutcome::RejectedStaleEventtime`]); the host reactor clock
///   is monotonic, so such a sample is reordered or corrupt. Equal
///   `eventtime` replaces the sample (fresher estimate for the same
///   instant).
/// * `estimated_print_time` jumping backwards or forwards by any amount
///   with a valid `eventtime` → accepted: Klipper restarts legitimately
///   reset `print_time`, and the correlator must follow the newest
///   estimate. Callers that need jump detection can compare
///   [`ClockCorrelator::latest_sample`] before/after.
/// * Conversion of a non-finite value, or one whose result overflows to
///   non-finite → `None`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ClockCorrelator {
    /// Newest accepted (`eventtime`, `estimated_print_time`) sample.
    sample: Option<(f64, f64)>,
}

/// Result of offering a sample to [`ClockCorrelator::add_sample`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleOutcome {
    /// The sample was accepted and is now the correlation anchor.
    Accepted,
    /// Rejected: at least one component was NaN or infinite.
    RejectedNonFinite,
    /// Rejected: `eventtime` was older than the current anchor's.
    RejectedStaleEventtime,
}

impl ClockCorrelator {
    /// Creates an empty correlator; conversions return `None` until a
    /// sample is accepted.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Offers an (`eventtime`, `estimated_print_time`) pair, typically
    /// from a `toolhead` status update.
    pub fn add_sample(&mut self, eventtime: f64, estimated_print_time: f64) -> SampleOutcome {
        if !eventtime.is_finite() || !estimated_print_time.is_finite() {
            return SampleOutcome::RejectedNonFinite;
        }
        if let Some((last_eventtime, _)) = self.sample {
            if eventtime < last_eventtime {
                return SampleOutcome::RejectedStaleEventtime;
            }
        }
        self.sample = Some((eventtime, estimated_print_time));
        SampleOutcome::Accepted
    }

    /// The newest accepted (`eventtime`, `estimated_print_time`) pair.
    #[must_use]
    pub fn latest_sample(&self) -> Option<(f64, f64)> {
        self.sample
    }

    /// Maps a `print_time` to host `eventtime`. `None` when no sample has
    /// been accepted yet, or the input/result is non-finite.
    #[must_use]
    pub fn print_time_to_eventtime(&self, print_time: f64) -> Option<f64> {
        let (eventtime, estimated_print_time) = self.sample?;
        finite(eventtime + (print_time - estimated_print_time))
    }

    /// Maps a host `eventtime` to `print_time`. `None` when no sample has
    /// been accepted yet, or the input/result is non-finite.
    #[must_use]
    pub fn eventtime_to_print_time(&self, eventtime: f64) -> Option<f64> {
        let (sample_eventtime, estimated_print_time) = self.sample?;
        finite(estimated_print_time + (eventtime - sample_eventtime))
    }
}

/// `Some(v)` iff `v` is finite.
fn finite(v: f64) -> Option<f64> {
    v.is_finite().then_some(v)
}

/// Converts MCU clock ticks to and from `print_time` seconds for one MCU.
///
/// The conversion is `print_time = clock / freq`
/// (`klippy/clocksync.py`, `ClockSync.clock_to_print_time` /
/// `print_time_to_clock`), with `freq` the MCU's `CLOCK_FREQ` constant —
/// available from [`McuStatus::clock_freq`](crate::status::McuStatus).
///
/// # 64-bit clocks
///
/// The MCU reports 32-bit clocks on the wire, but Klipper widens them to
/// 64 bits **host-side** (`klippy/clocksync.py`,
/// `ClockSync.clock32_to_clock64` and `_handle_clock`), and
/// `dump_stepper`'s `first_clock`/`last_clock` come from that widened
/// domain (`klippy/extras/motion_report.py` uses `mcu_stepper` dump data
/// and `clock_to_print_time` directly). This type therefore relies on
/// tick inputs already being correct 64-bit values — no wrap recovery is
/// needed or attempted here. Conversion via `f64` is exact for ticks
/// below 2⁵³ (a 180 MHz MCU reaches 2⁵³ after ≈1.6 years of uptime);
/// beyond that the error is one part in 2⁵³.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct McuClock {
    freq: f64,
}

impl McuClock {
    /// Creates a converter for an MCU with the given `CLOCK_FREQ` in Hz.
    /// Rejects non-finite or non-positive frequencies.
    pub fn new(freq: f64) -> Result<Self, ClockError> {
        if !freq.is_finite() || freq <= 0.0 {
            return Err(ClockError::InvalidFrequency(freq));
        }
        Ok(Self { freq })
    }

    /// The MCU clock frequency in Hz.
    #[must_use]
    pub fn freq(&self) -> f64 {
        self.freq
    }

    /// Converts a 64-bit MCU clock tick value to `print_time` seconds.
    #[must_use]
    pub fn clock_to_print_time(&self, clock: u64) -> f64 {
        // Precision: exact below 2^53 ticks; documented on the type.
        #[allow(clippy::cast_precision_loss)]
        let ticks = clock as f64;
        ticks / self.freq
    }

    /// Converts `print_time` seconds to MCU clock ticks (rounded to
    /// nearest). `None` for non-finite inputs and inputs outside
    /// `0..=u64::MAX` ticks; Klipper never schedules negative clocks.
    #[must_use]
    pub fn print_time_to_clock(&self, print_time: f64) -> Option<u64> {
        let ticks = (print_time * self.freq).round();
        // `u64::MAX as f64` rounds up to exactly 2^64; accept [0, 2^64).
        #[allow(clippy::cast_precision_loss)]
        let limit = u64::MAX as f64;
        if !ticks.is_finite() || ticks < 0.0 || ticks >= limit {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Some(ticks as u64)
    }
}

/// Reconstructs a monotonic 64-bit `receive_seq` from the 32-bit-wrapped
/// values in `mcu.last_stats` (see
/// [`McuLastStats::receive_seq`](crate::status::McuLastStats)).
///
/// # Policy
///
/// * The first observation anchors the output at the raw value.
/// * A forward delta below 2³¹ (mod 2³²) advances the output by that
///   delta. With ~1 Hz stats refresh, a real printer advances a few
///   hundred blocks per second at most, so any plausible inter-sample
///   advance — even minutes of missed samples — stays far below 2³¹.
/// * A delta of 2³¹ or more is interpreted as a **regression** (MCU
///   restart resetting the counter, or a stale/reordered reading). The
///   output holds its value (never decreases) and re-anchors on the new
///   raw value, reported as [`SeqKind::Regressed`]. This trades
///   undercounting after an MCU restart for the guarantee that the
///   widened counter is non-decreasing, which is what WAL ordering needs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ReceiveSeqWidener {
    state: Option<WidenerState>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct WidenerState {
    last_raw: u32,
    widened: u64,
}

/// Result of one [`ReceiveSeqWidener::observe`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeqUpdate {
    /// The widened, non-decreasing counter value.
    pub widened: u64,
    /// How this observation related to the previous one.
    pub kind: SeqKind,
}

/// Classification of an observation relative to the previous one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeqKind {
    /// First observation; the widened value is the raw value.
    First,
    /// The counter advanced by `delta` (interpreting mod-2³² wraparound).
    Advanced {
        /// Forward movement in blocks.
        delta: u32,
    },
    /// The counter reported the same value.
    Unchanged,
    /// Apparent backward movement (see type-level policy); the widened
    /// value did not advance.
    Regressed {
        /// How far backwards the raw counter appeared to move (mod 2³²).
        apparent_backstep: u32,
    },
}

impl ReceiveSeqWidener {
    /// Creates an empty widener.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The current widened value, if any observation has been made.
    #[must_use]
    pub fn current(&self) -> Option<u64> {
        self.state.map(|s| s.widened)
    }

    /// Feeds one raw `receive_seq` reading. Values are masked to 32 bits
    /// first: today's Klipper only ever emits the low 32 bits
    /// (`klippy/chelper/serialqueue.c`, `serialqueue_get_stats` formats
    /// with `%u` after an `(int)` cast), and masking keeps the delta
    /// arithmetic correct even if a future Klipper emitted wider values.
    pub fn observe(&mut self, raw: u64) -> SeqUpdate {
        #[allow(clippy::cast_possible_truncation)]
        let raw = (raw & u64::from(u32::MAX)) as u32;
        let Some(state) = self.state.as_mut() else {
            let widened = u64::from(raw);
            self.state = Some(WidenerState {
                last_raw: raw,
                widened,
            });
            return SeqUpdate {
                widened,
                kind: SeqKind::First,
            };
        };
        let delta = raw.wrapping_sub(state.last_raw);
        let kind = if delta == 0 {
            SeqKind::Unchanged
        } else if delta < 1 << 31 {
            state.widened = state.widened.saturating_add(u64::from(delta));
            SeqKind::Advanced { delta }
        } else {
            SeqKind::Regressed {
                apparent_backstep: delta.wrapping_neg(),
            }
        };
        state.last_raw = raw;
        SeqUpdate {
            widened: state.widened,
            kind,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ClockCorrelator, McuClock, ReceiveSeqWidener, SampleOutcome, SeqKind, SeqUpdate};
    use crate::error::ClockError;

    // --- ClockCorrelator ---

    #[test]
    fn empty_correlator_returns_none() {
        let c = ClockCorrelator::new();
        assert_eq!(c.print_time_to_eventtime(1.0), None);
        assert_eq!(c.eventtime_to_print_time(1.0), None);
        assert_eq!(c.latest_sample(), None);
    }

    #[test]
    fn known_value_round_trip() {
        let mut c = ClockCorrelator::new();
        // eventtime 3052153.38, estimated_print_time 812.5.
        assert_eq!(c.add_sample(3_052_153.38, 812.5), SampleOutcome::Accepted);
        // A trapq move at print_time 813.0 happened 0.5 s after the sample.
        let et = c.print_time_to_eventtime(813.0).unwrap();
        assert!((et - 3_052_153.88).abs() < 1e-9);
        // Inverse recovers the print_time.
        let pt = c.eventtime_to_print_time(et).unwrap();
        assert!((pt - 813.0).abs() < 1e-9);
    }

    #[test]
    fn newer_sample_wins() {
        let mut c = ClockCorrelator::new();
        assert_eq!(c.add_sample(100.0, 10.0), SampleOutcome::Accepted);
        assert_eq!(c.add_sample(101.0, 11.5), SampleOutcome::Accepted);
        assert_eq!(c.latest_sample(), Some((101.0, 11.5)));
        let et = c.print_time_to_eventtime(11.5).unwrap();
        assert!((et - 101.0).abs() < 1e-12);
    }

    #[test]
    fn equal_eventtime_replaces_sample() {
        let mut c = ClockCorrelator::new();
        assert_eq!(c.add_sample(100.0, 10.0), SampleOutcome::Accepted);
        assert_eq!(c.add_sample(100.0, 10.25), SampleOutcome::Accepted);
        assert_eq!(c.latest_sample(), Some((100.0, 10.25)));
    }

    #[test]
    fn rejects_non_finite_samples() {
        let mut c = ClockCorrelator::new();
        for (e, p) in [
            (f64::NAN, 1.0),
            (1.0, f64::NAN),
            (f64::INFINITY, 1.0),
            (1.0, f64::NEG_INFINITY),
        ] {
            assert_eq!(c.add_sample(e, p), SampleOutcome::RejectedNonFinite);
        }
        assert_eq!(c.latest_sample(), None);
    }

    #[test]
    fn rejects_backwards_eventtime_keeping_anchor() {
        let mut c = ClockCorrelator::new();
        assert_eq!(c.add_sample(100.0, 10.0), SampleOutcome::Accepted);
        assert_eq!(
            c.add_sample(99.0, 11.0),
            SampleOutcome::RejectedStaleEventtime
        );
        assert_eq!(c.latest_sample(), Some((100.0, 10.0)));
    }

    #[test]
    fn accepts_print_time_jumps_in_both_directions() {
        // Klipper restart: estimated_print_time resets to near zero.
        let mut c = ClockCorrelator::new();
        assert_eq!(c.add_sample(100.0, 5_000.0), SampleOutcome::Accepted);
        assert_eq!(c.add_sample(101.0, 0.5), SampleOutcome::Accepted);
        assert_eq!(c.latest_sample(), Some((101.0, 0.5)));
        // Huge forward jump is also followed.
        assert_eq!(c.add_sample(102.0, 9.0e300), SampleOutcome::Accepted);
    }

    #[test]
    fn conversions_are_total_on_pathological_inputs() {
        let mut c = ClockCorrelator::new();
        assert_eq!(c.add_sample(100.0, 10.0), SampleOutcome::Accepted);
        assert_eq!(c.print_time_to_eventtime(f64::NAN), None);
        assert_eq!(c.eventtime_to_print_time(f64::INFINITY), None);
        // Result overflowing to infinity is reported as None, not inf.
        let mut c2 = ClockCorrelator::new();
        assert_eq!(c2.add_sample(1.0e308, -1.0e308), SampleOutcome::Accepted);
        assert_eq!(c2.print_time_to_eventtime(1.0e308), None);
    }

    // --- McuClock ---

    #[test]
    fn rejects_bad_frequencies() {
        for f in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(matches!(
                McuClock::new(f),
                Err(ClockError::InvalidFrequency(_))
            ));
        }
    }

    #[test]
    fn tick_conversion_known_values() {
        // 180 MHz, matching e.g. an STM32F446 main MCU.
        let clock = McuClock::new(180_000_000.0).unwrap();
        assert!((clock.freq() - 180.0e6).abs() < f64::EPSILON);
        assert!((clock.clock_to_print_time(180_000_000) - 1.0).abs() < 1e-12);
        assert!((clock.clock_to_print_time(90_000_000) - 0.5).abs() < 1e-12);
        assert_eq!(clock.print_time_to_clock(1.0), Some(180_000_000));
        // Rounding to nearest tick.
        assert_eq!(
            clock.print_time_to_clock(1.0 + 0.4 / 180.0e6),
            Some(180_000_000)
        );
    }

    #[test]
    fn tick_conversion_beyond_32_bits() {
        // 16 MHz AVR after ~4.6 days: clock exceeds 2^32.
        let clock = McuClock::new(16_000_000.0).unwrap();
        let ticks: u64 = 6_400_000_000_000; // 400,000 s
        assert!(ticks > u64::from(u32::MAX));
        assert!((clock.clock_to_print_time(ticks) - 400_000.0).abs() < 1e-6);
        assert_eq!(clock.print_time_to_clock(400_000.0), Some(ticks));
    }

    #[test]
    fn print_time_to_clock_rejects_out_of_domain() {
        let clock = McuClock::new(1_000_000.0).unwrap();
        assert_eq!(clock.print_time_to_clock(f64::NAN), None);
        assert_eq!(clock.print_time_to_clock(f64::INFINITY), None);
        assert_eq!(clock.print_time_to_clock(-0.001), None);
        assert_eq!(clock.print_time_to_clock(1.0e300), None);
        // -0.0 rounds to 0 and is accepted.
        assert_eq!(clock.print_time_to_clock(-0.0), Some(0));
    }

    // --- ReceiveSeqWidener ---

    #[test]
    fn first_observation_anchors() {
        let mut w = ReceiveSeqWidener::new();
        assert_eq!(w.current(), None);
        assert_eq!(
            w.observe(41),
            SeqUpdate {
                widened: 41,
                kind: SeqKind::First,
            }
        );
        assert_eq!(w.current(), Some(41));
    }

    #[test]
    fn advances_and_holds_on_unchanged() {
        let mut w = ReceiveSeqWidener::new();
        w.observe(100);
        assert_eq!(
            w.observe(150),
            SeqUpdate {
                widened: 150,
                kind: SeqKind::Advanced { delta: 50 },
            }
        );
        assert_eq!(
            w.observe(150),
            SeqUpdate {
                widened: 150,
                kind: SeqKind::Unchanged,
            }
        );
    }

    #[test]
    fn widens_across_a_32_bit_wrap() {
        let mut w = ReceiveSeqWidener::new();
        w.observe(u64::from(u32::MAX) - 5); // 4294967290
        let update = w.observe(10); // wrapped: +16
        assert_eq!(update.kind, SeqKind::Advanced { delta: 16 });
        assert_eq!(update.widened, u64::from(u32::MAX) - 5 + 16);
    }

    #[test]
    fn regression_holds_value_and_reanchors() {
        let mut w = ReceiveSeqWidener::new();
        w.observe(1_000_000);
        // MCU restart: counter resets to 1 (serialqueue.c sets
        // receive_seq = 1 on init).
        let update = w.observe(1);
        assert_eq!(
            update.kind,
            SeqKind::Regressed {
                apparent_backstep: 999_999,
            }
        );
        assert_eq!(update.widened, 1_000_000); // non-decreasing
                                               // Counting resumes from the new anchor.
        assert_eq!(
            w.observe(11),
            SeqUpdate {
                widened: 1_000_010,
                kind: SeqKind::Advanced { delta: 10 },
            }
        );
    }

    #[test]
    fn masks_input_to_32_bits() {
        let mut w = ReceiveSeqWidener::new();
        w.observe((1_u64 << 40) | 7); // masked to 7
        assert_eq!(w.current(), Some(7));
        assert_eq!(w.observe(9).kind, SeqKind::Advanced { delta: 2 });
    }

    #[test]
    fn exact_half_range_delta_is_a_regression() {
        let mut w = ReceiveSeqWidener::new();
        w.observe(0);
        let update = w.observe(1_u64 << 31);
        assert_eq!(
            update.kind,
            SeqKind::Regressed {
                apparent_backstep: 1 << 31,
            }
        );
        assert_eq!(update.widened, 0);
    }
}
