//! Host clock access (Linux).
//!
//! Record timestamps (`mono_ns`) use `CLOCK_MONOTONIC` — deliberately
//! the same clock Klipper's reactor `eventtime` is read from
//! (`time.monotonic()` in `klippy/reactor.py`) — so daemon-captured
//! timestamps and Klipper-reported event times share one axis and the
//! heartbeat's (`est_sample_mono_ns`, `est_sample_print_time`) pair can
//! anchor the print-time correlation without cross-clock conversion.

use std::time::{SystemTime, UNIX_EPOCH};

use rustix::time::{clock_gettime, ClockId};

/// Current `CLOCK_MONOTONIC` reading in nanoseconds.
#[must_use]
pub fn now_mono_ns() -> u64 {
    let ts = clock_gettime(ClockId::Monotonic);
    // tv_sec is non-negative for a monotonic clock; tv_nsec < 1e9.
    #[allow(clippy::cast_sign_loss)]
    let (sec, nsec) = (ts.tv_sec as u64, ts.tv_nsec as u64);
    sec.saturating_mul(1_000_000_000).saturating_add(nsec)
}

/// Current wall-clock time in nanoseconds since the Unix epoch. For
/// post-mortem human correlation only; may be stepped by NTP.
#[must_use]
pub fn now_wall_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX))
}

#[cfg(test)]
mod tests {
    use super::{now_mono_ns, now_wall_ns};

    #[test]
    fn monotonic_clock_is_nonzero_and_monotone() {
        let a = now_mono_ns();
        let b = now_mono_ns();
        assert!(a > 0);
        assert!(b >= a);
    }

    #[test]
    fn wall_clock_is_after_2020() {
        // 2020-01-01 in ns since the epoch.
        assert!(now_wall_ns() > 1_577_836_800_000_000_000);
    }
}
