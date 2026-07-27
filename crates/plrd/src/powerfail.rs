//! The power-failing GPIO watcher: a dedicated Linux thread that turns a
//! hold-up-backed GPIO edge into a durable [`plr_wal::MarkerKind::PowerFailing`]
//! marker, inside the ~1 s the Pi survives after the DC rail begins failing.
//!
//! # The operator's contract (two tiers)
//!
//! The operator's hold-up hardware raises a GPIO edge when power begins
//! failing. The watcher's ratified response is two tiers:
//!
//! * **Mandatory (must complete in ~20 ms):** journal a `PowerFailing`
//!   marker carrying the edge's monotonic timestamp, then `fdatasync` the
//!   WAL segment and the heartbeat file. This is
//!   [`PowerFailResponse::mandatory`]. The edge timestamp is the exact-T
//!   that converts reconstruction from inference to arithmetic — the
//!   plan-at-T is already on disk (durable plan capture leads execution),
//!   so T was the only unknown.
//! * **Best-effort (only if power lingers; MUST NOT delay the mandatory
//!   tier):** a clean daemon exit so the filesystem is not mid-write when
//!   the rail dies. This is [`PowerFailResponse::best_effort`], and
//!   [`run_watcher`] calls it **strictly after** the mandatory tier has
//!   fully returned.
//!
//! **Never send a printer command on the edge** — the MCU and heaters die
//! with the 24 V rail on their own, and commanding a browning-out machine
//! is forbidden. Nothing in this module talks to the printer.
//!
//! # Why blocking, never dropping
//!
//! The socket reader must never block (Klipper disconnects slow clients),
//! so its WAL sends drop under backpressure. The power-fail marker is the
//! opposite case: it is the single most important record in the log and
//! there is exactly one, so [`WalChannelResponse::mandatory`] uses a
//! **blocking** channel send — it waits for space rather than dropping.
//! The watcher runs on its own thread, so blocking it has no external
//! consequence, and the WAL thread drains continuously, so the wait is the
//! time to free one slot (measured in the mandatory-tier latency test).
//!
//! # The abstracted edge source
//!
//! [`EdgeSource`] is the thin OS-facing seam: the real
//! [`GpioEdgeSource`](#impl) reads real edges from `/dev/gpiochipN` and is
//! **not** exercisable in CI (it needs hardware — stated honestly rather
//! than faked), while [`run_watcher`] and the whole response path are
//! driven by a synthetic edge in the tests, on every platform the code
//! compiles on.
//!
//! # Platform
//!
//! The core ([`EdgeSource`], [`PowerFailResponse`], [`run_watcher`],
//! [`WatcherOutcome`]) compiles and is tested on every platform. The
//! concrete `gpiocdev` source, the WAL-channel response, and [`spawn`] are
//! Linux-only (`#[cfg(target_os = "linux")]`), like the rest of the daemon.

use std::fmt;
use std::time::Duration;

/// A failure of the OS-facing edge source (the GPIO line closed, an ioctl
/// failed, the chip vanished). Terminates the watcher; the daemon logs it
/// and keeps recording — losing the power-fail *sensor* must never take
/// down the recorder.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgeError(String);

impl EdgeError {
    /// Wraps a human-readable reason.
    #[must_use]
    pub fn new(reason: impl Into<String>) -> Self {
        Self(reason.into())
    }
}

impl fmt::Display for EdgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for EdgeError {}

/// The thin OS-facing seam the watcher reads edges through. Real hardware
/// implements it with `gpiocdev` ([`GpioEdgeSource`](#impl)); tests
/// implement it with a synthetic edge so the whole response path is
/// exercisable without a GPIO chip.
pub trait EdgeSource {
    /// Blocks until the next edge (no polling), or fails. On the real
    /// source this is a blocking `read_edge_event`.
    fn wait_for_edge(&mut self) -> Result<(), EdgeError>;

    /// Re-reads the line **now** and reports whether the power-failing
    /// level is (still) asserted. Called once after the debounce window to
    /// reject a blip that has already relaxed.
    fn is_asserted(&mut self) -> Result<bool, EdgeError>;
}

/// The two-tier response to a confirmed power-fail edge. Split into two
/// methods precisely so [`run_watcher`] can guarantee the ordering the
/// contract requires: the mandatory tier runs and fully returns before the
/// best-effort tier is even called.
pub trait PowerFailResponse {
    /// **Mandatory tier.** Journal the `PowerFailing` marker at
    /// `edge_mono_ns` and make it (and the heartbeat file) durable. Must
    /// not depend on, or be delayed by, the best-effort tier.
    fn mandatory(&mut self, edge_mono_ns: u64);

    /// **Best-effort tier.** Runs only after [`mandatory`](Self::mandatory)
    /// has returned. Its failure or slowness cannot affect the mandatory
    /// tier, which is already complete.
    fn best_effort(&mut self);
}

/// Why [`run_watcher`] returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatcherOutcome {
    /// A confirmed edge fired the response; the watcher's job is done.
    Fired,
    /// The edge source ended (closed or errored) before any confirmed
    /// edge. The daemon logs this and keeps recording without power-fail
    /// protection.
    SourceEnded,
}

/// Drives the edge source and the two-tier response.
///
/// On each edge: capture the edge's monotonic timestamp (via `now_mono_ns`,
/// so the marker carries the *edge* instant, a lower bound on the physical
/// cut — before the debounce delay, not after), wait `debounce`, then
/// re-read the line. If the power-failing level persists, run the mandatory
/// tier to completion and *then* the best-effort tier, and return
/// [`WatcherOutcome::Fired`]. A blip that has relaxed by the re-read is
/// discarded and the watcher keeps waiting. A source error ends the watcher
/// with [`WatcherOutcome::SourceEnded`].
///
/// `debounce` of zero is legitimate (the mandatory-tier latency test uses
/// it to time the tier alone); production passes the configured window.
pub fn run_watcher<S, R, C>(
    mut source: S,
    mut response: R,
    debounce: Duration,
    now_mono_ns: C,
) -> WatcherOutcome
where
    S: EdgeSource,
    R: PowerFailResponse,
    C: Fn() -> u64,
{
    loop {
        if source.wait_for_edge().is_err() {
            return WatcherOutcome::SourceEnded;
        }
        // The marker carries the edge instant, captured BEFORE the debounce
        // sleep: it is a lower bound on the cut, which reconstruction widens
        // upward by the hold-up margin (see MarkerKind::PowerFailing).
        let edge_mono_ns = now_mono_ns();
        if !debounce.is_zero() {
            std::thread::sleep(debounce);
        }
        match source.is_asserted() {
            // Confirmed: MANDATORY tier to completion, THEN best-effort.
            Ok(true) => {
                response.mandatory(edge_mono_ns);
                response.best_effort();
                return WatcherOutcome::Fired;
            }
            // A spurious blip (EMI on a browning-out rail) that has already
            // relaxed: ignore it and keep watching. Journaling it would be a
            // false marker — safe (reconstruction discards a non-tail
            // PowerFailing marker) but noise, so the debounce filters it.
            Ok(false) => {}
            Err(_) => return WatcherOutcome::SourceEnded,
        }
    }
}

// ----------------------------------------------------------------------
// Linux-only concrete pieces: the gpiocdev edge source, the WAL-channel
// response, and the spawn helper.
// ----------------------------------------------------------------------

#[cfg(target_os = "linux")]
pub use linux::{spawn, WalChannelResponse};

#[cfg(target_os = "linux")]
mod linux {
    use std::thread::JoinHandle;

    use gpiocdev::line::{EdgeDetection, Value};
    use gpiocdev::Request;
    use plr_wal::{Marker, MarkerKind, WalRecord};

    use super::{run_watcher, EdgeError, EdgeSource, PowerFailResponse, WatcherOutcome};
    use crate::config::{PowerFailEdge, PowerFailGpio};
    use crate::sender::{SyncPolicy, WalCmd};

    /// The real edge source: a `gpiocdev` request on one line of a GPIO
    /// character device, configured for the active edge, with a blocking
    /// edge read and a single-line value read for the debounce re-read.
    ///
    /// **Not exercisable in CI:** opening `/dev/gpiochipN` and reading a
    /// real edge needs hardware and privileges no runner has. The response
    /// path it feeds is fully tested through the synthetic [`EdgeSource`];
    /// this type is the thin, honest untested boundary.
    pub struct GpioEdgeSource {
        request: Request,
        line: u32,
        /// The physical level that means "power failing": high for a rising
        /// active edge, low for a falling one.
        asserted_high: bool,
    }

    impl GpioEdgeSource {
        /// Opens the configured line for edge detection. Errors if the chip
        /// or line is unavailable.
        pub fn open(cfg: &PowerFailGpio) -> Result<Self, EdgeError> {
            let (edge, asserted_high) = match cfg.active_edge {
                PowerFailEdge::Rising => (EdgeDetection::RisingEdge, true),
                PowerFailEdge::Falling => (EdgeDetection::FallingEdge, false),
            };
            let request = Request::builder()
                .on_chip(&cfg.chip)
                .with_consumer("plrd-powerfail")
                .with_line(cfg.line)
                .with_edge_detection(edge)
                .request()
                .map_err(|e| {
                    EdgeError::new(format!(
                        "opening {} line {}: {e}",
                        cfg.chip.display(),
                        cfg.line
                    ))
                })?;
            Ok(Self {
                request,
                line: cfg.line,
                asserted_high,
            })
        }
    }

    impl EdgeSource for GpioEdgeSource {
        fn wait_for_edge(&mut self) -> Result<(), EdgeError> {
            // Blocking: the kernel wakes us on the edge. No polling.
            self.request
                .read_edge_event()
                .map(|_| ())
                .map_err(|e| EdgeError::new(format!("reading edge event: {e}")))
        }

        fn is_asserted(&mut self) -> Result<bool, EdgeError> {
            let value = self
                .request
                .value(self.line)
                .map_err(|e| EdgeError::new(format!("reading line value: {e}")))?;
            let high = value == Value::Active;
            Ok(high == self.asserted_high)
        }
    }

    /// The production response: the mandatory tier hands the WAL thread a
    /// `PowerFailing` marker over the same channel the recorder uses; the
    /// best-effort tier fires the injected clean-exit hook.
    pub struct WalChannelResponse {
        tx: std::sync::mpsc::SyncSender<WalCmd>,
        cleanup: Box<dyn FnMut() + Send>,
    }

    impl WalChannelResponse {
        /// `tx` is a clone of the WAL channel's sender (from `daemon`);
        /// `cleanup` triggers the daemon's power-fail clean exit (it must
        /// NOT write a `RecorderStopped` marker — the print really did die).
        pub fn new(
            tx: std::sync::mpsc::SyncSender<WalCmd>,
            cleanup: Box<dyn FnMut() + Send>,
        ) -> Self {
            Self { tx, cleanup }
        }
    }

    impl PowerFailResponse for WalChannelResponse {
        fn mandatory(&mut self, edge_mono_ns: u64) {
            // BLOCKING send = never-dropped: waits for one channel slot
            // rather than dropping (the socket reader drops; this must not).
            // The WAL thread then appends+fsyncs the marker on its Immediate
            // path AND force-syncs the heartbeat file (see `walsvc`'s Append
            // handler). The send returns once the command is enqueued, ahead
            // of any best-effort `Shutdown`, so FIFO delivery guarantees the
            // marker is journaled before the clean exit's final sync.
            let marker = WalRecord::Marker(Marker {
                mono_ns: edge_mono_ns,
                kind: MarkerKind::PowerFailing,
            });
            if self
                .tx
                .send(WalCmd::Append {
                    record: marker,
                    sync: SyncPolicy::Immediate,
                })
                .is_err()
            {
                // The WAL thread is already gone; nothing can be journaled.
                // Logged, never fatal — the rail is dying regardless.
                eprintln!(
                    "plrd: power-fail edge observed, but the WAL thread is gone; \
                     the PowerFailing marker could not be journaled"
                );
            }
        }

        fn best_effort(&mut self) {
            (self.cleanup)();
        }
    }

    /// Spawns the watcher on its own named thread. Opening the GPIO is done
    /// on that thread so a slow or failing open never blocks daemon
    /// startup; an open failure logs loudly and ends the watcher
    /// (`WatcherOutcome::SourceEnded`) **without** taking down the recorder
    /// — a missing power-fail sensor must never stop the daemon that would
    /// record the power loss anyway.
    pub fn spawn(cfg: PowerFailGpio, response: WalChannelResponse) -> JoinHandle<WatcherOutcome> {
        std::thread::Builder::new()
            .name("plrd-powerfail".to_owned())
            .spawn(move || match GpioEdgeSource::open(&cfg) {
                Ok(source) => run_watcher(
                    source,
                    response,
                    cfg.debounce,
                    crate::hostclock::now_mono_ns,
                ),
                Err(e) => {
                    eprintln!(
                        "plrd: FATAL for power-fail protection (recording continues): {e}. \
                         The [power_fail_gpio] section is configured but the line could not be \
                         opened; the daemon will record a power loss but cannot pre-stamp it."
                    );
                    WatcherOutcome::SourceEnded
                }
            })
            .expect("spawning the power-fail thread cannot fail")
    }
}

#[cfg(test)]
mod tests {
    use super::{run_watcher, EdgeError, EdgeSource, PowerFailResponse, WatcherOutcome};
    use std::collections::VecDeque;
    use std::time::Duration;

    /// A scripted edge source: `asserts[i]` is the value `is_asserted`
    /// returns for the i-th edge. `wait_for_edge` succeeds while edges
    /// remain and then reports the source ended.
    pub(super) struct SyntheticEdge {
        asserts: VecDeque<bool>,
    }

    impl SyntheticEdge {
        pub(super) fn new(asserts: impl IntoIterator<Item = bool>) -> Self {
            Self {
                asserts: asserts.into_iter().collect(),
            }
        }
    }

    impl EdgeSource for SyntheticEdge {
        fn wait_for_edge(&mut self) -> Result<(), EdgeError> {
            if self.asserts.is_empty() {
                Err(EdgeError::new("no more synthetic edges"))
            } else {
                Ok(())
            }
        }

        fn is_asserted(&mut self) -> Result<bool, EdgeError> {
            self.asserts
                .pop_front()
                .ok_or_else(|| EdgeError::new("no synthetic edge"))
        }
    }

    /// Records the tiers in the order they run, with the edge timestamp the
    /// mandatory tier was handed.
    #[derive(Default)]
    struct RecordingResponse {
        events: Vec<String>,
        mandatory_edge: Option<u64>,
    }

    impl PowerFailResponse for &mut RecordingResponse {
        fn mandatory(&mut self, edge_mono_ns: u64) {
            self.mandatory_edge = Some(edge_mono_ns);
            self.events.push("mandatory".to_owned());
        }
        fn best_effort(&mut self) {
            self.events.push("best_effort".to_owned());
        }
    }

    #[test]
    fn a_confirmed_edge_runs_mandatory_then_best_effort_with_the_edge_time() {
        let source = SyntheticEdge::new([true]);
        let mut response = RecordingResponse::default();
        let outcome = run_watcher(source, &mut response, Duration::ZERO, || 12_345);
        assert_eq!(outcome, WatcherOutcome::Fired);
        // The ordering the contract requires: mandatory FULLY before
        // best-effort, so a slow/failing best-effort can never delay it.
        assert_eq!(response.events, vec!["mandatory", "best_effort"]);
        // And the marker carries the edge timestamp (the lower bound on the
        // cut), captured before any debounce.
        assert_eq!(response.mandatory_edge, Some(12_345));
    }

    #[test]
    fn a_spurious_edge_is_debounced_away_and_the_watcher_keeps_waiting() {
        // First edge relaxes by the re-read (a blip); second is genuine.
        let source = SyntheticEdge::new([false, true]);
        let mut response = RecordingResponse::default();
        let outcome = run_watcher(source, &mut response, Duration::ZERO, || 7);
        assert_eq!(outcome, WatcherOutcome::Fired);
        // The blip fired NOTHING; only the genuine edge did.
        assert_eq!(response.events, vec!["mandatory", "best_effort"]);
    }

    #[test]
    fn a_blip_with_no_following_edge_never_fires() {
        // The only edge relaxes: no response runs, and the source then ends.
        let source = SyntheticEdge::new([false]);
        let mut response = RecordingResponse::default();
        let outcome = run_watcher(source, &mut response, Duration::ZERO, || 0);
        assert_eq!(outcome, WatcherOutcome::SourceEnded);
        assert!(response.events.is_empty(), "a lone blip must fire nothing");
        assert_eq!(response.mandatory_edge, None);
    }

    #[test]
    fn an_edge_source_that_ends_immediately_reports_source_ended() {
        let source = SyntheticEdge::new([]);
        let mut response = RecordingResponse::default();
        let outcome = run_watcher(source, &mut response, Duration::ZERO, || 0);
        assert_eq!(outcome, WatcherOutcome::SourceEnded);
        assert!(response.events.is_empty());
    }

    /// The debounce is actually observed between the edge and the re-read.
    /// A source that records the wall time of `wait_for_edge` and
    /// `is_asserted` proves the gap is at least the debounce.
    #[test]
    fn the_debounce_window_elapses_before_the_re_read() {
        struct TimedEdge {
            edge_at: Option<std::time::Instant>,
            gap: std::sync::Arc<std::sync::Mutex<Option<Duration>>>,
            fired: bool,
        }
        impl EdgeSource for TimedEdge {
            fn wait_for_edge(&mut self) -> Result<(), EdgeError> {
                if self.fired {
                    return Err(EdgeError::new("done"));
                }
                self.edge_at = Some(std::time::Instant::now());
                Ok(())
            }
            fn is_asserted(&mut self) -> Result<bool, EdgeError> {
                *self.gap.lock().unwrap() = self.edge_at.map(|t| t.elapsed());
                self.fired = true;
                Ok(true)
            }
        }
        let gap = std::sync::Arc::new(std::sync::Mutex::new(None));
        let source = TimedEdge {
            edge_at: None,
            gap: std::sync::Arc::clone(&gap),
            fired: false,
        };
        let mut response = RecordingResponse::default();
        let debounce = Duration::from_millis(5);
        run_watcher(source, &mut response, debounce, || 0);
        let observed = gap.lock().unwrap().expect("re-read happened");
        assert!(
            observed >= debounce,
            "re-read must be at least one debounce after the edge: {observed:?}"
        );
    }
}

// The mandatory-tier latency + durability measurement runs against a REAL
// WAL service, so it is Linux-only (walsvc is Linux-only).
#[cfg(all(test, target_os = "linux"))]
mod linux_tests {
    use std::time::{Duration, Instant};

    use plr_wal::{scan, MarkerKind, WalRecord};

    use super::linux::WalChannelResponse;
    use super::tests::SyntheticEdge;
    use super::{run_watcher, WatcherOutcome};
    use crate::sender::{HeartbeatData, WalCmd};
    use crate::walsvc::{self, WalSvcCfg};

    fn temp_dir(tag: &str) -> std::path::PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-powerfail-test-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn wal_cfg(dir: &std::path::Path) -> WalSvcCfg {
        WalSvcCfg {
            wal_dir: dir.to_path_buf(),
            heartbeat_path: dir.join("heartbeat.bin"),
            receive_seq_path: dir.join("receive_seq.bin"),
            batch_interval: Duration::from_millis(500),
            heartbeat_period: Duration::from_millis(50),
            heartbeat_o_dsync: false,
            rotate_bytes: 1 << 20,
            wal_heartbeat_quiet_every: 300,
        }
    }

    /// **Requirement 6, measured not asserted.** A synthetic edge drives
    /// the real response against a real WAL service; we time from the edge
    /// to the `PowerFailing` marker being durable on disk. The CI bound is
    /// generous (< 500 ms); the real figure is logged. This also proves the
    /// mandatory tier's two syncs both happened (the marker is on disk AND
    /// the heartbeat file advanced) and that the best-effort tier ran only
    /// after.
    #[test]
    fn mandatory_tier_latency_edge_to_marker_durable_is_well_under_the_ci_bound() {
        let dir = temp_dir("latency");
        let (tx, rx) = std::sync::mpsc::sync_channel::<WalCmd>(1024);
        let wal = walsvc::spawn(wal_cfg(&dir), rx);
        // A live correlation sample so the heartbeat file has something to
        // rewrite in the mandatory tier's second sync.
        tx.send(WalCmd::Heartbeat(Some(HeartbeatData {
            print_time: 5.0,
            est_sample_mono_ns: 1_000_000_000,
            est_sample_print_time: 5.0,
            active: true,
        })))
        .unwrap();
        // Let a first heartbeat land so the file has a baseline sequence.
        let hb_path = dir.join("heartbeat.bin");
        wait_until(Duration::from_secs(5), || {
            std::fs::read(&hb_path)
                .ok()
                .and_then(|b| plr_wal::recover_heartbeat(&b).ok())
                .is_some()
        });
        let seq_before = plr_wal::recover_heartbeat(&std::fs::read(&hb_path).unwrap())
            .unwrap()
            .heartbeat
            .sequence;

        // Record when best-effort ran, to prove it followed the mandatory
        // tier. Zero debounce: we are timing the tier itself.
        let best_effort_ran = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let flag = std::sync::Arc::clone(&best_effort_ran);
        let response = WalChannelResponse::new(
            tx.clone(),
            Box::new(move || flag.store(true, std::sync::atomic::Ordering::SeqCst)),
        );

        let start = Instant::now();
        let outcome = run_watcher(SyntheticEdge::new([true]), response, Duration::ZERO, || {
            crate::hostclock::now_mono_ns()
        });
        assert_eq!(outcome, WatcherOutcome::Fired);

        // Poll the segment until the durable PowerFailing marker appears.
        let seg = dir.join("wal-000001.plr");
        wait_until(Duration::from_secs(5), || {
            power_failing_marker_present(&seg)
        });
        let latency = start.elapsed();

        assert!(
            latency < Duration::from_millis(500),
            "mandatory tier took {latency:?}, over the 500 ms CI bound"
        );
        eprintln!("power-fail mandatory tier latency (edge -> marker durable): {latency:?}");

        // The heartbeat file's second sync advanced its sequence: both
        // fsyncs of the mandatory tier happened.
        let seq_after = plr_wal::recover_heartbeat(&std::fs::read(&hb_path).unwrap())
            .unwrap()
            .heartbeat
            .sequence;
        assert!(
            seq_after > seq_before,
            "the mandatory tier must force a fresh heartbeat-file sync ({seq_before} -> {seq_after})"
        );
        // Best-effort ran (after the mandatory tier — run_watcher's order).
        assert!(best_effort_ran.load(std::sync::atomic::Ordering::SeqCst));

        tx.send(WalCmd::Shutdown).unwrap();
        wal.join().unwrap().unwrap();
    }

    /// The mandatory tier is never delayed by best-effort work: even a
    /// best-effort hook that sleeps a long time cannot postpone the marker
    /// becoming durable, because `run_watcher` runs and returns from the
    /// mandatory tier first. We prove the marker is durable *before* the
    /// slow best-effort hook has finished.
    #[test]
    fn a_slow_best_effort_tier_does_not_delay_the_mandatory_tier() {
        let dir = temp_dir("never-delayed");
        let (tx, rx) = std::sync::mpsc::sync_channel::<WalCmd>(1024);
        let wal = walsvc::spawn(wal_cfg(&dir), rx);
        tx.send(WalCmd::Heartbeat(Some(HeartbeatData {
            print_time: 5.0,
            est_sample_mono_ns: 1_000_000_000,
            est_sample_print_time: 5.0,
            active: true,
        })))
        .unwrap();

        let done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let flag = std::sync::Arc::clone(&done);
        // A best-effort hook that blocks for a long time.
        let response = WalChannelResponse::new(
            tx.clone(),
            Box::new(move || {
                std::thread::sleep(Duration::from_secs(2));
                flag.store(true, std::sync::atomic::Ordering::SeqCst);
            }),
        );

        // Run the watcher on its own thread so we can observe durability
        // while the best-effort hook is still sleeping.
        let seg = dir.join("wal-000001.plr");
        let handle = std::thread::spawn(move || {
            run_watcher(SyntheticEdge::new([true]), response, Duration::ZERO, || {
                crate::hostclock::now_mono_ns()
            })
        });

        // The marker becomes durable well before the 2 s best-effort sleep.
        wait_until(Duration::from_secs(1), || {
            power_failing_marker_present(&seg)
        });
        assert!(
            !done.load(std::sync::atomic::Ordering::SeqCst),
            "the marker was durable while best-effort was still running — good — \
             but the best-effort flag was already set, so timing proved nothing"
        );

        assert_eq!(handle.join().unwrap(), WatcherOutcome::Fired);
        tx.send(WalCmd::Shutdown).unwrap();
        wal.join().unwrap().unwrap();
    }

    fn power_failing_marker_present(segment: &std::path::Path) -> bool {
        let Ok(bytes) = std::fs::read(segment) else {
            return false;
        };
        scan(&bytes).records.iter().any(|r| {
            matches!(
                &r.record,
                WalRecord::Marker(m) if m.kind == MarkerKind::PowerFailing
            )
        })
    }

    fn wait_until(budget: Duration, mut cond: impl FnMut() -> bool) {
        let deadline = Instant::now() + budget;
        while Instant::now() < deadline {
            if cond() {
                return;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        panic!("condition not met within {budget:?}");
    }
}
