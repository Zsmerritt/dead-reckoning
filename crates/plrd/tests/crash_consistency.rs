//! Crash-consistency test: the project's durability payoff.
//!
//! A child process (this crate's own binary, hidden `__crash-writer`
//! subcommand) appends records in a tight loop with a real `fdatasync`
//! after every append, reporting durability acknowledgements over a
//! stdout pipe:
//!
//! ```text
//! P <offset> <count>   appended through <offset>, fdatasync in flight
//! S <offset> <count>   fdatasync returned: prefix below <offset> durable
//! ```
//!
//! The parent waits for the child's first durability ack, lets it run a
//! randomized distance further, SIGKILLs it mid-stream, scans the
//! segment file, and asserts the durable-prefix contract:
//!
//! 1. the scan yields a valid prefix whose end reason is an
//!    expected-after-power-loss variant;
//! 2. **every record the child acked as durable is present** — the last
//!    `S <offset> <count>` line the parent observed means at least
//!    `count` records and `offset` bytes must survive;
//! 3. recovered records are exactly the deterministic sequence the child
//!    wrote (no reordering, no invention).
//!
//! # Why the kill point is chosen from acks, not from the clock
//!
//! The kill has to land while writes are in flight, *and* the child has
//! to have acked something durable — otherwise the contract under test
//! is vacuous. The second of those was originally assumed by sleeping a
//! random 20–120 ms, which is a race against real `fdatasync` latency:
//! it failed once under `cargo llvm-cov` (instrumentation slows the
//! child several-fold) on a `/mnt/c` 9p mount (where an `fdatasync`
//! costs tens of milliseconds), with a 25 ms draw.
//!
//! So the precondition is now observed rather than assumed: wait for the
//! first ack with a generous budget, then advance a randomized number of
//! further acks before killing. The kill is still unsynchronized with
//! the child — which never stops writing — so it still lands mid-append,
//! and the randomization now varies the *depth* of the tear in the log
//! instead of a wall-clock instant.
//!
//! SIGKILL kills the process, not the kernel: page-cache contents
//! survive, so this test proves the *process-death* half of the
//! contract (ack ⇒ durable ordering, torn-tail recovery). The
//! power-loss half — that `fdatasync` acks imply media durability — is
//! exactly what `fdatasync` is specified to provide and cannot be
//! exercised without pulling a plug; the WAL service therefore only ever
//! acks after `fdatasync` returns, which this test verifies end to end.

#![cfg(target_os = "linux")]

use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;
use std::time::Duration;

/// A unique per-iteration temp dir (no tempfile dep by policy).
fn temp_dir(tag: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!(
        "plrd-crash-test-{tag}-{}-{nanos}",
        std::process::id()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Cheap deterministic-enough randomness without a dependency: clock
/// nanoseconds stirred with a multiplier.
fn pseudo_random(seed: u64) -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .subsec_nanos();
    u64::from(nanos)
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(seed)
}

/// How long to wait for the child's **first** durability ack.
///
/// This is a precondition, not a timing assertion. The test needs the
/// child to have acked something durable before it kills it; how long
/// that takes is a property of the machine, not of the code under test.
/// Assuming a fixed span is a race — it is how this test failed once
/// under `cargo llvm-cov`, whose instrumentation slows the child
/// several-fold, on a `/mnt/c` 9p mount where a single `fdatasync` can
/// take tens of milliseconds. The budget below is never consumed on a
/// healthy run, where the first ack lands in single-digit milliseconds.
const FIRST_ACK_TIMEOUT: Duration = Duration::from_secs(30);

/// How long to wait for each *further* ack while advancing to the kill
/// point. Running out here is not a failure — the precondition is
/// already satisfied and killing now is still mid-stream — so this only
/// bounds how long a stalled child can delay the iteration.
const NEXT_ACK_TIMEOUT: Duration = Duration::from_secs(5);

/// Upper bound on how many acks past the first the kill may land, so the
/// tear lands at a different depth in the log on each iteration.
const MAX_EXTRA_ACKS: u64 = 64;

/// Upper bound on the sub-cycle jitter before the kill (microseconds).
/// Decorrelates the kill from the instant an `fdatasync` returned, so
/// tears land mid-append as well as between appends.
const MAX_KILL_JITTER_US: u64 = 5_000;

struct Ack {
    offset: u64,
    count: u64,
}

/// Blocks until the child reports its first durability ack.
///
/// Fails with a diagnosis naming what was waited for and what happened
/// instead — the two distinguishable outcomes are "the writer is stuck"
/// and "the writer died", which need very different follow-up.
fn await_first_ack(rx: &mpsc::Receiver<Ack>, iteration: u64) -> Ack {
    match rx.recv_timeout(FIRST_ACK_TIMEOUT) {
        Ok(ack) => ack,
        Err(mpsc::RecvTimeoutError::Timeout) => panic!(
            "iteration {iteration}: crash-writer child produced no durability ack within \
             {FIRST_ACK_TIMEOUT:?}; it is stuck before its first fdatasync, or this \
             environment cannot fdatasync at all"
        ),
        Err(mpsc::RecvTimeoutError::Disconnected) => panic!(
            "iteration {iteration}: crash-writer child exited before acking anything \
             durable (check its stderr above)"
        ),
    }
}

/// Runs one kill iteration; returns (records survived, records acked).
fn run_one(iteration: u64) -> (usize, u64) {
    let dir = temp_dir(&format!("iter{iteration}"));
    let mut child: Child = Command::new(env!("CARGO_BIN_EXE_plrd"))
        .arg("__crash-writer")
        .arg(&dir)
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("spawn crash-writer child");
    let stdout = child.stdout.take().expect("piped stdout");

    // Reader thread: stream acks to the main thread as they arrive.
    let (tx, rx) = mpsc::channel::<Ack>();
    let reader = std::thread::spawn(move || {
        for line in BufReader::new(stdout).lines() {
            let Ok(line) = line else { break };
            let mut parts = line.split_whitespace();
            let tag = parts.next();
            let offset = parts.next().and_then(|p| p.parse::<u64>().ok());
            let count = parts.next().and_then(|p| p.parse::<u64>().ok());
            if let (Some("S"), Some(offset), Some(count)) = (tag, offset, count) {
                if tx.send(Ack { offset, count }).is_err() {
                    break;
                }
            }
        }
    });

    // The kill must land while writes are in flight, and the child must
    // already have acked something durable for the contract to have any
    // teeth. Both used to be assumed from a fixed 20–120 ms window; the
    // ack is now *observed* and only then is the kill moment chosen.
    let mut last_ack = await_first_ack(&rx, iteration);

    // Advance a randomized distance further into the stream so the tear
    // lands at a different depth each iteration. The child writes in an
    // unbounded tight loop, so it is still mid-stream however far we go;
    // exhausting the budget here is fine, not a failure.
    let extra_acks = pseudo_random(iteration) % MAX_EXTRA_ACKS;
    for _ in 0..extra_acks {
        match rx.recv_timeout(NEXT_ACK_TIMEOUT) {
            Ok(ack) => last_ack = ack,
            Err(_) => break,
        }
    }
    // Jitter inside one append/fsync cycle so the kill is not correlated
    // with the instant an ack was printed. Purely a spread: a short
    // sleep cannot make any assertion below fail.
    std::thread::sleep(Duration::from_micros(
        pseudo_random(iteration.wrapping_add(0x5a5a_5a5a)) % MAX_KILL_JITTER_US,
    ));

    // SIGKILL: no cleanup, no flush — the closest a test can get to
    // yanking the process's power.
    child.kill().expect("SIGKILL child");
    let _ = child.wait();
    reader.join().unwrap();
    // Acks may still be queued from before the kill; they all count
    // (they were printed after fdatasync returned).
    while let Ok(ack) = rx.try_recv() {
        last_ack = ack;
    }

    // Guaranteed by `await_first_ack`; kept as an explicit statement of
    // the precondition the assertions below rely on.
    assert!(
        last_ack.count > 0,
        "iteration {iteration}: no durability ack survived to the assertions"
    );

    // Scan what survived.
    let segment = dir.join("wal-000001.plr");
    let bytes = std::fs::read(&segment).expect("segment file must exist");
    let scan = plr_wal::scan(&bytes);

    // 1. The end of the log is an expected post-power-loss shape.
    assert!(
        scan.end.is_expected_after_power_loss(),
        "iteration {iteration}: scan ended with unexpected reason {:?} at {}",
        scan.end,
        scan.truncation_offset,
    );

    // 2. Every acked record survived: the durable prefix reaches at
    //    least the last acked offset and count.
    assert!(
        scan.truncation_offset >= last_ack.offset,
        "iteration {iteration}: durable prefix ends at {} but {} bytes were acked",
        scan.truncation_offset,
        last_ack.offset,
    );
    assert!(
        scan.records.len() as u64 >= last_ack.count,
        "iteration {iteration}: {} records recovered but {} were acked durable",
        scan.records.len(),
        last_ack.count,
    );

    // 3. Recovered records are exactly the deterministic sequence the
    //    child wrote: record i is a SubscriptionGap marker with
    //    mono_ns == i, bounds (3i, 7i).
    for (i, scanned) in scan.records.iter().enumerate() {
        let i = i as u64;
        let plr_wal::WalRecord::Marker(marker) = &scanned.record else {
            panic!("iteration {iteration}: record {i} has wrong kind: {scanned:?}");
        };
        assert_eq!(marker.mono_ns, i, "iteration {iteration}");
        assert_eq!(
            marker.kind,
            plr_wal::MarkerKind::SubscriptionGap {
                start_mono_ns: i.saturating_mul(3),
                end_mono_ns: i.saturating_mul(7),
            },
            "iteration {iteration}"
        );
    }

    let _ = std::fs::remove_dir_all(&dir);
    (scan.records.len(), last_ack.count)
}

#[test]
fn killed_writer_never_loses_an_acked_record() {
    let mut total_survived = 0_usize;
    let mut total_acked = 0_u64;
    for iteration in 0..8 {
        let (survived, acked) = run_one(iteration);
        total_survived += survived;
        total_acked += acked;
    }
    // Sanity: the harness actually exercised real volume.
    assert!(
        total_acked >= 8,
        "suspiciously few acked records across all iterations: {total_acked}"
    );
    println!(
        "crash-consistency: 8 iterations, {total_acked} records acked durable, \
         {total_survived} records recovered (>= acked in every iteration)"
    );
}
