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
//! The parent SIGKILLs the child at a random moment mid-stream, scans
//! the segment file, and asserts the durable-prefix contract:
//!
//! 1. the scan yields a valid prefix whose end reason is an
//!    expected-after-power-loss variant;
//! 2. **every record the child acked as durable is present** — the last
//!    `S <offset> <count>` line the parent observed means at least
//!    `count` records and `offset` bytes must survive;
//! 3. recovered records are exactly the deterministic sequence the child
//!    wrote (no reordering, no invention).
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

struct Ack {
    offset: u64,
    count: u64,
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

    // Let the child run for a random 20–120 ms, tracking the last
    // durability ack observed *before* the kill.
    let deadline = Duration::from_millis(20 + pseudo_random(iteration) % 100);
    let start = std::time::Instant::now();
    let mut last_ack = Ack {
        offset: 0,
        count: 0,
    };
    while start.elapsed() < deadline {
        match rx.recv_timeout(Duration::from_millis(5)) {
            Ok(ack) => last_ack = ack,
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }
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

    assert!(
        last_ack.count > 0,
        "iteration {iteration}: child produced no durability acks before the kill \
         (deadline {deadline:?}) — test environment too slow?"
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
