//! The WAL writer service: the one thread that owns the files and the
//! only place in the project where "durable" becomes a syscall.
//!
//! # Thread model
//!
//! A dedicated **sync** thread (std I/O + rustix, no tokio) owns the
//! segment file, the heartbeat file, and the receive-seq sidecar. The
//! async side talks to it over one bounded `std::sync::mpsc` channel
//! (see `sender` for the drop policy). Sync I/O on its own thread keeps
//! `fdatasync` stalls away from the socket reader — Klipper disconnects
//! slow clients.
//!
//! # Durability rules (the point of this crate)
//!
//! * **Motion records** (`TrapqSegment`, `StepperRange`): appended
//!   immediately, `fdatasync`'d on a batch cadence (default 0.5 s —
//!   matching Klipper's own 0.5 s dump batching, so the extra loss
//!   window is at most one batch on top of an already-batched source).
//! * **Markers and contexts**: `fdatasync` immediately after the append.
//!   They are rare and each one changes what recovery is allowed to do.
//! * **Heartbeat**: rewritten in place at the configured rate (default
//!   10 Hz) using the dual-slot protocol (`slot_for_sequence`), then
//!   `fdatasync`'d — or written through `O_DSYNC` when configured, which
//!   makes the `write` itself synchronous (same guarantee, one syscall).
//!   Every Nth heartbeat is also appended to the WAL (batched) so the
//!   log itself carries correlation samples.
//! * **Receive-seq sidecar**: rewritten + `fdatasync`'d on every counter
//!   advance (~1 Hz). Torn writes only lose the observation, which is
//!   the safe direction (see `seqfile`).
//!
//! # Segment rotation — crash ordering
//!
//! Rotation at the size threshold performs, in order:
//!
//! 1. `fdatasync` the finished segment — every record of segment N is
//!    durable before N+1 can exist, so no crash can surface N+1 while N
//!    still has an undurable tail;
//! 2. create segment N+1 (`O_EXCL`), write its header, `fdatasync` it —
//!    if power dies here the directory entry may be lost or the file may
//!    be empty/torn, both of which `plr_wal::scan` classifies as
//!    expected-after-power-loss shapes;
//! 3. `fsync` the WAL directory — the directory entry for N+1 is
//!    durable before any record is appended to it, so a record acked as
//!    durable can never live in a file whose *name* is not.
//!
//! Appends to N+1 only start after step 3 completes (single-threaded
//! loop). The daemon never resumes an old segment: each start creates a
//! fresh segment (index = max existing + 1), leaving crash evidence in
//! earlier segments untouched for `plrd scan`.
//!
//! # Heartbeat sequence across restarts
//!
//! On startup the existing heartbeat file (if any) is recovered and the
//! sequence resumes from `recovered + 1`. Starting over at 0 would make
//! the pre-crash slot *newer* by the wrapping comparison and recovery
//! would report stale liveness; resuming preserves newest-wins. The file
//! is never zeroed: a daemon restart must not destroy crash evidence.

use std::fs::File;
use std::io;
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::time::{Duration, Instant};

use plr_wal::heartbeat::HEARTBEAT_FILE_LEN;
use plr_wal::{
    encode_slot, recover_heartbeat, slot_for_sequence, Heartbeat, SegmentHeader, WalError,
    WalRecord, WalWriter,
};
use rustix::fs::{Mode, OFlags};

use crate::hostclock::{now_mono_ns, now_wall_ns};
use crate::scan::{segment_file_name, segment_index};
use crate::sender::{HeartbeatData, SyncPolicy, WalCmd};
use crate::seqfile::encode_seq;

use crate::convert::WAL_HEARTBEAT_EVERY;

/// Service configuration (derived from `config::Config` by the daemon).
#[derive(Debug, Clone)]
pub struct WalSvcCfg {
    /// Directory for segments and the sidecar.
    pub wal_dir: PathBuf,
    /// Heartbeat file path.
    pub heartbeat_path: PathBuf,
    /// Receive-seq sidecar path.
    pub receive_seq_path: PathBuf,
    /// Batch `fdatasync` interval for motion records.
    pub batch_interval: Duration,
    /// Heartbeat rewrite period.
    pub heartbeat_period: Duration,
    /// Open the heartbeat file `O_DSYNC` instead of per-write
    /// `fdatasync`.
    pub heartbeat_o_dsync: bool,
    /// Segment rotation threshold in bytes.
    pub rotate_bytes: u64,
}

/// Fatal service errors. Anything here means durability can no longer be
/// promised; the daemon exits nonzero and systemd restarts it.
#[derive(Debug, thiserror::Error)]
pub enum WalSvcError {
    /// File or sync syscall failure.
    #[error("wal service i/o: {0}")]
    Io(#[from] io::Error),
}

/// Spawns the service thread. It exits `Ok` after a `Shutdown` command
/// (or when every sender is dropped), `Err` on fatal I/O.
pub fn spawn(
    cfg: WalSvcCfg,
    rx: Receiver<WalCmd>,
) -> std::thread::JoinHandle<Result<(), WalSvcError>> {
    std::thread::Builder::new()
        .name("plrd-wal".to_owned())
        .spawn(move || Service::init(cfg)?.run(&rx))
        .expect("spawning the WAL thread cannot fail")
}

struct Service {
    cfg: WalSvcCfg,
    dir: File,
    writer: WalWriter<File>,
    seg_index: u64,
    hb_file: File,
    hb_seq: u64,
    hb_data: Option<HeartbeatData>,
    seq_file: File,
    dirty: bool,
    batch_deadline: Option<Instant>,
    next_heartbeat: Instant,
}

impl Service {
    fn init(cfg: WalSvcCfg) -> Result<Self, WalSvcError> {
        std::fs::create_dir_all(&cfg.wal_dir)?;
        let dir = File::open(&cfg.wal_dir)?;

        let seg_index = next_segment_index(&cfg.wal_dir)?;
        let writer = create_segment(&cfg.wal_dir, seg_index)?;

        // Heartbeat: resume the sequence from any prior file (see module
        // docs) and never destroy prior contents.
        let hb_seq = match std::fs::read(&cfg.heartbeat_path) {
            Ok(bytes) => {
                recover_heartbeat(&bytes).map_or(0, |r| r.heartbeat.sequence.wrapping_add(1))
            }
            Err(_) => 0,
        };
        let hb_file = open_heartbeat(&cfg.heartbeat_path, cfg.heartbeat_o_dsync)?;
        // Fix the file size once; rewrites never change it afterwards.
        if hb_file.metadata()?.len() != HEARTBEAT_FILE_LEN as u64 {
            hb_file.set_len(HEARTBEAT_FILE_LEN as u64)?;
        }
        fdatasync(&hb_file)?;

        let seq_file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&cfg.receive_seq_path)?;

        // Make the directory entries for everything created above
        // durable before the first record can be acknowledged.
        fsync(&dir)?;
        for parent in [cfg.heartbeat_path.parent(), cfg.receive_seq_path.parent()]
            .into_iter()
            .flatten()
        {
            if parent != cfg.wal_dir {
                fsync(&File::open(parent)?)?;
            }
        }

        let next_heartbeat = Instant::now() + cfg.heartbeat_period;
        Ok(Self {
            cfg,
            dir,
            writer,
            seg_index,
            hb_file,
            hb_seq,
            hb_data: None,
            seq_file,
            dirty: false,
            batch_deadline: None,
            next_heartbeat,
        })
    }

    fn run(mut self, rx: &Receiver<WalCmd>) -> Result<(), WalSvcError> {
        loop {
            let now = Instant::now();
            let mut deadline = self.next_heartbeat;
            if let Some(batch) = self.batch_deadline {
                deadline = deadline.min(batch);
            }
            match rx.recv_timeout(deadline.saturating_duration_since(now)) {
                Ok(WalCmd::Shutdown) | Err(RecvTimeoutError::Disconnected) => break,
                Ok(cmd) => self.handle(cmd)?,
                Err(RecvTimeoutError::Timeout) => {}
            }
            let now = Instant::now();
            if self.batch_deadline.is_some_and(|b| now >= b) {
                self.sync_segment()?;
            }
            if now >= self.next_heartbeat {
                self.heartbeat_tick()?;
                // Fixed cadence, but never a burst of catch-up beats
                // after a stall.
                self.next_heartbeat += self.cfg.heartbeat_period;
                if self.next_heartbeat < now {
                    self.next_heartbeat = now + self.cfg.heartbeat_period;
                }
            }
        }
        // Final sync: everything handed to the OS becomes durable before
        // a clean exit.
        self.sync_segment()?;
        Ok(())
    }

    fn handle(&mut self, cmd: WalCmd) -> Result<(), WalSvcError> {
        match cmd {
            WalCmd::Append { record, sync } => self.append(&record, sync),
            WalCmd::Heartbeat(data) => {
                self.hb_data = data;
                Ok(())
            }
            WalCmd::ReceiveSeq { mono_ns, widened } => {
                self.seq_file
                    .write_all_at(&encode_seq(mono_ns, widened), 0)?;
                fdatasync(&self.seq_file)?;
                Ok(())
            }
            WalCmd::Shutdown => unreachable!("Shutdown is handled by the run loop"),
        }
    }

    fn append(&mut self, record: &WalRecord, sync: SyncPolicy) -> Result<(), WalSvcError> {
        match self.writer.append(record) {
            Ok(_) => {}
            // The stream may now end in a partial frame; that is exactly
            // the torn-tail shape the scan recovers from, but this
            // writer must not continue.
            Err(WalError::Io(e)) => return Err(e.into()),
            // Data-shaped rejections (non-finite floats from a confused
            // Klipper, oversized payloads): skip the record, keep the
            // log intact, say so.
            Err(e) => {
                eprintln!("plrd: WAL record skipped: {e}");
                return Ok(());
            }
        }
        self.dirty = true;
        match sync {
            SyncPolicy::Immediate => self.sync_segment()?,
            SyncPolicy::Batched => {
                if self.batch_deadline.is_none() {
                    self.batch_deadline = Some(Instant::now() + self.cfg.batch_interval);
                }
            }
        }
        if self.writer.offset() >= self.cfg.rotate_bytes {
            self.rotate()?;
        }
        Ok(())
    }

    fn sync_segment(&mut self) -> Result<(), WalSvcError> {
        if self.dirty {
            fdatasync(self.writer.get_mut())?;
            self.dirty = false;
        }
        self.batch_deadline = None;
        Ok(())
    }

    /// See "Segment rotation — crash ordering" in the module docs.
    fn rotate(&mut self) -> Result<(), WalSvcError> {
        self.sync_segment()?; // 1. finished segment fully durable
        let next = self.seg_index + 1;
        let writer = create_segment(&self.cfg.wal_dir, next)?; // 2. header durable
        fsync(&self.dir)?; // 3. directory entry durable
        self.writer = writer;
        self.seg_index = next;
        Ok(())
    }

    fn heartbeat_tick(&mut self) -> Result<(), WalSvcError> {
        // No correlation sample yet (or paused after a socket loss): no
        // liveness claim.
        let Some(data) = self.hb_data else {
            return Ok(());
        };
        let heartbeat = Heartbeat {
            sequence: self.hb_seq,
            mono_ns: now_mono_ns(),
            wall_ns: now_wall_ns(),
            print_time: data.print_time,
            est_sample_mono_ns: data.est_sample_mono_ns,
            est_sample_print_time: data.est_sample_print_time,
            wal_offset: self.writer.offset(),
        };
        let slot = slot_for_sequence(heartbeat.sequence);
        self.hb_file
            .write_all_at(&encode_slot(&heartbeat), slot.offset() as u64)?;
        if !self.cfg.heartbeat_o_dsync {
            // With O_DSYNC the write above already returned durable.
            fdatasync(&self.hb_file)?;
        }
        self.hb_seq = self.hb_seq.wrapping_add(1);
        if heartbeat.sequence.is_multiple_of(WAL_HEARTBEAT_EVERY) {
            self.append(&WalRecord::Heartbeat(heartbeat), SyncPolicy::Batched)?;
        }
        Ok(())
    }
}

/// Creates segment `index` (`O_EXCL`), writes its header, and makes the
/// header durable. The caller owns directory durability.
fn create_segment(wal_dir: &Path, index: u64) -> Result<WalWriter<File>, WalSvcError> {
    let path = wal_dir.join(segment_file_name(index));
    let file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&path)?;
    let mut writer = WalWriter::create(file, &SegmentHeader::new(now_wall_ns(), now_mono_ns()))
        .map_err(|e| match e {
            WalError::Io(io) => WalSvcError::Io(io),
            // Header encoding is infallible; only I/O can fail here.
            other => WalSvcError::Io(io::Error::other(other.to_string())),
        })?;
    fdatasync(writer.get_mut())?;
    Ok(writer)
}

/// One larger index than any existing segment (1 for an empty dir).
fn next_segment_index(wal_dir: &Path) -> Result<u64, WalSvcError> {
    let mut max = 0;
    for entry in std::fs::read_dir(wal_dir)? {
        let name = entry?.file_name();
        if let Some(index) = name.to_str().and_then(segment_index) {
            max = max.max(index);
        }
    }
    Ok(max + 1)
}

fn open_heartbeat(path: &Path, o_dsync: bool) -> Result<File, WalSvcError> {
    if o_dsync {
        let fd = rustix::fs::open(
            path,
            OFlags::RDWR | OFlags::CREATE | OFlags::DSYNC | OFlags::CLOEXEC,
            Mode::from_raw_mode(0o644),
        )
        .map_err(io::Error::from)?;
        Ok(File::from(fd))
    } else {
        Ok(std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(path)?)
    }
}

fn fdatasync(file: &File) -> Result<(), WalSvcError> {
    rustix::fs::fdatasync(file).map_err(|e| WalSvcError::Io(e.into()))
}

fn fsync(file: &File) -> Result<(), WalSvcError> {
    rustix::fs::fsync(file).map_err(|e| WalSvcError::Io(e.into()))
}

/// The crash-consistency child process (hidden `__crash-writer` CLI):
/// appends deterministic records in a tight loop, `fdatasync`ing after
/// every append, and reports durability over stdout:
///
/// ```text
/// P <offset> <count>   written through <offset> (<count> records), sync in flight
/// S <offset> <count>   fdatasync returned: everything below <offset> is durable
/// ```
///
/// The parent SIGKILLs this process at a random moment and then verifies
/// the durable-prefix contract against the last `S` line it observed.
/// Runs until killed (bounded by a generous cap so an orphaned child
/// terminates on its own).
pub fn crash_writer_main(dir: &Path) -> u8 {
    match crash_writer(dir) {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("plrd __crash-writer: {e}");
            1
        }
    }
}

fn crash_writer(dir: &Path) -> Result<(), WalSvcError> {
    use std::io::Write;
    std::fs::create_dir_all(dir)?;
    let dir_file = File::open(dir)?;
    let mut writer = create_segment(dir, 1)?;
    fsync(&dir_file)?;
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    for i in 0..200_000_u64 {
        // Deterministic, identity-carrying records of varying size so
        // torn tails land at unaligned offsets.
        let record = WalRecord::Marker(plr_wal::Marker {
            mono_ns: i,
            kind: plr_wal::MarkerKind::SubscriptionGap {
                start_mono_ns: i.saturating_mul(3),
                end_mono_ns: i.saturating_mul(7),
            },
        });
        writer.append(&record).map_err(|e| match e {
            WalError::Io(io) => WalSvcError::Io(io),
            other => WalSvcError::Io(io::Error::other(other.to_string())),
        })?;
        let offset = writer.offset();
        let count = i + 1;
        writeln!(out, "P {offset} {count}")?;
        out.flush()?;
        fdatasync(writer.get_mut())?;
        writeln!(out, "S {offset} {count}")?;
        out.flush()?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{crash_writer_main, spawn, WalSvcCfg, WAL_HEARTBEAT_EVERY};
    use crate::scan::{segment_file_name, segment_index};
    use crate::sender::{HeartbeatData, SyncPolicy, WalCmd};
    use crate::seqfile::decode_seq;
    use plr_wal::heartbeat::HEARTBEAT_FILE_LEN;
    use plr_wal::{
        recover_heartbeat, scan, Marker, MarkerKind, ScanEnd, SlotError, TrapqSegment, WalRecord,
    };
    use std::path::{Path, PathBuf};
    use std::sync::mpsc::{sync_channel, SyncSender};
    use std::time::{Duration, Instant};

    // --- waiting on the service, deterministically --------------------
    //
    // These tests drive a real thread with a real timer, so "has the
    // heartbeat fired yet?" is a question about the scheduler, not about
    // the code under test. Sleeping a fixed span and asserting is a race:
    // it passes on an idle laptop and fails on a saturated CI runner that
    // starves the heartbeat thread for hundreds of milliseconds — which
    // is exactly how these tests broke. Instead, poll for the condition
    // with a generous budget: the assertion then means "the recorder does
    // produce this", not "it produced it within 80 ms on this machine".
    //
    // The budget is never consumed on a healthy machine (the conditions
    // land in a few periods), so this makes the suite deterministic
    // without making it slower.

    /// Overall budget for a polled condition.
    const WAIT_TIMEOUT: Duration = Duration::from_secs(10);
    /// How often a polled condition is re-checked.
    const WAIT_POLL: Duration = Duration::from_millis(2);

    /// Polls `probe` until it yields `Ok`, or fails the test naming what
    /// it waited for and what it last observed.
    ///
    /// `probe` returns `Err(observation)` while the condition does not
    /// hold; that observation is what the failure message reports, so a
    /// timeout says *why* it never happened rather than just that it
    /// did not.
    fn wait_for<T>(what: &str, mut probe: impl FnMut() -> Result<T, String>) -> T {
        let deadline = Instant::now() + WAIT_TIMEOUT;
        loop {
            let observed = match probe() {
                Ok(value) => return value,
                Err(observed) => observed,
            };
            assert!(
                Instant::now() < deadline,
                "timed out after {WAIT_TIMEOUT:?} waiting for {what}; last observed: {observed}"
            );
            std::thread::sleep(WAIT_POLL);
        }
    }

    /// Reads the heartbeat file, or describes why it cannot be used yet.
    fn read_heartbeat(path: &Path) -> Result<plr_wal::HeartbeatRecovery, String> {
        let bytes = std::fs::read(path).map_err(|e| format!("heartbeat file unreadable: {e}"))?;
        recover_heartbeat(&bytes).map_err(|e| format!("no valid heartbeat slot yet: {e}"))
    }

    /// Waits until the heartbeat file reports `sequence >= min_sequence`.
    ///
    /// `min_sequence` counts beats: 0 is "at least one beat landed", 1 is
    /// "at least two" (which is also what makes both slots valid).
    fn await_heartbeat(path: &Path, min_sequence: u64) -> plr_wal::HeartbeatRecovery {
        wait_for(&format!("heartbeat sequence >= {min_sequence}"), || {
            let hb = read_heartbeat(path)?;
            if hb.heartbeat.sequence >= min_sequence {
                Ok(hb)
            } else {
                Err(format!("sequence {}", hb.heartbeat.sequence))
            }
        })
    }

    /// Waits until both heartbeat slots are valid.
    ///
    /// The dual-slot protocol alternates, so an untorn recovery proves at
    /// least two beats landed — the precondition for testing the
    /// one-tick-older fallback.
    fn await_both_slots_valid(path: &Path) -> plr_wal::HeartbeatRecovery {
        wait_for("both heartbeat slots valid (>= 2 beats)", || {
            let hb = read_heartbeat(path)?;
            if hb.torn.is_none() {
                Ok(hb)
            } else {
                Err(format!(
                    "slot {:?} not valid yet: {:?}",
                    hb.slot.other(),
                    hb.torn
                ))
            }
        })
    }

    /// Waits until the receive-seq sidecar holds `expected`.
    ///
    /// Commands are consumed from the channel in order, so observing a
    /// `ReceiveSeq` land proves every command sent before it has already
    /// been applied. That turns "did the service process my command?"
    /// into an observable fact instead of a sleep.
    fn await_receive_seq(path: &Path, expected: (u64, u64)) {
        wait_for(&format!("receive_seq sidecar == {expected:?}"), || {
            let bytes = std::fs::read(path).map_err(|e| format!("sidecar unreadable: {e}"))?;
            match decode_seq(&bytes) {
                Some(seq) if seq == expected => Ok(()),
                other => Err(format!("sidecar holds {other:?}")),
            }
        });
    }

    /// A unique per-test temp dir (no tempfile dep by policy).
    fn temp_dir(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-walsvc-test-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn cfg(dir: &Path, o_dsync: bool, rotate: u64) -> WalSvcCfg {
        WalSvcCfg {
            wal_dir: dir.to_path_buf(),
            heartbeat_path: dir.join("heartbeat.bin"),
            receive_seq_path: dir.join("receive_seq.bin"),
            batch_interval: Duration::from_millis(40),
            heartbeat_period: Duration::from_millis(15),
            heartbeat_o_dsync: o_dsync,
            rotate_bytes: rotate,
        }
    }

    fn trapq_record(mono_ns: u64) -> WalRecord {
        WalRecord::TrapqSegment(TrapqSegment {
            mono_ns,
            queue: "toolhead".to_owned(),
            print_time: 12.5,
            duration: 0.075,
            start_velocity: 40.0,
            acceleration: -1500.0,
            start_x: 10.0,
            start_y: 20.0,
            start_z: 0.4,
            x_r: 0.6,
            y_r: 0.8,
            z_r: 0.0,
        })
    }

    fn heartbeat_data() -> HeartbeatData {
        HeartbeatData {
            print_time: 12.625,
            est_sample_mono_ns: 5_500,
            est_sample_print_time: 12.61,
        }
    }

    fn send(tx: &SyncSender<WalCmd>, cmd: WalCmd) {
        tx.send(cmd).expect("service alive");
    }

    fn scan_segment(dir: &Path, index: u64) -> plr_wal::RecoveryScan {
        scan(&std::fs::read(dir.join(segment_file_name(index))).unwrap())
    }

    #[test]
    fn end_to_end_append_sync_rotate_then_scan_validates() {
        let dir = temp_dir("e2e");
        // Rotation threshold small enough that ~20 trapq records span
        // multiple segments (a trapq frame is ~350 bytes).
        let (tx, rx) = sync_channel(256);
        let handle = spawn(cfg(&dir, false, 2_000), rx);

        send(&tx, WalCmd::Heartbeat(Some(heartbeat_data())));
        let mut sent = Vec::new();
        for i in 0..20_u64 {
            let record = trapq_record(i);
            sent.push(record.clone());
            send(
                &tx,
                WalCmd::Append {
                    record,
                    sync: SyncPolicy::Batched,
                },
            );
        }
        let marker = WalRecord::Marker(Marker {
            mono_ns: 999,
            kind: MarkerKind::SocketLost,
        });
        sent.push(marker.clone());
        send(
            &tx,
            WalCmd::Append {
                record: marker,
                sync: SyncPolicy::Immediate,
            },
        );
        send(
            &tx,
            WalCmd::ReceiveSeq {
                mono_ns: 777,
                widened: 4_242,
            },
        );
        // Wait for the beats the assertions below need (>= 2 file
        // beats, which also guarantees beat 0's WAL heartbeat record was
        // appended) instead of guessing at a duration.
        await_heartbeat(&dir.join("heartbeat.bin"), 1);
        send(&tx, WalCmd::Shutdown);
        handle.join().unwrap().unwrap();

        // Multiple segments; every segment scans back valid.
        let mut indices: Vec<u64> = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(|e| segment_index(e.unwrap().file_name().to_str().unwrap()))
            .collect();
        indices.sort_unstable();
        assert!(
            indices.len() >= 2,
            "expected rotation to produce >= 2 segments, got {indices:?}"
        );
        let mut recovered = Vec::new();
        for &index in &indices {
            let result = scan_segment(&dir, index);
            assert_eq!(result.end, ScanEnd::CleanEof, "segment {index}");
            assert!(result.header.is_some(), "segment {index}");
            recovered.extend(result.records.into_iter().map(|r| r.record));
        }
        // The deterministic subset (everything but the service's own
        // heartbeat records) survives byte-exact and in order.
        let non_heartbeat: Vec<WalRecord> = recovered
            .iter()
            .filter(|r| !matches!(r, WalRecord::Heartbeat(_)))
            .cloned()
            .collect();
        assert_eq!(non_heartbeat, sent);
        // Heartbeat records were interleaved at the 1-in-N cadence.
        let wal_heartbeats = recovered
            .iter()
            .filter(|r| matches!(r, WalRecord::Heartbeat(_)))
            .count();
        assert!(wal_heartbeats >= 1, "expected >= 1 WAL heartbeat record");

        // Heartbeat file: recovered, fresh sequence, correct payload,
        // wal_offset within the log.
        let hb = recover_heartbeat(&std::fs::read(dir.join("heartbeat.bin")).unwrap()).unwrap();
        assert!(hb.heartbeat.sequence >= 1);
        assert!((hb.heartbeat.print_time - 12.625).abs() < 1e-12);
        assert_eq!(hb.heartbeat.est_sample_mono_ns, 5_500);
        assert!(hb.heartbeat.wal_offset > 0);
        assert!(hb.heartbeat.mono_ns > 0);

        // Sidecar decodes to what was sent.
        let seq = std::fs::read(dir.join("receive_seq.bin")).unwrap();
        assert_eq!(decode_seq(&seq), Some((777, 4_242)));
    }

    #[test]
    fn heartbeat_torn_slot_falls_back_to_previous_beat() {
        let dir = temp_dir("torn");
        let (tx, rx) = sync_channel(16);
        let handle = spawn(cfg(&dir, false, 1 << 20), rx);
        send(&tx, WalCmd::Heartbeat(Some(heartbeat_data())));
        // The fallback can only be tested once both slots hold a beat;
        // wait for that rather than for a duration that might cover it.
        await_both_slots_valid(&dir.join("heartbeat.bin"));
        send(&tx, WalCmd::Shutdown);
        handle.join().unwrap().unwrap();

        let path = dir.join("heartbeat.bin");
        let mut bytes = std::fs::read(&path).unwrap();
        assert_eq!(bytes.len(), HEARTBEAT_FILE_LEN);
        let intact = recover_heartbeat(&bytes).unwrap();
        assert!(
            intact.torn.is_none(),
            "both slots must be valid after >= 2 beats"
        );
        // Tear the *newest* slot mid-field, as a power cut mid-rewrite
        // would.
        let newest_offset = intact.slot.offset();
        bytes[newest_offset + 17] ^= 0x80;
        std::fs::write(&path, &bytes).unwrap();
        let recovered = recover_heartbeat(&std::fs::read(&path).unwrap()).unwrap();
        assert_eq!(recovered.slot, intact.slot.other());
        assert_eq!(
            recovered.heartbeat.sequence,
            intact.heartbeat.sequence.wrapping_sub(1),
            "fallback must be exactly one tick older"
        );
        assert_eq!(
            recovered.torn,
            Some((intact.slot, SlotError::CrcMismatch)),
            "the tear must be reported"
        );
    }

    #[test]
    fn heartbeat_o_dsync_path_produces_valid_slots() {
        let dir = temp_dir("dsync");
        let (tx, rx) = sync_channel(16);
        // o_dsync = true: the O_DSYNC write path, not per-write fsync.
        let handle = spawn(cfg(&dir, true, 1 << 20), rx);
        send(&tx, WalCmd::Heartbeat(Some(heartbeat_data())));
        await_heartbeat(&dir.join("heartbeat.bin"), 1);
        send(&tx, WalCmd::Shutdown);
        handle.join().unwrap().unwrap();
        let hb = recover_heartbeat(&std::fs::read(dir.join("heartbeat.bin")).unwrap()).unwrap();
        assert!((hb.heartbeat.print_time - 12.625).abs() < 1e-12);
        assert!(hb.heartbeat.sequence >= 1);
    }

    #[test]
    fn no_heartbeats_without_correlation_data_and_pause_works() {
        let dir = temp_dir("pause");
        let (tx, rx) = sync_channel(16);
        let handle = spawn(cfg(&dir, false, 1 << 20), rx);
        let hb_path = dir.join("heartbeat.bin");
        let seq_path = dir.join("receive_seq.bin");
        // No Heartbeat command yet: no liveness claim may be written.
        // This one waits rather than polls on purpose — absence cannot
        // be polled for, and the wait is in the safe direction: a slower
        // machine only gives the service *more* chances to violate the
        // invariant. Several heartbeat periods (15 ms) is plenty.
        std::thread::sleep(Duration::from_millis(60));
        assert!(
            read_heartbeat(&hb_path).is_err(),
            "a heartbeat was written without correlation data"
        );

        // Provide data and wait for a real beat.
        send(&tx, WalCmd::Heartbeat(Some(heartbeat_data())));
        await_heartbeat(&hb_path, 0);

        // Pause, then prove the pause was *applied* by observing a
        // command sent after it take effect: the run loop consumes the
        // channel in order, so the sidecar landing means the pause did.
        send(&tx, WalCmd::Heartbeat(None));
        send(
            &tx,
            WalCmd::ReceiveSeq {
                mono_ns: 1,
                widened: 99,
            },
        );
        await_receive_seq(&seq_path, (1, 99));
        let paused = read_heartbeat(&hb_path).unwrap().heartbeat.sequence;

        // Now give the service every chance to beat anyway. Again a
        // sleep, again in the safe direction: it can only make a
        // regression more likely to be caught.
        std::thread::sleep(Duration::from_millis(60));
        let still = read_heartbeat(&hb_path).unwrap().heartbeat.sequence;
        assert_eq!(paused, still, "paused heartbeat must not advance");
        send(&tx, WalCmd::Shutdown);
        handle.join().unwrap().unwrap();
    }

    #[test]
    fn restart_resumes_heartbeat_sequence_and_starts_new_segment() {
        let dir = temp_dir("restart");
        let (tx, rx) = sync_channel(16);
        let hb_path = dir.join("heartbeat.bin");
        let handle = spawn(cfg(&dir, false, 1 << 20), rx);
        send(&tx, WalCmd::Heartbeat(Some(heartbeat_data())));
        await_heartbeat(&hb_path, 0);
        send(&tx, WalCmd::Shutdown);
        handle.join().unwrap().unwrap();
        let first_run = read_heartbeat(&hb_path).unwrap();

        let (tx, rx) = sync_channel(16);
        let handle = spawn(cfg(&dir, false, 1 << 20), rx);
        send(&tx, WalCmd::Heartbeat(Some(heartbeat_data())));
        // Wait for a beat that could only come from the second run: the
        // sequence resumes past the first run's, so this is exactly the
        // "sequence resumed" property the assertions below check.
        await_heartbeat(&hb_path, first_run.heartbeat.sequence + 1);
        send(&tx, WalCmd::Shutdown);
        handle.join().unwrap().unwrap();
        let second_run = read_heartbeat(&hb_path).unwrap();

        // Sequence resumed (newest-wins preserved across restarts), and
        // each run created its own segment.
        assert!(second_run.heartbeat.sequence > first_run.heartbeat.sequence);
        assert!(dir.join(segment_file_name(1)).exists());
        assert!(dir.join(segment_file_name(2)).exists());
        assert_eq!(scan_segment(&dir, 1).end, ScanEnd::CleanEof);
        assert_eq!(scan_segment(&dir, 2).end, ScanEnd::CleanEof);
    }

    #[test]
    fn wal_heartbeat_cadence_is_one_in_n() {
        let dir = temp_dir("cadence");
        let (tx, rx) = sync_channel(16);
        let mut c = cfg(&dir, false, 1 << 20);
        c.heartbeat_period = Duration::from_millis(5);
        let handle = spawn(c, rx);
        send(&tx, WalCmd::Heartbeat(Some(heartbeat_data())));
        // Enough beats for the 1-in-N ratio to be meaningful. The
        // assertion below is self-calibrating (it derives the expected
        // record count from the beats that actually happened), so this
        // only has to guarantee the sample is big enough.
        await_heartbeat(&dir.join("heartbeat.bin"), 3 * WAL_HEARTBEAT_EVERY);
        send(&tx, WalCmd::Shutdown);
        handle.join().unwrap().unwrap();
        let result = scan_segment(&dir, 1);
        let wal_heartbeats = result
            .records
            .iter()
            .filter(|r| matches!(r.record, WalRecord::Heartbeat(_)))
            .count() as u64;
        let file_beats = recover_heartbeat(&std::fs::read(dir.join("heartbeat.bin")).unwrap())
            .unwrap()
            .heartbeat
            .sequence
            + 1;
        // One WAL record per WAL_HEARTBEAT_EVERY file beats (allow the
        // off-by-one of where the modulus window started).
        let expected = file_beats / WAL_HEARTBEAT_EVERY;
        assert!(
            wal_heartbeats >= expected.saturating_sub(1) && wal_heartbeats <= expected + 1,
            "beats {file_beats}, wal records {wal_heartbeats}"
        );
    }

    #[test]
    fn nonfinite_record_is_skipped_without_killing_the_service() {
        let dir = temp_dir("nonfinite");
        let (tx, rx) = sync_channel(16);
        let handle = spawn(cfg(&dir, false, 1 << 20), rx);
        let mut bad = trapq_record(1);
        if let WalRecord::TrapqSegment(seg) = &mut bad {
            seg.acceleration = f64::NAN;
        }
        send(
            &tx,
            WalCmd::Append {
                record: bad,
                sync: SyncPolicy::Immediate,
            },
        );
        let good = trapq_record(2);
        send(
            &tx,
            WalCmd::Append {
                record: good.clone(),
                sync: SyncPolicy::Immediate,
            },
        );
        send(&tx, WalCmd::Shutdown);
        handle.join().unwrap().unwrap();
        let result = scan_segment(&dir, 1);
        assert_eq!(result.end, ScanEnd::CleanEof);
        let records: Vec<WalRecord> = result.records.into_iter().map(|r| r.record).collect();
        assert_eq!(records, vec![good]);
    }

    #[test]
    fn crash_writer_rejects_unusable_directory() {
        // The kill-loop proper lives in tests/crash_consistency.rs;
        // here, only the error path of the entry point.
        let dir = temp_dir("crash-smoke");
        let bogus = dir.join("segments-as-file");
        std::fs::write(&bogus, b"not a dir").unwrap();
        assert_eq!(crash_writer_main(&bogus), 1);
    }
}
