//! Conversion of classified Klipper messages into WAL records.
//!
//! # Record mapping
//!
//! | Klipper source                              | WAL record       | Durability |
//! |---------------------------------------------|------------------|------------|
//! | `dump_trapq` batch row (per move)           | `TrapqSegment`   | batched    |
//! | `dump_stepper` batch (whole batch)          | `StepperRange`   | batched    |
//! | status delta with a *meaningful* change     | `Context`        | immediate  |
//! | `toolhead.estimated_print_time` + eventtime | heartbeat sample | (10 Hz file, see `walsvc`) |
//! | `mcu.last_stats.receive_seq` (widened)      | sidecar file     | own file, synced on advance |
//! | lifecycle events (socket, print end)        | `Marker`         | immediate  |
//! | `exclude_object` status delta               | `Context.exclude` | immediate |
//! | `idle_timeout`, `probe`                     | observed only — no `Context` field today; subscribed for forward compatibility |
//!
//! # What counts as a meaningful change (Context triggers)
//!
//! * `virtual_sdcard.file_position` advance — throttled to at most one
//!   position-only context per [`POSITION_CONTEXT_MIN_NS`] (position
//!   advances arrive with every 0.25 s status refresh during a print;
//!   journaling each would double the immediate-fsync load while adding
//!   nothing reconstruction can use, since motion records already
//!   interpolate between contexts).
//! * `virtual_sdcard.file_path` change (load/unload).
//! * G-code *state* change: `speed_factor`, `extrude_factor`,
//!   `absolute_coordinates`, `absolute_extrude`, `homing_origin`.
//!   Position/speed fields change with every move and are captured en
//!   passant, not triggers.
//! * Heater target or fan speed change (setpoints, exact comparison).
//! * Transform change: bed-mesh activation/profile, `z_thermal_adjust`
//!   enable flag, skew profile. The continuously-drifting
//!   `z_thermal_adjust.current_z_adjust` is merged silently and rides
//!   along on the next context.
//! * **Exclude-object change**: the first `exclude_object` observation,
//!   any change to `objects` (a new `EXCLUDE_OBJECT_DEFINE`, an
//!   `EXCLUDE_OBJECT_START` auto-definition, or the wholesale clear of
//!   `EXCLUDE_OBJECT_DEFINE RESET=1`) and any change to
//!   `excluded_objects` (a cancellation, an `EXCLUDE_OBJECT RESET=1`,
//!   or an individual un-exclude). See the dedicated section below.
//!
//! # Exclude-object trigger rules
//!
//! An operator usually cancels an object *because it failed* — it
//! detached, warped, or turned into spaghetti. Klipper holds that
//! decision only in RAM, so it must reach the disk before the power
//! does: an exclusion change forces an **immediate** (fsync'd)
//! `Context` rather than waiting for the batch window, and bypasses
//! [`POSITION_CONTEXT_MIN_NS`], which throttles position-only contexts.
//!
//! What that does **not** buy is undroppability. Records — contexts
//! included — are dropped when the WAL channel fills, because blocking
//! the socket reader gets the daemon disconnected by Klipper
//! (`crate::sender`); only markers are never dropped. The guarantee is
//! therefore: *if* an exclusion-bearing context reaches the WAL thread
//! it is fsync'd before the next record, and *if* it does not, the
//! sender journals a never-dropped
//! [`plr_wal::MarkerKind::ExclusionUpdateLost`] marker so
//! reconstruction knows to distrust the surviving set. Recovery never
//! silently loses a cancellation; at worst it has to ask.
//!
//! # Positive journaling
//!
//! The excluded set is journaled from the **first** `exclude_object`
//! observation onward, including when it is empty. "Zero objects
//! excluded as of t" is a recorded fact, so reconstruction can tell
//! *nothing was cancelled* from *we never looked* — the distinction the
//! whole provenance machinery in `plr-reconstruct` rests on.
//!
//! * `objects` change → immediate context carrying
//!   [`ExcludeState::definitions`] `= Some(..)`.
//! * `excluded_objects` change → immediate context. The excluded set is
//!   short, so it rides in **every** context that carries exclude state
//!   (`definitions` does not — see the payload-size rationale on
//!   [`ExcludeState`]).
//! * `current_object` change is **merged silently and never triggers**:
//!   `EXCLUDE_OBJECT_START`/`END` bracket every object on every layer,
//!   so triggering on it would turn an N-object, M-layer print into
//!   N×M immediate fsyncs while adding nothing recovery needs. It rides
//!   along on the next context like `current_z_adjust`.
//! * [`Recorder::reset_session`] drops the snapshot, so the next
//!   session's initial full status re-journals definitions — Klipper
//!   resets `exclude_object` on restart, and stale definitions must not
//!   outlive it.
//!
//! Geometry is normalized on the way in by
//! [`ExcludeObjectDef::normalized`]: non-finite or malformed outlines
//! become `PolygonFidelity::Unusable` and over-long ones become their
//! bounding box, so a hostile polygon can never make the whole context
//! non-finite and cost us the excluded set.
//!
//! # `print_stats`: journaled, and also a clean-shutdown signal
//!
//! `print_stats.state` is recorded verbatim into every context
//! ([`plr_wal::Context::print_state`]) and a change to it triggers an
//! immediate context, because it is the printer's authoritative print
//! state machine and it changes a handful of times per print, not per
//! move. Recording the state itself is what lets recovery tell "the
//! operator ended this print" from "the machine died" without inferring
//! it from `is_active` edges or byte ratios.
//!
//! The same merge also drives signal 1 below. Both matter: the journaled
//! field is *evidence* a later `plrd detect` reads directly, while the
//! `CleanShutdown` marker is the *control signal* that stops
//! `plr_reconstruct::reconstruct` producing a recovery at all. Either one
//! surviving alone is enough to avoid a false offer.
//!
//! # Clean-shutdown detection
//!
//! Three independent signals, OR-ed. Any one of them journals a
//! `CleanShutdown` marker; a **pause** trips none of them.
//!
//! 1. **`print_stats.state` left an in-progress state for a finished
//!    one** — `printing`/`paused` → `complete`/`cancelled`/`standby`.
//!    This is the authoritative signal and the only one that is a state
//!    machine rather than an inference. Klipper sets it in
//!    `klippy/extras/print_stats.py`: `reset()` → `"standby"`,
//!    `note_start()` → `"printing"`, `note_pause()` → `"paused"`, and
//!    `_note_finish()` → `"complete"` (from `note_complete`),
//!    `"cancelled"` (from `note_cancel`) or `"error"` (from
//!    `note_error`). `error` is deliberately **not** a finished state
//!    here: a print that died on an error is exactly what recovery is
//!    for.
//! 2. **`virtual_sdcard.file_path` went `Some` → `None`** while a print
//!    was in progress this session. `virtual_sdcard.do_cancel` /
//!    `_reset_file` close the file (`self.current_file = None`), so
//!    `file_path()` starts reporting `null`.
//! 3. **`virtual_sdcard.is_active` went true → false** in an update
//!    that, once merged, shows the reader at or past the end of the file
//!    (`file_position >= file_size`) or no file loaded.
//!
//! ## Why signal 3 alone was not enough
//!
//! `is_active` is `self.work_timer is not None`
//! (`klippy/extras/virtual_sdcard.py`, `is_active`), and `do_cancel`
//! begins with `do_pause()`, whose body is guarded:
//!
//! ```text
//! def do_pause(self):
//!     if self.work_timer is not None:
//!         self.must_pause_work = True
//!         while self.work_timer is not None and not self.cmd_from_sd:
//!             self.reactor.pause(self.reactor.monotonic() + .001)
//! ```
//!
//! * **Pause → cancel** (the default Mainsail/Fluidd button order, the
//!   filament-runout flow, and the post-error flow) already cleared
//!   `work_timer`, so `do_pause()` is a no-op, `is_active` never
//!   changes, and the status **diff carries no `is_active` key at all**.
//!   Signal 3 cannot fire; signals 1 and 2 both do, deterministically.
//! * **Direct cancel while printing** races: the 1 ms `reactor.pause`
//!   loop yields to the reactor, so the status subscription can sample
//!   *between* `work_timer` going `None` and the file closing. That
//!   update carries `is_active: false` with the file still loaded
//!   mid-file — a pause, as far as signal 3 can tell — and the *next*
//!   update carries `file_path: null` with no `is_active` key. Signal 2
//!   catches the second update; signal 1 catches whichever update
//!   carries `print_stats`.
//!
//! ## Why signal 2 needs a session gate
//!
//! `reset_session` exists because a klippy `RESTART` resets
//! `virtual_sdcard` server-side, and diffing the fresh baseline against
//! the stale snapshot would look like a deliberate end. That baseline
//! reports `file_path: null`, which is precisely a `Some` → `None`
//! transition against the pre-restart snapshot. Signal 2 is therefore
//! gated on [`Recorder`]'s `print_in_progress`, set only by a *positive*
//! observation that a print is running (`is_active: true`, or
//! `print_stats.state` in `printing`/`paused`) and cleared by
//! `reset_session`. After a restart the gate is closed, so the killed
//! print still reads as an unclean stop.
//!
//! ## Duplicate markers are expected and harmless
//!
//! The three signals are independent, not prioritized, so one print end
//! can journal the marker **more than once** — most commonly when
//! `file_path: null` and `print_stats.state == "cancelled"` arrive in
//! separate status updates, tripping signals 2 and 1 in turn. That is
//! deliberate: suppressing the second would mean picking a "primary"
//! signal, and the whole point is that no single signal is reliable.
//!
//! Duplicates cost one extra 40-odd-byte marker record and change no
//! reader's answer: `plr_reconstruct::timeline::ingest` keeps the *index*
//! of the newest `CleanShutdown` marker (`clean_marker_idx = Some(idx)`,
//! last write wins) and only calls the log clean when no motion record
//! follows it, so N markers in a row decide exactly what one would.

use std::collections::BTreeMap;

use plr_klipper::{
    ClockCorrelator, ExcludeObjectDefinition, ExcludeObjectSnapshot, GcodeMoveStatus, Notification,
    ReceiveSeqWidener, ResponseTemplate, SampleOutcome, SeqKind, StatusUpdate, StepperBatch,
    TrapqBatch, VirtualSdcardStatus,
};
use plr_reconstruct::ReconstructConfig;
use plr_wal::{
    Context, ExcludeObjectDef, ExcludeState, FanTarget, GcodeState, HeaterTarget, MarkerKind,
    StepChunk, StepperRange, TransformObservations, TrapqSegment, VirtualSdState, WalRecord,
};
use serde_json::{Map, Value};

use crate::config::Config;
use crate::sender::{HeartbeatData, SyncPolicy};

/// The response-template key used to demultiplex subscriptions. Klipper
/// echoes the template at the top level of every asynchronous message and
/// does **not** name the producing endpoint, so every subscription gets a
/// distinct value under this key.
pub const TEMPLATE_KEY: &str = "k";

/// Minimum spacing of contexts triggered *only* by file-position advance.
pub const POSITION_CONTEXT_MIN_NS: u64 = 1_000_000_000;

/// The **reader's** conservative expected spacing basis for WAL heartbeat
/// *records*, as a divisor of the heartbeat-file rate: one record per this
/// many file beats (10 Hz / 10 = 1 Hz). Consumed by [`reconstruct_config`]
/// to set `heartbeat_period_ns`, from which recovery derives its
/// coverage-gap tolerance.
///
/// # Why this is the reader basis, and stays 1 Hz even though the writer
/// now emits faster
///
/// The *writer's* active cadence is [`WAL_HEARTBEAT_ACTIVE_EVERY`] (every
/// file beat, 10 Hz — see there for why). The reader, however, must
/// tolerate the **sparsest** active stream it can be handed, which is the
/// pre-throttle 1 Hz WAL a printer running an older `plrd` wrote (and
/// older segments in the same directory). Deriving the gap tolerance from
/// 1 Hz keeps every such stream reading as continuous; a denser 10 Hz
/// stream is a fortiori continuous under it. Tightening the basis to
/// 10 Hz would make every pre-throttle WAL read as a chain of holes — the
/// exact false-positive [`crate::convert`] warns about — for a coverage
/// check that idle spacing never even enters. So this is deliberately the
/// loose, backward-compatible value.
///
/// It lives here rather than in `walsvc` because it is a contract between
/// the writer and the reader, and `walsvc` is Linux-only while this module
/// is not.
pub const WAL_HEARTBEAT_EVERY: u64 = 10;

/// The **writer's** active-regime cadence: append a WAL heartbeat *record*
/// every heartbeat-file beat (one-in-one → the full 10 Hz file rate) while
/// a print is in progress or motion is recent.
///
/// # Why the full rate while printing
///
/// The stop window's lower bound `t_a` is the newest heartbeat's print
/// time; a sparse record stream lets `t_a` lag the true power cut, and
/// measured on real crash WALs that lag was a major contributor to
/// 2.5–4.0 s stop windows, which multiply through XY/E evidence into
/// hundreds of match candidates and force a manual recovery. Denser
/// heartbeat *records* pin `t_a` to within ~100 ms of the cut. The byte
/// cost is noise during printing — motion records already fill a 16 MiB
/// segment every 3–5 minutes, against which 10 Hz heartbeats add a few
/// percent — whereas the same records were the *dominant* cost while idle,
/// which is exactly what [`WAL_HEARTBEAT_QUIET_EVERY`] throttles. Same
/// mechanism, opposite dial at each end.
///
/// Not a reader contract (denser-than-expected is always safe for the
/// reader): writer-side only, consumed by `walsvc`.
pub const WAL_HEARTBEAT_ACTIVE_EVERY: u64 = 1;

/// Append a WAL `Heartbeat` record only every Nth heartbeat-file rewrite
/// **while the recorder is idle** (no print in progress and no recent
/// motion). At the default 10 Hz file rate this is one WAL heartbeat
/// record every 30 s, versus one per second while active.
///
/// # Why this exists
///
/// The heartbeat *file* (128 B, rewritten in place) does not grow, but a
/// WAL heartbeat *record* is appended to the log at the active cadence
/// regardless of whether anything is printing — ~250 B/s, ~7.8 GB/year on
/// an idle printer, measured at ~16.8 MB per 19.6 h idle segment on the
/// user's machine. Throttling the idle *record* cadence by this factor
/// (30×) removes almost all of that while keeping a coarse liveness trail
/// in the log itself.
///
/// # Why the reader does not need it
///
/// Unlike [`WAL_HEARTBEAT_EVERY`], this is **not** a reader contract:
/// reconstruction's heartbeat-continuity reasoning
/// (`plr_reconstruct::exclude`) only ever runs across a *stop-window
/// coverage span*, which lies inside a recoverable print — and the idle
/// regime is entered only once a print has conclusively ended or when no
/// print is running, so no coverage span ever contains an idle span.
/// A [`plr_wal::MarkerKind::RecordingQuiescent`] marker records the regime
/// change so that invariant is checkable in the log. The value therefore
/// lives writer-side only; `walsvc` reads it (via `WalSvcCfg`).
pub const WAL_HEARTBEAT_QUIET_EVERY: u64 = 300;

/// How long after the last observed motion the recorder keeps full
/// heartbeat cadence when no print is otherwise known to be in progress,
/// in nanoseconds (5 s).
///
/// # Why the data plane, not just `print_stats`
///
/// `print_stats.state` and `virtual_sdcard.is_active` arrive by
/// subscription and lag the start of a print; the opening moments of a
/// print are already the weakest recovery case, so cadence must rise at or
/// before the first instant motion is possible. Motion (`dump_trapq` /
/// `dump_stepper`) data arriving **is** proof motion began, so it is used
/// as an independent trigger: the regime is active whenever a print is in
/// progress (`Recorder::print_in_progress`, which stays set through a
/// print's dwells and pauses) **or** motion arrived within this window.
///
/// The window also bounds the cost of a stray manual jog while idle: after
/// the jog the regime falls back to idle once this much time passes with
/// no further motion, rather than latching active forever. During a real
/// print `print_in_progress` holds the regime active regardless, so this
/// window only governs motion that is *not* part of a file print.
///
/// The residual race it cannot close: between the instant Klipper produces
/// the first move of a print and the instant plrd receives that batch
/// (one dump-batch period, ≤ ~0.5 s), the regime is still idle. During
/// that sub-second window the WAL heartbeat *records* are still sparse —
/// but the heartbeat *file* is unaffected (so `t_a` is unaffected), and no
/// exclusion context (the only thing continuity gates) can exist that
/// early, so nothing downstream is weakened.
pub const IDLE_AFTER_MOTION_NS: u64 = 5_000_000_000;

/// Derives the reconstruction tunables from the daemon's own
/// configuration, so recovery is not left guessing at rates this process
/// already knows.
///
/// Only the heartbeat cadence is derived today; everything else keeps
/// [`ReconstructConfig::default`]. The cadence matters because
/// `plr-reconstruct` uses heartbeat *continuity* to tell a long dwell
/// (the writer was alive and journaled no object cancellation — nothing
/// to confirm) from a stalled recorder (a cancellation may be missing).
/// Get the period wrong and an on-time stream reads as a chain of holes,
/// which makes every recovery of a plate with objects prompt the
/// operator.
///
/// The period is the **WAL** heartbeat period, not the heartbeat-file
/// period: `walsvc` rewrites the file at `heartbeat_hz` but appends a
/// `Heartbeat` record only every [`WAL_HEARTBEAT_EVERY`]-th tick, so
///
/// ```text
/// period = WAL_HEARTBEAT_EVERY / heartbeat_hz
/// ```
///
/// `None` is for entry points that run without a loaded config (the
/// forensic `scan` subcommand, and `detect`, which classifies the
/// previous session and does not read the exclusion report); they get
/// the documented defaults.
#[must_use]
pub fn reconstruct_config(config: Option<&Config>) -> ReconstructConfig {
    let defaults = ReconstructConfig::default();
    let Some(config) = config else {
        return defaults;
    };
    // `Config::validate` guarantees heartbeat_hz is finite and > 0, but
    // this must stay total for a hand-built Config: fall back rather
    // than produce a nonsense period.
    #[allow(clippy::cast_precision_loss)]
    let period_ns = WAL_HEARTBEAT_EVERY as f64 / config.heartbeat_hz * 1e9;
    #[allow(clippy::cast_precision_loss)]
    let max = u64::MAX as f64;
    if !period_ns.is_finite() || period_ns < 1.0 || period_ns >= max {
        return defaults;
    }
    ReconstructConfig {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        heartbeat_period_ns: period_ns as u64,
        ..defaults
    }
}

/// Which subscription a notification belongs to, decoded from the echoed
/// response template.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Route {
    /// The `objects/subscribe` status stream.
    Status,
    /// A `motion_report/dump_trapq` stream for the named queue.
    Trapq(String),
    /// A `motion_report/dump_stepper` stream for the named stepper.
    Stepper(String),
}

/// Template for the status subscription: `{"k": "status"}`.
#[must_use]
pub fn status_template() -> ResponseTemplate {
    template("status")
}

/// Template for a trapq dump: `{"k": "trapq:<name>"}`.
#[must_use]
pub fn trapq_template(name: &str) -> ResponseTemplate {
    template(&format!("trapq:{name}"))
}

/// Template for a stepper dump: `{"k": "stepper:<name>"}`.
#[must_use]
pub fn stepper_template(name: &str) -> ResponseTemplate {
    template(&format!("stepper:{name}"))
}

fn template(value: &str) -> ResponseTemplate {
    let mut t = ResponseTemplate::new();
    t.insert(TEMPLATE_KEY.to_owned(), Value::String(value.to_owned()));
    t
}

/// Decodes the routing key from an echoed template. `None` for messages
/// this daemon did not subscribe to (ignored upstream).
#[must_use]
pub fn route_of(template: &Map<String, Value>) -> Option<Route> {
    let key = template.get(TEMPLATE_KEY)?.as_str()?;
    if key == "status" {
        return Some(Route::Status);
    }
    if let Some(name) = key.strip_prefix("trapq:") {
        return Some(Route::Trapq(name.to_owned()));
    }
    if let Some(name) = key.strip_prefix("stepper:") {
        return Some(Route::Stepper(name.to_owned()));
    }
    None
}

/// Everything one inbound message produced.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Output {
    /// Records to append, in order, with their durability.
    pub records: Vec<(WalRecord, SyncPolicy)>,
    /// Refreshed heartbeat payload, when the message updated it.
    pub heartbeat: Option<HeartbeatData>,
    /// A widened `receive_seq` observation `(mono_ns, widened)` to
    /// persist, when the counter advanced.
    pub receive_seq: Option<(u64, u64)>,
    /// The print ended on purpose (complete or cancelled); the caller
    /// journals a `CleanShutdown` marker.
    pub clean_shutdown: bool,
    /// A heartbeat-cadence regime change the caller must journal as a
    /// marker. `Some(RecordingQuiescent)` when the recorder has newly
    /// entered the idle regime — an active → idle edge, or the first
    /// evaluation of a session that starts idle (the idle-from-birth case);
    /// `None` otherwise. Never dropped — it is the recorded fact that
    /// explains the sparse heartbeat stream that follows (see
    /// [`plr_wal::MarkerKind::RecordingQuiescent`]).
    pub regime_marker: Option<MarkerKind>,
}

/// Stateful converter: merges Klipper's diff-style status stream into a
/// full snapshot and emits WAL records per the module-level mapping.
///
/// Pure logic — no I/O, no clocks. The caller supplies `mono_ns`
/// (host `CLOCK_MONOTONIC`, the same clock Klipper's reactor `eventtime`
/// runs on) with every message.
// Four bools, each a distinct and independent observation about the live
// session (is the reader active, is a print in progress, has
// `exclude_object` ever been seen, do its definitions still need
// journaling). They are not a state enum in disguise: every one of the 16
// combinations is reachable, and folding them into flag structs would
// only add indirection to a hot-path merge.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug)]
pub struct Recorder {
    correlator: ClockCorrelator,
    widener: ReceiveSeqWidener,
    heater_names: Vec<String>,
    fan_names: Vec<String>,
    gcode: Option<GcodeState>,
    file_path: Option<String>,
    file_position: u64,
    file_size: u64,
    is_active: bool,
    /// Newest `print_stats.state`, verbatim, for
    /// [`plr_wal::Context::print_state`]. `None` before the first
    /// observation of the session — which is exactly "not observed".
    print_state: Option<String>,
    /// Newest `print_stats.info.current_layer` / `.total_layer`, the
    /// slicer-fed layer marks, for [`plr_wal::Context::current_layer`] and
    /// [`Context::total_layer`]. `None` = **not observed**: no
    /// `SET_PRINT_STATS_INFO` from the slicer yet this session, or Klipper
    /// reported `info` with null layers (`total_layer == 0`). Updated only
    /// when a diff carries `print_stats.info` (a complete dict — see
    /// [`PrintStatsStatus::info`]); absence of `info` leaves them intact.
    /// Cleared by [`Self::reset_session`] for the same reason
    /// [`Self::print_state`] is: the marks belong to the dead klippy
    /// instance.
    current_layer: Option<u32>,
    total_layer: Option<u32>,
    /// Whether a print has been *positively* observed running this
    /// session — `virtual_sdcard.is_active == true`, or
    /// `print_stats.state` in `printing`/`paused`.
    ///
    /// This is the "was there a print to end?" gate for both the
    /// `print_stats` transition (signal 1) and the `file_path`
    /// `Some` → `None` transition (signal 2), so a klippy restart cannot
    /// forge either — see the module-level "Why signal 2 needs a session
    /// gate". Cleared by `reset_session` and by a print end.
    print_in_progress: bool,
    transforms: TransformObservations,
    heaters: BTreeMap<String, f64>,
    fans: BTreeMap<String, f64>,
    latest_print_time: f64,
    /// Newest `toolhead.print_time` verbatim — the **trapq append
    /// frontier** — for [`plr_wal::Context::print_time`].
    ///
    /// Deliberately **not** [`Self::latest_print_time`], which is a `max`
    /// across `toolhead.print_time`, *both* trapq queues' row end times
    /// and every stepper batch's `last_step_time`. That conflation is
    /// correct for the heartbeat's "newest motion we know of" but wrong
    /// here: a reader compares this value against *per-queue* durable
    /// coverage to certify that coverage, and a max polluted by the
    /// extruder queue's own rows would let the certificate pass using the
    /// very rows it is supposed to be testing. Over-claiming coverage is
    /// the containment-unsafe direction, so this stays separate and
    /// verbatim.
    ///
    /// `None` until a status update carries `toolhead.print_time`.
    toolhead_print_time: Option<f64>,
    est_sample: Option<(u64, f64)>,
    /// Host-monotonic time (ns) of the newest motion record produced this
    /// session (a non-empty trapq batch or any stepper batch), or `None`
    /// if none yet. Feeds the heartbeat-cadence regime: motion arriving is
    /// proof a print (or a manual move) is under way, independent of the
    /// lagging status plane. See [`IDLE_AFTER_MOTION_NS`].
    last_motion_mono_ns: Option<u64>,
    /// The heartbeat-cadence regime as of the last handled message: `true`
    /// = full cadence (a print is in progress or motion is recent), `false`
    /// = idle/throttled. Held so the active → idle edge can be detected and
    /// journaled as a [`plr_wal::MarkerKind::RecordingQuiescent`] marker.
    /// Starts `false` (assume idle until a positive signal), the
    /// conservative direction: a print running at daemon start raises it on
    /// the first status/motion.
    recording_active: bool,
    /// Whether the regime has been evaluated at least once this session.
    /// `false` at construction and after [`Self::reset_session`]. The first
    /// evaluation of a session journals a `RecordingQuiescent` marker when
    /// it finds the recorder idle, not only on a later active → idle *edge*
    /// — otherwise a daemon that starts on an idle printer (the commonest
    /// shape: the real capture's segment 1 is 19.6 h of exactly this) would
    /// write a sparse heartbeat stream with no marker anywhere in the
    /// session, leaving the sparseness an ambiguous absence rather than the
    /// recorded fact this fix exists to make it.
    regime_evaluated: bool,
    last_context_mono_ns: Option<u64>,
    last_context_file_position: u64,
    exclude: ExcludeObjectSnapshot,
    /// Whether `exclude_object` has ever been observed this session.
    /// Gates the whole `Context.exclude` field: `None` must keep meaning
    /// "not observed", never "nothing excluded".
    exclude_seen: bool,
    /// Whether the current definition list still needs journaling.
    exclude_definitions_dirty: bool,
}

impl Default for Recorder {
    fn default() -> Self {
        Self::new()
    }
}

impl Recorder {
    /// A recorder with no snapshot yet. Heater/fan names default to the
    /// standard `fan` object only; set the real list after querying
    /// Klipper's `heaters` object.
    #[must_use]
    pub fn new() -> Self {
        Self {
            correlator: ClockCorrelator::new(),
            widener: ReceiveSeqWidener::new(),
            heater_names: Vec::new(),
            fan_names: vec!["fan".to_owned()],
            gcode: None,
            file_path: None,
            file_position: 0,
            file_size: 0,
            is_active: false,
            print_state: None,
            current_layer: None,
            total_layer: None,
            print_in_progress: false,
            transforms: TransformObservations {
                bed_mesh_active: false,
                bed_mesh_profile: None,
                z_thermal_adjust_enabled: None,
                z_thermal_adjust_offset: None,
                skew_active: false,
                skew_profile: None,
            },
            heaters: BTreeMap::new(),
            fans: BTreeMap::new(),
            latest_print_time: 0.0,
            toolhead_print_time: None,
            est_sample: None,
            last_motion_mono_ns: None,
            recording_active: false,
            regime_evaluated: false,
            last_context_mono_ns: None,
            last_context_file_position: 0,
            exclude: ExcludeObjectSnapshot::new(),
            exclude_seen: false,
            exclude_definitions_dirty: false,
        }
    }

    /// Sets the heater object names discovered from the `heaters` status
    /// object (e.g. `["extruder", "heater_bed"]`).
    pub fn set_heater_names(&mut self, names: Vec<String>) {
        self.heater_names = names;
    }

    /// Forgets session-scoped state after a socket loss.
    ///
    /// The clean-shutdown detector keys on an `is_active` true→false
    /// *transition*, which is only meaningful within one uninterrupted
    /// subscription stream: a klippy RESTART mid-print resets
    /// `virtual_sdcard` server-side, and diffing the fresh baseline
    /// against the stale snapshot would journal a false `CleanShutdown`
    /// for a print the restart killed. Likewise the correlation sample
    /// and motion frontier belong to the old klippy instance (a restart
    /// resets `print_time`), so heartbeats must not resume from them —
    /// they stay cleared until the new session's status provides fresh
    /// values. Merged *configuration-ish* state (heater targets,
    /// transforms) is kept: the initial full status of the next session
    /// overwrites it wholesale anyway.
    ///
    /// `exclude_object` state is dropped rather than kept: Klipper
    /// clears it on restart (`_reset_state`) and on
    /// `virtual_sdcard:reset_file`, so carrying the old session's
    /// cancellations forward would journal exclusions the printer no
    /// longer honours. Clearing forces the next session's initial full
    /// status to re-journal definitions and the excluded set from
    /// scratch.
    /// `print_stats` state is dropped for the same reason
    /// `virtual_sdcard` state is: Klipper's copy belongs to the klippy
    /// instance that died. Dropping it also closes the
    /// `print_in_progress` gate, which is what stops the next session's
    /// fresh (empty) `virtual_sdcard` baseline from reading as a
    /// deliberate cancel.
    pub fn reset_session(&mut self) {
        self.is_active = false;
        self.print_state = None;
        // The layer marks belong to the dead klippy instance's print, same
        // as `print_state`; drop them so the next session re-observes from
        // its own `SET_PRINT_STATS_INFO` rather than carrying a stale layer
        // forward into a different (or absent) print.
        self.current_layer = None;
        self.total_layer = None;
        self.print_in_progress = false;
        self.est_sample = None;
        self.latest_print_time = 0.0;
        // Same reason as `latest_print_time`: a RESTART resets Klipper's
        // print-time axis, so the old instance's append frontier must not
        // be journaled against the new instance's trapq rows. A stale-high
        // value would certify durable coverage that does not exist —
        // the containment-unsafe direction.
        self.toolhead_print_time = None;
        // The killed session's motion belongs to the dead klippy instance;
        // a stale value must not hold the new session's regime active. The
        // regime falls back to idle until the new session provides fresh
        // motion or a printing status. `regime_evaluated` is cleared too, so
        // the reconnected session is treated like a fresh start: if it comes
        // back to an idle printer, its first evaluation journals a
        // `RecordingQuiescent` (bounding the sparse stream that follows),
        // and if it comes back mid-print, the first active evaluation simply
        // resumes dense heartbeats with no marker.
        self.last_motion_mono_ns = None;
        self.recording_active = false;
        self.regime_evaluated = false;
        self.exclude = ExcludeObjectSnapshot::new();
        self.exclude_seen = false;
        self.exclude_definitions_dirty = false;
    }

    /// Handles one routed notification. Payloads that fail to parse are
    /// reported as `Err`; the caller logs and continues (a malformed
    /// message must not kill the recording session).
    pub fn on_notification(
        &mut self,
        route: &Route,
        notification: &Notification,
        mono_ns: u64,
    ) -> Result<Output, plr_klipper::MessageError> {
        match route {
            Route::Status => Ok(self.on_status(&notification.status_update()?, mono_ns, false)),
            Route::Trapq(name) => Ok(self.on_trapq(name, &notification.trapq_batch()?, mono_ns)),
            Route::Stepper(name) => {
                Ok(self.on_stepper(name, &notification.stepper_batch()?, mono_ns))
            }
        }
    }

    /// Handles the *initial* full status (the `objects/subscribe`
    /// response body): merges it and force-emits a baseline context.
    pub fn on_initial_status(
        &mut self,
        result: &Value,
        mono_ns: u64,
    ) -> Result<Output, serde_json::Error> {
        let update: StatusUpdate = serde_json::from_value(result.clone())?;
        Ok(self.on_status(&update, mono_ns, true))
    }

    /// Merges a status update and emits records per the module-level
    /// trigger rules.
    pub fn on_status(
        &mut self,
        update: &StatusUpdate,
        mono_ns: u64,
        force_context: bool,
    ) -> Output {
        let mut out = Output::default();
        let mut state_changed = force_context;

        self.merge_toolhead(update);
        out.receive_seq = self.observe_receive_seq(update, mono_ns);
        if let Ok(Some(gm)) = update.status.gcode_move() {
            state_changed |= self.merge_gcode(&gm);
        }
        // `print_stats` is merged BEFORE `virtual_sdcard` so that a
        // single update carrying both (the normal shape of a cancel)
        // opens the `print_in_progress` gate from the authoritative
        // signal before the `file_path` signal consults it.
        let print_stats = self.merge_print_stats(update);
        out.clean_shutdown = print_stats.clean_shutdown;
        // A print-state change is journaled immediately: it is the whole
        // point of recording the field, and it happens a handful of times
        // per print, not per move. A layer-mark change is journaled the same
        // way and for the same reason.
        state_changed |= print_stats.state_changed;
        state_changed |= print_stats.layer_changed;
        let mut position_advanced = false;
        if let Ok(Some(vsd)) = update.status.virtual_sdcard() {
            let vsd_result = self.merge_virtual_sdcard(&vsd);
            state_changed |= vsd_result.path_changed;
            position_advanced = vsd_result.position_advanced;
            out.clean_shutdown |= vsd_result.clean_shutdown;
        }
        state_changed |= self.merge_heaters_and_fans(update);
        state_changed |= self.merge_transforms(update);
        state_changed |= self.merge_exclude_object(update);

        let position_due = self
            .last_context_mono_ns
            .is_none_or(|last| mono_ns.saturating_sub(last) >= POSITION_CONTEXT_MIN_NS);
        let position_trigger = position_advanced
            && self.file_position != self.last_context_file_position
            && position_due;
        if state_changed || position_trigger {
            if let Some(context) = self.build_context(mono_ns) {
                // Definitions are journaled once per change: clear the
                // dirty flag only when this context actually carried
                // them (build_context returns `None` before a full
                // g-code state exists, and then nothing was written).
                let carried_definitions = context
                    .exclude
                    .as_ref()
                    .is_some_and(|state| state.definitions.is_some());
                out.records
                    .push((WalRecord::Context(context), SyncPolicy::Immediate));
                self.last_context_mono_ns = Some(mono_ns);
                self.last_context_file_position = self.file_position;
                self.exclude_definitions_dirty &= !carried_definitions;
            }
        }
        out.regime_marker = self.note_regime(mono_ns);
        out.heartbeat = self.heartbeat_data(mono_ns);
        out
    }

    /// Converts a trapq batch: one `TrapqSegment` per move.
    pub fn on_trapq(&mut self, queue: &str, batch: &TrapqBatch, mono_ns: u64) -> Output {
        let mut out = Output::default();
        for m in &batch.data {
            let segment = TrapqSegment {
                mono_ns,
                queue: queue.to_owned(),
                print_time: m.time,
                duration: m.duration,
                start_velocity: m.start_velocity,
                acceleration: m.acceleration,
                start_x: m.start_position[0],
                start_y: m.start_position[1],
                start_z: m.start_position[2],
                x_r: m.direction[0],
                y_r: m.direction[1],
                z_r: m.direction[2],
            };
            let end = segment.end_time();
            if end.is_finite() {
                self.latest_print_time = self.latest_print_time.max(end);
            }
            out.records
                .push((WalRecord::TrapqSegment(segment), SyncPolicy::Batched));
        }
        // Motion arriving is the data-plane proof that a print (or a
        // manual move) is under way; it raises the heartbeat regime at or
        // before the status plane can. Only a batch that actually carried
        // moves counts — an empty dump batch is not motion.
        if !batch.data.is_empty() {
            self.last_motion_mono_ns = Some(mono_ns);
        }
        out.regime_marker = self.note_regime(mono_ns);
        out.heartbeat = self.heartbeat_data(mono_ns);
        out
    }

    /// Converts a stepper batch into one `StepperRange`.
    pub fn on_stepper(&mut self, stepper: &str, batch: &StepperBatch, mono_ns: u64) -> Output {
        let range = StepperRange {
            mono_ns,
            stepper: stepper.to_owned(),
            first_clock: batch.first_clock,
            last_clock: batch.last_clock,
            first_step_time: batch.first_step_time,
            last_step_time: batch.last_step_time,
            start_position: batch.start_position,
            start_mcu_position: batch.start_mcu_position,
            step_distance: batch.step_distance,
            steps: batch.data.iter().map(step_chunk).collect(),
        };
        if range.last_step_time.is_finite() {
            self.latest_print_time = self.latest_print_time.max(range.last_step_time);
        }
        let mut out = Output {
            records: vec![(WalRecord::StepperRange(range), SyncPolicy::Batched)],
            ..Output::default()
        };
        // A stepper dump is always committed motion (one range per batch),
        // so it always raises the regime — the data-plane trigger.
        self.last_motion_mono_ns = Some(mono_ns);
        out.regime_marker = self.note_regime(mono_ns);
        out.heartbeat = self.heartbeat_data(mono_ns);
        out
    }

    /// The current heartbeat payload; `None` until a correlation sample
    /// (`estimated_print_time` + `eventtime`) has been observed — no
    /// liveness claim without one.
    ///
    /// `mono_ns` is the capture time of the message being handled; it sets
    /// the [`HeartbeatData::active`] cadence regime (see
    /// [`Self::regime_active`]).
    #[must_use]
    pub fn heartbeat_data(&self, mono_ns: u64) -> Option<HeartbeatData> {
        let active = self.regime_active(mono_ns);
        self.est_sample.map(
            |(est_sample_mono_ns, est_sample_print_time)| HeartbeatData {
                print_time: self.latest_print_time,
                est_sample_mono_ns,
                est_sample_print_time,
                active,
            },
        )
    }

    /// The heartbeat-cadence regime at `mono_ns`: `true` (full cadence)
    /// while a print is in progress **or** motion arrived within
    /// [`IDLE_AFTER_MOTION_NS`], `false` (idle/throttled) otherwise.
    ///
    /// Uses the data plane (recent motion) as well as the status plane
    /// (`print_in_progress`) so cadence rises at the first instant motion
    /// is possible, not when the lagging `print_stats`/`is_active` status
    /// finally arrives — see [`IDLE_AFTER_MOTION_NS`].
    ///
    /// # An errored print holds full cadence until reset
    ///
    /// `print_in_progress` is cleared only by a *positively finished* state
    /// (`complete`/`cancelled`/`standby`) or by `reset_session`, never by
    /// `error` — an errored print is exactly what recovery exists for, so it
    /// must stay recoverable. A printer left sitting in `error` therefore
    /// keeps recording at the full 10 Hz rate (the pre-fix byte rate) until
    /// the operator resets it. This is deliberate and conservative; the
    /// ~8 B/s idle claim is for a genuinely idle (`standby`) printer, not an
    /// errored one.
    #[must_use]
    pub fn regime_active(&self, mono_ns: u64) -> bool {
        if self.print_in_progress {
            return true;
        }
        self.last_motion_mono_ns
            .is_some_and(|last| mono_ns.saturating_sub(last) < IDLE_AFTER_MOTION_NS)
    }

    /// Updates the stored regime for `mono_ns` and returns the marker the
    /// caller must journal when the recorder is now idle *and* that idleness
    /// is newly established — either the active → idle edge, or the first
    /// evaluation of a session that starts idle.
    ///
    /// Marking the active → idle edge explains a sparse heartbeat stream
    /// that would otherwise read as a stalled recorder. Marking the *first*
    /// idle evaluation covers the idle-from-birth session (a daemon that
    /// starts on an idle printer): without it that session would carry a
    /// throttled stream with no marker anywhere, an ambiguous absence rather
    /// than a recorded fact. The idle → active transition needs no marker —
    /// the resumed dense heartbeat stream (whose first record `walsvc`
    /// forces at once) is itself the liveness proof, and it bounds the quiet
    /// span.
    fn note_regime(&mut self, mono_ns: u64) -> Option<MarkerKind> {
        let now_active = self.regime_active(mono_ns);
        let first_evaluation = !self.regime_evaluated;
        let was_active = self.recording_active;
        self.regime_evaluated = true;
        self.recording_active = now_active;
        // Idle now, and that idleness is new: a fall from active, or a
        // session whose very first evaluation is idle.
        (!now_active && (was_active || first_evaluation)).then_some(MarkerKind::RecordingQuiescent)
    }

    fn merge_toolhead(&mut self, update: &StatusUpdate) {
        let Ok(Some(th)) = update.status.toolhead() else {
            return;
        };
        if let Some(est) = th.estimated_print_time {
            if self.correlator.add_sample(update.eventtime, est) == SampleOutcome::Accepted {
                if let Some(ns) = eventtime_to_ns(update.eventtime) {
                    self.est_sample = Some((ns, est));
                }
            }
        }
        if let Some(pt) = th.print_time {
            if pt.is_finite() {
                self.latest_print_time = self.latest_print_time.max(pt);
                // Verbatim, and NOT a running max: a klippy restart resets
                // the print-time axis, and a stale-high value would let a
                // reader certify coverage it does not have. Klipper's own
                // `self.print_time` is monotone within one klippy
                // instance, so tracking the newest observation is faithful
                // to the source; `reset_session` clears it at the
                // instance boundary.
                self.toolhead_print_time = Some(pt);
            }
        }
    }

    fn observe_receive_seq(&mut self, update: &StatusUpdate, mono_ns: u64) -> Option<(u64, u64)> {
        let mcu = update.status.mcu().ok().flatten()?;
        let raw = mcu.last_stats.as_ref()?.receive_seq?;
        let seq = self.widener.observe(raw);
        match seq.kind {
            SeqKind::First | SeqKind::Advanced { .. } => Some((mono_ns, seq.widened)),
            SeqKind::Unchanged | SeqKind::Regressed { .. } => None,
        }
    }

    /// Merges a `gcode_move` diff; `true` when a trigger field changed.
    // Setpoints and mode flags come verbatim from Klipper's JSON; exact
    // float equality is the correct change test for them.
    #[allow(clippy::float_cmp)]
    fn merge_gcode(&mut self, gm: &GcodeMoveStatus) -> bool {
        let Some(g) = self.gcode.as_mut() else {
            self.gcode = full_gcode_state(gm);
            return self.gcode.is_some();
        };
        let mut trigger = false;
        if let Some(v) = gm.speed_factor {
            trigger |= v != g.speed_factor;
            g.speed_factor = v;
        }
        if let Some(v) = gm.extrude_factor {
            trigger |= v != g.extrude_factor;
            g.extrude_factor = v;
        }
        if let Some(v) = gm.absolute_coordinates {
            trigger |= v != g.absolute_coordinates;
            g.absolute_coordinates = v;
        }
        if let Some(v) = gm.absolute_extrude {
            trigger |= v != g.absolute_extrude;
            g.absolute_extrude = v;
        }
        if let Some(v) = &gm.homing_origin {
            trigger |= *v != g.homing_origin;
            g.homing_origin.clone_from(v);
        }
        // Captured but not triggers (change with every move):
        if let Some(v) = gm.speed {
            g.speed = v;
        }
        if let Some(v) = &gm.position {
            g.position.clone_from(v);
        }
        if let Some(v) = &gm.gcode_position {
            g.gcode_position.clone_from(v);
        }
        trigger
    }

    /// Merges `print_stats.state` and reports whether the transition is a
    /// deliberate print end (see the module-level signal 1).
    ///
    /// Unrecognized state strings are kept as
    /// [`PrintState::Other`]: a future Klipper state must never be
    /// mistaken for a finished one, and must not silently overwrite the
    /// knowledge that a print *was* running either — which is why
    /// `print_in_progress` is only ever cleared by a state that is
    /// positively finished (or by `reset_session`).
    fn merge_print_stats(&mut self, update: &StatusUpdate) -> PrintStatsMerge {
        let mut merge = PrintStatsMerge::default();
        let Ok(Some(stats)) = update.status.get::<PrintStatsStatus>("print_stats") else {
            return merge;
        };
        // Layer marks first, and unconditionally on the state below: a diff
        // can carry `info` with no `state` key (the layer changed but the
        // state did not), so this must run before the `state` early-return.
        // When `info` is present it is the *complete* dict (see
        // [`PrintStatsStatus::info`]); its absence means "unchanged", so the
        // stored marks are kept. A `total_layer == 0` reset arrives as
        // `info` with both fields null, which lands here as `None` — the
        // honest "not observed", not a fabricated layer 0.
        if let Some(info) = stats.info {
            if self.current_layer != info.current_layer || self.total_layer != info.total_layer {
                merge.layer_changed = true;
                self.current_layer = info.current_layer;
                self.total_layer = info.total_layer;
            }
        }
        let Some(reported_text) = stats.state else {
            return merge;
        };
        // The verbatim string is what gets journaled; the parse is only for
        // deciding what the transition means.
        if self.print_state.as_deref() != Some(reported_text.as_str()) {
            merge.state_changed = true;
            self.print_state = Some(reported_text.clone());
        }
        let reported = PrintState::parse(&reported_text);
        if reported.is_in_progress() {
            self.print_in_progress = true;
            return merge;
        }
        // A finished state is only a *transition* if a print was known to
        // be running. Without that, this is just the state of an idle
        // printer the daemon has only now started watching.
        if reported.is_finished() && self.print_in_progress {
            self.print_in_progress = false;
            merge.clean_shutdown = true;
        }
        merge
    }

    fn merge_virtual_sdcard(&mut self, vsd: &VirtualSdcardStatus) -> VsdMerge {
        let mut merge = VsdMerge::default();
        if let Some(path) = &vsd.file_path {
            // Outer Option: present in this diff. Inner: nullable value.
            if *path != self.file_path {
                merge.path_changed = true;
                // Signal 2: the loaded file was closed while a print was
                // known to be running. `do_cancel`/`_reset_file` are the
                // only things that do this, and they are deterministic
                // where the `is_active` edge is not.
                if path.is_none() && self.file_path.is_some() && self.print_in_progress {
                    merge.clean_shutdown = true;
                    self.print_in_progress = false;
                }
                self.file_path.clone_from(path);
            }
        }
        if let Some(pos) = vsd.file_position {
            if pos != self.file_position {
                merge.position_advanced = true;
                self.file_position = pos;
            }
        }
        if let Some(size) = vsd.file_size {
            self.file_size = size;
        }
        if let Some(active) = vsd.is_active {
            if active {
                // A positive observation that a print is running: this is
                // what opens the signal-2 gate on printers whose
                // `print_stats` never reaches us.
                self.print_in_progress = true;
            } else if self.is_active {
                // Signal 3, unchanged: the historical edge test.
                let complete = self.file_size > 0 && self.file_position >= self.file_size;
                let cancelled = self.file_path.is_none();
                merge.clean_shutdown |= complete || cancelled;
            }
            self.is_active = active;
        }
        merge
    }

    // Heater targets and fan speeds are setpoints echoed from Klipper's
    // JSON; exact equality is the correct change test.
    #[allow(clippy::float_cmp)]
    fn merge_heaters_and_fans(&mut self, update: &StatusUpdate) -> bool {
        let mut trigger = false;
        for name in &self.heater_names.clone() {
            if let Ok(Some(h)) = update.status.heater(name) {
                if let Some(target) = h.target {
                    if self.heaters.get(name) != Some(&target) {
                        trigger = true;
                        self.heaters.insert(name.clone(), target);
                    }
                }
            }
        }
        for name in &self.fan_names.clone() {
            if let Ok(Some(f)) = update.status.fan(name) {
                if let Some(speed) = f.speed {
                    if self.fans.get(name) != Some(&speed) {
                        trigger = true;
                        self.fans.insert(name.clone(), speed);
                    }
                }
            }
        }
        trigger
    }

    fn merge_transforms(&mut self, update: &StatusUpdate) -> bool {
        let mut trigger = false;
        if let Ok(Some(bm)) = update.status.bed_mesh() {
            if let Some(active) = bm.mesh_active() {
                trigger |= active != self.transforms.bed_mesh_active;
                self.transforms.bed_mesh_active = active;
            }
            if let Some(profile) = bm.profile_name {
                let profile = non_empty(profile);
                trigger |= profile != self.transforms.bed_mesh_profile;
                self.transforms.bed_mesh_profile = profile;
            }
        }
        if let Ok(Some(z)) = update.status.z_thermal_adjust() {
            if let Some(enabled) = z.enabled {
                trigger |= Some(enabled) != self.transforms.z_thermal_adjust_enabled;
                self.transforms.z_thermal_adjust_enabled = Some(enabled);
            }
            if let Some(offset) = z.current_z_adjust {
                // Continuous drift: merged silently, never a trigger.
                self.transforms.z_thermal_adjust_offset = Some(offset);
            }
        }
        if let Ok(Some(sk)) = update.status.skew_correction() {
            if let Some(name) = sk.current_profile_name {
                // Observational only; plr-wal documents why this cannot
                // be trusted as skew state. Recorded for provenance.
                let profile = non_empty(name);
                trigger |= profile != self.transforms.skew_profile;
                self.transforms.skew_active = profile.is_some();
                self.transforms.skew_profile = profile;
            }
        }
        trigger
    }

    /// Merges an `exclude_object` diff; `true` when the cancellation
    /// picture changed and a context must be journaled immediately.
    ///
    /// `current_object` is merged but deliberately never triggers — see
    /// the module-level trigger rules.
    fn merge_exclude_object(&mut self, update: &StatusUpdate) -> bool {
        let Ok(Some(status)) = update.status.exclude_object() else {
            return false;
        };
        let first_observation = !self.exclude_seen;
        self.exclude_seen = true;
        let change = self.exclude.merge(&status);
        self.exclude_definitions_dirty |= first_observation || change.definitions;
        first_observation || change.definitions || change.excluded
    }

    /// The exclude-object payload for the next context, or `None` when
    /// `exclude_object` has never been observed (the module is not
    /// configured, or no update carried it yet).
    fn exclude_state(&self) -> Option<Box<ExcludeState>> {
        self.exclude_seen.then(|| {
            Box::new(ExcludeState {
                definitions: self.exclude_definitions_dirty.then(|| {
                    self.exclude
                        .objects
                        .iter()
                        .map(exclude_definition)
                        .collect()
                }),
                excluded: self.exclude.excluded_objects.clone(),
                current: self.exclude.current_object.clone(),
            })
        })
    }

    /// Builds a context snapshot; `None` until a full `gcode_move` state
    /// has been seen (always present after the initial subscribe
    /// response).
    fn build_context(&self, mono_ns: u64) -> Option<Context> {
        let gcode = self.gcode.clone()?;
        Some(Context {
            mono_ns,
            // The authoritative print state, verbatim. `None` until the
            // first `print_stats` observation of the session, which is
            // exactly "not observed".
            print_state: self.print_state.clone(),
            // The slicer-fed layer marks, verbatim. `None` until a
            // `SET_PRINT_STATS_INFO` was observed this session (the
            // operator's own OrcaSlicer emits none, so this is commonly
            // `None`) — an honest "not observed", never layer 0. A consumer
            // must treat `current_layer` as an *upper* bound on the
            // physically-printing layer; see `plr_wal::Context::current_layer`.
            current_layer: self.current_layer,
            total_layer: self.total_layer,
            // The trapq append frontier, paired atomically with
            // `virtual_sdcard.file_position` below because Klipper
            // produced both in one `_do_query` pass under
            // `assert_no_pause`. Journaling the pair is what lets recovery
            // certify durable trapq coverage of a file offset; see
            // `plr_wal::Context::print_time`.
            print_time: self.toolhead_print_time,
            virtual_sdcard: self.file_path.clone().map(|file_path| VirtualSdState {
                file_path,
                file_position: self.file_position,
                // 0 is Klipper's "no file loaded" value, not a real size, so
                // it must stay `None` = not observed rather than become a
                // size nothing can match.
                file_size: (self.file_size > 0).then_some(self.file_size),
            }),
            gcode,
            transforms: self.transforms.clone(),
            heaters: self
                .heaters
                .iter()
                .map(|(name, target)| HeaterTarget {
                    name: name.clone(),
                    target: *target,
                })
                .collect(),
            fans: self
                .fans
                .iter()
                .map(|(name, speed)| FanTarget {
                    name: name.clone(),
                    speed: *speed,
                })
                .collect(),
            exclude: self.exclude_state(),
        })
    }
}

/// Maps one Klipper object definition to its WAL form, normalizing the
/// geometry so a hostile outline can never poison the record.
///
/// The name is stored as Klipper reports it: `exclude_object.py` already
/// upper-cases every name it stores (`name.upper()` in
/// `cmd_EXCLUDE_OBJECT_DEFINE` and `cmd_EXCLUDE_OBJECT_START`), and the
/// same strings populate `excluded_objects`, so re-casing here could
/// only introduce a mismatch.
fn exclude_definition(def: &ExcludeObjectDefinition) -> ExcludeObjectDef {
    ExcludeObjectDef::normalized(def.name.clone(), def.center_xy(), def.polygon_xy())
}

/// The `print_stats` status fields this daemon reads.
///
/// `klippy/extras/print_stats.py`, `PrintStats.get_status`, returns
/// `{filename, total_duration, print_duration, filament_used, state,
/// message, info: {total_layer, current_layer}}`. `state` and `info` are
/// consumed here; the rest is derived timing the WAL already records
/// better. Declared locally rather than in `plr-klipper` because the
/// generic [`plr_klipper::Status::get`] accessor already covers it and
/// nothing outside this daemon needs the shape.
///
/// `state` and `info` are `Option` because Klipper's subscription stream
/// sends **diffs**: an update that changed only `print_duration` carries
/// neither key.
#[derive(Debug, Clone, serde::Deserialize)]
struct PrintStatsStatus {
    #[serde(default)]
    state: Option<String>,
    /// `print_stats.info`, present in a diff only when *some* sub-field of
    /// it changed. Klipper's subscription diff compares whole top-level
    /// field values (`QueryStatusHelper._do_query`,
    /// `klippy/webhooks.py`), so when `info` is present it carries the
    /// **complete** dict — both `current_layer` and `total_layer`, each an
    /// int or JSON `null` — never a partial. Absence means "unchanged
    /// since the last update", so the recorder keeps its stored marks.
    #[serde(default)]
    info: Option<PrintStatsInfo>,
}

/// `print_stats.info`: the slicer-fed layer marks
/// (`klippy/extras/print_stats.py`, set by `SET_PRINT_STATS_INFO`).
/// Both are nullable: `total_layer == 0` makes Klipper report both `null`.
#[derive(Debug, Clone, serde::Deserialize)]
struct PrintStatsInfo {
    #[serde(default)]
    current_layer: Option<u32>,
    #[serde(default)]
    total_layer: Option<u32>,
}

/// `print_stats.state`, the authoritative print state machine.
///
/// Every variant but [`Other`](PrintState::Other) is a literal Klipper
/// assigns in `klippy/extras/print_stats.py`: `reset()` → `standby`,
/// `note_start()` → `printing`, `note_pause()` → `paused`, and
/// `_note_finish()` → `complete` / `cancelled` / `error`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrintState {
    /// No print loaded (`reset()`).
    Standby,
    /// A print is running (`note_start()`).
    Printing,
    /// A print is paused (`note_pause()`); it can still be resumed.
    Paused,
    /// The file reached its end (`note_complete()`).
    Complete,
    /// The operator cancelled (`note_cancel()`).
    Cancelled,
    /// The print failed (`note_error()`).
    Error,
    /// A state string this version does not know. Never treated as
    /// finished — a future Klipper state must not suppress a recovery.
    Other,
}

impl PrintState {
    /// Maps a Klipper state string onto this enum. Unknown strings become
    /// [`PrintState::Other`].
    #[must_use]
    pub fn parse(state: &str) -> Self {
        match state {
            "standby" => Self::Standby,
            "printing" => Self::Printing,
            "paused" => Self::Paused,
            "complete" => Self::Complete,
            "cancelled" => Self::Cancelled,
            "error" => Self::Error,
            _ => Self::Other,
        }
    }

    /// `true` while a print exists and could still make progress.
    #[must_use]
    pub const fn is_in_progress(self) -> bool {
        matches!(self, Self::Printing | Self::Paused)
    }

    /// `true` when the print ended on purpose *and the state alone proves
    /// it* — `complete` or `cancelled`, both of which Klipper only ever
    /// reaches through `_note_finish()` at the end of a real print.
    ///
    /// [`Standby`](PrintState::Standby) is excluded, unlike
    /// [`is_finished`](Self::is_finished): `PrintStats.reset()` sets it on
    /// every klippy re-init, so a `FIRMWARE_RESTART` after a recoverable
    /// death journals `standby` for a print that very much did not end on
    /// purpose. This module can tell the difference because
    /// `print_in_progress` gates the transition; a *reader* of the journaled
    /// field has no such context, so it must not treat `standby` as proof.
    /// See `detect::last_print_state_for`.
    #[must_use]
    pub const fn is_conclusive_end(self) -> bool {
        matches!(self, Self::Complete | Self::Cancelled)
    }

    /// `true` when the print ended **on purpose**.
    ///
    /// [`Error`](PrintState::Error) is excluded deliberately: an errored
    /// print is the exact case power-loss recovery exists for, so it must
    /// never journal a clean shutdown. [`Standby`](PrintState::Standby)
    /// counts because the only way to reach it from a running print is
    /// `virtual_sdcard._reset_file`, which the operator asked for.
    #[must_use]
    pub const fn is_finished(self) -> bool {
        matches!(self, Self::Standby | Self::Complete | Self::Cancelled)
    }
}

/// Result of merging one `print_stats` diff.
#[derive(Debug, Default, Clone, Copy)]
struct PrintStatsMerge {
    /// `print_stats.state` differs from the recorded one, so a `Context`
    /// must be journaled immediately.
    state_changed: bool,
    /// A layer mark (`print_stats.info.current_layer`/`.total_layer`)
    /// changed, so a `Context` must be journaled immediately — the same
    /// treatment [`Self::state_changed`] gets, and for the same reason: a
    /// layer change happens a handful of times per print, not per move, and
    /// pins the layer answer to the instant the layer-change line parsed.
    layer_changed: bool,
    /// The state left `printing`/`paused` for a finished state: signal 1.
    clean_shutdown: bool,
}

/// Result of merging one `virtual_sdcard` diff.
#[derive(Debug, Default, Clone, Copy)]
struct VsdMerge {
    path_changed: bool,
    position_advanced: bool,
    clean_shutdown: bool,
}

/// Builds a complete `GcodeState` from a status object carrying every
/// field (the initial full response); `None` if any field is missing.
fn full_gcode_state(gm: &GcodeMoveStatus) -> Option<GcodeState> {
    Some(GcodeState {
        speed_factor: gm.speed_factor?,
        speed: gm.speed?,
        extrude_factor: gm.extrude_factor?,
        absolute_coordinates: gm.absolute_coordinates?,
        absolute_extrude: gm.absolute_extrude?,
        homing_origin: gm.homing_origin.clone()?,
        position: gm.position.clone()?,
        gcode_position: gm.gcode_position.clone()?,
    })
}

/// Passes a dump row through verbatim. The row fields are the signed C
/// ints of Klipper's `struct pull_history_steps`
/// (`klippy/chelper/stepcompress.h`): a negative `count` is a
/// reverse-direction chunk (`stepcompress.c:372`) and a negative
/// `interval` is a wrapped u32 tick count — both are real data the WAL
/// must preserve, so no clamping is performed.
fn step_chunk(step: &plr_klipper::StepperStep) -> StepChunk {
    StepChunk {
        interval: step.interval,
        count: step.count,
        add: step.add,
    }
}

fn non_empty(s: String) -> Option<String> {
    if s.is_empty() {
        None
    } else {
        Some(s)
    }
}

/// Converts a Klipper reactor `eventtime` (`CLOCK_MONOTONIC` seconds) to
/// nanoseconds. `None` for values outside the representable range.
fn eventtime_to_ns(eventtime: f64) -> Option<u64> {
    if !eventtime.is_finite() || eventtime < 0.0 {
        return None;
    }
    let ns = eventtime * 1e9;
    // 2^63 ns is ~292 years of uptime; anything above is garbage.
    #[allow(clippy::cast_precision_loss)]
    let limit = (1_u64 << 63) as f64;
    if ns >= limit {
        return None;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    Some(ns as u64)
}

#[cfg(test)]
mod tests {
    // Values in these tests are chosen exactly and parsed from JSON;
    // exact float comparison is intended.
    #![allow(clippy::float_cmp)]

    use super::{
        eventtime_to_ns, route_of, status_template, stepper_template, trapq_template, Recorder,
        Route, POSITION_CONTEXT_MIN_NS,
    };
    use crate::sender::SyncPolicy;
    use plr_klipper::{StatusUpdate, StepperBatch, TrapqBatch};
    use plr_wal::{ExcludeObjectDef, PolygonFidelity, WalRecord};
    use serde_json::json;

    fn status(v: serde_json::Value) -> StatusUpdate {
        serde_json::from_value(v).unwrap()
    }

    /// A full initial status as the subscribe response would carry.
    fn initial_status(eventtime: f64) -> serde_json::Value {
        json!({
            "eventtime": eventtime,
            "status": {
                "toolhead": {"print_time": 10.0, "estimated_print_time": 9.5,
                              "position": [5.0, 6.0, 0.2, 100.0]},
                "gcode_move": {
                    "speed_factor": 1.0, "speed": 1500.0, "extrude_factor": 1.0,
                    "absolute_coordinates": true, "absolute_extrude": false,
                    "homing_origin": [0.0, 0.0, 0.0, 0.0],
                    "position": [5.0, 6.0, 0.2, 100.0],
                    "gcode_position": [5.0, 6.0, 0.2, 100.0]
                },
                "virtual_sdcard": {"file_path": "/g/x.gcode", "progress": 0.5,
                                    "is_active": true, "file_position": 1000,
                                    "file_size": 2000},
                "extruder": {"temperature": 214.9, "target": 215.0, "power": 0.6},
                "heater_bed": {"temperature": 60.1, "target": 60.0, "power": 0.2},
                "fan": {"speed": 0.75, "rpm": null},
                "mcu": {"last_stats": {"receive_seq": 4100}},
                "bed_mesh": {"profile_name": "default",
                              "mesh_matrix": [[0.01, 0.02], [0.0, 0.01]]},
            }
        })
    }

    fn recorder_with_snapshot() -> Recorder {
        let mut r = Recorder::new();
        r.set_heater_names(vec!["extruder".into(), "heater_bed".into()]);
        let out = r.on_initial_status(&initial_status(100.0), 1_000).unwrap();
        assert_eq!(out.records.len(), 1, "initial context expected");
        r
    }

    #[test]
    fn reconstruct_config_derives_the_wal_heartbeat_period() {
        use crate::config::Config;

        // The reader's period is derived from the conservative 1 Hz basis
        // (WAL_HEARTBEAT_EVERY = 10 divisor of the 10 Hz file rate), NOT
        // the writer's actual active cadence (now one record per file beat,
        // WAL_HEARTBEAT_ACTIVE_EVERY = 1). The basis is deliberately the
        // sparsest active stream the reader can be handed — a pre-throttle
        // 1 Hz WAL — so those still read as continuous; the denser 10 Hz
        // stream is a fortiori continuous. Tightening it to 10 Hz would
        // make every pre-throttle WAL look like a chain of holes.
        let config = Config::default();
        assert_eq!(config.heartbeat_hz, 10.0);
        assert_eq!(
            super::reconstruct_config(Some(&config)).heartbeat_period_ns,
            1_000_000_000
        );

        // A non-default rate is followed, not assumed away (the daemon's
        // own test fixtures configure 20 Hz).
        let faster = Config {
            heartbeat_hz: 20.0,
            ..Config::default()
        };
        assert_eq!(
            super::reconstruct_config(Some(&faster)).heartbeat_period_ns,
            500_000_000
        );
        let slower = Config {
            heartbeat_hz: 2.0,
            ..Config::default()
        };
        assert_eq!(
            super::reconstruct_config(Some(&slower)).heartbeat_period_ns,
            5_000_000_000
        );

        // Entry points without a loaded config get the documented
        // defaults rather than a fabricated rate.
        assert_eq!(
            super::reconstruct_config(None),
            plr_reconstruct::ReconstructConfig::default()
        );

        // Everything else is left at the defaults.
        let derived = super::reconstruct_config(Some(&faster));
        let defaults = plr_reconstruct::ReconstructConfig::default();
        assert_eq!(derived.extension_horizon, defaults.extension_horizon);
        assert_eq!(
            derived.exclusion_freshness_horizon,
            defaults.exclusion_freshness_horizon
        );
        assert_eq!(
            derived.heartbeat_gap_tolerance,
            defaults.heartbeat_gap_tolerance
        );
    }

    #[test]
    fn reconstruct_config_is_total_on_a_degenerate_rate() {
        use crate::config::Config;

        // `Config::validate` rejects these, but a hand-built Config must
        // not be able to produce a nonsense period.
        for hz in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e-30, 1e30] {
            let config = Config {
                heartbeat_hz: hz,
                ..Config::default()
            };
            let derived = super::reconstruct_config(Some(&config));
            assert!(
                derived.validate().is_ok(),
                "heartbeat_hz {hz} produced an invalid config"
            );
        }
    }

    #[test]
    fn templates_and_routes_round_trip() {
        assert_eq!(route_of(&status_template()), Some(Route::Status));
        assert_eq!(
            route_of(&trapq_template("toolhead")),
            Some(Route::Trapq("toolhead".into()))
        );
        assert_eq!(
            route_of(&stepper_template("stepper_z1")),
            Some(Route::Stepper("stepper_z1".into()))
        );
        // Foreign or absent templates are not routed.
        assert_eq!(route_of(&serde_json::Map::new()), None);
        let mut foreign = serde_json::Map::new();
        foreign.insert("k".into(), json!("mystery:thing"));
        assert_eq!(route_of(&foreign), None);
        let mut wrong_type = serde_json::Map::new();
        wrong_type.insert("k".into(), json!(42));
        assert_eq!(route_of(&wrong_type), None);
    }

    #[test]
    fn initial_status_emits_full_context_and_heartbeat_sample() {
        let mut r = Recorder::new();
        r.set_heater_names(vec!["extruder".into(), "heater_bed".into()]);
        let out = r.on_initial_status(&initial_status(100.0), 1_000).unwrap();
        let (record, sync) = &out.records[0];
        assert_eq!(*sync, SyncPolicy::Immediate);
        let WalRecord::Context(ctx) = record else {
            panic!("expected context, got {record:?}");
        };
        assert_eq!(ctx.mono_ns, 1_000);
        let vsd = ctx.virtual_sdcard.as_ref().unwrap();
        assert_eq!(vsd.file_path, "/g/x.gcode");
        assert_eq!(vsd.file_position, 1_000);
        assert_eq!(ctx.gcode.speed_factor, 1.0);
        assert!(!ctx.gcode.absolute_extrude);
        assert_eq!(ctx.heaters.len(), 2);
        assert_eq!(ctx.heaters[0].name, "extruder");
        assert_eq!(ctx.heaters[0].target, 215.0);
        assert_eq!(ctx.fans[0].speed, 0.75);
        assert!(ctx.transforms.bed_mesh_active);
        assert_eq!(ctx.transforms.bed_mesh_profile.as_deref(), Some("default"));
        // Heartbeat sample: eventtime 100 s -> 100e9 ns, est 9.5.
        let hb = out.heartbeat.unwrap();
        assert_eq!(hb.est_sample_mono_ns, 100_000_000_000);
        assert_eq!(hb.est_sample_print_time, 9.5);
        assert_eq!(hb.print_time, 10.0);
        // receive_seq first observation is persisted.
        assert_eq!(out.receive_seq, Some((1_000, 4_100)));
        assert!(!out.clean_shutdown);
    }

    #[test]
    fn unchanged_diff_emits_nothing() {
        let mut r = recorder_with_snapshot();
        // Klipper only sends changed fields; position-only churn below
        // the throttle window emits no context.
        let out = r.on_status(
            &status(json!({"eventtime": 100.3, "status": {
                "toolhead": {"estimated_print_time": 9.8},
            }})),
            1_500,
            false,
        );
        assert!(out.records.is_empty());
        assert!(!out.clean_shutdown);
        // The heartbeat sample still refreshed.
        assert_eq!(out.heartbeat.unwrap().est_sample_print_time, 9.8);
    }

    #[test]
    fn state_changes_trigger_immediate_context() {
        let mut r = recorder_with_snapshot();
        // M220 S50 → speed_factor 0.5.
        let out = r.on_status(
            &status(json!({"eventtime": 101.0, "status": {
                "gcode_move": {"speed_factor": 0.5},
            }})),
            2_000,
            false,
        );
        assert_eq!(out.records.len(), 1);
        let (WalRecord::Context(ctx), SyncPolicy::Immediate) = &out.records[0] else {
            panic!("expected immediate context");
        };
        assert_eq!(ctx.gcode.speed_factor, 0.5);
        // Other merged fields survived.
        assert_eq!(ctx.gcode.speed, 1500.0);

        // Heater target change triggers; same target again does not.
        let update = json!({"eventtime": 102.0, "status": {
            "extruder": {"target": 220.0},
        }});
        assert_eq!(
            r.on_status(&status(update.clone()), 3_000, false)
                .records
                .len(),
            1
        );
        assert!(r
            .on_status(&status(update), 4_000, false)
            .records
            .is_empty());

        // Fan speed change triggers.
        let out = r.on_status(
            &status(json!({"eventtime": 103.0, "status": {"fan": {"speed": 1.0}}})),
            5_000,
            false,
        );
        assert_eq!(out.records.len(), 1);

        // Transform changes trigger: mesh cleared.
        let out = r.on_status(
            &status(json!({"eventtime": 104.0, "status": {
                "bed_mesh": {"profile_name": "", "mesh_matrix": [[]]},
            }})),
            6_000,
            false,
        );
        assert_eq!(out.records.len(), 1);
        let (WalRecord::Context(ctx), _) = &out.records[0] else {
            panic!("expected context");
        };
        assert!(!ctx.transforms.bed_mesh_active);
        assert_eq!(ctx.transforms.bed_mesh_profile, None);
    }

    #[test]
    fn position_advance_contexts_are_throttled() {
        let mut r = recorder_with_snapshot();
        let t0 = 1_000; // initial context timestamp
        let advance = |pos: u64| {
            json!({"eventtime": 100.5, "status": {
                "virtual_sdcard": {"file_position": pos},
            }})
        };
        // Within the throttle window: merged, not journaled.
        let out = r.on_status(&status(advance(1_100)), t0 + 1_000, false);
        assert!(out.records.is_empty());
        // Past the window: journaled with the *latest* position.
        let out = r.on_status(
            &status(advance(1_200)),
            t0 + POSITION_CONTEXT_MIN_NS + 1,
            false,
        );
        assert_eq!(out.records.len(), 1);
        let (WalRecord::Context(ctx), SyncPolicy::Immediate) = &out.records[0] else {
            panic!("expected immediate context");
        };
        assert_eq!(ctx.virtual_sdcard.as_ref().unwrap().file_position, 1_200);
        // A state change bypasses the throttle entirely.
        let out = r.on_status(
            &status(json!({"eventtime": 100.9, "status": {
                "virtual_sdcard": {"file_position": 1_300},
                "gcode_move": {"absolute_extrude": true},
            }})),
            t0 + POSITION_CONTEXT_MIN_NS + 2,
            false,
        );
        assert_eq!(out.records.len(), 1);
    }

    // -----------------------------------------------------------------
    // Part 3: opportunistic `print_stats.info` layer-mark capture.
    // -----------------------------------------------------------------

    /// A `print_stats.info` diff carrying the slicer's layer marks updates
    /// the recorder and triggers an immediate context (like the
    /// `print_stats.state` path), and the context journals both marks.
    #[test]
    fn a_layer_mark_change_triggers_an_immediate_context() {
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 101.0, "status": {
                "print_stats": {"info": {"current_layer": 7, "total_layer": 250}},
            }})),
            2_000,
            false,
        );
        assert_eq!(
            out.records.len(),
            1,
            "a layer change must journal a context"
        );
        let (WalRecord::Context(ctx), SyncPolicy::Immediate) = &out.records[0] else {
            panic!("expected immediate context");
        };
        assert_eq!(ctx.current_layer, Some(7));
        assert_eq!(ctx.total_layer, Some(250));
    }

    /// The layer marks arrive in a diff that carries **no `state` key** —
    /// the common shape, since the slicer's `SET_PRINT_STATS_INFO` changes
    /// only `info`. This is the regression the `merge_print_stats` early
    /// return on a missing `state` used to cause: `info` must be read
    /// before that return, or every layer mark is dropped.
    #[test]
    fn a_layer_mark_diff_without_a_state_key_is_not_dropped() {
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 101.0, "status": {
                "print_stats": {"info": {"current_layer": 3, "total_layer": 120}},
            }})),
            2_000,
            false,
        );
        assert_eq!(out.records.len(), 1);
        let (WalRecord::Context(ctx), _) = &out.records[0] else {
            panic!("expected context");
        };
        assert_eq!(ctx.current_layer, Some(3));
        assert_eq!(ctx.total_layer, Some(120));
    }

    /// The same marks reported again do not re-trigger a context (a diff
    /// echoing unchanged `info`), and an `info`-absent later diff keeps the
    /// stored marks — Klipper omits `info` when it did not change, so the
    /// next journaled context still carries the layer.
    #[test]
    fn unchanged_layer_marks_do_not_retrigger_but_persist() {
        let mut r = recorder_with_snapshot();
        let first = r.on_status(
            &status(json!({"eventtime": 101.0, "status": {
                "print_stats": {"info": {"current_layer": 5, "total_layer": 90}},
            }})),
            2_000,
            false,
        );
        assert_eq!(first.records.len(), 1);
        // Same marks again: no new context.
        let repeat = r.on_status(
            &status(json!({"eventtime": 102.0, "status": {
                "print_stats": {"info": {"current_layer": 5, "total_layer": 90}},
            }})),
            3_000,
            false,
        );
        assert!(
            repeat.records.is_empty(),
            "an unchanged layer must not re-journal"
        );
        // A later state change (with no `info` key) still carries the
        // stored marks: absence means "unchanged", not "cleared".
        let later = r.on_status(
            &status(json!({"eventtime": 103.0, "status": {
                "gcode_move": {"speed_factor": 0.5},
            }})),
            4_000,
            false,
        );
        let (WalRecord::Context(ctx), _) = &later.records[0] else {
            panic!("expected context");
        };
        assert_eq!(ctx.current_layer, Some(5));
        assert_eq!(ctx.total_layer, Some(90));
    }

    /// A `total_layer == 0` reset arrives as `info` with both fields null;
    /// the marks must go to `None` (the honest "not observed"), not a
    /// fabricated layer 0, and the transition triggers a context.
    #[test]
    fn a_null_layer_info_resets_the_marks_to_not_observed() {
        let mut r = recorder_with_snapshot();
        r.on_status(
            &status(json!({"eventtime": 101.0, "status": {
                "print_stats": {"info": {"current_layer": 12, "total_layer": 200}},
            }})),
            2_000,
            false,
        );
        let out = r.on_status(
            &status(json!({"eventtime": 102.0, "status": {
                "print_stats": {"info": {"current_layer": null, "total_layer": null}},
            }})),
            3_000,
            false,
        );
        assert_eq!(out.records.len(), 1, "the reset transition must journal");
        let (WalRecord::Context(ctx), _) = &out.records[0] else {
            panic!("expected context");
        };
        assert_eq!(ctx.current_layer, None);
        assert_eq!(ctx.total_layer, None);
    }

    #[test]
    fn clean_shutdown_on_complete_but_not_on_pause() {
        // Complete: position reaches size, then is_active drops.
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "virtual_sdcard": {"file_position": 2_000, "progress": 1.0,
                                    "is_active": false},
            }})),
            9_000_000_000,
            false,
        );
        assert!(out.clean_shutdown);

        // Pause: is_active drops mid-file with the file still loaded.
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "virtual_sdcard": {"is_active": false},
            }})),
            9_000_000_000,
            false,
        );
        assert!(!out.clean_shutdown);
        // Resume then cancel (file cleared in the same update).
        let out = r.on_status(
            &status(json!({"eventtime": 201.0, "status": {
                "virtual_sdcard": {"is_active": true},
            }})),
            9_100_000_000,
            false,
        );
        assert!(!out.clean_shutdown);
        let out = r.on_status(
            &status(json!({"eventtime": 202.0, "status": {
                "virtual_sdcard": {"file_path": null, "is_active": false,
                                    "file_position": 0},
            }})),
            9_200_000_000,
            false,
        );
        assert!(out.clean_shutdown);
    }

    // -----------------------------------------------------------------
    // D2 / D5 / D6: the clean-shutdown signals that were missing
    // -----------------------------------------------------------------

    /// **D2.** Pause, then cancel — the default Mainsail/Fluidd button
    /// order, the filament-runout flow, and the post-error flow.
    ///
    /// `do_cancel` starts with `do_pause()`, whose body is guarded by
    /// `if self.work_timer is not None`. The PAUSE already cleared
    /// `work_timer`, so that guard fails, `is_active` never changes, and
    /// the cancel's status diff carries **no `is_active` key at all** —
    /// only `file_path: null` (plus `print_stats.state`, since
    /// `do_cancel` calls `note_cancel()`). Keying on the `is_active`
    /// edge therefore cannot see this, deterministically, every time.
    #[test]
    fn pause_then_cancel_journals_a_clean_shutdown() {
        let mut r = recorder_with_snapshot();
        // PAUSE: is_active drops, the file stays loaded mid-file.
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "virtual_sdcard": {"is_active": false},
                "print_stats": {"state": "paused"},
            }})),
            9_000_000_000,
            false,
        );
        assert!(!out.clean_shutdown, "a pause is not a print end");

        // CANCEL_PRINT: exactly the diff Klipper produces. Note the
        // absence of `is_active`.
        let out = r.on_status(
            &status(json!({"eventtime": 201.0, "status": {
                "print_stats": {"state": "cancelled"},
                "virtual_sdcard": {"file_path": null, "file_position": 0,
                                    "file_size": 0},
            }})),
            9_100_000_000,
            false,
        );
        assert!(
            out.clean_shutdown,
            "pause->cancel must journal a CleanShutdown marker"
        );
    }

    /// The same flow on a printer whose `print_stats` never reaches the
    /// recorder: signal 2 (`file_path` Some -> None) carries it alone.
    #[test]
    fn pause_then_cancel_journals_a_clean_shutdown_without_print_stats() {
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "virtual_sdcard": {"is_active": false},
            }})),
            9_000_000_000,
            false,
        );
        assert!(!out.clean_shutdown);
        let out = r.on_status(
            &status(json!({"eventtime": 201.0, "status": {
                "virtual_sdcard": {"file_path": null, "file_position": 0,
                                    "file_size": 0},
            }})),
            9_100_000_000,
            false,
        );
        assert!(out.clean_shutdown);
    }

    /// **D5.** The direct-cancel race: `do_pause`'s 1 ms `reactor.pause`
    /// loop yields to the reactor, so the 250 ms status subscription can
    /// sample *between* `work_timer` going `None` and the file closing.
    ///
    /// That intermediate update looks exactly like a pause (`is_active`
    /// false, file still loaded mid-file); the file-clearing update that
    /// follows carries no `is_active`. Two updates, neither of which the
    /// `is_active` edge test can call a print end.
    #[test]
    fn the_direct_cancel_race_shape_journals_a_clean_shutdown() {
        let mut r = recorder_with_snapshot();
        // Sampled inside do_pause's spin loop.
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "virtual_sdcard": {"is_active": false, "file_position": 1_400},
            }})),
            9_000_000_000,
            false,
        );
        assert!(
            !out.clean_shutdown,
            "mid-file with the file loaded is indistinguishable from a pause"
        );
        // Sampled after current_file.close().
        let out = r.on_status(
            &status(json!({"eventtime": 200.25, "status": {
                "print_stats": {"state": "cancelled"},
                "virtual_sdcard": {"file_path": null, "file_position": 0,
                                    "file_size": 0},
            }})),
            9_250_000_000,
            false,
        );
        assert!(out.clean_shutdown, "the race must not lose the marker");
    }

    /// A pause alone, in every shape it arrives in, is never a print end.
    #[test]
    fn a_pause_is_never_a_clean_shutdown() {
        for diff in [
            json!({"virtual_sdcard": {"is_active": false}}),
            json!({"print_stats": {"state": "paused"}}),
            json!({"virtual_sdcard": {"is_active": false},
                   "print_stats": {"state": "paused"}}),
            // Paused, then a heater change: still paused, still not an end.
            json!({"print_stats": {"state": "paused"},
                   "extruder": {"target": 0.0}}),
        ] {
            let mut r = recorder_with_snapshot();
            let out = r.on_status(
                &status(json!({"eventtime": 200.0, "status": diff})),
                9_000_000_000,
                false,
            );
            assert!(!out.clean_shutdown, "{diff:?} must not be a print end");
        }
    }

    /// **D6.** `print_stats` is the authoritative state machine, and its
    /// transitions out of `printing`/`paused` are what the marker keys on.
    /// The recorder journals `virtual_sdcard.file_size` alongside the
    /// position, so a reader can tell "same path" from "same file". Klipper's
    /// 0 (no file loaded) stays `None` — "not observed" — rather than
    /// becoming a size nothing can match.
    #[test]
    fn the_file_size_is_journaled_beside_the_position() {
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "virtual_sdcard": {"file_position": 1_200},
            }})),
            9_000_000_000,
            false,
        );
        let (WalRecord::Context(ctx), _) = &out.records[0] else {
            panic!("expected a context");
        };
        let vsd = ctx.virtual_sdcard.as_ref().expect("vsd");
        assert_eq!(vsd.file_position, 1_200);
        assert_eq!(vsd.file_size, Some(2_000), "from the initial status");

        // A printer that never reported a size, and Klipper's 0 sentinel,
        // both stay "not observed".
        let mut r = Recorder::new();
        let out = r.on_status(
            &status(json!({"eventtime": 1.0, "status": {
                "gcode_move": {
                    "speed_factor": 1.0, "speed": 1500.0, "extrude_factor": 1.0,
                    "absolute_coordinates": true, "absolute_extrude": true,
                    "homing_origin": [0.0, 0.0, 0.0, 0.0],
                    "position": [0.0, 0.0, 0.0, 0.0],
                    "gcode_position": [0.0, 0.0, 0.0, 0.0]
                },
                "virtual_sdcard": {"file_path": "/g/y.gcode", "file_position": 0,
                                    "file_size": 0},
            }})),
            1_000,
            true,
        );
        let (WalRecord::Context(ctx), _) = &out.records[0] else {
            panic!("expected a context");
        };
        assert_eq!(ctx.virtual_sdcard.as_ref().unwrap().file_size, None);
    }

    #[test]
    fn print_stats_transitions_decide_the_marker() {
        use super::PrintState;
        // The finished states, straight from `printing`.
        for state in ["complete", "cancelled", "standby"] {
            let mut r = recorder_with_snapshot();
            let out = r.on_status(
                &status(json!({"eventtime": 200.0, "status": {
                    "print_stats": {"state": state},
                }})),
                9_000_000_000,
                false,
            );
            assert!(out.clean_shutdown, "printing -> {state} is a print end");
        }
        // `error` is NOT a deliberate end: an errored print is exactly
        // what recovery exists for.
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "print_stats": {"state": "error", "message": "Move out of range"},
            }})),
            9_000_000_000,
            false,
        );
        assert!(
            !out.clean_shutdown,
            "an errored print must stay recoverable"
        );
        // Neither is a state this version does not know.
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "print_stats": {"state": "hibernating"},
            }})),
            9_000_000_000,
            false,
        );
        assert!(!out.clean_shutdown);
        assert_eq!(PrintState::parse("hibernating"), PrintState::Other);
        assert!(!PrintState::Other.is_finished());
        assert!(!PrintState::Other.is_in_progress());
        assert!(!PrintState::Error.is_finished());
        assert!(PrintState::Printing.is_in_progress());
        assert!(PrintState::Paused.is_in_progress());
        assert!(PrintState::Standby.is_finished());
        assert!(PrintState::Complete.is_finished());
        assert!(PrintState::Cancelled.is_finished());
        // The narrower read-side predicate excludes `standby`, which a
        // klippy re-init sets for a print that did not end on purpose.
        assert!(!PrintState::Standby.is_conclusive_end());
        assert!(PrintState::Complete.is_conclusive_end());
        assert!(PrintState::Cancelled.is_conclusive_end());
        assert!(!PrintState::Error.is_conclusive_end());
        assert!(!PrintState::Other.is_conclusive_end());
        assert!(!PrintState::Printing.is_conclusive_end());
        assert!(!PrintState::Paused.is_conclusive_end());
        // A diff that carries print_stats without a `state` key (only the
        // duration ticked) changes nothing.
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "print_stats": {"print_duration": 12.5},
            }})),
            9_000_000_000,
            false,
        );
        assert!(!out.clean_shutdown);
        // And a finished state with no in-progress state ever observed is
        // not a transition: a daemon starting up beside an idle printer
        // must not journal a print end.
        let mut r = Recorder::new();
        let out = r.on_status(
            &status(json!({"eventtime": 1.0, "status": {
                "print_stats": {"state": "standby"},
            }})),
            1_000,
            false,
        );
        assert!(!out.clean_shutdown);
    }

    /// The session gate: after a klippy RESTART the fresh baseline reports
    /// `file_path: null`, which against the stale snapshot *is* a
    /// Some -> None transition. It must not forge a print end.
    #[test]
    fn a_restart_baseline_does_not_forge_a_file_path_clean_shutdown() {
        let mut r = recorder_with_snapshot(); // printing /g/x.gcode
        r.reset_session();
        let out = r.on_status(
            &status(json!({"eventtime": 150.0, "status": {
                "virtual_sdcard": {"file_path": null, "file_position": 0,
                                    "file_size": 0},
            }})),
            50_000,
            false,
        );
        assert!(
            !out.clean_shutdown,
            "the restart killed the print; it must stay recoverable"
        );
        // A print started after the restart reopens the gate, and its
        // cancel is seen again.
        let out = r.on_status(
            &status(json!({"eventtime": 160.0, "status": {
                "virtual_sdcard": {"file_path": "/g/y.gcode", "is_active": true,
                                    "file_position": 0, "file_size": 5_000},
                "print_stats": {"state": "printing"},
            }})),
            60_000,
            false,
        );
        assert!(!out.clean_shutdown);
        let out = r.on_status(
            &status(json!({"eventtime": 170.0, "status": {
                "virtual_sdcard": {"file_path": null},
            }})),
            70_000,
            false,
        );
        assert!(out.clean_shutdown);
    }

    /// `reset_session` drops the print state as well, so nothing about the
    /// dead klippy instance leaks into the next session's inference.
    #[test]
    fn reset_session_drops_the_print_state() {
        let mut r = recorder_with_snapshot();
        let _ = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "print_stats": {"state": "printing"},
            }})),
            9_000_000_000,
            false,
        );
        r.reset_session();
        // With the previous state forgotten, a `cancelled` from the new
        // session's baseline is not a transition out of printing.
        let out = r.on_status(
            &status(json!({"eventtime": 210.0, "status": {
                "print_stats": {"state": "cancelled"},
            }})),
            9_100_000_000,
            false,
        );
        assert!(!out.clean_shutdown);
    }

    /// `reset_session` drops the layer marks too: they belong to the dead
    /// klippy instance's print, so the next session must re-observe them
    /// from its own `SET_PRINT_STATS_INFO` rather than journal a stale
    /// layer against a different (or absent) print.
    #[test]
    fn reset_session_drops_the_layer_marks() {
        let mut r = recorder_with_snapshot();
        let _ = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "print_stats": {"info": {"current_layer": 33, "total_layer": 90}},
            }})),
            9_000_000_000,
            false,
        );
        r.reset_session();
        // A state change in the new session journals a context; the stale
        // marks must not ride along on it.
        let out = r.on_status(
            &status(json!({"eventtime": 210.0, "status": {
                "gcode_move": {"speed_factor": 0.5},
            }})),
            9_100_000_000,
            false,
        );
        let (WalRecord::Context(ctx), _) = &out.records[0] else {
            panic!("expected context");
        };
        assert_eq!(ctx.current_layer, None);
        assert_eq!(ctx.total_layer, None);
    }

    #[test]
    fn session_reset_prevents_false_clean_shutdown_across_reconnects() {
        // A print is running, klippy RESTARTs (socket loss), the daemon
        // reconnects: the fresh baseline reports is_active=false with no
        // file loaded. Diffed naively against the stale snapshot this
        // looks exactly like a cancel — it must NOT be journaled as a
        // clean shutdown, because the restart killed the print.
        let mut r = recorder_with_snapshot(); // is_active = true
        r.reset_session();
        // eventtime keeps counting across the restart (it is the host
        // monotonic clock); print_time restarts near zero.
        let baseline = json!({
            "eventtime": 150.0,
            "status": {
                "toolhead": {"print_time": 0.1, "estimated_print_time": 0.1},
                "gcode_move": {
                    "speed_factor": 1.0, "speed": 1500.0, "extrude_factor": 1.0,
                    "absolute_coordinates": true, "absolute_extrude": false,
                    "homing_origin": [0.0, 0.0, 0.0, 0.0],
                    "position": [0.0, 0.0, 0.0, 0.0],
                    "gcode_position": [0.0, 0.0, 0.0, 0.0]
                },
                "virtual_sdcard": {"file_path": null, "progress": 0.0,
                                    "is_active": false, "file_position": 0,
                                    "file_size": 0},
            }
        });
        let out = r.on_initial_status(&baseline, 50_000).unwrap();
        assert!(
            !out.clean_shutdown,
            "a klippy restart mid-print must not be journaled as clean"
        );
        // Heartbeats resume from the *new* instance's sample — with the
        // reset print-time frontier, not the pre-restart maximum.
        let hb = out.heartbeat.unwrap();
        assert_eq!(hb.est_sample_mono_ns, 150_000_000_000);
        assert_eq!(hb.est_sample_print_time, 0.1);
        assert_eq!(hb.print_time, 0.1);
    }

    #[test]
    fn reset_session_pauses_heartbeat_until_fresh_sample() {
        let mut r = recorder_with_snapshot();
        assert!(r.heartbeat_data(2_000).is_some());
        r.reset_session();
        assert!(
            r.heartbeat_data(2_000).is_none(),
            "no liveness claim from a dead session's sample"
        );
    }

    /// An idle recorder throttles its heartbeat cadence; the **data plane**
    /// (motion arriving) raises it at or before the lagging status plane
    /// can; and the active → idle fall journals exactly one
    /// `RecordingQuiescent` marker so the sparse stream that follows is a
    /// recorded fact.
    #[test]
    fn idle_regime_throttles_and_marks_but_motion_raises_it_first() {
        let mut r = Recorder::new();
        r.set_heater_names(vec!["extruder".into()]);
        // Idle initial status: printer in standby, nothing active.
        let idle = json!({"eventtime": 100.0, "status": {
            "toolhead": {"print_time": 0.0, "estimated_print_time": 9.5,
                          "position": [0.0, 0.0, 0.0, 0.0]},
            "gcode_move": {"speed_factor": 1.0, "speed": 1500.0, "extrude_factor": 1.0,
                "absolute_coordinates": true, "absolute_extrude": true,
                "homing_origin": [0.0, 0.0, 0.0, 0.0], "position": [0.0, 0.0, 0.0, 0.0],
                "gcode_position": [0.0, 0.0, 0.0, 0.0]},
            "virtual_sdcard": {"file_path": null, "is_active": false,
                                "file_position": 0, "file_size": 0},
            "print_stats": {"state": "standby"},
        }});
        let out = r.on_initial_status(&idle, 1_000_000_000).unwrap();
        assert_eq!(
            out.regime_marker,
            Some(plr_wal::MarkerKind::RecordingQuiescent),
            "a session that starts idle marks its quiescence on the first evaluation"
        );
        assert!(
            !out.heartbeat.unwrap().active,
            "idle => throttled heartbeat cadence"
        );

        // Motion arrives (trapq) BEFORE any `printing` status: the regime
        // must already be active — the Trap-2 data-plane trigger.
        let batch: TrapqBatch = serde_json::from_value(json!({"data": [
            [12.5, 0.25, 40.0, -1500.0, [10.0, 20.0, 0.3], [1.0, 0.0, 0.0]]]}))
        .unwrap();
        let out = r.on_trapq("toolhead", &batch, 2_000_000_000);
        assert!(
            out.heartbeat.unwrap().active,
            "motion alone raises the regime, without waiting for print_stats"
        );
        assert_eq!(out.regime_marker, None, "the rise is deliberately unmarked");

        // 6 s after the last motion, still no print in progress: the regime
        // falls back to idle and journals the marker exactly once.
        let out = r.on_status(
            &status(json!({"eventtime": 108.0, "status": {
                "toolhead": {"estimated_print_time": 9.6}}})),
            8_000_000_000,
            false,
        );
        assert!(
            !out.heartbeat.unwrap().active,
            "no motion for > IDLE_AFTER_MOTION_NS => idle again"
        );
        assert_eq!(
            out.regime_marker,
            Some(plr_wal::MarkerKind::RecordingQuiescent),
            "the active -> idle fall must be journaled"
        );

        // Staying idle does not re-emit the marker.
        let out = r.on_status(
            &status(json!({"eventtime": 110.0, "status": {
                "toolhead": {"estimated_print_time": 9.7}}})),
            10_000_000_000,
            false,
        );
        assert_eq!(out.regime_marker, None, "one marker per fall, not per beat");
        assert!(!out.heartbeat.unwrap().active);
    }

    /// A print in progress holds the regime active through a long dwell
    /// (heating, `G4`) even with no motion for tens of seconds, so
    /// heartbeats stay dense and no `RecordingQuiescent` is journaled
    /// mid-print — the property that keeps a quiet span from ever
    /// overlapping a recoverable window.
    #[test]
    fn a_print_in_progress_stays_active_through_a_long_dwell() {
        // `recorder_with_snapshot` processed an initial status with
        // `is_active: true`, so a print is in progress and the regime rose.
        let mut r = recorder_with_snapshot();
        assert!(r.regime_active(1_000), "a loaded print is active");
        // 30 s later, only estimated_print_time has ticked — no motion.
        let out = r.on_status(
            &status(json!({"eventtime": 200.0, "status": {
                "toolhead": {"estimated_print_time": 40.0}}})),
            40_000_000_000,
            false,
        );
        assert!(
            out.heartbeat.unwrap().active,
            "a dwell mid-print must stay at full cadence"
        );
        assert_eq!(
            out.regime_marker, None,
            "no quiescent marker may be journaled inside a print"
        );
    }

    /// A daemon that starts on an idle printer (the commonest shape — the
    /// real capture's segment 1 is 19.6 h of it) journals its quiescence on
    /// the very first evaluation, so the sparse stream that follows is a
    /// recorded fact and not an ambiguous absence. A session that starts
    /// mid-print does NOT mark (dense heartbeats speak for themselves).
    #[test]
    fn a_session_that_starts_idle_marks_its_quiescence() {
        let idle = json!({"eventtime": 100.0, "status": {
            "toolhead": {"print_time": 0.0, "estimated_print_time": 9.5,
                          "position": [0.0, 0.0, 0.0, 0.0]},
            "gcode_move": {"speed_factor": 1.0, "speed": 1500.0, "extrude_factor": 1.0,
                "absolute_coordinates": true, "absolute_extrude": true,
                "homing_origin": [0.0, 0.0, 0.0, 0.0], "position": [0.0, 0.0, 0.0, 0.0],
                "gcode_position": [0.0, 0.0, 0.0, 0.0]},
            "virtual_sdcard": {"file_path": null, "is_active": false,
                                "file_position": 0, "file_size": 0},
            "print_stats": {"state": "standby"},
        }});
        let mut r = Recorder::new();
        r.set_heater_names(vec!["extruder".into()]);
        let out = r.on_initial_status(&idle, 1_000_000_000).unwrap();
        assert_eq!(
            out.regime_marker,
            Some(plr_wal::MarkerKind::RecordingQuiescent),
            "the idle-from-birth session must journal quiescence on its first evaluation"
        );

        // A session that starts mid-print (is_active: true) must NOT mark:
        // the dense heartbeat stream is its own liveness proof.
        let mut r = recorder_with_snapshot();
        // recorder_with_snapshot already ran the initial (active) status.
        assert!(r.regime_active(1));
        // Re-run the constructor's assertion path: the initial status
        // carried is_active: true, so no quiescent marker was produced.
        let out = r.on_status(
            &status(json!({"eventtime": 100.1, "status": {
                    "toolhead": {"estimated_print_time": 9.6}}})),
            1_100_000_000,
            false,
        );
        assert_eq!(
            out.regime_marker, None,
            "an active-from-birth session must not journal a quiescent marker"
        );
    }

    #[test]
    fn trapq_batches_map_to_segments_and_advance_print_time() {
        let mut r = recorder_with_snapshot();
        let batch: TrapqBatch = serde_json::from_value(json!({"data": [
            [12.5, 0.25, 40.0, -1500.0, [10.0, 20.0, 0.3], [1.0, 0.0, 0.0]],
            [12.75, 0.1, 5.0, 0.0, [20.0, 20.0, 0.3], [0.0, 1.0, 0.0]],
        ]}))
        .unwrap();
        let out = r.on_trapq("toolhead", &batch, 7_000);
        assert_eq!(out.records.len(), 2);
        let (WalRecord::TrapqSegment(s0), SyncPolicy::Batched) = &out.records[0] else {
            panic!("expected batched trapq segment");
        };
        assert_eq!(s0.queue, "toolhead");
        assert_eq!(s0.mono_ns, 7_000);
        assert_eq!(s0.print_time, 12.5);
        assert_eq!(s0.duration, 0.25);
        assert_eq!(s0.start_velocity, 40.0);
        assert_eq!(s0.acceleration, -1500.0);
        assert_eq!((s0.start_x, s0.start_y, s0.start_z), (10.0, 20.0, 0.3));
        assert_eq!((s0.x_r, s0.y_r, s0.z_r), (1.0, 0.0, 0.0));
        // Heartbeat print_time advanced to the batch's end (12.85).
        assert_eq!(out.heartbeat.unwrap().print_time, 12.85);
    }

    #[test]
    fn stepper_batches_map_to_ranges_with_chunk_conversion() {
        let mut r = Recorder::new();
        // Rows include real captured triple-Z values: a wrapped-u32
        // interval and reverse-direction (negative) counts
        // (stepcompress.c:372), plus a set_position marker row.
        let batch: StepperBatch = serde_json::from_value(json!({
            "data": [[-2_136_919_700, 1, 0], [10_000, 976, 0], [9855, -40, 187],
                     [12_000, -1, 0], [0, 0, 0]],
            "start_position": 12.7,
            "start_mcu_position": -3175,
            "step_distance": 0.0025,
            "first_clock": 5_000_000_000_u64,
            "first_step_time": 27.7,
            "last_clock": 5_009_862_855_u64,
            "last_step_time": 27.83
        }))
        .unwrap();
        let out = r.on_stepper("stepper_z", &batch, 8_000);
        assert_eq!(out.records.len(), 1);
        let (WalRecord::StepperRange(range), SyncPolicy::Batched) = &out.records[0] else {
            panic!("expected batched stepper range");
        };
        assert_eq!(range.stepper, "stepper_z");
        assert_eq!(range.first_clock, 5_000_000_000);
        assert_eq!(range.last_clock, 5_009_862_855);
        assert_eq!(range.start_mcu_position, -3175);
        assert_eq!(range.steps.len(), 5);
        assert_eq!(
            (
                range.steps[1].interval,
                range.steps[1].count,
                range.steps[1].add
            ),
            (10_000, 976, 0)
        );
        // Signed values pass through the conversion verbatim.
        assert_eq!(range.steps[0].interval, -2_136_919_700);
        assert_eq!(range.steps[2].count, -40);
        assert_eq!(range.steps[2].add, 187);
        assert_eq!(range.steps[3].count, -1);
        assert_eq!((range.steps[4].interval, range.steps[4].count), (0, 0));
        // No est sample yet: no heartbeat claim.
        assert!(out.heartbeat.is_none());
    }

    #[test]
    fn step_chunk_conversion_preserves_signed_history_values() {
        // Full i32 range must survive: these are the C `int` fields of
        // struct pull_history_steps, not the MCU wire widths.
        let step: plr_klipper::StepperStep =
            serde_json::from_value(json!([i32::MIN, i32::MAX, -40_000])).unwrap();
        let chunk = super::step_chunk(&step);
        assert_eq!(chunk.interval, i32::MIN);
        assert_eq!(chunk.count, i32::MAX);
        assert_eq!(chunk.add, -40_000);
        // A realistic reverse Z chunk keeps its sign exactly.
        let step: plr_klipper::StepperStep = serde_json::from_value(json!([4964, -40, 0])).unwrap();
        let chunk = super::step_chunk(&step);
        assert_eq!((chunk.interval, chunk.count, chunk.add), (4964, -40, 0));
    }

    #[test]
    fn receive_seq_persists_on_advance_only() {
        let mut r = recorder_with_snapshot(); // saw 4100 (First)
        let seq_update = |seq: u64, et: f64| {
            status(json!({"eventtime": et, "status": {
                "mcu": {"last_stats": {"receive_seq": seq}},
            }}))
        };
        // Unchanged: nothing persisted.
        assert_eq!(
            r.on_status(&seq_update(4_100, 101.0), 2_000, false)
                .receive_seq,
            None
        );
        // Advance: persisted with the widened value.
        assert_eq!(
            r.on_status(&seq_update(4_600, 102.0), 3_000, false)
                .receive_seq,
            Some((3_000, 4_600))
        );
        // Regression (MCU restart): held, not persisted.
        assert_eq!(
            r.on_status(&seq_update(3, 103.0), 4_000, false).receive_seq,
            None
        );
    }

    #[test]
    fn notification_routing_dispatches_and_rejects_bad_payloads() {
        let mut r = recorder_with_snapshot();
        let notification = |params: serde_json::Value| plr_klipper::Notification {
            params,
            template: serde_json::Map::new(),
        };
        let out = r
            .on_notification(
                &Route::Trapq("extruder".into()),
                &notification(
                    json!({"data": [[1.0, 0.5, 2.0, 0.0, [7.0, 0.0, 0.0], [1.0, 0.0, 0.0]]]}),
                ),
                5_000,
            )
            .unwrap();
        assert_eq!(out.records.len(), 1);
        let (WalRecord::TrapqSegment(seg), _) = &out.records[0] else {
            panic!("expected trapq segment");
        };
        assert_eq!(seg.queue, "extruder");
        // Status route parses too.
        let out = r
            .on_notification(
                &Route::Status,
                &notification(json!({"eventtime": 105.0, "status": {}})),
                6_000,
            )
            .unwrap();
        assert!(out.records.is_empty());
        // Malformed payloads surface as errors, not panics.
        assert!(r
            .on_notification(
                &Route::Stepper("stepper_z".into()),
                &notification(json!(7)),
                1
            )
            .is_err());
        assert!(r
            .on_notification(&Route::Status, &notification(json!({"nope": 1})), 1)
            .is_err());
    }

    #[test]
    fn no_context_before_full_gcode_state() {
        let mut r = Recorder::new();
        // A bare diff cannot build a full snapshot: no context yet.
        let out = r.on_status(
            &status(json!({"eventtime": 1.0, "status": {
                "gcode_move": {"speed_factor": 0.5},
            }})),
            100,
            true,
        );
        assert!(out.records.is_empty());
        assert!(out.heartbeat.is_none());
    }

    #[test]
    fn stale_eventtime_does_not_regress_heartbeat_sample() {
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 50.0, "status": {
                "toolhead": {"estimated_print_time": 999.0},
            }})),
            2_000,
            false,
        );
        // Rejected by the correlator: the sample keeps the old anchor.
        assert_eq!(out.heartbeat.unwrap().est_sample_print_time, 9.5);
    }

    /// Extracts the single context a status update produced.
    fn only_context(out: &super::Output) -> &plr_wal::Context {
        assert_eq!(out.records.len(), 1, "expected exactly one record");
        let (WalRecord::Context(ctx), SyncPolicy::Immediate) = &out.records[0] else {
            panic!("expected an immediate context, got {:?}", out.records[0]);
        };
        ctx
    }

    /// An `exclude_object` status delta, in Klipper's shape.
    fn exclude_update(eventtime: f64, exclude: &serde_json::Value) -> StatusUpdate {
        status(json!({"eventtime": eventtime, "status": {"exclude_object": exclude}}))
    }

    #[test]
    fn no_exclude_object_observation_leaves_the_field_absent() {
        // A printer without [exclude_object] never sends the object.
        // `None` must keep meaning "not observed", never "nothing
        // excluded" — that distinction is the whole point downstream.
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 101.0, "status": {"gcode_move": {"speed_factor": 0.5}}})),
            2_000,
            false,
        );
        assert_eq!(only_context(&out).exclude, None);
    }

    #[test]
    fn first_exclude_observation_journals_definitions_once() {
        let mut r = recorder_with_snapshot();
        // The initial full status of a plate with two objects.
        let out = r.on_status(
            &exclude_update(
                101.0,
                &json!({
                    "objects": [
                        {"name": "CUBE_ID_0_COPY_0", "center": [50.0, 50.0],
                         "polygon": [[45.0, 45.0], [55.0, 45.0], [55.0, 55.0], [45.0, 55.0]]},
                        {"name": "CUBE_ID_1_COPY_0"}
                    ],
                    "excluded_objects": [],
                    "current_object": null
                }),
            ),
            2_000,
            false,
        );
        let state = only_context(&out).exclude.clone().unwrap();
        let definitions = state.definitions.unwrap();
        assert_eq!(definitions.len(), 2);
        assert_eq!(definitions[0].name, "CUBE_ID_0_COPY_0");
        assert_eq!(definitions[0].center, Some([50.0, 50.0]));
        assert_eq!(definitions[0].polygon.len(), 4);
        assert_eq!(definitions[0].fidelity, PolygonFidelity::Exact);
        assert_eq!(
            definitions[1],
            ExcludeObjectDef::name_only("CUBE_ID_1_COPY_0")
        );
        assert!(state.excluded.is_empty());
        assert_eq!(state.current, None);

        // A later, unrelated trigger must NOT re-journal the polygons.
        let out = r.on_status(
            &status(json!({"eventtime": 102.0, "status": {"fan": {"speed": 0.2}}})),
            3_000,
            false,
        );
        let state = only_context(&out).exclude.clone().unwrap();
        assert_eq!(
            state.definitions, None,
            "definitions are journaled once, not in every context"
        );
        assert!(state.excluded.is_empty());
    }

    #[test]
    fn the_empty_excluded_set_is_journaled_positively() {
        // Positive journaling: the very first exclude_object
        // observation records "zero objects excluded as of t" as a
        // fact. Without this, reconstruction could not distinguish
        // "nothing was cancelled" from "we never looked", and the
        // absence-of-record path would be the common case rather than
        // the rare one.
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &exclude_update(
                101.0,
                &json!({"objects": [{"name": "A"}, {"name": "B"}],
                        "excluded_objects": [], "current_object": null}),
            ),
            2_000,
            false,
        );
        let state = only_context(&out).exclude.clone().unwrap();
        assert!(
            state.excluded.is_empty(),
            "an empty set is still journaled, not omitted"
        );
        assert_eq!(state.definitions.unwrap().len(), 2);

        // And it keeps riding along on every later context, so the
        // newest surviving one always states the set.
        let out = r.on_status(
            &status(json!({"eventtime": 102.0, "status": {"fan": {"speed": 0.3}}})),
            3_000,
            false,
        );
        let state = only_context(&out).exclude.clone().unwrap();
        assert!(state.excluded.is_empty());
        assert_eq!(state.definitions, None, "definitions are not repeated");
    }

    #[test]
    fn cancelling_an_object_forces_an_immediate_context() {
        let mut r = recorder_with_snapshot();
        r.on_status(
            &exclude_update(
                101.0,
                &json!({"objects": [{"name": "A"}, {"name": "B"}],
                       "excluded_objects": [], "current_object": "A"}),
            ),
            2_000,
            false,
        );
        // EXCLUDE_OBJECT NAME=B — arrives as a diff carrying only the
        // changed field. This is the record we cannot afford to lose to
        // the batch window.
        let out = r.on_status(
            &exclude_update(101.5, &json!({"excluded_objects": ["B"]})),
            2_100,
            false,
        );
        let (record, sync) = &out.records[0];
        assert_eq!(*sync, SyncPolicy::Immediate, "exclusions must be fsync'd");
        let WalRecord::Context(ctx) = record else {
            panic!("expected a context");
        };
        let state = ctx.exclude.clone().unwrap();
        assert_eq!(state.excluded, vec!["B".to_owned()]);
        assert_eq!(state.definitions, None, "definitions unchanged");

        // Re-sending the same set changes nothing.
        assert!(r
            .on_status(
                &exclude_update(102.0, &json!({"excluded_objects": ["B"]})),
                2_200,
                false
            )
            .records
            .is_empty());
    }

    #[test]
    fn exclusion_change_bypasses_the_position_throttle() {
        let mut r = recorder_with_snapshot();
        r.on_status(
            &exclude_update(
                101.0,
                &json!({"objects": [{"name": "A"}], "excluded_objects": [], "current_object": null}),
            ),
            2_000,
            false,
        );
        // Well inside POSITION_CONTEXT_MIN_NS of the previous context: a
        // position-only advance would be swallowed, an exclusion is not.
        let out = r.on_status(
            &status(json!({"eventtime": 101.1, "status": {
                "virtual_sdcard": {"file_position": 1_100},
                "exclude_object": {"excluded_objects": ["A"]},
            }})),
            2_001,
            false,
        );
        let ctx = only_context(&out);
        assert_eq!(ctx.exclude.clone().unwrap().excluded, vec!["A".to_owned()]);
        assert_eq!(ctx.virtual_sdcard.as_ref().unwrap().file_position, 1_100);
    }

    #[test]
    fn exclude_object_reset_and_define_reset_are_journaled() {
        let mut r = recorder_with_snapshot();
        r.on_status(
            &exclude_update(
                101.0,
                &json!({"objects": [{"name": "A"}, {"name": "B"}],
                       "excluded_objects": ["A", "B"], "current_object": null}),
            ),
            2_000,
            false,
        );
        // EXCLUDE_OBJECT RESET=1: exclusions clear, definitions stay.
        let out = r.on_status(
            &exclude_update(102.0, &json!({"excluded_objects": []})),
            3_000,
            false,
        );
        let state = only_context(&out).exclude.clone().unwrap();
        assert!(state.excluded.is_empty());
        assert_eq!(state.definitions, None);

        // EXCLUDE_OBJECT_DEFINE RESET=1 runs _reset_file(): everything
        // clears at once, and the empty definition list is journaled
        // explicitly (Some(vec![]) != None).
        let out = r.on_status(
            &exclude_update(
                103.0,
                &json!({"objects": [], "excluded_objects": [], "current_object": null}),
            ),
            4_000,
            false,
        );
        let state = only_context(&out).exclude.clone().unwrap();
        assert_eq!(state.definitions, Some(Vec::new()));
        assert!(state.excluded.is_empty());
    }

    #[test]
    fn current_object_changes_ride_along_without_triggering() {
        let mut r = recorder_with_snapshot();
        r.on_status(
            &exclude_update(
                101.0,
                &json!({"objects": [{"name": "A"}, {"name": "B"}],
                       "excluded_objects": [], "current_object": null}),
            ),
            2_000,
            false,
        );
        // EXCLUDE_OBJECT_START NAME=A: one per object per layer. If this
        // triggered, an N-object M-layer print would cost N*M fsyncs.
        let out = r.on_status(
            &exclude_update(101.2, &json!({"current_object": "A"})),
            2_100,
            false,
        );
        assert!(out.records.is_empty(), "current_object must not trigger");
        // ... but it rides along on the next real trigger.
        let out = r.on_status(
            &status(json!({"eventtime": 101.3, "status": {"fan": {"speed": 0.4}}})),
            2_200,
            false,
        );
        assert_eq!(
            only_context(&out).exclude.clone().unwrap().current,
            Some("A".to_owned())
        );
        // EXCLUDE_OBJECT_END sets it back to JSON null.
        let out = r.on_status(
            &exclude_update(101.4, &json!({"current_object": null})),
            2_300,
            false,
        );
        assert!(out.records.is_empty());
    }

    #[test]
    fn new_object_definitions_appearing_mid_print_are_journaled() {
        // EXCLUDE_OBJECT_START NAME=<unknown> auto-defines a name-only
        // object, so `objects` grows mid-print
        // (exclude_object.py:199-204).
        let mut r = recorder_with_snapshot();
        r.on_status(
            &exclude_update(
                101.0,
                &json!({"objects": [{"name": "A"}], "excluded_objects": [], "current_object": null}),
            ),
            2_000,
            false,
        );
        let out = r.on_status(
            &exclude_update(102.0, &json!({"objects": [{"name": "A"}, {"name": "B"}]})),
            3_000,
            false,
        );
        let definitions = only_context(&out)
            .exclude
            .clone()
            .unwrap()
            .definitions
            .unwrap();
        assert_eq!(definitions.len(), 2);
    }

    #[test]
    fn hostile_geometry_is_normalized_and_keeps_the_record_writable() {
        let mut r = recorder_with_snapshot();
        let mut long_polygon: Vec<serde_json::Value> = Vec::new();
        for i in 0..=plr_wal::MAX_POLYGON_POINTS {
            #[allow(clippy::cast_precision_loss)]
            let t = i as f64;
            long_polygon.push(json!([t, 2.0 * t]));
        }
        let out = r.on_status(
            &exclude_update(
                101.0,
                &json!({
                    "objects": [
                        {"name": "BADPOINT", "polygon": [[0.0, 0.0], [1.0], [2.0, 2.0]]},
                        {"name": "SHORT", "polygon": [[0.0, 0.0], [1.0, 1.0]]},
                        {"name": "HUGE", "polygon": long_polygon},
                        {"name": "BADCENTER", "center": [0.0]}
                    ],
                    "excluded_objects": ["BADPOINT"],
                    "current_object": null
                }),
            ),
            2_000,
            false,
        );
        let ctx = only_context(&out);
        let definitions = ctx.exclude.clone().unwrap().definitions.unwrap();
        // A one-component point invalidates the ring rather than being
        // dropped: a partial ring encloses the wrong region. (A literal
        // NaN cannot reach us — JSON has no NaN token and serde_json
        // rejects one — so arity is the realistic corruption.)
        assert_eq!(
            definitions[0].fidelity,
            PolygonFidelity::Unusable { source_points: 3 }
        );
        assert!(definitions[0].polygon.is_empty());
        assert_eq!(
            definitions[1].fidelity,
            PolygonFidelity::Unusable { source_points: 2 }
        );
        // Over-long outlines become their bounding box, flagged.
        let expected_points = u32::try_from(plr_wal::MAX_POLYGON_POINTS + 1).unwrap();
        assert_eq!(
            definitions[2].fidelity,
            PolygonFidelity::BoundingBox {
                source_points: expected_points
            }
        );
        #[allow(clippy::cast_precision_loss)]
        let max = plr_wal::MAX_POLYGON_POINTS as f64;
        assert_eq!(
            definitions[2].polygon,
            vec![[0.0, 0.0], [max, 0.0], [max, 2.0 * max], [0.0, 2.0 * max]]
        );
        assert_eq!(definitions[3].center, None);
        // The excluded set survived intact and the record is writable.
        assert_eq!(
            ctx.exclude.clone().unwrap().excluded,
            vec!["BADPOINT".to_owned()]
        );
        assert!(WalRecord::Context(ctx.clone()).values_are_finite());
    }

    #[test]
    fn session_reset_drops_exclude_state_and_re_journals_definitions() {
        let mut r = recorder_with_snapshot();
        r.on_status(
            &exclude_update(
                101.0,
                &json!({"objects": [{"name": "A"}], "excluded_objects": ["A"],
                       "current_object": null}),
            ),
            2_000,
            false,
        );
        r.reset_session();
        // Klipper cleared exclude_object across the restart; the fresh
        // baseline must re-journal from scratch, not inherit "A".
        let out = r.on_status(
            &exclude_update(
                150.0,
                &json!({"objects": [], "excluded_objects": [], "current_object": null}),
            ),
            50_000,
            false,
        );
        let state = only_context(&out).exclude.clone().unwrap();
        assert_eq!(state.definitions, Some(Vec::new()));
        assert!(state.excluded.is_empty());
    }

    #[test]
    fn exclude_state_is_held_until_a_context_can_be_built() {
        // Before a full gcode_move state exists no context is written;
        // the pending definitions must not be lost.
        let mut r = Recorder::new();
        let out = r.on_status(
            &exclude_update(
                1.0,
                &json!({"objects": [{"name": "A"}], "excluded_objects": ["A"],
                       "current_object": null}),
            ),
            100,
            false,
        );
        assert!(out.records.is_empty());
        let out = r.on_initial_status(&initial_status(100.0), 1_000).unwrap();
        let state = only_context(&out).exclude.clone().unwrap();
        assert_eq!(
            state.definitions,
            Some(vec![ExcludeObjectDef::name_only("A")])
        );
        assert_eq!(state.excluded, vec!["A".to_owned()]);
    }

    #[test]
    fn malformed_exclude_object_payload_is_ignored() {
        // A wrong-shaped object must not kill the recording session.
        let mut r = recorder_with_snapshot();
        let out = r.on_status(
            &status(json!({"eventtime": 101.0, "status": {
                "exclude_object": {"objects": "not-a-list"},
            }})),
            2_000,
            false,
        );
        assert!(out.records.is_empty());
    }

    #[test]
    fn eventtime_conversion_is_total() {
        assert_eq!(eventtime_to_ns(1.5), Some(1_500_000_000));
        assert_eq!(eventtime_to_ns(0.0), Some(0));
        assert_eq!(eventtime_to_ns(-1.0), None);
        assert_eq!(eventtime_to_ns(f64::NAN), None);
        assert_eq!(eventtime_to_ns(f64::INFINITY), None);
        assert_eq!(eventtime_to_ns(1e300), None);
    }
}
