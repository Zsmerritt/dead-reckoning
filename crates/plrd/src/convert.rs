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
//! | `exclude_object`, `idle_timeout`, `probe`   | observed only — no `Context` field today; subscribed for forward compatibility |
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
//!
//! # Clean-shutdown detection
//!
//! Keyed on `virtual_sdcard.is_active` transitioning true→false in a
//! status update where, after merging that same update, either the file
//! reader reached the end of the file (`file_position >= file_size`,
//! complete) or the loaded file was cleared (`file_path == null`,
//! cancel via `SDCARD_RESET_FILE`). A pause (`is_active` false, file
//! still loaded mid-file) does **not** count.

use std::collections::BTreeMap;

use plr_klipper::{
    ClockCorrelator, GcodeMoveStatus, Notification, ReceiveSeqWidener, ResponseTemplate,
    SampleOutcome, SeqKind, StatusUpdate, StepperBatch, TrapqBatch, VirtualSdcardStatus,
};
use plr_wal::{
    Context, FanTarget, GcodeState, HeaterTarget, StepChunk, StepperRange, TransformObservations,
    TrapqSegment, VirtualSdState, WalRecord,
};
use serde_json::{Map, Value};

use crate::sender::{HeartbeatData, SyncPolicy};

/// The response-template key used to demultiplex subscriptions. Klipper
/// echoes the template at the top level of every asynchronous message and
/// does **not** name the producing endpoint, so every subscription gets a
/// distinct value under this key.
pub const TEMPLATE_KEY: &str = "k";

/// Minimum spacing of contexts triggered *only* by file-position advance.
pub const POSITION_CONTEXT_MIN_NS: u64 = 1_000_000_000;

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
}

/// Stateful converter: merges Klipper's diff-style status stream into a
/// full snapshot and emits WAL records per the module-level mapping.
///
/// Pure logic — no I/O, no clocks. The caller supplies `mono_ns`
/// (host `CLOCK_MONOTONIC`, the same clock Klipper's reactor `eventtime`
/// runs on) with every message.
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
    transforms: TransformObservations,
    heaters: BTreeMap<String, f64>,
    fans: BTreeMap<String, f64>,
    latest_print_time: f64,
    est_sample: Option<(u64, f64)>,
    last_context_mono_ns: Option<u64>,
    last_context_file_position: u64,
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
            est_sample: None,
            last_context_mono_ns: None,
            last_context_file_position: 0,
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
    pub fn reset_session(&mut self) {
        self.is_active = false;
        self.est_sample = None;
        self.latest_print_time = 0.0;
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
        let mut position_advanced = false;
        if let Ok(Some(vsd)) = update.status.virtual_sdcard() {
            let vsd_result = self.merge_virtual_sdcard(&vsd);
            state_changed |= vsd_result.path_changed;
            position_advanced = vsd_result.position_advanced;
            out.clean_shutdown = vsd_result.clean_shutdown;
        }
        state_changed |= self.merge_heaters_and_fans(update);
        state_changed |= self.merge_transforms(update);

        let position_due = self
            .last_context_mono_ns
            .is_none_or(|last| mono_ns.saturating_sub(last) >= POSITION_CONTEXT_MIN_NS);
        let position_trigger = position_advanced
            && self.file_position != self.last_context_file_position
            && position_due;
        if state_changed || position_trigger {
            if let Some(context) = self.build_context(mono_ns) {
                out.records
                    .push((WalRecord::Context(context), SyncPolicy::Immediate));
                self.last_context_mono_ns = Some(mono_ns);
                self.last_context_file_position = self.file_position;
            }
        }
        out.heartbeat = self.heartbeat_data();
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
        out.heartbeat = self.heartbeat_data();
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
        out.heartbeat = self.heartbeat_data();
        out
    }

    /// The current heartbeat payload; `None` until a correlation sample
    /// (`estimated_print_time` + `eventtime`) has been observed — no
    /// liveness claim without one.
    #[must_use]
    pub fn heartbeat_data(&self) -> Option<HeartbeatData> {
        self.est_sample.map(
            |(est_sample_mono_ns, est_sample_print_time)| HeartbeatData {
                print_time: self.latest_print_time,
                est_sample_mono_ns,
                est_sample_print_time,
            },
        )
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

    fn merge_virtual_sdcard(&mut self, vsd: &VirtualSdcardStatus) -> VsdMerge {
        let mut merge = VsdMerge::default();
        if let Some(path) = &vsd.file_path {
            // Outer Option: present in this diff. Inner: nullable value.
            if *path != self.file_path {
                merge.path_changed = true;
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
            if self.is_active && !active {
                let complete = self.file_size > 0 && self.file_position >= self.file_size;
                let cancelled = self.file_path.is_none();
                merge.clean_shutdown = complete || cancelled;
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

    /// Builds a context snapshot; `None` until a full `gcode_move` state
    /// has been seen (always present after the initial subscribe
    /// response).
    fn build_context(&self, mono_ns: u64) -> Option<Context> {
        let gcode = self.gcode.clone()?;
        Some(Context {
            mono_ns,
            virtual_sdcard: self.file_path.clone().map(|file_path| VirtualSdState {
                file_path,
                file_position: self.file_position,
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
        })
    }
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
    use plr_wal::WalRecord;
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
        assert!(r.heartbeat_data().is_some());
        r.reset_session();
        assert!(
            r.heartbeat_data().is_none(),
            "no liveness claim from a dead session's sample"
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
