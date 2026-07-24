//! Machine configuration sourced from Klipper's `[plr]` printer.cfg
//! section, via the `configfile` status object.
//!
//! # Where the values come from
//!
//! Klipper's `configfile` status object (`klippy/configfile.py`,
//! `PrinterConfig.get_status`) exposes:
//!
//! * `settings` — `ConfigValidate.get_status`: every option every
//!   module *accessed* while loading the config, as
//!   `{section: {option: typed_value}}` with section/option names
//!   lowercased (`ConfigValidate._build_status_settings` builds it from
//!   the access-tracking map). Values carry the parsed type
//!   (`getfloat` → number, `getboolean` → bool, `get` → string), and —
//!   crucially — the autosave (`SAVE_CONFIG`) block is merged into the
//!   file config before access tracking starts
//!   (`ConfigAutoSave.load_main_config`), so autosaved options such as
//!   `[plr] self_locking_z` appear here as ordinary settings after a
//!   restart.
//! * `config` — the raw (string-valued) file config; not used here:
//!   `settings` already carries the typed, default-resolved view.
//!
//! The daemon queries `configfile` (plus the plugin's `plr` status
//! object) over the Klipper API socket at recover time, so the machine
//! snapshot is **the running config**, every run — which is exactly why
//! the crc32c config-hash blessing of the legacy `/etc/plrd.conf`
//! `[machine]` path is unnecessary in `[plr]` mode (see
//! [`LIVE_CONFIG_HASH`]).
//!
//! # Defense in depth
//!
//! The plugin's `PLR_SETUP` shows the operator the same derivations
//! (primary-MCU Z, empty `activate_gcode`, single probe); plrd
//! re-derives every one of them independently from the settings here.
//! A plugin bug cannot bless a machine the daemon would refuse.

use std::collections::BTreeMap;

use plr_recovery::{MachineConfig, PlanConfig, ProbeConfig, ProbeKind, ZStepper};
use serde_json::{Map, Value};

/// Sentinel used for both `config_hash` and `validated_config_hash` in
/// `[plr]` mode: the machine snapshot is read from the live config on
/// every run, so the change-detection check is satisfied by
/// construction and the equality check passes trivially. The legacy
/// `[machine]` path keeps its real crc32c blessing.
pub const LIVE_CONFIG_HASH: &str = "live:[plr]";

/// The name of the primary MCU in Klipper (`[mcu]`).
const PRIMARY_MCU: &str = "mcu";

/// The typed `[plr]` section, parsed from `configfile.settings.plr`.
///
/// `probe_method` is required; every other key falls back to the same
/// default the plugin's config parsing uses, so a settings map from an
/// older plugin still reads coherently. Wrong-typed values are hard
/// errors — a `[plr]` section that cannot be read faithfully must
/// refuse, never guess.
#[derive(Debug, Clone, PartialEq)]
pub struct PlrSettings {
    /// `tap`, `load_cell`, or `adxl_drag`.
    pub probe_method: String,
    /// Accelerometer chip for `adxl_drag`; may be empty.
    pub accel_chip: String,
    /// The plugin's view of the WAL directory (informational here; the
    /// daemon's own recorder path comes from /etc/plrd.conf).
    pub wal_dir: String,
    /// The control-socket path the plugin connects to. Must match the
    /// daemon's `control_socket` in /etc/plrd.conf.
    pub control_socket: String,
    /// Probe speed, mm/s (continuous descent methods).
    pub probe_speed: f64,
    /// Envelope margin, mm.
    pub envelope_margin: f64,
    /// Sag allowance, mm.
    pub sag_allowance: f64,
    /// Drag pass XY speed, mm/s.
    pub drag_speed: f64,
    /// Drag staircase Z decrement, mm.
    pub drag_z_step: f64,
    /// Drag contact threshold as a multiple of the noise floor.
    pub drag_sensitivity: f64,
    /// Contact-selection exclusion radius around the crash point, mm.
    pub exclusion_radius: f64,
    /// Entry-move feedrate, mm/min.
    pub entry_feedrate: f64,
    /// Operator attestation autosaved by the plugin's `PLR_SETUP`.
    pub self_locking_z: bool,
    /// Autosaved probe resolution, mm; `None` before first calibration.
    pub probe_resolution: Option<f64>,
    /// Every autosaved `noise_floor_*` option (the plugin's future
    /// `PLR_NOISE_TEST` writes these). Non-empty means the noise floor
    /// is calibrated.
    pub noise_floor: BTreeMap<String, f64>,
}

/// Reads a required-or-defaulted f64 option.
fn opt_f64(section: &Map<String, Value>, key: &str, default: f64) -> Result<f64, String> {
    match section.get(key) {
        None => Ok(default),
        Some(v) => v
            .as_f64()
            .ok_or_else(|| format!("[plr] {key} is not a number: {v}")),
    }
}

/// Reads a defaulted string option.
fn opt_str(section: &Map<String, Value>, key: &str, default: &str) -> Result<String, String> {
    match section.get(key) {
        None => Ok(default.to_owned()),
        Some(v) => v
            .as_str()
            .map(str::to_owned)
            .ok_or_else(|| format!("[plr] {key} is not a string: {v}")),
    }
}

impl PlrSettings {
    /// Parses `configfile.settings.plr`. See the type docs for the
    /// required/optional split.
    pub fn parse(plr: &Map<String, Value>) -> Result<Self, String> {
        let probe_method = plr
            .get("probe_method")
            .ok_or_else(|| "[plr] probe_method is missing".to_owned())?
            .as_str()
            .ok_or_else(|| "[plr] probe_method is not a string".to_owned())?
            .to_owned();
        if !matches!(probe_method.as_str(), "tap" | "load_cell" | "adxl_drag") {
            return Err(format!(
                "[plr] probe_method {probe_method:?} is not tap, load_cell, or adxl_drag"
            ));
        }
        let mut noise_floor = BTreeMap::new();
        for (key, value) in plr {
            if let Some(_suffix) = key.strip_prefix("noise_floor_") {
                let number = value
                    .as_f64()
                    .ok_or_else(|| format!("[plr] {key} is not a number: {value}"))?;
                noise_floor.insert(key.clone(), number);
            }
        }
        let probe_resolution = match plr.get("probe_resolution") {
            None => None,
            Some(v) => Some(
                v.as_f64()
                    .ok_or_else(|| format!("[plr] probe_resolution is not a number: {v}"))?,
            ),
        };
        let self_locking_z = match plr.get("self_locking_z") {
            None => false,
            Some(v) => v
                .as_bool()
                .ok_or_else(|| format!("[plr] self_locking_z is not a boolean: {v}"))?,
        };
        // Numeric tunables default to the plan builder's defaults so an
        // older plugin (fewer keys) still yields the documented
        // behavior.
        let d = PlanConfig::default();
        Ok(Self {
            probe_method,
            accel_chip: opt_str(plr, "accel_chip", "")?,
            wal_dir: opt_str(plr, "wal_dir", "/var/lib/plrd/wal")?,
            control_socket: opt_str(plr, "control_socket", "/var/lib/plrd/plrd.sock")?,
            probe_speed: opt_f64(plr, "probe_speed", d.probe_speed)?,
            envelope_margin: opt_f64(plr, "envelope_margin", d.margin)?,
            sag_allowance: opt_f64(plr, "sag_allowance", d.sag_allowance)?,
            drag_speed: opt_f64(plr, "drag_speed", d.drag_speed)?,
            drag_z_step: opt_f64(plr, "drag_z_step", d.drag_z_step)?,
            drag_sensitivity: opt_f64(plr, "drag_sensitivity", d.drag_sensitivity)?,
            exclusion_radius: opt_f64(plr, "exclusion_radius", 5.0)?,
            entry_feedrate: opt_f64(plr, "entry_feedrate", d.entry_feed)?,
            self_locking_z,
            probe_resolution,
            noise_floor,
        })
    }

    /// The representative calibrated noise floor, if any:
    /// `noise_floor_rms` when present (the primary calibration value),
    /// else the first `noise_floor_*` key in sorted order. The exact
    /// key set firms up with the plugin's `PLR_NOISE_TEST`; validation
    /// only needs "calibrated, finite, positive".
    #[must_use]
    pub fn representative_noise_floor(&self) -> Option<f64> {
        self.noise_floor
            .get("noise_floor_rms")
            .or_else(|| self.noise_floor.values().next())
            .copied()
    }

    /// The plan-builder tunables carried by `[plr]`. Out-of-band values
    /// are *not* clamped here: `plan_recovery` validates the config and
    /// refuses, which is the honest failure.
    #[must_use]
    pub fn plan_config(&self) -> PlanConfig {
        PlanConfig {
            probe_speed: self.probe_speed,
            margin: self.envelope_margin,
            sag_allowance: self.sag_allowance,
            entry_feed: self.entry_feedrate,
            drag_speed: self.drag_speed,
            drag_z_step: self.drag_z_step,
            drag_sensitivity: self.drag_sensitivity,
            ..PlanConfig::default()
        }
    }
}

/// The MCU a config pin lives on: strip the `!`/`^`/`~`/`*` modifier
/// prefixes, then everything before a `:` names the MCU; a bare pin is
/// on the primary (`klippy/pins.py`, `parse_pin`/`lookup_pin` — pins
/// are written `[chip_name:]pin` with optional modifier prefixes).
fn pin_mcu(pin: &str) -> String {
    let trimmed = pin.trim_start_matches(['!', '^', '~', '*']);
    match trimmed.split_once(':') {
        Some((mcu, _)) => mcu.trim().to_owned(),
        None => PRIMARY_MCU.to_owned(),
    }
}

/// `true` when a section name is a Z stepper: `stepper_z` or
/// `stepper_z<N>`.
fn is_z_stepper(section: &str) -> bool {
    section
        .strip_prefix("stepper_z")
        .is_some_and(|suffix| suffix.is_empty() || suffix.bytes().all(|b| b.is_ascii_digit()))
}

/// A gcode-shaped option (e.g. `activate_gcode`) is "verified no-move"
/// here only when it is empty: plrd does not simulate arbitrary gcode,
/// so anything non-empty must be attested through the legacy path or
/// emptied. Missing counts as empty (Klipper defaults these to "").
fn gcode_option_empty(section: &Map<String, Value>, key: &str) -> bool {
    section
        .get(key)
        .and_then(Value::as_str)
        .is_none_or(|s| s.trim().is_empty())
}

/// One probe-ish config section mapped onto [`ProbeConfig`].
fn probe_from_section(section: &Map<String, Value>, kind: ProbeKind) -> ProbeConfig {
    ProbeConfig {
        kind,
        // A missing z_offset is represented as NaN so validation
        // reports ProbeZOffsetNonFinite instead of inventing a zero.
        z_offset: section
            .get("z_offset")
            .and_then(Value::as_f64)
            .unwrap_or(f64::NAN),
        activate_gcode_no_move: gcode_option_empty(section, "activate_gcode"),
        deactivate_gcode_no_move: gcode_option_empty(section, "deactivate_gcode"),
    }
}

/// Assembles the recovery `MachineConfig` from the full
/// `configfile.settings` map (the `[plr]` section already parsed).
/// Returns the config plus human-readable assembly notes.
///
/// Total by design: missing cross-check data becomes the
/// `MachineConfig` shape that *fails validation* (empty probe list,
/// `None` `position_min`, ...) rather than an error here — so the
/// operator sees every problem in one `validate_machine` report.
pub fn machine_from_settings(
    settings: &Map<String, Value>,
    plr: &PlrSettings,
    type_annotations_present: bool,
) -> (MachineConfig, Vec<String>) {
    let mut notes = Vec::new();
    let section = |name: &str| settings.get(name).and_then(Value::as_object);

    // Z steppers with their MCUs, derived from each section's step_pin.
    let mut z_steppers: Vec<ZStepper> = Vec::new();
    for (name, value) in settings {
        if !is_z_stepper(name) {
            continue;
        }
        let mcu = value
            .as_object()
            .and_then(|s| s.get("step_pin"))
            .and_then(Value::as_str)
            .map_or_else(
                || {
                    notes.push(format!(
                        "section [{name}] has no step_pin; assuming {PRIMARY_MCU}"
                    ));
                    PRIMARY_MCU.to_owned()
                },
                pin_mcu,
            );
        z_steppers.push(ZStepper {
            name: name.clone(),
            mcu,
        });
    }

    // position_min: the Z rail's, falling back to [printer]
    // minimum_z_position.
    let z_position_min = section("stepper_z")
        .and_then(|s| s.get("position_min"))
        .and_then(Value::as_f64)
        .or_else(|| {
            section("printer")
                .and_then(|s| s.get("minimum_z_position"))
                .and_then(Value::as_f64)
        });

    // Probes, per the [plr] probe_method (authoritative — see below).
    let probes = match plr.probe_method.as_str() {
        "adxl_drag" => {
            // The nozzle is the stylus: no z_offset, no probe gcode. A
            // coexisting [probe] section (kept for ordinary bed
            // leveling) is deliberately NOT counted as a second probe:
            // probe_method selects what recovery uses.
            vec![ProbeConfig {
                kind: ProbeKind::AdxlDrag {
                    chip: plr.accel_chip.clone(),
                },
                z_offset: 0.0,
                activate_gcode_no_move: true,
                deactivate_gcode_no_move: true,
            }]
        }
        method => {
            // The method's own section first (validation reads
            // probes[0]); the other traditional section, if ALSO
            // present, is appended so the single-probe check reports
            // the inconsistency.
            let tap = ("probe", ProbeKind::Tap);
            let load_cell = ("load_cell_probe", ProbeKind::LoadCell);
            let ordered = if method == "load_cell" {
                [load_cell, tap]
            } else {
                [tap, load_cell]
            };
            ordered
                .into_iter()
                .filter_map(|(name, kind)| section(name).map(|s| probe_from_section(s, kind)))
                .collect()
        }
    };

    let virtual_sdcard_root = section("virtual_sdcard")
        .and_then(|s| s.get("path"))
        .and_then(Value::as_str)
        .map(str::to_owned);
    if virtual_sdcard_root
        .as_deref()
        .is_some_and(|r| r.starts_with('~'))
    {
        notes.push(
            "[virtual_sdcard] path uses `~`; plrd cannot expand another user's home — \
             use an absolute path or recovery will refuse the file as non-top-level"
                .to_owned(),
        );
    }

    let machine = MachineConfig {
        force_move_enabled: section("force_move")
            .and_then(|s| s.get("enable_force_move"))
            .and_then(Value::as_bool)
            .unwrap_or(false),
        z_self_locking_attested: plr.self_locking_z,
        z_steppers,
        primary_mcu: PRIMARY_MCU.to_owned(),
        type_annotations_present,
        probes,
        z_position_min,
        // Live-config mode: change detection is satisfied by
        // construction (module docs); both sides carry the sentinel so
        // the equality check passes trivially.
        config_hash: LIVE_CONFIG_HASH.to_owned(),
        validated_config_hash: Some(LIVE_CONFIG_HASH.to_owned()),
        virtual_sdcard_root,
        noise_floor: plr.representative_noise_floor(),
    };
    (machine, notes)
}

/// What one live query of the Klipper API socket returned.
#[derive(Debug, Clone)]
pub struct KlippySnapshot {
    /// `configfile.settings`: `{section: {option: typed value}}`.
    pub settings: Map<String, Value>,
    /// The plugin's `plr` status object, when the plugin is loaded
    /// (`{method, configured, attested, probe_resolution,
    /// daemon_alive}` plus `last_drag_result` after a drag probe).
    pub plr_object: Option<Map<String, Value>>,
}

impl KlippySnapshot {
    /// Extracts the snapshot from an `objects/query` result. (Off-Unix
    /// only tests reach this: the stub query cannot produce a result.)
    #[cfg_attr(not(unix), allow(dead_code))]
    pub fn from_query_result(result: &Value) -> Result<Self, String> {
        let status = result
            .get("status")
            .and_then(Value::as_object)
            .ok_or_else(|| "objects/query result has no status map".to_owned())?;
        let settings = status
            .get("configfile")
            .and_then(|c| c.get("settings"))
            .and_then(Value::as_object)
            .cloned()
            .ok_or_else(|| {
                "configfile.settings absent from the query result (klippy still starting?)"
                    .to_owned()
            })?;
        let plr_object = status
            .get("plr")
            .and_then(Value::as_object)
            .filter(|m| !m.is_empty())
            .cloned();
        Ok(Self {
            settings,
            plr_object,
        })
    }

    /// The `[plr]` settings section, when the config has one.
    #[must_use]
    pub fn plr_section(&self) -> Option<&Map<String, Value>> {
        self.settings.get("plr").and_then(Value::as_object)
    }
}

/// Queries `configfile` + `plr` over the Klipper API socket (Unix
/// stream, `0x03`-framed JSON — the same wire `client.rs` records
/// from, but as a one-shot blocking call: recover-time code is
/// synchronous and must not entangle itself with the recorder's
/// session).
#[cfg(unix)]
pub fn query_klippy_snapshot(
    socket: &std::path::Path,
    timeout: std::time::Duration,
) -> Result<KlippySnapshot, String> {
    use std::io::{Read as _, Write as _};

    use plr_klipper::{classify, FrameEvent, FrameSplitter, Inbound, Request, SubscriptionObjects};

    let mut objects = SubscriptionObjects::new();
    objects.insert("configfile".to_owned(), None);
    objects.insert("plr".to_owned(), None);
    let frame = Request::ObjectsQuery { objects }
        .to_frame(1)
        .map_err(|e| format!("cannot encode objects/query: {e}"))?;

    let mut stream = std::os::unix::net::UnixStream::connect(socket)
        .map_err(|e| format!("cannot connect to klippy at {}: {e}", socket.display()))?;
    stream
        .set_read_timeout(Some(timeout))
        .and_then(|()| stream.set_write_timeout(Some(timeout)))
        .map_err(|e| format!("cannot set socket timeouts: {e}"))?;
    stream
        .write_all(&frame)
        .map_err(|e| format!("cannot send objects/query to klippy: {e}"))?;

    let deadline = std::time::Instant::now() + timeout;
    let mut splitter = FrameSplitter::new();
    let mut buf = vec![0_u8; 64 * 1024];
    loop {
        if std::time::Instant::now() >= deadline {
            return Err(format!(
                "klippy at {} did not answer objects/query within {timeout:?}",
                socket.display()
            ));
        }
        let n = stream
            .read(&mut buf)
            .map_err(|e| format!("read from klippy failed: {e}"))?;
        if n == 0 {
            return Err("klippy closed the socket mid-query".to_owned());
        }
        for event in splitter.feed(&buf[..n]) {
            let FrameEvent::Frame(frame) = event else {
                continue; // oversized frames cannot be ours
            };
            match classify(&frame) {
                // Subscription noise from other clients does not reach
                // this fresh connection; only our reply matters.
                Ok(Inbound::Response { id: 1, result }) => {
                    return KlippySnapshot::from_query_result(&result);
                }
                Ok(Inbound::Error { id: 1, error }) => {
                    return Err(format!(
                        "klippy rejected objects/query: {:?}",
                        error.message
                    ));
                }
                Ok(_) | Err(_) => {}
            }
        }
    }
}

/// Non-Unix stub: there is no Unix-socket transport, so the snapshot is
/// always unavailable and callers take their documented
/// klippy-unreachable path (legacy fallback or refusal).
#[cfg(not(unix))]
pub fn query_klippy_snapshot(
    socket: &std::path::Path,
    _timeout: std::time::Duration,
) -> Result<KlippySnapshot, String> {
    Err(format!(
        "cannot query klippy at {}: unix sockets are unsupported on this platform",
        socket.display()
    ))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::{machine_from_settings, pin_mcu, KlippySnapshot, PlrSettings, LIVE_CONFIG_HASH};
    use plr_recovery::{validate_machine, PrereqFailure, ProbeKind};
    use serde_json::{json, Map, Value};

    /// A realistic `configfile` status body for a `[plr]`-commissioned
    /// tap machine. Shape per `klippy/configfile.py`:
    /// `PrinterConfig.get_status` returns `config` (raw strings),
    /// `settings` (typed, access-tracked, **autosave-merged** — note
    /// `self_locking_z`/`probe_resolution` present as ordinary
    /// settings), `warnings`, and the save-pending fields.
    pub(crate) fn configfile_status(plr_overrides: &[(&str, Value)]) -> Value {
        let mut plr = json!({
            "probe_method": "tap",
            "accel_chip": "",
            "wal_dir": "/var/lib/plrd/wal",
            "control_socket": "/var/lib/plrd/plrd.sock",
            "probe_speed": 1.0,
            "envelope_margin": 0.5,
            "sag_allowance": 0.2,
            "drag_speed": 5.0,
            "drag_z_step": 0.05,
            "drag_sensitivity": 3.0,
            "exclusion_radius": 5.0,
            "entry_feedrate": 1200.0,
            // SAVE_CONFIG-persisted: merged into settings after
            // restart (ConfigAutoSave.load_main_config).
            "self_locking_z": true,
            "probe_resolution": 0.0125
        });
        for (key, value) in plr_overrides {
            plr[*key] = value.clone();
        }
        json!({
            "config": {"printer": {"kinematics": "corexy"}},
            "warnings": [],
            "save_config_pending": false,
            "save_config_pending_items": {},
            "settings": {
                "plr": plr,
                "printer": {
                    "kinematics": "corexy",
                    "max_velocity": 300.0,
                    "minimum_z_position": -3.0
                },
                "force_move": {"enable_force_move": true},
                "stepper_z": {
                    "step_pin": "PB0",
                    "position_min": -2.0,
                    "position_max": 250.0
                },
                "stepper_z1": {"step_pin": "!PB3"},
                "probe": {
                    "z_offset": -0.1,
                    "activate_gcode": "",
                    "deactivate_gcode": ""
                },
                "virtual_sdcard": {"path": "/home/pi/printer_data/gcodes"}
            }
        })
    }

    // Test-helper ergonomics: callers hand over the fixture values.
    #[allow(clippy::needless_pass_by_value)]
    pub(crate) fn query_result(configfile: Value, plr_object: Value) -> Value {
        json!({
            "eventtime": 100.5,
            "status": {"configfile": configfile, "plr": plr_object}
        })
    }

    /// A minimal fake klippy API socket (Unix only): serves every
    /// connection the same `objects/query` response, over the real
    /// 0x03-framed wire. The accept thread is detached; it dies with
    /// the process.
    #[cfg(unix)]
    pub(crate) fn spawn_fake_klippy(tag: &str, response: Value) -> std::path::PathBuf {
        use std::io::{Read as _, Write as _};
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-fake-klippy-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("klippy.sock");
        let listener = std::os::unix::net::UnixListener::bind(&path).unwrap();
        std::thread::spawn(move || {
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else { return };
                let response = response.clone();
                std::thread::spawn(move || {
                    let mut buf = Vec::new();
                    let mut chunk = [0_u8; 4096];
                    loop {
                        match stream.read(&mut chunk) {
                            Ok(0) | Err(_) => return,
                            Ok(n) => buf.extend_from_slice(&chunk[..n]),
                        }
                        if let Some(pos) = buf.iter().position(|&b| b == 0x03) {
                            let request: Value =
                                serde_json::from_slice(&buf[..pos]).unwrap_or(Value::Null);
                            let id = request.get("id").cloned().unwrap_or(Value::Null);
                            let mut reply =
                                serde_json::to_vec(&json!({"id": id, "result": response})).unwrap();
                            reply.push(0x03);
                            let _ = stream.write_all(&reply);
                            return;
                        }
                    }
                });
            }
        });
        path
    }

    #[cfg(unix)]
    #[test]
    fn live_query_round_trips_over_a_real_socket() {
        let response = query_result(configfile_status(&[]), plr_object());
        let path = spawn_fake_klippy("query", response);
        let snapshot =
            super::query_klippy_snapshot(&path, std::time::Duration::from_secs(5)).unwrap();
        assert!(snapshot.plr_section().is_some());
        assert!(snapshot.plr_object.is_some());
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        assert_eq!(plr.probe_method, "tap");
    }

    #[cfg(unix)]
    #[test]
    fn live_query_reports_unreachable_and_dead_sockets() {
        let err = super::query_klippy_snapshot(
            std::path::Path::new("/nonexistent-plrd/klippy.sock"),
            std::time::Duration::from_millis(200),
        )
        .unwrap_err();
        assert!(err.contains("cannot connect"), "{err}");
    }

    pub(crate) fn plr_object() -> Value {
        json!({
            "method": "tap",
            "configured": true,
            "attested": true,
            "probe_resolution": 0.0125,
            "daemon_alive": true
        })
    }

    fn parse_fixture(overrides: &[(&str, Value)]) -> (KlippySnapshot, PlrSettings) {
        let result = query_result(configfile_status(overrides), plr_object());
        let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        (snapshot, plr)
    }

    #[test]
    fn full_fixture_parses_and_validates() {
        let (snapshot, plr) = parse_fixture(&[]);
        assert_eq!(plr.probe_method, "tap");
        assert!(plr.self_locking_z);
        assert_eq!(plr.probe_resolution, Some(0.0125));
        assert!(plr.noise_floor.is_empty());
        assert_eq!(plr.control_socket, "/var/lib/plrd/plrd.sock");

        let (machine, notes) = machine_from_settings(&snapshot.settings, &plr, true);
        assert!(notes.is_empty(), "{notes:?}");
        assert!(machine.force_move_enabled);
        assert!(machine.z_self_locking_attested);
        // Both Z steppers discovered; MCUs derived from step_pin
        // (modifier prefixes stripped, bare pin = primary MCU).
        assert_eq!(machine.z_steppers.len(), 2);
        assert!(machine.z_steppers.iter().all(|s| s.mcu == "mcu"));
        assert_eq!(machine.probes.len(), 1);
        assert_eq!(machine.probes[0].kind, ProbeKind::Tap);
        assert!((machine.probes[0].z_offset - (-0.1)).abs() < 1e-12);
        assert!(machine.probes[0].activate_gcode_no_move);
        // The rail's own position_min wins over [printer].
        assert_eq!(machine.z_position_min, Some(-2.0));
        assert_eq!(
            machine.virtual_sdcard_root.as_deref(),
            Some("/home/pi/printer_data/gcodes")
        );
        // Live-config mode: change detection satisfied by construction.
        assert_eq!(machine.config_hash, LIVE_CONFIG_HASH);
        assert_eq!(
            machine.validated_config_hash.as_deref(),
            Some(LIVE_CONFIG_HASH)
        );
        assert!(validate_machine(&machine).is_ok());
    }

    #[test]
    fn plan_config_carries_the_plr_tunables() {
        let (_, plr) = parse_fixture(&[
            ("probe_speed", json!(1.5)),
            ("envelope_margin", json!(0.8)),
            ("sag_allowance", json!(0.3)),
            ("entry_feedrate", json!(900.0)),
            ("drag_z_step", json!(0.08)),
        ]);
        let config = plr.plan_config();
        assert!((config.probe_speed - 1.5).abs() < 1e-12);
        assert!((config.margin - 0.8).abs() < 1e-12);
        assert!((config.sag_allowance - 0.3).abs() < 1e-12);
        assert!((config.entry_feed - 900.0).abs() < 1e-12);
        assert!((config.drag_z_step - 0.08).abs() < 1e-12);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn adxl_drag_machine_builds_the_drag_probe() {
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
            ("noise_floor_rms", json!(118.0)),
            ("noise_floor_peak", json!(410.0)),
        ]);
        assert_eq!(plr.noise_floor.get("noise_floor_rms").copied(), Some(118.0));
        assert_eq!(plr.representative_noise_floor(), Some(118.0));
        let (machine, _) = machine_from_settings(&snapshot.settings, &plr, true);
        // probe_method is authoritative: the coexisting [probe]
        // section does not create a second probe object.
        assert_eq!(machine.probes.len(), 1);
        assert_eq!(
            machine.probes[0].kind,
            ProbeKind::AdxlDrag {
                chip: "adxl345".to_owned()
            }
        );
        assert_eq!(machine.noise_floor, Some(118.0));
        assert!(validate_machine(&machine).is_ok());
    }

    #[test]
    fn uncalibrated_drag_machine_fails_validation_with_the_hint() {
        let (snapshot, plr) = parse_fixture(&[
            ("probe_method", json!("adxl_drag")),
            ("accel_chip", json!("adxl345")),
        ]);
        assert!(plr.noise_floor.is_empty());
        let (machine, _) = machine_from_settings(&snapshot.settings, &plr, true);
        let rejection = validate_machine(&machine).unwrap_err();
        assert!(rejection
            .failures
            .contains(&PrereqFailure::NoiseFloorMissing));
    }

    #[test]
    fn missing_cross_check_sections_fail_validation_not_parsing() {
        // Strip every cross-check section: assembly stays total and
        // validation lists every gap.
        let (snapshot, plr) = parse_fixture(&[]);
        let mut settings = Map::new();
        settings.insert("plr".to_owned(), snapshot.settings["plr"].clone());
        let (machine, _) = machine_from_settings(&settings, &plr, false);
        let rejection = validate_machine(&machine).unwrap_err();
        let f = &rejection.failures;
        assert!(f.contains(&PrereqFailure::ForceMoveDisabled));
        assert!(f.contains(&PrereqFailure::NoZSteppers));
        assert!(f.contains(&PrereqFailure::NoProbe));
        assert!(f.contains(&PrereqFailure::PositionMinUnknown));
        assert!(f.contains(&PrereqFailure::SdcardRootUnknown));
        assert!(f.contains(&PrereqFailure::NoTypeAnnotations));
    }

    #[test]
    fn secondary_mcu_z_and_nonempty_activate_gcode_are_detected() {
        let result = query_result(configfile_status(&[]), plr_object());
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        snapshot.settings["stepper_z1"]["step_pin"] = json!("^!z_board:PA1");
        snapshot.settings["probe"]["activate_gcode"] = json!("G1 Z5\n");
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (machine, _) = machine_from_settings(&snapshot.settings, &plr, true);
        let rejection = validate_machine(&machine).unwrap_err();
        assert!(rejection.failures.iter().any(|f| matches!(
            f,
            PrereqFailure::ZStepperOffPrimaryMcu { stepper, mcu }
                if stepper == "stepper_z1" && mcu == "z_board"
        )));
        assert!(rejection
            .failures
            .contains(&PrereqFailure::ProbeActivateGcodeMoves));
    }

    #[test]
    fn both_probe_sections_present_is_a_multiple_probe_refusal() {
        let result = query_result(configfile_status(&[]), plr_object());
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        snapshot.settings.insert(
            "load_cell_probe".to_owned(),
            json!({"z_offset": -0.15, "activate_gcode": ""}),
        );
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (machine, _) = machine_from_settings(&snapshot.settings, &plr, true);
        // The method's section leads; the stray one still counts.
        assert_eq!(machine.probes[0].kind, ProbeKind::Tap);
        let rejection = validate_machine(&machine).unwrap_err();
        assert!(rejection
            .failures
            .contains(&PrereqFailure::MultipleProbes { count: 2 }));
    }

    #[test]
    fn load_cell_method_reads_its_own_section_first() {
        let result = query_result(
            configfile_status(&[("probe_method", json!("load_cell"))]),
            plr_object(),
        );
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        let settings = &mut snapshot.settings;
        let probe = settings.remove("probe").unwrap();
        let mut lc = probe.as_object().unwrap().clone();
        lc.insert("z_offset".to_owned(), json!(-0.15));
        settings.insert("load_cell_probe".to_owned(), Value::Object(lc));
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (machine, _) = machine_from_settings(&snapshot.settings, &plr, true);
        assert_eq!(machine.probes.len(), 1);
        assert_eq!(machine.probes[0].kind, ProbeKind::LoadCell);
        assert!((machine.probes[0].z_offset - (-0.15)).abs() < 1e-12);
    }

    #[test]
    fn tilde_sdcard_root_is_noted() {
        let result = query_result(configfile_status(&[]), plr_object());
        let mut snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        snapshot.settings["virtual_sdcard"]["path"] = json!("~/printer_data/gcodes");
        let plr = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap();
        let (_, notes) = machine_from_settings(&snapshot.settings, &plr, true);
        assert!(notes.iter().any(|n| n.contains('~')), "{notes:?}");
    }

    #[test]
    fn parse_rejects_bad_types_and_methods() {
        for (key, value, needle) in [
            ("probe_method", json!(7), "not a string"),
            ("probe_method", json!("laser"), "not tap, load_cell"),
            ("probe_speed", json!("fast"), "not a number"),
            ("self_locking_z", json!("yes"), "not a boolean"),
            ("accel_chip", json!(3), "not a string"),
            ("noise_floor_rms", json!("loud"), "not a number"),
            ("probe_resolution", json!(false), "not a number"),
        ] {
            let result = query_result(configfile_status(&[(key, value)]), plr_object());
            let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
            let err = PlrSettings::parse(snapshot.plr_section().unwrap()).unwrap_err();
            assert!(err.contains(needle), "{key}: {err}");
        }
        // A missing probe_method is required.
        let result = query_result(configfile_status(&[]), plr_object());
        let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        let mut section = snapshot.plr_section().unwrap().clone();
        section.remove("probe_method");
        assert!(PlrSettings::parse(&section)
            .unwrap_err()
            .contains("probe_method is missing"));
    }

    #[test]
    fn missing_optional_keys_fall_back_to_plan_defaults() {
        let minimal: Map<String, Value> =
            json!({"probe_method": "tap"}).as_object().unwrap().clone();
        let plr = PlrSettings::parse(&minimal).unwrap();
        assert!(!plr.self_locking_z);
        assert_eq!(plr.probe_resolution, None);
        let d = plr_recovery::PlanConfig::default();
        assert!((plr.probe_speed - d.probe_speed).abs() < 1e-12);
        assert!((plr.drag_z_step - d.drag_z_step).abs() < 1e-12);
        assert!(plr.plan_config().validate().is_ok());
    }

    #[test]
    fn snapshot_extraction_handles_absent_pieces() {
        // No status at all.
        assert!(KlippySnapshot::from_query_result(&json!({})).is_err());
        // configfile absent (klippy still starting).
        let err = KlippySnapshot::from_query_result(&json!({"status": {"plr": {}}})).unwrap_err();
        assert!(err.contains("configfile.settings"), "{err}");
        // Plugin not loaded: klippy maps unknown objects to {}.
        let result = json!({"status": {
            "configfile": {"settings": {"printer": {}}},
            "plr": {}
        }});
        let snapshot = KlippySnapshot::from_query_result(&result).unwrap();
        assert!(snapshot.plr_object.is_none());
        assert!(snapshot.plr_section().is_none());
    }

    #[test]
    fn pin_mcu_strips_modifiers_and_prefixes() {
        assert_eq!(pin_mcu("PB0"), "mcu");
        assert_eq!(pin_mcu("!PB0"), "mcu");
        assert_eq!(pin_mcu("^!PB0"), "mcu");
        assert_eq!(pin_mcu("z_board:PA1"), "z_board");
        assert_eq!(pin_mcu("~*aux: PA1"), "aux");
    }
}
