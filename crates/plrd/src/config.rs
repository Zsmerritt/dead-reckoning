//! Daemon configuration: a hand-rolled `key = value` file.
//!
//! # Format choice
//!
//! The config is a `key = value` file (`#` comments, one optional
//! `[machine]` section) — the same family as `printer.cfg` /
//! `moonraker.conf` that Klipper users already hand-edit. The
//! alternatives were worse: serde+JSON would force users to write JSON
//! by hand for a dozen scalar settings, and TOML would need a new
//! external dependency, which the workspace forbids. The parser is
//! small, total on any input, and reports the line number of every
//! error.
//!
//! Unknown keys are errors (typo safety: a misspelled durability knob
//! silently falling back to a default would be exactly the kind of quiet
//! failure this project exists to prevent). Duplicate keys are errors for
//! the same reason.
//!
//! # The `[machine]` section
//!
//! Recovery execution (`plrd recover`) refuses to plan without a
//! validated machine snapshot (`plr_recovery::MachineConfig`). The
//! operator supplies the attestation-shaped parts here; the runtime
//! fills in what it can observe (see `pipeline::machine_config`):
//!
//! | key | maps to | notes |
//! |---|---|---|
//! | `force_move_enabled` | `MachineConfig::force_move_enabled` | `[force_move]` with `enable_force_move: True` in printer.cfg |
//! | `z_self_locking_attested` | `z_self_locking_attested` | operator attestation; software cannot observe leadscrew mechanics |
//! | `z_steppers` | `z_steppers` | `name` or `name:mcu` pairs; bare names assume the primary MCU |
//! | `primary_mcu` | `primary_mcu` | default `mcu` |
//! | `probe_kind` | `ProbeConfig::kind` | `tap` or `load_cell` |
//! | `probe_z_offset` | `ProbeConfig::z_offset` | the configured probe `z_offset`, mm |
//! | `probe_activate_gcode_no_move` | `activate_gcode_no_move` | attestation that `activate_gcode` commands no motion |
//! | `probe_deactivate_gcode_no_move` | `deactivate_gcode_no_move` | ditto for `deactivate_gcode` |
//! | `z_position_min` | `z_position_min` | Z rail `position_min` (or `[printer] minimum_z_position`), mm |
//! | `klipper_config_path` | feeds `config_hash` | plrd checksums this file at recover time for change detection |
//! | `validated_config_hash` | `validated_config_hash` | the checksum the prerequisites were blessed against (printed by `plrd recover` on mismatch) |
//! | `virtual_sdcard_root` | `virtual_sdcard_root` | `[virtual_sdcard] path` |
//!
//! `type_annotations_present` and `config_hash` are **not** config keys:
//! the former is observed from the actual print file (`;TYPE:` scan) and
//! the latter is computed from `klipper_config_path` at recover time.
//!
//! # The `[power_fail_gpio]` section — a daemon-side key, like `wal_retention_bytes`
//!
//! The optional `[power_fail_gpio]` section configures the power-failing
//! GPIO watcher ([`crate::powerfail`]). It is a **daemon-side** setting,
//! parsed here and consumed only inside `plrd`, so the klippy
//! `daemon_keys.py` contract does **not** apply to it — exactly as it does
//! not apply to `wal_retention_bytes`. `daemon_keys.py` governs the
//! `[plr]` options an operator writes in *printer.cfg* that the Klipper
//! plugin must *declare* so klippy publishes them into
//! `configfile.settings.plr` for `plrd::plrcfg` to read; a key that lives
//! only in *plrd.conf* (this file) never travels that path, is never read
//! from `configfile.settings`, and so must not appear in `DAEMON_KEYS`
//! (verified: neither `wal_retention_bytes` nor any `power_fail_gpio.*`
//! key is in `klippy_plugin/plr/daemon_keys.py`).
//!
//! **Absent section = feature entirely off** (the default): the operator's
//! hold-up hardware does not exist yet, so this ships dormant. When the
//! section *is* present, `chip`, `line`, and `active_edge` are all
//! required (refuse, never guess a safety-relevant polarity); `debounce_ms`
//! defaults to [`PowerFailGpio::DEFAULT_DEBOUNCE_MS`].

use std::path::{Path, PathBuf};
use std::time::Duration;

/// All daemon settings, with defaults suitable for a stock Klipper host.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// Path to Klipper's API Unix socket (klippy's `-a` flag).
    pub klipper_socket: PathBuf,
    /// Directory holding WAL segments and sidecar files. Created if
    /// missing.
    pub wal_dir: PathBuf,
    /// Heartbeat file path; `None` means `<wal_dir>/heartbeat.bin`.
    pub heartbeat_path: Option<PathBuf>,
    /// Z stepper names to subscribe `motion_report/dump_stepper` for.
    /// These bound committed Z motion (`t_b`) during reconstruction.
    pub z_steppers: Vec<String>,
    /// Trapq names to subscribe `motion_report/dump_trapq` for.
    pub trapq_queues: Vec<String>,
    /// Heartbeat rewrite rate in Hz.
    pub heartbeat_hz: f64,
    /// Batch `fdatasync` interval for the motion log, milliseconds.
    /// Marker and context records are synced immediately regardless.
    pub batch_sync_ms: u64,
    /// Open the heartbeat file with `O_DSYNC` instead of calling
    /// `fdatasync` after every rewrite. Same durability, one syscall per
    /// heartbeat instead of two.
    pub heartbeat_o_dsync: bool,
    /// Rotate to a new WAL segment when the current one reaches this many
    /// bytes.
    pub segment_rotate_bytes: u64,
    /// Best-effort byte cap on the WAL directory. At each daemon start
    /// (before the WAL service opens any segment) retention prunes whole
    /// **old sessions**, oldest first, until the directory fits this cap —
    /// but it never deletes the newest session or any pinned session, so
    /// this is a floor on how much is reclaimed, not a hard ceiling.
    ///
    /// Honest worst case: `wal_retention_bytes` + the current session's
    /// growth since the last restart (a full print, or idle accumulation
    /// until the next restart) + any pinned prints
    /// (`pending_recovery.json` / `frame_invalid.json`). See
    /// `crate::retention`.
    pub wal_retention_bytes: u64,
    /// Bounded channel capacity between the async socket reader and the
    /// sync WAL thread. When full, motion records are dropped and the gap
    /// is journaled (see `sender::WalSender`).
    pub channel_capacity: usize,
    /// Moonraker WebSocket JSON-RPC endpoint
    /// (`ws://host:port/websocket`, Moonraker docs
    /// `external_api/introduction`).
    pub moonraker_url: String,
    /// Path of the daemon's control socket (the UNIX stream socket
    /// `plrd run` serves; the Klipper plugin's client side reads the
    /// same path from its `[plr]` `control_socket` setting). The two
    /// values must agree — install defaults keep them aligned; change
    /// both together or the plugin will talk to a dead path.
    pub control_socket: PathBuf,
    /// The `[machine]` section (see the module docs table).
    pub machine: MachineSection,
    /// The optional `[power_fail_gpio]` section. `None` — the default —
    /// means the power-failing GPIO watcher is entirely off (the
    /// operator's hold-up hardware does not exist yet; this ships
    /// dormant). See [`PowerFailGpio`] and [`crate::powerfail`].
    pub power_fail_gpio: Option<PowerFailGpio>,
}

/// Which GPIO edge signals that the DC rail has begun failing.
///
/// The `active_edge` a `[power_fail_gpio]` section declares is
/// **required**, never defaulted: a hold-up module may signal failure
/// with a power-good line going low (a falling edge) or a dedicated
/// fault line going high (a rising edge), and guessing the polarity of a
/// safety signal is exactly the quiet mistake this config refuses to
/// make. The confirmation re-read after the debounce checks that the line
/// still reads the *asserted* level (high for [`Rising`](Self::Rising),
/// low for [`Falling`](Self::Falling)); a blip that has already relaxed is
/// treated as spurious.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerFailEdge {
    /// Power-failing is signalled by a low→high transition; the asserted
    /// level is high.
    Rising,
    /// Power-failing is signalled by a high→low transition; the asserted
    /// level is low.
    Falling,
}

/// The `[power_fail_gpio]` section: a dedicated Linux thread watches this
/// GPIO line for the edge a hold-up module raises when power begins
/// failing (see [`crate::powerfail`]). A **daemon-side** setting — parsed
/// here, consumed only in `plrd`, outside the `daemon_keys.py` contract
/// (module docs).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PowerFailGpio {
    /// GPIO character-device chip path, e.g. `/dev/gpiochip0`. Required.
    pub chip: PathBuf,
    /// Line offset on that chip. Required.
    pub line: u32,
    /// Which edge means "power is failing". Required (see
    /// [`PowerFailEdge`]).
    pub active_edge: PowerFailEdge,
    /// Debounce re-read window: after the edge the watcher waits this long
    /// and re-reads the line, journaling the marker only if the asserted
    /// level persists. Defaults to
    /// [`DEFAULT_DEBOUNCE_MS`](Self::DEFAULT_DEBOUNCE_MS).
    pub debounce: Duration,
}

impl PowerFailGpio {
    /// Default debounce window (ms) — a few milliseconds absorbs the EMI
    /// blip of a browning-out rail without materially delaying the
    /// mandatory tier (a false marker degrades honest-wide anyway; see
    /// [`crate::powerfail`]).
    pub const DEFAULT_DEBOUNCE_MS: u64 = 5;
    /// Upper bound on a sane debounce window (ms). The hold-up budget is
    /// ~1 s; a debounce anywhere near it would eat the window it exists to
    /// protect. Refused above this, never clamped.
    pub const MAX_DEBOUNCE_MS: u64 = 500;

    /// A section-in-progress with sentinel required fields and the default
    /// debounce; `parse` finalises and `validate` domain-checks it.
    fn incomplete() -> Self {
        Self {
            chip: PathBuf::new(),
            line: 0,
            active_edge: PowerFailEdge::Falling,
            debounce: Duration::from_millis(Self::DEFAULT_DEBOUNCE_MS),
        }
    }
}

/// One Z stepper entry from the `[machine]` section.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MachineZStepper {
    /// Config section name, e.g. `stepper_z`.
    pub name: String,
    /// MCU the stepper is wired to (`primary_mcu` when written bare).
    pub mcu: Option<String>,
}

/// Operator-supplied machine snapshot inputs (see the module docs).
// The bools mirror `plr_recovery::MachineConfig`'s independent operator
// attestations one-to-one; collapsing them into enums would break that
// correspondence for no clarity gain.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, PartialEq)]
pub struct MachineSection {
    /// `[force_move]` present with `enable_force_move: True`.
    pub force_move_enabled: bool,
    /// Operator attestation: self-locking Z leadscrews.
    pub z_self_locking_attested: bool,
    /// Z steppers with optional MCU (`name` or `name:mcu`).
    pub z_steppers: Vec<MachineZStepper>,
    /// Primary MCU name.
    pub primary_mcu: String,
    /// Probe kind: `tap` or `load_cell`.
    pub probe_kind: Option<String>,
    /// Probe `z_offset`, mm.
    pub probe_z_offset: Option<f64>,
    /// Attestation: probe `activate_gcode` commands no motion.
    pub probe_activate_gcode_no_move: bool,
    /// Attestation: probe `deactivate_gcode` commands no motion.
    pub probe_deactivate_gcode_no_move: bool,
    /// Z rail `position_min`, mm.
    pub z_position_min: Option<f64>,
    /// Klipper config file to checksum for change detection.
    pub klipper_config_path: Option<PathBuf>,
    /// Checksum the prerequisites were validated against.
    pub validated_config_hash: Option<String>,
    /// `[virtual_sdcard] path`.
    pub virtual_sdcard_root: Option<String>,
}

impl Default for MachineSection {
    fn default() -> Self {
        Self {
            force_move_enabled: false,
            z_self_locking_attested: false,
            z_steppers: Vec::new(),
            primary_mcu: "mcu".to_owned(),
            probe_kind: None,
            probe_z_offset: None,
            probe_activate_gcode_no_move: false,
            probe_deactivate_gcode_no_move: false,
            z_position_min: None,
            klipper_config_path: None,
            validated_config_hash: None,
            virtual_sdcard_root: None,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            klipper_socket: PathBuf::from("/tmp/klippy_uds"),
            wal_dir: PathBuf::from("/var/lib/plrd/wal"),
            heartbeat_path: None,
            z_steppers: vec!["stepper_z".to_owned()],
            trapq_queues: vec!["toolhead".to_owned(), "extruder".to_owned()],
            heartbeat_hz: 10.0,
            batch_sync_ms: 500,
            heartbeat_o_dsync: false,
            segment_rotate_bytes: 16 * 1024 * 1024,
            wal_retention_bytes: 1024 * 1024 * 1024,
            channel_capacity: 1024,
            moonraker_url: "ws://127.0.0.1:7125/websocket".to_owned(),
            control_socket: PathBuf::from("/var/lib/plrd/plrd.sock"),
            machine: MachineSection::default(),
            power_fail_gpio: None,
        }
    }
}

impl Config {
    /// The effective heartbeat file path.
    #[must_use]
    pub fn heartbeat_file(&self) -> PathBuf {
        self.heartbeat_path
            .clone()
            .unwrap_or_else(|| self.wal_dir.join("heartbeat.bin"))
    }

    /// The receive-seq sidecar path (not configurable; always lives next
    /// to the segments so `plrd scan --wal` finds it).
    #[must_use]
    pub fn receive_seq_file(&self) -> PathBuf {
        self.wal_dir.join("receive_seq.bin")
    }

    /// The power-fail sidecar path (not configurable; lives next to the
    /// segments like the other sidecars). The power-fail watcher
    /// (`crate::powerfail`) writes the edge timestamp here as its first,
    /// channel-bypassing durability copy; recovery reads it back
    /// (epoch-admitted like the heartbeat file). Basename must match
    /// `crate::scan::POWER_FAIL_FILE_NAME`.
    #[must_use]
    pub fn power_fail_sidecar_file(&self) -> PathBuf {
        self.wal_dir.join("power_fail.bin")
    }

    /// Reads and parses a config file.
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("cannot read config {}: {e}", path.display()))?;
        Self::parse(&text)
    }

    /// Parses config text. Every error carries its 1-based line number.
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut config = Self::default();
        let mut seen: Vec<String> = Vec::new();
        let mut section: Option<&'static str> = None;
        for (idx, raw_line) in text.lines().enumerate() {
            let lineno = idx + 1;
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if let Some(name) = line.strip_prefix('[') {
                let Some(name) = name.strip_suffix(']') else {
                    return Err(format!("line {lineno}: unterminated section header"));
                };
                section = match name.trim() {
                    "machine" => Some("machine"),
                    "power_fail_gpio" => Some("power_fail_gpio"),
                    other => return Err(format!("line {lineno}: unknown section `[{other}]`")),
                };
                continue;
            }
            let Some((key, value)) = line.split_once('=') else {
                return Err(format!("line {lineno}: expected `key = value`"));
            };
            let (key, value) = (key.trim(), value.trim());
            let full_key = match section {
                None => key.to_owned(),
                Some(s) => format!("{s}.{key}"),
            };
            if seen.contains(&full_key) {
                return Err(format!("line {lineno}: duplicate key `{full_key}`"));
            }
            config
                .apply(&full_key, value)
                .map_err(|e| format!("line {lineno}: {e}"))?;
            seen.push(full_key);
        }
        // `[power_fail_gpio]` required-key presence: the section-in-progress
        // built lazily in `apply` cannot tell a sentinel `line = 0` from an
        // explicit one, so presence (not value) is checked here against the
        // keys actually seen. Refuse, never guess — the polarity especially.
        if config.power_fail_gpio.is_some() {
            for required in ["chip", "line", "active_edge"] {
                let key = format!("power_fail_gpio.{required}");
                if !seen.iter().any(|k| k == &key) {
                    return Err(format!(
                        "`[power_fail_gpio]` is present but `{required}` is not set; \
                         chip, line and active_edge are all required (omit the whole \
                         section to leave the feature off)"
                    ));
                }
            }
        }
        config.validate()?;
        Ok(config)
    }

    /// Applies a single `key = value` pair (section keys are dotted).
    fn apply(&mut self, key: &str, value: &str) -> Result<(), String> {
        match key {
            "klipper_socket" => self.klipper_socket = parse_path(value)?,
            "wal_dir" => self.wal_dir = parse_path(value)?,
            "heartbeat_path" => self.heartbeat_path = Some(parse_path(value)?),
            "z_steppers" => self.z_steppers = parse_list(value)?,
            "trapq_queues" => self.trapq_queues = parse_list(value)?,
            "heartbeat_hz" => self.heartbeat_hz = parse_f64(value)?,
            "batch_sync_ms" => self.batch_sync_ms = parse_u64(value)?,
            "heartbeat_o_dsync" => self.heartbeat_o_dsync = parse_bool(value)?,
            "segment_rotate_bytes" => self.segment_rotate_bytes = parse_u64(value)?,
            "wal_retention_bytes" => self.wal_retention_bytes = parse_u64(value)?,
            "channel_capacity" => {
                self.channel_capacity = usize::try_from(parse_u64(value)?)
                    .map_err(|_| "value does not fit in usize".to_owned())?;
            }
            "moonraker_url" => {
                if value.is_empty() {
                    return Err("moonraker_url must be non-empty".to_owned());
                }
                value.clone_into(&mut self.moonraker_url);
            }
            "control_socket" => self.control_socket = parse_path(value)?,
            "machine.force_move_enabled" => self.machine.force_move_enabled = parse_bool(value)?,
            "machine.z_self_locking_attested" => {
                self.machine.z_self_locking_attested = parse_bool(value)?;
            }
            "machine.z_steppers" => self.machine.z_steppers = parse_z_steppers(value)?,
            "machine.primary_mcu" => {
                if value.is_empty() {
                    return Err("primary_mcu must be non-empty".to_owned());
                }
                value.clone_into(&mut self.machine.primary_mcu);
            }
            "machine.probe_kind" => {
                if value != "tap" && value != "load_cell" {
                    return Err(format!("probe_kind `{value}` is not `tap` or `load_cell`"));
                }
                self.machine.probe_kind = Some(value.to_owned());
            }
            "machine.probe_z_offset" => self.machine.probe_z_offset = Some(parse_f64(value)?),
            "machine.probe_activate_gcode_no_move" => {
                self.machine.probe_activate_gcode_no_move = parse_bool(value)?;
            }
            "machine.probe_deactivate_gcode_no_move" => {
                self.machine.probe_deactivate_gcode_no_move = parse_bool(value)?;
            }
            "machine.z_position_min" => self.machine.z_position_min = Some(parse_f64(value)?),
            "machine.klipper_config_path" => {
                self.machine.klipper_config_path = Some(parse_path(value)?);
            }
            "machine.validated_config_hash" => {
                self.machine.validated_config_hash = Some(value.to_owned());
            }
            "machine.virtual_sdcard_root" => {
                self.machine.virtual_sdcard_root = Some(value.to_owned());
            }
            "power_fail_gpio.chip" => {
                self.power_fail_gpio_mut().chip = parse_path(value)?;
            }
            "power_fail_gpio.line" => {
                self.power_fail_gpio_mut().line = u32::try_from(parse_u64(value)?)
                    .map_err(|_| "line does not fit in u32".to_owned())?;
            }
            "power_fail_gpio.active_edge" => {
                self.power_fail_gpio_mut().active_edge = match value {
                    "rising" => PowerFailEdge::Rising,
                    "falling" => PowerFailEdge::Falling,
                    other => {
                        return Err(format!(
                            "active_edge `{other}` is not `rising` or `falling`"
                        ))
                    }
                };
            }
            "power_fail_gpio.debounce_ms" => {
                self.power_fail_gpio_mut().debounce = Duration::from_millis(parse_u64(value)?);
            }
            other => return Err(format!("unknown key `{other}`")),
        }
        Ok(())
    }

    /// The `[power_fail_gpio]` section, created (with sentinel required
    /// fields) on first touch. Finalised by presence checks in
    /// [`Self::parse`] and domain checks in [`Self::validate`].
    fn power_fail_gpio_mut(&mut self) -> &mut PowerFailGpio {
        self.power_fail_gpio
            .get_or_insert_with(PowerFailGpio::incomplete)
    }

    /// Domain checks; every rule states its reason.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.heartbeat_hz.is_finite()
            && self.heartbeat_hz > 0.0
            && self.heartbeat_hz <= 1000.0)
        {
            return Err("heartbeat_hz must be in (0, 1000]".to_owned());
        }
        if self.batch_sync_ms == 0 || self.batch_sync_ms > 60_000 {
            return Err("batch_sync_ms must be in [1, 60000]".to_owned());
        }
        // A segment must at least hold its header plus a real record.
        if self.segment_rotate_bytes < 4096 {
            return Err("segment_rotate_bytes must be >= 4096".to_owned());
        }
        // The retention cap must hold at least one full segment: the
        // newest session is always kept whole, so a cap below one rotation
        // unit could never be met and is a configuration mistake.
        // Refuse, do not clamp (typo safety, like every other knob here).
        if self.wal_retention_bytes < self.segment_rotate_bytes {
            return Err(format!(
                "wal_retention_bytes ({}) must be >= segment_rotate_bytes ({}): the cap has \
                 to hold at least one full segment. The newest session and any pinned prints \
                 are retained regardless, so peak disk use is this cap plus the current \
                 session plus pinned prints.",
                self.wal_retention_bytes, self.segment_rotate_bytes
            ));
        }
        if self.channel_capacity < 8 {
            return Err("channel_capacity must be >= 8".to_owned());
        }
        if self.z_steppers.is_empty() {
            return Err("z_steppers must name at least one stepper".to_owned());
        }
        if self.trapq_queues.is_empty() {
            return Err("trapq_queues must name at least one queue".to_owned());
        }
        if let Some(pf) = &self.power_fail_gpio {
            // `chip` non-empty is already enforced by `parse_path`; the
            // presence of the key is enforced in `parse`. Here: the
            // debounce band. Refuse (not clamp), like every other knob.
            let debounce_ms = u64::try_from(pf.debounce.as_millis()).unwrap_or(u64::MAX);
            if debounce_ms > PowerFailGpio::MAX_DEBOUNCE_MS {
                return Err(format!(
                    "power_fail_gpio.debounce_ms ({}) must be <= {}: a debounce near the \
                     ~1 s hold-up budget would eat the window it protects",
                    debounce_ms,
                    PowerFailGpio::MAX_DEBOUNCE_MS
                ));
            }
        }
        Ok(())
    }
}

fn parse_path(value: &str) -> Result<PathBuf, String> {
    if value.is_empty() {
        return Err("path value must be non-empty".to_owned());
    }
    Ok(PathBuf::from(value))
}

fn parse_list(value: &str) -> Result<Vec<String>, String> {
    let items: Vec<String> = value
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
        .collect();
    if items.is_empty() {
        return Err("list must contain at least one item".to_owned());
    }
    Ok(items)
}

fn parse_f64(value: &str) -> Result<f64, String> {
    value
        .parse::<f64>()
        .map_err(|_| format!("`{value}` is not a number"))
}

fn parse_u64(value: &str) -> Result<u64, String> {
    value
        .parse::<u64>()
        .map_err(|_| format!("`{value}` is not a non-negative integer"))
}

fn parse_bool(value: &str) -> Result<bool, String> {
    match value {
        "true" => Ok(true),
        "false" => Ok(false),
        other => Err(format!("`{other}` is not `true` or `false`")),
    }
}

/// Parses `name` / `name:mcu` entries.
fn parse_z_steppers(value: &str) -> Result<Vec<MachineZStepper>, String> {
    parse_list(value)?
        .into_iter()
        .map(|item| match item.split_once(':') {
            None => Ok(MachineZStepper {
                name: item,
                mcu: None,
            }),
            Some((name, mcu)) => {
                let (name, mcu) = (name.trim(), mcu.trim());
                if name.is_empty() || mcu.is_empty() {
                    return Err(format!(
                        "z stepper entry `{item}` is not `name` or `name:mcu`"
                    ));
                }
                Ok(MachineZStepper {
                    name: name.to_owned(),
                    mcu: Some(mcu.to_owned()),
                })
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::Config;
    use std::path::PathBuf;

    #[test]
    fn empty_input_yields_defaults() {
        let config = Config::parse("").unwrap();
        assert_eq!(config, Config::default());
        assert_eq!(
            config.heartbeat_file(),
            PathBuf::from("/var/lib/plrd/wal/heartbeat.bin")
        );
        assert_eq!(
            config.receive_seq_file(),
            PathBuf::from("/var/lib/plrd/wal/receive_seq.bin")
        );
    }

    #[test]
    fn full_config_round_trips_every_key() {
        let text = "\
# plrd config
klipper_socket = /home/pi/printer_data/comms/klippy.sock
wal_dir = /home/pi/plr/wal

heartbeat_path = /home/pi/plr/hb.bin
z_steppers = stepper_z, stepper_z1 ,stepper_z2
trapq_queues = toolhead,extruder
heartbeat_hz = 20
batch_sync_ms = 250
heartbeat_o_dsync = true
segment_rotate_bytes = 8388608
channel_capacity = 64
";
        let config = Config::parse(text).unwrap();
        assert_eq!(
            config.klipper_socket,
            PathBuf::from("/home/pi/printer_data/comms/klippy.sock")
        );
        assert_eq!(config.wal_dir, PathBuf::from("/home/pi/plr/wal"));
        assert_eq!(
            config.heartbeat_file(),
            PathBuf::from("/home/pi/plr/hb.bin")
        );
        assert_eq!(config.z_steppers, ["stepper_z", "stepper_z1", "stepper_z2"]);
        assert_eq!(config.trapq_queues, ["toolhead", "extruder"]);
        assert!((config.heartbeat_hz - 20.0).abs() < f64::EPSILON);
        assert_eq!(config.batch_sync_ms, 250);
        assert!(config.heartbeat_o_dsync);
        assert_eq!(config.segment_rotate_bytes, 8_388_608);
        assert_eq!(config.channel_capacity, 64);
    }

    #[test]
    fn errors_carry_line_numbers() {
        assert!(Config::parse("klipper_socket /x")
            .unwrap_err()
            .starts_with("line 1:"));
        let err = Config::parse("\n\nbogus_key = 1").unwrap_err();
        assert!(err.starts_with("line 3:"), "{err}");
        assert!(err.contains("unknown key"));
    }

    #[test]
    fn duplicate_keys_are_rejected() {
        let err = Config::parse("heartbeat_hz = 10\nheartbeat_hz = 20").unwrap_err();
        assert!(err.contains("duplicate key"), "{err}");
    }

    #[test]
    fn bad_values_are_rejected() {
        for text in [
            "heartbeat_hz = fast",
            "batch_sync_ms = -1",
            "batch_sync_ms = many",
            "heartbeat_o_dsync = yes",
            "segment_rotate_bytes = 1e6",
            "klipper_socket =",
            "z_steppers = ,",
        ] {
            assert!(Config::parse(text).is_err(), "accepted: {text}");
        }
    }

    #[test]
    fn domain_validation_rejects_out_of_range_values() {
        for text in [
            "heartbeat_hz = 0",
            "heartbeat_hz = 1001",
            "batch_sync_ms = 0",
            "batch_sync_ms = 60001",
            "segment_rotate_bytes = 4095",
            "channel_capacity = 7",
        ] {
            assert!(Config::parse(text).is_err(), "accepted: {text}");
        }
        // Boundary values are accepted.
        assert!(Config::parse("heartbeat_hz = 1000\nbatch_sync_ms = 1").is_ok());
        assert!(Config::parse("segment_rotate_bytes = 4096\nchannel_capacity = 8").is_ok());
    }

    #[test]
    fn wal_retention_bytes_parses_defaults_and_validates() {
        // Default is 1 GiB.
        assert_eq!(Config::parse("").unwrap().wal_retention_bytes, 1 << 30);
        // Explicit value round-trips.
        let config = Config::parse("wal_retention_bytes = 536870912").unwrap();
        assert_eq!(config.wal_retention_bytes, 512 * 1024 * 1024);
        // Must be >= segment_rotate_bytes: refused (not clamped) when below.
        let err = Config::parse("segment_rotate_bytes = 16777216\nwal_retention_bytes = 8388608")
            .unwrap_err();
        assert!(err.contains("must be >= segment_rotate_bytes"), "{err}");
        // Equal to one segment is the accepted boundary.
        assert!(
            Config::parse("segment_rotate_bytes = 16777216\nwal_retention_bytes = 16777216")
                .is_ok()
        );
        // Non-integer refused.
        assert!(Config::parse("wal_retention_bytes = lots").is_err());
    }

    #[test]
    fn load_reports_missing_file() {
        let err = Config::load(std::path::Path::new("/nonexistent/plrd.conf")).unwrap_err();
        assert!(err.contains("cannot read config"), "{err}");
    }

    #[test]
    fn control_socket_key_parses_and_defaults() {
        let config = Config::parse("").unwrap();
        assert_eq!(
            config.control_socket,
            PathBuf::from("/var/lib/plrd/plrd.sock")
        );
        let config = Config::parse("control_socket = /run/plrd/ctl.sock").unwrap();
        assert_eq!(config.control_socket, PathBuf::from("/run/plrd/ctl.sock"));
        assert!(Config::parse("control_socket =").is_err());
    }

    #[test]
    fn machine_section_round_trips_every_key() {
        let text = "\
moonraker_url = ws://printer.local:7125/websocket

[machine]
force_move_enabled = true
z_self_locking_attested = true
z_steppers = stepper_z, stepper_z1:mcu, stepper_z2 : aux
primary_mcu = mcu
probe_kind = tap
probe_z_offset = -0.1
probe_activate_gcode_no_move = true
probe_deactivate_gcode_no_move = true
z_position_min = -2.0
klipper_config_path = /home/pi/printer_data/config/printer.cfg
validated_config_hash = 1a2b3c4d
virtual_sdcard_root = /home/pi/printer_data/gcodes
";
        let config = Config::parse(text).unwrap();
        assert_eq!(config.moonraker_url, "ws://printer.local:7125/websocket");
        let m = &config.machine;
        assert!(m.force_move_enabled);
        assert!(m.z_self_locking_attested);
        assert_eq!(m.z_steppers.len(), 3);
        assert_eq!(m.z_steppers[0].name, "stepper_z");
        assert_eq!(m.z_steppers[0].mcu, None);
        assert_eq!(m.z_steppers[1].mcu.as_deref(), Some("mcu"));
        assert_eq!(m.z_steppers[2].name, "stepper_z2");
        assert_eq!(m.z_steppers[2].mcu.as_deref(), Some("aux"));
        assert_eq!(m.probe_kind.as_deref(), Some("tap"));
        assert_eq!(m.probe_z_offset, Some(-0.1));
        assert!(m.probe_activate_gcode_no_move);
        assert!(m.probe_deactivate_gcode_no_move);
        assert_eq!(m.z_position_min, Some(-2.0));
        assert_eq!(
            m.klipper_config_path,
            Some(PathBuf::from("/home/pi/printer_data/config/printer.cfg"))
        );
        assert_eq!(m.validated_config_hash.as_deref(), Some("1a2b3c4d"));
        assert_eq!(
            m.virtual_sdcard_root.as_deref(),
            Some("/home/pi/printer_data/gcodes")
        );
    }

    #[test]
    fn section_errors_are_caught() {
        for (text, needle) in [
            ("[rocket]", "unknown section"),
            ("[machine", "unterminated section"),
            ("[machine]\nbogus = 1", "unknown key `machine.bogus`"),
            ("[machine]\nprobe_kind = laser", "not `tap` or `load_cell`"),
            ("[machine]\nz_steppers = :mcu", "not `name` or `name:mcu`"),
            ("[machine]\nprimary_mcu =", "non-empty"),
            ("moonraker_url =", "non-empty"),
            (
                "[machine]\nz_position_min = -2\nz_position_min = -3",
                "duplicate key `machine.z_position_min`",
            ),
        ] {
            let err = Config::parse(text).unwrap_err();
            assert!(err.contains(needle), "text {text:?} gave {err}");
        }
        // A top-level key after a section stays namespaced to the
        // section (sections do not end), so it must be rejected.
        assert!(Config::parse("[machine]\nheartbeat_hz = 5").is_err());
        // Same key name in and out of the section are distinct keys.
        let config =
            Config::parse("z_steppers = stepper_z\n[machine]\nz_steppers = stepper_z:mcu").unwrap();
        assert_eq!(config.z_steppers, vec!["stepper_z"]);
        assert_eq!(config.machine.z_steppers[0].mcu.as_deref(), Some("mcu"));
    }

    #[test]
    fn power_fail_gpio_absent_is_off_by_default() {
        // The whole point of the dormant default: no section, feature off.
        assert_eq!(Config::parse("").unwrap().power_fail_gpio, None);
        assert_eq!(Config::default().power_fail_gpio, None);
    }

    #[test]
    fn power_fail_gpio_section_parses_and_defaults_debounce() {
        use super::{PowerFailEdge, PowerFailGpio};
        let config = Config::parse(
            "[power_fail_gpio]\nchip = /dev/gpiochip0\nline = 23\nactive_edge = falling\n",
        )
        .unwrap();
        let pf = config.power_fail_gpio.expect("section present");
        assert_eq!(pf.chip, PathBuf::from("/dev/gpiochip0"));
        assert_eq!(pf.line, 23);
        assert_eq!(pf.active_edge, PowerFailEdge::Falling);
        assert_eq!(
            pf.debounce,
            std::time::Duration::from_millis(PowerFailGpio::DEFAULT_DEBOUNCE_MS)
        );
        // Explicit debounce and the rising polarity round-trip too.
        let config = Config::parse(
            "[power_fail_gpio]\nchip = /dev/gpiochip1\nline = 5\nactive_edge = rising\n\
             debounce_ms = 12\n",
        )
        .unwrap();
        let pf = config.power_fail_gpio.unwrap();
        assert_eq!(pf.active_edge, PowerFailEdge::Rising);
        assert_eq!(pf.debounce, std::time::Duration::from_millis(12));
    }

    #[test]
    fn power_fail_gpio_requires_chip_line_and_edge() {
        // Present-but-incomplete refuses; it never guesses a required
        // field (least of all the polarity of a safety signal).
        for text in [
            "[power_fail_gpio]\nline = 1\nactive_edge = falling",
            "[power_fail_gpio]\nchip = /dev/gpiochip0\nactive_edge = falling",
            "[power_fail_gpio]\nchip = /dev/gpiochip0\nline = 1",
            "[power_fail_gpio]\ndebounce_ms = 5",
        ] {
            let err = Config::parse(text).unwrap_err();
            assert!(
                err.contains("is not set") && err.contains("required"),
                "text {text:?} gave {err}"
            );
        }
    }

    #[test]
    fn power_fail_gpio_rejects_bad_values() {
        // Unknown edge, non-integer line, out-of-band debounce, unknown key.
        for text in [
            "[power_fail_gpio]\nchip = /dev/gpiochip0\nline = 1\nactive_edge = sideways",
            "[power_fail_gpio]\nchip = /dev/gpiochip0\nline = notanumber\nactive_edge = falling",
            "[power_fail_gpio]\nchip = /dev/gpiochip0\nline = 1\nactive_edge = falling\n\
             debounce_ms = 100000",
            "[power_fail_gpio]\nchip = /dev/gpiochip0\nline = 1\nactive_edge = falling\nbogus = 1",
            "[power_fail_gpio]\nchip =\nline = 1\nactive_edge = falling",
        ] {
            assert!(Config::parse(text).is_err(), "accepted: {text}");
        }
        // The debounce band boundary is accepted.
        assert!(Config::parse(
            "[power_fail_gpio]\nchip = /dev/gpiochip0\nline = 1\nactive_edge = falling\n\
             debounce_ms = 500"
        )
        .is_ok());
    }
}
