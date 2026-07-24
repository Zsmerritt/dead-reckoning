//! Daemon configuration: a hand-rolled `key = value` file.
//!
//! # Format choice
//!
//! The config is a flat `key = value` file (`#` comments, no sections) —
//! the same family as `printer.cfg` / `moonraker.conf` that Klipper users
//! already hand-edit. The alternatives were worse: serde+JSON would force
//! users to write JSON by hand for a dozen scalar settings, and TOML
//! would need a new external dependency, which the workspace forbids.
//! The parser is ~80 lines, total on any input, and reports the line
//! number of every error.
//!
//! Unknown keys are errors (typo safety: a misspelled durability knob
//! silently falling back to a default would be exactly the kind of quiet
//! failure this project exists to prevent). Duplicate keys are errors for
//! the same reason.

use std::path::{Path, PathBuf};

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
    /// Bounded channel capacity between the async socket reader and the
    /// sync WAL thread. When full, motion records are dropped and the gap
    /// is journaled (see `sender::WalSender`).
    pub channel_capacity: usize,
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
            channel_capacity: 1024,
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
        for (idx, raw_line) in text.lines().enumerate() {
            let lineno = idx + 1;
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let Some((key, value)) = line.split_once('=') else {
                return Err(format!("line {lineno}: expected `key = value`"));
            };
            let (key, value) = (key.trim(), value.trim());
            if seen.iter().any(|s| s == key) {
                return Err(format!("line {lineno}: duplicate key `{key}`"));
            }
            config
                .apply(key, value)
                .map_err(|e| format!("line {lineno}: {e}"))?;
            seen.push(key.to_owned());
        }
        config.validate()?;
        Ok(config)
    }

    /// Applies a single `key = value` pair.
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
            "channel_capacity" => {
                self.channel_capacity = usize::try_from(parse_u64(value)?)
                    .map_err(|_| "value does not fit in usize".to_owned())?;
            }
            other => return Err(format!("unknown key `{other}`")),
        }
        Ok(())
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
        if self.channel_capacity < 8 {
            return Err("channel_capacity must be >= 8".to_owned());
        }
        if self.z_steppers.is_empty() {
            return Err("z_steppers must name at least one stepper".to_owned());
        }
        if self.trapq_queues.is_empty() {
            return Err("trapq_queues must name at least one queue".to_owned());
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
    fn load_reports_missing_file() {
        let err = Config::load(std::path::Path::new("/nonexistent/plrd.conf")).unwrap_err();
        assert!(err.contains("cannot read config"), "{err}");
    }
}
