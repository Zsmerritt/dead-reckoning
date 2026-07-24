//! Hand-rolled command-line parsing.
//!
//! Deliberately not a dependency: three subcommands with one or two flags
//! each do not justify a CLI framework, and the workspace policy is "no
//! new external deps". Parsing is total: any argv produces either a
//! [`Command`] or a one-line error (the caller prints [`USAGE`] and exits
//! with code 2).

use std::path::PathBuf;

/// One-screen usage text, printed on parse errors and `plrd help`.
pub const USAGE: &str = "\
plrd - dead-reckoning power-loss recovery daemon

USAGE:
    plrd run --config <path>       run the recorder daemon (Linux only)
    plrd scan --wal <dir> [--heartbeat <path>]
                                   offline: scan a WAL directory and print
                                   a recovery report
    plrd version                   print the version
    plrd help                      print this help

EXIT CODES:
    0  success
    1  runtime failure (I/O error, WAL failure, daemon error)
    2  usage error (bad arguments)
    3  unsupported platform (the daemon runs only on Linux)";

/// A parsed invocation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// `plrd run --config <path>`: run the daemon.
    Run {
        /// Path to the configuration file.
        config: PathBuf,
    },
    /// `plrd scan --wal <dir>`: offline scan + recovery report.
    Scan {
        /// WAL directory containing segments and sidecar files.
        wal: PathBuf,
        /// Heartbeat file path override (default `<wal>/heartbeat.bin`).
        heartbeat: Option<PathBuf>,
    },
    /// `plrd version`: print the crate version.
    Version,
    /// `plrd help`: print [`USAGE`].
    Help,
    /// Hidden test harness: `plrd __crash-writer <dir>` appends records
    /// in a tight loop with per-record `fdatasync`, printing durability
    /// acknowledgements to stdout, until killed. Used by the
    /// crash-consistency integration test; not part of the public CLI.
    CrashWriter {
        /// Directory to create the WAL segment in.
        dir: PathBuf,
    },
}

/// Parses `args` (argv without the program name).
pub fn parse(args: &[String]) -> Result<Command, String> {
    let mut it = args.iter();
    let Some(cmd) = it.next() else {
        return Err("missing command".to_owned());
    };
    match cmd.as_str() {
        "run" => parse_run(&mut it),
        "scan" => parse_scan(&mut it),
        "version" | "--version" | "-V" => reject_extra(&mut it, Command::Version),
        "help" | "--help" | "-h" => reject_extra(&mut it, Command::Help),
        "__crash-writer" => {
            let dir = it
                .next()
                .ok_or_else(|| "__crash-writer requires a directory argument".to_owned())?;
            reject_extra(&mut it, Command::CrashWriter { dir: dir.into() })
        }
        other => Err(format!("unknown command `{other}`")),
    }
}

/// Parses `run` flags: exactly `--config <path>`.
fn parse_run<'a>(it: &mut impl Iterator<Item = &'a String>) -> Result<Command, String> {
    let mut config: Option<PathBuf> = None;
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--config" => assign_value(it, "--config", &mut config)?,
            other => return Err(format!("run: unknown flag `{other}`")),
        }
    }
    let config = config.ok_or_else(|| "run: missing required --config <path>".to_owned())?;
    Ok(Command::Run { config })
}

/// Parses `scan` flags: `--wal <dir>` (required) and `--heartbeat <path>`.
fn parse_scan<'a>(it: &mut impl Iterator<Item = &'a String>) -> Result<Command, String> {
    let mut wal: Option<PathBuf> = None;
    let mut heartbeat: Option<PathBuf> = None;
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--wal" => assign_value(it, "--wal", &mut wal)?,
            "--heartbeat" => assign_value(it, "--heartbeat", &mut heartbeat)?,
            other => return Err(format!("scan: unknown flag `{other}`")),
        }
    }
    let wal = wal.ok_or_else(|| "scan: missing required --wal <dir>".to_owned())?;
    Ok(Command::Scan { wal, heartbeat })
}

/// Consumes the value for `flag` into `slot`, rejecting duplicates and a
/// missing value.
fn assign_value<'a>(
    it: &mut impl Iterator<Item = &'a String>,
    flag: &str,
    slot: &mut Option<PathBuf>,
) -> Result<(), String> {
    if slot.is_some() {
        return Err(format!("duplicate flag `{flag}`"));
    }
    let value = it
        .next()
        .ok_or_else(|| format!("flag `{flag}` requires a value"))?;
    *slot = Some(PathBuf::from(value));
    Ok(())
}

/// Returns `ok` only when no arguments remain.
fn reject_extra<'a>(
    it: &mut impl Iterator<Item = &'a String>,
    ok: Command,
) -> Result<Command, String> {
    match it.next() {
        None => Ok(ok),
        Some(extra) => Err(format!("unexpected argument `{extra}`")),
    }
}

#[cfg(test)]
mod tests {
    use super::{parse, Command, USAGE};

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| (*s).to_owned()).collect()
    }

    #[test]
    fn run_requires_config() {
        assert_eq!(
            parse(&args(&["run", "--config", "/etc/plrd.conf"])),
            Ok(Command::Run {
                config: "/etc/plrd.conf".into()
            })
        );
        assert!(parse(&args(&["run"])).unwrap_err().contains("--config"));
        assert!(parse(&args(&["run", "--config"]))
            .unwrap_err()
            .contains("requires a value"));
        assert!(parse(&args(&["run", "--config", "a", "--config", "b"]))
            .unwrap_err()
            .contains("duplicate"));
        assert!(parse(&args(&["run", "--wat"]))
            .unwrap_err()
            .contains("unknown flag"));
    }

    #[test]
    fn scan_parses_wal_and_optional_heartbeat() {
        assert_eq!(
            parse(&args(&["scan", "--wal", "/var/lib/plrd/wal"])),
            Ok(Command::Scan {
                wal: "/var/lib/plrd/wal".into(),
                heartbeat: None
            })
        );
        assert_eq!(
            parse(&args(&["scan", "--heartbeat", "/tmp/hb", "--wal", "w"])),
            Ok(Command::Scan {
                wal: "w".into(),
                heartbeat: Some("/tmp/hb".into())
            })
        );
        assert!(parse(&args(&["scan"])).unwrap_err().contains("--wal"));
        assert!(parse(&args(&["scan", "--wal", "a", "b"]))
            .unwrap_err()
            .contains("unknown flag"));
    }

    #[test]
    fn version_and_help_aliases() {
        for v in ["version", "--version", "-V"] {
            assert_eq!(parse(&args(&[v])), Ok(Command::Version));
        }
        for h in ["help", "--help", "-h"] {
            assert_eq!(parse(&args(&[h])), Ok(Command::Help));
        }
        assert!(parse(&args(&["version", "x"])).is_err());
    }

    #[test]
    fn crash_writer_is_parsed_but_hidden() {
        assert_eq!(
            parse(&args(&["__crash-writer", "/tmp/x"])),
            Ok(Command::CrashWriter {
                dir: "/tmp/x".into()
            })
        );
        assert!(parse(&args(&["__crash-writer"])).is_err());
        // Hidden: not advertised in the usage text.
        assert!(!USAGE.contains("__crash-writer"));
    }

    #[test]
    fn empty_and_unknown_commands_error() {
        assert!(parse(&[]).unwrap_err().contains("missing command"));
        assert!(parse(&args(&["frobnicate"]))
            .unwrap_err()
            .contains("unknown command"));
    }
}
