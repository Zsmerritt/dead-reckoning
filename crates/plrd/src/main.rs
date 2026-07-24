//! `plrd` — the dead-reckoning daemon.
//!
//! The one Linux-shaped crate in the workspace: the always-on WAL
//! recorder with real durability (`fdatasync` / `O_DSYNC` via rustix),
//! the Klipper API-socket client, the 10 Hz heartbeat writer, and the
//! gated recovery flow (`plrd recover`: WAL → reconstruction → validated
//! plan; dry-run by default, operator-confirmed step-by-step execution
//! via Moonraker — see `recover` for the full gate stack).
//!
//! # Platform split
//!
//! The **daemon** (`plrd run`) is Linux-only: its durability guarantees
//! are made of Linux syscall semantics and are never mocked — on any
//! other platform it refuses to run (exit code 3). The **offline tools**
//! (`plrd scan`, `plrd recover`, `plrd version`) deliberately work
//! everywhere: `scan` only *reads* files, so a WAL directory copied off
//! a printer can be analyzed on any machine, and `recover` defaults to
//! a dry run that plans and prints without sending anything — the
//! cross-platform-safe mode (its `--execute` path talks to Moonraker
//! and is exercised by the Linux test suite). `cargo check -p plrd`
//! stays green on Windows; Linux CI is the authority for this crate's
//! lints and tests.
//!
//! # Exit codes (also in `cli::USAGE`)
//!
//! * `0` — success
//! * `1` — runtime failure (I/O, WAL, daemon error)
//! * `2` — usage error
//! * `3` — unsupported platform (`run` off Linux)

mod cli;
// `detect`'s production caller (boot detection) lives in the Linux-only
// daemon; the module itself is pure and tested everywhere.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
mod detect;
mod executor;
mod moonraker;
mod pipeline;
// `[plr]` machine-config sourcing: parsing is pure and cross-platform;
// the one-shot klippy query is Unix-only with a documented stub
// elsewhere (there is no Unix-socket transport off Unix).
mod plrcfg;
mod recover;
mod scan;

// These compile on every platform (their logic and tests are pure), but
// their callers — the daemon and WAL service — exist only on Linux, so
// off-Linux they are exercised solely by `cargo test` and rustc's
// dead-code pass must be told so. Linux builds get full dead-code
// checking for them.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
mod config;
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
mod convert;
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
mod sender;
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
mod seqfile;

// Loopback fake-Moonraker server shared by the client/executor/recover
// tests. Test-only by construction.
#[cfg(test)]
mod testmoon;

#[cfg(target_os = "linux")]
mod client;
#[cfg(target_os = "linux")]
mod ctrlsock;
#[cfg(target_os = "linux")]
mod daemon;
#[cfg(target_os = "linux")]
mod hostclock;
#[cfg(target_os = "linux")]
mod sdnotify;
#[cfg(target_os = "linux")]
mod walsvc;

use std::process::ExitCode;

use cli::Command;

/// Success.
pub const EXIT_OK: u8 = 0;
/// Runtime failure.
pub const EXIT_RUNTIME: u8 = 1;
/// Usage error.
pub const EXIT_USAGE: u8 = 2;
/// Command requires Linux.
pub const EXIT_UNSUPPORTED: u8 = 3;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match cli::parse(&args) {
        Ok(command) => ExitCode::from(run(&command)),
        Err(e) => {
            eprintln!("plrd: {e}");
            eprintln!("{}", cli::USAGE);
            ExitCode::from(EXIT_USAGE)
        }
    }
}

/// Executes a parsed command; returns the process exit code.
fn run(command: &Command) -> u8 {
    match command {
        Command::Version => {
            println!("plrd {}", env!("CARGO_PKG_VERSION"));
            EXIT_OK
        }
        Command::Help => {
            println!("{}", cli::USAGE);
            EXIT_OK
        }
        Command::Scan {
            wal,
            heartbeat,
            config,
        } => {
            let mut stdout = std::io::stdout();
            match scan::run_scan(wal, heartbeat.as_deref(), &mut stdout) {
                Ok(()) => {
                    // Optional machine-config-mode report: an unreadable
                    // config is an error, but only after the WAL report
                    // (which never needed it) has been printed.
                    if let Some(config_path) = config {
                        match config::Config::load(config_path) {
                            Ok(config) => {
                                pipeline::report_machine_mode(&config, &mut stdout);
                                EXIT_OK
                            }
                            Err(e) => {
                                eprintln!("plrd: {e}");
                                EXIT_RUNTIME
                            }
                        }
                    } else {
                        EXIT_OK
                    }
                }
                Err(e) => {
                    eprintln!("plrd: {e}");
                    EXIT_RUNTIME
                }
            }
        }
        Command::Recover {
            config,
            execute,
            confirm,
            step,
        } => {
            let options = recover::RecoverOptions::new(*execute, *confirm, *step);
            // BufReader over Stdin (not StdinLock): the recover flow's
            // prompt reader must be Send — the same signature the
            // control socket's shared execution path requires — and
            // StdinLock is deliberately not Send.
            let mut stdin = std::io::BufReader::new(std::io::stdin());
            let mut stdout = std::io::stdout();
            recover::run_recover(config, &options, &mut stdin, &mut stdout)
        }
        Command::Run { config } => run_daemon(config),
        Command::CrashWriter { dir } => run_crash_writer(dir),
    }
}

#[cfg(target_os = "linux")]
fn run_daemon(config: &std::path::Path) -> u8 {
    daemon::run(config)
}

/// Non-Linux stub: the daemon's durability is made of Linux semantics,
/// so anywhere else it refuses to run.
#[cfg(not(target_os = "linux"))]
fn run_daemon(_config: &std::path::Path) -> u8 {
    unsupported("run")
}

#[cfg(target_os = "linux")]
fn run_crash_writer(dir: &std::path::Path) -> u8 {
    walsvc::crash_writer_main(dir)
}

#[cfg(not(target_os = "linux"))]
fn run_crash_writer(_dir: &std::path::Path) -> u8 {
    unsupported("__crash-writer")
}

#[cfg(not(target_os = "linux"))]
fn unsupported(command: &str) -> u8 {
    eprintln!(
        "plrd: `{command}` requires Linux; this platform is `{}`",
        std::env::consts::OS
    );
    EXIT_UNSUPPORTED
}

#[cfg(test)]
mod tests {
    use super::{run, EXIT_OK, EXIT_RUNTIME};
    use crate::cli::Command;

    #[test]
    fn version_and_help_succeed() {
        assert_eq!(run(&Command::Version), EXIT_OK);
        assert_eq!(run(&Command::Help), EXIT_OK);
    }

    #[test]
    fn scan_of_a_missing_directory_is_a_runtime_error() {
        assert_eq!(
            run(&Command::Scan {
                wal: "/nonexistent-plrd-wal".into(),
                heartbeat: None,
                config: None,
            }),
            EXIT_RUNTIME
        );
    }

    #[test]
    fn scan_with_unreadable_config_is_a_runtime_error_after_the_report() {
        // A valid WAL is required first; reuse the pipeline fixture.
        let (dir, _config) = crate::pipeline::e2e_tests::fixture("main-scan-cfg");
        assert_eq!(
            run(&Command::Scan {
                wal: dir.clone(),
                heartbeat: None,
                config: Some("/nonexistent/plrd.conf".into()),
            }),
            EXIT_RUNTIME
        );
        // With a readable config the mode report succeeds (klippy
        // unreachable -> legacy/undetermined text, still exit 0).
        let config_path = dir.join("plrd.conf");
        std::fs::write(&config_path, format!("wal_dir = {}\n", dir.display())).unwrap();
        assert_eq!(
            run(&Command::Scan {
                wal: dir,
                heartbeat: None,
                config: Some(config_path),
            }),
            EXIT_OK
        );
    }

    #[cfg(not(target_os = "linux"))]
    #[test]
    fn run_refuses_off_linux() {
        assert_eq!(
            run(&Command::Run {
                config: "/etc/plrd.conf".into()
            }),
            super::EXIT_UNSUPPORTED
        );
        assert_eq!(
            run(&Command::CrashWriter { dir: "/tmp".into() }),
            super::EXIT_UNSUPPORTED
        );
    }
}
