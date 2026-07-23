//! `plrd` — the dead-reckoning daemon. Linux-only: tokio runtime, Unix
//! sockets to Moonraker/Klipper, durable WAL writes (fdatasync / `O_DSYNC`
//! via rustix), and systemd integration. On non-Linux targets the binary
//! compiles to a stub that reports the unsupported platform and exits
//! nonzero, so `cargo check --workspace` stays green on Windows.

use std::process::ExitCode;

/// Prints `message` to stderr and returns the process exit status shared by
/// both stub entry points (always failure, since no daemon runs yet).
fn stub_exit(message: &str) -> u8 {
    eprintln!("{message}");
    1
}

/// Real daemon entry point (Linux). Still a stub: the tokio runtime, socket
/// setup, and WAL I/O land with feature work on later branches.
#[cfg(target_os = "linux")]
fn main() -> ExitCode {
    ExitCode::from(stub_exit(
        "plrd: daemon not implemented yet (foundation scaffolding only)",
    ))
}

/// Non-Linux stub: `plrd` depends on Linux durability and socket semantics,
/// so anywhere else it refuses to run.
#[cfg(not(target_os = "linux"))]
fn main() -> ExitCode {
    let os = std::env::consts::OS;
    ExitCode::from(stub_exit(&format!(
        "plrd: unsupported platform `{os}`; the daemon runs only on Linux"
    )))
}

#[cfg(test)]
mod tests {
    use super::stub_exit;

    #[test]
    fn stub_exit_is_nonzero() {
        assert_ne!(stub_exit("plrd test: ignore this line"), 0);
    }

    #[test]
    fn stub_exit_is_stable_across_messages() {
        let first = stub_exit("plrd test: message one");
        let second = stub_exit("plrd test: message two");
        assert_eq!(first, second);
    }
}
