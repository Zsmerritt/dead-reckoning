//! Hand-rolled `sd_notify(3)` (Linux).
//!
//! systemd's readiness protocol is one datagram — `READY=1` — to the
//! Unix socket named by `$NOTIFY_SOCKET`. That does not justify a
//! dependency: this is the whole protocol in ~20 lines, including the
//! abstract-namespace form (a leading `@`), with no unsafe code
//! (`std::os::linux::net::SocketAddrExt` covers abstract addresses).
//!
//! Best-effort by design: a daemon that cannot notify (not running under
//! systemd, or a broken socket) must still record — errors are reported
//! to the caller for logging, never fatal.

use std::os::linux::net::SocketAddrExt;
use std::os::unix::net::{SocketAddr, UnixDatagram};

/// Sends `READY=1` to `$NOTIFY_SOCKET` if set. Returns whether a
/// notification was actually sent (for logging).
pub fn notify_ready() -> std::io::Result<bool> {
    match std::env::var_os("NOTIFY_SOCKET") {
        None => Ok(false),
        Some(path) => {
            send_state(&path, b"READY=1")?;
            Ok(true)
        }
    }
}

/// Sends one state datagram to the given notify socket path.
fn send_state(socket_path: &std::ffi::OsStr, state: &[u8]) -> std::io::Result<()> {
    let socket = UnixDatagram::unbound()?;
    let bytes = socket_path.as_encoded_bytes();
    if let Some(name) = bytes.strip_prefix(b"@") {
        // Abstract namespace: systemd passes these as "@<name>".
        let addr = SocketAddr::from_abstract_name(name)?;
        socket.send_to_addr(state, &addr)?;
    } else {
        socket.send_to(state, socket_path)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::send_state;
    use std::os::unix::net::UnixDatagram;

    #[test]
    fn sends_ready_to_a_path_socket() {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir =
            std::env::temp_dir().join(format!("plrd-sdnotify-test-{}-{nanos}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("notify.sock");
        let receiver = UnixDatagram::bind(&path).unwrap();
        send_state(path.as_os_str(), b"READY=1").unwrap();
        let mut buf = [0_u8; 64];
        let n = receiver.recv(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"READY=1");
    }

    #[test]
    fn sends_ready_to_an_abstract_socket() {
        use std::os::linux::net::SocketAddrExt;
        let name = format!("plrd-test-{}", std::process::id());
        let addr = std::os::unix::net::SocketAddr::from_abstract_name(name.as_bytes()).unwrap();
        let receiver = UnixDatagram::bind_addr(&addr).unwrap();
        send_state(std::ffi::OsStr::new(&format!("@{name}")), b"READY=1").unwrap();
        let mut buf = [0_u8; 64];
        let n = receiver.recv(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"READY=1");
    }

    #[test]
    fn missing_socket_path_errors_but_is_not_fatal_upstream() {
        assert!(send_state(std::ffi::OsStr::new("/nonexistent/notify.sock"), b"READY=1").is_err());
    }
}
