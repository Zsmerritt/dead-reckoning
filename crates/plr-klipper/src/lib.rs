//! Pure-logic model of the Klipper/Moonraker API: object schemas and
//! protocol message parsing, with no Klipper patches required. No sockets
//! or I/O here — the websocket transport lives in `plrd`.

/// One-line statement of this crate's purpose, used in diagnostics.
pub const PURPOSE: &str = "Klipper/Moonraker API object model and message parsing";

/// Placeholder API: composes the crate's name and purpose.
///
/// Exists so the test harness, lints, and coverage gate exercise real code
/// in this crate; feature APIs replace it.
#[must_use]
pub fn crate_summary() -> String {
    let name = env!("CARGO_PKG_NAME");
    format!("{name}: {PURPOSE}")
}

#[cfg(test)]
mod tests {
    use super::{crate_summary, PURPOSE};

    #[test]
    fn summary_is_name_prefixed_and_ends_with_purpose() {
        let summary = crate_summary();
        assert!(summary.starts_with("plr-klipper: "));
        assert!(summary.ends_with(PURPOSE));
        assert_eq!(summary.len(), "plr-klipper: ".len() + PURPOSE.len());
    }
}
