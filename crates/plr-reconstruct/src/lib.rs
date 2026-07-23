//! Reconstruction of the printer's last known state — toolhead position,
//! temperatures, and G-code file offset — from a write-ahead log recovered
//! after power loss. Pure logic; std fs is allowed in tests only.

/// One-line statement of this crate's purpose, used in diagnostics.
pub const PURPOSE: &str = "printer state reconstruction from a recovered write-ahead log";

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
        assert!(summary.starts_with("plr-reconstruct: "));
        assert!(summary.ends_with(PURPOSE));
        assert_eq!(summary.len(), "plr-reconstruct: ".len() + PURPOSE.len());
    }
}
