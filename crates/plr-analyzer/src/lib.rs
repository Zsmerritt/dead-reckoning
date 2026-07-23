//! Analysis of sliced G-code and printed-part geometry, e.g. selecting
//! safe part-referenced Z-probe locations for recovery. Pure logic,
//! cross-platform by construction.

/// One-line statement of this crate's purpose, used in diagnostics.
pub const PURPOSE: &str = "printed-part geometry analysis for part-referenced Z probing";

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
        assert!(summary.starts_with("plr-analyzer: "));
        assert!(summary.ends_with(PURPOSE));
        assert_eq!(summary.len(), "plr-analyzer: ".len() + PURPOSE.len());
    }
}
