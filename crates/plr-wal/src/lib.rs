//! Append-only motion write-ahead log for power-loss recovery: record
//! formats, encoding/decoding, and integrity checking for durable
//! print-state journaling. Pure logic — no syscalls or I/O here; all
//! durability I/O (fdatasync, `O_DSYNC`) lives in `plrd` and is never mocked.

/// One-line statement of this crate's purpose, used in diagnostics.
pub const PURPOSE: &str = "motion write-ahead log record formats and integrity checks";

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
        assert!(summary.starts_with("plr-wal: "));
        assert!(summary.ends_with(PURPOSE));
        assert_eq!(summary.len(), "plr-wal: ".len() + PURPOSE.len());
    }
}
