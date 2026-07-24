//! Lethal-command guard scan for user-provided start/resume macro text
//! (design doc §8, step 10).
//!
//! `G28`, `Z_TILT_ADJUST` and `QUAD_GANTRY_LEVEL` inside a user macro
//! are lethal during recovery: each probes or homes downward at
//! `horizontal_move_z` ≈ 5 mm, which drives the nozzle through a tall
//! part. Any macro text the daemon would execute during recovery must
//! pass through this scan first; offending lines are stripped (or the
//! whole macro refused — the caller chooses by inspecting the typed
//! result).
//!
//! # Policy (deliberate, tested)
//!
//! * **Commented-out occurrences are inert.** A guarded token after `;`
//!   (Klipper strips `;` comments before dispatch) or on a full-line
//!   `#` comment (never survives config parsing) cannot execute. They
//!   are *reported* with [`GuardHit::in_comment`]` == true` but neither
//!   stripped nor treated as refusal-worthy.
//! * **Jinja is treated as live.** A guarded token inside `{% ... %}` /
//!   `{ ... }` template constructs may or may not execute depending on
//!   runtime state this crate cannot evaluate; it is conservatively
//!   treated as executable and stripped.
//! * **Inline `#` is not a comment.** Klipper's g-code dispatch does
//!   not treat mid-line `#` as a comment start, so only full-line `#`
//!   lines count as comments here.
//! * **Nested macros are out of scope.** This scan sees exactly the
//!   text it is given; a macro that *calls* another macro containing
//!   `G28` is not detected through the call. Callers must scan every
//!   macro body they intend to execute.
//!
//! Matching is case-insensitive on word boundaries (`G28` matches
//! `g28` and `G28 X` but not `G280` or `MY_G28_WRAPPER`).

use serde::{Deserialize, Serialize};

/// The commands that must never execute during recovery.
pub const GUARDED_COMMANDS: [&str; 3] = ["G28", "Z_TILT_ADJUST", "QUAD_GANTRY_LEVEL"];

/// One occurrence of a guarded command in scanned macro text.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GuardHit {
    /// Which guarded command was found (uppercase, one of
    /// [`GUARDED_COMMANDS`]).
    pub command: String,
    /// 0-based line index in the scanned text.
    pub line_index: usize,
    /// The full offending line, verbatim.
    pub line_text: String,
    /// `true` when the occurrence is inert (inside a comment — see the
    /// module policy).
    pub in_comment: bool,
}

/// Result of [`scan_macro_text`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GuardScan {
    /// Every occurrence found, in line order.
    pub hits: Vec<GuardHit>,
}

impl GuardScan {
    /// Hits that would actually execute (not in comments).
    #[must_use]
    pub fn executable_hits(&self) -> Vec<&GuardHit> {
        self.hits.iter().filter(|h| !h.in_comment).collect()
    }

    /// `true` when no executable guarded command was found.
    /// Comment-only occurrences do not make a macro unclean.
    #[must_use]
    pub fn is_clean(&self) -> bool {
        self.hits.iter().all(|h| h.in_comment)
    }
}

/// Result of [`sanitize_macro_text`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum GuardOutcome {
    /// No executable guarded command: the text is returned unchanged.
    Clean {
        /// The original text, verbatim.
        text: String,
    },
    /// Executable guarded commands were found; each offending line was
    /// replaced by a comment recording what was removed.
    Stripped {
        /// The sanitized text (same line count as the input).
        text: String,
        /// The hits that caused stripping (executable hits only).
        removed: Vec<GuardHit>,
    },
}

/// The executable portion of a macro line: `None` when the whole line
/// is a comment, otherwise the text before any `;` comment.
fn executable_part(line: &str) -> Option<&str> {
    let trimmed = line.trim_start();
    if trimmed.starts_with('#') {
        return None; // full-line config comment
    }
    Some(match line.find(';') {
        Some(idx) => &line[..idx],
        None => line,
    })
}

/// `true` when `c` continues a command word (so `G28` does not match
/// inside `G280` or `MY_G28_WRAPPER`).
fn is_word_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

/// Case-insensitive word-boundary search for `token` in `text`.
fn contains_word(text: &str, token: &str) -> bool {
    let upper = text.to_ascii_uppercase();
    let mut search = upper.as_str();
    let mut consumed = 0usize;
    while let Some(pos) = search.find(token) {
        let abs = consumed + pos;
        let before_ok = upper[..abs]
            .chars()
            .next_back()
            .is_none_or(|c| !is_word_char(c));
        let after_ok = upper[abs + token.len()..]
            .chars()
            .next()
            .is_none_or(|c| !is_word_char(c));
        if before_ok && after_ok {
            return true;
        }
        consumed = abs + token.len();
        search = &upper[consumed..];
    }
    false
}

/// Scans macro text for guarded commands (see the module policy).
/// Total: never panics on any input text.
#[must_use]
pub fn scan_macro_text(text: &str) -> GuardScan {
    let mut hits = Vec::new();
    for (line_index, line) in text.lines().enumerate() {
        let executable = executable_part(line);
        for token in GUARDED_COMMANDS {
            let in_executable = executable.is_some_and(|part| contains_word(part, token));
            let anywhere = contains_word(line, token);
            if in_executable {
                hits.push(GuardHit {
                    command: token.to_owned(),
                    line_index,
                    line_text: line.to_owned(),
                    in_comment: false,
                });
            } else if anywhere {
                hits.push(GuardHit {
                    command: token.to_owned(),
                    line_index,
                    line_text: line.to_owned(),
                    in_comment: true,
                });
            }
        }
    }
    GuardScan { hits }
}

/// Strips every line carrying an executable guarded command, replacing
/// it with a comment that records the removal (line count preserved).
/// Callers preferring refusal over stripping should match on
/// [`GuardOutcome::Stripped`] and refuse instead of using the text.
#[must_use]
pub fn sanitize_macro_text(text: &str) -> GuardOutcome {
    let scan = scan_macro_text(text);
    if scan.is_clean() {
        return GuardOutcome::Clean {
            text: text.to_owned(),
        };
    }
    let removed: Vec<GuardHit> = scan.hits.into_iter().filter(|h| !h.in_comment).collect();
    let stripped_lines: std::collections::BTreeSet<usize> =
        removed.iter().map(|h| h.line_index).collect();
    let mut out = String::with_capacity(text.len());
    for (idx, line) in text.lines().enumerate() {
        if idx > 0 {
            out.push('\n');
        }
        if stripped_lines.contains(&idx) {
            out.push_str("; [plr-recovery] stripped guarded command: ");
            out.push_str(line.trim());
        } else {
            out.push_str(line);
        }
    }
    if text.ends_with('\n') {
        out.push('\n');
    }
    GuardOutcome::Stripped { text: out, removed }
}

#[cfg(test)]
mod tests {
    use super::{sanitize_macro_text, scan_macro_text, GuardOutcome, GUARDED_COMMANDS};

    #[test]
    fn clean_macro_is_clean() {
        let scan = scan_macro_text("G90\nM104 S200\nSET_GCODE_OFFSET Z=0.1\n");
        assert!(scan.hits.is_empty());
        assert!(scan.is_clean());
        let GuardOutcome::Clean { text } = sanitize_macro_text("G90\n") else {
            panic!("expected Clean");
        };
        assert_eq!(text, "G90\n");
    }

    #[test]
    fn every_guarded_command_is_detected_case_insensitively() {
        for token in GUARDED_COMMANDS {
            let lower = token.to_ascii_lowercase();
            let scan = scan_macro_text(&format!("{lower} X Y\n"));
            assert_eq!(scan.hits.len(), 1, "{token} not detected");
            assert_eq!(scan.hits[0].command, token);
            assert!(!scan.hits[0].in_comment);
            assert!(!scan.is_clean());
        }
    }

    #[test]
    fn word_boundaries_prevent_false_positives() {
        let scan = scan_macro_text("G280\nMY_G28_WRAPPER\nG2 X8\nQUAD_GANTRY_LEVELING_HELPER\n");
        assert!(scan.hits.is_empty());
    }

    #[test]
    fn commented_out_occurrences_are_reported_inert() {
        let scan =
            scan_macro_text("; G28 disabled on purpose\n  # Z_TILT_ADJUST\nG90 ; not G28 here\n");
        assert_eq!(scan.hits.len(), 3);
        assert!(scan.hits.iter().all(|h| h.in_comment));
        assert!(scan.is_clean());
        assert!(matches!(
            sanitize_macro_text("; G28\n"),
            GuardOutcome::Clean { .. }
        ));
    }

    #[test]
    fn code_before_a_comment_still_counts() {
        let scan = scan_macro_text("G28 ; home all\n");
        assert_eq!(scan.executable_hits().len(), 1);
    }

    #[test]
    fn jinja_wrapped_commands_are_treated_as_live() {
        let text = "{% if not printer.toolhead.homed_axes %}\nG28\n{% endif %}\n";
        let scan = scan_macro_text(text);
        assert_eq!(scan.executable_hits().len(), 1);
        // Single-line jinja too.
        let scan = scan_macro_text("{% if x %}QUAD_GANTRY_LEVEL{% endif %}\n");
        assert_eq!(scan.executable_hits().len(), 1);
    }

    #[test]
    fn inline_hash_is_not_a_comment() {
        // Mid-line `#` does not start a comment in Klipper g-code
        // dispatch; the G28 is live.
        let scan = scan_macro_text("G90 # G28\n");
        assert_eq!(scan.executable_hits().len(), 1);
    }

    #[test]
    fn stripping_preserves_line_count_and_removes_only_executable_hits() {
        let text = "G90\nG28\n; G28 note\nZ_TILT_ADJUST\nM104 S150\n";
        let GuardOutcome::Stripped { text: out, removed } = sanitize_macro_text(text) else {
            panic!("expected Stripped");
        };
        assert_eq!(removed.len(), 2);
        assert_eq!(out.lines().count(), text.lines().count());
        let rescanned = scan_macro_text(&out);
        assert!(rescanned.is_clean(), "sanitized text must be clean: {out}");
        // Untouched lines survive verbatim.
        assert!(out.contains("G90"));
        assert!(out.contains("M104 S150"));
        assert!(out.contains("; G28 note"));
        assert!(out.ends_with('\n'));
    }

    #[test]
    fn multiple_tokens_on_one_line_are_all_reported() {
        let scan = scan_macro_text("G28\nG28 Z_TILT_ADJUST\n");
        assert_eq!(scan.hits.len(), 3);
        let GuardOutcome::Stripped { text, .. } = sanitize_macro_text("G28 Z_TILT_ADJUST") else {
            panic!("expected Stripped");
        };
        assert!(scan_macro_text(&text).is_clean());
        assert!(!text.ends_with('\n'));
    }
}
