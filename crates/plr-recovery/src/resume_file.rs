//! The recovery-file generator: pure logic that emits the CONTENT of a
//! standalone `<original_stem>_RECOVERY.gcode` file the plan's final step
//! selects with `M23`/`M24`. The daemon writes the returned string into
//! the `virtual_sdcard` root before execution begins.
//!
//! # Why a separate file (not `M26` into the original)
//!
//! The old plan seeked into the original print with `M26 S<byte>`. The
//! recovery-UX milestone replaces that with a generated file so the
//! reheat-to-print-temperature, the final re-home, the purge, and the
//! entry moves are all *in the file* — visible to every UI, replayable,
//! and structurally ordered so no part-directed motion can precede
//! temperature attainment (the heating gate, [`verify_heating_gate`]).
//!
//! # File layout (in emission order)
//!
//! 1. **Header comment block** — generated-by line, timestamp, source
//!    file name, matched offset, plan id, then the ORIGINAL file's
//!    leading comment/metadata block (slicer header), capped at
//!    [`RecoveryFileSpec::header_cap`] lines so UIs still show
//!    print metadata.
//! 2. **Temps at the park position** — `M140`/`M104` (set targets), then
//!    the BLOCKING `M190`/`M109` (wait). This is the heating gate: no
//!    part-directed motion may precede these waits.
//! 3. **`G28 X Y`** — the final re-home, done at the parked Z with Z
//!    untouched (safe: XY homes at the park height).
//! 4. **Purge** — a configured `purge_macro` call, or the built-in purge
//!    (`G92 E0` / `G1 E<amount> F<slow>` / `G92 E0`); only when enabled.
//! 5. **Entry moves** — travel above the part interior, descend, prime,
//!    restore modes/feedrate (the plan builder pre-computes these).
//! 6. **The original file's byte tail** from the matched line-boundary
//!    offset, verbatim.
//!
//! # Heating-gate guarantee ([`verify_heating_gate`])
//!
//! The structure makes it IMPOSSIBLE for part-directed motion to precede
//! temperature attainment: between the blocking `M190`/`M109` and the
//! re-home / purge / entry there is no positioning `G0`/`G1` carrying an
//! X/Y word. The invariant re-parses the emitted preamble with
//! `plr-gcode` and asserts exactly that.

use plr_gcode::{LineBody, LineIter};
use serde::{Deserialize, Serialize};

use crate::plan::fmt_num;

/// The built-in / configured purge behaviour of a recovery file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PurgeSpec {
    /// A configured purge macro to CALL instead of the built-in purge —
    /// present only when `purge_macro` is set AND that macro exists on
    /// the machine (existence is resolved by the daemon; the plan
    /// builder is told the result).
    pub macro_call: Option<String>,
    /// Built-in purge extrusion length, mm (used when `macro_call` is
    /// `None`).
    pub amount: f64,
    /// Built-in purge feedrate, mm/min (deliberately slow).
    pub feed: f64,
}

/// Everything the generator needs to emit a recovery file EXCEPT the raw
/// original bytes (which the daemon streams). The plan builder derives
/// this so the entry-move / temperature / park logic lives in one place;
/// the daemon then calls [`build_recovery_file`] with the original file
/// bytes.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct RecoveryFileSpec {
    /// The generated file's sanitized top-level name (matches the plan's
    /// `M23`).
    pub name: String,
    /// The original print file's basename (header only).
    pub source_name: String,
    /// A short plan identifier (header only).
    pub plan_id: String,
    /// Byte offset in the ORIGINAL file where the verbatim tail begins
    /// (a line boundary per the `plr-gcode` span contract) — also the
    /// "matched offset" recorded in the header.
    pub tail_offset: u64,
    /// Bed target, °C (`None` leaves the bed unheated: no `M140`/`M190`).
    pub bed: Option<f64>,
    /// Nozzle print target, °C.
    pub nozzle: f64,
    /// Purge behaviour, or `None` when purging is disabled.
    pub purge: Option<PurgeSpec>,
    /// The entry-move commands (travel above the part, descend, prime,
    /// restore modes/feedrate), pre-built by the plan builder so the
    /// file and the old plan share one derivation.
    pub entry_commands: Vec<String>,
    /// Cap on leading comment lines copied from the original file.
    pub header_cap: usize,
}

/// A generated recovery file plus the offset into `content` at which the
/// verbatim original tail begins (the boundary between the generated
/// preamble and the streamed copy).
#[derive(Debug, Clone, PartialEq)]
pub struct GeneratedRecoveryFile {
    /// The full file content.
    pub content: String,
    /// Byte offset in `content` where the verbatim original tail starts.
    pub tail_start: usize,
}

/// Emits the recovery file content for `spec`, streaming the verbatim
/// tail from `original[spec.tail_offset..]`. `timestamp` is placed in the
/// header verbatim (the daemon passes wall-clock; tests pass a fixed
/// placeholder).
///
/// Total: never panics. A `tail_offset` past the end of `original`
/// yields an empty tail (the preamble is still well-formed).
#[must_use]
pub fn build_recovery_file(
    spec: &RecoveryFileSpec,
    original: &[u8],
    timestamp: &str,
) -> GeneratedRecoveryFile {
    use std::fmt::Write as _;
    let mut out = String::new();

    // (a) Header comment block. (`write!` into a String is infallible.)
    out.push_str("; generated-by dead-reckoning power-loss recovery\n");
    let _ = writeln!(out, "; generated-at {timestamp}");
    let _ = writeln!(out, "; source-file {}", spec.source_name);
    let _ = writeln!(out, "; matched-offset {}", spec.tail_offset);
    let _ = writeln!(out, "; plan-id {}", spec.plan_id);
    out.push_str("; --- original file header (metadata) ---\n");
    for line in leading_comment_lines(original, spec.header_cap) {
        out.push_str(&line);
        out.push('\n');
    }
    out.push_str("; --- end original file header ---\n");

    // (b) Temperatures AT the park position: set targets, then BLOCK on
    // attainment. This is the heating gate: nothing part-directed below
    // may run until both waits clear.
    if let Some(bed) = spec.bed {
        let _ = writeln!(out, "M140 S{}", fmt_num(bed));
    }
    let _ = writeln!(out, "M104 S{}", fmt_num(spec.nozzle));
    if let Some(bed) = spec.bed {
        let _ = writeln!(out, "M190 S{}", fmt_num(bed));
    }
    let _ = writeln!(out, "M109 S{}", fmt_num(spec.nozzle));

    // (c) The final re-home. Z is untouched, so homing XY at the parked
    // height is safe.
    out.push_str("G28 X Y\n");

    // (d) Purge (only when enabled).
    if let Some(purge) = &spec.purge {
        if let Some(macro_call) = &purge.macro_call {
            out.push_str(macro_call);
            out.push('\n');
        } else {
            out.push_str("G92 E0\n");
            let _ = writeln!(
                out,
                "G1 E{} F{}",
                fmt_num(purge.amount),
                fmt_num(purge.feed)
            );
            out.push_str("G92 E0\n");
        }
    }

    // (e) Entry moves (from above the part interior into the resume
    // point), pre-built by the plan builder.
    for command in &spec.entry_commands {
        out.push_str(command);
        out.push('\n');
    }

    // (f) The verbatim original tail. `tail_start` marks where the
    // streamed copy begins so callers can prove it byte-for-byte.
    let tail_start = out.len();
    let offset = usize::try_from(spec.tail_offset).unwrap_or(usize::MAX);
    if offset < original.len() {
        out.push_str(&String::from_utf8_lossy(&original[offset..]));
    }

    GeneratedRecoveryFile {
        content: out,
        tail_start,
    }
}

/// The original file's leading comment/blank block: every line up to the
/// first COMMAND line, keeping comment lines verbatim (blank lines are
/// dropped), capped at `cap` emitted lines. Reuses the `plr-gcode`
/// byte-faithful parser so slicer metadata (`;TYPE:`, `; filament ...`,
/// thumbnails' base64 comment lines, …) is carried into the header.
fn leading_comment_lines(original: &[u8], cap: usize) -> Vec<String> {
    let mut lines = Vec::new();
    for line in LineIter::new(original, 0) {
        match &line.body {
            LineBody::Comment(comment) => {
                if lines.len() >= cap {
                    break;
                }
                lines.push(comment.text.clone());
            }
            // Blank lines in the header are skipped (they carry no
            // metadata) but do not end the block.
            LineBody::Blank => {}
            // The first real command ends the leading comment block.
            LineBody::Command { .. } => break,
        }
    }
    lines
}

/// A heating-gate violation: the emitted preamble would let
/// part-directed motion precede temperature attainment (or the re-home /
/// entry are mis-ordered).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum HeatingGateViolation {
    /// A positioning `G0`/`G1` carrying an X or Y word appears before
    /// both blocking temperature waits.
    #[error("XY move {command:?} precedes the blocking temperature waits")]
    XyBeforeTempWait {
        /// The offending command.
        command: String,
    },
    /// The blocking nozzle wait (`M109`) is missing from the preamble.
    #[error("the preamble has no blocking M109 nozzle wait")]
    MissingNozzleWait,
    /// A bed target was set (`M140`) but the blocking bed wait (`M190`)
    /// is missing.
    #[error("a bed target was set but the preamble has no blocking M190 bed wait")]
    MissingBedWait,
    /// The final `G28 X Y` re-home is missing.
    #[error("the preamble has no G28 X Y re-home")]
    MissingReHome,
    /// The `G28 X Y` re-home appears before the temperature waits.
    #[error("the G28 X Y re-home precedes the temperature waits")]
    ReHomeBeforeTempWait,
    /// An entry (positioning) move appears before the re-home.
    #[error("an entry move {command:?} precedes the G28 X Y re-home")]
    EntryBeforeReHome {
        /// The offending command.
        command: String,
    },
}

/// Classifies a preamble command line.
struct PreLine {
    name: String,
    has_xy: bool,
    is_g28: bool,
}

fn classify_preamble(content: &str) -> Vec<PreLine> {
    let mut out = Vec::new();
    for line in LineIter::new(content.as_bytes(), 0) {
        let LineBody::Command { command, .. } = &line.body else {
            continue;
        };
        let name = command.name.to_ascii_uppercase();
        let has_xy = command.get("X").is_some() || command.get("Y").is_some();
        // `G28 X Y` parses as command G28 with X/Y flag params.
        let is_g28 = name == "G28";
        out.push(PreLine {
            name,
            has_xy,
            is_g28,
        });
    }
    out
}

/// Verifies the heating-gate invariant on a generated recovery file: no
/// positioning `G0`/`G1` XY move precedes the blocking temperature waits,
/// the `G28 X Y` re-home follows them, and entry moves follow the
/// re-home. Only the generated PREAMBLE (`content[..tail_start]`) is
/// checked — the verbatim tail is the operator's own file and is out of
/// scope.
///
/// # Errors
///
/// A [`HeatingGateViolation`] naming the first structural problem.
pub fn verify_heating_gate(file: &GeneratedRecoveryFile) -> Result<(), HeatingGateViolation> {
    let preamble = &file.content[..file.tail_start];
    let lines = classify_preamble(preamble);

    // Was a bed target set? Then M190 is required.
    let bed_target = lines.iter().any(|l| l.name == "M140");
    let m109_idx = lines.iter().position(|l| l.name == "M109");
    let m190_idx = lines.iter().position(|l| l.name == "M190");

    let Some(m109_idx) = m109_idx else {
        return Err(HeatingGateViolation::MissingNozzleWait);
    };
    if bed_target && m190_idx.is_none() {
        return Err(HeatingGateViolation::MissingBedWait);
    }
    // The gate clears only after BOTH blocking waits.
    let gate_idx = m190_idx.map_or(m109_idx, |m190| m109_idx.max(m190));

    // Rule 1: no positioning XY move before the gate clears (the re-home
    // is homing, not a positioning move, and comes after anyway).
    for line in lines.iter().take(gate_idx) {
        if matches!(line.name.as_str(), "G0" | "G1") && line.has_xy {
            return Err(HeatingGateViolation::XyBeforeTempWait {
                command: line.name.clone(),
            });
        }
    }

    // Rule 2: the G28 X Y re-home exists and follows the gate.
    let Some(g28_idx) = lines.iter().position(|l| l.is_g28) else {
        return Err(HeatingGateViolation::MissingReHome);
    };
    if g28_idx <= gate_idx {
        return Err(HeatingGateViolation::ReHomeBeforeTempWait);
    }

    // Rule 3: entry positioning moves follow the re-home.
    for line in lines.iter().take(g28_idx) {
        if matches!(line.name.as_str(), "G0" | "G1") && line.has_xy {
            return Err(HeatingGateViolation::EntryBeforeReHome {
                command: line.name.clone(),
            });
        }
    }
    Ok(())
}

/// Sanitizes a recovery file name for the `virtual_sdcard` top level:
/// keeps ASCII alphanumerics, `.`, `-`, `_`; replaces every other byte
/// (including any path separator) with `_`. Never empty (falls back to
/// `recovery`).
#[must_use]
pub fn sanitize_name(stem: &str) -> String {
    let mut out = String::with_capacity(stem.len());
    for ch in stem.chars() {
        if ch.is_ascii_alphanumeric() || matches!(ch, '.' | '-' | '_') {
            out.push(ch);
        } else {
            out.push('_');
        }
    }
    if out.is_empty() {
        "recovery".to_owned()
    } else {
        out
    }
}

/// The recovery file name for an original file name, collision-resolved
/// against `taken` (the existing top-level names in the sdcard root):
/// `<stem>_RECOVERY.gcode`, then `<stem>_RECOVERY-2.gcode`, `-3`, … until
/// a free name is found. The stem is the original name with a single
/// trailing extension removed and sanitized.
#[must_use]
pub fn recovery_file_name(original_name: &str, taken: &dyn Fn(&str) -> bool) -> String {
    let base = original_name
        .rsplit(['/', '\\'])
        .next()
        .unwrap_or(original_name);
    let stem = base.rsplit_once('.').map_or(base, |(s, _)| s);
    let stem = sanitize_name(stem);
    let candidate = format!("{stem}_RECOVERY.gcode");
    if !taken(&candidate) {
        return candidate;
    }
    for n in 2..u32::MAX {
        let candidate = format!("{stem}_RECOVERY-{n}.gcode");
        if !taken(&candidate) {
            return candidate;
        }
    }
    // Unreachable in practice (u32::MAX names taken); a stable fallback.
    format!("{stem}_RECOVERY-x.gcode")
}

#[cfg(test)]
mod tests {
    use super::{
        build_recovery_file, recovery_file_name, sanitize_name, verify_heating_gate,
        GeneratedRecoveryFile, HeatingGateViolation, PurgeSpec, RecoveryFileSpec,
    };

    fn spec() -> RecoveryFileSpec {
        RecoveryFileSpec {
            name: "part_RECOVERY.gcode".to_owned(),
            source_name: "part.gcode".to_owned(),
            plan_id: "plr-128".to_owned(),
            tail_offset: 0,
            bed: Some(60.0),
            nozzle: 210.0,
            purge: Some(PurgeSpec {
                macro_call: None,
                amount: 5.0,
                feed: 300.0,
            }),
            entry_commands: vec![
                "G90".to_owned(),
                "M83".to_owned(),
                "G0 Z1.35 F1200".to_owned(),
                "G0 X30 Y30 F1200".to_owned(),
                "G1 Z0.35 F1200".to_owned(),
                "G1 F1800".to_owned(),
            ],
            header_cap: 200,
        }
    }

    #[test]
    fn layout_is_in_the_documented_order() {
        let original = b"; slicer 1.0\n; filament PLA\nG28\nG1 X10 Y10 E1\nG1 X20 Y20 E2\n";
        let mut s = spec();
        // Tail starts at the second depositing line boundary.
        s.tail_offset = (b"; slicer 1.0\n; filament PLA\nG28\nG1 X10 Y10 E1\n".len()) as u64;
        let file = build_recovery_file(&s, original, "TS");
        let c = &file.content;
        // Header carries the metadata and the original slicer comments.
        assert!(c.contains("; generated-by dead-reckoning"));
        assert!(c.contains("; generated-at TS"));
        assert!(c.contains("; source-file part.gcode"));
        assert!(c.contains("; slicer 1.0"));
        assert!(c.contains("; filament PLA"));
        // Temps in order: set then block.
        let m140 = c.find("M140 S60").unwrap();
        let m104 = c.find("M104 S210").unwrap();
        let m190 = c.find("M190 S60").unwrap();
        let m109 = c.find("M109 S210").unwrap();
        assert!(m140 < m104 && m104 < m190 && m190 < m109);
        // Re-home then purge then entry then tail.
        let g28 = c.rfind("G28 X Y").unwrap();
        let purge = c.find("G1 E5 F300").unwrap();
        let entry = c.find("G0 X30 Y30").unwrap();
        assert!(m109 < g28 && g28 < purge && purge < entry);
        // The verbatim tail is byte-identical to the original slice.
        assert_eq!(
            file.content.as_bytes()[file.tail_start..],
            original[usize::try_from(s.tail_offset).unwrap()..]
        );
    }

    #[test]
    fn tail_is_byte_verbatim_for_arbitrary_offsets() {
        let original = b"; h\nG28\nG1 X1 Y1 E1\nG1 X2 Y2 E2\nG1 X3 Y3 E3\nG1 X4 Y4 E4\n";
        // Every line boundary is a valid resume offset.
        let mut offset = 0usize;
        for line in original.split_inclusive(|&b| b == b'\n') {
            let mut s = spec();
            s.tail_offset = offset as u64;
            let file = build_recovery_file(&s, original, "TS");
            assert_eq!(
                &file.content.as_bytes()[file.tail_start..],
                &original[offset..],
                "tail mismatch at offset {offset}"
            );
            offset += line.len();
        }
    }

    #[test]
    fn header_is_capped() {
        // 500 comment lines, cap 200.
        use std::fmt::Write as _;
        let mut original = String::new();
        for i in 0..500 {
            let _ = writeln!(original, "; meta {i}");
        }
        original.push_str("G1 X1 Y1 E1\n");
        let mut s = spec();
        s.header_cap = 200;
        let file = build_recovery_file(&s, original.as_bytes(), "TS");
        // Count only within the generated preamble (the verbatim tail
        // repeats the original comments and is out of the cap's scope).
        let copied = file.content[..file.tail_start].matches("; meta ").count();
        assert_eq!(copied, 200);
    }

    #[test]
    fn heating_gate_holds_for_a_normal_file() {
        let original = b"; h\nG1 X1 Y1 E1\n";
        let file = build_recovery_file(&spec(), original, "TS");
        assert!(verify_heating_gate(&file).is_ok());
    }

    #[test]
    fn heating_gate_holds_without_a_bed() {
        let mut s = spec();
        s.bed = None;
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        assert!(verify_heating_gate(&file).is_ok());
    }

    #[test]
    fn heating_gate_catches_an_xy_move_before_the_wait() {
        // Hostile entry commands: an XY move injected such that it lands
        // in the preamble before the temperature waits is impossible via
        // the builder (entry comes last), so hand-craft the content.
        let file = GeneratedRecoveryFile {
            content: "M104 S210\nG1 X5 Y5\nM109 S210\nG28 X Y\n; tail\n".to_owned(),
            tail_start: "M104 S210\nG1 X5 Y5\nM109 S210\nG28 X Y\n".len(),
        };
        assert_eq!(
            verify_heating_gate(&file),
            Err(HeatingGateViolation::XyBeforeTempWait {
                command: "G1".to_owned()
            })
        );
    }

    #[test]
    fn heating_gate_catches_missing_and_early_rehome() {
        // No M109 at all.
        let no_wait = GeneratedRecoveryFile {
            content: "M104 S210\nG28 X Y\n".to_owned(),
            tail_start: "M104 S210\nG28 X Y\n".len(),
        };
        assert_eq!(
            verify_heating_gate(&no_wait),
            Err(HeatingGateViolation::MissingNozzleWait)
        );
        // Bed target set but no M190.
        let no_bed_wait = GeneratedRecoveryFile {
            content: "M140 S60\nM104 S210\nM109 S210\nG28 X Y\n".to_owned(),
            tail_start: "M140 S60\nM104 S210\nM109 S210\nG28 X Y\n".len(),
        };
        assert_eq!(
            verify_heating_gate(&no_bed_wait),
            Err(HeatingGateViolation::MissingBedWait)
        );
        // Re-home before the wait.
        let early = GeneratedRecoveryFile {
            content: "M104 S210\nG28 X Y\nM109 S210\n".to_owned(),
            tail_start: "M104 S210\nG28 X Y\nM109 S210\n".len(),
        };
        assert_eq!(
            verify_heating_gate(&early),
            Err(HeatingGateViolation::ReHomeBeforeTempWait)
        );
        // No re-home at all.
        let no_home = GeneratedRecoveryFile {
            content: "M104 S210\nM109 S210\n".to_owned(),
            tail_start: "M104 S210\nM109 S210\n".len(),
        };
        assert_eq!(
            verify_heating_gate(&no_home),
            Err(HeatingGateViolation::MissingReHome)
        );
    }

    #[test]
    fn heating_gate_catches_entry_before_rehome() {
        // An entry XY move between the waits and the re-home.
        let file = GeneratedRecoveryFile {
            content: "M104 S210\nM109 S210\nG1 X5 Y5\nG28 X Y\n".to_owned(),
            tail_start: "M104 S210\nM109 S210\nG1 X5 Y5\nG28 X Y\n".len(),
        };
        assert_eq!(
            verify_heating_gate(&file),
            Err(HeatingGateViolation::EntryBeforeReHome {
                command: "G1".to_owned()
            })
        );
    }

    #[test]
    fn purge_macro_is_called_when_configured() {
        let mut s = spec();
        s.purge = Some(PurgeSpec {
            macro_call: Some("CLEAN_AND_PURGE".to_owned()),
            amount: 5.0,
            feed: 300.0,
        });
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        assert!(file.content.contains("CLEAN_AND_PURGE"));
        assert!(!file.content.contains("G92 E0"));
    }

    #[test]
    fn purge_disabled_emits_no_purge() {
        let mut s = spec();
        s.purge = None;
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        assert!(!file.content.contains("G92 E0"));
        assert!(verify_heating_gate(&file).is_ok());
    }

    #[test]
    fn names_sanitize_and_avoid_collisions() {
        assert_eq!(sanitize_name("part 1"), "part_1");
        assert_eq!(sanitize_name("a/b\\c"), "a_b_c");
        assert_eq!(sanitize_name(""), "recovery");
        // No collision: the plain name.
        assert_eq!(
            recovery_file_name("/g/part.gcode", &|_| false),
            "part_RECOVERY.gcode"
        );
        // First two taken → -2, then -3.
        let taken = |n: &str| matches!(n, "part_RECOVERY.gcode" | "part_RECOVERY-2.gcode");
        assert_eq!(
            recovery_file_name("part.gcode", &taken),
            "part_RECOVERY-3.gcode"
        );
        // A spaced/subdir original stem is sanitized.
        assert_eq!(
            recovery_file_name("/g/my part.gcode", &|_| false),
            "my_part_RECOVERY.gcode"
        );
    }

    #[test]
    fn offset_past_end_yields_empty_tail() {
        let mut s = spec();
        s.tail_offset = 10_000;
        let file = build_recovery_file(&s, b"; h\nG1 X1\n", "TS");
        assert_eq!(file.tail_start, file.content.len());
    }
}
