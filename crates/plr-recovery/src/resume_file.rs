//! The recovery-file generator: pure logic that emits the CONTENT of a
//! standalone `<original_stem>_RECOVERY.gcode` file the plan's final step
//! selects with `M23`/`M24`. The daemon writes the returned bytes into
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
//! 4. **Re-park** — `G0 X<park> Y<park>` back to the part-clear park
//!    point. `G28` in step 3 drives the toolhead to the machine's homing
//!    XY, DISCARDING the part-clear park position the plan established
//!    and verified. The purge below must not run there: at the homed XY
//!    the nozzle may sit over the part (or, at the park Z which is
//!    `resume_z + delta`, in mid-air above it), so extruding would drop a
//!    string that is still attached to the tip — which the entry moves
//!    would then drag across the print, defeating the `CLEAN_NOZZLE` the
//!    plan ran minutes earlier precisely to guarantee a clean tip. The
//!    park point is already computed, part-clear and bounds-checked, so
//!    the file simply travels back to it.
//! 5. **Purge** — a configured `purge_macro` call, or the built-in purge
//!    (`G92 E0` / `G1 E<amount> F<slow>` / `G92 E0`); only when enabled.
//!    Runs at the re-parked, part-clear position (step 4).
//! 6. **Entry moves** — travel above the part interior, descend, prime,
//!    restore modes/feedrate (the plan builder pre-computes these).
//! 7. **The original file's byte tail** from the matched line-boundary
//!    offset, **byte-verbatim**.
//!
//! # Byte fidelity
//!
//! The file is assembled as `Vec<u8>` and the tail is
//! `extend_from_slice`d, never transcoded: print files legitimately carry
//! non-UTF-8 bytes (latin-1 in slicer comments, binary thumbnail
//! payloads), and a lossy decode would rewrite them as `EF BF BD` and
//! change the tail's length. [`GeneratedRecoveryFile::tail_bytes`] is
//! byte-identical to `original[tail_offset..]` (property-tested over
//! arbitrary bytes).
//!
//! # Heating-gate guarantee ([`verify_heating_gate`])
//!
//! The structure makes it IMPOSSIBLE for part-directed motion to precede
//! temperature attainment: before the blocking `M190`/`M109` there is no
//! motion command (`G0`/`G1`/`G2`/`G3`) carrying an X/Y word, and no Z
//! DESCENT (a relative lift is the only Z motion tolerated — see
//! [`ZIntent`]). The invariant re-parses the emitted preamble with
//! `plr-gcode` and asserts exactly that.

use plr_gcode::{LineBody, LineIter};
use serde::{Deserialize, Serialize};

use crate::plan::fmt_num;

/// Motion commands for the heating gate. Mirrors
/// `crate::plan::is_motion_command`'s g-code set (arcs included: `G2`/`G3`
/// position the toolhead exactly as `G0`/`G1` do, and a gate that ignored
/// them would let an arc walk to the part before the waits).
const MOTION_COMMANDS: [&str; 4] = ["G0", "G1", "G2", "G3"];

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
    /// The part-clear reheat park point `[x, y]`, mm — the same point the
    /// plan's park step travelled to. The file travels BACK to it after
    /// `G28 X Y` (which discards it) so the purge runs clear of the part.
    pub park: [f64; 2],
    /// Feedrate of the post-`G28` re-park travel, mm/min.
    pub park_feed: f64,
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
///
/// `content` is raw BYTES, not a `String`: the tail is copied
/// byte-verbatim from the original print file, which may contain non-UTF-8
/// sequences (see the module docs).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GeneratedRecoveryFile {
    /// The full file content, as bytes.
    pub content: Vec<u8>,
    /// Byte offset in `content` where the verbatim original tail starts.
    pub tail_start: usize,
}

impl GeneratedRecoveryFile {
    /// The generated preamble (everything before the verbatim tail).
    /// Always valid UTF-8: the generator writes only ASCII commands and
    /// the copied header comment lines, which came through `plr-gcode`'s
    /// lossy line decode.
    #[must_use]
    pub fn preamble(&self) -> &[u8] {
        &self.content[..self.tail_start]
    }

    /// The verbatim tail: byte-identical to `original[tail_offset..]`.
    #[must_use]
    pub fn tail_bytes(&self) -> &[u8] {
        &self.content[self.tail_start..]
    }

    /// The preamble as text (for rendering / previews / assertions).
    #[must_use]
    pub fn preamble_text(&self) -> std::borrow::Cow<'_, str> {
        String::from_utf8_lossy(self.preamble())
    }
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
    // The preamble is pure ASCII, so it is built as text and then
    // appended as bytes; the tail is copied byte-verbatim (module docs).
    let mut pre = String::new();

    // (a) Header comment block. (`write!` into a String is infallible.)
    pre.push_str("; generated-by dead-reckoning power-loss recovery\n");
    let _ = writeln!(pre, "; generated-at {timestamp}");
    let _ = writeln!(pre, "; source-file {}", spec.source_name);
    let _ = writeln!(pre, "; matched-offset {}", spec.tail_offset);
    let _ = writeln!(pre, "; plan-id {}", spec.plan_id);
    pre.push_str("; --- original file header (metadata) ---\n");
    for line in leading_comment_lines(original, spec.header_cap) {
        pre.push_str(&line);
        pre.push('\n');
    }
    pre.push_str("; --- end original file header ---\n");

    // (b) Temperatures AT the park position: set targets, then BLOCK on
    // attainment. This is the heating gate: nothing part-directed below
    // may run until both waits clear.
    if let Some(bed) = spec.bed {
        let _ = writeln!(pre, "M140 S{}", fmt_num(bed));
    }
    let _ = writeln!(pre, "M104 S{}", fmt_num(spec.nozzle));
    if let Some(bed) = spec.bed {
        let _ = writeln!(pre, "M190 S{}", fmt_num(bed));
    }
    let _ = writeln!(pre, "M109 S{}", fmt_num(spec.nozzle));

    // (c) The final re-home. Z is untouched, so homing XY at the parked
    // height is safe.
    pre.push_str("G28 X Y\n");

    // (d) Re-park: `G28` drove the toolhead to the machine's homing XY,
    // discarding the part-clear park point. Travel back to it BEFORE the
    // purge — purging at the homed XY would drop a nozzle-attached string
    // over the part (or in mid-air above it at the park Z) that the entry
    // moves would then drag across the print, undoing the plan's
    // CLEAN_NOZZLE. Absolute mode is asserted first: the file's own entry
    // moves may later switch to relative.
    pre.push_str("G90\n");
    let _ = writeln!(
        pre,
        "G0 X{} Y{} F{}",
        fmt_num(spec.park[0]),
        fmt_num(spec.park[1]),
        fmt_num(spec.park_feed)
    );

    // (e) Purge, at the re-parked part-clear position (only when enabled).
    if let Some(purge) = &spec.purge {
        if let Some(macro_call) = &purge.macro_call {
            pre.push_str(macro_call);
            pre.push('\n');
        } else {
            pre.push_str("G92 E0\n");
            let _ = writeln!(
                pre,
                "G1 E{} F{}",
                fmt_num(purge.amount),
                fmt_num(purge.feed)
            );
            pre.push_str("G92 E0\n");
        }
    }

    // (f) Entry moves (from above the part interior into the resume
    // point), pre-built by the plan builder.
    for command in &spec.entry_commands {
        pre.push_str(command);
        pre.push('\n');
    }

    // (g) The verbatim original tail, copied as raw BYTES (never
    // transcoded). `tail_start` marks where the copy begins so callers can
    // prove it byte-for-byte.
    let mut content = pre.into_bytes();
    let tail_start = content.len();
    let offset = usize::try_from(spec.tail_offset).unwrap_or(usize::MAX);
    if offset < original.len() {
        content.extend_from_slice(&original[offset..]);
    }

    GeneratedRecoveryFile {
        content,
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
    /// A Z move that is (or may be) a DESCENT appears before the blocking
    /// temperature waits. Only an unambiguous relative lift is tolerated
    /// there; see [`ZIntent`].
    #[error("Z move {command:?} may descend before the blocking temperature waits")]
    ZDescentBeforeTempWait {
        /// The offending command.
        command: String,
    },
    /// The post-`G28` re-park travel back to the part-clear park point is
    /// missing, so the purge/entry would run at the machine's homing XY.
    #[error("the preamble does not travel back to the park point after G28 X Y")]
    MissingRePark,
}

/// What a motion command's Z word does, as far as the gate can prove it
/// from the file alone.
///
/// The gate has no runtime Z, so it can only trust motion whose direction
/// is unambiguous from the text:
///
/// * [`ZIntent::None`] — no Z word: irrelevant to Z safety.
/// * [`ZIntent::Lift`] — a RELATIVE (`G91`) move with a strictly positive
///   Z: provably away from the part. This is the carve-out the park lift
///   needs.
/// * [`ZIntent::MaybeDescent`] — anything else: a relative non-positive Z
///   (a descent), or ANY absolute Z (whose direction depends on the
///   unknown current Z, so it may descend into the part). Refused before
///   the waits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZIntent {
    /// The command carries no Z word.
    None,
    /// A provable relative lift (strictly positive Z in `G91`).
    Lift,
    /// A descent, or a Z whose direction cannot be proven.
    MaybeDescent,
}

/// Classifies a preamble command line.
struct PreLine {
    name: String,
    has_xy: bool,
    is_g28: bool,
    z_intent: ZIntent,
}

fn classify_preamble(preamble: &[u8]) -> Vec<PreLine> {
    let mut out = Vec::new();
    // Klipper powers up in absolute mode; the generated preamble asserts
    // G90 explicitly before its own moves.
    let mut absolute = true;
    for line in LineIter::new(preamble, 0) {
        let LineBody::Command { command, .. } = &line.body else {
            continue;
        };
        let name = command.name.to_ascii_uppercase();
        match name.as_str() {
            "G90" => absolute = true,
            "G91" => absolute = false,
            _ => {}
        }
        let has_xy = command.get("X").is_some() || command.get("Y").is_some();
        // `G28 X Y` parses as command G28 with X/Y flag params.
        let is_g28 = name == "G28";
        let z_intent = match command.get("Z").map(str::parse::<f64>) {
            None => ZIntent::None,
            // A relative, strictly-positive Z is a provable lift; a
            // non-positive one descends. An absolute Z (or an unparsable
            // one) cannot be proven safe without the runtime Z.
            Some(Ok(v)) if !absolute && v > 0.0 => ZIntent::Lift,
            Some(_) => ZIntent::MaybeDescent,
        };
        out.push(PreLine {
            name,
            has_xy,
            is_g28,
            z_intent,
        });
    }
    out
}

/// `true` when the command is a toolhead-positioning move. Includes the
/// arcs `G2`/`G3` (see [`MOTION_COMMANDS`]).
fn is_motion(name: &str) -> bool {
    MOTION_COMMANDS.contains(&name)
}

/// Verifies the heating-gate invariant on a generated recovery file:
///
/// 1. no motion command (`G0`/`G1`/`G2`/`G3`) carrying X/Y, and no Z move
///    that is not a provable relative lift ([`ZIntent`]), precedes the
///    blocking temperature waits;
/// 2. the `G28 X Y` re-home exists and follows those waits;
/// 3. no positioning move precedes the re-home;
/// 4. the preamble travels BACK to the park point after `G28` (so the
///    purge and entry never run at the machine's homing XY — see the
///    module docs, layout step 4).
///
/// Only the generated PREAMBLE ([`GeneratedRecoveryFile::preamble`]) is
/// checked — the verbatim tail is the operator's own file and is out of
/// scope.
///
/// # Errors
///
/// A [`HeatingGateViolation`] naming the first structural problem.
pub fn verify_heating_gate(file: &GeneratedRecoveryFile) -> Result<(), HeatingGateViolation> {
    let lines = classify_preamble(file.preamble());

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

    // Rule 1: before the gate clears, no XY motion and no Z motion whose
    // direction is not a provable lift. (The re-home is homing, not a
    // positioning move, and comes after anyway.)
    for line in lines.iter().take(gate_idx) {
        if !is_motion(&line.name) {
            continue;
        }
        if line.has_xy {
            return Err(HeatingGateViolation::XyBeforeTempWait {
                command: line.name.clone(),
            });
        }
        if line.z_intent == ZIntent::MaybeDescent {
            return Err(HeatingGateViolation::ZDescentBeforeTempWait {
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

    // Rule 3: positioning moves follow the re-home.
    for line in lines.iter().take(g28_idx) {
        if is_motion(&line.name) && line.has_xy {
            return Err(HeatingGateViolation::EntryBeforeReHome {
                command: line.name.clone(),
            });
        }
    }

    // Rule 4: the re-park travel exists after the re-home. Without it the
    // purge would run at the homing XY the G28 just moved to.
    let re_parked = lines
        .iter()
        .skip(g28_idx + 1)
        .any(|l| is_motion(&l.name) && l.has_xy);
    if !re_parked {
        return Err(HeatingGateViolation::MissingRePark);
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

    /// Wraps hand-written preamble text as a generated file with an empty
    /// tail (hostile-shape tests for the heating gate).
    fn hand_built(preamble: &str) -> GeneratedRecoveryFile {
        GeneratedRecoveryFile {
            content: preamble.as_bytes().to_vec(),
            tail_start: preamble.len(),
        }
    }

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
            park: [180.0, 20.0],
            park_feed: 6000.0,
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
        s.tail_offset =
            u64::try_from(b"; slicer 1.0\n; filament PLA\nG28\nG1 X10 Y10 E1\n".len()).unwrap();
        let file = build_recovery_file(&s, original, "TS");
        let c = file.preamble_text().into_owned();
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
        // Re-home, then the RE-PARK travel, then purge, then entry.
        let g28 = c.find("G28 X Y").unwrap();
        let repark = c.find("G0 X180 Y20 F6000").unwrap();
        let purge = c.find("G1 E5 F300").unwrap();
        let entry = c.find("G0 X30 Y30").unwrap();
        assert!(m109 < g28, "waits precede the re-home");
        assert!(g28 < repark, "the re-park follows the re-home");
        assert!(
            repark < purge,
            "the purge must run AFTER travelling back to the part-clear park point"
        );
        assert!(purge < entry);
        // The verbatim tail is byte-identical to the original slice.
        assert_eq!(
            file.tail_bytes(),
            &original[usize::try_from(s.tail_offset).unwrap()..]
        );
    }

    /// Finding 1 regression: the built-in purge must never run at the
    /// homed XY that `G28 X Y` leaves the toolhead at.
    #[test]
    fn purge_never_runs_at_the_homed_xy() {
        let file = build_recovery_file(&spec(), b"; h\nG1 X1 Y1 E1\n", "TS");
        let text = file.preamble_text().into_owned();
        let g28 = text.find("G28 X Y").expect("re-home");
        let purge = text.find("G1 E5 F300").expect("purge");
        let between = &text[g28..purge];
        assert!(
            between.contains("G0 X180 Y20"),
            "a travel back to the park point must sit between G28 and the purge: {between}"
        );
        // ...and the gate agrees.
        assert!(verify_heating_gate(&file).is_ok());
    }

    /// Finding 2 regression: non-UTF-8 tails survive byte-for-byte.
    #[test]
    fn tail_is_byte_verbatim_for_non_utf8_originals() {
        // A latin-1 e-acute (0xE9) in a slicer comment, plus a stray 0xFF:
        // a lossy decode would rewrite both as EF BF BD and change the len.
        let original: Vec<u8> = b"; caf\xE9 \xFF\nG1 X1 Y1 E1\nG1 X2 Y2 E2\n".to_vec();
        let mut s = spec();
        s.tail_offset = u64::try_from(original.iter().position(|&b| b == b'\n').unwrap() + 1)
            .expect("offset fits");
        let file = build_recovery_file(&s, &original, "TS");
        let expected = &original[usize::try_from(s.tail_offset).unwrap()..];
        assert_eq!(file.tail_bytes(), expected);
        // Byte-for-byte: no replacement characters were introduced.
        assert!(!file
            .tail_bytes()
            .windows(3)
            .any(|w| w == [0xEF, 0xBF, 0xBD]));

        // A tail that IS the non-UTF-8 region round-trips too.
        let mut s2 = spec();
        s2.tail_offset = 0;
        let file2 = build_recovery_file(&s2, &original, "TS");
        assert_eq!(file2.tail_bytes(), &original[..]);
    }

    #[test]
    fn tail_is_byte_verbatim_for_arbitrary_offsets() {
        let original = b"; h\nG28\nG1 X1 Y1 E1\nG1 X2 Y2 E2\nG1 X3 Y3 E3\nG1 X4 Y4 E4\n";
        let mut offset = 0usize;
        for line in original.split_inclusive(|&b| b == b'\n') {
            let mut s = spec();
            s.tail_offset = u64::try_from(offset).unwrap();
            let file = build_recovery_file(&s, original, "TS");
            assert_eq!(
                file.tail_bytes(),
                &original[offset..],
                "tail mismatch at offset {offset}"
            );
            offset += line.len();
        }
    }

    #[test]
    fn header_is_capped() {
        use std::fmt::Write as _;
        let mut original = String::new();
        for i in 0..500 {
            let _ = writeln!(original, "; meta {i}");
        }
        original.push_str("G1 X1 Y1 E1\n");
        let mut s = spec();
        s.header_cap = 200;
        let file = build_recovery_file(&s, original.as_bytes(), "TS");
        let copied = file.preamble_text().matches("; meta ").count();
        assert_eq!(copied, 200);
    }

    #[test]
    fn heating_gate_holds_for_a_normal_file() {
        let file = build_recovery_file(&spec(), b"; h\nG1 X1 Y1 E1\n", "TS");
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
        let file = hand_built("M104 S210\nG1 X5 Y5\nM109 S210\nG28 X Y\nG0 X1 Y1\n");
        assert_eq!(
            verify_heating_gate(&file),
            Err(HeatingGateViolation::XyBeforeTempWait {
                command: "G1".to_owned()
            })
        );
    }

    /// Finding 4 regression: arcs are motion too.
    #[test]
    fn heating_gate_catches_arc_moves_before_the_wait() {
        for arc in ["G2", "G3"] {
            let file = hand_built(&format!(
                "M104 S210\n{arc} X5 Y5 I1 J1\nM109 S210\nG28 X Y\nG0 X1 Y1\n"
            ));
            assert_eq!(
                verify_heating_gate(&file),
                Err(HeatingGateViolation::XyBeforeTempWait {
                    command: arc.to_owned()
                }),
                "{arc} must be treated as motion"
            );
        }
        // An arc between the waits and the re-home is caught too.
        let file = hand_built("M104 S210\nM109 S210\nG3 X5 Y5 I1 J1\nG28 X Y\nG0 X1 Y1\n");
        assert_eq!(
            verify_heating_gate(&file),
            Err(HeatingGateViolation::EntryBeforeReHome {
                command: "G3".to_owned()
            })
        );
    }

    /// Finding 5 regression: a Z-only DESCENT before the waits is a
    /// violation; only a provable relative lift is tolerated.
    #[test]
    fn heating_gate_catches_a_z_descent_before_the_wait() {
        // Absolute Z: direction unknowable without the runtime Z, refused.
        let file = hand_built("M104 S210\nG90\nG1 Z-20\nM109 S210\nG28 X Y\nG0 X1 Y1\n");
        assert_eq!(
            verify_heating_gate(&file),
            Err(HeatingGateViolation::ZDescentBeforeTempWait {
                command: "G1".to_owned()
            })
        );
        // Relative negative Z: an unambiguous descent, refused.
        let file = hand_built("M104 S210\nG91\nG1 Z-20\nM109 S210\nG28 X Y\nG0 X1 Y1\n");
        assert_eq!(
            verify_heating_gate(&file),
            Err(HeatingGateViolation::ZDescentBeforeTempWait {
                command: "G1".to_owned()
            })
        );
        // A relative LIFT is the documented carve-out and passes.
        let file = hand_built("M104 S210\nG91\nG1 Z5\nG90\nM109 S210\nG28 X Y\nG0 X1 Y1\n");
        assert!(verify_heating_gate(&file).is_ok());
        // An absolute Z even in the "up" direction is still unprovable.
        let file = hand_built("M104 S210\nG90\nG1 Z200\nM109 S210\nG28 X Y\nG0 X1 Y1\n");
        assert_eq!(
            verify_heating_gate(&file),
            Err(HeatingGateViolation::ZDescentBeforeTempWait {
                command: "G1".to_owned()
            })
        );
    }

    /// Finding 1 regression at the gate level: a file that forgets to
    /// travel back to the park point after G28 is refused.
    #[test]
    fn heating_gate_catches_a_missing_re_park() {
        let file = hand_built("M104 S210\nM109 S210\nG28 X Y\nG92 E0\nG1 E5 F300\n");
        assert_eq!(
            verify_heating_gate(&file),
            Err(HeatingGateViolation::MissingRePark)
        );
    }

    #[test]
    fn heating_gate_catches_missing_and_early_rehome() {
        let no_wait = hand_built("M104 S210\nG28 X Y\n");
        assert_eq!(
            verify_heating_gate(&no_wait),
            Err(HeatingGateViolation::MissingNozzleWait)
        );
        let no_bed_wait = hand_built("M140 S60\nM104 S210\nM109 S210\nG28 X Y\n");
        assert_eq!(
            verify_heating_gate(&no_bed_wait),
            Err(HeatingGateViolation::MissingBedWait)
        );
        let early = hand_built("M104 S210\nG28 X Y\nM109 S210\n");
        assert_eq!(
            verify_heating_gate(&early),
            Err(HeatingGateViolation::ReHomeBeforeTempWait)
        );
        let no_home = hand_built("M104 S210\nM109 S210\n");
        assert_eq!(
            verify_heating_gate(&no_home),
            Err(HeatingGateViolation::MissingReHome)
        );
    }

    #[test]
    fn heating_gate_catches_entry_before_rehome() {
        let file = hand_built("M104 S210\nM109 S210\nG1 X5 Y5\nG28 X Y\nG0 X1 Y1\n");
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
        let text = file.preamble_text().into_owned();
        assert!(text.contains("CLEAN_AND_PURGE"));
        assert!(!text.contains("G92 E0"));
        // Even a macro purge runs after the re-park.
        assert!(text.find("G0 X180 Y20").unwrap() < text.find("CLEAN_AND_PURGE").unwrap());
    }

    #[test]
    fn purge_disabled_emits_no_purge_but_still_re_parks() {
        let mut s = spec();
        s.purge = None;
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        let text = file.preamble_text().into_owned();
        assert!(!text.contains("G92 E0"));
        // The re-park is unconditional: the entry moves start from a
        // known, part-clear position either way.
        assert!(text.contains("G0 X180 Y20"));
        assert!(verify_heating_gate(&file).is_ok());
    }

    #[test]
    fn names_sanitize_and_avoid_collisions() {
        assert_eq!(sanitize_name("part 1"), "part_1");
        assert_eq!(sanitize_name("a/b\\c"), "a_b_c");
        assert_eq!(sanitize_name(""), "recovery");
        assert_eq!(
            recovery_file_name("/g/part.gcode", &|_| false),
            "part_RECOVERY.gcode"
        );
        let taken = |n: &str| matches!(n, "part_RECOVERY.gcode" | "part_RECOVERY-2.gcode");
        assert_eq!(
            recovery_file_name("part.gcode", &taken),
            "part_RECOVERY-3.gcode"
        );
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
        assert!(file.tail_bytes().is_empty());
    }
}
