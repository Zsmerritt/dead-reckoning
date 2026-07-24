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
//! 4. **Re-park / purge travel** — `G0 X.. Y..` (then an optional
//!    `G1 Z..`) to a KNOWN destination. `G28` in step 3 drives the
//!    toolhead to the machine's homing XY, DISCARDING the part-clear park
//!    position the plan established and verified. Nothing may extrude or
//!    enter the part from there: at the homed XY the nozzle may sit over
//!    the part (or, at the park Z which is `resume_z + delta`, in mid-air
//!    above it), so extruding would drop a string still attached to the
//!    tip — which the entry moves would then drag across the print,
//!    defeating the `CLEAN_NOZZLE` the plan ran minutes earlier precisely
//!    to guarantee a clean tip. The destination is
//!    [`RecoveryFileSpec::post_home_target`]: the built-in purge location
//!    when one is configured, else the park point.
//! 5. **Purge** — per the resolved [`PurgePlan`]: a `purge_macro` call
//!    (which owns its own positioning, amount and speed — plr emits
//!    nothing else and does not reposition for it), or the built-in
//!    `G92 E0` / `G1 E<amount> F<speed>` / optional
//!    `G1 E-<retract> F<speed>` / `G92 E0` at the location reached in
//!    step 4. Absent entirely when `purge_enable = false`.
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

/// Positioning g-codes: the ONE motion-command set shared by the heating
/// gate ([`verify_heating_gate`]) and the recovery-file pre-flight
/// ([`crate::preflight::preflight_recovery_file`]), so their coverage
/// cannot drift apart. Mirrors `crate::plan::is_motion_command`'s set.
///
/// Arcs are included deliberately: `G2`/`G3` position the toolhead
/// exactly as `G0`/`G1` do, so a gate that ignored them would let an arc
/// walk to the part before the temperature waits, and a bounds check that
/// ignored them would miss an out-of-range arc endpoint.
pub const MOTION_COMMANDS: [&str; 4] = ["G0", "G1", "G2", "G3"];

/// `true` when `name` (already uppercased) is a positioning move.
#[must_use]
pub fn is_motion_command(name: &str) -> bool {
    MOTION_COMMANDS.contains(&name)
}

/// How the recovery file purges, once the `[plr]` precedence table has
/// been resolved by the plan builder.
///
/// The three coherent paths an operator can be on — hand it to a macro,
/// fully specify the built-in, or turn it off — map onto
/// `Some(Macro)` / `Some(BuiltIn)` / `None`. A fourth situation,
/// "`purge_macro` set but the macro does not exist", is not representable
/// here at all: the builder REFUSES to plan
/// ([`crate::RecoveryError::PurgeMacroMissing`]) rather than silently
/// substituting the built-in.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PurgePlan {
    /// A configured `purge_macro` that exists on the machine owns the
    /// purge ENTIRELY: its own positioning, amount and speed. plr emits
    /// the call and nothing else, and never repositions the toolhead for
    /// it — the macro's motion is unknowable here, exactly as with
    /// `CLEAN_NOZZLE`.
    Macro {
        /// The macro name to call.
        call: String,
    },
    /// The built-in purge, fully specified.
    BuiltIn {
        /// Where to purge, mm. Defaults to the reheat park point (already
        /// computed, part-clear and bounds-checked).
        point: [f64; 2],
        /// Absolute Z to purge at, mm. `None` keeps whatever height the
        /// park lift left in effect (the elevated park Z).
        z: Option<f64>,
        /// Extrusion length, mm.
        amount: f64,
        /// Extrusion feedrate, mm/min.
        speed: f64,
        /// Filament retracted after purging, mm, to help break the
        /// string. `0` disables the retract.
        retract: f64,
        /// Travel feedrate used to reach `point`/`z`, mm/min.
        travel_feed: f64,
    },
}

impl PurgePlan {
    /// The built-in purge's location, when this is a built-in purge.
    #[must_use]
    pub fn built_in_point(&self) -> Option<[f64; 2]> {
        match self {
            PurgePlan::BuiltIn { point, .. } => Some(*point),
            PurgePlan::Macro { .. } => None,
        }
    }
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
    /// Purge behaviour, or `None` when purging is disabled
    /// (`purge_enable = false`).
    pub purge: Option<PurgePlan>,
    /// The part-clear reheat park point `[x, y]`, mm — the same point the
    /// plan's park step travelled to. The file travels BACK to it after
    /// `G28 X Y` (which discards it) so the purge runs clear of the part.
    pub park: [f64; 2],
    /// Feedrate of the post-`G28` re-park travel, mm/min.
    pub park_feed: f64,
    /// Feedrate of the purge-Z DESCENT, mm/min — the slow entry feedrate,
    /// not the travel feedrate: a descent toward the bed is a near-part
    /// move and every other one in this file is speed-limited.
    pub descend_feed: f64,
    /// The entry-move commands (travel above the part, descend, prime,
    /// restore modes/feedrate), pre-built by the plan builder so the
    /// file and the old plan share one derivation.
    pub entry_commands: Vec<String>,
    /// `(clamp, restore)` acceleration, mm/s², wrapped around the entry
    /// moves: `[plr]` `accel_entry` and the machine's own configured
    /// `max_accel`. `None` emits neither command.
    ///
    /// The entry moves live HERE, not in the plan — so these are the
    /// commands that actually descend toward the part, and the one place
    /// a low acceleration matters most. Both values are literals because
    /// the generated file has no runtime-placeholder machinery.
    ///
    /// Both or neither, deliberately: a clamp this file could not undo
    /// would govern the entire remaining print, which is worse than not
    /// clamping. The plan builder only fills this in when it knows both
    /// numbers, and warns when it does not.
    #[serde(default)]
    pub entry_accel: Option<(f64, f64)>,
    /// Cap on leading comment lines copied from the original file.
    pub header_cap: usize,
}

impl RecoveryFileSpec {
    /// Where the single post-`G28` travel goes, and at what Z.
    ///
    /// `G28 X Y` discards the part-clear park position, so the file must
    /// travel somewhere known before it extrudes or enters the part. That
    /// destination is the BUILT-IN PURGE LOCATION when one is configured
    /// (there is no reason to visit the park point and then immediately
    /// leave it), and the park point otherwise — including when a purge
    /// macro owns the purge, since plr does not reposition for a macro.
    ///
    /// One resolver, used by both the emitter and the heating gate, so
    /// the guard can never check a different destination than the file
    /// actually travels to.
    #[must_use]
    pub fn post_home_target(&self) -> ([f64; 2], Option<f64>) {
        match &self.purge {
            Some(PurgePlan::BuiltIn { point, z, .. }) => (*point, *z),
            Some(PurgePlan::Macro { .. }) | None => (self.park, None),
        }
    }
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
    // The destination is the built-in purge location when one is
    // configured (no reason to visit the park point then immediately
    // leave), else the park point — including for a purge MACRO, since
    // plr does not reposition for a macro.
    let (dest, dest_z) = spec.post_home_target();
    let travel_feed = match &spec.purge {
        Some(PurgePlan::BuiltIn { travel_feed, .. }) => *travel_feed,
        Some(PurgePlan::Macro { .. }) | None => spec.park_feed,
    };
    pre.push_str("G90\n");
    let _ = writeln!(
        pre,
        "G0 X{} Y{} F{}",
        fmt_num(dest[0]),
        fmt_num(dest[1]),
        fmt_num(travel_feed)
    );
    // An explicit purge Z descends only AFTER the XY travel completes, so
    // the nozzle never sweeps low across whatever lies in between — and
    // at the SLOW entry feedrate, not the travel feedrate: this is a
    // descent toward the bed, the same class of near-part move as the
    // entry moves, all of which are deliberately speed-limited.
    if let Some(z) = dest_z {
        let _ = writeln!(pre, "G1 Z{} F{}", fmt_num(z), fmt_num(spec.descend_feed));
    }

    // (e) Purge, at the destination reached above (only when enabled).
    match &spec.purge {
        // A macro owns its own positioning, amount and speed: emit the
        // call and NOTHING else.
        Some(PurgePlan::Macro { call }) => {
            pre.push_str(call);
            pre.push('\n');
        }
        Some(PurgePlan::BuiltIn {
            amount,
            speed,
            retract,
            ..
        }) => {
            // RELATIVE E is asserted FIRST and is load-bearing. Klipper
            // powers up in ABSOLUTE extrusion (`gcode_move.py:29`, applied
            // at 141-151), and the extrusion mode in force here is
            // otherwise UNKNOWABLE — the operator's CLEAN_NOZZLE macro may
            // have left either mode set. In absolute E, `G1 E-<retract>`
            // after `G1 E<amount>` does not retract by `retract`: it moves
            // the axis TO `-retract`, a retraction of `amount + retract`.
            // At `purge_amount = 50` that is a 51 mm pull, which yanks
            // filament clear of a direct-drive extruder's gears and the
            // rest of the print extrudes nothing.
            //
            // `build_entry_commands` already obeys this rule (it emits its
            // own `M83` before the prime); the purge must too.
            pre.push_str("M83\n");
            pre.push_str("G92 E0\n");
            let _ = writeln!(pre, "G1 E{} F{}", fmt_num(*amount), fmt_num(*speed));
            if *retract > 0.0 {
                let _ = writeln!(pre, "G1 E-{} F{}", fmt_num(*retract), fmt_num(*speed));
            }
            pre.push_str("G92 E0\n");
        }
        None => {}
    }

    // (f) Entry moves (from above the part interior into the resume
    // point), pre-built by the plan builder — optionally wrapped in the
    // `accel_entry` clamp.
    //
    // This is the near-part motion: the entry block descends toward the
    // printed surface, so it is where a reduced acceleration earns its
    // keep. The clamp is emitted AFTER the blocking M190/M109 above
    // (`SET_VELOCITY_LIMIT` is not a motion command, so the heating gate
    // is indifferent to it either way) and the restore is emitted
    // unconditionally alongside it — including when the entry block is
    // empty, so the pair can never be left half-applied.
    if let Some((clamp, _)) = spec.entry_accel {
        let _ = writeln!(pre, "SET_VELOCITY_LIMIT ACCEL={}", fmt_num(clamp));
    }
    for command in &spec.entry_commands {
        pre.push_str(command);
        pre.push('\n');
    }
    if let Some((_, restore)) = spec.entry_accel {
        let _ = writeln!(pre, "SET_VELOCITY_LIMIT ACCEL={}", fmt_num(restore));
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
    /// missing: the first motion after the re-home is not an XY travel
    /// (or there is none at all), so the purge/entry would run at the
    /// machine's homing XY.
    #[error(
        "the first motion after G28 X Y is not an XY travel back to the park point \
         (the purge/entry would run at the homed XY)"
    )]
    MissingRePark,
    /// The first motion after `G28` IS an XY travel, but not to the
    /// part-clear park point the plan computed and bounds-checked.
    #[error(
        "the post-G28 re-park travels to ({found_x}, {found_y}) but the plan's \
         part-clear park point is ({park_x}, {park_y})"
    )]
    ReParkMismatch {
        /// X the preamble travels to.
        found_x: String,
        /// Y the preamble travels to.
        found_y: String,
        /// X of the plan's park point.
        park_x: String,
        /// Y of the plan's park point.
        park_y: String,
    },
    /// An extruder-moving command (the built-in purge) appears before the
    /// post-`G28` re-park travel, so it would extrude at the homed XY (or
    /// at the parked position, if it precedes the re-home).
    #[error("purge command {command:?} precedes the post-G28 re-park travel")]
    PurgeBeforeRePark {
        /// The offending command.
        command: String,
    },
    /// The post-`G28` re-park is a RELATIVE move. Its X/Y words are then
    /// deltas from wherever homing left the toolhead, not a destination,
    /// so it cannot be shown to reach the part-clear point — and the
    /// recovery-file pre-flight skips relative moves too, so both guards
    /// would fail open together.
    #[error(
        "the post-G28 re-park {command:?} runs in RELATIVE mode (G91); it must be an \
         absolute move so its target is a real destination"
    )]
    RelativeRePark {
        /// The offending command.
        command: String,
    },
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
///
/// The several booleans are independent, orthogonal facts about ONE
/// parsed line (does it carry XY, is it the re-home, does it touch E,
/// what mode is it in) that the gate's rules read individually; folding
/// them into an enum would only re-expand at every use site.
#[allow(clippy::struct_excessive_bools)]
struct PreLine {
    name: String,
    has_xy: bool,
    is_g28: bool,
    z_intent: ZIntent,
    /// Literal X target, when the command carries a parsable one.
    x: Option<f64>,
    /// Literal Y target, when the command carries a parsable one.
    y: Option<f64>,
    /// The command carries an `E` word (the built-in purge does).
    touches_e: bool,
    /// `true` when the command executes in ABSOLUTE coordinate mode.
    absolute: bool,
}

/// Tolerance, mm, when matching the emitted re-park coordinates against
/// the spec's park point. Commands render coordinates at five decimals
/// ([`fmt_num`]), so a re-parsed value can differ by half an ulp of that
/// quantization.
const PARK_MATCH_EPSILON: f64 = 1e-4;

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
        // Literal XY targets, for the re-park coordinate check. `G28 X Y`
        // carries flag params with no value, which do not parse — exactly
        // right: the re-home is not a positioning move.
        let x = command.get("X").and_then(|v| v.parse::<f64>().ok());
        let y = command.get("Y").and_then(|v| v.parse::<f64>().ok());
        // Any command that changes the extruder axis: the built-in purge
        // (`G92 E0`, `G1 E<amt>`). The re-park must precede all of these.
        let touches_e = command.get("E").is_some();
        out.push(PreLine {
            name,
            has_xy,
            is_g28,
            z_intent,
            x,
            y,
            touches_e,
            absolute,
        });
    }
    out
}

use is_motion_command as is_motion;

/// Verifies the heating-gate invariant on a generated recovery file:
///
/// 1. no motion command (`G0`/`G1`/`G2`/`G3`) carrying X/Y, and no Z move
///    that is not a provable relative lift ([`ZIntent`]), precedes the
///    blocking temperature waits;
/// 2. the `G28 X Y` re-home exists and follows those waits;
/// 3. no positioning move precedes the re-home;
/// 4. the FIRST motion after `G28` is an XY travel back to `park` — the
///    plan's part-clear, bounds-checked reheat park point — and no
///    extruder-moving command precedes it (so the purge and entry never
///    run at the machine's homing XY; see the module docs, layout step 4).
///
/// The expected destination comes from `spec`
/// ([`RecoveryFileSpec::post_home_target`] — the built-in purge location
/// when one is configured, else the park point), passed in rather than
/// read back out of the file: a file that certified its own coordinate
/// would prove nothing.
///
/// Only the generated PREAMBLE ([`GeneratedRecoveryFile::preamble`]) is
/// checked — the verbatim tail is the operator's own file and is out of
/// scope.
///
/// # Errors
///
/// A [`HeatingGateViolation`] naming the first structural problem.
pub fn verify_heating_gate(
    file: &GeneratedRecoveryFile,
    spec: &RecoveryFileSpec,
) -> Result<(), HeatingGateViolation> {
    let (park, _) = spec.post_home_target();
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

    // Rule 3: positioning moves AND extrusion follow the re-home. The
    // extrusion half matters independently: a `G1 E5` between the
    // temperature waits and the re-home would purge at the parked
    // position, which is exactly the deposit-then-drag hazard rule 4
    // exists to prevent — it just happens earlier.
    for line in lines.iter().take(g28_idx) {
        if is_motion(&line.name) && line.has_xy {
            return Err(HeatingGateViolation::EntryBeforeReHome {
                command: line.name.clone(),
            });
        }
        if line.touches_e {
            return Err(HeatingGateViolation::PurgeBeforeRePark {
                command: line.name.clone(),
            });
        }
    }

    // Rule 4: the FIRST motion after the re-home is the re-park travel,
    // and it goes to the plan's part-clear park point.
    //
    // "First motion", not "any motion": the entry moves always contain an
    // XY travel, so an "any" test could never fail for a generated file —
    // it would pass just as happily with the re-park deleted, or moved
    // after the purge. Anchoring on the first motion makes both of those
    // fire, because the purge's `G1 E<amt>` and the entry's `G0 Z<hop>`
    // are themselves motion commands carrying no XY.
    //
    // Caveat, documented rather than pretended away: a configured
    // `purge_macro` is an opaque macro call (like `CLEAN_NOZZLE`), so its
    // internal motion is unknowable here — the same limit the plan's
    // clean-nozzle step carries.
    let after_home = &lines[g28_idx + 1..];
    // The built-in purge must not sneak in ahead of the re-park.
    let first_motion_idx = after_home.iter().position(|l| is_motion(&l.name));
    if let Some(purge_idx) = after_home.iter().position(|l| l.touches_e) {
        if first_motion_idx.is_none_or(|m| purge_idx < m) {
            return Err(HeatingGateViolation::PurgeBeforeRePark {
                command: after_home[purge_idx].name.clone(),
            });
        }
    }
    let Some(re_park) = first_motion_idx.map(|i| &after_home[i]) else {
        return Err(HeatingGateViolation::MissingRePark);
    };
    if !re_park.has_xy {
        return Err(HeatingGateViolation::MissingRePark);
    }
    // The re-park must be an ABSOLUTE move. In relative mode the same
    // `X`/`Y` words are deltas from wherever `G28` left the toolhead, so
    // they do NOT identify a destination and the coordinate match below
    // would be meaningless. This also closes a fail-open pair: the
    // recovery-file pre-flight skips relative moves (their coordinates
    // are not positions), so without this check a `G91` slipped in ahead
    // of the re-park would defeat BOTH guards at once.
    if !re_park.absolute {
        return Err(HeatingGateViolation::RelativeRePark {
            command: re_park.name.clone(),
        });
    }
    match (re_park.x, re_park.y) {
        (Some(x), Some(y))
            if (x - park[0]).abs() <= PARK_MATCH_EPSILON
                && (y - park[1]).abs() <= PARK_MATCH_EPSILON => {}
        (x, y) => {
            return Err(HeatingGateViolation::ReParkMismatch {
                found_x: x.map_or_else(|| "?".to_owned(), fmt_num),
                found_y: y.map_or_else(|| "?".to_owned(), fmt_num),
                park_x: fmt_num(park[0]),
                park_y: fmt_num(park[1]),
            })
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
        GeneratedRecoveryFile, HeatingGateViolation, PurgePlan, RecoveryFileSpec,
    };

    /// The park point every fixture in this module uses.
    const PARK: [f64; 2] = [180.0, 20.0];

    /// A spec whose `post_home_target` is `PARK` (no purge), for the
    /// hand-built hostile-shape gate tests.
    fn park_only_spec() -> RecoveryFileSpec {
        RecoveryFileSpec {
            park: PARK,
            purge: None,
            ..spec()
        }
    }

    /// Wraps hand-written preamble text as a generated file with an empty
    /// tail (hostile-shape tests for the heating gate).
    fn hand_built(preamble: &str) -> GeneratedRecoveryFile {
        GeneratedRecoveryFile {
            content: preamble.as_bytes().to_vec(),
            tail_start: preamble.len(),
        }
    }

    fn built_in_purge() -> PurgePlan {
        PurgePlan::BuiltIn {
            point: PARK,
            z: None,
            amount: 5.0,
            speed: 300.0,
            retract: 0.0,
            travel_feed: 6000.0,
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
            purge: Some(built_in_purge()),
            park: PARK,
            park_feed: 6000.0,
            descend_feed: 1200.0,
            entry_commands: vec![
                "G90".to_owned(),
                "M83".to_owned(),
                "G0 Z1.35 F1200".to_owned(),
                "G0 X30 Y30 F1200".to_owned(),
                "G1 Z0.35 F1200".to_owned(),
                "G1 F1800".to_owned(),
            ],
            entry_accel: None,
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
        assert!(c.contains("; generated-by dead-reckoning"));
        assert!(c.contains("; generated-at TS"));
        assert!(c.contains("; source-file part.gcode"));
        assert!(c.contains("; slicer 1.0"));
        assert!(c.contains("; filament PLA"));
        let m140 = c.find("M140 S60").unwrap();
        let m104 = c.find("M104 S210").unwrap();
        let m190 = c.find("M190 S60").unwrap();
        let m109 = c.find("M109 S210").unwrap();
        assert!(m140 < m104 && m104 < m190 && m190 < m109);
        let g28 = c.find("G28 X Y").unwrap();
        let repark = c.find("G0 X180 Y20 F6000").unwrap();
        let purge = c.find("G1 E5 F300").unwrap();
        let entry = c.find("G0 X30 Y30").unwrap();
        assert!(m109 < g28, "waits precede the re-home");
        assert!(g28 < repark, "the re-park follows the re-home");
        assert!(
            repark < purge,
            "the purge must run AFTER travelling back to the part-clear point"
        );
        assert!(purge < entry);
        assert_eq!(
            file.tail_bytes(),
            &original[usize::try_from(s.tail_offset).unwrap()..]
        );
    }

    /// Replays a generated preamble through this repo's own Klipper
    /// mirror ([`plr_gcode::GcodeState`]) and returns the NET filament
    /// delta in mm, plus the single most-negative move (the deepest
    /// retraction).
    ///
    /// Text assertions cannot see extrusion-MODE bugs: `G1 E-2` reads the
    /// same whether it retracts 2 mm or 7 mm. Only a replay that honors
    /// `M82`/`M83`/`G92` the way Klipper does can tell them apart, which
    /// is why the purge is tested this way.
    fn replay_filament(preamble: &str) -> (f64, f64) {
        use plr_gcode::{GcodeState, Line, LineIter};
        let mut state = GcodeState::new();
        let mut net = 0.0;
        let mut deepest = 0.0_f64;
        for line in LineIter::new(preamble.as_bytes(), 0).collect::<Vec<Line>>() {
            let before = state.last_position[3];
            let _ = state.apply(&line);
            // G92 re-frames the axis without moving filament.
            if line.command().map(|c| c.name.as_str()) == Some("G92") {
                continue;
            }
            let delta = state.last_position[3] - before;
            net += delta;
            deepest = deepest.min(delta);
        }
        (net, deepest)
    }

    /// BLOCKER regression: the built-in purge must extrude and retract by
    /// the CONFIGURED amounts, in every extrusion mode it could inherit.
    ///
    /// Klipper powers up in ABSOLUTE E, and the mode on entry to the
    /// purge is unknowable (a `CLEAN_NOZZLE` macro may have set either).
    /// Without an `M83`, `G1 E-{retract}` after `G1 E{amount}` moves the
    /// axis TO `-retract` — a retraction of `amount + retract`, which at
    /// `purge_amount = 50` pulls filament clear of a direct-drive
    /// extruder's gears. This asserts by REPLAY, not string match.
    #[test]
    fn the_built_in_purge_extrudes_and_retracts_exactly_the_configured_amounts() {
        for (amount, retract) in [(5.0, 2.0), (50.0, 1.0), (5.0, 0.0), (0.5, 0.5)] {
            let s = RecoveryFileSpec {
                purge: Some(PurgePlan::BuiltIn {
                    point: PARK,
                    z: None,
                    amount,
                    speed: 300.0,
                    retract,
                    travel_feed: 6000.0,
                }),
                // No entry moves: isolate the purge's own filament.
                entry_commands: vec![],
                ..spec()
            };
            let file = build_recovery_file(&s, b"; h\n", "TS");
            let (net, deepest) = replay_filament(&file.preamble_text());
            assert!(
                (net - (amount - retract)).abs() < 1e-9,
                "amount {amount} retract {retract}: net filament {net} must be \
                 {} (extrude minus retract), preamble:\n{}",
                amount - retract,
                file.preamble_text()
            );
            // The retraction MOVE itself must be exactly `retract` — this
            // is what the absolute-mode bug got catastrophically wrong.
            assert!(
                (deepest + retract).abs() < 1e-9,
                "amount {amount} retract {retract}: deepest single retraction was {deepest}, \
                 must be -{retract} (an absolute-E purge would retract amount+retract)"
            );
        }
    }

    /// The purge is immune to the extrusion mode it inherits: the same
    /// filament result whether the preceding (macro) g-code left the
    /// machine in absolute or relative E.
    #[test]
    fn the_purge_result_is_independent_of_the_inherited_e_mode() {
        let s = RecoveryFileSpec {
            purge: Some(PurgePlan::BuiltIn {
                point: PARK,
                z: None,
                amount: 5.0,
                speed: 300.0,
                retract: 2.0,
                travel_feed: 6000.0,
            }),
            entry_commands: vec![],
            ..spec()
        };
        let preamble = build_recovery_file(&s, b"; h\n", "TS")
            .preamble_text()
            .into_owned();
        let absolute_first = replay_filament(&format!("M82\n{preamble}"));
        let relative_first = replay_filament(&format!("M83\n{preamble}"));
        assert!((absolute_first.0 - relative_first.0).abs() < 1e-9);
        assert!((absolute_first.0 - 3.0).abs() < 1e-9, "{absolute_first:?}");
        assert!((absolute_first.1 + 2.0).abs() < 1e-9, "{absolute_first:?}");
    }

    /// Finding 1 regression: the built-in purge must never run at the
    /// homed XY that `G28 X Y` leaves the toolhead at.
    #[test]
    fn purge_never_runs_at_the_homed_xy() {
        let s = spec();
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1 E1\n", "TS");
        let text = file.preamble_text().into_owned();
        let g28 = text.find("G28 X Y").expect("re-home");
        let purge = text.find("G1 E5 F300").expect("purge");
        assert!(
            text[g28..purge].contains("G0 X180 Y20"),
            "a travel to the purge point must sit between G28 and the purge"
        );
        assert!(verify_heating_gate(&file, &s).is_ok());
    }

    // ---- purge precedence (all four paths) ---------------------------

    /// Path 1: `purge_enable = false` → nothing emitted, but the file
    /// still re-parks so the entry starts from a known clear point.
    #[test]
    fn purge_path_disabled_emits_nothing() {
        let s = park_only_spec();
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        let text = file.preamble_text().into_owned();
        assert!(!text.contains("G92 E0"));
        assert!(!text.contains("G1 E"));
        assert!(text.contains("G0 X180 Y20"), "the re-park is unconditional");
        assert!(verify_heating_gate(&file, &s).is_ok());
    }

    /// Path 2: a macro owns the purge — the call and NOTHING else, and
    /// plr does not reposition for it (the travel goes to the park point,
    /// not to any purge location).
    #[test]
    fn purge_path_macro_emits_only_the_call() {
        let s = RecoveryFileSpec {
            purge: Some(PurgePlan::Macro {
                call: "CLEAN_AND_PURGE".to_owned(),
            }),
            ..spec()
        };
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        let text = file.preamble_text().into_owned();
        assert!(text.contains("CLEAN_AND_PURGE"));
        // No built-in purge commands at all.
        assert!(!text.contains("G92 E0"));
        assert!(!text.contains("G1 E5"));
        // The post-home travel is the park point (plr does not
        // reposition for a macro).
        assert!(s
            .post_home_target()
            .0
            .iter()
            .zip(PARK)
            .all(|(a, b)| (a - b).abs() < 1e-12));
        assert!(text.find("G0 X180 Y20").unwrap() < text.find("CLEAN_AND_PURGE").unwrap());
        assert!(verify_heating_gate(&file, &s).is_ok());
    }

    /// Path 4 defaults: with no explicit coordinates the built-in purge
    /// happens at the reheat park point, and the travel goes there.
    #[test]
    fn purge_path_built_in_defaults_to_the_park_point() {
        let s = spec();
        let (pt, z) = s.post_home_target();
        assert!(pt.iter().zip(PARK).all(|(a, b)| (a - b).abs() < 1e-12));
        assert!(z.is_none());
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        let text = file.preamble_text().into_owned();
        assert!(text.contains("G0 X180 Y20 F6000"));
        assert!(verify_heating_gate(&file, &s).is_ok());
    }

    /// Path 4 explicit: coordinates, Z, amount, speed and retract are all
    /// honored, in the documented order.
    #[test]
    fn purge_path_built_in_honors_every_knob() {
        let s = RecoveryFileSpec {
            purge: Some(PurgePlan::BuiltIn {
                point: [12.5, 7.5],
                z: Some(0.6),
                amount: 8.0,
                speed: 250.0,
                retract: 1.5,
                travel_feed: 4200.0,
            }),
            ..spec()
        };
        // The post-home travel now targets the PURGE point, not the park.
        let (pt, z) = s.post_home_target();
        assert!((pt[0] - 12.5).abs() < 1e-12 && (pt[1] - 7.5).abs() < 1e-12);
        assert!((z.unwrap() - 0.6).abs() < 1e-12);
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        let text = file.preamble_text().into_owned();
        let travel = text.find("G0 X12.5 Y7.5 F4200").expect("purge travel");
        // The DESCENT uses the slow entry feedrate, not the travel
        // feedrate: it is a near-part move toward the bed.
        let descend = text.find("G1 Z0.6 F1200").expect("purge Z descent");
        assert!(
            !text.contains("G1 Z0.6 F4200"),
            "the descent must not use the fast travel feedrate: {text}"
        );
        let zero = text.find("G92 E0").expect("E zero");
        let push = text.find("G1 E8 F250").expect("purge extrusion");
        let retract = text.find("G1 E-1.5 F250").expect("purge retract");
        // Travel, THEN descend, then zero/extrude/retract/zero.
        assert!(travel < descend, "descend only after the XY travel lands");
        assert!(descend < zero && zero < push && push < retract);
        // The trailing G92 E0 re-zeroes after the retract.
        assert!(text.rfind("G92 E0").unwrap() > retract);
        assert!(verify_heating_gate(&file, &s).is_ok());
    }

    /// A zero retract emits no retract line at all.
    #[test]
    fn purge_retract_zero_emits_no_retract() {
        let file = build_recovery_file(&spec(), b"; h\nG1 X1 Y1\n", "TS");
        assert!(!file.preamble_text().contains("G1 E-"));
    }

    /// The gate follows the purge location: with a configured purge point
    /// the first post-`G28` motion must reach THAT point, not the park.
    #[test]
    fn the_gate_requires_travel_to_the_configured_purge_point() {
        let s = RecoveryFileSpec {
            purge: Some(PurgePlan::BuiltIn {
                point: [12.5, 7.5],
                z: None,
                amount: 5.0,
                speed: 300.0,
                retract: 0.0,
                travel_feed: 6000.0,
            }),
            ..spec()
        };
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        assert!(verify_heating_gate(&file, &s).is_ok());
        // Checked against a spec whose purge point differs: mismatch.
        let other = RecoveryFileSpec {
            purge: Some(PurgePlan::BuiltIn {
                point: [99.0, 99.0],
                z: None,
                amount: 5.0,
                speed: 300.0,
                retract: 0.0,
                travel_feed: 6000.0,
            }),
            ..spec()
        };
        assert!(matches!(
            verify_heating_gate(&file, &other),
            Err(HeatingGateViolation::ReParkMismatch { .. })
        ));
    }

    // ---- byte fidelity ----------------------------------------------

    #[test]
    fn tail_is_byte_verbatim_for_non_utf8_originals() {
        let original: Vec<u8> = b"; caf\xE9 \xFF\nG1 X1 Y1 E1\nG1 X2 Y2 E2\n".to_vec();
        let mut s = spec();
        s.tail_offset = u64::try_from(original.iter().position(|&b| b == b'\n').unwrap() + 1)
            .expect("offset fits");
        let file = build_recovery_file(&s, &original, "TS");
        let expected = &original[usize::try_from(s.tail_offset).unwrap()..];
        assert_eq!(file.tail_bytes(), expected);
        assert!(!file
            .tail_bytes()
            .windows(3)
            .any(|w| w == [0xEF, 0xBF, 0xBD]));

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
        assert_eq!(file.preamble_text().matches("; meta ").count(), 200);
    }

    // ---- heating gate -----------------------------------------------

    #[test]
    fn heating_gate_holds_for_a_normal_file() {
        let s = spec();
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1 E1\n", "TS");
        assert!(verify_heating_gate(&file, &s).is_ok());
    }

    #[test]
    fn heating_gate_holds_without_a_bed() {
        let s = RecoveryFileSpec {
            bed: None,
            ..spec()
        };
        let file = build_recovery_file(&s, b"; h\nG1 X1 Y1\n", "TS");
        assert!(verify_heating_gate(&file, &s).is_ok());
    }

    #[test]
    fn heating_gate_catches_an_xy_move_before_the_wait() {
        let file = hand_built("M104 S210\nG1 X5 Y5\nM109 S210\nG28 X Y\nG0 X180 Y20\n");
        assert_eq!(
            verify_heating_gate(&file, &park_only_spec()),
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
                "M104 S210\n{arc} X5 Y5 I1 J1\nM109 S210\nG28 X Y\nG0 X180 Y20\n"
            ));
            assert_eq!(
                verify_heating_gate(&file, &park_only_spec()),
                Err(HeatingGateViolation::XyBeforeTempWait {
                    command: arc.to_owned()
                }),
                "{arc} must be treated as motion"
            );
        }
        let file = hand_built("M104 S210\nM109 S210\nG3 X5 Y5 I1 J1\nG28 X Y\nG0 X180 Y20\n");
        assert_eq!(
            verify_heating_gate(&file, &park_only_spec()),
            Err(HeatingGateViolation::EntryBeforeReHome {
                command: "G3".to_owned()
            })
        );
    }

    /// Finding 5 regression: a Z DESCENT before the waits is a violation;
    /// only a provable relative lift is tolerated.
    #[test]
    fn heating_gate_catches_a_z_descent_before_the_wait() {
        let s = park_only_spec();
        // Absolute Z: direction unknowable without the runtime Z.
        let file = hand_built("M104 S210\nG90\nG1 Z-20\nM109 S210\nG28 X Y\nG0 X180 Y20\n");
        assert_eq!(
            verify_heating_gate(&file, &s),
            Err(HeatingGateViolation::ZDescentBeforeTempWait {
                command: "G1".to_owned()
            })
        );
        // Relative negative Z: an unambiguous descent.
        let file = hand_built("M104 S210\nG91\nG1 Z-20\nM109 S210\nG28 X Y\nG0 X180 Y20\n");
        assert_eq!(
            verify_heating_gate(&file, &s),
            Err(HeatingGateViolation::ZDescentBeforeTempWait {
                command: "G1".to_owned()
            })
        );
        // A relative LIFT is the documented carve-out and passes.
        let file = hand_built("M104 S210\nG91\nG1 Z5\nG90\nM109 S210\nG28 X Y\nG0 X180 Y20\n");
        assert!(
            verify_heating_gate(&file, &s).is_ok(),
            "a provable relative lift must pass: {}",
            file.preamble_text()
        );
        // An absolute Z even in the "up" direction is still unprovable.
        let file = hand_built("M104 S210\nG90\nG1 Z200\nM109 S210\nG28 X Y\nG0 X180 Y20\n");
        assert_eq!(
            verify_heating_gate(&file, &s),
            Err(HeatingGateViolation::ZDescentBeforeTempWait {
                command: "G1".to_owned()
            })
        );
    }

    /// Rebuilds a generated file from a mutated copy of its REAL preamble.
    /// Used to prove the re-park guard bites on files this generator
    /// actually produces — a hand-built stub could pass a rule that is
    /// vacuous in practice.
    fn mutate_real_preamble(
        s: &RecoveryFileSpec,
        original: &[u8],
        edit: impl Fn(Vec<String>) -> Vec<String>,
    ) -> GeneratedRecoveryFile {
        let built = build_recovery_file(s, original, "TS");
        let lines: Vec<String> = built.preamble_text().lines().map(str::to_owned).collect();
        let mut preamble = edit(lines).join("\n");
        preamble.push('\n');
        let tail_start = preamble.len();
        let mut content = preamble.into_bytes();
        content.extend_from_slice(built.tail_bytes());
        GeneratedRecoveryFile {
            content,
            tail_start,
        }
    }

    /// Finding 1 regression on the REAL generated preamble: deleting just
    /// the re-park line must fire.
    ///
    /// The earlier "any XY motion after G28" formulation could never fail
    /// here — the entry moves always contain `G0 X.. Y..` — so this is
    /// what proves the guard is real. With the purge DISABLED the first
    /// motion after the re-home becomes the entry's Z hop (no XY), which
    /// is `MissingRePark`.
    #[test]
    fn deleting_the_re_park_from_a_real_file_is_caught() {
        let s = park_only_spec();
        let original = b"; h\nG1 X1 Y1 E1\n";
        assert!(verify_heating_gate(&build_recovery_file(&s, original, "TS"), &s).is_ok());

        let mutated = mutate_real_preamble(&s, original, |lines| {
            lines
                .into_iter()
                .filter(|l| !l.starts_with("G0 X180 Y20"))
                .collect()
        });
        assert_eq!(
            verify_heating_gate(&mutated, &s),
            Err(HeatingGateViolation::MissingRePark),
            "preamble:\n{}",
            mutated.preamble_text()
        );
    }

    /// Same deletion with the built-in purge ENABLED: the purge is now
    /// the first thing after the re-home, which is the more precise
    /// `PurgeBeforeRePark` — it would extrude at the homed XY.
    #[test]
    fn deleting_the_re_park_with_a_purge_is_caught_as_purge_first() {
        let s = spec();
        let mutated = mutate_real_preamble(&s, b"; h\nG1 X1 Y1 E1\n", |lines| {
            lines
                .into_iter()
                .filter(|l| !l.starts_with("G0 X180 Y20"))
                .collect()
        });
        assert_eq!(
            verify_heating_gate(&mutated, &s),
            Err(HeatingGateViolation::PurgeBeforeRePark {
                command: "G92".to_owned()
            }),
            "preamble:\n{}",
            mutated.preamble_text()
        );
    }

    /// Finding 1 regression: moving the re-park to AFTER the purge fires.
    #[test]
    fn moving_the_re_park_after_the_purge_is_caught() {
        let s = spec();
        let mutated = mutate_real_preamble(&s, b"; h\nG1 X1 Y1 E1\n", |lines| {
            let mut out: Vec<String> = lines
                .iter()
                .filter(|l| !l.starts_with("G0 X180 Y20"))
                .cloned()
                .collect();
            let after_purge = out
                .iter()
                .rposition(|l| l == "G92 E0")
                .expect("built-in purge")
                + 1;
            out.insert(after_purge, "G0 X180 Y20 F6000".to_owned());
            out
        });
        assert_eq!(
            verify_heating_gate(&mutated, &s),
            Err(HeatingGateViolation::PurgeBeforeRePark {
                command: "G92".to_owned()
            }),
            "preamble:\n{}",
            mutated.preamble_text()
        );
    }

    /// Finding 1 regression: a re-park to the WRONG point is caught with
    /// both coordinates named.
    #[test]
    fn a_re_park_to_the_wrong_point_is_caught() {
        let s = spec();
        let mutated = mutate_real_preamble(&s, b"; h\nG1 X1 Y1 E1\n", |lines| {
            lines
                .into_iter()
                .map(|l| {
                    if l.starts_with("G0 X180 Y20") {
                        "G0 X5 Y5 F6000".to_owned()
                    } else {
                        l
                    }
                })
                .collect()
        });
        let err = verify_heating_gate(&mutated, &s).unwrap_err();
        assert!(
            matches!(&err, HeatingGateViolation::ReParkMismatch { found_x, park_x, .. }
                if found_x == "5" && park_x == "180"),
            "{err:?}"
        );
    }

    /// Item 9(a) regression, on the REAL preamble: flipping the `G90`
    /// before the re-park to `G91` must fire.
    ///
    /// This mutation previously defeated BOTH guards at once — the gate
    /// ignored coordinate mode when matching the re-park XY, and
    /// `preflight_recovery_file` skips relative moves entirely because
    /// their coordinates are deltas rather than positions.
    #[test]
    fn a_relative_re_park_is_caught() {
        let s = spec();
        let mutated = mutate_real_preamble(&s, b"; h\nG1 X1 Y1 E1\n", |lines| {
            let mut seen_home = false;
            lines
                .into_iter()
                .map(|l| {
                    if l == "G28 X Y" {
                        seen_home = true;
                        l
                    } else if seen_home && l == "G90" {
                        seen_home = false; // only the first one after G28
                        "G91".to_owned()
                    } else {
                        l
                    }
                })
                .collect()
        });
        assert_eq!(
            verify_heating_gate(&mutated, &s),
            Err(HeatingGateViolation::RelativeRePark {
                command: "G0".to_owned()
            }),
            "preamble:\n{}",
            mutated.preamble_text()
        );
    }

    /// Item 9(b) regression: an extrusion between the temperature waits
    /// and the re-home slips past the XY-only positioning rule, but must
    /// still be refused — it would purge at the parked position.
    #[test]
    fn an_extrusion_before_the_re_home_is_caught() {
        let file = hand_built("M104 S210\nM109 S210\nG1 E5 F300\nG28 X Y\nG0 X180 Y20\n");
        assert_eq!(
            verify_heating_gate(&file, &park_only_spec()),
            Err(HeatingGateViolation::PurgeBeforeRePark {
                command: "G1".to_owned()
            })
        );
        // A bare `G92 E0` there is caught too.
        let file = hand_built("M104 S210\nM109 S210\nG92 E0\nG28 X Y\nG0 X180 Y20\n");
        assert_eq!(
            verify_heating_gate(&file, &park_only_spec()),
            Err(HeatingGateViolation::PurgeBeforeRePark {
                command: "G92".to_owned()
            })
        );
    }

    #[test]
    fn heating_gate_catches_a_missing_re_park() {
        let file = hand_built("M104 S210\nM109 S210\nG28 X Y\n");
        assert_eq!(
            verify_heating_gate(&file, &park_only_spec()),
            Err(HeatingGateViolation::MissingRePark)
        );
        let file = hand_built("M104 S210\nM109 S210\nG28 X Y\nG92 E0\nG1 E5 F300\n");
        assert_eq!(
            verify_heating_gate(&file, &spec()),
            Err(HeatingGateViolation::PurgeBeforeRePark {
                command: "G92".to_owned()
            })
        );
    }

    #[test]
    fn heating_gate_catches_missing_and_early_rehome() {
        let s = park_only_spec();
        let no_wait = hand_built("M104 S210\nG28 X Y\n");
        assert_eq!(
            verify_heating_gate(&no_wait, &s),
            Err(HeatingGateViolation::MissingNozzleWait)
        );
        let no_bed_wait = hand_built("M140 S60\nM104 S210\nM109 S210\nG28 X Y\n");
        assert_eq!(
            verify_heating_gate(&no_bed_wait, &s),
            Err(HeatingGateViolation::MissingBedWait)
        );
        let early = hand_built("M104 S210\nG28 X Y\nM109 S210\n");
        assert_eq!(
            verify_heating_gate(&early, &s),
            Err(HeatingGateViolation::ReHomeBeforeTempWait)
        );
        let no_home = hand_built("M104 S210\nM109 S210\n");
        assert_eq!(
            verify_heating_gate(&no_home, &s),
            Err(HeatingGateViolation::MissingReHome)
        );
    }

    #[test]
    fn heating_gate_catches_entry_before_rehome() {
        let file = hand_built("M104 S210\nM109 S210\nG1 X5 Y5\nG28 X Y\nG0 X180 Y20\n");
        assert_eq!(
            verify_heating_gate(&file, &park_only_spec()),
            Err(HeatingGateViolation::EntryBeforeReHome {
                command: "G1".to_owned()
            })
        );
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

    /// The entry moves live in THIS file, so `accel_entry` has to be
    /// applied here — the plan's phases never touch the motion that
    /// actually descends toward the part.
    #[test]
    fn the_entry_accel_pair_wraps_the_entry_moves() {
        let mut s = spec();
        s.entry_accel = Some((600.0, 3_000.0));
        let file = build_recovery_file(&s, b"", "TS");
        let text = file.preamble_text().into_owned();
        let lines: Vec<&str> = text.lines().collect();
        let clamp = lines
            .iter()
            .position(|l| *l == "SET_VELOCITY_LIMIT ACCEL=600")
            .expect("clamp");
        let restore = lines
            .iter()
            .position(|l| *l == "SET_VELOCITY_LIMIT ACCEL=3000")
            .expect("restore");
        let first_entry = lines
            .iter()
            .position(|l| *l == "G0 Z1.35 F1200")
            .expect("first entry move");
        let last_entry = lines
            .iter()
            .position(|l| *l == "G1 F1800")
            .expect("last entry move");
        assert!(
            clamp < first_entry,
            "the clamp must precede the entry moves"
        );
        assert!(
            last_entry < restore,
            "the restore must follow the entry moves"
        );
        // The clamp comes AFTER the blocking heat waits, so the heating
        // gate is untouched by it.
        let m109 = lines
            .iter()
            .position(|l| l.starts_with("M109"))
            .expect("M109");
        assert!(m109 < clamp);
        verify_heating_gate(&file, &s).expect("the heating gate still holds");
    }

    /// The pair is emitted as a PAIR, always. An entry block that happens
    /// to be empty must not leave a clamp with no restore: that would
    /// govern the whole remaining print.
    #[test]
    fn the_entry_accel_restore_is_emitted_even_with_no_entry_moves() {
        let mut s = spec();
        s.entry_commands = vec![];
        s.entry_accel = Some((600.0, 3_000.0));
        let text = build_recovery_file(&s, b"", "TS")
            .preamble_text()
            .into_owned();
        let clamps: Vec<&str> = text
            .lines()
            .filter(|l| l.starts_with("SET_VELOCITY_LIMIT"))
            .collect();
        assert_eq!(
            clamps,
            vec![
                "SET_VELOCITY_LIMIT ACCEL=600",
                "SET_VELOCITY_LIMIT ACCEL=3000"
            ],
            "a clamp without its restore would outlive the recovery"
        );
    }

    /// Unset: not a single byte of accel machinery in the file.
    #[test]
    fn no_entry_accel_emits_nothing() {
        let s = spec();
        assert_eq!(s.entry_accel, None);
        let with_none = build_recovery_file(&s, b"", "TS");
        assert!(!with_none.preamble_text().contains("SET_VELOCITY_LIMIT"));
        // And the file is byte-identical to one built before the field
        // existed (the field is `#[serde(default)]` and emits nothing).
        let mut other = spec();
        other.entry_accel = None;
        assert_eq!(build_recovery_file(&other, b"", "TS"), with_none);
    }
}
