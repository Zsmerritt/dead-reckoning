//! The completion gate: *is there any printing work left in this file
//! after byte `offset`?*
//!
//! # Why a percentage cannot answer this
//!
//! Every mainstream slicer appends a **footer** after the last extruding
//! move: the end g-code (cooldown, retract, park, motors off) and then a
//! serialized copy of the whole print profile as comments. A stock
//! `PrusaSlicer` footer is one `; prusaslicer_config = begin` block of
//! ~324 `; key = value` lines — about 14 KB — and `OrcaSlicer` writes the
//! equivalent between `; CONFIG_BLOCK_START` and `; CONFIG_BLOCK_END`.
//!
//! So a **functionally complete** print stops roughly 14.5 KB short of
//! EOF, and the same absolute distance reads as a wildly different
//! percentage depending on file size:
//!
//! | file size | complete print reads as |
//! |-----------|-------------------------|
//! | 300 KB    | ~95 %                   |
//! | 2 MB      | ~99 %                   |
//! | 20 MB     | ~99.93 %                |
//!
//! No percentage threshold separates "finished" from "died on the last
//! layer" across that range. The only honest test is a *content* test:
//! replay the remainder of the file and ask whether any of it deposits
//! plastic. That is what [`remaining_work`] does.
//!
//! # The safety asymmetry
//!
//! This module exists to **suppress** a recovery offer, so it may only
//! ever answer "complete" on positive proof. Every way of not knowing —
//! an unreadable tail, a replay that failed, a byte cap exceeded — must
//! reach the caller as [`WorkUnknown`] (or never reach this module at
//! all), and the caller must then announce. A false offer wastes the
//! operator's time; a suppressed offer loses their print.
//!
//! # What counts as work
//!
//! Exactly one thing: a `G0`/`G1`/`G2`/`G3` whose *planned* move yields a
//! positive E displacement in Klipper-internal extruder units — i.e.
//! [`MoveKind::Extrusion`] in the [`LayerModel`]. No new extruder logic
//! is introduced here and none is needed: `plr-gcode`'s state machine
//! already replays `M82`/`M83`, `G92 E`, and `extrude_factor` byte-exactly
//! before classifying a move, and arcs are already decomposed into chords
//! that carry their share of E. This module only *reads* that
//! classification.
//!
//! Everything else in a footer is therefore not work, without needing a
//! per-command allow-list: comments and blank lines, `M104 S0`/`M140 S0`/
//! `M107`, `M84`/`M18`, `G28`, travel and retract moves (no positive E),
//! `M400`, `M117`/`M118`/`RESPOND`, `SET_*`/`SAVE_*`/`RESTORE_*`,
//! `EXCLUDE_OBJECT_*`, `M900`/`M204`/`M205`, `G4`, and bare macro calls
//! such as `PRINT_END`.
//!
//! # Cancelled objects
//!
//! If the operator cancelled every object that still had deposition
//! ahead of the stop point, the file *still contains* those lines —
//! Klipper skips them at execution time, so the g-code is unchanged.
//! [`remaining_work`] therefore takes an optional [`ExclusionOracle`] and
//! ignores deposition attributed to an excluded object.
//!
//! It does so **only when the oracle is conclusive**. An inconclusive
//! exclusion picture counts excluded work as work, because a
//! false-positive offer is recoverable and a false negative is not.
//! Deposition attributed to no object at all (skirt, brim, prime line,
//! wipe tower) always counts: `None` means "not attributable", never
//! "excluded".

use serde::{Deserialize, Serialize};

use plr_gcode::LineIter;

use crate::model::{LayerModel, ModelStop, MoveKind, SimMove};

/// How many un-run command names [`remaining_work`] will name in
/// [`RemainingWork::EndSequenceOnly`]. A stock end g-code is a dozen
/// commands; the cap only bites on pathological input and is reported
/// through [`RemainingWork::commands_truncated`].
pub const MAX_NAMED_COMMANDS: usize = 32;

/// Answers "is this object cancelled?" for [`remaining_work`].
///
/// # Why a trait instead of `&plr_reconstruct::ExclusionReport`
///
/// `plr_reconstruct::ExclusionReport` is the production implementor and
/// the only one that matters, but this crate sits directly on top of
/// `plr-gcode` and nothing else: it is pure geometry over replayed
/// g-code, and `plr-recovery` is the layer that composes it with
/// reconstruction. Taking the report by value here would pull `plr-wal`
/// and `plr-klipper` in behind it and invert that layering for one
/// two-method query. The adapter lives in the caller (`plrd`), which
/// already owns both crates.
pub trait ExclusionOracle {
    /// `true` only when nothing the log records as lost postdates the
    /// newest exclusion observation and that observation is fresh — i.e.
    /// [`is_excluded`](Self::is_excluded) may be trusted to say *no*.
    ///
    /// When this is `false`, [`remaining_work`] ignores the oracle
    /// entirely and counts excluded deposition as work.
    fn is_conclusive(&self) -> bool;

    /// `true` when `object` (upper-cased, as Klipper stores it) is in the
    /// cancelled set.
    fn is_excluded(&self, object: &str) -> bool;
}

/// What is left to print after the tested offset.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum RemainingWork {
    /// Plastic still has to be deposited: the print is unfinished and a
    /// recovery offer is warranted.
    Extrusion {
        /// Byte offset of the first depositing line that still counts —
        /// a line boundary, so it is `M26`-safe.
        first_offset: u64,
        /// How many depositing moves still count.
        moves: u32,
        /// How many distinct layers those moves span.
        layers: u32,
    },
    /// No deposition remains, but un-run commands do: the end g-code.
    ///
    /// The print is **functionally complete**. The commands are named so
    /// the operator knows exactly what did not run (typically the
    /// cooldown, the park, and motors-off) and can decide for themselves;
    /// callers must **not** offer to execute them. A `PRINT_END` macro
    /// routinely homes, drops the bed, or moves Z, and none of the
    /// envelope or pre-flight analysis that guards a real recovery plan
    /// applies to an opaque macro body.
    EndSequenceOnly {
        /// Command names in file order (duplicates kept — "`M104` twice"
        /// is information), capped at [`MAX_NAMED_COMMANDS`].
        commands: Vec<String>,
    },
    /// Nothing at all remains but comments and blank lines — the print
    /// ran its end g-code and stopped inside the trailing config block.
    Nothing,
}

impl RemainingWork {
    /// `true` when no deposition remains, i.e. the print is functionally
    /// complete and a recovery must **not** be announced.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        !matches!(self, Self::Extrusion { .. })
    }

    /// `true` when the named-command list hit [`MAX_NAMED_COMMANDS`] and
    /// more commands follow.
    #[must_use]
    pub fn commands_truncated(&self) -> bool {
        match self {
            Self::EndSequenceOnly { commands } => commands.len() >= MAX_NAMED_COMMANDS,
            Self::Extrusion { .. } | Self::Nothing => false,
        }
    }
}

/// Why the remaining work could not be established.
///
/// Every variant obliges the caller to **announce** the recovery: this
/// is the "we do not know" channel, kept separate from
/// [`RemainingWork`] so that no absence of evidence can be mistaken for
/// proof of completion.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, thiserror::Error)]
pub enum WorkUnknown {
    /// The g-code replay stopped early on a line it could not apply, so
    /// the move stream does not cover the whole remainder.
    #[error("g-code replay failed at byte {offset}; the remainder was not modeled")]
    ReplayFailed {
        /// Byte offset of the offending line.
        offset: u64,
    },
    /// The modeled window does not reach the tested offset, so nothing
    /// was replayed at or after it.
    #[error("the modeled window ends at byte {window_end}, before the tested offset {offset}")]
    OffsetOutsideWindow {
        /// The offset that was asked about.
        offset: u64,
        /// One past the last byte the window covers.
        window_end: u64,
    },
    /// The remainder holds command lines, but not one of them is a
    /// *traditional* g-code command — so it does not look like g-code at
    /// all, and "no extrusion in it" is not a statement about a print.
    ///
    /// See [`remaining_work`]'s "The remainder has to be g-code".
    #[error(
        "the remainder from byte {offset} has {commands} command line(s) but no traditional \
         g-code among them; it does not look like an end sequence"
    )]
    UnintelligibleRemainder {
        /// The offset the remainder starts at.
        offset: u64,
        /// How many command lines were found.
        commands: u32,
    },
    /// The file contradicts the anchor's claim about one of the two modes
    /// that decide E displacement, over part of the tested region, so the
    /// classification there cannot be trusted.
    ///
    /// See [`remaining_work`]'s "Trusting the extruder frame".
    #[error(
        "the file sets {} {} at byte {offset}, contradicting the anchor's claim of {} \
         over the tested region before it",
        if *file_absolute { "absolute" } else { "relative" },
        axis.name(),
        if *file_absolute { "relative" } else { "absolute" }
    )]
    ExtrudeModeContradiction {
        /// Byte offset of the `M82`/`M83`/`G90`/`G91` that contradicts the
        /// anchor.
        offset: u64,
        /// Which of the two modes the command sets.
        axis: ModeAxis,
        /// The value that command sets (`true` = absolute). The anchor
        /// claimed the opposite, which is what makes it a contradiction.
        file_absolute: bool,
    },
}

/// Which of the two mode flags a command sets.
///
/// **Both** decide E displacement. `plr-gcode` computes the effective
/// extruder frame as their **conjunction** — `state.rs`:
/// `let absolute = self.absolute_coord && self.absolute_extrude;`, citing
/// Klipper's `gcode_move.py` (`absolute_extrude` is only consulted when
/// `absolute_coord` is set, so `G91` makes E relative whatever `M82` said).
/// A guard that watched `M82`/`M83` alone would miss a wrong
/// `absolute_coordinates` entirely.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ModeAxis {
    /// `G90`/`G91` — `GcodeState::absolute_coord`.
    Coordinates,
    /// `M82`/`M83` — `GcodeState::absolute_extrude`.
    Extrusion,
}

impl ModeAxis {
    /// Human name for the error message.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Coordinates => "coordinates",
            Self::Extrusion => "extrusion",
        }
    }
}

/// The extrusion frame the replay started from — the anchor context's
/// `gcode_move` mode flags.
///
/// Both are needed: see [`ModeAxis`] for why the effective frame is their
/// conjunction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AnchorFrame {
    /// `gcode_move.absolute_coordinates` (`G90`/`G91`).
    pub absolute_coordinates: bool,
    /// `gcode_move.absolute_extrude` (`M82`/`M83`).
    pub absolute_extrude: bool,
}

impl AnchorFrame {
    /// The anchor's claim for one axis.
    const fn claim(self, axis: ModeAxis) -> bool {
        match axis {
            ModeAxis::Coordinates => self.absolute_coordinates,
            ModeAxis::Extrusion => self.absolute_extrude,
        }
    }
}

/// A mode command, which axis it sets, and where it sits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ModeCommand {
    offset: u64,
    axis: ModeAxis,
    absolute: bool,
}

/// The mode command a line carries, if any.
fn mode_command(line: &plr_gcode::Line) -> Option<ModeCommand> {
    let (axis, absolute) = match line.command()?.name.as_str() {
        "G90" => (ModeAxis::Coordinates, true),
        "G91" => (ModeAxis::Coordinates, false),
        "M82" => (ModeAxis::Extrusion, true),
        "M83" => (ModeAxis::Extrusion, false),
        _ => return None,
    };
    Some(ModeCommand {
        offset: line.span.start,
        axis,
        absolute,
    })
}

/// Is any printing work left in `tail` after `offset`?
///
/// * `model` — a [`LayerModel`] built from `tail` with the same
///   `base_offset`, replaying from the interpreter state that was live at
///   `base_offset` (a WAL-reconstructed [`plr_gcode::GcodeState`]). The
///   E frame must be correct: a stale absolute-E baseline can make a real
///   extrusion look like a retract, which would suppress a needed offer.
/// * `tail`/`base_offset` — the bytes the model was built from, and the
///   stream offset they begin at. Re-scanned here (a `LineIter` census)
///   to name the un-run commands, exactly as
///   `plr_recovery::preheat::scan_file_temps` re-scans for temperatures.
/// * `offset` — the offset to test at. Pass the **low** end of the
///   possible-stop window (`stop_set.file_window.start`): the last
///   *durable* file position overstates progress by up to the processing
///   lead, and the window's high end is deliberately the maximum of an
///   ambiguous range, which would hide work.
/// * `anchor` — the mode flags the replay *started* from, i.e. the
///   `absolute_coord`/`absolute_extrude` of the state passed to
///   [`crate::model::build_layer_model`]. Used only by the trust check
///   below; see "Trusting the extruder frame".
/// * `exclusions` — see [`ExclusionOracle`]; `None` counts every
///   deposition.
///
/// Total: never panics for any bytes, any model, any offset.
///
/// # Trusting the extruder frame
///
/// Whether a move deposits is decided by its E displacement, and that
/// depends on the mode in force: relative E means the displacement *is* the
/// E parameter, absolute E means it is the parameter minus a running
/// baseline. Replay the wrong mode and the classification is wrong.
///
/// **Two** flags decide it, as a conjunction — `plr-gcode`'s `state.rs`
/// computes `let absolute = self.absolute_coord && self.absolute_extrude;`
/// after Klipper's `gcode_move.py`, so `G91` forces relative E whatever
/// `M82` said. See [`ModeAxis`]. Each flag is checked independently.
///
/// For each flag, over the tested region its value comes from one of two
/// places:
///
/// * **the file**, if the corresponding command (`G90`/`G91` or
///   `M82`/`M83`) sits in `[base_offset, offset]` — the replay executed it,
///   so the value at `offset` is whatever the file said. Nothing to doubt.
/// * **`anchor`** otherwise. That value is correct by construction: it
///   mirrors Klipper's own `gcode_move` status, read straight from the
///   printer and journaled verbatim.
///
/// In the second case the file can still *contradict* the claim: a command
/// after `offset` setting the opposite value means everything in
/// `[offset, that command)` was classified under a flag the file disagrees
/// with. This function then refuses to answer
/// ([`WorkUnknown::ExtrudeModeContradiction`]) rather than suppress.
///
/// Checking the flags independently is deliberately conservative: a wrong
/// `absolute_coordinates` cannot change the effective frame when
/// `absolute_extrude` is already `false` (the conjunction is `false`
/// either way), and this still refuses. Over-refusing costs a dry run.
///
/// **This cannot fire in production without the WAL misreporting a field it
/// reads directly from Klipper.** It exists because the cost of being wrong
/// is asymmetric: getting the mode wrong in one direction over-reports
/// extrusion and *announces* a recovery that was not needed — the operator
/// does a dry run and finds out — while the other direction under-reports
/// and *suppresses*, and a suppressed offer means the operator is never told
/// their print could have been resumed. That is a lost print they had no
/// chance to save. This module may only suppress on positive proof, and a
/// mode contradiction is precisely a reason to doubt the proof, so it
/// forfeits suppression even where the arithmetic would have come out right.
///
/// # The remainder has to be g-code
///
/// `Nothing` and `EndSequenceOnly` both report completion, so between them
/// they accept *any* remainder that holds no positive extrusion — including
/// one that is not g-code. A file whose tail was replaced by four hundred
/// bytes of `x` parses as a single command line named `XXXX…`: no extrusion,
/// so "complete".
///
/// The discriminator is the one Klipper itself uses. Its dispatcher splits
/// commands into *traditional* (a letter plus a number — `G1`, `M104`) and
/// *extended* (`PRINT_END`, `SET_FAN_SPEED`), and `plr-gcode` mirrors that
/// split in [`plr_gcode::CommandParams`], citing
/// `GCodeDispatch.is_traditional_gcode`. Every end sequence in the fixture
/// corpus contains traditional commands — the cooldowns, the park, the
/// motors-off are all `M`-codes — while arbitrary bytes tokenize as extended
/// names. So: a remainder with command lines but **no traditional command**
/// is [`WorkUnknown::UnintelligibleRemainder`], and a remainder with no
/// command lines at all stays `Nothing` (a config-block tail, which is
/// exactly what a trailing comment block is).
///
/// The cost of being wrong here is one refusal: a hypothetical footer built
/// only from extended macros (`PRINT_END` and nothing else) announces
/// instead of suppressing. No slicer in the corpus emits that, and
/// announcing is the recoverable direction.
///
/// # Errors
///
/// [`WorkUnknown`] when the model does not actually cover the remainder, when
/// the extruder frame over it cannot be trusted, or when the remainder is not
/// intelligible as g-code — the caller must announce rather than treat any of
/// those as completion.
pub fn remaining_work(
    model: &LayerModel,
    tail: &[u8],
    base_offset: u64,
    offset: u64,
    anchor: AnchorFrame,
    exclusions: Option<&dyn ExclusionOracle>,
) -> Result<RemainingWork, WorkUnknown> {
    // A replay that stopped early did not model everything after it, so
    // "no extrusion found" would be an artefact of the truncation.
    if let ModelStop::LineError { offset: at, .. } = &model.stop {
        return Err(WorkUnknown::ReplayFailed { offset: *at });
    }
    let window_end = base_offset.saturating_add(tail.len() as u64);
    // `offset == window_end` is legitimate and means "the file ends
    // here": nothing remains. Beyond it, the model says nothing.
    if offset > window_end {
        return Err(WorkUnknown::OffsetOutsideWindow { offset, window_end });
    }
    check_extrude_mode(model, tail, base_offset, offset, anchor)?;

    let counted: Vec<&SimMove> = model
        .moves
        .iter()
        .filter(|m| m.kind == MoveKind::Extrusion && m.span.start >= offset)
        .filter(|m| counts_as_work(m, exclusions))
        .collect();
    if let Some(first) = counted.first() {
        let mut layers: Vec<u32> = counted.iter().filter_map(|m| m.layer).collect();
        layers.sort_unstable();
        layers.dedup();
        return Ok(RemainingWork::Extrusion {
            first_offset: first.span.start,
            moves: u32::try_from(counted.len()).unwrap_or(u32::MAX),
            layers: u32::try_from(layers.len()).unwrap_or(u32::MAX),
        });
    }

    let census = census(tail, base_offset, offset);
    if census.names.is_empty() {
        // No command lines at all: comments and blanks, i.e. a config-block
        // tail. Intelligible, and nothing remains.
        return Ok(RemainingWork::Nothing);
    }
    if !census.saw_traditional {
        return Err(WorkUnknown::UnintelligibleRemainder {
            offset,
            commands: census.command_lines,
        });
    }
    Ok(RemainingWork::EndSequenceOnly {
        commands: census.names,
    })
}

/// The extruder-frame trust check described on [`remaining_work`].
///
/// One forward pass over the window, tracking each [`ModeAxis`]
/// independently. For an axis:
///
/// 1. a command in `[base_offset, offset]` settles it — the file, not the
///    anchor, governs the tested region, so nothing can contradict it;
/// 2. otherwise the **first** command after `offset` governs from that
///    point. If it sets the opposite of the anchor's claim, the region
///    before it was classified under a value the file disagrees with:
///    refuse. Only the first matters — after it the value is
///    file-established, so a later command is an ordinary change from a
///    known state.
///
/// A command sitting exactly *at* `offset` counts as case 1: nothing in the
/// tested region precedes it.
///
/// A contradiction must also be *material* — the doubtful span
/// `[offset, command)` must actually contain a move. The refusal exists
/// because moves in that span may be mis-classified; a span of comments,
/// temperature commands and the like has nothing to mis-classify. This is
/// what keeps a whole-file replay from a fresh interpreter state — where the
/// file declares its modes a few bytes into the header, after a tested
/// offset of 0 — on the trustworthy path.
///
/// # Errors
///
/// [`WorkUnknown::ExtrudeModeContradiction`] for the first material
/// contradiction found, in file order.
fn check_extrude_mode(
    model: &LayerModel,
    tail: &[u8],
    base_offset: u64,
    offset: u64,
    anchor: AnchorFrame,
) -> Result<(), WorkUnknown> {
    // Per axis: `true` once the file has settled it, so later commands on
    // that axis are changes from a known state rather than contradictions.
    let mut settled = [false; 2];
    let index = |axis: ModeAxis| usize::from(axis == ModeAxis::Extrusion);
    for line in LineIter::new(tail, base_offset) {
        let Some(mode) = mode_command(&line) else {
            continue;
        };
        let slot = index(mode.axis);
        if settled[slot] {
            continue;
        }
        settled[slot] = true;
        if mode.offset <= offset {
            // Case 1: the file governs this axis over the whole tested
            // region.
            continue;
        }
        // Case 2: the first command for this axis after the tested offset.
        if mode.absolute == anchor.claim(mode.axis) {
            continue;
        }
        let doubtful_span_has_moves = model
            .moves
            .iter()
            .any(|m| m.span.start >= offset && m.span.start < mode.offset);
        if !doubtful_span_has_moves {
            continue;
        }
        return Err(WorkUnknown::ExtrudeModeContradiction {
            offset: mode.offset,
            axis: mode.axis,
            file_absolute: mode.absolute,
        });
    }
    // Any axis with no command in the window runs on the anchor's claim,
    // which is correct by construction (it mirrors Klipper's `gcode_move`
    // status). This is the common path — slicers declare both modes once, in
    // the start g-code, which a mid-print window does not contain.
    Ok(())
}

/// Whether one depositing move counts against the exclusion picture.
///
/// The conservative direction is "counts": an absent or inconclusive
/// oracle, and deposition attributed to no object, all count.
fn counts_as_work(move_: &SimMove, exclusions: Option<&dyn ExclusionOracle>) -> bool {
    let Some(oracle) = exclusions else {
        return true;
    };
    if !oracle.is_conclusive() {
        return true;
    }
    let Some(object) = move_.object.as_deref() else {
        return true;
    };
    !oracle.is_excluded(object)
}

/// What a `LineIter` pass over the remainder found.
struct Census {
    /// Command names in file order, capped at [`MAX_NAMED_COMMANDS`].
    names: Vec<String>,
    /// Command lines seen, *uncapped* — the intelligibility test is about
    /// the whole remainder, not the part that fits in a message.
    command_lines: u32,
    /// Whether any command was *traditional* (letter + number). See
    /// [`remaining_work`]'s "The remainder has to be g-code".
    saw_traditional: bool,
}

/// Walks the remainder at or after `offset`, naming its commands and
/// recording whether any of them is traditional g-code.
///
/// Comment-only and blank lines yield no command
/// ([`plr_gcode::Line::command`] returns `None`), and so does a
/// degenerate line such as a bare line number, whose command name
/// Klipper leaves empty. The whole remainder is walked even once the name
/// list is full, because `saw_traditional` must consider all of it.
fn census(tail: &[u8], base_offset: u64, offset: u64) -> Census {
    let mut census = Census {
        names: Vec::new(),
        command_lines: 0,
        saw_traditional: false,
    };
    for line in LineIter::new(tail, base_offset) {
        if line.span.start < offset {
            continue;
        }
        let Some(command) = line.command() else {
            continue;
        };
        if command.name.is_empty() {
            continue;
        }
        census.command_lines = census.command_lines.saturating_add(1);
        census.saw_traditional |=
            matches!(command.params, plr_gcode::CommandParams::Traditional { .. });
        if census.names.len() < MAX_NAMED_COMMANDS {
            census.names.push(command.name.clone());
        }
    }
    census
}

#[cfg(test)]
mod tests {
    use super::{
        remaining_work, AnchorFrame, ExclusionOracle, ModeAxis, RemainingWork, WorkUnknown,
        MAX_NAMED_COMMANDS,
    };
    use crate::model::{build_layer_model, ModelConfig, ModelStop};
    use plr_gcode::GcodeState;

    /// A stand-in for `plr_reconstruct::ExclusionReport`.
    struct Oracle {
        conclusive: bool,
        excluded: Vec<&'static str>,
    }

    impl ExclusionOracle for Oracle {
        fn is_conclusive(&self) -> bool {
            self.conclusive
        }
        fn is_excluded(&self, object: &str) -> bool {
            self.excluded.contains(&object)
        }
    }

    /// The [`AnchorFrame`] matching a [`GcodeState`].
    fn frame_of(state: &GcodeState) -> AnchorFrame {
        AnchorFrame {
            absolute_coordinates: state.absolute_coord,
            absolute_extrude: state.absolute_extrude,
        }
    }

    /// A frame with both flags set as given (coordinates first).
    fn frame(absolute_coordinates: bool, absolute_extrude: bool) -> AnchorFrame {
        AnchorFrame {
            absolute_coordinates,
            absolute_extrude,
        }
    }

    fn work(
        text: &str,
        offset: u64,
        oracle: Option<&dyn ExclusionOracle>,
    ) -> Result<RemainingWork, WorkUnknown> {
        let state = GcodeState::new();
        let anchor = frame_of(&state);
        let bytes = text.as_bytes();
        let model = build_layer_model(state, bytes, 0, &ModelConfig::default());
        remaining_work(&model, bytes, 0, offset, anchor, oracle)
    }

    /// The offset of the line starting with `needle`.
    fn at(text: &str, needle: &str) -> u64 {
        text.find(needle).expect("needle present") as u64
    }

    const PRINT: &str = "G90\nM83\nG1 Z0.2 F7200\n\
                         G1 X10 Y10 F9000\nG1 X20 Y10 E1 F1800\n\
                         G1 Z0.4 F7200\nG1 X10 Y10 E1\n\
                         M107\nM104 S0\nM140 S0\nG1 E-2 F2100\n\
                         G28 X Y\nM84\n\
                         ; prusaslicer_config = begin\n; layer_height = 0.2\n\
                         ; prusaslicer_config = end\n";

    #[test]
    fn extrusion_ahead_of_the_offset_is_work() {
        let got = work(PRINT, 0, None).unwrap();
        let RemainingWork::Extrusion {
            first_offset,
            moves,
            layers,
        } = got
        else {
            panic!("expected Extrusion, got {got:?}");
        };
        assert_eq!(first_offset, at(PRINT, "G1 X20 Y10 E1"));
        assert_eq!(moves, 2);
        assert_eq!(layers, 2);
        assert!(!got.is_complete());
    }

    #[test]
    fn a_footer_only_remainder_names_the_end_sequence() {
        // Stop right after the last depositing line: only the end g-code
        // and the config block remain.
        let offset = at(PRINT, "M107");
        let got = work(PRINT, offset, None).unwrap();
        assert_eq!(
            got,
            RemainingWork::EndSequenceOnly {
                commands: vec![
                    "M107".to_owned(),
                    "M104".to_owned(),
                    "M140".to_owned(),
                    "G1".to_owned(),
                    "G28".to_owned(),
                    "M84".to_owned(),
                ],
            }
        );
        assert!(got.is_complete());
        assert!(!got.commands_truncated());
    }

    #[test]
    fn a_config_block_only_remainder_is_nothing() {
        let offset = at(PRINT, "; prusaslicer_config = begin");
        let got = work(PRINT, offset, None).unwrap();
        assert_eq!(got, RemainingWork::Nothing);
        assert!(got.is_complete());
        assert!(!got.commands_truncated());
        // And at exactly EOF, likewise — an empty remainder is proof of
        // completion, not an unknown.
        let eof = PRINT.len() as u64;
        assert_eq!(work(PRINT, eof, None).unwrap(), RemainingWork::Nothing);
    }

    #[test]
    fn travel_retract_and_chatter_are_not_work() {
        // Everything a `PRINT_END` plausibly contains, and nothing that
        // deposits: the answer must never be `Extrusion`.
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X1 Y1 E1 F1800\n\
                    \n; a comment\nM400\nM117 done\nM118 done\n\
                    RESPOND PREFIX=x MSG=\"done\"\n\
                    SET_FAN_SPEED FAN=x SPEED=0\nSAVE_GCODE_STATE NAME=x\n\
                    RESTORE_GCODE_STATE NAME=x\nEXCLUDE_OBJECT_END\n\
                    M900 K0\nM204 S500\nM205 X8 Y8\nG4 P100\nPRINT_END\n\
                    G1 X50 Y50 F9000\nG1 E-4 F2100\nG1 Z10 F600\n\
                    G28\nM18\nM107\nM104 S0\nM140 S0\n";
        let offset = at(text, "\n; a comment") + 1;
        let got = work(text, offset, None).unwrap();
        assert!(got.is_complete(), "{got:?}");
        let RemainingWork::EndSequenceOnly { commands } = &got else {
            panic!("expected EndSequenceOnly, got {got:?}");
        };
        assert!(commands.contains(&"PRINT_END".to_owned()), "{commands:?}");
        assert!(commands.contains(&"G28".to_owned()), "{commands:?}");
        // Blank and comment lines produce no command name.
        assert!(!commands.iter().any(String::is_empty), "{commands:?}");
    }

    #[test]
    fn a_positive_e_move_after_the_end_sequence_is_still_work() {
        // A wipe-tower / purge line hiding behind the cooldown: work.
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X1 Y1 E1 F1800\n\
                    M107\nM104 S0\nG1 X9 Y9 E0.5 F1800\nM84\n";
        let offset = at(text, "M107");
        let got = work(text, offset, None).unwrap();
        let RemainingWork::Extrusion { first_offset, .. } = got else {
            panic!("expected Extrusion, got {got:?}");
        };
        assert_eq!(first_offset, at(text, "G1 X9 Y9 E0.5"));
    }

    #[test]
    fn arc_moves_count_through_their_chords() {
        // G2/G3 chords carry their share of E; no arc-specific logic
        // exists here, which is exactly the point.
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X10 Y0 F9000\n\
                    G3 X0 Y10 I-10 E3 F1800\nM84\n";
        let got = work(text, at(text, "G3"), None).unwrap();
        let RemainingWork::Extrusion {
            first_offset,
            moves,
            ..
        } = got
        else {
            panic!("expected Extrusion, got {got:?}");
        };
        assert_eq!(first_offset, at(text, "G3"));
        assert!(moves > 1, "the arc expanded into chords: {moves}");
    }

    const PLATE: &str = "G90\nM83\nG1 Z0.2 F7200\n\
                         EXCLUDE_OBJECT_START NAME=part_a\n\
                         G1 X10 Y10 E1 F1800\n\
                         EXCLUDE_OBJECT_END\n\
                         EXCLUDE_OBJECT_START NAME=PART_B\n\
                         G1 X20 Y20 E1 F1800\n\
                         EXCLUDE_OBJECT_END\nM84\n";

    #[test]
    fn every_remaining_object_cancelled_is_no_work_when_conclusive() {
        let oracle = Oracle {
            conclusive: true,
            excluded: vec!["PART_A", "PART_B"],
        };
        let got = work(PLATE, 0, Some(&oracle)).unwrap();
        assert!(got.is_complete(), "{got:?}");
        // The lines are still in the file, so the census still sees them.
        let RemainingWork::EndSequenceOnly { commands } = &got else {
            panic!("expected EndSequenceOnly, got {got:?}");
        };
        assert!(commands.contains(&"G1".to_owned()));
    }

    #[test]
    fn one_surviving_object_is_still_work() {
        let oracle = Oracle {
            conclusive: true,
            excluded: vec!["PART_A"],
        };
        let got = work(PLATE, 0, Some(&oracle)).unwrap();
        let RemainingWork::Extrusion {
            first_offset,
            moves,
            ..
        } = got
        else {
            panic!("expected Extrusion, got {got:?}");
        };
        assert_eq!(first_offset, at(PLATE, "G1 X20 Y20 E1"));
        assert_eq!(moves, 1);
    }

    #[test]
    fn an_inconclusive_oracle_counts_excluded_work_as_work() {
        let oracle = Oracle {
            conclusive: false,
            excluded: vec!["PART_A", "PART_B"],
        };
        let got = work(PLATE, 0, Some(&oracle)).unwrap();
        let RemainingWork::Extrusion { moves, .. } = got else {
            panic!("expected Extrusion, got {got:?}");
        };
        assert_eq!(moves, 2, "both objects must count while in doubt");
    }

    #[test]
    fn unattributed_deposition_always_counts() {
        // A skirt outside any bracket cannot be cancelled, so it counts
        // even when every named object is excluded.
        let text = "G90\nM83\nG1 Z0.2 F7200\nG1 X5 Y5 E1 F1800\n\
                    EXCLUDE_OBJECT_START NAME=part_a\n\
                    G1 X10 Y10 E1 F1800\nEXCLUDE_OBJECT_END\nM84\n";
        let oracle = Oracle {
            conclusive: true,
            excluded: vec!["PART_A"],
        };
        let got = work(text, 0, Some(&oracle)).unwrap();
        let RemainingWork::Extrusion {
            first_offset,
            moves,
            ..
        } = got
        else {
            panic!("expected Extrusion, got {got:?}");
        };
        assert_eq!(first_offset, at(text, "G1 X5 Y5 E1"));
        assert_eq!(moves, 1);
    }

    #[test]
    fn a_nameless_start_leaves_the_deposition_unattributed() {
        let text = "G90\nM83\nG1 Z0.2 F7200\n\
                    EXCLUDE_OBJECT_START NAME=part_a\n\
                    EXCLUDE_OBJECT_START\n\
                    G1 X10 Y10 E1 F1800\nM84\n";
        let oracle = Oracle {
            conclusive: true,
            excluded: vec!["PART_A"],
        };
        assert!(!work(text, 0, Some(&oracle)).unwrap().is_complete());
    }

    #[test]
    fn an_unparseable_remainder_is_unknown_not_complete() {
        // A line the state machine refuses stops the replay; the model
        // then covers only the prefix, so completion is unprovable.
        let text = "G90\nM83\nG1 X1 Y1 E1 F1800\nG2 X5 Y5\nG1 X9 Y9 E1\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
        let ModelStop::LineError { offset, .. } = model.stop else {
            panic!("this fixture must fail replay, got {:?}", model.stop);
        };
        assert_eq!(
            remaining_work(&model, bytes, 0, offset, frame(true, true), None),
            Err(WorkUnknown::ReplayFailed { offset })
        );
        // And the error renders for an operator log.
        let rendered = WorkUnknown::ReplayFailed { offset }.to_string();
        assert!(rendered.contains("replay failed"), "{rendered}");
    }

    #[test]
    fn an_offset_past_the_window_is_unknown_not_complete() {
        let text = "G90\nM83\nG1 X1 Y1 E1 F1800\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
        let past = bytes.len() as u64 + 1;
        let err = remaining_work(&model, bytes, 0, past, frame(true, true), None).unwrap_err();
        assert_eq!(
            err,
            WorkUnknown::OffsetOutsideWindow {
                offset: past,
                window_end: bytes.len() as u64,
            }
        );
        assert!(err.to_string().contains("before the tested offset"));
    }

    #[test]
    fn the_named_command_list_is_capped() {
        let mut text = String::from("G90\nM83\nG1 X1 Y1 E1 F1800\n");
        let offset = text.len() as u64;
        for _ in 0..(MAX_NAMED_COMMANDS + 10) {
            text.push_str("M400\n");
        }
        let got = work(&text, offset, None).unwrap();
        let RemainingWork::EndSequenceOnly { commands } = &got else {
            panic!("expected EndSequenceOnly, got {got:?}");
        };
        assert_eq!(commands.len(), MAX_NAMED_COMMANDS);
        assert!(got.commands_truncated());
    }

    #[test]
    fn a_base_offset_shifts_the_whole_census() {
        let text = "M107\nM104 S0\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 1_000, &ModelConfig::default());
        // Test at the second line, mid-window.
        let got = remaining_work(&model, bytes, 1_000, 1_005, frame(true, true), None).unwrap();
        assert_eq!(
            got,
            RemainingWork::EndSequenceOnly {
                commands: vec!["M104".to_owned(), "M84".to_owned()],
            }
        );
        // An offset before the window start still only sees the window.
        let got = remaining_work(&model, bytes, 1_000, 0, frame(true, true), None).unwrap();
        assert_eq!(
            got,
            RemainingWork::EndSequenceOnly {
                commands: vec!["M107".to_owned(), "M104".to_owned(), "M84".to_owned()],
            }
        );
    }

    // -----------------------------------------------------------------
    // The extruder-frame trust check
    // -----------------------------------------------------------------

    /// A window whose `M82`/`M83` contradicts the anchor's claim refuses to
    /// answer — **even though the extrusion arithmetic says complete**.
    ///
    /// The window is a relative-E footer: `G1 E-0.8` and the wipe moves all
    /// retract under either reading, so `Extrusion` is not found either way.
    /// The refusal is about trust, not arithmetic.
    #[test]
    fn a_contradicted_extrude_mode_refuses_even_when_the_arithmetic_says_complete() {
        // Tested offset 0; the M83 sits after it and says relative, while
        // the anchor claims absolute.
        let text = "M107\nG1 E-0.8 F2100\nM83\nG1 X9 Y9 E-0.04 F1800\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
        let anchor_absolute = true;
        let err =
            remaining_work(&model, bytes, 0, 0, frame(true, anchor_absolute), None).unwrap_err();
        assert_eq!(
            err,
            WorkUnknown::ExtrudeModeContradiction {
                offset: at(text, "M83"),
                axis: ModeAxis::Extrusion,
                file_absolute: false,
            }
        );
        let rendered = err.to_string();
        assert!(
            rendered.contains("relative extrusion at byte"),
            "{rendered}"
        );
        assert!(rendered.contains("claim of absolute"), "{rendered}");

        // And the mirror: the file says absolute, the anchor said relative.
        let text = "M107\nG1 E-0.8 F2100\nM82\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(
            plr_gcode::GcodeState::new(),
            bytes,
            0,
            &ModelConfig::default(),
        );
        let err = remaining_work(&model, bytes, 0, 0, frame(true, false), None).unwrap_err();
        assert_eq!(
            err,
            WorkUnknown::ExtrudeModeContradiction {
                offset: at(text, "M82"),
                axis: ModeAxis::Extrusion,
                file_absolute: true,
            }
        );
    }

    /// **`G90`/`G91` count too.** The effective extruder frame is
    /// `absolute_coord && absolute_extrude` (`plr-gcode`'s `state.rs`, after
    /// Klipper's `gcode_move.py`), so a wrong `absolute_coordinates` is just
    /// as capable of mis-classifying a move as a wrong `absolute_extrude` —
    /// and a guard watching only `M82`/`M83` would miss it entirely.
    #[test]
    fn a_contradicted_coordinate_mode_also_refuses() {
        // Under the anchor's (absolute, absolute) claim the trailing move
        // reads as +0.7 of deposition; under the file's `G91` — which makes
        // the conjunction false, hence relative E — it is a retract. So the
        // claim alone decides the answer, which is exactly when trust
        // matters.
        let text = "G1 E-0.8 F2100\nM107\nG91\nG1 X1 Y1 E-0.1 F1800\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
        let err = remaining_work(&model, bytes, 0, 0, frame(true, true), None).unwrap_err();
        assert_eq!(
            err,
            WorkUnknown::ExtrudeModeContradiction {
                offset: at(text, "G91"),
                axis: ModeAxis::Coordinates,
                file_absolute: false,
            }
        );
        assert!(
            err.to_string().contains("relative coordinates at byte"),
            "{err}"
        );
        // Agreeing on coordinates, and the extrusion axis untouched by the
        // window: trusted.
        let got = remaining_work(&model, bytes, 0, 0, frame(false, true), None).expect("trusted");
        assert!(got.is_complete(), "{got:?}");
    }

    /// The two axes are tracked independently, and the first contradiction
    /// in file order is the one reported.
    #[test]
    fn each_mode_axis_is_tracked_independently() {
        // `G90` agrees with the anchor, `M83` does not.
        let text = "G1 E-0.8 F2100\nG90\nM83\nG1 X1 Y1 E-0.1 F1800\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
        let err = remaining_work(&model, bytes, 0, 0, frame(true, true), None).unwrap_err();
        assert_eq!(
            err,
            WorkUnknown::ExtrudeModeContradiction {
                offset: at(text, "M83"),
                axis: ModeAxis::Extrusion,
                file_absolute: false,
            }
        );
        // With the extrusion claim corrected, both axes agree: trusted.
        assert!(
            remaining_work(&model, bytes, 0, 0, frame(true, false), None)
                .expect("trusted")
                .is_complete()
        );
    }

    /// A coordinate-mode command settled **before** the tested offset takes
    /// that axis out of doubt, exactly as for the extrusion axis.
    #[test]
    fn a_coordinate_mode_command_before_the_offset_settles_that_axis() {
        let text = "G91\nG1 X1 Y1 E0.5 F1800\nM107\nG1 E-0.8 F2100\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
        let offset = at(text, "M107");
        for coordinates in [true, false] {
            assert!(
                remaining_work(&model, bytes, 0, offset, frame(coordinates, true), None)
                    .expect("the file settled the coordinate axis")
                    .is_complete()
            );
        }
    }

    /// A window whose mode command **agrees** with the anchor suppresses
    /// exactly as it would without the check.
    #[test]
    fn an_agreeing_extrude_mode_still_suppresses() {
        let text = "M107\nG1 E-0.8 F2100\nM83\nG1 X9 Y9 E-0.04 F1800\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(
            plr_gcode::GcodeState {
                absolute_extrude: false,
                ..plr_gcode::GcodeState::new()
            },
            bytes,
            0,
            &ModelConfig::default(),
        );
        let got = remaining_work(&model, bytes, 0, 0, frame(true, false), None).expect("trusted");
        assert!(got.is_complete(), "{got:?}");
    }

    /// A mode command at or **before** the tested offset means the file
    /// established the mode over the whole tested region, so the anchor's
    /// claim is irrelevant and cannot be contradicted — no refusal, however
    /// wrong the claim was.
    #[test]
    fn a_mode_command_before_the_tested_offset_settles_it() {
        let text = "M83\nG1 X1 Y1 E0.5 F1800\nM107\nG1 E-0.8 F2100\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
        let offset = at(text, "M107");
        // A blatantly wrong anchor claim, and still no refusal.
        for anchor_absolute in [true, false] {
            let got = remaining_work(&model, bytes, 0, offset, frame(true, anchor_absolute), None)
                .expect("the file settled the mode");
            assert!(got.is_complete(), "{got:?}");
        }
        // A command sitting exactly AT the tested offset counts as
        // "before": nothing in the tested region precedes it.
        let text = "G1 X1 Y1 E0.5 F1800\nM83\nM107\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
        let offset = at(text, "M83");
        assert!(
            remaining_work(&model, bytes, 0, offset, frame(true, true), None)
                .expect("at-offset command settles it")
                .is_complete()
        );
    }

    /// Only the FIRST post-offset command matters: after it the mode is
    /// file-established, so a later `M82`/`M83` is an ordinary change from a
    /// known state rather than a contradiction.
    #[test]
    fn only_the_first_mode_command_after_the_offset_is_checked() {
        let text = "M107\nM83\nG1 E-0.8 F1800\nM82\nM84\n";
        let bytes = text.as_bytes();
        let model = build_layer_model(
            plr_gcode::GcodeState {
                absolute_extrude: false,
                ..plr_gcode::GcodeState::new()
            },
            bytes,
            0,
            &ModelConfig::default(),
        );
        // First command (M83) agrees with the anchor; the later M82 does
        // not, and must not matter.
        let got = remaining_work(&model, bytes, 0, 0, frame(true, false), None).expect("trusted");
        assert!(got.is_complete(), "{got:?}");
    }

    /// The common path is unchanged: a window with no `M82`/`M83` at all
    /// runs on the anchor's claim and never refuses.
    #[test]
    fn a_window_with_no_mode_command_is_unaffected() {
        let text = "M107\nM104 S0\nM140 S0\nG1 E-0.8 F2100\nM84\n";
        let bytes = text.as_bytes();
        assert!(!text.contains("M82") && !text.contains("M83"));
        for anchor_absolute in [true, false] {
            let model = build_layer_model(
                plr_gcode::GcodeState {
                    absolute_extrude: anchor_absolute,
                    ..plr_gcode::GcodeState::new()
                },
                bytes,
                0,
                &ModelConfig::default(),
            );
            let got = remaining_work(&model, bytes, 0, 0, frame(true, anchor_absolute), None)
                .expect("no mode command, no doubt");
            assert!(got.is_complete(), "{got:?}");
        }
    }

    /// **A remainder that is not g-code proves nothing.** `Nothing` and
    /// `EndSequenceOnly` both report completion, so without this a tail
    /// replaced by arbitrary bytes tokenizes as one extended command, holds
    /// no extrusion, and reads as "complete".
    #[test]
    fn a_remainder_that_is_not_gcode_is_unknown_not_complete() {
        for len in [1_usize, 3, 40, 400] {
            let garbage = "x".repeat(len);
            let text = format!("G1 X1 Y1 E1 F1800\n{garbage}");
            let bytes = text.as_bytes();
            let offset = at(&text, &garbage);
            let model = build_layer_model(GcodeState::new(), bytes, 0, &ModelConfig::default());
            let got = remaining_work(&model, bytes, 0, offset, frame(true, true), None);
            assert_eq!(
                got,
                Err(WorkUnknown::UnintelligibleRemainder {
                    offset,
                    commands: 1,
                }),
                "{len} bytes of garbage must not read as an end sequence"
            );
            assert!(got.unwrap_err().to_string().contains("no traditional"));
        }
    }

    /// The discriminator is Klipper's own traditional/extended split, so a
    /// real end sequence passes and a comment-only tail still says `Nothing`.
    #[test]
    fn traditional_commands_make_a_remainder_intelligible() {
        // One traditional command among extended ones is enough.
        let text = "G1 X1 Y1 E1 F1800\nPRINT_END\nM84\n";
        let got = work(text, at(text, "PRINT_END"), None).expect("intelligible");
        assert_eq!(
            got,
            RemainingWork::EndSequenceOnly {
                commands: vec!["PRINT_END".to_owned(), "M84".to_owned()],
            }
        );
        // Extended commands alone are refused — no slicer emits such a
        // footer, and announcing is the recoverable direction.
        let text = "G1 X1 Y1 E1 F1800\nPRINT_END\n";
        let err = work(text, at(text, "PRINT_END"), None).unwrap_err();
        assert!(matches!(
            err,
            WorkUnknown::UnintelligibleRemainder { commands: 1, .. }
        ));
        // Comments and blanks are intelligible: a config-block tail.
        let text = "G1 X1 Y1 E1 F1800\n; k = v\n\n; k2 = v2\n";
        assert_eq!(
            work(text, at(text, "; k = v"), None).expect("comments"),
            RemainingWork::Nothing
        );
    }

    #[test]
    fn an_empty_tail_is_nothing() {
        let model = build_layer_model(GcodeState::new(), &[], 0, &ModelConfig::default());
        assert_eq!(
            remaining_work(&model, &[], 0, 0, frame(true, true), None).unwrap(),
            RemainingWork::Nothing
        );
    }
}
