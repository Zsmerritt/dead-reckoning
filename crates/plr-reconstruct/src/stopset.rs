//! The possible-stop set: every state the machine can plausibly have
//! stopped in, as a union of WAL-evaluated states and a
//! forward-simulated extension.
//!
//! # Construction
//!
//! 1. **WAL span.** Trapq segments (toolhead and extruder queues) are
//!    evaluated over `[t_a, wal_eval_end]` with the exact
//!    `position_at` math Klipper's `motion_report` uses.
//!    `wal_eval_end = max(t_b, end of durable trapq data)` — trapq rows
//!    are journaled as moves are *planned* (up to ~1 s ahead of
//!    execution), so durable rows can describe motion beyond the
//!    committed boundary `t_b`. Evaluating out to them is deliberate:
//!    lines processed before the last WAL flush but executing after
//!    `t_b` exist **only** there (the forward extension starts at the
//!    last context's processing frontier and cannot see them), and
//!    including planned-but-never-executed states merely widens the set,
//!    which is the safe direction. Dwell gaps are handled: for any time
//!    the trapq does not cover, the machine holds the end position of
//!    the last preceding segment, which contributes a candidate.
//! 2. **Extension.** From the last context's file offset and g-code
//!    state, [`plr_gcode::simulate`] runs for
//!    `extension_horizon + max(0, wal_eval_end - t_ext_start)` seconds
//!    of simulated motion, where `t_ext_start` is the print time at
//!    which the machine **begins executing the first simulated line**
//!    ([`extension_start_time`]) — *not* the anchor snapshot's capture
//!    time. The two differ by seconds whenever the g-code reader stalls
//!    on one long move: the frontier then sits still while the queued
//!    move executes, so the snapshot records a file position whose
//!    motion started long before the snapshot itself. Measuring from the
//!    capture time spends the horizon re-simulating already-executed
//!    motion and under-covers the window (the bug behind proptest seed
//!    `9938d965…`). The simulator's per-line accounting is a documented
//!    lower bound on real durations **with one documented exception**:
//!    the window starts from zero velocity, so the *first* move's
//!    duration can be overestimated by up to one acceleration ramp
//!    (`plr-gcode/src/sim.rs` module docs). Simulating
//!    `T - t_ext_start` seconds therefore consumes every line the
//!    machine could have executed by real time `T` less at most that one
//!    ramp — ~0.1 s at Klipper-typical limits, which the default 2 s
//!    `extension_horizon` covers with margin, and which matters most
//!    when the horizon is short.
//! 3. **Z is exact.** The extension's Z candidates come from
//!    [`plr_gcode::scan_z_events`] over exactly the consumed lines — a
//!    byte-faithful replay with no timing model. Combined with the WAL
//!    span's per-segment Z enumeration, the guarantee holds: the
//!    Z-projection of the possible-stop set is exactly enumerable —
//!    `{z_layer, z_layer − hop}` plateaus plus short ramp intervals at
//!    worst. [`PossibleStopSet::z_span`] (max − min over candidates) is
//!    what sizes the probe envelope downstream.
//! 4. **XY/E.** XY timing fidelity only affects line-match granularity,
//!    so XY is reported as a bounding region; E as intervals in both
//!    the Klipper-internal frame and the file frame.
//!
//! # E frames
//!
//! Trapq E is Klipper-internal accumulated E: `G92 E` shifts only
//! `gcode_move`'s `base_position`, and M221 is baked in before the
//! trapq. Matching WAL E to file E therefore requires the
//! `(base_e, extrude_factor)` pair that was active when the motion was
//! *processed*. The primary path recovers those pairs **exactly** by
//! replaying the file from the offset-window floor context through the
//! extension end and recording the g-code-frame E after every line
//! ([`file_frame_e`]'s replay). Because the interpreter is
//! deterministic, this reconstructs the frame of every candidate line —
//! including frames created *and* replaced between two context flushes
//! (e.g. `G92 E0` + retract processed in one burst right after a
//! dwell), which no context snapshot ever captured; relying on
//! snapshotted frames alone was a proven containment hole (see the
//! checked-in proptest regression seed). Only when the replay cannot
//! run or cannot reach the window end does the computation fall back to
//! unioning the WAL-internal interval converted under recent context
//! frames, flagged [`Degradation::e_file_frames_incomplete`].
//!
//! # Durable extruder coverage, and the `e_internal` band
//!
//! `e_internal` comes from the trapq evaluation plus the extension. The
//! durable WAL can be missing extruder rows for lines the anchor context
//! already counts as processed, for two independent reasons (both proved
//! from Klipper's source in [`Context::print_time`]'s docs): the move
//! waits in a `LookAheadQueue` invisible to `dump_trapq`, and then waits
//! up to one ~0.5 s dump batch. Measured against real `OrcaSlicer` output
//! at real print settings, the frontier runs **17–119 lines** ahead over
//! 0.5 s and **22–147 lines** over 0.65 s (median–max).
//!
//! ## What is actually missing is an *interior extremum*, not an endpoint
//!
//! This is the trap, and it is worth being explicit because the obvious
//! reading of the hazard suggests a fix that does not work. Both
//! **endpoints** of the un-evidenced band are already known exactly:
//!
//! * the low end from the newest durable extruder row, and
//! * the high end from `anchor.gcode.position[3]`, which is Klipper's
//!   `gcode_move.last_position` — internal accumulated E at the
//!   *processing* frontier, updated inside `cmd_G1` ahead of the
//!   lookahead and the trapq — and which already seeds the extension via
//!   [`anchor_state_from_context`].
//!
//! So "bound E by the value at the frontier" looks sufficient and is
//! **not**. The hazard is a *non-monotone excursion* strictly inside the
//! band: a retract to `E−5` and back leaves both endpoints at `E₀`, and
//! neither the trapq evaluation nor the extension (which starts at `F`,
//! after the excursion) ever sees `E−5`. Only a replay of the band's
//! lines recovers it.
//!
//! ## The fix: union the replay's internal E over the certified band
//!
//! [`replay_file_e`] already computes exact cumulative E after **every**
//! line from the floor forward, so the excursion is already in reach; the
//! only missing ingredient was a *safe* band start. `Context::print_time`
//! supplies it. See [`coverage_certified_context`] for the certificate
//! and the one premise it rests on, and [`Degradation::e_internal_band`]
//! for what each outcome costs.
//!
//! Unioning over the **whole** loose-floored window instead was tried and
//! reverted: it broke 18 daemon end-to-end tests with "below layer
//! granularity", i.e. manual fallback or a wrong line on every real
//! recovery. The certified band is bounded by the *coverage lag*
//! (sub-second, tens of lines) rather than by `max_processing_lead`
//! (3 s, hundreds of lines), which is why it does not reproduce that.
//!
//! `e_file` was never affected: on the exact path (`reached_end` and a
//! certain floor) the replay already covers those lines. But
//! `plr-analyzer`'s `StopEvidence` (fed by `plrd`) carries only
//! `e_internal` and discards `e_file`, so the hole was reachable.
//!
//! # File-offset window
//!
//! Context snapshots record the g-code **processing frontier**, which
//! leads execution by up to `max_processing_lead` (Klipper's lookahead
//! buffering plus step-generation lead). The candidate window is
//! therefore `[frontier recorded at or before t_a - max_processing_lead,
//! extension resume offset]`: the low end is the newest frontier old
//! enough that execution had certainly passed it by `t_a`; the high end
//! is where the simulated extension stopped consuming.

use plr_gcode::{scan_z_events, simulate, GcodeState, Line, LineIter, StopReason, ZScanConfig};
use plr_wal::{Context, TrapqSegment};

use crate::config::ReconstructConfig;
use crate::error::{ContextDefect, ReconstructError};
use crate::timeline::WalTimeline;
use crate::window::{is_observation_gap, ns_to_s, StopWindow};

/// A closed interval `[lo, hi]` (mm or mm-of-filament).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Interval {
    /// Lower bound.
    pub lo: f64,
    /// Upper bound (`>= lo` for every interval this crate constructs
    /// from finite inputs).
    pub hi: f64,
}

impl Interval {
    /// The degenerate interval `[v, v]`.
    #[must_use]
    pub const fn point(v: f64) -> Self {
        Self { lo: v, hi: v }
    }

    /// The interval spanning two values in either order.
    #[must_use]
    pub fn from_pair(a: f64, b: f64) -> Self {
        Self {
            lo: a.min(b),
            hi: a.max(b),
        }
    }

    /// Grows the interval to include `v` (NaN is ignored: `min`/`max`
    /// return the non-NaN operand).
    pub fn expand(&mut self, v: f64) {
        self.lo = self.lo.min(v);
        self.hi = self.hi.max(v);
    }

    /// The smallest interval containing both.
    #[must_use]
    pub fn union(self, other: Self) -> Self {
        Self {
            lo: self.lo.min(other.lo),
            hi: self.hi.max(other.hi),
        }
    }

    /// `true` when `v` lies within the interval widened by `tol`.
    #[must_use]
    pub fn contains(&self, v: f64, tol: f64) -> bool {
        v >= self.lo - tol && v <= self.hi + tol
    }

    /// `hi - lo`.
    #[must_use]
    pub fn width(&self) -> f64 {
        self.hi - self.lo
    }
}

/// Axis-aligned XY bounding region of the possible stop positions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct XyRegion {
    /// X extent, mm.
    pub x: Interval,
    /// Y extent, mm.
    pub y: Interval,
}

impl XyRegion {
    fn expand(&mut self, x: f64, y: f64) {
        self.x.expand(x);
        self.y.expand(y);
    }

    /// `true` when the point lies inside the region widened by `tol`.
    #[must_use]
    pub fn contains(&self, x: f64, y: f64, tol: f64) -> bool {
        self.x.contains(x, tol) && self.y.contains(y, tol)
    }
}

/// Which evidence produced a Z candidate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Provenance {
    /// Evaluated from durable trapq data (measured planning, exact).
    Wal,
    /// Enumerated by the exact Z scan over the forward extension.
    Extension,
}

/// Whether a Z candidate is a held level or a transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZKind {
    /// A Z level the machine dwelt at (layer height, hop height): a
    /// point candidate.
    Plateau,
    /// A Z transition (hop ramp, layer change, spiral chord): the stop
    /// can be anywhere inside the interval.
    Ramp,
}

/// One member of the exact Z enumeration.
#[derive(Debug, Clone, PartialEq)]
pub struct ZCandidate {
    /// The Z value (plateau: `lo == hi`) or ramp extent, mm, in
    /// Klipper-internal coordinates.
    pub z: Interval,
    /// Which evidence produced it.
    pub provenance: Provenance,
    /// `false` when the underlying Z knowledge was lost (G28 in the
    /// extension without re-establishment): the value must not be
    /// trusted, and [`PossibleStopSet::z_span`] excludes it.
    pub z_known: bool,
    /// Plateau or ramp.
    pub kind: ZKind,
}

/// Byte-offset candidate window in the printed file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OffsetWindow {
    /// Inclusive low end: a processing frontier old enough that
    /// execution had certainly passed it by `t_a`.
    pub start: u64,
    /// Inclusive high end: the extension's resume offset (a line
    /// boundary, `M26`-safe).
    pub end: u64,
}

impl OffsetWindow {
    /// `true` when `offset` lies within the window.
    #[must_use]
    pub const fn contains(&self, offset: u64) -> bool {
        offset >= self.start && offset <= self.end
    }
}

/// What the forward extension did.
#[derive(Debug, Clone, PartialEq)]
pub struct ExtensionSummary {
    /// File offset the simulation started at (the anchor context's
    /// `virtual_sdcard` position).
    pub anchor_offset: u64,
    /// The anchor context's capture time mapped to print time, when the
    /// correlation could place it.
    pub anchor_print_time: Option<f64>,
    /// The effective simulated-motion horizon, seconds
    /// (`extension_horizon + catch-up`).
    pub horizon: f64,
    /// Lines actually consumed.
    pub lines_consumed: usize,
    /// Line-boundary offset the simulation stopped at.
    pub resume_offset: Option<u64>,
    /// Why the simulation stopped consuming input.
    pub stop: StopReason,
}

/// How much confidence downstream matching can place in the result.
///
/// # This value currently decides nothing
///
/// It is computed in [`compute_stop_set`] and it reads like a control
/// signal, but **no consumer anywhere in the workspace acts on it**. Its
/// only use outside this crate is a forensic `eprintln!` in `plrd::scan`.
/// `plr-analyzer`'s matcher derives its own `MatchConfidence` from the
/// candidate count and never receives this field, and nothing in the
/// `plrd` pipeline consults it before offering a plan.
///
/// That matters because several flags on [`Degradation`] are documented as
/// forcing [`Confidence::PerLayer`] "so automation refuses rather than
/// resuming" — see [`Degradation::extension_truncated`]. **That claim is
/// not true today.** Setting this field is a confession, not a guarantee.
/// Any new flag that needs to be consequential must reach the code that
/// decides whether to offer a recovery; wiring it here is a no-op, which
/// was verified by experiment (forcing `PerLayer` from a new flag changed
/// the behaviour of zero tests).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Confidence {
    /// Evidence covers the window without known holes: candidates are
    /// enumerated at per-line granularity.
    #[default]
    PerLine,
    /// A known observation hole (subscription gap, unavailable or
    /// truncated extension, erroring line) means matching should trust
    /// only per-layer granularity.
    PerLayer,
}

/// Honest degradation report. Every flag is independent evidence-quality
/// information; none of them invalidates the containment guarantee
/// except as documented on the flag.
// Justified: these are orthogonal boolean facts, not a state machine;
// collapsing them into enums would obscure which combinations occurred.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Degradation {
    /// Overall line-match confidence (see [`Confidence`]).
    pub confidence: Confidence,
    /// A subscription gap or socket loss overlaps the window: WAL
    /// candidates inside it may be missing. Containment then relies on
    /// the extension alone.
    pub observation_gap: bool,
    /// No file tail (or no `virtual_sdcard` state) was available: the
    /// extension did not run. **The containment guarantee is void for
    /// true power loss** in this state — only WAL evidence is reported.
    pub extension_unavailable: bool,
    /// The extension's coverage of the far end of the window is not
    /// guaranteed. Two causes:
    ///
    /// * it hit its line budget before its time horizon; or
    /// * the print-time axis was unusable (the `wal_eval_end - start_pt`
    ///   span overflowed to non-finite), so the horizon could not be
    ///   sized in time at all. The extension then runs unbounded for
    ///   maximal coverage and sets this flag, because a horizon that
    ///   cannot be computed cannot be claimed to bound anything — see
    ///   [`run_extension`].
    ///
    /// Either way this sets [`Confidence::PerLayer`] — which, despite how
    /// this comment used to read, does **not** make automation refuse:
    /// nothing consumes that field. See [`Confidence`].
    pub extension_truncated: bool,
    /// The extension stopped at an unparseable/unsupported line;
    /// candidates beyond it are missing.
    pub extension_error: bool,
    /// A G28 inside the extension window invalidated Z knowledge for
    /// some candidates (their `z_known` is `false`).
    pub unknown_z_in_extension: bool,
    /// A G28 inside the extension window invalidated XY knowledge; the
    /// XY region includes untrusted values.
    pub unknown_xy_in_extension: bool,
    /// The E frame (`G92 E`/M221) shifted inside the extension window
    /// (informational; the replay-based file-frame E handles shifts
    /// exactly).
    pub e_frame_shift_in_extension: bool,
    /// The file-frame E replay could not run or could not reach the
    /// window end; `e_file` fell back to (or was widened by) the
    /// snapshot-frame union, which cannot see a frame created and
    /// replaced between two context flushes. Treat `e_file` as
    /// best-effort in this state.
    pub e_file_frames_incomplete: bool,
    /// No durable trapq row precedes the anchor context, so the
    /// extension's simulated clock was placed on the print-time axis
    /// from the `t_a - max_processing_lead` reader-lead bound instead of
    /// from motion evidence (see [`extension_start_time`]).
    ///
    /// Not a degradation of the result: that bound is a genuine lower
    /// bound on when the frontier begins executing, so coverage is
    /// preserved — it is *information*, telling an operator that the
    /// horizon rests on the reader-lead premise rather than on measured
    /// motion. Normal early in a print, before any trapq row is durable.
    pub extension_start_unanchored: bool,
    /// No context old enough to predate `t_a - max_processing_lead`
    /// exists; the offset-window floor fell back to the oldest known
    /// frontier and may be optimistic.
    pub offset_floor_uncertain: bool,
    /// The anchor context could not be placed on the print-time axis;
    /// the extension horizon used a conservative fallback catch-up.
    pub anchor_time_unknown: bool,
    /// How `e_internal` handled the un-evidenced extruder band — see the
    /// module-level "Durable extruder coverage".
    pub e_internal_band: BandOutcome,
}

/// What became of the `e_internal` un-evidenced-band union.
///
/// The band exists because durable extruder trapq rows can be missing for
/// lines the anchor already counts as processed; an E excursion confined
/// to those lines (a retract) is otherwise bounded by nothing. This says
/// which of three situations the log presented, because they have
/// genuinely different consequences and collapsing them into one boolean
/// would hide the one that matters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BandOutcome {
    /// No context carries `toolhead.print_time`, so no coverage
    /// certificate is computable: a **pre-change WAL**, or a printer whose
    /// status never carried `toolhead.print_time`.
    ///
    /// `e_internal` is left exactly as older readers computed it, and
    /// confidence is **not** forced down. That is deliberate and it is the
    /// only honest choice: the hazard on such a log is precisely what it
    /// has always been, and forcing per-layer here would turn every
    /// recovery from an existing log into a manual fallback — a large
    /// regression to mitigate nothing, since no new information is
    /// available to act on. The limit is real and unmitigated in this
    /// state; that is what this variant records.
    #[default]
    Uncertifiable,
    /// A certificate was found: every move from lines at or before the
    /// certified context's frontier is durable, and `e_internal` was
    /// widened by the replayed internal-E excursion over
    /// `(certified frontier, anchor frontier]`.
    ///
    /// The normal outcome during a print. Informational: it records that
    /// the band rests on the [`max_lookahead_lead`] premise
    /// ([`crate::config::ReconstructConfig::max_lookahead_lead`]) rather
    /// than on direct evidence of queue depth, which cannot be observed.
    Certified,
    /// `toolhead.print_time` **is** present but no context could be
    /// certified — durable extruder coverage never reached any context's
    /// append frontier plus the lookahead premise. Typical very early in a
    /// print, and after a drop storm that ate extruder rows.
    ///
    /// `e_internal` is **not** widened, and the reason is a measurement,
    /// not a preference: widening to the floor-wide band here reproduces
    /// the reverted whole-window union and regresses two `plrd` end-to-end
    /// tests to `ManualFallback` (see [`band_e_internal`]). So this variant
    /// currently *reports* an unmitigated exposure rather than trading it
    /// for a useless answer.
    ///
    /// **This flag has no decision consumer yet, and saying so is the
    /// point.** [`Degradation::confidence`] is not the lever it appears to
    /// be: nothing outside this crate reads it. Its only consumer in the
    /// whole workspace is a forensic `eprintln!` in `plrd::scan`; the
    /// matcher computes its own `MatchConfidence` from candidate count and
    /// never sees it. Wiring this variant to [`Confidence::PerLayer`] was
    /// tried and changed the behaviour of exactly zero tests, which is the
    /// definition of a vacuous guard. A real consumer has to live at the
    /// point that decides whether to offer a plan, and choosing it is a
    /// deliberate design decision rather than something to smuggle in
    /// here — see this crate's module docs.
    Uncertified,
}

/// The possible-stop set: everything downstream recovery needs to
/// bound where the machine actually stopped.
#[derive(Debug, Clone, PartialEq)]
pub struct PossibleStopSet {
    /// Lower time bound (copied from the window).
    pub t_a: f64,
    /// Upper time bound of WAL evaluation:
    /// `max(t_b, end of durable trapq data)`. See the module docs for
    /// why this exceeds `t_b`.
    pub wal_eval_end: f64,
    /// The exact Z enumeration, sorted by `z.lo` then `z.hi`,
    /// deduplicated per `z_merge_tolerance`.
    pub z_candidates: Vec<ZCandidate>,
    /// XY bounding region, `None` when no XY evidence exists at all.
    pub xy: Option<XyRegion>,
    /// E interval in the Klipper-internal (trapq) frame.
    pub e_internal: Option<Interval>,
    /// E interval in the file frame (what `G1 E...` words refer to).
    pub e_file: Option<Interval>,
    /// File byte-offset candidate window.
    pub file_window: Option<OffsetWindow>,
    /// What the forward extension did, `None` when it could not run.
    pub extension: Option<ExtensionSummary>,
    /// Honest evidence-quality report.
    pub degradation: Degradation,
}

impl PossibleStopSet {
    /// The Z span (max − min over trusted candidates, hop included):
    /// the quantity that sizes the probe envelope downstream. Excludes
    /// candidates with `z_known == false`; `None` when no trusted
    /// candidate exists.
    #[must_use]
    pub fn z_span(&self) -> Option<Interval> {
        let mut span: Option<Interval> = None;
        for c in self.z_candidates.iter().filter(|c| c.z_known) {
            span = Some(span.map_or(c.z, |s| s.union(c.z)));
        }
        span
    }

    /// `true` when some candidate (trusted or not) contains `z` within
    /// `tol`.
    #[must_use]
    pub fn contains_z(&self, z: f64, tol: f64) -> bool {
        self.z_candidates.iter().any(|c| c.z.contains(z, tol))
    }
}

/// A slice of the printed file: `bytes[0]` sits at `base_offset` in the
/// file. Pass the whole file with `base_offset == 0`, or just a tail
/// covering the last context's `file_position`.
#[derive(Debug, Clone, Copy)]
pub struct FileTail<'a> {
    /// File offset of `bytes[0]`.
    pub base_offset: u64,
    /// The raw file bytes.
    pub bytes: &'a [u8],
}

/// Builds the possible-stop set. Errors on missing/unusable
/// prerequisites ([`ReconstructError::NoContext`],
/// [`ReconstructError::MalformedContext`],
/// [`ReconstructError::FileTailMismatch`]); everything else degrades
/// with flags.
pub fn compute_stop_set(
    timeline: &WalTimeline,
    window: &StopWindow,
    file_tail: Option<&FileTail<'_>>,
    config: &ReconstructConfig,
) -> Result<PossibleStopSet, ReconstructError> {
    let anchor = timeline
        .contexts
        .last()
        .ok_or(ReconstructError::NoContext)?;
    let wal_eval_end = timeline
        .trapq_end_time()
        .map_or(window.t_b, |end| end.max(window.t_b));

    let (mut z_candidates, mut xy) =
        eval_toolhead_span(&timeline.toolhead_segments, window.t_a, wal_eval_end);
    let wal_e = eval_extruder_span(&timeline.extruder_segments, window.t_a, wal_eval_end);

    let mut degradation = Degradation {
        observation_gap: observation_gap_overlaps_window(timeline, window, config),
        ..Degradation::default()
    };

    let extension = run_extension(
        timeline,
        anchor,
        window,
        wal_eval_end,
        file_tail,
        config,
        &mut degradation,
    )?;
    let floor = floor_context(timeline, window, anchor, config, &mut degradation);

    let mut e_file = file_frame_e(
        timeline,
        window,
        anchor,
        floor.as_ref(),
        extension.as_ref(),
        file_tail,
        wal_e,
        config,
        &mut degradation,
    );
    let mut e_internal = wal_e;
    if let Some(ext) = &extension {
        z_candidates.extend(ext.z.iter().cloned());
        if let Some(ext_xy) = ext.xy {
            xy = Some(xy.map_or(ext_xy, |w| XyRegion {
                x: w.x.union(ext_xy.x),
                y: w.y.union(ext_xy.y),
            }));
        }
        e_internal = union_opt(e_internal, ext.e_internal);
        e_file = union_opt(e_file, ext.e_file);
    }

    // Close the un-evidenced extruder band: durable extruder rows can be
    // missing for lines the anchor already counts as processed, and an E
    // excursion confined to them (a retract) is bounded by nothing else
    // this crate holds. See the module-level "Durable extruder coverage".
    e_internal = union_opt(
        e_internal,
        band_e_internal(
            timeline,
            anchor,
            floor.as_ref(),
            file_tail,
            config,
            &mut degradation,
        ),
    );

    let file_window = offset_window(anchor, floor.as_ref(), extension.as_ref());

    degradation.confidence = if degradation.observation_gap
        || degradation.extension_unavailable
        || degradation.extension_truncated
        || degradation.extension_error
    {
        Confidence::PerLayer
    } else {
        Confidence::PerLine
    };

    let z_candidates = merge_z_candidates(z_candidates, config.z_merge_tolerance);

    Ok(PossibleStopSet {
        t_a: window.t_a,
        wal_eval_end,
        z_candidates,
        xy,
        e_internal,
        e_file,
        file_window,
        extension: extension.map(|e| e.summary),
        degradation,
    })
}

/// The internal-frame E interval over the un-evidenced extruder band, and
/// the [`BandOutcome`] that says how much the answer can be trusted.
///
/// Three outcomes, in the order they are decided:
///
/// 1. **No `toolhead.print_time` anywhere** → [`BandOutcome::Uncertifiable`],
///    `None`. A pre-change WAL. Behaviour is left exactly as it was, which
///    is the only non-regressive choice: no new information exists to act
///    on, and forcing per-layer would make every recovery from an existing
///    log a manual fallback.
/// 2. **Certified** → band `(certified frontier, anchor frontier]`.
/// 3. **Present but uncertified** → band `(floor, anchor frontier]`, the
///    maximally conservative choice, and confidence drops to per-layer in
///    [`compute_stop_set`].
///
/// The replay always seeds from the **floor** context (the oldest state
/// this crate trusts) regardless of where the band starts, because only a
/// floor-seeded replay reconstructs the `(base_e, extrude_factor)` frames
/// in force inside the band — the same reason [`file_frame_e`]'s replay
/// starts there.
fn band_e_internal(
    timeline: &WalTimeline,
    anchor: &Context,
    floor: Option<&FloorContext<'_>>,
    file_tail: Option<&FileTail<'_>>,
    config: &ReconstructConfig,
    degradation: &mut Degradation,
) -> Option<Interval> {
    if !any_print_time(&timeline.contexts) {
        degradation.e_internal_band = BandOutcome::Uncertifiable;
        return None;
    }
    let cov_end = extruder_coverage_end(&timeline.extruder_segments);
    let certified = coverage_certified_context(&timeline.contexts, cov_end, config);
    let (floor, tail, anchor_vsd) = (floor?, file_tail?, anchor.virtual_sdcard.as_ref()?);
    let band_end = anchor_vsd.file_position;

    let Some(vsd) = certified.and_then(|c| c.virtual_sdcard.as_ref()) else {
        degradation.e_internal_band = BandOutcome::Uncertified;
        // Deliberately does NOT widen. Widening to the floor here was
        // implemented and measured: it regresses
        // `full_pipeline_reaches_a_validated_plan` and
        // `a_recoverable_print_is_not_denied_by_the_identity_check` to
        // `ManualFallback("... 12 candidate lines across layers [0, 1];
        // below layer granularity")` — i.e. it reproduces exactly the
        // whole-window union that was reverted before. Isolated by
        // experiment: with the widening removed and the flag still set, all
        // 19 pipeline tests pass, so the widening is the sole cause.
        //
        // So this state reports the honest flag and leaves `e_internal` as
        // older readers computed it. The containment exposure is unchanged
        // from before this change — it is now *labelled* rather than
        // silent, which is strictly more than the log carried before, and
        // less than a guarantee.
        return None;
    };
    degradation.e_internal_band = BandOutcome::Certified;
    // A certified frontier ahead of the anchor's would invert the band;
    // clamp rather than trust it. Contexts are journaled in order so this
    // needs a reordered log to happen, but an inverted band would silently
    // collect nothing.
    let band_start = vsd.file_position.min(band_end);
    replay_band_internal_e(floor, band_start, band_end, tail, config)
}

/// End of durable extruder-queue coverage in print time: the newest
/// `print_time + duration` across journaled extruder rows.
///
/// Computed **per queue** (extruder rows only). A max that also folded in
/// toolhead rows would let travel moves certify coverage of extrusion that
/// was never journaled — and a pure retract produces an extruder row with
/// *no* toolhead row at all, because `Move.is_kinematic_move` is false for
/// extrude-only moves (`klippy/toolhead.py`), so the two queues genuinely
/// do not track each other.
fn extruder_coverage_end(segments: &[TrapqSegment]) -> Option<f64> {
    segments
        .iter()
        .map(TrapqSegment::end_time)
        .filter(|t| t.is_finite())
        .fold(None, |acc: Option<f64>, t| {
            Some(acc.map_or(t, |a: f64| a.max(t)))
        })
}

/// The newest context whose processing frontier is **certified** to be
/// fully covered by durable extruder trapq rows, with the file offset that
/// certifies.
///
/// # The certificate
///
/// Let `C_E` be the end of durable extruder coverage (`cov_end`) and let a
/// context report the atomic pair `(F, P)` — processing frontier and trapq
/// append frontier ([`Context::print_time`]). The context is certified
/// when
///
/// ```text
/// C_E >= P + max_lookahead_lead
/// ```
///
/// **Why that implies every move from lines `<= F` is durable**, in three
/// steps, each resting on a cited Klipper property:
///
/// 1. *Everything already appended at the snapshot is durable.* Appended
///    moves end at print time `<= P` (that is what `self.print_time`
///    means: `_advance_move_time` sets it to the end of the last appended
///    move). Batches deliver **contiguous** print-time coverage —
///    `DumpTrapQ._process_batch` resumes at
///    `last_batch_msg.print_time + min(move_t, 0.1)` and extracts to
///    `NEVER_TIME` (`klippy/extras/motion_report.py`) — so every row
///    ending `<= C_E` has been delivered. With `P <= C_E`, all of them
///    have.
/// 2. *The residue is bounded, by premise.* Lines `<= F` whose moves are
///    still in the `LookAheadQueue` get appended later, starting at print
///    time `P` (`_process_lookahead` seeds `next_move_time =
///    self.print_time`) and in FIFO file order. By
///    [`max_lookahead_lead`](crate::config::ReconstructConfig::max_lookahead_lead)
///    the queue holds at most that much move time, so the residue ends by
///    `P + max_lookahead_lead <= C_E` — hence it too is delivered.
/// 3. *Nothing from lines `<= F` is left.* Steps 1 and 2 exhaust them:
///    every such move is either appended-by-snapshot or residue.
///
/// Step 2 is the premise; steps 1 and 3 are proved. See the config field
/// for the violation mode and its cost.
///
/// # Direction of every approximation
///
/// * `cov_end` is computed from rows actually in the durable log, so drops
///   and batching **lower** it → certificate harder to satisfy → band
///   wider. Safe.
/// * `P` errs **high** under the one-line sampling skew
///   ([`Context::print_time`]), which demands *more* coverage. Safe.
/// * Scanning newest-first and taking the first certified context makes
///   the band as narrow as the certificate allows; taking any older
///   context would only widen it. Safe either way, so precision wins.
fn coverage_certified_context<'a>(
    contexts: &'a [Context],
    cov_end: Option<f64>,
    config: &ReconstructConfig,
) -> Option<&'a Context> {
    let cov_end = cov_end?;
    if !cov_end.is_finite() {
        return None;
    }
    // A non-finite or negative premise would make the comparison
    // meaningless; fall back to "no certificate" (widest band).
    let lead = config.max_lookahead_lead;
    if !lead.is_finite() || lead < 0.0 {
        return None;
    }
    contexts.iter().rev().find(|ctx| {
        ctx.print_time
            .is_some_and(|p| p.is_finite() && cov_end >= p + lead)
    })
}

/// `true` when any context carries `toolhead.print_time` — i.e. the log is
/// new enough for a coverage certificate to be *computable* at all.
/// Distinguishes [`BandOutcome::Uncertifiable`] from
/// [`BandOutcome::Uncertified`].
fn any_print_time(contexts: &[Context]) -> bool {
    contexts
        .iter()
        .any(|c| c.print_time.is_some_and(f64::is_finite))
}

/// Replays the un-evidenced band and returns the **internal**-frame E
/// interval across it, which is what `e_internal` is missing.
///
/// `band_start` is the certified frontier (or the floor, uncertified);
/// `band_end` the anchor frontier. Both are widened by one line before
/// replaying — see "one-line sampling skew" on
/// [`plr_wal::GcodeState::position`]: the seed state can already include
/// the line after the recorded frontier, and under relative extrusion
/// (`M83`, the slicer default) re-applying it shifts the replayed E by one
/// line's delta in the *unsafe* direction. Starting one line earlier and
/// ending one line later makes the reported interval a superset of the
/// truth either way, at a cost of two lines of width.
fn replay_band_internal_e(
    seed: &FloorContext<'_>,
    band_start: u64,
    band_end: u64,
    tail: &FileTail<'_>,
    config: &ReconstructConfig,
) -> Option<Interval> {
    let tail_end = tail.base_offset.saturating_add(tail.bytes.len() as u64);
    if seed.file_position < tail.base_offset || seed.file_position > tail_end {
        return None;
    }
    let mut state = anchor_state_from_context(&seed.ctx.gcode).ok()?;
    // Fits: file_position - base_offset <= bytes.len().
    #[allow(clippy::cast_possible_truncation)]
    let skip = (seed.file_position - tail.base_offset) as usize;
    let bytes = tail.bytes.get(skip..).unwrap_or(&[]);
    let mut budget = config.sim.max_lines.unwrap_or(usize::MAX);
    let mut out: Option<Interval> = None;
    // `prev_e` carries the E value from one line back so the band can be
    // widened by a leading line without a second pass; `past_end` grants
    // exactly one trailing line.
    let mut prev_e: Option<f64> = None;
    let mut past_end = false;
    for line in LineIter::new(bytes, seed.file_position) {
        if budget == 0 || state.apply(&line).is_err() {
            break;
        }
        budget -= 1;
        // Internal (trapq-frame) accumulated E — the frame `e_internal` is
        // expressed in. NOT `gcode_position()[3]`, which is the file frame
        // that `replay_file_e` collects.
        let e = state.last_position[3];
        if line.span.end >= band_start {
            // Leading slack: on first entry, seed from the line *before*
            // the band so a one-line-late seed cannot shift the interval
            // off the truth.
            if out.is_none() {
                if let Some(p) = prev_e.filter(|v| v.is_finite()) {
                    out = Some(Interval::point(p));
                }
            }
            if e.is_finite() {
                match &mut out {
                    Some(iv) => iv.expand(e),
                    None => out = Some(Interval::point(e)),
                }
            }
        }
        if line.span.end >= band_end {
            // One trailing line, then stop.
            if past_end {
                break;
            }
            past_end = true;
        }
        prev_e = Some(e);
    }
    out
}

/// Union of two optional intervals.
fn union_opt(a: Option<Interval>, b: Option<Interval>) -> Option<Interval> {
    match (a, b) {
        (Some(a), Some(b)) => Some(a.union(b)),
        (a, b) => a.or(b),
    }
}

/// Converts a WAL context's `gcode_move` snapshot into a simulator
/// state, mirroring the daemon's capture of Klipper's
/// `gcode_move.get_status` exactly:
///
/// * status `speed_factor` is `speed_factor * 60` → internal factor is
///   `status / 60`;
/// * status `speed` is `_get_gcode_speed()` = `speed / speed_factor` →
///   internal speed is `status_speed * internal_factor`;
/// * `base_position = position - gcode_position` per axis, with the E
///   component scaled back through `extrude_factor`
///   (`_get_gcode_position` divides by it).
///
/// Position knowledge starts fully known: the snapshot came from a live,
/// printing Klipper.
pub fn anchor_state_from_context(
    gcode: &plr_wal::GcodeState,
) -> Result<GcodeState, ReconstructError> {
    let defect = |defect| Err(ReconstructError::MalformedContext { defect });
    if !gcode.values_are_finite() {
        return defect(ContextDefect::NonFinite);
    }
    let (Some(pos), Some(gpos), Some(origin)) = (
        first_four(&gcode.position),
        first_four(&gcode.gcode_position),
        first_four(&gcode.homing_origin),
    ) else {
        return defect(ContextDefect::TooFewAxes);
    };
    let extrude_factor = gcode.extrude_factor;
    if !(extrude_factor.is_finite() && extrude_factor > 0.0) {
        return defect(ContextDefect::BadExtrudeFactor);
    }
    let factor_mult = gcode.speed_factor;
    if !(factor_mult.is_finite() && factor_mult > 0.0) {
        return defect(ContextDefect::BadSpeedFactor);
    }
    let speed_factor = factor_mult / 60.0;
    let speed = gcode.speed * speed_factor;
    if !(speed.is_finite() && speed > 0.0) {
        return defect(ContextDefect::NonPositiveSpeed);
    }
    let base_position = [
        pos[0] - gpos[0],
        pos[1] - gpos[1],
        pos[2] - gpos[2],
        pos[3] - gpos[3] * extrude_factor,
    ];
    Ok(GcodeState {
        absolute_coord: gcode.absolute_coordinates,
        absolute_extrude: gcode.absolute_extrude,
        base_position,
        last_position: pos,
        homing_position: origin,
        speed,
        speed_factor,
        extrude_factor,
        ..GcodeState::default()
    })
}

/// First four components of a coordinate vector, or `None`.
fn first_four(v: &[f64]) -> Option<[f64; 4]> {
    Some([*v.first()?, *v.get(1)?, *v.get(2)?, *v.get(3)?])
}

/// Evaluates toolhead trapq segments over `[t_a, t_end]`, producing Z
/// candidates and the XY bounding region. Positions between segments
/// (dwells) hold the previous segment's end, which the clamped
/// end-sample covers; a dwell *at* `t_a` is covered by sampling the
/// last segment that ended before the window.
fn eval_toolhead_span(
    segments: &[TrapqSegment],
    t_a: f64,
    t_end: f64,
) -> (Vec<ZCandidate>, Option<XyRegion>) {
    let mut z = Vec::new();
    let mut xy: Option<XyRegion> = None;
    let mut expand_xy = |x: f64, y: f64| {
        if x.is_finite() && y.is_finite() {
            match &mut xy {
                Some(region) => region.expand(x, y),
                None => {
                    xy = Some(XyRegion {
                        x: Interval::point(x),
                        y: Interval::point(y),
                    });
                }
            }
        }
    };
    let mut last_before: Option<&TrapqSegment> = None;
    let mut covers_t_a = false;
    for seg in segments {
        if seg.end_time() < t_a {
            let newer = last_before.is_none_or(|prev| seg.end_time() >= prev.end_time());
            if newer {
                last_before = Some(seg);
            }
            continue;
        }
        if seg.print_time > t_end {
            continue;
        }
        // Overlap is non-empty: end_time >= t_a and print_time <= t_end.
        let lo = t_a.max(seg.print_time);
        let hi = t_end.min(seg.end_time());
        if seg.print_time <= t_a {
            covers_t_a = true;
        }
        let samples = sample_extrema(seg, lo, hi);
        for s in &samples {
            expand_xy(s[0], s[1]);
        }
        // Exact-zero z_r means Klipper planned a Z-constant move; any
        // nonzero component is a ramp and gets interval treatment.
        #[allow(clippy::float_cmp)]
        let plateau = seg.z_r == 0.0;
        let z_interval = samples
            .iter()
            .map(|s| Interval::point(s[2]))
            .reduce(Interval::union);
        if let Some(mut interval) = z_interval {
            let kind = if plateau {
                interval = Interval::point(interval.lo);
                ZKind::Plateau
            } else {
                ZKind::Ramp
            };
            if interval.lo.is_finite() && interval.hi.is_finite() {
                z.push(ZCandidate {
                    z: interval,
                    provenance: Provenance::Wal,
                    z_known: true,
                    kind,
                });
            }
        }
    }
    if !covers_t_a {
        if let Some(seg) = last_before {
            // Dwell at the window start: the machine holds this
            // segment's end position.
            let p = seg.position_at(t_a);
            expand_xy(p[0], p[1]);
            if p[2].is_finite() {
                z.push(ZCandidate {
                    z: Interval::point(p[2]),
                    provenance: Provenance::Wal,
                    z_known: true,
                    kind: ZKind::Plateau,
                });
            }
        }
    }
    (z, xy)
}

/// Evaluates extruder trapq segments over `[t_a, t_end]` into an
/// internal-E interval (filament position rides in the X slot).
fn eval_extruder_span(segments: &[TrapqSegment], t_a: f64, t_end: f64) -> Option<Interval> {
    let mut e: Option<Interval> = None;
    let mut expand = |v: f64| {
        if v.is_finite() {
            match &mut e {
                Some(iv) => iv.expand(v),
                None => e = Some(Interval::point(v)),
            }
        }
    };
    let mut last_before: Option<&TrapqSegment> = None;
    let mut covers_t_a = false;
    for seg in segments {
        if seg.end_time() < t_a {
            if last_before.is_none_or(|prev| seg.end_time() >= prev.end_time()) {
                last_before = Some(seg);
            }
            continue;
        }
        if seg.print_time > t_end {
            continue;
        }
        if seg.print_time <= t_a {
            covers_t_a = true;
        }
        let lo = t_a.max(seg.print_time);
        let hi = t_end.min(seg.end_time());
        for s in sample_extrema(seg, lo, hi) {
            expand(s[0]);
        }
    }
    if !covers_t_a {
        if let Some(seg) = last_before {
            expand(seg.position_at(t_a)[0]);
        }
    }
    e
}

/// Samples a segment's position at the overlap bounds plus the interior
/// velocity-zero point if one exists: since per-axis position is
/// `start + r * dist(t)` with `dist` quadratic in `t`, these samples
/// bound every interior position per axis.
fn sample_extrema(seg: &TrapqSegment, lo: f64, hi: f64) -> Vec<[f64; 3]> {
    let mut samples = vec![seg.position_at(lo), seg.position_at(hi)];
    #[allow(clippy::float_cmp)] // exact zero: no acceleration, no interior extremum
    if seg.acceleration != 0.0 {
        let t_star = seg.print_time - seg.start_velocity / seg.acceleration;
        if t_star.is_finite() && t_star > lo && t_star < hi {
            samples.push(seg.position_at(t_star));
        }
    }
    samples
}

/// Everything the forward extension contributes.
struct ExtensionResult {
    summary: ExtensionSummary,
    z: Vec<ZCandidate>,
    xy: Option<XyRegion>,
    e_internal: Option<Interval>,
    e_file: Option<Interval>,
}

/// Print time at which the extension's **first simulated move begins
/// executing** — the origin of the simulated clock, and therefore of the
/// horizon.
///
/// The extension starts at the anchor context's processing frontier `F`.
/// The machine reaches `F` only after finishing every move produced by
/// lines before `F`, at a print time this function lower-bounds. Getting
/// it wrong in the *late* direction is a containment bug, not an
/// efficiency loss: a frontier can correspond to an execution time
/// seconds *earlier* than the snapshot that recorded it, and a horizon
/// measured from the snapshot time is then spent re-simulating
/// already-executed motion (the containment hole behind proptest seed
/// `9938d965…`).
///
/// Bounds used, smallest wins:
///
/// * **`trapq_end_time_journaled_by(anchor.mono_ns)` — motion evidence,
///   the anchored branch.** The newest trapq row journaled by the
///   anchor's capture time; its end is a lower bound on when motion
///   preceding `F` completes.
///
///   **This is a lower bound, not the quantity itself, and the
///   distinction is load-bearing.** An earlier version of this comment
///   claimed "every line up to `F` was processed by the anchor's capture
///   time, so its trapq row was journaled by then". That is **false**, in
///   two independent ways. Klipper's reader advances `file_position`
///   *after* running a line (`klippy/extras/virtual_sdcard.py`,
///   `work_handler`), but the move that line produced first sits in a
///   Python-side `LookAheadQueue` that `dump_trapq` cannot see at all —
///   `trapq_extract_old` (`klippy/chelper/trapq.c`) walks only
///   `tq->moves` and `tq->history` — and is appended to the trapq only on
///   a later flush (`ToolHead._process_lookahead`); *then* it waits up to
///   one ~0.5 s dump batch (`klippy/extras/bulk_sensor.py`,
///   `BATCH_INTERVAL`). So rows for lines before `F` are routinely
///   missing from this set.
///
///   It remains safe **here** precisely because a lower bound is what
///   this function wants: missing rows can only lower the value, and only
///   the smallest term shortens the horizon. It is **not** safe wherever
///   the same premise would license treating trapq evidence as *complete*
///   — see the module-level "Durable extruder coverage".
/// * **`t_a - max_processing_lead` — the degenerate branch**, taken when
///   no durable trapq row precedes the anchor and reported as
///   [`Degradation::extension_start_unanchored`]. It rests on the same
///   premise as the offset-window floor rather than a new invention: the
///   reader runs at most `max_processing_lead` ahead of execution, so a
///   frontier recorded at or before `t_a` cannot have begun executing
///   earlier than `t_a - max_processing_lead` unless the reader stalled
///   for longer than that — the very condition the motion-evidence bound
///   detects whenever rows are present. Containment on this branch is
///   therefore proven *from that premise*, not from direct evidence, and
///   the flag says which branch was taken.
/// * **`anchor_pt`** — the anchor's own capture time. With the reader
///   ahead of execution (the usual case) execution has not reached `F`
///   yet, so this is a valid, looser bound.
///
/// `window.t_a` seeds the fold, so **`start_pt <= t_a` on every path**,
/// including when both other bounds are absent or non-finite. That cap
/// is also the safety envelope against a klippy restart resetting the
/// print-time axis while host `mono_ns` keeps advancing: however far
/// forward a stale correlation places `anchor_pt`, the origin can never
/// be pushed past the instant the machine was last *proven* to be
/// executing, so the horizon can never shrink below the stop window
/// itself.
fn extension_start_time(
    timeline: &WalTimeline,
    window: &StopWindow,
    anchor: &Context,
    anchor_pt: Option<f64>,
    config: &ReconstructConfig,
    degradation: &mut Degradation,
) -> f64 {
    let journaled = timeline.trapq_end_time_journaled_by(anchor.mono_ns);
    let degenerate = if journaled.is_none() {
        degradation.extension_start_unanchored = true;
        Some(window.t_a - config.max_processing_lead)
    } else {
        None
    };
    // Every available bound participates on EVERY path: "anchored" names
    // which evidence exists, not which term wins. On the anchored branch
    // `anchor_pt` still competes and can be the smallest of the three,
    // which is deliberate — only the smallest term can shorten the
    // horizon, so a term being *larger* than another never matters, and
    // each is independently a valid lower bound on when the frontier
    // begins executing (or, for `t_a`, the cap justified above). Taking
    // the min can therefore only widen coverage, never narrow it.
    //
    // `t_a` is both the third bound and the seed of the fold, so the
    // result is finite even when neither other source is available, and
    // non-finite candidates are dropped rather than poisoning the min.
    [journaled, degenerate, anchor_pt]
        .into_iter()
        .flatten()
        .filter(|pt| pt.is_finite())
        .fold(window.t_a, f64::min)
}

/// Runs the forward extension; `Ok(None)` when it cannot run (no
/// `virtual_sdcard` state or no file tail), flagged on `degradation`.
fn run_extension(
    timeline: &WalTimeline,
    anchor: &Context,
    window: &StopWindow,
    wal_eval_end: f64,
    file_tail: Option<&FileTail<'_>>,
    config: &ReconstructConfig,
    degradation: &mut Degradation,
) -> Result<Option<ExtensionResult>, ReconstructError> {
    let (Some(vsd), Some(tail)) = (&anchor.virtual_sdcard, file_tail) else {
        degradation.extension_unavailable = true;
        return Ok(None);
    };
    let tail_end = tail.base_offset.saturating_add(tail.bytes.len() as u64);
    if vsd.file_position < tail.base_offset || vsd.file_position > tail_end {
        return Err(ReconstructError::FileTailMismatch {
            base_offset: tail.base_offset,
            tail_end,
            file_position: vsd.file_position,
        });
    }
    let anchor_state = anchor_state_from_context(&anchor.gcode)?;

    let anchor_pt = window.mono_ns_to_print_time(anchor.mono_ns);
    if anchor_pt.is_none() {
        degradation.anchor_time_unknown = true;
    }
    let start_pt = extension_start_time(timeline, window, anchor, anchor_pt, config, degradation);
    // The simulated clock starts when the machine begins the first
    // simulated line, at print time `start_pt`; the latest possible stop
    // is `wal_eval_end + extension_horizon`.
    //
    // Coverage rests on plr_gcode's per-line accounting being a lower
    // bound on real durations (`min_move_t`, distance over capped cruise
    // velocity), which holds for every move *except the first*: the
    // simulation window starts from zero velocity while the real machine
    // was typically mid-motion at the resume offset, so the first move's
    // duration can be OVERestimated by up to one acceleration ramp
    // (`plr-gcode/src/sim.rs` module docs). The excess is bounded by
    // `max_velocity / max_accel` (0.1 s at the Klipper-typical
    // 300 mm/s and 3000 mm/s², one twentieth of the 2 s default
    // `extension_horizon`) and it bites hardest when the horizon is
    // short. So the claim is: simulating this many seconds consumes at
    // least every line the machine could have executed by then, minus at
    // most one accel ramp of the first move — not an unqualified "every
    // line", and the ramp is why `extension_horizon` carries margin over
    // the ~1.5 s of unreceived tail it must cover rather than being
    // sized to it exactly.
    //
    // `wal_eval_end` and `start_pt` both derive from raw
    // `TrapqSegment::print_time`/`duration` values, range-checked at
    // ingest only for finiteness, so this is an untrusted-input surface.
    // A hostile-but-finite value (±1e300) makes the span huge; that
    // cannot panic (`simulate` bounds itself by `sim.max_lines`) and it
    // degrades honestly: the line budget stops consumption,
    // `StopReason::LineBudget` sets `extension_truncated`, which forces
    // `Confidence::PerLayer`.
    //
    // If the subtraction itself overflows to ±infinity (or produces NaN
    // from two same-signed infinities), the time axis is unusable. The
    // fallback then has to be the CONSERVATIVE branch, not the
    // convenient one: collapsing to `extension_horizon` alone would give
    // the NARROWEST possible horizon — a confident, narrow answer built
    // on arithmetic that just failed, which is the containment-unsafe
    // direction and exactly the bug this module was fixed for. So an
    // unusable axis means an unbounded horizon (consumption limited only
    // by `sim.max_lines`, giving maximal coverage) plus
    // `extension_truncated`, which forces `PerLayer` regardless of where
    // the simulation happens to stop — an honest refusal instead of a
    // confident guess, and the same honest branch the paragraph above
    // describes for huge-but-finite values.
    //
    // Note `NaN.max(0.0)` returns 0.0 (`f64::max` prefers the non-NaN
    // operand), so the finiteness test must come BEFORE the clamp or NaN
    // would silently reach the narrow path.
    let span = wal_eval_end - start_pt;
    let horizon = if span.is_finite() {
        config.extension_horizon + span.max(0.0)
    } else {
        degradation.extension_truncated = true;
        f64::INFINITY
    };

    // Byte skip fits in usize: file_position - base_offset <= bytes.len().
    #[allow(clippy::cast_possible_truncation)]
    let skip = (vsd.file_position - tail.base_offset) as usize;
    let bytes = tail.bytes.get(skip..).unwrap_or(&[]);
    let lines: Vec<Line> = match config.sim.max_lines {
        Some(cap) => LineIter::new(bytes, vsd.file_position).take(cap).collect(),
        None => LineIter::new(bytes, vsd.file_position).collect(),
    };
    let collect_capped = config.sim.max_lines.is_some_and(|cap| {
        lines.len() == cap && lines.last().is_some_and(|l| l.span.end < tail_end)
    });

    let mut sim_config = config.sim.clone();
    sim_config.max_duration = Some(horizon);
    let mut sim_state = anchor_state.clone();
    let sim = simulate(&mut sim_state, &lines, &sim_config);
    let consumed = lines.get(..sim.lines_consumed).unwrap_or(&lines[..]);

    // `|=`, not `=`: an unusable time axis already set this above, and a
    // simulation that then stopped at end-of-input must not clear it.
    degradation.extension_truncated |= matches!(sim.stop, StopReason::LineBudget) || collect_capped;
    degradation.extension_error = matches!(sim.stop, StopReason::LineError { .. });

    let z = extension_z_candidates(&anchor_state, consumed, degradation);
    let (xy, e_internal) = extension_xy_e(&anchor_state, &sim.moves, degradation);
    let e_file = extension_file_e(&anchor_state, consumed, degradation);

    Ok(Some(ExtensionResult {
        summary: ExtensionSummary {
            anchor_offset: vsd.file_position,
            anchor_print_time: anchor_pt,
            horizon,
            lines_consumed: sim.lines_consumed,
            resume_offset: sim.resume_offset,
            stop: sim.stop,
        },
        z,
        xy,
        e_internal,
        e_file: Some(e_file),
    }))
}

/// Exact Z enumeration over exactly the consumed lines: the anchor
/// level, plus a ramp and a resulting plateau per Z event, via
/// [`scan_z_events`] (no timing model involved).
fn extension_z_candidates(
    anchor_state: &GcodeState,
    consumed: &[Line],
    degradation: &mut Degradation,
) -> Vec<ZCandidate> {
    let mut z_state = anchor_state.clone();
    let z_scan = scan_z_events(
        &mut z_state,
        consumed,
        &ZScanConfig {
            max_lines: None,
            max_events: None,
        },
    );
    let mut z = vec![ZCandidate {
        z: Interval::point(anchor_state.last_position[2]),
        provenance: Provenance::Extension,
        z_known: true,
        kind: ZKind::Plateau,
    }];
    for event in &z_scan.events {
        if !event.z_known {
            degradation.unknown_z_in_extension = true;
        }
        z.push(ZCandidate {
            z: Interval::from_pair(event.z_from, event.z_to),
            provenance: Provenance::Extension,
            z_known: event.z_known,
            kind: ZKind::Ramp,
        });
        z.push(ZCandidate {
            z: Interval::point(event.z_to),
            provenance: Provenance::Extension,
            z_known: event.z_known,
            kind: ZKind::Plateau,
        });
    }
    z
}

/// XY region and internal-E interval from the timed moves. Chords are
/// straight lines, so endpoints bound every interior position.
fn extension_xy_e(
    anchor_state: &GcodeState,
    moves: &[plr_gcode::TimedMove],
    degradation: &mut Degradation,
) -> (Option<XyRegion>, Option<Interval>) {
    let mut xy = XyRegion {
        x: Interval::point(anchor_state.last_position[0]),
        y: Interval::point(anchor_state.last_position[1]),
    };
    let mut e_internal = Interval::point(anchor_state.last_position[3]);
    for tm in moves {
        let m = &tm.planned;
        if !(m.start_known[0] && m.start_known[1] && m.end_known[0] && m.end_known[1]) {
            degradation.unknown_xy_in_extension = true;
        }
        let end = m.kinematic_end();
        xy.expand(m.start[0], m.start[1]);
        xy.expand(end[0], end[1]);
        e_internal.expand(m.start[3]);
        e_internal.expand(end[3]);
    }
    (Some(xy), Some(e_internal))
}

/// File-frame E: replays the consumed lines, recording the g-code E
/// after every line and watching the `(base_e, extrude_factor)` frame.
fn extension_file_e(
    anchor_state: &GcodeState,
    consumed: &[Line],
    degradation: &mut Degradation,
) -> Interval {
    let mut frame_state = anchor_state.clone();
    let mut e_file = Interval::point(frame_state.gcode_position()[3]);
    let mut frame = (frame_state.base_position[3], frame_state.extrude_factor);
    for line in consumed {
        if frame_state.apply(line).is_err() {
            break; // same prefix simulate applied cleanly; defensive only
        }
        e_file.expand(frame_state.gcode_position()[3]);
        let now = (frame_state.base_position[3], frame_state.extrude_factor);
        #[allow(clippy::float_cmp)] // any change at all is a frame shift
        if now != frame {
            degradation.e_frame_shift_in_extension = true;
            frame = now;
        }
    }
    e_file
}

/// The offset-window floor: the newest context (for the anchor's file)
/// whose capture time predates `t_a - max_processing_lead` — execution
/// at any in-window time had certainly passed its recorded frontier.
/// Falls back to the oldest same-file context (flagged
/// `offset_floor_uncertain`) when nothing is old enough.
struct FloorContext<'a> {
    ctx: &'a Context,
    file_position: u64,
    /// `true` when the fallback was taken: the floor may sit *after*
    /// motion that was still executing in the window.
    uncertain: bool,
}

fn floor_context<'a>(
    timeline: &'a WalTimeline,
    window: &StopWindow,
    anchor: &Context,
    config: &ReconstructConfig,
    degradation: &mut Degradation,
) -> Option<FloorContext<'a>> {
    let anchor_vsd = anchor.virtual_sdcard.as_ref()?;
    let threshold = window.t_a - config.max_processing_lead;
    let same_file = |ctx: &&'a Context| {
        ctx.virtual_sdcard
            .as_ref()
            .is_some_and(|v| v.file_path == anchor_vsd.file_path)
    };
    let fp_of = |ctx: &&'a Context| ctx.virtual_sdcard.as_ref().map_or(0, |v| v.file_position);
    let old_enough = timeline
        .contexts
        .iter()
        .filter(same_file)
        .filter(|ctx| {
            window
                .mono_ns_to_print_time(ctx.mono_ns)
                .is_some_and(|pt| pt <= threshold)
        })
        .max_by_key(fp_of);
    let (ctx, uncertain) = if let Some(ctx) = old_enough {
        (ctx, false)
    } else {
        degradation.offset_floor_uncertain = true;
        let oldest = timeline
            .contexts
            .iter()
            .filter(same_file)
            .min_by_key(fp_of)?;
        (oldest, true)
    };
    Some(FloorContext {
        file_position: fp_of(&ctx),
        ctx,
        uncertain,
    })
}

/// File-frame ("g-code") E over the whole candidate window.
///
/// Primary (exact) path: replay the file from the **floor context**
/// through the extension end, recording the g-code-frame E after every
/// line. The interpreter is deterministic, so this reconstructs the
/// exact `(base_e, extrude_factor)` frame of *every* line in the offset
/// window — including frames created and replaced between two context
/// flushes (e.g. `G92 E0` + retract processed in a burst after a
/// dwell), which no snapshot ever captured. Consecutive per-line values
/// bracket every mid-line E because E moves monotonically within a line
/// (one `G1` sets one E target).
///
/// Only the *file* frame is taken from this replay. The replay spans the
/// whole offset window, whose floor is a deliberately loose **lower
/// bound** on the stop offset, so its Klipper-internal E envelope would
/// cover states the liveness proof at `t_a` already excludes — and
/// `e_internal` is what the downstream line matcher narrows candidates
/// with, so widening it that far costs per-line granularity on every
/// recovery. File-frame E does not have that problem: it is consumed
/// only as a frame-corrected reading of the same interval.
///
/// Fallback for the file frame (when the replay cannot run or cannot
/// reach the window end — no file tail, tail not covering the floor,
/// malformed floor snapshot, line budget, unparseable line): union the
/// WAL-internal interval converted under every recent context frame,
/// flagged [`Degradation::e_file_frames_incomplete`] because a frame
/// that lived entirely between two flushes is unrecoverable without the
/// file. The fallback is also unioned in when the floor itself is
/// uncertain, since the replay may then start past in-window motion.
#[allow(clippy::too_many_arguments)] // one cohesive assembly step; a param struct would just rename the call site
fn file_frame_e(
    timeline: &WalTimeline,
    window: &StopWindow,
    anchor: &Context,
    floor: Option<&FloorContext<'_>>,
    extension: Option<&ExtensionResult>,
    file_tail: Option<&FileTail<'_>>,
    wal_e: Option<Interval>,
    config: &ReconstructConfig,
    degradation: &mut Degradation,
) -> Option<Interval> {
    let end_offset = extension
        .and_then(|e| e.summary.resume_offset)
        .or_else(|| anchor.virtual_sdcard.as_ref().map(|v| v.file_position));
    let replay = match (floor, file_tail, end_offset) {
        (Some(floor), Some(tail), Some(end)) => replay_file_e(floor, end, tail, config),
        _ => None,
    };
    let context_fallback =
        || wal_e.and_then(|iv| convert_e_to_file_frame(iv, timeline, window, anchor, config));
    let Some(replay) = replay else {
        degradation.e_file_frames_incomplete = true;
        return context_fallback();
    };
    if replay.reached_end && floor.is_some_and(|f| !f.uncertain) {
        return Some(replay.file);
    }
    if !replay.reached_end {
        degradation.e_file_frames_incomplete = true;
    }
    union_opt(Some(replay.file), context_fallback())
}

/// Replays file bytes from the floor context's offset up to
/// `end_offset`, returning the interval of per-line g-code-frame E
/// values and whether the replay actually reached `end_offset`.
/// `None` when the replay cannot start (tail does not cover the floor,
/// or the floor snapshot cannot seed a state).
fn replay_file_e(
    floor: &FloorContext<'_>,
    end_offset: u64,
    tail: &FileTail<'_>,
    config: &ReconstructConfig,
) -> Option<ReplayedE> {
    let tail_end = tail.base_offset.saturating_add(tail.bytes.len() as u64);
    if floor.file_position < tail.base_offset || floor.file_position > tail_end {
        return None;
    }
    let mut state = anchor_state_from_context(&floor.ctx.gcode).ok()?;
    // Fits: file_position - base_offset <= bytes.len().
    #[allow(clippy::cast_possible_truncation)]
    let skip = (floor.file_position - tail.base_offset) as usize;
    let bytes = tail.bytes.get(skip..).unwrap_or(&[]);
    let mut file = Interval::point(state.gcode_position()[3]);
    let mut position = floor.file_position;
    let mut budget = config.sim.max_lines.unwrap_or(usize::MAX);
    let mut truncated = false;
    for line in LineIter::new(bytes, floor.file_position) {
        if line.span.start >= end_offset {
            break;
        }
        if budget == 0 || state.apply(&line).is_err() {
            truncated = true;
            break;
        }
        budget -= 1;
        file.expand(state.gcode_position()[3]);
        position = line.span.end;
    }
    Some(ReplayedE {
        file,
        reached_end: !truncated && (position >= end_offset || position >= tail_end),
    })
}

/// Per-line file-frame E envelope recovered by replaying the offset
/// window, plus whether the replay reached the window end.
struct ReplayedE {
    /// G-code-frame ("file") E across every replayed line.
    file: Interval,
    /// Whether the replay actually reached the window end.
    reached_end: bool,
}

/// Converts a Klipper-internal E interval to the file frame by unioning
/// the conversion under every recent context's `(base_e, extrude_factor)`
/// pair — the fallback path of [`file_frame_e`]; a frame that was
/// created *and* replaced between two context flushes is invisible
/// here, which is why the replay path is primary.
fn convert_e_to_file_frame(
    internal: Interval,
    timeline: &WalTimeline,
    window: &StopWindow,
    anchor: &Context,
    config: &ReconstructConfig,
) -> Option<Interval> {
    let cutoff = window.t_a - config.max_processing_lead - 1.0;
    let mut out: Option<Interval> = None;
    for ctx in &timeline.contexts {
        let recent = std::ptr::eq(ctx, anchor)
            || window
                .mono_ns_to_print_time(ctx.mono_ns)
                .is_none_or(|pt| pt >= cutoff);
        if !recent {
            continue;
        }
        let Some(frame) = e_frame_of(&ctx.gcode) else {
            continue;
        };
        let (base_e, factor) = frame;
        let converted = Interval::from_pair(
            (internal.lo - base_e) / factor,
            (internal.hi - base_e) / factor,
        );
        if converted.lo.is_finite() && converted.hi.is_finite() {
            out = Some(out.map_or(converted, |o| o.union(converted)));
        }
    }
    out
}

/// The `(base_e, extrude_factor)` pair a context's snapshot implies, or
/// `None` when the snapshot cannot support the conversion.
fn e_frame_of(gcode: &plr_wal::GcodeState) -> Option<(f64, f64)> {
    let pos_e = *gcode.position.get(3)?;
    let gpos_e = *gcode.gcode_position.get(3)?;
    let factor = gcode.extrude_factor;
    if !(factor.is_finite() && factor > 0.0 && pos_e.is_finite() && gpos_e.is_finite()) {
        return None;
    }
    Some((pos_e - gpos_e * factor, factor))
}

/// Builds the file byte-offset candidate window from the floor context
/// and the extension end; see module docs for the floor rationale.
fn offset_window(
    anchor: &Context,
    floor: Option<&FloorContext<'_>>,
    extension: Option<&ExtensionResult>,
) -> Option<OffsetWindow> {
    let anchor_vsd = anchor.virtual_sdcard.as_ref()?;
    let start = floor.map_or(anchor_vsd.file_position, |f| f.file_position);
    let end = extension
        .and_then(|e| e.summary.resume_offset)
        .unwrap_or(anchor_vsd.file_position)
        .max(anchor_vsd.file_position);
    Some(OffsetWindow {
        start: start.min(end),
        end,
    })
}

/// `true` when a subscription gap or socket loss could overlap
/// `[t_a - max_processing_lead, cut]` on the host-monotonic axis.
fn observation_gap_overlaps_window(
    timeline: &WalTimeline,
    window: &StopWindow,
    config: &ReconstructConfig,
) -> bool {
    let threshold_s = window.print_time_to_mono_s(window.t_a - config.max_processing_lead);
    timeline
        .markers
        .iter()
        .filter(|m| is_observation_gap(&m.kind))
        .any(|m| {
            let marker_end_s =
                if let plr_wal::MarkerKind::SubscriptionGap { end_mono_ns, .. } = m.kind {
                    ns_to_s(end_mono_ns)
                } else {
                    ns_to_s(m.mono_ns)
                };
            threshold_s.is_none_or(|threshold| marker_end_s >= threshold)
        })
}

/// Sorts and deduplicates Z candidates: identical (within `tol`)
/// intervals with the same kind, provenance, and knowledge merge.
fn merge_z_candidates(mut candidates: Vec<ZCandidate>, tol: f64) -> Vec<ZCandidate> {
    candidates.sort_by(|a, b| {
        (a.z.lo, a.z.hi, a.kind as u8, a.provenance as u8)
            .partial_cmp(&(b.z.lo, b.z.hi, b.kind as u8, b.provenance as u8))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut out: Vec<ZCandidate> = Vec::with_capacity(candidates.len());
    for c in candidates {
        let duplicate = out.last().is_some_and(|prev| {
            prev.kind == c.kind
                && prev.provenance == c.provenance
                && prev.z_known == c.z_known
                && (prev.z.lo - c.z.lo).abs() <= tol
                && (prev.z.hi - c.z.hi).abs() <= tol
        });
        if !duplicate {
            out.push(c);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use plr_gcode::StopReason;
    use plr_wal::{GcodeState as WalGcodeState, Marker, MarkerKind, WalRecord};

    use super::{
        anchor_state_from_context, compute_stop_set, coverage_certified_context,
        extruder_coverage_end, BandOutcome, Confidence, FileTail, Interval, PossibleStopSet,
        Provenance, ZKind,
    };
    use crate::config::ReconstructConfig;
    use crate::error::{ContextDefect, ReconstructError};
    use crate::testutil::{
        context_at, context_with_gcode, context_with_print_time, heartbeat_at, ingest_records,
        stepper_range, stepper_range_with_clock, trapq_segment_xyz,
    };
    use crate::window::compute_stop_window;

    const FREQ: f64 = 180_000_000.0;

    fn cfg() -> ReconstructConfig {
        ReconstructConfig {
            mcu_freq: Some(FREQ),
            ..ReconstructConfig::default()
        }
    }

    /// Timeline with heartbeat at print time `t_hb`, a Z commit out to
    /// `t_commit`, and the given extra records.
    fn base_records(t_hb: f64, t_commit: f64, extra: Vec<WalRecord>) -> Vec<WalRecord> {
        // Mono axis: 1 s of mono per 1 s of print time, pt 0 at mono 0.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let mono = |pt: f64| (pt * 1e9) as u64;
        let mut records = vec![
            WalRecord::Heartbeat(heartbeat_at(mono(t_hb), t_hb)),
            WalRecord::StepperRange(stepper_range_with_clock(
                "stepper_z",
                t_commit,
                FREQ,
                mono(t_commit),
            )),
        ];
        records.extend(extra);
        records
    }

    /// Regression for the internal-E containment hole (proptest seed
    /// 9938d965…): the anchor context's processing frontier is stalled
    /// on ONE long move, so the frontier's execution time sits seconds
    /// behind the snapshot that recorded it. Sizing the extension
    /// horizon from the snapshot's capture time spends the whole budget
    /// re-simulating already-executed motion and leaves everything the
    /// machine really did afterwards outside the reported E interval.
    #[test]
    fn extension_horizon_starts_where_the_stalled_frontier_executes() {
        // File: one 212 mm move (4.24 s at 50 mm/s) at the frontier,
        // then four short extruding moves of 10 mm (0.2 s each).
        let text = concat!(
            "G1 X200 Y200 E5 F3000
",
            "G1 X210 Y200 E6
",
            "G1 X220 Y200 E7
",
            "G1 X230 Y200 E8
",
            "G1 X240 Y200 E9
",
        );
        // Durable trapq for the motion PRECEDING the frontier: journaled
        // at mono 10 s, executing until print time 16.0. This is the
        // print time at which the frontier's first line starts.
        let preceding = trapq_segment_xyz(
            "toolhead",
            10.0,
            6.0,
            [0.0, 0.0, 0.2],
            [1.0, 0.0, 0.0],
            20.0,
            10_000_000_000,
        );
        let gcode = WalGcodeState {
            speed_factor: 1.0,
            speed: 3000.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![50.0, 50.0, 0.2, 4.0],
            gcode_position: vec![50.0, 50.0, 0.2, 4.0],
        };
        // Heartbeat/commit at 20.0/20.1; the anchor context is captured
        // at 20.0 but its frontier began executing at 16.0.
        let records = base_records(
            20.0,
            20.1,
            vec![
                WalRecord::TrapqSegment(preceding),
                WalRecord::Context(context_with_gcode(20_000_000_000, 0, gcode)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        let ext = set.extension.clone().expect("extension ran");

        // Horizon origin is the frontier's execution start (16.0), not
        // the snapshot time (20.0): 2.0 + (20.1 - 16.0) = 6.1 s.
        assert!(
            (ext.horizon - 6.1).abs() < 1e-9,
            "horizon {} should be measured from the stalled frontier",
            ext.horizon
        );
        // The old formula (2.0 + (t_b - anchor_pt) = 2.1 s) could not
        // even finish the 4.24 s move; the corrected one reaches the
        // short moves after it.
        assert!(
            ext.lines_consumed >= 3,
            "consumed only {} lines",
            ext.lines_consumed
        );
        let e = set.e_internal.expect("internal E interval");
        // E after the long move is 5; the short moves take it to 7+.
        assert!(e.contains(5.0, 1e-9), "e_internal {e:?}");
        assert!(
            e.contains(7.0, 1e-9),
            "e_internal {e:?} misses motion the machine had time to execute"
        );
        assert!(!set.degradation.extension_start_unanchored);
    }

    // --- the un-evidenced extruder band -------------------------------
    //
    // These build the real product of the system the hazard describes: a
    // retract *and* its unretract sit strictly between the newest durable
    // extruder row and the anchor frontier, so both endpoints look normal
    // and only the interior excursion is missing. Every one of them fails
    // on the pre-change code path (`BandOutcome::Uncertifiable`), which is
    // what stops them being green tautologies.

    /// File whose lines up to offset `F` contain a retract to E−4 and back.
    /// Absolute E so the values are readable; the trailing lines give the
    /// extension something to consume.
    const RETRACT_TEXT: &str = concat!(
        "G1 X60 Y50 E10 F3000\n", //  0..21  E=10
        "G1 E6\n",                // 21..28  retract to 6  <-- the excursion
        "G1 E10\n",               // 28..36  back to 10
        "G1 X70 Y50 E11\n",       // 36..52  E=11
        "G1 X80 Y50 E12\n",       // 52..68
        "G1 X90 Y50 E13\n",
    );

    /// Offset just past `G1 E10` (the unretract): the anchor frontier, so
    /// the excursion is strictly *inside* the processed-but-un-evidenced
    /// region and invisible to both trapq rows and the extension.
    const RETRACT_FRONTIER: u64 = 36;

    fn e10_gcode(e: f64) -> WalGcodeState {
        WalGcodeState {
            speed_factor: 1.0,
            speed: 3000.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![60.0, 50.0, 0.2, e],
            gcode_position: vec![60.0, 50.0, 0.2, e],
        }
    }

    /// Builds the retract scenario. `extruder_cov_end` is where durable
    /// extruder-queue coverage ends in print time; `anchor_print_time` is
    /// the anchor's journaled append frontier.
    fn retract_set(
        extruder_cov_end: f64,
        floor_print_time: Option<f64>,
        anchor_print_time: Option<f64>,
    ) -> PossibleStopSet {
        // Extruder row covering only up to `extruder_cov_end`: the
        // excursion's own rows never made it to the log.
        let e_row = trapq_segment_xyz(
            "extruder",
            8.0,
            extruder_cov_end - 8.0,
            [10.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            1.0,
            8_000_000_000,
        );
        // A floor context old enough to seed the replay, and the anchor.
        let mut floor = context_with_gcode(9_000_000_000, 0, e10_gcode(10.0));
        floor.print_time = floor_print_time;
        let mut anchor = context_with_gcode(20_000_000_000, RETRACT_FRONTIER, e10_gcode(10.0));
        anchor.print_time = anchor_print_time;
        let records = base_records(
            20.0,
            20.1,
            vec![
                WalRecord::TrapqSegment(e_row),
                WalRecord::Context(floor),
                WalRecord::Context(anchor),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: RETRACT_TEXT.as_bytes(),
        };
        compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap()
    }

    /// The hazard, and the fix. Durable extruder coverage ends at 10.0; the
    /// floor context's append frontier is 9.0, so with the 0.5 s lookahead
    /// premise `10.0 >= 9.0 + 0.5` certifies it. The band then runs from
    /// the floor frontier to the anchor frontier and the replay recovers
    /// E = 6 — a value neither the trapq evaluation nor the extension can
    /// see, because it is bracketed by E = 10 on both sides.
    #[test]
    fn band_recovers_a_retract_excursion_no_other_evidence_bounds() {
        let set = retract_set(10.0, Some(9.0), Some(19.5));
        assert_eq!(set.degradation.e_internal_band, BandOutcome::Certified);
        let e = set.e_internal.expect("internal E interval");
        assert!(
            e.contains(6.0, 1e-9),
            "e_internal {e:?} must contain the retract low point E=6"
        );
    }

    /// The guard is not vacuous: the *same* scenario on a pre-change WAL
    /// (no context carries `toolhead.print_time`) leaves the excursion
    /// unbounded. This is the containment hole as it exists today, pinned
    /// so that the fix above is demonstrably doing the work.
    #[test]
    fn without_print_time_the_retract_excursion_is_unbounded() {
        let set = retract_set(10.0, None, None);
        assert_eq!(set.degradation.e_internal_band, BandOutcome::Uncertifiable);
        let e = set.e_internal.expect("internal E interval");
        assert!(
            !e.contains(6.0, 1e-9),
            "pre-change path unexpectedly bounded E=6 ({e:?}); if this now \
             holds, the band test above proves nothing"
        );
    }

    /// Coverage that never reaches any context's append frontier plus the
    /// premise yields `Uncertified` — and, by the measured decision in
    /// [`band_e_internal`], no widening.
    #[test]
    fn coverage_behind_every_append_frontier_is_uncertified() {
        // cov_end 10.0 but the floor claims an append frontier of 9.8, so
        // 10.0 >= 9.8 + 0.5 is false; likewise for the anchor.
        let set = retract_set(10.0, Some(9.8), Some(19.5));
        assert_eq!(set.degradation.e_internal_band, BandOutcome::Uncertified);
    }

    /// The certificate must reject a non-finite premise and a non-finite
    /// `print_time` rather than certifying on them — `NaN >= x` is false,
    /// but relying on that silently is how a guard rots.
    #[test]
    fn non_finite_inputs_never_certify() {
        let contexts = vec![context_with_print_time(1_000_000_000, 10, 5.0)];
        let sane = cfg();
        assert!(coverage_certified_context(&contexts, Some(f64::NAN), &sane).is_none());
        // Infinity would certify *every* context — the unsafe direction —
        // so it is rejected rather than trusted. `extruder_coverage_end`
        // already filters non-finite row ends, so this is defence in depth.
        assert!(coverage_certified_context(&contexts, Some(f64::INFINITY), &sane).is_none());
        assert!(coverage_certified_context(&contexts, None, &sane).is_none());
        let bad_premise = ReconstructConfig {
            max_lookahead_lead: f64::NAN,
            ..cfg()
        };
        assert!(coverage_certified_context(&contexts, Some(100.0), &bad_premise).is_none());
        let negative = ReconstructConfig {
            max_lookahead_lead: -1.0,
            ..cfg()
        };
        assert!(coverage_certified_context(&contexts, Some(100.0), &negative).is_none());
        // A context whose own print_time is non-finite must not certify.
        let nan_ctx = vec![context_with_print_time(1_000_000_000, 10, f64::NAN)];
        assert!(coverage_certified_context(&nan_ctx, Some(100.0), &sane).is_none());
    }

    /// `extruder_coverage_end` must ignore the toolhead queue. A pure
    /// retract produces an extruder row and **no** toolhead row
    /// (`Move.is_kinematic_move` is false for extrude-only moves), so
    /// letting toolhead rows raise the coverage end would certify
    /// extrusion that was never journaled.
    #[test]
    fn coverage_end_is_per_queue_and_ignores_the_toolhead() {
        let th = trapq_segment_xyz(
            "toolhead",
            10.0,
            5.0, // ends at 15.0, far ahead
            [0.0, 0.0, 0.2],
            [1.0, 0.0, 0.0],
            20.0,
            10_000_000_000,
        );
        let ex = trapq_segment_xyz(
            "extruder",
            8.0,
            1.0, // ends at 9.0
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            1.0,
            8_000_000_000,
        );
        let timeline = ingest_records(base_records(
            20.0,
            20.1,
            vec![
                WalRecord::TrapqSegment(th),
                WalRecord::TrapqSegment(ex),
                WalRecord::Context(context_with_print_time(20_000_000_000, 0, 19.0)),
            ],
        ));
        let cov = extruder_coverage_end(&timeline.extruder_segments);
        assert_eq!(cov, Some(9.0), "toolhead rows must not raise coverage");
    }

    /// A long file of 10 mm extruding moves (0.2 s each at F3000), so a
    /// horizon difference of seconds is observable as a line count.
    fn long_tail_text() -> String {
        use std::fmt::Write as _;
        let mut text = String::from("G1 X60 Y50 E1 F3000\n");
        for i in 0..400 {
            let x = 60 + i % 40;
            let e = 2 + i;
            let _ = writeln!(text, "G1 X{x} Y50 E{e}");
        }
        text
    }

    /// With no durable trapq row before the anchor there is no motion
    /// evidence for the frontier's execution start, so the origin falls
    /// back to the reader-lead bound `t_a - max_processing_lead`. The
    /// assertion is on the resulting **horizon**: a test that only
    /// checked the flag would still pass with the origin left at the
    /// capture time, which is exactly the hole this branch had.
    #[test]
    fn unanchored_extension_start_uses_the_reader_lead_bound() {
        let records = base_records(
            20.0,
            20.1,
            vec![WalRecord::Context(context_at(20_000_000_000, 0))],
        );
        let timeline = ingest_records(records);
        assert!(
            timeline
                .trapq_end_time_journaled_by(20_000_000_000)
                .is_none(),
            "fixture must carry no trapq evidence before the anchor"
        );
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let text = long_tail_text();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let config = cfg();
        let set = compute_stop_set(&timeline, &window, Some(&tail), &config).unwrap();
        let ext = set.extension.clone().expect("extension ran");
        // t_ext_start = t_a - max_processing_lead = 20.0 - 3.0 = 17.0;
        // horizon = 2.0 + (20.1 - 17.0) = 5.1 s.
        let expected =
            config.extension_horizon + (window.t_b - (window.t_a - config.max_processing_lead));
        assert!(
            (ext.horizon - expected).abs() < 1e-9 && (ext.horizon - 5.1).abs() < 1e-9,
            "horizon {} is not the reader-lead bound {expected}",
            ext.horizon
        );
        // Anchoring on the capture time would give 2.1 s: less than half.
        let capture_time_horizon = config.extension_horizon + (window.t_b - window.t_a);
        assert!(ext.horizon > capture_time_horizon * 2.0);
        assert!(set.degradation.extension_start_unanchored);
        // The flag is information, not degradation: coverage is intact,
        // so confidence stays per-line and a resume is not refused.
        assert_eq!(set.degradation.confidence, Confidence::PerLine);
    }

    /// `t_a` seeds the origin fold, so the origin can never be pushed
    /// past the instant the machine was last *proven* to be executing.
    /// That cap is the envelope against a klippy restart resetting the
    /// print-time axis while host `mono_ns` keeps advancing, which would
    /// otherwise place `anchor_pt` far in the future and shrink the
    /// horizon below the stop window.
    #[test]
    fn an_anchor_dated_after_t_a_cannot_shrink_the_horizon() {
        // Both non-`t_a` bounds sit *after* `t_a` here, so the cap is the
        // binding term: durable planned motion reaching print time 24.0,
        // and a context whose `mono_ns` maps to print time 40.0 (the
        // klippy-restart shape — the print-time axis reset while host
        // monotonic time kept advancing).
        let planned = trapq_segment_xyz(
            "toolhead",
            21.0,
            3.0,
            [0.0, 0.0, 0.2],
            [1.0, 0.0, 0.0],
            20.0,
            19_500_000_000,
        );
        let records = base_records(
            20.0,
            20.1,
            vec![
                WalRecord::TrapqSegment(planned),
                WalRecord::Context(context_at(40_000_000_000, 0)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert!((window.t_a - 20.0).abs() < 1e-9);
        let text = long_tail_text();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let config = cfg();
        let set = compute_stop_set(&timeline, &window, Some(&tail), &config).unwrap();
        assert!((set.wal_eval_end - 24.0).abs() < 1e-9);
        let ext = set.extension.expect("extension ran");
        // The context dates itself 20 s after t_a, and motion evidence
        // reaches 24.0 — both later than t_a...
        assert_eq!(ext.anchor_print_time, Some(40.0));
        assert!(timeline
            .trapq_end_time_journaled_by(40_000_000_000)
            .is_some_and(|end| end > window.t_a));
        // ...so without the `t_a` seed the origin would be 24.0 and the
        // horizon would collapse to `extension_horizon` (2.0 s), losing
        // the whole stop window. Capped, it is 2.0 + (24.0 - 20.0).
        assert!((ext.horizon - 6.0).abs() < 1e-9, "horizon {}", ext.horizon);
        assert!(
            ext.horizon > config.extension_horizon + (window.t_b - window.t_a),
            "horizon {} fell back to the uncapped origin",
            ext.horizon
        );
        assert!(!set.degradation.extension_start_unanchored);
    }

    /// The horizon runs out to `wal_eval_end` = `max(t_b, durable trapq
    /// end)`, not to `t_b`. Trapq rows are journaled as Klipper *plans*
    /// moves, so durable knowledge routinely extends past the committed
    /// boundary; ending the horizon at `t_b` under-simulates by exactly
    /// that difference.
    #[test]
    fn horizon_end_follows_trapq_knowledge_past_the_commit_boundary() {
        // Planned motion journaled before the anchor but executing out to
        // print time 24.0, while Z commits only to 20.1.
        let planned = trapq_segment_xyz(
            "toolhead",
            19.0,
            5.0,
            [0.0, 0.0, 0.2],
            [1.0, 0.0, 0.0],
            20.0,
            19_500_000_000,
        );
        let records = base_records(
            20.0,
            20.1,
            vec![
                WalRecord::TrapqSegment(planned),
                WalRecord::Context(context_at(20_000_000_000, 0)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert!((window.t_b - 20.1).abs() < 1e-6, "t_b {}", window.t_b);
        let text = long_tail_text();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let config = cfg();
        let set = compute_stop_set(&timeline, &window, Some(&tail), &config).unwrap();
        // Durable trapq knowledge reaches 24.0, ~4 s beyond t_b.
        assert!((set.wal_eval_end - 24.0).abs() < 1e-9);
        assert!(set.wal_eval_end - window.t_b > 3.0);
        let ext = set.extension.expect("extension ran");
        // Origin is capped at t_a (20.0), end is wal_eval_end (24.0):
        // horizon = 2.0 + 4.0 = 6.0 s. Ending at t_b would give 2.1 s.
        assert!((ext.horizon - 6.0).abs() < 1e-9, "horizon {}", ext.horizon);
        let t_b_horizon = config.extension_horizon + (window.t_b - window.t_a);
        assert!(ext.horizon > t_b_horizon * 2.0);
        // And the extra seconds are real coverage: ~0.2 s per 10 mm line,
        // so 6.0 s reaches far more lines than 2.1 s could.
        assert!(
            ext.lines_consumed >= 25,
            "consumed only {} lines",
            ext.lines_consumed
        );
        let e = set.e_internal.expect("internal E");
        assert!(e.hi > 20.0, "e_internal {e:?} stops short of the horizon");
    }

    /// When the print-time axis overflows, the horizon fallback must be
    /// the conservative branch. Every value here is finite — so ingest
    /// accepts all of it — but `wal_eval_end - start_pt` is
    /// `1e308 - (-1e308)`, which overflows to infinity.
    ///
    /// Collapsing to `extension_horizon` alone in that case would hand
    /// back the *narrowest* horizon, i.e. a confident narrow answer
    /// resting on arithmetic that just failed. The honest outcome is an
    /// unbounded horizon plus `extension_truncated`, so confidence drops
    /// out of `PerLine` and automation refuses. Restoring the old
    /// `else { 0.0 }` fallback fails this test rather than only
    /// contradicting a comment.
    #[test]
    fn an_overflowing_time_axis_refuses_instead_of_narrowing() {
        let huge = 1.0e308_f64;
        // t_a = -1e308 (heartbeat and its correlation sample), committed
        // motion reported at +1e308, and no trapq row — so the origin is
        // the degenerate bound at t_a - lead and stays hugely negative.
        let records = vec![
            WalRecord::Heartbeat(heartbeat_at(0, -huge)),
            WalRecord::StepperRange(stepper_range("stepper_z", huge, 1_000)),
            WalRecord::Context(context_at(2_000, 0)),
        ];
        let timeline = ingest_records(records);
        // Precondition: the hostile values survived ingest intact.
        assert_eq!(timeline.stepper_ranges.len(), 1);
        let config = ReconstructConfig {
            mcu_freq: None,
            ..ReconstructConfig::default()
        };
        let window = compute_stop_window(&timeline, None, &config).unwrap();
        assert!((window.t_a - -huge).abs() < 1.0);
        assert!((window.t_b - huge).abs() < 1.0);
        let text = long_tail_text();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &config).unwrap();
        let ext = set.extension.expect("extension ran");
        // Unbounded, not `extension_horizon`: the axis was unusable, so
        // coverage is maximal rather than minimal.
        assert!(
            ext.horizon.is_infinite(),
            "horizon {} collapsed to the narrow fallback",
            ext.horizon
        );
        assert!(ext.horizon > config.extension_horizon);
        // And the answer is labelled as a refusal, whatever the
        // simulation's own stop reason turned out to be.
        assert!(set.degradation.extension_truncated);
        assert_ne!(
            set.degradation.confidence,
            Confidence::PerLine,
            "an unusable time axis must not yield per-line confidence"
        );
    }

    #[test]
    fn no_context_is_a_typed_error() {
        let timeline = ingest_records(base_records(10.0, 10.5, vec![]));
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        assert_eq!(
            compute_stop_set(&timeline, &window, None, &cfg()),
            Err(ReconstructError::NoContext)
        );
    }

    #[test]
    fn wal_only_z_and_xy_from_segments() {
        // Constant-Z travel at z=0.4 over [10, 11], then a hop ramp
        // 0.4 -> 0.8 over [11, 11.1].
        let records = base_records(
            10.5,
            11.05,
            vec![
                WalRecord::TrapqSegment(trapq_segment_xyz(
                    "toolhead",
                    10.0,
                    1.0,
                    [10.0, 20.0, 0.4],
                    [1.0, 0.0, 0.0],
                    30.0,
                    1_000,
                )),
                WalRecord::TrapqSegment(trapq_segment_xyz(
                    "toolhead",
                    11.0,
                    0.1,
                    [40.0, 20.0, 0.4],
                    [0.0, 0.0, 1.0],
                    4.0,
                    1_001,
                )),
                WalRecord::Context(context_at(10_600_000_000, 0)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let set = compute_stop_set(&timeline, &window, None, &cfg()).unwrap();

        // Window [10.5, max(t_b=11.05, trapq end 11.1)].
        assert!((set.wal_eval_end - 11.1).abs() < 1e-9);
        // Plateau at 0.4 and a ramp 0.4 -> 0.8 (clamped at eval end).
        assert!(set.contains_z(0.4, 1e-9));
        assert!(set.contains_z(0.6, 1e-9), "inside the hop ramp");
        let span = set.z_span().unwrap();
        assert!((span.lo - 0.4).abs() < 1e-9);
        assert!((span.hi - 0.8).abs() < 1e-9);
        // XY: x spans [25 (pos at t_a=10.5), 40], y constant 20.
        let xy = set.xy.unwrap();
        assert!(xy.contains(25.0, 20.0, 1e-9));
        assert!(xy.contains(40.0, 20.0, 1e-9));
        assert!(!xy.contains(10.0, 20.0, 1e-6), "pre-t_a positions excluded");
        // Extension could not run: flagged, confidence degraded.
        assert!(set.degradation.extension_unavailable);
        assert_eq!(set.degradation.confidence, Confidence::PerLayer);
        assert!(set.extension.is_none());
    }

    #[test]
    fn dwell_at_window_start_holds_last_position() {
        // Motion ends at 9.0; heartbeat at 10.5 (dwell); nothing after.
        let records = base_records(
            10.5,
            8.9,
            vec![
                WalRecord::TrapqSegment(trapq_segment_xyz(
                    "toolhead",
                    8.0,
                    1.0,
                    [10.0, 20.0, 0.4],
                    [1.0, 0.0, 0.0],
                    30.0,
                    1_000,
                )),
                WalRecord::TrapqSegment(trapq_segment_xyz(
                    "extruder",
                    8.0,
                    1.0,
                    [100.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    2.0,
                    1_001,
                )),
                WalRecord::Context(context_at(10_400_000_000, 0)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let set = compute_stop_set(&timeline, &window, None, &cfg()).unwrap();
        // Held position: x = 10 + 30*1 = 40, z = 0.4, e = 102.
        let xy = set.xy.unwrap();
        assert!(xy.contains(40.0, 20.0, 1e-9));
        assert!((xy.x.width()) < 1e-9, "dwell: a single point");
        assert!(set.contains_z(0.4, 1e-9));
        let e = set.e_internal.unwrap();
        assert!(e.contains(102.0, 1e-9));
        assert!(e.width() < 1e-9);
    }

    #[test]
    fn interior_velocity_reversal_is_sampled() {
        // A segment that decelerates through zero and reverses inside
        // the window: v0=10, a=-10 over [10,12]; x(t) peaks at t*=11
        // with dist 5. Start x=0 -> max x=5, end x=0.
        let mut seg = trapq_segment_xyz(
            "toolhead",
            10.0,
            2.0,
            [0.0, 0.0, 0.2],
            [1.0, 0.0, 0.0],
            10.0,
            1_000,
        );
        seg.acceleration = -10.0;
        let records = base_records(
            10.0,
            12.0,
            vec![
                WalRecord::TrapqSegment(seg),
                WalRecord::Context(context_at(10_000_000_000, 0)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let set = compute_stop_set(&timeline, &window, None, &cfg()).unwrap();
        let xy = set.xy.unwrap();
        assert!(xy.contains(5.0, 0.0, 1e-9), "interior extremum included");
    }

    #[test]
    fn extension_enumerates_z_exactly_and_bounds_offsets() {
        let gcode_text = "G1 X60 Y50 E101 F3000\nG1 E100.2 F3000\nG1 Z0.6 F3000\nG1 X80 Y50\nG1 Z0.2\nG1 E101.0\nG1 X90 Y50 E102\n";
        // Anchor context: at (50, 50, 0.2), E internal 100, absolute
        // modes, file offset 0.
        let gcode = WalGcodeState {
            speed_factor: 1.0,
            speed: 3000.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![50.0, 50.0, 0.2, 100.0],
            gcode_position: vec![50.0, 50.0, 0.2, 100.0],
        };
        let records = base_records(
            10.0,
            10.1,
            vec![
                WalRecord::TrapqSegment(trapq_segment_xyz(
                    "toolhead",
                    9.5,
                    0.5,
                    [40.0, 50.0, 0.2],
                    [1.0, 0.0, 0.0],
                    20.0,
                    9_500_000_000,
                )),
                WalRecord::Context(context_with_gcode(10_000_000_000, 0, gcode)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: gcode_text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();

        // Z candidates: WAL plateau 0.2, extension plateau 0.2, hop
        // ramp 0.2->0.6, plateau 0.6, ramp 0.6->0.2 (merged plateaus).
        assert!(set.contains_z(0.2, 1e-9));
        assert!(set.contains_z(0.6, 1e-9));
        assert!(set.contains_z(0.4, 1e-9), "mid-hop ramp covered");
        let span = set.z_span().unwrap();
        assert!((span.lo - 0.2).abs() < 1e-9);
        assert!((span.hi - 0.6).abs() < 1e-9);
        let ramps = set
            .z_candidates
            .iter()
            .filter(|c| c.kind == ZKind::Ramp)
            .count();
        // Hop up (0.2 -> 0.6) and hop down (0.6 -> 0.2) are the same
        // interval and merge into one ramp candidate.
        assert_eq!(ramps, 1, "identical up/down ramps merge");
        assert!(set
            .z_candidates
            .iter()
            .any(|c| c.kind == ZKind::Plateau && c.z.contains(0.6, 1e-9)));
        assert!(set
            .z_candidates
            .iter()
            .any(|c| c.provenance == Provenance::Extension));

        // Offsets: whole extension consumed; window ends at EOF.
        let fw = set.file_window.unwrap();
        assert_eq!(fw.end, gcode_text.len() as u64);
        assert!(fw.start <= fw.end);
        // Only the anchor context exists and it is not older than
        // t_a - lead: floor is honest about that.
        assert!(set.degradation.offset_floor_uncertain);

        // E: internal from extension spans [100, 102] and dips to the
        // retract 100.2... the retract targets file E 100.2 = internal
        // 100.2 here (factor 1, base 0).
        let e = set.e_internal.unwrap();
        assert!(e.contains(100.0, 1e-9));
        assert!(e.contains(102.0, 1e-9));
        let ef = set.e_file.unwrap();
        assert!(ef.contains(100.0, 1e-9));
        assert!(ef.contains(102.0, 1e-9));
        assert!(!set.degradation.e_frame_shift_in_extension);
        assert_eq!(set.degradation.confidence, Confidence::PerLine);
        let ext = set.extension.unwrap();
        assert_eq!(ext.anchor_offset, 0);
        assert_eq!(ext.stop, StopReason::EndOfInput);
        assert_eq!(ext.lines_consumed, 7);
    }

    #[test]
    fn e_frame_shift_in_extension_is_flagged_and_file_e_tracks_it() {
        // G92 E0 mid-extension: internal E keeps accumulating, file E
        // rebases to 0.
        let gcode_text = "G1 X60 Y50 E101 F3000\nG92 E0\nG1 X70 Y50 E1\n";
        let gcode = WalGcodeState {
            speed_factor: 1.0,
            speed: 3000.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![50.0, 50.0, 0.2, 100.0],
            gcode_position: vec![50.0, 50.0, 0.2, 100.0],
        };
        let records = base_records(
            10.0,
            10.1,
            vec![WalRecord::Context(context_with_gcode(
                10_000_000_000,
                0,
                gcode,
            ))],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: gcode_text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        assert!(set.degradation.e_frame_shift_in_extension);
        let ef = set.e_file.unwrap();
        // Pre-G92 file E 100..101 and post-G92 0..1 all covered.
        assert!(ef.contains(100.5, 1e-9));
        assert!(ef.contains(0.5, 1e-9));
        let e = set.e_internal.unwrap();
        assert!(e.contains(102.0, 1e-9), "internal E accumulates: 100+1+1");
    }

    #[test]
    fn wal_e_converts_through_recent_context_frames() {
        // Extruder motion 100 -> 102 internal; context frame has
        // base_e = 90 and factor 0.5: file E = (E - 90) / 0.5.
        let gcode = WalGcodeState {
            speed_factor: 1.0,
            speed: 3000.0,
            extrude_factor: 0.5,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![50.0, 50.0, 0.2, 100.0],
            gcode_position: vec![50.0, 50.0, 0.2, 20.0],
        };
        let records = base_records(
            10.5,
            11.0,
            vec![
                WalRecord::TrapqSegment(trapq_segment_xyz(
                    "extruder",
                    10.0,
                    1.0,
                    [100.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    2.0,
                    1_000,
                )),
                WalRecord::Context(context_with_gcode(10_000_000_000, 0, gcode)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let set = compute_stop_set(&timeline, &window, None, &cfg()).unwrap();
        let ef = set.e_file.unwrap();
        // internal [101 (t_a=10.5), 102] -> file [(101-90)/0.5, (102-90)/0.5] = [22, 24].
        assert!((ef.lo - 22.0).abs() < 1e-9, "ef.lo = {}", ef.lo);
        assert!((ef.hi - 24.0).abs() < 1e-9);
        // No file tail: the exact replay could not run, and the
        // snapshot-frame fallback is honestly flagged.
        assert!(set.degradation.e_file_frames_incomplete);
    }

    #[test]
    fn file_e_recovers_frames_never_snapshotted() {
        // Regression for the fault-injection counterexample (seed
        // ae60143c...): a `G92 E0` + retract processed in a burst
        // between two context flushes. The retract executes in an
        // E-frame that exists in NO context snapshot — the old context
        // predates the first G92, the anchor's frontier is already past
        // a second G92. Only the file replay from the floor context can
        // reconstruct the intermediate frame's (negative) file E.
        let text =
            "G1 X60 Y50 E101 F3000\nG92 E0\nG1 E-0.8\nG1 X70 Y50 E-0.2\nG92 E0\nG1 X80 Y50 E0.5\n";
        let end = text.len() as u64;
        let old_gcode = WalGcodeState {
            speed_factor: 1.0,
            speed: 3000.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![50.0, 50.0, 0.2, 100.0],
            gcode_position: vec![50.0, 50.0, 0.2, 100.0],
        };
        // Anchor = state after replaying all six lines from old_gcode:
        // internal E 101.3, base_e 100.8 (second G92), gcode E 0.5.
        let anchor_gcode = WalGcodeState {
            speed_factor: 1.0,
            speed: 3000.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![80.0, 50.0, 0.2, 101.3],
            gcode_position: vec![80.0, 50.0, 0.2, 0.5],
        };
        // Heartbeat pt 10 => t_a = 10; old context at pt 5 predates
        // t_a - max_processing_lead (7): a certain floor.
        let records = base_records(
            10.0,
            10.1,
            vec![
                WalRecord::Context(context_with_gcode(5_000_000_000, 0, old_gcode)),
                WalRecord::Context(context_with_gcode(10_000_000_000, end, anchor_gcode)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        let ef = set.e_file.unwrap();
        // The full retract depth in the never-snapshotted frame.
        assert!(ef.contains(-0.8, 1e-9), "e_file = {ef:?}");
        // And the pre-G92 frame values.
        assert!(ef.contains(101.0, 1e-9), "e_file = {ef:?}");
        // The replay reached the window end from a certain floor: the
        // result is exact, not best-effort.
        assert!(!set.degradation.e_file_frames_incomplete);
        assert!(!set.degradation.offset_floor_uncertain);
        let fw = set.file_window.unwrap();
        assert_eq!((fw.start, fw.end), (0, end));
    }

    #[test]
    fn observation_gap_degrades_confidence() {
        let records = base_records(
            10.0,
            10.1,
            vec![
                WalRecord::Marker(Marker {
                    mono_ns: 9_800_000_000,
                    kind: MarkerKind::SubscriptionGap {
                        start_mono_ns: 9_500_000_000,
                        end_mono_ns: 9_800_000_000,
                    },
                }),
                WalRecord::Context(context_at(10_000_000_000, 0)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let set = compute_stop_set(&timeline, &window, None, &cfg()).unwrap();
        assert!(set.degradation.observation_gap);
        assert_eq!(set.degradation.confidence, Confidence::PerLayer);
    }

    #[test]
    fn old_gap_markers_do_not_degrade() {
        // Gap ended 100 s before the window: irrelevant.
        let records = base_records(
            200.0,
            200.1,
            vec![
                WalRecord::Marker(Marker {
                    mono_ns: 100_000_000_000,
                    kind: MarkerKind::SubscriptionGap {
                        start_mono_ns: 99_000_000_000,
                        end_mono_ns: 100_000_000_000,
                    },
                }),
                WalRecord::Context(context_at(200_000_000_000, 0)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let set = compute_stop_set(&timeline, &window, None, &cfg()).unwrap();
        assert!(!set.degradation.observation_gap);
    }

    #[test]
    fn file_tail_must_cover_anchor_offset() {
        let records = base_records(
            10.0,
            10.1,
            vec![WalRecord::Context(context_at(10_000_000_000, 500))],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 600,
            bytes: b"G1 X1\n",
        };
        assert!(matches!(
            compute_stop_set(&timeline, &window, Some(&tail), &cfg()),
            Err(ReconstructError::FileTailMismatch {
                base_offset: 600,
                file_position: 500,
                ..
            })
        ));
    }

    #[test]
    fn offset_floor_uses_old_enough_context() {
        // Contexts at pt 5.0 (offset 100) and 9.9 (offset 400); t_a=10,
        // lead 3.0 -> threshold 7.0: floor = 100, not 400.
        let records = base_records(
            10.0,
            10.1,
            vec![
                WalRecord::Context(context_at(5_000_000_000, 100)),
                WalRecord::Context(context_at(9_900_000_000, 400)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let set = compute_stop_set(&timeline, &window, None, &cfg()).unwrap();
        // Extension unavailable: end falls back to the anchor offset.
        let fw = set.file_window.unwrap();
        assert_eq!(fw.start, 100);
        assert_eq!(fw.end, 400);
        assert!(!set.degradation.offset_floor_uncertain);
        assert!(fw.contains(250));
        assert!(!fw.contains(401));
    }

    #[test]
    #[allow(clippy::float_cmp)] // fields are copied verbatim; exact equality is the claim
    fn anchor_conversion_reproduces_klipper_arithmetic() {
        // Klipper status of a state with speed_factor 50% (0.5 mult),
        // internal speed 25 mm/s, extrude factor 0.95, G92-shifted E.
        let gcode = WalGcodeState {
            speed_factor: 0.5,
            speed: 3000.0, // status speed = internal / (0.5/60) = 3000
            extrude_factor: 0.95,
            absolute_coordinates: true,
            absolute_extrude: false,
            homing_origin: vec![0.0, 0.0, -0.12, 0.0],
            position: vec![10.0, 20.0, 0.4, 512.7],
            gcode_position: vec![10.0, 20.0, 0.52, 100.0],
        };
        let state = anchor_state_from_context(&gcode).unwrap();
        assert!((state.speed - 25.0).abs() < 1e-12);
        assert!((state.speed_factor - 0.5 / 60.0).abs() < 1e-15);
        assert!((state.extrude_factor - 0.95).abs() < 1e-15);
        assert!(state.absolute_coord);
        assert!(!state.absolute_extrude);
        assert_eq!(state.last_position, [10.0, 20.0, 0.4, 512.7]);
        // base = pos - gpos (E scaled): base_z = 0.4 - 0.52; base_e =
        // 512.7 - 100*0.95.
        assert!((state.base_position[2] - -0.12).abs() < 1e-12);
        assert!((state.base_position[3] - (512.7 - 95.0)).abs() < 1e-12);
        assert_eq!(state.homing_position, [0.0, 0.0, -0.12, 0.0]);
        // Round-trip: the converted state reports the same g-code
        // position Klipper did.
        let gpos = state.gcode_position();
        assert!((gpos[2] - 0.52).abs() < 1e-12);
        assert!((gpos[3] - 100.0).abs() < 1e-9);
        assert_eq!(state.position_known, [true; 4]);
    }

    #[test]
    fn anchor_conversion_rejects_defects() {
        let good = || WalGcodeState {
            speed_factor: 1.0,
            speed: 1500.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![0.0; 4],
            gcode_position: vec![0.0; 4],
        };
        let mut short = good();
        short.position = vec![0.0; 3];
        assert_eq!(
            anchor_state_from_context(&short),
            Err(ReconstructError::MalformedContext {
                defect: ContextDefect::TooFewAxes
            })
        );
        let mut nan = good();
        nan.speed = f64::NAN;
        assert_eq!(
            anchor_state_from_context(&nan),
            Err(ReconstructError::MalformedContext {
                defect: ContextDefect::NonFinite
            })
        );
        let mut zero_extrude = good();
        zero_extrude.extrude_factor = 0.0;
        assert_eq!(
            anchor_state_from_context(&zero_extrude),
            Err(ReconstructError::MalformedContext {
                defect: ContextDefect::BadExtrudeFactor
            })
        );
        let mut negative_factor = good();
        negative_factor.speed_factor = -1.0;
        assert_eq!(
            anchor_state_from_context(&negative_factor),
            Err(ReconstructError::MalformedContext {
                defect: ContextDefect::BadSpeedFactor
            })
        );
        let mut zero_speed = good();
        zero_speed.speed = 0.0;
        assert_eq!(
            anchor_state_from_context(&zero_speed),
            Err(ReconstructError::MalformedContext {
                defect: ContextDefect::NonPositiveSpeed
            })
        );
    }

    #[test]
    fn interval_and_region_math() {
        let mut iv = Interval::point(1.0);
        iv.expand(3.0);
        iv.expand(0.5);
        assert_eq!(iv, Interval { lo: 0.5, hi: 3.0 });
        assert!((iv.width() - 2.5).abs() < 1e-12);
        assert!(iv.contains(0.5, 0.0));
        assert!(iv.contains(3.000_001, 1e-5));
        assert!(!iv.contains(3.1, 1e-5));
        let pair = Interval::from_pair(2.0, -1.0);
        assert_eq!(pair, Interval { lo: -1.0, hi: 2.0 });
        assert_eq!(pair.union(iv), Interval { lo: -1.0, hi: 3.0 });
    }

    #[test]
    fn z_candidates_are_merged_and_sorted() {
        // Two identical WAL plateaus (same layer from many segments)
        // merge; extension copy of the same Z stays (different
        // provenance).
        let records = base_records(
            10.2,
            11.0,
            vec![
                WalRecord::TrapqSegment(trapq_segment_xyz(
                    "toolhead",
                    10.0,
                    0.4,
                    [0.0, 0.0, 0.2],
                    [1.0, 0.0, 0.0],
                    10.0,
                    1_000,
                )),
                WalRecord::TrapqSegment(trapq_segment_xyz(
                    "toolhead",
                    10.4,
                    0.4,
                    [4.0, 0.0, 0.2],
                    [0.0, 1.0, 0.0],
                    10.0,
                    1_001,
                )),
                WalRecord::Context(context_at(10_500_000_000, 0)),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let set = compute_stop_set(&timeline, &window, None, &cfg()).unwrap();
        let wal_plateaus = set
            .z_candidates
            .iter()
            .filter(|c| c.kind == ZKind::Plateau && c.provenance == Provenance::Wal)
            .count();
        assert_eq!(wal_plateaus, 1, "identical plateaus merged");
        assert!(set.z_candidates.windows(2).all(|w| w[0].z.lo <= w[1].z.lo));
    }
}
