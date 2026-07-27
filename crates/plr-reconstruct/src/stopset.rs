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
//! ## Still open, and the widening approach is measured out
//!
//! [`replay_file_e`] already computes exact cumulative E after **every**
//! line from the floor forward, so the excursion is in reach; what was
//! missing was a *safe* band start. [`plr_wal::Context::print_time`] — the
//! trapq append frontier, journaled since this was investigated — supplies
//! one: paired with `file_position` in a single Klipper status pass, it
//! certifies that every move from lines at or before some earlier
//! frontier is durable. **No reader consumes it yet**, deliberately.
//!
//! Every variant that closes the hole by *widening* `e_internal` has been
//! built and measured, and none is affordable:
//!
//! * **Whole loose-floored window.** Broke 18 daemon end-to-end tests with
//!   "below layer granularity" — manual fallback or a wrong line on every
//!   real recovery. Reverted before this module was written.
//! * **Floor-wide band under an uncertified certificate.** 12 candidate
//!   lines across layers `[0, 1]`, `MatchError::Inconclusive`.
//! * **The coverage-certified band** — the narrow one, bounded by the
//!   sub-second coverage lag rather than by `max_processing_lead`. Still
//!   10 candidate lines, still `Inconclusive`.
//!
//! The reason none of them fit is not that the band is wide; it is that
//! there is **no room at all**. `plrd`'s end-to-end fixture already
//! produces exactly 8 candidates against `MatchConfig::ambiguity_limit`
//! of 8, so *any* widening anywhere tips it. That is a property of the
//! harness, not of this crate, and it is recorded at the fixture.
//!
//! ## And it is not just that fixture — measured at realistic density
//!
//! The obvious objection is that `plrd`'s fixture carries 0.65 mm of E per
//! line against the 0.028–0.078 mm real slicers emit, so its zero margin
//! is an artifact. That was the working hypothesis, and it is **wrong**.
//! `plr-analyzer`'s `e_widening_cost_at_realistic_density` measures the
//! same ladder over `fixtures/real/realistic_orca.gcode`, and the answer
//! depends entirely on how loose the XY region is:
//!
//! | XY half-width | mean candidates, E pad 0 → 8 mm | refusals |
//! | --- | --- | --- |
//! | 0.05 mm (unrealistic) | 2.04 → 2.78 | none |
//! | 5 mm | 11.2 → 31.2 | 2 of 23 |
//! | 15–30 mm (**production**) | already beyond the limit at pad 0 | — |
//!
//! Production is the bottom row: `plrd`'s own pipeline reports XY regions
//! **30–60 mm wide**, because `xy` is a bounding region over everything
//! the WAL span and the extension touched. In that regime the E constraint
//! removes **63–81 %** of candidates and matters in *every* sampled stop
//! point, so it is not the marginal filter a tight-box measurement makes
//! it look like — and widening it therefore costs real refusals.
//!
//! Two consequences worth carrying: the band is unaffordable for a reason
//! that is not fixture-specific, and per-line granularity on a real part
//! may be rarer than this crate's vocabulary suggests, since candidate
//! counts at production XY widths exceed `ambiguity_limit` even with
//! perfect E evidence.
//!
//! So the route chosen is the one that needs **no** widening: feed the
//! matcher the `e_file` interval, which is already exact on the replay
//! path and already computed here, instead of discarding it. See below.
//!
//! `e_file` was never affected: on the exact path (`reached_end` and a
//! certain floor) the replay covers those lines, so the interval *does*
//! contain the truth. But `plr-analyzer`'s `StopEvidence` (fed by `plrd`'s
//! `stop_evidence`) carries only `e_internal` and **discards `e_file`**,
//! which is what makes the hole reachable in production. Two independent
//! investigations reached that same discarded interval — this one from the
//! recorder side, and an earlier review probing a different bug, which
//! found a case where `e_internal` missed the truth and `e_file` contained
//! it on exactly the `reached_end` + certain-floor path.
//!
//! **Until that lands the limit is open**, and it is pinned as an
//! executable assertion rather than prose:
//! `e_internal_does_not_bound_a_retract_inside_the_un_evidenced_band` in
//! this module's tests asserts the bug, and says in its own docs that
//! closing it means inverting the assertion, not deleting the test.
//!
//! # File-offset window
//!
//! Context snapshots record the g-code **processing frontier**, which
//! leads execution by up to `max_processing_lead` (Klipper's lookahead
//! buffering plus step-generation lead). The candidate window is
//! therefore `[frontier recorded at or before t_a - max_processing_lead,
//! min(extension resume offset, frontier cap)]`: the low end is the newest
//! frontier old enough that execution had certainly passed it by `t_a`;
//! the high end is where the simulated extension stopped consuming, capped
//! by the frontier bound below.
//!
//! # Frontier cap — the first deliberate narrowing of the high end
//!
//! The extension's `resume_offset` alone sets the high end to
//! ~committed-motion + `extension_horizon` of simulated path, sized from
//! motion *planning* ([`run_extension`]). That over-covers when the
//! journaled trapq was planned far ahead of what the machine actually
//! executed before the cut. A second, independent upper bound comes from
//! the parser-leads-execution fact and tightens it.
//!
//! **The fact.** Klipper reads g-code in a single in-order stream whose
//! reader (`virtual_sdcard.work_handler`) strictly leads physical
//! execution: it runs ahead by at most `max_processing_lead`
//! ([`crate::config::ReconstructConfig::max_processing_lead`], the
//! lookahead buffer + step-gen lead). So at every instant the executed
//! file offset is `<=` the parse frontier at that instant.
//!
//! **The bound.** Let `F` be the newest durable context's frontier
//! (`virtual_sdcard.file_position`), captured at print time `t_ctx`, and
//! let the physical cut be at print time `t_cut`. Then
//!
//! ```text
//! P_stop = exec_offset(t_cut)
//!        = exec_offset(t_ctx) + path physically executed over [t_ctx, t_cut]
//!        <= F + path executable from F in Δt seconds        (Δt = t_cut − t_ctx)
//! ```
//!
//! because `exec_offset(t_ctx) <= F` (parser leads) and execution reaches
//! `F` no earlier than `t_ctx`, so the `[F, P_stop]` leg took at most `Δt`
//! seconds. Simulating `Δt` seconds forward from `F` with
//! [`plr_gcode::simulate`] — whose per-line time is a *lower* bound on the
//! real duration — therefore consumes at least every line the machine
//! could have executed, so its `resume_offset` is an *upper* bound on
//! `P_stop`. The window's high end becomes `min(extension_resume, cap)`;
//! the cap can only ever narrow, never widen.
//!
//! **Δt: the honest containment claim.** Containment does NOT rest on
//! "no term errs short." It rests on this: **every *observable* contributor
//! to `t_cut − t_ctx` is priced at its admitted maximum, and the sole
//! *unobservable* residual is hazard-pinned.** `Δt = t_cut − t_ctx =
//! staleness + Δt_tail`:
//!
//! * `staleness = max(0, t_a − t_ctx)` — the anchor context's measured age
//!   relative to the newest durable heartbeat `t_a` (`window.t_a`). A
//!   frontier that stopped being durable well before `t_a` (dropped
//!   contexts, a torn context tail) makes this large, which *loosens* the
//!   cap. Clamped at 0: an anchor newer than `t_a` is closer to the cut, so
//!   the tail term already covers it.
//! * `Δt_tail` bounds `t_cut − t_a` as a sum of four priced terms plus one
//!   pinned residual:
//!   1. **Record spacing** — the widest gap the heartbeat tail actually
//!      shows ([`heartbeat_tail_spacing_ns`]), *not* a nominal period. The
//!      newest durable heartbeat can trail the cut by about the recent
//!      cadence; guard 3 has already refused a tail whose newest gap
//!      exceeds `heartbeat_period_ns * heartbeat_gap_tolerance`, so this is
//!      bounded by `period * gap_tolerance`. Pricing the nominal 1× period
//!      was the review's blocker: it is neither what modern 10 Hz WALs show
//!      (~0.1 s) nor what guard 3 admits (3.0×).
//!   2. **Batch durability lag** — the newest *durable* heartbeat trails
//!      the newest *written* one, because `plrd::walsvc` appends heartbeat
//!      records with `SyncPolicy::Batched` and `fdatasync`s on a batch
//!      cadence (`Config::batch_sync_ms`, 0.5 s default). Priced from
//!      [`ReconstructConfig::durability_lag_ns`], which `plrd` derives from
//!      that same `batch_sync_ms` — never restated crate-locally.
//!   3. **Two independent subscription draws**, `2 * SUBSCRIPTION_REFRESH`
//!      (`webhooks.py:469`, 0.25 s each). The heartbeat's `print_time` and
//!      the anchor context's `file_position` come from *separate*
//!      `QueryStatusHelper` samples, each up to one refresh stale, and the
//!      clock correlation absorbs only the heartbeat's draw.
//!
//! **The terminal-stall residual (hazard pin, not priced).** One
//! contributor is unobservable: if `plrd` itself stalls (stops appending
//! heartbeats) while klippy keeps executing, the terminal gap `t_cut − t_a`
//! can exceed the widest *observed* tail gap without leaving any evidence,
//! *provided it stays below the observation-gap threshold* (a larger stall
//! surfaces as a `SubscriptionGap`/`SocketLost` and trips guard 2). Below
//! that threshold there is nothing in the WAL to price against, so — as the
//! epoch work documented its marker-less-restart pin — this residual is
//! **acknowledged, not guarded**: under a sub-threshold terminal writer
//! stall the cap can understate `Δt`. `terminal_writer_stall_residual_is_pinned`
//! constructs the shape and asserts the residual exists. Every other path
//! is priced.
//!
//! **Modern-WAL margin (worked).** On the real 10 Hz capture the observed
//! tail spacing is small, so `Δt_tail ≈ (tail gap) + 0.5 (durability) +
//! 0.5 (two draws) ≈ 1.3 s`, and the cap stays decisively useful (it trims
//! 3.3–4.7 s of over-covered horizon on the two real crashes). A legacy
//! 1 Hz stream shows wider gaps, so `Δt_tail` can approach `3.0 (tail) +
//! 1.0 = 4.0 s`; there the cap may narrow to a near-no-op — **acceptable,
//! because a useless-but-sound cap on a sparse legacy stream beats an
//! unsound one.**
//!
//! **The power-fail exact-T term (independent, tighter).** When the crash
//! epoch's tail carries a [`plr_wal::MarkerKind::PowerFailing`] edge with
//! no motion after it ([`WalTimeline::power_failing_tail`]), the cut is
//! bounded *directly*: `t_cut <= edge_print_time +
//! POWER_FAIL_HOLD_UP_MARGIN_S`, the moment power began failing plus the
//! admitted hold-up overshoot. This yields a second sound upper bound on
//! `Δt = t_cut - t_ctx`, and the cap takes `min(priced Δt, power-fail Δt)`
//! — sound because the minimum of two upper bounds is an upper bound, and
//! decisive because the edge replaces the entire inferred tail chain
//! (spacing + durability + draws + the un-priced terminal-stall residual)
//! with the cause of death itself. It is applied only where the priced cap
//! already holds (below); letting it survive a broken heartbeat tail or an
//! observation gap on its own — it depends on neither — is a deliberately
//! deferred follow-up, not built on this branch.
//!
//! **When the cap does not apply (guards; falls back to extension-only).**
//! If any premise fails the cap is `None` and the high end is the extension
//! resume offset alone:
//!
//! * `anchor_pt` unplaceable on the print-time axis — staleness is
//!   unmeasurable ([`Degradation::anchor_time_unknown`]).
//! * an observation gap overlaps the window
//!   ([`Degradation::observation_gap`]: a `SubscriptionGap`/`SocketLost`
//!   means motion could have advanced past `t_a` unobserved for longer than
//!   the tail spacing prices — this is the observable half of the stall
//!   above, and it trips here).
//! * the heartbeat tail is broken — the newest inter-heartbeat gap exceeds
//!   `heartbeat_period_ns * heartbeat_gap_tolerance`, so `t_a` is a stale
//!   island ([`heartbeat_tail_spacing_ns`] returns `None`).
//!
//! **The one-line skew (edge case a).** `virtual_sdcard.file_position`
//! can lag `gcode_move`'s `last_position` by exactly one line
//! ([`plr_wal::GcodeState::position`] docs): the true frontier can be
//! `F + 1` line. The cap therefore adds **one line of slack** to its
//! computed offset — the safe (looser) direction — so a stop on the
//! skewed line is never excluded.
//!
//! **The stalled frontier (edge case b) is loose, never tight.** When the
//! reader stalls mid-long-move `F` sits still while queued motion
//! executes, so `t_ctx` can postdate the frontier's execution by seconds.
//! The derivation never assumed otherwise: `Δt` bounds `t_cut − t_ctx`
//! regardless, and `simulate`'s per-line lower-bound timing keeps the
//! resume offset an upper bound on `P_stop`. A stall can only make the cap
//! looser (a larger true `Δt` than the machine used), never tighter.
//!
//! **Epoch isolation (edge case c).** The cap reads only `anchor`,
//! `window.t_a`, and `timeline.heartbeats`, all of which
//! [`crate::reconstruct`] has already narrowed to the crash epoch before
//! ingestion — an older boot's frontier or heartbeat can never feed it.

use plr_gcode::{scan_z_events, simulate, GcodeState, Line, LineIter, StopReason, ZScanConfig};
use plr_wal::{Context, TrapqSegment};

use crate::config::ReconstructConfig;
use crate::error::{ContextDefect, ReconstructError};
use crate::timeline::WalTimeline;
use crate::window::{is_observation_gap, ns_to_s, StopWindow};

/// Klipper's subscription refresh period, seconds
/// (`SUBSCRIPTION_REFRESH_TIME = .25`, `klippy/webhooks.py:469`): the
/// `QueryStatusHelper` re-queries subscribed objects on this timer, so any
/// single status sample (the one that fed the newest heartbeat, or the
/// anchor context) can be up to this stale relative to the instant its
/// values were true in Klipper. Added to the frontier cap's `Δt_tail` so
/// understating the sample age cannot make the cap tight. See the
/// module-level "Frontier cap".
const SUBSCRIPTION_REFRESH_S: f64 = 0.25;

/// Conservative hold-up margin, seconds: how long motion may keep
/// executing *after* a [`plr_wal::MarkerKind::PowerFailing`] edge before
/// the 24 V rail actually browns out and the MCU/heaters die.
///
/// The edge fires when power *begins* failing, so the physical cut is a
/// print time `>= edge`; this margin is the admitted upper bound on how
/// far past the edge the machine can still have moved (the DC-rail-fed
/// hold-up module keeps the Pi alive `>= 1 s`, and the watcher's debounce
/// re-read — a few ms — is dwarfed by it and folded in here). Used by the
/// frontier cap: the cut is at most `edge_print_time + this`, an
/// **independent, tighter** upper bound on `t_cut` than the priced
/// `Δt_tail` chain. **Widening is safe**: a larger value only loosens the
/// cap (never excludes the true stop), so it is set generously rather than
/// tuned; 1.0 s comfortably covers observed rail-collapse behaviour while
/// still trimming multiple seconds off the extension-only high end.
///
/// A `const`, not a config knob, deliberately: the brief calls for "a
/// conservative hold-up margin constant, documented", and a single audited
/// value avoids widening `ReconstructConfig` (a public, struct-literal-
/// constructed type) for a bound whose only safe direction is "large
/// enough".
pub(crate) const POWER_FAIL_HOLD_UP_MARGIN_S: f64 = 1.0;

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
    /// The frontier cap that was applied to the offset window's high end,
    /// when its premises held (see the module-level "Frontier cap"):
    /// `F + path executable in Δt`, plus one line of skew slack. `None`
    /// when the cap did not apply (a hazard pin fired) — the high end is
    /// then [`Self::resume_offset`] alone. Surfaced for reporting the
    /// before/after delta; the effective window high end is
    /// `min(resume_offset, frontier_cap)`.
    pub frontier_cap: Option<u64>,
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
    /// The merged WAL spanned more than one boot/firmware epoch and this
    /// reconstruction was scoped to the crash epoch alone: evidence from
    /// older boots, a pre-restart idle session, or a post-crash idle boot
    /// was **excluded** before ingestion (see [`crate::epoch`]).
    ///
    /// Informational, not a loss of fidelity: those records are not
    /// evidence about *this* crash, so removing them makes the result
    /// correct rather than uncertain — it does **not** move
    /// [`Self::confidence`]. Set by [`crate::reconstruct`], which owns the
    /// partition; [`crate::compute_stop_set`] never sets it on its own.
    /// Recompute [`crate::select_crash_epoch`] over the scan for the
    /// per-epoch detail behind this flag.
    pub cross_epoch_evidence_discarded: bool,
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

    let frontier_cap = frontier_cap_offset(
        timeline,
        window,
        anchor_pt,
        &anchor_state,
        &lines,
        vsd.file_position,
        tail_end,
        config,
        degradation.observation_gap,
    );

    Ok(Some(ExtensionResult {
        summary: ExtensionSummary {
            anchor_offset: vsd.file_position,
            anchor_print_time: anchor_pt,
            horizon,
            lines_consumed: sim.lines_consumed,
            resume_offset: sim.resume_offset,
            stop: sim.stop,
            frontier_cap,
        },
        z,
        xy,
        e_internal,
        e_file: Some(e_file),
    }))
}

/// The frontier cap: an upper bound on the physical stop offset derived
/// from the parser-leads-execution fact, or `None` when a premise fails
/// (the caller then leaves the offset-window high end at the extension
/// resume offset alone). See the module-level "Frontier cap" for the full
/// derivation and the conservatism of every term.
///
/// `anchor_pt` is the anchor context's print time (`None` when it could
/// not be placed on the print-time axis); `anchor_state` and `lines` are
/// the extension's own seed state and file lines *from the frontier
/// forward*, reused verbatim so the cap simulates exactly the path the
/// extension did, only for a shorter `Δt`. `observation_gap` is the flag
/// [`compute_stop_set`] computed before the extension ran.
#[allow(clippy::too_many_arguments)]
fn frontier_cap_offset(
    timeline: &WalTimeline,
    window: &StopWindow,
    anchor_pt: Option<f64>,
    anchor_state: &GcodeState,
    lines: &[Line],
    frontier: u64,
    tail_end: u64,
    config: &ReconstructConfig,
    observation_gap: bool,
) -> Option<u64> {
    // Guard 1: without the anchor's print time the staleness term is
    // unmeasurable, so `Δt` cannot be bounded — fall back.
    let anchor_pt = anchor_pt.filter(|pt| pt.is_finite())?;
    // Guard 2: an observation gap means motion could have advanced past
    // `t_a` unobserved for longer than a heartbeat spacing, voiding the
    // `Δt_tail` premise.
    if observation_gap {
        return None;
    }
    let t_a = window.t_a;
    if !t_a.is_finite() {
        return None;
    }
    // Guard 3 AND the spacing term in one tail walk: a broken tail (the
    // newest heartbeat is a stale island) refuses the cap; otherwise the
    // terminal gap `t_cut - t_a` is priced at the widest gap the tail
    // actually shows — evidence, not a nominal period.
    let spacing_ns = heartbeat_tail_spacing_ns(timeline, config)?;

    let staleness = (t_a - anchor_pt).max(0.0);
    // Δt_tail bounds `t_cut - t_a`. Every OBSERVABLE contributor is priced
    // at its admitted maximum (see the module-level "Frontier cap"):
    //   * record spacing — the widest gap the heartbeat tail shows (guard 3
    //     above has already refused a tail whose newest gap exceeds
    //     tolerance, so this is bounded by `period * gap_tolerance`);
    //   * batch durability lag — one `fdatasync` batch, because the newest
    //     DURABLE heartbeat trails the newest WRITTEN one (`plrd::walsvc`
    //     appends heartbeat records `SyncPolicy::Batched`); config-derived,
    //     never a literal here;
    //   * two independent 0.25 s subscription draws — the heartbeat's status
    //     sample AND the anchor context's `file_position` sample are each up
    //     to `SUBSCRIPTION_REFRESH` stale, and the clock correlation absorbs
    //     only the heartbeat's draw, not the anchor's.
    // The sole UNOBSERVABLE contributor — a terminal writer stall below the
    // observation-gap threshold — is hazard-pinned, not priced (see the
    // module doc's "terminal-stall residual").
    let dt_tail =
        ns_to_s(spacing_ns) + ns_to_s(config.durability_lag_ns) + 2.0 * SUBSCRIPTION_REFRESH_S;
    let dt = staleness + dt_tail;
    if !dt.is_finite() {
        return None;
    }
    // Independent, tighter bound from a tail power-fail edge (exact-T). The
    // cut is at most `edge_print_time + POWER_FAIL_HOLD_UP_MARGIN_S`, so
    // `t_cut - t_ctx <= (edge_pt + margin) - anchor_pt`. Both this and the
    // priced `Δt` above are sound upper bounds on `t_cut - t_ctx`, so their
    // `min` is still a sound upper bound — it can only ever narrow the cap
    // in the safe (containment-preserving) direction, and does so
    // decisively because the marker replaces the whole inferred tail chain
    // with the moment power actually began failing.
    //
    // Scope note (minimal, this branch): the power-fail bound is applied
    // here, *after* the cap's existing premises (guards 1-3) have held, so
    // it tightens an already-applicable cap rather than reviving one the
    // guards refused. Making the power-fail bound survive a broken
    // heartbeat tail or an observation gap on its own — it does not depend
    // on either — is the deferred "exact mode" the brief holds for a
    // follow-up; it is not built here.
    let dt = match power_fail_dt(timeline, window, anchor_pt) {
        Some(pf) => dt.min(pf),
        None => dt,
    };

    // Simulate forward from the frontier for `Δt` seconds. `simulate`'s
    // per-line timing is a lower bound on real durations, so it consumes at
    // least every line executable in `Δt` — its resume offset is an upper
    // bound on the executed offset. The line that pushes cumulative time
    // over `Δt` is itself consumed (the check is post-line), which already
    // contributes toward, but is not relied on for, the skew slack below.
    let mut sim_config = config.sim.clone();
    sim_config.max_duration = Some(dt);
    let mut state = anchor_state.clone();
    let sim = simulate(&mut state, lines, &sim_config);
    let raw = sim.resume_offset.unwrap_or(frontier).max(frontier);

    // Edge case (a), the one-line sampling skew: `file_position` can lag
    // `gcode_move.last_position` by exactly one line
    // (`plr_wal::GcodeState::position`), so the true frontier may be one
    // line past `frontier`. Add one line of slack — the end of the first
    // line the `Δt` simulation did NOT consume — which is `>= raw` and the
    // safe (looser) direction. When every line was consumed there is no
    // further line, so the tail end is the honest cap.
    let slacked = lines
        .get(sim.lines_consumed)
        .map_or(tail_end, |l| l.span.end);
    Some(raw.max(slacked).min(tail_end))
}

/// The `Δt` upper bound implied by a **tail** power-fail edge, or `None`
/// when there is none or it cannot be placed on the print-time axis.
///
/// A [`WalTimeline::power_failing_tail`] at host-monotonic `edge` maps to
/// print time `edge_pt`; the physical cut is at most
/// `edge_pt + POWER_FAIL_HOLD_UP_MARGIN_S` (the edge is a *lower* bound on
/// the cut, the margin the admitted overshoot). Subtracting the anchor's
/// print time gives an upper bound on `t_cut - t_ctx` = `Δt`. Clamped at 0
/// (an edge that maps before the anchor means the cut is essentially at
/// the frontier; the frontier + one line of skew still applies downstream)
/// and required finite.
fn power_fail_dt(timeline: &WalTimeline, window: &StopWindow, anchor_pt: f64) -> Option<f64> {
    let edge = timeline.power_failing_tail()?;
    let edge_pt = window.mono_ns_to_print_time(edge)?;
    let dt = (edge_pt + POWER_FAIL_HOLD_UP_MARGIN_S) - anchor_pt;
    dt.is_finite().then(|| dt.max(0.0))
}

/// The heartbeat tail's observed spacing, in nanoseconds — the widest gap
/// between consecutive durable heartbeats over the contiguous run ending at
/// the newest sample — or `None` when the tail is broken (the newest gap
/// exceeds `heartbeat_period_ns * heartbeat_gap_tolerance`, so `t_a` is a
/// stale island) or there are no samples.
///
/// This serves the frontier cap two ways at once: a `None` result is guard
/// 3 (refuse the cap), and a `Some(gap)` result is the cap's spacing term —
/// the terminal gap `t_cut - t_a` priced at the widest gap the tail
/// actually shows rather than a nominal period. So a dense 10 Hz stream
/// (small gaps) keeps a tight cap and a sparse stream honestly loosens it.
///
/// The walk stops at the first over-tolerance gap going backward: an early
/// idle→active coverage break does not inflate the recent-cadence estimate,
/// but if that break IS the newest gap, `t_a` is isolated and the cap is
/// refused. A single sample carries no gap evidence, so the conservative
/// tolerance spacing (the widest the guard would admit) is returned; zero
/// samples cannot reach the cap (the pipeline errors first) and are refused.
fn heartbeat_tail_spacing_ns(timeline: &WalTimeline, config: &ReconstructConfig) -> Option<u64> {
    let hbs = &timeline.heartbeats;
    let n = hbs.len();
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    let tol_ns = (config.heartbeat_period_ns as f64 * config.heartbeat_gap_tolerance) as u64;
    match n {
        0 => None,
        1 => Some(tol_ns),
        _ => {
            let mut max_gap_ns = 0_u64;
            for i in (1..n).rev() {
                let gap_ns = hbs[i].mono_ns.saturating_sub(hbs[i - 1].mono_ns);
                if gap_ns > tol_ns {
                    if i == n - 1 {
                        return None; // newest gap broken: t_a is a stale island
                    }
                    break; // an older coverage break ends the contiguous tail
                }
                max_gap_ns = max_gap_ns.max(gap_ns);
            }
            Some(max_gap_ns)
        }
    }
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
    let uncapped = extension
        .and_then(|e| e.summary.resume_offset)
        .unwrap_or(anchor_vsd.file_position)
        .max(anchor_vsd.file_position);
    // The frontier cap can only narrow the high end, never widen it, and
    // never below the frontier itself (a valid stop position). When it did
    // not apply the high end is the extension resume offset alone. See the
    // module-level "Frontier cap".
    let end = extension
        .and_then(|e| e.summary.frontier_cap)
        .map_or(uncapped, |cap| {
            uncapped.min(cap.max(anchor_vsd.file_position))
        });
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
        anchor_state_from_context, compute_stop_set, Confidence, FileTail, Interval, Provenance,
        ZKind,
    };
    use crate::config::ReconstructConfig;
    use crate::error::{ContextDefect, ReconstructError};
    use crate::testutil::{
        context_at, context_with_gcode, heartbeat_at, ingest_records, stepper_range,
        stepper_range_with_clock, trapq_segment_xyz,
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

    // --- the un-evidenced extruder band: the OPEN hazard ----------------

    /// File whose lines up to the anchor frontier contain a retract to E-4
    /// and back. Absolute E so the values are readable; the trailing lines
    /// give the extension something to consume.
    const RETRACT_TEXT: &str = concat!(
        "G1 X60 Y50 E10 F3000
", //  0..21  E=10
        "G1 E6
", // 21..28  retract to 6  <-- the excursion
        "G1 E10
", // 28..36  back to 10
        "G1 X70 Y50 E11
", // 36..52  E=11
        "G1 X80 Y50 E12
", // 52..68
        "G1 X90 Y50 E13
",
    );

    /// Offset just past `G1 E10` (the unretract), so the excursion is
    /// strictly *inside* the processed-but-un-evidenced region.
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

    /// **Pins the open containment limit in `e_internal`.**
    ///
    /// Durable extruder coverage stops before the retract's own rows, so a
    /// retract to E = 6 and back to E = 10 sits entirely inside lines the
    /// anchor context already counts as processed. Both *endpoints* read
    /// E = 10, so neither the trapq evaluation (which sees only coverage)
    /// nor the extension (which starts at the frontier, after the
    /// excursion) can bound the interior — and this asserts that
    /// `e_internal` does **not** contain E = 6.
    ///
    /// This test asserts the bug, deliberately. It exists so the limit is
    /// executable rather than prose, and so whatever finally closes it has
    /// a target: `e_file`'s replay already recovers E = 6 on the exact
    /// path, and `plrd::pipeline`'s `stop_evidence` discards `e_file`. When
    /// that is fixed this assertion must be inverted, not deleted. See the
    /// module-level "Durable extruder coverage".
    #[test]
    fn e_internal_does_not_bound_a_retract_inside_the_un_evidenced_band() {
        // Extruder coverage ending at print time 9.0 — before the anchor.
        let e_row = trapq_segment_xyz(
            "extruder",
            8.0,
            1.0,
            [10.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            1.0,
            8_000_000_000,
        );
        let floor = context_with_gcode(9_000_000_000, 0, e10_gcode(10.0));
        let anchor = context_with_gcode(20_000_000_000, RETRACT_FRONTIER, e10_gcode(10.0));
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
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        let e = set.e_internal.expect("internal E interval");
        assert!(
            !e.contains(6.0, 1e-9),
            "e_internal {e:?} now bounds the retract low point E=6 — the open              limit this pins has been closed; invert this assertion and update              the module docs rather than deleting the test"
        );
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

    // ------------------------------------------------------------------
    // Frontier cap (Part 1). See the module-level "Frontier cap".
    // ------------------------------------------------------------------

    /// A long file of equal 5 mm X moves from the anchor's X50, one per
    /// line — enough path that neither the extension horizon nor Δt
    /// reaches EOF, so the cap can be observed narrowing.
    fn long_x_march() -> String {
        use std::fmt::Write as _;
        let mut s = String::new();
        for i in 1..=80 {
            let _ = writeln!(s, "G1 X{} Y50 F3000", 50 + 5 * i);
        }
        s
    }

    /// The default anchor gcode at (50, 50, 0.2), E 100, absolute.
    fn march_gcode() -> WalGcodeState {
        WalGcodeState {
            speed_factor: 1.0,
            speed: 3000.0,
            extrude_factor: 1.0,
            absolute_coordinates: true,
            absolute_extrude: true,
            homing_origin: vec![0.0; 4],
            position: vec![50.0, 50.0, 0.2, 100.0],
            gcode_position: vec![50.0, 50.0, 0.2, 100.0],
        }
    }

    #[test]
    fn frontier_cap_narrows_the_offset_window_below_the_extension() {
        // Fresh anchor at pt 10 (t_a = 10, staleness 0). No trapq, so the
        // extension start falls to the reader-lead bound (t_a - 3 = 7) and
        // the horizon is 2 + (10.1 - 7) = 5.1 s of simulated path; the cap
        // is only Δt = 0 + 1.0 (heartbeat spacing) + 0.25 = 1.25 s.
        let text = long_x_march();
        let records = base_records(
            10.0,
            10.1,
            vec![WalRecord::Context(context_with_gcode(
                10_000_000_000,
                0,
                march_gcode(),
            ))],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        let ext = set.extension.clone().expect("extension ran");
        let fw = set.file_window.unwrap();

        let cap = ext.frontier_cap.expect("cap applied on a clean tail");
        let uncapped = ext.resume_offset.expect("extension consumed lines");
        assert!(
            cap < uncapped,
            "cap {cap} did not narrow below the extension resume {uncapped}"
        );
        assert_eq!(fw.end, cap, "the window high end must be the cap");
        assert!(fw.end < text.len() as u64, "cap must not reach EOF here");
        assert!(fw.start <= fw.end);
    }

    /// A tail power-fail edge collapses the window further through the
    /// existing frontier-cap path: the same fixture, plus a
    /// `PowerFailing` marker whose print time is ~`t_a`, produces a
    /// strictly smaller cap (the exact-T bound `edge_pt + 1.0` undercuts
    /// the priced `Δt`). A spurious marker (motion after it) changes
    /// nothing. This is requirement 4's window collapse and the
    /// honest-wide-never-narrow property in one place.
    #[test]
    fn a_tail_power_fail_edge_tightens_the_frontier_cap() {
        let text = long_x_march();
        let base = || {
            base_records(
                10.0,
                10.1,
                vec![WalRecord::Context(context_with_gcode(
                    10_000_000_000,
                    0,
                    march_gcode(),
                ))],
            )
        };
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let cap_of = |records: Vec<WalRecord>| -> u64 {
            let timeline = ingest_records(records);
            let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
            let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
            set.file_window.unwrap().end
        };

        let plain = cap_of(base());

        // Genuine tail edge at mono 10.2 s (after the stepper at 10.1 s):
        // maps to print time ~10.2, so the exact-T bound is ~11.2 - 10.0 =
        // ~1.2 s, well under the priced Δt.
        let mut with_edge = base();
        with_edge.push(WalRecord::Marker(Marker {
            mono_ns: 10_200_000_000,
            kind: MarkerKind::PowerFailing,
        }));
        let capped = cap_of(with_edge);
        assert!(
            capped < plain,
            "the power-fail edge must tighten the cap: {capped} !< {plain}"
        );

        // Spurious edge BEFORE the motion (mono 5 s < stepper 10.1 s): not
        // a tail fact, so the cap is exactly the plain one — never narrower.
        let mut spurious = base();
        spurious.insert(
            0,
            WalRecord::Marker(Marker {
                mono_ns: 5_000_000_000,
                kind: MarkerKind::PowerFailing,
            }),
        );
        assert_eq!(
            cap_of(spurious),
            plain,
            "a spurious (non-tail) edge must not narrow the window"
        );
    }

    #[test]
    fn frontier_cap_slack_is_exactly_one_line() {
        use plr_gcode::{simulate, Line, LineIter};
        // Same fresh-anchor march. Recompute the Δt the cap runs — from the
        // config's own terms, not a hardcoded number — and prove the window
        // high end is exactly one line past where that simulation stopped
        // (the one-line skew slack, edge case a; no more, no less).
        let text = long_x_march();
        let records = base_records(
            10.0,
            10.1,
            vec![WalRecord::Context(context_with_gcode(
                10_000_000_000,
                0,
                march_gcode(),
            ))],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        let fw = set.file_window.unwrap();

        // Δt = staleness + spacing + durability + 2·refresh. For this
        // fresh-anchor fixture the anchor's print time equals t_a, so
        // staleness is 0; the single heartbeat yields the conservative
        // tolerance spacing (no cadence evidence). Derived via the cap's own
        // spacing helper so the two cannot drift.
        #[allow(clippy::cast_precision_loss)]
        let spacing_s = super::heartbeat_tail_spacing_ns(&timeline, &cfg()).unwrap() as f64 / 1e9;
        #[allow(clippy::cast_precision_loss)]
        let durability_s = cfg().durability_lag_ns as f64 / 1e9;
        let dt = spacing_s + durability_s + 2.0 * super::SUBSCRIPTION_REFRESH_S;

        let anchor_state = anchor_state_from_context(&march_gcode()).unwrap();
        let lines: Vec<Line> = LineIter::new(text.as_bytes(), 0).collect();
        let mut sc = cfg().sim.clone();
        sc.max_duration = Some(dt);
        let mut st = anchor_state.clone();
        let sim = simulate(&mut st, &lines, &sc);
        let raw = sim.resume_offset.unwrap();
        let with_slack = lines[sim.lines_consumed].span.end;
        assert!(with_slack > raw, "test needs a further line to exist");
        assert_eq!(
            fw.end, with_slack,
            "cap must add exactly one line of skew slack (raw {raw}, Δt {dt})"
        );
    }

    /// Reviewer probe (platform: `plr-reconstruct`). The blocker was that
    /// pricing `Δt_tail` at the nominal 1× period (+ one draw) excluded
    /// reachable file on a legacy 1 Hz stream (~54 B / ~3 lines). With the
    /// batch durability lag and the second subscription draw priced, the cap
    /// now CONTAINS that file. Heartbeats 1 s apart, fresh anchor
    /// (staleness 0): observed spacing 1.0 s, so `Δt_tail` = 1.0 + 0.5
    /// (durability) + 0.5 (two draws) = 2.0 s, versus the old-too-tight
    /// 1.0 + 0.25 = 1.25 s.
    #[test]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn legacy_1hz_cap_contains_what_the_nominal_bound_excluded() {
        use plr_gcode::{simulate, Line, LineIter};
        let text = long_x_march();
        let mono = |pt: f64| (pt * 1e9) as u64;
        let records = vec![
            WalRecord::Heartbeat(heartbeat_at(mono(8.0), 8.0)),
            WalRecord::Heartbeat(heartbeat_at(mono(9.0), 9.0)),
            WalRecord::Heartbeat(heartbeat_at(mono(10.0), 10.0)),
            WalRecord::StepperRange(stepper_range_with_clock(
                "stepper_z",
                10.0,
                FREQ,
                mono(10.0),
            )),
            WalRecord::Context(context_with_gcode(mono(10.0), 0, march_gcode())),
        ];
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        let fw = set.file_window.unwrap();

        let anchor_state = anchor_state_from_context(&march_gcode()).unwrap();
        let lines: Vec<Line> = LineIter::new(text.as_bytes(), 0).collect();
        let offset_for = |dt: f64| -> u64 {
            let mut sc = cfg().sim.clone();
            sc.max_duration = Some(dt);
            let mut st = anchor_state.clone();
            simulate(&mut st, &lines, &sc).resume_offset.unwrap()
        };
        let old_too_tight = offset_for(1.0 + 0.25);
        let new_correct = offset_for(1.0 + 0.5 + 0.5);
        assert!(
            new_correct > old_too_tight,
            "the priced-in terms must reach further file (old {old_too_tight}, new {new_correct})"
        );
        assert!(
            fw.end >= new_correct,
            "the cap must now CONTAIN the file the nominal bound excluded \
             (old {old_too_tight}, new {new_correct}, cap {})",
            fw.end
        );
    }

    /// Reviewer probe (platform: `plr-reconstruct`). Guard 3 admits gaps up
    /// to 3.0× the 1 s basis; a 2.9 s terminal gap is admitted and the
    /// spacing term must be priced at the observed 2.9 s, not the nominal
    /// 1 s. A 3.1 s gap exceeds tolerance and the cap falls back.
    #[test]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn frontier_cap_prices_a_wide_but_admitted_tail_gap_and_refuses_a_broken_one() {
        let text = long_x_march();
        let mono = |pt: f64| (pt * 1e9) as u64;
        let build = |gap: f64| {
            ingest_records(vec![
                WalRecord::Heartbeat(heartbeat_at(mono(10.0 - gap), 10.0 - gap)),
                WalRecord::Heartbeat(heartbeat_at(mono(10.0), 10.0)),
                WalRecord::StepperRange(stepper_range_with_clock(
                    "stepper_z",
                    10.0,
                    FREQ,
                    mono(10.0),
                )),
                WalRecord::Context(context_with_gcode(mono(10.0), 0, march_gcode())),
            ])
        };
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };

        // 2.9 s gap: admitted; spacing priced at the observed 2.9 s.
        let tl = build(2.9);
        let window = compute_stop_window(&tl, None, &cfg()).unwrap();
        let set = compute_stop_set(&tl, &window, Some(&tail), &cfg()).unwrap();
        assert!(
            set.extension.as_ref().unwrap().frontier_cap.is_some(),
            "a 2.9 s gap is within the 3.0 s tolerance — cap applies"
        );
        #[allow(clippy::cast_precision_loss)]
        let spacing_s = super::heartbeat_tail_spacing_ns(&tl, &cfg()).unwrap() as f64 / 1e9;
        assert!(
            (spacing_s - 2.9).abs() < 1e-6,
            "spacing must be the observed 2.9 s, got {spacing_s}"
        );

        // 3.1 s gap: exceeds tolerance — the tail is broken, cap falls back.
        let tl2 = build(3.1);
        let window2 = compute_stop_window(&tl2, None, &cfg()).unwrap();
        let set2 = compute_stop_set(&tl2, &window2, Some(&tail), &cfg()).unwrap();
        assert!(
            set2.extension.as_ref().unwrap().frontier_cap.is_none(),
            "a 3.1 s gap breaks the tail — cap must refuse"
        );
    }

    /// The terminal-stall residual pin (platform: `plr-reconstruct`). The one
    /// unpriced contributor: a terminal writer stall (`plrd` stops appending
    /// heartbeats while klippy keeps executing) below the observation-gap
    /// threshold leaves no evidence, so the cap — which prices `Δt_tail` from
    /// the OBSERVED heartbeat cadence — is blind to how far motion actually
    /// extended past `t_a`. This pins the residual: a dense tail yields a
    /// small `Δt_tail` while durable motion extends much further past `t_a`.
    /// Acknowledged, not guarded (see the module doc).
    #[test]
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    fn terminal_writer_stall_residual_is_pinned() {
        let mono = |pt: f64| (pt * 1e9) as u64;
        let mut records = Vec::new();
        // Dense 10 Hz tail ending at t_a = 10.0 (0.1 s gaps).
        for i in 0..=20u64 {
            let pt = 10.0 - 0.1 * (20 - i) as f64;
            records.push(WalRecord::Heartbeat(heartbeat_at(mono(pt), pt)));
        }
        // Durable motion (a dumped-ahead plan) reaching 2.0 s past t_a, with
        // NO further heartbeat and NO SubscriptionGap marker.
        records.push(WalRecord::StepperRange(stepper_range_with_clock(
            "stepper_z",
            12.0,
            FREQ,
            mono(12.0),
        )));
        records.push(WalRecord::Context(context_with_gcode(
            mono(10.0),
            0,
            march_gcode(),
        )));
        let timeline = ingest_records(records);

        let dt_tail = super::heartbeat_tail_spacing_ns(&timeline, &cfg()).unwrap() as f64 / 1e9
            + cfg().durability_lag_ns as f64 / 1e9
            + 2.0 * super::SUBSCRIPTION_REFRESH_S;
        let motion_past_ta = 12.0 - 10.0;
        assert!(
            motion_past_ta > dt_tail,
            "residual pin: motion can reach {motion_past_ta}s past t_a while Δt_tail prices \
             only {dt_tail}s from the observed cadence — a sub-threshold terminal stall is \
             unpriced and unguarded"
        );
    }

    #[test]
    fn frontier_cap_falls_back_on_an_observation_gap() {
        // A subscription gap overlapping the window voids the Δt_tail
        // premise: the cap must not apply, and the high end is the
        // extension resume offset alone (today's behavior).
        let text = long_x_march();
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
                WalRecord::Context(context_with_gcode(10_000_000_000, 0, march_gcode())),
            ],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        let ext = set.extension.clone().unwrap();
        assert!(set.degradation.observation_gap);
        assert!(ext.frontier_cap.is_none(), "cap must fall back on a gap");
        assert_eq!(
            set.file_window.unwrap().end,
            ext.resume_offset.unwrap(),
            "high end must be the uncapped extension resume"
        );
    }

    #[test]
    fn frontier_cap_falls_back_on_a_broken_heartbeat_tail() {
        // The two newest heartbeats are 5 s apart (> 1.0 * 3.0 tolerance):
        // t_a may be a stale island far from the cut, so the cap refuses.
        let text = long_x_march();
        let mut records = base_records(
            10.0,
            10.1,
            vec![WalRecord::Context(context_with_gcode(
                10_000_000_000,
                0,
                march_gcode(),
            ))],
        );
        // An older, isolated heartbeat: the newest inter-sample gap becomes
        // 10 - 5 = 5 s.
        records.push(WalRecord::Heartbeat(heartbeat_at(5_000_000_000, 5.0)));
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let tail = FileTail {
            base_offset: 0,
            bytes: text.as_bytes(),
        };
        let set = compute_stop_set(&timeline, &window, Some(&tail), &cfg()).unwrap();
        let ext = set.extension.clone().unwrap();
        assert!(
            ext.frontier_cap.is_none(),
            "cap must fall back on a broken heartbeat tail"
        );
        assert_eq!(set.file_window.unwrap().end, ext.resume_offset.unwrap());
    }

    #[test]
    fn frontier_cap_guard_refuses_without_anchor_time() {
        use plr_gcode::{Line, LineIter};
        // Guard 1 is a type-level possibility the ingest finiteness filter
        // prevents end-to-end (a finite mono always maps to a finite print
        // time), so it is exercised at the function boundary directly:
        // `anchor_pt = None` must yield no cap.
        let text = long_x_march();
        let records = base_records(
            10.0,
            10.1,
            vec![WalRecord::Context(context_with_gcode(
                10_000_000_000,
                0,
                march_gcode(),
            ))],
        );
        let timeline = ingest_records(records);
        let window = compute_stop_window(&timeline, None, &cfg()).unwrap();
        let anchor_state = anchor_state_from_context(&march_gcode()).unwrap();
        let lines: Vec<Line> = LineIter::new(text.as_bytes(), 0).collect();
        let cap = super::frontier_cap_offset(
            &timeline,
            &window,
            None, // anchor time unknown
            &anchor_state,
            &lines,
            0,
            text.len() as u64,
            &cfg(),
            false,
        );
        assert!(cap.is_none(), "no anchor time => no cap");
        // And the positive control: with the anchor time present, it does
        // apply — so the guard, not some other refusal, is what fired.
        let cap_ok = super::frontier_cap_offset(
            &timeline,
            &window,
            Some(10.0),
            &anchor_state,
            &lines,
            0,
            text.len() as u64,
            &cfg(),
            false,
        );
        assert!(cap_ok.is_some());
    }

    #[test]
    fn stalled_frontier_keeps_the_cap_loose_not_tight() {
        // Edge case (b): the reader stalled on one long move, so the
        // frontier's execution began at pt 16 while the snapshot is at 20.
        // The cap simulates only Δt from the frontier, but because Δt
        // bounds t_cut - t_ctx regardless of the stall and simulate's
        // per-line timing is a lower bound, the cap can only be loose. The
        // window must still contain the stalled long move's span and never
        // collapse below the frontier.
        let text = concat!(
            "G1 X200 Y200 E5 F3000\n",
            "G1 X210 Y200 E6\n",
            "G1 X220 Y200 E7\n",
            "G1 X230 Y200 E8\n",
            "G1 X240 Y200 E9\n",
        );
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
        let fw = set.file_window.unwrap();
        // The long first move (4.24 s) alone exceeds Δt = 1.25 s, so the
        // Δt sim consumes it (post-check) plus one skew line: the cap sits
        // at or past the second line boundary, never below the frontier.
        assert!(fw.start <= fw.end);
        assert!(fw.contains(0), "frontier must remain a candidate");
        let first_line_end = text.find('\n').unwrap() as u64 + 1;
        assert!(
            fw.end >= first_line_end,
            "the stalled long move must stay inside the window: end {} < {}",
            fw.end,
            first_line_end
        );
    }
}
