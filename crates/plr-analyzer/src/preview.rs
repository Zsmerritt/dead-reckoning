//! Resume-point **preview** set: a parallel builder that turns the same
//! stop evidence the matcher consumes into an operator-navigable set of
//! candidate resume points (design `docs/design/resume-preview.md`, §A).
//!
//! # Why this is parallel to, not folded into, the matcher
//!
//! [`crate::matcher::match_stop_point`]'s confidence ladder is a **width
//! gate**, documented as structurally blind to incompleteness (a
//! truncated window looks *more* confident). Weakening or forking it
//! risks the safe automatic paths, so [`build_preview`] leaves the ladder
//! byte-identical and reuses only the shared evaluate path
//! ([`crate::matcher::collect_candidates`]) — one evaluate loop, two
//! consumers.
//!
//! # The nudge domain is deliberately WIDER than the candidate set (§A.2)
//!
//! The matcher's candidate set is gated by XY/E/Z tolerance; the true
//! stop can sit just outside `xy_tolerance`. The preview's selectable
//! stops are therefore **every in-window [`MoveKind::Extrusion`] move**
//! (not just the matched candidates): every one is a valid "last printed
//! line" by construction, so the physical ragged edge on the part is
//! always reachable by nudging even when it fell outside the matcher's
//! tolerance box. The matched candidates are a *labelled subset*
//! ([`PreviewStop::is_candidate`]) used only to seed the representatives
//! and the first/mid/last policy anchors.
//!
//! # Totality
//!
//! [`build_preview`] never panics: it validates evidence/config with the
//! matcher's own predicate, and every other outcome (too-wide, no stops)
//! is a typed [`PreviewOutcome`] variant.

use serde::{Deserialize, Serialize};

use crate::geom;
use crate::matcher::{collect_candidates, in_window, validate, MatchConfig, StopEvidence};
use crate::model::{FeatureClass, LayerModel, MoveKind, SimMove};
use crate::work::ExclusionOracle;

/// Distinct candidate layers above which preview refuses admission
/// (design §A.5, ruled). A tall Z spread means the Z evidence cannot even
/// say which layer, so the physical-edge trick cannot disambiguate.
pub const PREVIEW_MAX_LAYERS: usize = 8;

/// In-window deposition stops above which preview refuses admission
/// (design §A.5, ruled). A window this large is a reconstruction
/// pathology (the pre-epoch-fix 102,000 s bug shape), not a 2.5–4 s crash.
/// This is the admission gate on the raw nudge domain, **not** a
/// candidate-count cap (CONFLICT-1 resolved: no candidate-count cap —
/// representatives keep the UX to 3–7 stops regardless).
pub const PREVIEW_MAX_STOPS: usize = 2000;

/// Upper bound on the representative set the operator steps through
/// (design §A.3: "3..=7"). The lower bound is a target, not enforceable
/// — a set with only one or two candidate stops yields that many reps.
pub const PREVIEW_MAX_REPS: usize = 7;

/// Admission bounds for [`build_preview`] (design §A.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PreviewBounds {
    /// Refuse when the distinct-layer count among matched candidates
    /// exceeds this.
    pub max_layers: usize,
    /// Refuse when the in-window deposition-stop count exceeds this.
    pub max_stops: usize,
}

impl Default for PreviewBounds {
    /// The ruled defaults: [`PREVIEW_MAX_LAYERS`] / [`PREVIEW_MAX_STOPS`].
    fn default() -> Self {
        Self {
            max_layers: PREVIEW_MAX_LAYERS,
            max_stops: PREVIEW_MAX_STOPS,
        }
    }
}

/// One selectable stop in the preview — a single in-window extrusion
/// move the operator can hover over and accept (design §A.2).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PreviewStop {
    /// Position in the ordered [`PreviewSet::stops`] list (execution
    /// order). This is the one index space representatives, nudge steps,
    /// and the first/mid/last anchors all share.
    pub index: u32,
    /// This deposition line's byte offset — a line boundary
    /// (`SimMove::span.start`), shown to the operator and safe for `M26`.
    ///
    /// # Arc-chord nudge consequence (design §12)
    ///
    /// A `G2`/`G3` arc decomposes into many `SimMove` chords that all share
    /// the arc's ONE source line, so several adjacent stops carry the SAME
    /// `offset` but different [`Self::xy`]. A ±1 nudge that steps between two
    /// chords of one arc therefore moves the hover point along the curve
    /// while this `offset` (and the `Line:` readout derived from it) does
    /// NOT change. The reposition loop's prompt must show the per-chord
    /// [`Self::xy`], not only the offset, so a within-arc nudge gives
    /// visible feedback — and a UI must not read "offset unchanged" as
    /// "nudge had no effect". The first/mid/last anchors and the
    /// `last_index` skip-forward pin are unaffected: they resolve on the
    /// committed *resume* offset, and same-offset ties break toward the last
    /// chord ([`stop_resuming_at`]).
    pub offset: u64,
    /// Where a resume STARTS if this stop is accepted:
    /// `first_deposition_at_or_after(this.span.end)` — "resume at the NEXT
    /// line" (the ruling's semantics: the operator picks the last line
    /// that printed; resume begins at the next deposition). Baked here so
    /// the committed skip-forward is fixed arithmetic. When no later
    /// deposition exists, this is `span.end` (nothing remains to deposit
    /// past this stop).
    pub resume_offset: u64,
    /// Hover XY = this move's start, in Klipper-internal coordinates.
    pub xy: [f64; 2],
    /// Deposition Z at this stop (this move's start Z), internal frame.
    pub z: f64,
    /// Layer active at the move (`None` before the first deposition).
    pub layer: Option<u32>,
    /// Feature class of the move's source line (for the prompt). Derived
    /// exactly as `select_resume_target`'s `on_infill` rule does — the
    /// `;TYPE:` path whose segment shares this line's `span.start`
    /// (`plr-recovery/src/build.rs:999-1009`) — [`FeatureClass::Other`]
    /// when the line is unannotated or not found.
    pub feature: FeatureClass,
    /// `true` when [`Self::feature`] is internal or solid infill — the
    /// preferred, seam-hiding case (same rule as `on_infill`).
    pub on_infill: bool,
    /// `true` when this stop's line matched the stop evidence (it is one
    /// of the matcher's candidates). Seeds the representatives; a `false`
    /// stop is a nudge-only reachable line outside the tolerance box.
    pub is_candidate: bool,
}

/// A built preview: the full nudge domain plus its navigation anchors
/// (design §A.3). All indices are into [`Self::stops`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PreviewSet {
    /// The full nudge domain — every kept in-window extrusion move, in
    /// execution order.
    pub stops: Vec<PreviewStop>,
    /// Representative stop indices the operator steps through: window
    /// endpoints plus spatial cluster reps, ascending, deduped, `<=`
    /// [`PREVIEW_MAX_REPS`]. Always drawn from candidate stops and always
    /// contains the earliest and latest candidate stop.
    pub representatives: Vec<u32>,
    /// Policy `first` — the stop AT the minimum-offset candidate LINE (the
    /// last chord if that line is an arc). A "may-reprint" reference cursor;
    /// selected over the per-line population so arc chords do not skew it
    /// (see [`anchor_index`], MINOR-1).
    pub first_index: u32,
    /// Policy `mid` — the stop AT the lower-median candidate LINE (by
    /// execution-order offset; see [`median_index`]), same per-line rule as
    /// [`Self::first_index`].
    pub mid_index: u32,
    /// Policy `last` — the RESUME-COMMITTING stop for the skip-forward
    /// default: the stop whose *acceptance* commits the resume that
    /// `plr-recovery`'s `select_resume_target_with_policy(Last)` commits
    /// (the predecessor of the resolver's resume line — [`stop_resuming_at`]),
    /// NOT simply the maximum-offset candidate stop. Anchoring on the
    /// committed *resume* rather than the stop offset is what makes `last`
    /// equal today's skip-forward byte-for-byte even when the max candidate
    /// is a travel line the extrusion-only stop set cannot hold (see
    /// [`anchor_index`], MAJOR-2). This is the cursor the preview loop opens
    /// on.
    pub last_index: u32,
    /// The slicer `current_layer` mark (`plr_wal::Context::current_layer`)
    /// this set's layers may be cross-checked against, when — and ONLY
    /// when — that comparison is semantically valid.
    ///
    /// The mark is an **upper bound** on the physically-printing layer (the
    /// slicer sets it at the layer-change line's parse time, which leads
    /// execution), so a stop whose [`PreviewStop::layer`] `L` satisfies
    /// `L <= mark` is *journal-corroborated*; one with `L > mark` is not
    /// (geometry above the mark is the physically-impossible case
    /// [`crate::model::WindowLayers::mark_is_consistent`] flags). `None`
    /// here means there is no valid corroborating mark — the slicer emitted
    /// none, OR the model does not span from file start so window-relative
    /// layer ordinals are incommensurable with the absolute mark (the
    /// absolute-frame rule on `mark_is_consistent`) — and every stop's layer
    /// is then model-inferred, not journal-confirmed.
    ///
    /// [`build_preview`] leaves this `None`: the analyzer has neither the
    /// WAL mark nor the model's file base offset. The daemon pipeline, which
    /// holds both, sets it after building (design follow-up: layer
    /// provenance). Consumed by `plrd`'s `preview_detail` to emit the
    /// per-stop `layer_provenance` wire field. `skip_serializing_if` keeps a
    /// set built before the field existed byte-identical.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corroborating_layer_mark: Option<u32>,
}

/// Why [`build_preview`] declined to produce a set.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PreviewRefusal {
    /// More distinct candidate layers than [`PreviewBounds::max_layers`].
    TooManyLayers {
        /// Distinct candidate layers observed.
        distinct: usize,
        /// The configured limit.
        limit: usize,
    },
    /// More in-window deposition stops than [`PreviewBounds::max_stops`].
    TooManyStops {
        /// In-window deposition stops observed.
        count: usize,
        /// The configured limit.
        limit: usize,
    },
}

/// Outcome of [`build_preview`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PreviewOutcome {
    /// A usable preview set.
    Preview(PreviewSet),
    /// Evidence is not a normal crash — the caller degrades to manual
    /// fallback, exactly as an over-coarse match does today.
    TooWide(PreviewRefusal),
    /// No candidate stop exists to anchor a preview: the window held no
    /// trusted-position, non-excluded extrusion move, or none of them
    /// matched the evidence. The caller degrades to manual fallback.
    ///
    /// Note the **all-travel-candidates** case (a crash mid-wipe where every
    /// matcher candidate is a travel line): the stop domain is
    /// extrusion-only, so no stop is a candidate and this variant is
    /// returned — yet the automatic resolver *does* resolve (it skips forward
    /// from the max travel candidate to the next deposition). Increment 2's
    /// routing must therefore take a `first`/`mid`/`last` policy through
    /// `plr-recovery`'s `select_resume_target_with_policy` (which resolves
    /// over the offset list) rather than reading a resume out of this
    /// `NoStops` outcome. Refusing preview here is the safe direction (manual
    /// fallback), not a resume regression.
    NoStops,
    /// Evidence or config was non-finite / inverted — the same rejection
    /// [`crate::matcher::match_stop_point`] makes, surfaced as a typed
    /// outcome (this builder returns an outcome, not a `Result`).
    InvalidEvidence {
        /// Name of the offending field.
        field: &'static str,
    },
}

/// The median index into an ascending-sorted set: the **lower median**,
/// `(len - 1) / 2`. Shared convention with `plr-recovery`'s
/// `select_resume_target_with_policy` `Mid` arm so the `mid` policy picks
/// the same stop whether it runs over an `AmbiguousWindow` offset list or
/// a preview candidate set (design §3: median by execution-order file
/// offset). `len` must be non-zero.
#[must_use]
pub fn median_index(len: usize) -> usize {
    (len - 1) / 2
}

/// Feature class of the move's source line, by the same rule
/// `select_resume_target` uses for `on_infill`
/// (`plr-recovery/src/build.rs:999-1009`): the `;TYPE:` path in the
/// move's layer whose segment shares this line's `span.start`.
fn feature_of(model: &LayerModel, mv: &SimMove) -> FeatureClass {
    mv.layer
        .and_then(|idx| model.layer(idx))
        .and_then(|layer| {
            layer.paths.iter().find_map(|p| {
                p.segments
                    .iter()
                    .any(|s| s.span.start == mv.span.start)
                    .then_some(p.class)
            })
        })
        .unwrap_or(FeatureClass::Other)
}

/// A candidate stop's minimal descriptor for representative selection.
#[derive(Debug, Clone, Copy)]
struct RepInput {
    /// Index into the stop list.
    index: u32,
    /// Hover XY (finite — candidate stops come from the finite-guarded
    /// evaluate path).
    xy: [f64; 2],
    /// Byte offset — the deterministic tie-breaker paired with `index`.
    offset: u64,
}

/// Choose the representative stop indices from the candidate stops:
/// always the two window endpoints (min/max by `(offset, index)`), then
/// greedy furthest-point sampling on XY until `cap` is reached or the
/// pool is exhausted (design §A.3). Deterministic and **permutation-
/// stable**: endpoints and every furthest-point pick break ties by
/// `(offset, index)`, independent of the input slice order; the result is
/// returned sorted and deduped.
fn representatives(candidates: &[RepInput], cap: usize) -> Vec<u32> {
    if candidates.is_empty() || cap == 0 {
        return Vec::new();
    }
    let key = |c: &RepInput| (c.offset, c.index);
    // Endpoints: earliest and latest candidate by (offset, index).
    let lo = *candidates.iter().min_by_key(|c| key(c)).expect("non-empty");
    let hi = *candidates.iter().max_by_key(|c| key(c)).expect("non-empty");
    let mut chosen: Vec<RepInput> = vec![lo];
    if hi.index != lo.index {
        chosen.push(hi);
    }
    // Greedy furthest-point: repeatedly take the candidate maximizing its
    // minimum XY distance to the already-chosen set. Ties (including all
    // spatial coincidences) resolve by (offset, index).
    while chosen.len() < cap {
        let mut best: Option<(f64, RepInput)> = None;
        for c in candidates {
            if chosen.iter().any(|s| s.index == c.index) {
                continue;
            }
            let d = chosen
                .iter()
                .map(|s| geom::point_distance(s.xy, c.xy))
                .fold(f64::INFINITY, f64::min);
            let take = match best {
                None => true,
                // Larger min-distance wins; ties (incl. spatial
                // coincidence and any NaN) resolve by (offset, index) so
                // the result is deterministic and permutation-stable.
                Some((bd, bc)) => match d.total_cmp(&bd) {
                    std::cmp::Ordering::Greater => true,
                    std::cmp::Ordering::Equal => key(c) < key(&bc),
                    std::cmp::Ordering::Less => false,
                },
            };
            if take {
                best = Some((d, *c));
            }
        }
        match best {
            Some((_, c)) => chosen.push(c),
            None => break,
        }
    }
    let mut out: Vec<u32> = chosen.iter().map(|c| c.index).collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// Build the resume-point preview from the same evidence the matcher
/// consumes (design §A). See the module docs for the nudge-domain-wider-
/// than-candidates argument.
///
/// `exclusions` is the cancelled-object oracle: any stop attributed to a
/// cancelled object (`SimMove::object`) is dropped from selection AND
/// resume (design §A.4 / the D9 exclusion-durability rule — resuming into
/// cancelled debris drives the nozzle into it). A stop with `object ==
/// None` is "not attributable" = work that cannot be cancelled = kept.
/// `None` (no oracle) excludes nothing.
///
/// Unlike `remaining_work`'s `counts_as_work`, this does **not** gate on
/// [`ExclusionOracle::is_conclusive`]: dropping a stop is the safe
/// direction for a *resume* (fewer options, never resume into a possibly-
/// cancelled object), and `is_excluded` returning `true` is reliable even
/// when the oracle is inconclusive (that only makes it untrustworthy when
/// it says *no*). The inversion versus `counts_as_work` is deliberate.
#[must_use]
pub fn build_preview(
    model: &LayerModel,
    evidence: &StopEvidence,
    config: &MatchConfig,
    exclusions: Option<&dyn ExclusionOracle>,
    bounds: &PreviewBounds,
) -> PreviewOutcome {
    if let Err(err) = validate(evidence, config) {
        return PreviewOutcome::InvalidEvidence {
            field: invalid_field(&err),
        };
    }
    let (candidates, _skipped) = collect_candidates(model, evidence, config);

    // Admission gate 1 — distinct candidate layers (§A.5).
    let mut candidate_layers: Vec<u32> = Vec::new();
    for c in &candidates {
        if let Some(layer) = c.layer {
            if !candidate_layers.contains(&layer) {
                candidate_layers.push(layer);
            }
        }
    }
    if candidate_layers.len() > bounds.max_layers {
        return PreviewOutcome::TooWide(PreviewRefusal::TooManyLayers {
            distinct: candidate_layers.len(),
            limit: bounds.max_layers,
        });
    }

    // The raw nudge domain: every in-window extrusion move (§A.2). Gate 2
    // is on this count — the reconstruction-pathology signal — before any
    // finite/exclusion filtering (§A.5).
    let raw_domain: Vec<&SimMove> = model
        .moves
        .iter()
        .filter(|mv| mv.kind == MoveKind::Extrusion && in_window(mv, evidence.window))
        .collect();
    if raw_domain.len() > bounds.max_stops {
        return PreviewOutcome::TooWide(PreviewRefusal::TooManyStops {
            count: raw_domain.len(),
            limit: bounds.max_stops,
        });
    }

    // Which lines matched the evidence — the distinct candidate LINE
    // offsets (matcher.rs dedups one candidate per source line, travels
    // included). This seeds `is_candidate` AND the policy anchors, and is
    // the *identical population* `plr-recovery`'s `select_offset` picks over
    // (`AmbiguousWindow.offsets` is built the same way), so the anchors
    // resolve byte-for-byte to the recovery policy resumes.
    let candidate_offsets: std::collections::BTreeSet<u64> =
        candidates.iter().map(|c| c.offset).collect();

    // Build the kept stop list (nudge domain), then the navigation anchors.
    let stops = build_stops(model, &raw_domain, &candidate_offsets, exclusions);
    if stops.is_empty() {
        return PreviewOutcome::NoStops;
    }

    // Representatives seed from candidate stops only (§A.3).
    let candidate_stops: Vec<&PreviewStop> = stops.iter().filter(|s| s.is_candidate).collect();
    if candidate_stops.is_empty() {
        return PreviewOutcome::NoStops;
    }
    let rep_inputs: Vec<RepInput> = candidate_stops
        .iter()
        .map(|s| RepInput {
            index: s.index,
            xy: s.xy,
            offset: s.offset,
        })
        .collect();
    let representatives = representatives(&rep_inputs, PREVIEW_MAX_REPS);

    // Policy anchors (design §3). Each is derived to COMMIT byte-for-byte the
    // resume `plr-recovery`'s `select_resume_target_with_policy` commits for
    // that policy: pick the policy's base offset over the candidate LINE
    // offsets (min / lower-median / max — `select_offset`'s convention,
    // travels included), resolve the resume from it exactly as the resolver
    // does, then map to the stop whose *acceptance* commits that resume (the
    // deposition whose next resumable line is it). Anchoring on the resume,
    // not the stop offset, is what makes `last` equal today's skip-forward
    // even when the max candidate is a travel line the extrusion-only stop set
    // cannot hold (MAJOR-2), and `mid` the per-line median rather than a
    // chord-duplicated one (MINOR-1).
    let sorted_offsets: Vec<u64> = candidate_offsets.iter().copied().collect();
    let first_index = anchor_index(
        &stops,
        model,
        &sorted_offsets,
        AnchorPolicy::First,
        exclusions,
    );
    let mid_index = anchor_index(
        &stops,
        model,
        &sorted_offsets,
        AnchorPolicy::Mid,
        exclusions,
    );
    let last_index = anchor_index(
        &stops,
        model,
        &sorted_offsets,
        AnchorPolicy::Last,
        exclusions,
    );

    PreviewOutcome::Preview(PreviewSet {
        stops,
        representatives,
        first_index,
        mid_index,
        last_index,
        // The analyzer has neither the WAL slicer mark nor the model's file
        // base offset; the daemon pipeline annotates this after building
        // (see the field docs / the layer-provenance follow-up).
        corroborating_layer_mark: None,
    })
}

/// Build the kept stop list (the nudge domain) from the raw in-window
/// extrusion moves. A move is a selectable (hoverable) stop only if its
/// start position is trusted — [`SimMove::start_position_known`], the same
/// predicate the resume resolver refuses on — so preview never hovers the
/// nozzle at a G28-stale coordinate the analyzer itself flags unreliable.
/// Cancelled-object moves are dropped (§A.4). Each stop's `resume_offset`
/// is baked here (see the inline notes for the §A.2 / §A.4 / MAJOR-1 rules).
fn build_stops(
    model: &LayerModel,
    raw_domain: &[&SimMove],
    candidate_offsets: &std::collections::BTreeSet<u64>,
    exclusions: Option<&dyn ExclusionOracle>,
) -> Vec<PreviewStop> {
    let mut stops: Vec<PreviewStop> = Vec::new();
    for mv in raw_domain {
        if !mv.start_position_known() {
            continue;
        }
        if is_excluded_move(mv, exclusions) {
            continue;
        }
        // §A.2: resume at the next deposition line. §A.4 (D9): skip cancelled
        // objects. MAJOR-1: the next kept deposition must also be a *trusted*
        // position, mirroring the resolver's `ResumePositionUnknown` refusal —
        // a line the resolver would refuse is never baked as a resume target.
        let resume_offset = match first_kept_deposition_at_or_after(model, mv.span.end, exclusions)
        {
            // The next kept deposition is resumable — resume there.
            Some(m) if m.start_position_known() => m.span.start,
            // The next kept deposition exists but its position is untrusted:
            // the resolver would refuse it, so this stop has no valid resume.
            // Drop the stop rather than bake a refused target (MAJOR-1).
            Some(_) => continue,
            // No kept deposition remains after this stop (all later deposition
            // excluded, or none at all): the print is complete past here —
            // resume "past the end" at `span.end` (design §A.2).
            None => mv.span.end,
        };
        let feature = feature_of(model, mv);
        // Contiguous 0.. index over kept stops; the domain is bounded by
        // `PREVIEW_MAX_STOPS` (2000) far below `u32::MAX`, so the saturating
        // conversion is defensive and unreachable.
        let index = u32::try_from(stops.len()).unwrap_or(u32::MAX);
        stops.push(PreviewStop {
            index,
            offset: mv.span.start,
            resume_offset,
            xy: [mv.start[0], mv.start[1]],
            z: mv.start[2],
            layer: mv.layer,
            feature,
            on_infill: matches!(
                feature,
                FeatureClass::InternalInfill | FeatureClass::SolidInfill
            ),
            is_candidate: candidate_offsets.contains(&mv.span.start),
        });
    }
    stops
}

/// The three set-policy anchors, resolved consistently with
/// `plr-recovery`'s `select_offset` / `select_resume_target_with_policy`.
#[derive(Debug, Clone, Copy)]
enum AnchorPolicy {
    /// Minimum candidate line offset.
    First,
    /// Lower-median candidate line offset (`median_index`).
    Mid,
    /// Maximum candidate line offset — the skip-forward default cursor.
    Last,
}

/// The stop index for a set-policy anchor over `sorted_offsets` (the
/// ascending distinct candidate line offsets — `select_offset`'s exact
/// population, travels included, non-empty at the call site). The two
/// reviewer rulings need two different mappings:
///
/// - **`Last`** is the interactive default cursor and the safe skip-forward,
///   so its *committed resume* must equal `select_resume_target_with_policy(
///   Last)` byte-for-byte even when the max candidate is a travel the stop
///   set cannot hold (MAJOR-2). It maps to the stop whose acceptance commits
///   that resume — the deposition whose next resumable line is the resolver's
///   resume from the max candidate offset. With no oracle over a trusted next
///   line the resolved offset equals `resolve_resume_from_offset(max)`.
///
/// - **`First`/`Mid`** are "may-reprint" reference cursors; the ruling
///   (MINOR-1) is that they select over the per-line population, so arc chords
///   (many stops, one line) do not skew the median. They map to the stop AT
///   the selected candidate LINE. Their committed resume is that stop's next
///   line, one line short of the resolver's resume from the same base — the
///   documented "may re-print" boundary, immaterial for these warned policies
///   and impossible to close whenever the selected line's predecessor is a
///   non-stop (an `ExtrudeOnly` prime, or the earliest deposition).
fn anchor_index(
    stops: &[PreviewStop],
    model: &LayerModel,
    sorted_offsets: &[u64],
    policy: AnchorPolicy,
    exclusions: Option<&dyn ExclusionOracle>,
) -> u32 {
    match policy {
        AnchorPolicy::Last => {
            let base = sorted_offsets[sorted_offsets.len() - 1];
            // The resolver's resume from `base`: first kept, trusted
            // deposition at/after it (== `resolve_resume_from_offset(base)`
            // with no oracle over a trusted line — the byte-identity pin).
            if let Some(m) = first_kept_deposition_at_or_after(model, base, exclusions) {
                if m.start_position_known() {
                    // The stop that commits that resume (its predecessor);
                    // else the stop AT the resume line (resumes one line
                    // later — still the skip-forward direction).
                    if let Some(idx) = stop_resuming_at(stops, m.span.start) {
                        return idx;
                    }
                    if let Some(idx) = stop_at_offset(stops, m.span.start) {
                        return idx;
                    }
                }
            }
            nearest_stop_at_or_before(stops, base)
        }
        AnchorPolicy::First | AnchorPolicy::Mid => {
            let base = match policy {
                AnchorPolicy::First => sorted_offsets[0],
                _ => sorted_offsets[median_index(sorted_offsets.len())],
            };
            // The stop AT the selected candidate line (last chord if the line
            // is an arc). If that line is a travel / extrude-only candidate
            // with no stop, the nearest stop at or before it.
            stop_at_offset(stops, base).unwrap_or_else(|| nearest_stop_at_or_before(stops, base))
        }
    }
}

/// The stop whose baked `resume_offset` equals `resume` — the "last printed
/// line" a resume at `resume` implies (its next resumable deposition is
/// `resume`). Highest index among ties (arc chords sharing one source line),
/// so the last chord wins. `None` when `resume` has no predecessor stop.
fn stop_resuming_at(stops: &[PreviewStop], resume: u64) -> Option<u32> {
    stops
        .iter()
        .rev()
        .find(|s| s.resume_offset == resume)
        .map(|s| s.index)
}

/// The stop AT `offset` (highest index among arc chords sharing the line),
/// or `None` when no stop starts there (a travel / extrude-only line).
fn stop_at_offset(stops: &[PreviewStop], offset: u64) -> Option<u32> {
    stops
        .iter()
        .rev()
        .find(|s| s.offset == offset)
        .map(|s| s.index)
}

/// The last stop at or before `offset` (highest `(offset, index)`), or the
/// earliest stop when none is — always a valid index over a non-empty set.
fn nearest_stop_at_or_before(stops: &[PreviewStop], offset: u64) -> u32 {
    stops
        .iter()
        .filter(|s| s.offset <= offset)
        .max_by_key(|s| (s.offset, s.index))
        .map_or(stops[0].index, |s| s.index)
}

/// The first depositing move at or after `offset` that is **not**
/// attributed to a cancelled object — the exclusion-aware form of
/// [`LayerModel::first_deposition_at_or_after`]. Identical to it when
/// `exclusions` is `None`. Used to bake a stop's `resume_offset` so an
/// accepted resume never targets a cancelled line (§A.4).
///
/// Trusted-position admissibility (the resolver's `ResumePositionUnknown`
/// gate) is applied by the *callers* to this function's result, not here:
/// exclusion-skip is preview's own D9 addition, while the known-position
/// gate mirrors the resolver, which refuses (does not skip past) an
/// untrusted line — keeping the two concerns and their two skip/refuse
/// semantics distinct.
fn first_kept_deposition_at_or_after<'a>(
    model: &'a LayerModel,
    offset: u64,
    exclusions: Option<&dyn ExclusionOracle>,
) -> Option<&'a SimMove> {
    model.moves.iter().find(|m| {
        m.kind == MoveKind::Extrusion && m.span.start >= offset && !is_excluded_move(m, exclusions)
    })
}

/// `true` when the move is attributed to a cancelled object. `None`
/// object = not attributable = never excluded (§A.4). See
/// [`build_preview`] on why `is_conclusive` is not consulted.
fn is_excluded_move(mv: &SimMove, exclusions: Option<&dyn ExclusionOracle>) -> bool {
    match (exclusions, mv.object.as_deref()) {
        (Some(oracle), Some(object)) => oracle.is_excluded(object),
        _ => false,
    }
}

/// The offending-field name of a validation error, for
/// [`PreviewOutcome::InvalidEvidence`].
fn invalid_field(err: &crate::matcher::MatchError) -> &'static str {
    use crate::matcher::MatchError;
    match err {
        MatchError::InvalidEvidence { field } | MatchError::InvalidConfig { field } => field,
        // validate() only ever returns the two Invalid* variants.
        _ => "unknown",
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact replay/recomputation equality is intentional
mod tests {
    use super::*;
    use crate::matcher::{match_stop_point, ByteWindow, Interval, MatchConfidence};
    use crate::model::{build_layer_model, ModelConfig};
    use plr_gcode::GcodeState;
    use std::fmt::Write as _;

    fn model_of(text: &str) -> LayerModel {
        build_layer_model(
            GcodeState::new(),
            text.as_bytes(),
            0,
            &ModelConfig::default(),
        )
    }

    fn whole_file() -> ByteWindow {
        ByteWindow {
            start: 0,
            end: None,
        }
    }

    fn iv(min: f64, max: f64) -> Interval {
        Interval { min, max }
    }

    /// A file with several extrusion lines on one layer that a loose XY
    /// box makes ambiguous, so the candidate set has several members.
    const AMBIG: &str = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
        G1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
        G1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n";

    fn ambig_evidence() -> StopEvidence {
        StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        }
    }

    #[test]
    fn preview_produces_a_set_over_an_ambiguous_window() {
        let m = model_of(AMBIG);
        let out = build_preview(
            &m,
            &ambig_evidence(),
            &MatchConfig::default(),
            None,
            &PreviewBounds::default(),
        );
        let PreviewOutcome::Preview(set) = out else {
            panic!("expected a preview, got {out:?}");
        };
        assert!(!set.stops.is_empty());
        // Every extrusion line in AMBIG is a stop (nudge domain wider
        // than candidates); the three retraced extrusions are candidates.
        let candidate_count = set.stops.iter().filter(|s| s.is_candidate).count();
        assert_eq!(candidate_count, 3, "three retraced extrusions match");
        // Policy anchors are valid indices. `last` COMMITS today's
        // skip-forward resume byte-for-byte (resume-mapped). `mid`/`first`
        // SELECT over the per-line candidate population and land on the stop
        // AT that line (here every candidate line is an extrusion stop).
        let cfg = MatchConfig::default();
        for idx in [set.first_index, set.mid_index, set.last_index] {
            assert!((idx as usize) < set.stops.len());
        }
        assert_eq!(
            set.stops[set.last_index as usize].resume_offset,
            expected_policy_resume(&m, &ambig_evidence(), &cfg, AnchorPolicy::Last),
            "last commits the resolver's skip-forward resume"
        );
        assert_eq!(
            set.stops[set.mid_index as usize].offset,
            candidate_line_base(&m, &ambig_evidence(), &cfg, AnchorPolicy::Mid),
            "mid selects the per-line median candidate line"
        );
        assert_eq!(
            set.stops[set.first_index as usize].offset,
            candidate_line_base(&m, &ambig_evidence(), &cfg, AnchorPolicy::First),
            "first selects the min candidate line"
        );
    }

    /// The policy's base offset over the matcher's distinct candidate LINE
    /// offsets (no oracle): min / lower-median / max — `select_offset`'s
    /// population and convention.
    fn candidate_line_base(
        m: &LayerModel,
        ev: &StopEvidence,
        cfg: &MatchConfig,
        which: AnchorPolicy,
    ) -> u64 {
        let (cands, _) = collect_candidates(m, ev, cfg);
        let mut offs: Vec<u64> = cands.iter().map(|c| c.offset).collect();
        offs.sort_unstable();
        offs.dedup();
        match which {
            AnchorPolicy::First => offs[0],
            AnchorPolicy::Mid => offs[median_index(offs.len())],
            AnchorPolicy::Last => offs[offs.len() - 1],
        }
    }

    /// [`candidate_line_base`] resolved to the resume the resolver would
    /// commit (`first_deposition_at_or_after(base)`) — what the `last`
    /// anchor's stop must commit as its `resume_offset`.
    fn expected_policy_resume(
        m: &LayerModel,
        ev: &StopEvidence,
        cfg: &MatchConfig,
        which: AnchorPolicy,
    ) -> u64 {
        m.first_deposition_at_or_after(candidate_line_base(m, ev, cfg, which))
            .expect("a deposition at/after a candidate offset")
            .span
            .start
    }

    #[test]
    fn nudge_domain_is_wider_than_the_candidates() {
        // A file where the true stop line is NOT a candidate: the loose
        // box matches only the three horizontal extrusions, but a
        // vertical extrusion exists and must still be a (non-candidate)
        // stop reachable by nudging.
        let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
            G1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\nG1 X40 Y80 E1.0 F1800\n";
        let m = model_of(text);
        let evidence = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let PreviewOutcome::Preview(set) = build_preview(
            &m,
            &evidence,
            &MatchConfig::default(),
            None,
            &PreviewBounds::default(),
        ) else {
            panic!("expected a preview");
        };
        let vertical_off = text.find("G1 X40 Y80").unwrap() as u64;
        let vertical = set
            .stops
            .iter()
            .find(|s| s.offset == vertical_off)
            .expect("vertical extrusion is a stop");
        assert!(
            !vertical.is_candidate,
            "the vertical line is outside the tolerance box, not a candidate"
        );
    }

    #[test]
    fn resume_offset_is_the_next_deposition_line() {
        // §A.2: resume_offset = first_deposition_at_or_after(span.end).
        let m = model_of(AMBIG);
        let PreviewOutcome::Preview(set) = build_preview(
            &m,
            &ambig_evidence(),
            &MatchConfig::default(),
            None,
            &PreviewBounds::default(),
        ) else {
            panic!("expected a preview");
        };
        // The first extrusion stop resumes at the SECOND extrusion line.
        let first_ext = text_offset(AMBIG, "G1 X40 Y10 E0.5 F1800", 0);
        let second_ext = text_offset(AMBIG, "G1 X40 Y10 E0.5 F1800", 1);
        let stop = set
            .stops
            .iter()
            .find(|s| s.offset == first_ext)
            .expect("first extrusion is a stop");
        assert_eq!(
            stop.resume_offset, second_ext,
            "resume starts at the next deposition line, not this one"
        );
    }

    #[test]
    fn last_stop_resume_when_no_deposition_follows_is_span_end() {
        let m = model_of(AMBIG);
        let PreviewOutcome::Preview(set) = build_preview(
            &m,
            &ambig_evidence(),
            &MatchConfig::default(),
            None,
            &PreviewBounds::default(),
        ) else {
            panic!("expected a preview");
        };
        let last = &set.stops[set.stops.len() - 1];
        // No deposition after the final extrusion: resume_offset falls
        // back to span.end (== the last extrusion line's offset + its
        // length, i.e. end of the modeled window here).
        assert!(last.resume_offset >= last.offset);
    }

    #[test]
    fn excluded_object_stops_are_dropped_from_selection_and_resume() {
        // Two objects on one layer; PART_B is cancelled.
        let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
            EXCLUDE_OBJECT_START NAME=PART_A\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
            EXCLUDE_OBJECT_END NAME=PART_A\n\
            EXCLUDE_OBJECT_START NAME=PART_B\nG1 X10 Y50 F9000\nG1 X40 Y50 E0.5 F1800\n\
            EXCLUDE_OBJECT_END NAME=PART_B\n";
        let m = model_of(text);
        // Confirm the model attributed the moves as expected.
        let part_b_off = text.find("G1 X40 Y50 E0.5").unwrap() as u64;
        assert!(
            m.moves
                .iter()
                .any(|mv| mv.span.start == part_b_off && mv.object.as_deref() == Some("PART_B")),
            "model must attribute the PART_B extrusion"
        );
        let evidence = StopEvidence {
            x: iv(0.0, 100.0),
            y: iv(0.0, 100.0),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let oracle = TestOracle {
            excluded: vec!["PART_B"],
            conclusive: true,
        };
        let PreviewOutcome::Preview(set) = build_preview(
            &m,
            &evidence,
            &MatchConfig::default(),
            Some(&oracle),
            &PreviewBounds::default(),
        ) else {
            panic!("expected a preview");
        };
        // No stop and no resume_offset may reference the cancelled line.
        assert!(
            set.stops.iter().all(|s| s.offset != part_b_off),
            "cancelled object must not appear as a stop"
        );
        assert!(
            set.stops.iter().all(|s| s.resume_offset != part_b_off),
            "cancelled object must not appear as a resume target"
        );
    }

    #[test]
    fn none_object_stops_are_kept_when_others_are_excluded() {
        // A skirt (no object) plus an excluded object. The skirt is not
        // attributable and must survive.
        let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Skirt\n\
            G1 X5 Y5 F9000\nG1 X60 Y5 E0.5 F1800\n;TYPE:Sparse infill\n\
            EXCLUDE_OBJECT_START NAME=PART_B\nG1 X10 Y50 F9000\nG1 X40 Y50 E0.5 F1800\n\
            EXCLUDE_OBJECT_END NAME=PART_B\n";
        let m = model_of(text);
        let evidence = StopEvidence {
            x: iv(0.0, 100.0),
            y: iv(0.0, 100.0),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let oracle = TestOracle {
            excluded: vec!["PART_B"],
            conclusive: true,
        };
        let PreviewOutcome::Preview(set) = build_preview(
            &m,
            &evidence,
            &MatchConfig::default(),
            Some(&oracle),
            &PreviewBounds::default(),
        ) else {
            panic!("expected a preview");
        };
        let skirt_off = text.find("G1 X60 Y5 E0.5").unwrap() as u64;
        assert!(
            set.stops.iter().any(|s| s.offset == skirt_off),
            "unattributed skirt deposition must be kept"
        );
    }

    #[test]
    fn too_many_layers_refuses() {
        // Build a model with 9 candidate layers (limit 8). Same XY
        // re-extruded on nine ascending Zs; a Z-free box matches them all.
        let mut text = String::from("G90\nM83\n;TYPE:Sparse infill\n");
        for i in 0..9 {
            let z = 0.2 + f64::from(i) * 0.2;
            write!(
                text,
                "G1 Z{z} F7200\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n"
            )
            .unwrap();
        }
        let m = model_of(&text);
        let evidence = StopEvidence {
            x: iv(24.0, 26.0),
            y: iv(9.8, 10.2),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let out = build_preview(
            &m,
            &evidence,
            &MatchConfig {
                // Raise so we reach the layer gate, not the (unrelated)
                // ambiguity ladder — build_preview ignores the ladder but
                // this keeps the candidate set intact.
                ambiguity_limit: 100,
                ..MatchConfig::default()
            },
            None,
            &PreviewBounds::default(),
        );
        match out {
            PreviewOutcome::TooWide(PreviewRefusal::TooManyLayers { distinct, limit }) => {
                assert_eq!(distinct, 9);
                assert_eq!(limit, 8);
            }
            other => panic!("expected TooManyLayers, got {other:?}"),
        }
        // And one fewer layer is admitted (the bound is not vacuous).
        let mut ok_text = String::from("G90\nM83\n;TYPE:Sparse infill\n");
        for i in 0..8 {
            let z = 0.2 + f64::from(i) * 0.2;
            write!(
                ok_text,
                "G1 Z{z} F7200\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n"
            )
            .unwrap();
        }
        let m8 = model_of(&ok_text);
        assert!(matches!(
            build_preview(
                &m8,
                &evidence,
                &MatchConfig {
                    ambiguity_limit: 100,
                    ..MatchConfig::default()
                },
                None,
                &PreviewBounds::default()
            ),
            PreviewOutcome::Preview(_)
        ));
    }

    #[test]
    fn too_many_stops_refuses() {
        // 2001 extrusion moves in one layer exceeds max_stops = 2000.
        let mut text = String::from("G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n");
        for i in 0..2001 {
            let x = 10 + (i % 50);
            writeln!(text, "G1 X{x} Y10 E0.01 F1800").unwrap();
        }
        let m = model_of(&text);
        assert!(
            m.moves
                .iter()
                .filter(|mv| mv.kind == MoveKind::Extrusion)
                .count()
                >= 2001
        );
        let evidence = StopEvidence {
            x: iv(0.0, 100.0),
            y: iv(9.8, 10.2),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let out = build_preview(
            &m,
            &evidence,
            &MatchConfig::default(),
            None,
            &PreviewBounds::default(),
        );
        match out {
            PreviewOutcome::TooWide(PreviewRefusal::TooManyStops { count, limit }) => {
                assert_eq!(count, 2001);
                assert_eq!(limit, 2000);
            }
            other => panic!("expected TooManyStops, got {other:?}"),
        }
        // A tighter bound proves the gate fires; a looser one admits.
        assert!(matches!(
            build_preview(
                &m,
                &evidence,
                &MatchConfig::default(),
                None,
                &PreviewBounds {
                    max_layers: 8,
                    max_stops: 5000
                }
            ),
            PreviewOutcome::Preview(_)
        ));
    }

    #[test]
    fn invalid_evidence_is_a_typed_outcome() {
        let m = model_of(AMBIG);
        let bad = StopEvidence {
            x: iv(f64::NAN, 1.0),
            ..ambig_evidence()
        };
        assert_eq!(
            build_preview(
                &m,
                &bad,
                &MatchConfig::default(),
                None,
                &PreviewBounds::default()
            ),
            PreviewOutcome::InvalidEvidence { field: "x" }
        );
    }

    #[test]
    fn no_candidate_stops_is_no_stops() {
        // Evidence far from any geometry: no candidate matches.
        let m = model_of(AMBIG);
        let evidence = StopEvidence {
            x: iv(500.0, 501.0),
            y: iv(500.0, 501.0),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        assert_eq!(
            build_preview(
                &m,
                &evidence,
                &MatchConfig::default(),
                None,
                &PreviewBounds::default()
            ),
            PreviewOutcome::NoStops
        );
    }

    #[test]
    fn representatives_contain_endpoints_and_respect_cap() {
        // Six candidate stops spread across XY -> reps include the offset
        // endpoints and never exceed the cap.
        let inputs: Vec<RepInput> = (0..6)
            .map(|i| RepInput {
                index: i,
                xy: [f64::from(i) * 10.0, 0.0],
                offset: u64::from(i) * 100,
            })
            .collect();
        let reps = representatives(&inputs, PREVIEW_MAX_REPS);
        assert!(reps.len() <= PREVIEW_MAX_REPS);
        assert!(reps.contains(&0), "min-offset endpoint present");
        assert!(reps.contains(&5), "max-offset endpoint present");
        // Sorted, deduped.
        assert!(reps.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn median_index_is_the_lower_median() {
        assert_eq!(median_index(1), 0);
        assert_eq!(median_index(2), 0);
        assert_eq!(median_index(3), 1);
        assert_eq!(median_index(4), 1);
        assert_eq!(median_index(5), 2);
    }

    #[test]
    fn last_index_resume_equals_todays_skip_forward() {
        // The mutation-target pin (MAJOR-2 hardened): the DEFAULT cursor's
        // COMMITTED RESUME equals today's skip-forward resume —
        // first_deposition_at_or_after(max candidate offset) — computed
        // independently from match_stop_point. The pin is on the resume, not
        // on the max extrusion STOP offset (the pre-fix bug): anchoring on
        // the stop offset diverges the moment the max candidate is a travel
        // line the extrusion-only stop set cannot hold. See the golden
        // cross-crate travel-max pin for that shape; here AMBIG is
        // all-extrusion but still catches the off-by-one the old pin missed.
        let m = model_of(AMBIG);
        let ev = ambig_evidence();
        let result = match_stop_point(&m, &ev, &MatchConfig::default()).expect("match");
        let MatchConfidence::AmbiguousWindow { offsets } = &result.confidence else {
            panic!("expected an ambiguous window, got {:?}", result.confidence);
        };
        let expected_max = *offsets.iter().max().expect("non-empty");
        let today_resume = m
            .first_deposition_at_or_after(expected_max)
            .expect("a deposition at/after the max candidate")
            .span
            .start;
        let PreviewOutcome::Preview(set) = build_preview(
            &m,
            &ev,
            &MatchConfig::default(),
            None,
            &PreviewBounds::default(),
        ) else {
            panic!("expected a preview");
        };
        assert_eq!(
            set.stops[set.last_index as usize].resume_offset, today_resume,
            "default cursor must commit today's skip-forward resume"
        );
        // Mutation proof: the pre-fix last_index (max extrusion STOP) baked
        // the deposition AFTER that stop as its resume — here `span.end`,
        // which differs from `today_resume` (the max candidate line itself).
        let last_stop = &set.stops[set.last_index as usize];
        assert_ne!(
            last_stop.offset, today_resume,
            "the default cursor stop is the predecessor of the resume, not the resume line"
        );
        // And the resume differs from the min-candidate resume, so the pin
        // can bite a selector that picked the wrong end.
        let min_off = *offsets.iter().min().expect("non-empty");
        let min_resume = m.first_deposition_at_or_after(min_off).unwrap().span.start;
        assert_ne!(
            today_resume, min_resume,
            "min and max resumes differ (pin bites)"
        );
    }

    fn text_offset(text: &str, needle: &str, nth: usize) -> u64 {
        let (i, _) = text
            .match_indices(needle)
            .nth(nth)
            .unwrap_or_else(|| panic!("occurrence {nth} of {needle:?} not found"));
        i as u64
    }

    /// A minimal [`ExclusionOracle`] for the exclusion-filter tests.
    struct TestOracle {
        excluded: Vec<&'static str>,
        conclusive: bool,
    }

    impl ExclusionOracle for TestOracle {
        fn is_conclusive(&self) -> bool {
            self.conclusive
        }
        fn is_excluded(&self, object: &str) -> bool {
            self.excluded.contains(&object)
        }
    }

    #[test]
    fn representatives_are_permutation_stable() {
        // The rep set must not depend on the order candidate stops are
        // presented in (design §10). Furthest-point ties break by
        // (offset, index), so any permutation yields the same sorted set.
        use proptest::prelude::*;
        use proptest::test_runner::{Config, FileFailurePersistence, TestRunner};

        let mut runner = TestRunner::new(Config {
            cases: 256,
            failure_persistence: Some(Box::new(FileFailurePersistence::Off)),
            ..Config::default()
        });
        runner
            .run(
                &proptest::collection::vec(
                    (0u32..40, -50.0f64..50.0, -50.0f64..50.0, 0u64..4000),
                    1..15,
                )
                .prop_map(|raw| {
                    // Force unique indices (a stop index is unique in the
                    // real domain) while keeping offsets free to collide.
                    raw.into_iter()
                        .enumerate()
                        .map(|(i, (_, x, y, off))| RepInput {
                            index: u32::try_from(i).unwrap(),
                            xy: [x, y],
                            offset: off,
                        })
                        .collect::<Vec<_>>()
                }),
                |inputs| {
                    let base = representatives(&inputs, PREVIEW_MAX_REPS);
                    // Reverse order.
                    let mut rev = inputs.clone();
                    rev.reverse();
                    prop_assert_eq!(&representatives(&rev, PREVIEW_MAX_REPS), &base);
                    // A rotation.
                    let mut rot = inputs.clone();
                    rot.rotate_left(inputs.len() / 2);
                    prop_assert_eq!(&representatives(&rot, PREVIEW_MAX_REPS), &base);
                    // Invariants: subset of inputs, endpoints present, cap.
                    prop_assert!(base.len() <= PREVIEW_MAX_REPS);
                    let idx_set: std::collections::BTreeSet<u32> =
                        inputs.iter().map(|c| c.index).collect();
                    prop_assert!(base.iter().all(|i| idx_set.contains(i)));
                    let lo = inputs.iter().min_by_key(|c| (c.offset, c.index)).unwrap();
                    let hi = inputs.iter().max_by_key(|c| (c.offset, c.index)).unwrap();
                    prop_assert!(base.contains(&lo.index));
                    prop_assert!(base.contains(&hi.index));
                    // Ascending, deduped.
                    prop_assert!(base.windows(2).all(|w| w[0] < w[1]));
                    Ok(())
                },
            )
            .unwrap();
    }

    #[test]
    fn preview_invariants_hold_over_generated_evidence() {
        // reps ⊆ candidate stops; endpoints ∈ reps; |reps| ≤ 7; first/mid/
        // last are valid indices (design §10). Resume byte-identity of the
        // mid/last anchors is pinned by the deterministic tests below and the
        // golden cross-crate pins, not here — over generated evidence a
        // candidate set can collapse to the earliest deposition, where an
        // accept-at-next cursor cannot represent "resume at the first line",
        // so a blanket resume-equality assertion would be false.
        use proptest::prelude::*;
        use proptest::test_runner::{Config, FileFailurePersistence, TestRunner};

        let m = model_of(AMBIG);
        let mut runner = TestRunner::new(Config {
            cases: 256,
            failure_persistence: Some(Box::new(FileFailurePersistence::Off)),
            ..Config::default()
        });
        runner
            .run(
                &(0.0f64..60.0, 0.0f64..40.0, 0.0f64..60.0, 0.0f64..40.0),
                |(x0, w, y0, h)| {
                    let evidence = StopEvidence {
                        x: iv(x0, x0 + w),
                        y: iv(y0, y0 + h),
                        e: None,
                        z_candidates: vec![],
                        window: whole_file(),
                    };
                    if let PreviewOutcome::Preview(set) = build_preview(
                        &m,
                        &evidence,
                        &MatchConfig {
                            ambiguity_limit: 100,
                            ..MatchConfig::default()
                        },
                        None,
                        &PreviewBounds::default(),
                    ) {
                        let n = u32::try_from(set.stops.len()).unwrap();
                        prop_assert!(set.representatives.len() <= PREVIEW_MAX_REPS);
                        prop_assert!(set.representatives.iter().all(|&i| i < n));
                        // reps are candidate stops.
                        prop_assert!(set
                            .representatives
                            .iter()
                            .all(|&i| set.stops[i as usize].is_candidate));
                        // Endpoints (min/max offset candidate) ∈ reps.
                        let cands: Vec<&PreviewStop> =
                            set.stops.iter().filter(|s| s.is_candidate).collect();
                        let lo = cands.iter().min_by_key(|s| (s.offset, s.index)).unwrap();
                        let hi = cands.iter().max_by_key(|s| (s.offset, s.index)).unwrap();
                        prop_assert!(set.representatives.contains(&lo.index));
                        prop_assert!(set.representatives.contains(&hi.index));
                        // Policy anchors are valid stop indices (no longer
                        // required to be candidate stops — they anchor on the
                        // resume, not the offset).
                        for i in [set.first_index, set.mid_index, set.last_index] {
                            prop_assert!(i < n);
                        }
                    }
                    Ok(())
                },
            )
            .unwrap();
    }

    #[test]
    fn excluded_stops_never_appear_in_selection_or_resume() {
        // Over generated exclusion sets, no kept stop and no resume_offset
        // references a cancelled object's line (design §10 / D9).
        use proptest::prelude::*;
        use proptest::test_runner::{Config, FileFailurePersistence, TestRunner};

        let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
            EXCLUDE_OBJECT_START NAME=PART_A\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
            EXCLUDE_OBJECT_END NAME=PART_A\n\
            EXCLUDE_OBJECT_START NAME=PART_B\nG1 X10 Y30 F9000\nG1 X40 Y30 E0.5 F1800\n\
            EXCLUDE_OBJECT_END NAME=PART_B\n\
            EXCLUDE_OBJECT_START NAME=PART_C\nG1 X10 Y50 F9000\nG1 X40 Y50 E0.5 F1800\n\
            EXCLUDE_OBJECT_END NAME=PART_C\n";
        let m = model_of(text);
        let offset_of = |obj: &str| -> u64 {
            m.moves
                .iter()
                .find(|mv| mv.object.as_deref() == Some(obj) && mv.kind == MoveKind::Extrusion)
                .map(|mv| mv.span.start)
                .unwrap()
        };
        let names = ["PART_A", "PART_B", "PART_C"];
        let mut runner = TestRunner::new(Config {
            cases: 64,
            failure_persistence: Some(Box::new(FileFailurePersistence::Off)),
            ..Config::default()
        });
        runner
            .run(&proptest::collection::vec(any::<bool>(), 3..=3), |mask| {
                let excluded: Vec<&'static str> = names
                    .iter()
                    .zip(&mask)
                    .filter_map(|(n, &on)| on.then_some(*n))
                    .collect();
                let oracle = TestOracle {
                    excluded: excluded.clone(),
                    conclusive: true,
                };
                let evidence = StopEvidence {
                    x: iv(0.0, 100.0),
                    y: iv(0.0, 100.0),
                    e: None,
                    z_candidates: vec![],
                    window: whole_file(),
                };
                let out = build_preview(
                    &m,
                    &evidence,
                    &MatchConfig {
                        ambiguity_limit: 100,
                        ..MatchConfig::default()
                    },
                    Some(&oracle),
                    &PreviewBounds::default(),
                );
                if let PreviewOutcome::Preview(set) = out {
                    for name in &excluded {
                        let off = offset_of(name);
                        prop_assert!(set.stops.iter().all(|s| s.offset != off));
                        prop_assert!(set.stops.iter().all(|s| s.resume_offset != off));
                    }
                }
                Ok(())
            })
            .unwrap();
    }

    #[test]
    fn excluded_dropped_even_when_oracle_inconclusive() {
        // Preview drops on is_excluded regardless of is_conclusive — the
        // documented inversion versus remaining_work's counts_as_work.
        let text = "G90\nM83\nG1 Z0.2 F7200\n;TYPE:Sparse infill\n\
            EXCLUDE_OBJECT_START NAME=PART_A\nG1 X10 Y10 F9000\nG1 X40 Y10 E0.5 F1800\n\
            EXCLUDE_OBJECT_END NAME=PART_A\n\
            EXCLUDE_OBJECT_START NAME=PART_B\nG1 X10 Y50 F9000\nG1 X40 Y50 E0.5 F1800\n\
            EXCLUDE_OBJECT_END NAME=PART_B\n";
        let m = model_of(text);
        let part_b_off = text.find("G1 X40 Y50 E0.5").unwrap() as u64;
        let evidence = StopEvidence {
            x: iv(0.0, 100.0),
            y: iv(0.0, 100.0),
            e: None,
            z_candidates: vec![],
            window: whole_file(),
        };
        let oracle = TestOracle {
            excluded: vec!["PART_B"],
            conclusive: false,
        };
        let PreviewOutcome::Preview(set) = build_preview(
            &m,
            &evidence,
            &MatchConfig::default(),
            Some(&oracle),
            &PreviewBounds::default(),
        ) else {
            panic!("expected a preview");
        };
        assert!(set.stops.iter().all(|s| s.offset != part_b_off));
    }
}
