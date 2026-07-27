# Design: interactive resume-point preview + nudge

Status: **DESIGN — not ratified, no code written.** Branch `feat/resume-preview`
off `main` (`0872dc6`). Author: principal-directed design pass. This document
is the thing to ratify; implementation waits on the principal's ruling on the
items marked **PROPOSED** and the two **CONFLICT** notes in §9.

All file:line citations are against `main @ 0872dc6`.

---

## 0. What is settled (the operator's ruling, designed to — not re-litigated)

Real crashes reconstruct to a 2.5–4.0 s stop window → ~500–1,100 candidate
resume lines over 1–4 layers → the matcher returns coarse and recovery falls
back to manual. Per-line automatic matching at real evidence widths is
unachievable (measured, settled). The ruling turns the wall into a UX:

1. **Candidate preview** — lift, travel to a candidate XY, hold at a safe
   standoff above the printed geometry (never contact), ask accept / next /
   previous. Step through **representatives** (window endpoints + spatial
   cluster reps), 3–7 stops, never every line.
2. **Nudge** — from a representative, step the hover point ±1 (fine) / ±10
   (coarse) deposition lines ALONG THE TOOLPATH, landing on deposition-line
   endpoints only (skip travels). The part records where printing stopped (a
   ragged extrusion edge); the operator aligns to it. Semantics: operator picks
   "the last line that printed"; resume starts at the NEXT line.
3. **`resume_candidate_policy`** = `first` / `mid` / `last` / `ask`; `ask`
   (preview) is the default. `last` = skip-forward, safe automatic. `first`
   re-prints possibly-existing geometry — kept, warned plainly. Every prompt
   shows the current line/byte offset as it changes (adjacent lines <1 mm apart;
   feedback must not rely on visible motion).
4. **Console-command parity** for everything; dialogs are the enhancement, the
   bare console is the floor (standing project rule).

---

## 1. The gap in the current code (with citations)

**Candidates already exist; the coarse paths throw them away.** The matcher
evaluates every consistent line into `MatchResult.candidates`
(`plr-analyzer/src/matcher.rs:441-450`), each `MatchCandidate` carrying exactly
what preview needs — `offset`, `span`, `position:[f64;4]`, `layer`, `kind`,
`e_agreement`, `xy_distance` (`matcher.rs:159-184`). The confidence ladder
(`matcher.rs:451-506`) then classifies by count:

- `UniqueLine` / `AmbiguousWindow` (≤ `ambiguity_limit`, default 8) → `Ok`,
  candidates preserved — the safe automatic paths.
- `> limit`, one layer → `LayerOnly` — **line candidates discarded**, only the
  layer survives (`matcher.rs:499-500`).
- `> limit`, several layers → `MatchError::Inconclusive` (`matcher.rs:242-247`,
  `501-503`) — the **whole `MatchResult` is an `Err`**, candidates gone.

`select_resume_target` (`plr-recovery/src/build.rs:977-1016`) then requires a
line offset: `AmbiguousWindow` already picks `offsets.iter().max()`
(`build.rs:983-987`) — **that IS `last` / skip-forward, today** — while
`LayerOnly` returns `FallbackReason::MatchTooCoarse` (`build.rs:988-990`) and the
`Inconclusive` `Err` never reaches it: the pipeline converts it to
`ManualFallback` at the match call (`plrd/src/pipeline.rs:587-590`).

**So the raw material for preview is computed on every recovery and dropped at
exactly the two outcomes the operator wants a preview for.** The feature is
mostly a matter of *not discarding* it, plus a live motion loop.

**The confirm machinery exists but is binary and one-shot.** `ConfirmKind`
(`plrd/src/executor.rs:187-208`) has three variants; `ConfirmAnswer` is
`Continue`/`Abort`/`TimedOut` (`executor.rs:246-268`); `ask` is a single
question that resolves to proceed-or-abort (`executor.rs:1000-1038`). The
`Phase::ZConfirmStandoff` pause (`plan.rs:179-197`, driven at
`executor.rs:850-858`) is the exact precedent for preview: it lifts to a
rail-clamped standoff that **cannot descend** (`RuntimeComputation::ParkZ` →
`park_z_at`, `plan.rs:615-651`) and pauses AFTER the lift verifies. Preview is
this, but a *loop* of move→ask→move with a richer answer set.

**The socket + plugin already model a pause loop.** `SocketConfirmer`
(`ctrlsock.rs:258-298`) mints a fresh token per `confirm()`, single-flights on
`session.outstanding` (`cmd_recover_confirm`, `ctrlsock.rs:1146-1194`), and
`report_pause` renders `awaiting_confirmation` (`ctrlsock.rs:1248-1292`). The
plugin's guarded state machine puts `running` and `awaiting_confirmation` at the
**same alarm level** precisely so "the confirm loop moves between them freely in
both directions (pause → answer → pause)" (`recovery.py:178-188`). **Preview
repositioning needs no new plugin state** — only new answer commands and a wider
answer vocabulary.

---

## 2. Decision A — the analyzer/preview surface

### A.1 Do NOT overload `MatchConfidence`; add a parallel builder

The matcher's confidence ladder is a **width gate** and is documented as
structurally blind to incompleteness (architecture memory, CRITICAL FINDING
2026-07-24: a truncated window looks *more* confident). Weakening or forking it
risks the automatic paths. Keep `match_stop_point` **byte-identical**; the
automatic paths (`UniqueLine`, small `AmbiguousWindow`) stay exactly as they are.

Add a **parallel** builder in a new module `plr-analyzer/src/preview.rs`:

```
pub fn build_preview(
    model: &LayerModel,
    evidence: &StopEvidence,
    config: &MatchConfig,
    excluded: &ExcludedSet,      // cancelled objects (upper-cased names)
    bounds: &PreviewBounds,
) -> PreviewOutcome;
```

To avoid duplicating the evaluate loop, **refactor the candidate collection out
of `match_stop_point`** into a shared `fn collect_candidates(model, evidence,
config) -> (Vec<MatchCandidate>, usize /*skipped_unknown*/)` (the body of
`matcher.rs:416-450`). `match_stop_point` calls it then runs the ladder;
`build_preview` calls it then builds the preview set. One evaluate path, two
consumers — the ninth/eighth-corollary discipline (no second, subtly-different
predicate).

### A.2 The nudge domain is DELIBERATELY WIDER than the candidate set

This is the load-bearing safety argument and the answer to the reviewer's
sharpest attack ("preview lets the operator confidently accept a wrong line from
a set that never contained the truth"). The matcher's candidate set is gated by
XY/E/Z tolerance; the true stop can sit just outside `xy_tolerance`. So:

> **The selectable nudge stops = every in-window `MoveKind::Extrusion` move in
> `model.moves`** (`model.rs:149-156`, `first_deposition_at_or_after` precedent
> `model.rs:364-368`), not just the matched candidates. Every one is a valid
> "last printed line" by construction, and the physical ragged edge is always
> reachable by nudging even when it fell outside the matcher's tolerance box.

The matched candidates are a *labelled subset* used only to seed the
representatives (§A.3). `PreviewStop`:

```
pub struct PreviewStop {
    pub index: u32,          // position in the ordered stop list
    pub offset: u64,         // this deposition line's byte offset (shown to operator)
    pub resume_offset: u64,  // where a resume STARTS if this stop is accepted:
                             //   first_deposition_at_or_after(this.span.end)
                             //   -> "resume at the NEXT line" (the ruling's semantics)
    pub xy: [f64; 2],        // hover XY = this move's start (Klipper-internal frame)
    pub z: f64,              // deposition Z at this stop
    pub layer: Option<u32>,
    pub feature: FeatureClass,   // for the prompt (infill/wall/surface/...)
    pub on_infill: bool,         // reuse select_resume_target's rule (build.rs:999-1009)
    pub is_candidate: bool,      // matched the evidence (seeds representatives)
}
```

`resume_offset` is baked at build time so the committed skip-forward is identical
arithmetic to today's `first_deposition_at_or_after` and can be regression-pinned.

### A.3 Representatives = window endpoints + spatial cluster reps (in the analyzer)

```
pub struct PreviewSet {
    pub stops: Vec<PreviewStop>,       // the full nudge domain, execution order
    pub representatives: Vec<u32>,     // indices INTO stops; 3..=7; ascending
    pub first_index: u32,              // policy `first`  = min-offset candidate stop
    pub mid_index: u32,                // policy `mid`    = median-offset candidate stop
    pub last_index: u32,              // policy `last`   = max-offset candidate stop == today's skip-forward
}
```

`representatives` is computed **from the candidate stops** (`is_candidate`), never
the whole domain:

1. Always include the earliest and the latest candidate stop (the window
   endpoints — the ruling's explicit anchors).
2. Cluster the remaining candidate stops by XY (proposed: greedy furthest-point /
   k-means-lite over `stop.xy`, `k` chosen so total reps ≤ 7), pick the
   candidate nearest each cluster centroid, map it to its stop index.
3. Dedup, sort ascending, clamp to ≤ 7.

All in `preview.rs`, pure logic, native-testable (proptest: reps are always a
subset of stops, always contain the endpoints, never exceed the cap, and are
stable under input permutation). Representatives sharing the stop coordinate
system means **next/prev jumps between representatives and nudge steps ±1/±10
within `stops` use one index space** — no dual bookkeeping.

### A.4 Exclusion + structural filtering applied to every stop

`SimMove.object` (`model.rs:176-188`) already attributes each move to its
`exclude_object` bracket. `build_preview` **drops any stop attributed to a
cancelled object** — from selection AND (because `resume_offset` is a property of
the stop) from the resume itself. This is the D9 / exclusion-durability rule
(operator-UX memory: cancelled objects stay cancelled through recovery, SAFETY
not waste — resuming into cancelled debris drives the nozzle into it). Stops with
`object == None` are "not attributable" = work that cannot be cancelled = kept
(`model.rs:181-183`).

Structural-contact declines (`plr-analyzer` structure machinery) gate *contact*
(drag/tap force) and do **not** apply to a no-force hover — but lateral travel at
the hover plane must clear geometry, which §E.1's single-plane hover guarantees
by construction (plane ≥ every stop's Z). No per-stop structural filter is
needed; stated so a reviewer does not read its absence as an oversight.

### A.5 Bounds — when preview still refuses (**PROPOSED**)

`build_preview` returns `PreviewOutcome::TooWide` (→ `ManualFallback`, exactly as
today) when the evidence is not a normal crash:

- distinct layers among candidates > `PREVIEW_MAX_LAYERS` — **PROPOSED 8**. A
  tall spread means Z evidence cannot even say which layer; the physical-edge
  trick cannot disambiguate across many layers.
- in-window deposition stops > `PREVIEW_MAX_STOPS` — **PROPOSED 2000**. A window
  this large is a reconstruction pathology (the pre-epoch-fix 102,000 s bug
  shape), not a 2.5–4 s crash.

Note these gate *admission to preview*, not the UX size — representatives keep the
operator to 3–7 stops regardless. The candidate COUNT itself (500–1,100 real,
~50–300 post-epoch-fix per the operator-UX memory) is **not** a refusal reason;
serializing ~2000 `PreviewStop`s is tens of KB. **Principal must rule the two
numbers.** See CONFLICT-1 (§9) on the 500–1,100 headline vs any count cap.

---

## 3. Decision B — resume-point selection & policy

`resume_candidate_policy` selects *which stop becomes the resume point*:

| policy | stop chosen | safety | UX |
|---|---|---|---|
| `last` | `last_index` (max offset) | **skip-forward, safe** — never double-prints; bounded sub-line void | automatic, no prompt |
| `first`| `first_index` (min offset) | **re-prints geometry that may exist** — nozzle plows the existing wall | automatic + LOUD warning |
| `mid`  | `mid_index` (median offset) | may re-print (resumes before true stop) | automatic + warning |
| `ask`  | operator picks via preview | operator-chosen; nudge domain wider than candidates | interactive (default) |

- **`mid` across layers (PROPOSED rule):** the median by **execution-order file
  offset** among candidate stops — a single well-defined stop even when the set
  spans layers. It is *not* geometric-middle and *not* middle-layer. Documented as
  "may re-print" and warned like `first`. (`mid` is only meaningful because the
  candidate set is ordered; stated so it isn't quietly defined as middle-layer,
  which would be ambiguous when a layer has thousands of moves.)
- Policy applies wherever a *set* exists: `AmbiguousWindow` (today already `last`)
  and the new preview set. `UniqueLine` ignores policy (one line). This makes
  `first`/`mid` reach the small-ambiguous case too, which the operator's ruling
  implies ("a setting to pick first/mid/last/ask").
- **Regression pin:** `last_index` MUST equal today's
  `AmbiguousWindow → offsets.max()` / `first_deposition_at_or_after` result
  byte-for-byte, so the default-safe automatic path is unchanged. A mutation-proof
  test (mutate the selector, watch the pin bite) — not a `2.0 == 2.0` tautology
  (fourth corollary).

Selection lives in `plr-recovery` (a plan decision): extend `select_resume_target`
to take the policy and, for a set, choose first/mid/last; `ask` routes to the
preview plan (§C). Non-`ask` policies produce a **normal plan** resuming at the
chosen offset — no preview UI, no motion loop — so a headless / dialog-less setup
that sets `last` gets today's flow with a real resume point instead of manual
fallback. That is a strict win even before the preview UI exists.

---

## 4. Decision C — recovery-file late-binding (the structural tension)

**The tension, stated plainly.** The generated recovery file's verbatim tail
begins at `resume_offset` and its entry moves target the resume XY
(`RecoveryPlan.resume_offset` `plan.rs:956`, `RecoveryFileSpec`
`plan.rs:963-969`; generated in `finalize_recovery_file` at pipeline/dry-run time
`pipeline.rs:643-651`). Under `ask`, **the resume point is not known until the
operator accepts at execute time** — so the file cannot be finalized at dry-run.
Dry-run must still preview and must NOT write (`recover.rs:265-274`).

**Resolution (recommended): inject a `RecoveryFileWriter`, late-bind on Accept.**

- The plan carries the `PreviewSet` and a `RecoveryFileSpec` **template**
  (everything except the offset-dependent tail + resume-XY entry move).
- Dry-run renders its preview against the **`last` stop** (the skip-forward
  default), and tells the operator so: "file preview shown for the skip-forward
  point; the actual file is generated from the point you accept."
- Execute path injects a `RecoveryFileWriter` trait — the same dependency-injection
  shape as `Confirmer` / `FrameGuard` / `Exclusivity`
  (`executor.rs:276-319`). Its closure (built in `recover.rs`, capturing the
  original file bytes + spec) does `build_recovery_file(spec.with_resume(chosen))`
  and writes it. Dry-run injects a **no-op writer** — so "dry-run cannot write" is
  a *type* fact (the writer it is handed has no filesystem capability), not a
  discipline (twelfth corollary: remove the capability, don't document its
  non-use).
- On preview **Accept**, the executor calls the writer with the chosen
  `PreviewStop`, THEN proceeds to the mesh/`M23`/`M24` steps that select the
  now-materialised file. The entry move recomputes from `chosen.xy`.

This localizes the late-binding to one injected seam and keeps every existing
invariant (dry-run silent; file written once, on execute, after the operator
commits). It is the **main structural change** and the first thing the reviewer
should attack. Alternative considered and rejected: pre-generate a file per
representative — nudge produces arbitrary offsets, so it cannot cover the domain.

---

## 5. Decision D — the motion protocol (reposition-between-answers)

### D.1 New confirm kind + answer vocabulary

```
// executor.rs
enum ConfirmKind { Diagnosis, ZHeight, StepDebug, Preview }   // + Preview

enum PreviewAnswer {                 // NOT folded into ConfirmAnswer:
    Accept,                          //   the binary `ask` stays binary
    NextRep, PrevRep,                //   jump to next/prev representative
    Nudge(i32),                      //   +1/-1 (fine), +10/-10 (coarse) along stops
    Abort,
    TimedOut,                        //   walk-away; recorded distinct, treated as Abort
}
```

`ConfirmAnswer` (`executor.rs:246-268`) and the binary `ask` are untouched.
Preview gets its own driver (below) returning the terminal choice.

### D.2 The executor preview loop

Inserted at a new `Phase::ResumePreview` (numbered 7a″, immediately after
`TrueZDeclare`/`ZConfirmStandoff`, i.e. after the frame is fully declared and the
nozzle is near the part). Present iff `plan.preview.is_some()`. Pseudocode of the
new driver, mirroring the `ZConfirmStandoff` block (`executor.rs:850-858`) but
looping:

```
let set = &plan.preview.stops;
let mut cursor = plan.preview.default_index;   // = last_index (skip-forward)
let mut rep_ptr = index_of_nearest_rep(cursor);
// One-time: lift to the single hover plane (never descends; §E.1)
reassert_exclusivity(...)?;                    // barrier
run(G1 Z{hover_plane} F{z_speed})?;            // ParkZ-clamped, verified
loop {
    reassert_exclusivity(...)?;                // barrier BEFORE every reposition send
    run(G1 X{set[cursor].xy.x} Y{...} F{travel})?;   // verify position settles
    let point = preview_point(&set[cursor], cursor, set.len());  // offset/line/xy/z/feature
    match ask_preview(point, confirm_deadline, confirmer, transcript).await {
        Accept   => break Ok(set[cursor].clone()),   // -> writer, then M23/M24 steps
        NextRep  => { rep_ptr = min(rep_ptr+1, reps.len()-1); cursor = reps[rep_ptr]; }
        PrevRep  => { rep_ptr = rep_ptr.saturating_sub(1);    cursor = reps[rep_ptr]; }
        Nudge(d) => { cursor = clamp(cursor as i64 + d, 0, set.len()-1); rep_ptr = nearest_rep(cursor); }
        Abort | TimedOut => return finish_abort(...),  // frame_invalid = true (past ShiftedFrame)
    }
}
```

Key properties, each a reviewer target:

- **Exclusivity re-taken before EVERY reposition send** (`reassert_exclusivity`
  `executor.rs:888-905`). Each answer is a human-time gap up to
  `confirm_timeout_s`; the eleventh corollary is explicit — "a barrier must be
  re-taken at every point where control returns to another party — after every
  pause, gate, or human answer." A preview that reasserts once at entry is the
  bug.
- **Never descends.** The one Z motion is the entry lift to the hover plane via
  `park_z_at` (`plan.rs:630-651`), which clamps `.max(current_z)`. Repositions are
  XY-only at the fixed plane. There is no per-stop Z move, so no stop's Z can
  produce a descent (a reviewer will try to craft one — the type prevents it).
- **Deadline per pause.** Each `ask_preview` stamps and enforces
  `confirm_deadline` (`plan.confirm_timeout_s` else `DEFAULT_CONFIRM_TIMEOUT` =
  10 min, `executor.rs:162`, `762`), fresh per question. Walk-away → one pause
  times out → clean abort. The operator gets a full budget per interaction, not a
  shrinking global one.
- **Frame handling.** `ResumePreview` is after `Phase::ShiftedFrame`
  (`executor.rs:744-746`, `824`), so an abort/timeout during preview sets
  `frame_invalid = true`, writes the frame-invalidation marker, and forces a fresh
  dry run before any resume. Correct: the nozzle has been driven around the part;
  Z-frame trust must be re-established. Documented for the operator.
- **`M112` during preview.** M112 → klippy shutdown → the plugin's shutdown
  handler fires `recover_confirm ... abort` from its detached thread
  (`recovery.py:1440-1492`); the parked pause resolves to `Abort`; frame invalid.
  The plugin's existing "refuse `continue` when shutdown, allow `abort`"
  (`recovery.py:758-765`) **generalizes to: refuse any repositioning answer
  (accept/next/prev/nudge) when `printer.is_shutdown()`, allow only abort** — you
  cannot reposition a shut-down machine.

### D.3 Socket + single-flight (mostly free)

Each `ask_preview` pause flows through `SocketConfirmer::confirm`
(`ctrlsock.rs:267-297`) → fresh token → `report_pause` renders
`awaiting_confirmation` with a new `confirm_kind: "preview"` and a `detail`
carrying the current stop (offset/line/xy/z/feature/rep-position). `recover_confirm`
(`ctrlsock.rs:1146-1194`) widens its answer parse (`1150-1159`) from
`continue`/`abort` to the preview vocabulary; a non-terminal answer resolves the
current pause and the loop raises the *next* pause (a new token), which
`drive_session` (`ctrlsock.rs:1198-1244`) reports exactly as it already reports a
second confirm-point. Double-click on a nudge → stale token → `unknown-token`
(`1175-1183`), harmless. **The single-flight token machinery handles preview
repositions with no change beyond the vocabulary widening.**

---

## 6. Decision E — safety

### E.1 Standoff / hover plane

Reuse the `ZConfirmStandoff` derivation: rail-clamped `ParkZ`
(`step_z_confirm_standoff` `build.rs:1890-1910`, `entry_hop` default 1.0 mm
`build.rs:560`). **PROPOSED: one hover plane per preview session**, computed at
entry as `min(z_max, max(stop.z for stop in stops) + preview_standoff)` and never
lowered. All XY repositions happen at that single Z. Rationale: different stops
sit on different layers; a single plane ≥ every stop's Z guarantees lateral
travel clears all modeled geometry and makes "never descend" a structural fact
rather than a per-move check. The visible gap over a low first layer is larger
than `preview_standoff`, but **the offset readout — not the gap — is the
alignment feedback** (adjacent lines <1 mm; the ruling forbids relying on visible
motion anyway). New key **`preview_standoff` (PROPOSED, default = `entry_hop`)**.
Post-probe the nozzle is LOW, so preview entry is a **lift**, consistent with
never-descend. (Alternative — per-stop standoff for a tighter visual — is a
PROPOSED refinement; it costs a Z move per reposition and a descent-guard on each.
Recommend against for v1.)

### E.2 Temperature / oozing during deliberation

Preview runs post-probe/post-TrueZ and can last **minutes** (operator studying a
ragged edge). A hot nozzle hovering over the part oozes onto it. The generated
recovery file already does the print-temp reheat + purge behind its heating gate
(operator-UX memory, `feat/recovery-file`), so **preview does not need a hot
nozzle**. **PROPOSED: on preview entry, command `M104 S{preview_nozzle_temp}`
(default 0)** so the nozzle cools and cannot ooze during deliberation; the standoff
means even a drip cannot bridge to the part. New key **`preview_nozzle_temp`
(PROPOSED, default 0)**. The reheat cost is paid once, in the recovery file, after
Accept — no net change to total recovery time versus a plan that reheats there
anyway. Reviewer note: verify the cool-down command does not race the recovery
file's own `M109` reheat (it precedes it in execution order, so no interlock
collision — but pin it).

### E.3 Exclusions — see §A.4

Cancelled-object stops filtered from selection AND resume, using `SimMove.object`.
This closes the D9 hazard for the preview path specifically (the pipeline still
passes `exclude_objects: &[]` at `pipeline.rs:625` — that is a separate,
pre-existing gap tracked in the completion-gate audit; preview must consume the
*real* excluded set once it is wired, and until then treat `None`-object stops as
non-cancellable work, never as excluded).

### E.4 Interaction with `confirm_z_before_resume`

Under `policy = ask`, the preview's Accept **already shows and commits the Z** for
the chosen stop, so a separate `ZConfirmStandoff` pause is redundant. **PROPOSED
rule:** when `policy = ask` and `confirm_z_before_resume` is also set, emit the
preview and **skip** the standalone `Phase::ZConfirmStandoff` (the preview accept
subsumes it). Under `first`/`mid`/`last`, `confirm_z_before_resume` behaves
exactly as today. Flag for ruling: alternatively run both (preview, then a final
Z-confirm on the accepted point) — more prompts, marginal extra safety. Recommend
subsume.

### E.5 Walk-away

Deadline → `TimedOut` → clean abort → `frame_invalid` (past frame declare) →
marker → fresh dry run required. The machine is left at the hover plane (high,
cool, cleared) — the safe resting state. Same shape as every other confirm
timeout; nothing preview-specific to invent.

---

## 7. Decision F — plugin UX

### F.1 Prompt layout (within `action:prompt` constraints)

`prompts.py` spec: line-oriented, buttons fire plain g-code, pipe-delimited fields,
OctoApp renders pipe buttons as inert text so the console fallback is the working
path (`prompts.py:28-35`, `94-107`). New renderer branch in `confirm_ui.py`
(contract source `confirm_ui.py:8-35`; `question()` `104-122` gains a `preview`
case):

```
Power-loss recovery — align the resume point   (stop 3 of 5)
The part shows where printing stopped: a ragged edge where extrusion ends.
Move the hover point to that edge, then Accept.
  Line:  G1 X132.4 Y88.1 E1877.20        (byte 244,118)   <-- UPDATES EVERY STEP
  Layer: 42        Feature: internal infill        Standoff: 1.0 mm
[ Accept ]  [◀ Prev ]  [ Next ▶ ]        (primary)
[ -10 ]  [ -1 ]  [ +1 ]  [ +10 ]         (nudge, along the toolpath)
[ Abort ]                                 (footer)
Console: PLR_RECOVER_ACCEPT | _PREV | _NEXT | _NUDGE FWD=1|BACK=1|FWD=10|BACK=10 | PLR_RECOVER_ABORT
```

- **The offset/line display is refreshed on every reposition** — the byte offset
  and the `Line:` g-code are re-emitted each pause because adjacent stops can be
  <1 mm apart and the operator must not rely on seeing the nozzle move (ruling 2).
  The `detail` map from `report_pause` carries the current stop; the renderer
  prints it verbatim.
- **`first`/`mid` warnings** surface in the dry-run report and, for `ask`, whenever
  the operator nudges to a stop *earlier* than the skip-forward default: the prompt
  adds "this point is before the safe skip-forward line; accepting re-prints
  existing geometry" — advisory tier, matching the three-tier policy.

### F.2 Console commands (the floor)

New param-free / small-param commands, wired like `PLR_RECOVER_CONTINUE`
(`recovery.py:1621-1628`), each calling a widened `answer()`:

- `PLR_RECOVER_ACCEPT`, `PLR_RECOVER_NEXT`, `PLR_RECOVER_PREV`, `PLR_RECOVER_ABORT`
  (abort already exists).
- `PLR_RECOVER_NUDGE FWD=<n>` / `BACK=<n>` (n ∈ {1,10}) — one command with a param
  works in console and in button g-code across clients; buttons pass `FWD=1`,
  `FWD=10`, etc. (Discrete `_NUDGE_FWD` aliases optional; recommend the single
  parametrized command to keep the surface small.)

`answer()` (`recovery.py:750-820`) widens its guard `answer not in (continue,
abort)` (`recovery.py:752`) to the preview vocabulary and applies the shutdown
rule from §D.2 (refuse any repositioning answer when shut down; allow abort). The
`reshow` path (`recovery.py:581-597`) already re-emits the outstanding question on
`PLR_STATUS` — it works for preview unchanged, so an operator who lost the dialog
gets the current stop back rather than clicking blind.

Every button names a plain command and every command is named in the fallback
lines — the whole preview is completable from a bare console (portability rule,
`prompts.py:28-35`).

---

## 8. PROPOSED items needing the principal's ruling

1. **Preview admission bounds** (§A.5): `PREVIEW_MAX_LAYERS = 8`,
   `PREVIEW_MAX_STOPS = 2000`. See CONFLICT-1 on any candidate-count cap.
2. **`mid` across layers** = median candidate stop by execution-order file offset
   (§3). Warned like `first`.
3. **Single hover plane** vs per-stop standoff (§E.1); new key `preview_standoff`
   (default `entry_hop`).
4. **`preview_nozzle_temp`** default 0 = cool during deliberation (§E.2).
5. **`confirm_z_before_resume` subsumed by preview** under `policy = ask` (§E.4).
6. **`[plr]` schema additions** — the principal owns the `[plr]` schema to prevent
   drift (console-milestone memory). New keys: `resume_candidate_policy`
   (`first|mid|last|ask`, default `ask`), `preview_standoff`, `preview_nozzle_temp`.
   Parsed in `plrcfg.rs` with the `probe_method`-style explicit choice check
   (`plrcfg.rs:402-410`) for the enum, `opt_bool`/numeric helpers for the rest;
   carried onto `PlanConfig` (`plrcfg.rs:542`) then the plan.
7. **Console command shape** — one parametrized `PLR_RECOVER_NUDGE FWD=/BACK=` vs
   discrete aliases (§F.2).
8. **Nudge step sizes** — ±1 / ±10 fixed, or a third `±100`? The ruling says two
   sizes; recommend keep two.

---

## 9. Where the ruling and the code disagree — stop and rule

**CONFLICT-1 — "500–1,100 candidates" vs any count-based admission cap.** The
ruling and the headline both cite 500–1,100 candidate lines; the operator-UX
memory records ~50–300 *after* the epoch fix. If the principal wants a candidate
*count* cap (rather than the layer/window caps proposed in §A.5), it must sit
above the real distribution or it refuses exactly the crashes preview exists for.
**Design decision made:** do NOT cap on candidate count at all — representatives
(3–7) bound the UX, and the nudge domain is bounded by the window, not the
candidate count. The refusal is on *layers* and *total window stops*. Confirm this
is acceptable, or supply a count cap ≥ the real max.

**CONFLICT-2 — "descend to a safe standoff above it" vs never-descend.** The
ruling phrases each stop as "move to XY, descend to a standoff." Post-probe the
nozzle is LOW, so the safe realization is a **lift** to a single hover plane, and
repositions are XY-only with no descent (§E.1). This is a deliberate deviation for
safety (never drive toward contact over printed geometry), presented — not
silently adapted. If the principal wants a literal per-stop descend-to-standoff,
that reintroduces a guarded descent per reposition; recommend against.

**No hard conflict** with the confirm/exclusion/frame machinery — preview composes
with all of it.

---

## 10. Scope map

### Files per crate

**`plr-analyzer`** (pure logic, native-testable — the whole increment 1)
- `src/matcher.rs` — extract `collect_candidates` (refactor of `416-450`); ladder
  and public API otherwise **byte-identical** (regression-pinned).
- `src/preview.rs` **(new)** — `PreviewStop`, `PreviewSet`, `PreviewOutcome`,
  `PreviewBounds`, `build_preview`, representative clustering, exclusion filter,
  first/mid/last index derivation.
- `src/lib.rs` — re-exports.

**`plr-recovery`**
- `src/build.rs` — `select_resume_target` takes policy; picks first/mid/last from a
  set; `ask` routes to preview. `PlanConfig` gains `resume_candidate_policy`,
  `preview_standoff`, `preview_nozzle_temp`. New `Phase::ResumePreview` step
  builder (entry lift to hover plane; the cool-down `M104`).
- `src/plan.rs` — `Phase::ResumePreview` (7a″); `RecoveryPlan.preview:
  Option<PreviewSpec>`; `PreviewSpec` (stops + reps + hover params + default
  index). `RecoveryFileSpec` template mode.
- `src/machine.rs` / config validation — new keys validated with the rest (an
  absurd value refused with a diagnosis at plan time).

**`plrd`**
- `src/executor.rs` — `ConfirmKind::Preview`; `PreviewAnswer`; `ask_preview`; the
  preview loop; `RecoveryFileWriter` trait + injection; per-reposition
  exclusivity; hover-plane lift; frame_invalid on preview abort.
- `src/ctrlsock.rs` — `recover_confirm` answer vocabulary widened; `report_pause`
  `detail` for preview; `SocketConfirmer` unchanged beyond passing the new answers;
  `Observed`/`recover_state` mirror gains the current-stop line.
- `src/pipeline.rs` — on coarse match consult policy: `ask` → `build_preview` →
  preview plan; `first/mid/last` → normal plan at chosen offset; `TooWide` →
  `ManualFallback`. Pass the real excluded set once wired.
- `src/recover.rs` — build the `RecoveryFileWriter` closure (execute path) / no-op
  (dry-run); narrate the preview outcome; dry-run preview against `last`.
- `src/plrcfg.rs` — parse the three new `[plr]` keys; carry onto `PlanConfig`.

**`klippy_plugin/plr`**
- `recovery.py` — widen `answer()` vocabulary + shutdown rule; new command entry
  points; preview state uses existing `awaiting_confirmation`/`running` loop.
- `confirm_ui.py` — preview renderer branch; `question()` preview case; offset
  display.
- `plugin.py` — register `PLR_RECOVER_ACCEPT/NEXT/PREV/NUDGE`.

**Docs** — `docs/operations.md` (preview walkthrough, policy table, the
never-descend/cool-nozzle behavior, "abort during preview invalidates the frame").

### Tests that move / are added

- `matcher.rs` tests unchanged (the refactor keeps behavior) — plus a pin that
  `collect_candidates` output equals the pre-refactor candidate set.
- New `preview.rs` proptests: reps ⊆ stops, endpoints always present, ≤7,
  permutation-stable, excluded-object stops absent, `last_index` == today's
  skip-forward offset (the byte-for-byte regression pin, mutation-proven).
- `build.rs` / `plan.rs`: policy selection (first/mid/last), `ResumePreview` step
  shape, never-descends (proptest over stop Z), preview-subsumes-Zconfirm.
- `executor.rs`: the loop (accept/next/prev/nudge/abort/timeout), exclusivity
  re-taken per reposition (fake `Exclusivity` counts calls == repositions),
  frame_invalid on preview abort, shutdown refuses non-abort, writer called once on
  accept with the chosen offset, no-op writer on dry-run.
- `ctrlsock` integration: full preview conversation over the socket
  (token-per-pause, stale-token double-nudge → `unknown-token`).
- Python `tests/test_recovery_confirm.py` (existing) extended for the preview
  vocabulary + shutdown rule; `confirm_ui` renderer tests for offset-refresh and
  console fallback completeness.

### What the adversarial reviewer will attack

1. **Truth-excluding candidate set** — resolved by the wider nudge domain (§A.2);
   the reviewer will check that nudge really reaches non-candidate lines.
2. **Barrier once, not per reposition** (eleventh corollary) — fake exclusivity
   must show one recheck per send.
3. **A stop Z that makes the hover descend** — the single-plane + `park_z_at`
   clamp must make it a type impossibility; reviewer crafts a high-then-low layer
   order.
4. **frame_invalid false on preview abort** — must be true (past ShiftedFrame).
5. **Dry-run writes the recovery file** — the no-op writer must be a capability the
   dry-run path literally does not hold (twelfth corollary).
6. **`last_index` != today's skip-forward** — regression pin, mutation-proven.
7. **Contract drift** — the plugin renderer must read `report_pause`'s producer
   fields byte-for-byte (agent-contract discipline; first/P1 near-miss).
8. **Vacuous bound** — `PREVIEW_MAX_*` must be reachable by a real (synthesized)
   input, not decorative (fourth corollary).
9. **Cancelled object resurfaces** in a stop or resume (D9).
10. **Ooze during a long pause** — cool-nozzle command present and ordered before
    the file's reheat.
11. **`mid` across layers ill-defined** — median-offset rule, tested across a
    multi-layer set.
12. **Recovery-file/preview offset mismatch** — the file generated on Accept must
    match the previewed stop's offset exactly.

---

## 11. Increment plan (each independently green)

**Increment 1 — analyzer preview core** (`plr-analyzer` only; Windows
`default-members`, no daemon/plugin).
- Refactor `collect_candidates`; add `preview.rs` (stops, reps, bounds, exclusion,
  first/mid/last). Extend `select_resume_target` (or a sibling) for policy over a
  set. `MatchConfidence` and the automatic paths **byte-identical**, pinned.
- Green: Windows `cargo test -p plr-analyzer -p plr-recovery`. No behavior change to
  any shipping path (nothing calls `build_preview` yet).

**Increment 2 — plan + daemon plumbing + motion loop** (`plr-recovery` + `plrd`;
Linux gate authoritative — `scripts/gates-linux.sh`, plrd included).
- `[plr] resume_candidate_policy` + the two proposed keys; `PlanConfig`;
  `Phase::ResumePreview`; `RecoveryPlan.preview`; pipeline routing by policy;
  `RecoveryFileWriter` injection + late-bind; `ConfirmKind::Preview` +
  `PreviewAnswer` + executor loop (exclusivity per reposition, never-descend hover,
  deadline/abort/frame_invalid, shutdown rule); socket vocabulary + `report_pause`
  detail.
- Green: Linux `cargo test --workspace` + `scripts/gates-linux.sh`. The
  `first`/`mid`/`last` paths are already usable end-to-end here (headless, no
  dialog) — a strict win: coarse matches that were `ManualFallback` now resume.
- Note: increment 2 is the largest surface and touches `pipeline.rs`, which is a
  known merge-conflict hot spot — sequence it when no other branch holds
  `pipeline.rs`.

**Increment 3 — plugin UX** (`klippy_plugin`; pytest ≥90%).
- `confirm_ui` preview renderer; new commands; `answer()` vocabulary + shutdown
  rule; offset-refresh; console fallbacks; `docs/operations.md`.
- Green: `pytest` + ruff. The console path is the acceptance floor; dialog buttons
  are the enhancement.
- **Carried over from increment 2 (obligation):** the current preview stop is
  published on every pause through `report_pause`'s `detail` map (the field
  contract above), but the `recover_state` status-poll mirror (`ctrlsock`'s
  `Observed`/`LivePause`/`StateSnapshot`) still carries only token / kind / step /
  deadline, NOT the stop. Increment 3's `PLR_STATUS` reshow reads `recover_state`,
  so increment 3 must extend that mirror to carry the current stop's `detail` (or
  the plugin must reshow from the last `report_pause` payload instead). Increment 2
  left `report_pause` — the authoritative per-pause producer — complete; only the
  poll mirror is deferred.
- **Also carried over (decision):** increment 2 implemented and tested the whole
  motion loop / writer / socket vocabulary but did NOT turn the interactive `ask`
  path on in the pipeline, because doing so flips the shipped default from
  automatic skip-forward to an interactive pause (a bare-CLI / headless recovery
  then aborts ambiguous resumes that auto-complete today). `ask` resolves as `last`
  in the interim; `first`/`mid`/`last` deliver the headless win. Flipping `ask` on
  is a one-line pipeline change (build the preview set for `Ask` too and pass it
  through as the attached preview) plus retargeting the automatic-completion tests
  to an explicit policy.

Order 1 → 2 → 3. Each merges to `main` independently; after 1 the analyzer can
produce preview sets, after 2 a headless operator has working `first/mid/last`
over the socket (and `ask` once the default-flip is ruled), after 3 the dialog UX
lands.

---

## 12. Residual risks

- **Recovery-file late-binding** is the one structural change; if the injected
  writer seam proves awkward against `execute()`'s single-pass shape, the fallback
  is to split execution at Accept (heavier). Prototype the seam first in
  increment 2.
- **Representative clustering quality** is unproven on real geometry until an E5
  bench session; the algorithm is bounded and safe (reps ⊆ candidates) but "are
  these the *right* 5 stops" needs the operator on `duender`. The nudge domain
  makes a poor rep set recoverable (nudge to anywhere), so this is a UX-quality
  risk, not a safety one.
- **Cool-nozzle/reheat timing** (§E.2) interacts with the recovery file's own
  `M109`; pin the ordering.
- The pre-existing `exclude_objects: &[]` gap (`pipeline.rs:625`) means preview's
  exclusion filter is inert until that is wired; until then it conservatively keeps
  all non-attributable stops (safe direction).
- **Arc-chord nudge UX (increment 2/3).** A `G2`/`G3` arc decomposes into many
  `SimMove` chords that all share the one source line's byte span, so the arc line
  contributes several adjacent `PreviewStop`s at the *same* `offset` but different
  hover XY. A ±1 nudge that steps between two chords of the same arc therefore moves
  the hover point along the curve while the `offset`/`Line:` readout does **not**
  change — the operator sees the nozzle-target coordinate shift but the byte offset
  hold. Increment 2's prompt must show the per-chord hover XY (not only the offset)
  so within-arc nudges give visible feedback, and increment 3's renderer must not
  treat "offset unchanged" as "nudge had no effect." (Increment 1 note: the
  first/mid/last anchors and the `last_index` skip-forward pin are unaffected — they
  resolve on the committed *resume* offset, and `stop_resuming_at` breaks same-offset
  ties toward the last chord.)
