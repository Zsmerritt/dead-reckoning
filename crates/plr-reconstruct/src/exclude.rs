//! The excluded-object picture: what the operator cancelled, how sure
//! we are of it, and which object a point on the bed belongs to.
//!
//! # Why this needs a provenance flag
//!
//! Klipper's `[exclude_object]` lets an operator cancel one object
//! mid-print, and operators overwhelmingly do that *because the object
//! failed* — it detached, warped, or turned into spaghetti. Klipper
//! holds that decision only in RAM
//! (`klippy/extras/exclude_object.py`, `_reset_state` / `_reset_file`),
//! so a power loss erases it. If recovery quietly assumed "nothing was
//! excluded" it would resume printing into the debris.
//!
//! An empty excluded set therefore has to be qualified by *where it
//! came from*, which is what [`ExclusionProvenance`] records:
//!
//! | WAL journaled exclude state? | print file defines objects? | provenance |
//! |---|---|---|
//! | yes | — | [`Journaled`](ExclusionProvenance::Journaled) — authoritative |
//! | no  | no  | [`NoObjectsDefined`](ExclusionProvenance::NoObjectsDefined) — nothing could have been cancelled |
//! | no  | yes | [`RecordLost`](ExclusionProvenance::RecordLost) — **dangerous**, see below |
//! | no  | unknown (file not checkable) | [`Unknown`](ExclusionProvenance::Unknown) |
//!
//! [`RecordLost`](ExclusionProvenance::RecordLost) is the case that
//! matters: the print has cancelable objects, no cancellation record
//! survived, and a resume will print **all** of them. It is reported as
//! [`ExclusionDiagnostic::CancellationRecordLost`] with the object
//! names, and [`ExclusionReport::requires_operator_confirmation`]
//! returns `true` — it is an operator-confirmable condition, never a
//! silent default.
//!
//! # Reading definitions out of the print file
//!
//! The definitions live in the file itself: slicers (and Moonraker's
//! object processor) emit an `EXCLUDE_OBJECT_DEFINE` block in the
//! header, and `EXCLUDE_OBJECT_START NAME=` auto-defines any object
//! Klipper has not seen (`cmd_EXCLUDE_OBJECT_START`,
//! `exclude_object.py` lines 199-204). [`parse_object_definitions`] replays
//! both, including `EXCLUDE_OBJECT_DEFINE RESET=1`, using `plr-gcode`'s
//! Klipper-faithful tokenizer.
//!
//! # Point-in-object lookup
//!
//! Downstream consumers (contact-point selection, the resume file) need
//! to answer "is this XY on an object the operator cancelled?".
//! [`ExclusionReport::objects_at`] answers it from the outlines, with
//! boundary points counted as **inside** (the conservative direction:
//! an object's edge is part of the object). Objects whose outline the
//! WAL could not journal exactly are flagged
//! [`ExclusionDiagnostic::GeometryDegraded`]; objects with no usable
//! outline can never match, which
//! [`ExclusionReport::geometry_is_complete`] reports.
//!
//! Totality: every function here is total — no panics for any input,
//! including NaN coordinates and degenerate rings (property-tested).

use plr_gcode::{parse_line, ByteSpan};
use plr_wal::{ExcludeObjectDef, PolygonFidelity};

use crate::stopset::FileTail;
use crate::timeline::WalTimeline;

/// Half-width of the on-edge test used by [`point_in_polygon`], in mm.
///
/// A point within this distance of an outline edge counts as inside.
/// 1 µm is far below any slicer's coordinate resolution and far above
/// the rounding error of the cross-product test.
pub const EDGE_TOLERANCE_MM: f64 = 1e-6;

/// Where the excluded-object picture came from — the qualifier that
/// makes an empty excluded set safe (or not) to act on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExclusionProvenance {
    /// The WAL journaled `exclude_object` state. The excluded set is
    /// authoritative: it is exactly what Klipper knew at the last
    /// durable context.
    Journaled,
    /// The WAL journaled no exclude state, and the print file defines
    /// no objects at all. Nothing could have been cancelled; an empty
    /// excluded set is correct.
    NoObjectsDefined,
    /// The WAL journaled no exclude state, but the print file **does**
    /// define objects. Whether the operator cancelled any of them is
    /// unknowable from durable evidence, and a resume would print all
    /// of them. See [`ExclusionDiagnostic::CancellationRecordLost`].
    RecordLost,
    /// The WAL journaled no exclude state and the print file could not
    /// be checked (no bytes supplied, or only a tail that does not
    /// start at offset 0), so we cannot even tell whether the print has
    /// cancelable objects.
    Unknown,
}

impl ExclusionProvenance {
    /// `true` when the excluded set can be acted on without asking the
    /// operator — only [`Journaled`](Self::Journaled) and
    /// [`NoObjectsDefined`](Self::NoObjectsDefined) qualify.
    #[must_use]
    pub const fn is_conclusive(self) -> bool {
        matches!(self, Self::Journaled | Self::NoObjectsDefined)
    }
}

/// An honest statement about what the exclusion picture does not know.
/// Diagnostics never invalidate the report; they say what a consumer
/// must surface to the operator.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExclusionDiagnostic {
    /// **The dangerous case.** This print has cancelable objects, no
    /// cancellation record survived the stop, and a resume will print
    /// *all* of them — including any the operator had cancelled because
    /// it failed. The operator must confirm before resuming.
    CancellationRecordLost {
        /// Every object the print file defines, in file order. Listed
        /// so the operator can check the plate against them.
        objects: Vec<String>,
    },
    /// No exclude state was journaled and the print file could not be
    /// checked for object definitions, so even the *existence* of
    /// cancelable objects is unknown.
    FileNotChecked,
    /// Names in the excluded set that have no definition, so their
    /// geometry is unknown and point-in-object lookup cannot place
    /// them. Their exclusion is still authoritative.
    ExcludedObjectUndefined {
        /// The undefined excluded names.
        objects: Vec<String>,
    },
    /// Objects whose journaled outline is not a verbatim copy of what
    /// Klipper reported (bounding-box substitution, an unusable ring,
    /// or a fidelity written by a newer format revision). Geometric
    /// answers about these objects are approximate or unavailable.
    GeometryDegraded {
        /// The affected object names.
        objects: Vec<String>,
    },
    /// Objects the print file defines that the journaled definitions do
    /// not mention. Informational: the WAL's excluded set is still
    /// authoritative, but a consumer replaying
    /// `EXCLUDE_OBJECT_DEFINE` should take the full list from the file.
    DefinitionsIncomplete {
        /// The names present only in the file.
        objects: Vec<String>,
    },
}

/// The excluded-object picture for one reconstruction.
#[derive(Debug, Clone, PartialEq)]
pub struct ExclusionReport {
    /// Where this picture came from. Read it before trusting
    /// [`excluded`](Self::excluded).
    pub provenance: ExclusionProvenance,
    /// Known object definitions: the journaled ones when the provenance
    /// is [`Journaled`](ExclusionProvenance::Journaled), otherwise the
    /// ones parsed out of the print file (empty when neither exists).
    pub definitions: Vec<ExcludeObjectDef>,
    /// Names of the objects the operator cancelled, in Klipper's sorted
    /// order. **Always empty unless the provenance is
    /// [`Journaled`](ExclusionProvenance::Journaled)** — an empty set
    /// from any other provenance means "unrecorded", not "none".
    pub excluded: Vec<String>,
    /// The object being printed at the last journaled context, when one
    /// was active (`EXCLUDE_OBJECT_START` without a matching `END`).
    pub current_object: Option<String>,
    /// Everything the report cannot vouch for.
    pub diagnostics: Vec<ExclusionDiagnostic>,
}

impl ExclusionReport {
    /// The report for a WAL that journaled nothing and a file that was
    /// never checked.
    #[must_use]
    pub fn unknown() -> Self {
        Self {
            provenance: ExclusionProvenance::Unknown,
            definitions: Vec::new(),
            excluded: Vec::new(),
            current_object: None,
            diagnostics: vec![ExclusionDiagnostic::FileNotChecked],
        }
    }

    /// `true` when a resume must not proceed without the operator
    /// confirming what is on the bed: the cancellation record was lost,
    /// or the file could not be checked at all.
    #[must_use]
    pub fn requires_operator_confirmation(&self) -> bool {
        !self.provenance.is_conclusive()
    }

    /// `true` when `name` is known to have been cancelled. Comparison
    /// is case-insensitive; Klipper upper-cases every name it stores
    /// (`name.upper()`, `exclude_object.py` lines 200/224/250).
    #[must_use]
    pub fn is_excluded(&self, name: &str) -> bool {
        self.excluded
            .iter()
            .any(|excluded| excluded.eq_ignore_ascii_case(name))
    }

    /// The definition of `name`, if known (case-insensitive).
    #[must_use]
    pub fn definition(&self, name: &str) -> Option<&ExcludeObjectDef> {
        self.definitions
            .iter()
            .find(|def| def.name.eq_ignore_ascii_case(name))
    }

    /// Every object whose outline contains `(x, y)`, in definition
    /// order. Boundary points count as inside. Objects with no usable
    /// outline never match — check
    /// [`geometry_is_complete`](Self::geometry_is_complete) before
    /// reading "no match" as "not on any object".
    ///
    /// Overlapping outlines are possible (nested or touching parts), so
    /// this returns all matches rather than picking one.
    #[must_use]
    pub fn objects_at(&self, x: f64, y: f64) -> Vec<&ExcludeObjectDef> {
        self.definitions
            .iter()
            .filter(|def| point_in_polygon(x, y, &def.polygon))
            .collect()
    }

    /// The first object whose outline contains `(x, y)`, in definition
    /// order (Klipper keeps `objects` sorted by name, so this is the
    /// alphabetically first match). See [`objects_at`](Self::objects_at)
    /// for the overlap caveat.
    #[must_use]
    pub fn object_at(&self, x: f64, y: f64) -> Option<&ExcludeObjectDef> {
        self.definitions
            .iter()
            .find(|def| point_in_polygon(x, y, &def.polygon))
    }

    /// The first **cancelled** object whose outline contains `(x, y)`.
    /// This is the "is this point on a cancelled part?" query.
    ///
    /// A `None` answer is only as strong as
    /// [`provenance`](Self::provenance) and
    /// [`geometry_is_complete`](Self::geometry_is_complete) allow.
    #[must_use]
    pub fn excluded_object_at(&self, x: f64, y: f64) -> Option<&ExcludeObjectDef> {
        self.definitions
            .iter()
            .find(|def| self.is_excluded(&def.name) && point_in_polygon(x, y, &def.polygon))
    }

    /// `true` when every known definition carries a verbatim outline,
    /// so a "not on any object" answer is trustworthy.
    #[must_use]
    pub fn geometry_is_complete(&self) -> bool {
        self.definitions
            .iter()
            .all(|def| def.fidelity == PolygonFidelity::Exact)
    }

    /// The definitions of the cancelled objects, in definition order.
    #[must_use]
    pub fn excluded_definitions(&self) -> Vec<&ExcludeObjectDef> {
        self.definitions
            .iter()
            .filter(|def| self.is_excluded(&def.name))
            .collect()
    }
}

/// Builds the excluded-object picture from the WAL timeline and, when
/// available, the print file.
///
/// `file` must start at byte 0 of the print file for the
/// object-definition scan to be conclusive; a partial tail cannot prove
/// the absence of a header `EXCLUDE_OBJECT_DEFINE` block and yields
/// [`ExclusionProvenance::Unknown`].
#[must_use]
pub fn resolve_exclusions(timeline: &WalTimeline, file: Option<&FileTail<'_>>) -> ExclusionReport {
    let journaled = journaled_state(timeline);
    let file_scan = file
        .filter(|tail| tail.base_offset == 0)
        .map(|tail| parse_object_definitions(tail.bytes));

    match journaled {
        Some(state) => journaled_report(state, file_scan.as_ref()),
        None => match file_scan {
            None => ExclusionReport::unknown(),
            Some(scan) if scan.is_empty() => ExclusionReport {
                provenance: ExclusionProvenance::NoObjectsDefined,
                definitions: Vec::new(),
                excluded: Vec::new(),
                current_object: None,
                diagnostics: Vec::new(),
            },
            Some(scan) => {
                let objects = scan.names();
                let mut diagnostics = vec![ExclusionDiagnostic::CancellationRecordLost { objects }];
                push_geometry_diagnostic(&scan.definitions, &mut diagnostics);
                ExclusionReport {
                    provenance: ExclusionProvenance::RecordLost,
                    definitions: scan.definitions,
                    excluded: Vec::new(),
                    current_object: None,
                    diagnostics,
                }
            }
        },
    }
}

/// The merged exclude state carried by the WAL's contexts.
struct JournaledState {
    definitions: Vec<ExcludeObjectDef>,
    excluded: Vec<String>,
    current: Option<String>,
}

/// Merges every context's exclude payload in append order. Definitions
/// are journaled only when they change (`plr_wal::ExcludeState`), so the
/// newest `Some(..)` wins and later contexts carry the set forward.
/// Returns `None` when no context carried exclude state at all.
fn journaled_state(timeline: &WalTimeline) -> Option<JournaledState> {
    let mut seen = false;
    let mut state = JournaledState {
        definitions: Vec::new(),
        excluded: Vec::new(),
        current: None,
    };
    for context in &timeline.contexts {
        let Some(exclude) = &context.exclude else {
            continue;
        };
        seen = true;
        if let Some(definitions) = &exclude.definitions {
            state.definitions.clone_from(definitions);
        }
        state.excluded.clone_from(&exclude.excluded);
        state.current.clone_from(&exclude.current);
    }
    seen.then_some(state)
}

/// Assembles the report for a WAL that journaled exclude state,
/// cross-checking the print file when it was scannable.
fn journaled_report(state: JournaledState, file_scan: Option<&FileObjectScan>) -> ExclusionReport {
    let mut diagnostics = Vec::new();

    let undefined: Vec<String> = state
        .excluded
        .iter()
        .filter(|name| {
            !state
                .definitions
                .iter()
                .any(|def| def.name.eq_ignore_ascii_case(name))
        })
        .cloned()
        .collect();
    if !undefined.is_empty() {
        diagnostics.push(ExclusionDiagnostic::ExcludedObjectUndefined { objects: undefined });
    }

    push_geometry_diagnostic(&state.definitions, &mut diagnostics);

    if let Some(scan) = file_scan {
        let missing: Vec<String> = scan
            .definitions
            .iter()
            .filter(|def| {
                !state
                    .definitions
                    .iter()
                    .any(|known| known.name.eq_ignore_ascii_case(&def.name))
            })
            .map(|def| def.name.clone())
            .collect();
        if !missing.is_empty() {
            diagnostics.push(ExclusionDiagnostic::DefinitionsIncomplete { objects: missing });
        }
    }

    ExclusionReport {
        provenance: ExclusionProvenance::Journaled,
        definitions: state.definitions,
        excluded: state.excluded,
        current_object: state.current,
        diagnostics,
    }
}

/// Adds a [`ExclusionDiagnostic::GeometryDegraded`] note when any
/// definition's outline is not verbatim.
fn push_geometry_diagnostic(
    definitions: &[ExcludeObjectDef],
    diagnostics: &mut Vec<ExclusionDiagnostic>,
) {
    let degraded: Vec<String> = definitions
        .iter()
        .filter(|def| def.fidelity.is_degraded())
        .map(|def| def.name.clone())
        .collect();
    if !degraded.is_empty() {
        diagnostics.push(ExclusionDiagnostic::GeometryDegraded { objects: degraded });
    }
}

/// What a scan of the print file found out about cancelable objects.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FileObjectScan {
    /// Definitions in file order, deduplicated by name (a later
    /// `EXCLUDE_OBJECT_DEFINE` for the same name replaces an earlier
    /// one, which is what a consumer wants; Klipper itself appends both
    /// and lets the first win on lookup).
    pub definitions: Vec<ExcludeObjectDef>,
    /// `EXCLUDE_OBJECT_DEFINE` / `EXCLUDE_OBJECT_START` lines whose
    /// parameters could not be parsed (unbalanced quoting, a token with
    /// no `=`). They still prove the file has cancelable objects, so
    /// they are counted rather than ignored.
    pub unparsed_lines: usize,
}

impl FileObjectScan {
    /// `true` when the file defines no cancelable objects at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.definitions.is_empty() && self.unparsed_lines == 0
    }

    /// The defined object names, in file order.
    #[must_use]
    pub fn names(&self) -> Vec<String> {
        self.definitions
            .iter()
            .map(|def| def.name.clone())
            .collect()
    }
}

/// Scans print-file bytes for the object definitions Klipper would
/// build from them.
///
/// Replays the three commands that mutate `exclude_object.objects`:
///
/// * `EXCLUDE_OBJECT_DEFINE NAME=<n> [CENTER=x,y] [POLYGON=[[x,y],...]]`
///   — adds `n.upper()` with the parsed geometry. `CENTER` is read as
///   `json.loads('[%s]' % value)` and `POLYGON` as `json.loads(value)`,
///   matching exclude_object.py:256-270.
/// * `EXCLUDE_OBJECT_DEFINE RESET=<anything non-empty>` — `_reset_file()`,
///   clearing everything found so far. Klipper's test is Python
///   truthiness on the raw string, so `RESET=0` resets too.
/// * `EXCLUDE_OBJECT_START NAME=<n>` — adds a name-only object when `n`
///   is not already known (`exclude_object.py` lines 199-204).
///
/// Lines are pre-filtered on the ASCII substring `EXCLUDE_OBJECT`
/// before being tokenized, so scanning a whole multi-megabyte print
/// file costs one pass of substring search plus a parse of the handful
/// of matching lines.
///
/// Total: never panics, for any bytes.
#[must_use]
pub fn parse_object_definitions(bytes: &[u8]) -> FileObjectScan {
    let mut scan = FileObjectScan::default();
    for raw in bytes.split(|&b| b == b'\n') {
        if !contains_exclude_object(raw) {
            continue;
        }
        let line = parse_line(strip_cr(raw), ByteSpan { start: 0, end: 0 });
        let Some(command) = line.command() else {
            continue;
        };
        match command.name.as_str() {
            "EXCLUDE_OBJECT_DEFINE" => apply_define(command, &mut scan),
            "EXCLUDE_OBJECT_START" => apply_start(command, &mut scan),
            _ => {}
        }
    }
    scan
}

/// Applies one `EXCLUDE_OBJECT_DEFINE` line to the scan.
fn apply_define(command: &plr_gcode::Command, scan: &mut FileObjectScan) {
    if command.is_malformed_extended() {
        scan.unparsed_lines += 1;
        return;
    }
    // Klipper checks `if reset:` on the raw string: any non-empty value
    // (including "0") triggers `_reset_file()`.
    if command.get("RESET").is_some_and(|value| !value.is_empty()) {
        scan.definitions.clear();
        return;
    }
    // An empty/absent NAME makes the command a listing request.
    let Some(name) = command.get("NAME").filter(|name| !name.is_empty()) else {
        return;
    };
    let center = command.get("CENTER").and_then(parse_center);
    let polygon = command.get("POLYGON").map(parse_polygon);
    let def = ExcludeObjectDef::normalized(name.to_uppercase(), center, polygon);
    upsert(&mut scan.definitions, def);
}

/// Applies one `EXCLUDE_OBJECT_START` line to the scan.
fn apply_start(command: &plr_gcode::Command, scan: &mut FileObjectScan) {
    if command.is_malformed_extended() {
        scan.unparsed_lines += 1;
        return;
    }
    // Klipper's cmd_EXCLUDE_OBJECT_START requires NAME (gcmd.get('NAME')
    // raises without it); a line without one defines nothing.
    let Some(name) = command.get("NAME").filter(|name| !name.is_empty()) else {
        return;
    };
    let name = name.to_uppercase();
    if !scan
        .definitions
        .iter()
        .any(|def| def.name.eq_ignore_ascii_case(&name))
    {
        scan.definitions.push(ExcludeObjectDef::name_only(name));
    }
}

/// Inserts `def`, replacing any existing definition of the same name.
fn upsert(definitions: &mut Vec<ExcludeObjectDef>, def: ExcludeObjectDef) {
    match definitions
        .iter_mut()
        .find(|known| known.name.eq_ignore_ascii_case(&def.name))
    {
        Some(existing) => *existing = def,
        None => definitions.push(def),
    }
}

/// `json.loads('[%s]' % value)` reduced to a finite `[x, y]` pair.
fn parse_center(value: &str) -> Option<[f64; 2]> {
    let parsed: Vec<f64> = serde_json::from_str(&format!("[{value}]")).ok()?;
    let (&x, &y) = (parsed.first()?, parsed.get(1)?);
    (x.is_finite() && y.is_finite()).then_some([x, y])
}

/// `json.loads(value)` reduced to finite `[x, y]` points, in the shape
/// [`ExcludeObjectDef::normalized`] expects: `Err(count)` when the
/// outline exists but cannot be used.
fn parse_polygon(value: &str) -> Result<Vec<[f64; 2]>, usize> {
    let Ok(parsed) = serde_json::from_str::<Vec<Vec<f64>>>(value) else {
        return Err(0);
    };
    let mut points = Vec::with_capacity(parsed.len());
    for point in &parsed {
        match (point.first(), point.get(1)) {
            (Some(&x), Some(&y)) if x.is_finite() && y.is_finite() => points.push([x, y]),
            _ => return Err(parsed.len()),
        }
    }
    Ok(points)
}

/// Case-insensitive ASCII substring test for `EXCLUDE_OBJECT`, the
/// cheap pre-filter that keeps whole-file scanning affordable.
fn contains_exclude_object(line: &[u8]) -> bool {
    const NEEDLE: &[u8] = b"EXCLUDE_OBJECT";
    line.len() >= NEEDLE.len()
        && line.windows(NEEDLE.len()).any(|window| {
            window
                .iter()
                .zip(NEEDLE)
                .all(|(byte, want)| byte.eq_ignore_ascii_case(want))
        })
}

/// Drops a trailing `\r` left by CRLF line endings.
fn strip_cr(line: &[u8]) -> &[u8] {
    match line.split_last() {
        Some((b'\r', head)) => head,
        _ => line,
    }
}

/// `true` when `(x, y)` lies inside `polygon`, with points on an edge
/// counted as inside (within [`EDGE_TOLERANCE_MM`]).
///
/// Even-odd (ray-casting) rule, matching how a slicer's outline is
/// meant to be read. Total: returns `false` rather than panicking for a
/// non-finite query point, a non-finite vertex, or a ring of fewer than
/// three points.
#[must_use]
pub fn point_in_polygon(x: f64, y: f64, polygon: &[[f64; 2]]) -> bool {
    if !x.is_finite() || !y.is_finite() || polygon.len() < 3 {
        return false;
    }
    if polygon.iter().flatten().any(|value| !value.is_finite()) {
        return false;
    }
    // The boundary is part of the object: test it first so an edge hit
    // never depends on the parity rule's tie-breaking.
    for (a, b) in edges(polygon) {
        if point_on_segment(x, y, a, b) {
            return true;
        }
    }
    let mut inside = false;
    for (a, b) in edges(polygon) {
        // Half-open crossing rule: an edge counts when the ray's y lies
        // in [min, max) of the edge, so a vertex is counted once.
        if (a[1] > y) != (b[1] > y) {
            let t = (y - a[1]) / (b[1] - a[1]);
            let crossing = a[0] + t * (b[0] - a[0]);
            if x < crossing {
                inside = !inside;
            }
        }
    }
    inside
}

/// The closed ring's edges, last vertex back to first.
fn edges(polygon: &[[f64; 2]]) -> impl Iterator<Item = ([f64; 2], [f64; 2])> + '_ {
    polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
        .map(|(a, b)| (*a, *b))
}

/// `true` when `(x, y)` lies on the segment `a`–`b` within
/// [`EDGE_TOLERANCE_MM`]. Handles the degenerate zero-length segment
/// (repeated vertex) as a point test.
fn point_on_segment(x: f64, y: f64, a: [f64; 2], b: [f64; 2]) -> bool {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let (px, py) = (x - a[0], y - a[1]);
    let length = dx.hypot(dy);
    if length <= EDGE_TOLERANCE_MM {
        return px.hypot(py) <= EDGE_TOLERANCE_MM;
    }
    // Perpendicular distance from the (infinite) line through a and b.
    if (px * dy - py * dx).abs() / length > EDGE_TOLERANCE_MM {
        return false;
    }
    // ... and within the segment's extent.
    let projection = (px * dx + py * dy) / length;
    projection >= -EDGE_TOLERANCE_MM && projection <= length + EDGE_TOLERANCE_MM
}

#[cfg(test)]
mod tests {
    // Fixture coordinates are exact decimal literals that round-trip
    // through the parser unchanged; exact comparison is the property
    // under test.
    #![allow(clippy::float_cmp)]

    use std::fmt::Write as _;

    use plr_wal::{Context, ExcludeState, PolygonFidelity, WalRecord};

    use super::{
        parse_object_definitions, point_in_polygon, resolve_exclusions, ExcludeObjectDef,
        ExclusionDiagnostic, ExclusionProvenance, ExclusionReport,
    };
    use crate::stopset::FileTail;
    use crate::testutil::{context_at, ingest_records};
    use crate::timeline::WalTimeline;

    /// A 20 mm square centred on (100, 100).
    fn square(name: &str, cx: f64, cy: f64) -> ExcludeObjectDef {
        ExcludeObjectDef {
            name: name.to_owned(),
            center: Some([cx, cy]),
            polygon: vec![
                [cx - 10.0, cy - 10.0],
                [cx + 10.0, cy - 10.0],
                [cx + 10.0, cy + 10.0],
                [cx - 10.0, cy + 10.0],
            ],
            fidelity: PolygonFidelity::Exact,
        }
    }

    fn timeline_with(exclude: Option<ExcludeState>) -> WalTimeline {
        let mut context = context_at(1_000, 0);
        context.exclude = exclude.map(Box::new);
        ingest_records(vec![WalRecord::Context(context)])
    }

    fn tail(bytes: &[u8]) -> FileTail<'_> {
        FileTail {
            base_offset: 0,
            bytes,
        }
    }

    const TWO_OBJECT_FILE: &[u8] = b"; generated by a slicer\n\
EXCLUDE_OBJECT_DEFINE NAME=Cube_id_0_copy_0 CENTER=100,100 POLYGON=[[90,90],[110,90],[110,110],[90,110]]\n\
EXCLUDE_OBJECT_DEFINE NAME=Cube_id_1_copy_0 CENTER=150,100 POLYGON=[[140,90],[160,90],[160,110],[140,110]]\n\
G1 X10 Y10 F3000\n\
EXCLUDE_OBJECT_START NAME=Cube_id_0_copy_0\n\
G1 X100 Y100 E1\n\
EXCLUDE_OBJECT_END NAME=Cube_id_0_copy_0\n";

    // --- provenance -------------------------------------------------

    #[test]
    fn journaled_state_is_authoritative() {
        let timeline = timeline_with(Some(ExcludeState {
            definitions: Some(vec![square("A", 100.0, 100.0), square("B", 150.0, 100.0)]),
            excluded: vec!["B".to_owned()],
            current: Some("A".to_owned()),
        }));
        let report = resolve_exclusions(&timeline, None);
        assert_eq!(report.provenance, ExclusionProvenance::Journaled);
        assert!(report.provenance.is_conclusive());
        assert!(!report.requires_operator_confirmation());
        assert_eq!(report.excluded, vec!["B".to_owned()]);
        assert_eq!(report.current_object.as_deref(), Some("A"));
        assert!(report.is_excluded("b"), "names compare case-insensitively");
        assert!(!report.is_excluded("A"));
        assert_eq!(report.diagnostics, Vec::new());
        assert!(report.geometry_is_complete());
        assert_eq!(report.definition("A").map(|d| d.name.as_str()), Some("A"));
        assert_eq!(report.definition("Z"), None);
        assert_eq!(
            report
                .excluded_definitions()
                .iter()
                .map(|d| d.name.as_str())
                .collect::<Vec<_>>(),
            vec!["B"]
        );
    }

    #[test]
    fn definitions_are_carried_forward_across_contexts() {
        // plr_wal::ExcludeState journals definitions once; later
        // contexts carry `None` and only refresh the excluded set.
        let mut first = context_at(1_000, 0);
        first.exclude = Some(Box::new(ExcludeState {
            definitions: Some(vec![square("A", 100.0, 100.0)]),
            excluded: Vec::new(),
            current: None,
        }));
        let mut second = context_at(2_000, 10);
        second.exclude = Some(Box::new(ExcludeState {
            definitions: None,
            excluded: vec!["A".to_owned()],
            current: None,
        }));
        let timeline = ingest_records(vec![WalRecord::Context(first), WalRecord::Context(second)]);
        let report = resolve_exclusions(&timeline, None);
        assert_eq!(report.definitions.len(), 1);
        assert_eq!(report.excluded, vec!["A".to_owned()]);
    }

    #[test]
    fn no_record_and_no_objects_in_file_is_reported_plainly() {
        let timeline = timeline_with(None);
        let file = b"G28\nG1 X10 Y10 F3000\nG1 Z0.2\n";
        let report = resolve_exclusions(&timeline, Some(&tail(file)));
        assert_eq!(report.provenance, ExclusionProvenance::NoObjectsDefined);
        assert!(!report.requires_operator_confirmation());
        assert!(report.definitions.is_empty());
        assert!(report.excluded.is_empty());
        assert_eq!(report.diagnostics, Vec::new());
    }

    #[test]
    fn no_record_but_file_defines_objects_is_the_dangerous_case() {
        let timeline = timeline_with(None);
        let report = resolve_exclusions(&timeline, Some(&tail(TWO_OBJECT_FILE)));
        assert_eq!(report.provenance, ExclusionProvenance::RecordLost);
        assert!(!report.provenance.is_conclusive());
        assert!(
            report.requires_operator_confirmation(),
            "a resume would print every object, cancelled or not"
        );
        assert_eq!(
            report.diagnostics,
            vec![ExclusionDiagnostic::CancellationRecordLost {
                objects: vec!["CUBE_ID_0_COPY_0".to_owned(), "CUBE_ID_1_COPY_0".to_owned(),],
            }]
        );
        // The excluded set stays empty but is NOT authoritative.
        assert!(report.excluded.is_empty());
        assert_eq!(report.definitions.len(), 2);
        // The geometry still answers point queries.
        assert_eq!(
            report.object_at(100.0, 100.0).map(|d| d.name.as_str()),
            Some("CUBE_ID_0_COPY_0")
        );
    }

    #[test]
    fn no_record_and_unusable_file_is_unknown() {
        let timeline = timeline_with(None);
        // No file at all.
        let report = resolve_exclusions(&timeline, None);
        assert_eq!(report.provenance, ExclusionProvenance::Unknown);
        assert!(report.requires_operator_confirmation());
        assert_eq!(
            report.diagnostics,
            vec![ExclusionDiagnostic::FileNotChecked]
        );

        // A tail that does not start at byte 0 cannot prove the header
        // held no EXCLUDE_OBJECT_DEFINE block.
        let partial = FileTail {
            base_offset: 4_096,
            bytes: b"G1 X1\n",
        };
        let report = resolve_exclusions(&timeline, Some(&partial));
        assert_eq!(report.provenance, ExclusionProvenance::Unknown);
        assert_eq!(ExclusionReport::unknown(), report);
    }

    #[test]
    fn a_context_without_exclude_state_does_not_count_as_journaled() {
        // Pre-change WALs decode with `exclude: None`; that must fall
        // through to the file check, not masquerade as "nothing
        // excluded".
        let timeline = ingest_records(vec![WalRecord::Context(context_at(1_000, 0))]);
        let report = resolve_exclusions(&timeline, Some(&tail(TWO_OBJECT_FILE)));
        assert_eq!(report.provenance, ExclusionProvenance::RecordLost);
    }

    #[test]
    fn empty_timeline_with_no_file_is_unknown() {
        let timeline = ingest_records(Vec::new());
        assert_eq!(
            resolve_exclusions(&timeline, None).provenance,
            ExclusionProvenance::Unknown
        );
    }

    // --- diagnostics ------------------------------------------------

    #[test]
    fn excluded_object_without_a_definition_is_flagged() {
        let timeline = timeline_with(Some(ExcludeState {
            definitions: Some(vec![square("A", 100.0, 100.0)]),
            excluded: vec!["A".to_owned(), "GHOST".to_owned()],
            current: None,
        }));
        let report = resolve_exclusions(&timeline, None);
        assert_eq!(
            report.diagnostics,
            vec![ExclusionDiagnostic::ExcludedObjectUndefined {
                objects: vec!["GHOST".to_owned()],
            }]
        );
        // The exclusion itself is still authoritative.
        assert!(report.is_excluded("GHOST"));
    }

    #[test]
    fn degraded_geometry_is_flagged_and_narrows_the_answer() {
        let mut boxed = square("BIG", 100.0, 100.0);
        boxed.fidelity = PolygonFidelity::BoundingBox { source_points: 900 };
        let mut broken = square("BROKEN", 200.0, 100.0);
        broken.fidelity = PolygonFidelity::Unusable { source_points: 2 };
        broken.polygon.clear();
        let timeline = timeline_with(Some(ExcludeState {
            definitions: Some(vec![boxed, broken]),
            excluded: Vec::new(),
            current: None,
        }));
        let report = resolve_exclusions(&timeline, None);
        assert_eq!(
            report.diagnostics,
            vec![ExclusionDiagnostic::GeometryDegraded {
                objects: vec!["BIG".to_owned(), "BROKEN".to_owned()],
            }]
        );
        assert!(!report.geometry_is_complete());
        // An object with no usable outline can never match.
        assert!(report.objects_at(200.0, 100.0).is_empty());
    }

    #[test]
    fn file_definitions_missing_from_the_wal_are_flagged() {
        let timeline = timeline_with(Some(ExcludeState {
            definitions: Some(vec![square("CUBE_ID_0_COPY_0", 100.0, 100.0)]),
            excluded: Vec::new(),
            current: None,
        }));
        let report = resolve_exclusions(&timeline, Some(&tail(TWO_OBJECT_FILE)));
        assert_eq!(report.provenance, ExclusionProvenance::Journaled);
        assert_eq!(
            report.diagnostics,
            vec![ExclusionDiagnostic::DefinitionsIncomplete {
                objects: vec!["CUBE_ID_1_COPY_0".to_owned()],
            }]
        );
    }

    // --- file scanning ----------------------------------------------

    #[test]
    fn file_scan_reads_define_and_start_lines() {
        let scan = parse_object_definitions(TWO_OBJECT_FILE);
        assert!(!scan.is_empty());
        assert_eq!(scan.unparsed_lines, 0);
        assert_eq!(
            scan.names(),
            vec!["CUBE_ID_0_COPY_0".to_owned(), "CUBE_ID_1_COPY_0".to_owned()]
        );
        let first = &scan.definitions[0];
        assert_eq!(first.center, Some([100.0, 100.0]));
        assert_eq!(first.fidelity, PolygonFidelity::Exact);
        assert_eq!(first.polygon.len(), 4);
        assert_eq!(first.polygon[0], [90.0, 90.0]);
    }

    #[test]
    fn file_scan_handles_start_only_files_and_crlf() {
        // Some pipelines emit only START/END markers; Klipper
        // auto-defines those objects.
        let file = b"G1 X1\r\nEXCLUDE_OBJECT_START NAME=part_a\r\nG1 X2\r\nEXCLUDE_OBJECT_START NAME=part_a\r\nEXCLUDE_OBJECT_START NAME=part_b\r\n";
        let scan = parse_object_definitions(file);
        assert_eq!(scan.names(), vec!["PART_A".to_owned(), "PART_B".to_owned()]);
        assert_eq!(scan.definitions[0], ExcludeObjectDef::name_only("PART_A"));
    }

    #[test]
    fn file_scan_replays_define_reset() {
        // EXCLUDE_OBJECT_DEFINE RESET=<truthy> runs _reset_file(). Note
        // Python truthiness: "0" is a non-empty string and still resets.
        let file = b"EXCLUDE_OBJECT_DEFINE NAME=A\nEXCLUDE_OBJECT_DEFINE RESET=0\nEXCLUDE_OBJECT_DEFINE NAME=B\n";
        assert_eq!(parse_object_definitions(file).names(), vec!["B".to_owned()]);
        // An empty RESET value is falsy in Python, so it does not reset;
        // with no NAME either, the command is a listing request.
        let file = b"EXCLUDE_OBJECT_DEFINE NAME=A\nEXCLUDE_OBJECT_DEFINE RESET=\n";
        assert_eq!(parse_object_definitions(file).names(), vec!["A".to_owned()]);
    }

    #[test]
    fn file_scan_redefinition_replaces_and_bare_lines_are_ignored() {
        let file = b"EXCLUDE_OBJECT_DEFINE NAME=A CENTER=1,2\nEXCLUDE_OBJECT_DEFINE NAME=a CENTER=3,4\nEXCLUDE_OBJECT_DEFINE\nEXCLUDE_OBJECT_DEFINE NAME=\nEXCLUDE_OBJECT_START\nEXCLUDE_OBJECT NAME=A\nEXCLUDE_OBJECT_END\n";
        let scan = parse_object_definitions(file);
        assert_eq!(scan.names(), vec!["A".to_owned()]);
        assert_eq!(scan.definitions[0].center, Some([3.0, 4.0]));
        assert_eq!(scan.unparsed_lines, 0);
    }

    #[test]
    fn file_scan_classifies_bad_geometry_without_dropping_the_object() {
        let file = br"EXCLUDE_OBJECT_DEFINE NAME=BADPOLY POLYGON=[[0,0],[1]]
EXCLUDE_OBJECT_DEFINE NAME=NOTJSON POLYGON=nonsense
EXCLUDE_OBJECT_DEFINE NAME=TOOFEW POLYGON=[[0,0],[1,1]]
EXCLUDE_OBJECT_DEFINE NAME=BADCENTER CENTER=oops
EXCLUDE_OBJECT_DEFINE NAME=SHORTCENTER CENTER=5
";
        let scan = parse_object_definitions(file);
        assert_eq!(scan.definitions.len(), 5);
        assert_eq!(
            scan.definitions[0].fidelity,
            PolygonFidelity::Unusable { source_points: 2 }
        );
        assert_eq!(
            scan.definitions[1].fidelity,
            PolygonFidelity::Unusable { source_points: 0 }
        );
        assert_eq!(
            scan.definitions[2].fidelity,
            PolygonFidelity::Unusable { source_points: 2 }
        );
        assert_eq!(scan.definitions[3].center, None);
        assert_eq!(scan.definitions[4].center, None);
    }

    #[test]
    fn file_scan_counts_unparsable_object_lines() {
        // An unbalanced quote makes Klipper's shlex pass raise; the line
        // still proves the file has cancelable objects.
        let file =
            b"EXCLUDE_OBJECT_DEFINE NAME=\"unterminated\nEXCLUDE_OBJECT_START NAME='also bad\n";
        let scan = parse_object_definitions(file);
        assert_eq!(scan.unparsed_lines, 2);
        assert!(scan.definitions.is_empty());
        assert!(!scan.is_empty(), "unparsed lines still mean 'has objects'");

        let timeline = timeline_with(None);
        let report = resolve_exclusions(&timeline, Some(&tail(file)));
        assert_eq!(report.provenance, ExclusionProvenance::RecordLost);
        assert_eq!(
            report.diagnostics,
            vec![ExclusionDiagnostic::CancellationRecordLost {
                objects: Vec::new()
            }]
        );
    }

    #[test]
    fn file_scan_over_a_huge_polygon_summarizes_it() {
        let mut line = String::from("EXCLUDE_OBJECT_DEFINE NAME=HUGE POLYGON=[");
        for i in 0..=plr_wal::MAX_POLYGON_POINTS {
            if i > 0 {
                line.push(',');
            }
            let _ = write!(line, "[{i},{i}]");
        }
        line.push_str("]\n");
        let scan = parse_object_definitions(line.as_bytes());
        let expected = u32::try_from(plr_wal::MAX_POLYGON_POINTS + 1).unwrap();
        assert_eq!(
            scan.definitions[0].fidelity,
            PolygonFidelity::BoundingBox {
                source_points: expected
            }
        );
        assert_eq!(scan.definitions[0].polygon.len(), 4);
    }

    #[test]
    fn file_scan_ignores_unrelated_lines_cheaply() {
        let file = b"G1 X1 Y1\n; EXCLUDE_OBJECT_DEFINE NAME=COMMENTED\nM104 S200\n";
        // A commented-out define is a comment line, not a command.
        assert!(parse_object_definitions(file).is_empty());
        assert!(parse_object_definitions(b"").is_empty());
        assert!(parse_object_definitions(b"\n\n\n").is_empty());
    }

    // --- point-in-object --------------------------------------------

    #[test]
    fn point_lookup_covers_inside_outside_and_boundary() {
        let sq = square("A", 100.0, 100.0);
        let polygon = &sq.polygon;
        assert!(point_in_polygon(100.0, 100.0, polygon), "centre");
        assert!(point_in_polygon(90.5, 90.5, polygon), "near corner inside");
        assert!(!point_in_polygon(89.0, 100.0, polygon), "outside");
        assert!(!point_in_polygon(100.0, 200.0, polygon), "far outside");
        // Boundary counts as inside: edges and vertices alike.
        assert!(point_in_polygon(90.0, 100.0, polygon), "left edge");
        assert!(point_in_polygon(110.0, 100.0, polygon), "right edge");
        assert!(point_in_polygon(100.0, 90.0, polygon), "bottom edge");
        assert!(point_in_polygon(90.0, 90.0, polygon), "vertex");
        assert!(point_in_polygon(110.0, 110.0, polygon), "opposite vertex");
    }

    #[test]
    fn point_lookup_is_total_on_degenerate_rings() {
        assert!(!point_in_polygon(0.0, 0.0, &[]));
        assert!(!point_in_polygon(0.0, 0.0, &[[0.0, 0.0]]));
        assert!(!point_in_polygon(0.0, 0.0, &[[0.0, 0.0], [1.0, 1.0]]));
        // A ring of repeated points: the on-segment test degenerates to
        // a point test and must not divide by zero.
        let repeated = [[1.0, 1.0], [1.0, 1.0], [1.0, 1.0]];
        assert!(point_in_polygon(1.0, 1.0, &repeated));
        assert!(!point_in_polygon(2.0, 2.0, &repeated));
        // Non-finite inputs never panic and never claim containment.
        let sq = square("A", 0.0, 0.0);
        assert!(!point_in_polygon(f64::NAN, 0.0, &sq.polygon));
        assert!(!point_in_polygon(0.0, f64::INFINITY, &sq.polygon));
        assert!(!point_in_polygon(
            0.0,
            0.0,
            &[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 1.0]]
        ));
    }

    #[test]
    fn point_lookup_handles_concave_and_multi_object_plates() {
        // An L-shaped object: the notch is outside even though it is
        // inside the bounding box.
        let l_shape = ExcludeObjectDef {
            name: "L".to_owned(),
            center: None,
            polygon: vec![
                [0.0, 0.0],
                [10.0, 0.0],
                [10.0, 4.0],
                [4.0, 4.0],
                [4.0, 10.0],
                [0.0, 10.0],
            ],
            fidelity: PolygonFidelity::Exact,
        };
        assert!(point_in_polygon(2.0, 2.0, &l_shape.polygon));
        assert!(!point_in_polygon(8.0, 8.0, &l_shape.polygon), "the notch");

        // Two overlapping objects: both are reported.
        let timeline = timeline_with(Some(ExcludeState {
            definitions: Some(vec![
                square("A", 100.0, 100.0),
                square("B", 105.0, 100.0),
                l_shape,
            ]),
            excluded: vec!["B".to_owned()],
            current: None,
        }));
        let report = resolve_exclusions(&timeline, None);
        let both = report.objects_at(103.0, 100.0);
        assert_eq!(
            both.iter().map(|d| d.name.as_str()).collect::<Vec<_>>(),
            vec!["A", "B"]
        );
        assert_eq!(
            report.object_at(103.0, 100.0).map(|d| d.name.as_str()),
            Some("A")
        );
        // Only B was cancelled, so only B answers the safety question.
        assert_eq!(
            report
                .excluded_object_at(103.0, 100.0)
                .map(|d| d.name.as_str()),
            Some("B")
        );
        // (92, 100) is inside A only, and A was not cancelled.
        assert_eq!(
            report.object_at(92.0, 100.0).map(|d| d.name.as_str()),
            Some("A")
        );
        assert_eq!(report.excluded_object_at(92.0, 100.0), None);
        assert!(report.objects_at(1_000.0, 1_000.0).is_empty());
        assert_eq!(report.object_at(1_000.0, 1_000.0), None);
    }

    #[test]
    fn contexts_without_definitions_still_report_the_excluded_set() {
        // A WAL whose definitions record was never written (e.g. the
        // daemon attached after the header): the exclusion survives,
        // the geometry does not.
        let timeline = timeline_with(Some(ExcludeState {
            definitions: None,
            excluded: vec!["A".to_owned()],
            current: None,
        }));
        let report = resolve_exclusions(&timeline, None);
        assert_eq!(report.provenance, ExclusionProvenance::Journaled);
        assert!(report.definitions.is_empty());
        assert_eq!(
            report.diagnostics,
            vec![ExclusionDiagnostic::ExcludedObjectUndefined {
                objects: vec!["A".to_owned()],
            }]
        );
        assert!(report.geometry_is_complete(), "no definitions, no doubt");
        assert!(report.excluded_definitions().is_empty());
    }

    #[test]
    fn non_finite_contexts_are_dropped_before_the_report_sees_them() {
        // Ingestion drops records with non-finite floats; the exclude
        // payload of such a context must not leak into the report.
        let mut context: Context = context_at(1_000, 0);
        context.gcode.position[0] = f64::NAN;
        context.exclude = Some(Box::new(ExcludeState {
            definitions: Some(vec![square("A", 0.0, 0.0)]),
            excluded: vec!["A".to_owned()],
            current: None,
        }));
        let timeline = ingest_records(vec![WalRecord::Context(context)]);
        assert_eq!(
            resolve_exclusions(&timeline, None).provenance,
            ExclusionProvenance::Unknown
        );
    }
}
