//! The excluded-object picture: what the operator cancelled, how sure
//! we are of it, and which object a point on the bed belongs to.
//!
//! # The question is "are we certain?", not "did we see a cancellation?"
//!
//! Klipper's `[exclude_object]` lets an operator cancel one object
//! mid-print, and operators overwhelmingly do that *because the object
//! failed* — it detached, warped, or turned into spaghetti. Klipper
//! holds that decision only in RAM
//! (`klippy/extras/exclude_object.py`, `_reset_state` / `_reset_file`),
//! so a power loss erases it.
//!
//! The naive design asks the operator only when it saw no cancellation
//! and stays silent when it saw one. That is backwards twice over: a
//! journaled exclusion can still be stale or incomplete, and a journaled
//! empty set is perfectly trustworthy when the log is intact. This
//! module therefore keys confirmation on **uncertainty**:
//!
//! * The recorder journals the excluded set **positively**, from the
//!   first `exclude_object` observation onward — "zero objects excluded
//!   as of t" is a recorded fact, not an absence. Absence of a record is
//!   consequently rare rather than routine.
//! * Every report carries *as of when* exclusion state was last durable
//!   ([`ExclusionReport::observed_mono_ns`]) and how far that lags the
//!   end of the reconstruction's stop window
//!   ([`ExclusionReport::freshness`]). Consumers decide on a number.
//! * [`ExclusionReport::is_conclusive`] is true only when **nothing was
//!   lost and the knowledge is fresh**. Every reason it can be false is
//!   a named [`UncertaintyCause`] attached to an
//!   [`ExclusionDiagnostic::ExclusionStateUncertain`] diagnostic, with
//!   the `at_risk` objects — those defined but not recorded as excluded,
//!   i.e. the ones that might have been cancelled without us knowing.
//!
//! Concretely, a journaled excluded set is **not** authoritative when a
//! subscription gap, socket loss, resubscribe, or
//! [`plr_wal::MarkerKind::ExclusionUpdateLost`] marker postdates the
//! newest exclude-bearing context, when the log's durable tail did not
//! end cleanly, or when the freshness gap exceeds
//! [`ReconstructConfig::exclusion_freshness_horizon`].
//!
//! # Confirmation is per-object, structurally
//!
//! [`ExclusionReport::confirmation`] returns `Some` exactly when the
//! report is not conclusive, and what it returns *is* the per-object
//! payload: every known object with its recorded state
//! ([`ObjectKnowledge`]). A consumer cannot obtain the "must ask" signal
//! without also receiving the list, so the prompt is a per-object
//! selection with known exclusions pre-selected — a yes/no prompt is not
//! a faithful rendering of this type.
//!
//! # Reading definitions out of the print file
//!
//! The definitions live in the file itself: slicers (and Moonraker's
//! object processor) emit an `EXCLUDE_OBJECT_DEFINE` block in the
//! header, and `EXCLUDE_OBJECT_START NAME=` auto-defines any object
//! Klipper has not seen (`cmd_EXCLUDE_OBJECT_START`,
//! `exclude_object.py` lines 199-204). [`parse_object_definitions`]
//! replays both, including `EXCLUDE_OBJECT_DEFINE RESET=1`, using
//! `plr-gcode`'s Klipper-faithful tokenizer.
//!
//! # Point-in-object lookup
//!
//! Downstream consumers (contact-point selection, the resume file) need
//! to answer "is this XY on an object the operator cancelled?".
//! [`ExclusionReport::objects_at`] answers it from the outlines, with
//! boundary points counted as **inside** (the conservative direction: an
//! object's edge is part of the object). Objects whose outline is
//! missing or approximate are called out by
//! [`ExclusionDiagnostic::ExcludedObjectWithoutOutline`] and
//! [`ExclusionDiagnostic::GeometryDegraded`]; never read "no match" as
//! "not on any object" without checking
//! [`ExclusionReport::geometry_is_complete`].
//!
//! Totality: every function here is total — no panics for any input,
//! including NaN coordinates and degenerate rings (property-tested).

use plr_gcode::{parse_line, ByteSpan};
use plr_wal::{ExcludeObjectDef, MarkerKind, PolygonFidelity, ScanEnd};

use crate::config::ReconstructConfig;
use crate::stopset::FileTail;
use crate::timeline::WalTimeline;
use crate::window::StopWindow;

/// Half-width of the on-edge test used by [`point_in_polygon`], in mm.
///
/// A point within this distance of an outline edge counts as inside.
/// 1 µm is far below any slicer's coordinate resolution and far above
/// the rounding error of the cross-product test.
pub const EDGE_TOLERANCE_MM: f64 = 1e-6;

/// Where the excluded-object picture came from.
///
/// This says *what evidence exists*, not whether it can be trusted — for
/// that use [`ExclusionReport::is_conclusive`], which additionally
/// weighs everything the log says was lost.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExclusionProvenance {
    /// The WAL journaled `exclude_object` state, so the excluded set is
    /// a recorded fact as of [`ExclusionReport::observed_mono_ns`] —
    /// including when that fact is "nothing is excluded".
    Journaled,
    /// The WAL journaled no exclude state, and the print file defines no
    /// objects at all. Nothing could have been cancelled.
    NoObjectsDefined,
    /// The WAL journaled no exclude state, but the print file **does**
    /// define objects. Whether the operator cancelled any of them is
    /// unknowable from durable evidence.
    RecordLost,
    /// The WAL journaled no exclude state and the print file could not
    /// be checked (no bytes supplied, or only a tail that does not start
    /// at offset 0), so even the existence of cancelable objects is
    /// unknown.
    Unknown,
}

/// How current the journaled exclusion knowledge is, relative to the end
/// of the reconstruction's stop window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExclusionFreshness {
    /// Exclusion state was durable `gap_s` seconds of print time before
    /// the end of the stop window. A healthy recovery shows a small
    /// positive number: contexts refresh at most once per
    /// `POSITION_CONTEXT_MIN_NS` (1 s) while printing, dump batching
    /// adds ~0.5 s, and the stop window deliberately extends past the
    /// last context by the planning/extension lead.
    Known {
        /// Print-time seconds between the newest journaled exclusion
        /// state and the end of the stop window. Negative values (the
        /// observation postdates the window) are clamped to `0.0`.
        gap_s: f64,
    },
    /// An observation exists but could not be placed on the print-time
    /// axis (no stop window supplied, or the clock correlation could not
    /// map its timestamp), so its age is unknown.
    Unknown,
    /// No journaled observation exists to be fresh or stale.
    NoObservation,
}

/// A named reason the excluded set cannot be treated as authoritative.
///
/// Each variant is a distinct, independently-testable condition;
/// consumers may render them individually.
#[derive(Debug, Clone, PartialEq)]
pub enum UncertaintyCause {
    /// A known observation gap (records dropped under WAL backpressure,
    /// or a resubscription hole) extends past the newest journaled
    /// exclusion state: a cancellation inside it would not be in the
    /// log.
    ObservationGap {
        /// Host-monotonic start of the gap (ns).
        start_mono_ns: u64,
        /// Host-monotonic end of the gap (ns).
        end_mono_ns: u64,
    },
    /// The Klipper API socket dropped after the newest journaled
    /// exclusion state; everything the operator did afterwards is
    /// unobserved.
    SocketLost {
        /// Host-monotonic time of the drop (ns).
        mono_ns: u64,
    },
    /// The daemon re-established its subscriptions after the newest
    /// journaled exclusion state, so a hole precedes the new baseline.
    Resubscribed {
        /// Host-monotonic time of the resubscribe (ns).
        mono_ns: u64,
    },
    /// The daemon journaled that a `Context` carrying an exclusion
    /// **change** was dropped under backpressure
    /// ([`plr_wal::MarkerKind::ExclusionUpdateLost`]) — direct evidence
    /// that a cancellation is missing from the log.
    ExclusionUpdateDropped {
        /// Host-monotonic time of the dropped update (ns).
        mono_ns: u64,
    },
    /// The WAL's durable tail did not end cleanly, which is the normal
    /// shape of a power-loss log: records after the truncation point —
    /// possibly including a cancellation — never became durable.
    LogTailIncomplete {
        /// How the recovery scan stopped.
        scan_end: ScanEnd,
    },
    /// Exclusion knowledge is older than the configured horizon, so a
    /// cancellation in the unrecorded interval cannot be ruled out.
    Stale {
        /// The measured freshness gap, print-time seconds.
        gap_s: f64,
        /// The configured horizon it exceeded, seconds.
        horizon_s: f64,
    },
    /// An observation exists but could not be placed on the print-time
    /// axis, so its age cannot be bounded.
    FreshnessUnknown,
    /// No exclusion state was journaled at all, and the print file
    /// defines cancelable objects: a resume would print every one of
    /// them, including any the operator had cancelled.
    NoRecord,
    /// No exclusion state was journaled and the print file could not be
    /// checked for object definitions.
    FileNotChecked,
}

/// An honest statement about what the exclusion picture does not know.
/// Diagnostics never invalidate the report; they say what a consumer
/// must surface to the operator.
#[derive(Debug, Clone, PartialEq)]
pub enum ExclusionDiagnostic {
    /// The excluded set cannot be treated as authoritative, for the
    /// named reason. Any occurrence of this variant makes
    /// [`ExclusionReport::is_conclusive`] false.
    ExclusionStateUncertain {
        /// Which condition fired.
        cause: UncertaintyCause,
        /// Objects that are defined but not recorded as excluded — the
        /// ones that might have been cancelled without us knowing. Empty
        /// when no definitions are known at all, in which case the
        /// operator has nothing to check against but the plate itself.
        at_risk: Vec<String>,
    },
    /// Names in the excluded set that have no definition, so their
    /// geometry is unknown and point-in-object lookup cannot place them.
    /// Their exclusion is still recorded.
    ExcludedObjectUndefined {
        /// The undefined excluded names.
        objects: Vec<String>,
    },
    /// Excluded objects that *are* defined but carry no usable outline —
    /// the shape `EXCLUDE_OBJECT_START`'s auto-definition produces (name
    /// only), and what a malformed `POLYGON=` degrades to.
    /// [`ExclusionReport::excluded_object_at`] answers `None` for these
    /// **everywhere on the bed**, so a caller filtering contact points
    /// by geometry alone would silently treat a cancelled part as
    /// printable.
    ExcludedObjectWithoutOutline {
        /// The affected excluded object names.
        objects: Vec<String>,
    },
    /// Objects whose journaled outline is not a verbatim copy of what
    /// Klipper reported (bounding-box substitution, an unusable ring, or
    /// a fidelity written by a newer format revision). Geometric answers
    /// about these objects are approximate or unavailable.
    GeometryDegraded {
        /// The affected object names.
        objects: Vec<String>,
    },
    /// Objects the print file defines that the journaled definitions do
    /// not mention. Informational: a consumer replaying
    /// `EXCLUDE_OBJECT_DEFINE` should take the full list from the file.
    DefinitionsIncomplete {
        /// The names present only in the file.
        objects: Vec<String>,
    },
}

/// What the log records about one object's exclusion state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectKnowledge {
    /// Recorded as cancelled as of the report's observation.
    Excluded,
    /// Recorded as *not* cancelled as of the report's observation. This
    /// is a positive fact, not an absence — the recorder journals the
    /// excluded set on every exclude-bearing context.
    Included,
    /// No durable record of this object's exclusion state exists.
    Unrecorded,
}

/// One object and what is known about it: the row of a per-object
/// confirmation prompt.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectState {
    /// Object name, upper-cased as Klipper stores it.
    pub name: String,
    /// The definition, when one is known (journaled or read from the
    /// print file). `None` for an excluded name with no definition.
    pub definition: Option<ExcludeObjectDef>,
    /// What the log says about this object.
    pub knowledge: ObjectKnowledge,
}

impl ObjectState {
    /// Whether a per-object prompt should start with this object
    /// selected for exclusion: exactly the objects already recorded as
    /// cancelled.
    #[must_use]
    pub const fn preselected(&self) -> bool {
        matches!(self.knowledge, ObjectKnowledge::Excluded)
    }
}

/// Everything a consumer needs to ask the operator which objects should
/// stay cancelled. Obtainable only from
/// [`ExclusionReport::confirmation`], and only when the report is not
/// conclusive — so the "must ask" signal and the per-object payload
/// cannot be separated.
#[derive(Debug, Clone, PartialEq)]
pub struct ExclusionConfirmation {
    /// Why confirmation is required. Never empty.
    pub causes: Vec<UncertaintyCause>,
    /// Every known object with its recorded state. Render as a
    /// per-object selection with [`ObjectState::preselected`] rows
    /// pre-ticked; this is deliberately not a yes/no payload.
    pub objects: Vec<ObjectState>,
    /// When exclusion state was last durable (host-monotonic ns), when
    /// anything was ever journaled.
    pub observed_mono_ns: Option<u64>,
    /// How stale that knowledge is.
    pub freshness: ExclusionFreshness,
}

/// The excluded-object picture for one reconstruction.
///
/// Fields are private: "a non-empty excluded set exists only when it was
/// journaled" is enforced by construction, not by convention — nothing
/// outside this module can build a report that claims exclusions it did
/// not read out of the log. Read through the accessors.
#[derive(Debug, Clone, PartialEq)]
pub struct ExclusionReport {
    provenance: ExclusionProvenance,
    definitions: Vec<ExcludeObjectDef>,
    excluded: Vec<String>,
    current_object: Option<String>,
    observed_mono_ns: Option<u64>,
    observed_print_time: Option<f64>,
    freshness: ExclusionFreshness,
    /// A context positively reported Klipper's object list (as opposed
    /// to every context carrying `definitions: None`, which only means
    /// "unchanged"). Needed to tell "Klipper knows no objects" from "we
    /// never received the list".
    definitions_observed: bool,
    /// Whether the print file defines cancelable objects, when the file
    /// could be scanned from byte 0.
    file_defines_objects: Option<bool>,
    diagnostics: Vec<ExclusionDiagnostic>,
}

impl ExclusionReport {
    /// The report for a WAL that journaled nothing and a print file that
    /// was never checked.
    #[must_use]
    pub fn unknown() -> Self {
        Self {
            provenance: ExclusionProvenance::Unknown,
            definitions: Vec::new(),
            excluded: Vec::new(),
            current_object: None,
            observed_mono_ns: None,
            observed_print_time: None,
            freshness: ExclusionFreshness::NoObservation,
            definitions_observed: false,
            file_defines_objects: None,
            diagnostics: vec![ExclusionDiagnostic::ExclusionStateUncertain {
                cause: UncertaintyCause::FileNotChecked,
                at_risk: Vec::new(),
            }],
        }
    }

    /// Where the picture came from.
    #[must_use]
    pub const fn provenance(&self) -> ExclusionProvenance {
        self.provenance
    }

    /// Known object definitions: the journaled ones when the provenance
    /// is [`ExclusionProvenance::Journaled`], otherwise the ones parsed
    /// out of the print file (empty when neither exists).
    #[must_use]
    pub fn definitions(&self) -> &[ExcludeObjectDef] {
        &self.definitions
    }

    /// Names recorded as cancelled, in Klipper's sorted order.
    ///
    /// Non-empty only when the provenance is
    /// [`ExclusionProvenance::Journaled`]. Read it together with
    /// [`is_conclusive`](Self::is_conclusive): a recorded set can still
    /// be stale or incomplete.
    #[must_use]
    pub fn excluded(&self) -> &[String] {
        &self.excluded
    }

    /// The object being printed at the newest journaled context, when
    /// one was active (`EXCLUDE_OBJECT_START` with no matching `END`).
    #[must_use]
    pub fn current_object(&self) -> Option<&str> {
        self.current_object.as_deref()
    }

    /// Host-monotonic time (ns) of the newest context that carried
    /// exclude state — *as of when* the excluded set is known.
    #[must_use]
    pub const fn observed_mono_ns(&self) -> Option<u64> {
        self.observed_mono_ns
    }

    /// The same instant on the print-time axis, when the stop window's
    /// clock correlation could place it.
    #[must_use]
    pub const fn observed_print_time(&self) -> Option<f64> {
        self.observed_print_time
    }

    /// How current the journaled knowledge is.
    #[must_use]
    pub const fn freshness(&self) -> ExclusionFreshness {
        self.freshness
    }

    /// Everything the report cannot vouch for.
    #[must_use]
    pub fn diagnostics(&self) -> &[ExclusionDiagnostic] {
        &self.diagnostics
    }

    /// Every named reason the excluded set is not authoritative, in
    /// detection order. Empty exactly when the report is conclusive.
    #[must_use]
    pub fn uncertainty_causes(&self) -> Vec<&UncertaintyCause> {
        self.diagnostics
            .iter()
            .filter_map(|d| match d {
                ExclusionDiagnostic::ExclusionStateUncertain { cause, .. } => Some(cause),
                _ => None,
            })
            .collect()
    }

    /// `true` when nothing was lost and the knowledge is fresh, so the
    /// excluded set may be acted on without asking the operator.
    ///
    /// This is the predicate to gate an automatic resume on — not
    /// [`provenance`](Self::provenance), which only says what evidence
    /// exists.
    #[must_use]
    pub fn is_conclusive(&self) -> bool {
        self.uncertainty_causes().is_empty()
    }

    /// `true` when a resume must not proceed without the operator
    /// confirming which objects stay cancelled.
    #[must_use]
    pub fn requires_operator_confirmation(&self) -> bool {
        !self.is_conclusive()
    }

    /// The per-object confirmation payload, or `None` when the report is
    /// conclusive and no prompt is needed.
    #[must_use]
    pub fn confirmation(&self) -> Option<ExclusionConfirmation> {
        if self.is_conclusive() {
            return None;
        }
        Some(ExclusionConfirmation {
            causes: self.uncertainty_causes().into_iter().cloned().collect(),
            objects: self.object_states(),
            observed_mono_ns: self.observed_mono_ns,
            freshness: self.freshness,
        })
    }

    /// Every known object with its recorded state: the definitions in
    /// order, followed by any excluded name that has no definition.
    ///
    /// Objects are [`ObjectKnowledge::Unrecorded`] whenever no exclusion
    /// state was journaled — a defined object in a `RecordLost` report
    /// is *not* "included", it is unknown.
    #[must_use]
    pub fn object_states(&self) -> Vec<ObjectState> {
        let journaled = self.provenance == ExclusionProvenance::Journaled;
        let mut states: Vec<ObjectState> = self
            .definitions
            .iter()
            .map(|def| ObjectState {
                name: def.name.clone(),
                definition: Some(def.clone()),
                knowledge: knowledge_of(journaled, self.is_excluded(&def.name)),
            })
            .collect();
        for name in &self.excluded {
            if !states.iter().any(|s| s.name.eq_ignore_ascii_case(name)) {
                states.push(ObjectState {
                    name: name.clone(),
                    definition: None,
                    knowledge: ObjectKnowledge::Excluded,
                });
            }
        }
        states
    }

    /// `true` when `name` is recorded as cancelled. Comparison is
    /// case-insensitive; Klipper upper-cases every name it stores
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
    /// alphabetically first match).
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
    /// [`is_conclusive`](Self::is_conclusive) and
    /// [`geometry_is_complete`](Self::geometry_is_complete) allow: an
    /// excluded object with no outline answers `None` everywhere.
    #[must_use]
    pub fn excluded_object_at(&self, x: f64, y: f64) -> Option<&ExcludeObjectDef> {
        self.definitions
            .iter()
            .find(|def| self.is_excluded(&def.name) && point_in_polygon(x, y, &def.polygon))
    }

    /// `true` when every cancelled object carries a verbatim outline, so
    /// a "not on a cancelled object" answer is trustworthy. `false` when
    /// any excluded object is undefined, outline-less, or approximated.
    #[must_use]
    pub fn geometry_is_complete(&self) -> bool {
        self.excluded.iter().all(|name| {
            self.definition(name)
                .is_some_and(|def| def.fidelity == PolygonFidelity::Exact)
        })
    }

    /// The definitions of the cancelled objects, in definition order.
    #[must_use]
    pub fn excluded_definitions(&self) -> Vec<&ExcludeObjectDef> {
        self.definitions
            .iter()
            .filter(|def| self.is_excluded(&def.name))
            .collect()
    }

    /// Defined objects not recorded as excluded — the ones a lost or
    /// stale record might have hidden a cancellation for.
    fn at_risk(&self) -> Vec<String> {
        self.definitions
            .iter()
            .filter(|def| !self.is_excluded(&def.name))
            .map(|def| def.name.clone())
            .collect()
    }
}

/// Maps "was anything journaled" plus "is this name in the set" to the
/// per-object knowledge state.
const fn knowledge_of(journaled: bool, excluded: bool) -> ObjectKnowledge {
    match (journaled, excluded) {
        (_, true) => ObjectKnowledge::Excluded,
        (true, false) => ObjectKnowledge::Included,
        (false, false) => ObjectKnowledge::Unrecorded,
    }
}

/// The reconstruction context [`resolve_exclusions`] needs to judge how
/// fresh the journaled exclusion knowledge is, and to check the print
/// file for object definitions.
#[derive(Debug, Clone, Copy, Default)]
pub struct ExclusionInputs<'a> {
    /// The computed stop window, used to place the newest exclude-
    /// bearing context on the print-time axis. `None` makes freshness
    /// [`ExclusionFreshness::Unknown`], which is itself an uncertainty.
    pub window: Option<&'a StopWindow>,
    /// End of the possible-stop set's evaluation span, print-time
    /// seconds ([`crate::PossibleStopSet::wal_eval_end`]). The freshness
    /// gap is measured to here.
    pub stop_end_print_time: Option<f64>,
    /// Print-file bytes. Must start at offset 0 for the
    /// object-definition scan to be conclusive.
    pub file: Option<&'a FileTail<'a>>,
}

/// Builds the excluded-object picture from the WAL timeline, the
/// reconstruction's time bounds, and (when available) the print file.
///
/// Total: never fails, never panics, for any inputs.
#[must_use]
pub fn resolve_exclusions(
    timeline: &WalTimeline,
    inputs: &ExclusionInputs<'_>,
    config: &ReconstructConfig,
) -> ExclusionReport {
    let journaled = journaled_state(timeline);
    let file_scan = inputs
        .file
        .filter(|tail| tail.base_offset == 0)
        .map(|tail| parse_object_definitions(tail.bytes));

    let mut report = match journaled {
        Some(state) => journaled_report(state, file_scan.as_ref(), inputs),
        None => match file_scan {
            // Diagnostics are appended uniformly below; the standalone
            // `unknown()` constructor carries its own for callers that
            // build one directly.
            None => ExclusionReport {
                provenance: ExclusionProvenance::Unknown,
                definitions: Vec::new(),
                excluded: Vec::new(),
                current_object: None,
                observed_mono_ns: None,
                observed_print_time: None,
                freshness: ExclusionFreshness::NoObservation,
                definitions_observed: false,
                file_defines_objects: None,
                diagnostics: Vec::new(),
            },
            Some(scan) if scan.is_empty() => ExclusionReport {
                provenance: ExclusionProvenance::NoObjectsDefined,
                definitions: Vec::new(),
                excluded: Vec::new(),
                current_object: None,
                observed_mono_ns: None,
                observed_print_time: None,
                freshness: ExclusionFreshness::NoObservation,
                definitions_observed: false,
                file_defines_objects: Some(false),
                diagnostics: Vec::new(),
            },
            Some(scan) => ExclusionReport {
                provenance: ExclusionProvenance::RecordLost,
                definitions: scan.definitions,
                excluded: Vec::new(),
                current_object: None,
                observed_mono_ns: None,
                observed_print_time: None,
                freshness: ExclusionFreshness::NoObservation,
                definitions_observed: false,
                file_defines_objects: Some(true),
                diagnostics: Vec::new(),
            },
        },
    };

    // Uncertainty first — it decides whether a prompt is needed — then
    // the geometry notes gathered while assembling the report.
    let at_risk = report.at_risk();
    let mut diagnostics: Vec<ExclusionDiagnostic> = uncertainty_causes(&report, timeline, config)
        .into_iter()
        .map(|cause| ExclusionDiagnostic::ExclusionStateUncertain {
            cause,
            at_risk: at_risk.clone(),
        })
        .collect();
    diagnostics.append(&mut report.diagnostics);
    report.diagnostics = diagnostics;
    report
}

/// Collects every named reason this report cannot be trusted.
fn uncertainty_causes(
    report: &ExclusionReport,
    timeline: &WalTimeline,
    config: &ReconstructConfig,
) -> Vec<UncertaintyCause> {
    let mut causes = Vec::new();
    match report.provenance {
        // Nothing to cancel: the empty set is correct by construction.
        ExclusionProvenance::NoObjectsDefined => return causes,
        ExclusionProvenance::RecordLost => {
            causes.push(UncertaintyCause::NoRecord);
            return causes;
        }
        ExclusionProvenance::Unknown => {
            causes.push(UncertaintyCause::FileNotChecked);
            return causes;
        }
        ExclusionProvenance::Journaled => {}
    }

    // Nothing to cancel, positively established: Klipper reported an
    // empty object list, nothing is excluded, and the print file
    // defines no objects either. No lost record could have hidden a
    // cancellation of an object that does not exist, so a torn tail
    // must not drag the operator into a prompt with an empty list.
    if report.definitions.is_empty()
        && report.excluded.is_empty()
        && report.definitions_observed
        && report.file_defines_objects == Some(false)
    {
        return causes;
    }

    // Anything the log says happened *after* our newest exclusion
    // knowledge could have carried a cancellation we never saw.
    let observed = report.observed_mono_ns.unwrap_or(0);
    for marker in &timeline.markers {
        match marker.kind {
            MarkerKind::SubscriptionGap {
                start_mono_ns,
                end_mono_ns,
            } if end_mono_ns > observed => causes.push(UncertaintyCause::ObservationGap {
                start_mono_ns,
                end_mono_ns,
            }),
            MarkerKind::SocketLost if marker.mono_ns > observed => {
                causes.push(UncertaintyCause::SocketLost {
                    mono_ns: marker.mono_ns,
                });
            }
            MarkerKind::Resubscribed if marker.mono_ns > observed => {
                causes.push(UncertaintyCause::Resubscribed {
                    mono_ns: marker.mono_ns,
                });
            }
            MarkerKind::ExclusionUpdateLost if marker.mono_ns > observed => {
                causes.push(UncertaintyCause::ExclusionUpdateDropped {
                    mono_ns: marker.mono_ns,
                });
            }
            _ => {}
        }
    }

    // A power-loss log normally ends mid-frame: records after the
    // truncation point — possibly a cancellation — never became durable.
    if timeline.scan_end != ScanEnd::CleanEof {
        causes.push(UncertaintyCause::LogTailIncomplete {
            scan_end: timeline.scan_end.clone(),
        });
    }

    match report.freshness {
        ExclusionFreshness::Known { gap_s } if gap_s > config.exclusion_freshness_horizon => {
            causes.push(UncertaintyCause::Stale {
                gap_s,
                horizon_s: config.exclusion_freshness_horizon,
            });
        }
        ExclusionFreshness::Known { .. } => {}
        ExclusionFreshness::Unknown | ExclusionFreshness::NoObservation => {
            causes.push(UncertaintyCause::FreshnessUnknown);
        }
    }
    causes
}

/// The merged exclude state carried by the WAL's contexts.
struct JournaledState {
    definitions: Vec<ExcludeObjectDef>,
    /// A context carried `definitions: Some(_)` — Klipper's object list
    /// was positively reported, not merely never refreshed.
    definitions_observed: bool,
    excluded: Vec<String>,
    current: Option<String>,
    observed_mono_ns: u64,
}

/// Merges every context's exclude payload in append order. Definitions
/// are journaled only when they change ([`plr_wal::ExcludeState`]), so
/// the newest `Some(..)` wins and later contexts carry the set forward.
/// Returns `None` when no context carried exclude state at all.
fn journaled_state(timeline: &WalTimeline) -> Option<JournaledState> {
    let mut seen = false;
    let mut state = JournaledState {
        definitions: Vec::new(),
        definitions_observed: false,
        excluded: Vec::new(),
        current: None,
        observed_mono_ns: 0,
    };
    for context in &timeline.contexts {
        let Some(exclude) = context.exclude.as_deref() else {
            continue;
        };
        seen = true;
        if let Some(definitions) = &exclude.definitions {
            state.definitions.clone_from(definitions);
            state.definitions_observed = true;
        }
        state.excluded.clone_from(&exclude.excluded);
        state.current.clone_from(&exclude.current);
        state.observed_mono_ns = state.observed_mono_ns.max(context.mono_ns);
    }
    seen.then_some(state)
}

/// Assembles the report for a WAL that journaled exclude state,
/// cross-checking the print file when it was scannable.
fn journaled_report(
    state: JournaledState,
    file_scan: Option<&FileObjectScan>,
    inputs: &ExclusionInputs<'_>,
) -> ExclusionReport {
    let observed_print_time = inputs
        .window
        .and_then(|window| window.mono_ns_to_print_time(state.observed_mono_ns))
        .filter(|pt| pt.is_finite());
    let freshness = match (observed_print_time, inputs.stop_end_print_time) {
        (Some(observed), Some(end)) if end.is_finite() => ExclusionFreshness::Known {
            gap_s: (end - observed).max(0.0),
        },
        _ => ExclusionFreshness::Unknown,
    };

    let mut report = ExclusionReport {
        provenance: ExclusionProvenance::Journaled,
        definitions: state.definitions,
        excluded: state.excluded,
        current_object: state.current,
        observed_mono_ns: Some(state.observed_mono_ns),
        observed_print_time,
        freshness,
        definitions_observed: state.definitions_observed,
        file_defines_objects: file_scan.map(|scan| !scan.is_empty()),
        diagnostics: Vec::new(),
    };

    let undefined: Vec<String> = report
        .excluded
        .iter()
        .filter(|name| report.definition(name).is_none())
        .cloned()
        .collect();
    if !undefined.is_empty() {
        report
            .diagnostics
            .push(ExclusionDiagnostic::ExcludedObjectUndefined { objects: undefined });
    }

    // An excluded object with no usable outline makes every geometric
    // "is this point on a cancelled part?" query answer `None`.
    let outline_less: Vec<String> = report
        .excluded_definitions()
        .iter()
        .filter(|def| def.polygon.len() < 3)
        .map(|def| def.name.clone())
        .collect();
    if !outline_less.is_empty() {
        report
            .diagnostics
            .push(ExclusionDiagnostic::ExcludedObjectWithoutOutline {
                objects: outline_less,
            });
    }

    let degraded: Vec<String> = report
        .definitions
        .iter()
        .filter(|def| def.fidelity.is_degraded())
        .map(|def| def.name.clone())
        .collect();
    if !degraded.is_empty() {
        report
            .diagnostics
            .push(ExclusionDiagnostic::GeometryDegraded { objects: degraded });
    }

    if let Some(scan) = file_scan {
        let missing: Vec<String> = scan
            .definitions
            .iter()
            .filter(|def| report.definition(&def.name).is_none())
            .map(|def| def.name.clone())
            .collect();
        if !missing.is_empty() {
            report
                .diagnostics
                .push(ExclusionDiagnostic::DefinitionsIncomplete { objects: missing });
        }
    }

    report
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
    /// they are counted rather than ignored. `EXCLUDE_OBJECT_DEFINE
    /// RESET=1` clears the count along with the definitions, mirroring
    /// Klipper's `_reset_file()`.
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

/// Scans print-file bytes for the object definitions Klipper would build
/// from them.
///
/// Replays the three commands that mutate `exclude_object.objects`:
///
/// * `EXCLUDE_OBJECT_DEFINE NAME=<n> [CENTER=x,y] [POLYGON=[[x,y],...]]`
///   — adds `n.upper()` with the parsed geometry. `CENTER` is read as
///   `json.loads('[%s]' % value)` and `POLYGON` as `json.loads(value)`,
///   matching `exclude_object.py` lines 256-270.
/// * `EXCLUDE_OBJECT_DEFINE RESET=<anything non-empty>` —
///   `_reset_file()`, clearing everything found so far. Klipper's test
///   is Python truthiness on the raw string, so `RESET=0` resets too.
/// * `EXCLUDE_OBJECT_START NAME=<n>` — adds a name-only object when `n`
///   is not already known (`exclude_object.py` lines 199-204).
///
/// Lines are pre-filtered on the ASCII substring `EXCLUDE_OBJECT` before
/// being tokenized, so scanning a whole multi-megabyte print file costs
/// one pass of substring search plus a parse of the handful of matching
/// lines.
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
    // (including "0") triggers `_reset_file()`, which clears everything
    // learned so far — including lines we could not parse.
    if command.get("RESET").is_some_and(|value| !value.is_empty()) {
        scan.definitions.clear();
        scan.unparsed_lines = 0;
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

/// Case-insensitive ASCII substring test for `EXCLUDE_OBJECT`, the cheap
/// pre-filter that keeps whole-file scanning affordable.
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
/// Even-odd (ray-casting) rule, matching how a slicer's outline is meant
/// to be read. Total: returns `false` rather than panicking for a
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

    use plr_wal::{
        Context, ExcludeState, Marker, MarkerKind, PolygonFidelity, RecoveryScan, ScanEnd,
        WalRecord,
    };

    use super::{
        parse_object_definitions, point_in_polygon, resolve_exclusions, ExcludeObjectDef,
        ExclusionDiagnostic, ExclusionFreshness, ExclusionInputs, ExclusionProvenance,
        ExclusionReport, ObjectKnowledge, UncertaintyCause,
    };
    use crate::config::ReconstructConfig;
    use crate::stopset::FileTail;
    use crate::testutil::{context_at, heartbeat_at, ingest_records, scan_of};
    use crate::timeline::{ingest, WalTimeline};
    use crate::window::{compute_stop_window, StopWindow};

    /// A 20 mm square centred on `(cx, cy)`.
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

    fn exclude_state(defs: Option<Vec<ExcludeObjectDef>>, excluded: &[&str]) -> ExcludeState {
        ExcludeState {
            definitions: defs,
            excluded: excluded.iter().map(|s| (*s).to_owned()).collect(),
            current: None,
        }
    }

    /// A timeline holding one exclude-bearing context at `mono_ns`,
    /// ending cleanly with no lifecycle markers.
    fn timeline_with(exclude: Option<ExcludeState>) -> WalTimeline {
        let mut context = context_at(1_000, 0);
        context.exclude = exclude.map(Box::new);
        ingest_records(vec![WalRecord::Context(context)])
    }

    /// A timeline with an exclude-bearing context at `mono_ns` plus the
    /// given markers, and an explicit scan end.
    fn timeline_with_markers(
        context_mono_ns: u64,
        exclude: ExcludeState,
        markers: &[(u64, MarkerKind)],
        end: ScanEnd,
    ) -> WalTimeline {
        let mut context = context_at(context_mono_ns, 0);
        context.exclude = Some(Box::new(exclude));
        let mut records = vec![WalRecord::Context(context)];
        for (mono_ns, kind) in markers {
            records.push(WalRecord::Marker(Marker {
                mono_ns: *mono_ns,
                kind: kind.clone(),
            }));
        }
        let mut scan = scan_of(records);
        scan.end = end;
        ingest(&scan, None)
    }

    fn tail(bytes: &[u8]) -> FileTail<'_> {
        FileTail {
            base_offset: 0,
            bytes,
        }
    }

    fn cfg() -> ReconstructConfig {
        ReconstructConfig::default()
    }

    /// Inputs with no window and no file: freshness is unknowable.
    fn bare() -> ExclusionInputs<'static> {
        ExclusionInputs::default()
    }

    /// Inputs whose freshness places the observation `gap_s` before the
    /// end of the stop window, using a real correlated stop window.
    fn window_for(mono_ns: u64, print_time: f64) -> StopWindow {
        let scan = scan_of(vec![
            WalRecord::Heartbeat(heartbeat_at(mono_ns, print_time)),
            WalRecord::Context(context_at(mono_ns, 0)),
        ]);
        let timeline = ingest(&scan, None);
        compute_stop_window(&timeline, None, &cfg()).expect("window")
    }

    /// A conclusive baseline: journaled state, clean scan, no markers,
    /// fresh knowledge.
    fn conclusive_case() -> (WalTimeline, StopWindow) {
        let timeline = timeline_with_markers(
            1_000_000_000,
            exclude_state(Some(vec![square("A", 100.0, 100.0)]), &["A"]),
            &[],
            ScanEnd::CleanEof,
        );
        let window = window_for(1_000_000_000, 10.0);
        (timeline, window)
    }

    fn resolve(
        timeline: &WalTimeline,
        window: &StopWindow,
        stop_end: f64,
        file: Option<&FileTail<'_>>,
    ) -> ExclusionReport {
        resolve_exclusions(
            timeline,
            &ExclusionInputs {
                window: Some(window),
                stop_end_print_time: Some(stop_end),
                file,
            },
            &cfg(),
        )
    }

    const TWO_OBJECT_FILE: &[u8] = b"; generated by a slicer\n\
EXCLUDE_OBJECT_DEFINE NAME=Cube_id_0_copy_0 CENTER=100,100 POLYGON=[[90,90],[110,90],[110,110],[90,110]]\n\
EXCLUDE_OBJECT_DEFINE NAME=Cube_id_1_copy_0 CENTER=150,100 POLYGON=[[140,90],[160,90],[160,110],[140,110]]\n\
G1 X10 Y10 F3000\n\
EXCLUDE_OBJECT_START NAME=Cube_id_0_copy_0\n\
G1 X100 Y100 E1\n\
EXCLUDE_OBJECT_END NAME=Cube_id_0_copy_0\n";

    // --- conclusiveness ---------------------------------------------

    #[test]
    fn a_clean_fresh_journaled_log_is_conclusive() {
        let (timeline, window) = conclusive_case();
        let report = resolve(&timeline, &window, 10.5, None);
        assert_eq!(report.provenance(), ExclusionProvenance::Journaled);
        assert_eq!(report.excluded(), ["A".to_owned()]);
        assert_eq!(report.observed_mono_ns(), Some(1_000_000_000));
        assert_eq!(report.freshness(), ExclusionFreshness::Known { gap_s: 0.5 });
        assert!(report.is_conclusive());
        assert!(!report.requires_operator_confirmation());
        assert_eq!(report.uncertainty_causes(), Vec::<&UncertaintyCause>::new());
        assert_eq!(report.confirmation(), None, "no prompt when certain");
    }

    #[test]
    fn a_journaled_empty_set_is_conclusive_too() {
        // The positive-journaling half of the design: "zero objects
        // excluded as of t" is a fact, and a clean fresh log must not
        // pester the operator about it.
        let timeline = timeline_with_markers(
            1_000_000_000,
            exclude_state(Some(vec![square("A", 100.0, 100.0)]), &[]),
            &[],
            ScanEnd::CleanEof,
        );
        let window = window_for(1_000_000_000, 10.0);
        let report = resolve(&timeline, &window, 10.5, None);
        assert!(report.excluded().is_empty());
        assert!(report.is_conclusive());
        let states = report.object_states();
        assert_eq!(states.len(), 1);
        assert_eq!(states[0].knowledge, ObjectKnowledge::Included);
        assert!(!states[0].preselected());
    }

    #[test]
    fn a_plate_with_no_objects_never_prompts_however_torn_the_log() {
        // The common case on a printer that merely *has*
        // [exclude_object] configured: Klipper reports an empty object
        // list, the file defines none, and a power loss tears the tail.
        // There is nothing a lost record could have cancelled, so
        // dragging the operator into an empty prompt would be pure
        // confirmation fatigue.
        let timeline = timeline_with_markers(
            1_000_000_000,
            exclude_state(Some(Vec::new()), &[]),
            &[
                (1_500_000_000, MarkerKind::SocketLost),
                (1_600_000_000, MarkerKind::ExclusionUpdateLost),
            ],
            ScanEnd::FrameCrcMismatch,
        );
        let window = window_for(1_000_000_000, 10.0);
        let file = tail(
            b"G28
G1 X10 Y10 F3000
",
        );
        let report = resolve(&timeline, &window, 400.0, Some(&file));
        assert_eq!(report.provenance(), ExclusionProvenance::Journaled);
        assert!(
            report.is_conclusive(),
            "no objects means nothing to confirm: {:?}",
            report.uncertainty_causes()
        );
        assert_eq!(report.confirmation(), None);
        assert!(report.object_states().is_empty());
    }

    #[test]
    fn an_empty_object_list_still_prompts_when_it_was_never_confirmed() {
        // Same shape, but the definitions record itself never arrived
        // (every context carried `definitions: None`), so "no objects"
        // is an absence, not a fact — and the file was not checkable.
        let timeline = timeline_with_markers(
            1_000_000_000,
            exclude_state(None, &[]),
            &[],
            ScanEnd::FrameCrcMismatch,
        );
        let window = window_for(1_000_000_000, 10.0);
        let report = resolve(&timeline, &window, 10.5, None);
        assert!(!report.is_conclusive());
        // ... and it stays a prompt when the file *does* define objects.
        let file = tail(TWO_OBJECT_FILE);
        let report = resolve(&timeline, &window, 10.5, Some(&file));
        assert!(!report.is_conclusive());
        assert!(report
            .diagnostics()
            .contains(&ExclusionDiagnostic::DefinitionsIncomplete {
                objects: vec!["CUBE_ID_0_COPY_0".to_owned(), "CUBE_ID_1_COPY_0".to_owned()],
            }));
    }

    #[test]
    fn an_observation_gap_after_the_exclusion_state_defeats_conclusiveness() {
        // The blocker: WalSender drops context records under
        // backpressure and journals the hole. A cancellation inside it
        // is simply not in the log, so the surviving (empty) set must
        // not be reported as authoritative.
        let timeline = timeline_with_markers(
            1_000_000_000,
            exclude_state(Some(vec![square("A", 100.0, 100.0)]), &[]),
            &[(
                2_000_000_000,
                MarkerKind::SubscriptionGap {
                    start_mono_ns: 1_500_000_000,
                    end_mono_ns: 2_000_000_000,
                },
            )],
            ScanEnd::CleanEof,
        );
        let window = window_for(1_000_000_000, 10.0);
        let report = resolve(&timeline, &window, 10.5, None);
        assert!(!report.is_conclusive());
        assert_eq!(
            report.uncertainty_causes(),
            vec![&UncertaintyCause::ObservationGap {
                start_mono_ns: 1_500_000_000,
                end_mono_ns: 2_000_000_000,
            }]
        );
        // ... and it names the objects that might have been cancelled.
        assert_eq!(
            report.diagnostics(),
            [ExclusionDiagnostic::ExclusionStateUncertain {
                cause: UncertaintyCause::ObservationGap {
                    start_mono_ns: 1_500_000_000,
                    end_mono_ns: 2_000_000_000,
                },
                at_risk: vec!["A".to_owned()],
            }]
        );
    }

    #[test]
    fn a_gap_that_predates_the_exclusion_state_does_not() {
        // Knowledge recorded *after* the hole supersedes it.
        let timeline = timeline_with_markers(
            3_000_000_000,
            exclude_state(Some(vec![square("A", 100.0, 100.0)]), &[]),
            &[(
                1_000_000_000,
                MarkerKind::SubscriptionGap {
                    start_mono_ns: 500_000_000,
                    end_mono_ns: 1_000_000_000,
                },
            )],
            ScanEnd::CleanEof,
        );
        let window = window_for(3_000_000_000, 10.0);
        let report = resolve(&timeline, &window, 10.5, None);
        assert!(report.is_conclusive(), "{:?}", report.uncertainty_causes());
    }

    #[test]
    fn socket_loss_and_resubscribe_after_the_state_defeat_conclusiveness() {
        for (kind, expected) in [
            (
                MarkerKind::SocketLost,
                UncertaintyCause::SocketLost {
                    mono_ns: 2_000_000_000,
                },
            ),
            (
                MarkerKind::Resubscribed,
                UncertaintyCause::Resubscribed {
                    mono_ns: 2_000_000_000,
                },
            ),
        ] {
            let timeline = timeline_with_markers(
                1_000_000_000,
                exclude_state(Some(vec![square("A", 100.0, 100.0)]), &[]),
                &[(2_000_000_000, kind)],
                ScanEnd::CleanEof,
            );
            let window = window_for(1_000_000_000, 10.0);
            let report = resolve(&timeline, &window, 10.5, None);
            assert!(!report.is_conclusive());
            assert_eq!(report.uncertainty_causes(), vec![&expected]);
        }
    }

    #[test]
    fn a_dropped_exclusion_update_marker_defeats_conclusiveness() {
        // The undroppable evidence WalSender emits when a context
        // carrying a cancellation loses the backpressure race.
        let timeline = timeline_with_markers(
            1_000_000_000,
            exclude_state(Some(vec![square("A", 100.0, 100.0)]), &[]),
            &[(1_500_000_000, MarkerKind::ExclusionUpdateLost)],
            ScanEnd::CleanEof,
        );
        let window = window_for(1_000_000_000, 10.0);
        let report = resolve(&timeline, &window, 10.5, None);
        assert!(!report.is_conclusive());
        assert_eq!(
            report.uncertainty_causes(),
            vec![&UncertaintyCause::ExclusionUpdateDropped {
                mono_ns: 1_500_000_000,
            }]
        );
    }

    #[test]
    fn a_torn_log_tail_defeats_conclusiveness() {
        // The normal shape of a power-loss WAL: records after the
        // truncation point never became durable, and one of them could
        // have been the cancellation.
        for end in [
            ScanEnd::TruncatedPayload,
            ScanEnd::FrameCrcMismatch,
            ScanEnd::TruncatedFrameHeader,
        ] {
            let timeline = timeline_with_markers(
                1_000_000_000,
                exclude_state(Some(vec![square("A", 100.0, 100.0)]), &[]),
                &[],
                end.clone(),
            );
            let window = window_for(1_000_000_000, 10.0);
            let report = resolve(&timeline, &window, 10.5, None);
            assert!(!report.is_conclusive(), "{end:?} must not be conclusive");
            assert_eq!(
                report.uncertainty_causes(),
                vec![&UncertaintyCause::LogTailIncomplete { scan_end: end }]
            );
        }
    }

    #[test]
    fn stale_knowledge_defeats_conclusiveness_and_reports_the_number() {
        let (timeline, window) = conclusive_case();
        // 30 s of print time between the observation and the end of the
        // stop window: far beyond the 5 s default horizon.
        let report = resolve(&timeline, &window, 40.0, None);
        assert_eq!(
            report.freshness(),
            ExclusionFreshness::Known { gap_s: 30.0 }
        );
        assert!(!report.is_conclusive());
        assert_eq!(
            report.uncertainty_causes(),
            vec![&UncertaintyCause::Stale {
                gap_s: 30.0,
                horizon_s: 5.0,
            }]
        );
        // The horizon is configurable: widen it and the same log passes.
        let relaxed = ReconstructConfig {
            exclusion_freshness_horizon: 60.0,
            ..cfg()
        };
        let report = resolve_exclusions(
            &timeline,
            &ExclusionInputs {
                window: Some(&window),
                stop_end_print_time: Some(40.0),
                file: None,
            },
            &relaxed,
        );
        assert!(report.is_conclusive());
    }

    #[test]
    fn unplaceable_knowledge_defeats_conclusiveness() {
        // Without a stop window the observation cannot be dated, so its
        // age cannot be bounded — that is an uncertainty, not a pass.
        let timeline = timeline_with(Some(exclude_state(
            Some(vec![square("A", 100.0, 100.0)]),
            &["A"],
        )));
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        assert_eq!(report.provenance(), ExclusionProvenance::Journaled);
        assert_eq!(report.freshness(), ExclusionFreshness::Unknown);
        assert!(!report.is_conclusive());
        assert_eq!(
            report.uncertainty_causes(),
            vec![&UncertaintyCause::FreshnessUnknown]
        );
        // A negative gap (observation after the window end) clamps.
        let window = window_for(1_000, 10.0);
        let report = resolve(&timeline, &window, -100.0, None);
        assert_eq!(report.freshness(), ExclusionFreshness::Known { gap_s: 0.0 });
    }

    #[test]
    fn every_cause_is_collected_not_just_the_first() {
        let timeline = timeline_with_markers(
            1_000_000_000,
            exclude_state(Some(vec![square("A", 100.0, 100.0)]), &[]),
            &[
                (1_500_000_000, MarkerKind::SocketLost),
                (1_600_000_000, MarkerKind::ExclusionUpdateLost),
            ],
            ScanEnd::TruncatedPayload,
        );
        let window = window_for(1_000_000_000, 10.0);
        let report = resolve(&timeline, &window, 100.0, None);
        let causes = report.uncertainty_causes();
        assert_eq!(causes.len(), 4, "{causes:?}");
        assert!(matches!(causes[0], UncertaintyCause::SocketLost { .. }));
        assert!(matches!(
            causes[1],
            UncertaintyCause::ExclusionUpdateDropped { .. }
        ));
        assert!(matches!(
            causes[2],
            UncertaintyCause::LogTailIncomplete { .. }
        ));
        assert!(matches!(causes[3], UncertaintyCause::Stale { .. }));
    }

    // --- provenance -------------------------------------------------

    #[test]
    fn no_record_and_no_objects_in_file_is_reported_plainly() {
        let timeline = timeline_with(None);
        let file = b"G28\nG1 X10 Y10 F3000\nG1 Z0.2\n";
        let report = resolve_exclusions(
            &timeline,
            &ExclusionInputs {
                file: Some(&tail(file)),
                ..ExclusionInputs::default()
            },
            &cfg(),
        );
        assert_eq!(report.provenance(), ExclusionProvenance::NoObjectsDefined);
        assert!(report.is_conclusive(), "nothing could have been cancelled");
        assert!(report.diagnostics().is_empty());
        assert!(report.object_states().is_empty());
    }

    #[test]
    fn no_record_but_file_defines_objects_is_the_dangerous_case() {
        let timeline = timeline_with(None);
        let report = resolve_exclusions(
            &timeline,
            &ExclusionInputs {
                file: Some(&tail(TWO_OBJECT_FILE)),
                ..ExclusionInputs::default()
            },
            &cfg(),
        );
        assert_eq!(report.provenance(), ExclusionProvenance::RecordLost);
        assert!(report.requires_operator_confirmation());
        assert_eq!(
            report.diagnostics(),
            [ExclusionDiagnostic::ExclusionStateUncertain {
                cause: UncertaintyCause::NoRecord,
                at_risk: vec!["CUBE_ID_0_COPY_0".to_owned(), "CUBE_ID_1_COPY_0".to_owned(),],
            }]
        );
        assert!(report.excluded().is_empty());
        // Neither "excluded" nor "included": simply unrecorded.
        let states = report.object_states();
        assert_eq!(states.len(), 2);
        assert!(states
            .iter()
            .all(|s| s.knowledge == ObjectKnowledge::Unrecorded && !s.preselected()));
        // Geometry from the file still answers point queries.
        assert_eq!(
            report.object_at(100.0, 100.0).map(|d| d.name.as_str()),
            Some("CUBE_ID_0_COPY_0")
        );
    }

    #[test]
    fn no_record_and_unusable_file_is_unknown() {
        let timeline = timeline_with(None);
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        assert_eq!(report.provenance(), ExclusionProvenance::Unknown);
        assert!(report.requires_operator_confirmation());
        assert_eq!(
            report.uncertainty_causes(),
            vec![&UncertaintyCause::FileNotChecked]
        );
        assert_eq!(report, ExclusionReport::unknown());

        // A tail that does not start at byte 0 cannot prove the header
        // held no EXCLUDE_OBJECT_DEFINE block.
        let partial = FileTail {
            base_offset: 4_096,
            bytes: b"G1 X1\n",
        };
        let report = resolve_exclusions(
            &timeline,
            &ExclusionInputs {
                file: Some(&partial),
                ..ExclusionInputs::default()
            },
            &cfg(),
        );
        assert_eq!(report.provenance(), ExclusionProvenance::Unknown);
    }

    #[test]
    fn a_context_without_exclude_state_does_not_count_as_journaled() {
        // Pre-change WALs decode with `exclude: None`; that must fall
        // through to the file check, not masquerade as "nothing
        // excluded".
        let timeline = ingest_records(vec![WalRecord::Context(context_at(1_000, 0))]);
        let report = resolve_exclusions(
            &timeline,
            &ExclusionInputs {
                file: Some(&tail(TWO_OBJECT_FILE)),
                ..ExclusionInputs::default()
            },
            &cfg(),
        );
        assert_eq!(report.provenance(), ExclusionProvenance::RecordLost);
    }

    #[test]
    fn definitions_are_carried_forward_and_the_newest_context_dates_the_report() {
        let mut first = context_at(1_000, 0);
        first.exclude = Some(Box::new(exclude_state(
            Some(vec![square("A", 100.0, 100.0)]),
            &[],
        )));
        let mut second = context_at(2_000, 10);
        second.exclude = Some(Box::new(exclude_state(None, &["A"])));
        let timeline = ingest_records(vec![WalRecord::Context(first), WalRecord::Context(second)]);
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        assert_eq!(report.definitions().len(), 1);
        assert_eq!(report.excluded(), ["A".to_owned()]);
        assert_eq!(report.observed_mono_ns(), Some(2_000));
    }

    #[test]
    fn empty_timeline_with_no_file_is_unknown() {
        let timeline = ingest_records(Vec::new());
        assert_eq!(
            resolve_exclusions(&timeline, &bare(), &cfg()).provenance(),
            ExclusionProvenance::Unknown
        );
    }

    #[test]
    fn non_finite_contexts_are_dropped_before_the_report_sees_them() {
        let mut context: Context = context_at(1_000, 0);
        context.gcode.position[0] = f64::NAN;
        context.exclude = Some(Box::new(exclude_state(
            Some(vec![square("A", 0.0, 0.0)]),
            &["A"],
        )));
        let timeline = ingest_records(vec![WalRecord::Context(context)]);
        assert_eq!(
            resolve_exclusions(&timeline, &bare(), &cfg()).provenance(),
            ExclusionProvenance::Unknown
        );
    }

    // --- confirmation payload ---------------------------------------

    #[test]
    fn confirmation_is_per_object_with_known_exclusions_preselected() {
        // The user's requirement, made structural: the "must ask" signal
        // cannot be obtained without the per-object list.
        let timeline = timeline_with_markers(
            1_000_000_000,
            exclude_state(
                Some(vec![
                    square("A", 100.0, 100.0),
                    square("B", 150.0, 100.0),
                    square("C", 200.0, 100.0),
                ]),
                &["B"],
            ),
            &[(1_500_000_000, MarkerKind::ExclusionUpdateLost)],
            ScanEnd::CleanEof,
        );
        let window = window_for(1_000_000_000, 10.0);
        let report = resolve(&timeline, &window, 10.5, None);
        let confirmation = report.confirmation().expect("prompt required");
        assert_eq!(confirmation.causes.len(), 1);
        assert_eq!(confirmation.observed_mono_ns, Some(1_000_000_000));
        assert_eq!(
            confirmation.freshness,
            ExclusionFreshness::Known { gap_s: 0.5 }
        );
        // Every object appears, not just the excluded or the at-risk.
        let rows: Vec<(&str, ObjectKnowledge, bool)> = confirmation
            .objects
            .iter()
            .map(|o| (o.name.as_str(), o.knowledge, o.preselected()))
            .collect();
        assert_eq!(
            rows,
            vec![
                ("A", ObjectKnowledge::Included, false),
                ("B", ObjectKnowledge::Excluded, true),
                ("C", ObjectKnowledge::Included, false),
            ]
        );
    }

    #[test]
    fn confirmation_lists_excluded_names_that_have_no_definition() {
        let timeline = timeline_with(Some(exclude_state(
            Some(vec![square("A", 100.0, 100.0)]),
            &["A", "GHOST"],
        )));
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        let confirmation = report.confirmation().expect("freshness unknown");
        let ghost = confirmation
            .objects
            .iter()
            .find(|o| o.name == "GHOST")
            .expect("undefined excluded names still get a row");
        assert_eq!(ghost.knowledge, ObjectKnowledge::Excluded);
        assert!(ghost.preselected());
        assert_eq!(ghost.definition, None);
    }

    // --- geometry diagnostics ---------------------------------------

    #[test]
    fn excluded_object_without_a_definition_is_flagged() {
        let timeline = timeline_with(Some(exclude_state(
            Some(vec![square("A", 100.0, 100.0)]),
            &["A", "GHOST"],
        )));
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        assert!(report
            .diagnostics()
            .contains(&ExclusionDiagnostic::ExcludedObjectUndefined {
                objects: vec!["GHOST".to_owned()],
            }));
        assert!(report.is_excluded("GHOST"));
        assert!(!report.geometry_is_complete());
    }

    #[test]
    fn an_excluded_object_with_no_outline_is_named() {
        // EXCLUDE_OBJECT_START auto-definitions are name-only. Without
        // this diagnostic, `excluded_object_at` answers None across the
        // whole bed and a geometry-only filter treats a cancelled part
        // as printable.
        let timeline = timeline_with(Some(exclude_state(
            Some(vec![
                ExcludeObjectDef::name_only("NAMEONLY"),
                square("A", 100.0, 100.0),
            ]),
            &["NAMEONLY"],
        )));
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        assert!(report.diagnostics().contains(
            &ExclusionDiagnostic::ExcludedObjectWithoutOutline {
                objects: vec!["NAMEONLY".to_owned()],
            }
        ));
        assert!(!report.geometry_is_complete());
        // Confirmed: it really does answer None everywhere.
        assert_eq!(report.excluded_object_at(0.0, 0.0), None);
        assert_eq!(report.excluded_object_at(100.0, 100.0), None);
    }

    #[test]
    fn degraded_geometry_is_flagged_and_narrows_the_answer() {
        let mut boxed = square("BIG", 100.0, 100.0);
        boxed.fidelity = PolygonFidelity::BoundingBox { source_points: 900 };
        let mut broken = square("BROKEN", 200.0, 100.0);
        broken.fidelity = PolygonFidelity::Unusable { source_points: 2 };
        broken.polygon.clear();
        let timeline = timeline_with(Some(exclude_state(
            Some(vec![boxed, broken]),
            &["BIG", "BROKEN"],
        )));
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        assert!(report
            .diagnostics()
            .contains(&ExclusionDiagnostic::GeometryDegraded {
                objects: vec!["BIG".to_owned(), "BROKEN".to_owned()],
            }));
        assert!(report.diagnostics().contains(
            &ExclusionDiagnostic::ExcludedObjectWithoutOutline {
                objects: vec!["BROKEN".to_owned()],
            }
        ));
        assert!(!report.geometry_is_complete());
        assert!(report.objects_at(200.0, 100.0).is_empty());
        // The bounding box still answers, conservatively.
        assert!(report.excluded_object_at(100.0, 100.0).is_some());
    }

    #[test]
    fn geometry_is_complete_only_grades_the_excluded_objects() {
        // A degraded outline on an object nobody cancelled cannot make
        // the "is this on a cancelled part?" answer wrong.
        let mut boxed = square("BIG", 100.0, 100.0);
        boxed.fidelity = PolygonFidelity::BoundingBox { source_points: 900 };
        let timeline = timeline_with(Some(exclude_state(
            Some(vec![boxed, square("A", 200.0, 100.0)]),
            &["A"],
        )));
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        assert!(report.geometry_is_complete());
        assert!(report
            .diagnostics()
            .contains(&ExclusionDiagnostic::GeometryDegraded {
                objects: vec!["BIG".to_owned()],
            }));
    }

    #[test]
    fn file_definitions_missing_from_the_wal_are_flagged() {
        let timeline = timeline_with(Some(exclude_state(
            Some(vec![square("CUBE_ID_0_COPY_0", 100.0, 100.0)]),
            &[],
        )));
        let report = resolve_exclusions(
            &timeline,
            &ExclusionInputs {
                file: Some(&tail(TWO_OBJECT_FILE)),
                ..ExclusionInputs::default()
            },
            &cfg(),
        );
        assert_eq!(report.provenance(), ExclusionProvenance::Journaled);
        assert!(report
            .diagnostics()
            .contains(&ExclusionDiagnostic::DefinitionsIncomplete {
                objects: vec!["CUBE_ID_1_COPY_0".to_owned()],
            }));
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
        // _reset_file() clears unparsable lines too: they described
        // objects that no longer exist.
        let file = b"EXCLUDE_OBJECT_DEFINE NAME=\"unterminated\nEXCLUDE_OBJECT_DEFINE RESET=1\n";
        let scan = parse_object_definitions(file);
        assert_eq!(scan.unparsed_lines, 0);
        assert!(scan.is_empty());
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
        let file =
            b"EXCLUDE_OBJECT_DEFINE NAME=\"unterminated\nEXCLUDE_OBJECT_START NAME='also bad\n";
        let scan = parse_object_definitions(file);
        assert_eq!(scan.unparsed_lines, 2);
        assert!(scan.definitions.is_empty());
        assert!(!scan.is_empty(), "unparsed lines still mean 'has objects'");

        let timeline = timeline_with(None);
        let report = resolve_exclusions(
            &timeline,
            &ExclusionInputs {
                file: Some(&tail(file)),
                ..ExclusionInputs::default()
            },
            &cfg(),
        );
        assert_eq!(report.provenance(), ExclusionProvenance::RecordLost);
        assert_eq!(
            report.uncertainty_causes(),
            vec![&UncertaintyCause::NoRecord]
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
        let repeated = [[1.0, 1.0], [1.0, 1.0], [1.0, 1.0]];
        assert!(point_in_polygon(1.0, 1.0, &repeated));
        assert!(!point_in_polygon(2.0, 2.0, &repeated));
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

        let timeline = timeline_with(Some(exclude_state(
            Some(vec![
                square("A", 100.0, 100.0),
                square("B", 105.0, 100.0),
                l_shape,
            ]),
            &["B"],
        )));
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        let both = report.objects_at(103.0, 100.0);
        assert_eq!(
            both.iter().map(|d| d.name.as_str()).collect::<Vec<_>>(),
            vec!["A", "B"]
        );
        assert_eq!(
            report.object_at(103.0, 100.0).map(|d| d.name.as_str()),
            Some("A")
        );
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
    fn accessors_expose_the_whole_picture() {
        let mut context = context_at(1_000, 0);
        context.exclude = Some(Box::new(ExcludeState {
            definitions: Some(vec![square("A", 100.0, 100.0)]),
            excluded: vec!["A".to_owned()],
            current: Some("A".to_owned()),
        }));
        let timeline = ingest_records(vec![WalRecord::Context(context)]);
        let report = resolve_exclusions(&timeline, &bare(), &cfg());
        assert_eq!(report.current_object(), Some("A"));
        assert_eq!(report.definition("a").map(|d| d.name.as_str()), Some("A"));
        assert_eq!(report.definition("Z"), None);
        assert_eq!(report.observed_print_time(), None);
        assert_eq!(
            report
                .excluded_definitions()
                .iter()
                .map(|d| d.name.as_str())
                .collect::<Vec<_>>(),
            vec!["A"]
        );
    }

    #[test]
    fn a_recovery_scan_helper_keeps_marker_order() {
        // Guards the fixture helper itself: markers must reach the
        // timeline in append order for the postdating test to mean
        // anything.
        let scan: RecoveryScan = scan_of(vec![
            WalRecord::Marker(Marker {
                mono_ns: 1,
                kind: MarkerKind::SocketLost,
            }),
            WalRecord::Marker(Marker {
                mono_ns: 2,
                kind: MarkerKind::Resubscribed,
            }),
        ]);
        assert!(matches!(scan.records.as_slice(), [_, _]));
        let timeline = ingest(&scan, None);
        assert_eq!(timeline.markers.len(), 2);
        assert_eq!(timeline.markers[0].mono_ns, 1);
    }
}
