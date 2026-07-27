//! The recovery pipeline: WAL directory → reconstruction → stop-point
//! match + contact selection → validated `RecoveryPlan`.
//!
//! ```text
//! scan::load_merged ─► plr_reconstruct::reconstruct
//!        │                      │ (possible-stop set)
//!        │                      ▼
//!        │   validate_machine (FIRST — refusal is fatal)
//!        │                      │
//!        │                      ▼
//!        │   plr_analyzer::{build_layer_model, match_stop_point,
//!        │                  select_contact_zone}
//!        │                      │
//!        ▼                      ▼
//!   print file ──────► plr_recovery::plan_recovery ──► PlanBundle
//! ```
//!
//! Pure with respect to the printer: this module reads local files only
//! and produces **data**. Nothing here talks to Moonraker or Klipper —
//! execution is `executor`'s job, behind its own gates.
//!
//! # Machine snapshot assembly — two modes
//!
//! At recover time the machine snapshot comes from one of two sources,
//! resolved by [`resolve_machine_source`]:
//!
//! * **`[plr]` mode** — the Klipper config carries a `[plr]` section:
//!   plrd queries klippy's API socket for `configfile.settings` (plus
//!   the plugin's `plr` status object) and assembles the snapshot from
//!   the **live running config** (`plrcfg`). The `[plr]` section is
//!   authoritative; `/etc/plrd.conf [machine]` is ignored with an info
//!   note. The crc32c blessing is satisfied by construction in this
//!   mode (`plrcfg::LIVE_CONFIG_HASH`): the values are re-read from the
//!   running config on every run, so there is no stale snapshot to
//!   detect. When klippy is unreachable, `[plr]` mode cannot exist:
//!   see [`resolve_machine_source`] for the honest fallback/refusal.
//! * **Legacy mode** — no `[plr]` section: the config's `[machine]`
//!   section is used unchanged (back-compat), with two runtime
//!   observations:
//!
//!   * `type_annotations_present` — scanned from the actual print file
//!     (`;TYPE:` anywhere in the modeled window), not attested;
//!   * `config_hash` — a `crc32c:`-prefixed checksum of the file named
//!     by `machine.klipper_config_path`, compared against
//!     `machine.validated_config_hash`. This detects printer.cfg edits
//!     since the prerequisites were blessed (change-detection checksum,
//!     not a cryptographic hash — an operator gate, not a security
//!     boundary). On mismatch the computed value is printed so the
//!     operator can re-bless deliberately.
//!
//! `exclude_objects` are still passed empty here, but **not** because
//! the data is missing: the WAL context now journals exclude-object
//! state (`plr_wal::ExcludeState`, written by `convert`) and
//! reconstruction resolves it into
//! `plr_reconstruct::RecoveryReconstruction::exclusions`. Wiring that
//! report into the resume file's `EXCLUDE_OBJECT_DEFINE` /
//! `EXCLUDE_OBJECT` replay is the remaining work. Whoever does it must
//! gate on `ExclusionReport::is_conclusive()` and, when it is false,
//! drive a per-object operator confirmation from
//! `ExclusionReport::confirmation()` — resuming a cancelled part prints
//! into the debris that caused the cancellation.

use std::io::Write;
use std::path::Path;

use plr_analyzer::{
    build_layer_model, build_preview, match_stop_point, select_contact_zone, ByteWindow,
    ContactConfig, ContactOutcome, Interval, LayerModel, MatchConfidence, MatchConfig, MatchError,
    MatchResult, ModelConfig, PreviewBounds, PreviewOutcome, PreviewSet, StopEvidence,
};
use plr_reconstruct::{
    anchor_state_from_context, reconstruct, select_crash_epoch, FileTail, PossibleStopSet,
    ReconstructInputs, Reconstruction, RecoveryReconstruction,
};
use plr_recovery::{
    plan_recovery, resolve_resume_with_preview, validate_machine, MachineConfig, MachineRejection,
    PlanConfig, PlanInputs, PlanOutcome, ProbeConfig, ProbeKind, RecoveryPlan, ResumePolicy,
    ZStepper,
};
use plr_wal::crc32c;

use crate::config::{Config, MachineSection};
use crate::plrcfg::{self, KlippySnapshot, PlrSettings};
use crate::scan;

/// How long one recover-time klippy query may take. Short: klippy is
/// local, and an unreachable klippy has a defined outcome.
pub(crate) const KLIPPY_QUERY_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(3);

/// A plan plus the context the executor and the operator prompt need.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanBundle {
    /// The validated, ordered plan.
    pub plan: RecoveryPlan,
    /// Absolute path of the print file being resumed.
    pub file_path: String,
    /// The machine snapshot the plan was validated against.
    pub machine: MachineConfig,
    /// The generated recovery-file content the executor writes into the
    /// `virtual_sdcard` root before execution (the file the final `M23`
    /// step selects). Raw BYTES: the tail is a byte-verbatim copy of the
    /// original print file, which may not be UTF-8.
    pub recovery_file_content: Vec<u8>,
    /// Absolute path the recovery-file content is written to (the sdcard
    /// root joined with the collision-resolved recovery file name).
    ///
    /// Chosen here, written later by the executor's `WriteRecoveryFile`
    /// gate — so the write is `create_new` with a bounded re-resolve
    /// retry, never an unconditional truncate: a file that appeared in
    /// between must never be silently clobbered.
    pub recovery_file_path: std::path::PathBuf,
    /// The sdcard root the recovery file lives in, kept so the write gate
    /// can re-resolve a fresh collision-free name if the chosen path was
    /// taken between planning and writing.
    pub sdcard_root: std::path::PathBuf,
    /// The original print file's basename, for that re-resolve.
    pub recovery_source_name: String,
    /// The original print file's raw bytes — carried ONLY for a preview
    /// (`ask`) plan, where the recovery file cannot be pre-generated
    /// because the resume point is not known until the operator accepts in
    /// the reposition loop (design §4). The execute path's
    /// [`crate::executor::RecoveryFileWriter`] rebuilds the file from these
    /// bytes plus the plan's `recovery_file` template on Accept. `None` for
    /// an ordinary plan (its content is already in
    /// [`Self::recovery_file_content`], written before execution).
    pub original_file_bytes: Option<Vec<u8>>,
}

/// Every outcome the pipeline can reach. Only `Plan` is executable.
#[derive(Debug, Clone, PartialEq)]
pub enum PipelineOutcome {
    /// The WAL ends with a deliberate print end; nothing to recover.
    CleanShutdown,
    /// The WAL tail is unclean, but the print had **no printing work
    /// left**: it finished, and at most its end sequence did not run.
    /// Nothing to recover.
    ///
    /// Reported separately from [`CleanShutdown`](Self::CleanShutdown)
    /// because the cause differs and the operator-facing wording has to:
    /// "the WAL ends with a clean print end" is false here — the log ends
    /// torn. What is true is "the print finished". Conflating the two would
    /// tell an operator whose printer lost power during the cooldown that
    /// their log ended cleanly, which it did not.
    Complete(Box<CompletionReport>),
    /// Machine prerequisites failed (all failures listed). Fatal for
    /// both dry-run and execute.
    MachineRejected(MachineRejection),
    /// Automation declines; the operator must recover manually.
    ManualFallback(String),
    /// The pipeline could not produce an answer (missing evidence,
    /// unreadable file, reconstruction error).
    NotPossible(String),
    /// A validated plan.
    Plan(Box<PlanBundle>),
}

/// Why a print needed no recovery even though its log ended uncleanly.
#[derive(Debug, Clone, PartialEq)]
pub struct CompletionReport {
    /// Absolute path of the print file.
    pub file: String,
    /// The offset the gate tested at (`stop_set.file_window.start`).
    pub tested_offset: u64,
    /// Size of the print file, bytes.
    pub file_size: u64,
    /// What the replay found after [`tested_offset`](Self::tested_offset).
    pub work: plr_analyzer::RemainingWork,
}

impl CompletionReport {
    /// Bytes of the file after the tested offset — the near-constant
    /// 14–18 KB slicer footer a percentage would have read as progress.
    #[must_use]
    pub fn trailing_bytes(&self) -> u64 {
        self.file_size.saturating_sub(self.tested_offset)
    }

    /// The end-sequence commands that did not run, if any.
    ///
    /// **Never offer to execute these.** An end macro routinely homes,
    /// drops the bed, or moves Z, and none of the envelope or pre-flight
    /// analysis that guards a recovery plan applies to an opaque macro
    /// body. Naming them is the whole value.
    #[must_use]
    pub fn unrun_commands(&self) -> &[String] {
        match &self.work {
            plr_analyzer::RemainingWork::EndSequenceOnly { commands } => commands,
            plr_analyzer::RemainingWork::Extrusion { .. }
            | plr_analyzer::RemainingWork::Nothing => &[],
        }
    }
}

/// A one-line note when the merged WAL spans more than one boot/firmware
/// epoch, describing that recovery is scoped to the newest printing one.
/// `None` for a single-epoch log (the common case).
fn epoch_note(merged: &plr_wal::RecoveryScan) -> Option<String> {
    let selection = select_crash_epoch(merged);
    selection.partitioned().then(|| {
        format!(
            "pipeline: WAL spans {} epochs; recovering the newest printing epoch \
             ({} older, {} newer discarded)",
            selection.epochs.len(),
            selection.discarded_older(),
            selection.discarded_newer(),
        )
    })
}

/// Runs the full pipeline, narrating progress to `out`. `Err` only for
/// hard I/O failures on the WAL directory itself.
pub fn run_pipeline(config: &Config, out: &mut dyn Write) -> Result<PipelineOutcome, String> {
    let mut say = |text: &str| {
        let _ = writeln!(out, "{text}");
    };
    let merged = scan::load_merged(&config.wal_dir)?;
    say(&format!(
        "pipeline: {} WAL records, tail: {}",
        merged.records.len(),
        merged.end
    ));
    // Reconstruction scopes itself to the crash epoch; narrate it when the
    // log spans more than one boot/firmware session so the operator knows
    // older epochs were excluded rather than merged (see plr_reconstruct::epoch).
    if let Some(note) = epoch_note(&merged) {
        say(&note);
    }

    let heartbeat = match scan::load_heartbeat(&config.heartbeat_file()) {
        Ok(recovery) => Some(recovery),
        Err(reason) => {
            say(&format!("pipeline: {reason}"));
            None
        }
    };
    let receive_seq = scan::load_receive_seq(&config.receive_seq_file());
    // The power-fail edge (the watcher's channel-bypassing exact-T fact):
    // the sidecar if still present, else the copy boot detection PERSISTED
    // into `pending_recovery.json` (the daemon deletes the write-once
    // sidecar at boot once consumed). The SAME resolver `plrd scan` uses
    // (`scan::power_fail_edge`), so the two never disagree about one power
    // loss. Fed through the same `power_fail_edge_mono_ns` input, so a
    // persisted edge obeys the identical `sidecar_admits` band and a stale
    // one cannot resurrect against an unrelated later crash.
    let power_fail_edge_mono_ns = scan::power_fail_edge(&config.wal_dir);

    // The print file is optional until we know recovery is needed: a
    // clean shutdown must classify as clean even with no file around.
    let named_file = scan::last_print_file(&merged);
    let file_read = named_file
        .as_ref()
        .map(|(path, _)| (path.clone(), std::fs::read(path)));
    let file_bytes = match &file_read {
        Some((path, Ok(bytes))) => {
            say(&format!(
                "pipeline: print file {path} ({} bytes)",
                bytes.len()
            ));
            Some(bytes.as_slice())
        }
        _ => None,
    };

    let inputs = ReconstructInputs {
        scan: &merged,
        heartbeat: heartbeat.as_ref(),
        file_tail: file_bytes.map(|bytes| FileTail {
            base_offset: 0,
            bytes,
        }),
        receive_seq,
        power_fail_edge_mono_ns,
    };
    let recovery = match reconstruct(&inputs, &crate::convert::reconstruct_config(Some(config))) {
        Ok(Reconstruction::CleanShutdown(_)) => return Ok(PipelineOutcome::CleanShutdown),
        Ok(Reconstruction::Recovery(recovery)) => recovery,
        Err(e) => {
            return Ok(PipelineOutcome::NotPossible(format!(
                "reconstruction failed: {e}"
            )))
        }
    };
    // Recovery is needed: now the file is mandatory.
    let (file_path, file_bytes) = match (&named_file, &file_read) {
        (Some(_), Some((path, Ok(bytes)))) => (path.clone(), bytes.as_slice()),
        (Some(_), Some((path, Err(e)))) => {
            return Ok(PipelineOutcome::NotPossible(format!(
                "print file {path} unreadable: {e}"
            )))
        }
        _ => {
            return Ok(PipelineOutcome::NotPossible(
                "no WAL context names a print file; nothing to resume".to_owned(),
            ))
        }
    };
    say(&format!(
        "pipeline: stop window t_a {:.3}s .. t_b {:.3}s, class {:?}",
        recovery.window.t_a, recovery.window.t_b, recovery.window.class
    ));

    // Machine prerequisites run FIRST — before any planning — and a
    // refusal is fatal for every mode. Which snapshot source applies
    // ([plr] live config vs legacy /etc/plrd.conf) is resolved here.
    let type_annotations = contains_type_annotations(file_bytes);
    let (machine, plan_config, contact_config, legacy, macros) =
        match machine_inputs(config, type_annotations, &mut say) {
            Ok(inputs) => inputs,
            Err(outcome) => return Ok(outcome),
        };
    if let Err(rejection) = validate_machine(&machine) {
        if legacy {
            // Only legacy mode has a hash to re-bless; [plr] mode reads
            // the live config each run.
            say(&format!(
                "pipeline: machine hash computed: {}",
                machine.config_hash
            ));
        }
        return Ok(PipelineOutcome::MachineRejected(rejection));
    }
    say("pipeline: machine prerequisites validated");

    Ok(plan_from_recovery(
        &recovery,
        &machine,
        &plan_config,
        &contact_config,
        &macros,
        &file_path,
        file_bytes,
        &mut say,
    ))
}

/// Assembles the mode-dependent planning inputs
/// `(machine, plan config, contact config, legacy?)`, narrating the
/// mode decision. `Err` carries the early pipeline outcome.
fn machine_inputs(
    config: &Config,
    type_annotations: bool,
    say: &mut dyn FnMut(&str),
) -> Result<
    (
        MachineConfig,
        PlanConfig,
        ContactConfig,
        bool,
        std::collections::BTreeSet<String>,
    ),
    PipelineOutcome,
> {
    match resolve_machine_source(config) {
        MachineSource::Unavailable { reason } => Err(PipelineOutcome::NotPossible(format!(
            "machine configuration unavailable: {reason}"
        ))),
        MachineSource::Plr(source) => {
            say("pipeline: machine config from the [plr] section of the live Klipper config");
            say("pipeline: note: /etc/plrd.conf [machine] is ignored while [plr] exists");
            for note in &source.notes {
                say(&format!("pipeline: note: {note}"));
            }
            let (machine, assembly_notes) = plrcfg::machine_from_settings(
                &source.snapshot.settings,
                &source.snapshot.config,
                &source.plr,
                type_annotations,
            );
            for note in assembly_notes {
                say(&format!("pipeline: note: {note}"));
            }
            let contact = ContactConfig {
                exclusion_radius: source.plr.exclusion_radius,
                ..ContactConfig::default()
            };
            // Macro existence for the clean-nozzle step and the purge
            // fallback is resolved from the live config sections.
            let macros = plrcfg::gcode_macro_names(&source.snapshot.settings);
            Ok((machine, source.plr.plan_config(), contact, false, macros))
        }
        MachineSource::Legacy { note } => {
            say("pipeline: machine config from /etc/plrd.conf [machine] (legacy mode)");
            if let Some(note) = note {
                say(&format!("pipeline: note: {note}"));
            }
            let machine = machine_config(
                &config.machine,
                type_annotations,
                config.machine.klipper_config_path.as_deref(),
            );
            // The legacy [machine] path predates the Klipper plugin, so
            // the plugin's `PLR_TOUCH` command may not exist on this
            // printer: fall back to the stock single `PROBE` (the
            // consensus touch is a [plr]-mode feature). See
            // `PlanConfig::legacy_single_probe`.
            //
            // For the same reason it resolves the interactive `ask` default
            // to its documented automatic fallback `last` (skip-forward):
            // `resume_candidate_policy` is a [plr] key the legacy path
            // cannot carry, and the resume preview is presented through the
            // [plr] plugin's confirm dialog — a legacy setup has no client
            // to answer a preview pause, so attaching one would fail-closed
            // abort every legacy recovery. `last` keeps legacy recoveries
            // automatic, exactly as they were before the increment-3 flip.
            let plan_config = PlanConfig {
                legacy_single_probe: true,
                resume_candidate_policy: ResumePolicy::Last,
                ..PlanConfig::default()
            };
            // Legacy mode cannot see the running config's macro sections,
            // so no clean-nozzle / purge macro is known (the recovery
            // file falls back to the built-in purge, and the clean-nozzle
            // step requires operator confirmation).
            Ok((
                machine,
                plan_config,
                ContactConfig::default(),
                true,
                std::collections::BTreeSet::new(),
            ))
        }
    }
}

/// The `[plr]`-mode payload of [`MachineSource`] (boxed: the settings
/// snapshot dwarfs the other variants).
pub(crate) struct PlrSource {
    /// The queried `configfile.settings` (+ `plr` status object).
    pub snapshot: KlippySnapshot,
    /// The parsed `[plr]` section.
    pub plr: PlrSettings,
    /// Non-fatal observations surfaced to the operator.
    pub notes: Vec<String>,
}

/// Which snapshot source `plrd recover` uses this run.
pub(crate) enum MachineSource {
    /// The Klipper config has a `[plr]` section: it is authoritative.
    Plr(Box<PlrSource>),
    /// No `[plr]` section (or klippy unreachable with a commissioned
    /// legacy snapshot): the `/etc/plrd.conf [machine]` path applies.
    Legacy {
        /// Set when legacy applies as a fallback rather than by
        /// absence of `[plr]`.
        note: Option<String>,
    },
    /// Neither source can be used; recovery (and dry-run) refuse.
    Unavailable {
        /// The operator-facing reason.
        reason: String,
    },
}

/// Resolves the machine-config mode by querying klippy's API socket.
///
/// # klippy unreachable
///
/// `[plr]` mode cannot be *detected*, let alone read, without klippy —
/// and the WAL does not journal the `[plr]` settings, so there is no
/// recorded copy to plan from honestly (the context format carries
/// interpreter state, not printer.cfg). Therefore:
///
/// * a commissioned legacy `[machine]` snapshot (`probe_kind` set) is
///   trusted with an info note — back-compat: that path never needed
///   klippy, and its crc32c blessing still detects a printer.cfg that
///   changed since blessing (including one that has since grown a
///   `[plr]` section);
/// * otherwise both recovery **and dry-run** refuse with a clear
///   message, rather than dry-running against invented machine data.
pub(crate) fn resolve_machine_source(config: &Config) -> MachineSource {
    match plrcfg::query_klippy_snapshot(&config.klipper_socket, KLIPPY_QUERY_TIMEOUT) {
        Ok(snapshot) => match snapshot.plr_section() {
            Some(section) => match PlrSettings::parse(section) {
                Ok(plr) => {
                    let mut notes = Vec::new();
                    match &snapshot.plr_object {
                        Some(object) => {
                            // The plugin's get_status key is
                            // `probe_method` (klippy_plugin/plr/
                            // plugin.py); `method` kept as a fallback
                            // for older builds.
                            let reported = object
                                .get("probe_method")
                                .or_else(|| object.get("method"))
                                .and_then(serde_json::Value::as_str);
                            if reported.is_some_and(|m| m != plr.probe_method) {
                                notes.push(format!(
                                    "plr status object reports method {reported:?} but \
                                     configfile settings say {:?}; the settings win",
                                    plr.probe_method
                                ));
                            }
                        }
                        None => notes.push(
                            "[plr] section exists but the plr status object is empty \
                             (plugin not loaded?)"
                                .to_owned(),
                        ),
                    }
                    MachineSource::Plr(Box::new(PlrSource {
                        snapshot,
                        plr,
                        notes,
                    }))
                }
                // A [plr] section that cannot be read faithfully must
                // refuse — falling back to legacy would contradict the
                // operator's visible configuration.
                Err(e) => MachineSource::Unavailable {
                    reason: format!("the [plr] section is present but unreadable: {e}"),
                },
            },
            None => MachineSource::Legacy { note: None },
        },
        Err(e) => {
            if config.machine.probe_kind.is_some() {
                MachineSource::Legacy {
                    note: Some(format!(
                        "klippy is unreachable ({e}); trusting the commissioned legacy \
                         [machine] snapshot — its config-hash blessing still detects a \
                         printer.cfg changed since blessing (including one that grew [plr])"
                    )),
                }
            } else {
                MachineSource::Unavailable {
                    reason: format!(
                        "klippy is unreachable ({e}) and no legacy [machine] snapshot is \
                         commissioned. The [plr] settings live only in the running Klipper \
                         config (the WAL does not journal them), so even a dry run would \
                         be guesswork; start klippy and retry"
                    ),
                }
            }
        }
    }
}

/// One-paragraph machine-config-mode report (used by `plrd scan
/// --config` and the control socket's `status`).
pub(crate) fn report_machine_mode(config: &Config, out: &mut dyn Write) {
    let mut line = |text: &str| {
        let _ = writeln!(out, "{text}");
    };
    match resolve_machine_source(config) {
        MachineSource::Plr(source) => {
            line("machine-config mode: [plr] (live Klipper config; /etc/plrd.conf [machine] ignored)");
            line(&format!(
                "  probe_method = {}, control_socket = {}",
                source.plr.probe_method, source.plr.control_socket
            ));
            for note in &source.notes {
                line(&format!("  note: {note}"));
            }
        }
        MachineSource::Legacy { note } => {
            line("machine-config mode: legacy [machine] section of /etc/plrd.conf");
            if let Some(note) = note {
                line(&format!("  note: {note}"));
            }
        }
        MachineSource::Unavailable { reason } => {
            line(&format!("machine-config mode: UNDETERMINED — {reason}"));
        }
    }
}

/// The analysis half: model, match, contact, plan. Infallible in the
/// `Result` sense — every failure is itself a typed outcome.
#[allow(clippy::too_many_arguments)] // the analysis half threads several borrowed inputs
#[allow(clippy::too_many_lines)] // linear analysis pipeline + policy routing
fn plan_from_recovery(
    recovery: &RecoveryReconstruction,
    machine: &MachineConfig,
    plan_config: &PlanConfig,
    contact_config: &ContactConfig,
    macros: &std::collections::BTreeSet<String>,
    file_path: &str,
    file_bytes: &[u8],
    say: &mut dyn FnMut(&str),
) -> PipelineOutcome {
    let AnchoredModel {
        model,
        base_offset,
        anchor,
    } = match anchored_model(recovery, file_path, file_bytes, say) {
        Ok(anchored) => anchored,
        Err(outcome) => return outcome,
    };
    let stop_set = &recovery.stop_set;

    // Part 2: map the (cap-narrowed) offset window through the layer model
    // and narrate which layer(s) the stop can be in. Reporting only — the
    // matcher's ladder below is unchanged. The slicer layer mark (Part 3),
    // when present, is folded in as an upper-bound cross-check.
    narrate_layer_attribution(
        &model,
        stop_set,
        anchor.current_layer,
        anchor.total_layer,
        base_offset,
        say,
    );

    match completion_check(recovery, &model, base_offset, anchor, file_path, file_bytes) {
        CompletionCheck::Complete(report) => {
            narrate_completion(&report, say);
            return PipelineOutcome::Complete(Box::new(report));
        }
        // The WAL's byte offsets no longer address this file, so a resume
        // plan built from them would seek into content we never recorded.
        // Refuse outright rather than plan against a stale offset.
        CompletionCheck::OffsetsInvalid(reason) => {
            say(&format!("pipeline: {reason}"));
            return PipelineOutcome::NotPossible(reason);
        }
        CompletionCheck::CarryOn(reason) => {
            if let Some(reason) = reason {
                say(&format!(
                    "pipeline: completion gate cannot suppress: {reason}"
                ));
            }
        }
    }

    let Some(evidence) = stop_evidence(stop_set, base_offset) else {
        return PipelineOutcome::ManualFallback(
            "the reconstruction has no XY region; stop-point matching is impossible \
             (probe placement cannot be chosen automatically)"
                .to_owned(),
        );
    };
    let match_result = match match_stop_point(&model, &evidence, &MatchConfig::default()) {
        Ok(result) => {
            say(&format!(
                "pipeline: match confidence {:?} ({} candidates)",
                result.confidence,
                result.candidates.len()
            ));
            result
        }
        // Inconclusive is the design's NAMED common real-crash shape (many
        // consistent lines across 1-4 layers): the confidence ladder
        // discards every line, but `build_preview` works from the raw
        // evidence and never needed the match result. Route it into the
        // SAME policy/preview path as LayerOnly, with a placeholder
        // LayerOnly confidence. That placeholder is consulted only on the
        // no-preview-set fallback (`resolve_resume_with_preview(None)`),
        // where a coarse layer correctly degrades to manual — so any-policy
        // `NoStops` still falls back to manual, while a usable set resumes:
        // `first`/`mid`/`last` automatically from the anchor, and `ask`
        // (increment 3) by attaching the interactive preview.
        Err(MatchError::Inconclusive { lines, layers }) => {
            say(&format!(
                "pipeline: match inconclusive: {lines} candidate lines across layers \
                 {layers:?}; routing the raw evidence through the resume-point preview"
            ));
            MatchResult {
                candidates: Vec::new(),
                confidence: MatchConfidence::LayerOnly {
                    layer: layers.first().copied().unwrap_or(0),
                },
                skipped_unknown: 0,
            }
        }
        Err(e) => return PipelineOutcome::ManualFallback(format!("stop-point match failed: {e}")),
    };

    // Resume-point routing by `resume_candidate_policy` (design §D / §11).
    // The preview builder is the parallel path (§A.1); it recovers the
    // candidate stops the matcher's confidence ladder discards on a coarse
    // match, so `first`/`mid`/`last` resume automatically from the set's
    // anchor — the headless win. A UniqueLine ignores policy entirely (one
    // line, nothing to pick). Exclusions are `None` until the pipeline
    // wires the real excluded set (§E.3 pre-existing gap); until then
    // build_preview keeps all non-attributable stops (safe direction).
    //
    // INCREMENT 3 — the `ask` flip (design §11). The set is now built for
    // `ask` too and passed through, so `plan_recovery` ATTACHES it as the
    // interactive preview (build.rs: only `ask` attaches; first/mid/last get
    // a plain automatic plan at the anchor's offset). This ships the ruled
    // default: `ask` = the interactive preview. A headless / bare-CLI
    // recovery whose confirmer fail-closes now ABORTS an ambiguous resume
    // rather than auto-completing it — deliberate: the dialog is the
    // intended path, and `last` remains the explicit skip-forward for a
    // headless setup that wants automatic completion. `ask` with NO stops
    // (NoStops) still falls back to the resolver's `last` anchor, so an
    // empty set never leaves the operator without a resume.
    let policy = plan_config.resume_candidate_policy;
    let build_set = !matches!(match_result.confidence, MatchConfidence::UniqueLine { .. });
    let mut preview_owned: Option<PreviewSet> = if build_set {
        match build_preview(
            &model,
            &evidence,
            &MatchConfig::default(),
            None,
            &PreviewBounds::default(),
        ) {
            PreviewOutcome::Preview(set) => Some(set),
            // first/mid/last with NoStops fall back to the resolver (never
            // read a resume out of NoStops — increment-1 binding note);
            // TooWide / invalid degrade to manual.
            PreviewOutcome::NoStops => None,
            other => {
                return PipelineOutcome::ManualFallback(format!(
                    "resume preview declined ({other:?}); manual recovery required"
                ))
            }
        }
    } else {
        None
    };
    // Annotate the set with the slicer layer mark its stops' layers may be
    // cross-checked against (the layer-provenance follow-up). The mark is
    // ABSOLUTE (from file start) and window ordinals equal absolute layers
    // only when the model itself spans from file start (`base_offset == 0`)
    // — the identical absolute-frame gate `narrate_layer_attribution`
    // applies. On a mid-file model the comparison is incommensurable, so the
    // mark is withheld (`None`) and every stop reads as model-inferred; the
    // per-stop `L <= mark` upper-bound check that yields "journal" lives in
    // the daemon's `preview_detail`. A stale/withheld mark never fabricates
    // provenance — it degrades to inferred.
    if let Some(set) = preview_owned.as_mut() {
        set.corroborating_layer_mark = if base_offset == 0 {
            anchor.current_layer
        } else {
            None
        };
    }
    let preview = preview_owned.as_ref();

    let (resume, contact) = match resume_and_contact(
        &model,
        &match_result,
        policy,
        preview,
        &evidence,
        contact_config,
        say,
    ) {
        Ok(pair) => pair,
        Err(outcome) => return outcome,
    };

    // usize conversion validated by `anchored_model`.
    let base_usize = usize::try_from(base_offset).unwrap_or(usize::MAX);
    let file_temps =
        plr_recovery::scan_file_temps(&file_bytes[base_usize..], base_offset, resume.offset);

    // Clean-nozzle macro presence and the purge-macro fallback are
    // resolved from the running config's [gcode_macro ...] sections.
    let clean_nozzle_macro_present =
        macros.contains(&plan_config.clean_nozzle_macro.to_ascii_uppercase());
    let purge_macro_present = plan_config
        .purge_macro
        .as_deref()
        .is_some_and(|m| macros.contains(&m.to_ascii_uppercase()));

    let reconstruction = Reconstruction::Recovery(Box::new(recovery.clone()));
    let plan_inputs = PlanInputs {
        machine,
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps,
        exclude_objects: &[],
        clean_nozzle_macro_present,
        purge_macro_present,
        preview,
    };
    match plan_recovery(&plan_inputs, plan_config) {
        Ok(PlanOutcome::NoRecoveryNeeded) => PipelineOutcome::CleanShutdown,
        Ok(PlanOutcome::ManualFallback { reason }) => {
            PipelineOutcome::ManualFallback(format!("planner declined: {reason:?}"))
        }
        Ok(PlanOutcome::Plan(plan)) => {
            // The contact point the pre-flight anchors on (the analyzer's
            // selected probe site).
            let contact_point = match &contact {
                ContactOutcome::Candidates(c) => {
                    c.first().map_or([0.0, 0.0], |candidate| candidate.point)
                }
                ContactOutcome::Declined(_) => [0.0, 0.0],
            };
            match finalize_recovery_file(*plan, machine, contact_point, file_bytes, say) {
                Ok(bundle) => PipelineOutcome::Plan(Box::new(PlanBundle {
                    file_path: file_path.to_owned(),
                    machine: machine.clone(),
                    ..bundle
                })),
                Err(reason) => PipelineOutcome::NotPossible(reason),
            }
        }
        Err(e) => PipelineOutcome::NotPossible(format!("planning failed: {e}")),
    }
}

/// Resolves the recovery file's collision-free name against the sdcard
/// root, patches the plan's `M23`/`resume_file`/spec to match, and
/// generates the file CONTENT (not written yet — dry-run must not write).
/// Returns a partly-filled [`PlanBundle`] (the caller fills `file_path`
/// and `machine`).
fn finalize_recovery_file(
    mut plan: RecoveryPlan,
    machine: &MachineConfig,
    contact_point: [f64; 2],
    file_bytes: &[u8],
    say: &mut dyn FnMut(&str),
) -> Result<PlanBundle, String> {
    let root = machine
        .virtual_sdcard_root
        .as_deref()
        .ok_or_else(|| "virtual_sdcard root unknown; cannot place the recovery file".to_owned())?;
    let root_path = std::path::Path::new(root.trim_end_matches(['/', '\\']));

    // Existing top-level names in the sdcard root (best-effort: an
    // unreadable dir just means no known collisions).
    let taken: std::collections::BTreeSet<String> = std::fs::read_dir(root_path)
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .filter_map(|e| e.file_name().into_string().ok())
        .collect();

    let resolved =
        plr_recovery::recovery_file_name(&plan.recovery_file.source_name, &|n| taken.contains(n));
    if resolved != plan.recovery_file.name {
        say(&format!(
            "pipeline: recovery file name collided; using {resolved}"
        ));
        // Patch every place the name appears so M23 matches the file.
        for step in &mut plan.steps {
            for command in &mut step.commands {
                if let Some(rest) = command.strip_prefix("M23 ") {
                    if rest == plan.recovery_file.name {
                        *command = format!("M23 {resolved}");
                    }
                }
            }
        }
        plan.recovery_file.name.clone_from(&resolved);
        plan.resume_file = resolved;
    }

    let timestamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
        .to_string();
    let generated = plr_recovery::build_recovery_file(&plan.recovery_file, file_bytes, &timestamp);

    // The heating gate is a build-time invariant: refuse to proceed if it
    // ever fails to hold (defense in depth over the generator).
    if let Err(violation) = plr_recovery::verify_heating_gate(&generated, &plan.recovery_file) {
        return Err(format!(
            "generated recovery file violates the heating gate: {violation}"
        ));
    }
    // The file's OWN absolute coordinates — the re-park travel, the purge,
    // and the entry moves that used to be the plan's Entry step — get the
    // same axis-limit pre-flight the plan itinerary gets. Klipper plays
    // the file back with no verification, so an out-of-range coordinate
    // here would only surface as a mid-recovery "Move out of range" AFTER
    // the probe established the Z reference.
    if let Err(e) = plr_recovery::preflight_generated_file(&generated, machine, contact_point) {
        return Err(format!("generated recovery file failed pre-flight: {e}"));
    }

    let recovery_file_path = root_path.join(&plan.recovery_file.name);
    let recovery_source_name = plan.recovery_file.source_name.clone();
    // A preview plan carries the original bytes so the accept-time writer
    // can rebuild the file for the chosen stop (§4). An ordinary plan does
    // not — its content is `generated.content`, written before execution —
    // so it keeps the bundle small.
    let original_file_bytes = if plan.preview.is_some() {
        Some(file_bytes.to_vec())
    } else {
        None
    };
    Ok(PlanBundle {
        plan,
        file_path: String::new(),
        machine: machine.clone(),
        recovery_file_content: generated.content,
        recovery_file_path,
        sdcard_root: root_path.to_path_buf(),
        recovery_source_name,
        original_file_bytes,
    })
}

/// Picks the resume target and the probe contact zone, narrating the
/// candidate count. `Err` carries the manual-fallback outcome.
///
/// The resume is resolved through the SAME `resolve_resume_with_preview`
/// selector the plan builder uses (policy + optional preview set), so the
/// contact zone is chosen for the exact layer the plan will resume on — no
/// drift between the two.
fn resume_and_contact(
    model: &plr_analyzer::LayerModel,
    match_result: &MatchResult,
    policy: ResumePolicy,
    preview: Option<&PreviewSet>,
    evidence: &StopEvidence,
    contact_config: &ContactConfig,
    say: &mut dyn FnMut(&str),
) -> Result<(plr_recovery::ResumeTarget, ContactOutcome), PipelineOutcome> {
    let resume =
        resolve_resume_with_preview(model, match_result, policy, preview).map_err(|reason| {
            PipelineOutcome::ManualFallback(format!("no safe resume point: {reason:?}"))
        })?;
    let Some(resume_layer) = resume.layer else {
        return Err(PipelineOutcome::ManualFallback(
            "resume point has no layer attribution; contact selection impossible".to_owned(),
        ));
    };
    let crash_xy = [evidence.x.midpoint(), evidence.y.midpoint()];
    let contact = select_contact_zone(model, resume_layer, crash_xy, contact_config)
        .map_err(|e| PipelineOutcome::ManualFallback(format!("contact selection failed: {e}")))?;
    if let ContactOutcome::Candidates(candidates) = &contact {
        say(&format!(
            "pipeline: {} probe candidate(s); best at ({:.2}, {:.2})",
            candidates.len(),
            candidates.first().map_or(f64::NAN, |c| c.point[0]),
            candidates.first().map_or(f64::NAN, |c| c.point[1]),
        ));
    }
    Ok((resume, contact))
}

/// Narrates a completion into the pipeline report.
fn narrate_completion(report: &CompletionReport, say: &mut dyn FnMut(&str)) {
    say(&format!(
        "pipeline: the print is COMPLETE — no extrusion remains after byte {} of {} \
         ({} trailing bytes are the slicer footer); nothing to resume",
        report.tested_offset,
        report.file_size,
        report.trailing_bytes(),
    ));
    if !report.unrun_commands().is_empty() {
        say(&format!(
            "pipeline: these end-sequence commands did not run: {} (NOT offered — an end \
             macro homes and moves Z, and none of the envelope or pre-flight analysis \
             applies to it)",
            report.unrun_commands().join(" ")
        ));
    }
}

/// What the pipeline should do about the completion gate's answer.
enum CompletionCheck {
    /// The print finished; report it and stop.
    Complete(CompletionReport),
    /// The WAL's offsets do not address the file on disk. Nothing derived
    /// from them is usable, including a resume plan.
    OffsetsInvalid(String),
    /// Carry on planning. `Some(reason)` when the gate could not answer and
    /// that is worth narrating; `None` when work simply remains.
    CarryOn(Option<String>),
}

/// The completion gate, run here as well as in boot detection — because a
/// stale `pending_recovery.json`, a manual `plrd recover`, or the wizard can
/// all reach this point for a print that in fact finished.
///
/// Without it the planner declines with `NoResumeDeposition` (correctly —
/// there is no deposition left to resume at) and that surfaces as a
/// *failure* telling the operator to fix a reported issue, for a print that
/// simply ran to the end.
///
/// Every precondition lives in `detect::completion_verdict`, which is the
/// only way to reach a `Complete` answer from either path. This function
/// supplies inputs and does not decide which checks apply — see that
/// function's "One gate, two call sites" for why that is structural rather
/// than stylistic.
fn completion_check(
    recovery: &RecoveryReconstruction,
    model: &plr_analyzer::LayerModel,
    base_offset: u64,
    anchor: &plr_wal::Context,
    file_path: &str,
    file_bytes: &[u8],
) -> CompletionCheck {
    let Some(window) = recovery.stop_set.file_window.as_ref() else {
        return CompletionCheck::CarryOn(None);
    };
    // `anchored_model` already validated this conversion.
    let base_usize = usize::try_from(base_offset).unwrap_or(usize::MAX);
    let Some(tail) = file_bytes.get(base_usize..) else {
        return CompletionCheck::CarryOn(None);
    };
    let file_size = file_bytes.len() as u64;
    match crate::detect::completion_verdict(&crate::detect::GateInputs {
        anchor,
        model: crate::detect::ModelSource::Built(model),
        tail,
        base_offset,
        tested_offset: window.start,
        file_size,
        exclusions: &recovery.exclusions,
    }) {
        crate::detect::GateVerdict::Complete(work) => CompletionCheck::Complete(CompletionReport {
            file: file_path.to_owned(),
            tested_offset: window.start,
            file_size,
            work,
        }),
        crate::detect::GateVerdict::MustNotSuppress(refusal) => {
            let reason = refusal.reason();
            if refusal.invalidates_offsets() {
                CompletionCheck::OffsetsInvalid(
                    reason.unwrap_or_else(|| "the print file changed".to_owned()),
                )
            } else {
                CompletionCheck::CarryOn(reason)
            }
        }
    }
}

/// The replay of the print file, and the context it started from.
pub(crate) struct AnchoredModel<'a> {
    /// The layer model over `[base_offset, EOF)`.
    pub model: plr_analyzer::LayerModel,
    /// Stream offset the model begins at (the anchor's file position).
    pub base_offset: u64,
    /// **The context the model was actually replayed from.**
    ///
    /// Carried rather than re-derived. `detect::GateInputs::anchor` requires
    /// the context the model was built from, because the extruder-frame
    /// trust check reads its mode flags and the identity check reads its
    /// journaled file size — and a second, independent selection can return
    /// a different context with different flags, which would make the gate
    /// check a frame the replay never used. Keeping the reference makes that
    /// a structural guarantee instead of an assumption.
    pub anchor: &'a plr_wal::Context,
}

/// Selects the anchor context and replays the file into a layer model.
/// `Err` carries the early outcome.
fn anchored_model<'a>(
    recovery: &'a RecoveryReconstruction,
    file_path: &str,
    file_bytes: &[u8],
    say: &mut dyn FnMut(&str),
) -> Result<AnchoredModel<'a>, PipelineOutcome> {
    // Anchor context: the newest context at or before the offset floor whose
    // `virtual_sdcard` names *this* print file; falling back to the oldest
    // such context. Shared with boot detection (`detect::anchor_context`) so
    // that exactly one selection feeds both the replay and the completion
    // gate — see `AnchoredModel::anchor`.
    //
    // The name match is a tightening over the previous name-blind selection:
    // a context describing a different print could otherwise supply both the
    // interpreter state the replay used and the journaled size the gate's
    // identity check compared against.
    let floor = recovery.stop_set.file_window.as_ref().map(|w| w.start);
    let Some(anchor) = crate::detect::anchor_context(&recovery.timeline.contexts, file_path, floor)
    else {
        return Err(PipelineOutcome::NotPossible(format!(
            "no WAL context names {file_path}"
        )));
    };
    let base_offset = anchor
        .virtual_sdcard
        .as_ref()
        .map_or(0, |v| v.file_position);
    let Ok(base_usize) = usize::try_from(base_offset) else {
        return Err(PipelineOutcome::NotPossible(
            "context offset overflow".to_owned(),
        ));
    };
    if base_usize > file_bytes.len() {
        return Err(PipelineOutcome::NotPossible(format!(
            "context file offset {base_offset} exceeds the file ({} bytes); wrong file?",
            file_bytes.len()
        )));
    }
    let state = match anchor_state_from_context(&anchor.gcode) {
        Ok(state) => state,
        Err(e) => {
            return Err(PipelineOutcome::NotPossible(format!(
                "WAL context state invalid: {e}"
            )))
        }
    };
    let model = build_layer_model(
        state,
        &file_bytes[base_usize..],
        base_offset,
        &ModelConfig::default(),
    );
    say(&format!(
        "pipeline: layer model from byte {base_offset}: {} layers",
        model.layers.len()
    ));
    Ok(AnchoredModel {
        model,
        base_offset,
        anchor,
    })
}

/// Narrates the geometric layer attribution of the (cap-narrowed) offset
/// window, folding in the slicer layer mark.
///
/// Reporting only. **`Layer::index` is window-relative** — the model is
/// built from the mid-file anchor (`base_offset`), so its layer 0 is the
/// anchor's layer, not file layer 0. The slicer `current_layer` mark is
/// **absolute** (from file start), and Klipper resets it to 0 on a
/// `total_layer` change (`print_stats.py`). Comparing the two only makes
/// sense when the model itself spans from file start (`base_offset == 0`);
/// there, and only there, window ordinals are absolute and the upper-bound
/// cross-check (physical layer `<= current_layer`, parse leads execution)
/// is valid. On a mid-file model the geometric layers are reported as
/// window-relative and the mark is reported verbatim with no verdict —
/// avoiding both the vacuous-true and the spurious "impossible" alarm a
/// relative-vs-absolute comparison would produce.
fn narrate_layer_attribution(
    model: &LayerModel,
    stop_set: &PossibleStopSet,
    current_layer: Option<u32>,
    total_layer: Option<u32>,
    base_offset: u64,
    say: &mut dyn FnMut(&str),
) {
    let Some(window) = stop_set.file_window.as_ref() else {
        say("pipeline: layer attribution: no offset window (nothing to map)");
        return;
    };
    // OffsetWindow.end is inclusive; layers_in_window takes an exclusive end.
    let wl = model.layers_in_window(window.start, Some(window.end.saturating_add(1)));
    // Window ordinals equal absolute file layers only when the model spans
    // from file start.
    let absolute = base_offset == 0;
    if absolute {
        say(&format!(
            "pipeline: layer attribution: {} (offset window bytes {}..={})",
            wl.describe(),
            window.start,
            window.end
        ));
    } else {
        say(&format!(
            "pipeline: layer attribution: stop spans {} geometric layer(s){} within the offset \
             window bytes {}..={}, window-relative to the resume anchor at byte {base_offset} \
             (absolute layer numbers would require modeling from file start)",
            wl.layers.len(),
            if wl.before_first {
                " plus the pre-first-layer preamble"
            } else {
                ""
            },
            window.start,
            window.end
        ));
    }
    match current_layer {
        None => say(
            "pipeline: layer marks unavailable (slicer emitted no SET_PRINT_STATS_INFO); \
             geometric attribution above carries the answer",
        ),
        Some(mark) => {
            let of = total_layer.map_or_else(String::new, |t| format!(" of {t}"));
            if !absolute {
                // Mid-file model: window ordinals are NOT absolute, so no
                // consistency verdict — report the mark verbatim.
                say(&format!(
                    "pipeline: slicer reported current_layer={mark}{of} (absolute) — no \
                     consistency check: the geometric attribution above is window-relative to a \
                     mid-file anchor, not an absolute layer number"
                ));
            } else if wl.mark_is_consistent(mark) {
                say(&format!(
                    "pipeline: slicer layer mark current_layer={mark}{of} (upper bound on the \
                     physical layer; parse leads execution) — consistent with the geometric \
                     attribution above (cross-check, not a narrowing)"
                ));
            } else {
                say(&format!(
                    "pipeline: NOTE slicer layer mark current_layer={mark}{of} is BELOW every \
                     geometrically attributed layer — the mark is an upper bound on the physical \
                     layer, so geometry above it is unexpected; trusting geometry, flagging the \
                     discrepancy for evidence review"
                ));
            }
        }
    }
}

/// Maps the possible-stop set onto the matcher's evidence contract.
/// `None` when the set has no XY region (matching would be meaningless).
fn stop_evidence(stop_set: &PossibleStopSet, base_offset: u64) -> Option<StopEvidence> {
    let xy = stop_set.xy.as_ref()?;
    let mut z_candidates = Vec::new();
    for c in &stop_set.z_candidates {
        z_candidates.push(c.z.lo);
        if (c.z.hi - c.z.lo).abs() > 1e-9 {
            z_candidates.push(c.z.hi);
            z_candidates.push((c.z.lo + c.z.hi) * 0.5);
        }
    }
    let window = stop_set.file_window.as_ref().map_or(
        ByteWindow {
            start: base_offset,
            end: None,
        },
        |w| ByteWindow {
            start: w.start,
            // OffsetWindow.end is inclusive; ByteWindow.end exclusive.
            end: Some(w.end.saturating_add(1)),
        },
    );
    Some(StopEvidence {
        x: Interval {
            min: xy.x.lo,
            max: xy.x.hi,
        },
        y: Interval {
            min: xy.y.lo,
            max: xy.y.hi,
        },
        e: stop_set.e_internal.as_ref().map(|e| Interval {
            min: e.lo,
            max: e.hi,
        }),
        z_candidates,
        window,
    })
}

/// `;TYPE:` annotation scan (see the module docs).
fn contains_type_annotations(bytes: &[u8]) -> bool {
    bytes.windows(6).any(|w| w == b";TYPE:")
}

/// Assembles the `plr_recovery::MachineConfig` snapshot (see the module
/// docs for the split between attested and observed fields).
pub(crate) fn machine_config(
    section: &MachineSection,
    type_annotations_present: bool,
    klipper_config: Option<&Path>,
) -> MachineConfig {
    let probes = match (&section.probe_kind, section.probe_z_offset) {
        (Some(kind), Some(z_offset)) => vec![ProbeConfig {
            kind: if kind == "load_cell" {
                ProbeKind::LoadCell
            } else {
                ProbeKind::Tap
            },
            z_offset,
            activate_gcode_no_move: section.probe_activate_gcode_no_move,
            deactivate_gcode_no_move: section.probe_deactivate_gcode_no_move,
        }],
        // Missing kind or offset: no probe in the snapshot; validation
        // reports NoProbe (the honest failure).
        _ => Vec::new(),
    };
    MachineConfig {
        force_move_enabled: section.force_move_enabled,
        z_self_locking_attested: section.z_self_locking_attested,
        z_steppers: section
            .z_steppers
            .iter()
            .map(|s| ZStepper {
                name: s.name.clone(),
                mcu: s.mcu.clone().unwrap_or_else(|| section.primary_mcu.clone()),
            })
            .collect(),
        primary_mcu: section.primary_mcu.clone(),
        type_annotations_present,
        probes,
        z_position_min: section.z_position_min,
        // The legacy path cannot see the running Klipper config, so
        // `[printer] max_accel` is unknown here. That costs only the
        // recovery FILE's entry-accel clamp (the plan warns when
        // `accel_entry` is set and cannot be honoured there).
        max_accel: None,
        config_hash: klipper_config_hash(klipper_config),
        validated_config_hash: section.validated_config_hash.clone(),
        virtual_sdcard_root: section.virtual_sdcard_root.clone(),
        // The legacy path predates the ADXL drag method and offers no
        // way to commission it (probe_kind is tap|load_cell only), so
        // there is never a noise floor here.
        noise_floor: None,
        noise_floor_speed: None,
        // The legacy /etc/plrd.conf [machine] path has no axis-limit
        // keys, so the whole-itinerary pre-flight's limit checks are
        // skipped for it ("where known").
        axis_limits: plr_recovery::AxisLimits::default(),
    }
}

/// Change-detection checksum of the Klipper config file (module docs).
pub(crate) fn klipper_config_hash(path: Option<&Path>) -> String {
    let Some(path) = path else {
        return "unavailable (machine.klipper_config_path not set)".to_owned();
    };
    match std::fs::read(path) {
        Ok(bytes) => format!("crc32c:{:08x}", crc32c(&bytes)),
        Err(e) => format!("unreadable ({e})"),
    }
}

/// End-to-end fixture tests: a real WAL directory plus a real print
/// file, through the whole pipeline.
#[cfg(test)]
pub(crate) mod e2e_tests {
    use super::{run_pipeline, PipelineOutcome};
    use crate::config::{Config, MachineSection, MachineZStepper};
    use plr_wal::{
        Context, FanTarget, GcodeState, Heartbeat, HeaterTarget, SegmentHeader,
        TransformObservations, TrapqSegment, VirtualSdState, WalRecord, WalWriter,
    };
    use std::path::PathBuf;

    /// The synthetic two-layer print: layer 0 at Z 0.2, layer 1 at
    /// Z 0.4. Each layer is a solid 20 x 20 mm boustrophedon hatch at
    /// X 40..60 / Y 40..60 (0.4 mm line spacing — the probeable body)
    /// plus the original three-move L ending at (30, 30), all deposition
    /// annotated internal infill. Layer 0 ends at (30, 30) with 3.0 mm of
    /// extrusion, exactly the crash position [`crash_context`] reports.
    ///
    /// Three properties this fixture has to keep, each learned from a
    /// failure:
    ///
    /// * **A real solid layer, not a sketch.** `plr-analyzer`'s
    ///   structural checks measure the material that actually holds the
    ///   part to the bed. The bare three-move L this used to be offers
    ///   ~19 mm² against a 100 mm² bed-contact bar and is correctly
    ///   refused as unprobeable, which left these end-to-end tests
    ///   asserting against a manual fallback instead of the plan path
    ///   they exist to cover. The hatch is what makes the part probeable.
    /// * **The L carries the extrusion, the hatch barely any.** The
    ///   reconstruction hands the matcher a *swept* stop region — here
    ///   E 3.0..3.06 — so every move whose E range meets that window is
    ///   a candidate. Hatch rows at a realistic 0.058 mm of E each put
    ///   nine lines in the window and the match goes inconclusive; at
    ///   0.02 mm with the L carrying 1.96 mm, one line explains the stop.
    /// * **Layer 1 prints the L first, then the hatch.** The forward
    ///   sweep starts at the crash, so the first move after the layer
    ///   change is what it reaches. Leading with the big-E L keeps that
    ///   neighbourhood as unambiguous as the pre-hatch fixture's was.
    ///
    /// # This fixture has ZERO margin against `ambiguity_limit`
    ///
    /// Measured, on `6cf2f68` and unchanged since: the two planning
    /// end-to-end tests reach the matcher with **exactly 8 candidate
    /// lines** — offsets `[2104, 2121, 2138, 2152, 2189, 2212, 2229,
    /// 2246]`, from `e_internal` `[3.00, 4.96]` — against
    /// `plr_analyzer::MatchConfig::ambiguity_limit` of **8**. That is
    /// `MatchConfidence::AmbiguousWindow`, the last rung before
    /// `MatchError::Inconclusive`.
    ///
    /// So **any** widening of any evidence interval, anywhere upstream,
    /// tips these tests from a plan to a `ManualFallback`. Not
    /// hypothetical: it is what happened when the un-evidenced extruder
    /// band was unioned into `e_internal` — 10 candidates with the
    /// coverage-certified band, 12 floor-wide, both `Inconclusive` across
    /// layers `[0, 1]` — and it is why that work is not in the tree. See
    /// `plr_reconstruct::stopset`'s "Durable extruder coverage".
    ///
    /// If you are reading this because two planning tests just started
    /// failing with "below layer granularity", the cause is almost
    /// certainly an upstream interval that got wider, not a bug in these
    /// tests. **Do not raise `ambiguity_limit` to buy room**: that trades a
    /// visible failure for an invisible one, because the candidate count is
    /// the only width gate the pipeline has — nothing consumes
    /// `plr_reconstruct::Confidence` (see its docs).
    const MODEL_TEXT: &str = "G90
M83
G1 Z0.2 F7200
G1 X40 Y40 F9000
;TYPE:Internal infill
G1 X60 Y40 E0.02 F1800
G1 X60 Y40.4 E0.0004
G1 X40 Y40.4 E0.02
G1 X40 Y40.8 E0.0004
G1 X60 Y40.8 E0.02
G1 X60 Y41.2 E0.0004
G1 X40 Y41.2 E0.02
G1 X40 Y41.6 E0.0004
G1 X60 Y41.6 E0.02
G1 X60 Y42 E0.0004
G1 X40 Y42 E0.02
G1 X40 Y42.4 E0.0004
G1 X60 Y42.4 E0.02
G1 X60 Y42.8 E0.0004
G1 X40 Y42.8 E0.02
G1 X40 Y43.2 E0.0004
G1 X60 Y43.2 E0.02
G1 X60 Y43.6 E0.0004
G1 X40 Y43.6 E0.02
G1 X40 Y44 E0.0004
G1 X60 Y44 E0.02
G1 X60 Y44.4 E0.0004
G1 X40 Y44.4 E0.02
G1 X40 Y44.8 E0.0004
G1 X60 Y44.8 E0.02
G1 X60 Y45.2 E0.0004
G1 X40 Y45.2 E0.02
G1 X40 Y45.6 E0.0004
G1 X60 Y45.6 E0.02
G1 X60 Y46 E0.0004
G1 X40 Y46 E0.02
G1 X40 Y46.4 E0.0004
G1 X60 Y46.4 E0.02
G1 X60 Y46.8 E0.0004
G1 X40 Y46.8 E0.02
G1 X40 Y47.2 E0.0004
G1 X60 Y47.2 E0.02
G1 X60 Y47.6 E0.0004
G1 X40 Y47.6 E0.02
G1 X40 Y48 E0.0004
G1 X60 Y48 E0.02
G1 X60 Y48.4 E0.0004
G1 X40 Y48.4 E0.02
G1 X40 Y48.8 E0.0004
G1 X60 Y48.8 E0.02
G1 X60 Y49.2 E0.0004
G1 X40 Y49.2 E0.02
G1 X40 Y49.6 E0.0004
G1 X60 Y49.6 E0.02
G1 X60 Y50 E0.0004
G1 X40 Y50 E0.02
G1 X40 Y50.4 E0.0004
G1 X60 Y50.4 E0.02
G1 X60 Y50.8 E0.0004
G1 X40 Y50.8 E0.02
G1 X40 Y51.2 E0.0004
G1 X60 Y51.2 E0.02
G1 X60 Y51.6 E0.0004
G1 X40 Y51.6 E0.02
G1 X40 Y52 E0.0004
G1 X60 Y52 E0.02
G1 X60 Y52.4 E0.0004
G1 X40 Y52.4 E0.02
G1 X40 Y52.8 E0.0004
G1 X60 Y52.8 E0.02
G1 X60 Y53.2 E0.0004
G1 X40 Y53.2 E0.02
G1 X40 Y53.6 E0.0004
G1 X60 Y53.6 E0.02
G1 X60 Y54 E0.0004
G1 X40 Y54 E0.02
G1 X40 Y54.4 E0.0004
G1 X60 Y54.4 E0.02
G1 X60 Y54.8 E0.0004
G1 X40 Y54.8 E0.02
G1 X40 Y55.2 E0.0004
G1 X60 Y55.2 E0.02
G1 X60 Y55.6 E0.0004
G1 X40 Y55.6 E0.02
G1 X40 Y56 E0.0004
G1 X60 Y56 E0.02
G1 X60 Y56.4 E0.0004
G1 X40 Y56.4 E0.02
G1 X40 Y56.8 E0.0004
G1 X60 Y56.8 E0.02
G1 X60 Y57.2 E0.0004
G1 X40 Y57.2 E0.02
G1 X40 Y57.6 E0.0004
G1 X60 Y57.6 E0.02
G1 X60 Y58 E0.0004
G1 X40 Y58 E0.02
G1 X40 Y58.4 E0.0004
G1 X60 Y58.4 E0.02
G1 X60 Y58.8 E0.0004
G1 X40 Y58.8 E0.02
G1 X40 Y59.2 E0.0004
G1 X60 Y59.2 E0.02
G1 X60 Y59.6 E0.0004
G1 X40 Y59.6 E0.02
G1 X40 Y60 E0.0004
G1 X60 Y60 E0.02
G1 X0 Y0 F9000
;TYPE:Internal infill
G1 X10 Y10 E0.65 F1800
G1 X30 Y10 E0.65
G1 X30 Y30 E0.66
G1 Z0.4 F7200
G1 X0 Y0 F9000
;TYPE:Internal infill
G1 X10 Y10 E0.65 F1800
G1 X30 Y10 E0.65
G1 X30 Y30 E0.66
G1 X40 Y40 F9000
;TYPE:Internal infill
G1 X60 Y40 E0.02
G1 X60 Y40.4 E0.0004
G1 X40 Y40.4 E0.02
G1 X40 Y40.8 E0.0004
G1 X60 Y40.8 E0.02
G1 X60 Y41.2 E0.0004
G1 X40 Y41.2 E0.02
G1 X40 Y41.6 E0.0004
G1 X60 Y41.6 E0.02
G1 X60 Y42 E0.0004
G1 X40 Y42 E0.02
G1 X40 Y42.4 E0.0004
G1 X60 Y42.4 E0.02
G1 X60 Y42.8 E0.0004
G1 X40 Y42.8 E0.02
G1 X40 Y43.2 E0.0004
G1 X60 Y43.2 E0.02
G1 X60 Y43.6 E0.0004
G1 X40 Y43.6 E0.02
G1 X40 Y44 E0.0004
G1 X60 Y44 E0.02
G1 X60 Y44.4 E0.0004
G1 X40 Y44.4 E0.02
G1 X40 Y44.8 E0.0004
G1 X60 Y44.8 E0.02
G1 X60 Y45.2 E0.0004
G1 X40 Y45.2 E0.02
G1 X40 Y45.6 E0.0004
G1 X60 Y45.6 E0.02
G1 X60 Y46 E0.0004
G1 X40 Y46 E0.02
G1 X40 Y46.4 E0.0004
G1 X60 Y46.4 E0.02
G1 X60 Y46.8 E0.0004
G1 X40 Y46.8 E0.02
G1 X40 Y47.2 E0.0004
G1 X60 Y47.2 E0.02
G1 X60 Y47.6 E0.0004
G1 X40 Y47.6 E0.02
G1 X40 Y48 E0.0004
G1 X60 Y48 E0.02
G1 X60 Y48.4 E0.0004
G1 X40 Y48.4 E0.02
G1 X40 Y48.8 E0.0004
G1 X60 Y48.8 E0.02
G1 X60 Y49.2 E0.0004
G1 X40 Y49.2 E0.02
G1 X40 Y49.6 E0.0004
G1 X60 Y49.6 E0.02
G1 X60 Y50 E0.0004
G1 X40 Y50 E0.02
G1 X40 Y50.4 E0.0004
G1 X60 Y50.4 E0.02
G1 X60 Y50.8 E0.0004
G1 X40 Y50.8 E0.02
G1 X40 Y51.2 E0.0004
G1 X60 Y51.2 E0.02
G1 X60 Y51.6 E0.0004
G1 X40 Y51.6 E0.02
G1 X40 Y52 E0.0004
G1 X60 Y52 E0.02
G1 X60 Y52.4 E0.0004
G1 X40 Y52.4 E0.02
G1 X40 Y52.8 E0.0004
G1 X60 Y52.8 E0.02
G1 X60 Y53.2 E0.0004
G1 X40 Y53.2 E0.02
G1 X40 Y53.6 E0.0004
G1 X60 Y53.6 E0.02
G1 X60 Y54 E0.0004
G1 X40 Y54 E0.02
G1 X40 Y54.4 E0.0004
G1 X60 Y54.4 E0.02
G1 X60 Y54.8 E0.0004
G1 X40 Y54.8 E0.02
G1 X40 Y55.2 E0.0004
G1 X60 Y55.2 E0.02
G1 X60 Y55.6 E0.0004
G1 X40 Y55.6 E0.02
G1 X40 Y56 E0.0004
G1 X60 Y56 E0.02
G1 X60 Y56.4 E0.0004
G1 X40 Y56.4 E0.02
G1 X40 Y56.8 E0.0004
G1 X60 Y56.8 E0.02
G1 X60 Y57.2 E0.0004
G1 X40 Y57.2 E0.02
G1 X40 Y57.6 E0.0004
G1 X60 Y57.6 E0.02
G1 X60 Y58 E0.0004
G1 X40 Y58 E0.02
G1 X40 Y58.4 E0.0004
G1 X60 Y58.4 E0.02
G1 X60 Y58.8 E0.0004
G1 X40 Y58.8 E0.02
G1 X40 Y59.2 E0.0004
G1 X60 Y59.2 E0.02
G1 X60 Y59.6 E0.0004
G1 X40 Y59.6 E0.02
G1 X40 Y60 E0.0004
G1 X60 Y60 E0.02
";

    fn temp_dir(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-pipeline-e2e-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// The crash moment: the reader finished layer 0 and sits at the
    /// start of the `G1 Z0.4` line (a line boundary).
    fn crash_offset() -> u64 {
        MODEL_TEXT.find("G1 Z0.4").unwrap() as u64
    }

    fn crash_context(file_path: &str) -> Context {
        Context {
            mono_ns: 5_000_000_000,
            // Faithful to the recorder: Klipper reports
            // `toolhead.print_time` (the trapq append frontier) in the same
            // status pass as `file_position`. 10.0 matches this fixture's
            // heartbeat and the end of `preceding_motion`.
            print_time: Some(10.0),
            virtual_sdcard: Some(VirtualSdState {
                file_path: file_path.to_owned(),
                file_position: crash_offset(),
                file_size: None,
            }),
            gcode: GcodeState {
                speed_factor: 1.0,
                speed: 1_800.0,
                extrude_factor: 1.0,
                absolute_coordinates: true,
                absolute_extrude: false,
                homing_origin: vec![0.0, 0.0, 0.0, 0.0],
                position: vec![30.0, 30.0, 0.2, 3.0],
                gcode_position: vec![30.0, 30.0, 0.2, 3.0],
            },
            transforms: TransformObservations {
                bed_mesh_active: false,
                bed_mesh_profile: None,
                z_thermal_adjust_enabled: None,
                z_thermal_adjust_offset: None,
                skew_active: false,
                skew_profile: None,
            },
            heaters: vec![
                HeaterTarget {
                    name: "extruder".to_owned(),
                    target: 210.0,
                },
                HeaterTarget {
                    name: "heater_bed".to_owned(),
                    target: 60.0,
                },
            ],
            fans: vec![FanTarget {
                name: "fan".to_owned(),
                speed: 0.5,
            }],
            exclude: None,
            print_state: None,
            current_layer: None,
            total_layer: None,
        }
    }

    fn heartbeat() -> Heartbeat {
        Heartbeat {
            sequence: 7,
            mono_ns: 5_000_000_000,
            wall_ns: 1_700_000_000_000_000_000,
            print_time: 10.0,
            est_sample_mono_ns: 5_000_000_000,
            est_sample_print_time: 10.0,
            wal_offset: 64,
        }
    }

    /// One durable trapq row for the motion that precedes the crash
    /// context's processing frontier, ending exactly at the heartbeat's
    /// print time.
    ///
    /// Load-bearing, and the reason is narrow: a zero-trapq WAL is
    /// perfectly possible (see
    /// [`early_print_fixture`] — a cut in the opening moments of a print,
    /// or shortly after a klippy/plrd restart, has no durable motion row
    /// yet). What cannot happen is this fixture's *combination*: a
    /// `Context` reporting a frontier at byte 2138 with the toolhead
    /// already moved to (30, 30, 0.2) and 3 mm extruded, while the log
    /// contains no trapq row for any of that motion. Klipper journals a
    /// row per planned move, so motion that demonstrably happened has
    /// rows; claiming the motion without them is an impossible input.
    ///
    /// It matters because without a row journaled before the anchor
    /// context `plr-reconstruct` cannot place the frontier on the
    /// print-time axis from motion evidence and falls back to the
    /// reader-lead bound (`t_a` minus `max_processing_lead`), which
    /// correctly assumes execution may lag the frontier by seconds and so
    /// widens the extension horizon — and with it the offset candidate
    /// window, past what the matcher can resolve per line. With this row
    /// the fixture exercises the anchored path, which is the path a
    /// mid-print crash takes. The unanchored path keeps its own
    /// end-to-end coverage in
    /// [`an_early_print_cut_without_motion_evidence_falls_back_to_manual`].
    fn preceding_motion() -> TrapqSegment {
        TrapqSegment {
            mono_ns: 4_500_000_000,
            queue: "toolhead".to_owned(),
            print_time: 9.5,
            duration: 0.5,
            start_velocity: 20.0,
            acceleration: 0.0,
            start_x: 20.0,
            start_y: 30.0,
            start_z: 0.2,
            x_r: 1.0,
            y_r: 0.0,
            z_r: 0.0,
        }
    }

    /// Builds a WAL dir + print file + config primed to reach a plan.
    pub(crate) fn fixture(tag: &str) -> (PathBuf, Config) {
        let dir = temp_dir(tag);
        let gcode_path = dir.join("part.gcode");
        std::fs::write(&gcode_path, MODEL_TEXT.as_bytes()).unwrap();
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer.append(&WalRecord::Heartbeat(heartbeat())).unwrap();
        writer
            .append(&WalRecord::TrapqSegment(preceding_motion()))
            .unwrap();
        // An early context (before any deposition) anchors the layer
        // model so it covers layer 0 — contact selection probes layer
        // N−1, which must exist in the modeled window.
        let mut early = crash_context(gcode_path.to_str().unwrap());
        early.mono_ns = 1_000_000_000;
        // The append frontier must move with the clock: an earlier context
        // claiming the later context's print_time is a shape the recorder
        // cannot produce (Klipper's `print_time` is monotone within a
        // klippy instance) and would hand the coverage certificate a lie.
        early.print_time = Some(6.0);
        if let Some(vsd) = &mut early.virtual_sdcard {
            vsd.file_position = 8; // after G90/M83, before layer 0
        }
        early.gcode.position = vec![0.0, 0.0, 0.0, 0.0];
        early.gcode.gcode_position = vec![0.0, 0.0, 0.0, 0.0];
        writer.append(&WalRecord::Context(early)).unwrap();
        writer
            .append(&WalRecord::Context(crash_context(
                gcode_path.to_str().unwrap(),
            )))
            .unwrap();
        std::fs::write(dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        // Klipper config for change detection.
        let printer_cfg = dir.join("printer.cfg");
        std::fs::write(&printer_cfg, b"[force_move]\nenable_force_move: True\n").unwrap();
        let hash = super::klipper_config_hash(Some(&printer_cfg));
        let machine = MachineSection {
            force_move_enabled: true,
            z_self_locking_attested: true,
            z_steppers: vec![MachineZStepper {
                name: "stepper_z".to_owned(),
                mcu: None,
            }],
            primary_mcu: "mcu".to_owned(),
            probe_kind: Some("tap".to_owned()),
            probe_z_offset: Some(-0.1),
            probe_activate_gcode_no_move: true,
            probe_deactivate_gcode_no_move: true,
            z_position_min: Some(-2.0),
            klipper_config_path: Some(printer_cfg),
            validated_config_hash: Some(hash),
            virtual_sdcard_root: Some(dir.to_string_lossy().into_owned()),
        };
        let config = Config {
            wal_dir: dir.clone(),
            machine,
            ..Config::default()
        };
        (dir, config)
    }

    /// The internally consistent zero-trapq shape: a cut in the opening
    /// moments of a print (or just after a klippy/plrd restart), before
    /// any move was planned. The frontier has **not** moved — position is
    /// the origin, nothing extruded — so the absence of trapq rows is
    /// exactly what the machine state claims, unlike [`fixture`], whose
    /// frontier has moved and therefore must carry a row.
    ///
    /// This is the end-to-end coverage of `plr-reconstruct`'s unanchored
    /// horizon branch: with no motion evidence the extension origin comes
    /// from the reader-lead bound, which widens the candidate window past
    /// per-line resolution. See
    /// [`an_early_print_cut_without_motion_evidence_falls_back_to_manual`]
    /// for the outcome that is deliberately accepted.
    /// An ANCHORED fixture (motion evidence present, so the window is
    /// consistent) whose committed extruder position is lowered to widen
    /// the E band by a hair — enough to admit a 9th candidate line and tip
    /// the layer-0/1 window from `AmbiguousWindow` (8) to
    /// `MatchError::Inconclusive` (>8 across layers [0, 1]). The stop is at
    /// the layer 0->1 boundary (crash at `G1 Z0.4`), so the `last` anchor
    /// resumes at layer >= 1 and contact selection finds layer 0 below it —
    /// the full Inconclusive -> policy -> Plan chain in one fixture.
    fn inconclusive_fixture(tag: &str, committed_e: f64) -> (PathBuf, Config) {
        let (dir, config) = fixture(tag);
        let gcode_path = dir.join("part.gcode");
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer.append(&WalRecord::Heartbeat(heartbeat())).unwrap();
        // Keep the anchoring trapq row: the window stays consistent (motion
        // evidence exists), unlike the unanchored early-print shape.
        writer
            .append(&WalRecord::TrapqSegment(preceding_motion()))
            .unwrap();
        let mut early = crash_context(gcode_path.to_str().unwrap());
        early.mono_ns = 1_000_000_000;
        early.print_time = Some(6.0);
        if let Some(vsd) = &mut early.virtual_sdcard {
            vsd.file_position = 8;
        }
        early.gcode.position = vec![0.0, 0.0, 0.0, 0.0];
        early.gcode.gcode_position = vec![0.0, 0.0, 0.0, 0.0];
        writer.append(&WalRecord::Context(early)).unwrap();
        let mut crash = crash_context(gcode_path.to_str().unwrap());
        crash.gcode.position[3] = committed_e;
        crash.gcode.gcode_position[3] = committed_e;
        writer.append(&WalRecord::Context(crash)).unwrap();
        std::fs::write(dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        (dir, config)
    }

    /// THE FULL Inconclusive -> policy -> Plan CHAIN, in one test. The
    /// matcher's confidence ladder throws away every candidate line as
    /// `MatchError::Inconclusive` (>8 consistent lines across layers
    /// [0, 1]); the pipeline routes that raw evidence through
    /// `build_preview`, and — in this legacy-mode fixture, where `ask`
    /// resolves to `last` — the `last` anchor lands on a layer-1 stop, which
    /// contact selection can probe (layer 0 exists below it) and the planner
    /// turns into a validated Plan. This is the headless win: a coarse
    /// Inconclusive crash that used to be `ManualFallback` at the match call
    /// now resumes automatically. Contrast
    /// `an_early_print_cut_without_motion_evidence_falls_back_to_manual`,
    /// where the same Inconclusive routing lands on LAYER 0 and correctly
    /// declines (no layer below to probe).
    #[test]
    fn inconclusive_evidence_resolves_to_a_layer_ge_1_plan() {
        // E=2.0 -> the E band admits ~10 candidate lines across layers
        // [0, 1] (Inconclusive), with margin below the 8->9 tip so a small
        // upstream change does not silently drop it back to AmbiguousWindow.
        let (_dir, config) = inconclusive_fixture("inconclusive-plan", 2.0);
        let (outcome, output) = run(&config);
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!("expected a Plan from Inconclusive evidence, got {outcome:?}\n{output}");
        };
        // The match was genuinely Inconclusive (the Err path), across two
        // layers — not the AmbiguousWindow rung the other plan tests ride.
        assert!(
            output.contains("match inconclusive") && output.contains("across layers [0, 1]"),
            "the evidence must reach the Inconclusive routing: {output}"
        );
        let count: usize = output
            .lines()
            .find(|l| l.contains("match inconclusive"))
            .and_then(|l| l.split_whitespace().find_map(|w| w.parse().ok()))
            .unwrap_or(0);
        assert!(
            count > 8,
            "Inconclusive requires > ambiguity_limit: {output}"
        );
        // Legacy mode resolves `ask` -> `last`, so this is an AUTOMATIC
        // resume: no interactive preview attached, a real resume line.
        let plan = &bundle.plan;
        assert!(
            plan.preview.is_none(),
            "legacy `last` is automatic: {output}"
        );
        assert!(plan.resume_preview_step_iff_spec());
        // Layer >= 1 is proven structurally: the plan resolved a contact
        // (probe) step, which requires a layer BELOW the resume — impossible
        // at layer 0 (that is exactly why the early-print layer-0 fixture
        // declines). The resume also lands at or past the layer-1 boundary.
        assert!(
            plan.first_index(plr_recovery::Phase::Probe).is_some(),
            "a Plan from this evidence must have probed layer N-1: {output}"
        );
        assert!(
            plan.resume_offset >= crash_offset(),
            "resume {} must be at/after the layer-1 boundary {}: {output}",
            plan.resume_offset,
            crash_offset()
        );
        assert_eq!(plan.recovery_file.tail_offset, plan.resume_offset);
    }

    fn early_print_fixture(tag: &str) -> (PathBuf, Config) {
        let (dir, config) = fixture(tag);
        let gcode_path = dir.join("part.gcode");
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer.append(&WalRecord::Heartbeat(heartbeat())).unwrap();
        let mut early = crash_context(gcode_path.to_str().unwrap());
        early.mono_ns = 5_000_000_000;
        if let Some(vsd) = &mut early.virtual_sdcard {
            vsd.file_position = 8; // after G90/M83, before layer 0
        }
        // Nothing has moved and nothing has been extruded, which is why
        // no trapq row exists: consistent, not impossible.
        early.gcode.position = vec![0.0, 0.0, 0.0, 0.0];
        early.gcode.gcode_position = vec![0.0, 0.0, 0.0, 0.0];
        writer.append(&WalRecord::Context(early)).unwrap();
        std::fs::write(dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        (dir, config)
    }

    fn run(config: &Config) -> (PipelineOutcome, String) {
        let mut out = Vec::new();
        let outcome = run_pipeline(config, &mut out).expect("pipeline hard error");
        (outcome, String::from_utf8(out).unwrap())
    }

    /// End-to-end coverage of `plr-reconstruct`'s **unanchored** horizon
    /// branch, and a ratification of the trade it makes.
    ///
    /// A cut before any move was planned leaves no durable trapq row, so
    /// the extension's start time cannot be read from motion evidence and
    /// falls back to the reader-lead bound (`t_a` minus
    /// `max_processing_lead`). That bound assumes execution may lag the
    /// recorded frontier by seconds, which is what keeps the set containing
    /// the truth, and it widens the offset candidate window.
    ///
    /// The outcome is `ManualFallback`. The frontier cap (Part 1) narrows
    /// only the offset window's *high* end, from the parser-leads-execution
    /// bound; on this unanchored early crash the cap's honestly-priced Δt
    /// (record spacing + batch durability lag + two subscription draws —
    /// large here, since the fixture carries a single heartbeat, so the
    /// conservative tolerance spacing applies) barely trims the reader-lead
    /// horizon. The window therefore stays wider than per-line granularity —
    /// 15 candidate lines within layer 0 — and the *matcher* declines
    /// ("stop-point match failed … below layer granularity"), the deliberate
    /// degradation this fixture ratifies.
    ///
    /// **History of this golden.** It moved twice, both deliberate. The
    /// first cap shipped with a too-tight Δt (nominal 1× period + one draw =
    /// 1.25 s) that over-narrowed this window to 7 candidates and flipped
    /// the decline to contact selection — the exact over-narrowing the
    /// adversarial review flagged as unsound. Pricing Δt honestly (batch
    /// durability lag + the second subscription draw + observed-spacing
    /// terminal gap) restores the matcher-ambiguity decline and re-arms the
    /// `count >= 12` guard, which now again catches an unsound re-narrowing.
    ///
    /// Containment: the cap narrows only the high end, so the window LOW end
    /// is still the frontier (byte 8) and the truth — a stop that had not
    /// advanced past the frontier — remains contained (asserted below). The
    /// attribution line is window-relative (base 8 ≠ file start), the
    /// MAJOR-fix behavior: no absolute layer claim on a mid-file model.
    ///
    /// `docs/operations.md` carries the operator-facing half — an early-print
    /// power loss lands in manual recovery, which costs the operator little
    /// because almost nothing has printed.
    #[test]
    fn an_early_print_cut_without_motion_evidence_falls_back_to_manual() {
        let (_dir, config) = early_print_fixture("early-print-unanchored");
        let (outcome, output) = run(&config);
        let PipelineOutcome::ManualFallback(reason) = outcome else {
            panic!("expected ManualFallback, got {outcome:?}\n{output}");
        };
        // The honestly-priced Δt leaves the window past per-line
        // granularity, so the matcher declines Inconclusive. MAJOR-2 routes
        // that raw evidence through the resume-preview path instead of
        // failing at the match call. In LEGACY mode the increment-3
        // pipeline resolves `ask` to `last` (legacy carries no [plr]), so
        // the raw evidence builds a preview set and `last` anchors on a
        // layer-0 stop — which contact selection then declines, because a
        // layer-0 resume has no layer N-1 to probe under the nozzle. The
        // outcome is unchanged (ManualFallback: an early-print cut cannot
        // resume); only the decline reason moved from "match too coarse" to
        // the honest "no probeable layer below layer 0". The candidate
        // count and granularity message still live in the routing
        // NARRATION (asserted below).
        assert!(
            reason.contains("resume layer 0 out of range")
                || (reason.contains("no safe resume point") && reason.contains("MatchTooCoarse")),
            "reason changed: {reason}\n{output}"
        );
        assert!(
            output.contains("match inconclusive")
                && output.contains("candidate lines across layers"),
            "the inconclusive routing must be narrated: {output}"
        );
        // Re-armed numeric guard: the reader-lead widening leaves > 12
        // candidate lines (parsed from the narration now). A cap that
        // over-narrows (the review's blocker) would drop this below 12.
        let count: usize = output
            .lines()
            .find(|l| l.contains("match inconclusive"))
            .and_then(|l| l.split_whitespace().find_map(|w| w.parse().ok()))
            .unwrap_or(0);
        assert!(
            count >= 12,
            "only {count} candidate lines — an over-narrowing cap is back: {output}"
        );
        // Containment: the window LOW end is still the frontier (byte 8).
        // The matcher declined, so there is no AmbiguousWindow offsets line;
        // the offset window is visible in the attribution narration instead.
        assert!(
            output.contains("offset window bytes 8..="),
            "the frontier (byte 8) must remain the window low end: {output}"
        );
        // MAJOR fix: a mid-file model reports window-relative layers, never
        // an absolute layer number.
        assert!(
            output.contains("layer attribution:") && output.contains("window-relative"),
            "mid-file attribution must be window-relative: {output}"
        );
        // Nothing is lost by declining here: a stop this early has no layer
        // below the resume layer to probe. See docs/operations.md.
        assert!(
            output.contains("layer model from byte 8: 2 layers"),
            "{output}"
        );
    }

    #[test]
    fn full_pipeline_reaches_a_validated_plan() {
        let (_dir, config) = fixture("plan");
        let (outcome, output) = run(&config);
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!("expected a plan, got {outcome:?}\n{output}");
        };
        let plan = &bundle.plan;
        // This AmbiguousWindow fixture runs in LEGACY mode (klippy
        // unreachable + a commissioned [machine]), where the increment-3
        // pipeline resolves the interactive `ask` default to its automatic
        // `last` fallback (legacy has no [plr] plugin to present a preview
        // through — see `machine_inputs`). So a legacy recovery is an
        // AUTOMATIC skip-forward plan: NO ResumePreview step, no preview
        // spec, resume at a real line — not a ManualFallback and not a
        // preview pause. `plr_mode_ask_attaches_the_resume_preview` covers
        // the [plr]-mode default where the preview IS attached.
        assert!(
            plan.first_index(plr_recovery::Phase::ResumePreview)
                .is_none(),
            "legacy `ask`->`last` must not attach a preview step: {output}"
        );
        assert!(plan.preview.is_none(), "{output}");
        assert!(plan.resume_preview_step_iff_spec());
        // The plan honors every structural invariant plr-recovery
        // promises.
        assert!(plan.idle_timeout_first(), "{output}");
        assert!(plan.steppers_enabled_before_motion());
        assert!(plan.temp_verify_precedes_probe());
        assert!(plan.probe_step_precedes_mesh_load());
        assert!(plan.no_g28_after_shifted_declare());
        // Resume targets the interrupted file at a line boundary at or
        // after the crash offset (skip-forward is the safe direction).
        // The plan now selects the GENERATED recovery file.
        assert_eq!(plan.resume_file, "part_RECOVERY.gcode");
        assert_eq!(plan.recovery_file.source_name, "part.gcode");
        assert!(
            plan.resume_offset >= crash_offset(),
            "resume {} before crash {}",
            plan.resume_offset,
            crash_offset()
        );
        assert_eq!(plan.recovery_file.tail_offset, plan.resume_offset);
        // The pipeline generated the recovery-file content and resolved
        // its write path under the sdcard root.
        let content = String::from_utf8_lossy(&bundle.recovery_file_content).into_owned();
        assert!(content.contains("G28 X Y"));
        // The purge runs AFTER travelling back to the part-clear park
        // point, never at the homed XY G28 leaves behind (finding 1).
        let g28 = content.find("G28 X Y").expect("re-home");
        let purge = content.find("G1 E").expect("purge");
        assert!(
            content[g28..purge].contains("G0 X"),
            "a re-park travel must sit between G28 and the purge"
        );
        assert!(bundle
            .recovery_file_path
            .to_string_lossy()
            .ends_with("part_RECOVERY.gcode"));
        assert!(
            output.contains("machine prerequisites validated"),
            "{output}"
        );
        // Part 2: the cap-narrowed offset window is mapped through the
        // layer model and narrated. This fixture's anchor context carries
        // no slicer marks, so the marks-unavailable branch fires and the
        // geometric attribution carries the answer.
        assert!(
            output.contains("layer attribution:"),
            "layer attribution must be narrated: {output}"
        );
        assert!(
            output.contains("layer marks unavailable"),
            "a mark-less fixture must say so honestly: {output}"
        );
    }

    /// **D1 in the pipeline.** A print that finished must not be planned
    /// for. Without the gate the planner declines with
    /// `NoResumeDeposition` and the operator is told to fix a reported
    /// issue — for a print that simply ran to the end.
    /// Writes a *finished* print into the fixture: `MODEL_TEXT` plus an end
    /// sequence plus a config block, with both contexts inside the footer, so
    /// the completion gate would answer `Complete` unless a precondition
    /// stops it. Returns the footer's offset and the file's size.
    ///
    /// `journaled_size` is what the WAL claims `virtual_sdcard.file_size`
    /// was; `truncate_to` optionally shortens the file on disk afterwards.
    fn write_finished_print(
        dir: &std::path::Path,
        journaled_size: Option<u64>,
        truncate_to: Option<usize>,
    ) -> (u64, u64) {
        let gcode_path = dir.join("part.gcode");
        let footer = "M107\nM104 S0\nM140 S0\nG1 E-0.8 F2100\nG1 Z10 F600\nM84\n";
        let mut text = MODEL_TEXT.to_owned();
        let footer_at = text.len() as u64;
        text.push_str(footer);
        for i in 0..300 {
            use std::fmt::Write as _;
            let _ = writeln!(text, "; some_config_key_{i} = 0");
        }
        if let Some(len) = truncate_to {
            text.truncate(len);
        }
        std::fs::write(&gcode_path, text.as_bytes()).unwrap();
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer.append(&WalRecord::Heartbeat(heartbeat())).unwrap();
        for position in [footer_at, footer_at + footer.len() as u64] {
            let mut ctx = crash_context(gcode_path.to_str().unwrap());
            if position == footer_at {
                ctx.mono_ns = 1_000_000_000;
            }
            if let Some(vsd) = &mut ctx.virtual_sdcard {
                vsd.file_position = position.min(text.len() as u64);
                vsd.file_size = journaled_size;
            }
            writer.append(&WalRecord::Context(ctx)).unwrap();
        }
        std::fs::write(dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        (footer_at, text.len() as u64)
    }

    /// **The re-slice probe.** An operator re-slices under the same filename
    /// before running recovery — the ordinary iteration loop — so the path
    /// still resolves but the content is different and `file_position` indexes
    /// into bytes we never saw. `plrd recover` must not answer "nothing to
    /// recover"; it must say the file changed.
    ///
    /// This is the operator's primary tool, and an earlier revision of this
    /// branch enforced the identity check only in boot detection, so this
    /// path suppressed silently. See `detect::completion_verdict`.
    #[test]
    fn a_re_sliced_file_is_not_reported_complete_by_the_recover_path() {
        let (dir, config) = fixture("reslice");
        // The WAL claims a much larger file than the one on disk.
        let (_, on_disk) = write_finished_print(&dir, Some(512_004), None);
        assert_ne!(on_disk, 512_004);
        let (outcome, output) = run(&config);
        // Not merely "not complete": the WAL's offsets no longer address this
        // file, so a resume plan built from them would seek into content we
        // never recorded. The outcome must be the REFUSAL, not the planner's
        // `ManualFallback("no safe resume point: NoResumeDeposition")` — that
        // is the phantom-issue failure this feature exists to stop.
        let PipelineOutcome::NotPossible(reason) = &outcome else {
            panic!("expected NotPossible, got {outcome:?}\n{output}");
        };
        assert!(reason.contains("512004"), "{reason}");
        assert!(reason.contains("re-sliced"), "{reason}");
        assert!(
            output.contains("512004"),
            "the reason must be narrated: {output}"
        );
    }

    /// The other direction of the same denial: a print that is genuinely
    /// recoverable must NOT be refused. A widened `invalidates_offsets` would
    /// deny a resumable print, which is the expensive mistake.
    #[test]
    fn a_recoverable_print_is_not_denied_by_the_identity_check() {
        let (_dir, config) = fixture("identity-ok");
        // `fixture` writes a mid-print WAL whose contexts journal no size at
        // all, so the identity check has nothing to object to.
        let (outcome, output) = run(&config);
        assert!(
            matches!(outcome, PipelineOutcome::Plan(_)),
            "a recoverable print must still plan: {outcome:?}\n{output}"
        );
    }

    /// **The truncation probe.** A file truncated at the stop offset, with no
    /// journaled size to compare against. Zero bytes of remainder is zero
    /// evidence — and answering `Complete` here would print the
    /// self-refuting "0 trailing bytes are the slicer footer", when a footer
    /// is 14-18 KB.
    #[test]
    fn a_file_truncated_at_the_stop_offset_is_not_reported_complete() {
        let (dir, config) = fixture("truncated");
        // Truncate to exactly the footer offset, and journal no size at all.
        let footer_at = MODEL_TEXT.len();
        let (_, on_disk) = write_finished_print(&dir, None, Some(footer_at));
        assert_eq!(on_disk, footer_at as u64);
        let (outcome, output) = run(&config);
        assert!(
            !matches!(outcome, PipelineOutcome::Complete(_)),
            "an empty remainder must never report completion: {outcome:?}\n{output}"
        );
        assert!(
            !output.contains("0 trailing bytes"),
            "the self-refuting line must never be printed: {output}"
        );
    }

    /// The same finished print, unperturbed, still reports `Complete` — so
    /// the two probes above are testing the preconditions and not merely a
    /// broken fixture.
    #[test]
    fn the_probe_scaffolding_still_reports_a_genuine_completion() {
        let (dir, config) = fixture("probe-control");
        // Journal the size the file actually has: identity holds.
        let (footer_at, on_disk) = write_finished_print(&dir, None, None);
        let (_, again) = write_finished_print(&dir, Some(on_disk), None);
        assert_eq!(again, on_disk);
        let (outcome, output) = run(&config);
        let PipelineOutcome::Complete(report) = outcome else {
            panic!("expected Complete, got {outcome:?}\n{output}");
        };
        assert_eq!(report.tested_offset, footer_at);
        assert!(report.trailing_bytes() > 6_000);
    }

    #[test]
    fn a_finished_print_reaches_no_recovery_instead_of_a_failure() {
        let (dir, config) = fixture("complete");
        // Append a footer and point the crash context at its first byte:
        // the print reached the end of its deposition.
        let gcode_path = dir.join("part.gcode");
        let footer = "M107
M104 S0
M140 S0
G1 E-0.8 F2100
G1 Z10 F600
M84
";
        let mut text = MODEL_TEXT.to_owned();
        let footer_at = text.len() as u64;
        text.push_str(footer);
        // Plus a config block, as every slicer writes.
        for i in 0..300 {
            use std::fmt::Write as _;
            let _ = writeln!(text, "; some_config_key_{i} = 0");
        }
        std::fs::write(&gcode_path, text.as_bytes()).unwrap();
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer.append(&WalRecord::Heartbeat(heartbeat())).unwrap();
        // Both contexts sit inside the footer: the reader had already
        // dispatched every depositing line and was working through the
        // end g-code when the host died. `file_window.start` is the
        // newest context provably behind execution, so it lands in the
        // footer whichever branch `floor_context` takes.
        let mut early = crash_context(gcode_path.to_str().unwrap());
        early.mono_ns = 1_000_000_000;
        if let Some(vsd) = &mut early.virtual_sdcard {
            vsd.file_position = footer_at;
        }
        writer.append(&WalRecord::Context(early)).unwrap();
        let mut done = crash_context(gcode_path.to_str().unwrap());
        if let Some(vsd) = &mut done.virtual_sdcard {
            vsd.file_position = footer_at + footer.len() as u64;
        }
        writer.append(&WalRecord::Context(done)).unwrap();
        std::fs::write(dir.join("wal-000001.plr"), writer.into_inner()).unwrap();

        let (outcome, output) = run(&config);
        let PipelineOutcome::Complete(report) = outcome else {
            panic!(
                "expected Complete, got {outcome:?}
{output}"
            );
        };
        assert_eq!(report.tested_offset, footer_at);
        assert!(
            report.trailing_bytes() > 6_000,
            "{}",
            report.trailing_bytes()
        );
        assert_eq!(report.unrun_commands()[0], "M107");
        assert!(output.contains("is COMPLETE"), "{output}");
        assert!(output.contains("did not run: M107"), "{output}");
        assert!(output.contains("NOT offered"), "{output}");
        // `recover.rs` renders this outcome accurately (exit 0, and no
        // claim that the log ended cleanly): see
        // `recover::tests::non_plan_outcomes_map_to_exit_codes`.
    }

    #[test]
    fn default_machine_section_is_rejected_before_planning() {
        let (_dir, mut config) = fixture("rejected");
        // A barely-commissioned legacy section (probe_kind names the
        // mode; everything else default): the pipeline must reach the
        // legacy path and list every unmet prerequisite. probe_kind is
        // what marks the legacy snapshot as commissioned — without it
        // (and with klippy unreachable) the outcome is the documented
        // `NotPossible` refusal, tested separately.
        config.machine = MachineSection {
            probe_kind: Some("tap".to_owned()),
            ..MachineSection::default()
        };
        let (outcome, output) = run(&config);
        let PipelineOutcome::MachineRejected(rejection) = outcome else {
            panic!("expected machine rejection, got {outcome:?}\n{output}");
        };
        // Every unmet prerequisite is listed, not just the first.
        assert!(rejection.failures.len() >= 5, "{rejection:?}");
        assert!(output.contains("machine hash computed"), "{output}");
        assert!(output.contains("legacy mode"), "{output}");
    }

    #[test]
    fn uncommissioned_machine_with_unreachable_klippy_refuses() {
        // No [plr] reachable (the fixture's klippy socket points
        // nowhere) and no commissioned legacy snapshot: recovery AND
        // dry-run refuse with the documented message instead of
        // planning from invented machine data.
        let (_dir, mut config) = fixture("unavailable");
        config.machine = MachineSection::default();
        let (outcome, output) = run(&config);
        let PipelineOutcome::NotPossible(reason) = outcome else {
            panic!("expected not-possible, got {outcome:?}\n{output}");
        };
        assert!(reason.contains("klippy is unreachable"), "{reason}");
        assert!(reason.contains("dry run"), "{reason}");
    }

    #[test]
    fn missing_print_file_is_not_possible() {
        let (dir, config) = fixture("nofile");
        std::fs::remove_file(dir.join("part.gcode")).unwrap();
        let (outcome, _) = run(&config);
        let PipelineOutcome::NotPossible(reason) = outcome else {
            panic!("expected not-possible, got {outcome:?}");
        };
        assert!(reason.contains("unreadable"), "{reason}");
    }

    /// [plr]-mode e2e (Unix: needs a fake klippy on a real socket):
    /// the machine snapshot comes from the live settings and the
    /// LEGACY SECTION IS IGNORED — the fixture's [machine] is left at
    /// its all-false defaults, which would refuse instantly in legacy
    /// mode, and the pipeline still reaches a plan.
    #[cfg(unix)]
    pub(crate) fn plr_fixture(
        tag: &str,
        plr_overrides: &[(&str, serde_json::Value)],
    ) -> (PathBuf, Config) {
        use crate::plrcfg::tests as fixtures;
        let (dir, mut config) = fixture(tag);
        let mut configfile = fixtures::configfile_status(plr_overrides);
        // Point [virtual_sdcard] at the fixture dir so the print file
        // is top-level.
        configfile["settings"]["virtual_sdcard"]["path"] =
            serde_json::Value::String(dir.to_string_lossy().into_owned());
        let response = fixtures::query_result(configfile, fixtures::plr_object());
        config.klipper_socket = fixtures::spawn_fake_klippy(tag, response);
        // The legacy section is deliberately UNcommissioned: only [plr]
        // can make this machine pass.
        config.machine = MachineSection::default();
        (dir, config)
    }

    #[cfg(unix)]
    #[test]
    fn plr_mode_reaches_a_plan_and_ignores_the_legacy_section() {
        let (_dir, config) = plr_fixture("plr-plan", &[]);
        let (outcome, output) = run(&config);
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!("expected a plan, got {outcome:?}\n{output}");
        };
        assert!(
            output.contains("machine config from the [plr] section"),
            "{output}"
        );
        assert!(
            output.contains("[machine] is ignored while [plr] exists"),
            "{output}"
        );
        // Tap method in [plr] mode: the consensus PLR_TOUCH path with
        // the [plr] tunables, wrapped by an accel clamp/restore.
        let probe = bundle
            .plan
            .steps
            .iter()
            .find(|s| s.phase == plr_recovery::Phase::Probe)
            .expect("probe step");
        assert_eq!(
            probe.commands,
            vec!["PLR_TOUCH SAMPLES=3 SAMPLE_RANGE=0.01 SPEED=1 RETRACT=2 TOUCH_ACCEL=100"]
        );
        assert!(bundle
            .plan
            .first_index(plr_recovery::Phase::AccelClamp)
            .is_some());
        assert!(bundle.plan.accel_clamp_precedes_probe());
        assert!(bundle.plan.accel_restore_follows_probe());
        // Live-config mode: the hash blessing is satisfied by
        // construction.
        assert_eq!(bundle.machine.config_hash, crate::plrcfg::LIVE_CONFIG_HASH);
    }

    #[cfg(unix)]
    #[test]
    fn plr_mode_ask_attaches_the_resume_preview() {
        // THE INCREMENT-3 `ask` FLIP, shipped. In [plr] mode the ratified
        // default is `ask`, and the pipeline now BUILDS the preview set for
        // it and passes it through, so `plan_recovery` attaches the
        // interactive preview: a `Phase::ResumePreview` step and a preview
        // spec, opening the reposition loop instead of resuming
        // automatically. (Legacy mode still resolves ask->last — proven by
        // `full_pipeline_reaches_a_validated_plan`, which is the SAME WAL in
        // legacy mode and attaches NO preview.)
        //
        // The AmbiguousWindow "plan" fixture reaches the matcher with a
        // small candidate set (not UniqueLine), so `build_preview` yields a
        // usable set rather than NoStops — the condition for `ask` to attach.
        let (_dir, config) = plr_fixture("plr-ask-preview", &[]);
        let (outcome, output) = run(&config);
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!("expected a plan, got {outcome:?}\n{output}");
        };
        let plan = &bundle.plan;
        assert!(
            plan.preview.is_some(),
            "the ratified `ask` default must attach a preview spec: {output}"
        );
        assert!(
            plan.first_index(plr_recovery::Phase::ResumePreview)
                .is_some(),
            "`ask` must add a ResumePreview step: {output}"
        );
        // The structural invariant: step iff spec, and the step sits after
        // the shifted-frame declare (an abort during preview invalidates the
        // frame).
        assert!(plan.resume_preview_step_iff_spec());
        let preview = plan.preview.as_ref().expect("preview spec");
        assert!(
            !preview.stops.is_empty(),
            "the preview must carry the nudge domain: {output}"
        );
        // The reps are a non-empty subset of the stops (the navigation
        // anchors the operator steps between), and the default index opens
        // on `last` (the skip-forward dry-run anchor) — a valid stop index.
        assert!(!preview.representatives.is_empty(), "{output}");
        assert!(
            (preview.default_index as usize) < preview.stops.len(),
            "default index out of range: {output}"
        );
        // The routing was narrated as `ask` attaching the preview, not a
        // headless automatic resume.
        assert!(
            output.contains("machine config from the [plr] section"),
            "{output}"
        );
    }

    /// The ask-preview fixture, but with a journaled slicer `current_layer`
    /// mark (`SET_PRINT_STATS_INFO CURRENT_LAYER=`) on the WAL contexts, so
    /// the pipeline's absolute-frame corroboration gate is actually
    /// exercised. The mark is placed on BOTH contexts so whichever one
    /// anchors the replay carries it. `anchor_fp` is the anchor context's
    /// journaled `file_position` — the replay's `base_offset`: 0 makes the
    /// model span from file start (absolute frame, mark commensurable), any
    /// other value a mid-file model (mark withheld).
    #[cfg(unix)]
    fn plr_fixture_with_layer_mark(tag: &str, mark: u32, anchor_fp: u64) -> (PathBuf, Config) {
        let (dir, config) = plr_fixture(tag, &[]);
        let gcode_path = dir.join("part.gcode");
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer.append(&WalRecord::Heartbeat(heartbeat())).unwrap();
        writer
            .append(&WalRecord::TrapqSegment(preceding_motion()))
            .unwrap();
        let mut early = crash_context(gcode_path.to_str().unwrap());
        early.mono_ns = 1_000_000_000;
        early.print_time = Some(6.0);
        if let Some(vsd) = &mut early.virtual_sdcard {
            vsd.file_position = anchor_fp;
        }
        early.gcode.position = vec![0.0, 0.0, 0.0, 0.0];
        early.gcode.gcode_position = vec![0.0, 0.0, 0.0, 0.0];
        early.current_layer = Some(mark);
        early.total_layer = Some(mark + 5);
        writer.append(&WalRecord::Context(early)).unwrap();
        let mut crash = crash_context(gcode_path.to_str().unwrap());
        crash.current_layer = Some(mark);
        crash.total_layer = Some(mark + 5);
        writer.append(&WalRecord::Context(crash)).unwrap();
        std::fs::write(dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        (dir, config)
    }

    #[cfg(unix)]
    #[test]
    fn preview_provenance_is_withheld_on_a_mid_file_model_despite_a_slicer_mark() {
        // MAJOR-3: the absolute-frame corroboration gate is NON-vacuous.
        // This fixture CARRIES a journaled slicer current_layer mark, and the
        // model is built from a MID-FILE anchor (base_offset != 0) whose
        // window-relative layer ordinals are incommensurable with the
        // absolute mark — so the mark MUST be withheld:
        // corroborating_layer_mark is None and every stop reads as
        // model-inferred, never journal. Mutating the gate `base_offset == 0`
        // to `true` would wrongly ADOPT the mark (Some), which this test
        // catches.
        let (_dir, config) = plr_fixture_with_layer_mark("preview-provenance-midfile", 40, 8);
        let (outcome, output) = run(&config);
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!("expected a plan, got {outcome:?}\n{output}");
        };
        // The model must be mid-file for this gate to apply — the narration
        // proves it (a base-0 model would say "offset window bytes" instead).
        assert!(
            output.contains("window-relative to the resume anchor"),
            "the fixture must build a MID-FILE model for this gate to bite: {output}"
        );
        let preview = bundle
            .plan
            .preview
            .as_ref()
            .expect("the ratified `ask` default attaches a preview");
        assert_eq!(
            preview.corroborating_layer_mark, None,
            "a mid-file model must withhold the (incommensurable) slicer mark: {output}"
        );
    }

    #[cfg(unix)]
    #[test]
    fn preview_provenance_adopts_the_slicer_mark_on_a_base_zero_model() {
        // MAJOR-3, the other direction: with the anchor at file_position 0
        // the model spans from FILE START, so window ordinals ARE absolute
        // and the mark is commensurable — the pipeline adopts it as
        // corroborating_layer_mark, which the daemon's `preview_detail` then
        // turns into "journal" for stops whose layer is `<= mark`. Together
        // with the mid-file test above this pins BOTH sides of the
        // `base_offset == 0` gate (a mutation either way is caught).
        let (_dir, config) = plr_fixture_with_layer_mark("preview-provenance-base0", 40, 0);
        let (outcome, output) = run(&config);
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!("expected a plan, got {outcome:?}\n{output}");
        };
        // The model spans from file start — the narration says so.
        assert!(
            output.contains("layer attribution:") && !output.contains("window-relative"),
            "the fixture must build a BASE-0 (whole-file) model: {output}"
        );
        let preview = bundle
            .plan
            .preview
            .as_ref()
            .expect("the ratified `ask` default attaches a preview");
        assert_eq!(
            preview.corroborating_layer_mark,
            Some(40),
            "a base-0 model must adopt the absolute slicer mark: {output}"
        );
    }

    #[cfg(unix)]
    #[test]
    fn plr_mode_drag_method_emits_the_drag_probe() {
        use serde_json::json;
        let (_dir, config) = plr_fixture(
            "plr-drag",
            &[
                ("probe_method", json!("adxl_drag")),
                ("accel_chip", json!("adxl345")),
                ("noise_floor_rms", json!(118.0)),
                ("drag_speed", json!(4.0)),
                ("drag_z_step", json!(0.04)),
                ("drag_sensitivity", json!(2.5)),
                // Calibrated at 20 mm/s, planned at 4 mm/s: the plan
                // must carry the speed-mismatch warning (never a
                // refusal).
                ("noise_floor_speed", json!(20.0)),
            ],
        );
        let (outcome, output) = run(&config);
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!("expected a plan, got {outcome:?}\n{output}");
        };
        let probe = bundle
            .plan
            .steps
            .iter()
            .find(|s| s.phase == plr_recovery::Phase::Probe)
            .expect("probe step");
        assert_eq!(
            probe.commands,
            vec!["PLR_DRAG_PROBE CHIP=\"adxl345\" SPEED=4 Z_STEP=0.04 SENSITIVITY=2.5"]
        );
        // The drag envelope has no speed-proportional term.
        assert_eq!(
            bundle.plan.envelope.params.overshoot,
            plr_recovery::OvershootTerm::DragStep { drag_z_step: 0.04 }
        );
        // The [plr] noise_floor_speed rode through to the plan warning.
        assert!(
            bundle.plan.warnings.iter().any(|w| matches!(
                w,
                plr_recovery::PlanWarning::NoiseFloorSpeedMismatch {
                    calibrated_at,
                    drag_speed,
                } if (*calibrated_at - 20.0).abs() < 1e-12 && (*drag_speed - 4.0).abs() < 1e-12
            )),
            "{:?}",
            bundle.plan.warnings
        );
    }

    #[cfg(unix)]
    #[test]
    fn plr_mode_uncalibrated_drag_is_rejected_with_the_hint() {
        use serde_json::json;
        let (_dir, config) = plr_fixture(
            "plr-drag-uncal",
            &[
                ("probe_method", json!("adxl_drag")),
                ("accel_chip", json!("adxl345")),
            ],
        );
        let (outcome, output) = run(&config);
        let PipelineOutcome::MachineRejected(rejection) = outcome else {
            panic!("expected rejection, got {outcome:?}\n{output}");
        };
        assert!(
            rejection
                .failures
                .iter()
                .any(|f| f.to_string().contains("run PLR_NOISE_TEST first")),
            "{rejection:?}"
        );
        // [plr] mode never prints the legacy hash line.
        assert!(!output.contains("machine hash computed"), "{output}");
    }

    #[cfg(unix)]
    #[test]
    fn unreadable_plr_section_refuses_rather_than_falling_back() {
        use serde_json::json;
        // probe_method is invalid: the section exists but cannot be
        // trusted; even a commissioned legacy snapshot must NOT be
        // silently substituted.
        let (_dir, mut config) = plr_fixture("plr-bad", &[("probe_method", json!("laser"))]);
        config.machine = fixture("plr-bad-legacy").1.machine;
        let (outcome, _output) = run(&config);
        let PipelineOutcome::NotPossible(reason) = outcome else {
            panic!("expected not-possible, got {outcome:?}");
        };
        assert!(
            reason.contains("[plr] section is present but unreadable"),
            "{reason}"
        );
    }

    #[cfg(unix)]
    #[test]
    fn no_plr_section_stays_on_the_legacy_path() {
        use crate::plrcfg::tests as fixtures;
        let (_dir, mut config) = fixture("no-plr-legacy");
        // A live klippy whose config has no [plr] section at all.
        let configfile = serde_json::json!({
            "settings": {"printer": {"kinematics": "corexy"}},
            "config": {}, "warnings": [],
            "save_config_pending": false, "save_config_pending_items": {}
        });
        let response = fixtures::query_result(configfile, serde_json::json!({}));
        config.klipper_socket = fixtures::spawn_fake_klippy("no-plr", response);
        let (outcome, output) = run(&config);
        assert!(
            matches!(outcome, PipelineOutcome::Plan(_)),
            "legacy path must still plan: {outcome:?}\n{output}"
        );
        assert!(output.contains("legacy mode"), "{output}");
        // No fallback note: legacy applies by absence of [plr], not by
        // klippy being unreachable.
        assert!(!output.contains("klippy is unreachable"), "{output}");
    }

    #[test]
    fn clean_wal_short_circuits() {
        let (dir, config) = fixture("clean");
        // Rewrite the WAL with a clean-shutdown tail.
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer.append(&WalRecord::Heartbeat(heartbeat())).unwrap();
        writer
            .append(&WalRecord::Marker(plr_wal::Marker {
                mono_ns: 6_000_000_000,
                kind: plr_wal::MarkerKind::CleanShutdown,
            }))
            .unwrap();
        std::fs::write(dir.join("wal-000001.plr"), writer.into_inner()).unwrap();
        let (outcome, _) = run(&config);
        assert_eq!(outcome, PipelineOutcome::CleanShutdown);
    }
}

#[cfg(test)]
mod tests {
    use super::{contains_type_annotations, klipper_config_hash, machine_config, stop_evidence};
    use crate::config::{MachineSection, MachineZStepper};
    use plr_reconstruct::{
        Confidence, Degradation, Interval as RInterval, OffsetWindow, PossibleStopSet, Provenance,
        XyRegion, ZCandidate, ZKind,
    };
    use plr_recovery::{validate_machine, PrereqFailure, ProbeKind};
    use std::path::PathBuf;

    fn full_section() -> MachineSection {
        MachineSection {
            force_move_enabled: true,
            z_self_locking_attested: true,
            z_steppers: vec![
                MachineZStepper {
                    name: "stepper_z".to_owned(),
                    mcu: None,
                },
                MachineZStepper {
                    name: "stepper_z1".to_owned(),
                    mcu: Some("mcu".to_owned()),
                },
            ],
            primary_mcu: "mcu".to_owned(),
            probe_kind: Some("tap".to_owned()),
            probe_z_offset: Some(-0.1),
            probe_activate_gcode_no_move: true,
            probe_deactivate_gcode_no_move: true,
            z_position_min: Some(-2.0),
            klipper_config_path: None,
            validated_config_hash: None,
            virtual_sdcard_root: Some("/g".to_owned()),
        }
    }

    #[test]
    fn machine_config_maps_the_section_faithfully() {
        let m = machine_config(&full_section(), true, None);
        assert!(m.force_move_enabled);
        assert!(m.z_self_locking_attested);
        assert_eq!(m.z_steppers.len(), 2);
        // Bare stepper names inherit the primary MCU.
        assert_eq!(m.z_steppers[0].mcu, "mcu");
        assert!(m.type_annotations_present);
        assert_eq!(m.probes.len(), 1);
        assert_eq!(m.probes[0].kind, ProbeKind::Tap);
        assert!((m.probes[0].z_offset - (-0.1)).abs() < 1e-12);
        assert!(m.config_hash.contains("unavailable"));
    }

    #[test]
    fn load_cell_kind_and_missing_probe_fields() {
        let mut section = full_section();
        section.probe_kind = Some("load_cell".to_owned());
        let m = machine_config(&section, true, None);
        assert_eq!(m.probes[0].kind, ProbeKind::LoadCell);
        // Kind without offset (or vice versa) yields no probe: the
        // validator reports NoProbe instead of inventing a zero offset.
        section.probe_z_offset = None;
        let m = machine_config(&section, true, None);
        assert!(m.probes.is_empty());
        let rejection = validate_machine(&m).unwrap_err();
        assert!(rejection.failures.contains(&PrereqFailure::NoProbe));
    }

    #[test]
    fn config_hash_is_stable_and_detects_change() {
        let dir = std::env::temp_dir().join(format!(
            "plrd-pipeline-hash-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let cfg = dir.join("printer.cfg");
        std::fs::write(&cfg, b"[probe]\nz_offset: -0.1\n").unwrap();
        let h1 = klipper_config_hash(Some(&cfg));
        assert!(h1.starts_with("crc32c:"), "{h1}");
        assert_eq!(h1, klipper_config_hash(Some(&cfg)));
        std::fs::write(&cfg, b"[probe]\nz_offset: -0.2\n").unwrap();
        assert_ne!(h1, klipper_config_hash(Some(&cfg)));
        assert!(klipper_config_hash(Some(&PathBuf::from("/nonexistent"))).contains("unreadable"));
    }

    #[test]
    fn type_annotation_scan_finds_the_marker() {
        assert!(contains_type_annotations(b"G1 X0\n;TYPE:WALL-OUTER\nG1 X1"));
        assert!(!contains_type_annotations(b"G1 X0\n; type: nope\n"));
        assert!(!contains_type_annotations(b""));
    }

    fn stop_set(xy: Option<XyRegion>, window: Option<OffsetWindow>) -> PossibleStopSet {
        PossibleStopSet {
            t_a: 1.0,
            wal_eval_end: 2.0,
            z_candidates: vec![
                ZCandidate {
                    z: RInterval { lo: 4.2, hi: 4.2 },
                    provenance: Provenance::Wal,
                    z_known: true,
                    kind: ZKind::Plateau,
                },
                ZCandidate {
                    z: RInterval { lo: 4.0, hi: 4.4 },
                    provenance: Provenance::Wal,
                    z_known: false,
                    kind: ZKind::Ramp,
                },
            ],
            xy,
            e_internal: Some(RInterval { lo: 10.0, hi: 12.0 }),
            e_file: None,
            file_window: window,
            extension: None,
            // `..Degradation::default()` (every flag clear) so that adding
            // a degradation flag upstream cannot break this fixture. The
            // two fields kept explicit are not read by `stop_evidence`;
            // they are spelled out because they define the "healthy set"
            // baseline these tests start from, and a reader should not
            // have to know `Degradation::default()` to see it.
            degradation: Degradation {
                confidence: Confidence::PerLine,
                observation_gap: false,
                ..Degradation::default()
            },
        }
    }

    #[test]
    fn stop_evidence_maps_the_set_onto_the_matcher_contract() {
        let set = stop_set(
            Some(XyRegion {
                x: RInterval { lo: 10.0, hi: 12.0 },
                y: RInterval { lo: 20.0, hi: 21.0 },
            }),
            Some(OffsetWindow {
                start: 100,
                end: 500,
            }),
        );
        let evidence = stop_evidence(&set, 40).unwrap();
        assert!((evidence.x.min - 10.0).abs() < 1e-12);
        assert!((evidence.y.max - 21.0).abs() < 1e-12);
        let e = evidence.e.unwrap();
        assert!((e.min - 10.0).abs() < 1e-12 && (e.max - 12.0).abs() < 1e-12);
        // Point candidate contributes one value; the ramp contributes
        // lo, hi, and midpoint.
        assert_eq!(evidence.z_candidates.len(), 4);
        assert!(evidence.z_candidates.contains(&4.2));
        // Inclusive → exclusive window conversion.
        assert_eq!(evidence.window.start, 100);
        assert_eq!(evidence.window.end, Some(501));
    }

    #[test]
    fn machine_source_resolution_with_unreachable_klippy() {
        use super::{resolve_machine_source, MachineSource};
        use crate::config::Config;
        // Uncommissioned legacy + unreachable klippy: unavailable, and
        // the reason explains why even a dry run refuses.
        let config = Config {
            klipper_socket: std::path::PathBuf::from("/nonexistent-plrd/klippy.sock"),
            ..Config::default()
        };
        let MachineSource::Unavailable { reason } = resolve_machine_source(&config) else {
            panic!("expected unavailable");
        };
        assert!(reason.contains("klippy is unreachable"), "{reason}");
        // Commissioned legacy: trusted, with the fallback note.
        let mut commissioned = config;
        commissioned.machine.probe_kind = Some("tap".to_owned());
        let MachineSource::Legacy { note } = resolve_machine_source(&commissioned) else {
            panic!("expected legacy fallback");
        };
        assert!(
            note.as_deref()
                .is_some_and(|n| n.contains("config-hash blessing")),
            "{note:?}"
        );
    }

    #[test]
    fn machine_mode_report_renders_each_mode() {
        use super::report_machine_mode;
        use crate::config::Config;
        let config = Config {
            klipper_socket: std::path::PathBuf::from("/nonexistent-plrd/klippy.sock"),
            ..Config::default()
        };
        let mut out = Vec::new();
        report_machine_mode(&config, &mut out);
        let text = String::from_utf8(out).unwrap();
        assert!(text.contains("UNDETERMINED"), "{text}");
        let mut commissioned = config;
        commissioned.machine.probe_kind = Some("tap".to_owned());
        let mut out = Vec::new();
        report_machine_mode(&commissioned, &mut out);
        let text = String::from_utf8(out).unwrap();
        assert!(text.contains("legacy [machine]"), "{text}");
        assert!(text.contains("note:"), "{text}");
    }

    #[cfg(unix)]
    #[test]
    fn machine_mode_report_renders_plr_mode() {
        use super::report_machine_mode;
        use crate::config::Config;
        use crate::plrcfg::tests as fixtures;
        let response =
            fixtures::query_result(fixtures::configfile_status(&[]), fixtures::plr_object());
        let config = Config {
            klipper_socket: fixtures::spawn_fake_klippy("mode-report", response),
            ..Config::default()
        };
        let mut out = Vec::new();
        report_machine_mode(&config, &mut out);
        let text = String::from_utf8(out).unwrap();
        assert!(text.contains("mode: [plr]"), "{text}");
        assert!(text.contains("probe_method = tap"), "{text}");
    }

    #[test]
    fn stop_evidence_without_xy_is_none_and_window_falls_back() {
        assert!(stop_evidence(&stop_set(None, None), 40).is_none());
        let set = stop_set(
            Some(XyRegion {
                x: RInterval { lo: 0.0, hi: 1.0 },
                y: RInterval { lo: 0.0, hi: 1.0 },
            }),
            None,
        );
        let evidence = stop_evidence(&set, 40).unwrap();
        assert_eq!(evidence.window.start, 40);
        assert_eq!(evidence.window.end, None);
    }
}
