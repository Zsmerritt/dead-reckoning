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
    build_layer_model, match_stop_point, select_contact_zone, ByteWindow, ContactConfig,
    ContactOutcome, Interval, MatchConfig, ModelConfig, StopEvidence,
};
use plr_reconstruct::{
    anchor_state_from_context, reconstruct, FileTail, PossibleStopSet, ReconstructInputs,
    Reconstruction, RecoveryReconstruction,
};
use plr_recovery::{
    plan_recovery, select_resume_target, validate_machine, MachineConfig, MachineRejection,
    PlanConfig, PlanInputs, PlanOutcome, ProbeConfig, ProbeKind, RecoveryPlan, ZStepper,
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
}

/// Every outcome the pipeline can reach. Only `Plan` is executable.
#[derive(Debug, Clone, PartialEq)]
pub enum PipelineOutcome {
    /// The WAL ends with a deliberate print end; nothing to recover.
    CleanShutdown,
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

    let heartbeat = match scan::load_heartbeat(&config.heartbeat_file()) {
        Ok(recovery) => Some(recovery),
        Err(reason) => {
            say(&format!("pipeline: {reason}"));
            None
        }
    };
    let receive_seq = scan::load_receive_seq(&config.receive_seq_file());

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
            let plan_config = PlanConfig {
                legacy_single_probe: true,
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
    let (model, base_offset) = match anchored_model(recovery, file_bytes, say) {
        Ok(pair) => pair,
        Err(outcome) => return outcome,
    };
    let stop_set = &recovery.stop_set;

    let Some(evidence) = stop_evidence(stop_set, base_offset) else {
        return PipelineOutcome::ManualFallback(
            "the reconstruction has no XY region; stop-point matching is impossible \
             (probe placement cannot be chosen automatically)"
                .to_owned(),
        );
    };
    let match_result = match match_stop_point(&model, &evidence, &MatchConfig::default()) {
        Ok(result) => result,
        Err(e) => return PipelineOutcome::ManualFallback(format!("stop-point match failed: {e}")),
    };
    say(&format!(
        "pipeline: match confidence {:?} ({} candidates)",
        match_result.confidence,
        match_result.candidates.len()
    ));

    let resume = match select_resume_target(&model, &match_result) {
        Ok(resume) => resume,
        Err(reason) => {
            return PipelineOutcome::ManualFallback(format!("no safe resume point: {reason:?}"))
        }
    };
    let Some(resume_layer) = resume.layer else {
        return PipelineOutcome::ManualFallback(
            "resume point has no layer attribution; contact selection impossible".to_owned(),
        );
    };
    let crash_xy = [evidence.x.midpoint(), evidence.y.midpoint()];
    let contact = match select_contact_zone(&model, resume_layer, crash_xy, contact_config) {
        Ok(outcome) => outcome,
        Err(e) => return PipelineOutcome::ManualFallback(format!("contact selection failed: {e}")),
    };
    if let ContactOutcome::Candidates(candidates) = &contact {
        say(&format!(
            "pipeline: {} probe candidate(s); best at ({:.2}, {:.2})",
            candidates.len(),
            candidates.first().map_or(f64::NAN, |c| c.point[0]),
            candidates.first().map_or(f64::NAN, |c| c.point[1]),
        ));
    }

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
    Ok(PlanBundle {
        plan,
        file_path: String::new(),
        machine: machine.clone(),
        recovery_file_content: generated.content,
        recovery_file_path,
        sdcard_root: root_path.to_path_buf(),
        recovery_source_name,
    })
}

/// Selects the anchor context and replays the file into a layer model.
/// `Err` carries the early outcome.
fn anchored_model(
    recovery: &RecoveryReconstruction,
    file_bytes: &[u8],
    say: &mut dyn FnMut(&str),
) -> Result<(plr_analyzer::LayerModel, u64), PipelineOutcome> {
    // Anchor context: the newest context at or before the offset floor
    // (its interpreter state seeds the replay); fall back to the oldest
    // context naming the file.
    let floor = recovery.stop_set.file_window.as_ref().map(|w| w.start);
    let contexts = &recovery.timeline.contexts;
    let anchor = contexts
        .iter()
        .rev()
        .find(|c| {
            c.virtual_sdcard
                .as_ref()
                .is_some_and(|v| floor.is_none_or(|f| v.file_position <= f))
        })
        .or_else(|| contexts.iter().find(|c| c.virtual_sdcard.is_some()));
    let Some(anchor) = anchor else {
        return Err(PipelineOutcome::NotPossible(
            "no WAL context carries virtual_sdcard state".to_owned(),
        ));
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
    Ok((model, base_offset))
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
        TransformObservations, VirtualSdState, WalRecord, WalWriter,
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
            virtual_sdcard: Some(VirtualSdState {
                file_path: file_path.to_owned(),
                file_position: crash_offset(),
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

    /// Builds a WAL dir + print file + config primed to reach a plan.
    pub(crate) fn fixture(tag: &str) -> (PathBuf, Config) {
        let dir = temp_dir(tag);
        let gcode_path = dir.join("part.gcode");
        std::fs::write(&gcode_path, MODEL_TEXT.as_bytes()).unwrap();
        let mut writer = WalWriter::create(Vec::new(), &SegmentHeader::new(1, 1)).unwrap();
        writer.append(&WalRecord::Heartbeat(heartbeat())).unwrap();
        // An early context (before any deposition) anchors the layer
        // model so it covers layer 0 — contact selection probes layer
        // N−1, which must exist in the modeled window.
        let mut early = crash_context(gcode_path.to_str().unwrap());
        early.mono_ns = 1_000_000_000;
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

    fn run(config: &Config) -> (PipelineOutcome, String) {
        let mut out = Vec::new();
        let outcome = run_pipeline(config, &mut out).expect("pipeline hard error");
        (outcome, String::from_utf8(out).unwrap())
    }

    #[test]
    fn full_pipeline_reaches_a_validated_plan() {
        let (_dir, config) = fixture("plan");
        let (outcome, output) = run(&config);
        let PipelineOutcome::Plan(bundle) = outcome else {
            panic!("expected a plan, got {outcome:?}\n{output}");
        };
        let plan = &bundle.plan;
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
            degradation: Degradation {
                confidence: Confidence::PerLine,
                observation_gap: false,
                extension_unavailable: false,
                extension_truncated: false,
                extension_error: false,
                unknown_z_in_extension: false,
                unknown_xy_in_extension: false,
                e_frame_shift_in_extension: false,
                offset_floor_uncertain: false,
                e_file_frames_incomplete: false,
                anchor_time_unknown: false,
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
