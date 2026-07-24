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
//! `exclude_objects` are passed empty: the WAL context format does not
//! journal exclude-object state (documented in `convert`), so restoring
//! exclusions is out of scope until the WAL format grows a field.

use std::io::Write;
use std::path::Path;

use plr_analyzer::{
    build_layer_model, match_stop_point, select_contact_zone, ByteWindow, ContactConfig,
    ContactOutcome, Interval, MatchConfig, ModelConfig, StopEvidence,
};
use plr_reconstruct::{
    anchor_state_from_context, reconstruct, FileTail, PossibleStopSet, ReconstructConfig,
    ReconstructInputs, Reconstruction, RecoveryReconstruction,
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
    let recovery = match reconstruct(&inputs, &ReconstructConfig::default()) {
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
    let source = resolve_machine_source(config);
    let type_annotations = contains_type_annotations(file_bytes);
    let (machine, plan_config, contact_config, legacy) = match &source {
        MachineSource::Unavailable { reason } => {
            return Ok(PipelineOutcome::NotPossible(format!(
                "machine configuration unavailable: {reason}"
            )))
        }
        MachineSource::Plr {
            snapshot,
            plr,
            notes,
        } => {
            say("pipeline: machine config from the [plr] section of the live Klipper config");
            say("pipeline: note: /etc/plrd.conf [machine] is ignored while [plr] exists");
            for note in notes {
                say(&format!("pipeline: note: {note}"));
            }
            let (machine, assembly_notes) =
                plrcfg::machine_from_settings(&snapshot.settings, plr, type_annotations);
            for note in assembly_notes {
                say(&format!("pipeline: note: {note}"));
            }
            let contact = ContactConfig {
                exclusion_radius: plr.exclusion_radius,
                ..ContactConfig::default()
            };
            (machine, plr.plan_config(), contact, false)
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
            (
                machine,
                PlanConfig::default(),
                ContactConfig::default(),
                true,
            )
        }
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
        &file_path,
        file_bytes,
        &mut say,
    ))
}

/// Which snapshot source `plrd recover` uses this run.
pub(crate) enum MachineSource {
    /// The Klipper config has a `[plr]` section: it is authoritative.
    Plr {
        /// The queried `configfile.settings` (+ `plr` status object).
        snapshot: KlippySnapshot,
        /// The parsed `[plr]` section.
        plr: PlrSettings,
        /// Non-fatal observations surfaced to the operator.
        notes: Vec<String>,
    },
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
/// * a commissioned legacy `[machine]` snapshot (probe_kind set) is
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
                            let reported = object.get("method").and_then(serde_json::Value::as_str);
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
                    MachineSource::Plr {
                        snapshot,
                        plr,
                        notes,
                    }
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
        MachineSource::Plr { plr, notes, .. } => {
            line("machine-config mode: [plr] (live Klipper config; /etc/plrd.conf [machine] ignored)");
            line(&format!(
                "  probe_method = {}, control_socket = {}",
                plr.probe_method, plr.control_socket
            ));
            for note in notes {
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
fn plan_from_recovery(
    recovery: &RecoveryReconstruction,
    machine: &MachineConfig,
    plan_config: &PlanConfig,
    contact_config: &ContactConfig,
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

    let reconstruction = Reconstruction::Recovery(Box::new(recovery.clone()));
    let plan_inputs = PlanInputs {
        machine,
        reconstruction: &reconstruction,
        contact: &contact,
        match_result: &match_result,
        model: &model,
        file_temps,
        exclude_objects: &[],
    };
    match plan_recovery(&plan_inputs, plan_config) {
        Ok(PlanOutcome::NoRecoveryNeeded) => PipelineOutcome::CleanShutdown,
        Ok(PlanOutcome::ManualFallback { reason }) => {
            PipelineOutcome::ManualFallback(format!("planner declined: {reason:?}"))
        }
        Ok(PlanOutcome::Plan(plan)) => PipelineOutcome::Plan(Box::new(PlanBundle {
            plan: *plan,
            file_path: file_path.to_owned(),
            machine: machine.clone(),
        })),
        Err(e) => PipelineOutcome::NotPossible(format!("planning failed: {e}")),
    }
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

    /// The synthetic two-layer print (mirrors plr-recovery's shared
    /// scenario): layer 0 at Z 0.2, layer 1 at Z 0.4, all deposition
    /// annotated internal infill.
    const MODEL_TEXT: &str = "\
G90
M83
G1 Z0.2 F7200
;TYPE:Internal infill
G1 X10 Y10 E1 F1800
G1 X30 Y10 E1
G1 X30 Y30 E1
G1 Z0.4 F7200
;TYPE:Internal infill
G1 X10 Y10 E1 F1800
G1 X30 Y10 E1
G1 X30 Y30 E1
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
        assert_eq!(plan.resume_file, "part.gcode");
        assert!(
            plan.resume_offset >= crash_offset(),
            "resume {} before crash {}",
            plan.resume_offset,
            crash_offset()
        );
        assert_eq!(plan.m26_offset(), Some(plan.resume_offset));
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
    fn plr_fixture(tag: &str, plr_overrides: &[(&str, serde_json::Value)]) -> (PathBuf, Config) {
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
        // Tap method: the probe step is a PROBE with the [plr] speed.
        let probe = bundle
            .plan
            .steps
            .iter()
            .find(|s| s.phase == plr_recovery::Phase::Probe)
            .expect("probe step");
        assert_eq!(probe.commands, vec!["PROBE PROBE_SPEED=1 SAMPLES=1"]);
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
            vec!["PLR_DRAG_PROBE CHIP=adxl345 SPEED=4 Z_STEP=0.04 SENSITIVITY=2.5"]
        );
        // The drag envelope has no speed-proportional term.
        assert_eq!(
            bundle.plan.envelope.params.overshoot,
            plr_recovery::OvershootTerm::DragStep { drag_z_step: 0.04 }
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
