# Operations

Day-2 usage of a deployed `plrd`: understanding its logs, reading scan
reports, disk expectations, the post-power-loss procedure for v1, the
drag-oracle operating notes, and troubleshooting. Companion documents:
[install](install.md) · [architecture](architecture.md) ·
[klippy_plugin/README.md](../klippy_plugin/README.md) (console command
reference) · [worked example](../examples/recovery-walkthrough.md).

## Understanding plrd's logs

The daemon logs to stderr, which systemd routes to journald:

```sh
journalctl -u plrd -f        # follow live
journalctl -u plrd -b -1     # the boot before this one (post-mortems)
```

Every line is prefixed `plrd:`. The complete catalog of messages the daemon
emits, what each means, and what (if anything) to do:

**Boot-time detection** (runs at every daemon start, before anything
else; it only writes a state file and announces — recovery starts only
when *you* run `plrd recover`):

| Message | Meaning / action |
| --- | --- |
| `unfinished print detected: <file> at byte <n> (~NN%) (<class>); run 'plrd recover'` | The previous session's WAL ends uncleanly with a print in progress. `pending_recovery.json` is (re)written in the WAL dir; the same message is announced on the printer console |
| `pending-recovery announcement delivered` | The console message landed via Moonraker (`RESPOND`, falling back to `M117`; retried for ~5 minutes while klippy comes up) |
| `could not deliver the pending-recovery announcement (gave up)` | Moonraker unreachable or both commands rejected (no `[respond]`/`[display_status]` in printer.cfg). Cosmetic only — `plrd recover` works regardless |

**Connection lifecycle** (normal operation — plrd is designed to wait for
and outlive Klipper):

| Message | Meaning / action |
| --- | --- |
| `cannot connect to <socket>: <err>; retrying in <t>` | Klipper is down or `klipper_socket` is wrong. Capped exponential backoff (250 ms → 8 s). Persistent while Klipper runs ⇒ fix the socket path ([troubleshooting](#troubleshooting)) |
| `klippy state <state>; waiting for ready` | Connected, but klippy is still starting (or in error/shutdown); plrd polls `info` until `ready` before subscribing |
| `klippy state changed to "<state>"` | The periodic info poll saw a transition. A `"shutdown"` here keeps the socket and subscriptions alive — motion just stops (the quiet-tail crash classification is built to read exactly this); a klippy `RESTART` instead drops the socket |
| `klipper session ended: <err>` | The socket dropped. If subscriptions were live, a `SocketLost` marker is journaled (immediately durable), heartbeats pause (no liveness claim without a live socket), and plrd reconnects; the first successful resubscribe journals `Resubscribed` |

**Data-quality warnings** (recording continues; the affected record is
skipped and reconstruction sees an honest gap rather than wrong data):

| Message | Meaning / action |
| --- | --- |
| `unparseable <route> payload: <err>` | A `dump_trapq`/`dump_stepper`/status notification did not match the expected shape. One-off: ignore. Every batch for one stepper: a format mismatch between plrd and your Klipper version — file a bug with the message. (This exact symptom — `invalid value: integer -40, expected u64` on every `dump_stepper` batch — was the field bug fixed by the signed-stepper change; upgrade if you see it) |
| `initial status unparseable: <err>` | The first status snapshot after subscribing failed to parse; subsequent notifications still record |
| `WAL record skipped: <err>` | A record was rejected at the writer (non-finite float from a confused Klipper, oversized payload). The log stays intact; the record is dropped loudly |
| `unclassifiable frame: <err>` / `oversized frame discarded (<n> bytes)` | Protocol garbage on the socket; the frame is skipped |
| `klipper error for request <id>: <err>` | Klipper returned an error to one of plrd's requests (e.g. a subscription to an object this printer lacks that errors instead of returning empty) |

**Startup / fatal** (systemd restarts the service on every exit,
`Restart=always`):

| Message | Meaning / action |
| --- | --- |
| `cannot read config <path>` / `line <n>: <error>` | Config missing or invalid — unknown keys, duplicate keys, and out-of-range values are hard errors by design; the line number points at the offender. Exit 1 |
| `sd_notify failed (continuing): <err>` | Not running under systemd (e.g. started by hand); harmless |
| `client stopped: <err>` / `WAL service failed: <err>` / `WAL thread panicked` | Fatal runtime failure (a WAL I/O error lands here). Exit 1 → prompt restart; investigate the underlying disk error in the surrounding journal lines |

The *durable* counterpart of the log is the WAL itself: lifecycle markers
(`SocketLost`, `Resubscribed`, `SubscriptionGap`, `CleanShutdown`) are
journal entries that survive power loss and are what reconstruction actually
consumes — see [What markers mean](#what-markers-mean). The scan report is
the tool for reading those.

## Reading `plrd scan` reports

`plrd scan --wal <dir>` is a pure reader (no durability syscalls), so it
works on any platform — copy the WAL directory off the printer and analyze
it on a laptop if the printer host is dead. An annotated real report (the
WAL behind it is generated in the [walkthrough](../examples/recovery-walkthrough.md)):

```text
plrd scan: /var/lib/plrd/wal
segment 7 (/var/lib/plrd/wal/wal-000007.plr): 16 records (trapq 8, stepper 1, context 2, marker 0, heartbeat 5)
  valid prefix ends at byte 4438: torn frame payload at end of log (expected after power loss: yes)
```

Segments are reported oldest → newest (names embed a monotonic index; each
daemon start creates a fresh one). For each: the record counts by kind, then
where and why the valid prefix ends. **`expected after power loss: yes`**
means the end shape is one power loss legitimately produces (clean EOF, torn
frame header, torn payload). Only the *newest* segment may legitimately end
torn — rotation syncs a segment before opening its successor, so a torn
earlier segment is reported with a loud `WARNING`: it indicates corruption
of previously durable data, and everything downstream deserves suspicion.

```text
heartbeat /var/lib/plrd/wal/heartbeat.bin: slot A seq 212 print_time 21.2000s wal_offset 4630
```

The winning heartbeat slot: sequence, print time, and the WAL offset at
heartbeat time. After a mid-rewrite power cut you will also see
`other slot B torn: … (expected after power loss mid-rewrite)` — that is the
dual-slot design working, not a problem.

```text
receive_seq sidecar: widened 41872 at mono 21250000000 ns
print file: /home/pi/printer_data/gcodes/two_layer_hatch.gcode (1424 bytes)
```

The sidecar observation (used only as a time bound on `t_b`), and whether
the printed file named by the newest context could be read. If it cannot,
scan still runs but reports `forward extension disabled` — see the
containment caveat below.

```text
reconstruction: RECOVERY
  crash class: host death or power loss (torn WAL tail: true)
  stop window: t_a 21.2000s .. t_b 21.9500s (t_b source: ReceiveSeq)
  window anomaly: NoMcuFrequency
```

The verdict is one of three:

- `CLEAN SHUTDOWN` — the print ended on purpose; *no recovery is needed and
  none should be attempted*.
- `RECOVERY` — an unclean stop; the possible-stop set follows.
- `not possible: <error>` — reconstruction prerequisites are missing (most
  commonly: no context snapshot because no print was active). The WAL
  prefix above is still valid evidence.

For `RECOVERY`: the crash class ([three classes](architecture.md#crash-classes)),
the stop window `[t_a, t_b]` with the source of `t_b`, and any window
anomalies. `NoMcuFrequency` is currently expected on every report (the MCU
`CLOCK_FREQ` is not journaled in v1; Klipper-converted step times are used
instead).

```text
  WAL evaluation span: 21.2000s .. 21.9500s
  file offset window: bytes 1105 .. 1424
  Z candidates: 2
    z [0.4000, 0.4000] mm  kind Plateau  provenance Wal  known true
    z [0.4000, 0.4000] mm  kind Plateau  provenance Extension  known true
  XY region: x [20.000, 40.000] mm, y [20.000, 40.000] mm
  E internal frame: [7.7375, 11.6000] mm
  E file frame: [5.4800, 11.6000] mm
  forward extension: ExtensionSummary { anchor_offset: 1276, .. }
  confidence: PerLine; degradations: Degradation { .. }
```

This is the possible-stop set:

- **Z candidates** are the exact enumeration — plateaus (`lo == hi`) and
  ramps, each with provenance (`Wal` = evaluated from durable trapq data,
  `Extension` = enumerated by the forward simulation) and a `known` flag
  (`false` means a `G28` in the extension invalidated Z knowledge; such
  candidates are excluded from the envelope-sizing span). The span across
  trusted candidates is what sizes the probe envelope.
- **file offset window** bounds where in the file the machine stopped;
  **XY region** and the two **E intervals** bound position and extrusion.
- **confidence / degradations**: `PerLine` with all flags false is the good
  case. Every `true` flag is honest evidence-quality information — the
  important ones:
  - `extension_unavailable: true` — the printed file was missing; **the
    containment guarantee is void for true power loss**. Re-run the scan on
    the printer (or copy the print file alongside) before trusting the set.
  - `observation_gap: true` — a subscription gap or socket loss overlaps
    the window (disk stall, Klipper restart); WAL candidates inside it may
    be missing and containment leans on the extension.
  - `confidence: PerLayer` — match only at layer granularity; automatic
    resume would be refused (`MatchTooCoarse`), manual recovery indicated.

## What markers mean

| Marker | Meaning | Operational significance |
| --- | --- | --- |
| `CleanShutdown` | Print ended or was cancelled on purpose | Scan reports CLEAN SHUTDOWN; never attempt recovery past one |
| `SocketLost` | Klipper's API socket dropped (e.g. `RESTART`) | Motion after this point was not observed; heartbeats pause (no liveness claim without a live socket) |
| `Resubscribed` | The daemon reconnected and re-established subscriptions | Normal after a Klipper restart |
| `SubscriptionGap {start, end}` | A known observation hole — either the reported gap between records, or a channel-overflow drop during a disk stall | Reconstruction treats it as an honest gap (`observation_gap` degradation) instead of silently missing motion |

## Disk sizing and write load

Expected write load, from the design (measuring it on your host is one of
the open E1–E5 validation tasks):

- **WAL appends**: ~1–6 KB/s during motion; a trickle when idle. A long
  24 h print is therefore ~90–520 MB of segments, worst case.
- **Heartbeat**: a 128-byte file rewritten and synced 10× per second while
  Klipper is connected — ~860 k syncs per day. This is the flash-wear item,
  not the volume item. `heartbeat_o_dsync = true` halves the syscalls, not
  the writes.
- **receive_seq sidecar**: 24 bytes, ~1 rewrite+sync per second.

Recommendations:

- Prefer putting `wal_dir` on a **dedicated partition or a cheap USB
  stick** rather than the root SD card: it isolates the sync-heavy
  heartbeat wear from the OS medium, keeps WAL growth from filling the
  root filesystem, and makes post-mortem analysis trivial (pull the stick,
  `plrd scan` it on a laptop). Any filesystem with honest fsync works
  (ext4 recommended); it must be mounted at boot before `plrd` starts. If
  you move `wal_dir`, also grant the service write access — see
  [examples/plrd.service.override.conf](../examples/plrd.service.override.conf).
- **plrd never deletes segments.** Rotation (default every 16 MiB) starts a
  new file; old ones are crash evidence and retention is deliberately the
  operator's. After a print has completed cleanly (or a recovery is fully
  resolved), old `wal-*.plr` files can be deleted freely — never delete the
  newest segment, `heartbeat.bin`, or `receive_seq.bin` while the daemon is
  running. Deleting all-but-the-newest segment after each successful print
  keeps the directory small. Stale segments also make `plrd scan` slower
  and noisier (it merges every segment it finds), so pruning helps analysis
  too.

Daemon logs go to stderr, i.e. to journald under systemd; journald's own
rotation applies (`journalctl -u plrd`). The daemon writes no log files of
its own.

## After a real power loss

The whole procedure runs from the **printer console** (Mainsail/Fluidd);
every step has an equal CLI alternative on the printer host, noted
inline.

1. **Do not home anything yet.** Do not run `G28`. On a moving-bed-Z
   machine, homing Z (or letting the idle timeout's `M84` clear state and
   then homing) can drive the bed into the nozzle or the nozzle through the
   part. Leave the printer as it is; the bed holds position because the Z
   leadscrews are self-locking (a commissioning prerequisite).
2. **Power the host and let plrd tell you.** At startup the daemon
   classifies the previous session's WAL: an unclean end with a print in
   progress writes `pending_recovery.json` into the WAL directory and
   announces on the printer console (via `RESPOND`/`M117`):
   `unfinished print detected: <file> at byte <n> (~NN%); run 'plrd
   recover'`. Detection never executes anything.
3. **Check the state**: type **`PLR_STATUS`**. The daemon half of the
   report shows the pending recovery (file, byte offset, rough %, crash
   class), WAL segment count, heartbeat age, and the machine-config
   mode + validation summary — refusal reasons show up here *before*
   you attempt anything. (CLI: `plrd scan --wal <wal_dir>` gives the
   deeper evidence report — the full possible-stop set, degradations,
   Z candidates; read it against the
   [section above](#reading-plrd-scan-reports). `plrd scan` also runs
   off-host if the printer host died.)
   - `CLEAN SHUTDOWN` → nothing to recover, you are done.
   - `not possible` or `extension_unavailable` → fix the missing input
     (usually the print file path) before trusting anything.
4. **Dry-run the recovery**: type **`PLR_RECOVER`** (CLI:
   `plrd recover --config /etc/plrd.conf`). Both run the identical
   pipeline and gate stack, print the full plan and every command that
   would be sent — and send nothing. See
   [the next section](#recovering-with-plr_recover--plrd-recover) for
   reading the plan. Typed declines (vase mode, single wall, layer-only
   match, no safe contact zone…) mean automation refuses for this part
   — that is the system being honest, not a bug; recover manually using
   the rendered plan phases as the checklist (below).
5. **Execute**: **`PLR_RECOVER EXECUTE=1 CONFIRM=YES`** — both
   arguments required verbatim, anything less refuses (CLI:
   `plrd recover --config /etc/plrd.conf --execute --confirm`, which
   additionally asks an interactive yes). The console command **returns
   immediately** and the recovery reports as it goes; it must, because a
   console command that waited would hold klippy's only thread and stall
   the heaters (see
   [the plugin README](../klippy_plugin/README.md#every-daemon-call-is-asynchronous--and-why-that-is-a-safety-property)).
   For your first recoveries set **`debug_confirm_each_step: True`** in
   `[plr]`: plrd then stops before every step and prints the exact
   commands it is about to send, and you answer
   **`PLR_RECOVER_CONTINUE`** (or `PLR_RECOVER_ABORT`) — the same pause
   the CLI's `--step` gives you, from the console. Supervise it: the
   printer must already be ready and idle, the nozzle will approach the
   part at probing temperature, and any failed verification aborts and
   leaves the printer as-is with a transcript to review.
6. **Answer whatever plrd asks.** Any *confirmable* failure stops and
   explains itself — what happened, why it matters, what to change — and
   offers to continue anyway (`PLR_RECOVER_CONTINUE`) or stop
   (`PLR_RECOVER_ABORT`). So does `confirm_z_before_resume` if you set it,
   which is the last moment a human can compare plrd's believed Z against
   the actual nozzle. Unanswered questions abort cleanly on plrd's own
   `confirm_timeout_s`; that abort invalidates the Z frame, so a fresh dry
   run is required before retrying.

### Manual fallback

When the pipeline declines — or the machine is not
[commissioned](install.md#commissioning-from-the-console) — execution
is a human job, guided by the same plan structure
([architecture](architecture.md#the-plan-is-data-the-trust-model)).
Respect the phase order and verify each step's outcome before the next
(the plan's `ok?:` lines name the exact Klipper status fields). The
non-negotiables:

- `SET_IDLE_TIMEOUT TIMEOUT=86400` **first** — the default idle timeout
  runs `M84`, and any motor-off clears ALL homed state.
- Never `G28` after declaring the shifted frame. Home **XY only**, ever.
- Probe at 1–2 mm/s, `SAMPLES=1` (with the plugin installed, prefer a
  `PLR_TOUCH` consensus — same speed rules), only after declaring the
  shifted frame
  (`SET_KINEMATIC_POSITION Z = position_min + envelope`), with the
  nozzle in the 140–160 °C band (warm enough not to plough cold plastic,
  cool enough not to ooze).
- If the probe reports **no trigger over the full envelope**, stop: the
  part was never touched and is beyond the envelope — the reconstruction
  and reality disagree, and continuing means guessing.
- If any verification fails, **abort** (heaters to safe state, motion
  stopped). The plan format has no "continue past a failure" action, and
  neither should you.

## Recovering with `PLR_RECOVER` / `plrd recover`

Console `PLR_RECOVER` and CLI `plrd recover --config <path>` are the
same machinery: the console command calls the daemon's control socket,
which runs the identical pipeline (WAL → reconstruction → stop-point
match → contact selection → validated plan) and the identical gate
stack — the plugin adds a client-side consent check on top, it replaces
nothing. The differences: the CLI's interactive TTY prompt is replaced by
the explicit `EXECUTE=1 CONFIRM=YES` arguments; the CLI's `--step` flag is
CLI-only, and the console equivalent is the `[plr]` key
`debug_confirm_each_step`, which makes plrd pause before every step and
ask over the socket (answer with `PLR_RECOVER_CONTINUE` /
`PLR_RECOVER_ABORT`); and the console command returns as soon as the
recovery has started rather than when it finishes. A complete real
transcript of everything below is in the
[walkthrough](../examples/recovery-walkthrough.md#from-evidence-to-a-plan-plrd-recover).

**Reading a dry run.** The default invocation prints, in order: pipeline
progress lines (`pipeline: stop window …`, `pipeline: match confidence …`,
`pipeline: N probe candidate(s); best at (x, y)`), the fully rendered plan
(same format as the golden fixture — envelope header, numbered steps with
`send:`/`pre:`/`ok?:`/`fail:` lines), a summary
(`recover: plan has N steps, M commands; resume <file> @ byte <n>`), and
the banner:

```text
recover: DRY RUN — nothing was sent. Re-run with --execute --confirm
         to execute after review.
```

The dry path provably cannot send anything: no network client is
constructed on it. Review at minimum: the envelope header (does the gap
match the scan's Z span?), the probe point (step 6 — is it on printed
part, away from the crash?), the resume byte offset, and any
`# warning:` lines.

**The execution gates**, in order (each verified by tests; no connection
to Moonraker exists until gate 4, and no G-code is sent until every gate
has passed):

1. dry run by default;
2. `--execute` without `--confirm` → refused (usage error);
3. interactive consent — the plan is shown, you must answer `y`;
4. Moonraker must be reachable and the printer **ready and idle**
   (`webhooks.state == ready`, `print_stats.state` ∈ standby / complete /
   cancelled / error, `virtual_sdcard.is_active == false`);
5. machine prerequisites already validated (else no plan exists at all);
6. with `--step`: a prompt before every step.

**The transcript.** Execution refuses to start unless it can create
`recovery-transcript-<unix-seconds>.jsonl` in the WAL directory. Every
command sent, every Moonraker response, every verification evaluation,
every runtime computation (the `{true_z}` substitution), every prompt,
and the final outcome is appended as one JSON line each — it is the
authoritative record of what the recovery actually did. After an abort
(`recover: ABORTED at step N [phase]: <reason>`), the printer is left
as-is; read the transcript bottom-up to see exactly which predicate
failed on which field before retrying. After
`recover: COMPLETED — N steps executed and verified.` the stale
`pending_recovery.json` is removed.

`pending_recovery.json` itself is operator UX state (file, byte offset,
rough %, crash class, detection time) — safe to read, never required by
the pipeline, cleared automatically on a clean shutdown or completed
recovery.

**What an abort leaves behind.** Every abort record (console and
transcript) carries a typed failure classification alongside the step's
abort reason: `probe-triggered-early` (the probe fired before the
descent — usually debris or a probe fault), `no-trigger` (full travel
with no contact, which also covers a `PLR_TOUCH` consensus that could
not assemble an agreeing subset), `move-out-of-range`, or `unknown`.
Before the abort is recorded, any registered **cleanup commands** run
(today: restoring the pre-clamp `max_accel` after an abort inside the
consensus-touch bracket) — each transcribed, and a cleanup failure
never masks the original reason. If the abort happened **at or after
the shifted-frame declaration**, the printer's Z frame is unknown:
`frame_invalid.json` is written next to the WAL and any further
`--execute` (CLI or console) is refused —

```text
recover: REFUSED — the printer's Z frame is unknown after an aborted recovery
(aborted at step 9 [probe]: probe-no-trigger). Re-run a dry run (plrd scan /
plrd recover without --execute) for a fresh plan before resuming.
```

— until a fresh dry run regenerates the plan and clears the marker.
That is deliberate: the stale plan's coordinates were computed for a
frame that no longer exists.

**Itinerary pre-flight.** Also note the failure mode you should *never*
see at execution time: every coordinate a plan commands is validated
before the plan is returned (travel limits, probe-site anchoring, the
shifted-frame Z), and a violation is a typed rejection listing **every**
offending coordinate at once — a plan that would move out of bounds is
refused at planning, not discovered mid-recovery. If Klipper itself
still reports `Move out of range` during execution, that is the typed
`move-out-of-range` abort above, and worth reporting as a bug with the
transcript.

## Touch consensus day-2: `PLR_TOUCH` and `PLR_PROBE_TEST`

(Tap / load-cell machines; full parameter reference in
[klippy_plugin/README.md](../klippy_plugin/README.md).)

**`PLR_TOUCH`** is the same consensus sampler recovery plans use, run
by hand at the current XY — useful to sanity-check the probe on the
actual part surface class you care about before trusting a recovery.
Captured through the plugin's own test harness (one noisy first touch,
then three agreeing):

```text
PLR_TOUCH consensus at X:120.000 Y:120.000 (want 3 touches within 0.010, window 5, budget 10, speed 1.50)
PLR_TOUCH: median 0.405100, range 0.002, min 0.404900, max 0.406200
  4 touches used of 10 (window 5, limit 0.010)
```

Read the last line: it took 4 touches to find 3 contemporaneous ones
agreeing within 10 µm (the noisy first touch fell out of the window's
best subset). Many touches used — or exhaustion — means a noisy probe
or a bad surface. Exhaustion names the failed criteria, lists every
sample, and prints a **copy-pasteable retry** with the touch budget
escalated 1.5×:

```text
PLR_TOUCH failed: could not find 3 touches within 0.010 of each other in a sliding window of 5, after 6 touches.
  samples taken: [0.420000, 0.380000, 0.440000, 0.360000, 0.460000, 0.340000]
Retry with a larger touch budget:
  PLR_TOUCH START=1 SAMPLES=3 MAX_SAMPLES=9 SAMPLE_RANGE=0.010 SPEED=1.50 RETRACT=2.00 TOUCH_ACCEL=100
```

Run the retry as printed, or take the hint and investigate the probe
instead of loosening `SAMPLE_RANGE` (its 0.015 mm cap is a refusal, not
a suggestion).

**`PLR_PROBE_TEST START=1`** is the verification tier: it runs
`SEQUENCES` (default 5) *full* consensus sequences and requires the
per-sequence **medians** to agree within `VERIFY_RANGE` (default 2× the
per-sequence `SAMPLE_RANGE`), early-exiting the moment they cannot. On
success it stages `probe_resolution` — recovery's trust radius for your
probe — for `SAVE_CONFIG`. Re-run it (and `SAVE_CONFIG`) after any
change to the probe or Z hardware; the
[staleness machinery below](#calibration-staleness-when-recovery-stops-trusting-your-numbers)
will insist anyway.

## Calibration staleness: when recovery stops trusting your numbers

Every calibration the console stages (`probe_resolution`, the
`noise_floor_*` group) is stamped with a **fingerprint of the hardware
config it was measured under** plus the plugin/Klipper versions
([architecture](architecture.md#stamped-calibrations-fingerprints-and-three-tier-validation)).
On every restart the stamps are re-checked, three ways:

- **VALID** — used normally; nothing to see.
- **LEGACY** — the value predates stamping (an older install). It still
  works, with a one-time console warning; the stamps appear when you
  next re-run the calibration + `SAVE_CONFIG`.
- **INVALID** — the fingerprint no longer matches (or the plugin was
  downgraded below the staging version). The value is **treated as
  absent everywhere**: drag commands refuse with "calibrated under a
  different hardware configuration — re-run PLR_NOISE_TEST", `PLR_SETUP`
  shows a `[FAIL]` row with the old-vs-new fingerprints, `get_status`
  reports `calibrations_valid: false`, and plrd independently reaches
  the same verdict (it recomputes the fingerprint itself), so recovery
  planning refuses too.

**"I changed my Z motors — why is recovery refusing?"** Because that is
the design: the fingerprint covers every `stepper_z*` section and the
active probe/accel-chip section, so a Z-kinematics or probe-hardware
change invalidates exactly the calibrations that depended on it. The
fix is never to edit anything by hand — re-run the calibration on the
new hardware:

```text
G28
PLR_PROBE_TEST START=1        ; tap / load_cell (probe_resolution group)
PLR_NOISE_TEST START=1        ; adxl_drag (noise-floor group; away from parts!)
PLR_DRAG_CALIBRATE START=1    ; optional: re-pick sensitivity too
SAVE_CONFIG
```

Notes on the mechanics, so nothing surprises you:

- The two groups invalidate **independently** — a new accelerometer
  invalidates the noise floor but not `probe_resolution`; new Z
  steppers invalidate both (both fingerprints cover `stepper_z*`).
- Unrelated printer.cfg churn (`[fan]`, macros, `[display]`) never
  invalidates anything, and neither does re-formatting a covered
  section (the fingerprint canonicalizes whitespace and numeric
  spelling).
- The stale option text stays in printer.cfg until the re-calibration
  overwrites it (Klipper gives plugins no way to delete an autosave
  option) — it is ignored, not honored. Do not hand-edit `cal_*` keys
  to "fix" a mismatch: the values would still be measurements of
  hardware you no longer have.
- Staging refuses entirely when the running Klipper version cannot be
  determined (nothing is written, no partial stamps) — retry once
  Klipper has fully started.

## Operating the ADXL drag oracle

Notes for `probe_method: adxl_drag` day-2 use (the command reference is
[klippy_plugin/README.md](../klippy_plugin/README.md); the design is in
[architecture](architecture.md#the-adxl-drag-oracle)). Honest status
up front: the oracle's safety bounds are tested, but its detection
quality is **bench-unvalidated (E5)** — supervise it.

**The sensitivity knob (`drag_sensitivity`, 0–100).** The knob maps to
a contact threshold as a multiplier over the measured noise floor,
log-interpolated: 0 → 8.0× (least sensitive), 50 → 4.0×, 100 → 1.5×
(most sensitive). **Wobbly or noisy machines should run low numbers** —
a higher multiplier means fewer false triggers, at the cost of needing
a firmer contact signature. Rigid, quiet machines can run high numbers
to catch fainter contact. Tune it like any tunable
(`PLR_SET PARAM=drag_sensitivity VALUE=…`, then `SAVE_CONFIG`), and
read your current headroom off the `PLR_NOISE_TEST` report — it shows
the measured peak against the threshold at your current sensitivity.

**Re-run `PLR_NOISE_TEST` after changing `drag_speed`** (or anything
mechanical: accel-chip mount, toolhead mass, belt tension). The noise
floor is measured *while moving* at the configured drag speed precisely
because stepper harmonics and frame vibration must be inside the
baseline; a floor captured at one speed does not transfer to another.
The system nags for you on the speed half: `PLR_NOISE_TEST` persists
the capture speed (`noise_floor_speed`), and a recovery plan whose
`drag_speed` strays more than 20% from it carries a
`NoiseFloorSpeedMismatch` **warning** (never a refusal) telling you to
recalibrate. Mechanical changes it cannot see — recalibrating after
those stays on you.
The test also refuses obvious foot-guns: run it with the toolhead well
away from any printed part (a pass that touches one corrupts the floor
in the dangerous direction — real contact becomes invisible).

**`trigger_z` brackets, it does not measure.** A drag probe reports the
Z of the **last clean pass**; the surface lies somewhere in
`(trigger_z − drag_z_step, trigger_z]`. That conservative endpoint is
what the recovery arithmetic consumes — overshoot is bounded by one
`drag_z_step` by construction (each pass runs at fixed Z; only the
staircase decrement can put the nozzle below the surface). Smaller
`drag_z_step` = tighter bracket and more passes; the default 0.05 mm is
a first-layer-scale bracket. Expect *bounded, by-design* nozzle contact
with the part surface — the last pass drags across it.

**Pick the sensitivity empirically: `PLR_DRAG_CALIBRATE START=1`.**
Rather than guessing a knob value, run the clear-air sweep after
`PLR_NOISE_TEST` (same session works): it finds the most sensitive
knob that produces zero false contacts — entirely at a Z where contact
is impossible (it refuses to descend; `CLEAR_Z` must be ≥ 5 mm above
the Z floor) — and stages `drag_sensitivity = accepted − MARGIN` for
`SAVE_CONFIG`. If even the least sensitive knob false-triggers, that is
diagnosis, not error: your moving noise floor is bad — re-run
`PLR_NOISE_TEST` (away from any part!) or fix the accelerometer
mounting, then retry with the printed command.

**When a drag probe aborts** (`last_drag_error` in `get_status`, error
text on the console): every abort restores the starting Z, and the
hardened aborts carry a stable `[code]` token in the error text —

| Token | What happened | What to do |
| --- | --- | --- |
| `[drag_envelope_exhausted]` | The staircase reached the travel floor with no contact | The surface is not where reconstruction expected, or sensitivity is too low to hear contact. Stop and think ("reconstruction and reality disagree" — same rule as a no-trigger probe in the [manual fallback](#manual-fallback) list); check the noise floor and sensitivity before re-running |
| `[drag_time_budget]` | Wall-clock budget (`MAX_SECONDS`, default 120 s) exhausted mid-staircase | Something is slower than it should be (huge envelope, tiny `drag_z_step`, slow captures). Re-check the envelope in the dry run; raise `MAX_SECONDS` only after understanding *why* it was slow |
| `[drag_stalled]` | Many consecutive flat clean passes with real descent and no rising signal (warns once at half budget, aborts at full) | The signal is not approaching contact at all — likely a wrong site or a stale/corrupt noise floor. Re-run `PLR_NOISE_TEST`; verify the probe site over actual part |
| `[drag_implausible_signal]` | A substantial signal (≥ 50% of threshold) *fell* monotonically across three descending passes | Physically backwards (vibration should rise approaching the part): the baseline drifted or the site is wrong. Re-run `PLR_NOISE_TEST` or move; do not chase it down |
| `invalid pass (coverage_gap: …)` | A pass's accel samples did not bracket the pass motion | A capture that starts late or ends early could hide the contact burst — the pass is refused, never trusted. Usually a transient; if persistent, check the accel chip's sample rate and comms (the other `invalid pass` reasons — `too_few_samples`, non-finite, constant signal, rate collapse — share this shape) |

Unclassifiable passes (too few samples, frozen signal, collapsed sample
rate) abort the same way — a pass is never assumed clean.

**Temperature drift.** If `[plr] noise_floor_temp_sensor` is set and
`PLR_NOISE_TEST` staged a baseline temperature, a drag probe run more
than ±15 °C away widens its threshold (+2%/°C past the band, cap +50%)
and says so on the console. Widening costs detection margin — treat the
warning as a nudge to re-run `PLR_NOISE_TEST` at the temperature you
actually print at. No sensor configured = no covariate (nothing is
guessed).

## Troubleshooting

**`recover: REFUSED — the printer's Z frame is unknown after an aborted
recovery …`.**
A previous `--execute` aborted at or after the shifted-frame
declaration, so the Z frame Klipper believes in is not one anybody
declared deliberately (`frame_invalid.json` in the WAL directory
records the step/phase/reason). This is not a fault to clear by
deleting the file: run a fresh dry run (`plrd recover` without
`--execute`, or console `PLR_RECOVER`) — a newly generated plan clears
the marker — review it, then execute. A completed recovery also clears
it. See
[what an abort leaves behind](#recovering-with-plr_recover--plrd-recover).

**A drag command refuses with `calibrated under a different hardware
configuration — re-run PLR_NOISE_TEST`, or `PLR_SETUP` shows a
fingerprint `[FAIL]` after a config change.**
The calibration-staleness defense: the persisted value was measured on
hardware whose config slice no longer matches (the report shows the
staged vs current fingerprints). Re-run the named calibration and
`SAVE_CONFIG` — the full workflow, including which changes invalidate
which group, is
[Calibration staleness](#calibration-staleness-when-recovery-stops-trusting-your-numbers).
Do not hand-edit `cal_*` keys.

**`plrd: cannot connect to <socket>: No such file or directory` (or
`Connection refused`), repeating.**
The `klipper_socket` path in `/etc/plrd.conf` does not match klippy's `-a`
argument, or Klipper is not running. Find the real path in the klipper
service definition (Moonraker installs: `~/printer_data/comms/klippy.sock`).
The retry loop (capped exponential backoff, 250 ms → 8 s) is normal
behavior, not a crash — plrd is designed to wait for Klipper.

**Klipper errors on startup with `Section 'plr' is not a valid config
section` (or `PLR_SETUP` is an unknown command).**
The `[plr]` section is in printer.cfg but klippy cannot load the plugin
— the `klippy/extras/plr` symlink is missing or dangling. Re-run
`scripts/install.sh` (it repairs the link; `--klipper <path>` if your
checkout is not `~/klipper`) or link manually:
`ln -sfn ~/dead-reckoning/klippy_plugin/plr ~/klipper/klippy/extras/plr`,
then restart Klipper. The reverse also holds: after
`scripts/uninstall.sh`, remove the `[plr]` section (and its autosaved
`#*# [plr]` block) before the next Klipper restart.

**Console: `plrd not reachable at /var/lib/plrd/plrd.sock — is the
service running? (systemctl status plrd)`.**
The plugin's `PLR_STATUS` / `PLR_RECOVER` could not connect to the
daemon's control socket. `PLR_STATUS` still prints the plugin half of
its report; the daemon half needs `plrd run` up. Check
`systemctl status plrd` and `journalctl -u plrd -n 50`. If the service
is running, the paths disagree: `control_socket` in `/etc/plrd.conf`
(where the daemon binds) must equal `control_socket` in the `[plr]`
section (where the plugin connects) — the defaults match; if you
changed one, change both and restart both sides. A related message,
`plrd did not answer '<cmd>' within <n>s`, means the daemon is up but
slow (a long pipeline run on a big WAL can push `recover_dryrun`
toward its 120 s budget on slow media — prune old segments, see
[disk sizing](#disk-sizing-and-write-load)).

**Control-socket permissions (mode 0666) — and how to tighten them.**
The daemon deliberately creates `/var/lib/plrd/plrd.sock` world-writable:
the stock install runs plrd as root while klippy — the socket's one
intended client — runs as an unprivileged user whose identity plrd
cannot know at bind time, so any same-user or fixed-group mode would
break the plugin out of the box. This is acceptable on a
single-operator printer host because the socket's mutating surface is
narrow and gated: `recover_execute` demands an explicit confirm, still
passes machine validation, the klippy-ready + printer-idle gate, and
transcript-or-refuse, and can only ever execute the deterministic
pipeline plan — there is no arbitrary-G-code or configuration surface.
On a multi-user host, tighten it: run plrd as the klippy user (systemd
`User=`), or add a drop-in with
`ExecStartPost=chgrp <group> %S/plrd/plrd.sock` + `chmod 0660` (plrd
itself keeps no group-database dependency). The full rationale lives in
`crates/plrd/src/ctrlsock.rs`.

**Klipper log shows `Closing unresponsive client`, WAL has `SocketLost` /
`Resubscribed` markers.**
Klipper disconnects clients whose socket stays blocked. plrd is built so
this should essentially never fire from its side (the socket reader never
blocks on the disk; overflow drops motion records and journals the gap),
but a severely overloaded host can still starve the reader. Check host load
and storage health; the WAL evidence of the episode is the marker pair plus
a `SubscriptionGap`, and reconstruction over that window degrades honestly
(`observation_gap`).

**`no WAL segments (wal-*.plr) found in <dir>`.**
Wrong `--wal` directory, or the daemon never ran against this directory.
Check `wal_dir` in the config and `systemctl status plrd`.

**Service fails immediately; journal shows a config error with a line
number.**
The config parser rejects unknown keys, duplicate keys, and out-of-range
values as hard errors, on purpose. The message names the line and the rule
(e.g. `line 7: unknown key 'hartbeat_hz'`).

**Scan reports a torn non-newest segment (loud WARNING).**
Rotation syncs a segment before creating its successor, so this shape means
previously-durable data was corrupted after the fact — failing storage, an
fsync-dishonest medium (e.g. WAL on a 9p/network mount), or manual
tampering. Treat every downstream number with suspicion and investigate the
medium before the next print.

**Mesh not restored after recovery / `AdaptiveMeshNotRestorable` warning.**
The WAL shows bed-mesh transforms were active but no loadable profile name
was recorded — adaptive meshes (KAMP-style, per-print) have empty profile
names and cannot be re-loaded by name. The plan deliberately continues
without the mesh (first layers of the resumed region rely on the true-Z
probe datum instead). If you want meshes restored after recovery, print
with a named, saved mesh profile. Similarly, `SkewProfileUnknown` means
skew was active but unnamed and is not restored.

**`recover: REFUSED — machine prerequisites failed` (list of checks).**
Working as designed: the machine is not (fully) commissioned. In `[plr]`
mode, fix the listed items from the console
([commissioning guide](install.md#commissioning-from-the-console) —
usually a missing `PLR_SETUP ACCEPT_SELF_LOCKING_Z=1` + `SAVE_CONFIG`,
or an uncalibrated noise floor for `adxl_drag`); `PLR_SETUP` shows the
same derivations the daemon checks. On the legacy `[machine]` path the
extra cause is printer.cfg changing since the last blessing
(`config changed since validation`): re-verify the checklist against the
*current* printer.cfg, then paste the printed `crc32c:` value into
`validated_config_hash`
([legacy guide](install.md#legacy-commissioning-the-machine-section)).

**`recover: cannot reach Moonraker: …` (after answering yes).**
The `moonraker_url` in `/etc/plrd.conf` is wrong or Moonraker is down.
Default is `ws://127.0.0.1:7125/websocket`. Note the gate order: this
connection is only attempted *after* your interactive consent — a dry run
never connects at all.

**`recover: REFUSED — klippy state is … / printer is not idle`.**
The ready-and-idle gate: klippy must report `ready`, `print_stats.state`
must be standby/complete/cancelled/error, and `virtual_sdcard` must not be
active. Clear the printer state (e.g. firmware-restart after the outage)
and retry; nothing was sent.

**No console announcement after a power loss.**
The boot announcement needs `[respond]` (primary) or `[display_status]`
(fallback) in printer.cfg, Moonraker reachable, and klippy up within the
retry window (~5 minutes). Check `journalctl -u plrd` for the
`unfinished print detected` line — detection and `plrd recover` work
regardless of announcement delivery; the console message is convenience.

**Scan (or reconstruction) from an older build rejects records from a
newer recorder's WAL.**
Version skew across the stepper-signedness fix: newer WALs can contain
negative step-chunk values (any Z lift emits negative counts) that a
pre-fix reader refuses (symptom shape: `invalid value: integer -40,
expected u64`). Older WALs remain readable by newer builds, but not the
other way around — use a `plrd` at least as new as the recorder that wrote
the WAL. See the
[format compatibility note](architecture.md#stage-1--record).

**Heartbeat reported `unrecoverable` by scan.**
Both slots failed validation — the file is not a heartbeat file (wrong
path?) or the medium corrupted both slots. Reconstruction falls back to WAL
heartbeat records (1 Hz), which widens `t_a` by up to a second; the report
says exactly what it used.

**`reconstruction: not possible: no context record in the recovered WAL`.**
No print was active during the recorded window (or the WAL is from a
freshly started daemon), so nothing anchors a file offset. Nothing to
recover.
