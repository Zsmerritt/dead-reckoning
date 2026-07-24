# Operations

Day-2 usage of a deployed `plrd`: understanding its logs, reading scan
reports, disk expectations, the post-power-loss procedure for v1, and
troubleshooting. Companion documents: [install](install.md) ·
[architecture](architecture.md) ·
[worked example](../examples/recovery-walkthrough.md).

## Understanding plrd's logs

The daemon logs to stderr, which systemd routes to journald:

```sh
journalctl -u plrd -f        # follow live
journalctl -u plrd -b -1     # the boot before this one (post-mortems)
```

Every line is prefixed `plrd:`. The complete catalog of messages the daemon
emits, what each means, and what (if anything) to do:

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

The v1 flow. Remember what v1 is: reconstruction and plan generation are
implemented and tested; **plan execution via Moonraker is a scaffold that
refuses to run**. Executing the recovery is a human job, guided by the
tooling.

1. **Do not home anything yet.** Do not run `G28`. On a moving-bed-Z
   machine, homing Z (or letting the idle timeout's `M84` clear state and
   then homing) can drive the bed into the nozzle or the nozzle through the
   part. Leave the printer as it is; the bed holds position because the Z
   leadscrews are self-locking (a commissioning prerequisite).
2. **Scan.** Power the host (or pull the WAL medium) and run
   `plrd scan --wal <wal_dir>` — on the printer host if possible, so the
   printed G-code file is readable and the forward extension can run. Read
   the report against the [section above](#reading-plrd-scan-reports).
   - `CLEAN SHUTDOWN` → nothing to recover, you are done.
   - `not possible` or `extension_unavailable` → fix the missing input
     (usually the print file path) and re-scan before trusting anything.
3. **Generate and inspect the plan.** Plan generation is a library API in
   v1 (`plr_recovery::plan_recovery`); there is no `plrd` subcommand that
   emits a plan yet. The rendered plan format — and a complete example of
   one — is in the [walkthrough](../examples/recovery-walkthrough.md#the-recovery-plan);
   the checked-in golden output
   (`crates/plr-recovery/tests/golden/normal_tap.txt`) is the source of
   truth for what a plan contains. A plan may also come back as a typed
   **manual fallback** (vase mode, single-wall part, layer-only match, no
   safe contact zone): that is the system telling you automatic-style
   recovery is not safe for this part, not a bug.
4. **Execute manually, step by step, in plan order.** The phase order and
   its verifications exist for physical reasons
   ([architecture](architecture.md#the-plan-is-data-the-trust-model));
   respect them even when working by hand, and verify each step's outcome
   before the next (the plan's `ok?:` lines tell you exactly which Klipper
   status fields to check). The non-negotiables:
   - `SET_IDLE_TIMEOUT TIMEOUT=86400` **first** — the default idle timeout
     runs `M84`, and any motor-off clears ALL homed state.
   - Never `G28` after declaring the shifted frame. Home **XY only**, ever.
   - Probe at 1–2 mm/s, `SAMPLES=1`, only after declaring the shifted frame
     (`SET_KINEMATIC_POSITION Z = position_min + envelope`), with the
     nozzle in the 140–160 °C band (warm enough not to plough cold plastic,
     cool enough not to ooze).
   - If the probe reports **no trigger over the full envelope**, stop: the
     part was never touched and is beyond the envelope — the reconstruction
     and reality disagree, and continuing means guessing.
   - If any verification fails, **abort** (heaters to safe state, motion
     stopped). The plan format has no "continue past a failure" action, and
     neither should you.

## Troubleshooting

**`plrd: cannot connect to <socket>: No such file or directory` (or
`Connection refused`), repeating.**
The `klipper_socket` path in `/etc/plrd.conf` does not match klippy's `-a`
argument, or Klipper is not running. Find the real path in the klipper
service definition (Moonraker installs: `~/printer_data/comms/klippy.sock`).
The retry loop (capped exponential backoff, 250 ms → 8 s) is normal
behavior, not a crash — plrd is designed to wait for Klipper.

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
