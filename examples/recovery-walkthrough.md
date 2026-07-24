# Recovery walkthrough: record → power cut → scan → recover

A synthetic but end-to-end honest tour of the pipeline. Everything below
was produced by the real tools in this repository:

- the WAL directories were written with `plr-wal`'s actual writer APIs
  (the same encoder the daemon uses), shaped exactly like a mid-print
  power loss — including the torn trailing frame, the dual-slot heartbeat
  file with correct slot parity, and the receive-seq sidecar — by small
  throwaway programs (not part of the repo; ~150 lines each against the
  public `plr_wal` API);
- every console block is verbatim output of a `plrd` binary built from
  this repo: `plrd scan`, the boot-time detection, and the full
  `plrd recover` flow including its refusals.

Background reading: [architecture](../docs/architecture.md) explains every
concept used here; [operations](../docs/operations.md#reading-plrd-scan-reports)
is the field guide to the report format.

## The scenario

The printer is printing `two_layer_hatch.gcode` — the checked-in fixture
`fixtures/synthetic/two_layer_hatch.gcode`: a two-layer, fully
`;TYPE:`-annotated square with a sparse-infill hatch, sliced with relative
extrusion (`M83`). Layer 1 at Z 0.2, layer 2 at Z 0.4:

```gcode
;LAYER_CHANGE
;Z:0.4
G1 E-0.8 F2100
G1 Z0.4 F7200
G1 E0.8 F2100
;TYPE:Inner wall
G1 X22 Y22 F9000
G1 X38 Y22 E0.53 F2400
...
```

While the layer-2 inner wall is executing (~40 mm/s, print time ~20–21.7 s
on the WAL's clock), power dies at print time ≈ 21.3 s, mid-append of a
trapq record. At that moment the WAL durably holds:

- 5 heartbeat records (1 Hz WAL cadence) — the heartbeat *file* is newer,
  at 10 Hz;
- 2 context snapshots: one at the layer-change retract (byte offset 1105),
  one with the processing frontier at the start of layer-2 sparse infill
  (byte offset 1276) — remember contexts record where *processing* got to,
  which runs ahead of execution;
- 8 trapq segments (toolhead + extruder queues) covering the inner wall,
  including two legs planned *ahead* of execution;
- 1 committed Z-stepper range from the 0.2 → 0.4 layer-change move —
  the newest committed Z evidence;
- a torn partial frame at the tail (the record being appended when power
  died).

## The scan

```console
$ plrd scan --wal /var/lib/plrd/wal
plrd scan: /var/lib/plrd/wal
segment 7 (/var/lib/plrd/wal/wal-000007.plr): 16 records (trapq 8, stepper 1, context 2, marker 0, heartbeat 5)
  valid prefix ends at byte 4438: torn frame payload at end of log (expected after power loss: yes)
heartbeat /var/lib/plrd/wal/heartbeat.bin: slot A seq 212 print_time 21.2000s wal_offset 4630
receive_seq sidecar: widened 41872 at mono 21250000000 ns
print file: /home/pi/printer_data/gcodes/two_layer_hatch.gcode (1424 bytes)
reconstruction: RECOVERY
  crash class: host death or power loss (torn WAL tail: true)
  stop window: t_a 21.2000s .. t_b 21.9500s (t_b source: ReceiveSeq)
  window anomaly: NoMcuFrequency
  WAL evaluation span: 21.2000s .. 21.9500s
  file offset window: bytes 1105 .. 1424
  Z candidates: 2
    z [0.4000, 0.4000] mm  kind Plateau  provenance Wal  known true
    z [0.4000, 0.4000] mm  kind Plateau  provenance Extension  known true
  XY region: x [20.000, 40.000] mm, y [20.000, 40.000] mm
  E internal frame: [7.7375, 11.6000] mm
  E file frame: [5.4800, 11.6000] mm
  forward extension: ExtensionSummary { anchor_offset: 1276, anchor_print_time: Some(21.0), horizon: 2.9499999999999993, lines_consumed: 8, resume_offset: Some(1424), stop: DurationReached }
  confidence: PerLine; degradations: Degradation { confidence: PerLine, observation_gap: false, extension_unavailable: false, extension_truncated: false, extension_error: false, unknown_z_in_extension: false, unknown_xy_in_extension: false, e_frame_shift_in_extension: false, e_file_frames_incomplete: false, offset_floor_uncertain: false, anchor_time_unknown: false }
```

Reading it:

- **The torn tail is expected.** The valid prefix ends in `torn frame
  payload` — the shape a power cut mid-append legitimately produces
  (`expected after power loss: yes`). The record being appended is lost;
  everything before it survived, CRC-verified.
- **`t_a = 21.2 s`** — the last durable heartbeat (slot A, sequence 212)
  proves the machine was alive and executing at 21.2 s. The true stop
  cannot be earlier.
- **`t_b = 21.95 s`, source `ReceiveSeq`** — the newest committed
  Z-stepper evidence ends at 19.9 s (the layer change), but the sidecar's
  acked-block observation at 21.25 s plus the 0.7 s step-generation lead
  pushes the committed-motion bound to 21.95 s. Bounds only ever widen the
  window — that is the safe direction.
- **`NoMcuFrequency`** — expected on every v1 report: the MCU `CLOCK_FREQ`
  is not journaled yet, so Klipper-converted step times were trusted.
- **Z candidates: the payoff.** Both the WAL evaluation and the forward
  extension (which simulated 8 more lines from the frontier at byte 1276,
  through the end of the little file) agree on a single trusted plateau at
  **Z = 0.4 mm**. The Z span is 0 — the probe envelope will be sized by
  sag allowance and margin alone. Had the crash landed inside a z-hop or a
  layer change, additional plateau/ramp candidates would appear here and
  widen the envelope accordingly.
- **The file offset window `[1105, 1424]`** brackets where in the file the
  machine stopped: from the newest frontier old enough that execution had
  certainly passed it, to where the simulated extension stopped consuming.
- **Nothing degraded.** `confidence: PerLine`, all flags false: evidence
  covers the window with no known holes, so downstream matching may trust
  line granularity.

### The same WAL, scanned on a laptop

`plrd scan` is cross-platform; here the WAL directory was copied to a
Windows machine where `/home/pi/printer_data/gcodes/two_layer_hatch.gcode`
does not exist:

```text
print file: /home/pi/printer_data/gcodes/two_layer_hatch.gcode unreadable (The system cannot find the path specified. (os error 3)); forward extension disabled
...
  Z candidates: 1
    z [0.4000, 0.4000] mm  kind Plateau  provenance Wal  known true
  XY region: x [22.000, 26.000] mm, y [22.000, 38.000] mm
  confidence: PerLayer; degradations: Degradation { confidence: PerLayer, ... extension_unavailable: true, e_file_frames_incomplete: true, ... }
```

Same WAL, honest downgrade: without the print file the forward extension
cannot run, so the set shrinks to WAL evidence only and the report says so
loudly (`extension_unavailable: true`, confidence drops to `PerLayer`).
**In this state the containment guarantee is void for true power loss** —
copy the print file alongside the WAL (or scan on the printer) before
trusting the set.

## From evidence to a plan: `plrd recover`

Running `plrd recover` on the scenario above ends in a **typed decline**:
the tiny fixture's lines are so short (0.28–0.66 mm of filament each) that
the E evidence window spans more than eight candidate lines, and the match
degrades to layer granularity — which the planner refuses to auto-resume:

```text
pipeline: match confidence LayerOnly { layer: 0 } (11 candidates)
recover: automation declined — no safe resume point: MatchTooCoarse { layer: 0 }
  Manual recovery is required; the report above is the evidence.
```

That refusal is the design working ([honest degradation](../docs/architecture.md#the-guarantee-and-its-honest-caveats)):
ambiguity never silently collapses into a guess. So the rest of this
walkthrough uses a second synthetic scenario with realistic part geometry
— `hatch_xl.gcode`, the same two-layer crossing-hatch design scaled to an
80 mm square (2–5 mm of filament per line), crashed mid-way through
layer 2's big sparse-infill diagonal. Same generator approach, same torn
tail, heartbeat at `t_a = 21.2 s`, committed boundary `t_b = 21.95 s`.

### Boot detection

The daemon notices at its next start, before connecting to anything:

```text
plrd: unfinished print detected: /home/pi/printer_data/gcodes/hatch_xl.gcode at byte 943 (~83%) (HostDeathOrPowerLoss { torn_tail: true }); run `plrd recover`
```

The same message is announced on the printer console (via `RESPOND`,
falling back to `M117`), and `pending_recovery.json` appears in the WAL
directory:

```json
{
  "detected_wall_ns": 1784867018379782816,
  "file": "/home/pi/printer_data/gcodes/hatch_xl.gcode",
  "file_position": 943,
  "file_size": 1141,
  "percent": 82.64680105170903,
  "crash_class": "HostDeathOrPowerLoss { torn_tail: true }"
}
```

### The commissioning gate, first

With a filled-in `[machine]` section but no blessing yet, recovery
refuses and prints the computed printer.cfg checksum:

```text
pipeline: stop window t_a 21.200s .. t_b 21.950s, class HostDeathOrPowerLoss { torn_tail: true }
pipeline: machine hash computed: crc32c:658a94bb
recover: REFUSED — machine prerequisites failed:
  - machine prerequisites have never been validated
  Fix the machine/config, set machine.validated_config_hash to the
  computed hash printed above once re-validated, and retry.
```

After re-walking the [commissioning checklist](../docs/install.md#commissioning-checklist)
and pasting `validated_config_hash = crc32c:658a94bb` into the config
(the [blessing flow](../docs/install.md#commissioning-the-machine-section)),
recovery proceeds.

### The dry run

`plrd recover --config /etc/plrd.conf` — dry run is the default, and the
dry path never constructs a network client (the `moonraker_url` in this
test config points at a port with nothing listening, on purpose):

```console
$ plrd recover --config /tmp/plrd-recover2.conf
pipeline: 21 WAL records, tail: torn frame payload at end of log
pipeline: print file /home/pi/printer_data/gcodes/hatch_xl.gcode (1141 bytes)
pipeline: stop window t_a 21.200s .. t_b 21.950s, class HostDeathOrPowerLoss { torn_tail: true }
pipeline: machine prerequisites validated
pipeline: layer model from byte 0: 2 layers
pipeline: match confidence AmbiguousWindow { offsets: [918, 943, 961, 986, 1004, 1046, 1063, 1087] } (8 candidates)
pipeline: 5 probe candidate(s); best at (108.00, 132.00)
# dead-reckoning recovery plan
# resume: hatch_xl.gcode @ byte 1087
# envelope: gap 0.2 + 0.15 x speed 1 + margin 0.5 = 0.85 mm
# shifted frame: Z declared 0.85 above position_min -2
# warning: ResumeNotOnInfill
 1. [idle-timeout] disarm the idle timeout FIRST (its default M84 would clear all homed state)
      send: SET_IDLE_TIMEOUT TIMEOUT=86400
      ok?:  idle_timeout.idle_timeout within 0.5 of 86400
      fail: abort (idle-timeout-not-applied)
 2. [stepper-enable] energize the Z steppers (enabling never touches homed state; there is no M17)
      send: SET_STEPPER_ENABLE STEPPER=stepper_z ENABLE=1
      ok?:  stepper_enable.steppers.stepper_z is true
      fail: abort (stepper-enable-failed)
 3. [preheat] bed to target; nozzle to the warm-but-below-ooze probing band
      send: M140 S60
      send: M104 S150
      ok?:  heater_bed.temperature >= 57
      ok?:  extruder.temperature in [140, 160] C
      fail: abort (preheat-failed)
 4. [home-xy] home XY only (never bare G28, never Z: the bed rises into a fixed gantry)
      send: G28 X Y
      ok?:  toolhead.homed_axes contains "x"
      ok?:  toolhead.homed_axes contains "y"
      fail: abort (homing-failed)
 5. [shifted-frame] declare the shifted frame: Klipper's rail limit now structurally bounds the descent
      send: SET_KINEMATIC_POSITION Z=-1.15
      ok?:  toolhead.homed_axes contains "z"
      ok?:  toolhead.position.2 within 0.05 of -1.15
      fail: abort (shifted-frame-not-declared)
 6. [probe-approach] XY travel to the selected contact point (no Z motion)
      send: G90
      send: G0 X108 Y132 F6000
      ok?:  toolhead.position.0 within 0.25 of 108
      ok?:  toolhead.position.1 within 0.25 of 132
      fail: abort (approach-failed)
 7. [probe] single-sample probe (SAMPLES=1: the toolhead rests exactly at the halt position)
      pre:  extruder.temperature in [140, 160] C
      pre:  toolhead.homed_axes contains "x"
      pre:  toolhead.homed_axes contains "y"
      pre:  toolhead.homed_axes contains "z"
      send: PROBE PROBE_SPEED=1 SAMPLES=1
      ok?:  probe.last_z_result present and finite
      fail: abort (probe-no-trigger)
 8. [true-z-declare] true-Z arithmetic and kinematic re-declaration (never a gcode offset)
      send: SET_KINEMATIC_POSITION Z={true_z}
      ok?:  toolhead.position.2 within 0.05 of the computed true Z
      fail: abort (true-z-declare-failed)
 9. [final-declare] final true-frame declaration after all transforms are in place
      send: SET_KINEMATIC_POSITION Z={true_z}
      ok?:  toolhead.homed_axes contains "z"
      ok?:  toolhead.position.2 within 0.05 of the computed true Z
      fail: abort (final-declare-failed)
10. [restore-frame] lift off the part, then replay offsets, factors, skew, print temperatures, fans, feedrate
      send: G91
      send: G1 Z1 F1200
      send: G90
      send: SET_GCODE_OFFSET X=0 Y=0 Z=0
      send: M220 S100
      send: M221 S100
      send: M104 S210
      send: M140 S60
      send: M106 S128
      send: G1 F3000
      ok?:  gcode_move.speed_factor within 0.01 of 1
      ok?:  gcode_move.extrude_factor within 0.01 of 1
      ok?:  extruder.temperature in [207, 213] C
      ok?:  heater_bed.temperature >= 57
      fail: abort (restore-failed)
11. [entry] enter from above the part interior, speed-limited; prime; final E frame and modes
      send: G90
      send: M83
      send: G0 Z1.4 F1200
      send: G0 X160 Y80 F1200
      send: G1 Z0.4 F1200
      send: G1 E0.4 F1800
      send: G92 E81.44
      send: M83
      send: G90
      send: G1 F3000
      ok?:  toolhead.position.0 within 0.25 of 160
      ok?:  toolhead.position.1 within 0.25 of 80
      fail: abort (entry-failed)
12. [file-select] select the file (top level only), restore exclude-object state, seek to the line boundary
      send: M23 hatch_xl.gcode
      send: M26 S1087
      ok?:  virtual_sdcard.file_path equals "/home/pi/printer_data/gcodes/hatch_xl.gcode"
      ok?:  virtual_sdcard.file_position within 0.5 of 1087
      fail: abort (file-select-failed)
13. [resume-start] start playback
      send: M24
      ok?:  virtual_sdcard.is_active is true
      ok?:  idle_timeout.state equals "Printing"
      fail: abort (resume-start-failed)

recover: plan has 13 steps, 34 commands; resume hatch_xl.gcode @ byte 1087
recover: DRY RUN — nothing was sent. Re-run with --execute --confirm
         to execute after review.
```

What to notice, mapped to the [trust model](../docs/architecture.md#the-plan-is-data-the-trust-model):

- **The ambiguity resolved honestly.** Eight candidate lines fit the
  evidence; skip-forward is the conservative direction (resuming earlier
  would double-extrude over printed geometry), so the resume target is
  the **latest** offset (1087 — the start of layer-2's outer wall), and
  the plan says so in a warning: `ResumeNotOnInfill` (the seam of the
  resume will land on a wall, not hidden in infill).
- **The probe point (108, 132)** is the midpoint of a layer-1 sparse
  infill diagonal — plastic that exists, that layer 2 will rebury, well
  away from the crash region. Five ranked candidates were found.
- **Step 1 before everything**: the idle timeout's default `M84` would
  clear all homed state; a later naive `G28` would crash the bed into
  the nozzle. Machine-checked plan invariants (`idle_timeout_first`,
  `no_g28_after_shifted_declare`, …) enforce the ordering.
- **Step 5 is the safety centerpiece**: after
  `SET_KINEMATIC_POSITION Z=-1.15`, the probe descent toward
  `position_min = -2` is bounded by Klipper's own rail-limit checking —
  0.85 mm of envelope-sized travel, probe trusted for measurement only.
  The envelope header shows the arithmetic: Z span 0 (single trusted
  plateau) + sag 0.2 + 0.15 × 1 mm/s + margin 0.5.
- **`{true_z}` is the only placeholder**: the executor computes
  `true_Z = z_prev_top + (halt − trigger)` from the live probe result and
  substitutes it; a non-finite result aborts, never substitutes. This
  Tap-probe plan reads the raw trigger from `probe.last_z_result`; a
  `[load_cell_probe]` plan differs in exactly one principled way — it
  reads `probe.last_probe_position[2]` (`bed_z`) and adds the configured
  `z_offset` back.
- **Step 10 lifts off the part before heating to print temperature** —
  the nozzle must not dwell pressed into layer-1 plastic while the
  temperature verification polls.
- The rendered format's source of truth is the golden test output,
  `crates/plr-recovery/tests/golden/normal_tap.txt`.

### The execution gates

Each gate refuses loudly, and nothing is sent until all of them pass:

```console
$ plrd recover --config ... --execute
recover: REFUSED — --execute requires --confirm.

$ plrd recover --config ... --execute --confirm     # then answer: n
recover: about to EXECUTE the plan above on the printer.
Execute this recovery on the printer? [y/N] recover: declined by operator; nothing was sent.

$ plrd recover --config ... --execute --confirm     # then answer: y
recover: about to EXECUTE the plan above on the printer.
Execute this recovery on the printer? [y/N] recover: cannot reach Moonraker: moonraker connection: IO error: Connection refused (os error 111)
```

(The last line is this walkthrough's deliberately dead `moonraker_url`
being contacted **only after** consent — on a real printer the connection
succeeds, the ready-and-idle gate runs, execution writes its JSONL
transcript into the WAL directory, and any failed verification aborts
with a typed reason. See
[operations](../docs/operations.md#recovering-with-plrd-recover).)

## Reproducing

- The WAL generators are intentionally not in the repo (they fake daemon
  output; the daemon's own output is covered by the crash-consistency
  test). Any ~150-line program against `plr_wal`'s public API —
  `WalWriter`, `encode_slot`, the documented 24-byte sidecar layout — can
  produce an equivalent directory; truncate the final frame to simulate
  the torn tail.
- The `plrd recover` outputs used a config whose `[machine]` section was
  commissioned exactly as
  [install.md](../docs/install.md#commissioning-the-machine-section)
  describes, against a minimal fake printer.cfg.
- The rendered-plan format regenerates via the golden test:
  `cargo test -p plr-recovery --test golden` (set `PLR_BLESS=1` to
  re-bless after intentional changes).
