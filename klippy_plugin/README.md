# klippy_plugin — Klipper console plugin for dead-reckoning

Klipper (klippy) extras plugin for dead-reckoning power-loss recovery.
All user interaction happens through `PLR_*` g-code console commands
registered by a `[plr]` config section; the heavy lifting (journaling,
reconstruction, recovery planning, motion) stays in the Rust daemon
(`plrd`), reached over its control socket.

## Install

The plugin ships as a python package that klippy loads like any other
extras module. `scripts/install.sh` (repo root) creates the symlink
for you; manually it is:

```sh
ln -sfn /path/to/dead-reckoning/klippy_plugin/plr ~/klipper/klippy/extras/plr
sudo systemctl restart klipper
```

Then add a `[plr]` section to `printer.cfg` (all options below have
defaults; an empty `[plr]` section is valid — a commented starter block
is `examples/printer-plr-section.cfg` at the repo root) and
install/enable the `plrd` service (see `docs/install.md` at the repo
root).

## `[plr]` reference

```ini
[plr]
probe_method: tap
#accel_chip:
#wal_dir: /var/lib/plrd/wal
#control_socket: /var/lib/plrd/plrd.sock
#probe_speed: 1.5
#envelope_margin: 0.5
#sag_allowance: 0.2
#drag_speed: 20.0
#drag_z_step: 0.05
#drag_sensitivity: 30.0
#exclusion_radius: 5.0
#entry_feedrate: 1800.0
```

| option | default | range | meaning |
| --- | --- | --- | --- |
| `probe_method` | `tap` | `tap` \| `load_cell` \| `adxl_drag` | Which contact oracle recovery uses: `tap` needs a `[probe]` section, `load_cell` a `[load_cell_probe]` section, `adxl_drag` an accel chip (`accel_chip`). |
| `accel_chip` | none | section name | Accel chip section, e.g. `adxl345` or `adxl345 bed`. Required if (and only if) `probe_method: adxl_drag`. |
| `noise_floor_temp_sensor` | none | sensor name | Optional klippy sensor object (e.g. `temperature_sensor chamber`) to read the current temperature from for the drag oracle's temperature covariate. When unset the covariate is skipped (no guessing). |
| `wal_dir` | `/var/lib/plrd/wal` | path | plrd's WAL directory; the plugin reads `<wal_dir>/heartbeat.bin` as a recorder liveness hint. |
| `control_socket` | `/var/lib/plrd/plrd.sock` | path | plrd's UNIX control socket for `PLR_STATUS`/`PLR_RECOVER`. |
| `probe_speed` | `1.5` | `[1.0, 2.0]` mm/s | Recovery probe descent speed. |
| `envelope_margin` | `0.5` | `>= 0` mm | Extra clearance around the reconstructed part envelope. |
| `sag_allowance` | `0.2` | `>= 0` mm | Expected unpowered Z sag budget when matching the stop point. |
| `drag_speed` | `20.0` | `(0, 100]` mm/s | Lateral speed of the drag-oracle pass. |
| `drag_z_step` | `0.05` | `(0, 0.2]` mm | Z staircase decrement between drag passes. |
| `drag_sensitivity` | `30.0` | `[0, 100]` | Drag threshold multiplier over the measured noise floor. |
| `exclusion_radius` | `5.0` | `>= 0` mm | Radius around the probed contact kept out of the resume path. |
| `entry_feedrate` | `1800.0` | `(0, 1800]` mm/min | Feedrate cap for the re-entry approach move. |

These keys are a **fixed schema** shared with `plrd` — renaming one is a
cross-repo breaking change.

Values the plugin persists itself (never hand-edit; they live in the
`SAVE_CONFIG` autosave block under `[plr]`): `self_locking_z` (operator
attestation staged by `PLR_SETUP ACCEPT_SELF_LOCKING_Z=1`),
`probe_resolution` (measured by `PLR_PROBE_TEST`),
`noise_floor_rms` / `noise_floor_still_rms` / `noise_floor_peak` /
`noise_floor_speed` / `noise_floor_temp` (measured by `PLR_NOISE_TEST`),
and `drag_sensitivity` (staged by `PLR_DRAG_CALIBRATE`).

## Command reference

### `PLR_SETUP [ACCEPT_SELF_LOCKING_Z=1]`

Commissioning report: every automated check with `[PASS]`/`[WARN]`/
`[FAIL]` markers and remediation hints —

- `[force_move]` present with `enable_force_move` (recovery needs
  `SET_KINEMATIC_POSITION`);
- exactly one probe section matching `probe_method`;
- every `stepper_z*` section's pins on the primary MCU;
- probe `activate_gcode`/`deactivate_gcode` empty (verified from the
  config, not attested);
- a finite lower Z bound (`[stepper_z] position_min`, falling back to
  `[printer] minimum_z_position`);
- accel chip sections detected (informational);
- recorder heartbeat file fresh (**liveness hint only** — durability is
  proven by plrd from the WAL at recovery time).

The one thing software cannot check is whether your Z axis holds
position unpowered (leadscrew printers generally do; belted-Z printers
generally do not). If — and only if — yours does:

```
PLR_SETUP ACCEPT_SELF_LOCKING_Z=1
SAVE_CONFIG
```

With every check green and the attestation saved, `PLR_SETUP` reports
`COMMISSIONED`.

### `PLR_SET [PARAM=<name> VALUE=<v>]`

Runtime tunables. With no arguments, lists every tunable with its live
value, valid range, and an `[awaiting SAVE_CONFIG]` marker for values
changed this session. With `PARAM=`/`VALUE=`, validates against the
schema range, applies immediately, and stages the value for
`SAVE_CONFIG`:

```
PLR_SET PARAM=probe_speed VALUE=1.8
SAVE_CONFIG        ; optional, persists across restarts
```

Unknown names and out-of-range values are refused with the valid list /
range in the error.

### `PLR_TOUCH [SAMPLES=3] [MAX_SAMPLES=10] [SAMPLE_RANGE=0.010] [SPEED=] [RETRACT=2.0] [TOUCH_ACCEL=100]`

One **sliding-window consensus touch** at the current XY (probe_method
`tap`/`load_cell` only). Ported from Cartographer3D's Survey Touch: the
command touches the bed one descent at a time and, once it has at least
`SAMPLES` touches, searches only the most recent `SAMPLES + 2` touches
(the sliding window) for `SAMPLES` touches that agree within
`SAMPLE_RANGE`. It reports the **median** of that agreeing subset as the
trigger height and returns as soon as a window agrees.

The sliding window is deliberate: it stops a consensus being assembled
from touches taken at unrelated moments across a noisy run — the
agreeing touches must be contemporaneous. Each touch is wrapped in three
safety invariants: a **retract-before-arm** (a descent never begins
below `RETRACT`, default 2.0 mm, min 1.0), an **accel clamp** to
`TOUCH_ACCEL` (default 100, range 50–1000 mm/s²) restored after the
descent on every path, and a **retract-after-trigger** back to a safe
standoff.

- `SAMPLES` (≥3), `MAX_SAMPLES` (the touch budget; `SAMPLES`–20), and
  the window `SAMPLES + 2` is capped at 10.
- `SAMPLE_RANGE` has a **hard cap of 0.015 mm** — a larger value is
  refused, not clamped (a consensus looser than that is not worth
  trusting for recovery).
- `SPEED` is the descent speed (defaults to the `[plr]` `probe_speed`).

**This command moves the toolhead.** It refuses while a print is active,
when unhomed, and for `probe_method: adxl_drag` (use `PLR_DRAG_PROBE`).
On success the result surfaces in `get_status` as
`last_touch_result: {median_z, range, samples_used, touches}` for plrd.
If the touch budget is exhausted without a consensus, the error names
the criteria that failed and prints a copy-pasteable retry with
`MAX_SAMPLES` escalated 1.5×.

### `PLR_PROBE_TEST [SEQUENCES=5] [VERIFY_RANGE=] START=1` (plus the `PLR_TOUCH` parameters)

Probe repeatability **verification** (probe_method `tap`/`load_cell`
only). Where the old command ran N single probes, it now runs
`SEQUENCES` full consensus touch sequences (each exactly what
`PLR_TOUCH` does, so it accepts all the `PLR_TOUCH` parameters) and
checks that their per-sequence medians agree:

- acceptance = range of the sequence medians ≤ `VERIFY_RANGE` (default
  2× `SAMPLE_RANGE`, capped at 4×; a `VERIFY_RANGE` below `SAMPLE_RANGE`
  or above 4× is refused loudly);
- it **early-exits** the moment the running median range exceeds the
  limit — no point taking more sequences once inconsistent;
- `SEQUENCES` accepts 3–10.

**This command moves the toolhead** (repeated descents at the current
XY): without `START=1` it only prints what it would do; it also refuses
while a print is active or the printer is not fully homed.

On success it prints a `PROBE_ACCURACY`-style block and stages
`probe_resolution = max(2*stddev_of_medians, median_range/2, 0.005)` for
`SAVE_CONFIG` — plrd uses this as the trust radius for recovery probing.
On failure (medians disagree, or a sequence cannot reach consensus at
all) it prints a copy-pasteable retry with `SEQUENCES` escalated and
`VERIFY_RANGE` loosened (still capped).

### `PLR_STATUS`

Plugin-side state (probe method, attestation, probe resolution, live
tunables with pending-save markers) plus the daemon's own `status`
report over the control socket. If plrd is unreachable the plugin
state still prints, with a clear hint
(`systemctl status plrd`).

### `PLR_RECOVER [EXECUTE=1 CONFIRM=YES] [STEP=1]`

Power-loss recovery, driven by plrd:

- `PLR_RECOVER` — **dry run** (default): plrd validates the machine
  and prints the full plan; no motion.
- `PLR_RECOVER EXECUTE=1 CONFIRM=YES` — execute. Both arguments are
  required verbatim; anything less refuses client-side. plrd still
  enforces every gate server-side (machine validation, klippy
  ready+idle, transcript), so the console consent is additive.
- `STEP=1` is accepted syntax but the daemon **refuses** it over the
  control socket (`per-step mode is CLI-only`): the one-shot socket
  protocol has no multi-round confirmation dialogue. For a step-by-step
  recovery use the CLI: `plrd recover --execute --confirm --step`.

### `PLR_NOISE_TEST [CHIP=] [SPEED=] [DURATION=2.0] START=1`

Measures the accelerometer noise floor the drag oracle thresholds
against. Two captures: ~2 s standing still, then four no-contact
lateral passes at the drag `SPEED` at the current Z (the same pass
geometry `PLR_DRAG_PROBE` uses). Stages four keys for `SAVE_CONFIG`:

- `noise_floor_rms` — the **moving** capture's RMS. This is the
  reference the threshold is built from, deliberately not the still
  RMS: drag passes classify samples taken *while moving*, so stepper
  harmonics and frame vibration must be inside the baseline or every
  pass would false-trigger on the machine's own motion.
- `noise_floor_still_rms` — diagnostics; a large moving/still gap hints
  at a loose accel mount or a resonant frame.
- `noise_floor_peak` — the moving capture's max windowed RMS, the exact
  statistic the classifier thresholds, so the report can show your
  headroom at the current sensitivity.
- `noise_floor_speed` — the `SPEED` the moving baseline was captured
  at. The noise floor is speed-specific, so recovery plans **warn**
  (never refuse) when their `drag_speed` strays more than 20% from this
  calibration speed — the nudge to re-run `PLR_NOISE_TEST`.
- `noise_floor_temp` — the temperature (°C) the baseline was captured
  at, staged **only** when `noise_floor_temp_sensor` is configured and
  readable. `PLR_DRAG_PROBE` uses it to widen (never narrow) its
  threshold if the machine has since drifted (see below). No sensor, no
  reading → nothing staged and the covariate is silently skipped.

**This command moves the toolhead** — without `START=1` it only prints
the plan. It refuses while printing or unhomed.

> **Warning:** run it with the toolhead well away from any printed
> part. The command cannot know where parts are; a pass that touches
> one corrupts the noise floor (it would read as "normal", making real
> contact invisible).

### `PLR_DRAG_PROBE [CHIP=] [SPEED=] [Z_STEP=] [SENSITIVITY=] [PASS_LENGTH=8] [MAX_SECONDS=120] [STALL_PASSES=8]`

Locates the top of a solidified part with the accelerometer: the drag
oracle for `probe_method: adxl_drag`. Arguments default to the `[plr]`
tunables; `CHIP` accepts quoted spaced names (`CHIP="adxl345 bed"`).
Like Klipper's own `PROBE`, this command is a primitive and runs when
typed — no `START=` consent parameter (the scripted multi-move
diagnostics keep theirs). `MAX_SECONDS` (range `[30, 600]`) and
`STALL_PASSES` (`>= 2`) are optional hardening bounds (see below);
omitting them leaves the frozen `plrd` invocation contract unchanged.

How it works — **staircase with between-pass classification**, because
accelerometer data arrives in batches and there is no real-time halt:

1. Every lateral pass (default 8 mm back-and-forth, centered on the
   current XY) runs at a **fixed Z** — a pass physically cannot
   descend.
2. After each pass, the complete sample window is classified: contact
   iff its peak windowed RMS exceeds
   `noise_floor_rms x multiplier(sensitivity)`.
3. Clean pass → descend exactly `drag_z_step` and repeat. Contact →
   stop, lift `2 x drag_z_step` (never above the starting height).
4. Hard bounds computed **up front**: at most
   `ceil(available_travel / drag_z_step)` passes, where the travel
   floor is the kinematic Z limit plus one `drag_z_step` of reserve. No
   descent is ever commanded below the floor; hitting the bound with no
   contact aborts, restores the starting Z, and reports an error.
5. A pass that cannot be classified (too few samples, non-finite
   values, frozen signal, collapsed sample rate, or a **coverage gap** —
   see below) **aborts** the probe — it is never assumed clean.

**Staircase hardening** (typed aborts; each restores the starting Z and
sets `last_drag_error` with a `[code]` token):

- **Coverage bracketing** (`coverage_gap`): each pass's sample window
  must span the pass motion — the first sample no later than the motion
  start (plus a grace) and the last no earlier than the motion end
  (minus a grace). A batch that starts late or ends short could miss the
  contact burst, so the pass is refused rather than trusted.
- **Impossible signal** (`drag_implausible_signal`): if the
  ratio-to-threshold falls monotonically over **3 consecutive**
  descending clean passes while the first is already ≥ 50% of threshold,
  the probe refuses. Vibration should *rise* as the nozzle nears the
  part; a substantial signal that recedes as we descend means the noise
  floor drifted or the site is wrong — re-run `PLR_NOISE_TEST` or move.
- **Three independent bounds:** the up-front iteration bound
  (`drag_envelope_exhausted`), a **wall-clock budget** `MAX_SECONDS`
  (`drag_time_budget`), and a **no-progress stall** detector
  (`drag_stalled`). The stall detector watches for `STALL_PASSES`
  consecutive flat clean passes (ratio moving < 5% of threshold, no
  upward trend) that have descended ≥ `8 × drag_z_step` with the signal
  never approaching contact: it **warns once** at half the budget and
  aborts at the full budget, with the wall-clock budget as the outer
  hard cap.
- **Temperature covariate:** when `noise_floor_temp_sensor` is
  configured and `noise_floor_temp` was staged, and the current
  temperature deviates more than **15 °C** from it, the effective
  threshold is **widened** by +2% per °C beyond the band (capped at
  +50%) and a one-time warning is printed. It only ever *widens* (a
  hotter/colder machine is noisier), never narrows, so it can never
  manufacture a false contact. No sensor, no staged temperature, or a
  within-band reading → exactly the prior behavior.

`trigger_z` semantics: the reported height is the Z of the **last
clean pass**. The surface lies within `(trigger_z - drag_z_step,
trigger_z]`; reporting the conservative endpoint means the executor's
true-Z arithmetic can treat overshoot as bounded by one `Z_STEP`.
Results surface in `get_status` as
`last_drag_result: {trigger_z, passes, confidence}` (toolhead-frame Z);
failures null it and set `last_drag_error`.

Sensitivity guidance (`drag_sensitivity`, 0–100): the knob maps to a
threshold multiplier over the noise floor, log-interpolated between
anchors — 0 → 8.0x (least sensitive), 50 → 4.0x, 100 → 1.5x (most
sensitive). **Wobbly or noisy machines should run low numbers**: a
higher multiplier means fewer false triggers at the cost of needing a
firmer contact signature. Rigid, quiet machines can run high numbers to
catch fainter contact.

The intended flow:

```
G28                                  ; home
; position the head well AWAY from any part, at a safe Z
PLR_NOISE_TEST START=1               ; measure the baseline
PLR_DRAG_CALIBRATE START=1           ; (optional) pick drag_sensitivity
SAVE_CONFIG                          ; persist noise_floor_* + sensitivity (restarts)
; ... after a power loss, plrd's plan issues:
PLR_DRAG_PROBE                       ; or with CHIP=/SPEED=/Z_STEP=/SENSITIVITY=
```

**Honest limitations:**

- Batched accelerometer data means a *staircase*, not a continuous
  descent: the surface is only ever bracketed to within one
  `drag_z_step`, and each pass drags laterally at the Z where the
  previous pass was clean — expect (bounded) nozzle contact with the
  part surface by design.
- The noise floor is measured at one place and speed; classification
  compares against it wherever the probe runs. Big changes in speed or
  location can shift the real baseline — re-run `PLR_NOISE_TEST` after
  changing `drag_speed` or mechanics.
- Bench validation on real hardware is still pending (E5); until then
  treat detection quality (not the safety bounds) as unproven.

### `PLR_DRAG_CALIBRATE [CHIP=] [SPEED=] [CLEAR_Z=] [SCREEN_PASSES=3] [VERIFY_PASSES=6] [MARGIN=5] START=1`

Finds the **most sensitive** `drag_sensitivity` knob that never
false-triggers, running **entirely at a Z where contact is impossible**
— so an over-sensitive candidate fails *safely*, as a false contact in
clear air, never as a crash into a part. Requires `PLR_NOISE_TEST` first
(it classifies against the real noise floor) and `START=1` consent;
without `START=1` it prints the plan and moves nothing.

- `CLEAR_Z` (default: the current Z) must be **≥ 5 mm above** the
  kinematic Z floor, and the command **refuses to descend** to reach it:
  every pass runs at `CLEAR_Z` exactly. The only Z move it ever makes is
  an upward lift to `CLEAR_Z`.
- **Sensitive-first sweep** (ported from Cartographer's touch
  calibration): start at the most sensitive knob (100) and step *down*
  only as far as needed. At each candidate it runs `SCREEN_PASSES`
  screening passes — any contact in clear air is a false trigger — and
  steps the knob down by an adaptive amount (20% of the knob when
  failing badly, 10% when close, clamped to `[2, 15]` knob units). A
  screening survivor then runs `VERIFY_PASSES` passes that must **all**
  be contact-free (early-exiting on the first false contact). The
  **first (highest) surviving knob** is accepted.
- It stages `drag_sensitivity = accepted − MARGIN` (floored at 0) for
  `SAVE_CONFIG` and reports the headroom (threshold vs the worst
  clear-air peak).
- If even the least sensitive knob false-triggers, that is **not an
  error**: it prints a copy-pasteable retry and a hint to re-run
  `PLR_NOISE_TEST` or check the accel mounting.

## Commissioning in 5 console commands

```
PLR_SETUP                          ; 1. read the report, fix any [FAIL]
PLR_SETUP ACCEPT_SELF_LOCKING_Z=1  ; 2. attest self-locking Z (leadscrews!)
G28                                ; 3. home
PLR_PROBE_TEST START=1             ; 4. measure probe repeatability
SAVE_CONFIG                        ; 5. persist attestation + resolution
```

After the restart, `PLR_SETUP` should print `COMMISSIONED` and
`PLR_STATUS` should show plrd armed.

## The SAVE_CONFIG workflow

The plugin never writes `printer.cfg` directly. Anything it persists
goes through klippy's standard autosave staging (`configfile.set`):
the value takes effect immediately (where meaningful), and the console
reminds you to run `SAVE_CONFIG`, which rewrites the config's autosave
block and restarts klippy. On the restart the plugin reads the values
back as ordinary `[plr]` options. Until you run `SAVE_CONFIG`, staged
values are live-but-volatile and are marked `[awaiting SAVE_CONFIG]`
in `PLR_SET` / `PLR_STATUS` listings.

## Development

- One-time setup: `sh scripts/setup-py.sh` (creates `.venv` at the repo
  root with the pinned dev deps from `requirements-dev.txt`; required —
  the pre-commit hook fails without it).
- Gates (enforced by `.githooks/pre-commit` and CI): `ruff check`,
  `ruff format --check`, and pytest with ≥90% line coverage over `plr/`
  (`scripts/coverage-py.sh`).
- Version split: code under `plr/` must stay **Python 3.7
  syntax-compatible** (it runs inside klippy; Klipper supports 3.7+).
  The dev tooling floor is **Python 3.9** — see `pyproject.toml`.
- numpy is **optional at runtime** (the drag classifier has a
  pure-python fallback producing identical verdicts) but a pinned dev
  dependency, so the two code paths are tested against each other.
- Tests run against the fakes in `tests/fake_klippy.py` (wiring-only
  stand-ins for klippy objects; no physics). The plrd control-socket
  client is tested over real sockets; the AF_UNIX transport tests skip
  on CPython/Windows (no `socket.AF_UNIX` there) — run them under WSL
  or any POSIX host for full coverage:
  `python3 -m pytest klippy_plugin/tests`.
