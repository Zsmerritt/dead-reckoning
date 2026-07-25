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
#noise_floor_temp_sensor:
#wal_dir: /var/lib/plrd/wal
#control_socket: /var/lib/plrd/plrd.sock
#max_probe_nozzle_temp: 150.0
#clean_nozzle_macro: CLEAN_NOZZLE
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
| `max_probe_nozzle_temp` | `150.0` | `[80, 160]` °C | Contact operations refuse while the extruder is hotter than this: the **target** strictly, the **measured** temperature with a 2 °C tolerance (so 151 °C probes fine at the default, 153 °C does not). A molten nozzle oozes onto the part and skews contact readings — see [Nozzle cleanliness](#nozzle-cleanliness). |
| `clean_nozzle_macro` | `CLEAN_NOZZLE` | macro name | Name of the `[gcode_macro …]` recovery cleans the nozzle with before contact probing. Whether that macro exists selects auto-clean vs the wizard's manual clean-confirmation prompt — see [Nozzle cleanliness](#nozzle-cleanliness). |
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

### Recovery-UX keys the daemon owns

`[plr]` also carries the recovery-UX keys the plugin never uses itself —
the consensus-touch tunables (`touch_samples`, `touch_sample_range`,
`touch_retract`, `touch_accel`), the reheat park (`reheat_park_x/y`,
`reheat_park_delta_z`, `pre_home_z_lift`), the purge
(`purge_enable`, `purge_macro`, `purge_amount`, `purge_x/y/z`,
`purge_speed`, `purge_retract`), `drag_nozzle_temp`, the acceleration
overrides (`recovery_accel`, `accel_home`, `accel_travel`, `accel_probe`,
`accel_entry`), the confirm-points (`confirm_z_before_resume`,
`debug_confirm_each_step`, `confirm_timeout_s`) and the one hard-refusal
escape hatch `UNSAFE_allow_purge_z_below_bed`. Each is documented — with
its band, its default and the physics behind it — in
[`deploy/plrd.conf.example`](../deploy/plrd.conf.example); `plrd` reads
them and is the sole authority on their values.

The plugin still has to **declare** every one of them (`plr/daemon_keys.py`):
klippy refuses to start on a `[plr]` option no module read, and it builds
the option map `plrd` reads from exactly those reads — so a missing
declaration is both a printer that will not boot and a value the daemon
could never see. The plugin checks only the type (and that a number is
finite) and does not re-state `plrd`'s bands, so an out-of-band value
boots the printer and is refused later by `plrd` with a diagnosis naming
the key. `get_status` reports them under `daemon_config` (an unset key
reads `null`, meaning `plrd`'s own default applies) so you can see exactly
what the daemon will see.

### Calibration stamping and three-tier validation

Every persisted calibration value is stamped, at the moment it is staged,
with the machine/software identity it was measured under — ported from
Cartographer3D's stale-model defense (`config/model_validator.py`). The
stamps are ordinary `[plr]` autosave options (never hand-edit):

- `cal_fingerprint_noise_floor` / `cal_fingerprint_probe_resolution` — a
  CRC-32 **fingerprint** (8 hex digits) of the *calibration-relevant* config
  slice for each value-group: the `stepper_z*` sections, the active probe
  section (`[probe]`/`[load_cell_probe]` for `probe_resolution`; the
  `accel_chip` section for the noise floor), and the `[plr]` `probe_method`
  (plus `accel_chip` for the noise floor). Canonicalized so section/key order
  and whitespace never matter and integer-valued numbers normalize (`-2` ==
  `-2.0`); unrelated config (`[fan]`, `[display]`) never changes it. The two
  groups are fingerprinted independently — a stale noise floor does not
  invalidate a still-good `probe_resolution`, and vice versa.
- `cal_plugin_version` — the plugin `__version__` (single source in
  `plr/__init__.py`).
- `cal_klipper_version` — the running Klipper `software_version`. If it is
  unavailable when a calibration would be staged, `PLR_NOISE_TEST` /
  `PLR_PROBE_TEST` / `PLR_DRAG_CALIBRATE` **refuse to stage anything** rather
  than persist an unstamped value.

At every restart each value-group is classified:

1. **VALID** — the stamps match; the value is used normally.
2. **LEGACY** — a value is present but predates stamping; accepted with a
   warn-once (`get_status` reports `calibrations_valid: "legacy"`).
3. **INVALID** — the recomputed fingerprint differs, or the plugin
   `major.minor` regressed below the staging version; the value is treated as
   **absent everywhere** (commands refuse with a "calibrated under a different
   hardware configuration — re-run PLR_NOISE_TEST / PLR_PROBE_TEST" message,
   `PLR_SETUP` shows a `[FAIL]` row with the old-vs-new fingerprint, and
   `get_status` reports `calibrations_valid: false` with a per-group
   `calibration_status`). Klipper has no plugin-reachable way to delete an
   autosave option, so treat-as-absent is our stand-in for carto's "remove the
   incompatible model" — the stale text stays in `printer.cfg` until a
   re-calibration overwrites it, we simply never trust it.

The daemon `plrd` re-derives the same fingerprint from the live Klipper config
(defense in depth: byte-identical CRC-32 canonicalization); a noise floor
whose fingerprint no longer matches is treated as uncalibrated on the Rust
side too, so recovery refuses with the usual "run PLR_NOISE_TEST first" path.

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
  proven by plrd from the WAL at recovery time);
- nozzle-cleanliness mode: whether a `[gcode_macro CLEAN_NOZZLE]` exists
  (auto-clean) or the recovery wizard will ask for manual confirmation —
  see [Nozzle cleanliness](#nozzle-cleanliness).

`PLR_SETUP_WIZARD` walks the same report as an interactive prompt on
supported clients (Mainsail/Fluidd/KlipperScreen/OctoApp), one button per
remaining step — see [Recovery and setup wizards](#recovery-and-setup-wizards).

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
when unhomed, for `probe_method: adxl_drag` (use `PLR_DRAG_PROBE`), and
**while the nozzle is hot** (current *or* target above
`max_probe_nozzle_temp`; cool it — `M104 S0` — and retry). The
temperature gate is shared by all four contact commands (`PLR_TOUCH`,
`PLR_PROBE_TEST`, `PLR_DRAG_PROBE`, `PLR_DRAG_CALIBRATE`); see
[Nozzle cleanliness](#nozzle-cleanliness).
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

### Every daemon call is asynchronous — and why that is a safety property

Klippy has **one thread**. It runs a single `select`/dispatch loop that
invokes g-code handlers inline (`klippy/reactor.py:314-327`), so anything
a handler waits for, klippy waits for: no timer fires, no file descriptor
is serviced, and the g-code mutex the handler holds
(`klippy/gcode.py:239-241`) is not released.

That is not merely slow. Klipper's heaters compare every PWM update
against a deadline refreshed only from the reactor
(`MAX_MAINTHREAD_TIME = 5.0`, `klippy/extras/heaters.py:17`, :72-74,
:138-141): a reactor stalled past it silently drops every heater to 0 %
with its target still set, which `verify_heater` can escalate to a
printer shutdown (`klippy/extras/verify_heater.py:86-91`). A recovery is
exactly the window where this bites: the plan sets the bed temperature
first and holds for the probe temperature before any motion.

The MCU-side bound is tighter still: heater pins are armed with
`MAX_HEAT_TIME = 3.0` (`heaters.py:14`, :62) and a pin left at a
non-default value with no further update inside that window shuts the MCU
down (`src/pwmcmds.c:45-53`). The host stays clear of *that* only because
heater PWM is refreshed from the serial background thread rather than the
reactor (`klippy/serialhdl.py:41-65` dispatches registered responses;
`klippy/mcu.py:628-630` registers the ADC callback) — so a stalled
reactor turns the heaters off rather than shutting the MCU down. Mid-print
there is a further hazard in the same class: a reactor that stops flushing
the motion queue and then resumes is the classic source of Klipper's
`Timer too close`.

And it deadlocks: plrd drives the machine through Moonraker's
`printer.gcode.script`, which Moonraker forwards to klippy's API socket
(`klippy/webhooks.py:439-448`) — so it needs the reactor and the same
g-code mutex the blocked handler is holding.

**Therefore no `PLR_*` command ever waits for plrd inside its handler.**
Every daemon call runs on a worker thread and reports back through
`reactor.register_async_callback` (`klippy/reactor.py:199-205`), klippy's
own cross-thread wakeup. The visible consequences:

- commands **return immediately** and print their result when it arrives,
  so `PLR_STATUS` prints the plugin's own state first and plrd's block
  second;
- a failure a worker discovers arrives as an error line (`!!`) rather than
  a command error, because a callback has no command left to fail.

#### `PLR_RECOVER EXECUTE=1` must be the LAST line of any macro

**Rule, not advice.** The command returns while the recovery is still
running, and the blocker is the **g-code mutex**, not the reactor:
`run_script` holds it for the whole script (`klippy/gcode.py:239-241`), and
plrd needs that same mutex for every command it sends. So a macro's
*remaining* lines keep plrd out — starving it while it has the nozzle near
the part at probing temperature — even though they are perfectly
well-behaved klippy code that pauses the reactor politely. (`M109`/`M190`
block for minutes exactly that way: `reactor.pause` keeps the reactor
free, but the mutex stays held.) Anything after `PLR_RECOVER EXECUTE=1` in
a macro therefore delays plrd's next step by however long it takes, and
anything that moves races plrd for the machine.

Nothing enforces this today, and this document previously claimed
otherwise: plrd's ready+idle gate does **not** catch it. That gate queries
`webhooks`, `print_stats` and `virtual_sdcard` only
(`crates/plrd/src/recover.rs:672-692`), and a console-invoked macro leaves
all three in their accepted states, so the gate passes every time. The
enforceable fix is a daemon-side change — adding `idle_timeout` to that
gate, which the daemon already subscribes to (`crates/plrd/src/client.rs:69`)
and whose `"Printing"` state distinguishes a macro that moves from one that
only prints a message — and it is queued, not shipped. Until it lands, the
rule above is the whole protection.

### `PLR_STATUS`

Plugin-side state (probe method, attestation, probe resolution, live
tunables with pending-save markers, and the live recovery state) prints
immediately; the daemon's own `status` report follows when it answers. If
plrd is unreachable the plugin state still prints, with a clear hint
(`systemctl status plrd`).

### `PLR_RECOVER [EXECUTE=1 CONFIRM=YES]`

Power-loss recovery, driven by plrd:

- `PLR_RECOVER` — **dry run** (default): plrd validates the machine
  and prints the full plan; no motion.
- `PLR_RECOVER EXECUTE=1 CONFIRM=YES` — execute. Both arguments are
  required verbatim; anything less refuses client-side. plrd still
  enforces every gate server-side (machine validation, klippy
  ready+idle, transcript), so the console consent is additive. The
  command returns as soon as plrd has been asked to start; the recovery
  reports to the console as it goes, and stops to ask you questions (next
  section).
- `STEP=1` is **refused client-side**, with the remedy: the daemon
  rejects a `step` argument over the control socket outright
  (`per-step mode is CLI-only`), and the socket route to a step-by-step
  recovery is the `[plr]` key `debug_confirm_each_step`, which makes plrd
  stop and ask before **every** step (see below). `plrd recover --execute
  --confirm --step` remains the CLI equivalent.

### Answering plrd's questions: `PLR_RECOVER_CONTINUE` / `PLR_RECOVER_ABORT`

A recovery is not a single request. plrd **pauses** and asks whenever it
hits a *confirm-point*, and the plugin renders each one as a dialog whose
two buttons fire exactly these two console commands — so the whole
interaction is completable from a bare console on a client that renders no
dialog at all:

| command | meaning |
| --- | --- |
| `PLR_RECOVER_CONTINUE` | proceed despite what plrd reported |
| `PLR_RECOVER_ABORT` | stop the recovery cleanly, here |

There are three kinds of confirm-point, and the prompt asks a different
question for each:

- a **`Tier::Confirmable` diagnosis** — "continue despite this?". Every
  such failure says WHY it stopped, SUGGESTS a fix, and offers to continue
  anyway. When the diagnosis names an `UNSAFE_` override key, the prompt
  *says so as a fact* and tells you it is set in `printer.cfg` while the
  machine is idle — it is never a button.
- **`confirm_z_before_resume`** (a `[plr]` key) — "does this look right?":
  plrd lifts to a standoff, reports the Z it believes it is at and the
  arithmetic behind it, and waits for you to compare it against the actual
  nozzle. Declining invalidates the Z frame, so a fresh dry run is
  required before any resume.
- **`debug_confirm_each_step`** (a `[plr]` key) — "run the next step?"
  before every step, listing that step's exact commands.

Only one question is ever outstanding, and plrd runs exactly one recovery
at a time. `PLR_STATUS` re-states the outstanding question in full —
prompt included — and `PLR_WIZARD_START` does too rather than offering a
new recovery over the top of it.

#### What the plugin will and will not claim about the machine

This is the operator contract, and it is deliberately asymmetric: the
plugin will tell you it does not know, but it will never tell you nothing
is happening unless it can show that. `get_status` publishes
`recovery_state`, and the console prints the same thing from the same
source, so the two can never disagree:

| `recovery_state` | meaning | starting a new recovery |
| --- | --- | --- |
| `idle` | nothing known to be happening | allowed |
| `running` | plrd is executing **this** session's recovery; its report comes to this console | refused locally |
| `awaiting_confirmation` | plrd is paused on a question, and the plugin can still show it is live | refused locally |
| `plrd_busy` | plrd **told us** it is executing a recovery (`busy`) that this session cannot report on or answer — positive proof the machine is under its control | allowed: plrd re-answers `busy` while it is still working |
| `unknown` | plrd answered something that does not say what it is doing, or the plugin can no longer show a pause is live | allowed: it is the only way to find out |

`recovery_awaiting_confirmation` is true only in the
`awaiting_confirmation` row — never on the strength of a guess.
`recovery_can_answer` is published separately, because whether an answer can
still be **sent** is a property of the outstanding token rather than of the
state: a question outlives the plugin's ability to vouch for it, and the
console and the API say the same thing about it.

In the two "allowed" rows the console says **do not touch the printer**, and
the way to learn more is to try a recovery again — `PLR_WIZARD_START` (which
keeps the dry-run review) or `PLR_RECOVER EXECUTE=1 CONFIRM=YES`. plrd
answers `busy` if it is still working, and there is no path in the daemon to
two concurrent recoveries. **That attempt is also the only exit from those
two rows**, deliberately: nothing but plrd can tell this plugin the machine
is free, so the state stands until a recovery conversation reaches a terminal
answer. The wizard therefore still works in both rows, and carries the
warning into its dialog rather than replacing it.

If a question is still answerable in one of those rows,
`PLR_RECOVER_CONTINUE` / `PLR_RECOVER_ABORT` still work — and starting a new
recovery instead **abandons** it, says so, and sends `abort` to plrd for it
so it is not left waiting out its own deadline.

**"Sent" is never reported as "landed".** Wherever the plugin sends an abort
without waiting for the reply — a klippy shutdown, or abandoning a question —
it says the abort was *sent*, publishes `unknown`, and reports success only
once plrd has actually confirmed it (which is also the only thing that
publishes anything calmer). An abort that never arrived leaves plrd paused,
and it will run that step's cleanup commands when its own deadline expires;
a console that had already said "aborted, idle" is a machine that moves after
the operator was told it was over.

**Deadlines — two of them, and they end at different times.** plrd bounds
an unanswered question itself and **aborts cleanly** when it expires: that
is the safe direction and it is what you will see. Its deadline is
`confirm_timeout_s` in `[plr]` if you set it, otherwise plrd's own default,
which **no response reports**. So:

- the plugin's dialog **wait** is longer than any deadline plrd could be
  using (derived from the same `[plr]` value, plus named headroom), so
  plrd's own abort always wins the race and is always what gets reported;
- but if you did not set `confirm_timeout_s`, the plugin stops **claiming**
  the question is live once plrd's own default has passed: `recovery_state`
  becomes `unknown`, the console says plrd has probably aborted already, and
  a new recovery stops being refused. The question stays answerable, because
  plrd's reply is the only thing that can settle it.

If your answer arrives after plrd gave up, plrd says so and the plugin
reports it as the abort it is — never as a transport error. If plrd answers
that it is no longer waiting for that answer *without* saying the recovery
ended, the plugin says exactly that and goes to `unknown`: it will not tell
you a recovery aborted when plrd may still be paused with the nozzle over
the part.

If klippy shuts down (M112, an MCU fault) while a question is open — or if
one **arrives** after the shutdown, which is the likelier order — the plugin
sends `abort` for you so plrd stops immediately rather than at its deadline,
says it has been *sent*, and clears the dialog. It never leaves a live dialog
on screen whose Continue button could not work. Until plrd confirms that
abort the state is `unknown`, not `idle` (above). A shutdown *during*
execution cannot interrupt plrd: if the recovery is this session's, the
plugin says so and its report still arrives; if plrd is executing one this
plugin is not connected to (`plrd_busy`) it says that instead — **no report
for that one will appear here** — and points at `journalctl -u plrd`. A shutdown does **not** stop `PLR_STATUS`,
`PLR_RECOVER` or `PLR_WIZARD_START` from working — klippy stays up until
`FIRMWARE_RESTART`, and that is exactly when you need to know what plrd
thinks it is doing — but it does forbid *acting*: starting a recovery, or
answering `continue`, are both refused until the shutdown is cleared.

### Recovery and setup wizards

Two prompt-driven flows that render as interactive dialogs on clients
that support Klipper's **action prompts** (see
[Client support](#client-support-and-graceful-degradation) below) and
degrade to plain console text everywhere else. **Prompts are sugar;
console commands are the contract** — every prompt is paired with a
plain-text line naming the exact console command that advances it, so
you are never stuck on a client without prompt support.

**`PLR_WIZARD_START`** opens the guided power-loss recovery flow. It asks
plrd whether a recovery is pending (the `status` response's `pending`
field: `null` for none, otherwise the pending-recovery record) and, if so,
offers a dialog summarizing the interrupted print — file name, approximate
progress, resume byte and crash classification, each shown only when the
daemon actually reports it — with two choices:

- **Attempt recovery** → `PLR_WIZARD_DRYRUN`: fetches and prints the full
  recovery plan (no motion), then prompts the next step. It **asks
  *"Is the nozzle clean?"*** — **Nozzle is clean**
  (`PLR_WIZARD_CONFIRM_CLEAN`) or **abort** (`PLR_WIZARD_CANCEL`) —
  unless *both* independent sources agree the nozzle is cleaned
  automatically: plrd reports the plan does not need confirmation **and**
  the plugin can see the configured `[gcode_macro CLEAN_NOZZLE]` section.
  Only then does it skip to the execute prompt, which names that macro
  section as the reason. In particular it **asks anyway** when:
  - plrd is older (or the field is unreadable) and reports nothing about
    cleaning — the unknown case takes the conservative branch;
  - plrd says cleaning is automatic but no such macro is configured here
    — the two sources disagree, and the prompt says so.

  This asymmetry is deliberate: asking redundantly costs one click, while
  skipping the check when nothing actually cleans the nozzle silently
  corrupts the contact reading recovery depends on.
- **`PLR_WIZARD_EXECUTE`** runs the plan — **the printer WILL MOVE** (all
  motion is plrd's, over the control socket; the wizard itself never
  issues g-code). It returns immediately and hands off to the same
  recovery session `PLR_RECOVER` uses, so plrd's confirm-points appear as
  prompts answered with `PLR_RECOVER_CONTINUE` / `PLR_RECOVER_ABORT`
  (previous section). On success it reports *plrd has resumed the print*;
  a typed failure prints the remediation from the daemon's report.
- **`PLR_WIZARD_CANCEL`** dismisses the flow and resets at any point —
  except while plrd is executing: at a confirm-point it answers `abort`
  for you, and while a step is actually running it refuses rather than
  resetting the plugin's view of a machine plrd is still driving (M112 is
  the way to stop the machine now).

A second `PLR_WIZARD_START` while a flow is active simply re-shows the
current prompt; any daemon error resets the flow with a clear message.
`get_status` exposes `wizard_active` so UIs can tell a flow is mid-run.
plrd's boot announcement is expected to tell users to run
`PLR_WIZARD_START` after a power loss (the announcement text is owned by
the daemon).

**`PLR_SETUP_WIZARD`** walks the commissioning report (the exact
`PLR_SETUP` checks) as one dialog with a button per remaining step —
attest self-locking Z, run the probe test (or, for drag machines, the
noise test and drag calibrate), and finally `SAVE_CONFIG`. Each button
fires the underlying command directly; those commands remain the single
source of truth (and keep their own motion consent). Its **Close** button
(`PLR_WIZARD_CLOSE`) dismisses the dialog, and re-running the command
closes the previous dialog before opening a new one, so prompts never
stack or linger.

**`PLR_WIZARD_CLOSE`** closes whatever prompt is on screen. It is
display-only: it will not abandon an in-flight recovery (use
`PLR_WIZARD_CANCEL` for that; `PLR_WIZARD_START` re-shows the prompt).

#### Client support and graceful degradation

The action-prompt wire format is Mainsail's *Macro Prompts* spec
(`// action:prompt_begin`, `prompt_text`, `prompt_button
<label>|<gcode>|<color>`, `prompt_footer_button`, `prompt_show`,
`prompt_end`).

You do **not** need Klipper's `[respond]` module for these wizards. The
Mainsail docs list it because their examples emit prompts from a
`[gcode_macro]` using the `RESPOND` command; this plugin emits the same
bytes directly from plugin code, so no config change is required.

| client | support | what you get |
| --- | --- | --- |
| Mainsail ≥ 2.9.0 | full (verified) | interactive dialog, working buttons |
| KlipperScreen | full (verified) | interactive dialog, working buttons |
| Fluidd | **unverified** | assume console fallback unless your version shows the dialog |
| OctoPrint / OctoApp | plain prompts only | the older Action Command Prompt protocol has no pipe-delimited fields, gcode, or colors, so our buttons render as inert literal text — **use the console commands** |

On any client that does not render the dialog, the plain-text
instructions printed beside every prompt carry the entire flow — they name
the exact command for each choice, so nothing is unreachable.

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
diagnostics keep theirs). It still refuses while printing, unhomed, or
**while the nozzle is hot** (the shared temperature gate — see
[Nozzle cleanliness](#nozzle-cleanliness)); a gate refusal also sets
`last_drag_error`. `MAX_SECONDS` (range `[30, 600]`) and
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
without `START=1` it prints the plan and moves nothing. It shares the
contact-operation **temperature gate** (refuses while the nozzle is hot —
its clear-Z passes still drip; see
[Nozzle cleanliness](#nozzle-cleanliness)).

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

## Nozzle cleanliness

Every contact reading — a descending `tap`/`load_cell` touch **and** a
lateral `adxl_drag` pass — is only as trustworthy as the nozzle tip. A
bead of ooze or a drag of stringing changes where "contact" registers by
tens of microns, and recovery trusts that height to re-establish Z
against the part. Two mechanisms protect it:

**1. The temperature gate (automatic, non-negotiable).** All four contact
commands — `PLR_TOUCH`, `PLR_PROBE_TEST`, `PLR_DRAG_PROBE`,
`PLR_DRAG_CALIBRATE` — refuse before any motion when the extruder is
hotter than `max_probe_nozzle_temp` (default 150 °C, range `[80, 160]`).
The *target* is checked too: a nozzle at 45 °C but commanded to 250 °C is
already on its way up and is refused now, not after it melts onto the
part. The drag passes run at a guaranteed-clear Z, but a molten nozzle
still drips, so they are gated identically. Remediation is always in the
refusal: **cool the nozzle below the limit (`M104 S0`) and wait**, then
retry.

The two comparisons are deliberately **asymmetric**, which is why a
nozzle reading 151 °C still probes at the 150 °C default:

- the **measured** temperature is allowed a **2 °C tolerance** — it
  refuses only above `max_probe_nozzle_temp + 2`. Sensor noise and
  ordinary PID overshoot put the reading a few tenths above whatever was
  commanded, and recovery deliberately probes *at* this ceiling, so a
  strict comparison would refuse the recovery plan's own probe command
  mid-recovery;
- the **target** is compared strictly, with no tolerance. A target above
  the ceiling is an *intent* to get too hot, not measurement scatter, so
  the tolerance never licenses commanding a hotter nozzle.

**2. A clean tip (auto-macro or manual confirmation).** A cold nozzle can
still carry dried filament. The recovery wizard makes cleanliness an
explicit step:

- If you configure a nozzle-cleaning macro and name it in
  `clean_nozzle_macro` (default `CLEAN_NOZZLE`) **and** plrd's plan
  reports that cleaning is handled automatically, the wizard skips the
  question and the execute prompt names that macro section as the reason.
  The convention is a `[gcode_macro CLEAN_NOZZLE]` that wipes/purges the
  tip; the name is configurable so an existing wipe macro can be reused.
- **In every other case the wizard asks** you to confirm the nozzle is
  clean before executing (**Nozzle is clean** vs **It's dirty — abort**),
  and says why it is asking: no macro is configured, plrd reported
  nothing about cleaning (an older daemon), or plrd and the plugin
  disagree about whether a macro exists.

The skip therefore requires **both** independent sources to agree; an
unknown or contradictory answer takes the conservative branch. That is
deliberate — a redundant question costs one click, whereas skipping the
check when nothing cleans the nozzle silently corrupts the reference
measurement that recovery re-establishes Z from.

`PLR_SETUP` reports which mode applies, and `get_status` exposes
`clean_nozzle_macro_available` so UIs can surface it. Whether a plan
actually requires the manual confirmation is decided by plrd (it knows
the server-side park/purge configuration) and carried to the wizard as a
plan flag; the plugin-side macro detection is the second, independent
source and drives the report and status row.

## Commissioning in 5 console commands

```
PLR_SETUP                          ; 1. read the report, fix any [FAIL]
PLR_SETUP ACCEPT_SELF_LOCKING_Z=1  ; 2. attest self-locking Z (leadscrews!)
G28                                ; 3. home
PLR_PROBE_TEST START=1             ; 4. measure probe repeatability
SAVE_CONFIG                        ; 5. persist attestation + resolution
```

After the restart, `PLR_SETUP` should print `COMMISSIONED` and
`PLR_STATUS` should show plrd armed. (`adxl_drag` machines calibrate
with `PLR_NOISE_TEST` + `PLR_DRAG_CALIBRATE` instead of
`PLR_PROBE_TEST` — the full flow is
[docs/install.md → Commissioning from the console](../docs/install.md#commissioning-from-the-console).)

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
