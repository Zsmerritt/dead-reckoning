# klippy_plugin — Klipper console plugin for dead-reckoning

Klipper (klippy) extras plugin for dead-reckoning power-loss recovery.
All user interaction happens through `PLR_*` g-code console commands
registered by a `[plr]` config section; the heavy lifting (journaling,
reconstruction, recovery planning, motion) stays in the Rust daemon
(`plrd`), reached over its control socket.

## Install

The plugin ships as a python package that klippy loads like any other
extras module. Symlink it into your Klipper checkout and restart:

```sh
ln -s /path/to/dead-reckoning/klippy_plugin/plr ~/klipper/klippy/extras/plr
sudo systemctl restart klipper
```

Then add a `[plr]` section to `printer.cfg` (all options below have
defaults; an empty `[plr]` section is valid) and install/enable the
`plrd` service (see `docs/install.md` at the repo root).

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
`probe_resolution` (measured by `PLR_PROBE_TEST`), and `noise_floor_*`
(drag-oracle milestone).

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

### `PLR_PROBE_TEST [SAMPLES=10] START=1`

Probe repeatability test (probe_method `tap`/`load_cell` only), the
same loop as Klipper's `PROBE_ACCURACY` but at the `[plr]`
`probe_speed`. **This command moves the toolhead** (the probe descends
at the current XY): without `START=1` it only prints what it would do;
it also refuses while a print is active or the printer is not fully
homed. `SAMPLES` accepts 3–50.

Prints range/stddev/median of the trigger heights and stages
`probe_resolution = max(2*stddev, 0.005)` for `SAVE_CONFIG` — plrd uses
this as the trust radius for recovery probing.

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
- `STEP=1` — single-step mode (pause between plan steps).

### `PLR_NOISE_TEST`, `PLR_DRAG_PROBE`

Registered but pending the drag-oracle milestone; they currently
respond with a "not implemented yet" notice.

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
- Tests run against the fakes in `tests/fake_klippy.py` (wiring-only
  stand-ins for klippy objects; no physics). The plrd control-socket
  client is tested over real sockets; the AF_UNIX transport tests skip
  on CPython/Windows (no `socket.AF_UNIX` there) — run them under WSL
  or any POSIX host for full coverage:
  `python3 -m pytest klippy_plugin/tests`.
