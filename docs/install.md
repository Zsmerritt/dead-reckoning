# Installing plrd on a Klipper host

This guide covers a Raspberry Pi (or any Linux SBC) running Klipper, from
prerequisites through first-run verification and console commissioning.
Companion documents: [architecture](architecture.md) for what the daemon
records and why, [operations](operations.md) for day-2 usage,
[klippy_plugin/README.md](../klippy_plugin/README.md) for the full
`PLR_*` command and `[plr]` option reference, and
[examples/plrd.conf](../examples/plrd.conf) /
[examples/printer-plr-section.cfg](../examples/printer-plr-section.cfg)
for fully commented configs.

Contents:

1. [Quick install (one command)](#quick-install-one-command)
2. [Commissioning checklist](#commissioning-checklist) — is this machine a
   candidate for recovery at all?
3. [Where each prerequisite lives in the Klipper config](#where-each-prerequisite-lives)
4. [Building plrd manually](#building-plrd-manually)
5. [Installing manually](#installing-manually) — including the plugin
   symlink and the `[plr]` starter section
6. [Commissioning from the console](#commissioning-from-the-console)
   — required before recovery will execute anything
7. [Legacy: commissioning the machine section](#legacy-commissioning-the-machine-section)
   — the pre-plugin `/etc/plrd.conf [machine]` path
8. [Moonraker update manager integration](#moonraker-update-manager-integration)
9. [First-run verification](#first-run-verification)

## Quick install (one command)

On the printer host, as your normal printer user (not root; sudo is used
only where needed):

```sh
curl -sSL https://raw.githubusercontent.com/Zsmerritt/dead-reckoning/main/scripts/install.sh | bash
```

Prefer to read what you run? [`scripts/install.sh`](../scripts/install.sh)
is short and commented — download it, read it, run
`bash install.sh`. It is equally runnable from a clone
(`bash scripts/install.sh`). What it does:

1. checks/installs build prerequisites (and rustup if missing — the pinned
   toolchain from `rust-toolchain.toml` installs itself on first build);
2. clones the repo to `~/dead-reckoning` (or updates/uses an existing
   checkout) and builds `plrd` in release mode;
3. detects your `printer_data` directory, the Klipper API socket
   (`comms/klippy.sock`), and your `[stepper_z*]` sections from
   `printer.cfg`, and generates `/etc/plrd.conf` from them (an existing
   config is left untouched unless you pass `--force-config`; backups are
   timestamped);
4. installs the binary, config, and systemd unit, then enables and starts
   the service;
5. symlinks the Klipper console plugin into your klipper checkout:
   `<klipper>/klippy/extras/plr -> <repo>/klippy_plugin/plr`
   (auto-detects `~/klipper`; override with `--klipper <path>`; if no
   checkout is found it prints the manual `ln -sfn` command and
   continues). The symlink is idempotent across re-runs and kept healthy
   by the Moonraker `--refresh` update hook; anything at that path that
   is not our symlink is never touched.

Safety properties of the script itself: it **never** talks to the Klipper
socket, **never** edits `printer.cfg`, and **never** stops, starts, or
restarts the klipper or moonraker services (activating the plugin —
adding `[plr]` and restarting Klipper — is deliberately yours);
Moonraker-file edits happen only with `--moonraker` and are backed up
first.

Flags: `--yes` (non-interactive, take defaults), `--moonraker` (register
with Moonraker's update manager — see
[below](#moonraker-update-manager-integration)), `--no-service` (build and
stage only; no sudo, nothing installed, plugin not linked),
`--force-config`, `--dir <path>` (checkout location),
`--printer-data <path>`, `--klipper <path>`. To remove everything later:
[`scripts/uninstall.sh`](../scripts/uninstall.sh) (keeps the config and the
WAL — which holds recovery data — unless you explicitly ask for purge; the
plugin symlink is removed only if it points into a dead-reckoning
checkout, and you get a reminder to delete the `[plr]` section yourself).

After installing, add the `[plr]` section and restart Klipper (the
installer prints a starter block; the commented reference is
[examples/printer-plr-section.cfg](../examples/printer-plr-section.cfg)),
then continue with [first-run verification](#first-run-verification) and —
before ever executing a recovery —
[console commissioning](#commissioning-from-the-console).

## Commissioning checklist

Recovery on a moving-bed-Z machine is only safe when the machine satisfies
structural prerequisites. These are the exact checks
`plr_recovery::validate_machine` enforces (every failed check is reported,
not just the first). The console command **`PLR_SETUP`** runs the
automatable ones for you against the live config and prints
`[PASS]`/`[WARN]`/`[FAIL]` with remediation hints — but read the list
once **before** you rely on this system; the self-locking-Z item is a
physical fact only you can verify:

- [ ] **`[force_move]` with `enable_force_move: True`** is present in
  `printer.cfg`.
- [ ] **Self-locking Z leadscrews.** The bed must not back-drive under
  gravity when the steppers are unpowered. Software cannot observe this —
  it is an **operator attestation**. Typical Voron trapezoidal leadscrew Z
  drives qualify; belted or ballscrew Z drives that sag when de-energized do
  **not**, and this system is not safe on them.
- [ ] **Every Z stepper is on the primary MCU.** Multi-MCU Z is refused: the
  shifted-frame bound relies on single-MCU step accounting.
- [ ] **The sliced file carries `;TYPE:` annotations.** Contact-zone
  selection refuses to classify geometry without them. PrusaSlicer,
  OrcaSlicer, SuperSlicer and Cura all emit them by default
  (`;TYPE:Inner wall`, `;TYPE:WALL-INNER`, `;TYPE:Sparse infill`, …).
- [ ] **A contact oracle matching your `[plr]` `probe_method`.** For
  `tap` / `load_cell`: exactly one Tap-style `[probe]` or
  `[load_cell_probe]` — the probe must trigger on nozzle contact (Voron
  Tap, load cell); inductive-only probes cannot reference the printed
  part and do not qualify. For `adxl_drag`: an accelerometer section
  named by `accel_chip`, plus a noise-floor calibration
  (`PLR_NOISE_TEST`) before any drag probe — with the honest caveat that
  drag detection quality is bench-unvalidated (E5) even though its
  safety bounds are tested.
- [ ] **Probe `activate_gcode` / `deactivate_gcode` are empty or verified
  move-free.** A moving activate G-code would break the halt-position
  arithmetic.
- [ ] **The Z rail's `position_min` is known** (or `[printer]
  minimum_z_position` as fallback) — it anchors the probe envelope. A small
  negative value (e.g. `-2`) is typical and fine; "unset" is not.
- [ ] **`[virtual_sdcard]` root is known**, and you print top-level files
  from it (the `M23` resume path does not descend into subdirectories).
- [ ] **Config-change discipline.** With a `[plr]` section, the machine
  snapshot is re-read from the **live** Klipper config on every recovery
  run, so validation always reflects the config as it is now — no
  blessing ritual. On the legacy `[machine]` path the prerequisites are
  instead validated against a hash of the running Klipper config; if the
  config changes, they must be re-validated and re-blessed before any
  recovery.

Two operational prerequisites on top of the machine ones:

- [ ] **Durable storage for the WAL.** `wal_dir` must be on storage that
  survives power loss *with the printer* — see the
  [disk notes in operations](operations.md#disk-sizing-and-write-load) for
  the dedicated-partition / USB-stick option.
- [ ] **Empirical validation (open tasks E1–E5).** The logic pipeline is
  property- and golden-tested, but the project's empirical validation tasks
  — real-hardware fault injection (killing power mid-print and checking the
  reconstruction against the physical machine), probe repeatability on
  infill, and WAL write-load measurement on your host — are still open.
  Treat them as *your* commissioning steps: before trusting a recovery on a
  real part, kill power on a scrap print, run `plrd scan`, and check the
  reported possible-stop set against where the machine actually is.

## Where each prerequisite lives

How to check each item against a stock Klipper/Moonraker install:

| Prerequisite | Where to look |
| --- | --- |
| `[force_move]` | `printer.cfg`: a `[force_move]` section containing `enable_force_move: True` |
| Z steppers and their MCU | `printer.cfg`: `[stepper_z]`, `[stepper_z1]`, … — their `step_pin`/`dir_pin`/`enable_pin` must all be pins of the primary `[mcu]` (no `[mcu xxx]` prefix on the pin names) |
| Probe | `printer.cfg`: exactly one `[probe]` (Tap-style) or `[load_cell_probe]` section; note its `z_offset`; check `activate_gcode`/`deactivate_gcode` are absent, empty, or contain no motion commands |
| Accel chip (`adxl_drag` only) | `printer.cfg`: the `[adxl345]`-style section whose full name you put in `[plr] accel_chip` (e.g. `adxl345` or `adxl345 bed`) |
| Z `position_min` | `printer.cfg`: `position_min` under `[stepper_z]`, or `minimum_z_position` under `[printer]` |
| `[virtual_sdcard]` root | `printer.cfg`: the `path` value of `[virtual_sdcard]` (Moonraker installs: `~/printer_data/gcodes`) |
| Klipper API socket | the klippy service file (usually `/etc/systemd/system/klipper.service` or `~/printer_data/systemd/klipper.env`): the `-a <path>` argument of `klippy.py`. Moonraker installs use `~/printer_data/comms/klippy.sock`. This is the value for `klipper_socket` in `plrd.conf` |
| `;TYPE:` annotations | open a sliced file and search for `;TYPE:`; if absent, enable the feature-comment option in your slicer profile |

## Building plrd manually

(Skip this and the next section if you used the
[install script](#quick-install-one-command).) Rust is pinned by
`rust-toolchain.toml` (currently 1.97.x); rustup fetches the right
toolchain automatically on first build.

### On the printer host (tested path)

```sh
git clone https://github.com/Zsmerritt/dead-reckoning.git
cd dead-reckoning
cargo build --release -p plrd
```

A Pi 4/5 builds this in a few minutes. `--release` matters only for
politeness (the daemon is I/O-bound), but use it anyway.

### Cross-compiling from WSL or a Linux workstation

Standard Rust cross-compilation applies (this path is not exercised by CI —
the on-device build is the tested one). For a 64-bit Pi OS:

```sh
rustup target add aarch64-unknown-linux-gnu
sudo apt install gcc-aarch64-linux-gnu        # cross linker
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
    cargo build --release -p plrd --target aarch64-unknown-linux-gnu
scp target/aarch64-unknown-linux-gnu/release/plrd pi@printer:
```

WSL note: if you also intend to *develop* (run `cargo test --workspace`,
which includes the daemon's real-fsync tests), keep the clone inside the WSL
filesystem (e.g. `~/src/dead-reckoning`), not under `/mnt/c` — the 9p mount
does not give real fsync semantics, and durability code is never tested
against fakes.

## Installing manually

These are the same steps the header of `deploy/plrd.service` documents:

```sh
sudo cp target/release/plrd /usr/local/bin/plrd
sudo cp deploy/plrd.conf.example /etc/plrd.conf     # then edit — see below
sudo cp deploy/plrd.service /etc/systemd/system/plrd.service
sudo systemctl daemon-reload
sudo systemctl enable --now plrd
```

### The Klipper console plugin

The `PLR_*` console commands come from a klippy extras module that lives
in this repo and is linked — not copied — into your Klipper checkout, so
repo updates update the plugin (`scripts/install.sh` does this step
automatically, and its Moonraker `--refresh` hook re-checks the link on
every update):

```sh
ln -sfn ~/dead-reckoning/klippy_plugin/plr ~/klipper/klippy/extras/plr
```

Then add a `[plr]` section to `printer.cfg` and restart Klipper. The
minimal section is two lines — every other key has a default:

```ini
[plr]
probe_method: tap   # or load_cell; adxl_drag also needs accel_chip
```

The commented starter block with every key is
[examples/printer-plr-section.cfg](../examples/printer-plr-section.cfg);
the authoritative option table is
[klippy_plugin/README.md](../klippy_plugin/README.md). When this section
exists it is the **authoritative machine config**: plrd reads it from
the live Klipper config at recover time, and the legacy `[machine]`
section of `/etc/plrd.conf` [is ignored](#legacy-commissioning-the-machine-section).

### Editing `/etc/plrd.conf`

The format is `key = value` with `#` comments and one optional `[machine]`
section — the same family as `printer.cfg`. **Unknown and duplicate keys
are hard errors** (a misspelled durability knob silently falling back to a
default is exactly the kind of quiet failure this project exists to
prevent). Every top-level key and its default:

| Key | Default | Meaning |
| --- | --- | --- |
| `klipper_socket` | `/tmp/klippy_uds` | Klipper API Unix socket (klippy's `-a` flag). Moonraker installs: `/home/<user>/printer_data/comms/klippy.sock` |
| `wal_dir` | `/var/lib/plrd/wal` | WAL segments, heartbeat file, and receive-seq sidecar. Must survive power loss with the printer |
| `heartbeat_path` | `<wal_dir>/heartbeat.bin` | Heartbeat file override |
| `z_steppers` | `stepper_z` | Comma list of Z steppers to record committed-step history for; add `stepper_z1, stepper_z2, …` on multi-Z machines |
| `trapq_queues` | `toolhead,extruder` | Motion queues to record |
| `heartbeat_hz` | `10` | Heartbeat rewrite rate, Hz; each rewrite is synced. Valid range (0, 1000] |
| `batch_sync_ms` | `500` | Batch `fdatasync` interval for motion records, ms; markers/contexts always sync immediately. Valid [1, 60000] |
| `heartbeat_o_dsync` | `false` | Open the heartbeat file `O_DSYNC` instead of `fdatasync` per rewrite (same durability, one syscall instead of two) |
| `segment_rotate_bytes` | `16777216` | Rotate to a new WAL segment at this size. Minimum 4096 |
| `channel_capacity` | `1024` | Bounded queue between socket reader and WAL thread; on overflow motion records are dropped and the gap is journaled. Minimum 8 |
| `moonraker_url` | `ws://127.0.0.1:7125/websocket` | Moonraker WebSocket endpoint, used for the boot-time pending-recovery announcement and by `plrd recover --execute` |
| `control_socket` | `/var/lib/plrd/plrd.sock` | UNIX control socket served by `plrd run`; the plugin's `PLR_STATUS`/`PLR_RECOVER` connect here. Must match `[plr] control_socket` in printer.cfg — the defaults keep the two aligned, so change both together or neither. Mode 0666 by default (see [operations → socket permissions](operations.md#troubleshooting)) |

Plus the optional legacy `[machine]` section — the pre-plugin
recovery-commissioning snapshot, documented
[below](#legacy-commissioning-the-machine-section); with a `[plr]`
section in printer.cfg it is ignored.

For *recording*, the two values almost everyone must change are
`klipper_socket` (the install script detects it; point it at the real
klippy socket otherwise) and, on multi-Z machines, `z_steppers`. A fully
commented example for a Voron 2.4-style machine is at
[examples/plrd.conf](../examples/plrd.conf).

### The systemd unit

`deploy/plrd.service` is `Type=notify` (the daemon hand-rolls the one
`READY=1` sd_notify datagram), starts after `klipper.service` without
requiring it (plrd records around Klipper restarts and waits for the socket
on its own), and restarts on every exit. Hardening is strict
(`ProtectSystem=strict`); the WAL directory is provided as
`StateDirectory=plrd`, which matches the default
`wal_dir = /var/lib/plrd/wal`.

If you change `wal_dir` away from `/var/lib/plrd`, you must also grant the
service write access to the new location — see
[examples/plrd.service.override.conf](../examples/plrd.service.override.conf)
for a drop-in override that does this.

## Commissioning from the console

Recording needs none of this. **Executing a recovery does**: recovery
refuses — listing every failed check — until all machine prerequisites
validate, and a fresh install is deliberately not commissioned. With the
`[plr]` section in place, commissioning happens entirely at the printer
console.

### Tap / load-cell: the 5-command happy path

```text
PLR_SETUP                          ; 1. the report — fix any [FAIL] it lists
PLR_SETUP ACCEPT_SELF_LOCKING_Z=1  ; 2. attest self-locking Z (leadscrews!)
G28                                ; 3. home (PLR_PROBE_TEST refuses unhomed)
PLR_PROBE_TEST START=1             ; 4. measure probe repeatability
SAVE_CONFIG                        ; 5. persist attestation + probe_resolution
```

Step by step:

1. **`PLR_SETUP`** prints every automated check with
   `[PASS]`/`[WARN]`/`[FAIL]` and a remediation hint per failure
   (force_move, probe section vs `probe_method`, single-MCU Z steppers,
   move-free probe g-code, a finite lower Z bound, recorder heartbeat).
   Fix any `[FAIL]` in printer.cfg and restart until the report is
   clean.
2. **`PLR_SETUP ACCEPT_SELF_LOCKING_Z=1`** stages the one attestation
   software cannot check: that your Z axis holds position unpowered
   (leadscrew drives generally do; belted-Z drives generally do
   **not**, and this system is not safe on them). Only run this if it
   is physically true of your machine.
3. **`G28`** — `PLR_PROBE_TEST` moves the toolhead and refuses while
   unhomed or printing.
4. **`PLR_PROBE_TEST START=1`** probes repeatedly at the current XY
   (default 10 samples) and stages `probe_resolution` — the trust
   radius recovery gives your probe.
5. **`SAVE_CONFIG`** writes both staged values into the `[plr]`
   autosave block and restarts Klipper.

After the restart, `PLR_SETUP` reports
`COMMISSIONED — plr is ready to protect prints` and `PLR_STATUS` shows
the plugin state plus the daemon's own report.

### adxl_drag: one extra calibration

Same flow, with `PLR_NOISE_TEST` as the calibration step (and
`accel_chip` set in `[plr]`):

```text
PLR_SETUP                          ; fix any [FAIL]
PLR_SETUP ACCEPT_SELF_LOCKING_Z=1  ; attest self-locking Z
G28                                ; home
; move the toolhead well AWAY from any printed part, at a safe Z:
PLR_NOISE_TEST START=1             ; measure the accel noise floor
SAVE_CONFIG                        ; persist attestation + noise_floor_*
```

`PLR_NOISE_TEST` measures the noise floor the drag oracle thresholds
against (still + moving captures); recovery planning refuses `adxl_drag`
without a calibrated noise floor. Re-run it after changing `drag_speed`
or the machine's mechanics — see
[operations → drag-probe notes](operations.md#operating-the-adxl-drag-oracle).
Honest status: the drag oracle's safety bounds are tested, its detection
quality is bench-unvalidated (E5).

Tunables (probe speed, envelope margin, drag knobs…) are adjusted the
same console way: `PLR_SET PARAM=<name> VALUE=<v>`, then `SAVE_CONFIG`
when you want them to survive a restart. `PLR_SET` alone lists
everything with live values and ranges.

Finally, commission **empirically**: the first execution should be a
supervised run on a scrap print after a deliberate power-cut drill —
see [operations → After a real power loss](operations.md#after-a-real-power-loss)
and the [walkthrough](../examples/recovery-walkthrough.md). The CLI
`plrd recover --step` mode (pause before every step) is worth using for
those first runs; per-step mode is CLI-only.

## Legacy: commissioning the machine section

**This is the pre-plugin path — deprecated but fully working.** Use it
only on installs that have not adopted the `[plr]` section; whenever a
`[plr]` section exists in the running Klipper config, plrd sources the
machine snapshot from the live config and this entire `[machine]`
section is **ignored** (with an info note in the recover output). One
consequence worth knowing: if klippy is unreachable at recover time,
`[plr]`-mode recovery refuses (the settings live only in the running
config), while a commissioned legacy snapshot still validates — with
its hash blessing detecting any printer.cfg change since.

On this path, `plrd recover`
assembles a machine snapshot from the `[machine]` section of
`/etc/plrd.conf` and refuses — listing every failed check — until all
prerequisites validate. Every attestation defaults to `false`, so a fresh
install is deliberately not commissioned. The legacy path supports only
`tap` and `load_cell` probes; the ADXL drag oracle exists exclusively in
`[plr]` mode (its noise-floor calibration lives in the `[plr]` autosave
block).

The keys (each maps 1:1 onto a `plr-recovery` prerequisite; the
[commissioning checklist](#commissioning-checklist) is the physical truth
you are attesting to):

| `[machine]` key | Default | Set it to |
| --- | --- | --- |
| `force_move_enabled` | `false` | `true` only after confirming `[force_move]` with `enable_force_move: True` in printer.cfg |
| `z_self_locking_attested` | `false` | `true` only if your Z leadscrews are self-locking (bed holds position unpowered) |
| `z_steppers` | `stepper_z` | every Z stepper, as `name` or `name:mcu` (bare names assume `primary_mcu`; any Z stepper on a secondary MCU is refused) |
| `primary_mcu` | `mcu` | your primary MCU's name |
| `probe_kind` | unset | `tap` (`[probe]`) or `load_cell` (`[load_cell_probe]`) |
| `probe_z_offset` | unset | the probe's configured `z_offset`, mm |
| `probe_activate_gcode_no_move` | `false` | `true` only after checking `activate_gcode` is empty or commands no motion |
| `probe_deactivate_gcode_no_move` | `false` | ditto for `deactivate_gcode` |
| `z_position_min` | unset | the Z rail's `position_min` (or `[printer] minimum_z_position`), mm |
| `klipper_config_path` | unset | your `printer.cfg` path — plrd checksums it at recover time |
| `validated_config_hash` | unset | the blessing — see below |
| `virtual_sdcard_root` | unset | the `[virtual_sdcard] path` value |

Two values are *not* configurable, on purpose: `;TYPE:` annotation presence
is observed from the actual print file, and the running config hash is
computed from `klipper_config_path` at recover time.

**The blessing flow** (change detection): after filling in the section, run

```sh
plrd recover --config /etc/plrd.conf
```

On a never-blessed config it refuses and prints the computed checksum:

```text
pipeline: machine hash computed: crc32c:658a94bb
recover: REFUSED — machine prerequisites failed:
  - machine prerequisites have never been validated
```

Re-walk the checklist against your printer.cfg, then paste the printed
value into the config:

```ini
validated_config_hash = crc32c:658a94bb
```

From then on, any edit to printer.cfg changes the computed hash and
recovery refuses until you deliberately re-validate and re-bless. (It is a
crc32c change-detection checksum — an operator gate against forgotten
edits, not a security boundary.)

The empirical-commissioning advice from the
[console section](#commissioning-from-the-console) applies unchanged:
first execution on a scrap print, supervised, with `--step`.

## Moonraker update manager integration

`scripts/install.sh --moonraker` registers plrd in Moonraker's update
manager so Mainsail/Fluidd's update panel shows and updates it like
Klipper itself. [`deploy/moonraker-update-manager.conf`](../deploy/moonraker-update-manager.conf)
is the canonical hand-edit reference; the automated steps are:

1. append an `[update_manager plrd]` `git_repo` section to
   `moonraker.conf` — the section name **must** equal the systemd unit
   name (`plrd`), because Moonraker's `managed_services` only accepts the
   section-header name (its `svc_choices`), case-sensitively;
2. allow-list the service: a line reading exactly `plrd` in
   `<printer_data>/moonraker.asvc`;
3. install a systemd drop-in
   (`/etc/systemd/system/plrd.service.d/50-plrd-refresh.conf`) with an
   `ExecStartPre` that runs `install.sh --refresh`.

What an update then actually does: Moonraker performs `git pull` and
restarts the `plrd` service — it runs **no build steps** for `git_repo`
entries. The drop-in turns that restart into "rebuild if HEAD changed,
then start": `--refresh` compares the installed binary's build stamp to
the repo HEAD, rebuilds as the repo's owner (never root-in-checkout) when
they differ, and swaps only `/usr/local/bin/plrd`. A failed build is
stamped and **not** retried automatically, and never blocks startup — the
old binary keeps running until you fix the build and clear the stamp
(`sudo rm /var/lib/plrd/build-failed-head`).

Restarting plrd is always safe: it observes Klipper read-only and never
touches a running print. Nothing in the repo ever restarts Moonraker or
Klipper — after registering, restart Moonraker yourself.

Both companion edits are backed up first, and `uninstall.sh --moonraker`
reverses them.

## First-run verification

### 1. Service is up and ready

```sh
systemctl status plrd
```

Expect `Active: active (running)` — because the unit is `Type=notify`, the
service only reaches `active` after the WAL service owns its files. Klipper
being down does **not** fail the start; you will instead see connect retries
in the log:

```sh
journalctl -u plrd -f
# plrd: cannot connect to /home/pi/printer_data/comms/klippy.sock: No such
#       file or directory (os error 2); retrying in 250ms
```

Backoff is capped exponential (250 ms → 8 s). If this persists while Klipper
is running, the `klipper_socket` path is wrong — see
[troubleshooting](operations.md#troubleshooting).

### 2. WAL files exist

Immediately after the first start, before Klipper is even connected:

```sh
sudo ls -la /var/lib/plrd/wal
# heartbeat.bin      128 bytes  (dual-slot heartbeat file)
# receive_seq.bin      0 bytes  (sidecar; filled on first counter advance)
# wal-000001.plr      32 bytes  (segment header; grows as records append)
```

The segment file grows while Klipper is connected (fast during motion,
trickle when idle); `heartbeat.bin` stays exactly 128 bytes forever and is
rewritten at 10 Hz — heartbeats are only written while the Klipper socket is
live, because a heartbeat is a liveness claim. Each daemon start creates a
fresh segment (`wal-000002.plr`, …); old segments are never touched or
deleted — that retention is yours, see
[operations](operations.md#disk-sizing-and-write-load).

### 3. Scan reads back what was recorded

Run a short test print (or just let Klipper sit connected for a minute),
then:

```sh
sudo plrd scan --wal /var/lib/plrd/wal
```

You should see each segment's record counts, the heartbeat slot, and a
reconstruction verdict. While a print is *not* in progress the verdict will
usually be `reconstruction: not possible: …` (no context snapshot — that is
correct and honest, not an error; there is nothing to recover). See
[operations.md](operations.md#reading-plrd-scan-reports) for a full
annotated report and
[examples/recovery-walkthrough.md](../examples/recovery-walkthrough.md) for a
complete synthetic power-loss scenario.

### 4. Console commands respond

With the plugin linked, `[plr]` added, and Klipper restarted, type
`PLR_STATUS` in the console. You should see the plugin block (probe
method, attestation state, tunables) followed by the daemon's own
report — WAL segment count, heartbeat age, pending recovery,
machine-config mode — fetched over the control socket. If the daemon
half instead reads `plrd not reachable at /var/lib/plrd/plrd.sock — is
the service running? (systemctl status plrd)`, see
[operations → troubleshooting](operations.md#troubleshooting).

### 5. Dry-run the crash shape (recommended)

This is the miniature, on-host version of the open empirical validation
tasks (E1–E5): while printing a scrap part, kill the daemon uncleanly
(`sudo systemctl kill -s SIGKILL plrd`), or — with the printer supervised
and a part you do not care about — cut printer power. Then run `plrd scan`
and check that:

- the newest segment ends `expected after power loss: yes`;
- the stop window and Z candidates match where the machine physically is.

Do this **before** you ever need the answer to be right.
