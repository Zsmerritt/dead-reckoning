# Installing plrd on a Klipper host

This guide covers a Raspberry Pi (or any Linux SBC) running Klipper, from
prerequisites through first-run verification. Companion documents:
[architecture](architecture.md) for what the daemon records and why,
[operations](operations.md) for day-2 usage, and
[examples/plrd.conf](../examples/plrd.conf) for a fully commented config.

Contents:

1. [Commissioning checklist](#commissioning-checklist) — is this machine a
   candidate for recovery at all?
2. [Where each prerequisite lives in the Klipper config](#where-each-prerequisite-lives)
3. [Building plrd](#building-plrd)
4. [Installing binary, config, and systemd unit](#installing)
5. [First-run verification](#first-run-verification)

## Commissioning checklist

Recovery on a moving-bed-Z machine is only safe when the machine satisfies
structural prerequisites. These are the exact checks
`plr_recovery::validate_machine` enforces (every failed check is reported,
not just the first); walk the list **before** you rely on this system:

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
- [ ] **Exactly one Tap-style `[probe]` or `[load_cell_probe]`.** The probe
  must trigger on nozzle contact (Voron Tap, load cell). Inductive-only
  probes cannot reference the printed part and do not qualify.
- [ ] **Probe `activate_gcode` / `deactivate_gcode` are empty or verified
  move-free.** A moving activate G-code would break the halt-position
  arithmetic.
- [ ] **The Z rail's `position_min` is known** (or `[printer]
  minimum_z_position` as fallback) — it anchors the probe envelope. A small
  negative value (e.g. `-2`) is typical and fine; "unset" is not.
- [ ] **`[virtual_sdcard]` root is known**, and you print top-level files
  from it (the `M23` resume path does not descend into subdirectories).
- [ ] **Config-change discipline.** The prerequisites are validated against
  a hash of the running Klipper config; if the config changes, they must be
  re-validated before any recovery.

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
| Z `position_min` | `printer.cfg`: `position_min` under `[stepper_z]`, or `minimum_z_position` under `[printer]` |
| `[virtual_sdcard]` root | `printer.cfg`: the `path` value of `[virtual_sdcard]` (Moonraker installs: `~/printer_data/gcodes`) |
| Klipper API socket | the klippy service file (usually `/etc/systemd/system/klipper.service` or `~/printer_data/systemd/klipper.env`): the `-a <path>` argument of `klippy.py`. Moonraker installs use `~/printer_data/comms/klippy.sock`. This is the value for `klipper_socket` in `plrd.conf` |
| `;TYPE:` annotations | open a sliced file and search for `;TYPE:`; if absent, enable the feature-comment option in your slicer profile |

## Building plrd

Rust is pinned by `rust-toolchain.toml` (currently 1.97.x); rustup fetches
the right toolchain automatically on first build.

### On the printer host (tested path)

```sh
git clone https://github.com/zsmerritt/dead-reckoning.git
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

## Installing

These are the same steps the header of `deploy/plrd.service` documents:

```sh
sudo cp target/release/plrd /usr/local/bin/plrd
sudo cp deploy/plrd.conf.example /etc/plrd.conf     # then edit — see below
sudo cp deploy/plrd.service /etc/systemd/system/plrd.service
sudo systemctl daemon-reload
sudo systemctl enable --now plrd
```

### Editing `/etc/plrd.conf`

The format is flat `key = value` with `#` comments — the same family as
`printer.cfg`. **Unknown and duplicate keys are hard errors** (a misspelled
durability knob silently falling back to a default is exactly the kind of
quiet failure this project exists to prevent). Every key and its default:

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

The two values almost everyone must change are `klipper_socket` (point it at
the real klippy socket) and, on multi-Z machines, `z_steppers`. A fully
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

### 4. Dry-run the crash shape (recommended)

This is the miniature, on-host version of the open empirical validation
tasks (E1–E5): while printing a scrap part, kill the daemon uncleanly
(`sudo systemctl kill -s SIGKILL plrd`), or — with the printer supervised
and a part you do not care about — cut printer power. Then run `plrd scan`
and check that:

- the newest segment ends `expected after power loss: yes`;
- the stop window and Z candidates match where the machine physically is.

Do this **before** you ever need the answer to be right.
