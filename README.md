# dead-reckoning

**Power-loss recovery for Klipper 3D printers.** If your printer loses power
twenty hours into a twenty-four-hour print, dead-reckoning lets you resume
the print in place instead of throwing the part away.

It works like a flight recorder. While you print, a small background service
(`plrd`) runs on your printer's Linux host next to Klipper and continuously
journals what the printer is doing — position, file progress, temperatures —
to disk, in a way that survives having the power yanked mid-write. After a
power loss, its tools read that journal back, work out where the print
stopped, and build a step-by-step, checked recovery plan: re-find true Z by
gently touching the *printed part itself* with the nozzle (so recovery does
not depend on motors that lost their reference), re-heat, and resume from
the right line of G-code. No Klipper patches, no custom firmware — it talks
to Klipper's existing API socket.

**Honest status (v1):** the recorder, the reconstruction, and the plan
generator are implemented and tested. Automatic *execution* of a plan is
deliberately not enabled yet — you inspect the generated plan and drive the
recovery yourself. See [Project status](#project-status-v1).

## Quick install

Prerequisites: a moving-bed-Z printer that meets the
[project requirements](#project-requirements), and a Linux host (e.g.
Raspberry Pi) running Klipper.

```sh
git clone https://github.com/Zsmerritt/dead-reckoning.git
cd dead-reckoning
cargo build --release -p plrd
sudo cp target/release/plrd /usr/local/bin/plrd
sudo cp deploy/plrd.conf.example /etc/plrd.conf     # then edit — at minimum klipper_socket
sudo cp deploy/plrd.service /etc/systemd/system/plrd.service
sudo systemctl daemon-reload && sudo systemctl enable --now plrd
```

Then check it is recording (`systemctl status plrd`, WAL files appearing
under `/var/lib/plrd/wal`) and, after any unclean stop, inspect what the
journal knows:

```sh
plrd scan --wal /var/lib/plrd/wal
```

**The full guide — commissioning checklist, config editing, first-run
verification, cross-compiling — is [docs/install.md](docs/install.md).**
A complete worked example (record → power cut → scan → recovery plan) is
[examples/recovery-walkthrough.md](examples/recovery-walkthrough.md).

## Project requirements

dead-reckoning targets a specific machine class and refuses to plan a
recovery for anything else (`plr-recovery` validates all of this and reports
every failed check). You need:

- **A moving-bed-Z printer** (bed rises into a fixed gantry — Voron
  2.4/Trident class). Recovery **never re-homes Z**; the whole safety story
  is built around that.
- **A nozzle-contact probe: a Tap-style `[probe]` or a
  `[load_cell_probe]`** — required in v1. The probe must trigger on nozzle
  contact so it can reference the printed part; inductive-only probes do not
  qualify. **ADXL "drag probing" is NOT supported yet** (explicitly
  deferred). Probe `activate_gcode`/`deactivate_gcode` must be empty or
  verified move-free.
- **`[force_move]` with `enable_force_move: True`** in `printer.cfg`.
- **Self-locking Z leadscrews** (the bed must hold position unpowered) —
  an operator attestation; software cannot check this.
- **All Z steppers on the primary MCU** (multi-MCU Z is refused).
- **Slicer `;TYPE:` feature annotations** in your G-code — OrcaSlicer and
  PrusaSlicer (and SuperSlicer/Cura) emit them by default; recovery refuses
  to pick a probe point without them.
- **A known Z `position_min`** (or `[printer] minimum_z_position`) — it
  anchors the probe envelope.
- A Linux host for the daemon (the offline analysis tools run anywhere).

Out of scope in v1: multi-extruder machines.

## Configuration

Two configs matter — the daemon's and Klipper's:

- **`/etc/plrd.conf`** (flat `key = value`): every key, default, and valid
  range is tabulated in
  [docs/install.md → Editing /etc/plrd.conf](docs/install.md#editing-etcplrdconf).
  A fully commented example for a Voron-style multi-Z machine:
  [examples/plrd.conf](examples/plrd.conf). Most installs change only
  `klipper_socket` and (multi-Z) `z_steppers`.
- **`printer.cfg` prerequisites**: what each requirement above looks like in
  your Klipper config, and where to find it —
  [docs/install.md → Where each prerequisite lives](docs/install.md#where-each-prerequisite-lives).

## Understanding the logs

Two kinds of output tell you what the system is doing:

- **Daemon logs** — `plrd` logs to stderr, which systemd routes to journald
  (`journalctl -u plrd -f`). Every message the daemon emits (connect/backoff
  lines, klippy state changes, unparseable-payload warnings, skipped
  records) is cataloged in
  [docs/operations.md → Understanding plrd's logs](docs/operations.md#understanding-plrds-logs).
- **Scan reports** — `plrd scan` prints the post-mortem analysis; the
  line-by-line field guide is
  [docs/operations.md → Reading plrd scan reports](docs/operations.md#reading-plrd-scan-reports).

---

## How it works (the deeper detail)

Two ideas anchor the design. First, a **motion WAL** (write-ahead log):
print progress is recorded append-only with real durability guarantees
(`fdatasync` / `O_DSYNC`), so recovery works from what actually reached the
disk, not from optimistic state. Second, **part-referenced Z probing**:
after a power loss the true Z height is re-established by probing the
printed part itself, so resume height does not depend on steppers that lost
their reference.

```mermaid
flowchart LR
    subgraph printer ["While printing (Linux host, plrd daemon)"]
        K["Klipper API socket<br/>(motion_report dumps,<br/>status updates)"] --> C["plrd client task<br/>(async, never blocks)"]
        C -->|bounded channel| W["plrd WAL thread<br/>(fdatasync / O_DSYNC)"]
        W --> S["WAL segments<br/>(append-only, CRC32C)"]
        W --> H["heartbeat.bin<br/>(dual-slot, 10 Hz)"]
        W --> Q["receive_seq.bin<br/>(sidecar)"]
    end
    subgraph recovery ["After power loss (offline, any OS)"]
        S --> SCAN["plrd scan<br/>(plr-wal: torn-tail recovery)"]
        H --> SCAN
        Q --> SCAN
        G["printed .gcode file"] --> SCAN
        SCAN --> R["plr-reconstruct<br/>possible-stop set"]
        R --> A["plr-analyzer<br/>stop match + contact zone"]
        A --> P["plr-recovery<br/>typed, verifiable plan"]
        P --> X["execution<br/>(v1: scaffold — refuses;<br/>plans are for inspection)"]
    end
```

After an unclean stop, the offline pipeline reconstructs not a point
estimate but a **possible-stop set** — every state the machine can
plausibly have stopped in — because Klipper's motion dumps batch at ~0.5 s
and step generation runs ahead of execution, so the durable log can end
before the machine actually stopped. The Z projection of that set is exact
and enumerable; it sizes the **probe envelope** for a single, structurally
bounded probe of the printed part that re-establishes true Z without ever
re-homing Z. The full story, including the on-disk formats and the math:
[docs/architecture.md](docs/architecture.md).

## Safety guarantees — and their limits

Documented in full in [docs/architecture.md](docs/architecture.md). The
summary, stated as honestly as the code states it:

- **Possible-stop-set containment.** The true stop state is always contained
  in the reconstructed set — enforced by a fault-injection property test that
  synthesizes WALs with honest 0.5 s batch flushing, torn tails, and random
  power-cut points. *Limit:* if the printed G-code file is unavailable at
  scan time, the forward-simulated extension cannot run and the guarantee is
  **void for true power loss**; this is reported as a typed degradation, never
  silently.
- **Z is exact; the descent is structurally bounded.** The probe move happens
  inside a **shifted frame** declared via `SET_KINEMATIC_POSITION`, sized by
  the envelope formula (`expected_gap + 0.15 s × probe_speed + margin`), so
  Klipper's own rail-limit checking bounds the descent even with a faulty or
  disconnected probe. Probe speed is hard-capped to 1–2 mm/s (out-of-band
  speeds are rejected, never clamped). *Limit:* the bound protects the
  descent, not the measurement — a probe that triggers falsely still yields a
  wrong datum, which is why the plan verifies every step and aborts on any
  failed verification.
- **Z is NEVER re-homed.** On a moving-bed-Z machine there is no Z homing
  move that does not risk driving the bed into the nozzle. Plans home XY only
  (`G28 X Y`), verify that no `G28` appears after the shifted-frame
  declaration, and a guard scan strips `G28` / `Z_TILT_ADJUST` /
  `QUAD_GANTRY_LEVEL` from any user macro text recovery would execute.
- **Abort-only failure policy.** Every plan step carries machine-readable
  verification predicates; the executor contract is to never continue past a
  failed verification. There is no "retry and hope" path in v1.
- **Honest degradation.** Subscription gaps, missing file tails, adaptive
  (non-restorable) bed meshes, unparseable lines — all surface as typed flags
  or plan warnings, and several degrade to a **typed manual-recovery
  fallback** (vase mode, single wall, layer-only match…) rather than an
  unsafe automatic attempt.

## Crate map

The codebase is a Rust workspace split by *platform shape*, not by trait
abstraction: pure-logic crates are cross-platform by construction, and all
syscalls and I/O live in one Linux-only daemon crate. Durability code is
**never mocked** — that is a hard project rule.

| Crate | Kind | Purpose |
| --- | --- | --- |
| `plr-wal` | library, pure logic | Motion write-ahead log record formats, encoding/decoding, integrity checks |
| `plr-klipper` | library, pure logic | Klipper API-socket object model and protocol message parsing (no sockets) |
| `plr-gcode` | library, pure logic | G-code parsing, `gcode_move` state simulation, forward motion simulation |
| `plr-reconstruct` | library, pure logic | Possible-stop-set reconstruction from a recovered WAL |
| `plr-analyzer` | library, pure logic | Layer modeling, stop-point matching, contact-zone selection for the Z probe |
| `plr-recovery` | library, pure logic | Machine-prerequisite validation, probe-envelope arithmetic, recovery-plan generation |
| `plrd` | binary, Linux-only | The daemon: tokio, Unix sockets, durable WAL writes via rustix, systemd |

The pure-logic crates are the workspace `default-members`, so a bare
`cargo test` builds and tests exactly them on any OS. `plrd` compiles
everywhere (`cargo check -p plrd`), but on non-Linux targets the daemon is a
stub that prints an error and exits with code 3; the offline subcommands
(`plrd scan`, `plrd version`) work on every platform.

## Project status (v1)

Implemented and tested:

- The recorder daemon with its durability contract, including a
  SIGKILL crash-consistency integration test of the ack ⇒ durable ordering.
- Offline scan + reconstruction (`plrd scan`), cross-platform.
- The full planning pipeline (reconstruct → analyze → plan), property-tested
  and golden-tested, producing rendered, human-reviewable plans.

Deliberately **not** in v1:

- **Plan execution is a scaffold that refuses to run.** The Moonraker
  executor (`crates/plrd/src/executor.rs`) pins down the shape but returns
  `NotImplemented` unconditionally; executing a recovery moves a hot nozzle
  around a solidified print and ships only together with its own safety
  review. In v1 you generate and inspect plans; execution is manual (see
  [docs/operations.md](docs/operations.md#after-a-real-power-loss)).
- ADXL drag probing is deferred; multi-extruder machines are out of scope.
- The MCU `CLOCK_FREQ` is not journaled yet, so reconstruction falls back to
  Klipper-converted step times (reported as a `NoMcuFrequency` anomaly).
- Empirical validation tasks **E1–E5** — real-hardware fault injection, probe
  repeatability on infill, WAL write-load measurement — are open. They are
  the recommended commissioning steps for any machine that intends to trust
  a recovery: see [docs/install.md](docs/install.md#commissioning-checklist).

## Local development setup

Toolchain is pinned by `rust-toolchain.toml` (rustup installs it, plus
rustfmt/clippy/llvm-tools, automatically on first `cargo` invocation).
See [CONTRIBUTING.md](CONTRIBUTING.md) for the contribution workflow and
quality bar.

### Windows (native) — logic crates

Everything except `plrd`'s real runtime develops natively on Windows:

```sh
cargo test            # default members: all pure-logic crates
cargo check -p plrd   # compiles the non-Linux stub path
```

### WSL / Linux — plrd

`plrd` targets Linux (Unix sockets, `fdatasync`/`O_DSYNC`, systemd). Develop
and test it under WSL2 or a Linux box. **Clone the repo inside the WSL
filesystem (e.g. `~/src/dead-reckoning`), not under `/mnt/c`** — the 9p mount
of the Windows drive does not give real fsync semantics, and durability code
is never tested against fakes:

```sh
cargo test --workspace   # includes plrd
```

### One-time hook setup (all platforms)

```sh
sh scripts/setup-hooks.sh
```

This sets `git config core.hooksPath .githooks`. Every commit then runs, in
order and fail-fast: `cargo fmt --all --check`, clippy with `-D warnings`
(full workspace on Linux; `--exclude plrd` elsewhere), `cargo test`, and the
coverage gate.

### Tests, coverage, lints

```sh
cargo test                               # pure-logic crates (any OS)
cargo test --workspace                   # + plrd (Linux)
cargo fmt --all --check                  # formatting
cargo clippy --workspace --exclude plrd --all-targets -- -D warnings
sh scripts/coverage.sh                   # line-coverage gate, >=90%
```

Coverage uses [`cargo-llvm-cov`](https://github.com/taiki-e/cargo-llvm-cov)
(`cargo install cargo-llvm-cov` once). The script prints the summary table
and exits nonzero below 90% line coverage — the same gate CI enforces over
the full workspace on Linux. There are no exclusions; do not lower the
threshold, raise coverage.

Test fixtures live in `fixtures/synthetic/` (checked in, exercised by the
`plr-gcode`/`plr-analyzer` fixture tests) and `fixtures/real/` (drop any real
sliced `.gcode` files there; the fixture tests auto-discover them and run the
full parse + simulate + Z-scan pipeline over each).

CI ([`.github/workflows/ci.yml`](.github/workflows/ci.yml),
[runs](https://github.com/Zsmerritt/dead-reckoning/actions)) executes on
push/PR: full-workspace lint and tests on Linux, default-member tests on
Windows, and the coverage gate.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.
