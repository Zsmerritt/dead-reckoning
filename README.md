# dead-reckoning

Power-loss recovery (PLR) for Klipper 3D printers. When a printer loses power
mid-print, dead-reckoning's daemon has already journaled the motion state to a
durable write-ahead log; on restart it reconstructs the last known toolhead
position, temperatures, and G-code file offset, and builds a plan to resume
the print in place instead of scrapping it.

Two ideas anchor the design. First, a **motion WAL**: print progress is
recorded append-only with real durability guarantees (`fdatasync` / `O_DSYNC`),
so recovery works from what actually reached the disk, not from optimistic
state. Second, **part-referenced Z probing**: after a power loss the true Z
height is re-established by probing the printed part itself, so resume height
does not depend on steppers that lost their reference. All of this works
against Klipper's existing APIs (Moonraker) — **no Klipper patches**.

The codebase is a Rust workspace split by *platform shape*, not by trait
abstraction: pure-logic crates are cross-platform by construction, and all
syscalls and I/O live in one Linux-only daemon crate. Durability code is
**never mocked** — that is a hard project rule.

## Crate map

| Crate | Kind | Purpose |
| --- | --- | --- |
| `plr-wal` | library, pure logic | Motion write-ahead log record formats, encoding/decoding, integrity checks |
| `plr-klipper` | library, pure logic | Klipper/Moonraker API object model and protocol message parsing (no sockets) |
| `plr-gcode` | library, pure logic | G-code parsing and stream-position tracking primitives |
| `plr-reconstruct` | library, pure logic | Printer state reconstruction from a recovered WAL |
| `plr-analyzer` | library, pure logic | Printed-part geometry analysis, e.g. selecting part-referenced Z-probe points |
| `plr-recovery` | library, pure logic | Recovery planning and resume G-code generation |
| `plrd` | binary, Linux-only | The daemon: tokio, Unix sockets, durable WAL writes via rustix, systemd |

The pure-logic crates are the workspace `default-members`, so a bare
`cargo test` builds and tests exactly them on any OS. `plrd` compiles
everywhere (`cargo check -p plrd`), but on non-Linux targets it is a stub
that prints an error and exits nonzero.

## Development setup

Toolchain is pinned by `rust-toolchain.toml` (rustup installs it, plus
rustfmt/clippy/llvm-tools, automatically on first `cargo` invocation).

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

## Tests, coverage, lints

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

CI (`.github/workflows/ci.yml`) runs on push/PR: full-workspace lint and
tests on Linux, default-member tests on Windows, and the coverage gate.
