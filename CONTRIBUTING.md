# Contributing to dead-reckoning

Thanks for considering a contribution. This project journals safety-critical
printer state and plans motion around hot nozzles and solidified parts, so
the quality bar is deliberately strict and enforced by tooling — most of
"how to contribute" is "let the gates run".

## Setup

1. Clone and build (the toolchain is pinned by `rust-toolchain.toml`;
   rustup installs it, plus rustfmt/clippy/llvm-tools, automatically):

   ```sh
   git clone https://github.com/Zsmerritt/dead-reckoning.git
   cd dead-reckoning
   cargo test        # default members: the pure-logic crates, any OS
   ```

2. **Install the hooks (required, one-time):**

   ```sh
   sh scripts/setup-hooks.sh
   ```

   This sets `core.hooksPath` to `.githooks`. Every commit then runs,
   fail-fast: `cargo fmt --all --check`, clippy with `-D warnings` (full
   workspace on Linux, `--exclude plrd` elsewhere), `cargo test`, the
   ≥90% line-coverage gate, then the klippy-plugin gates — `ruff check`,
   `ruff format --check`, and pytest with its own ≥90% line-coverage
   gate over `klippy_plugin/plr`. If a hook fails, fix the cause — do
   not bypass hooks.

3. Install the coverage tool once: `cargo install cargo-llvm-cov`.

4. **Set up the python dev environment (required, one-time):**

   ```sh
   sh scripts/setup-py.sh
   ```

   This creates `.venv` at the repo root with the pinned dev deps from
   `klippy_plugin/requirements-dev.txt` (pytest, pytest-cov, coverage,
   ruff). The pre-commit hook *fails* — it never skips — when `.venv`
   is missing, so this step is not optional. Needs Python ≥ 3.9 on
   PATH; works from Git Bash on Windows and from Linux/WSL.

5. Working on `plrd` (the Linux-only daemon)? Develop under Linux or WSL2,
   with the clone **inside the WSL filesystem** (not `/mnt/c` — the 9p
   mount does not give real fsync semantics, and durability code is never
   tested against fakes). `cargo test --workspace` there includes the
   daemon's real-fsync and SIGKILL crash-consistency tests. See
   [README → Local development setup](README.md#local-development-setup).

## Quality bar

All of this is enforced by the pre-commit hook and CI; listed here so you
know what you are signing up for:

- **Formatting**: `rustfmt`, checked (`cargo fmt --all --check`).
- **Lints**: `clippy::all` **and `clippy::pedantic`** at deny-warnings in
  the gates. The few pedantic allows are workspace-level and individually
  justified in `Cargo.toml`; new `#[allow]`s need a justification comment
  at the site. `unsafe_code` is denied workspace-wide; `missing_docs`
  warns — public items get doc comments.
- **Coverage**: ≥ 90% line coverage over the workspace, no exclusions
  (`scripts/coverage.sh` locally, same gate in CI). Do not lower the
  threshold; raise coverage.
- **Python (klippy_plugin)**: the same bar, python-shaped — `ruff check`
  (pyflakes, pycodestyle, bugbear, isort) and `ruff format --check`,
  plus pytest with ≥ 90% line coverage over `klippy_plugin/plr`
  (`scripts/coverage-py.sh` locally, same gate in CI on 3.9 and 3.12).
  Dev deps are pinned exactly (`klippy_plugin/requirements-dev.txt`).
  Version split: plugin *source* stays Python 3.7 syntax-compatible
  (it runs inside klippy); the *dev tooling* floor is 3.9. Tests fake
  only klippy glue (`tests/fake_klippy.py`) — never physics, timing, or
  durability.
- **Tests**: unit tests beside the code, property tests (proptest) and
  golden tests under `tests/`. Conventions that matter:
  - **Proptest regression persistence**: failure seeds are persisted next
    to the test source (`FileFailurePersistence::WithSource`, files named
    `*.proptest-regressions`) and are **checked in**. If your change makes
    proptest find a failure, the seed file is part of the fix's history —
    commit it; never delete seeds to make a failure go away.
  - **Golden plan output** (`crates/plr-recovery/tests/golden/`):
    regenerate with `PLR_BLESS=1 cargo test -p plr-recovery --test golden`
    only for *intentional* rendering changes, and review the diff.
  - **Durability is never mocked.** Tests of sync behavior run real
    syscalls on Linux; there are no fake-fsync abstractions to test
    against, by design.
- **Dependencies**: the workspace policy is "no new external deps" without
  a strong case; versions live in `[workspace.dependencies]` only.
- **Line endings**: LF everywhere (enforced by `.gitattributes`); shell
  scripts and hooks must stay LF or they break under Git Bash.

## Branch / PR flow

- `main` is the integration branch and is gated by CI; work on topic
  branches (`fix/...`, `feat/...`, `docs/...`) and open a pull request
  against `main`.
- Keep commits scoped and messages explanatory — this repo's history
  favors "what and why" bodies (see `git log`). Toolchain bumps go in
  their own commit.
- CI ([`.github/workflows/ci.yml`](.github/workflows/ci.yml)) must be
  green: fmt + clippy over the full workspace and `cargo test --workspace`
  on Linux (the authority for `plrd`), `cargo test` of the default members
  on Windows, the ≥90% coverage gate on Linux, and the klippy-plugin
  job (ruff + pytest with its ≥90% gate, python 3.9 and 3.12). The pinned toolchain
  comes from `rust-toolchain.toml` in CI too, so "works locally" and
  "works in CI" mean the same compiler.
- Safety-relevant changes (anything touching `plr-recovery` plan
  generation, envelope math, machine validation, or the WAL durability
  path) should say in the PR body what invariant they preserve and which
  test proves it.

## License of contributions

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as in [README → License](README.md#license)
(MIT OR Apache-2.0), without any additional terms or conditions.
