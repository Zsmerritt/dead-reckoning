#!/bin/sh
# Runs the Rust gates with Linux scope — `--workspace`, so `plrd` is
# INCLUDED — and reports counts parsed from a captured log rather than
# from exit codes.
#
# Why this exists rather than "just run cargo": Windows developers'
# pre-commit hook excludes `plrd` (it is Linux-only), so a change to a
# shared crate can break `plrd` while every local gate stays green. This
# script is the Linux-side check, runnable from a Linux box, a container,
# or WSL against a Windows worktree.
#
# THE WINDOWS RUN IS NOT SUFFICIENT. Concretely, on 2026-07-24 this script
# caught six clippy diagnostics in `crates/plrd/src/detect.rs` — a
# too-similar binding and two `u64 as usize` casts, at `-D warnings` — on a
# branch whose Windows gate was clean and whose author believed it green.
# The Windows pre-commit hook passes `--exclude plrd`, so those diagnostics
# are not merely unlikely to be seen there, they are *structurally
# invisible*: the crate is never compiled. Any change touching `plrd`, or
# touching a crate `plrd` depends on, needs this script before it is called
# green. Do not conclude from a clean Windows run that this one is redundant.
#
# Two footguns it defends against. Both have produced false "green"
# reports on this project; the notes below record what was *measured* on
# the pinned toolchain, because a gate script that documents unverified
# behaviour is the same class of problem it exists to prevent.
#
#   1. Diagnostics vanishing on a repeat run. `cargo clippy` can skip
#      re-emitting warnings for units it considers fresh, so a second run
#      after a build prints nothing and reads exactly like "no warnings".
#      Measured on the toolchain this repo pins (cargo 1.97.1), the
#      behaviour did NOT reproduce: with a deliberate warning added, three
#      configurations — `-p <crate>`, `--workspace --all-targets`, and
#      `--workspace` after a `cargo test` had populated the cache — all
#      replayed the cached diagnostics on every run (3 of 3, identical
#      counts). Treat "clippy printed nothing, so it is clean" as
#      unverified rather than safe: this script never relies on silence
#      either way. It captures clippy's own invocation and counts
#      `^warning`/`^error` lines in that log, and `--clean-clippy` forces
#      `cargo clean -p` over the workspace crates when you want
#      diagnostics regenerated from scratch with certainty.
#   2. A verdict that outlives the step that failed. `$?` through the
#      Git Bash -> `wsl.exe` bridge reports the status of the *last*
#      command in the `sh -lc` string, so an earlier failure inside a
#      compound command is masked — observed directly while writing this:
#      a `git rev-parse` that failed (the worktree's `.git` file holds a
#      Windows path, meaningless inside WSL) left its variable empty while
#      the surrounding command still reported success. Anything that
#      parses such output can then "confirm" a run that never happened.
#      Every verdict below is therefore cross-checked against grep/awk
#      counts over a captured log, each gate runs as its own command with
#      its own status, and PASS is printed only when status and parsed
#      counts agree. `suites=0` is treated as failure for the same reason:
#      a test binary that never ran produces no `test result:` line, which
#      would otherwise sum to "0 failed".
#
# Usage (from the repo root, or any directory inside it):
#     sh scripts/gates-linux.sh [--clean-clippy]
#
# From Windows against a worktree, e.g.:
#     wsl.exe -e sh -lc 'cd /mnt/c/path/to/worktree \
#         && export PATH=$HOME/.cargo/bin:$PATH \
#            CARGO_TARGET_DIR=/tmp/gates-target \
#         && sh scripts/gates-linux.sh'
#
# `CARGO_TARGET_DIR` outside the Windows filesystem is strongly advised:
# sharing `target/` between Windows and Linux toolchains thrashes the
# whole cache on every switch.
set -u

cd "$(dirname "$0")/.." || exit 1

clean_clippy=0
[ "${1:-}" = "--clean-clippy" ] && clean_clippy=1

log_dir="${TMPDIR:-/tmp}/dead-reckoning-gates-$$"
mkdir -p "$log_dir" || exit 1

failures=0

fail() {
    printf 'GATE FAIL: %s\n' "$1" >&2
    failures=$((failures + 1))
}

# Counts `^warning`/`^error` lines, so a silent (fresh-crate) clippy run
# cannot be mistaken for a clean one — see footgun 1.
diagnostic_count() {
    grep -cE '^(warning|error)' "$1" 2>/dev/null || true
}

# Sums the per-suite `test result:` lines. `cargo test` prints one per
# binary, so a single suite failing while others pass is caught here even
# if the observed exit status lies — see footgun 2.
test_counts() {
    awk '/^test result:/ { passed += $4; failed += $6; suites += 1 }
         END { printf "%d %d %d", passed, failed, suites }' "$1"
}

echo "==> repo: $(pwd)"
# Resolves HEAD even for a git *worktree* whose `.git` file points at a
# Windows path, which is the normal shape when running this from WSL against a
# Windows checkout. Git inside WSL cannot follow `gitdir: C:/…` — it reads the
# drive-letter path as relative and fails — so the `C:/x` form is translated
# to `/mnt/c/x` and retried.
#
# This matters more than cosmetics: a gate report that cannot name the commit
# it ran against is exactly the ambiguity that let a false "green" claim stand
# on this project once already.
resolve_commit() {
    if git rev-parse --short HEAD 2>/dev/null; then
        return
    fi
    if [ ! -f .git ]; then
        echo '(unknown: not a git worktree)'
        return
    fi
    gitdir=$(sed -n 's/^gitdir: *//p' .git | tr -d '\r')
    case "$gitdir" in
        [A-Za-z]:[/\\]*)
            drive=$(printf '%s' "$gitdir" | cut -c1 | tr '[:upper:]' '[:lower:]')
            rest=$(printf '%s' "$gitdir" | cut -c3- | tr '\\' '/')
            git --git-dir="/mnt/$drive$rest" rev-parse --short HEAD 2>/dev/null \
                || echo "(unknown: cannot read /mnt/$drive$rest)"
            ;;
        *) echo "(unknown: gitdir $gitdir)" ;;
    esac
}

echo "==> commit: $(resolve_commit)"
echo "==> logs: $log_dir"
echo

echo "=== 1/4 cargo fmt --all --check ==="
if cargo fmt --all --check > "$log_dir/fmt.log" 2>&1; then
    echo "fmt: OK"
else
    tail -20 "$log_dir/fmt.log"
    fail "formatting (run 'cargo fmt --all')"
fi
echo

echo "=== 2/4 cargo clippy --workspace --all-targets -- -D warnings (plrd INCLUDED) ==="
if [ "$clean_clippy" -eq 1 ]; then
    echo "(--clean-clippy: cleaning workspace crates so diagnostics regenerate)"
    for crate in crates/*/; do
        name=$(basename "$crate")
        cargo clean -p "$name" > /dev/null 2>&1 || true
    done
fi
cargo clippy --workspace --all-targets -- -D warnings > "$log_dir/clippy.log" 2>&1
clippy_status=$?
clippy_diags=$(diagnostic_count "$log_dir/clippy.log")
echo "clippy: status=$clippy_status diagnostics=$clippy_diags"
if [ "$clippy_status" -ne 0 ] || [ "$clippy_diags" -ne 0 ]; then
    grep -E '^(warning|error)' "$log_dir/clippy.log" | head -20
    fail "clippy (status $clippy_status, $clippy_diags diagnostics)"
fi
echo

echo "=== 3/4 cargo test --workspace (plrd INCLUDED) ==="
cargo test --workspace > "$log_dir/test.log" 2>&1
test_status=$?
# Word splitting is the point here: test_counts prints three integers.
# shellcheck disable=SC2046
set -- $(test_counts "$log_dir/test.log")
passed=$1
failed=$2
suites=$3
echo "test: status=$test_status passed=$passed failed=$failed suites=$suites"
if [ "$test_status" -ne 0 ] || [ "$failed" -ne 0 ] || [ "$suites" -eq 0 ]; then
    grep -A20 '^failures:$' "$log_dir/test.log" | tail -21
    fail "tests (status $test_status, $failed failed across $suites suites)"
fi
echo

echo "=== 4/4 scripts/coverage.sh ==="
if sh scripts/coverage.sh > "$log_dir/coverage.log" 2>&1; then
    grep -E '^TOTAL' "$log_dir/coverage.log" || true
    echo "coverage: OK"
else
    tail -25 "$log_dir/coverage.log"
    fail "coverage gate"
fi
echo

if [ "$failures" -eq 0 ]; then
    echo "LINUX GATES: PASS (logs in $log_dir)"
    exit 0
fi
echo "LINUX GATES: FAIL ($failures gate(s)); logs in $log_dir" >&2
exit 1
