#!/bin/sh
# Line-coverage gate for the pure-logic (default-member) crates.
#
# Runs under Git Bash on Windows and any POSIX sh on Linux. Requires
# cargo-llvm-cov (https://github.com/taiki-e/cargo-llvm-cov):
#     cargo install cargo-llvm-cov
#
# The gate is real: this script's exit code fails the pre-commit hook and
# CI when line coverage drops below the threshold. Do not lower the
# threshold; raise coverage instead.
set -eu

THRESHOLD=90

cd "$(dirname "$0")/.."

if ! cargo llvm-cov --version >/dev/null 2>&1; then
    echo "error: cargo-llvm-cov is not installed." >&2
    echo "       install it with: cargo install cargo-llvm-cov" >&2
    exit 1
fi

echo "==> cargo llvm-cov over workspace default members (fail under ${THRESHOLD}% lines)"
# No package/file exclusions: everything in the default members counts.
cargo llvm-cov --fail-under-lines "$THRESHOLD"
