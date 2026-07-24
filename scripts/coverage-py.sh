#!/bin/sh
# Line-coverage gate for the klippy plugin (klippy_plugin/plr).
#
# A sibling of coverage.sh rather than an extension of it, on purpose:
# coverage.sh is the Rust gate and is invoked by the Rust-only CI
# coverage job, while this script is invoked by the python CI matrix job
# (which has no Rust toolchain). The pre-commit hook runs both. Same
# discipline as the Rust gate: this script's exit code IS the gate — it
# fails the hook and CI when line coverage over plr/ drops below the
# threshold. Do not lower the threshold; raise coverage instead.
#
# Interpreter resolution, in order:
#   1. $PYTHON, if set (CI sets this to the matrix interpreter);
#   2. the repo .venv (both layouts: Scripts/ on Windows, bin/ on POSIX);
#   3. hard failure with setup instructions — a missing environment is
#      an error, never a silent skip.
set -eu

THRESHOLD=90

cd "$(dirname "$0")/.."
repo_root=$(pwd)

if [ -n "${PYTHON:-}" ]; then
    py="$PYTHON"
elif [ -x "$repo_root/.venv/bin/python" ]; then
    py="$repo_root/.venv/bin/python"
elif [ -x "$repo_root/.venv/Scripts/python.exe" ]; then
    py="$repo_root/.venv/Scripts/python.exe"
else
    echo "error: no python environment for the coverage gate." >&2
    echo "       create one with: sh scripts/setup-py.sh" >&2
    exit 1
fi

echo "==> pytest over klippy_plugin (fail under ${THRESHOLD}% lines of plr/)"
cd "$repo_root/klippy_plugin"
"$py" -m pytest --cov=plr --cov-report=term-missing --cov-fail-under="$THRESHOLD"
