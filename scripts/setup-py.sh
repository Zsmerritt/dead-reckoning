#!/bin/sh
# One-time developer setup for the python gates: create .venv at the
# repo root and install the pinned dev deps from
# klippy_plugin/requirements-dev.txt. The pre-commit hook and
# scripts/coverage-py.sh use this environment; CI installs the same pins
# directly via actions/setup-python.
#
# Requires python >= 3.9 (the DEV-TOOLING floor; plugin SOURCE stays 3.7
# syntax-compatible — see klippy_plugin/pyproject.toml). POSIX sh only:
# must run under Git Bash on Windows (Scripts/ venv layout) and Linux/WSL
# (bin/ layout) unchanged.
set -eu

cd "$(dirname "$0")/.."

py=""
for candidate in python3 python; do
    if command -v "$candidate" >/dev/null 2>&1; then
        py="$candidate"
        break
    fi
done
if [ -z "$py" ]; then
    echo "error: no python3/python found on PATH (need Python >= 3.9)." >&2
    exit 1
fi

if ! "$py" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)'; then
    echo "error: $py is $("$py" -c 'import platform; print(platform.python_version())') — the dev tooling needs >= 3.9." >&2
    exit 1
fi

echo "==> creating .venv with $py ($("$py" -c 'import platform; print(platform.python_version())'))"
"$py" -m venv .venv

# Windows venvs use Scripts/, POSIX venvs use bin/.
if [ -x ".venv/bin/python" ]; then
    vpy=".venv/bin/python"
elif [ -x ".venv/Scripts/python.exe" ]; then
    vpy=".venv/Scripts/python.exe"
else
    echo "error: .venv was created but no python was found in it." >&2
    exit 1
fi

echo "==> installing pinned dev deps (klippy_plugin/requirements-dev.txt)"
"$vpy" -m pip install --quiet --upgrade pip
"$vpy" -m pip install --quiet -r klippy_plugin/requirements-dev.txt

echo "python dev environment ready: .venv (used by the pre-commit hook and scripts/coverage-py.sh)"
