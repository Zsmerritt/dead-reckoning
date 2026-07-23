#!/bin/sh
# One-time developer setup: point git at the versioned hooks directory.
set -eu

cd "$(dirname "$0")/.."

git config core.hooksPath .githooks
echo "core.hooksPath set to .githooks — pre-commit gate is now active."
