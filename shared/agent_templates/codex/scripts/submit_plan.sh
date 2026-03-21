#!/bin/sh
set -eu

# Thin wrapper around the Python submission helper.
# The shell script is the agent-facing command surface; Python does the validation.
PYTHON_BIN="${PYTHON_BIN:-}"
if [ -z "$PYTHON_BIN" ]; then
  PYTHON_BIN="$(command -v python 2>/dev/null || true)"
fi
if [ -z "$PYTHON_BIN" ]; then
  PYTHON_BIN="$(command -v python3 2>/dev/null || true)"
fi
if [ -z "$PYTHON_BIN" ]; then
  echo "Error: unable to locate python3 or python on PATH" >&2
  exit 127
fi

exec "$PYTHON_BIN" scripts/submit_plan.py "$@"
