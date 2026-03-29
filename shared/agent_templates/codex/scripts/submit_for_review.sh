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

PROBLEMOLOGIST_REPO_ROOT="${PROBLEMOLOGIST_REPO_ROOT:-}"
if [ -n "$PROBLEMOLOGIST_REPO_ROOT" ]; then
  if [ -n "${PYTHONPATH:-}" ]; then
    export PYTHONPATH="$PROBLEMOLOGIST_REPO_ROOT:$PYTHONPATH"
  else
    export PYTHONPATH="$PROBLEMOLOGIST_REPO_ROOT"
  fi
fi

if [ -z "${XDG_CACHE_HOME:-}" ]; then
  export XDG_CACHE_HOME="$PWD/.codex-cache"
fi
if [ -z "${XDG_CONFIG_HOME:-}" ]; then
  export XDG_CONFIG_HOME="$PWD/.codex-config"
fi
if [ -z "${TMPDIR:-}" ]; then
  export TMPDIR="$PWD/.codex-tmp"
fi
if [ -z "${TEMP:-}" ]; then
  export TEMP="$TMPDIR"
fi
if [ -z "${TMP:-}" ]; then
  export TMP="$TMPDIR"
fi
if [ -z "${MPLCONFIGDIR:-}" ]; then
  export MPLCONFIGDIR="$XDG_CACHE_HOME/matplotlib"
fi
mkdir -p "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME" "$TMPDIR" "$MPLCONFIGDIR"
export IS_HEAVY_WORKER=1

exec "$PYTHON_BIN" scripts/submit_for_review.py "$@"
