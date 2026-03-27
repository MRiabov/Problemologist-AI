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
if [ -z "${MPLCONFIGDIR:-}" ]; then
  export MPLCONFIGDIR="$XDG_CACHE_HOME/matplotlib"
fi
export IS_HEAVY_WORKER=1

if [ -z "${DISPLAY:-}" ] && command -v xvfb-run >/dev/null 2>&1; then
  exec xvfb-run -a -s "-screen 0 1280x1024x24" "$PYTHON_BIN" scripts/submit_for_review.py "$@"
fi

exec "$PYTHON_BIN" scripts/submit_for_review.py "$@"
