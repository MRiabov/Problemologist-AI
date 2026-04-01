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

export XDG_CACHE_HOME="$PWD/.codex-cache"
export XDG_CONFIG_HOME="$PWD/.codex-config"
export TMPDIR="$PWD/.codex-tmp"
export TEMP="$TMPDIR"
export TMP="$TMPDIR"
export MPLCONFIGDIR="$XDG_CACHE_HOME/matplotlib"
unset DISPLAY
unset XAUTHORITY
unset WAYLAND_DISPLAY
export LIBGL_ALWAYS_SOFTWARE=1
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa
export PYVISTA_OFF_SCREEN=true
export VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow
export PYGLET_HEADLESS=1
mkdir -p "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME" "$TMPDIR" "$MPLCONFIGDIR"
export IS_HEAVY_WORKER=1

exec "$PYTHON_BIN" scripts/submit_for_review.py "$@"
