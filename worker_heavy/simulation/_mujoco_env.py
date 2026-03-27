from __future__ import annotations

import os


def ensure_headless_mujoco() -> None:
    """Default MuJoCo to a headless EGL renderer when no override is set."""
    os.environ.setdefault("MUJOCO_GL", "egl")
