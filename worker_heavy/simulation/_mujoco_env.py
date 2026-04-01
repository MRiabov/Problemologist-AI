from __future__ import annotations

import os


def ensure_headless_mujoco() -> None:
    """Default MuJoCo to a headless EGL renderer when no override is set."""
    os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ["PYOPENGL_PLATFORM"] = "egl"
