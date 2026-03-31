from __future__ import annotations

import os


def configure_headless_vtk_egl() -> None:
    """Configure a process for headless EGL-based VTK rendering."""

    os.environ.pop("DISPLAY", None)
    os.environ.pop("XAUTHORITY", None)
    os.environ.pop("WAYLAND_DISPLAY", None)
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")
    os.environ.setdefault("VTK_DEFAULT_OPENGL_WINDOW", "vtkEGLRenderWindow")
    os.environ.setdefault("PYGLET_HEADLESS", "1")
