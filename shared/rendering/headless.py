from __future__ import annotations

import os
from functools import lru_cache
from typing import Any


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


@lru_cache(maxsize=1)
def _headless_vtk_render_window_class() -> type[Any]:
    requested = os.environ.get("VTK_DEFAULT_OPENGL_WINDOW", "").strip()
    if requested:
        if requested == "vtkEGLRenderWindow":
            try:
                from vtkmodules.vtkRenderingOpenGL2 import vtkEGLRenderWindow

                return vtkEGLRenderWindow
            except Exception:
                pass
        elif requested == "vtkOSOpenGLRenderWindow":
            try:
                from vtkmodules.vtkRenderingOpenGL2 import vtkOSOpenGLRenderWindow

                return vtkOSOpenGLRenderWindow
            except Exception:
                pass
        elif requested == "vtkRenderWindow":
            from vtkmodules.vtkRenderingCore import vtkRenderWindow

            return vtkRenderWindow

    try:
        from vtkmodules.vtkRenderingOpenGL2 import vtkEGLRenderWindow

        return vtkEGLRenderWindow
    except Exception:
        try:
            from vtkmodules.vtkRenderingOpenGL2 import vtkOSOpenGLRenderWindow

            return vtkOSOpenGLRenderWindow
        except Exception:
            from vtkmodules.vtkRenderingCore import vtkRenderWindow

            return vtkRenderWindow


def create_headless_vtk_render_window() -> Any:
    """Create a headless VTK render window, honoring the requested class."""

    window = _headless_vtk_render_window_class()()
    window.SetOffScreenRendering(True)
    return window
