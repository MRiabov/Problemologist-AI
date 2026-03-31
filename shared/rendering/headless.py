from __future__ import annotations

import os
from functools import lru_cache
from typing import Any

import structlog

logger = structlog.get_logger(__name__)


def configure_headless_vtk_egl() -> None:
    """Configure a process for headless EGL-based VTK rendering."""

    os.environ.pop("DISPLAY", None)
    os.environ.pop("XAUTHORITY", None)
    os.environ.pop("WAYLAND_DISPLAY", None)
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
    os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")
    os.environ.setdefault("VTK_DEFAULT_OPENGL_WINDOW", "vtkOSOpenGLRenderWindow")
    os.environ.setdefault("PYGLET_HEADLESS", "1")


@lru_cache(maxsize=1)
def _headless_vtk_render_window_class() -> type[Any]:
    requested = os.environ.get("VTK_DEFAULT_OPENGL_WINDOW", "").strip()
    if requested:
        if requested == "vtkEGLRenderWindow":
            try:
                from vtkmodules.vtkRenderingOpenGL2 import vtkEGLRenderWindow

                logger.info(
                    "headless_vtk_render_window_class_selected",
                    requested=requested,
                    selected="vtkEGLRenderWindow",
                )
                return vtkEGLRenderWindow
            except Exception:
                pass
        elif requested == "vtkOSOpenGLRenderWindow":
            try:
                from vtkmodules.vtkRenderingOpenGL2 import vtkOSOpenGLRenderWindow

                logger.info(
                    "headless_vtk_render_window_class_selected",
                    requested=requested,
                    selected="vtkOSOpenGLRenderWindow",
                )
                return vtkOSOpenGLRenderWindow
            except Exception:
                pass
        elif requested == "vtkRenderWindow":
            from vtkmodules.vtkRenderingCore import vtkRenderWindow

            logger.info(
                "headless_vtk_render_window_class_selected",
                requested=requested,
                selected="vtkRenderWindow",
            )
            return vtkRenderWindow

    try:
        from vtkmodules.vtkRenderingOpenGL2 import vtkEGLRenderWindow

        logger.info(
            "headless_vtk_render_window_class_selected",
            requested=requested or None,
            selected="vtkEGLRenderWindow",
        )
        return vtkEGLRenderWindow
    except Exception:
        try:
            from vtkmodules.vtkRenderingOpenGL2 import vtkOSOpenGLRenderWindow

            logger.info(
                "headless_vtk_render_window_class_selected",
                requested=requested or None,
                selected="vtkOSOpenGLRenderWindow",
            )
            return vtkOSOpenGLRenderWindow
        except Exception:
            from vtkmodules.vtkRenderingCore import vtkRenderWindow

            logger.info(
                "headless_vtk_render_window_class_selected",
                requested=requested or None,
                selected="vtkRenderWindow",
            )
            return vtkRenderWindow


def create_headless_vtk_render_window() -> Any:
    """Create a headless VTK render window, honoring the requested class."""

    window = _headless_vtk_render_window_class()()
    window.SetOffScreenRendering(True)
    return window
