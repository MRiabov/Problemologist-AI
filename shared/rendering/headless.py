from __future__ import annotations

from functools import lru_cache
from typing import Any

import structlog

from shared.runtime.headless import (
    HeadlessGLBackend,
    load_headless_opengl_config,
)
from shared.runtime.headless import (
    configure_headless_rendering as _configure_headless_rendering,
)
from shared.runtime.headless import (
    configure_headless_runtime as _configure_headless_runtime,
)

logger = structlog.get_logger(__name__)


def configure_headless_rendering() -> Any:
    """Configure the process for the selected headless render backend."""

    config = _configure_headless_rendering()
    _headless_vtk_render_window_class.cache_clear()
    return config


def configure_headless_runtime() -> Any:
    """Configure both MuJoCo and renderer headless backends."""

    config = _configure_headless_runtime()
    _headless_vtk_render_window_class.cache_clear()
    return config


def configure_headless_vtk_egl() -> None:
    """Backwards-compatible alias for the renderer headless bootstrap."""

    configure_headless_rendering()


@lru_cache(maxsize=1)
def _headless_vtk_render_window_class() -> type[Any]:
    config = load_headless_opengl_config()
    requested = config.rendering

    if requested is HeadlessGLBackend.EGL:
        try:
            from vtkmodules.vtkRenderingOpenGL2 import vtkEGLRenderWindow

            logger.info(
                "headless_vtk_render_window_class_selected",
                requested=requested.value,
                selected="vtkEGLRenderWindow",
            )
            return vtkEGLRenderWindow
        except Exception:
            pass

    if requested is HeadlessGLBackend.OSMESA:
        try:
            from vtkmodules.vtkRenderingOpenGL2 import vtkOSOpenGLRenderWindow

            logger.info(
                "headless_vtk_render_window_class_selected",
                requested=requested.value,
                selected="vtkOSOpenGLRenderWindow",
            )
            return vtkOSOpenGLRenderWindow
        except Exception:
            pass

    try:
        from vtkmodules.vtkRenderingCore import vtkRenderWindow

        logger.info(
            "headless_vtk_render_window_class_selected",
            requested=requested.value,
            selected="vtkRenderWindow",
        )
        return vtkRenderWindow
    except Exception:
        # Fall back to OSMesa if the requested backend could not be imported.
        from vtkmodules.vtkRenderingOpenGL2 import vtkOSOpenGLRenderWindow

        logger.info(
            "headless_vtk_render_window_class_selected",
            requested=requested.value,
            selected="vtkOSOpenGLRenderWindow",
        )
        return vtkOSOpenGLRenderWindow


def create_headless_vtk_render_window() -> Any:
    """Create a headless VTK render window using the configured backend."""

    window = _headless_vtk_render_window_class()()
    window.SetOffScreenRendering(True)
    return window
