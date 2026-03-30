from __future__ import annotations

import os
import subprocess
import sys

import structlog

logger = structlog.get_logger(__name__)

_VTK_DISPLAY_PROBE = """
from __future__ import annotations

import os
import sys

display = sys.argv[1]
xauthority = sys.argv[2] if len(sys.argv) > 2 else ""

os.environ["DISPLAY"] = display
if xauthority:
    os.environ["XAUTHORITY"] = xauthority
else:
    os.environ.pop("XAUTHORITY", None)

import vtk

window = vtk.vtkRenderWindow()
window.SetOffScreenRendering(True)
window.SetSize(1, 1)
renderer = vtk.vtkRenderer()
window.AddRenderer(renderer)
window.Render()
"""


def _probe_vtk_display(
    display: str, xauthority: str | None = None
) -> tuple[bool, str | None]:
    """Return whether VTK can connect to the ambient X display."""

    env = os.environ.copy()
    env["DISPLAY"] = display
    if xauthority:
        env["XAUTHORITY"] = xauthority
    else:
        env.pop("XAUTHORITY", None)

    try:
        completed = subprocess.run(
            [sys.executable, "-c", _VTK_DISPLAY_PROBE, display, xauthority or ""],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            check=False,
            timeout=15.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False, "VTK probe failed to launch"

    if completed.returncode == 0:
        return True, None

    stderr = (
        completed.stderr.decode("utf-8", errors="replace") if completed.stderr else ""
    )
    lines = [line for line in stderr.splitlines() if line.strip()]
    error_tail = (
        "\n".join(lines[-12:])
        if lines
        else f"VTK probe exited with {completed.returncode}"
    )
    return False, error_tail


def ensure_headless_vtk_display() -> str:
    """Validate the ambient X display for VTK rendering.

    Private Xvfb startup and scanned display fallbacks were removed. The
    renderer must run against the host's actual display/session auth.
    """

    display = os.environ.get("DISPLAY")
    if not display:
        raise ValueError(
            "deprecated functionality removed: private Xvfb fallback; ambient DISPLAY is required"
        )

    xauthority = os.environ.get("XAUTHORITY")
    if not xauthority:
        raise ValueError(
            "deprecated functionality removed: private Xvfb fallback; ambient XAUTHORITY is required"
        )

    probe_ok, probe_error = _probe_vtk_display(display, xauthority)
    if not probe_ok:
        logger.error(
            "ambient_vtk_display_rejected",
            display=display,
            xauthority=True,
            probe_error=probe_error,
        )
        raise ValueError(
            "deprecated functionality removed: private Xvfb fallback; ambient DISPLAY "
            f"{display!r} is unusable: {probe_error}"
        )

    os.environ["DISPLAY"] = display
    os.environ["XAUTHORITY"] = xauthority
    logger.info("ambient_vtk_display_ready", display=display)
    return display
