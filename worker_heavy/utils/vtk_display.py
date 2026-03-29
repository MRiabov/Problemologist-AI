from __future__ import annotations

import atexit
import os
import shutil
import subprocess
import sys
import threading
import time
from contextlib import suppress
from dataclasses import dataclass

import structlog

_DISPLAY_LOCK = threading.Lock()
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


@dataclass(slots=True)
class _DisplayState:
    process: subprocess.Popen[bytes] | None
    display: str
    xauthority: str | None = None


_DISPLAY_STATE: _DisplayState | None = None


def _stop_display() -> None:
    global _DISPLAY_STATE
    state = _DISPLAY_STATE
    if state is None:
        return

    process = state.process
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            with suppress(subprocess.TimeoutExpired):
                process.wait(timeout=2.0)

    _DISPLAY_STATE = None


def _probe_vtk_display(display: str, xauthority: str | None = None) -> bool:
    """Return True only when VTK can connect to the given X display."""

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
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=15.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return completed.returncode == 0


def _reuse_cached_ambient_display(
    existing_display: str | None, existing_xauthority: str | None
) -> bool | None:
    """Re-probe a cached ambient display and drop it if it went stale.

    Returns:
        True if the cached ambient display is still valid.
        False if the cached ambient display became stale and should be ignored.
        None if there was no matching cached ambient display to check.
    """

    global _DISPLAY_STATE

    state = _DISPLAY_STATE
    if (
        state is not None
        and state.process is None
        and existing_display == state.display
        and existing_xauthority == state.xauthority
    ):
        if _probe_vtk_display(existing_display, existing_xauthority):
            return True
        _DISPLAY_STATE = None
        return False
    return None


def _private_display_candidates() -> list[str]:
    """Return a short, low-numbered display search order for private Xvfb.

    The auto-selected displayfd path can land on stale high-number sockets on
    desktop hosts. A small candidate range is more predictable here and still
    fail-closed if none of the displays are usable.
    """

    base_display = 99
    candidate_count = 24
    pid_offset = os.getpid() % candidate_count
    ordered_numbers = list(
        range(base_display + pid_offset, base_display + candidate_count)
    ) + list(range(base_display, base_display + pid_offset))
    return [f":{display}" for display in ordered_numbers]


def _start_private_vtk_display() -> _DisplayState:
    xvfb_path = shutil.which("Xvfb")
    if xvfb_path is None:
        raise RuntimeError(
            "headless VTK rendering requires Xvfb, but Xvfb is not available"
        )

    last_error: str | None = None
    for display in _private_display_candidates():
        process = subprocess.Popen(
            [
                xvfb_path,
                display,
                "-ac",
                "-nolisten",
                "tcp",
                "-screen",
                "0",
                "1280x1024x24",
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            close_fds=True,
        )

        for _probe_attempt in range(5):
            if process.poll() is not None:
                last_error = f"Xvfb exited while starting display {display}"
                break
            if _probe_vtk_display(display):
                os.environ["DISPLAY"] = display
                os.environ.pop("XAUTHORITY", None)
                return _DisplayState(process=process, display=display)
            time.sleep(0.2)
        else:
            last_error = f"VTK could not connect to private Xvfb display {display}"

        _stop_display_process(process)

    raise RuntimeError(
        "failed to start a private Xvfb display for VTK"
        + (f": {last_error}" if last_error else "")
    )


def _stop_display_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            with suppress(subprocess.TimeoutExpired):
                process.wait(timeout=2.0)


def ensure_headless_vtk_display() -> str:
    """Start and pin a private Xvfb display for VTK rendering."""

    global _DISPLAY_STATE

    existing_display = os.environ.get("DISPLAY")
    existing_xauthority = os.environ.get("XAUTHORITY")
    state = _DISPLAY_STATE

    cached_ambient_display = _reuse_cached_ambient_display(
        existing_display, existing_xauthority
    )
    if cached_ambient_display is True:
        return existing_display

    if (
        cached_ambient_display is None
        and existing_display
        and _probe_vtk_display(existing_display, existing_xauthority)
    ):
        state = _DisplayState(
            process=None,
            display=existing_display,
            xauthority=existing_xauthority,
        )
        _DISPLAY_STATE = state
        return existing_display

    if existing_display:
        logger.error(
            "ambient_vtk_display_unusable_falling_back_to_private_xvfb",
            display=existing_display,
            xauthority=bool(existing_xauthority),
        )
    else:
        logger.error(
            "ambient_vtk_display_missing_falling_back_to_private_xvfb",
        )

    if state is not None and state.process is not None and state.process.poll() is None:
        os.environ["DISPLAY"] = state.display
        if state.xauthority:
            os.environ["XAUTHORITY"] = state.xauthority
        else:
            os.environ.pop("XAUTHORITY", None)
        return state.display

    with _DISPLAY_LOCK:
        state = _DISPLAY_STATE
        cached_ambient_display = _reuse_cached_ambient_display(
            existing_display, existing_xauthority
        )
        if cached_ambient_display is True:
            return existing_display
        if (
            cached_ambient_display is None
            and existing_display
            and _probe_vtk_display(existing_display, existing_xauthority)
        ):
            state = _DisplayState(
                process=None,
                display=existing_display,
                xauthority=existing_xauthority,
            )
            _DISPLAY_STATE = state
            return existing_display
        if (
            state is not None
            and state.process is not None
            and state.process.poll() is None
        ):
            os.environ["DISPLAY"] = state.display
            if state.xauthority:
                os.environ["XAUTHORITY"] = state.xauthority
            else:
                os.environ.pop("XAUTHORITY", None)
            return state.display

        state = _start_private_vtk_display()
        _DISPLAY_STATE = state
        atexit.register(_stop_display)
        return state.display
