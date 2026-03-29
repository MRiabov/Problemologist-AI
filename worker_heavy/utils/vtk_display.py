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


def _start_private_vtk_display() -> _DisplayState:
    xvfb_path = shutil.which("Xvfb")
    if xvfb_path is None:
        raise RuntimeError(
            "headless VTK rendering requires Xvfb, but Xvfb is not available"
        )

    last_error: str | None = None
    for _attempt in range(4):
        read_fd, write_fd = os.pipe()
        try:
            process = subprocess.Popen(
                [
                    xvfb_path,
                    "-ac",
                    "-nolisten",
                    "tcp",
                    "-screen",
                    "0",
                    "1280x1024x24",
                    "-displayfd",
                    str(write_fd),
                ],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                pass_fds=(write_fd,),
                close_fds=True,
            )
        finally:
            os.close(write_fd)

        try:
            display_bytes = b""
            while True:
                chunk = os.read(read_fd, 32)
                if not chunk:
                    break
                display_bytes += chunk
                if b"\n" in chunk:
                    break
        finally:
            os.close(read_fd)

        display_number = display_bytes.decode("utf-8", errors="replace").strip()
        if not display_number:
            last_error = "Xvfb did not report a display number"
            _stop_display_process(process)
            continue

        display = (
            display_number if display_number.startswith(":") else f":{display_number}"
        )
        if process.poll() is not None:
            last_error = f"Xvfb exited while starting display {display}"
            _stop_display_process(process)
            continue

        for _probe_attempt in range(5):
            if _probe_vtk_display(display):
                os.environ["DISPLAY"] = display
                os.environ.pop("XAUTHORITY", None)
                return _DisplayState(process=process, display=display)
            time.sleep(0.2)

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
    if (
        state is not None
        and state.process is None
        and existing_display == state.display
        and existing_xauthority == state.xauthority
    ):
        return state.display

    if existing_display and _probe_vtk_display(existing_display, existing_xauthority):
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
        if (
            state is not None
            and state.process is None
            and existing_display == state.display
            and existing_xauthority == state.xauthority
        ):
            return state.display
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
