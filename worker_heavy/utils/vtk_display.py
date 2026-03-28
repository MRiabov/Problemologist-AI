from __future__ import annotations

import atexit
import os
import shutil
import subprocess
import threading
from dataclasses import dataclass

_DISPLAY_LOCK = threading.Lock()


@dataclass(slots=True)
class _DisplayState:
    process: subprocess.Popen[bytes]
    display: str


_DISPLAY_STATE: _DisplayState | None = None


def _stop_display() -> None:
    global _DISPLAY_STATE
    state = _DISPLAY_STATE
    if state is None:
        return

    process = state.process
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass

    _DISPLAY_STATE = None


def ensure_headless_vtk_display() -> str:
    """Start and pin a private Xvfb display for VTK rendering."""

    global _DISPLAY_STATE

    state = _DISPLAY_STATE
    if state is not None and state.process.poll() is None:
        os.environ["DISPLAY"] = state.display
        return state.display

    with _DISPLAY_LOCK:
        state = _DISPLAY_STATE
        if state is not None and state.process.poll() is None:
            os.environ["DISPLAY"] = state.display
            return state.display

        xvfb_path = shutil.which("Xvfb")
        if xvfb_path is None:
            raise RuntimeError(
                "headless VTK rendering requires Xvfb, but Xvfb is not available"
            )

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
        if not display_number or process.poll() is not None:
            _stop_display()
            raise RuntimeError("failed to start a private Xvfb display for VTK")

        display = f":{display_number}"
        os.environ["DISPLAY"] = display
        _DISPLAY_STATE = _DisplayState(process=process, display=display)
        atexit.register(_stop_display)
        return display
