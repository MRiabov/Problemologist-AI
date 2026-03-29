from __future__ import annotations

import atexit
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from collections.abc import Sequence
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path

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

_MAX_ERROR_LINES = 12
_X11_SOCKET_DIR = Path("/tmp/.X11-unix")


def _tail_text(text: str | None, *, max_lines: int = _MAX_ERROR_LINES) -> str | None:
    if text is None:
        return None
    lines = [line for line in text.splitlines() if line.strip()]
    if not lines:
        return None
    return "\n".join(lines[-max_lines:])


def _tail_bytes(data: bytes | None, *, max_lines: int = _MAX_ERROR_LINES) -> str | None:
    if data is None:
        return None
    return _tail_text(data.decode("utf-8", errors="replace"), max_lines=max_lines)


def _format_error_message(base: str, details: Sequence[str | None]) -> str:
    rendered_details = [detail for detail in details if detail]
    if not rendered_details:
        return base
    return base + ": " + " | ".join(rendered_details)


def _display_paths(display: str) -> tuple[Path, Path]:
    display_number = display.lstrip(":")
    return _X11_SOCKET_DIR / f"X{display_number}", Path(f"/tmp/.X{display_number}-lock")


def _lockfile_pid(lock_path: Path) -> int | None:
    try:
        return int(lock_path.read_text(encoding="utf-8").strip())
    except Exception:
        return None


def _is_process_alive(pid: int) -> bool:
    return Path(f"/proc/{pid}").exists()


def _unix_socket_is_live(socket_path: Path) -> bool:
    if not socket_path.exists():
        return False
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(0.1)
            return client.connect_ex(str(socket_path)) == 0
    except Exception:
        return False


def _prepare_private_display_slot(display: str) -> bool:
    """Return True when the display slot is safe to reuse, cleaning stale files."""

    socket_path, lock_path = _display_paths(display)
    lock_pid = _lockfile_pid(lock_path) if lock_path.exists() else None

    if lock_pid is not None and _is_process_alive(lock_pid):
        return False

    if _unix_socket_is_live(socket_path):
        return False

    with suppress(FileNotFoundError, PermissionError):
        lock_path.unlink()
    with suppress(FileNotFoundError, PermissionError):
        socket_path.unlink()
    return True


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


def _probe_vtk_display(
    display: str, xauthority: str | None = None
) -> tuple[bool, str | None]:
    """Return whether VTK can connect to the given X display and why not."""

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
    return False, _tail_bytes(
        completed.stderr
    ) or f"VTK probe exited with {completed.returncode}"


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
        probe_ok, _probe_error = _probe_vtk_display(
            existing_display, existing_xauthority
        )
        if probe_ok:
            return True
        _DISPLAY_STATE = None
        return False
    return None


def _private_display_candidates() -> list[str]:
    """Return a deterministic mid/high display search order for private Xvfb.

    The repo convention is to start private X displays in the 10000-10100
    range so they do not collide with normal desktop/Xwayland displays.
    """

    base_display = 10000
    candidate_count = 101
    pid_offset = os.getpid() % candidate_count
    ordered_numbers = list(
        range(base_display + pid_offset, base_display + candidate_count)
    ) + list(range(base_display, base_display + pid_offset))
    return [f":{display}" for display in ordered_numbers]


def _start_private_vtk_display_via_displayfd() -> _DisplayState | None:
    """Start a private Xvfb display by asking Xvfb to choose a free slot.

    This avoids deterministic display-number collisions and is the preferred
    startup path. The legacy scanned range remains as a fallback.
    """

    xvfb_path = shutil.which("Xvfb")
    if xvfb_path is None:
        raise RuntimeError(
            "headless VTK rendering requires Xvfb, but Xvfb is not available"
        )

    last_error: str | None = None
    with tempfile.TemporaryFile(mode="w+t") as displayfd_file:
        process = subprocess.Popen(
            [
                xvfb_path,
                "-displayfd",
                str(displayfd_file.fileno()),
                "-ac",
                "-nolisten",
                "tcp",
                "-screen",
                "0",
                "1280x1024x24",
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            close_fds=True,
            pass_fds=(displayfd_file.fileno(),),
        )

        for _probe_attempt in range(25):
            if process.poll() is not None:
                stderr_tail = None
                if process.stderr is not None:
                    with suppress(Exception):
                        stderr_tail = _tail_bytes(process.stderr.read())
                last_error = _format_error_message(
                    "Xvfb exited while starting a private display via displayfd",
                    [stderr_tail],
                )
                break

            displayfd_file.seek(0)
            display_text = displayfd_file.read().strip()
            if display_text:
                display = (
                    display_text if display_text.startswith(":") else f":{display_text}"
                )
                probe_ok, probe_error = _probe_vtk_display(display)
                if probe_ok:
                    os.environ["DISPLAY"] = display
                    os.environ.pop("XAUTHORITY", None)
                    logger.info(
                        "private_vtk_display_started_via_displayfd",
                        display=display,
                        xvfb_pid=process.pid,
                    )
                    return _DisplayState(process=process, display=display)

                last_error = _format_error_message(
                    f"VTK could not connect to private Xvfb display {display}"
                    " via displayfd",
                    [probe_error],
                )

            time.sleep(0.2)

        _stop_display_process(process)

    if last_error:
        logger.debug(
            "private_vtk_display_displayfd_failed",
            error=last_error,
        )
    return None


def _start_private_vtk_display() -> _DisplayState:
    displayfd_state = _start_private_vtk_display_via_displayfd()
    if displayfd_state is not None:
        return displayfd_state

    xvfb_path = shutil.which("Xvfb")
    if xvfb_path is None:
        raise RuntimeError(
            "headless VTK rendering requires Xvfb, but Xvfb is not available"
        )

    last_error: str | None = None
    for display in _private_display_candidates():
        if not _prepare_private_display_slot(display):
            continue

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
            stderr=subprocess.PIPE,
            close_fds=True,
        )

        for _probe_attempt in range(5):
            if process.poll() is not None:
                stderr_tail = None
                if process.stderr is not None:
                    with suppress(Exception):
                        stderr_tail = _tail_bytes(process.stderr.read())
                last_error = _format_error_message(
                    f"Xvfb exited while starting display {display}",
                    [stderr_tail],
                )
                break
            probe_ok, probe_error = _probe_vtk_display(display)
            if probe_ok:
                os.environ["DISPLAY"] = display
                os.environ.pop("XAUTHORITY", None)
                return _DisplayState(process=process, display=display)
            last_error = _format_error_message(
                f"VTK could not connect to private Xvfb display {display}",
                [probe_error],
            )
            time.sleep(0.2)

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
    if process.stderr is not None:
        with suppress(Exception):
            process.stderr.close()


def ensure_headless_vtk_display() -> str:
    """Start and pin a private Xvfb display for VTK rendering."""

    global _DISPLAY_STATE

    existing_display = os.environ.get("DISPLAY")
    existing_xauthority = os.environ.get("XAUTHORITY")

    with _DISPLAY_LOCK:
        state = _DISPLAY_STATE
        cached_ambient_display = _reuse_cached_ambient_display(
            existing_display, existing_xauthority
        )
        if cached_ambient_display is True:
            return existing_display
        fallback_logged = False
        if cached_ambient_display is None and existing_display:
            probe_ok, probe_error = _probe_vtk_display(
                existing_display, existing_xauthority
            )
            if probe_ok:
                state = _DisplayState(
                    process=None,
                    display=existing_display,
                    xauthority=existing_xauthority,
                )
                _DISPLAY_STATE = state
                return existing_display
            logger.error(
                "ambient_vtk_display_unusable_falling_back_to_private_xvfb",
                display=existing_display,
                xauthority=bool(existing_xauthority),
                probe_error=probe_error,
            )
            fallback_logged = True
        if not fallback_logged:
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
