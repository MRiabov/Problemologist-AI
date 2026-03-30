from __future__ import annotations

import base64
import tempfile
from pathlib import Path
from typing import Sequence

import numpy as np
import structlog
from PIL import Image

from shared.rendering.renderer_client import (
    bundle_workspace_base64,
    render_simulation_video,
)

logger = structlog.get_logger(__name__)


def synthesize_placeholder_frames(
    seed_source: str,
    *,
    frame_count: int = 3,
    width: int = 64,
    height: int = 48,
) -> list[np.ndarray]:
    """Create deterministic placeholder frames for legacy simulation workflows."""
    seed = sum(ord(ch) for ch in seed_source) % 255
    frames: list[np.ndarray] = []
    for idx in range(frame_count):
        color = np.array(
            [
                (seed + idx * 40) % 255,
                (seed + idx * 70 + 85) % 255,
                (seed + idx * 110 + 170) % 255,
            ],
            dtype=np.uint8,
        )
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:] = color
        frames.append(frame)
    return frames


def render_simulation_video_bytes(
    frames: Sequence[np.ndarray],
    *,
    output_name: str,
    fps: int,
    session_id: str,
    width: int = 640,
    height: int = 480,
) -> bytes:
    """Stage frames locally, delegate MP4 encoding to worker-renderer, and return bytes."""
    if not frames:
        raise ValueError("simulation video requires at least one frame")

    with tempfile.TemporaryDirectory() as tmpdir:
        staging_root = Path(tmpdir)
        frames_dir = staging_root / "frames"
        frames_dir.mkdir(parents=True, exist_ok=True)
        frame_paths: list[str] = []

        for index, frame in enumerate(frames):
            frame_path = frames_dir / f"frame_{index:05d}.png"
            if frame.shape[:2] != (height, width):
                frame = np.array(Image.fromarray(frame).resize((width, height)))
            Image.fromarray(frame).save(frame_path)
            frame_paths.append(str(frame_path.relative_to(staging_root)))

        response = render_simulation_video(
            bundle_base64=bundle_workspace_base64(staging_root),
            frame_paths=frame_paths,
            output_name=output_name,
            fps=fps,
            session_id=session_id,
        )
        if not response.success:
            raise RuntimeError(response.message)
        if response.artifacts is None:
            raise RuntimeError("renderer returned no artifacts")

        render_blobs = response.artifacts.render_blobs_base64 or {}
        output_key = next(
            (key for key in render_blobs if Path(key).name == output_name),
            None,
        )
        if output_key is None:
            raise RuntimeError("renderer returned no simulation video blob")

        logger.info(
            "simulation_video_bytes_rendered",
            session_id=session_id,
            output_name=output_name,
            frame_count=len(frames),
        )
        return base64.b64decode(render_blobs[output_key])
