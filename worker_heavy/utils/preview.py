"""Preview design utility for CAD visualization."""

import base64
import os
from pathlib import Path

import structlog
from build123d import Compound, Part

from shared.models.schemas import BenchmarkDefinition
from shared.rendering import render_preview
from worker_heavy.utils.build123d_rendering import export_preview_scene_bundle

logger = structlog.get_logger(__name__)


def preview_design(
    component: Part | Compound,
    pitch: float = -35.0,
    yaw: float = 45.0,
    output_dir: Path | None = None,
    objectives: BenchmarkDefinition | None = None,
    width: int = 640,
    height: int = 480,
) -> Path:
    """Render a single view of a CAD component. Default (-35, 45) is ISO view."""
    del width, height
    preview_scene_bundle = export_preview_scene_bundle(
        component,
        objectives=objectives,
        workspace_root=Path.cwd(),
    )
    session_id = os.getenv("SESSION_ID") or None
    response = render_preview(
        bundle_base64=preview_scene_bundle,
        script_path="preview_scene.json",
        orbit_pitch=pitch,
        orbit_yaw=yaw,
        session_id=session_id,
    )
    if not response.success:
        raise RuntimeError(response.message or "build123d preview render failed")

    image_bytes_base64 = response.image_bytes_base64
    if not image_bytes_base64:
        raise RuntimeError("renderer returned no preview image bytes")

    if output_dir is None:
        output_dir = Path("/tmp")
    output_dir.mkdir(parents=True, exist_ok=True)

    image_name = Path(
        response.image_path or f"preview_pitch{int(pitch)}_yaw{int(yaw)}.jpg"
    ).name
    image_path = output_dir / image_name
    image_path.write_bytes(base64.b64decode(image_bytes_base64))

    logger.info("preview_saved", path=str(image_path), pitch=pitch, yaw=yaw)
    return image_path
