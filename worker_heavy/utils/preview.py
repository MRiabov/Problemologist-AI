"""Preview design utility for CAD visualization."""

import base64
import os
from pathlib import Path

import structlog
from build123d import Compound, Part

from shared.models.schemas import BenchmarkDefinition
from shared.rendering import (
    export_preview_scene_bundle,
    materialize_preview_response,
    render_preview,
    select_single_preview_render_subdir,
)
from shared.workers.schema import PreviewDesignResponse, PreviewRenderingType

logger = structlog.get_logger(__name__)


def preview(
    component: Part | Compound,
    orbit_pitch: float | list[float] = 45.0,
    orbit_yaw: float | list[float] = 45.0,
    rgb: bool | None = None,
    depth: bool | None = None,
    segmentation: bool | None = None,
    rendering_type: PreviewRenderingType | str | None = None,
    output_dir: Path | None = None,
    objectives: BenchmarkDefinition | None = None,
    width: int = 640,
    height: int = 480,
) -> PreviewDesignResponse:
    """Render a preview of a CAD component and persist it to the preview bucket."""
    del width, height

    workspace_root = Path.cwd()
    preview_scene_bundle = export_preview_scene_bundle(
        component,
        objectives=objectives,
        workspace_root=workspace_root,
    )
    session_id = os.getenv("SESSION_ID") or None
    response = render_preview(
        bundle_base64=preview_scene_bundle,
        script_path="preview_scene.json",
        orbit_pitch=orbit_pitch,
        orbit_yaw=orbit_yaw,
        rgb=rgb,
        depth=depth,
        segmentation=segmentation,
        rendering_type=(
            PreviewRenderingType(str(rendering_type))
            if rendering_type is not None
            else None
        ),
        session_id=session_id,
    )
    if not response.success:
        raise RuntimeError(response.message or "build123d preview render failed")

    image_bytes_base64 = response.image_bytes_base64
    if not image_bytes_base64:
        raise RuntimeError("renderer returned no preview image bytes")

    if output_dir is None:
        output_dir = (
            workspace_root
            / "renders"
            / select_single_preview_render_subdir(workspace_root)
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    materialized_path = materialize_preview_response(response, output_dir)
    if materialized_path is None:
        pitch_value = orbit_pitch[0] if isinstance(orbit_pitch, list) else orbit_pitch
        yaw_value = orbit_yaw[0] if isinstance(orbit_yaw, list) else orbit_yaw
        image_name = Path(
            response.image_path
            or f"preview_pitch{int(pitch_value)}_yaw{int(yaw_value)}.jpg"
        ).name
        materialized_path = output_dir / image_name
        materialized_path.write_bytes(base64.b64decode(image_bytes_base64))

    try:
        response.image_path = str(materialized_path.relative_to(workspace_root))
    except ValueError:
        response.image_path = str(materialized_path)
    response.artifact_path = response.image_path
    response.manifest_path = str(Path("renders") / "render_manifest.json")
    if isinstance(orbit_pitch, list):
        response.pitch = orbit_pitch[0] if len(orbit_pitch) == 1 else None
    else:
        response.pitch = orbit_pitch
    if isinstance(orbit_yaw, list):
        response.yaw = orbit_yaw[0] if len(orbit_yaw) == 1 else None
    else:
        response.yaw = orbit_yaw
    if response.status_text is None:
        response.status_text = response.message or "Preview generated successfully"

    logger.info(
        "preview_saved",
        path=str(materialized_path),
        orbit_pitch=orbit_pitch,
        orbit_yaw=orbit_yaw,
        rendering_type=response.rendering_type.value,
    )
    return response
