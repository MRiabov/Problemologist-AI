"""Preview design utility for CAD visualization."""

import base64
import os
from pathlib import Path

import structlog
from build123d import Compound, Part

from shared.models.schemas import BenchmarkDefinition
from shared.rendering import materialize_preview_response, render_preview
from shared.workers.schema import PreviewDesignResponse, PreviewRenderingType
from worker_heavy.utils.build123d_rendering import export_preview_scene_bundle
from worker_heavy.utils.rendering import select_single_preview_render_subdir

logger = structlog.get_logger(__name__)


def preview(
    component: Part | Compound,
    orbit_pitch: float = -35.0,
    orbit_yaw: float = 45.0,
    rendering_type: PreviewRenderingType | str = PreviewRenderingType.RGB,
    output_dir: Path | None = None,
    objectives: BenchmarkDefinition | None = None,
    width: int = 640,
    height: int = 480,
) -> PreviewDesignResponse:
    """Render a single view of a CAD component and persist it to the preview bucket."""
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
        rendering_type=PreviewRenderingType(str(rendering_type)),
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
        image_name = Path(
            response.image_path
            or f"preview_pitch{int(orbit_pitch)}_yaw{int(orbit_yaw)}.jpg"
        ).name
        materialized_path = output_dir / image_name
        materialized_path.write_bytes(base64.b64decode(image_bytes_base64))

    try:
        response.image_path = str(materialized_path.relative_to(workspace_root))
    except ValueError:
        response.image_path = str(materialized_path)
    response.artifact_path = response.image_path
    response.manifest_path = str(Path("renders") / "render_manifest.json")
    response.pitch = orbit_pitch
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
