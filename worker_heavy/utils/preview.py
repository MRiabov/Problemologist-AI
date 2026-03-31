"""
Preview design utility for CAD visualization.

Renders CAD models from specific camera angles for agent inspection.
"""

from pathlib import Path
from tempfile import TemporaryDirectory

import structlog
from build123d import Compound, Part
from PIL import Image

from shared.agents.config import load_agents_config
from shared.models.schemas import BenchmarkDefinition
from worker_heavy.utils.build123d_rendering import (
    Build123dRendererBackend,
    _preview_camera_distance,
    camera_position_from_orbit,
)

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
    """
    Render a single view of a CAD component. Default (-35, 45) is ISO view.

    Args:
        component: The build123d Part or Compound to render
        pitch: Camera elevation angle in degrees (negative = looking down)
        yaw: Camera azimuth angle in degrees (clockwise from front)
        output_dir: Directory to save the image (uses /tmp if None)
        width: Image width in pixels
        height: Image height in pixels

    Returns:
        Path to the saved preview image
    """
    render_policy = load_agents_config().render
    with TemporaryDirectory() as temp_build_dir:
        build_dir = Path(temp_build_dir)
        backend = Build123dRendererBackend(
            workspace_root=build_dir,
            objectives=objectives,
            rgb_axes=render_policy.rgb.axes,
            rgb_edges=render_policy.rgb.edges,
        )
        try:
            backend.load_scene(component)
            scene = backend.scene
            if scene is None:
                raise RuntimeError("build123d preview scene failed to load")

            distance = _preview_camera_distance(scene, width=width, height=height)
            camera_position = camera_position_from_orbit(
                scene.center, distance, pitch, yaw
            )
            backend.set_camera(
                "preview",
                pos=camera_position,
                lookat=scene.center,
                up=(0.0, 0.0, 1.0),
            )
            frame = backend.render_camera("preview", width, height)
        finally:
            backend.close()

    # Save image
    if output_dir is None:
        output_dir = Path("/tmp")
    output_dir.mkdir(parents=True, exist_ok=True)

    image_path = output_dir / f"preview_pitch{int(pitch)}_yaw{int(yaw)}.jpg"
    img = Image.fromarray(frame)
    img.save(image_path, "JPEG")

    logger.info("preview_saved", path=str(image_path), pitch=pitch, yaw=yaw)
    return image_path
