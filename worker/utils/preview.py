"""
Preview design utility for CAD visualization.

Renders CAD models from specific camera angles for agent inspection.
"""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import structlog
from build123d import Compound, Part
from PIL import Image

from worker.simulation.builder import SimulationBuilder

logger = structlog.get_logger(__name__)


def preview_design(
    component: Part | Compound,
    pitch: float = -35.0,
    yaw: float = 45.0,
    output_dir: Path | None = None,
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
    import mujoco

    # Build MJCF from component using SimulationBuilder
    with TemporaryDirectory() as temp_build_dir:
        build_dir = Path(temp_build_dir)
        builder = SimulationBuilder(output_dir=build_dir)
        scene_path = builder.build_from_assembly(component)

        # Load into MuJoCo
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)

        # Step once to initialize
        mujoco.mj_step(model, data)

        # Create renderer
        renderer = mujoco.Renderer(model, height, width)

        # Set up camera
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)

        # Calculate scene center and distance from bounding box
        cam.lookat = np.array([0, 0, 0.5])
        cam.distance = 2.0

        # Set camera angles
        cam.elevation = pitch
        cam.azimuth = yaw

        # Render
        renderer.update_scene(data, camera=cam)
        frame = renderer.render()

    # Save image
    if output_dir is None:
        output_dir = Path("/tmp")
    output_dir.mkdir(parents=True, exist_ok=True)

    image_path = output_dir / f"preview_pitch{int(pitch)}_yaw{int(yaw)}.jpg"
    img = Image.fromarray(frame)
    img.save(image_path, "JPEG")

    logger.info("preview_saved", path=str(image_path), pitch=pitch, yaw=yaw)
    return image_path
