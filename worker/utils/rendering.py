import os
from pathlib import Path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np
import structlog
from build123d import Compound
from PIL import Image

from worker.simulation.builder import SimulationBuilder

logger = structlog.get_logger(__name__)


def prerender_24_views(component: Compound, output_dir: str = None) -> list[str]:
    """
    Generates 24 renders (8 angles x 3 elevation levels) of the component using MuJoCo.
    Saves to output_dir.
    """
    if output_dir is None:
        output_dir = os.getenv("RENDERS_DIR", "./renders")

    output_path = Path(output_dir)
    logger.info("prerender_24_views_start", output_dir=str(output_path))
    output_path.mkdir(parents=True, exist_ok=True)

    saved_files = []

    try:
        # 1. Build MJCF using SimulationBuilder
        with TemporaryDirectory() as temp_build_dir:
            build_dir = Path(temp_build_dir)
            builder = SimulationBuilder(output_dir=build_dir)
            scene_path = builder.build_from_assembly(component)

            # 2. Load into MuJoCo
            model = mujoco.MjModel.from_xml_path(str(scene_path))
            data = mujoco.MjData(model)

            # Step once to initialize positions
            mujoco.mj_step(model, data)

            # 3. Initialize Renderer
            width, height = 640, 480
            renderer = mujoco.Renderer(model, height, width)

            # 4. Setup Camera
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)

            # Center the camera on the object
            bbox = component.bounding_box()
            center = [
                (bbox.min.X + bbox.max.X) / 2,
                (bbox.min.Y + bbox.max.Y) / 2,
                (bbox.min.Z + bbox.max.Z) / 2,
            ]
            cam.lookat = np.array(center)

            # Distance based on bbox size
            diag = np.sqrt(bbox.size.X**2 + bbox.size.Y**2 + bbox.size.Z**2)
            cam.distance = max(diag * 1.5, 0.5)

            # 8 horizontal angles
            angles = [0, 45, 90, 135, 180, 225, 270, 315]
            # 3 elevations
            elevations = [
                -15,
                -45,
                -75,
            ]  # MuJoCo uses negative elevation for looking down

            for elevation in elevations:
                for angle in angles:
                    filename = f"render_e{abs(elevation)}_a{angle}.png"
                    filepath = output_path / filename

                    cam.elevation = elevation
                    cam.azimuth = angle

                    # Render
                    renderer.update_scene(data, camera=cam)
                    frame = renderer.render()

                    # Save using PIL
                    img = Image.fromarray(frame)
                    img.save(filepath, "PNG")
                    saved_files.append(str(filepath))

        logger.info("prerender_complete", count=len(saved_files))
        return saved_files
    except Exception as e:
        logger.error("prerender_failed", error=str(e))
        raise
