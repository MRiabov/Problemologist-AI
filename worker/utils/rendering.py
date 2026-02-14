import os
from pathlib import Path
from tempfile import TemporaryDirectory

import matplotlib.pyplot as plt

# import mujoco  # Moved to lazy imports where needed
import numpy as np
import structlog
import trimesh
from build123d import Compound
from PIL import Image

from shared.simulation.backends import (
    SimulatorBackendType,
    StressField,
)
from worker.simulation.factory import get_simulation_builder

logger = structlog.get_logger(__name__)


def prerender_24_views(component: Compound, output_dir: str | None = None) -> list[str]:
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
        # 1. Build MJCF using get_simulation_builder
        with TemporaryDirectory() as temp_build_dir:
            build_dir = Path(temp_build_dir)
            builder = get_simulation_builder(
                output_dir=build_dir, backend_type=SimulatorBackendType.MUJOCO
            )
            scene_path = builder.build_from_assembly(component)

            # 2. Load into MuJoCo
            import mujoco

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


def render_stress_heatmap(
    stress_field: StressField,
    output_path: Path,
    mesh_path: Path | None = None,
    width: int = 800,
    height: int = 600,
) -> Path:
    """
    Renders a stress heatmap using PyVista (if available) or Matplotlib.
    For MVP, we use Matplotlib scatter if no mesh is provided, or trimesh if it is.
    """
    try:
        nodes = stress_field.nodes
        stresses = stress_field.stress

        if mesh_path and mesh_path.exists():
            # Use trimesh for 3D visualization if available
            mesh = trimesh.load(str(mesh_path))
            # Map stresses to vertices (simple nearest neighbor or interpolation)
            # For Genesis, stress is often per-node already.

            # Simple colormap mapping
            norm = plt.Normalize(vmin=stresses.min(), vmax=stresses.max())
            cmap = plt.get_cmap("jet")
            colors = cmap(norm(stresses))[:, :3] * 255  # RGB

            # If node count matches vertex count, apply directly
            if len(stresses) == len(mesh.vertices):
                mesh.visual.vertex_colors = colors.astype(np.uint8)

            scene = mesh.scene()
            data = scene.save_image(resolution=(width, height))
            with output_path.open("wb") as f:
                f.write(data)
        else:
            # Fallback to matplotlib 2D projection or simple scatter
            fig = plt.figure(figsize=(width / 100, height / 100))
            ax = fig.add_subplot(111, projection="3d")
            p = ax.scatter(
                nodes[:, 0], nodes[:, 1], nodes[:, 2], c=stresses, cmap="jet"
            )
            fig.colorbar(p, label="von Mises Stress (Pa)")
            plt.savefig(output_path)
            plt.close(fig)

        return output_path
    except Exception as e:
        logger.error("render_stress_heatmap_failed", error=str(e))
        # Create a blank error image
        img = Image.new("RGB", (width, height), color=(255, 0, 0))
        img.save(output_path)
        return output_path


class VideoRenderer:
    """Handles video generation for simulations."""

    def __init__(
        self, output_path: Path, width: int = 640, height: int = 480, fps: int = 30
    ):
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.frames = []

    def add_frame(self, frame: np.ndarray, particles: np.ndarray | None = None):
        """Adds a frame to the video. Optionally overlays particles."""
        if particles is not None:
            # Simple particle overlay logic for the simulation video
            # In Genesis, this is usually handled by the backend's internal renderer
            pass
        self.frames.append(frame)

    def save(self):
        """Saves the frames as an MP4 video."""
        if not self.frames:
            logger.warning("video_render_no_frames")
            return

        import cv2

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(
            str(self.output_path), fourcc, self.fps, (self.width, self.height)
        )

        for frame in self.frames:
            # Convert RGB to BGR for OpenCV
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            out.write(bgr_frame)
        out.release()
        logger.info("video_render_complete", path=str(self.output_path))
