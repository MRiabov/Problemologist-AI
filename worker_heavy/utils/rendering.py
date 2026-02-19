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
    StressField,
)
from shared.simulation.schemas import SimulatorBackendType
from worker_heavy.simulation.factory import get_simulation_builder

logger = structlog.get_logger(__name__)


def prerender_24_views(
    component: Compound,
    output_dir: str | None = None,
    backend_type: SimulatorBackendType = SimulatorBackendType.GENESIS,
    session_id: str | None = None,
    scene_path: str | Path | None = None,
    smoke_test_mode: bool = False,
    particle_budget: int | None = None,
) -> list[str]:
    """
    Generates 24 renders (8 angles x 3 elevation levels) of the component.
    Saves to output_dir.
    """
    if output_dir is None:
        output_dir = os.getenv("RENDERS_DIR", "./renders")

    output_path = Path(output_dir)
    logger.info(
        "prerender_24_views_start",
        output_dir=str(output_path),
        backend=backend_type,
        session_id=session_id,
        scene_path=str(scene_path) if scene_path else None,
        smoke_test_mode=smoke_test_mode,
        particle_budget=particle_budget,
    )
    output_path.mkdir(parents=True, exist_ok=True)

    saved_files = []

    try:
        # 1. Build Scene using get_simulation_builder (unless scene_path provided)
        with TemporaryDirectory() as temp_build_dir:
            if scene_path:
                final_scene_path = Path(scene_path)
            else:
                build_dir = Path(temp_build_dir)
                builder = get_simulation_builder(
                    output_dir=build_dir, backend_type=backend_type
                )
                final_scene_path = builder.build_from_assembly(
                    component, smoke_test_mode=smoke_test_mode
                )

            # 2. Initialize Backend
            from shared.simulation.backends import SimulationScene
            from worker_heavy.simulation.factory import get_physics_backend

            backend = get_physics_backend(
                backend_type,
                session_id=session_id,
                smoke_test_mode=smoke_test_mode,
                particle_budget=particle_budget,
            )
            scene = SimulationScene(scene_path=str(final_scene_path))

            # OPTIMIZATION: Use render_only=True to skip expensive physics build in Genesis.
            backend.load_scene(scene, render_only=True)

            # NOTE: We skip backend.step() here because it requires a built physics scene,
            # and for 24-view static renders we only need the geometric/visual state.
            # backend.step(0.002)

            # 3. Setup Camera Parameters
            bbox = component.bounding_box()
            center = (
                (bbox.min.X + bbox.max.X) / 2,
                (bbox.min.Y + bbox.max.Y) / 2,
                (bbox.min.Z + bbox.max.Z) / 2,
            )

            # Distance based on bbox size
            diag = np.sqrt(bbox.size.X**2 + bbox.size.Y**2 + bbox.size.Z**2)
            distance = max(diag * 1.5, 0.5)

            # 8 horizontal angles
            angles = [0, 45, 90, 135, 180, 225, 270, 315]
            # 3 elevations
            elevations = [
                -15,
                -45,
                -75,
            ]  # MuJoCo uses negative elevation for looking down

            if smoke_test_mode:
                logger.info("smoke_test_mode_reducing_render_views")
                angles = [45]
                elevations = [-45]

            width, height = 640, 480

            for elevation in elevations:
                for angle in angles:
                    filename = f"render_e{abs(elevation)}_a{angle}.png"
                    filepath = output_path / filename

                    # Calculate camera position from orbit
                    # azim=angle, elev=elevation
                    # MuJoCo orbit logic:
                    rad_azim = np.deg2rad(angle)
                    rad_elev = np.deg2rad(elevation)

                    # Simplified orbit calculation
                    x = center[0] + distance * np.cos(rad_elev) * np.sin(rad_azim)
                    y = center[1] - distance * np.cos(rad_elev) * np.cos(rad_azim)
                    z = center[2] - distance * np.sin(rad_elev)

                    backend.set_camera(
                        "prerender", pos=(x, y, z), lookat=center, up=(0, 0, 1)
                    )

                    # Render
                    frame = backend.render_camera("prerender", width, height)

                    # Save using PIL
                    img = Image.fromarray(frame)
                    img.save(filepath, "PNG")
                    saved_files.append(str(filepath))

            # Only close if it's not a cached backend
            if not session_id:
                backend.close()

        logger.info("prerender_complete", count=len(saved_files))
        return saved_files
    except Exception as e:
        import traceback

        logger.error("prerender_failed", error=str(e), stack=traceback.format_exc())
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
            # Multi-tenant / Dynamic Resolution Safeguard:
            # Ensure frame matches the expected VideoWriter resolution (width, height)
            h, w = frame.shape[:2]
            if w != self.width or h != self.height:
                frame = cv2.resize(frame, (self.width, self.height))

            # Convert RGB to BGR for OpenCV
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            out.write(bgr_frame)
        out.release()
        logger.info("video_render_complete", path=str(self.output_path))
