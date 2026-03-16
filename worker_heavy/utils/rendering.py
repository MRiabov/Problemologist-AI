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

from shared.agents.config import load_agents_config
from shared.models.schemas import BenchmarkDefinition
from shared.simulation.backends import (
    StressField,
)
from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)
from shared.workers.schema import (
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
)
from worker_heavy.simulation.factory import get_simulation_builder

logger = structlog.get_logger(__name__)


def _save_rgb_render(frame: np.ndarray, output_path: Path) -> None:
    Image.fromarray(frame).save(output_path, "PNG")


def _save_depth_render(depth_map: np.ndarray, output_path: Path) -> None:
    finite_mask = np.isfinite(depth_map)
    depth_image = np.zeros(depth_map.shape, dtype=np.uint8)
    if finite_mask.any():
        finite_depth = depth_map[finite_mask]
        min_depth = float(finite_depth.min())
        max_depth = float(finite_depth.max())
        if np.isclose(min_depth, max_depth):
            normalized = np.ones(depth_map.shape, dtype=np.float32)
        else:
            normalized = (depth_map - min_depth) / (max_depth - min_depth)
            normalized = 1.0 - np.clip(normalized, 0.0, 1.0)
        depth_image[finite_mask] = (normalized[finite_mask] * 255.0).astype(np.uint8)

    Image.fromarray(depth_image, mode="L").save(output_path, "PNG")


def _save_segmentation_render(segmentation_map: np.ndarray, output_path: Path) -> None:
    seg_image = np.zeros((*segmentation_map.shape[:2], 3), dtype=np.uint8)
    seg_image[..., :3] = segmentation_map[..., :3]
    Image.fromarray(seg_image, mode="RGB").save(output_path, "PNG")


def _workspace_render_path(output_path: Path, filename: str) -> str:
    prefix = output_path.name if output_path.name else "renders"
    return str(Path(prefix) / filename)


def _build_depth_metadata(
    *,
    group_key: str,
    siblings: RenderSiblingPaths,
    depth_frame: np.ndarray,
) -> RenderArtifactMetadata:
    finite_mask = np.isfinite(depth_frame)
    depth_min = float(depth_frame[finite_mask].min()) if finite_mask.any() else None
    depth_max = float(depth_frame[finite_mask].max()) if finite_mask.any() else None
    return RenderArtifactMetadata(
        modality="depth",
        group_key=group_key,
        siblings=siblings,
        depth_min_m=depth_min,
        depth_max_m=depth_max,
        depth_interpretation=(
            "Brighter pixels are nearer. Values are normalized per image from the "
            "metric-depth range reported by depth_min_m/depth_max_m."
        ),
    )


def prerender_24_views(
    component: Compound,
    output_dir: str | None = None,
    objectives: BenchmarkDefinition | None = None,
    backend_type: SimulatorBackendType | None = None,
    session_id: str | None = None,
    scene_path: str | Path | None = None,
    smoke_test_mode: bool | None = None,
    particle_budget: int | None = None,
) -> list[str]:
    """
    Generates 24 renders (8 angles x 3 elevation levels) of the component.
    Saves to output_dir.
    """
    from worker_heavy.config import settings

    if smoke_test_mode is None:
        smoke_test_mode = settings.smoke_test_mode
    resolved_backend_type = backend_type or get_default_simulator_backend()

    if output_dir is None:
        output_dir = os.getenv("RENDERS_DIR", "./renders")

    render_policy = load_agents_config().render
    output_path = Path(output_dir)
    logger.info(
        "prerender_24_views_start",
        output_dir=str(output_path),
        backend=resolved_backend_type,
        session_id=session_id,
        scene_path=str(scene_path) if scene_path else None,
        smoke_test_mode=smoke_test_mode,
        particle_budget=particle_budget,
        rgb_enabled=render_policy.rgb,
        depth_enabled=render_policy.depth,
        segmentation_enabled=render_policy.segmentation,
    )
    output_path.mkdir(parents=True, exist_ok=True)

    if not any((render_policy.rgb, render_policy.depth, render_policy.segmentation)):
        logger.info(
            "prerender_skipped_all_modalities_disabled",
            session_id=session_id,
        )
        return []

    saved_files = []
    manifest = RenderManifest()

    try:
        # 1. Build Scene using get_simulation_builder (unless scene_path provided)
        with TemporaryDirectory() as temp_build_dir:
            if scene_path:
                final_scene_path = Path(scene_path)
            else:
                build_dir = Path(temp_build_dir)
                builder = get_simulation_builder(
                    output_dir=build_dir, backend_type=resolved_backend_type
                )
                final_scene_path = builder.build_from_assembly(
                    component,
                    objectives=objectives,
                    smoke_test_mode=smoke_test_mode,
                )

            # 2. Initialize Backend
            from shared.simulation.backends import SimulationScene
            from worker_heavy.simulation.factory import get_physics_backend

            backend = get_physics_backend(
                resolved_backend_type,
                session_id=session_id,
                smoke_test_mode=smoke_test_mode,
                particle_budget=particle_budget,
            )
            scene = SimulationScene(scene_path=str(final_scene_path))

            # OPTIMIZATION: Use render_only=True to skip expensive physics build in Genesis.
            backend.load_scene(scene, render_only=True)
            if resolved_backend_type == SimulatorBackendType.MUJOCO:
                # Keep smoke-mode validation previews material-color dominant by
                # hiding the checkerboard floor in render-only snapshots.
                try:
                    import mujoco

                    floor_geom_id = mujoco.mj_name2id(
                        backend.model,
                        mujoco.mjtObj.mjOBJ_GEOM,
                        "floor",
                    )
                    if floor_geom_id >= 0:
                        backend.model.geom_rgba[floor_geom_id] = np.array(
                            [0.0, 0.0, 0.0, 0.0], dtype=np.float32
                        )
                except Exception:
                    logger.debug("mujoco_floor_hide_skipped", session_id=session_id)

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
                    group_key = Path(filename).stem

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
                    try:
                        depth_path = (
                            output_path / f"render_e{abs(elevation)}_a{angle}_depth.png"
                        )
                        segmentation_path = (
                            output_path
                            / f"render_e{abs(elevation)}_a{angle}_segmentation.png"
                        )
                        siblings = RenderSiblingPaths(
                            rgb=(
                                _workspace_render_path(output_path, filename)
                                if render_policy.rgb
                                else None
                            ),
                            depth=(
                                _workspace_render_path(output_path, depth_path.name)
                                if render_policy.depth
                                else None
                            ),
                            segmentation=(
                                _workspace_render_path(
                                    output_path, segmentation_path.name
                                )
                                if render_policy.segmentation
                                else None
                            ),
                        )
                        if (
                            resolved_backend_type == SimulatorBackendType.MUJOCO
                            and hasattr(backend, "render_camera_modalities")
                        ):
                            frame, depth_frame, segmentation_frame = (
                                backend.render_camera_modalities(
                                    "prerender",
                                    width,
                                    height,
                                    include_rgb=render_policy.rgb,
                                    include_depth=render_policy.depth,
                                    include_segmentation=render_policy.segmentation,
                                )
                            )
                            segmentation_legend = []
                            if (
                                render_policy.segmentation
                                and segmentation_frame is not None
                                and hasattr(backend, "describe_segmentation")
                            ):
                                segmentation_legend = backend.describe_segmentation(
                                    segmentation_frame
                                )
                                if hasattr(backend, "colorize_segmentation"):
                                    segmentation_frame = backend.colorize_segmentation(
                                        segmentation_frame
                                    )
                        else:
                            frame = backend.render_camera("prerender", width, height)
                            depth_frame = None
                            segmentation_frame = None
                            segmentation_legend = []

                        if render_policy.rgb and frame is not None:
                            _save_rgb_render(frame, filepath)
                            saved_files.append(str(filepath))
                            manifest.artifacts[siblings.rgb] = RenderArtifactMetadata(
                                modality="rgb",
                                group_key=group_key,
                                siblings=siblings,
                            )
                        if render_policy.depth and depth_frame is not None:
                            _save_depth_render(depth_frame, depth_path)
                            saved_files.append(str(depth_path))
                            if siblings.depth:
                                manifest.artifacts[siblings.depth] = (
                                    _build_depth_metadata(
                                        group_key=group_key,
                                        siblings=siblings,
                                        depth_frame=depth_frame,
                                    )
                                )
                        if (
                            render_policy.segmentation
                            and segmentation_frame is not None
                        ):
                            _save_segmentation_render(
                                segmentation_frame, segmentation_path
                            )
                            saved_files.append(str(segmentation_path))
                            if siblings.segmentation:
                                manifest.artifacts[siblings.segmentation] = (
                                    RenderArtifactMetadata(
                                        modality="segmentation",
                                        group_key=group_key,
                                        siblings=siblings,
                                        segmentation_legend=segmentation_legend,
                                    )
                                )
                    except Exception as e:
                        if "EGL" in str(e) or "display" in str(e).lower():
                            logger.warning(
                                "prerender_camera_failed_skipping",
                                error=str(e),
                                angle=angle,
                                elevation=elevation,
                                session_id=session_id,
                            )
                            # If rendering fails once due to EGL, it likely will fail for all views.
                            # We can break or continue. Let's continue to be safe, but it'll probably fail for all.
                            continue
                        raise

            # Only close if it's not a cached backend
            if not session_id:
                backend.close()

        manifest_path = output_path / "render_manifest.json"
        manifest_path.write_text(
            manifest.model_dump_json(indent=2),
            encoding="utf-8",
        )

        logger.info("prerender_complete", count=len(saved_files), session_id=session_id)
        return saved_files
    except Exception as e:
        import traceback

        logger.warning(
            "prerender_failed",
            error=str(e),
            stack=traceback.format_exc(),
            session_id=session_id,
        )
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
        logger.warning("render_stress_heatmap_failed", error=str(e))
        # Create a blank error image
        img = Image.new("RGB", (width, height), color=(255, 0, 0))
        img.save(output_path)
        return output_path


class VideoRenderer:
    """Handles video generation for simulations."""

    def __init__(
        self,
        output_path: Path,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        session_id: str | None = None,
    ):
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.session_id = session_id
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
            logger.error("video_render_no_frames", session_id=self.session_id)
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
        logger.info(
            "video_render_complete",
            path=str(self.output_path),
            session_id=self.session_id,
        )
