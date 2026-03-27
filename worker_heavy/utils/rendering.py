import os
import uuid
import time
from pathlib import Path

import matplotlib.pyplot as plt

# import mujoco  # Moved to lazy imports where needed
import numpy as np
import structlog
import trimesh
from build123d import Compound
from PIL import Image

from shared.agents.config import load_agents_config
from shared.git_utils import repo_revision
from shared.observability.events import emit_event
from shared.models.schemas import BenchmarkDefinition
from shared.simulation.backends import StressField
from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)
from shared.workers.schema import (
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
)
from worker_heavy.utils.build123d_rendering import (
    PREVIEW_BACKEND_NAME,
    render_preview_bundle,
)

logger = structlog.get_logger(__name__)


def _derived_episode_id(session_id: str | None) -> str | None:
    if not session_id:
        return None
    try:
        return str(uuid.UUID(session_id))
    except Exception:
        return str(uuid.uuid5(uuid.NAMESPACE_DNS, session_id))


def _workspace_environment_version(workspace_root: Path | None) -> str | None:
    if workspace_root is None:
        return None

    for rel_path in (
        "benchmark_assembly_definition.yaml",
        "assembly_definition.yaml",
    ):
        definition_path = workspace_root / rel_path
        if not definition_path.exists():
            continue
        try:
            import yaml

            from shared.models.schemas import AssemblyDefinition

            definition = AssemblyDefinition.model_validate(
                yaml.safe_load(definition_path.read_text(encoding="utf-8")) or {}
            )
            version = str(definition.version).strip()
            return version or None
        except Exception:
            continue
    return None


def build_render_manifest(
    artifacts: dict[str, RenderArtifactMetadata],
    *,
    workspace_root: Path | None = None,
    episode_id: str | None = None,
    worker_session_id: str | None = None,
    revision: str | None = None,
    environment_version: str | None = None,
    preview_evidence_paths: list[str] | None = None,
) -> RenderManifest:
    resolved_revision = revision
    if not resolved_revision and workspace_root is not None:
        resolved_revision = repo_revision(workspace_root)
        if resolved_revision is None:
            resolved_revision = repo_revision(Path(__file__).resolve().parents[2])

    if environment_version is None:
        environment_version = _workspace_environment_version(workspace_root)

    if preview_evidence_paths is None:
        preview_evidence_paths = sorted(
            path
            for path in artifacts
            if Path(path).suffix.lower() in {".png", ".jpg", ".jpeg", ".mp4"}
        )

    return RenderManifest(
        episode_id=_derived_episode_id(episode_id),
        worker_session_id=worker_session_id,
        revision=resolved_revision,
        environment_version=environment_version,
        preview_evidence_paths=preview_evidence_paths,
        artifacts=artifacts,
    )


def _build_preview_artifacts(
    saved_paths: list[str],
    legend_by_path: dict[str, list],
    *,
    workspace_root: Path,
) -> tuple[dict[str, RenderArtifactMetadata], list[str]]:
    render_paths = _workspace_relative_render_paths(saved_paths, workspace_root)
    artifacts: dict[str, RenderArtifactMetadata] = {}

    for saved_path, render_path in zip(saved_paths, render_paths, strict=True):
        rel_path = Path(render_path)
        render_dir = rel_path.parent
        filename = rel_path.name
        stem = rel_path.stem
        if filename.endswith("_depth.png"):
            group_key = stem.removesuffix("_depth")
            artifacts[render_path] = RenderArtifactMetadata(
                modality="depth",
                group_key=group_key,
                siblings=RenderSiblingPaths(
                    rgb=str(render_dir / f"{group_key}.png"),
                    depth=str(render_dir / f"{group_key}_depth.png"),
                    segmentation=str(render_dir / f"{group_key}_segmentation.png"),
                ),
                depth_interpretation=(
                    "Brighter pixels are nearer. Values are normalized per image "
                    "from the build123d/VTK preview renderer."
                ),
            )
            continue

        if filename.endswith("_segmentation.png"):
            group_key = stem.removesuffix("_segmentation")
            artifacts[render_path] = RenderArtifactMetadata(
                modality="segmentation",
                group_key=group_key,
                siblings=RenderSiblingPaths(
                    rgb=str(render_dir / f"{group_key}.png"),
                    depth=str(render_dir / f"{group_key}_depth.png"),
                    segmentation=str(render_dir / f"{group_key}_segmentation.png"),
                ),
                segmentation_legend=legend_by_path.get(saved_path, []),
            )
            continue

        group_key = stem
        artifacts[render_path] = RenderArtifactMetadata(
            modality="rgb",
            group_key=group_key,
            siblings=RenderSiblingPaths(
                rgb=str(render_dir / f"{group_key}.png"),
                depth=str(render_dir / f"{group_key}_depth.png"),
                segmentation=str(render_dir / f"{group_key}_segmentation.png"),
            ),
        )

    return artifacts, render_paths


def _workspace_relative_render_paths(
    paths: list[str], workspace_root: Path
) -> list[str]:
    relative_paths: list[str] = []
    for path in paths:
        abs_path = Path(path)
        try:
            rel_path = abs_path.relative_to(workspace_root)
        except ValueError:
            rel_path = abs_path
        relative_paths.append(str(rel_path))
    return relative_paths


def prerender_24_views(
    component: Compound,
    output_dir: str | None = None,
    objectives: BenchmarkDefinition | None = None,
    backend_type: SimulatorBackendType | None = None,
    session_id: str | None = None,
    scene_path: str | Path | None = None,
    smoke_test_mode: bool | None = None,
    particle_budget: int | None = None,
    revision: str | None = None,
    environment_version: str | None = None,
) -> list[str]:
    """
    Generates 24 renders (8 angles x 3 elevation levels) of the component.
    Saves to output_dir.
    """
    from worker_heavy.config import settings

    if smoke_test_mode is None:
        smoke_test_mode = settings.smoke_test_mode
    requested_backend = backend_type or get_default_simulator_backend()
    requested_backend_value = (
        requested_backend.value
        if hasattr(requested_backend, "value")
        else str(requested_backend)
    )

    if output_dir is None:
        output_dir = os.getenv("RENDERS_DIR", "./renders")

    render_policy = load_agents_config().render
    output_path = Path(output_dir)
    logger.info(
        "prerender_24_views_start",
        output_dir=str(output_path),
        backend=PREVIEW_BACKEND_NAME,
        requested_backend=requested_backend_value,
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

    try:
        workspace_root = output_path.parent

        emit_event(
            {
                "event_type": "validation_preview_backend_selected",
                "requested_physics_backend": requested_backend_value,
                "actual_preview_backend": PREVIEW_BACKEND_NAME,
                "purpose": "validation_static_preview",
                "session_id": session_id,
            }
        )

        preview_start = time.time()
        saved_paths, legend_by_path = render_preview_bundle(
            component,
            output_dir=output_path,
            objectives=objectives,
            smoke_test_mode=smoke_test_mode,
            workspace_root=workspace_root,
            include_rgb=render_policy.rgb,
            include_depth=render_policy.depth,
            include_segmentation=render_policy.segmentation,
        )
        manifest_artifacts, render_paths = _build_preview_artifacts(
            saved_paths,
            legend_by_path,
            workspace_root=workspace_root,
        )
        manifest = build_render_manifest(
            manifest_artifacts,
            workspace_root=workspace_root,
            episode_id=session_id,
            worker_session_id=session_id,
            revision=revision,
            environment_version=environment_version,
        )

        manifest_path = output_path / "render_manifest.json"
        manifest_path.write_text(
            manifest.model_dump_json(indent=2),
            encoding="utf-8",
        )

        emit_event(
            {
                "event_type": "validation_preview_render_complete",
                "preview_backend": PREVIEW_BACKEND_NAME,
                "image_count": len(render_paths),
                "elapsed_render_time": time.time() - preview_start,
                "artifact_paths": render_paths,
                "session_id": session_id,
            }
        )

        saved_files = list(render_paths)

        logger.info(
            "prerender_complete",
            count=len(saved_files),
            session_id=session_id,
            backend=PREVIEW_BACKEND_NAME,
        )
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
