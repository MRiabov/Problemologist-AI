from __future__ import annotations

import asyncio
import base64
import contextlib
import os
import tempfile
from pathlib import Path
from typing import Any

import structlog
from fastapi import APIRouter, Header, HTTPException

from shared.agents.config import load_agents_config
from shared.models.schemas import BenchmarkDefinition
from shared.workers.bundling import extract_bundle_base64
from shared.workers.loader import load_component_from_script
from shared.workers.persistence import collect_and_cleanup_events
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    PreviewDesignRequest,
    PreviewDesignResponse,
    RenderArtifactMetadata,
    RenderSiblingPaths,
    SegmentationLegendEntry,
    SimulationArtifacts,
    SimulationVideoRequest,
)
from worker_heavy.utils.build123d_rendering import (
    PreviewScene,
    render_preview_scene,
    render_preview_scene_bundle,
)
from worker_heavy.utils.file_validation import validate_benchmark_definition_yaml
from worker_heavy.utils.preview import preview_design
from worker_heavy.utils.rendering import build_render_manifest, prerender_24_views

logger = structlog.get_logger(__name__)
renderer_router = APIRouter()

_RENDER_ADMISSION_LOCK = asyncio.Lock()
_RENDER_BUSY = False
_RENDER_BUSY_CONTEXT: dict[str, str] = {}


def is_renderer_busy() -> bool:
    return _RENDER_BUSY


def renderer_busy_context() -> dict[str, str]:
    return dict(_RENDER_BUSY_CONTEXT)


def _busy_detail() -> dict[str, Any]:
    return {
        "code": "WORKER_BUSY",
        "message": "Renderer worker already has an active job",
        "active_job": renderer_busy_context(),
    }


@contextlib.asynccontextmanager
async def render_operation_admission(operation: str, session_id: str):
    global _RENDER_BUSY, _RENDER_BUSY_CONTEXT

    async with _RENDER_ADMISSION_LOCK:
        if _RENDER_BUSY:
            raise HTTPException(status_code=503, detail=_busy_detail())
        _RENDER_BUSY = True
        _RENDER_BUSY_CONTEXT = {
            "operation": operation,
            "session_id": session_id,
        }

    try:
        yield
    finally:
        async with _RENDER_ADMISSION_LOCK:
            _RENDER_BUSY = False
            _RENDER_BUSY_CONTEXT = {}


@contextlib.contextmanager
def _event_file_context(root: Path):
    previous = os.environ.get("EVENTS_FILE")
    os.environ["EVENTS_FILE"] = str(root / "events.jsonl")
    try:
        yield
    finally:
        if previous is None:
            os.environ.pop("EVENTS_FILE", None)
        else:
            os.environ["EVENTS_FILE"] = previous


@contextlib.contextmanager
def _bundle_context(bundle_base64: str | None):
    if not bundle_base64:
        with tempfile.TemporaryDirectory() as tmpdir:
            yield Path(tmpdir)
        return

    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        extract_bundle_base64(bundle_base64, root)
        yield root


def _load_workspace_benchmark_definition(
    root: Path, *, session_id: str | None = None
) -> BenchmarkDefinition | None:
    benchmark_path = root / "benchmark_definition.yaml"
    if not benchmark_path.exists():
        return None

    raw = benchmark_path.read_text(encoding="utf-8")
    is_valid, objectives_or_errors = validate_benchmark_definition_yaml(
        raw, session_id=session_id
    )
    if not is_valid:
        raise ValueError("; ".join(objectives_or_errors))
    return objectives_or_errors


def _load_preview_scene(root: Path) -> PreviewScene | None:
    scene_path = root / "preview_scene.json"
    if not scene_path.exists():
        return None
    scene = PreviewScene.model_validate_json(scene_path.read_text(encoding="utf-8"))
    for entity in scene.entities:
        if not entity.mesh_paths:
            continue
        entity.mesh_paths = [
            str((root / Path(mesh_path)).resolve())
            if not Path(mesh_path).is_absolute()
            else str(Path(mesh_path).resolve())
            for mesh_path in entity.mesh_paths
        ]
    return scene


def _collect_render_artifacts(
    root: Path, render_paths: list[str], *, session_id: str | None = None
) -> SimulationArtifacts:
    normalized_render_paths: list[str] = []
    render_blobs_base64: dict[str, str] = {}
    render_image_paths: list[str] = []

    for raw_path in render_paths:
        candidate = Path(raw_path)
        if candidate.is_absolute():
            try:
                rel_path = candidate.resolve().relative_to(root.resolve())
            except Exception:
                rel_path = Path(candidate.name)
            render_path = candidate
        else:
            rel_path = candidate
            render_path = root / candidate
        if not render_path.exists() or not render_path.is_file():
            continue
        suffix = render_path.suffix.lower()
        if suffix not in {".png", ".jpg", ".jpeg", ".mp4"}:
            continue
        rel_key = str(rel_path)
        normalized_render_paths.append(rel_key)
        render_blobs_base64[rel_key] = base64.b64encode(
            render_path.read_bytes()
        ).decode("ascii")
        if suffix in {".png", ".jpg", ".jpeg"}:
            render_image_paths.append(rel_key)

    render_manifest_path = root / "renders" / "render_manifest.json"
    if render_manifest_path.exists():
        render_blobs_base64[str(Path("renders") / "render_manifest.json")] = (
            base64.b64encode(render_manifest_path.read_bytes()).decode("ascii")
        )
    elif render_image_paths:
        synthesized_manifest = build_render_manifest(
            {
                path: RenderArtifactMetadata(modality="rgb")
                for path in sorted(dict.fromkeys(render_image_paths))
            },
            workspace_root=root,
            episode_id=session_id,
            worker_session_id=session_id,
        )
        render_blobs_base64[str(Path("renders") / "render_manifest.json")] = (
            base64.b64encode(
                synthesized_manifest.model_dump_json(indent=2).encode("utf-8")
            ).decode("ascii")
        )

    artifacts = SimulationArtifacts(render_paths=normalized_render_paths)
    artifacts.render_blobs_base64 = render_blobs_base64
    return artifacts


def _build_preview_manifest(
    *,
    root: Path,
    saved_paths: list[str],
    legend_by_path: dict[str, list[SegmentationLegendEntry]],
    session_id: str | None,
) -> Path:
    artifacts: dict[str, RenderArtifactMetadata] = {}
    preview_evidence_paths: list[str] = []

    for saved_path in saved_paths:
        rel_path = str(Path(saved_path))
        filename = Path(rel_path).name
        render_dir = Path(rel_path).parent
        stem = Path(rel_path).stem
        if filename.endswith("_depth.png"):
            group_key = stem.removesuffix("_depth")
            artifacts[rel_path] = RenderArtifactMetadata(
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
        elif filename.endswith("_segmentation.png"):
            group_key = stem.removesuffix("_segmentation")
            artifacts[rel_path] = RenderArtifactMetadata(
                modality="segmentation",
                group_key=group_key,
                siblings=RenderSiblingPaths(
                    rgb=str(render_dir / f"{group_key}.png"),
                    depth=str(render_dir / f"{group_key}_depth.png"),
                    segmentation=str(render_dir / f"{group_key}_segmentation.png"),
                ),
                segmentation_legend=list(legend_by_path.get(rel_path, [])),
            )
        else:
            group_key = stem
            artifacts[rel_path] = RenderArtifactMetadata(
                modality="rgb",
                group_key=group_key,
                siblings=RenderSiblingPaths(
                    rgb=str(render_dir / f"{group_key}.png"),
                    depth=str(render_dir / f"{group_key}_depth.png"),
                    segmentation=str(render_dir / f"{group_key}_segmentation.png"),
                ),
            )
        preview_evidence_paths.append(rel_path)

    manifest = build_render_manifest(
        artifacts,
        workspace_root=root,
        episode_id=session_id,
        worker_session_id=session_id,
        preview_evidence_paths=preview_evidence_paths,
    )
    manifest_path = root / "renders" / "render_manifest.json"
    manifest_path.write_text(manifest.model_dump_json(indent=2), encoding="utf-8")
    return manifest_path


def _encode_simulation_video(
    root: Path,
    *,
    frame_paths: list[str],
    output_name: str,
    fps: int,
) -> Path:
    if not frame_paths:
        raise ValueError("simulation video requires at least one captured frame")

    import cv2

    renders_dir = root / "renders"
    renders_dir.mkdir(parents=True, exist_ok=True)
    output_path = renders_dir / output_name

    first_frame_path = root / frame_paths[0]
    first_frame = cv2.imread(str(first_frame_path))
    if first_frame is None:
        raise ValueError(f"unable to read simulation frame: {frame_paths[0]}")

    height, width = first_frame.shape[:2]
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    try:
        for rel_path in frame_paths:
            frame_path = root / rel_path
            frame = cv2.imread(str(frame_path))
            if frame is None:
                raise ValueError(f"unable to read simulation frame: {rel_path}")
            if frame.shape[:2] != (height, width):
                frame = cv2.resize(frame, (width, height))
            writer.write(frame)
    finally:
        writer.release()

    logger.info(
        "renderer_simulation_video_complete",
        output_path=str(output_path),
        frame_count=len(frame_paths),
    )
    return output_path


@renderer_router.post("/benchmark/preview", response_model=PreviewDesignResponse)
async def api_preview(
    request: PreviewDesignRequest,
    x_session_id: str = Header(default="renderer"),
):
    """Render a single inspection preview in a dedicated renderer process."""
    try:
        async with render_operation_admission("preview", x_session_id):
            with _bundle_context(request.bundle_base64) as root:
                with _event_file_context(root):
                    objectives = _load_workspace_benchmark_definition(
                        root, session_id=x_session_id
                    )

                    renders_dir = root / "renders"
                    renders_dir.mkdir(parents=True, exist_ok=True)
                    scene = _load_preview_scene(root)
                    if scene is not None:
                        image_path = await asyncio.to_thread(
                            render_preview_scene,
                            scene,
                            pitch=request.pitch,
                            yaw=request.yaw,
                            output_dir=renders_dir,
                        )
                    else:
                        component = load_component_from_script(
                            script_path=root / request.script_path,
                            session_root=root,
                            script_content=request.script_content,
                        )
                        image_path = await asyncio.to_thread(
                            preview_design,
                            component,
                            pitch=request.pitch,
                            yaw=request.yaw,
                            output_dir=renders_dir,
                            objectives=objectives,
                        )

                events = collect_and_cleanup_events(root, session_id=x_session_id)
                return PreviewDesignResponse(
                    success=True,
                    message="Preview generated successfully",
                    image_path=str(image_path.relative_to(root)),
                    image_bytes_base64=base64.b64encode(image_path.read_bytes()).decode(
                        "ascii"
                    ),
                    events=events,
                )
    except HTTPException:
        raise
    except Exception as exc:
        logger.warning(
            "renderer_preview_failed", error=str(exc), session_id=x_session_id
        )
        return PreviewDesignResponse(success=False, message=str(exc))


@renderer_router.post("/benchmark/static-preview", response_model=BenchmarkToolResponse)
async def api_static_preview(
    request: BenchmarkToolRequest,
    x_session_id: str = Header(default="renderer"),
):
    """Render the multi-view validation preview bundle in a dedicated process."""
    try:
        async with render_operation_admission("static-preview", x_session_id):
            render_policy = load_agents_config().render
            if not any(
                (
                    render_policy.rgb.enabled,
                    render_policy.depth.enabled,
                    render_policy.segmentation.enabled,
                )
            ):
                raise ValueError(
                    "validation preview requires at least one enabled render "
                    "modality (rgb, depth, or segmentation)"
                )
            with _bundle_context(request.bundle_base64) as root:
                with _event_file_context(root):
                    objectives = _load_workspace_benchmark_definition(
                        root, session_id=x_session_id
                    )

                    renders_dir = root / "renders"
                    renders_dir.mkdir(parents=True, exist_ok=True)
                    scene = _load_preview_scene(root)
                    if scene is not None:
                        saved_paths, legend_by_path = await asyncio.to_thread(
                            render_preview_scene_bundle,
                            scene,
                            output_dir=renders_dir,
                            smoke_test_mode=bool(request.smoke_test_mode),
                            include_rgb=render_policy.rgb.enabled,
                            include_depth=render_policy.depth.enabled,
                            include_segmentation=render_policy.segmentation.enabled,
                            rgb_axes=render_policy.rgb.axes,
                            rgb_edges=render_policy.rgb.edges,
                            depth_axes=render_policy.depth.axes,
                            depth_edges=render_policy.depth.edges,
                            segmentation_axes=render_policy.segmentation.axes,
                            segmentation_edges=render_policy.segmentation.edges,
                        )
                        _build_preview_manifest(
                            root=root,
                            saved_paths=saved_paths,
                            legend_by_path=legend_by_path,
                            session_id=x_session_id,
                        )
                    else:
                        component = load_component_from_script(
                            script_path=root / request.script_path,
                            session_root=root,
                            script_content=request.script_content,
                        )
                        saved_paths, _legend_by_path = await asyncio.to_thread(
                            prerender_24_views,
                            component,
                            output_dir=renders_dir,
                            objectives=objectives,
                            backend_type=request.backend,
                            session_id=x_session_id,
                            smoke_test_mode=bool(request.smoke_test_mode),
                            particle_budget=request.particle_budget,
                        )

                events = collect_and_cleanup_events(root, session_id=x_session_id)
                return BenchmarkToolResponse(
                    success=True,
                    message="Static preview generated successfully",
                    artifacts=_collect_render_artifacts(
                        root, saved_paths, session_id=x_session_id
                    ),
                    events=events,
                )
    except HTTPException:
        raise
    except Exception as exc:
        logger.warning(
            "renderer_static_preview_failed", error=str(exc), session_id=x_session_id
        )
        return BenchmarkToolResponse(success=False, message=str(exc))


@renderer_router.post(
    "/benchmark/simulation-video", response_model=BenchmarkToolResponse
)
async def api_simulation_video(
    request: SimulationVideoRequest,
    x_session_id: str = Header(default="renderer"),
):
    """Encode captured simulation frames into an MP4 in the renderer worker."""
    try:
        async with render_operation_admission("simulation-video", x_session_id):
            with _bundle_context(request.bundle_base64) as root:
                with _event_file_context(root):
                    output_path = await asyncio.to_thread(
                        _encode_simulation_video,
                        root,
                        frame_paths=list(request.frame_paths),
                        output_name=request.output_name,
                        fps=request.fps,
                    )

                manifest_path = root / "renders" / "render_manifest.json"
                output_rel_path = str(output_path.relative_to(root))
                manifest = build_render_manifest(
                    {
                        output_rel_path: RenderArtifactMetadata(
                            modality="unknown",
                            group_key=Path(output_rel_path).stem,
                        )
                    },
                    workspace_root=root,
                    episode_id=request.session_id or x_session_id,
                    worker_session_id=x_session_id,
                )
                manifest_path.write_text(
                    manifest.model_dump_json(indent=2),
                    encoding="utf-8",
                )

                events = collect_and_cleanup_events(root, session_id=x_session_id)
                return BenchmarkToolResponse(
                    success=True,
                    message="Simulation video generated successfully",
                    artifacts=_collect_render_artifacts(
                        root, [output_rel_path], session_id=x_session_id
                    ),
                    events=events,
                )
    except HTTPException:
        raise
    except Exception as exc:
        logger.warning(
            "renderer_simulation_video_failed", error=str(exc), session_id=x_session_id
        )
        return BenchmarkToolResponse(success=False, message=str(exc))
