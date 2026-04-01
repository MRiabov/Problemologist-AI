from __future__ import annotations

import asyncio
import base64
import contextlib
import os
import tempfile
import time
import uuid
from pathlib import Path
from typing import Any

import structlog
from fastapi import APIRouter, Header, HTTPException
from PIL import Image

from shared.agents.config import load_agents_config
from shared.git_utils import repo_revision
from shared.models.schemas import BenchmarkDefinition
from shared.workers.bundling import extract_bundle_base64
from shared.workers.loader import load_component_from_script
from shared.workers.persistence import collect_and_cleanup_events
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    PreviewDesignRequest,
    PreviewDesignResponse,
    PreviewRenderingType,
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
    SegmentationLegendEntry,
    SimulationArtifacts,
    SimulationVideoRequest,
)
from worker_heavy.utils.build123d_rendering import (
    _OVERLAY_AXES_COLOR,
    _OVERLAY_EDGE_COLOR,
    _RGB_AXES_COLOR,
    _RGB_EDGE_COLOR,
    PreviewScene,
    _build_renderer,
    _composite_non_black,
    _depth_buffer_to_display_rgb,
    _preview_camera_distance,
    _preview_segmentation_legend,
    _render_view,
    camera_position_from_orbit,
    collect_preview_scene,
    render_preview_bundle,
    render_preview_scene_bundle,
)
from worker_heavy.utils.file_validation import validate_benchmark_definition_yaml
from worker_heavy.utils.rendering import (
    build_render_manifest,
    normalize_render_manifest,
    select_single_preview_render_subdir,
    select_static_preview_render_subdir,
)

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

    render_manifest_path = root / "renders" / "render_manifest.json"
    existing_manifest = None
    if render_manifest_path.exists():
        with contextlib.suppress(Exception):
            existing_manifest = RenderManifest.model_validate_json(
                render_manifest_path.read_text(encoding="utf-8")
            )

    if normalized_render_paths:
        resolved_revision = (
            os.environ.get("REPO_REVISION")
            or repo_revision(Path.cwd())
            or repo_revision(root)
            or repo_revision(Path(__file__).resolve().parents[2])
        )
        synthesized_manifest = normalize_render_manifest(
            render_paths=normalized_render_paths,
            workspace_root=root,
            existing_manifest=existing_manifest,
            episode_id=session_id,
            worker_session_id=session_id,
            revision=resolved_revision,
        )
        render_blobs_base64[str(Path("renders") / "render_manifest.json")] = (
            base64.b64encode(
                synthesized_manifest.model_dump_json(indent=2).encode("utf-8")
            ).decode("ascii")
        )

    artifacts = SimulationArtifacts(render_paths=normalized_render_paths)
    artifacts.render_blobs_base64 = render_blobs_base64
    return artifacts


def _write_text_atomic(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=str(path.parent), delete=False
    ) as tmp:
        tmp.write(content)
        tmp_path = Path(tmp.name)
    tmp_path.replace(path)


def _single_preview_group_key(pitch: float, yaw: float) -> str:
    return f"render_e{abs(int(round(pitch)))}_a{int(round(yaw))}"


def _resolve_single_preview_group_key(
    output_dir: Path, pitch: float, yaw: float
) -> str:
    base_key = _single_preview_group_key(pitch, yaw)
    candidate_paths = (
        output_dir / f"{base_key}.jpg",
        output_dir / f"{base_key}_depth.png",
        output_dir / f"{base_key}_segmentation.png",
    )
    if not any(path.exists() for path in candidate_paths):
        return base_key

    timestamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    suffix = uuid.uuid4().hex[:8]
    return f"{base_key}_{timestamp}_{suffix}"


def _render_single_preview(
    scene: PreviewScene,
    *,
    output_dir: Path,
    pitch: float,
    yaw: float,
    rendering_type: PreviewRenderingType,
    include_rgb_axes: bool,
    include_rgb_edges: bool,
    include_depth_axes: bool,
    include_depth_edges: bool,
    include_segmentation_axes: bool,
    include_segmentation_edges: bool,
) -> tuple[Path, dict[str, RenderArtifactMetadata]]:
    center = scene.center
    distance = _preview_camera_distance(scene, width=640, height=480)
    camera_position = camera_position_from_orbit(center, distance, pitch, yaw)
    group_key = _single_preview_group_key(pitch, yaw)
    workspace_root = output_dir.parent.parent

    output_dir.mkdir(parents=True, exist_ok=True)
    artifacts: dict[str, RenderArtifactMetadata] = {}
    group_key = _resolve_single_preview_group_key(output_dir, pitch, yaw)

    if rendering_type == PreviewRenderingType.RGB:
        bundle = _build_renderer(
            scene,
            width=640,
            height=480,
            segmentation=False,
            include_axes=include_rgb_axes,
            include_edges=include_rgb_edges,
            include_fill=True,
            axes_color=_RGB_AXES_COLOR,
            edge_color=_RGB_EDGE_COLOR,
            background=(0.98, 0.98, 0.99),
        )
        try:
            rgb_image, _ = _render_view(
                bundle,
                camera_position=camera_position,
                lookat=center,
                up=(0.0, 0.0, 1.0),
                capture_depth=False,
            )
        finally:
            del bundle

        image_path = output_dir / f"{group_key}.jpg"
        Image.fromarray(rgb_image).convert("RGB").save(image_path, "JPEG")
        rel_path = str(image_path.relative_to(workspace_root))
        artifacts[rel_path] = RenderArtifactMetadata(
            modality="rgb",
            group_key=group_key,
            siblings=RenderSiblingPaths(
                rgb=str(image_path.relative_to(workspace_root)),
                depth=str(
                    (output_dir / f"{group_key}_depth.png").relative_to(workspace_root)
                ),
                segmentation=str(
                    (output_dir / f"{group_key}_segmentation.png").relative_to(
                        workspace_root
                    )
                ),
            ),
        )
        return image_path, artifacts

    if rendering_type == PreviewRenderingType.DEPTH:
        depth_base_bundle = _build_renderer(
            scene,
            width=640,
            height=480,
            segmentation=False,
            include_axes=False,
            include_edges=False,
            include_fill=True,
            axes_color=_OVERLAY_AXES_COLOR,
            edge_color=_OVERLAY_EDGE_COLOR,
            background=(0.98, 0.98, 0.99),
        )
        depth_overlay_bundle: Any | None = None
        if include_depth_axes or include_depth_edges:
            depth_overlay_bundle = _build_renderer(
                scene,
                width=640,
                height=480,
                segmentation=False,
                include_axes=include_depth_axes,
                include_edges=include_depth_edges,
                include_fill=False,
                axes_color=_OVERLAY_AXES_COLOR,
                edge_color=_OVERLAY_EDGE_COLOR,
                background=(0.0, 0.0, 0.0),
            )
        try:
            _, depth_buffer = _render_view(
                depth_base_bundle,
                camera_position=camera_position,
                lookat=center,
                up=(0.0, 0.0, 1.0),
                capture_depth=True,
            )
            if depth_buffer is None:
                raise RuntimeError("depth rendering was disabled for render")
            depth_image, depth_range = _depth_buffer_to_display_rgb(depth_buffer)
            if depth_overlay_bundle is not None:
                depth_overlay, _ = _render_view(
                    depth_overlay_bundle,
                    camera_position=camera_position,
                    lookat=center,
                    up=(0.0, 0.0, 1.0),
                    capture_depth=False,
                )
                depth_image = _composite_non_black(depth_image, depth_overlay)
        finally:
            del depth_base_bundle
            if depth_overlay_bundle is not None:
                del depth_overlay_bundle

        image_path = output_dir / f"{group_key}_depth.png"
        Image.fromarray(depth_image, mode="RGB").save(image_path, "PNG")
        rel_path = str(image_path.relative_to(workspace_root))
        artifacts[rel_path] = RenderArtifactMetadata(
            modality="depth",
            group_key=group_key,
            siblings=RenderSiblingPaths(
                rgb=str((output_dir / f"{group_key}.jpg").relative_to(workspace_root)),
                depth=str(image_path.relative_to(workspace_root)),
                segmentation=str(
                    (output_dir / f"{group_key}_segmentation.png").relative_to(
                        workspace_root
                    )
                ),
            ),
            depth_interpretation=(
                "Camera-space depth in meters. False-color pixels are scaled "
                "from the build123d/VTK preview renderer's linear depth "
                "buffer; see depth_min_m and depth_max_m for the metric range."
            ),
            depth_min_m=depth_range[0],
            depth_max_m=depth_range[1],
        )
        return image_path, artifacts

    seg_base_bundle = _build_renderer(
        scene,
        width=640,
        height=480,
        segmentation=True,
        include_axes=False,
        include_edges=False,
        include_fill=True,
        axes_color=_OVERLAY_AXES_COLOR,
        edge_color=_OVERLAY_EDGE_COLOR,
        background=(0.0, 0.0, 0.0),
    )
    seg_overlay_bundle: Any | None = None
    if include_segmentation_axes or include_segmentation_edges:
        seg_overlay_bundle = _build_renderer(
            scene,
            width=640,
            height=480,
            segmentation=False,
            include_axes=include_segmentation_axes,
            include_edges=include_segmentation_edges,
            include_fill=False,
            axes_color=_OVERLAY_AXES_COLOR,
            edge_color=_OVERLAY_EDGE_COLOR,
            background=(0.0, 0.0, 0.0),
        )
    try:
        seg_image, _ = _render_view(
            seg_base_bundle,
            camera_position=camera_position,
            lookat=center,
            up=(0.0, 0.0, 1.0),
            capture_depth=False,
        )
        if seg_overlay_bundle is not None:
            seg_overlay, _ = _render_view(
                seg_overlay_bundle,
                camera_position=camera_position,
                lookat=center,
                up=(0.0, 0.0, 1.0),
                capture_depth=False,
            )
            seg_image = _composite_non_black(seg_image, seg_overlay)
    finally:
        del seg_base_bundle
        if seg_overlay_bundle is not None:
            del seg_overlay_bundle

    image_path = output_dir / f"{group_key}_segmentation.png"
    Image.fromarray(seg_image, mode="RGB").save(image_path, "PNG")
    rel_path = str(image_path.relative_to(workspace_root))
    artifacts[rel_path] = RenderArtifactMetadata(
        modality="segmentation",
        group_key=group_key,
        siblings=RenderSiblingPaths(
            rgb=str((output_dir / f"{group_key}.jpg").relative_to(workspace_root)),
            depth=str(
                (output_dir / f"{group_key}_depth.png").relative_to(workspace_root)
            ),
            segmentation=str(image_path.relative_to(workspace_root)),
        ),
        segmentation_legend=_preview_segmentation_legend(scene),
    )
    return image_path, artifacts


def _build_preview_manifest(
    *,
    root: Path,
    saved_paths: list[str],
    legend_by_path: dict[str, list[SegmentationLegendEntry]],
    depth_ranges_by_path: dict[str, tuple[float, float]] | None,
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
            depth_range = None
            if depth_ranges_by_path is not None:
                depth_range = depth_ranges_by_path.get(rel_path)
            artifacts[rel_path] = RenderArtifactMetadata(
                modality="depth",
                group_key=group_key,
                siblings=RenderSiblingPaths(
                    rgb=str(render_dir / f"{group_key}.png"),
                    depth=str(render_dir / f"{group_key}_depth.png"),
                    segmentation=str(render_dir / f"{group_key}_segmentation.png"),
                ),
                depth_interpretation=(
                    "Camera-space depth in meters. False-color pixels are scaled "
                    "from the build123d/VTK preview renderer's linear depth "
                    "buffer; see depth_min_m and depth_max_m for the metric "
                    "range."
                ),
                depth_min_m=depth_range[0] if depth_range is not None else None,
                depth_max_m=depth_range[1] if depth_range is not None else None,
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
    _write_text_atomic(manifest_path, manifest.model_dump_json(indent=2))
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

                    renders_dir = (
                        root / "renders" / select_single_preview_render_subdir(root)
                    )
                    render_policy = load_agents_config().render
                    if (
                        request.rendering_type == PreviewRenderingType.RGB
                        and not render_policy.rgb.enabled
                    ):
                        raise ValueError("rgb preview rendering is disabled")
                    if (
                        request.rendering_type == PreviewRenderingType.DEPTH
                        and not render_policy.depth.enabled
                    ):
                        raise ValueError("depth preview rendering is disabled")
                    if (
                        request.rendering_type == PreviewRenderingType.SEGMENTATION
                        and not render_policy.segmentation.enabled
                    ):
                        raise ValueError("segmentation preview rendering is disabled")
                    scene = _load_preview_scene(root)
                    if scene is not None:
                        image_path, artifacts = await asyncio.to_thread(
                            _render_single_preview,
                            scene,
                            output_dir=renders_dir,
                            pitch=request.orbit_pitch,
                            yaw=request.orbit_yaw,
                            rendering_type=request.rendering_type,
                            include_rgb_axes=render_policy.rgb.axes,
                            include_rgb_edges=render_policy.rgb.edges,
                            include_depth_axes=render_policy.depth.axes,
                            include_depth_edges=render_policy.depth.edges,
                            include_segmentation_axes=render_policy.segmentation.axes,
                            include_segmentation_edges=render_policy.segmentation.edges,
                        )
                    else:
                        component = load_component_from_script(
                            script_path=root / request.script_path,
                            session_root=root,
                            script_content=request.script_content,
                        )
                        with tempfile.TemporaryDirectory() as mesh_tmpdir:
                            preview_scene = await asyncio.to_thread(
                                collect_preview_scene,
                                component,
                                objectives=objectives,
                                workspace_root=root,
                                smoke_test_mode=bool(request.smoke_test_mode),
                                mesh_root=Path(mesh_tmpdir),
                            )
                            image_path, artifacts = await asyncio.to_thread(
                                _render_single_preview,
                                preview_scene,
                                output_dir=renders_dir,
                                pitch=request.orbit_pitch,
                                yaw=request.orbit_yaw,
                                rendering_type=request.rendering_type,
                                include_rgb_axes=render_policy.rgb.axes,
                                include_rgb_edges=render_policy.rgb.edges,
                                include_depth_axes=render_policy.depth.axes,
                                include_depth_edges=render_policy.depth.edges,
                                include_segmentation_axes=render_policy.segmentation.axes,
                                include_segmentation_edges=render_policy.segmentation.edges,
                            )

                preview_manifest = build_render_manifest(
                    artifacts,
                    workspace_root=root,
                    episode_id=x_session_id,
                    worker_session_id=x_session_id,
                    preview_evidence_paths=list(artifacts.keys()),
                )
                manifest_json = preview_manifest.model_dump_json(indent=2)
                _write_text_atomic(
                    root / "renders" / "render_manifest.json", manifest_json
                )

                events = collect_and_cleanup_events(root, session_id=x_session_id)
                return PreviewDesignResponse(
                    success=True,
                    status_text="Preview generated successfully",
                    message="Preview generated successfully",
                    artifact_path=str(image_path.relative_to(root)),
                    manifest_path=str(Path("renders") / "render_manifest.json"),
                    rendering_type=request.rendering_type,
                    pitch=request.orbit_pitch,
                    yaw=request.orbit_yaw,
                    image_path=str(image_path.relative_to(root)),
                    image_bytes_base64=base64.b64encode(image_path.read_bytes()).decode(
                        "ascii"
                    ),
                    render_manifest_json=manifest_json,
                    events=events,
                )
    except HTTPException:
        raise
    except Exception as exc:
        logger.warning(
            "renderer_preview_failed", error=str(exc), session_id=x_session_id
        )
        return PreviewDesignResponse(
            success=False,
            status_text="Preview generation failed",
            message=str(exc),
            rendering_type=request.rendering_type,
            pitch=request.orbit_pitch,
            yaw=request.orbit_yaw,
        )


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

                    renders_dir = (
                        root / "renders" / select_static_preview_render_subdir(root)
                    )
                    renders_dir.mkdir(parents=True, exist_ok=True)
                    scene = _load_preview_scene(root)
                    if scene is not None:
                        render_result = await asyncio.to_thread(
                            render_preview_scene_bundle,
                            scene,
                            output_dir=renders_dir,
                            workspace_root=root,
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
                            saved_paths=render_result.saved_paths,
                            legend_by_path=render_result.legend_by_path,
                            depth_ranges_by_path=render_result.depth_ranges_by_path,
                            session_id=x_session_id,
                        )
                    else:
                        component = load_component_from_script(
                            script_path=root / request.script_path,
                            session_root=root,
                            script_content=request.script_content,
                        )
                        render_result = await asyncio.to_thread(
                            render_preview_bundle,
                            component,
                            output_dir=renders_dir,
                            objectives=objectives,
                            smoke_test_mode=bool(request.smoke_test_mode),
                            workspace_root=root,
                            rgb_axes=render_policy.rgb.axes,
                            rgb_edges=render_policy.rgb.edges,
                            depth_axes=render_policy.depth.axes,
                            depth_edges=render_policy.depth.edges,
                            segmentation_axes=render_policy.segmentation.axes,
                            segmentation_edges=render_policy.segmentation.edges,
                            include_rgb=render_policy.rgb.enabled,
                            include_depth=render_policy.depth.enabled,
                            include_segmentation=render_policy.segmentation.enabled,
                        )
                        _build_preview_manifest(
                            root=root,
                            saved_paths=render_result.saved_paths,
                            legend_by_path=render_result.legend_by_path,
                            depth_ranges_by_path=render_result.depth_ranges_by_path,
                            session_id=x_session_id,
                        )

                events = collect_and_cleanup_events(root, session_id=x_session_id)
                return BenchmarkToolResponse(
                    success=True,
                    message="Static preview generated successfully",
                    artifacts=_collect_render_artifacts(
                        root, render_result.saved_paths, session_id=x_session_id
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
