from __future__ import annotations

import base64
import hashlib
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import TYPE_CHECKING, Any

import matplotlib.pyplot as plt
import structlog
import yaml
from build123d import ExportDXF, ExportSVG, LineType, Part
from build123d.build_enums import Unit

from shared.agents import load_agents_config
from shared.models.schemas import AssemblyDefinition, DraftingView
from shared.rendering import build_render_manifest
from shared.workers.loader import load_component_from_script
from shared.workers.schema import (
    PreviewDesignResponse,
    PreviewRenderingType,
    PreviewViewSpec,
    RenderArtifactMetadata,
    RenderSiblingPaths,
)
from worker_renderer.utils.scene_builder import normalize_preview_label

logger = structlog.get_logger(__name__)

if TYPE_CHECKING:
    from shared.observability.storage import S3Client


_VIEW_CAMERA_MAP: dict[
    str,
    tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
        float,
        float,
    ],
] = {
    "front": ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), 0.0, 0.0),
    "top": ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.0, 1.0, 0.0), -90.0, 0.0),
    "side": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 0.0, 90.0),
    "section": (
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, 1.0),
        0.0,
        0.0,
    ),
    "detail": (
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, 1.0),
        0.0,
        0.0,
    ),
    "isometric": (
        (1.0, -1.0, 1.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, 1.0),
        35.264,
        45.0,
    ),
}


def _vector_tuple(value: Any) -> tuple[float, float, float]:
    if hasattr(value, "X") and hasattr(value, "Y") and hasattr(value, "Z"):
        return (float(value.X), float(value.Y), float(value.Z))
    if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
        return (float(value.x), float(value.y), float(value.z))
    if isinstance(value, (tuple, list)) and len(value) == 3:
        return (float(value[0]), float(value[1]), float(value[2]))
    raise ValueError(f"Unsupported vector value: {value!r}")


def _sanitize_filename(text: str) -> str:
    cleaned = normalize_preview_label(text) or "drawing"
    return cleaned.replace("/", "_").replace("\\", "_")


def _load_assembly_definition(root: Path) -> AssemblyDefinition:
    assembly_path = root / "assembly_definition.yaml"
    if not assembly_path.exists():
        raise FileNotFoundError(
            "assembly_definition.yaml is required for drafting preview"
        )

    raw = assembly_path.read_text(encoding="utf-8")
    data = yaml.safe_load(raw) or {}
    definition = AssemblyDefinition.model_validate(data)
    if definition.drafting is None:
        raise ValueError(
            "assembly_definition.yaml.drafting is required when drafting preview is requested"
        )
    return definition


def _script_source_sha256(
    script_path: Path,
    *,
    script_content: str | None = None,
) -> str:
    if script_content is not None:
        return hashlib.sha256(script_content.encode("utf-8")).hexdigest()
    return hashlib.sha256(script_path.read_bytes()).hexdigest()


def _preview_storage_client() -> S3Client | None:
    try:
        from shared.observability.storage import S3Client, S3Config
    except ModuleNotFoundError:
        return None

    access_key = os.getenv("S3_ACCESS_KEY", os.getenv("AWS_ACCESS_KEY_ID"))
    secret_key = os.getenv("S3_SECRET_KEY", os.getenv("AWS_SECRET_ACCESS_KEY"))
    if not access_key or not secret_key:
        return None

    config = S3Config(
        endpoint_url=os.getenv("S3_ENDPOINT"),
        access_key_id=access_key,
        secret_access_key=secret_key,
        bucket_name=os.getenv("ASSET_S3_BUCKET", "problemologist"),
        region_name=os.getenv("AWS_REGION", "us-east-1"),
    )
    return S3Client(config)


def _project_edges(
    component: Part,
    view: DraftingView,
    *,
    look_at: tuple[float, float, float],
    focus: float,
) -> tuple[list[Any], list[Any], PreviewViewSpec]:
    camera_origin, viewport_up, _viewport_axis, pitch, yaw = _VIEW_CAMERA_MAP.get(
        view.projection, _VIEW_CAMERA_MAP["front"]
    )
    visible_edges, hidden_edges = component.project_to_viewport(
        camera_origin,
        viewport_up=viewport_up,
        look_at=look_at,
        focus=focus / max(view.scale, 1e-6),
    )
    return (
        list(visible_edges),
        list(hidden_edges),
        PreviewViewSpec(view_index=0, orbit_pitch=pitch, orbit_yaw=yaw),
    )


def _edge_segments(
    edges: list[Any],
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for edge in edges:
        start = edge.start_point()
        end = edge.end_point()
        segments.append(
            ((float(start.X), float(start.Y)), (float(end.X), float(end.Y)))
        )
    return segments


def _plot_drawing(
    *,
    png_path: Path,
    title: str,
    visible_edges: list[Any],
    hidden_edges: list[Any],
    width: int,
    height: int,
) -> None:
    visible_segments = _edge_segments(visible_edges)
    hidden_segments = _edge_segments(hidden_edges)
    points = [
        point for segment in visible_segments + hidden_segments for point in segment
    ]
    if not points:
        raise ValueError("drafting preview produced no projected edges")

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    x_pad = max((max(xs) - min(xs)) * 0.08, 1.0)
    y_pad = max((max(ys) - min(ys)) * 0.08, 1.0)

    fig, ax = plt.subplots(figsize=(width / 100, height / 100))
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")
    ax.set_title(title, fontsize=10, pad=12)

    for start, end in visible_segments:
        ax.plot((start[0], end[0]), (start[1], end[1]), color="#111111", linewidth=1.4)
    for start, end in hidden_segments:
        ax.plot(
            (start[0], end[0]),
            (start[1], end[1]),
            color="#707070",
            linewidth=1.0,
            linestyle="--",
        )

    ax.set_xlim(min(xs) - x_pad, max(xs) + x_pad)
    ax.set_ylim(min(ys) - y_pad, max(ys) + y_pad)
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _export_vector_sidecars(
    *,
    svg_path: Path,
    dxf_path: Path,
    visible_edges: list[Any],
    hidden_edges: list[Any],
) -> None:
    svg = ExportSVG(unit=Unit.MM)
    svg.add_layer("visible", line_type=LineType.CONTINUOUS)
    svg.add_layer("hidden", line_type=LineType.HIDDEN)
    svg.add_shape(visible_edges, layer="visible")
    svg.add_shape(hidden_edges, layer="hidden")
    svg.write(str(svg_path))

    dxf = ExportDXF(unit=Unit.MM, line_type=LineType.CONTINUOUS)
    dxf.add_layer("visible", line_type=LineType.CONTINUOUS)
    dxf.add_layer("hidden", line_type=LineType.HIDDEN)
    dxf.add_shape(visible_edges, layer="visible")
    dxf.add_shape(hidden_edges, layer="hidden")
    dxf.write(str(dxf_path))


def render_technical_drawing_preview(
    *,
    root: Path,
    script_path: str,
    session_id: str,
    agent_role: str | None = None,
    script_content: str | None = None,
) -> PreviewDesignResponse:
    assembly_definition = _load_assembly_definition(root)
    drafting = assembly_definition.drafting
    if drafting is None:
        raise ValueError("assembly_definition.yaml.drafting is required")

    component = load_component_from_script(
        script_path=(root / script_path)
        if not Path(script_path).is_absolute()
        else Path(script_path),
        session_root=root,
        script_content=script_content,
    )
    source_script_path = (
        (root / script_path)
        if not Path(script_path).is_absolute()
        else Path(script_path)
    )
    source_script_sha256 = _script_source_sha256(
        source_script_path,
        script_content=script_content,
    )

    render_policy = load_agents_config().render
    render_width = render_policy.image_resolution.width
    render_height = render_policy.image_resolution.height
    output_dir = (
        root
        / "renders"
        / (
            "benchmark_renders"
            if (agent_role or "").startswith("benchmark_")
            else "engineer_renders"
        )
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    bounding_box = component.bounding_box()
    look_at = _vector_tuple(bounding_box.center())
    focus = max(float(getattr(bounding_box, "diagonal", 0.0) or 0.0), 1.0)

    artifacts: dict[str, RenderArtifactMetadata] = {}
    png_paths: list[str] = []
    view_specs: list[PreviewViewSpec] = []
    render_blobs_base64: dict[str, str] = {}
    object_store_keys: dict[str, str] = {}
    png_upload_jobs: list[tuple[str, Path]] = []
    first_image_path: Path | None = None
    preview_client = _preview_storage_client()

    for index, view in enumerate(drafting.views):
        visible_edges, hidden_edges, view_spec = _project_edges(
            component,
            view,
            look_at=look_at,
            focus=focus,
        )
        view_specs.append(view_spec.model_copy(update={"view_index": index}, deep=True))
        view_slug = _sanitize_filename(f"{index:02d}_{view.view_id}")
        png_path = output_dir / f"{view_slug}.png"
        svg_path = output_dir / f"{view_slug}.svg"
        dxf_path = output_dir / f"{view_slug}.dxf"

        _plot_drawing(
            png_path=png_path,
            title=f"{drafting.title} - {view.view_id}",
            visible_edges=visible_edges,
            hidden_edges=hidden_edges,
            width=render_width,
            height=render_height,
        )
        _export_vector_sidecars(
            svg_path=svg_path,
            dxf_path=dxf_path,
            visible_edges=visible_edges,
            hidden_edges=hidden_edges,
        )

        png_rel = str(png_path.relative_to(root))
        svg_rel = str(svg_path.relative_to(root))
        dxf_rel = str(dxf_path.relative_to(root))
        png_paths.append(png_rel)
        artifacts[png_rel] = RenderArtifactMetadata(
            modality="unknown",
            group_key=view_slug,
            view_index=index,
            siblings=RenderSiblingPaths(rgb=png_rel, svg=svg_rel, dxf=dxf_rel),
        )
        png_upload_jobs.append((png_rel, png_path))
        render_blobs_base64[svg_rel] = base64.b64encode(svg_path.read_bytes()).decode(
            "ascii"
        )
        render_blobs_base64[dxf_rel] = base64.b64encode(dxf_path.read_bytes()).decode(
            "ascii"
        )
        if first_image_path is None:
            first_image_path = png_path

    if first_image_path is None:
        raise ValueError("drafting preview produced no output")

    if preview_client is not None and png_upload_jobs:
        with ThreadPoolExecutor(max_workers=min(8, len(png_upload_jobs))) as executor:
            futures = {
                executor.submit(preview_client.upload_file, png_path, png_rel): (
                    png_rel,
                    png_path,
                )
                for png_rel, png_path in png_upload_jobs
            }
            for future in as_completed(futures):
                png_rel, png_path = futures[future]
                try:
                    object_store_keys[png_rel] = future.result()
                except Exception:
                    logger.warning(
                        "technical_drawing_preview_object_store_upload_failed",
                        session_id=session_id,
                        agent_role=agent_role,
                        image_path=str(png_path),
                    )
                    render_blobs_base64[png_rel] = base64.b64encode(
                        png_path.read_bytes()
                    ).decode("ascii")
    else:
        for png_rel, png_path in png_upload_jobs:
            render_blobs_base64[png_rel] = base64.b64encode(
                png_path.read_bytes()
            ).decode("ascii")

    manifest = build_render_manifest(
        artifacts,
        workspace_root=root,
        episode_id=session_id,
        worker_session_id=session_id,
        preview_evidence_paths=png_paths,
        drafting=True,
        source_script_sha256=source_script_sha256,
    )
    manifest_path = output_dir / "render_manifest.json"
    manifest_path.write_text(manifest.model_dump_json(indent=2), encoding="utf-8")
    render_blobs_base64[str(manifest_path.relative_to(root))] = base64.b64encode(
        manifest_path.read_bytes()
    ).decode("ascii")

    response = PreviewDesignResponse(
        success=True,
        status_text="Drafting preview generated successfully",
        message="Drafting preview generated successfully",
        job_id=None,
        queued=False,
        view_count=len(view_specs),
        view_specs=view_specs,
        artifact_path=str(first_image_path.relative_to(root)),
        manifest_path=str(manifest_path.relative_to(root)),
        rendering_type=PreviewRenderingType.RGB,
        drafting=True,
        pitch=None,
        yaw=None,
        image_path=str(first_image_path.relative_to(root)),
        image_bytes_base64=(
            None
            if object_store_keys
            else base64.b64encode(first_image_path.read_bytes()).decode("ascii")
        ),
        render_blobs_base64=render_blobs_base64,
        object_store_keys=object_store_keys,
        render_manifest_json=manifest.model_dump_json(indent=2),
        events=[],
    )
    logger.info(
        "technical_drawing_preview_rendered",
        session_id=session_id,
        agent_role=agent_role,
        output_path=str(first_image_path),
        view_count=len(view_specs),
    )
    return response
