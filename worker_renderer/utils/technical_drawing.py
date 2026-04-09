from __future__ import annotations

import base64
import hashlib
import math
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any

import matplotlib.pyplot as plt
import structlog
import yaml
from build123d import ExportDXF, LineType, Part, Pos
from build123d.build_enums import Unit
from PIL import Image

from shared.agents import load_agents_config
from shared.models.schemas import AssemblyDefinition, DraftingLayout, DraftingView
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

_ORTHOGRAPHIC_TRIO = ("front", "top", "side")


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


def _load_assembly_definition(
    root: Path,
    *,
    benchmark_drafting: bool = False,
) -> AssemblyDefinition:
    assembly_candidates = (
        ("benchmark_assembly_definition.yaml", "benchmark_assembly_definition.yaml"),
        ("assembly_definition.yaml", "assembly_definition.yaml"),
    )
    if not benchmark_drafting:
        assembly_candidates = tuple(reversed(assembly_candidates))

    assembly_path = None
    required_name = "assembly_definition.yaml"
    for candidate_name, candidate_required_name in assembly_candidates:
        candidate_path = root / candidate_name
        if candidate_path.exists():
            assembly_path = candidate_path
            required_name = candidate_required_name
            break

    if assembly_path is None:
        raise FileNotFoundError(f"{required_name} is required for drafting preview")

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


def _shift_segments(
    segments: list[tuple[tuple[float, float], tuple[float, float]]],
    offset: tuple[float, float],
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    if offset == (0.0, 0.0):
        return segments
    shifted: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in segments:
        shifted.append(
            (
                (start[0] + offset[0], start[1] + offset[1]),
                (end[0] + offset[0], end[1] + offset[1]),
            )
        )
    return shifted


def _segment_bounds(
    segments: list[tuple[tuple[float, float], tuple[float, float]]],
) -> tuple[float, float, float, float]:
    points = [point for segment in segments for point in segment]
    if not points:
        raise ValueError("drafting preview produced no projected edges")
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def _resolve_layout_offset(
    drafting_layout: DraftingLayout | None,
    view_id: str,
) -> tuple[float, float]:
    if drafting_layout is None:
        return (0.0, 0.0)
    for layout_view in drafting_layout.views:
        if layout_view.view_id == view_id:
            return tuple(float(value) for value in layout_view.display_offset_mm)
    return (0.0, 0.0)


def _clone_drafting_view(
    view: DraftingView,
    *,
    view_id: str | None = None,
    projection: str | None = None,
) -> DraftingView:
    update: dict[str, Any] = {}
    if view_id is not None:
        update["view_id"] = view_id
    if projection is not None:
        update["projection"] = projection
    return view.model_copy(update=update, deep=True)


def _drafting_preview_views(
    drafting: Any,
) -> list[tuple[DraftingView, tuple[float, float], str | None]]:
    authored_views = list(drafting.views)
    if not authored_views:
        return []

    if drafting.layout is None and len(authored_views) == 1:
        source_view = authored_views[0]
        return [
            (
                _clone_drafting_view(
                    source_view,
                    view_id=view_id,
                    projection=view_id,
                ),
                (0.0, 0.0),
                None,
            )
            for view_id in _ORTHOGRAPHIC_TRIO
        ]

    return [
        (
            view,
            _resolve_layout_offset(drafting.layout, view.view_id),
            (
                f"{drafting.layout.mode}"
                + (
                    "+exploded"
                    if any(
                        layout_view.view_id == view.view_id and layout_view.exploded
                        for layout_view in drafting.layout.views
                    )
                    else ""
                )
                if drafting.layout
                else None
            ),
        )
        for view in authored_views
    ]


def _render_annotation_panel(
    *,
    ax: Any,
    view: DraftingView,
    bounds: tuple[float, float, float, float],
    title: str,
    layout_mode: str | None,
) -> None:
    min_x, max_x, min_y, max_y = bounds
    x_span = max(max_x - min_x, 1.0)
    y_span = max(max_y - min_y, 1.0)
    x_pad = max(x_span * 0.12, 1.0)
    y_pad = max(y_span * 0.12, 1.0)
    center_x = (min_x + max_x) / 2.0

    title_text = title
    if layout_mode:
        title_text = f"{title_text} [{layout_mode}]"
    if view.projection in {"section", "detail"}:
        suffix = (
            view.section_marker if view.projection == "section" else view.detail_target
        )
        if suffix:
            title_text = f"{title_text} - {suffix}"
    ax.set_title(title_text, fontsize=10, pad=12)

    header_y = max_y + y_pad * 0.95
    ax.text(
        center_x,
        header_y,
        f"View {view.view_id} ({view.projection})",
        ha="center",
        va="bottom",
        fontsize=9,
        fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.2", facecolor="#f7f7f7", edgecolor="#333333"),
    )

    if view.section_marker:
        ax.text(
            min_x,
            header_y,
            f"Section {view.section_marker}",
            ha="left",
            va="bottom",
            fontsize=8,
            bbox=dict(
                boxstyle="round,pad=0.15", facecolor="#ffffff", edgecolor="#555555"
            ),
        )
    if view.detail_target:
        ax.text(
            max_x,
            header_y,
            f"Detail {view.detail_target}",
            ha="right",
            va="bottom",
            fontsize=8,
            bbox=dict(
                boxstyle="round,pad=0.15", facecolor="#ffffff", edgecolor="#555555"
            ),
        )

    for index, datum in enumerate(view.datums):
        ax.text(
            min_x,
            max_y + y_pad * (0.58 - index * 0.26),
            f"Datum {datum}",
            ha="left",
            va="bottom",
            fontsize=8,
            bbox=dict(
                boxstyle="round,pad=0.15", facecolor="#eef6ff", edgecolor="#4a6fa5"
            ),
        )

    for index, dimension in enumerate(view.dimensions):
        dim_y = min_y - y_pad * (0.85 + index * 0.28)
        dim_label = f"{dimension.dimension_id} ({dimension.kind}) = {dimension.value:g}"
        if dimension.tolerance:
            dim_label = f"{dim_label} ± {dimension.tolerance}"
        if not dimension.binding:
            dim_label = f"{dim_label} [explanatory]"
        ax.annotate(
            dim_label,
            xy=(min_x, dim_y),
            xytext=(max_x, dim_y),
            ha="center",
            va="center",
            fontsize=8,
            bbox=dict(
                boxstyle="round,pad=0.18", facecolor="#fff6e8", edgecolor="#b26b00"
            ),
            arrowprops=dict(
                arrowstyle="<->",
                lw=0.8,
                color="#b26b00",
                shrinkA=0,
                shrinkB=0,
            ),
        )

    for index, callout in enumerate(view.callouts):
        callout_y = max_y - index * y_pad * 0.22
        ax.text(
            max_x + x_pad * 0.18,
            callout_y,
            f"{callout.callout_id}. {callout.label}",
            ha="left",
            va="center",
            fontsize=8,
            bbox=dict(
                boxstyle="round,pad=0.15", facecolor="#f4f4f4", edgecolor="#666666"
            ),
        )

    for index, note in enumerate(view.notes):
        note_y = min_y - y_pad * (1.38 + index * 0.3)
        note_face = "#fff0f0" if note.critical else "#fafafa"
        note_edge = "#b00020" if note.critical else "#777777"
        note_weight = "bold" if note.critical else "normal"
        ax.text(
            min_x,
            note_y,
            f"Note {note.note_id}: {note.text}",
            ha="left",
            va="center",
            fontsize=8,
            fontweight=note_weight,
            bbox=dict(
                boxstyle="round,pad=0.18", facecolor=note_face, edgecolor=note_edge
            ),
        )


def _plot_drawing(
    *,
    png_path: Path,
    svg_path: Path,
    title: str,
    visible_edges: list[Any],
    hidden_edges: list[Any],
    view: DraftingView,
    width: int,
    height: int,
    display_offset: tuple[float, float] = (0.0, 0.0),
    layout_mode: str | None = None,
) -> None:
    visible_segments = _edge_segments(visible_edges)
    hidden_segments = _edge_segments(hidden_edges)
    visible_segments = _shift_segments(visible_segments, display_offset)
    hidden_segments = _shift_segments(hidden_segments, display_offset)
    bounds = _segment_bounds(visible_segments + hidden_segments)
    min_x, max_x, min_y, max_y = bounds
    x_span = max(max_x - min_x, 1.0)
    y_span = max(max_y - min_y, 1.0)
    x_pad = max(x_span * 0.12, 1.0)
    y_pad = max(y_span * 0.12, 1.0)

    fig, ax = plt.subplots(figsize=(width / 100, height / 100))
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")
    _render_annotation_panel(
        ax=ax,
        view=view,
        bounds=bounds,
        title=title,
        layout_mode=layout_mode,
    )

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

    ax.set_xlim(min_x - x_pad * 1.8, max_x + x_pad * 1.8)
    ax.set_ylim(min_y - y_pad * 2.2, max_y + y_pad * 2.0)
    min_width = 640
    min_height = 480
    dpi = 150
    max_attempts = 4
    try:
        for _attempt in range(max_attempts):
            fig.savefig(png_path, dpi=dpi, bbox_inches="tight")
            fig.savefig(svg_path, bbox_inches="tight")
            with Image.open(png_path) as image:
                width, height = image.size
            if width >= min_width and height >= min_height:
                break

            scale = max(min_width / max(width, 1), min_height / max(height, 1))
            dpi = max(dpi + 1, int(math.ceil(dpi * scale)))
        else:
            raise ValueError(
                "drafting preview PNG stayed below the minimum render size after "
                f"{max_attempts} attempts"
            )
    finally:
        plt.close(fig)


def _export_dxf_sidecar(
    *,
    dxf_path: Path,
    visible_edges: list[Any],
    hidden_edges: list[Any],
    display_offset: tuple[float, float] = (0.0, 0.0),
) -> None:
    dxf = ExportDXF(unit=Unit.MM, line_type=LineType.CONTINUOUS)
    dxf.add_layer("visible", line_type=LineType.CONTINUOUS)
    dxf.add_layer("hidden", line_type=LineType.HIDDEN)
    offset_location = Pos(display_offset[0], display_offset[1], 0.0)
    dxf.add_shape([offset_location * edge for edge in visible_edges], layer="visible")
    dxf.add_shape([offset_location * edge for edge in hidden_edges], layer="hidden")
    dxf.write(str(dxf_path))


def render_technical_drawing_preview(
    *,
    root: Path,
    script_path: str,
    session_id: str,
    agent_role: str | None = None,
    script_content: str | None = None,
) -> PreviewDesignResponse:
    benchmark_drafting = (agent_role or "").startswith("benchmark_") or Path(
        script_path
    ).name.startswith("benchmark_")
    assembly_definition = _load_assembly_definition(
        root,
        benchmark_drafting=benchmark_drafting,
    )
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
    output_dir = root / "renders" / "current-episode"
    output_dir.mkdir(parents=True, exist_ok=True)

    bounding_box = component.bounding_box()
    look_at = _vector_tuple(bounding_box.center())
    focus = max(float(getattr(bounding_box, "diagonal", 0.0) or 0.0), 1.0)

    artifacts: dict[str, RenderArtifactMetadata] = {}
    png_paths: list[str] = []
    view_specs: list[PreviewViewSpec] = []
    render_blobs_base64: dict[str, str] = {}
    object_store_keys: dict[str, str] = {}
    artifact_upload_jobs: list[tuple[str, Path]] = []
    first_image_path: Path | None = None
    preview_client = _preview_storage_client()

    for index, (view, display_offset, layout_mode) in enumerate(
        _drafting_preview_views(drafting)
    ):
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
            svg_path=svg_path,
            title=f"{drafting.title} - {view.view_id}",
            visible_edges=visible_edges,
            hidden_edges=hidden_edges,
            display_offset=display_offset,
            layout_mode=layout_mode,
            view=view,
            width=render_width,
            height=render_height,
        )
        _export_dxf_sidecar(
            dxf_path=dxf_path,
            visible_edges=visible_edges,
            hidden_edges=hidden_edges,
            display_offset=display_offset,
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
        artifact_upload_jobs.extend(
            [
                (png_rel, png_path),
                (svg_rel, svg_path),
                (dxf_rel, dxf_path),
            ]
        )
        if first_image_path is None:
            first_image_path = png_path

    if first_image_path is None:
        raise ValueError("drafting preview produced no output")

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
    manifest_rel = str(manifest_path.relative_to(root))
    artifact_upload_jobs.append((manifest_rel, manifest_path))

    if preview_client is not None and artifact_upload_jobs:
        upload_inputs = [
            (artifact_path, rel_path)
            for rel_path, artifact_path in artifact_upload_jobs
        ]
        uploaded_keys, failed_uploads = preview_client.upload_files(
            upload_inputs,
            max_concurrency=min(8, len(artifact_upload_jobs)),
        )
        object_store_keys.update(uploaded_keys)

        for rel_path, artifact_path in artifact_upload_jobs:
            if rel_path not in failed_uploads:
                continue
            logger.warning(
                "technical_drawing_preview_object_store_upload_failed",
                session_id=session_id,
                agent_role=agent_role,
                image_path=str(artifact_path),
                error=str(failed_uploads[rel_path]),
            )
            render_blobs_base64[rel_path] = base64.b64encode(
                artifact_path.read_bytes()
            ).decode("ascii")
    else:
        for rel_path, artifact_path in artifact_upload_jobs:
            render_blobs_base64[rel_path] = base64.b64encode(
                artifact_path.read_bytes()
            ).decode("ascii")

    first_image_rel = str(first_image_path.relative_to(root))

    response = PreviewDesignResponse(
        success=True,
        status_text="Drafting preview generated successfully",
        message="Drafting preview generated successfully",
        job_id=None,
        queued=False,
        view_count=len(view_specs),
        view_specs=view_specs,
        artifact_path=first_image_rel,
        manifest_path=manifest_rel,
        rendering_type=PreviewRenderingType.RGB,
        drafting=True,
        pitch=None,
        yaw=None,
        image_path=first_image_rel,
        image_bytes_base64=(
            None
            if first_image_rel in object_store_keys
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
