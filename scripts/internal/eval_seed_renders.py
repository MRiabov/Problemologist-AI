from __future__ import annotations

import math
import shutil
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh
import yaml
from build123d import Align, Box, Compound, Cylinder, Location, Sphere
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from shared.agents import get_image_render_resolution
from shared.git_utils import repo_revision
from shared.models.schemas import BenchmarkDefinition, CompoundMetadata, PartMetadata
from shared.rendering import export_preview_scene_bundle
from shared.rendering.renderer_client import (
    materialize_render_artifacts,
    render_static_preview,
)
from shared.script_contracts import authored_script_path_for_reviewer_stage
from shared.workers.loader import load_component_from_script
from shared.workers.schema import (
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
    SegmentationLegendEntry,
)
from worker_renderer.utils.build123d_rendering import (
    _preview_camera_distance,
    _unique_color,
    _zone_color,
    camera_position_from_orbit,
)

_NO_RENDER_ROLE = {
    "benchmark_plan_reviewer",
    "benchmark_coder",
    "engineer_planner",
    "engineer_plan_reviewer",
}

_RENDER_ROLE_WITH_SCRIPT = {
    "benchmark_reviewer",
    "engineer_execution_reviewer",
    "electronics_reviewer",
}

_RENDER_ROLE_WITH_DEFINITION_PREVIEW = {
    "engineer_coder",
}

_VIEW_ORBITS = (
    (-15.0, 0.0),
    (-15.0, 45.0),
    (-15.0, 90.0),
    (-15.0, 135.0),
    (-15.0, 180.0),
    (-15.0, 225.0),
    (-15.0, 270.0),
    (-15.0, 315.0),
    (-45.0, 0.0),
    (-45.0, 45.0),
    (-45.0, 90.0),
    (-45.0, 135.0),
    (-45.0, 180.0),
    (-45.0, 225.0),
    (-45.0, 270.0),
    (-45.0, 315.0),
    (-75.0, 0.0),
    (-75.0, 45.0),
    (-75.0, 90.0),
    (-75.0, 135.0),
    (-75.0, 180.0),
    (-75.0, 225.0),
    (-75.0, 270.0),
    (-75.0, 315.0),
)

_IMAGE_SIZE = (
    get_image_render_resolution().width,
    get_image_render_resolution().height,
)
_VIEW_ANGLE_DEG = 30.0
_EDGE_COLOR = (24, 24, 24, 255)
_BACKGROUND_RGB = (248, 248, 250, 255)
_DEPTH_BACKGROUND_RGB = (0, 0, 0, 255)
_SEGMENTATION_BACKGROUND_RGB = (0, 0, 0, 255)


@dataclass(frozen=True)
class _WorldTriangle:
    vertices: np.ndarray
    entity_label: str
    semantic_label: str
    instance_id: str
    instance_name: str
    object_type: str
    object_id: int
    body_name: str | None
    geom_name: str | None
    color_rgba: tuple[float, float, float, float] | None
    segmentation_color_rgb: tuple[int, int, int] | None
    include_in_segmentation: bool
    zone_type: str | None


@dataclass(frozen=True)
class _ProjectedTriangle:
    points: tuple[tuple[float, float], tuple[float, float], tuple[float, float]]
    depth: float
    vertex_depths: tuple[float, float, float]
    triangle: _WorldTriangle


def _load_benchmark_definition(artifact_dir: Path) -> BenchmarkDefinition:
    definition_path = artifact_dir / "benchmark_definition.yaml"
    if not definition_path.exists():
        raise FileNotFoundError(f"benchmark_definition.yaml missing in {artifact_dir}")
    data = yaml.safe_load(definition_path.read_text(encoding="utf-8")) or {}
    return BenchmarkDefinition.model_validate(data)


def _load_benchmark_assembly_definition(artifact_dir: Path) -> dict[str, object]:
    assembly_path = artifact_dir / "benchmark_assembly_definition.yaml"
    if not assembly_path.exists():
        return {}
    return yaml.safe_load(assembly_path.read_text(encoding="utf-8")) or {}


def _build_moved_object_component(definition: BenchmarkDefinition) -> Compound:
    moved = definition.moved_object
    radius_range = moved.static_randomization.radius
    radius = float(max(radius_range)) if radius_range else 10.0
    shape = moved.shape.strip().lower()

    if shape == "sphere":
        part = Sphere(radius, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    elif shape in {"box", "cube"}:
        edge = radius * 2.0
        part = Box(edge, edge, edge, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    elif shape == "cylinder":
        part = Cylinder(
            radius=radius,
            height=radius * 2.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
    else:
        raise ValueError(
            f"Unsupported moved_object.shape '{moved.shape}'. "
            "Expected sphere, box, cube, or cylinder."
        )

    part = part.move(Location(tuple(moved.start_position)))
    part.label = moved.label
    part.metadata = PartMetadata(material_id=moved.material_id, fixed=False)

    preview = Compound(label=f"{moved.label}_preview", children=[part])
    preview.metadata = CompoundMetadata(fixed=False)
    return preview


def _load_preview_component(
    artifact_dir: Path, definition: BenchmarkDefinition, role_name: str
) -> Compound:
    if role_name in _RENDER_ROLE_WITH_SCRIPT:
        script_path = artifact_dir / authored_script_path_for_reviewer_stage(role_name)
        if not script_path.exists():
            raise FileNotFoundError(
                f"{script_path.name} missing in {artifact_dir} for {role_name}"
            )
        return load_component_from_script(script_path, session_root=artifact_dir)
    return _build_moved_object_component(definition)


def _role_name_for_artifact(artifact_dir: Path) -> str:
    return artifact_dir.parent.name


def _latest_revision() -> str:
    revision = repo_revision(ROOT)
    if not revision:
        raise RuntimeError("Unable to determine repository revision for seed renders")
    return revision


def _materialize_mesh(mesh: trimesh.Trimesh | trimesh.Scene) -> trimesh.Trimesh:
    if isinstance(mesh, trimesh.Trimesh):
        return mesh.copy()
    if isinstance(mesh, trimesh.Scene):
        geometries = [geom.copy() for geom in mesh.geometry.values()]
        if not geometries:
            raise ValueError("scene contained no geometry")
        return trimesh.util.concatenate(geometries)
    raise TypeError(f"Unsupported mesh type: {type(mesh)!r}")


def _transform_matrix(entity) -> np.ndarray:
    matrix = trimesh.transformations.euler_matrix(
        math.radians(float(entity.euler[0])),
        math.radians(float(entity.euler[1])),
        math.radians(float(entity.euler[2])),
        axes="sxyz",
    )
    matrix[:3, 3] = np.asarray(entity.pos, dtype=np.float64)
    return matrix


def _triangles_for_entity(entity) -> list[_WorldTriangle]:
    if entity.box_size is not None:
        mesh = trimesh.creation.box(
            extents=np.asarray(entity.box_size, dtype=np.float64) * 2.0
        )
        mesh.apply_transform(_transform_matrix(entity))
        color_rgba = entity.color_rgba
        segmentation_color = entity.segmentation_color_rgb
        return [
            _WorldTriangle(
                vertices=np.asarray(mesh.vertices[face], dtype=np.float64),
                entity_label=entity.label,
                semantic_label=entity.semantic_label,
                instance_id=entity.instance_id,
                instance_name=entity.instance_name,
                object_type=entity.object_type,
                object_id=entity.object_id,
                body_name=entity.body_name,
                geom_name=entity.geom_name,
                color_rgba=color_rgba,
                segmentation_color_rgb=segmentation_color,
                include_in_segmentation=entity.include_in_segmentation,
                zone_type=entity.zone_type,
            )
            for face in mesh.faces
        ]

    triangles: list[_WorldTriangle] = []
    if not entity.mesh_paths:
        return triangles

    transform = _transform_matrix(entity)
    for mesh_path in entity.mesh_paths:
        loaded = _materialize_mesh(trimesh.load(mesh_path, process=False))
        loaded.apply_transform(transform)
        for face in loaded.faces:
            triangles.append(
                _WorldTriangle(
                    vertices=np.asarray(loaded.vertices[face], dtype=np.float64),
                    entity_label=entity.label,
                    semantic_label=entity.semantic_label,
                    instance_id=entity.instance_id,
                    instance_name=entity.instance_name,
                    object_type=entity.object_type,
                    object_id=entity.object_id,
                    body_name=entity.body_name,
                    geom_name=entity.geom_name,
                    color_rgba=entity.color_rgba,
                    segmentation_color_rgb=entity.segmentation_color_rgb,
                    include_in_segmentation=entity.include_in_segmentation,
                    zone_type=entity.zone_type,
                )
            )
    return triangles


def _build_world_triangles(scene) -> list[_WorldTriangle]:
    triangles: list[_WorldTriangle] = []
    for entity in scene.entities:
        triangles.extend(_triangles_for_entity(entity))
    return triangles


def _camera_basis(
    camera_position: tuple[float, float, float],
    lookat: tuple[float, float, float],
    up: tuple[float, float, float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    cam = np.asarray(camera_position, dtype=np.float64)
    target = np.asarray(lookat, dtype=np.float64)
    up_vec = np.asarray(up, dtype=np.float64)

    forward = target - cam
    forward_norm = np.linalg.norm(forward)
    if forward_norm < 1e-9:
        forward = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        forward = forward / forward_norm

    up_norm = np.linalg.norm(up_vec)
    if up_norm < 1e-9:
        up_vec = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        up_vec = up_vec / up_norm

    right = np.cross(forward, up_vec)
    right_norm = np.linalg.norm(right)
    if right_norm < 1e-9:
        right = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        right = right / right_norm

    true_up = np.cross(right, forward)
    true_up_norm = np.linalg.norm(true_up)
    if true_up_norm < 1e-9:
        true_up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    else:
        true_up = true_up / true_up_norm

    return right, true_up, forward


def _project_triangle(
    triangle: _WorldTriangle,
    *,
    camera_position: tuple[float, float, float],
    basis: tuple[np.ndarray, np.ndarray, np.ndarray],
    focal_length: float,
    width: int,
    height: int,
) -> _ProjectedTriangle | None:
    cam = np.asarray(camera_position, dtype=np.float64)
    right, up_vec, forward = basis
    projected: list[tuple[float, float]] = []
    depths: list[float] = []

    for vertex in triangle.vertices:
        relative = vertex - cam
        x = float(np.dot(relative, right))
        y = float(np.dot(relative, up_vec))
        z = float(np.dot(relative, forward))
        if z <= 1e-6:
            return None
        screen_x = (width / 2.0) + (focal_length * (x / z))
        screen_y = (height / 2.0) - (focal_length * (y / z))
        projected.append((screen_x, screen_y))
        depths.append(z)

    depth = float(sum(depths) / len(depths))
    return _ProjectedTriangle(
        points=tuple(projected),
        depth=depth,
        vertex_depths=(depths[0], depths[1], depths[2]),
        triangle=triangle,
    )


def _normalize_depth(depth: float, depth_min: float, depth_max: float) -> float:
    if depth_max <= depth_min + 1e-9:
        return 0.0
    return max(0.0, min(1.0, (depth - depth_min) / (depth_max - depth_min)))


def _draw_axes_overlay(image: Image.Image, *, color: tuple[int, int, int, int]) -> None:
    draw = ImageDraw.Draw(image, "RGBA")
    w, h = image.size
    origin = (28, h - 28)
    draw.line([origin, (origin[0] + 30, origin[1])], fill=(230, 60, 60, 200), width=3)
    draw.line([origin, (origin[0], origin[1] - 30)], fill=(60, 190, 90, 200), width=3)
    draw.line(
        [origin, (origin[0] - 22, origin[1] - 18)],
        fill=(75, 120, 230, 200),
        width=3,
    )
    draw.ellipse(
        [
            (origin[0] - 2, origin[1] - 2),
            (origin[0] + 2, origin[1] + 2),
        ],
        fill=color,
    )


def _triangle_edge(
    ax: float, ay: float, bx: float, by: float, px: float, py: float
) -> float:
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax)


def _rasterize_projected_depth(
    projected_triangles: list[_ProjectedTriangle], *, width: int, height: int
) -> np.ndarray:
    depth_buffer = np.full((height, width), np.inf, dtype=np.float32)
    if not projected_triangles:
        return depth_buffer

    for projected in sorted(
        projected_triangles, key=lambda item: item.depth, reverse=True
    ):
        points = projected.points
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        min_x = max(0, int(math.floor(min(xs))))
        max_x = min(width - 1, int(math.ceil(max(xs))))
        min_y = max(0, int(math.floor(min(ys))))
        max_y = min(height - 1, int(math.ceil(max(ys))))
        if min_x > max_x or min_y > max_y:
            continue

        (x0, y0), (x1, y1), (x2, y2) = points
        area = _triangle_edge(x0, y0, x1, y1, x2, y2)
        if abs(area) < 1e-9:
            continue

        z0, z1, z2 = projected.vertex_depths
        inv_z0 = 1.0 / max(z0, 1e-9)
        inv_z1 = 1.0 / max(z1, 1e-9)
        inv_z2 = 1.0 / max(z2, 1e-9)
        sample_offset = 0.5

        for py in range(min_y, max_y + 1):
            sample_y = py + sample_offset
            for px in range(min_x, max_x + 1):
                sample_x = px + sample_offset
                w0 = _triangle_edge(x1, y1, x2, y2, sample_x, sample_y) / area
                w1 = _triangle_edge(x2, y2, x0, y0, sample_x, sample_y) / area
                w2 = 1.0 - w0 - w1
                if w0 < -1e-6 or w1 < -1e-6 or w2 < -1e-6:
                    continue

                denom = (w0 * inv_z0) + (w1 * inv_z1) + (w2 * inv_z2)
                if denom <= 1e-9:
                    continue
                depth = float(1.0 / denom)
                if depth < depth_buffer[py, px]:
                    depth_buffer[py, px] = depth

    return depth_buffer


def _render_projected_triangles(
    projected_triangles: list[_ProjectedTriangle],
    *,
    modality: str,
    width: int,
    height: int,
) -> Image.Image:
    if modality == "rgb":
        image = Image.new("RGBA", (width, height), _BACKGROUND_RGB)
    else:
        image = Image.new("RGBA", (width, height), _SEGMENTATION_BACKGROUND_RGB)

    if not projected_triangles:
        if modality == "rgb":
            _draw_axes_overlay(image, color=(35, 35, 35, 255))
        return image

    draw = ImageDraw.Draw(image, "RGBA")
    if modality == "depth":
        depth_buffer = _rasterize_projected_depth(
            projected_triangles, width=width, height=height
        )
        finite_mask = np.isfinite(depth_buffer)
        if not finite_mask.any():
            return Image.new("RGBA", (width, height), _DEPTH_BACKGROUND_RGB)

        depth_min = float(depth_buffer[finite_mask].min())
        depth_max = float(depth_buffer[finite_mask].max())
        if math.isclose(depth_min, depth_max):
            normalized = np.ones_like(depth_buffer, dtype=np.float32)
        else:
            normalized = (depth_buffer - depth_min) / (depth_max - depth_min)
        normalized = np.clip(normalized, 0.0, 1.0)
        brightness = np.zeros_like(depth_buffer, dtype=np.uint8)
        brightness[finite_mask] = ((1.0 - normalized[finite_mask]) * 255.0).astype(
            np.uint8
        )
        rgb = np.repeat(brightness[:, :, np.newaxis], 3, axis=2)
        alpha = np.where(finite_mask[:, :, np.newaxis], 255, 0).astype(np.uint8)
        return Image.fromarray(np.dstack([rgb, alpha]), mode="RGBA")

    depths = [tri.depth for tri in projected_triangles]
    depth_min = min(depths)
    depth_max = max(depths)

    for projected in sorted(
        projected_triangles, key=lambda item: item.depth, reverse=True
    ):
        triangle = projected.triangle
        points = projected.points

        if modality == "rgb":
            if triangle.object_type == "zone" and triangle.zone_type:
                fill = _zone_color(triangle.zone_type)
                rgba = (
                    int(round(fill[0] * 255.0)),
                    int(round(fill[1] * 255.0)),
                    int(round(fill[2] * 255.0)),
                    int(round(fill[3] * 255.0)),
                )
            else:
                fill_src = triangle.color_rgba or (0.62, 0.67, 0.73, 1.0)
                rgba = (
                    int(round(fill_src[0] * 255.0)),
                    int(round(fill_src[1] * 255.0)),
                    int(round(fill_src[2] * 255.0)),
                    int(round(fill_src[3] * 255.0)),
                )
            draw.polygon(points, fill=rgba, outline=_EDGE_COLOR)
            continue

        if triangle.include_in_segmentation:
            color = triangle.segmentation_color_rgb or _unique_color(triangle.object_id)
            draw.polygon(points, fill=(*color, 255))

    if modality == "rgb":
        _draw_axes_overlay(image, color=(35, 35, 35, 255))
    return image


def _render_context_views(scene, output_dir: Path) -> list[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    world_triangles = _build_world_triangles(scene)
    rendered_paths: list[Path] = []

    camera_distance = _preview_camera_distance(
        scene, width=_IMAGE_SIZE[0], height=_IMAGE_SIZE[1]
    )
    focal_length = (_IMAGE_SIZE[1] / 2.0) / math.tan(
        math.radians(_VIEW_ANGLE_DEG) / 2.0
    )

    for index, (pitch, yaw) in enumerate(_VIEW_ORBITS, start=1):
        camera_position = camera_position_from_orbit(
            scene.center, camera_distance, pitch, yaw
        )
        basis = _camera_basis(camera_position, scene.center, (0.0, 0.0, 1.0))
        projected_triangles = []
        for triangle in world_triangles:
            projected = _project_triangle(
                triangle,
                camera_position=camera_position,
                basis=basis,
                focal_length=focal_length,
                width=_IMAGE_SIZE[0],
                height=_IMAGE_SIZE[1],
            )
            if projected is not None:
                projected_triangles.append(projected)

        stem = f"render_e{int(abs(pitch))}_a{int(yaw)}"
        rgb_path = output_dir / f"{stem}.png"
        depth_path = output_dir / f"{stem}_depth.png"
        segmentation_path = output_dir / f"{stem}_segmentation.png"

        _render_projected_triangles(
            projected_triangles,
            modality="rgb",
            width=_IMAGE_SIZE[0],
            height=_IMAGE_SIZE[1],
        ).save(rgb_path, format="PNG")
        _render_projected_triangles(
            projected_triangles,
            modality="depth",
            width=_IMAGE_SIZE[0],
            height=_IMAGE_SIZE[1],
        ).save(depth_path, format="PNG")
        _render_projected_triangles(
            projected_triangles,
            modality="segmentation",
            width=_IMAGE_SIZE[0],
            height=_IMAGE_SIZE[1],
        ).save(segmentation_path, format="PNG")

        rendered_paths.extend([rgb_path, depth_path, segmentation_path])

    return rendered_paths


def _manifest_for_render_paths(
    *,
    artifact_dir: Path,
    render_paths: list[Path],
    scene,
    existing_manifest: RenderManifest | None,
) -> RenderManifest:
    revision = _latest_revision()

    environment_version = None
    if existing_manifest is not None and existing_manifest.environment_version:
        environment_version = existing_manifest.environment_version
    elif (artifact_dir / "benchmark_assembly_definition.yaml").exists():
        try:
            assembly = _load_benchmark_assembly_definition(artifact_dir)
            environment_version = str(assembly.get("version") or "").strip() or None
        except Exception:
            environment_version = None
    if environment_version is None and existing_manifest is not None:
        environment_version = existing_manifest.environment_version

    render_rel_paths = [
        str(path.relative_to(artifact_dir)).replace("\\", "/") for path in render_paths
    ]
    artifacts: dict[str, RenderArtifactMetadata] = {}
    legend_entries: list[SegmentationLegendEntry] = []

    for entity in scene.entities:
        if not entity.include_in_segmentation:
            continue
        color = entity.segmentation_color_rgb or _unique_color(entity.object_id)
        legend_entries.append(
            SegmentationLegendEntry(
                instance_id=entity.instance_id,
                instance_name=entity.instance_name,
                semantic_label=entity.semantic_label,
                object_type=entity.object_type,
                object_id=entity.object_id,
                body_name=entity.body_name,
                geom_name=entity.geom_name,
                color_rgb=color,
                color_hex="#" + "".join(f"{channel:02x}" for channel in color),
            )
        )

    for rel_path in render_rel_paths:
        path = Path(rel_path)
        stem = path.stem
        if rel_path.endswith("_depth.png"):
            group_key = stem.removesuffix("_depth")
            modality = "depth"
        elif rel_path.endswith("_segmentation.png"):
            group_key = stem.removesuffix("_segmentation")
            modality = "segmentation"
        else:
            group_key = stem
            modality = "rgb"

        siblings = RenderSiblingPaths(
            rgb=str(path.parent / f"{group_key}.png"),
            depth=str(path.parent / f"{group_key}_depth.png"),
            segmentation=str(path.parent / f"{group_key}_segmentation.png"),
        )
        artifact = RenderArtifactMetadata(
            modality=modality,
            group_key=group_key,
            siblings=siblings,
        )
        if modality == "depth":
            artifact.depth_interpretation = (
                "Brighter pixels are nearer. Values are normalized per image "
                "from the software preview renderer."
            )
        elif modality == "segmentation":
            artifact.segmentation_legend = list(legend_entries)
        artifacts[rel_path] = artifact

    return RenderManifest(
        episode_id=artifact_dir.name,
        worker_session_id=artifact_dir.name,
        revision=revision,
        environment_version=environment_version,
        preview_evidence_paths=render_rel_paths,
        artifacts=artifacts,
    )


def update_seed_artifact_renders(artifact_dir: Path) -> list[str]:
    artifact_dir = Path(artifact_dir)
    role_name = _role_name_for_artifact(artifact_dir)
    renders_dir = artifact_dir / "renders"
    images_dir = renders_dir / "images"

    if role_name in _NO_RENDER_ROLE:
        if renders_dir.exists():
            shutil.rmtree(renders_dir)
        return []

    if role_name not in _RENDER_ROLE_WITH_SCRIPT | _RENDER_ROLE_WITH_DEFINITION_PREVIEW:
        return []

    definition = _load_benchmark_definition(artifact_dir)
    component = _load_preview_component(artifact_dir, definition, role_name)

    if images_dir.exists():
        shutil.rmtree(images_dir)
    bundle_base64 = export_preview_scene_bundle(
        component,
        objectives=definition,
        workspace_root=artifact_dir,
        smoke_test_mode=False,
    )
    response = render_static_preview(
        bundle_base64=bundle_base64,
        script_path="preview_scene.json",
        session_id=f"seed-renders-{artifact_dir.name}-{uuid.uuid4().hex[:8]}",
        smoke_test_mode=False,
    )
    if not response.success or response.artifacts is None:
        raise RuntimeError(response.message or "seed render failed")

    rendered_paths = materialize_render_artifacts(
        response.artifacts, artifact_dir, default_subdir="renders"
    )

    return rendered_paths
