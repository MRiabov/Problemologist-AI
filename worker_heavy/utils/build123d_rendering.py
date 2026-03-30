from __future__ import annotations

import colorsys
import math
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import structlog
import vtk
from build123d import Compound
from PIL import Image
from pydantic import BaseModel, ConfigDict, Field
from vtk.util.numpy_support import vtk_to_numpy
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkFiltersCore import vtkFeatureEdges
from vtkmodules.vtkFiltersSources import vtkCubeSource
from vtkmodules.vtkIOGeometry import vtkOBJReader
from vtkmodules.vtkRenderingAnnotation import vtkCubeAxesActor2D
from vtkmodules.vtkRenderingCore import (
    vtkPolyDataMapper,
    vtkRenderer,
    vtkRenderWindow,
    vtkWindowToImageFilter,
)

from shared.models.schemas import BenchmarkDefinition
from shared.models.simulation import RendererCapabilities, RenderMode
from shared.simulation.backends import RendererBackend
from shared.workers.schema import SegmentationLegendEntry
from worker_heavy.simulation.builder import CommonAssemblyTraverser, MeshProcessor
from worker_heavy.utils.vtk_display import ensure_headless_vtk_display
from worker_heavy.workbenches.config import load_config, load_merged_config

logger = structlog.get_logger(__name__)

PREVIEW_BACKEND_NAME = "build123d_vtk"


class PreviewEntity(BaseModel):
    """Renderable body or zone used by the build123d preview renderer."""

    model_config = ConfigDict(extra="forbid")

    label: str
    semantic_label: str
    instance_id: str
    instance_name: str
    object_type: str
    object_id: int
    pos: tuple[float, float, float]
    euler: tuple[float, float, float]
    mesh_paths: list[str] = Field(default_factory=list)
    box_size: tuple[float, float, float] | None = None
    material_id: str | None = None
    body_name: str | None = None
    geom_name: str | None = None
    zone_type: str | None = None
    color_rgba: tuple[float, float, float, float] | None = None
    segmentation_color_rgb: tuple[int, int, int] | None = None
    include_in_segmentation: bool = True


class PreviewScene(BaseModel):
    """Strict render-scene bundle used for RGB/depth/segmentation preview."""

    model_config = ConfigDict(extra="forbid")

    entities: list[PreviewEntity] = Field(default_factory=list)
    bounds_min: tuple[float, float, float]
    bounds_max: tuple[float, float, float]
    center: tuple[float, float, float]
    diagonal: float


@dataclass
class _RendererBundle:
    renderer: vtkRenderer
    window: vtkRenderWindow


@dataclass
class _CameraState:
    pos: tuple[float, float, float] | None = None
    lookat: tuple[float, float, float] | None = None
    up: tuple[float, float, float] | None = None
    fov: float | None = None


def _rgba_from_hex(color: str, alpha: float = 1.0) -> tuple[float, float, float, float]:
    color = color.lstrip("#")
    if len(color) != 6:
        return (0.62, 0.67, 0.73, alpha)
    r = int(color[0:2], 16) / 255.0
    g = int(color[2:4], 16) / 255.0
    b = int(color[4:6], 16) / 255.0
    return (r, g, b, alpha)


def _material_color_rgba(
    material_id: str | None, manufacturing_config
) -> tuple[float, float, float, float]:
    if not material_id:
        return (0.62, 0.67, 0.73, 1.0)

    material = manufacturing_config.materials.get(material_id)
    if material is None and manufacturing_config.cnc:
        material = manufacturing_config.cnc.materials.get(material_id)
    if material is None and manufacturing_config.injection_molding:
        material = manufacturing_config.injection_molding.materials.get(material_id)
    if material is None and manufacturing_config.three_dp:
        material = manufacturing_config.three_dp.materials.get(material_id)

    if material is None or not getattr(material, "color", None):
        return (0.62, 0.67, 0.73, 1.0)
    return _rgba_from_hex(material.color, 1.0)


def _zone_color(zone_type: str) -> tuple[float, float, float, float]:
    if zone_type == "goal":
        return (0.20, 0.72, 0.34, 0.35)
    if zone_type == "build":
        return (0.55, 0.55, 0.55, 0.22)
    return (0.83, 0.20, 0.20, 0.32)


def _unique_color(index: int) -> tuple[int, int, int]:
    hue = (index * 0.61803398875) % 1.0
    saturation = 0.75
    value = 0.92
    red, green, blue = colorsys.hsv_to_rgb(hue, saturation, value)
    return (
        int(round(red * 255.0)),
        int(round(green * 255.0)),
        int(round(blue * 255.0)),
    )


def _hex_from_rgb(color_rgb: tuple[int, int, int]) -> str:
    return "#" + "".join(f"{channel:02x}" for channel in color_rgb)


_RGB_EDGE_COLOR = (0.08, 0.08, 0.08)
_OVERLAY_EDGE_COLOR = (0.95, 0.68, 0.18)
_RGB_AXES_COLOR = (0.12, 0.12, 0.12)
_OVERLAY_AXES_COLOR = (0.94, 0.94, 0.94)


def _clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


def _scene_axis_tick_count(scene: PreviewScene) -> int:
    spans = [scene.bounds_max[i] - scene.bounds_min[i] for i in range(3)]
    longest_span = max(max(spans), 1e-6)
    # Aim for roughly 20 mm spacing, then clamp to the requested 7-13 range.
    label_count = int(round(longest_span / 20.0)) + 1
    return _clamp(label_count, 7, 13)


def _scene_axis_label_format(scene: PreviewScene, label_count: int) -> str:
    spans = [scene.bounds_max[i] - scene.bounds_min[i] for i in range(3)]
    longest_span = max(max(spans), 1e-6)
    spacing = longest_span / max(label_count - 1, 1)
    if spacing >= 10.0:
        return "%.0f"
    if spacing >= 1.0:
        return "%.1f"
    if spacing >= 0.1:
        return "%.2f"
    return "%.3f"


def _depth_buffer_to_uint8(depth_buffer: np.ndarray) -> np.ndarray:
    finite_mask = np.isfinite(depth_buffer)
    depth_render = np.zeros(depth_buffer.shape, dtype=np.uint8)
    if not finite_mask.any():
        return depth_render

    depth_min = float(depth_buffer[finite_mask].min())
    depth_max = float(depth_buffer[finite_mask].max())
    if np.isclose(depth_min, depth_max):
        normalized = np.ones(depth_buffer.shape, dtype=np.float32)
    else:
        normalized = (depth_buffer - depth_min) / (depth_max - depth_min)
        normalized = 1.0 - np.clip(normalized, 0.0, 1.0)
    depth_render[finite_mask] = (normalized[finite_mask] * 255.0).astype(np.uint8)
    return depth_render


def _composite_non_black(
    base_image: np.ndarray, overlay_image: np.ndarray
) -> np.ndarray:
    if base_image.ndim == 2:
        base_rgb = np.repeat(base_image[:, :, np.newaxis], 3, axis=2)
    else:
        base_rgb = np.array(base_image, copy=True)

    if overlay_image.ndim == 2:
        overlay_rgb = np.repeat(overlay_image[:, :, np.newaxis], 3, axis=2)
    else:
        overlay_rgb = overlay_image

    mask = np.any(overlay_rgb > 0, axis=2)
    if mask.any():
        base_rgb[mask] = overlay_rgb[mask]
    return base_rgb


def _build_feature_edges_mapper(
    source: vtkCubeSource | vtkOBJReader,
) -> vtkPolyDataMapper:
    feature_edges = vtkFeatureEdges()
    feature_edges.SetInputConnection(source.GetOutputPort())
    feature_edges.BoundaryEdgesOn()
    feature_edges.FeatureEdgesOn()
    feature_edges.ManifoldEdgesOff()
    feature_edges.NonManifoldEdgesOff()
    feature_edges.SetFeatureAngle(45.0)
    feature_edges.ColoringOff()
    feature_edges.Update()

    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(feature_edges.GetOutputPort())
    mapper.ScalarVisibilityOff()
    return mapper


def _combined_bounds(
    component: Compound, objectives: BenchmarkDefinition | None
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    bbox = component.bounding_box()
    min_x, min_y, min_z = bbox.min.X, bbox.min.Y, bbox.min.Z
    max_x, max_y, max_z = bbox.max.X, bbox.max.Y, bbox.max.Z

    if objectives is not None:
        for box in [
            objectives.objectives.goal_zone,
            *objectives.objectives.forbid_zones,
            objectives.objectives.build_zone,
        ]:
            min_x = min(min_x, box.min[0])
            min_y = min(min_y, box.min[1])
            min_z = min(min_z, box.min[2])
            max_x = max(max_x, box.max[0])
            max_y = max(max_y, box.max[1])
            max_z = max(max_z, box.max[2])

    return (min_x, min_y, min_z), (max_x, max_y, max_z)


def _build_box_actor(
    size: tuple[float, float, float],
) -> tuple[vtkCubeSource, vtkPolyDataMapper]:
    source = vtkCubeSource()
    source.SetXLength(max(size[0] * 2.0, 1e-6))
    source.SetYLength(max(size[1] * 2.0, 1e-6))
    source.SetZLength(max(size[2] * 2.0, 1e-6))
    source.Update()
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    return source, mapper


def _build_mesh_actor(mesh_path: Path) -> tuple[vtkOBJReader, vtkPolyDataMapper]:
    reader = vtkOBJReader()
    reader.SetFileName(str(mesh_path))
    reader.Update()
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    return reader, mapper


def _make_transform(
    pos: tuple[float, float, float], euler: tuple[float, float, float]
) -> vtkTransform:
    transform = vtkTransform()
    transform.PostMultiply()
    transform.Translate(*pos)
    transform.RotateX(float(euler[0]))
    transform.RotateY(float(euler[1]))
    transform.RotateZ(float(euler[2]))
    return transform


def _style_preview_edge_actor(
    actor: vtk.vtkActor, color: tuple[float, float, float]
) -> None:
    prop = actor.GetProperty()
    prop.SetColor(*color)
    prop.SetOpacity(0.68)
    prop.SetLineWidth(1.2)
    prop.LightingOff()


def _build_preview_edge_actor(
    source: vtkCubeSource | vtkOBJReader,
    *,
    transform: vtkTransform,
    color: tuple[float, float, float] = _RGB_EDGE_COLOR,
) -> vtk.vtkActor:
    mapper = _build_feature_edges_mapper(source)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetUserTransform(transform)
    _style_preview_edge_actor(actor, color)
    return actor


def _build_axes_actor(
    scene: PreviewScene,
    renderer: vtkRenderer,
    *,
    color: tuple[float, float, float] = _RGB_AXES_COLOR,
) -> vtkCubeAxesActor2D:
    tick_count = _scene_axis_tick_count(scene)
    axes_actor = vtkCubeAxesActor2D()
    axes_actor.SetCamera(renderer.GetActiveCamera())
    axes_actor.SetBounds(
        scene.bounds_min[0],
        scene.bounds_max[0],
        scene.bounds_min[1],
        scene.bounds_max[1],
        scene.bounds_min[2],
        scene.bounds_max[2],
    )
    axes_actor.SetFlyModeToOuterEdges()
    axes_actor.SetNumberOfLabels(tick_count)
    axes_actor.SetLabelFormat(_scene_axis_label_format(scene, tick_count))
    axes_actor.SetXLabel("X")
    axes_actor.SetYLabel("Y")
    axes_actor.SetZLabel("Z")
    axes_actor.GetAxisLabelTextProperty().SetColor(*color)
    axes_actor.GetAxisLabelTextProperty().SetFontSize(12)
    axes_actor.GetAxisTitleTextProperty().SetColor(*color)
    axes_actor.GetAxisTitleTextProperty().SetFontSize(13)
    axes_actor.GetAxisLabelTextProperty().ShadowOff()
    axes_actor.GetAxisTitleTextProperty().ShadowOff()
    return axes_actor


def _load_manufacturing_config(workspace_root: Path):
    custom_config_path = workspace_root / "manufacturing_config.yaml"
    if custom_config_path.exists():
        return load_merged_config(custom_config_path)
    return load_config()


def _objective_zone_labels(objectives: BenchmarkDefinition | None) -> set[str]:
    if objectives is None:
        return set()
    labels = {"zone_goal", "zone_build"}
    labels.update(
        f"zone_forbid_{index}_{forbid.name}"
        for index, forbid in enumerate(objectives.objectives.forbid_zones)
    )
    return labels


def collect_preview_scene(
    component: Compound,
    *,
    objectives: BenchmarkDefinition | None,
    workspace_root: Path,
    smoke_test_mode: bool = False,
    mesh_root: Path | None = None,
) -> PreviewScene:
    """Materialize a renderable preview scene from a build123d assembly."""

    manufacturing_config = _load_manufacturing_config(workspace_root)
    traversed_parts = CommonAssemblyTraverser.traverse(component)
    objective_labels = _objective_zone_labels(objectives)
    render_entities: list[PreviewEntity] = []

    temp_dir = None
    if mesh_root is None:
        temp_dir = tempfile.TemporaryDirectory()
        temp_root = Path(temp_dir.name)
    else:
        temp_root = mesh_root
        temp_root.mkdir(parents=True, exist_ok=True)

    try:
        mesh_processor = MeshProcessor()
        for part_index, part_data in enumerate(traversed_parts):
            if part_data.is_zone:
                if part_data.label in objective_labels:
                    continue
                zone_type = part_data.zone_type or "forbid"
                zone_color = _zone_color(zone_type)
                render_entities.append(
                    PreviewEntity(
                        label=part_data.label,
                        semantic_label=part_data.label,
                        instance_id=part_data.label,
                        instance_name=part_data.label,
                        object_type="zone",
                        object_id=part_index,
                        pos=tuple(float(v) for v in part_data.pos),
                        euler=tuple(float(v) for v in part_data.euler),
                        box_size=tuple(
                            float(v)
                            for v in (part_data.zone_size or [0.05, 0.05, 0.05])
                        ),
                        zone_type=zone_type,
                        color_rgba=zone_color,
                        segmentation_color_rgb=_unique_color(part_index),
                        include_in_segmentation=False,
                    )
                )
                continue

            mesh_base = temp_root / part_data.label
            tolerance = 1.0 if smoke_test_mode else 0.1
            saved_paths = mesh_processor.process_geometry(
                part_data.part,
                mesh_base,
                decompose=False,
                tolerance=tolerance,
                angular_tolerance=tolerance,
            )
            obj_paths = sorted(
                str(path) for path in saved_paths if path.suffix == ".obj"
            )

            render_entities.append(
                PreviewEntity(
                    label=part_data.label,
                    semantic_label=part_data.label,
                    instance_id=part_data.label,
                    instance_name=part_data.label,
                    object_type="part",
                    object_id=part_index,
                    pos=tuple(float(v) for v in part_data.pos),
                    euler=tuple(float(v) for v in part_data.euler),
                    mesh_paths=obj_paths,
                    material_id=part_data.material_id
                    or ("cots-generic" if part_data.cots_id else None),
                    body_name=part_data.label,
                    geom_name=Path(obj_paths[0]).stem if obj_paths else None,
                    color_rgba=_material_color_rgba(
                        part_data.material_id
                        or ("cots-generic" if part_data.cots_id else None),
                        manufacturing_config,
                    ),
                    segmentation_color_rgb=_unique_color(part_index),
                )
            )

        if objectives is not None:
            objective_entities = [
                (
                    "zone_goal",
                    "goal",
                    objectives.objectives.goal_zone.min,
                    objectives.objectives.goal_zone.max,
                ),
                *(
                    (
                        f"zone_forbid_{index}_{zone.name}",
                        "forbid",
                        zone.min,
                        zone.max,
                    )
                    for index, zone in enumerate(objectives.objectives.forbid_zones)
                ),
                (
                    "zone_build",
                    "build",
                    objectives.objectives.build_zone.min,
                    objectives.objectives.build_zone.max,
                ),
            ]

            base_index = len(render_entities)
            for offset, (label, zone_type, zone_min, zone_max) in enumerate(
                objective_entities
            ):
                if label in {entity.label for entity in render_entities}:
                    continue
                pos = (
                    (zone_min[0] + zone_max[0]) / 2.0,
                    (zone_min[1] + zone_max[1]) / 2.0,
                    (zone_min[2] + zone_max[2]) / 2.0,
                )
                size = (
                    (zone_max[0] - zone_min[0]) / 2.0,
                    (zone_max[1] - zone_min[1]) / 2.0,
                    (zone_max[2] - zone_min[2]) / 2.0,
                )
                color_rgba = _zone_color(zone_type)
                render_entities.append(
                    PreviewEntity(
                        label=label,
                        semantic_label=label,
                        instance_id=label,
                        instance_name=label,
                        object_type="zone",
                        object_id=base_index + offset,
                        pos=pos,
                        euler=(0.0, 0.0, 0.0),
                        box_size=size,
                        zone_type=zone_type,
                        color_rgba=color_rgba,
                        segmentation_color_rgb=_unique_color(base_index + offset),
                        include_in_segmentation=False,
                    )
                )

        bounds_min, bounds_max = _combined_bounds(component, objectives)
        center = tuple((bounds_min[i] + bounds_max[i]) / 2.0 for i in range(3))
        diagonal = math.sqrt(
            sum((bounds_max[i] - bounds_min[i]) ** 2 for i in range(3))
        )
        return PreviewScene(
            entities=render_entities,
            bounds_min=bounds_min,
            bounds_max=bounds_max,
            center=center,
            diagonal=max(diagonal, 1e-6),
        )
    finally:
        if temp_dir is not None:
            temp_dir.cleanup()


def _add_entity_actors(
    renderer: vtkRenderer,
    entity: PreviewEntity,
    *,
    segmentation: bool,
    include_fill: bool,
    include_edges: bool,
    edge_color: tuple[float, float, float],
) -> None:
    if entity.kind if hasattr(entity, "kind") else False:  # pragma: no cover
        raise RuntimeError("legacy entity format is unsupported")

    if segmentation and not entity.include_in_segmentation:
        return

    if entity.box_size is not None:
        source, mapper = _build_box_actor(entity.box_size)
        actor = None
        if include_fill:
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.SetUserTransform(_make_transform(entity.pos, entity.euler))
    else:
        if not entity.mesh_paths:
            return
        for mesh_path in entity.mesh_paths:
            source, mapper = _build_mesh_actor(Path(mesh_path))
            actor = None
            if include_fill:
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.SetUserTransform(_make_transform(entity.pos, entity.euler))
                renderer.AddActor(actor)
                _apply_actor_style(actor, entity, segmentation=segmentation)
            if include_edges:
                renderer.AddActor(
                    _build_preview_edge_actor(
                        source,
                        transform=_make_transform(entity.pos, entity.euler),
                        color=edge_color,
                    )
                )
        return

    if actor is not None:
        renderer.AddActor(actor)
        _apply_actor_style(actor, entity, segmentation=segmentation)
    if include_edges:
        renderer.AddActor(
            _build_preview_edge_actor(
                source,
                transform=_make_transform(entity.pos, entity.euler),
                color=edge_color,
            )
        )


def _apply_actor_style(
    actor: vtk.vtkActor, entity: PreviewEntity, *, segmentation: bool
) -> None:
    prop = actor.GetProperty()
    if segmentation:
        color = entity.segmentation_color_rgb or (255, 255, 255)
        prop.SetColor(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0)
        prop.SetOpacity(1.0)
        prop.SetInterpolationToFlat()
        prop.LightingOff()
        prop.SetAmbient(1.0)
        prop.SetDiffuse(0.0)
        prop.SetSpecular(0.0)
        return

    rgba = entity.color_rgba or (0.62, 0.67, 0.73, 1.0)
    prop.SetColor(rgba[0], rgba[1], rgba[2])
    prop.SetOpacity(rgba[3])
    prop.SetInterpolationToFlat()
    prop.LightingOff()
    prop.SetAmbient(1.0)
    prop.SetDiffuse(0.0)
    prop.SetSpecular(0.0)


def _build_renderer(
    scene: PreviewScene,
    *,
    width: int,
    height: int,
    segmentation: bool,
    include_axes: bool,
    include_edges: bool,
    include_fill: bool,
    axes_color: tuple[float, float, float],
    edge_color: tuple[float, float, float],
    background: tuple[float, float, float],
) -> _RendererBundle:
    ensure_headless_vtk_display()
    renderer = vtkRenderer()
    renderer.SetBackground(*background)

    for entity in scene.entities:
        _add_entity_actors(
            renderer,
            entity,
            segmentation=segmentation,
            include_fill=include_fill,
            include_edges=include_edges,
            edge_color=edge_color,
        )

    if include_axes:
        axes_actor = _build_axes_actor(scene, renderer, color=axes_color)
        renderer.AddActor2D(axes_actor)

    render_window = vtkRenderWindow()
    render_window.AddRenderer(renderer)
    render_window.SetOffScreenRendering(True)
    render_window.SetSize(width, height)
    render_window.SetMultiSamples(0)
    return _RendererBundle(renderer=renderer, window=render_window)


def _configure_camera(
    renderer: vtkRenderer,
    *,
    camera_position: tuple[float, float, float],
    lookat: tuple[float, float, float],
    up: tuple[float, float, float],
) -> None:
    camera = renderer.GetActiveCamera()
    camera.SetPosition(*camera_position)
    camera.SetFocalPoint(*lookat)
    camera.SetViewUp(*up)
    camera.SetParallelProjection(False)


def _vtk_image_to_numpy_rgb(
    window: vtkRenderWindow, *, depth_buffer: bool = False
) -> np.ndarray:
    capture = vtkWindowToImageFilter()
    capture.SetInput(window)
    if depth_buffer:
        capture.SetInputBufferTypeToZBuffer()
        capture.ReadFrontBufferOff()
    capture.Update()
    image = capture.GetOutput()
    width, height, _ = image.GetDimensions()
    scalars = vtk_to_numpy(image.GetPointData().GetScalars())
    if depth_buffer:
        array = scalars.reshape(height, width).astype(np.float32)
        return np.flipud(array)
    channels = image.GetNumberOfScalarComponents()
    array = scalars.reshape(height, width, channels).astype(np.uint8)
    return np.flipud(array)


def _linearize_depth_buffer(
    depth_buffer: np.ndarray, near: float, far: float
) -> np.ndarray:
    depth = np.full_like(depth_buffer, np.inf, dtype=np.float32)
    valid = np.isfinite(depth_buffer) & (depth_buffer < 0.999999)
    if not valid.any():
        return depth

    z = depth_buffer[valid].astype(np.float32)
    denom = far + near - (2.0 * z - 1.0) * (far - near)
    with np.errstate(divide="ignore", invalid="ignore"):
        depth_values = (2.0 * near * far) / denom
    depth[valid] = depth_values.astype(np.float32)
    return depth


def _render_view(
    bundle: _RendererBundle,
    *,
    camera_position: tuple[float, float, float],
    lookat: tuple[float, float, float],
    up: tuple[float, float, float],
    capture_depth: bool = False,
) -> tuple[np.ndarray, np.ndarray | None]:
    _configure_camera(
        bundle.renderer,
        camera_position=camera_position,
        lookat=lookat,
        up=up,
    )
    bundle.renderer.ResetCameraClippingRange()
    bundle.window.Render()

    rgb_image = _vtk_image_to_numpy_rgb(bundle.window)
    depth_image = None
    if capture_depth:
        depth_buffer = _vtk_image_to_numpy_rgb(bundle.window, depth_buffer=True)
        near, far = bundle.renderer.GetActiveCamera().GetClippingRange()
        depth_image = _linearize_depth_buffer(depth_buffer, near, far)
    return rgb_image, depth_image


def camera_position_from_orbit(
    center: tuple[float, float, float],
    distance: float,
    elevation_deg: float,
    azimuth_deg: float,
) -> tuple[float, float, float]:
    rad_azim = math.radians(azimuth_deg)
    rad_elev = math.radians(elevation_deg)
    x = center[0] + distance * math.cos(rad_elev) * math.sin(rad_azim)
    y = center[1] - distance * math.cos(rad_elev) * math.cos(rad_azim)
    z = center[2] - distance * math.sin(rad_elev)
    return (x, y, z)


class Build123dRendererBackend(RendererBackend):
    """RendererBackend implementation backed by build123d geometry and VTK."""

    def __init__(
        self,
        *,
        workspace_root: Path,
        objectives: BenchmarkDefinition | None = None,
        smoke_test_mode: bool = False,
        include_axes: bool | None = None,
        include_edges: bool | None = None,
        rgb_axes: bool | None = None,
        rgb_edges: bool | None = None,
        depth_axes: bool | None = None,
        depth_edges: bool | None = None,
        segmentation_axes: bool | None = None,
        segmentation_edges: bool | None = None,
    ) -> None:
        self.workspace_root = workspace_root
        self.objectives = objectives
        self.smoke_test_mode = smoke_test_mode
        default_axes = True if include_axes is None else include_axes
        default_edges = True if include_edges is None else include_edges
        self.rgb_axes = default_axes if rgb_axes is None else rgb_axes
        self.rgb_edges = default_edges if rgb_edges is None else rgb_edges
        self.depth_axes = self.rgb_axes if depth_axes is None else depth_axes
        self.depth_edges = self.rgb_edges if depth_edges is None else depth_edges
        self.segmentation_axes = (
            self.rgb_axes if segmentation_axes is None else segmentation_axes
        )
        self.segmentation_edges = (
            self.rgb_edges if segmentation_edges is None else segmentation_edges
        )
        self.scene: PreviewScene | None = None
        self._camera_states: dict[str, _CameraState] = {}
        self._mesh_temp_dir: tempfile.TemporaryDirectory | None = None

    def load_scene(self, scene: Compound, render_only: bool = False) -> None:
        del render_only
        self._cleanup_mesh_temp_dir()
        self._mesh_temp_dir = tempfile.TemporaryDirectory()
        self.scene = collect_preview_scene(
            scene,
            objectives=self.objectives,
            workspace_root=self.workspace_root,
            smoke_test_mode=self.smoke_test_mode,
            mesh_root=Path(self._mesh_temp_dir.name),
        )

    def render(self) -> np.ndarray:
        return self.render_camera("preview", 640, 480)

    def get_render_capabilities(self) -> RendererCapabilities:
        return RendererCapabilities(
            backend_name=PREVIEW_BACKEND_NAME,
            artifact_modes_supported=[RenderMode.STATIC_PREVIEW],
            supports_default_view=True,
            supports_named_cameras=True,
            supports_rgb=True,
            supports_depth=True,
            supports_segmentation=True,
            default_view_label="preview",
        )

    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray:
        if self.scene is None:
            raise RuntimeError("Scene not loaded")

        logger.debug(
            "build123d_render_camera_start",
            camera_name=camera_name,
            width=width,
            height=height,
        )
        try:
            camera_position, lookat, up = self._resolve_camera(camera_name)
            bundle = _build_renderer(
                self.scene,
                width=width,
                height=height,
                segmentation=False,
                include_axes=self.rgb_axes,
                include_edges=self.rgb_edges,
                include_fill=True,
                axes_color=_RGB_AXES_COLOR,
                edge_color=_RGB_EDGE_COLOR,
                background=(0.98, 0.98, 0.99),
            )
            rgb_image, _ = _render_view(
                bundle,
                camera_position=camera_position,
                lookat=lookat,
                up=up,
                capture_depth=False,
            )
            logger.debug(
                "build123d_render_camera_complete",
                camera_name=camera_name,
                width=width,
                height=height,
                shape=tuple(rgb_image.shape),
            )
            return rgb_image
        except Exception as exc:
            logger.error(
                "build123d_render_camera_failed",
                camera_name=camera_name,
                width=width,
                height=height,
                error=str(exc),
            )
            raise

    def get_particle_positions(self) -> np.ndarray | None:
        return None

    def render_camera_modalities(
        self,
        camera_name: str,
        width: int,
        height: int,
        *,
        include_rgb: bool = True,
        include_depth: bool = True,
        include_segmentation: bool = True,
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        if self.scene is None:
            raise RuntimeError("Scene not loaded")

        logger.debug(
            "build123d_render_camera_modalities_start",
            camera_name=camera_name,
            width=width,
            height=height,
            include_rgb=include_rgb,
            include_depth=include_depth,
            include_segmentation=include_segmentation,
        )
        try:
            camera_position, lookat, up = self._resolve_camera(camera_name)
            rgb_bundle: _RendererBundle | None = None
            if include_rgb:
                rgb_bundle = _build_renderer(
                    self.scene,
                    width=width,
                    height=height,
                    segmentation=False,
                    include_axes=self.rgb_axes,
                    include_edges=self.rgb_edges,
                    include_fill=True,
                    axes_color=_RGB_AXES_COLOR,
                    edge_color=_RGB_EDGE_COLOR,
                    background=(0.98, 0.98, 0.99),
                )

            depth_base_bundle: _RendererBundle | None = None
            depth_overlay_bundle: _RendererBundle | None = None
            if include_depth:
                depth_base_bundle = _build_renderer(
                    self.scene,
                    width=width,
                    height=height,
                    segmentation=False,
                    include_axes=False,
                    include_edges=False,
                    include_fill=True,
                    axes_color=_OVERLAY_AXES_COLOR,
                    edge_color=_OVERLAY_EDGE_COLOR,
                    background=(0.98, 0.98, 0.99),
                )
                if self.depth_axes or self.depth_edges:
                    depth_overlay_bundle = _build_renderer(
                        self.scene,
                        width=width,
                        height=height,
                        segmentation=False,
                        include_axes=self.depth_axes,
                        include_edges=self.depth_edges,
                        include_fill=False,
                        axes_color=_OVERLAY_AXES_COLOR,
                        edge_color=_OVERLAY_EDGE_COLOR,
                        background=(0.0, 0.0, 0.0),
                    )

            seg_base_bundle: _RendererBundle | None = None
            seg_overlay_bundle: _RendererBundle | None = None
            if include_segmentation:
                seg_base_bundle = _build_renderer(
                    self.scene,
                    width=width,
                    height=height,
                    segmentation=True,
                    include_axes=False,
                    include_edges=False,
                    include_fill=True,
                    axes_color=_OVERLAY_AXES_COLOR,
                    edge_color=_OVERLAY_EDGE_COLOR,
                    background=(0.0, 0.0, 0.0),
                )
                if self.segmentation_axes or self.segmentation_edges:
                    seg_overlay_bundle = _build_renderer(
                        self.scene,
                        width=width,
                        height=height,
                        segmentation=False,
                        include_axes=self.segmentation_axes,
                        include_edges=self.segmentation_edges,
                        include_fill=False,
                        axes_color=_OVERLAY_AXES_COLOR,
                        edge_color=_OVERLAY_EDGE_COLOR,
                        background=(0.0, 0.0, 0.0),
                    )

            rgb_image: np.ndarray | None = None
            depth_image: np.ndarray | None = None
            seg_image: np.ndarray | None = None

            if rgb_bundle is not None:
                rgb_image, _ = _render_view(
                    rgb_bundle,
                    camera_position=camera_position,
                    lookat=lookat,
                    up=up,
                    capture_depth=False,
                )
            if depth_base_bundle is not None:
                _, depth_buffer = _render_view(
                    depth_base_bundle,
                    camera_position=camera_position,
                    lookat=lookat,
                    up=up,
                    capture_depth=True,
                )
                if depth_buffer is None:
                    raise RuntimeError("depth rendering was disabled for render")
                depth_render = _depth_buffer_to_uint8(depth_buffer)
                if depth_overlay_bundle is not None:
                    depth_overlay, _ = _render_view(
                        depth_overlay_bundle,
                        camera_position=camera_position,
                        lookat=lookat,
                        up=up,
                        capture_depth=False,
                    )
                    depth_image = _composite_non_black(depth_render, depth_overlay)
                else:
                    depth_image = depth_render
            if seg_base_bundle is not None:
                seg_image, _ = _render_view(
                    seg_base_bundle,
                    camera_position=camera_position,
                    lookat=lookat,
                    up=up,
                    capture_depth=False,
                )
                if seg_overlay_bundle is not None:
                    seg_overlay, _ = _render_view(
                        seg_overlay_bundle,
                        camera_position=camera_position,
                        lookat=lookat,
                        up=up,
                        capture_depth=False,
                    )
                    seg_image = _composite_non_black(seg_image, seg_overlay)
            logger.debug(
                "build123d_render_camera_modalities_complete",
                camera_name=camera_name,
                width=width,
                height=height,
                include_rgb=include_rgb,
                include_depth=include_depth,
                include_segmentation=include_segmentation,
                rgb_shape=tuple(rgb_image.shape) if rgb_image is not None else None,
                depth_shape=(
                    tuple(depth_image.shape) if depth_image is not None else None
                ),
                segmentation_shape=(
                    tuple(seg_image.shape) if seg_image is not None else None
                ),
            )
            return (
                rgb_image if include_rgb else None,
                depth_image if include_depth else None,
                seg_image if include_segmentation else None,
            )
        except Exception as exc:
            logger.error(
                "build123d_render_camera_modalities_failed",
                camera_name=camera_name,
                width=width,
                height=height,
                include_rgb=include_rgb,
                include_depth=include_depth,
                include_segmentation=include_segmentation,
                error=str(exc),
            )
            raise

    def describe_segmentation(
        self, segmentation_frame: np.ndarray
    ) -> list[SegmentationLegendEntry]:
        del segmentation_frame
        if self.scene is None:
            return []

        legend: list[SegmentationLegendEntry] = []
        for entity in self.scene.entities:
            if not entity.include_in_segmentation:
                continue
            if entity.segmentation_color_rgb is None:
                continue
            legend.append(
                SegmentationLegendEntry(
                    instance_id=entity.instance_id,
                    instance_name=entity.instance_name,
                    semantic_label=entity.semantic_label,
                    object_type=entity.object_type,
                    object_id=entity.object_id,
                    body_name=entity.body_name,
                    geom_name=entity.geom_name,
                    color_rgb=entity.segmentation_color_rgb,
                    color_hex=_hex_from_rgb(entity.segmentation_color_rgb),
                )
            )
        return legend

    def set_camera(
        self,
        camera_name: str,
        pos: tuple[float, float, float] | None = None,
        lookat: tuple[float, float, float] | None = None,
        up: tuple[float, float, float] | None = None,
        fov: float | None = None,
    ) -> None:
        self._camera_states[camera_name] = _CameraState(
            pos=pos,
            lookat=lookat,
            up=up,
            fov=fov,
        )

    def get_all_camera_names(self) -> list[str]:
        names = list(self._camera_states.keys())
        if not names:
            names.append("preview")
        return sorted(dict.fromkeys(names))

    def close(self) -> None:
        self.scene = None
        self._camera_states.clear()
        self._cleanup_mesh_temp_dir()

    def _resolve_camera(
        self, camera_name: str
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]:
        if self.scene is None:
            raise RuntimeError("Scene not loaded")

        state = self._camera_states.get(camera_name)
        if state and state.pos is not None and state.lookat is not None:
            return (
                state.pos,
                state.lookat,
                state.up or (0.0, 0.0, 1.0),
            )

        distance = max(self.scene.diagonal * 1.5, 0.5)
        default_lookat = self.scene.center
        default_pos = camera_position_from_orbit(
            default_lookat,
            distance,
            -35.0,
            45.0,
        )
        return default_pos, default_lookat, (0.0, 0.0, 1.0)

    def _cleanup_mesh_temp_dir(self) -> None:
        if self._mesh_temp_dir is None:
            return
        self._mesh_temp_dir.cleanup()
        self._mesh_temp_dir = None


def render_preview_view(
    component: Compound,
    *,
    output_path: Path,
    pitch: float,
    yaw: float,
    objectives: BenchmarkDefinition | None = None,
    smoke_test_mode: bool = False,
    workspace_root: Path,
    include_axes: bool = True,
    include_edges: bool = True,
    width: int = 640,
    height: int = 480,
) -> Path:
    """Render a single build123d/VTK preview image."""

    backend = Build123dRendererBackend(
        workspace_root=workspace_root,
        objectives=objectives,
        smoke_test_mode=smoke_test_mode,
        include_axes=include_axes,
        include_edges=include_edges,
    )
    try:
        backend.load_scene(component)
        scene = backend.scene
        if scene is None:
            raise RuntimeError("build123d preview scene failed to materialize")

        distance = max(scene.diagonal * 1.5, 0.5)
        camera_position = camera_position_from_orbit(scene.center, distance, pitch, yaw)
        backend.set_camera(
            "preview",
            pos=camera_position,
            lookat=scene.center,
            up=(0.0, 0.0, 1.0),
        )
        rgb_image = backend.render_camera("preview", width, height)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb_image).save(output_path, "PNG")
        return output_path
    finally:
        backend.close()


def render_preview_bundle(
    component: Compound,
    *,
    output_dir: Path,
    objectives: BenchmarkDefinition | None = None,
    smoke_test_mode: bool = False,
    workspace_root: Path,
    rgb_axes: bool = True,
    rgb_edges: bool = True,
    depth_axes: bool = True,
    depth_edges: bool = True,
    segmentation_axes: bool = True,
    segmentation_edges: bool = True,
    include_rgb: bool = True,
    include_depth: bool = True,
    include_segmentation: bool = True,
    width: int = 640,
    height: int = 480,
) -> tuple[list[str], dict[str, list[SegmentationLegendEntry]]]:
    """
    Render the standard preview bundle through build123d/VTK.

    Returns the saved render paths and the per-entity segmentation legend.
    """

    backend = Build123dRendererBackend(
        workspace_root=workspace_root,
        objectives=objectives,
        smoke_test_mode=smoke_test_mode,
        rgb_axes=rgb_axes,
        rgb_edges=rgb_edges,
        depth_axes=depth_axes,
        depth_edges=depth_edges,
        segmentation_axes=segmentation_axes,
        segmentation_edges=segmentation_edges,
    )
    try:
        backend.load_scene(component)
        scene = backend.scene
        if scene is None:
            raise RuntimeError("build123d preview scene failed to load")

        saved_paths: list[str] = []
        legend_entries = backend.describe_segmentation(
            np.zeros((1, 1, 3), dtype=np.uint8)
        )
        legend_by_path: dict[str, list[SegmentationLegendEntry]] = {}

        center = scene.center
        distance = max(scene.diagonal * 1.5, 0.5)
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        elevations = [-15, -45, -75]
        if smoke_test_mode:
            logger.info("smoke_test_mode_reducing_render_views")
            angles = [45]
            elevations = [-45]

        for elevation in elevations:
            for angle in angles:
                camera_position = camera_position_from_orbit(
                    center, distance, elevation, angle
                )
                group_key = f"render_e{abs(elevation)}_a{angle}"
                rgb_path = output_dir / f"{group_key}.png"
                depth_path = output_dir / f"{group_key}_depth.png"
                segmentation_path = output_dir / f"{group_key}_segmentation.png"

                backend.set_camera(
                    group_key,
                    pos=camera_position,
                    lookat=center,
                    up=(0.0, 0.0, 1.0),
                )
                rgb_image, depth_image, seg_image = backend.render_camera_modalities(
                    group_key,
                    width,
                    height,
                    include_rgb=include_rgb,
                    include_depth=include_depth,
                    include_segmentation=include_segmentation,
                )

                if rgb_image is not None:
                    Image.fromarray(rgb_image).save(rgb_path, "PNG")
                    saved_paths.append(str(rgb_path))

                if depth_image is not None:
                    if depth_image.ndim == 2:
                        Image.fromarray(depth_image, mode="L").save(depth_path, "PNG")
                    else:
                        Image.fromarray(depth_image, mode="RGB").save(depth_path, "PNG")
                    saved_paths.append(str(depth_path))

                if seg_image is not None:
                    Image.fromarray(seg_image, mode="RGB").save(
                        segmentation_path, "PNG"
                    )
                    saved_paths.append(str(segmentation_path))
                    legend_by_path[str(segmentation_path)] = legend_entries

        return saved_paths, legend_by_path
    finally:
        backend.close()


# Backward compatibility for the earlier name while the rename settles.
Build123dViewerBackend = Build123dRendererBackend
