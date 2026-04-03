from __future__ import annotations

from build123d import Align, Box, Compound, Cylinder, Location, Sphere
import yaml

from utils.metadata import CompoundMetadata, PartMetadata


def _load_yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        payload = yaml.safe_load(fh) or {}
    return payload if isinstance(payload, dict) else {}


def _make_box(label: str, size: tuple[float, float, float], center: tuple[float, float, float]):
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location(center)
    )
    part.label = label
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part


def _build_static_fixtures() -> Compound:
    children = [
        _make_box("left_start_deck", (28.0, 18.0, 4.0), (-30.0, 0.0, 2.0)),
        _make_box("right_goal_deck", (28.0, 18.0, 4.0), (30.0, 0.0, 2.0)),
        _make_box("bridge_reference_table", (8.0, 8.0, 18.0), (0.0, 0.0, 9.0)),
        _make_box("gap_floor_guard", (18.0, 4.0, 6.0), (0.0, -14.0, 3.0)),
    ]
    fixtures = Compound(children=children)
    fixtures.label = "benchmark_fixtures"
    fixtures.metadata = CompoundMetadata()
    return fixtures


def _build_moved_object(moved: dict):
    label = str(moved.get("label", "")).strip()
    if not label:
        raise ValueError("moved_object.label must be a non-empty string")
    start = moved.get("start_position", [0.0, 0.0, 0.0])
    radius_range = moved.get("static_randomization", {}).get("radius", [0.01, 0.01])
    radius = float(max(radius_range)) if radius_range else 0.01
    shape = str(moved.get("shape", "sphere")).strip().lower()
    material_id = str(moved.get("material_id", "abs")).strip()
    if not material_id:
        raise ValueError("moved_object.material_id must be a non-empty string")

    if shape == "sphere":
        moved_part = Sphere(
            radius, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        )
    elif shape in {"cube", "box"}:
        edge = radius * 2.0
        moved_part = Box(
            edge, edge, edge, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        )
    elif shape == "cylinder":
        moved_part = Cylinder(
            radius=radius,
            height=radius * 2.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
    else:
        raise ValueError(
            f"Unsupported moved_object.shape '{shape}'. Expected sphere, cube, box, or cylinder."
        )

    moved_part = moved_part.move(
        Location((float(start[0]), float(start[1]), float(start[2])))
    )
    moved_part.label = label
    moved_part.metadata = PartMetadata(material_id=material_id, fixed=False)
    return moved_part


def build() -> Compound:
    """Return the benchmark assembly geometry for this workspace."""

    payload = _load_yaml("benchmark_definition.yaml")
    moved_object = payload.get("moved_object", {})
    moved_part = _build_moved_object(moved_object if isinstance(moved_object, dict) else {})
    environment = Compound(children=[_build_static_fixtures(), moved_part])
    environment.label = "benchmark_environment"
    environment.metadata = CompoundMetadata()
    return environment


def _load_objectives() -> dict:
    payload = _load_yaml("benchmark_definition.yaml")
    objectives = payload.get("objectives", {})
    return objectives if isinstance(objectives, dict) else {}


def _build_objective_zone(label: str, bounds_min, bounds_max):
    size_x = float(bounds_max[0]) - float(bounds_min[0])
    size_y = float(bounds_max[1]) - float(bounds_min[1])
    size_z = float(bounds_max[2]) - float(bounds_min[2])
    center = (
        (float(bounds_min[0]) + float(bounds_max[0])) / 2.0,
        (float(bounds_min[1]) + float(bounds_max[1])) / 2.0,
        (float(bounds_min[2]) + float(bounds_max[2])) / 2.0,
    )
    zone = Box(
        max(size_x, 0.0),
        max(size_y, 0.0),
        max(size_z, 0.0),
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).move(Location(center))
    zone.label = label
    zone.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return zone


def objectives_geometry() -> Compound:
    """Return the benchmark objective overlay geometry for this workspace."""

    objectives = _load_objectives()
    children = []

    goal_zone = objectives.get("goal_zone")
    if isinstance(goal_zone, dict):
        children.append(
            _build_objective_zone(
                "zone_goal",
                goal_zone.get("min", [0.0, 0.0, 0.0]),
                goal_zone.get("max", [0.0, 0.0, 0.0]),
            )
        )

    for index, forbid_zone in enumerate(objectives.get("forbid_zones", []) or []):
        if isinstance(forbid_zone, dict):
            zone_name = str(forbid_zone.get("name", f"forbid_{index}")).strip() or f"forbid_{index}"
            children.append(
                _build_objective_zone(
                    f"zone_forbid_{index}_{zone_name}",
                    forbid_zone.get("min", [0.0, 0.0, 0.0]),
                    forbid_zone.get("max", [0.0, 0.0, 0.0]),
                )
            )

    build_zone = objectives.get("build_zone")
    if isinstance(build_zone, dict):
        children.append(
            _build_objective_zone(
                "zone_build",
                build_zone.get("min", [0.0, 0.0, 0.0]),
                build_zone.get("max", [0.0, 0.0, 0.0]),
            )
        )

    overlay = Compound(children=children) if children else Compound()
    overlay.label = "benchmark_objectives"
    overlay.metadata = CompoundMetadata()
    return overlay


result = build()
