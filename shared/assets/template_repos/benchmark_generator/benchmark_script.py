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
        raise ValueError("payload.label must be a non-empty string")
    start = moved.get("start_position", [0.0, 0.0, 0.0])
    radius_range = moved.get("static_randomization", {}).get("radius", [0.01, 0.01])
    radius = float(max(radius_range)) if radius_range else 0.01
    shape = str(moved.get("shape", "sphere")).strip().lower()
    material_id = str(moved.get("material_id", "abs")).strip()
    if not material_id:
        raise ValueError("payload.material_id must be a non-empty string")

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
            f"Unsupported payload.shape '{shape}'. Expected sphere, cube, box, or cylinder."
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
    payload = payload.get("payload", {})
    moved_part = _build_moved_object(payload if isinstance(payload, dict) else {})
    environment = Compound(children=[_build_static_fixtures(), moved_part])
    environment.label = "benchmark_environment"
    environment.metadata = CompoundMetadata()
    return environment


result = build()
