from __future__ import annotations

from build123d import Align, Compound, Location, Sphere
import yaml

from utils.metadata import CompoundMetadata, PartMetadata


def _load_moved_object() -> dict:
    """Load moved_object contract from planner handoff."""
    with open("benchmark_definition.yaml", encoding="utf-8") as fh:
        payload = yaml.safe_load(fh) or {}
    moved = payload.get("moved_object", {})
    return moved if isinstance(moved, dict) else {}


def _build_moved_object(moved: dict):
    label = str(moved.get("label", "")).strip()
    if not label:
        raise ValueError("moved_object.label must be a non-empty string")
    start = moved.get("start_position", [0.0, 0.0, 0.0])
    radius_range = moved.get("static_randomization", {}).get("radius", [0.01, 0.01])
    radius = float(max(radius_range)) if radius_range else 0.01
    material_id = str(moved.get("material_id", "abs")).strip()
    if not material_id:
        raise ValueError("moved_object.material_id must be a non-empty string")

    ball = Sphere(radius, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location((float(start[0]), float(start[1]), float(start[2])))
    )
    ball.label = label
    ball.metadata = PartMetadata(material_id=material_id, fixed=False)
    return ball


_moved_object_contract = _load_moved_object()
_moved_object = _build_moved_object(_moved_object_contract)

# Add authored benchmark fixtures to this children list.
# Every top-level child you add here must have a unique label, and that label
# must not be `environment` or start with `zone_` because the simulator
# reserves those names for the scene root and generated objective bodies.
result = Compound(children=[_moved_object])
result.label = "benchmark_environment"
result.metadata = CompoundMetadata()
