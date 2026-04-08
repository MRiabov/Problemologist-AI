from __future__ import annotations

import yaml
from build123d import Align, Box, Compound, Cylinder, Location, Sphere

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
    radius_range = moved.get("static_randomization", {}).get("radius", [15.0, 20.0])
    radius = float(max(radius_range)) if radius_range else 17.5
    shape = str(moved.get("shape", "sphere")).strip().lower()
    material_id = str(moved.get("material_id", "abs")).strip()
    if not material_id:
        raise ValueError("moved_object.material_id must be a non-empty string")

    if shape == "sphere":
        moved_part = Sphere(radius, align=(Align.CENTER, Align.CENTER, Align.CENTER))
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
    moved_part.metadata = PartMetadata(
        material_id=material_id,
    )
    return moved_part


def _build_benchmark_fixtures() -> list:
    """Build benchmark-owned fixture geometry from assembly definition."""
    with open("benchmark_assembly_definition.yaml", encoding="utf-8") as fh:
        asm = yaml.safe_load(fh) or {}

    parts_cfg = asm.get("manufactured_parts", [])
    fixtures = []

    for p in parts_cfg:
        part_name = str(p.get("part_name", ""))
        material_id = str(p.get("material_id", ""))
        bbox = p.get("stock_bbox_mm", {})
        bx = float(bbox.get("x", 100.0))
        by = float(bbox.get("y", 100.0))
        bz = float(bbox.get("z", 20.0))

        if part_name == "floor_slab":
            slab = Box(bx, by, bz, align=(Align.CENTER, Align.CENTER, Align.MIN))
            slab.label = "floor_slab"
            slab.metadata = PartMetadata(material_id=material_id, fixed=True)
            fixtures.append(slab)
        elif part_name == "forbid_wall_left":
            wall = Box(bx, by, bz, align=(Align.CENTER, Align.CENTER, Align.MIN))
            wall = wall.move(Location((-225.0, 0.0, 0.0)))
            wall.label = "forbid_wall_left"
            wall.metadata = PartMetadata(material_id=material_id, fixed=True)
            fixtures.append(wall)
        elif part_name == "forbid_wall_right":
            wall = Box(bx, by, bz, align=(Align.CENTER, Align.CENTER, Align.MIN))
            wall = wall.move(Location((225.0, 0.0, 0.0)))
            wall.label = "forbid_wall_right"
            wall.metadata = PartMetadata(material_id=material_id, fixed=True)
            fixtures.append(wall)

    return fixtures


def build() -> Compound:
    moved = _load_moved_object()
    moved_obj = _build_moved_object(moved)
    fixtures = _build_benchmark_fixtures()

    all_parts = [moved_obj] + fixtures
    compound = Compound(all_parts)
    compound.label = "environment"
    compound.metadata = CompoundMetadata()
    return compound


if __name__ == "__main__":
    result = build()
    print(f"Built benchmark compound with {len(result.children())} children")
