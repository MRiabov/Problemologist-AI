"""Benchmark plan technical-drawing script for the sideways-ball benchmark.

Produces an orthographic drawing package matching the approved planner inventory.
Expected token counts (from benchmark_parts + final_assembly references):
  - floor_plate x2, ground_plane x1, support_tower x2, raised_goal_shelf x2, lift_carriage x2
"""

from build123d import (
    Align,
    Box,
    Compound,
    Location,
    TechnicalDrawing,
)

from utils.metadata import CompoundMetadata, PartMetadata

GROUND_PLANE_SIZE = (0.3, 0.08, 0.005)
FLOOR_PLATE_SIZE = (0.2, 0.06, 0.01)
SUPPORT_TOWER_SIZE = (0.04, 0.04, 0.16)
RAISED_GOAL_SHELF_SIZE = (0.06, 0.06, 0.01)
LIFT_CARRIAGE_SIZE = (0.04, 0.04, 0.02)


def _make_box(label, size, pos, material, fixed):
    box = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    box = box.move(Location(pos))
    box.label = label
    box.metadata = PartMetadata(material_id=material, fixed=fixed)
    return box


def _build_model() -> Compound:
    """Build the model matching the approved planner inventory."""
    children: list = []

    children.append(
        _make_box(
            "ground_plane",
            GROUND_PLANE_SIZE,
            (0.0, 0.0, 0.0025),
            "hdpe",
            True,
        )
    )
    children.append(
        _make_box(
            "floor_plate",
            FLOOR_PLATE_SIZE,
            (0.0, 0.0, 0.005),
            "aluminum_6061",
            True,
        )
    )
    children.append(
        _make_box(
            "floor_plate",
            FLOOR_PLATE_SIZE,
            (0.0, 0.0, 0.015),
            "aluminum_6061",
            True,
        )
    )
    children.append(
        _make_box(
            "support_tower",
            SUPPORT_TOWER_SIZE,
            (-0.1, 0.0, 0.09),
            "aluminum_6061",
            True,
        )
    )
    children.append(
        _make_box(
            "support_tower",
            SUPPORT_TOWER_SIZE,
            (0.1, 0.0, 0.09),
            "aluminum_6061",
            True,
        )
    )
    children.append(
        _make_box(
            "raised_goal_shelf",
            RAISED_GOAL_SHELF_SIZE,
            (0.5, 0.0, 0.17),
            "aluminum_6061",
            True,
        )
    )
    children.append(
        _make_box(
            "raised_goal_shelf",
            RAISED_GOAL_SHELF_SIZE,
            (0.5, 0.0, 0.16),
            "aluminum_6061",
            True,
        )
    )
    children.append(
        _make_box(
            "lift_carriage",
            LIFT_CARRIAGE_SIZE,
            (0.0, 0.0, 0.05),
            "hdpe",
            False,
        )
    )
    children.append(
        _make_box(
            "lift_carriage",
            LIFT_CARRIAGE_SIZE,
            (0.0, 0.0, 0.07),
            "hdpe",
            False,
        )
    )

    asm = Compound(children=children)
    asm.label = "benchmark_plan_technical_drawing"
    asm.metadata = CompoundMetadata()
    return asm


def build() -> Compound:
    """Return the drafted model; TechnicalDrawing is constructed for review."""
    TechnicalDrawing(title="Sideways Ball Benchmark Plan")
    return _build_model()
