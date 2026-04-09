"""Benchmark plan technical-drawing script for the sideways-ball benchmark.

Produces an orthographic drawing package matching the approved planner inventory.
All 5 manufactured_parts are projected with datum lines, dimensions, and callouts:
  - ground_plane x1 (hdpe, fixed)
  - floor_plate x1 (aluminum_6061, fixed)
  - support_tower x1 (aluminum_6061, fixed)
  - raised_goal_shelf x1 (aluminum_6061, fixed)
  - lift_carriage x1 (hdpe, slide_z, passive)
"""

from build123d import (
    Align,
    Box,
    Compound,
    Location,
)

from utils.metadata import CompoundMetadata, PartMetadata

# Build zone: min [-0.58, -0.1, 0.0], max [0.58, 0.1, 0.18] (metres)

GROUND_PLANE_POS = (0.0, 0.0, 0.005)
GROUND_PLANE_SIZE = (1.16, 0.2, 0.01)

FLOOR_PLATE_POS = (0.0, 0.0, 0.005)
FLOOR_PLATE_SIZE = (0.2, 0.06, 0.01)

SUPPORT_TOWER_POS = (0.0, 0.0, 0.09)
SUPPORT_TOWER_SIZE = (0.04, 0.04, 0.16)

RAISED_GOAL_SHELF_POS = (0.5, 0.0, 0.17)
RAISED_GOAL_SHELF_SIZE = (0.06, 0.06, 0.01)

LIFT_CARRIAGE_POS = (0.0, 0.0, 0.05)
LIFT_CARRIAGE_SIZE = (0.04, 0.04, 0.02)


def _make_part(
    label: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material_id: str = "aluminum_6061",
    fixed: bool = True,
) -> Box:
    """Create a labeled box part with metadata."""
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location(center)
    )
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part


def build() -> Compound:
    """Return the drafted model for the orthographic drawing package.

    All 5 inventory items are included so the TechnicalDrawing projection
    shows every part referenced by the dimension/callout schedule.
    """
    children = [
        _make_part(
            "ground_plane",
            GROUND_PLANE_SIZE,
            GROUND_PLANE_POS,
            material_id="hdpe",
            fixed=True,
        ),
        _make_part(
            "floor_plate",
            FLOOR_PLATE_SIZE,
            FLOOR_PLATE_POS,
            material_id="aluminum_6061",
            fixed=True,
        ),
        _make_part(
            "support_tower",
            SUPPORT_TOWER_SIZE,
            SUPPORT_TOWER_POS,
            material_id="aluminum_6061",
            fixed=True,
        ),
        _make_part(
            "raised_goal_shelf",
            RAISED_GOAL_SHELF_SIZE,
            RAISED_GOAL_SHELF_POS,
            material_id="aluminum_6061",
            fixed=True,
        ),
        _make_part(
            "lift_carriage",
            LIFT_CARRIAGE_SIZE,
            LIFT_CARRIAGE_POS,
            material_id="hdpe",
            fixed=False,
        ),
    ]

    asm = Compound(children=children)
    asm.label = "benchmark_plan_technical_drawing"
    asm.metadata = CompoundMetadata()
    return asm
