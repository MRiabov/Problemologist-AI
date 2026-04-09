"""Benchmark plan technical-drawing script for the sideways-ball benchmark.

Produces an orthographic drawing package matching the approved planner inventory.
Matches manufactured_parts exactly:
  - ground_plane x1 (hdpe, fixed)
  - floor_plate x1 (aluminum_6061, fixed)
  - support_tower x1 (aluminum_6061, fixed)
  - raised_goal_shelf x1 (aluminum_6061, fixed)
  - lift_carriage x1 (hdpe, slide_z)
"""

from build123d import (
    Align,
    Box,
    Compound,
    Location,
    TechnicalDrawing,
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


def _build_model() -> Compound:
    """Build the preview model matching the approved inventory."""
    children: list = []

    gp = Box(*GROUND_PLANE_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    gp = gp.move(Location(GROUND_PLANE_POS))
    gp.label = "ground_plane"
    gp.metadata = PartMetadata(material_id="hdpe", fixed=True)
    children.append(gp)

    fp = Box(*FLOOR_PLATE_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    fp = fp.move(Location(FLOOR_PLATE_POS))
    fp.label = "floor_plate"
    fp.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(fp)

    st = Box(*SUPPORT_TOWER_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    st = st.move(Location(SUPPORT_TOWER_POS))
    st.label = "support_tower"
    st.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(st)

    gs = Box(*RAISED_GOAL_SHELF_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    gs = gs.move(Location(RAISED_GOAL_SHELF_POS))
    gs.label = "raised_goal_shelf"
    gs.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(gs)

    lc = Box(*LIFT_CARRIAGE_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    lc = lc.move(Location(LIFT_CARRIAGE_POS))
    lc.label = "lift_carriage"
    lc.metadata = PartMetadata(material_id="hdpe", fixed=False)
    children.append(lc)

    asm = Compound(children=children)
    asm.label = "benchmark_plan_technical_drawing"
    asm.metadata = CompoundMetadata()
    return asm


def build() -> Compound:
    """Return the drafted model; TechnicalDrawing is constructed for review."""
    TechnicalDrawing(title="Sideways Ball Benchmark Plan")
    return _build_model()
