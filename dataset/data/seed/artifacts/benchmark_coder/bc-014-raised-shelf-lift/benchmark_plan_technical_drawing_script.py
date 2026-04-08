"""Benchmark plan technical-drawing script for the motorized raised-shelf benchmark.

This script produces an orthographic drawing package that matches the
approved planner inventory exactly. Every label, quantity, and COTS
identity must mirror the planner handoff:

- floor_plate x2 (aluminum_6061, fixed)
- support_tower x2 (aluminum_6061, fixed)
- raised_goal_shelf x2 (aluminum_6061, fixed)
- lift_carriage x2 (hdpe, slide_z)
- drive_motor x1 (material_id only; COTS identity from assembly YAML)
"""

from build123d import (
    Align,
    Box,
    Compound,
    Location,
    PageSize,
    TechnicalDrawing,
)

from utils.metadata import CompoundMetadata, PartMetadata

# Geometry positions derived from benchmark_definition.yaml and
# benchmark_assembly_definition.yaml. Two instances of each manufactured part.

FLOOR_PLATE_POS = [(20.0, -45.0, 0.0), (20.0, 45.0, 0.0)]
FLOOR_PLATE_SIZE = (560.0, 180.0, 20.0)

SUPPORT_TOWER_POS = [(190.0, -40.0, 105.0), (270.0, 40.0, 105.0)]
SUPPORT_TOWER_SIZE = (80.0, 120.0, 210.0)

RAISED_GOAL_SHELF_POS = [(335.0, -40.0, 235.0), (415.0, 40.0, 235.0)]
RAISED_GOAL_SHELF_SIZE = (150.0, 120.0, 30.0)

LIFT_CARRIAGE_POS = [(-170.0, -35.0, 60.0), (-90.0, 35.0, 60.0)]
LIFT_CARRIAGE_SIZE = (120.0, 110.0, 18.0)

DRIVE_MOTOR_POS = (-130.0, 0.0, 10.0)
DRIVE_MOTOR_SIZE = (40.0, 20.0, 40.5)


def _build_model() -> Compound:
    """Build the model matching the approved planner inventory."""
    children: list = []

    # floor_plate x2
    for pos in FLOOR_PLATE_POS:
        fp = Box(*FLOOR_PLATE_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
        fp = fp.move(Location(pos))
        fp.label = "floor_plate"
        fp.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
        children.append(fp)

    # support_tower x2
    for pos in SUPPORT_TOWER_POS:
        st = Box(*SUPPORT_TOWER_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        st = st.move(Location(pos))
        st.label = "support_tower"
        st.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
        children.append(st)

    # raised_goal_shelf x2
    for pos in RAISED_GOAL_SHELF_POS:
        gs = Box(
            *RAISED_GOAL_SHELF_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        )
        gs = gs.move(Location(pos))
        gs.label = "raised_goal_shelf"
        gs.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
        children.append(gs)

    # lift_carriage x2
    for pos in LIFT_CARRIAGE_POS:
        lc = Box(*LIFT_CARRIAGE_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        lc = lc.move(Location(pos))
        lc.label = "lift_carriage"
        lc.metadata = PartMetadata(material_id="hdpe", fixed=False)
        children.append(lc)

    # drive_motor x1
    dm = Box(*DRIVE_MOTOR_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    dm = dm.move(Location(DRIVE_MOTOR_POS))
    dm.label = "drive_motor"
    dm.metadata = PartMetadata(material_id="steel_generic", fixed=True)
    children.append(dm)

    asm = Compound(children=children)
    asm.label = "benchmark_plan_technical_drawing"
    asm.metadata = CompoundMetadata()
    return asm


def build() -> TechnicalDrawing:
    """Return a TechnicalDrawing package for the approved planner inventory."""
    model = _build_model()
    drawing = TechnicalDrawing(
        page_size=PageSize.A3,
        title="Motorized Raised Shelf Benchmark Plan",
    )
    drawing.add_model(model)
    return drawing
