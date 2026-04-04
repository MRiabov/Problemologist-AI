"""Benchmark plan technical-drawing script for the motorized raised-shelf benchmark.

This script produces an orthographic drawing package that matches the
approved planner inventory exactly. Every label, quantity, and COTS
identity must mirror the planner handoff:

- floor_plate x1 (aluminum_6061, fixed)
- support_tower x1 (aluminum_6061, fixed)
- raised_goal_shelf x1 (aluminum_6061, fixed)
- lift_carriage x1 (hdpe, slide_z)
- drive_motor x1 (COTS ServoMotor_DS3218)
"""

from build123d import (
    Align,
    Box,
    Compound,
    Location,
    TechnicalDrawing,
)

from utils.metadata import CompoundMetadata, PartMetadata

# Geometry positions derived from benchmark_definition.yaml and
# benchmark_assembly_definition.yaml.

FLOOR_PLATE_POS = (20.0, 0.0, 10.0)
FLOOR_PLATE_SIZE = (560.0, 180.0, 20.0)

SUPPORT_TOWER_POS = (230.0, 0.0, 105.0)
SUPPORT_TOWER_SIZE = (80.0, 120.0, 210.0)

RAISED_GOAL_SHELF_POS = (375.0, 0.0, 235.0)
RAISED_GOAL_SHELF_SIZE = (150.0, 120.0, 30.0)

LIFT_CARRIAGE_POS = (-130.0, 0.0, 60.0)
LIFT_CARRIAGE_SIZE = (120.0, 110.0, 18.0)

DRIVE_MOTOR_POS = (-130.0, 0.0, 10.0)
DRIVE_MOTOR_SIZE = (40.0, 20.0, 40.5)


def _build_model() -> Compound:
    """Build the model matching the approved planner inventory."""
    children: list = []

    # floor_plate x1
    fp = Box(*FLOOR_PLATE_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fp = fp.move(Location(FLOOR_PLATE_POS))
    fp.label = "floor_plate"
    fp.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(fp)

    # support_tower x1
    st = Box(*SUPPORT_TOWER_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    st = st.move(Location(SUPPORT_TOWER_POS))
    st.label = "support_tower"
    st.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(st)

    # raised_goal_shelf x1
    gs = Box(*RAISED_GOAL_SHELF_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    gs = gs.move(Location(RAISED_GOAL_SHELF_POS))
    gs.label = "raised_goal_shelf"
    gs.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(gs)

    # lift_carriage x1
    lc = Box(*LIFT_CARRIAGE_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    lc = lc.move(Location(LIFT_CARRIAGE_POS))
    lc.label = "lift_carriage"
    lc.metadata = PartMetadata(material_id="hdpe", fixed=False)
    children.append(lc)

    # drive_motor x1 (COTS ServoMotor_DS3218 proxy)
    dm = Box(*DRIVE_MOTOR_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    dm = dm.move(Location(DRIVE_MOTOR_POS))
    dm.label = "drive_motor"
    dm.metadata = PartMetadata(cots_id="ServoMotor_DS3218", fixed=True)
    children.append(dm)

    asm = Compound(children=children)
    asm.label = "benchmark_plan_technical_drawing"
    asm.metadata = CompoundMetadata()
    return asm


def build() -> TechnicalDrawing:
    """Return a TechnicalDrawing package for the approved planner inventory."""
    model = _build_model()
    drawing = TechnicalDrawing(
        model,
        sheet_size=(420, 297),  # A3
        title="Motorized Raised Shelf Benchmark Plan",
        units="mm",
    )
    return drawing
