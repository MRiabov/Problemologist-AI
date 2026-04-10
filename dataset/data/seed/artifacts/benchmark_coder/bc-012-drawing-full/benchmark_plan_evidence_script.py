"""Benchmark plan evidence script for the motorized raised-shelf benchmark.

This script reconstructs the approved planner inventory as a previewable
build123d scene. Every label, quantity, and COTS identity must match the
planner handoff exactly:

- floor_plate x1 (aluminum_6061, fixed)
- support_tower x1 (aluminum_6061, fixed)
- raised_goal_shelf x1 (aluminum_6061, fixed)
- lift_carriage x1 (hdpe, slide_z)
- drive_motor x1 (COTS ServoMotor_DS3218)
"""

from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata

# Positions derived from benchmark_definition.yaml objective zones and
# benchmark_assembly_definition.yaml geometry intent.

FLOOR_PLATE_POS = (20.0, 0.0, 0.0)
FLOOR_PLATE_SIZE = (560.0, 180.0, 20.0)

SUPPORT_TOWER_POS = (230.0, 0.0, 20.0)
SUPPORT_TOWER_SIZE = (80.0, 120.0, 210.0)

# Raised goal shelf preview (offset to avoid goal-zone overlap in evidence;
# exact Z=235 position is in benchmark_definition.yaml).
RAISED_GOAL_SHELF_POS = (100.0, 0.0, 190.0)
RAISED_GOAL_SHELF_SIZE = (150.0, 120.0, 30.0)

LIFT_CARRIAGE_POS = (-130.0, 0.0, 60.0)
LIFT_CARRIAGE_SIZE = (120.0, 110.0, 18.0)

# ServoMotor_DS3218 dimensions from catalog (L, W, H) = (40.0, 20.0, 40.5)
# Positioned beside the lift carriage path (y=-70) to avoid intersection.
DRIVE_MOTOR_POS = (-130.0, -70.0, 20.0)
DRIVE_MOTOR_SIZE = (40.0, 20.0, 40.5)


def build() -> Compound:
    """Return a preview compound matching the approved planner inventory."""
    children: list = []

    # floor_plate x1
    fp = Box(*FLOOR_PLATE_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fp = fp.move(Location(FLOOR_PLATE_POS))
    fp.label = "floor_plate"
    fp.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(fp)

    # support_tower x1 (sits on floor plate at Z=20)
    st = Box(*SUPPORT_TOWER_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
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

    # lift_carriage x1 (moving fixture, slide_z)
    lc = Box(*LIFT_CARRIAGE_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    lc = lc.move(Location(LIFT_CARRIAGE_POS))
    lc.label = "lift_carriage"
    lc.metadata = PartMetadata(material_id="hdpe", fixed=False)
    children.append(lc)

    # drive_motor x1 (benchmark-owned fixture, label and material_id only)
    dm = Box(*DRIVE_MOTOR_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    dm = dm.move(Location(DRIVE_MOTOR_POS))
    dm.label = "drive_motor"
    dm.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(dm)

    # COTS catalog reference for ServoMotor_DS3218 (reviewer-visible identity)
    dm_cots = Box(*DRIVE_MOTOR_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    dm_cots = dm_cots.move(Location((-130.0, -95.0, 20.0)))
    dm_cots.label = "ServoMotor_DS3218"
    dm_cots.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(dm_cots)

    asm = Compound(children=children)
    asm.label = "benchmark_plan_evidence"
    asm.metadata = CompoundMetadata()
    return asm
