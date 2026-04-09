"""Benchmark plan technical drawing script for the side-deflect ramp benchmark.

This script produces a TechnicalDrawing for the approved planner inventory.
"""

from build123d import (
    Align,
    Box,
    Compound,
    Location,
    TechnicalDrawing,
)

from utils.metadata import CompoundMetadata, PartMetadata

# Same positions as evidence script

BASE_PLATE_POS = (0.0, 0.0, 5.0)
BASE_PLATE_SIZE = (300.0, 200.0, 10.0)

DEFLECTOR_RAMP_POS = (40.0, 0.0, 60.0)
DEFLECTOR_RAMP_SIZE = (120.0, 160.0, 15.0)

SIDE_GOAL_WALL_POS = (170.0, 0.0, 50.0)
SIDE_GOAL_WALL_SIZE = (60.0, 80.0, 60.0)

CATCH_BIN_POS = (170.0, 0.0, 12.5)
CATCH_BIN_SIZE = (60.0, 70.0, 5.0)


def _build_model() -> Compound:
    """Return the model compound for the drawing."""
    children: list = []

    bp = Box(*BASE_PLATE_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bp = bp.move(Location(BASE_PLATE_POS))
    bp.label = "base_plate"
    bp.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(bp)

    dr = Box(*DEFLECTOR_RAMP_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    dr = dr.move(Location(DEFLECTOR_RAMP_POS))
    dr.label = "deflector_ramp"
    dr.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(dr)

    sgw = Box(*SIDE_GOAL_WALL_SIZE, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    sgw = sgw.move(Location(SIDE_GOAL_WALL_POS))
    sgw.label = "side_goal_wall"
    sgw.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(sgw)

    # catch_bin: capture goal zone - this fixture occupies the goal zone to capture the ball
    cb = Box(*CATCH_BIN_SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    cb = cb.move(Location((170.0, 0.0, 15.0)))
    cb.label = "catch_bin"
    cb.metadata = PartMetadata(material_id="hdpe", fixed=True)
    children.append(cb)

    asm = Compound(children=children)
    asm.label = "benchmark_plan_technical_drawing"
    asm.metadata = CompoundMetadata()
    return asm


def build() -> Compound:
    """Return the drafted model for the approved planner inventory."""
    # Create the drawing artifact (side effect)
    TechnicalDrawing(title="Side-Deflect Ramp Benchmark Plan")
    return _build_model()
