"""Motorized raised-shelf benchmark implementation.

Implements the approved planner handoff for a benchmark where a
benchmark-owned servo lifts a projectile ball from floor height onto a
raised goal shelf while keeping a shelf-support clearance zone free.

Inventory (exact match with planner handoff):
- floor_plate x1 (aluminum_6061, fixed)
- support_tower x1 (aluminum_6061, fixed)
- raised_goal_shelf x1 (aluminum_6061, fixed)
- lift_carriage x1 (hdpe, slide_z axis)
- drive_motor x1 (COTS ServoMotor_DS3218, imported via from_catalog_id)
"""

from __future__ import annotations

from build123d import Align, Box, Compound, Location

from shared.cots.parts.motors import ServoMotor
from utils.metadata import CompoundMetadata, PartMetadata


def _build_floor_plate() -> Box:
    """Build the fixed base plate."""
    part = Box(560.0, 180.0, 20.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.move(Location((20.0, 0.0, 0.0)))
    part.label = "floor_plate"
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part


def _build_support_tower() -> Box:
    """Build the fixed vertical support tower."""
    part = Box(80.0, 120.0, 210.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.move(Location((230.0, 0.0, 20.0)))
    part.label = "support_tower"
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part


def _build_raised_goal_shelf() -> Box:
    """Build the fixed raised goal shelf."""
    part = Box(150.0, 120.0, 30.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    part = part.move(Location((375.0, 0.0, 235.0)))
    part.label = "raised_goal_shelf"
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part


def _build_lift_carriage() -> Box:
    """Build the moving lift carriage with a single slide_z DOF."""
    part = Box(120.0, 110.0, 18.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    part = part.move(Location((-130.0, 0.0, 60.0)))
    part.label = "lift_carriage"
    part.metadata = PartMetadata(material_id="hdpe", fixed=False)
    return part


def _build_drive_motor() -> ServoMotor:
    """Instantiate the benchmark-owned servo motor via COTS catalog lookup."""
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218")
    # Position the motor beside the lift carriage path (y=-70).
    motor = motor.move(Location((-130.0, -70.0, 20.0)))
    # Ensure the COTS label is stable and reviewer-visible.
    if not motor.label:
        motor.label = "drive_motor"
    return motor


def build() -> Compound:
    """Return the benchmark assembly geometry for this workspace.

    Every top-level child must have a unique label that is not
    ``environment``, does not start with ``zone_``, and does not start with
    ``benchmark_payload__`` because the simulator reserves those names.
    """
    # NOTE: Do NOT include the payload here — the simulation system spawns
    # `benchmark_payload__projectile_ball` independently from
    # `benchmark_definition.yaml`. Returning it from build() creates a duplicate
    # body that collides with the spawned ball, causing instant OUT_OF_BOUNDS.
    children = [
        _build_floor_plate(),
        _build_support_tower(),
        _build_raised_goal_shelf(),
        _build_lift_carriage(),
        _build_drive_motor(),
    ]
    environment = Compound(children=children)
    environment.label = "benchmark_environment"
    environment.metadata = CompoundMetadata()
    return environment


result = build()
