"""Motorized raised-shelf benchmark implementation.

Implements the approved planner handoff for a benchmark where a
benchmark-owned servo lifts a projectile ball from floor height onto a
raised goal shelf while keeping a shelf-support clearance zone free.

Inventory (exact match with planner handoff):
- floor_plate x2 (aluminum_6061, fixed)
- support_tower x2 (aluminum_6061, fixed)
- raised_goal_shelf x2 (aluminum_6061, fixed)
- lift_carriage x2 (hdpe, slide_z axis)
- drive_motor x1 (COTS ServoMotor_DS3218, imported via from_catalog_id)
"""

from __future__ import annotations

from build123d import Align, Box, Compound, Location

from shared.cots.parts.motors import ServoMotor
from utils.metadata import CompoundMetadata, PartMetadata


def _build_floor_plates() -> list[Box]:
    """Build the fixed base plates (2 instances)."""
    parts = []
    for pos in [(20.0, -45.0, 0.0), (20.0, 45.0, 0.0)]:
        part = Box(560.0, 180.0, 20.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        part = part.move(Location(pos))
        part.label = "floor_plate"
        part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
        parts.append(part)
    return parts


def _build_support_towers() -> list[Box]:
    """Build the fixed vertical support towers (2 instances)."""
    parts = []
    for pos in [(190.0, -40.0, 105.0), (270.0, 40.0, 105.0)]:
        part = Box(80.0, 120.0, 210.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        part = part.move(Location(pos))
        part.label = "support_tower"
        part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
        parts.append(part)
    return parts


def _build_raised_goal_shelves() -> list[Box]:
    """Build the fixed raised goal shelves (2 instances)."""
    parts = []
    for pos in [(335.0, -40.0, 235.0), (415.0, 40.0, 235.0)]:
        part = Box(150.0, 120.0, 30.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        part = part.move(Location(pos))
        part.label = "raised_goal_shelf"
        part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
        parts.append(part)
    return parts


def _build_lift_carriages() -> list[Box]:
    """Build the moving lift carriages with a single slide_z DOF (2 instances)."""
    parts = []
    for pos in [(-170.0, -35.0, 60.0), (-90.0, 35.0, 60.0)]:
        part = Box(120.0, 110.0, 18.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        part = part.move(Location(pos))
        part.label = "lift_carriage"
        part.metadata = PartMetadata(material_id="hdpe", fixed=False)
        parts.append(part)
    return parts


def _build_drive_motor() -> ServoMotor:
    """Instantiate the benchmark-owned servo motor via COTS catalog lookup."""
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218")
    # Position the motor beneath the lift carriages.
    motor = motor.move(Location((-130.0, 0.0, 10.0)))
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
    # NOTE: Do NOT include the payload here -- the simulation system spawns
    # `benchmark_payload__projectile_ball` independently from `benchmark_definition.yaml`.
    # Returning it from build() creates a duplicate body that collides with the spawned ball,
    # causing instant OUT_OF_BOUNDS.
    children = [
        *_build_floor_plates(),
        *_build_support_towers(),
        *_build_raised_goal_shelves(),
        *_build_lift_carriages(),
        _build_drive_motor(),
    ]
    environment = Compound(children=children)
    environment.label = "benchmark_environment"
    environment.metadata = CompoundMetadata()
    return environment


result = build()
