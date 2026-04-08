"""Solution plan evidence script for ec-001 smoke-test gravity ramp.

This script materializes the planner-approved gravity_ramp geometry
alongside the benchmark context so reviewers can inspect the draft
before implementation.
"""

from __future__ import annotations

import yaml
from build123d import Align, Box, Compound, Location, Part

from utils.metadata import CompoundMetadata, PartMetadata


def _load_planner_parts() -> dict:
    with open("assembly_definition.yaml", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    return data


def _build_gravity_ramp() -> Part:
    """Build the planner-approved gravity ramp."""
    # Plate: 600mm long (Y), 150mm wide (X), 8mm thick, tilted ~15 degrees
    plate = Box(150, 600, 8, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    plate.label = "gravity_ramp"
    plate.metadata = PartMetadata(material_id="aluminum_6061")

    # Side walls: 600mm long, 30mm thick, 25mm tall
    left_wall = Box(30, 600, 25, align=(Align.CENTER, Align.CENTER, Align.MIN))
    left_wall = left_wall.move(Location((-60.0, 0.0, 4.0)))
    left_wall.label = "gravity_ramp_left_wall"
    left_wall.metadata = PartMetadata(material_id="aluminum_6061")

    right_wall = Box(30, 600, 25, align=(Align.CENTER, Align.CENTER, Align.MIN))
    right_wall = right_wall.move(Location((60.0, 0.0, 4.0)))
    right_wall.label = "gravity_ramp_right_wall"
    right_wall.metadata = PartMetadata(material_id="aluminum_6061")

    # Position the ramp: high end near spawn (Y=-750, Z=1500), low end toward goal
    # Center of ramp at Y=375, tilt so high end is at spawn height
    ramp = Compound([plate, left_wall, right_wall])
    ramp = ramp.move(Location((0.0, -375.0, 1050.0)))
    # Rotate around X axis to create ~15 degree tilt
    ramp = ramp.rotate((0, 0, 0), (1, 0, 0), -15)
    ramp.label = "gravity_ramp_assembly"
    return ramp


def build() -> Compound:
    ramp = _build_gravity_ramp()
    compound = Compound([ramp])
    compound.label = "engineer_solution_preview"
    compound.metadata = CompoundMetadata()
    return compound


if __name__ == "__main__":
    result = build()
    print(f"Built solution evidence compound with {len(result.children())} children")
