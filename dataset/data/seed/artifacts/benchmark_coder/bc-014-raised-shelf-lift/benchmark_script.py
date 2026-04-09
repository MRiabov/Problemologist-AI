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

import yaml
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


def _load_objectives() -> dict:
    """Load objective zones from benchmark_definition.yaml."""
    with open("benchmark_definition.yaml", encoding="utf-8") as fh:
        payload = yaml.safe_load(fh) or {}
    objectives = payload.get("objectives", {})
    return objectives if isinstance(objectives, dict) else {}


def _build_objective_zone(label: str, bounds_min, bounds_max):
    """Build one AABB objective zone as a Box with a stable label."""
    size_x = float(bounds_max[0]) - float(bounds_min[0])
    size_y = float(bounds_max[1]) - float(bounds_min[1])
    size_z = float(bounds_max[2]) - float(bounds_min[2])
    center = (
        (float(bounds_min[0]) + float(bounds_max[0])) / 2.0,
        (float(bounds_min[1]) + float(bounds_max[1])) / 2.0,
        (float(bounds_min[2]) + float(bounds_max[2])) / 2.0,
    )
    zone = Box(
        max(size_x, 0.0),
        max(size_y, 0.0),
        max(size_z, 0.0),
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).move(Location(center))
    zone.label = label
    zone.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return zone


def objectives_geometry() -> Compound:
    """Return the benchmark objective overlay geometry for this workspace."""
    objectives = _load_objectives()
    children = []

    goal_zone = objectives.get("goal_zone")
    if isinstance(goal_zone, dict):
        children.append(
            _build_objective_zone(
                "zone_goal",
                goal_zone.get("min", [0.0, 0.0, 0.0]),
                goal_zone.get("max", [0.0, 0.0, 0.0]),
            )
        )

    for index, forbid_zone in enumerate(objectives.get("forbid_zones", []) or []):
        if isinstance(forbid_zone, dict):
            zone_name = (
                str(forbid_zone.get("name", f"forbid_{index}")).strip()
                or f"forbid_{index}"
            )
            children.append(
                _build_objective_zone(
                    f"zone_forbid_{index}_{zone_name}",
                    forbid_zone.get("min", [0.0, 0.0, 0.0]),
                    forbid_zone.get("max", [0.0, 0.0, 0.0]),
                )
            )

    build_zone = objectives.get("build_zone")
    if isinstance(build_zone, dict):
        children.append(
            _build_objective_zone(
                "zone_build",
                build_zone.get("min", [0.0, 0.0, 0.0]),
                build_zone.get("max", [0.0, 0.0, 0.0]),
            )
        )

    overlay = Compound(children=children) if children else Compound()
    overlay.label = "benchmark_objectives"
    overlay.metadata = CompoundMetadata()
    return overlay


result = build()
