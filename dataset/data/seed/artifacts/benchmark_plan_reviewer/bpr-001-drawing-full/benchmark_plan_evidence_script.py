from __future__ import annotations

from build123d import Align, Box, Compound, Location

from shared.models.schemas import CompoundMetadata, PartMetadata

# The `raised_goal_shelf` geometry is designed to occupy the goal zone so the object must rest there to score.


def _make_box(
    label: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material_id: str,
    *,
    fixed: bool = True,
):
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location(center)
    )
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part


def build() -> Compound:
    """Return the benchmark assembly geometry for this workspace."""
    children = [
        _make_box(
            "floor_plate",
            (200.0, 180.0, 20.0),
            (20.0, 0.0, 10.0),
            "aluminum_6061",
        ),
        _make_box(
            "support_tower",
            (80.0, 120.0, 210.0),
            (100.0, 0.0, 125.0),
            "aluminum_6061",
        ),
        _make_box(
            "raised_goal_shelf",
            (150.0, 120.0, 30.0),
            (375.0, 0.0, 235.0),
            "aluminum_6061",
        ),
        _make_box(
            "lift_carriage",
            (120.0, 110.0, 18.0),
            (-130.0, 0.0, 60.0),
            "hdpe",
            fixed=False,
        ),
        # COTS motor placeholder for benchmark-owned fixture inventory
        _make_box(
            "ServoMotor_DS3218",
            (40.0, 20.0, 40.5),
            (-140.0, -90.0, 60.25),
            "aluminum_6061",
        ),
    ]

    assembly = Compound(children=children)
    assembly.label = "benchmark_plan_evidence"
    assembly.metadata = CompoundMetadata()
    return assembly
