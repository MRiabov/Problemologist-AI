from __future__ import annotations

from build123d import Align, Box, Compound, Location

from shared.models.schemas import CompoundMetadata, PartMetadata

# The `right_goal_deck` geometry is designed to occupy the goal zone so the object must rest there to score.


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
            "left_start_deck",
            (180.0, 180.0, 70.0),
            (-220.0, 0.0, 35.0),
            "aluminum_6061",
        ),
        _make_box(
            "right_goal_deck",
            (200.0, 180.0, 70.0),
            (250.0, 0.0, 35.0),
            "aluminum_6061",
        ),
        _make_box(
            "bridge_reference_table",
            (80.0, 60.0, 15.0),
            (-200.0, 130.0, 50.0),
            "hdpe",
        ),
        _make_box(
            "gap_floor_guard",
            (120.0, 10.0, 5.0),
            (10.0, 160.0, 2.5),
            "aluminum_6061",
        ),
    ]

    assembly = Compound(children=children)
    assembly.label = "benchmark_plan_evidence"
    assembly.metadata = CompoundMetadata()
    return assembly
