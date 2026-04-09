"""Benchmark plan technical-drawing script for the sideways-ball benchmark.

Produces an orthographic drawing package for the approved planner inventory.
Every label and quantity must match the manufactured_parts section of
benchmark_assembly_definition.yaml exactly.
"""

from build123d import Align, Box, Compound, Location, TechnicalDrawing

from utils.metadata import CompoundMetadata, PartMetadata


def _make_box(
    label: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material_id: str = "aluminum_6061",
    fixed: bool = True,
):
    """Helper to create a labeled box part with metadata."""
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location(center)
    )
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part


def build() -> Compound:
    """Return the drafted model for the approved planner inventory."""
    TechnicalDrawing(
        title="Sideways Ball Benchmark Plan",
        sub_title="Orthographic trio for sideways ball transfer",
        drawing_number="BC-011",
    )

    # manufactured_parts (from benchmark_assembly_definition.yaml):
    #   ground_plane       qty=1
    #   floor_plate        qty=1
    #   support_tower      qty=1
    #   raised_goal_shelf  qty=1
    #   lift_carriage      qty=1

    fixtures = Compound(
        children=[
            _make_box(
                "ground_plane",
                (1.16, 0.2, 0.01),
                (0.0, 0.0, 0.005),
                material_id="hdpe",
                fixed=True,
            ),
            _make_box(
                "floor_plate",
                (0.2, 0.06, 0.01),
                (0.0, 0.0, 0.005),
                material_id="aluminum_6061",
                fixed=True,
            ),
            _make_box(
                "support_tower",
                (0.04, 0.04, 0.16),
                (0.0, 0.0, 0.09),
                material_id="aluminum_6061",
                fixed=True,
            ),
            _make_box(
                "raised_goal_shelf",
                (0.06, 0.06, 0.01),
                (0.5, 0.0, 0.17),
                material_id="aluminum_6061",
                fixed=True,
            ),
            _make_box(
                "lift_carriage",
                (0.04, 0.04, 0.02),
                (0.0, 0.0, 0.05),
                material_id="hdpe",
                fixed=False,
            ),
        ]
    )
    fixtures.label = "benchmark_fixtures"
    fixtures.metadata = CompoundMetadata()
    return fixtures
