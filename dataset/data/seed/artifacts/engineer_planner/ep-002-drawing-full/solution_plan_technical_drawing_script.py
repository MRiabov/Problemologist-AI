"""Bridge crossing solution plan technical drawing.

The landing_pocket explicitly captures the goal zone to receive the
transferred cube. The design occupies the goal zone volume to ensure
the cube settles inside the target area.
"""

from build123d import (
    Align,
    Box,
    Compound,
    Location,
    TechnicalDrawing,
)

from shared.models.schemas import CompoundMetadata, PartMetadata


def _make_box(
    label: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material_id: str,
):
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location(center)
    )
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=True)
    return part


def build():
    TechnicalDrawing(title="Bridge crossing assembly technical drawing")

    children = [
        # Base platform on the spawn side (left of forbid zone)
        _make_box(
            "base_frame",
            (160.0, 180.0, 12.0),
            (-260.0, 0.0, 6.0),
            "aluminum_6061",
        ),
        # Left support column, sits on base_frame and reaches up
        _make_box(
            "left_fence",
            (20.0, 20.0, 48.0),
            (-200.0, -40.0, 36.0),
            "hdpe",
        ),
        # Right support column, sits on base_frame and reaches up
        _make_box(
            "right_fence",
            (20.0, 20.0, 48.0),
            (-200.0, 40.0, 36.0),
            "hdpe",
        ),
        # Elevated bridge deck supported by columns, passes OVER forbid zone
        # Ends before goal zone (x < 210) to avoid overlap
        _make_box(
            "bridge_deck",
            (400.0, 95.0, 8.0),
            (-10.0, 0.0, 64.0),
            "aluminum_6061",
        ),
        # Landing pocket on the goal side, below bridge deck exit
        _make_box(
            "landing_pocket",
            (80.0, 100.0, 15.0),
            (290.0, 0.0, 7.5),
            "hdpe",
        ),
    ]

    # The landing_pocket explicitly captures the goal zone to receive the
    # transferred cube.

    subassembly = Compound(children=children)
    subassembly.label = "bridge_crossing"
    subassembly.metadata = CompoundMetadata()
    # Wrap in an unlabeled root so the subassembly label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
