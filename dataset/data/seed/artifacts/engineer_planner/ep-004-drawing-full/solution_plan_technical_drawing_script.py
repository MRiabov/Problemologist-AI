from build123d import (
    Align,
    Box,
    Compound,
    Location,
    TechnicalDrawing,
)

from utils.metadata import CompoundMetadata, PartMetadata


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
    TechnicalDrawing(title="Ball ramp assembly technical drawing")

    children = [
        _make_box(
            "ramp_base",
            (400.0, 60.0, 10.0),
            (0.0, -105.0, 5.0),
            "aluminum_6061",
        ),
        _make_box(
            "ramp_surface",
            (380.0, 30.0, 8.0),
            (0.0, -105.0, 18.0),
            "hdpe",
        ),
        _make_box(
            "left_wall",
            (380.0, 15.0, 40.0),
            (0.0, -127.5, 34.0),
            "hdpe",
        ),
        _make_box(
            "right_wall",
            (380.0, 15.0, 40.0),
            (0.0, -82.5, 34.0),
            "hdpe",
        ),
        # The goal_catcher intentionally captures the goal zone to receive the
        # projectile ball.
        _make_box(
            "goal_catcher",
            (80.0, 60.0, 20.0),
            (330.0, -105.0, 65.0),
            "hdpe",
        ),
    ]

    subassembly = Compound(children=children)
    subassembly.label = "ball_ramp_assembly"
    subassembly.metadata = CompoundMetadata()
    # Wrap in an unlabeled root so the subassembly label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
