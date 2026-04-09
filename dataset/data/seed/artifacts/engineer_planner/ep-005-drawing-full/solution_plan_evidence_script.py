from build123d import (
    Align,
    Box,
    Compound,
    Location,
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
    children = [
        _make_box(
            "freestanding_base",
            (300.0, 120.0, 12.0),
            (-50.0, 0.0, 6.0),
            "aluminum_6061",
        ),
        _make_box(
            "capture_funnel",
            (100.0, 100.0, 40.0),
            (-200.0, 0.0, 32.0),
            "hdpe",
        ),
        _make_box(
            "left_wall",
            (200.0, 15.0, 32.0),
            (0.0, -52.5, 34.0),
            "hdpe",
        ),
        _make_box(
            "right_wall",
            (200.0, 15.0, 32.0),
            (0.0, 52.5, 34.0),
            "hdpe",
        ),
        _make_box(
            "exit_tray",
            (80.0, 80.0, 35.0),
            (160.0, 0.0, 29.5),
            "hdpe",
        ),
        _make_box(
            "ballast_block",
            (120.0, 60.0, 18.0),
            (-50.0, 0.0, 21.0),
            "steel_carbon",
        ),
    ]

    subassembly = Compound(children=children)
    subassembly.label = "freestanding_transfer"
    subassembly.metadata = CompoundMetadata()
    # Wrap in an unlabeled root so the subassembly label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
