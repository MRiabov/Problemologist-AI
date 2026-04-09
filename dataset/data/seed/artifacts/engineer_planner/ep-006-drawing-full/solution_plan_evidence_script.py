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
            "guide_base",
            (350.0, 40.0, 10.0),
            (0.0, -115.0, 5.0),
            "aluminum_6061",
        ),
        _make_box(
            "left_guide",
            (350.0, 12.0, 30.0),
            (0.0, -129.0, 25.0),
            "hdpe",
        ),
        _make_box(
            "right_guide",
            (350.0, 12.0, 30.0),
            (0.0, -101.0, 25.0),
            "hdpe",
        ),
        _make_box(
            "entry_ramp",
            (60.0, 16.0, 20.0),
            (-175.0, -115.0, 20.0),
            "hdpe",
        ),
        _make_box(
            "exit_lip",
            (40.0, 40.0, 15.0),
            (240.0, -115.0, 17.5),
            "hdpe",
        ),
    ]

    subassembly = Compound(children=children)
    subassembly.label = "cube_guide"
    subassembly.metadata = CompoundMetadata()
    # Wrap in an unlabeled root so the subassembly label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
