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
    # Ramp positioned on the left side, clear of the platform_travel_clearance
    # forbid zone at [90, -90, 0] to [170, 90, 210].
    # Parts are spaced to avoid geometric intersections.
    children = [
        _make_box(
            "ramp_base",
            (160.0, 120.0, 10.0),
            (-60.0, 0.0, 5.0),
            "aluminum_6061",
        ),
        _make_box(
            "ramp_surface",
            (140.0, 100.0, 8.0),
            (-60.0, 0.0, 24.0),
            "hdpe",
        ),
        _make_box(
            "left_wall",
            (140.0, 16.0, 30.0),
            (-60.0, -66.0, 39.0),
            "hdpe",
        ),
        _make_box(
            "right_wall",
            (140.0, 16.0, 30.0),
            (-60.0, 66.0, 39.0),
            "hdpe",
        ),
        _make_box(
            "capture_lip",
            (20.0, 100.0, 12.0),
            (-135.0, 0.0, 34.0),
            "hdpe",
        ),
        _make_box(
            "goal_funnel",
            (40.0, 80.0, 24.0),
            (20.0, 0.0, 40.0),
            "hdpe",
        ),
    ]

    subassembly = Compound(children=children)
    subassembly.label = "platform_ramp"
    subassembly.metadata = CompoundMetadata()
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
