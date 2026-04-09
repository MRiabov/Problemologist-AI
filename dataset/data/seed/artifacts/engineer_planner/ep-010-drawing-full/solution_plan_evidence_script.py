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
    # Ramp positioned on the right side (x > 60) to clear the
    # direct_drop_dead_zone forbid zone at [-60, -45, 0] to [60, 45, 130].
    # All parts fit within build_zone [-120, -160, 0] to [360, 160, 280].
    children = [
        _make_box(
            "ramp_base",
            (120.0, 120.0, 10.0),
            (130.0, 0.0, 5.0),
            "aluminum_6061",
        ),
        _make_box(
            "ramp_surface",
            (260.0, 100.0, 8.0),
            (200.0, 0.0, 130.0),
            "hdpe",
        ),
        _make_box(
            "left_wall",
            (260.0, 16.0, 32.0),
            (200.0, -66.0, 146.0),
            "hdpe",
        ),
        _make_box(
            "right_wall",
            (260.0, 16.0, 32.0),
            (200.0, 66.0, 146.0),
            "hdpe",
        ),
        _make_box(
            "capture_lip",
            (30.0, 100.0, 10.0),
            (145.0, 0.0, 140.0),
            "hdpe",
        ),
        # The goal_funnel intentionally captures the goal zone to hand the ball
        # into the seeded lower bin goal area.
        _make_box(
            "goal_funnel",
            (80.0, 90.0, 24.0),
            (250.0, 0.0, 60.0),
            "hdpe",
        ),
    ]

    subassembly = Compound(children=children)
    subassembly.label = "lower_bin_ramp"
    subassembly.metadata = CompoundMetadata()
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
