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
    # Long ramp with elevated bridge section clearing the center_speed_bump
    # forbid zone at [60, -70, 0] to [150, 70, 55].
    # Bridge support at x=105 elevates ramp above z=55.
    children = [
        _make_box(
            "ramp_base",
            (300.0, 140.0, 10.0),
            (-250.0, 0.0, 5.0),
            "aluminum_6061",
        ),
        _make_box(
            "bridge_support",
            (50.0, 120.0, 60.0),
            (30.0, 0.0, 30.0),
            "aluminum_6061",
        ),
        _make_box(
            "ramp_surface",
            (800.0, 100.0, 8.0),
            (0.0, 0.0, 74.0),
            "hdpe",
        ),
        _make_box(
            "left_wall",
            (800.0, 16.0, 30.0),
            (0.0, -66.0, 89.0),
            "hdpe",
        ),
        _make_box(
            "right_wall",
            (800.0, 16.0, 30.0),
            (0.0, 66.0, 89.0),
            "hdpe",
        ),
        _make_box(
            "capture_lip",
            (30.0, 100.0, 10.0),
            (-400.0, 0.0, 84.0),
            "hdpe",
        ),
        # The goal_funnel intentionally captures the goal zone to hand the ball
        # into the seeded goal area.
        _make_box(
            "goal_funnel",
            (80.0, 100.0, 24.0),
            (455.0, 0.0, 88.0),
            "hdpe",
        ),
    ]

    subassembly = Compound(children=children)
    subassembly.label = "fast_transfer_ramp"
    subassembly.metadata = CompoundMetadata()
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
