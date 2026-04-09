"""Narrow ramp solution plan evidence.

The exit_funnel is designed to capture the goal zone and guide the ball
into the seeded goal area.
"""

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
    # Long ramp from spawn area to narrow goal zone.
    # No forbid zones in this benchmark; stay within build zone.
    children = [
        _make_box(
            "ramp_base",
            (400.0, 140.0, 10.0),
            (200.0, 0.0, 5.0),
            "aluminum_6061",
        ),
        _make_box(
            "ramp_surface",
            (560.0, 80.0, 8.0),
            (200.0, 0.0, 24.0),
            "hdpe",
        ),
        _make_box(
            "left_wall",
            (560.0, 14.0, 28.0),
            (200.0, -57.0, 39.0),
            "hdpe",
        ),
        _make_box(
            "right_wall",
            (560.0, 14.0, 28.0),
            (200.0, 57.0, 39.0),
            "hdpe",
        ),
        _make_box(
            "capture_lip",
            (30.0, 80.0, 10.0),
            (-75.0, 0.0, 34.0),
            "hdpe",
        ),
        _make_box(
            "exit_funnel",
            (50.0, 60.0, 20.0),
            (505.0, 0.0, 40.0),
            "hdpe",
        ),
    ]

    subassembly = Compound(children=children)
    subassembly.label = "narrow_ramp"
    subassembly.metadata = CompoundMetadata()
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
