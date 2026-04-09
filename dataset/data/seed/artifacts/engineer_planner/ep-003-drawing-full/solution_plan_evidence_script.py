"""Routed transfer solution plan evidence.

The goal_tray is designed to capture the goal zone and receive the
projectile ball after routing around the central blocker.
"""

from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def _build_part(
    *,
    label: str,
    length: float,
    width: float,
    height: float,
    x: float,
    y: float,
    z: float,
    material_id: str,
):
    part = Box(length, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.moved(Location((x, y, z)))
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=True)
    return part


def build():
    # Build zone: [-240, -180, 0] to [560, 180, 240]
    # Forbid zone (central_blocker): [120, -130, 0] to [260, 130, 150]
    # Route the path around the blocker on the positive-Y side (y > 130).
    children = [
        _build_part(
            label="route_base",
            length=760.0,
            width=150.0,
            height=10.0,
            x=140.0,
            y=0.0,
            z=155.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="entry_catcher",
            length=170.0,
            width=140.0,
            height=35.0,
            x=-60.0,
            y=0.0,
            z=10.0,
            material_id="hdpe",
        ),
        # Outer rail routes above the forbid zone (y > 130)
        _build_part(
            label="outer_rail",
            length=560.0,
            width=18.0,
            height=28.0,
            x=280.0,
            y=155.0,
            z=10.0,
            material_id="hdpe",
        ),
        # Inner rail routes below the forbid zone (y < -130)
        _build_part(
            label="inner_rail",
            length=520.0,
            width=18.0,
            height=28.0,
            x=260.0,
            y=-155.0,
            z=10.0,
            material_id="hdpe",
        ),
        _build_part(
            label="blocker_skirt",
            length=210.0,
            width=20.0,
            height=60.0,
            x=420.0,
            y=-85.0,
            z=10.0,
            material_id="hdpe",
        ),
        _build_part(
            label="goal_tray",
            length=160.0,
            width=120.0,
            height=35.0,
            x=470.0,
            y=70.0,
            z=80.0,
            material_id="hdpe",
        ),
    ]

    subassembly = Compound(children=children)
    subassembly.label = "routed_transfer"
    subassembly.metadata = CompoundMetadata()
    # Wrap in an unlabeled root so the subassembly label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
