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
    children = [
        _build_part(
            label="route_base",
            length=760.0,
            width=150.0,
            height=10.0,
            x=150.0,
            y=105.0,
            z=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="entry_catcher",
            length=170.0,
            width=140.0,
            height=35.0,
            x=40.0,
            y=110.0,
            z=20.0,
            material_id="hdpe",
        ),
        _build_part(
            label="outer_rail",
            length=560.0,
            width=18.0,
            height=28.0,
            x=280.0,
            y=145.0,
            z=60.0,
            material_id="hdpe",
        ),
        _build_part(
            label="inner_rail",
            length=520.0,
            width=18.0,
            height=28.0,
            x=260.0,
            y=160.0,
            z=100.0,
            material_id="hdpe",
        ),
        _build_part(
            label="blocker_skirt",
            length=210.0,
            width=20.0,
            height=60.0,
            x=105.0,
            y=150.0,
            z=140.0,
            material_id="hdpe",
        ),
        _build_part(
            label="goal_tray",
            length=160.0,
            width=120.0,
            height=35.0,
            x=80.0,
            y=120.0,
            z=190.0,
            material_id="hdpe",
        ),
    ]

    assembly = Compound(label="routed_transfer", children=children)
    assembly.metadata = CompoundMetadata()
    return assembly
