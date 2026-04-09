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
            label="freestanding_transfer",
            length=620.0,
            width=180.0,
            height=2.0,
            x=0.0,
            y=0.0,
            z=179.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="freestanding_base",
            length=620.0,
            width=180.0,
            height=12.0,
            x=0.0,
            y=0.0,
            z=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="capture_funnel",
            length=160.0,
            width=140.0,
            height=40.0,
            x=-200.0,
            y=0.0,
            z=20.0,
            material_id="hdpe",
        ),
        _build_part(
            label="left_wall",
            length=460.0,
            width=20.0,
            height=32.0,
            x=0.0,
            y=-80.0,
            z=60.0,
            material_id="hdpe",
        ),
        _build_part(
            label="right_wall",
            length=460.0,
            width=20.0,
            height=32.0,
            x=0.0,
            y=80.0,
            z=60.0,
            material_id="hdpe",
        ),
        _build_part(
            label="exit_tray",
            length=140.0,
            width=110.0,
            height=35.0,
            x=260.0,
            y=0.0,
            z=120.0,
            material_id="hdpe",
        ),
        _build_part(
            label="ballast_block",
            length=180.0,
            width=80.0,
            height=18.0,
            x=0.0,
            y=0.0,
            z=140.0,
            material_id="aluminum_6061",
        ),
    ]
    assembly = Compound(label="freestanding_transfer", children=children)
    assembly.metadata = CompoundMetadata()
    return assembly
