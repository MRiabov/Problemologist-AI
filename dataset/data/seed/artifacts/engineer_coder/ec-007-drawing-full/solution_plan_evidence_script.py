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
            label="low_friction_route",
            length=620.0,
            width=40.0,
            height=2.0,
            x=-30.0,
            y=120.0,
            z=170.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="slide_base",
            length=620.0,
            width=40.0,
            height=10.0,
            x=0.0,
            y=120.0,
            z=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="entry_box",
            length=160.0,
            width=120.0,
            height=38.0,
            x=-230.0,
            y=0.0,
            z=10.0,
            material_id="hdpe",
        ),
        _build_part(
            label="guide_wall_left",
            length=300.0,
            width=18.0,
            height=42.0,
            x=-80.0,
            y=-61.0,
            z=50.0,
            material_id="hdpe",
        ),
        _build_part(
            label="guide_wall_right",
            length=300.0,
            width=18.0,
            height=42.0,
            x=-65.0,
            y=61.0,
            z=50.0,
            material_id="hdpe",
        ),
        _build_part(
            label="blocker_bypass_panel",
            length=200.0,
            width=18.0,
            height=65.0,
            x=165.0,
            y=-110.0,
            z=90.0,
            material_id="hdpe",
        ),
        _build_part(
            label="goal_pocket",
            length=110.0,
            width=90.0,
            height=32.0,
            x=340.0,
            y=0.0,
            z=120.0,
            material_id="hdpe",
        ),
    ]
    assembly = Compound(label="low_friction_route", children=children)
    assembly.metadata = CompoundMetadata()
    return assembly
