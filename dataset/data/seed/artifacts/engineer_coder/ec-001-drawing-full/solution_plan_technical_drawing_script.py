from build123d import Align, Box, Compound, Location, TechnicalDrawing

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
    part.metadata = PartMetadata(material_id=material_id)
    return part


def build():
    TechnicalDrawing(title="EC-001 Gravity Ramp - Planner Evidence")
    children = [
        _build_part(
            label="gravity_ramp",
            length=150.0,
            width=600.0,
            height=8.0,
            x=0.0,
            y=-375.0,
            z=1050.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="gravity_ramp_left_wall",
            length=30.0,
            width=600.0,
            height=25.0,
            x=-60.0,
            y=-375.0,
            z=1054.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="gravity_ramp_right_wall",
            length=30.0,
            width=600.0,
            height=25.0,
            x=60.0,
            y=-375.0,
            z=1054.0,
            material_id="aluminum_6061",
        ),
    ]

    assembly = Compound(children=children)
    assembly.label = "gravity_ramp_assembly"
    assembly.metadata = CompoundMetadata()
    return assembly
