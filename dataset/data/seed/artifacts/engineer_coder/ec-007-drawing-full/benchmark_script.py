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
    fixed: bool = True,
):
    part = Box(length, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.moved(Location((x, y, z)))
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part


def build():
    children = [
        _build_part(
            label="environment_fixture",
            length=700.0,
            width=280.0,
            height=10.0,
            x=30.0,
            y=0.0,
            z=0.0,
            material_id="aluminum_6061",
        ),
    ]
    assembly = Compound(label="low_friction_benchmark", children=children)
    assembly.metadata = CompoundMetadata()
    return assembly
