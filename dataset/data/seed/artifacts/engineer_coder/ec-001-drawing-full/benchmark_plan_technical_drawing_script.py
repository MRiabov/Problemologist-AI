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
    part.metadata = PartMetadata(material_id=material_id, fixed=True)
    return part


def build():
    TechnicalDrawing(title="No-drill benchmark environment")
    children = [
        _build_part(
            label="benchmark_environment",
            length=600.0,
            width=300.0,
            height=2.0,
            x=0.0,
            y=0.0,
            z=11.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="environment_fixture",
            length=600.0,
            width=300.0,
            height=10.0,
            x=0.0,
            y=0.0,
            z=0.0,
            material_id="aluminum_6061",
        ),
    ]
    assembly = Compound(label="benchmark_environment", children=children)
    assembly.metadata = CompoundMetadata()
    return assembly
