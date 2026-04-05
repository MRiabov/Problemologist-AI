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
    TechnicalDrawing(title="Narrow goal funnel drafting")
    children = [
        _build_part(
            label="funnel_base",
            length=700.0,
            width=150.0,
            height=10.0,
            x=210.0,
            y=0.0,
            z=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="wide_entry",
            length=170.0,
            width=150.0,
            height=38.0,
            x=-55.0,
            y=0.0,
            z=10.0,
            material_id="hdpe",
        ),
        _build_part(
            label="taper_left",
            length=470.0,
            width=18.0,
            height=34.0,
            x=170.0,
            y=84.0,
            z=10.0,
            material_id="hdpe",
        ),
        _build_part(
            label="taper_right",
            length=470.0,
            width=18.0,
            height=34.0,
            x=170.0,
            y=-84.0,
            z=10.0,
            material_id="hdpe",
        ),
        _build_part(
            label="throat_insert",
            length=120.0,
            width=30.0,
            height=26.0,
            x=410.0,
            y=0.0,
            z=10.0,
            material_id="hdpe",
        ),
        _build_part(
            label="goal_pocket",
            length=90.0,
            width=55.0,
            height=28.0,
            x=515.0,
            y=0.0,
            z=10.0,
            material_id="hdpe",
        ),
    ]

    subassembly = Compound(label="precision_funnel", children=children)
    subassembly.metadata = CompoundMetadata()
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
