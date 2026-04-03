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
    fixed: bool = True,
):
    part = Box(length, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.moved(Location((x, y, z)))
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part


def build():
    TechnicalDrawing(title="Seeded drafting")
    children = [
        _build_part(
            label="base_frame",
            length=560.0,
            width=180.0,
            height=12.0,
            x=0.0,
            y=0.0,
            z=48.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="bridge_deck",
            length=300.0,
            width=95.0,
            height=8.0,
            x=10.0,
            y=0.0,
            z=60.0,
            material_id="aluminum_6061",
            fixed=False,
        ),
        _build_part(
            label="left_fence",
            length=300.0,
            width=20.0,
            height=35.0,
            x=10.0,
            y=-57.5,
            z=60.0,
            material_id="hdpe",
        ),
        _build_part(
            label="right_fence",
            length=300.0,
            width=20.0,
            height=35.0,
            x=10.0,
            y=57.5,
            z=60.0,
            material_id="hdpe",
        ),
        _build_part(
            label="landing_pocket",
            length=130.0,
            width=110.0,
            height=30.0,
            x=225.0,
            y=0.0,
            z=68.0,
            material_id="hdpe",
        ),
    ]

    assembly = Compound(children=children)
    assembly.label = "solution_plan_technical_drawing"
    assembly.metadata = CompoundMetadata()
    return assembly
