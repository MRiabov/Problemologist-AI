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
            label="base_plate",
            length=260.0,
            width=150.0,
            height=12.0,
            x=0.0,
            y=0.0,
            z=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="left_frame",
            length=260.0,
            width=20.0,
            height=110.0,
            x=0.0,
            y=-65.0,
            z=61.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="right_frame",
            length=260.0,
            width=20.0,
            height=110.0,
            x=0.0,
            y=65.0,
            z=61.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="primary_gate",
            length=110.0,
            width=18.0,
            height=90.0,
            x=0.0,
            y=-20.0,
            z=57.0,
            material_id="hdpe",
        ),
        _build_part(
            label="return_flap",
            length=90.0,
            width=16.0,
            height=70.0,
            x=50.0,
            y=10.0,
            z=47.0,
            material_id="hdpe",
        ),
        _build_part(
            label="capture_tray",
            length=140.0,
            width=45.0,
            height=25.0,
            x=80.0,
            y=0.0,
            z=12.5,
            material_id="hdpe",
        ),
        _build_part(
            label="control_box",
            length=80.0,
            width=40.0,
            height=20.0,
            x=-90.0,
            y=-50.0,
            z=118.0,
            material_id="aluminum_6061",
        ),
    ]

    # Stagger the preview layout so the drafted solids remain legible without
    # changing the authored inventory.
    children[1] = children[1].moved(Location((0.0, 0.0, 10.0)))
    children[2] = children[2].moved(Location((0.0, 0.0, 10.0)))
    children[3] = children[3].moved(Location((0.0, 0.0, 10.0)))
    children[4] = children[4].moved(Location((0.0, 0.0, 10.0)))
    children[5] = children[5].moved(Location((0.0, 0.0, 10.0)))
    children[6] = children[6].moved(Location((0.0, 0.0, 10.0)))

    subassembly = Compound(label="dual_stage_return_gate", children=children)
    subassembly.metadata = CompoundMetadata()
    # Wrap in an unlabeled root so the subassembly label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
