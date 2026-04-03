from build123d import Align, Box, BuildPart, Compound, Location

from shared.models.schemas import CompoundMetadata, PartMetadata


def _part(
    *,
    label: str,
    length: float,
    width: float,
    height: float,
    x: float,
    y: float,
    z: float,
    material_id: str,
    fixed: bool,
):
    with BuildPart() as part:
        Box(length, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part.part = part.part.move(Location((x, y, z)))
    part.part.label = label
    part.part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part.part


def build() -> Compound:
    base_frame = _part(
        label="base_frame",
        length=560.0,
        width=180.0,
        height=12.0,
        x=0.0,
        y=0.0,
        z=48.0,
        material_id="aluminum_6061",
        fixed=True,
    )
    bridge_deck = _part(
        label="bridge_deck",
        length=300.0,
        width=95.0,
        height=8.0,
        x=10.0,
        y=0.0,
        z=60.0,
        material_id="aluminum_6061",
        fixed=False,
    )
    left_fence = _part(
        label="left_fence",
        length=300.0,
        width=20.0,
        height=35.0,
        x=10.0,
        y=-57.5,
        z=60.0,
        material_id="hdpe",
        fixed=True,
    )
    right_fence = _part(
        label="right_fence",
        length=300.0,
        width=20.0,
        height=35.0,
        x=10.0,
        y=57.5,
        z=60.0,
        material_id="hdpe",
        fixed=True,
    )
    landing_pocket = _part(
        label="landing_pocket",
        length=130.0,
        width=110.0,
        height=30.0,
        x=225.0,
        y=0.0,
        z=68.0,
        material_id="hdpe",
        fixed=True,
    )

    assembly = Compound(
        label="bridge_deploy_assembly",
        children=[
            base_frame,
            bridge_deck,
            left_fence,
            right_fence,
            landing_pocket,
        ],
    )
    assembly.metadata = CompoundMetadata(fixed=False)
    return assembly
