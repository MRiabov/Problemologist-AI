from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def _build_part(
    *,
    label: str,
    length: float,
    width: float,
    height: float,
    x: float,
    material_id: str,
):
    part = Box(length, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.moved(Location((x, 0.0, 0.0)))
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
            x=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="entry_catcher",
            length=170.0,
            width=140.0,
            height=35.0,
            x=-200.0,
            material_id="hdpe",
        ),
        _build_part(
            label="outer_rail",
            length=560.0,
            width=18.0,
            height=28.0,
            x=140.0,
            material_id="hdpe",
        ),
        _build_part(
            label="inner_rail",
            length=520.0,
            width=18.0,
            height=28.0,
            x=120.0,
            material_id="hdpe",
        ),
        _build_part(
            label="blocker_skirt",
            length=210.0,
            width=20.0,
            height=60.0,
            x=280.0,
            material_id="hdpe",
        ),
        _build_part(
            label="goal_tray",
            length=160.0,
            width=120.0,
            height=35.0,
            x=330.0,
            material_id="hdpe",
        ),
    ]

    # Stagger the rails in Z so the preview stays compact without solids
    # intersecting in 3D.
    children[1] = children[1].moved(Location((0.0, 0.0, 10.0)))
    children[2] = children[2].moved(Location((0.0, 90.0, 10.0)))
    children[3] = children[3].moved(Location((0.0, -90.0, 10.0)))
    children[4] = children[4].moved(Location((0.0, 0.0, 10.0)))
    children[5] = children[5].moved(Location((0.0, 70.0, 80.0)))

    assembly = Compound(children=children)
    assembly.label = "solution_plan_evidence"
    assembly.metadata = CompoundMetadata()
    return assembly
