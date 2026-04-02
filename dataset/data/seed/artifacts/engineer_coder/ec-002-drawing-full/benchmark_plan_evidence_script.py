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
            label="left_start_deck",
            length=180.0,
            width=180.0,
            height=70.0,
            x=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="left_start_deck",
            length=180.0,
            width=180.0,
            height=70.0,
            x=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="right_goal_deck",
            length=200.0,
            width=180.0,
            height=70.0,
            x=260.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="right_goal_deck",
            length=200.0,
            width=180.0,
            height=70.0,
            x=260.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="bridge_reference_table",
            length=140.0,
            width=120.0,
            height=30.0,
            x=560.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="bridge_reference_table",
            length=140.0,
            width=120.0,
            height=30.0,
            x=560.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="gap_floor_guard",
            length=160.0,
            width=300.0,
            height=40.0,
            x=820.0,
            material_id="hdpe",
        ),
        _build_part(
            label="gap_floor_guard",
            length=160.0,
            width=300.0,
            height=40.0,
            x=820.0,
            material_id="hdpe",
        ),
    ]

    assembly = Compound(children=children)
    assembly.label = "benchmark_plan_evidence"
    assembly.metadata = CompoundMetadata()
    return assembly
