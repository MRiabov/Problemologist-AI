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
    fixed: bool = True,
):
    part = Box(length, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.moved(Location((x, 0.0, 0.0)))
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part


def build():
    children = [
        _build_part(
            label="entry_ramp",
            length=180.0,
            width=120.0,
            height=50.0,
            x=-150.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="gate_housing",
            length=120.0,
            width=140.0,
            height=150.0,
            x=100.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="gate_pivot_arm",
            length=18.0,
            width=120.0,
            height=90.0,
            x=100.0,
            material_id="steel_cold_rolled",
            fixed=False,
        ),
        _build_part(
            label="exit_tray",
            length=160.0,
            width=110.0,
            height=30.0,
            x=320.0,
            material_id="hdpe",
        ),
        _build_part(
            label="entry_ramp",
            length=180.0,
            width=120.0,
            height=50.0,
            x=-150.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="gate_housing",
            length=120.0,
            width=140.0,
            height=150.0,
            x=100.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="gate_pivot_arm",
            length=18.0,
            width=120.0,
            height=90.0,
            x=100.0,
            material_id="steel_cold_rolled",
            fixed=False,
        ),
        _build_part(
            label="exit_tray",
            length=160.0,
            width=110.0,
            height=30.0,
            x=320.0,
            material_id="hdpe",
        ),
    ]

    assembly = Compound(children=children)
    assembly.label = "benchmark_plan_evidence"
    assembly.metadata = CompoundMetadata()
    return assembly
