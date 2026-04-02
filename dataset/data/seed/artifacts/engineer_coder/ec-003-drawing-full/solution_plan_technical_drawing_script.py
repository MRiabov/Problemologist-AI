from build123d import Align, Box, Compound, Location, TechnicalDrawing

from shared.cots.parts.motors import ServoMotor
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
    TechnicalDrawing(title="Timed gate metering drafting")
    children = [
        _build_part(
            label="base_plate",
            length=520.0,
            width=130.0,
            height=10.0,
            x=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="settling_chute",
            length=220.0,
            width=90.0,
            height=35.0,
            x=-130.0,
            material_id="hdpe",
        ),
        _build_part(
            label="metering_wheel_guard",
            length=140.0,
            width=55.0,
            height=32.0,
            x=-35.0,
            material_id="hdpe",
        ),
        _build_part(
            label="guide_rail",
            length=150.0,
            width=18.0,
            height=24.0,
            x=55.0,
            material_id="hdpe",
        ),
        _build_part(
            label="post_gate_channel",
            length=220.0,
            width=70.0,
            height=30.0,
            x=180.0,
            material_id="hdpe",
        ),
        _build_part(
            label="goal_cup",
            length=95.0,
            width=95.0,
            height=35.0,
            x=320.0,
            material_id="hdpe",
        ),
        ServoMotor.from_catalog_id("ServoMotor_DS3218", label="ServoMotor_DS3218").move(
            Location((-92.0, -68.0, 34.0))
        ),
    ]

    assembly = Compound(children=children)
    assembly.label = "solution_plan_technical_drawing"
    assembly.metadata = CompoundMetadata()
    return assembly
