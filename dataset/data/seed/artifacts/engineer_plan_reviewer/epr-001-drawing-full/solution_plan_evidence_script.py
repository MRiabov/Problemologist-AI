from build123d import (
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Compound,
    Location,
    Plane,
    Rectangle,
    loft,
)

from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata, PartMetadata


def _tag_part(part, *, label: str, material_id: str, fixed: bool = True):
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part


def _build_plate(
    *,
    label: str,
    length: float,
    width: float,
    height: float,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    material_id: str,
) -> object:
    with BuildPart() as builder:
        Box(length, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return _tag_part(
        builder.part.moved(Location((x, y, z))),
        label=label,
        material_id=material_id,
    )


def _build_frustum(
    *,
    label: str,
    bottom_length: float,
    bottom_width: float,
    top_length: float,
    top_width: float,
    height: float,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    material_id: str,
) -> object:
    with BuildPart() as builder:
        with BuildSketch(Plane.XY.offset(0.0)):
            Rectangle(
                bottom_length,
                bottom_width,
                align=(Align.CENTER, Align.CENTER),
            )
        with BuildSketch(Plane.XY.offset(height)):
            Rectangle(top_length, top_width, align=(Align.CENTER, Align.CENTER))
        loft()
    return _tag_part(
        builder.part.moved(Location((x, y, z))),
        label=label,
        material_id=material_id,
    )


def _build_motor_drive() -> object:
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218")
    motor = motor.moved(Location((-460.0, -55.0, 30.0), (0.0, 90.0, 0.0)))
    return motor


def build() -> Compound:
    base_plate = _build_plate(
        label="base_plate",
        length=980.0,
        width=170.0,
        height=10.0,
        material_id="aluminum_6061",
    )
    entry_funnel = _build_frustum(
        label="entry_funnel",
        bottom_length=90.0,
        bottom_width=70.0,
        top_length=180.0,
        top_width=140.0,
        height=40.0,
        x=-455.0,
        y=0.0,
        z=10.0,
        material_id="hdpe",
    )
    roller_bed = _build_plate(
        label="roller_bed",
        length=820.0,
        width=70.0,
        height=28.0,
        x=45.0,
        y=0.0,
        z=10.0,
        material_id="hdpe",
    )
    idler_guide = _build_plate(
        label="idler_guide",
        length=820.0,
        width=18.0,
        height=24.0,
        x=45.0,
        y=0.0,
        z=38.0,
        material_id="hdpe",
    )
    goal_tray = _build_frustum(
        label="goal_tray",
        bottom_length=100.0,
        bottom_width=70.0,
        top_length=150.0,
        top_width=120.0,
        height=35.0,
        x=455.0,
        y=0.0,
        z=10.0,
        material_id="hdpe",
    )
    motor_drive = _build_motor_drive()

    assembly = Compound(
        children=[
            base_plate,
            entry_funnel,
            roller_bed,
            idler_guide,
            goal_tray,
            motor_drive,
        ]
    )
    assembly.label = "solution_plan_evidence"
    assembly.metadata = CompoundMetadata()
    return assembly
