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
    y: float,
    z: float,
    material_id: str,
):
    part = Box(length, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.moved(Location((x, y, z)))
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=True)
    return part


def _build_base_plate():
    base_plate = Box(520.0, 130.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    cutout = Box(120.0, 145.0, 12.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    cutout = cutout.moved(Location((115.0, 0.0, 5.0)))
    base_plate = base_plate.cut(cutout)
    base_plate.label = "base_plate"
    base_plate.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return base_plate


def build():
    TechnicalDrawing(title="Timed gate metering drafting")
    children = [
        _build_base_plate(),
        _build_part(
            label="settling_chute",
            length=220.0,
            width=90.0,
            height=35.0,
            x=-130.0,
            y=95.0,
            z=12.0,
            material_id="hdpe",
        ),
        _build_part(
            label="metering_wheel_guard",
            length=140.0,
            width=55.0,
            height=32.0,
            x=-35.0,
            y=-20.0,
            z=50.0,
            material_id="hdpe",
        ),
        _build_part(
            label="guide_rail",
            length=150.0,
            width=18.0,
            height=24.0,
            x=80.0,
            y=100.0,
            z=85.0,
            material_id="hdpe",
        ),
        _build_part(
            label="post_gate_channel",
            length=220.0,
            width=70.0,
            height=30.0,
            x=180.0,
            y=-105.0,
            z=112.0,
            material_id="hdpe",
        ),
        _build_part(
            label="goal_cup",
            length=95.0,
            width=95.0,
            height=35.0,
            x=320.0,
            y=0.0,
            z=145.0,
            material_id="hdpe",
        ),
        _build_motor(),
    ]

    assembly = Compound(children=children)
    assembly.label = "solution_plan_technical_drawing"
    assembly.metadata = CompoundMetadata()
    return assembly


def _build_motor():
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218")
    motor = motor.move(Location((-92.0, -68.0, 191.0)))
    motor.label = "ServoMotor_DS3218"
    motor.metadata.cots_id = None
    return motor
