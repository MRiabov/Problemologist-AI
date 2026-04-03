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


def _build_motor():
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218")
    motor = motor.move(Location((-205.0, -110.0, 20.0)))
    motor.label = "ServoMotor_DS3218"
    motor.metadata.cots_id = None
    return motor


def build():
    TechnicalDrawing(title="Raised shelf lift drafting")
    children = [
        _build_part(
            label="lift_base",
            length=280.0,
            width=150.0,
            height=10.0,
            x=0.0,
            y=0.0,
            z=0.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="left_frame",
            length=360.0,
            width=18.0,
            height=120.0,
            x=-20.0,
            y=-67.5,
            z=10.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="right_frame",
            length=360.0,
            width=18.0,
            height=120.0,
            x=-20.0,
            y=67.5,
            z=10.0,
            material_id="aluminum_6061",
        ),
        _build_part(
            label="belt_bed",
            length=300.0,
            width=95.0,
            height=18.0,
            x=0.0,
            y=0.0,
            z=10.0,
            material_id="hdpe",
        ),
        _build_part(
            label="upper_tray",
            length=160.0,
            width=120.0,
            height=24.0,
            x=370.0,
            y=0.0,
            z=220.0,
            material_id="hdpe",
        ),
        _build_motor(),
    ]

    assembly = Compound(children=children)
    assembly.label = "starter_assembly"
    assembly.metadata = CompoundMetadata()
    return assembly
