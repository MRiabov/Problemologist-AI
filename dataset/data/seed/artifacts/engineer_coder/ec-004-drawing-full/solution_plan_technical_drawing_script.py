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
    motor.label = "drive_motor"
    # Preserve the COTS identity so the inventory pair (label="drive_motor",
    # cots_id="ServoMotor_DS3218") matches assembly_definition.yaml.
    return motor


def build():
    TechnicalDrawing(title="Raised shelf lift drafting")

    # Build the shelf_lift subassembly containing the manufactured parts.
    shelf_lift_parts = [
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
    ]

    shelf_lift = Compound(children=shelf_lift_parts)
    shelf_lift.label = "shelf_lift"
    shelf_lift.metadata = CompoundMetadata()

    # Build the drive_motor COTS part.
    motor = _build_motor()

    # Wrap both in an unlabeled root so the identity-pair validator counts
    # shelf_lift and drive_motor as top-level inventory items.
    assembly = Compound(children=[shelf_lift, motor])
    assembly.metadata = CompoundMetadata()
    return assembly
