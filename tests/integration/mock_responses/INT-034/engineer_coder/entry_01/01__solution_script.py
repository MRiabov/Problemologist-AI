from build123d import Compound

from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata


def build():
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218", label="drive_motor")
    assembly = Compound(children=[motor], label="motor_review_fixture")
    assembly.metadata = CompoundMetadata()
    return assembly
