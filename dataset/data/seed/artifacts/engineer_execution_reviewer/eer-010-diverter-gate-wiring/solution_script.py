from build123d import Align, Box, BuildPart, Compound, Location

from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata, PartMetadata

CABLE_CORRIDOR_WAYPOINTS = [
    (-120.0, -55.0, 120.0),
    (-70.0, -55.0, 105.0),
    (-25.0, -50.0, 95.0),
]


def build() -> Compound:
    with BuildPart() as base:
        Box(180.0, 80.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    base.part.label = "mount_base"
    base.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as bracket:
        Box(40.0, 20.0, 60.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bracket.part = bracket.part.move(Location((-60.0, 0.0, 8.0)))
    bracket.part.label = "gate_bracket"
    bracket.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as gate:
        Box(70.0, 8.0, 50.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    gate.part = gate.part.move(Location((5.0, 0.0, 58.0)))
    gate.part.label = "diverter_gate"
    gate.part.metadata = PartMetadata(material_id="hdpe", fixed=False)

    gate_motor = ServoMotor.from_catalog_id(
        "ServoMotor_DS3218", label="gate_motor"
    ).move(Location((-82.0, 0.0, 22.0)))

    assembly = Compound(
        label="diverter_gate_assembly",
        children=[
            base.part,
            bracket.part,
            gate.part,
            gate_motor,
        ],
    )
    assembly.metadata = CompoundMetadata(fixed=False)
    return assembly
