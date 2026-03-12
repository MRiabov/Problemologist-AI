from build123d import Align, Box, BuildPart, Compound, Cylinder, Location

from shared.models.schemas import CompoundMetadata, PartMetadata

CABLE_CORRIDOR_WAYPOINTS = [
    (-120.0, -55.0, 120.0),
    (-70.0, -55.0, 105.0),
    (-25.0, -50.0, 95.0),
]


def build() -> Compound:
    with BuildPart() as base:
        Box(180.0, 80.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    base.part = base.part.move(Location((0.0, 0.0, 4.0)))
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

    with BuildPart() as motor_body:
        Cylinder(
            radius=12.0, height=35.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
        )
    motor_body.part = motor_body.part.move(Location((-96.0, 0.0, 22.0)))
    motor_body.part.label = "gate_motor"
    motor_body.part.metadata = PartMetadata(material_id="steel_cold_rolled", fixed=True)

    assembly = Compound(
        label="diverter_gate_assembly",
        children=[
            base.part,
            bracket.part,
            gate.part,
            motor_body.part,
        ],
    )
    assembly.metadata = CompoundMetadata(fixed=False)
    return assembly
