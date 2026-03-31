from build123d import Align, Box, BuildPart, Compound, Cylinder, Location

from shared.models.schemas import CompoundMetadata, PartMetadata


def build() -> Compound:
    with BuildPart() as part_1:
        Box(640, 180, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_1_part = part_1.part.move(Location((0, 0, 0)))
    part_1_part.label = "base_plate"
    part_1_part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as part_2:
        Box(420, 70, 110, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_2_part = part_2.part.move(Location((0, 0, 10)))
    part_2_part.label = "chute_body"
    part_2_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_3:
        Box(70, 8, 55, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_3_part = part_3.part.move(Location((30, 39, 72)))
    part_3_part.label = "meter_gate"
    part_3_part.metadata = PartMetadata(material_id="hdpe", fixed=False)

    with BuildPart() as part_4:
        Cylinder(radius=16, height=42, align=(Align.MIN, Align.CENTER, Align.CENTER))
    part_4_part = part_4.part.move(Location((-92, -68, 34)))
    part_4_part.label = "gate_motor"
    part_4_part.metadata = PartMetadata(material_id="steel_cold_rolled", fixed=True)

    with BuildPart() as part_5:
        Box(130, 110, 45, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_5_part = part_5.part.move(Location((275, 0, 10)))
    part_5_part.label = "goal_bin"
    part_5_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    assembly = Compound(
        label="timed_gate_review_seed",
        children=[part_1_part, part_2_part, part_3_part, part_4_part, part_5_part],
    )
    assembly.metadata = CompoundMetadata(fixed=False)
    return assembly
