from build123d import Align, Box, BuildPart, Compound, Cylinder, Location

from shared.models.schemas import CompoundMetadata, PartMetadata


def build() -> Compound:
    with BuildPart() as part_1:
        Box(620, 220, 12, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_1_part = part_1.part.move(Location((0, 0, 0)))
    part_1_part.label = "base_frame"
    part_1_part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as part_2:
        Box(90, 90, 320, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_2_part = part_2.part.move(Location((-190, 0, 12)))
    part_2_part.label = "support_tower"
    part_2_part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as part_3:
        Box(220, 120, 16, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_3_part = part_3.part.move(Location((30, 0, 180)))
    part_3_part.label = "lift_carriage"
    part_3_part.metadata = PartMetadata(material_id="hdpe", fixed=False)

    with BuildPart() as part_4:
        Box(180, 120, 14, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_4_part = part_4.part.move(Location((210, 0, 250)))
    part_4_part.label = "handoff_shelf"
    part_4_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_5:
        Cylinder(radius=18, height=48, align=(Align.MIN, Align.CENTER, Align.CENTER))
    part_5_part = part_5.part.move(Location((-170, -80, 38)))
    part_5_part.label = "lift_motor"
    part_5_part.metadata = PartMetadata(material_id="steel_cold_rolled", fixed=True)

    assembly = Compound(
        label="raised_shelf_lift_review_seed",
        children=[part_1_part, part_2_part, part_3_part, part_4_part, part_5_part],
    )
    assembly.metadata = CompoundMetadata(fixed=False)
    return assembly
