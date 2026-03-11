from build123d import Align, Box, BuildPart, Compound, Cylinder, Location

from shared.models.schemas import CompoundMetadata, PartMetadata


def build() -> Compound:
    with BuildPart() as part_1:
        Box(980, 170, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_1_part = part_1.part.move(Location((0, 0, 0)))
    part_1_part.label = "base_plate"
    part_1_part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as part_2:
        Box(180, 140, 40, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_2_part = part_2.part.move(Location((-360, 0, 10)))
    part_2_part.label = "entry_funnel"
    part_2_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_3:
        Box(820, 70, 28, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_3_part = part_3.part.move(Location((40, 0, 10)))
    part_3_part.label = "roller_bed"
    part_3_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_4:
        Box(820, 18, 24, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_4_part = part_4.part.move(Location((40, 24, 38)))
    part_4_part.label = "idler_guide"
    part_4_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_5:
        Box(150, 120, 35, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_5_part = part_5.part.move(Location((420, 0, 10)))
    part_5_part.label = "goal_tray"
    part_5_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_6:
        Cylinder(radius=18, height=55, align=(Align.MIN, Align.CENTER, Align.CENTER))
    part_6_part = part_6.part.move(Location((-120, -58, 24)))
    part_6_part.label = "gearmotor_12v_60rpm"
    part_6_part.metadata = PartMetadata(material_id="steel_cold_rolled", fixed=True)

    assembly = Compound(
        label="sideways_transfer_review_seed",
        children=[
            part_1_part,
            part_2_part,
            part_3_part,
            part_4_part,
            part_5_part,
            part_6_part,
        ],
    )
    assembly.metadata = CompoundMetadata(fixed=False)
    return assembly
