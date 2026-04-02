from build123d import Align, Box, BuildPart, Compound, Location

from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata, PartMetadata


def build() -> Compound:
    with BuildPart() as part_1:
        Box(980, 170, 10, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    part_1_part = part_1.part.move(Location((0, 0, 5)))
    part_1_part.label = "base_plate"
    part_1_part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as part_2:
        Box(180, 140, 40, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_2_part = part_2.part.move(Location((-460, 0, 10)))
    part_2_part.label = "entry_funnel"
    part_2_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_3:
        Box(780, 70, 28, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_3_part = part_3.part.move(Location((20, 0, 10)))
    part_3_part.label = "roller_bed"
    part_3_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_4:
        Box(780, 18, 24, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_4_part = part_4.part.move(Location((20, 24, 38)))
    part_4_part.label = "idler_guide"
    part_4_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as part_5:
        Box(150, 120, 35, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part_5_part = part_5.part.move(Location((485, 0, 10)))
    part_5_part.label = "goal_tray"
    part_5_part.metadata = PartMetadata(material_id="hdpe", fixed=True)

    part_6_part = ServoMotor.from_catalog_id(
        "ServoMotor_DS3218", label="drive_motor"
    ).move(Location((-120, -58, 37.5)))

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
