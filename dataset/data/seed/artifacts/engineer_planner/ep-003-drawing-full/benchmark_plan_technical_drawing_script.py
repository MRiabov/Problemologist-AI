from build123d import Align, Box, Compound, Location, TechnicalDrawing

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    TechnicalDrawing(title="Seeded benchmark drafting")

    # Single solid environment fixture that stays within the build zone
    # and avoids the central_blocker forbid zone [120, -130, 0] to [260, 130, 150].
    # Positioned on the left side (negative x) to represent the starting area.
    fixture = Box(300.0, 300.0, 200.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.move(Location((-90.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    # Wrap in an unlabeled root so the fixture label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
