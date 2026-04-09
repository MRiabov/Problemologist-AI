from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    # Single solid environment fixture that stays within the build zone
    # and avoids the floor_gap forbid zone [-70, -150, -5] to [90, 150, 45].
    # Positioned on the left side (negative x) to represent the starting area.
    fixture = Box(200.0, 300.0, 200.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.move(Location((-230.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    # Wrap in an unlabeled root so the fixture label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
