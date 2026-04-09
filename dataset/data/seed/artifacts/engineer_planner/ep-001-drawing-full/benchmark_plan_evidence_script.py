from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    # Single solid environment fixture that stays within the build zone
    # and avoids the shelf_support_clearance forbid zone [180, -95, 0] to [280, 95, 210].
    # Positioned on the left side (negative x) to represent the starting area.
    fixture = Box(300.0, 300.0, 200.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.move(Location((-100.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    # Wrap in an unlabeled root so the fixture label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
