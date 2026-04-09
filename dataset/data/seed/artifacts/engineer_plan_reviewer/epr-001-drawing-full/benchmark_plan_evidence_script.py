from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    # Environment fixture that defines the simulation boundary.
    fixture = Box(1200.0, 400.0, 250.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.move(Location((0.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    # Wrap in an unlabeled root so the fixture label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
