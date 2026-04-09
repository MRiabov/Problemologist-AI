from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    # Solid environment fixture that stays within the build zone
    # and avoids the gate_swing_keepout forbid zone [40, -70, 0] to [160, 70, 150].
    fixture = Box(200.0, 200.0, 160.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.move(Location((-160.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    # Wrap in an unlabeled root so the fixture label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
