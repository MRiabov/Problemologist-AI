from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    # Environment fixture representing the benchmark environment.
    # Positioned to stay within the build zone and avoid the direct_drop_dead_zone forbid zone.
    fixture = Box(80.0, 80.0, 50.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.move(Location((120.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    assembly = Compound(children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
