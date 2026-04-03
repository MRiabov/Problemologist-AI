from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    fixture = Box(10.0, 10.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.moved(Location((0.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    assembly = Compound(label="benchmark_environment", children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
