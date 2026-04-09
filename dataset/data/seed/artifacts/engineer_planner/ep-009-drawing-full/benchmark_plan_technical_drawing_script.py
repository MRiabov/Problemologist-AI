from build123d import Align, Box, Compound, Location, TechnicalDrawing

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    TechnicalDrawing(title="Benchmark environment technical drawing")

    # Environment fixture representing the benchmark environment.
    # Positioned to stay within the build zone and avoid the center_speed_bump forbid zone.
    fixture = Box(160.0, 120.0, 80.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.move(Location((-350.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    assembly = Compound(children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
