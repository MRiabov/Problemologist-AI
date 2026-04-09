from build123d import Align, Box, Compound, Location, TechnicalDrawing

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    TechnicalDrawing(title="Benchmark environment technical drawing")

    # Environment fixture positioned clear of the platform_travel_clearance
    # forbid zone at [90, -90, 0] to [170, 90, 210].
    # Placed on the left side (negative x) to represent the starting platform area.
    fixture = Box(160.0, 160.0, 80.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fixture = fixture.move(Location((-40.0, 0.0, 0.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    assembly = Compound(children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
