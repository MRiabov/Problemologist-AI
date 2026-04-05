from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    fixture = Box(40.0, 40.0, 40.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    fixture = fixture.move(Location((0.0, 0.0, 20.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    assembly = Compound(children=[fixture])
    assembly.label = "benchmark_plan_drafting"
    assembly.metadata = CompoundMetadata(fixed=True)
    return assembly
