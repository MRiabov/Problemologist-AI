from build123d import Align, Box, Compound, Location, TechnicalDrawing

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    TechnicalDrawing(title="Benchmark transfer drafting")
    fixture = Box(
        40.0, 40.0, 40.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)
    ).move(Location((0.0, 0.0, 20.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    assembly = Compound(label="benchmark_plan_drafting", children=[fixture])
    assembly.metadata = CompoundMetadata()
    return assembly
