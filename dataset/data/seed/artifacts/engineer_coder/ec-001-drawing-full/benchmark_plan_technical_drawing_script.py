from build123d import Align, Box, Compound, Location, TechnicalDrawing

from utils.metadata import CompoundMetadata, PartMetadata


def _make_fixture():
    fixture = Box(
        40.0, 40.0, 40.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)
    ).move(Location((0.0, 0.0, 20.0)))
    fixture.label = "environment_fixture"
    fixture.metadata = PartMetadata(
        material_id="aluminum_6061",
        fixed=True,
    )
    return fixture


def build():
    TechnicalDrawing(title="Benchmark transfer drafting")
    fixtures = Compound(children=[_make_fixture()])
    fixtures.label = "benchmark_plan_drafting"
    fixtures.metadata = CompoundMetadata()
    return fixtures
