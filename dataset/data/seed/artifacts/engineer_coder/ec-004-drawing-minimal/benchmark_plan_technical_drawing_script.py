from build123d import Align, Box, Compound, Location, TechnicalDrawing

from utils.metadata import CompoundMetadata, PartMetadata


def _make_fixture_box() -> Box:
    part = Box(40.0, 40.0, 40.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location((0.0, 0.0, 20.0))
    )
    part.label = "environment_fixture"
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part


def _make_fixture_wrapper() -> Compound:
    wrapper = Compound(children=[_make_fixture_box()])
    wrapper.label = "environment_fixture"
    wrapper.metadata = CompoundMetadata(fixed=True)
    return wrapper


def build():
    TechnicalDrawing(title="Raised shelf lift benchmark drafting")
    fixtures = Compound(children=[_make_fixture_wrapper()])
    fixtures.label = "benchmark_plan_drafting"
    fixtures.metadata = CompoundMetadata(fixed=True)
    return fixtures
