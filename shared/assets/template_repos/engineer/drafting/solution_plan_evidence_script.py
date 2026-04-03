from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def _make_box(label: str, size: tuple[float, float, float], center: tuple[float, float, float]):
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location(center)
    )
    part.label = label
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part


def build():
    solution = Compound(
        children=[
            _make_box("bridge_deck", (64.0, 8.0, 4.0), (0.0, 0.0, 18.0)),
            _make_box("left_support", (8.0, 12.0, 12.0), (-28.0, 0.0, 10.0)),
            _make_box("right_support", (8.0, 12.0, 12.0), (28.0, 0.0, 10.0)),
            _make_box("stop_lip", (4.0, 8.0, 2.0), (30.0, 0.0, 19.0)),
        ]
    )
    solution.label = "solution_bridge"
    solution.metadata = CompoundMetadata()
    return solution
