from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def _make_part(
    label: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material_id: str,
):
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location(center)
    )
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=True)
    return part


def build():
    solution = Compound(
        children=[
            _make_part(
                "base_plate",
                (980.0, 170.0, 10.0),
                (0.0, 0.0, 5.0),
                "aluminum_6061",
            ),
            _make_part(
                "entry_funnel",
                (180.0, 140.0, 40.0),
                (0.0, 0.0, 35.0),
                "hdpe",
            ),
            _make_part(
                "roller_bed",
                (780.0, 70.0, 28.0),
                (0.0, 0.0, 85.0),
                "hdpe",
            ),
            _make_part(
                "idler_guide",
                (780.0, 18.0, 24.0),
                (0.0, 0.0, 120.0),
                "hdpe",
            ),
            _make_part(
                "goal_tray",
                (150.0, 120.0, 35.0),
                (0.0, 0.0, 155.0),
                "hdpe",
            ),
            _make_part(
                "ServoMotor_DS3218",
                (42.0, 20.0, 36.0),
                (400.0, 0.0, 170.0),
                "aluminum_6061",
            ),
        ]
    )
    solution.label = "solution_plan_drafting"
    solution.metadata = CompoundMetadata()
    return solution
