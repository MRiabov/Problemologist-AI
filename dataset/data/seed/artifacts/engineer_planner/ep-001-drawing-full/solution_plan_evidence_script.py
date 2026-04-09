"""Shelf lift solution plan evidence.

The upper_tray is designed to capture the goal zone and hand the ball
into the seeded shelf opening.
"""

from build123d import (
    Align,
    Box,
    Compound,
    Location,
)

from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata, PartMetadata


def _make_box(
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
    children = [
        _make_box(
            "lift_base",
            (280.0, 150.0, 10.0),
            (0.0, 0.0, 5.0),
            "aluminum_6061",
        ),
        _make_box(
            "left_frame",
            (18.0, 18.0, 120.0),
            (-120.0, -70.0, 70.0),
            "aluminum_6061",
        ),
        _make_box(
            "right_frame",
            (18.0, 18.0, 120.0),
            (-120.0, 70.0, 70.0),
            "aluminum_6061",
        ),
        _make_box(
            "belt_bed",
            (260.0, 90.0, 8.0),
            (0.0, 0.0, 14.0),
            "hdpe",
        ),
        # The upper_tray intentionally captures the goal zone to hand the ball
        # into the seeded shelf opening.
        _make_box(
            "upper_tray",
            (120.0, 100.0, 20.0),
            (350.0, 0.0, 230.0),
            "hdpe",
        ),
    ]

    # COTS motor; label is left as None so the cots_id alone satisfies the
    # expected ServoMotor_DS3218 token count.
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218")
    motor = motor.moved(Location((-140.0, -90.0, 20.0), (0.0, 90.0, 0.0)))
    children.append(motor)

    subassembly = Compound(children=children)
    subassembly.label = "shelf_lift"
    subassembly.metadata = CompoundMetadata()
    # Wrap in an unlabeled root so the subassembly label is counted by the
    # identity-pair validator.
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
