from __future__ import annotations

from build123d import Align, Box, Compound, Location, Sphere

from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata, PartMetadata


def _box(
    *,
    label: str,
    size: tuple[float, float, float],
    location: tuple[float, float, float],
    material_id: str,
    fixed: bool,
) -> Box:
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    part = part.move(Location(location))
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=fixed)
    return part


def _sphere(
    *,
    label: str,
    radius: float,
    location: tuple[float, float, float],
    material_id: str,
) -> Sphere:
    part = Sphere(radius, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    part = part.move(Location(location))
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=False)
    return part


def build() -> Compound:
    ground_plane = _box(
        label="ground_plane",
        size=(0.72, 0.28, 0.01),
        location=(0.08, 0.0, 0.005),
        material_id="hdpe",
        fixed=True,
    )
    entry_ramp = _box(
        label="entry_ramp",
        size=(0.18, 0.12, 0.05),
        location=(-0.15, 0.0, 0.025),
        material_id="aluminum_6061",
        fixed=True,
    )
    gate_housing = _box(
        label="gate_housing",
        size=(0.12, 0.14, 0.15),
        location=(0.10, 0.0, 0.075),
        material_id="aluminum_6061",
        fixed=True,
    )
    gate_pivot_arm = _box(
        label="gate_pivot_arm",
        size=(0.018, 0.12, 0.09),
        location=(0.10, 0.0, 0.075),
        material_id="steel_cold_rolled",
        fixed=False,
    )
    exit_tray = _box(
        label="exit_tray",
        size=(0.16, 0.11, 0.03),
        location=(0.32, 0.0, 0.015),
        material_id="hdpe",
        fixed=True,
    )
    drive_motor = ServoMotor.from_catalog_id(
        "ServoMotor_DS3218",
        label="drive_motor",
    ).move(Location((0.115, -0.095, 0.0405)))
    drive_motor.label = "drive_motor"

    projectile_ball = _sphere(
        label="projectile_ball",
        radius=0.019,
        location=(-0.22, 0.0, 0.115),
        material_id="steel_bearing",
    )

    benchmark_compound = Compound(
        children=[
            ground_plane,
            entry_ramp,
            gate_housing,
            gate_pivot_arm,
            exit_tray,
            drive_motor,
            projectile_ball,
        ]
    )
    benchmark_compound.label = "benchmark_assembly"
    benchmark_compound.metadata = CompoundMetadata(fixed=False)
    return benchmark_compound
