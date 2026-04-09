from __future__ import annotations

import yaml
from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


# Add authored benchmark fixtures to this children list.
# Every top-level child you add here must have a unique label, and that label
# must not be `environment`, start with `zone_`, or start with
# `benchmark_payload__` because the simulator reserves those names for the
# scene root and generated objective bodies.
def build() -> Compound:
    """Return the benchmark assembly geometry for this workspace."""
    # NOTE: Do NOT include the payload here — the simulation system spawns
    # `benchmark_payload__projectile_ball` independently from
    # `benchmark_definition.yaml`. Returning it from build() creates a duplicate
    # body that collides with the spawned ball, causing instant OUT_OF_BOUNDS.
    environment = Compound(children=[])
    environment.label = "benchmark_environment"
    environment.metadata = CompoundMetadata()
    return environment


def _load_objectives() -> dict:
    """Load objective zones from benchmark_definition.yaml."""
    with open("benchmark_definition.yaml", encoding="utf-8") as fh:
        payload = yaml.safe_load(fh) or {}
    objectives = payload.get("objectives", {})
    return objectives if isinstance(objectives, dict) else {}


def _build_objective_zone(label: str, bounds_min, bounds_max):
    size_x = float(bounds_max[0]) - float(bounds_min[0])
    size_y = float(bounds_max[1]) - float(bounds_min[1])
    size_z = float(bounds_max[2]) - float(bounds_min[2])
    center = (
        (float(bounds_min[0]) + float(bounds_max[0])) / 2.0,
        (float(bounds_min[1]) + float(bounds_max[1])) / 2.0,
        (float(bounds_min[2]) + float(bounds_max[2])) / 2.0,
    )
    zone = Box(
        max(size_x, 0.0),
        max(size_y, 0.0),
        max(size_z, 0.0),
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).move(Location(center))
    zone.label = label
    zone.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return zone


def objectives_geometry() -> Compound:
    """Return the benchmark objective overlay geometry for this workspace."""

    objectives = _load_objectives()
    children = []

    goal_zone = objectives.get("goal_zone")
    if isinstance(goal_zone, dict):
        children.append(
            _build_objective_zone(
                "zone_goal",
                goal_zone.get("min", [0.0, 0.0, 0.0]),
                goal_zone.get("max", [0.0, 0.0, 0.0]),
            )
        )

    for index, forbid_zone in enumerate(objectives.get("forbid_zones", []) or []):
        if isinstance(forbid_zone, dict):
            zone_name = (
                str(forbid_zone.get("name", f"forbid_{index}")).strip()
                or f"forbid_{index}"
            )
            children.append(
                _build_objective_zone(
                    f"zone_forbid_{index}_{zone_name}",
                    forbid_zone.get("min", [0.0, 0.0, 0.0]),
                    forbid_zone.get("max", [0.0, 0.0, 0.0]),
                )
            )

    build_zone = objectives.get("build_zone")
    if isinstance(build_zone, dict):
        children.append(
            _build_objective_zone(
                "zone_build",
                build_zone.get("min", [0.0, 0.0, 0.0]),
                build_zone.get("max", [0.0, 0.0, 0.0]),
            )
        )

    overlay = Compound(children=children) if children else Compound()
    overlay.label = "benchmark_objectives"
    overlay.metadata = CompoundMetadata()
    return overlay


result = build()
