"""Static and visual randomization utilities for simulation benchmarks.

Implements material randomization per architecture spec:
- Randomly switch materials for moving parts (heavier/lighter/different friction)
- Apply material colors to parts for visual diversity
- Support optional minimum strength constraints for material selection
"""

import random
from typing import Any

from pydantic import BaseModel


class MaterialAssignment(BaseModel):
    """Material assignment for a part."""

    part_name: str
    material_id: str
    color: str  # Hex color from material config
    density_g_cm3: float
    friction_coef: float
    restitution: float


class RandomizationConfig(BaseModel):
    """Configuration for static randomization."""

    seed: int
    min_strength_mpa: float | None = None  # Minimum material yield strength
    material_whitelist: list[str] | None = None  # Allowed materials


def get_eligible_materials(
    materials: dict[str, Any],
    min_strength_mpa: float | None = None,
    whitelist: list[str] | None = None,
) -> list[str]:
    """Filter materials based on constraints.

    Args:
        materials: Materials config dict from manufacturing_config.yaml.
        min_strength_mpa: Minimum yield strength (elongation_stress_mpa).
        whitelist: If provided, only these materials are considered.

    Returns:
        List of eligible material IDs.
    """
    eligible = []

    for mat_id, props in materials.items():
        # Check whitelist
        if whitelist and mat_id not in whitelist:
            continue

        # Check minimum strength
        if min_strength_mpa is not None:
            strength = props.get("elongation_stress_mpa", 0)
            if strength < min_strength_mpa:
                continue

        eligible.append(mat_id)

    return eligible


def randomize_materials(
    moving_parts: list[str],
    materials: dict[str, Any],
    seed: int,
    min_strength_mpa: float | None = None,
    whitelist: list[str] | None = None,
) -> dict[str, MaterialAssignment]:
    """Assign random materials to moving parts.

    Args:
        moving_parts: List of part names that have degrees of freedom.
        materials: Materials config dict from manufacturing_config.yaml.
        seed: Random seed for reproducibility.
        min_strength_mpa: Minimum material yield strength for filtering.
        whitelist: If provided, only these materials are considered.

    Returns:
        Mapping of part_name -> MaterialAssignment.
    """
    rng = random.Random(seed)

    eligible = get_eligible_materials(
        materials, min_strength_mpa=min_strength_mpa, whitelist=whitelist
    )

    if not eligible:
        raise ValueError("No eligible materials found with given constraints")

    assignments: dict[str, MaterialAssignment] = {}

    for part_name in moving_parts:
        mat_id = rng.choice(eligible)
        props = materials[mat_id]

        assignments[part_name] = MaterialAssignment(
            part_name=part_name,
            material_id=mat_id,
            color=props.get("color", "#FFFFFF"),
            density_g_cm3=props.get("density_g_cm3", 1.0),
            friction_coef=props.get("friction_coef", 0.5),
            restitution=props.get("restitution", 0.5),
        )

    return assignments


def apply_material_to_mjcf_geom(
    material_assignment: MaterialAssignment,
) -> dict[str, str]:
    """Generate MJCF geom attributes for a material assignment.

    Args:
        material_assignment: The material assignment to apply.

    Returns:
        Dict of MJCF geom attributes (rgba, friction, etc.).
    """
    # Convert hex color to RGBA
    color = material_assignment.color.lstrip("#")
    r = int(color[0:2], 16) / 255
    g = int(color[2:4], 16) / 255
    b = int(color[4:6], 16) / 255

    return {
        "rgba": f"{r:.3f} {g:.3f} {b:.3f} 1",
        "friction": f"{material_assignment.friction_coef} 0.005 0.0001",
        # Density is applied to body inertial, not geom
    }
