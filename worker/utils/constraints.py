"""
Constraint validation utilities for simulation assemblies.

Per architecture spec Item 9: Realistic constraint validation ensures that
constrained parts are physically close enough to be validly connected.
"""

from build123d import Compound, Part

# Maximum distance (in mm) between parts for a valid constraint
MAX_CONSTRAINT_DISTANCE_MM = 5.0


def validate_constraint_proximity(
    part1: Part | Compound,
    part2: Part | Compound,
    max_distance_mm: float = MAX_CONSTRAINT_DISTANCE_MM,
) -> tuple[bool, str]:
    """
    Validate that two parts are close enough to be constrained together.

    Per architecture spec: Two parts must be physically close to be constrained.
    This prevents invalid assemblies where parts are connected across large gaps.

    Args:
        part1: First part in the constraint
        part2: Second part in the constraint
        max_distance_mm: Maximum allowed distance between bounding boxes (default 5mm)

    Returns:
        (True, "") if parts are close enough
        (False, error_message) if parts are too far apart
    """
    bb1 = part1.bounding_box()
    bb2 = part2.bounding_box()

    # Calculate minimum distance between bounding boxes
    # This is a conservative check - actual surfaces may be even further apart
    dx = max(0, max(bb1.min.X - bb2.max.X, bb2.min.X - bb1.max.X))
    dy = max(0, max(bb1.min.Y - bb2.max.Y, bb2.min.Y - bb1.max.Y))
    dz = max(0, max(bb1.min.Z - bb2.max.Z, bb2.min.Z - bb1.max.Z))

    min_distance = (dx**2 + dy**2 + dz**2) ** 0.5

    if min_distance > max_distance_mm:
        label1 = getattr(part1, "label", "part1")
        label2 = getattr(part2, "label", "part2")
        return False, (
            f"Parts '{label1}' and '{label2}' are too far apart to constrain "
            f"({min_distance:.2f}mm > {max_distance_mm}mm max)"
        )

    return True, ""


def validate_all_constraints(assembly: Compound) -> list[str]:
    """
    Validate all constraint relationships in an assembly.

    Currently, this checks that adjacent parts (by index) are close enough.
    A more sophisticated implementation would track explicit constraint metadata.

    Args:
        assembly: The compound assembly to validate

    Returns:
        List of constraint violation messages (empty if all valid)
    """
    violations = []
    children = list(assembly.children)

    # Skip zone parts (they don't need proximity constraints)
    physical_parts = [
        c for c in children if not getattr(c, "label", "").startswith("zone_")
    ]

    # For now, just validate that all physical parts are reasonably close
    # A more advanced implementation would track explicit joint/constraint metadata
    for i, part1 in enumerate(physical_parts):
        for part2 in physical_parts[i + 1 :]:
            is_valid, error = validate_constraint_proximity(part1, part2)
            if not is_valid:
                violations.append(error)

    return violations
