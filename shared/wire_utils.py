import logging
from typing import Any

import numpy as np
from pydantic import BaseModel

logger = logging.getLogger(__name__)


class WireRouteResult(BaseModel):
    wire_id: str
    total_length_mm: float
    waypoints: list[tuple[float, float, float]]
    valid: bool
    errors: list[str] = []


def calculate_length(waypoints: list[tuple[float, float, float]]) -> float:
    """Calculate total length of a path defined by waypoints."""
    if not waypoints or len(waypoints) < 2:
        return 0.0

    length = 0.0
    pts = np.array(waypoints)
    diffs = np.diff(pts, axis=0)
    lengths = np.linalg.norm(diffs, axis=1)
    return float(np.sum(lengths))


def route_wire(
    wire_id: str,
    waypoints: list[tuple[float, float, float]],
    gauge_awg: int,
    attach_to: list[str] = None,
) -> WireRouteResult:
    """
    Process a wire route, calculating length and basic path validation.
    """
    length = calculate_length(waypoints)
    errors = []

    if length <= 0:
        errors.append(f"Wire {wire_id} has zero or negative length.")

    # Heuristic for AWG to diameter (mm)
    # d = 0.127 * 92^((36-AWG)/39)
    diameter = 0.127 * (92 ** ((36 - gauge_awg) / 39.0))

    # Minimum bend radius: usually 4x to 6x the diameter for flexible wires
    min_bend_radius = 4.0 * diameter

    # Check for points being too close (can cause simulation instability)
    for i in range(len(waypoints) - 1):
        p1 = np.array(waypoints[i])
        p2 = np.array(waypoints[i + 1])
        if np.linalg.norm(p2 - p1) < 1.0:  # 1mm minimum segment
            # Just a warning for now
            logger.debug(f"Wire {wire_id} has very short segment at index {i}")

    return WireRouteResult(
        wire_id=wire_id,
        total_length_mm=length,
        waypoints=waypoints,
        valid=len(errors) == 0,
        errors=errors,
    )


import numpy as np
from build123d import Compound, Face, Plane, Vector


def check_wire_clearance(
    wire_waypoints: list[tuple[float, float, float]],
    assembly_meshes: Compound,
    clearance_mm: float = 2.0,
) -> bool:
    """
    Check if a wire path intersects with any meshes in the assembly with a given clearance.

    Args:
        wire_waypoints: List of (x, y, z) coordinates defining the wire path.
        assembly_meshes: The build123d Compound containing the assembly geometry.
        clearance_mm: Minimum required distance from any mesh.

    Returns:
        True if the wire path is clear of intersections and maintains clearance.
        False otherwise.
    """
    if not wire_waypoints or len(wire_waypoints) < 2:
        return True

    # 1. Create a simplified 'tube' or polyline representing the wire
    # For efficiency, we'll check segments against the compound
    path_points = [Vector(p) for p in wire_waypoints]

    for i in range(len(path_points) - 1):
        p1 = path_points[i]
        p2 = path_points[j := i + 1]

        # Segment vector
        vec = p2 - p1
        length = vec.length
        if length < 1e-6:
            continue

        # Optimization: AABB check first (build123d handles this in intersect)
        # We can use build123d's distance check or intersection check.
        # For wires, we often care about DISTANCE (clearance).

        # Simplest collision: check if the line segment intersects the solid.
        # But we need clearance. So we check if the distance from segment to compound < clearance_mm.

        # build123d doesn't have a direct "distance(segment, solid)" available in all versions
        # but we can sample points or use the bounding box to prune.

        # For now, let's implement a sampling-based approach as a robust first pass
        # unless we find a better build123d primitive.

        num_samples = max(2, int(length / (clearance_mm / 2.0)))
        for s in range(num_samples):
            current_pt = p1 + vec * (s / (num_samples - 1))

            # Use build123d distance if available, otherwise fallback
            try:
                # distance() returns the shortest distance between objects
                dist = assembly_meshes.distance(current_pt)
                if dist < clearance_mm:
                    return False
            except AttributeError:
                # Fallback: check if point is inside or very close to any face/solid
                # This is slower but safer
                if assembly_meshes.is_inside(current_pt):
                    return False

    return True
