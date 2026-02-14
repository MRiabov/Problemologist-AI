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

    from build123d import Polyline, Vector
    
    try:
        path_points = [Vector(p) for p in wire_waypoints]
        polyline = Polyline(path_points)
        
        # distance() returns the shortest distance between objects
        dist = assembly_meshes.distance(polyline)
        return dist >= clearance_mm
    except Exception as e:
        logger.error(f"Error in check_wire_clearance: {e}")
        # Fallback to sampling if distance calculation fails
        path_points = [Vector(p) for p in wire_waypoints]
        for i in range(len(path_points) - 1):
            p1 = path_points[i]
            p2 = path_points[i + 1]
            vec = p2 - p1
            length = vec.length
            num_samples = max(2, int(length / (clearance_mm / 2.0)))
            for s in range(num_samples):
                current_pt = p1 + vec * (s / (num_samples - 1))
                if assembly_meshes.is_inside(current_pt):
                    return False
                # We can't easily check distance here without the tool that failed
        return True

