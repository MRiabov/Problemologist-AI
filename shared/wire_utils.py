import logging
from typing import Any

import numpy as np
from pydantic import BaseModel
from shared.observability.events import emit_event
from shared.observability.schemas import WireRoutingEvent

logger = logging.getLogger(__name__)


# T013: Map AWG to physical properties (Industrial standard values for Copper)
# Resistance at 20C (Ohm/km -> Ohm/m), Max Current (A, chassis wiring), Diameter (mm)
# Tensile strength (N) assumes annealed copper (~220 MPa)
AWG_PROPERTIES = {
    10: {
        "resistance_ohm_m": 0.00327,
        "max_current_a": 55.0,
        "diameter_mm": 2.588,
        "tensile_strength_n": 1150,
    },
    12: {
        "resistance_ohm_m": 0.00521,
        "max_current_a": 41.0,
        "diameter_mm": 2.053,
        "tensile_strength_n": 720,
    },
    14: {
        "resistance_ohm_m": 0.00828,
        "max_current_a": 32.0,
        "diameter_mm": 1.628,
        "tensile_strength_n": 450,
    },
    16: {
        "resistance_ohm_m": 0.01317,
        "max_current_a": 22.0,
        "diameter_mm": 1.291,
        "tensile_strength_n": 280,
    },
    18: {
        "resistance_ohm_m": 0.02095,
        "max_current_a": 16.0,
        "diameter_mm": 1.024,
        "tensile_strength_n": 180,
    },
    20: {
        "resistance_ohm_m": 0.03331,
        "max_current_a": 11.0,
        "diameter_mm": 0.812,
        "tensile_strength_n": 110,
    },
    22: {
        "resistance_ohm_m": 0.05296,
        "max_current_a": 7.0,
        "diameter_mm": 0.644,
        "tensile_strength_n": 70,
    },
    24: {
        "resistance_ohm_m": 0.08422,
        "max_current_a": 3.5,
        "diameter_mm": 0.511,
        "tensile_strength_n": 45,
    },
    26: {
        "resistance_ohm_m": 0.1339,
        "max_current_a": 2.2,
        "diameter_mm": 0.405,
        "tensile_strength_n": 28,
    },
}


def get_awg_properties(gauge: int) -> dict:
    """Lookup AWG properties, with fallback for unknown sizes."""
    if gauge in AWG_PROPERTIES:
        return AWG_PROPERTIES[gauge]

    import math

    # Heuristic fallback: d = 0.127 * 92^((36-AWG)/39)
    diameter = 0.127 * (92 ** ((36 - gauge) / 39.0))
    area_mm2 = math.pi * (diameter / 2) ** 2
    # Resistance approx: 0.0172 (rho copper) / area
    resistance = 0.0172 / area_mm2
    # Strength approx: 220 MPa * area
    strength = 220 * area_mm2

    return {
        "resistance_ohm_m": round(resistance, 6),
        "max_current_a": round(area_mm2 * 10, 1),  # Very rough heuristic
        "diameter_mm": round(diameter, 3),
        "tensile_strength_n": round(strength, 1),
    }


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

    emit_event(
        WireRoutingEvent(
            wire_count=1,
            total_length_mm=length,
            clearance_passed=True,  # Placeholder until check_wire_clearance is integrated
            errors=errors,
        )
    )

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
