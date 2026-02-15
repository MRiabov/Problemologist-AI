import logging
import math
from typing import Any

import numpy as np
from build123d import Compound, Polyline, Spline, Vector
from pydantic import BaseModel
from shared.observability.events import emit_event
from shared.observability.schemas import WireRoutingEvent

from shared.models.schemas import WireConfig, WireTerminal

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


class WireRouteResult(BaseModel):
    """Legacy result model for wire routing."""

    wire_id: str
    total_length_mm: float
    waypoints: list[tuple[float, float, float]]
    valid: bool
    errors: list[str] = []


def get_awg_properties(gauge: int) -> dict:
    """Lookup AWG properties, with fallback for unknown sizes."""
    if gauge in AWG_PROPERTIES:
        return AWG_PROPERTIES[gauge]

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


# T010: Generate 3D paths from waypoints (Splines)
def generate_spline_points(
    waypoints: list[tuple[float, float, float]], samples_per_mm: float = 0.5
) -> list[tuple[float, float, float]]:
    """
    Generate a smooth spline path through waypoints and sample it.

    Args:
        waypoints: Control points for the spline.
        samples_per_mm: Density of sampling along the path.
    """
    if len(waypoints) < 2:
        return waypoints

    # Use build123d Spline
    path_points = [Vector(p) for p in waypoints]

    try:
        # Spline requires at least 2 points
        spline = Spline(path_points)
        length = spline.length
        num_samples = max(2, int(length * samples_per_mm))

        sampled_pts = []
        for i in range(num_samples):
            # t goes from 0 to 1
            t = i / (num_samples - 1)
            pt = spline.position_at(t)
            sampled_pts.append((float(pt.X), float(pt.Y), float(pt.Z)))
        return sampled_pts
    except Exception as e:
        logger.warning(f"Failed to generate spline, falling back to polyline: {e}")
        return waypoints


def calculate_path_length(
    waypoints: list[tuple[float, float, float]], use_spline: bool = True
) -> float:
    """Calculate the length of a path defined by waypoints."""
    if not waypoints or len(waypoints) < 2:
        return 0.0

    path_points = [Vector(p) for p in waypoints]
    try:
        if use_spline and len(waypoints) >= 2:
            return float(Spline(path_points).length)
        else:
            return float(Polyline(path_points).length)
    except Exception:
        # Fallback to manual distance sum
        length = 0.0
        for i in range(len(waypoints) - 1):
            p1 = np.array(waypoints[i])
            p2 = np.array(waypoints[i + 1])
            length += np.linalg.norm(p2 - p1)
        return float(length)


def calculate_length(waypoints: list[tuple[float, float, float]]) -> float:
    """Legacy length calculation (uses manual distance sum)."""
    if not waypoints or len(waypoints) < 2:
        return 0.0

    length = 0.0
    pts = np.array(waypoints)
    diffs = np.diff(pts, axis=0)
    lengths = np.linalg.norm(diffs, axis=1)
    return float(np.sum(lengths))


# T011: Implement check_wire_clearance
def check_wire_clearance(
    wire_waypoints: list[tuple[float, float, float]],
    assembly_meshes: Compound,
    clearance_mm: float = 2.0,
    ignore_endpoints_dist: float = 5.0,
) -> bool:
    """
    Check if a wire path intersects with any meshes in the assembly.

    Args:
        wire_waypoints: Path points.
        assembly_meshes: Geometry to check against.
        clearance_mm: Minimum required distance.
        ignore_endpoints_dist: Distance from start/end where intersections are ignored (attachment points).
    """
    if not wire_waypoints or len(wire_waypoints) < 2:
        return True

    # Generate a smooth spline for better collision checking
    path_points = [
        Vector(p) for p in generate_spline_points(wire_waypoints, samples_per_mm=1.0)
    ]

    # build123d Polyline from sampled points is a good proxy for the spline
    path_proxy = Polyline(path_points)

    try:
        # Check distance between the path and the assembly
        dist = assembly_meshes.distance(path_proxy)
        if dist >= clearance_mm:
            return True

        # If distance is low, check where it happens.
        # If it's only near the endpoints (attachment), we might allow it.
        # But distance() doesn't tell us WHERE.
        # So we sample and check individual points.
        start_pt = path_points[0]
        end_pt = path_points[-1]

        for pt in path_points:
            d_start = (pt - start_pt).length
            d_end = (pt - end_pt).length

            if d_start < ignore_endpoints_dist or d_end < ignore_endpoints_dist:
                continue

            # For points away from terminals, check distance
            # This is slow if done for every point, but distance(point, compound) is usually fast
            d = assembly_meshes.distance(pt)
            if d < clearance_mm:
                logger.info(f"Wire clearance violation: {d:.2f}mm at {pt}")
                return False

        return True
    except Exception as e:
        logger.error(f"Error in check_wire_clearance: {e}")
        return False


# T012: Implement route_wire helper
def route_wire(
    wire_id: str,
    from_comp: str,
    from_term: str,
    to_comp: str,
    to_term: str,
    gauge_awg: int,
    waypoints: list[tuple[float, float, float]] | None = None,
    routed_in_3d: bool = False,
) -> WireConfig:
    """
    Helper to define a wire connection and calculate its properties.
    """
    waypoints = waypoints or []

    # Calculate length
    length = calculate_path_length(waypoints, use_spline=routed_in_3d)

    # If no waypoints, it's a "logical" connection with zero length (will need manual override)
    # or it might be a direct line.
    if not waypoints and not routed_in_3d:
        # Logical connections are allowed but might need physical routing later
        pass

    emit_event(
        WireRoutingEvent(
            wire_count=1,
            total_length_mm=length,
            clearance_passed=True,  # Placeholder until check_wire_clearance is integrated in tool
            errors=[],
        )
    )

    return WireConfig(
        wire_id=wire_id,
        from_terminal=WireTerminal(component=from_comp, terminal=from_term),
        to_terminal=WireTerminal(component=to_comp, terminal=to_term),
        gauge_awg=gauge_awg,
        length_mm=round(length, 2),
        waypoints=waypoints,
        routed_in_3d=routed_in_3d,
    )


def route_wire_legacy(
    wire_id: str,
    waypoints: list[tuple[float, float, float]],
    gauge_awg: int,
    attach_to: list[str] = None,
) -> WireRouteResult:
    """
    Process a wire route (legacy signature), calculating length and basic path validation.
    """
    length = calculate_length(waypoints)
    errors = []

    if length <= 0:
        errors.append(f"Wire {wire_id} has zero or negative length.")

    # Heuristic for AWG to diameter (mm)
    diameter = 0.127 * (92 ** ((36 - gauge_awg) / 39.0))

    # Check for points being too close (can cause simulation instability)
    for i in range(len(waypoints) - 1):
        p1 = np.array(waypoints[i])
        p2 = np.array(waypoints[i + 1])
        if np.linalg.norm(p2 - p1) < 1.0:  # 1mm minimum segment
            logger.debug(f"Wire {wire_id} has very short segment at index {i}")

    emit_event(
        WireRoutingEvent(
            wire_count=1,
            total_length_mm=length,
            clearance_passed=True,
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
