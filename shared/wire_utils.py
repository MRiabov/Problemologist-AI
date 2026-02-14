from typing import Any, List, Tuple

import numpy as np
import structlog
from pydantic import BaseModel

logger = structlog.get_logger(__name__)


class WireRouteResult(BaseModel):
    wire_id: str
    total_length_mm: float
    waypoints: List[Tuple[float, float, float]]
    valid: bool
    errors: List[str] = []


def calculate_arc_length(waypoints: List[Tuple[float, float, float]]) -> float:
    """
    Calculate length of a curved path defined by waypoints using Spline interpolation.
    """
    if not waypoints or len(waypoints) < 2:
        return 0.0

    try:
        from build123d import Spline, Vector

        # Create a spline to represent the 3D arc
        pts = [Vector(p) for p in waypoints]
        # Spline requires at least 2 points
        wire = Spline(pts)
        return float(wire.length)
    except Exception as e:
        logger.warning("spline_length_failed_falling_back_to_linear", error=str(e))
        # Fallback to linear length
        pts = np.array(waypoints)
        diffs = np.diff(pts, axis=0)
        lengths = np.linalg.norm(diffs, axis=1)
        return float(np.sum(lengths))


def check_bend_radius(waypoints: List[Tuple[float, float, float]], min_radius: float) -> List[str]:
    """
    Check if the path violates the minimum bend radius.
    Uses Menger curvature approximation between consecutive waypoints.
    """
    if len(waypoints) < 3:
        return []

    errors = []
    pts = [np.array(p) for p in waypoints]

    for i in range(1, len(pts) - 1):
        p_prev = pts[i-1]
        p_curr = pts[i]
        p_next = pts[i+1]

        a = np.linalg.norm(p_curr - p_prev)
        b = np.linalg.norm(p_next - p_curr)
        c = np.linalg.norm(p_next - p_prev)

        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            continue

        # Area of triangle using Heron's formula
        s = (a + b + c) / 2
        area_sq = s * (s - a) * (s - b) * (s - c)
        if area_sq < 0: area_sq = 0
        area = np.sqrt(area_sq)

        # Menger curvature
        curvature = (4 * area) / (a * b * c)
        radius = 1.0 / curvature if curvature > 1e-9 else float('inf')

        if radius < min_radius:
            errors.append(f"Bend radius violation at waypoint {i}: {radius:.2f}mm < {min_radius:.2f}mm")

    return errors


def route_wire(
    wire_id: str,
    waypoints: List[Tuple[float, float, float]],
    gauge_awg: int,
    attach_to: List[str] = None,
) -> WireRouteResult:
    """
    Process a wire route, calculating arc length and validating bend radius.
    """
    # T016: Use arc length instead of linear length
    length = calculate_arc_length(waypoints)
    errors = []

    if length <= 0:
        errors.append(f"Wire {wire_id} has zero or negative length.")

    # Heuristic for AWG to diameter (mm)
    diameter = 0.127 * (92 ** ((36 - gauge_awg) / 39.0))

    # Minimum bend radius: usually 4x to 6x the diameter for flexible wires
    # T016: Implement real bend radius check
    min_bend_radius = 4.0 * diameter
    bend_errors = check_bend_radius(waypoints, min_bend_radius)
    errors.extend(bend_errors)

    # Check for points being too close
    for i in range(len(waypoints) - 1):
        p1 = np.array(waypoints[i])
        p2 = np.array(waypoints[i + 1])
        if np.linalg.norm(p2 - p1) < 0.1:  # 0.1mm minimum segment
            logger.debug(f"Wire {wire_id} has very short segment at index {i}")

    return WireRouteResult(
        wire_id=wire_id,
        total_length_mm=length,
        waypoints=waypoints,
        valid=len(errors) == 0,
        errors=errors,
    )


def check_wire_clearance(
    wire_waypoints: List[Tuple[float, float, float]], assembly_meshes: Any
) -> bool:
    """
    Check if a wire path intersects with any meshes in the assembly.
    Samples the curved path (3D arcs) for accurate collision detection.
    """
    if not wire_waypoints or len(wire_waypoints) < 2:
        return True

    # 1. Interpolate path to get more samples for "arcs"
    try:
        from build123d import Spline, Vector
        pts = [Vector(p) for p in wire_waypoints]
        wire = Spline(pts)

        # Sample points along the wire
        # 1 sample per mm, minimum 10 samples
        num_samples = max(int(wire.length), 10)
        sampled_points = [wire.position_at(t) for t in np.linspace(0, 1, num_samples)]
        points_to_check = [(p.X, p.Y, p.Z) for p in sampled_points]
    except Exception as e:
        logger.warning("path_interpolation_failed", error=str(e))
        points_to_check = wire_waypoints

    # 2. Convert assembly_meshes to a list of trimesh.Trimesh
    meshes = []
    try:
        items = assembly_meshes if isinstance(assembly_meshes, list) else [assembly_meshes]
        for item in items:
            if hasattr(item, "tessellate") or hasattr(item, "solids"):  # build123d
                from worker.workbenches.analysis_utils import part_to_trimesh
                meshes.append(part_to_trimesh(item))
            elif hasattr(item, "ray"):  # already a trimesh
                meshes.append(item)
    except Exception as e:
        logger.error("mesh_prep_failed", error=str(e))
        return True

    if not meshes:
        return True

    # 3. Check each sampled segment
    for i in range(len(points_to_check) - 1):
        p1 = np.array(points_to_check[i])
        p2 = np.array(points_to_check[i + 1])
        segment_vec = p2 - p1
        segment_len = np.linalg.norm(segment_vec)
        if segment_len < 1e-6: continue
        unit_dir = segment_vec / segment_len

        for mesh in meshes:
            # Point-in-mesh check
            if hasattr(mesh, "contains") and mesh.is_watertight:
                if any(mesh.contains([p1 + unit_dir * 1e-3, p2 - unit_dir * 1e-3])):
                    logger.info("wire_inside_mesh_detected", segment_index=i)
                    return False

            # Raycast check
            if mesh.ray.intersects_any([p1 + unit_dir * 1e-4], [unit_dir]):
                locations, _, _ = mesh.ray.intersects_location([p1 + unit_dir * 1e-4], [unit_dir], multiple_hits=False)
                if len(locations) > 0:
                    hit_dist = np.linalg.norm(locations[0] - (p1 + unit_dir * 1e-4))
                    if hit_dist < (segment_len - 2e-4):
                        logger.info("wire_intersection_detected", segment_index=i)
                        return False

    return True
