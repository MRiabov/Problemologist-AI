import structlog
import numpy as np
import trimesh
from typing import Any, Optional

from build123d import Compound, Part

from shared.type_checking import type_check
from shared.workbenches.analysis_utils import (
    check_undercuts,
    compute_part_hash,
    part_to_trimesh,
)
from shared.workbenches.base import Workbench
from shared.workbenches.models import (
    CostBreakdown,
    ManufacturingConfig,
    WorkbenchResult,
)

logger = structlog.get_logger()


@type_check
def _check_draft_angles(
    mesh: trimesh.Trimesh,
    pull_vector: tuple[float, float, float] = (0.0, 0.0, 1.0),
    min_angle_deg: float = 2.0,
) -> list[str]:
    """
    Functional check for draft angles.
    A face has insufficient draft if the angle between its normal and the pull direction
    is close to 90 degrees (meaning it's parallel to the pull direction).
    """
    logger.debug("checking_draft_angles", min_angle_deg=min_angle_deg)

    pull_vector = np.array(pull_vector)
    pull_vector = pull_vector / np.linalg.norm(pull_vector)

    # Dot product of face normals with pull direction
    # cos(theta) = dot(n, pull)
    # Draft angle alpha = |90 - theta|
    # sin(alpha) = |cos(theta)|
    # For small alpha, sin(alpha) approx alpha in radians
    dots = np.abs(np.dot(mesh.face_normals, pull_vector))

    # min_angle in radians
    min_sin = np.sin(np.radians(min_angle_deg))

    # Faces with sin(alpha) < sin(min_angle) have insufficient draft
    # We exclude faces that are nearly perpendicular to pull direction (top/bottom)
    # Top/bottom faces have dots close to 1.0.
    # Side faces (draft targets) have dots close to 0.0.

    insufficient_draft = np.where(dots < min_sin)[0]

    violations = []
    if len(insufficient_draft) > 0:
        violations.append(
            f"Insufficient draft angle: {len(insufficient_draft)} faces detected with < {min_angle_deg} degrees."
        )
        logger.warning("insufficient_draft_detected", count=len(insufficient_draft))

    return violations


@type_check
def _check_wall_thickness(
    mesh: trimesh.Trimesh, min_mm: float = 1.0, max_mm: float = 4.0
) -> list[str]:
    """
    Functional check for wall thickness consistency using raycasting.
    """
    logger.debug("checking_wall_thickness", min_mm=min_mm, max_mm=max_mm)

    violations = []

    # Sample points on the mesh and cast rays along normals to find opposite wall
    # For MVP, we sample a subset of faces to keep it fast
    sample_size = min(len(mesh.faces), 1000)
    face_indices = np.random.choice(len(mesh.faces), sample_size, replace=False)

    centers = mesh.triangles_center[face_indices]
    # Inward normals
    normals = -mesh.face_normals[face_indices]

    # Offset origins slightly to avoid self-intersection
    origins = centers + normals * 1e-4

    intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    locations, index_ray, index_tri = intersector.intersects_location(
        origins, normals, multiple_hits=False
    )

    if len(locations) > 0:
        # Distance between origin and hit point
        hit_origins = origins[index_ray]
        distances = np.linalg.norm(locations - hit_origins, axis=1)

        too_thin = np.where(distances < min_mm)[0]
        too_thick = np.where(distances > max_mm)[0]

        if len(too_thin) > 0:
            violations.append(
                f"Wall thickness too thin: {len(too_thin)} samples < {min_mm}mm"
            )
            logger.warning(
                "wall_too_thin", count=len(too_thin), min_dist=float(distances.min())
            )
        if len(too_thick) > 0:
            violations.append(
                f"Wall thickness too thick: {len(too_thick)} samples > {max_mm}mm"
            )
            logger.warning(
                "wall_too_thick", count=len(too_thick), max_dist=float(distances.max())
            )

    return violations


@type_check
def _calculate_im_cost(
    part: Part | Compound,
    config: ManufacturingConfig,
    quantity: int = 1,
    context: dict[str, Any] | None = None,
) -> CostBreakdown:
    """
    Calculates IM cost: Tooling (fixed) + (Material + Cycle) * Quantity.
    """
    im_cfg = config.injection_molding
    if not im_cfg:
        raise ValueError("Injection Molding configuration missing")

    material_name = config.defaults.get("material", "abs")
    if material_name not in im_cfg.materials:
        material_name = list(im_cfg.materials.keys())[0]

    material_cfg = im_cfg.materials[material_name]
    costs_cfg = im_cfg.costs

    logger.info("calculating_im_cost", material=material_name, quantity=quantity)

    # 1. Tooling (Fixed) Cost
    surface_area_cm2 = part.area / 100.0
    tooling_cost = costs_cfg.get(
        "base_mold_cost", 5000.0
    ) + surface_area_cm2 * costs_cfg.get("mold_cost_per_surface_area_cm2", 0.5)

    # Apply reuse discount
    is_reused = False
    if context is not None:
        part_hash = compute_part_hash(part)
        if part_hash in context:
            tooling_cost *= 0.1
            is_reused = True
        context[part_hash] = context.get(part_hash, 0) + quantity

    # 2. Material Cost per Unit
    volume_cm3 = part.volume / 1000.0
    density = material_cfg.get("density_g_cm3", 1.04)
    cost_per_kg = material_cfg.get("cost_per_kg", 2.5)
    mass_kg = (volume_cm3 * density) / 1000.0
    material_cost_per_part = mass_kg * cost_per_kg

    # 3. Cycle Cost per Unit
    # Injection time
    injection_rate = costs_cfg.get("injection_rate_cm3_s", 10.0)
    injection_time_s = volume_cm3 / injection_rate

    # Cooling time proxy (based on thickness)
    bbox = part.bounding_box()
    max_dim = max(bbox.size.X, bbox.size.Y, bbox.size.Z)
    cooling_time_s = max_dim * 0.5  # Simple heuristic for MVP

    cycle_time_s = max(injection_time_s, cooling_time_s)
    machine_rate_hr = material_cfg.get("machine_hourly_rate", 60.0)
    cycle_cost_per_part = (cycle_time_s / 3600.0) * machine_rate_hr

    unit_cost = material_cost_per_part + cycle_cost_per_part
    total_cost = tooling_cost + (unit_cost * quantity)

    logger.info(
        "im_cost_calculated", tooling=tooling_cost, unit=unit_cost, total=total_cost
    )

    return CostBreakdown(
        process="injection_molding",
        total_cost=total_cost,
        unit_cost=total_cost / quantity if quantity > 0 else 0.0,
        material_cost_per_unit=round(material_cost_per_part, 4),
        setup_cost=0.0,
        is_reused=is_reused,
        details={
            "tooling_cost": round(tooling_cost, 2),
            "cycle_cost_per_unit": round(cycle_cost_per_part, 4),
            "cycle_time_s": round(cycle_time_s, 2),
            "part_volume_cm3": round(volume_cm3, 2),
        },
        pricing_explanation=(
            f"Tooling (${tooling_cost:.2f}) + "
            f"({material_name} ${material_cost_per_part:.4f} + Cycle ${cycle_cost_per_part:.4f}) * {quantity}"
        ),
    )


@type_check
def analyze_im(part: Part | Compound, config: ManufacturingConfig) -> WorkbenchResult:
    """
    Functional entry point for Injection Molding analysis.
    """
    logger.info("starting_im_analysis")

    im_cfg = config.injection_molding
    constraints = im_cfg.constraints if im_cfg else {}

    mesh = part_to_trimesh(part)
    violations = []

    # 1. Draft Angle Check
    min_draft = constraints.get("min_draft_angle_deg", 2.0)
    violations.extend(_check_draft_angles(mesh, (0.0, 0.0, 1.0), min_draft))

    # 2. Undercut Check (Moldability)
    # For IM, we check if it can be pulled from +Z or -Z
    undercuts_plus = set(check_undercuts(mesh, (0, 0, 1)))
    undercuts_minus = set(check_undercuts(mesh, (0, 0, -1)))
    real_undercuts = undercuts_plus.intersection(undercuts_minus)

    if real_undercuts:
        msg = f"IM Moldability Violation: {len(real_undercuts)} faces occluded from both directions."
        logger.warning("im_undercuts_detected", count=len(real_undercuts))
        violations.append(msg)

    # 3. Wall Thickness Check
    min_wall = constraints.get("min_wall_thickness_mm", 1.0)
    max_wall = constraints.get("max_wall_thickness_mm", 4.0)
    violations.extend(_check_wall_thickness(mesh, min_wall, max_wall))

    # 4. Cost Calculation (single unit)
    cost_breakdown = _calculate_im_cost(part, config, quantity=1)

    is_manufacturable = len(violations) == 0

    logger.info(
        "im_analysis_complete",
        is_manufacturable=is_manufacturable,
        violations=len(violations),
    )

    return WorkbenchResult(
        is_manufacturable=is_manufacturable,
        unit_cost=cost_breakdown.unit_cost,
        violations=violations,
        metadata={
            "cost_breakdown": cost_breakdown.model_dump(),
            "undercut_count": len(real_undercuts),
        },
    )


@type_check
class InjectionMoldingWorkbench(Workbench):
    """
    Injection Molding Workbench (Workbench Class wrapper).
    """

    def __init__(self, config: ManufacturingConfig | None = None):
        from shared.workbenches.config import load_config

        self.config = config or load_config()

    def validate(self, part: Part) -> list[Exception | str]:
        result = analyze_im(part, self.config)
        return result.violations

    def calculate_cost(
        self,
        part: Part,
        quantity: int = 1,
        context: dict[str, Any] | None = None,
    ) -> CostBreakdown:
        return _calculate_im_cost(part, self.config, quantity, context)
