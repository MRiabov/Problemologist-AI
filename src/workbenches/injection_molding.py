import hashlib
from typing import Any

from build123d import Part

from src.workbenches.analysis_utils import (
    analyze_wall_thickness,
    check_draft_angle,
    check_undercuts,
    check_wall_thickness,
    load_config,
    part_to_trimesh,
)
from src.workbenches.base import Workbench


class InjectionMoldingWorkbench(Workbench):
    """
    Injection Molding Workbench for high-volume plastic production.
    """

    def __init__(self):
        self.config = load_config()["injection_molding"]
        self.materials = self.config["materials"]
        self.constraints = self.config["constraints"]
        self.costs_cfg = self.config["costs"]
        # Default to ABS for now
        self.default_material = "abs"

    def validate(self, part: Part) -> list[Exception | str]:
        """
        Validates the part for injection molding.
        Checks for:
        1. Draft (min_draft_angle_deg)
        2. Undercuts (raycasting in pull direction)
        3. Wall thickness (sampled min/max range)
        """
        mesh = part_to_trimesh(part)
        violations = []

        # 1. Draft Angle Analysis
        draft_violations = check_draft_angle(
            mesh,
            pull_vector=(0, 0, 1),
            min_angle_deg=self.config["constraints"]["min_draft_angle_deg"],
        )
        if draft_violations:
            violations.append(
                f"IM Draft Violation: {len(draft_violations)} faces have insufficient draft angle "
                f"(<{self.config['constraints']['min_draft_angle_deg']}°)."
            )

        # 2. Undercut Detection
        undercuts = check_undercuts(mesh, (0, 0, 1))
        if undercuts:
            violations.append(
                f"IM Undercut Violation: {len(undercuts)} undercut faces detected. "
                "Simple 2-part mold cannot eject geometry trapped in the pull direction."
            )

        # 3. Wall Thickness Analysis
        thickness_violations = check_wall_thickness(
            mesh,
            min_mm=self.config["constraints"]["min_wall_thickness_mm"],
            max_mm=self.config["constraints"]["max_wall_thickness_mm"],
        )
        if thickness_violations:
            violations.append(
                f"IM Wall Thickness Violation: {len(thickness_violations)} faces outside "
                f"[{self.config['constraints']['min_wall_thickness_mm']}, {self.config['constraints']['max_wall_thickness_mm']}] mm range."
            )

        return violations

    def calculate_cost(
        self, part: Part, quantity: int = 1, context: dict | None = None
    ) -> dict[str, Any]:
        """
        Calculates IM cost: Tooling (fixed) + (Material + Cycle) * Quantity.
        Tooling cost depends on surface area (complexity proxy).
        If part is reused, tooling cost is waived or heavily discounted.
        Returns detailed breakdown.
        """
        material_cfg = self.materials[self.default_material]

        # 0. Geometric Analysis (Needed for pricing metrics)
        mesh = part_to_trimesh(part)
        thickness_stats = analyze_wall_thickness(mesh)

        # 1. Tooling (Fixed) Cost
        # Base cost + surface area factor
        surface_area_cm2 = part.area / 100.0
        tooling_cost = (
            self.costs_cfg["base_mold_cost"]
            + surface_area_cm2 * self.costs_cfg["mold_cost_per_surface_area_cm2"]
        )

        # Apply reuse discount if part hash is in context
        is_reused = False
        if context is not None:
            part_hash = hashlib.md5(
                str(part.center()).encode() + str(part.volume).encode()
            ).hexdigest()
            if part_hash in context:
                # 90% discount on tooling for identical part reuse (reusing the mold)
                tooling_cost *= 0.1
                is_reused = True
            context[part_hash] = context.get(part_hash, 0) + quantity

        # 2. Material Cost per Unit
        volume_cm3 = part.volume / 1000.0
        mass_kg = (volume_cm3 * material_cfg["density_g_cm3"]) / 1000.0
        material_cost_per_part = mass_kg * material_cfg["cost_per_kg"]

        # 3. Cycle Cost per Unit
        # Formula: (Volume / injection_rate) * machine_rate
        # Refined: Cycle time is often limited by cooling time, which is proportional to thickness^2.
        # But stick to spec formula for now or simple enhancements?
        # User requested "how we determine the pricing".
        # Let's add cooling time factor based on max thickness if meaningful?
        # For now, keep simple volume rate but REPORT thickness so agent sees it.

        injection_rate_cm3_s = self.costs_cfg.get("injection_rate_cm3_s", 10.0)
        machine_rate_s = self.costs_cfg.get("machine_hourly_rate", 60.0) / 3600.0

        cycle_cost_per_part = (volume_cm3 / injection_rate_cm3_s) * machine_rate_s

        unit_cost = material_cost_per_part + cycle_cost_per_part
        total_cost = tooling_cost + (unit_cost * quantity)

        return {
            "total_cost": total_cost,
            "unit_cost": total_cost / quantity if quantity > 0 else 0.0,
            "breakdown": {
                "process": "injection_molding",
                "material_name": self.default_material,
                "tooling_cost": round(tooling_cost, 2),
                "is_reused": is_reused,
                "material_cost_per_unit": round(material_cost_per_part, 4),
                "cycle_cost_per_unit": round(cycle_cost_per_part, 4),
                "part_volume_cm3": round(volume_cm3, 2),
                "surface_area_cm2": round(surface_area_cm2, 2),
                "wall_thickness_stats": {
                    "min_mm": round(thickness_stats["min_mm"], 2),
                    "max_mm": round(thickness_stats["max_mm"], 2),
                    "average_mm": round(thickness_stats["average_mm"], 2),
                },
            },
        }
