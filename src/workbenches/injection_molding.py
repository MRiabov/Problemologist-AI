from typing import List, Union
from build123d import Part
from src.workbenches.base import Workbench
from src.workbenches.analysis_utils import (
    part_to_trimesh,
    check_draft_angle,
    check_undercuts,
    check_wall_thickness,
    load_config,
)


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

    def validate(self, part: Part) -> List[Union[Exception, str]]:
        """
        Validates the part for injection molding.
        Checks for: Draft Angle, Undercuts, and Wall Thickness.
        """
        return self.validate_geometry(part)

    def validate_geometry(self, part: Part) -> List[Union[Exception, str]]:
        """
        Analyzes part for:
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

    def calculate_cost(self, part: Part, quantity: int = 1, context: dict = None) -> float:
        """
        Calculates IM cost: Tooling (fixed) + (Material + Cycle) * Quantity.
        Tooling cost depends on surface area (complexity proxy).
        If part is reused, tooling cost is waived or heavily discounted.
        """
        material_cfg = self.materials[self.default_material]

        # 1. Tooling (Fixed) Cost
        # Base cost + surface area factor
        surface_area_cm2 = part.area / 100.0
        tooling_cost = (
            self.costs_cfg["base_mold_cost"]
            + surface_area_cm2 * self.costs_cfg["mold_cost_per_surface_area_cm2"]
        )

        # Apply reuse discount if part hash is in context
        if context is not None:
            import hashlib
            part_hash = hashlib.md5(str(part.center()).encode() + str(part.volume).encode()).hexdigest()
            if part_hash in context:
                # 90% discount on tooling for identical part reuse (reusing the mold)
                tooling_cost *= 0.1
            context[part_hash] = context.get(part_hash, 0) + quantity

        # 2. Material Cost per Unit
        volume_cm3 = part.volume / 1000.0
        mass_kg = (volume_cm3 * material_cfg["density_g_cm3"]) / 1000.0
        material_cost_per_part = mass_kg * material_cfg["cost_per_kg"]

        # 3. Cycle Cost per Unit
        # Formula: (Volume / injection_rate) * machine_rate
        injection_rate_cm3_s = self.costs_cfg.get("injection_rate_cm3_s", 10.0)
        machine_rate_s = self.costs_cfg.get("machine_hourly_rate", 60.0) / 3600.0
        
        cycle_cost_per_part = (volume_cm3 / injection_rate_cm3_s) * machine_rate_s

        unit_cost = material_cost_per_part + cycle_cost_per_part
        total_cost = tooling_cost + (unit_cost * quantity)
        return total_cost
