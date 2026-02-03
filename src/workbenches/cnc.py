import hashlib
from typing import List, Union

from build123d import Part

from src.workbenches.base import Workbench
from src.workbenches.analysis_utils import part_to_trimesh, check_undercuts, load_config


class CNCWorkbench(Workbench):
    """
    CNC Milling Workbench for 3-axis machining.
    """

    def __init__(self):
        self.config = load_config()["cnc"]
        self.materials = self.config["materials"]
        # Default to aluminum_6061 for now
        self.default_material = "aluminum_6061"

    def validate(self, part: Part) -> List[Union[Exception, str]]:
        """
        Validates the part for 3-axis milling.
        Primary check is for undercuts from the Z-axis.
        """
        mesh = part_to_trimesh(part)
        violations = []

        # Check for undercuts (assuming +Z axis pull/approach)
        undercuts = check_undercuts(mesh, (0, 0, 1))
        if undercuts:
            violations.append(
                f"CNC Machining Violation: {len(undercuts)} undercut faces detected. "
                "3-axis milling cannot reach occluded geometry from the Z-axis."
            )

        return violations

    def calculate_cost(
        self, part: Part, quantity: int = 1, context: dict = None
    ) -> float:
        """
        Calculates CNC cost: Setup + (Material + Run) * Quantity.
        If part is reused (found in context), setup cost is discounted.
        """
        material_cfg = self.materials[self.default_material]

        # 1. Material Cost
        # Density is in g/cm3, volume is in mm3
        volume_cm3 = part.volume / 1000.0
        mass_kg = (volume_cm3 * material_cfg["density_g_cm3"]) / 1000.0
        material_cost_per_part = mass_kg * material_cfg["cost_per_kg"]

        # 2. Run Cost (Simplified estimation)
        # Assume material removal rate (MRR) is 1000 mm3/min for aluminum
        mrr = 1000.0
        machining_time_min = part.volume / mrr
        run_cost_per_part = (machining_time_min / 60.0) * material_cfg[
            "machine_hourly_rate"
        ]

        # 3. Setup Cost
        # Assume 1 hour setup for simple 3-axis job
        setup_cost = material_cfg["machine_hourly_rate"]

        # Apply reuse discount if part hash is in context
        if context is not None:
            part_hash = hashlib.md5(
                str(part.center()).encode() + str(part.volume).encode()
            ).hexdigest()
            if part_hash in context:
                # 50% discount on setup for identical part reuse (e.g. same CAM program)
                setup_cost *= 0.5
            context[part_hash] = context.get(part_hash, 0) + quantity

        total_cost = (
            setup_cost + (material_cost_per_part + run_cost_per_part) * quantity
        )
        return total_cost
