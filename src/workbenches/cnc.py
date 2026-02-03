import hashlib
from typing import Any

from build123d import Part

from src.workbenches.analysis_utils import check_undercuts, load_config, part_to_trimesh
from src.workbenches.base import Workbench


class CNCWorkbench(Workbench):
    """
    CNC Milling Workbench for 3-axis machining.
    """

    def __init__(self):
        self.config = load_config()["cnc"]
        self.materials = self.config["materials"]
        # Default to aluminum_6061 for now
        self.default_material = "aluminum_6061"

    def validate(self, part: Part) -> list[Exception | str]:
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
        self, part: Part, quantity: int = 1, context: dict | None = None
    ) -> dict[str, Any]:
        """
        Calculates CNC cost: Setup + (Material + Run) * Quantity.
        If part is reused (found in context), setup cost is discounted.
        Returns detailed breakdown.
        """
        material_cfg = self.materials[self.default_material]

        # 0. Stock / Material Analysis
        bbox = part.bounding_box()
        # Add slight padding for stock (standard practice, but keeping simple for now)
        stock_dims = (bbox.size.X, bbox.size.Y, bbox.size.Z)
        stock_volume_cm3 = (stock_dims[0] * stock_dims[1] * stock_dims[2]) / 1000.0
        part_volume_cm3 = part.volume / 1000.0
        removed_volume_cm3 = stock_volume_cm3 - part_volume_cm3

        # 1. Material Cost (based on Stock, usually, but spec says bounding box volume)
        stock_mass_kg = (stock_volume_cm3 * material_cfg["density_g_cm3"]) / 1000.0
        material_cost_per_part = stock_mass_kg * material_cfg["cost_per_kg"]

        # 2. Run Cost
        # Assume material removal rate (MRR) is 1000 mm3/min for aluminum
        mrr = 1000.0
        # Machining time driven effectively by removed volume + some finishing for surface area
        # Simplified: just volume based like before, but let's use removed_volume if feasible?
        # The previous implementation used `part.volume / mrr`.
        # Realistically, it's removed volume. But to stay consistent with previous calibration,
        # checking spec: "Calculate based on material volume removal rate".
        # So yes, removed volume is better.
        machining_time_min = (removed_volume_cm3 * 1000.0) / mrr
        # Add basic finishing pass estimation (surface area based)?
        # For simplicity, let's stick to simple MRR model but use removed volume + overhead.
        # Actually, let's keep it close to previous behavior but use removed volume which is more physically accurate for milling.
        # Warning: if removed volume is huge (block -> thin shell), cost skyrockets. This is correct.

        run_cost_per_part = (machining_time_min / 60.0) * material_cfg[
            "machine_hourly_rate"
        ]

        # 3. Setup Cost
        # Assume 1 hour setup for simple 3-axis job
        setup_cost = material_cfg["machine_hourly_rate"]

        # Apply reuse discount if part hash is in context
        is_reused = False
        if context is not None:
            part_hash = hashlib.md5(
                str(part.center()).encode() + str(part.volume).encode()
            ).hexdigest()
            if part_hash in context:
                # 50% discount on setup for identical part reuse (e.g. same CAM program)
                setup_cost *= 0.5
                is_reused = True
            context[part_hash] = context.get(part_hash, 0) + quantity

        total_cost = (
            setup_cost + (material_cost_per_part + run_cost_per_part) * quantity
        )

        return {
            "total_cost": total_cost,
            "unit_cost": total_cost / quantity if quantity > 0 else 0.0,
            "breakdown": {
                "process": "cnc_milling",
                "material_name": self.default_material,
                "stock_dims_mm": [round(d, 2) for d in stock_dims],
                "stock_volume_cm3": round(stock_volume_cm3, 2),
                "part_volume_cm3": round(part_volume_cm3, 2),
                "removed_volume_cm3": round(removed_volume_cm3, 2),
                "machining_time_min": round(machining_time_min, 2),
                "setup_cost": round(setup_cost, 2),
                "material_cost_per_unit": round(material_cost_per_part, 2),
                "run_cost_per_unit": round(run_cost_per_part, 2),
                "is_reused": is_reused,
                "pricing_explanation": (
                    f"Cost is driven by material volume and machining time. "
                    f"Stock size ([{', '.join(f'{d:.1f}' for d in stock_dims)}]) determines material cost. "
                    f"Removed volume ({removed_volume_cm3:.2f} cm3) determines machining time ({machining_time_min:.2f} min). "
                    f"Setup cost (${setup_cost:.2f}) is fixed per part design (discounted if reused)."
                ),
            },
        }
