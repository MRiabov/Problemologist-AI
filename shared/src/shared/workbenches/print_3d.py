import hashlib
from typing import Any

from build123d import Part

from shared.type_checking import type_check
from shared.workbenches.base import Workbench
from shared.workbenches.models import CostBreakdown


@type_check
class Print3DWorkbench(Workbench):
    """
    Workbench for 3D Printing (FDM/SLA).
    Enforces manifold geometry and single-body parts.
    Cost is proportional to the volume of the part.
    """

    def __init__(self, material_cost: float = 0.05):
        """
        Initializes the 3D Print Workbench.

        Args:
            material_cost: Cost per unit volume.
        """
        self.material_cost = material_cost

    def validate(self, part: Part) -> list[Exception | str]:
        """
        Validates the part for 3D printing.
        Checks for:
        - Geometric validity (manifold, no self-intersections).
        - Closed geometry (watertight).
        - Single body (exactly one solid).
        """
        violations = []

        # Check geometric validity via OpenCASCADE
        if not part.is_valid:
            violations.append(
                "Geometry is not valid (non-manifold or self-intersecting)"
            )

        # Check if geometry is closed/watertight
        # In build123d, we can check if it's a solid or if it has any open shells.
        # For simplicity in MVP, we check if it has solids and if those solids are closed.
        solids = part.solids()
        if not solids:
            violations.append("Geometry contains no solids")
        else:
            for i, solid in enumerate(solids):
                # Shape.is_closed check
                if hasattr(solid, "is_closed") and not solid.is_closed:
                    violations.append(f"Solid {i} is not closed (not watertight)")

        # Enforce single body
        if len(solids) > 1:
            violations.append(
                f"Geometry must be a single body, found {len(solids)} solids"
            )

        return violations

    def calculate_cost(
        self,
        part: Part,
        quantity: int = 1,
        context: dict[str, Any] | None = None,
    ) -> CostBreakdown:
        """
        Calculates cost based on part volume and quantity.
        """
        is_reused = False
        if context is not None:
            part_hash = hashlib.md5(
                str(part.center()).encode() + str(part.volume).encode()
            ).hexdigest()
            if part_hash in context:
                is_reused = True
            context[part_hash] = context.get(part_hash, 0) + quantity

        unit_cost = part.volume * self.material_cost
        total_cost = unit_cost * quantity

        return CostBreakdown(
            process="print_3d",
            total_cost=total_cost,
            unit_cost=unit_cost,
            material_cost_per_unit=unit_cost,
            setup_cost=0.0,
            is_reused=is_reused,
            details={
                "part_volume": part.volume,
                "material_cost_rate": self.material_cost,
            },
            pricing_explanation=f"Cost is purely volume-based (${self.material_cost}/mm3).",
        )
