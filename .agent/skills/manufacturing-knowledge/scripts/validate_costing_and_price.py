import sys
from pathlib import Path

import yaml
from pydantic import ValidationError

# Add project root to sys.path to import shared models
sys.path.append(str(Path(__file__).resolve().parents[4]))

from shared.enums import ManufacturingMethod
from shared.logging import get_logger
from shared.models.schemas import AssemblyDefinition

logger = get_logger(__name__)


def calculate_part_cost(part, materials_data):
    """
    Simplified cost estimation logic for preliminary planning.
    """
    method = part.manufacturing_method
    material_id = part.material_id.lower().replace("-", "_")
    material = materials_data.get("materials", {}).get(material_id, {})

    if not material:
        logger.warning(f"Material {material_id} not found, using defaults")
        density = 2.7  # g/cm3 (aluminum)
        cost_per_kg = 6.0
    else:
        density = material.get("density_g_cm3", 2.7)
        cost_per_kg = material.get("cost_per_kg", 6.0)

    # Convert volume mm3 -> cm3
    volume_cm3 = part.part_volume_mm3 / 1000.0
    mass_g = volume_cm3 * density
    material_cost = (mass_g / 1000.0) * cost_per_kg

    if method == ManufacturingMethod.CNC:
        # CNC Formula: Setup + (Material + Run) * Quantity
        setup_cost = 80.0
        stock_vol = part.stock_volume_mm3 or (part.part_volume_mm3 * 2.0)
        removed_vol_cm3 = (stock_vol - part.part_volume_mm3) / 1000.0
        run_cost = removed_vol_cm3 * 2.0
        unit_cost = (setup_cost / part.quantity) + material_cost + run_cost
        return unit_cost, mass_g

    if method == ManufacturingMethod.INJECTION_MOLDING:
        # IM Formula: Tooling + (Material + Cycle) * Quantity
        tooling_cost = 2000.0
        cycle_cost = 0.5
        unit_cost = (tooling_cost / part.quantity) + material_cost + cycle_cost
        return unit_cost, mass_g

    if method == ManufacturingMethod.THREE_DP:
        # 3DP: Material + Machine Time
        machine_cost = (part.part_volume_mm3 / 1000.0) * 0.5
        unit_cost = material_cost + machine_cost
        return unit_cost, mass_g

    return 0.0, mass_g


def generate_dfm_suggestions(part):
    """
    Generates DFM (Design for Manufacturability) suggestions for a part.
    """
    suggestions = []
    method = part.manufacturing_method

    if method == ManufacturingMethod.CNC:
        if part.stock_volume_mm3 > 0:
            removal_ratio = (
                part.stock_volume_mm3 - part.part_volume_mm3
            ) / part.stock_volume_mm3
            if removal_ratio > 0.8:
                suggestions.append(
                    f"Extremely high material removal ({removal_ratio:.1%}). "
                    "Consider using a stock size closer to the final part dimensions "
                    "to reduce machining time and waste."
                )
            elif removal_ratio < 0.1:
                suggestions.append(
                    f"Very low material removal ({removal_ratio:.1%}). "
                    "Ensure there is enough stock for clean-up passes on all surfaces."
                )

            part_to_stock_ratio = part.part_volume_mm3 / part.stock_volume_mm3
            if part_to_stock_ratio < 0.15:
                suggestions.append(
                    f"Part volume is only {part_to_stock_ratio:.1%} of stock volume. "
                    "This is inefficient; consider a smaller stock or a different manufacturing method."
                )

    elif method == ManufacturingMethod.INJECTION_MOLDING:
        if part.wall_thickness_mm is not None:
            if part.wall_thickness_mm < 0.5:
                suggestions.append(
                    f"Wall thickness ({part.wall_thickness_mm}mm) is very thin. "
                    "This may lead to short shots or structural weakness. Target 0.8-3.0mm."
                )
            elif part.wall_thickness_mm > 4.0:
                suggestions.append(
                    f"Wall thickness ({part.wall_thickness_mm}mm) is very thick. "
                    "This may cause excessive sink marks and long cooling times. Target 0.8-3.0mm."
                )

    elif method == ManufacturingMethod.THREE_DP:
        if part.part_volume_mm3 > 500000:
            suggestions.append(
                "Large part volume for 3D printing. Consider if CNC or Injection Molding "
                "would be more cost-effective for this scale."
            )

    return suggestions


def main():
    cost_file = Path("assembly_definition.yaml")
    objectives_file = Path("objectives.yaml")

    if not cost_file.exists():
        print(f"Error: {cost_file} not found.")
        sys.exit(1)

    try:
        with open(cost_file) as f:
            data = yaml.safe_load(f)

        # Load material data
        materials_data = {
            "materials": {
                "aluminum_6061": {"density_g_cm3": 2.7, "cost_per_kg": 6.0},
                "abs": {"density_g_cm3": 1.04, "cost_per_kg": 2.5},
            }
        }

        # Validate with Pydantic
        estimation = AssemblyDefinition(**data)

        total_cost = 0.0
        total_weight_g = 0.0

        # Calculate for manufactured parts
        for part in estimation.manufactured_parts:
            unit_cost, weight = calculate_part_cost(part, materials_data)
            part.estimated_unit_cost_usd = round(unit_cost, 2)
            part.dfm_suggestions = generate_dfm_suggestions(part)
            total_cost += unit_cost * part.quantity
            total_weight_g += weight * part.quantity

        # Calculate for COTS parts
        for part in estimation.cots_parts:
            total_cost += part.unit_cost_usd * part.quantity
            total_weight_g += 10.0 * part.quantity

        # Global assembly suggestions
        assembly_suggestions = []
        if total_cost > estimation.constraints.planner_target_max_unit_cost_usd:
            assembly_suggestions.append(
                f"Total cost (${total_cost:.2f}) exceeds target (${estimation.constraints.planner_target_max_unit_cost_usd:.2f}). "
                "Look for ways to consolidate parts or use cheaper manufacturing methods."
            )

        if len(estimation.manufactured_parts) > 10:
            assembly_suggestions.append(
                f"High part count ({len(estimation.manufactured_parts)} manufactured parts). "
                "Consider consolidating components to reduce assembly complexity and setup costs."
            )

        estimation.dfm_suggestions = assembly_suggestions

        # Update totals
        estimation.totals.estimated_unit_cost_usd = round(total_cost, 2)
        estimation.totals.estimated_weight_g = round(total_weight_g, 2)

        # Write back to YAML
        with open(cost_file, "w") as f:
            yaml.dump(estimation.model_dump(mode="json"), f, sort_keys=False)

        print(f"Successfully validated and updated {cost_file}")
        print(
            f"Estimated Total Unit Cost: ${estimation.totals.estimated_unit_cost_usd}"
        )
        print(f"Estimated Total Weight: {estimation.totals.estimated_weight_g}g")

        # Update objectives.yaml if it exists
        if objectives_file.exists():
            with open(objectives_file) as f:
                obj_data = yaml.safe_load(f)

            # Sync totals to objectives.yaml
            obj_data["preliminary_totals"] = {
                "estimated_unit_cost_usd": estimation.totals.estimated_unit_cost_usd,
                "estimated_weight_g": estimation.totals.estimated_weight_g,
            }

            with open(objectives_file, "w") as f:
                yaml.dump(obj_data, f, sort_keys=False)
            print(f"Updated {objectives_file} with preliminary totals.")

    except ValidationError as e:
        print(f"Validation Error in {cost_file}:")
        print(e)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
