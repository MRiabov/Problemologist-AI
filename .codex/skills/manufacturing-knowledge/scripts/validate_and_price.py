#!/usr/bin/env python3
"""
Populate and validate the workspace assembly-definition YAML.
Calculates stock volumes and removed volumes based on planner-provided dimensions.
"""

import sys
from pathlib import Path

import structlog
import yaml
from pydantic import ValidationError

# Add project root to path to import shared models
sys.path.append(str(Path(__file__).parents[3]))
from shared.cots.runtime import get_catalog_item_with_metadata
from shared.enums import ManufacturingMethod
from shared.models.schemas import AssemblyDefinition
from worker_heavy.utils.dfm import (
    calculate_declared_assembly_cost,
    calculate_declared_assembly_weight,
    load_planner_manufacturing_config,
)

logger = structlog.get_logger(__name__)


def main():
    file_path = Path("assembly_definition.yaml")
    if not file_path.exists():
        benchmark_path = Path("benchmark_assembly_definition.yaml")
        file_path = benchmark_path if benchmark_path.exists() else file_path
    if not file_path.exists():
        print(
            "Error: assembly_definition.yaml or benchmark_assembly_definition.yaml not found."
        )
        sys.exit(1)

    try:
        content = file_path.read_text(encoding="utf-8")
        data = yaml.safe_load(content)
        if not data:
            print("Error: Empty YAML file.")
            sys.exit(1)

        # 1. Geometry Calculations
        for part in data.get("manufactured_parts", []):
            bbox = part.get("stock_bbox_mm")
            if bbox and isinstance(bbox, dict):
                x = float(bbox.get("x", 0))
                y = float(bbox.get("y", 0))
                z = float(bbox.get("z", 0))

                # Calculate stock volume
                stock_vol = x * y * z
                part["stock_volume_mm3"] = round(stock_vol, 2)

                # Calculate removed volume for CNC
                m_method = part.get("manufacturing_method")
                if m_method == ManufacturingMethod.CNC or m_method == "CNC":
                    part_vol = float(part.get("part_volume_mm3", 0))
                    part["removed_volume_mm3"] = round(
                        max(0.0, stock_vol - part_vol), 2
                    )
                else:
                    part["removed_volume_mm3"] = 0.0

        # 1b. Normalize COTS entries using the exact catalog snapshot.
        for part in data.get("cots_parts", []):
            part_id = part.get("part_id")
            if not part_id:
                print("Error: cots_parts entry is missing part_id.")
                sys.exit(1)

            lookup = get_catalog_item_with_metadata(str(part_id))
            if lookup is None:
                print(
                    f"Error: cots_parts entry '{part_id}' does not resolve to a catalog item."
                )
                sys.exit(1)

            catalog_item, catalog_metadata = lookup
            catalog_details = catalog_item.metadata or {}

            manufacturer = catalog_details.get("manufacturer", "unknown")
            if part.get("manufacturer") not in (None, "", manufacturer):
                print(
                    f"Error: cots_parts entry '{part_id}' manufacturer must match catalog manufacturer '{manufacturer}'."
                )
                sys.exit(1)
            part["manufacturer"] = manufacturer

            unit_cost = float(catalog_item.unit_cost)
            if (
                part.get("unit_cost_usd") is not None
                and abs(float(part["unit_cost_usd"]) - unit_cost) > 1e-6
            ):
                print(
                    f"Error: cots_parts entry '{part_id}' unit_cost_usd must match catalog unit cost {unit_cost}."
                )
                sys.exit(1)
            part["unit_cost_usd"] = round(unit_cost, 6)

            weight_g = float(catalog_item.weight_g)
            if (
                part.get("weight_g") is not None
                and abs(float(part["weight_g"]) - weight_g) > 1e-6
            ):
                print(
                    f"Error: cots_parts entry '{part_id}' weight_g must match catalog weight {weight_g}."
                )
                sys.exit(1)
            part["weight_g"] = round(weight_g, 6)

            part["catalog_version"] = catalog_metadata.get("catalog_version")
            part["bd_warehouse_commit"] = catalog_metadata.get("bd_warehouse_commit")
            part["catalog_snapshot_id"] = catalog_metadata.get("catalog_snapshot_id")
            part["generated_at"] = catalog_metadata.get("generated_at")

            if not part.get("source"):
                part["source"] = "catalog"

        # 2. Pydantic Validation
        estimation = AssemblyDefinition(**data)

        config_path = Path("manufacturing_config.yaml")
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        config = load_planner_manufacturing_config(config_path=config_path)
        data["totals"]["estimated_unit_cost_usd"] = calculate_declared_assembly_cost(
            estimation, config
        )
        data["totals"]["estimated_weight_g"] = calculate_declared_assembly_weight(
            estimation, config
        )
        estimation = AssemblyDefinition(**data)

        # 3. Write back normalized YAML
        updated_content = yaml.dump(data, sort_keys=False, default_flow_style=False)
        file_path.write_text(updated_content, encoding="utf-8")

        # 4. Success Output
        print(f"Success: {file_path.name} validated and updated.")
        print(f"Total Estimated Cost: ${estimation.totals.estimated_unit_cost_usd:.2f}")
        print(f"Total Estimated Weight: {estimation.totals.estimated_weight_g:.2f}g")

    except ValidationError as e:
        print(f"Validation Error: {e}")
        sys.exit(1)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: manufacturing_config.yaml invalid: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
