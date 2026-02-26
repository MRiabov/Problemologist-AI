from typing import Any

import structlog

from shared.enums import ElectronicComponentType
from shared.models.schemas import ElectronicsSection
from shared.wire_utils import get_awg_properties
from shared.workers.workbench_models import ManufacturingConfig

logger = structlog.get_logger(__name__)


def calculate_electronics_cost_and_weight(
    electronics: ElectronicsSection,
    config: ManufacturingConfig,
) -> tuple[float, float]:
    """
    Calculates the total cost and weight of the electronics section.

    Args:
        electronics: The electronics section of the assembly definition.
        config: Manufacturing configuration containing wire costs.

    Returns:
        tuple[float, float]: (total_cost_usd, total_weight_g)
    """
    total_cost = 0.0
    total_weight = 0.0

    # 1. Components
    for comp in electronics.components:
        cost = 0.0
        weight = 0.0

        # Try to price components based on their COTS ID if available
        if comp.cots_part_id:
            try:
                if comp.type == ElectronicComponentType.POWER_SUPPLY:
                    from shared.cots.parts.electronics import PowerSupply
                    part = PowerSupply(size=comp.cots_part_id)
                    cost = getattr(part, "price", 0.0)
                    weight = getattr(part, "weight_g", 0.0)

                elif comp.type == ElectronicComponentType.RELAY:
                    from shared.cots.parts.electronics import ElectronicRelay
                    part = ElectronicRelay(size=comp.cots_part_id)
                    cost = getattr(part, "price", 0.0)
                    weight = getattr(part, "weight_g", 0.0)

                elif comp.type == ElectronicComponentType.SWITCH:
                    from shared.cots.parts.electronics import Switch
                    part = Switch(size=comp.cots_part_id)
                    cost = getattr(part, "price", 0.0)
                    weight = getattr(part, "weight_g", 0.0)

                elif comp.type == ElectronicComponentType.CONNECTOR:
                    from shared.cots.parts.electronics import Connector
                    part = Connector(size=comp.cots_part_id)
                    cost = getattr(part, "price", 0.0)
                    weight = getattr(part, "weight_g", 0.0)

                elif comp.type == ElectronicComponentType.MOTOR:
                    from shared.cots.parts.motors import ServoMotor
                    part = ServoMotor(size=comp.cots_part_id)
                    cost = getattr(part, "price", 0.0)
                    weight = getattr(part, "weight_g", 0.0)

            except Exception as e:
                logger.error(
                    "failed_to_price_component",
                    cots_id=comp.cots_part_id,
                    type=comp.type,
                    error=str(e),
                )

        total_cost += cost
        total_weight += weight

    # 2. Wires
    for wire in electronics.wiring:
        length_m = wire.length_mm / 1000.0
        props = get_awg_properties(wire.gauge_awg)

        # Estimate weight based on copper density and diameter
        # Area (mm2) = pi * (d/2)^2
        import math

        area_mm2 = math.pi * (props["diameter_mm"] / 2.0) ** 2
        # Weight (g/m) = Area (mm2) * Density (8.96 g/cm3)
        # 1 mm2 * 1 m = 1000 mm3 = 1 cm3
        weight_g_m = area_mm2 * 8.96

        # Use cost from config if available, otherwise fallback to reasonable default
        cost_per_m = 0.5  # default

        if config.wires:
            awg_key = f"awg{wire.gauge_awg}"
            # Check if key exists in config.wires (which is a dict of WireDefinition)
            if hasattr(config.wires, "get"):
                wire_def = config.wires.get(awg_key)
                if wire_def:
                    cost_per_m = wire_def.cost_per_m
            # Handle potential dict access if it's not a Pydantic model (though it should be)
            elif isinstance(config.wires, dict) and awg_key in config.wires:
                 # Check if the value is an object or dict
                val = config.wires[awg_key]
                if hasattr(val, "cost_per_m"):
                    cost_per_m = val.cost_per_m
                elif isinstance(val, dict):
                    cost_per_m = val.get("cost_per_m", 0.5)

        total_cost += length_m * cost_per_m
        total_weight += length_m * weight_g_m

    return total_cost, total_weight
