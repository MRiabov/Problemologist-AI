import yaml
import structlog
from controller.clients.worker import WorkerClient

logger = structlog.get_logger(__name__)


async def analyze_electronics_metrics(worker: WorkerClient, session_id: str):
    """
    Analyzes the electronics output and returns metrics.
    Returns: (electrical_validity, wire_integrity, power_efficiency)
    """
    try:
        # Try to read assembly_definition.yaml which contains electronics metadata
        content = await worker.read_file("assembly_definition.yaml")
        data = yaml.safe_load(content)

        electronics = data.get("electronics")
        if not electronics:
            return 0.0, 0.0, 0.0

        # 1. Electrical Validity: Did it generate valid electronics?
        # We check if there's a power supply and at least one component
        has_psu = "power_supply" in electronics
        has_components = len(electronics.get("components", [])) > 0
        validity = 1.0 if (has_psu and has_components) else 0.5

        # 2. Wire Integrity: Are there wires and are they routed?
        wiring = electronics.get("wiring", [])
        if wiring:
            routed_count = sum(
                1 for w in wiring if w.get("routed_in_3d") or w.get("waypoints")
            )
            integrity = routed_count / len(wiring)
        else:
            integrity = 0.0

        # 3. Power Efficiency: total_draw vs max_current
        psu = electronics.get("power_supply", {})
        max_current = psu.get("max_current_a", 1.0)
        # In a real scenario we'd get this from a simulation log
        # For now, we look for estimated_draw in metadata if available, else placeholder
        draw = electronics.get("metadata", {}).get("estimated_draw_a", 0.5)

        # Optimal efficiency is often considered around 80% load for power supplies,
        # but for design we want to be within limits.
        if draw > max_current:
            efficiency = 0.0  # Overloaded
        else:
            # Score based on how well the supply is matched (not too oversized, not overloaded)
            load_factor = draw / max_current
            if 0.5 <= load_factor <= 0.9:
                efficiency = 1.0
            else:
                efficiency = 1.0 - abs(0.7 - load_factor)

        return validity, integrity, efficiency
    except Exception as e:
        logger.warning(
            "failed_to_analyze_electronics", session_id=session_id, error=str(e)
        )
        return 0.0, 0.0, 0.0
