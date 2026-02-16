import yaml
import structlog
from shared.models.schemas import AssemblyDefinition
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
        assembly = AssemblyDefinition.model_validate(data)

        electronics = assembly.electronics
        if not electronics:
            return 0.0, 0.0, 0.0

        # 1. Electrical Validity: Did it generate valid electronics?
        # We check if there's a power supply and at least one component
        # ElectronicsSection already enforces power_supply presence if not None
        has_psu = electronics.power_supply is not None
        has_components = len(electronics.components) > 0
        validity = 1.0 if (has_psu and has_components) else 0.5

        # 2. Wire Integrity: Are there wires and are they routed?
        wiring = electronics.wiring
        if wiring:
            routed_count = sum(1 for w in wiring if w.routed_in_3d or w.waypoints)
            integrity = routed_count / len(wiring)
        else:
            integrity = 0.0

        # 3. Power Efficiency: total_draw vs max_current
        psu = electronics.power_supply
        max_current = psu.max_current_a
        # In a real scenario we'd get this from a simulation log
        # For now, we look for estimated_draw in metadata if available, else placeholder
        # Note: electronics doesn't have a 'metadata' field in the schema yet,
        # so we might need to add it or skip this part for now.
        # But for now I will keep the original logic if it was using raw data
        # for extra fields not in schema yet.
        # Actually, AssemblyDefinition has it in some way? No.
        draw = 0.5  # Default draw

        # Optimal efficiency is often considered around 80% load for power supplies
        if draw > max_current:
            efficiency = 0.0  # Overloaded
        else:
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
