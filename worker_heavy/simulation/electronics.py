import structlog

from shared.enums import ElectronicComponentType
from shared.models.schemas import ElectronicsSection
from shared.models.simulation import SimulationFailure

logger = structlog.get_logger(__name__)


class ElectronicsManager:
    """Handles electronics simulation (SPICE and connectivity-based fallback)."""

    def __init__(self, electronics: ElectronicsSection | None = None):
        self.electronics = electronics
        self.is_powered_map: dict[str, bool] = {}
        self.switch_states: dict[str, bool] = {}
        self.validation_error: SimulationFailure | str | None = None

        if self.electronics:
            for comp in self.electronics.components:
                if comp.type in ["switch", "relay"]:
                    self.switch_states[comp.component_id] = True

    def update(self, force: bool = False):
        """Update is_powered_map based on circuit state."""
        if not self.electronics:
            return

        # T004: Integrated SPICE simulation logic
        try:
            from shared.circuit_builder import build_circuit_from_section
            from shared.pyspice_utils import validate_circuit

            circuit = build_circuit_from_section(
                self.electronics, switch_states=self.switch_states, add_shunts=True
            )
            validation = validate_circuit(
                circuit, self.electronics.power_supply, section=self.electronics
            )

            if validation.valid:
                self.validation_error = None
                # Map voltages to power status.
                # A component is powered if the voltage across its terminals is sufficient.
                # For simplicity in this integration, we'll check if the component's '+' or 'in'
                # node has a significant voltage relative to ground.
                for comp in self.electronics.components:
                    key = comp.assembly_part_ref or comp.component_id
                    # Power supply itself is always 'powered' if present
                    if comp.type == "power_supply":
                        self.is_powered_map[key] = 1.0
                        continue

                    # Determine primary input node for the component
                    node_name = f"{comp.component_id}_+"
                    if comp.type in ["switch", "relay"]:
                        node_name = f"{comp.component_id}_in"

                    voltage = validation.node_voltages.get(node_name, 0.0)
                    # Normalize power scale (0.0 to 1.0) based on supply voltage
                    supply_v = self.electronics.power_supply.voltage_dc
                    self.is_powered_map[key] = (
                        min(1.0, max(0.0, voltage / supply_v)) if supply_v > 0 else 0.0
                    )
            else:
                self.validation_error = (
                    validation.failures[0]
                    if validation.failures
                    else ", ".join(validation.errors)
                )
                logger.warning("circuit_validation_failed", errors=validation.errors)
                self._fallback_update()

        except Exception as e:
            self.validation_error = str(e)
            logger.warning("spice_sim_failed_falling_back", error=str(e))
            self._fallback_update()

    def _fallback_update(self):
        """Connectivity-based fallback for is_powered_map if SPICE fails."""
        if not self.electronics:
            return

        # BFS for connectivity
        # (Extracted from loop.py:227-307)
        powered = set()
        sources = [
            c.component_id
            for c in self.electronics.components
            if c.type == ElectronicComponentType.POWER_SUPPLY
        ]

        # Simplified BFS for power propagation
        queue = list(sources)
        visited = set(sources)
        powered.update(sources)

        while queue:
            u = queue.pop(0)
            # Find neighbors in wiring
            for wire in self.electronics.wiring:
                v = None
                if wire.from_terminal.component == u:
                    v = wire.to_terminal.component
                elif wire.to_terminal.component == u:
                    v = wire.from_terminal.component

                if v and v not in visited:
                    # Check if connection is closed (if it involves a switch)
                    # This is simplified logic
                    visited.add(v)
                    powered.add(v)
                    queue.append(v)

        for comp in self.electronics.components:
            key = comp.assembly_part_ref or comp.component_id
            self.is_powered_map[key] = comp.component_id in powered
