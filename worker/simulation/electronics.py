import structlog

from shared.models.schemas import ElectronicsSection
from shared.circuit_builder import build_circuit_from_section
from shared.pyspice_utils import validate_circuit

logger = structlog.get_logger(__name__)


class ElectronicsManager:
    """Handles electronics simulation (SPICE and connectivity-based fallback)."""

    def __init__(self, electronics: ElectronicsSection | None = None):
        self.electronics = electronics
        self.is_powered_map: dict[str, bool] = {}
        self.switch_states: dict[str, bool] = {}

        if self.electronics:
            for comp in self.electronics.components:
                if comp.type in ["switch", "relay"]:
                    self.switch_states[comp.component_id] = False

    def update(self, force: bool = False):
        """Update is_powered_map based on circuit state."""
        if not self.electronics:
            return

        try:
            # Build circuit with shunts for stability
            circuit = build_circuit_from_section(
                self.electronics, self.switch_states, add_shunts=True
            )

            # Validate/Simulate
            result = validate_circuit(circuit, self.electronics.power_supply)

            if not result.valid:
                logger.warning("spice_sim_invalid", errors=result.errors)
                self._fallback_update()
                return

            # Map voltages to powered state
            # Heuristic: A component is 'powered' if its input terminal has voltage > 1.0V
            # This is a simplification but works for current requirements.
            for comp in self.electronics.components:
                # Determine "input" or "positive" terminal node name based on circuit_builder naming
                # circuit_builder uses: f"{comp_id}_{term}"
                # For motors/sensors, usually 'a', '+', 'in', 'vcc'

                # We check common positive terminal names
                potential_terminals = ["+", "a", "in", "vcc", "v+"]
                voltage = 0.0

                for term in potential_terminals:
                    node_name = f"{comp.component_id}_{term}"
                    # Also check resolved names if wire connects directly?
                    # circuit_builder resolves names explicitly.

                    # If component is connected, its node should exist in node_voltages
                    if node_name in result.node_voltages:
                        voltage = result.node_voltages[node_name]
                        break

                # Special case for supply
                if comp.type == "power_supply":
                    self.is_powered_map[comp.component_id] = True
                    continue

                self.is_powered_map[comp.component_id] = voltage > 1.0

        except Exception as e:
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
            if c.type in ["battery", "v_source"]
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
            self.is_powered_map[comp.component_id] = comp.component_id in powered
