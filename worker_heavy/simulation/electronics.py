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
                    # Power supply itself is always 'powered' if present
                    if comp.type == "power_supply":
                        self.is_powered_map[comp.component_id] = 1.0
                        continue

                    # Determine primary input node for the component
                    node_name = f"{comp.component_id}_+"
                    if comp.type in ["switch", "relay"]:
                        node_name = f"{comp.component_id}_in"

                    voltage = validation.node_voltages.get(node_name, 0.0)
                    # Normalize power scale (0.0 to 1.0) based on supply voltage
                    supply_v = self.electronics.power_supply.voltage_dc
                    self.is_powered_map[comp.component_id] = (
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

        # Initialize is_powered_map to False (0.0) for all components
        self.is_powered_map = {c.component_id: 0.0 for c in self.electronics.components}

        # 1. Build Component Maps and Wire Adjacency
        comp_type_map = {c.component_id: c.type for c in self.electronics.components}
        comp_terminals: dict[str, set[str]] = {}

        # Map: (comp_id, terminal) -> list of (comp_id, terminal)
        wire_adj: dict[tuple[str, str], list[tuple[str, str]]] = {}

        for wire in self.electronics.wiring:
            u_comp = wire.from_terminal.component
            u_term = wire.from_terminal.terminal
            v_comp = wire.to_terminal.component
            v_term = wire.to_terminal.terminal

            u_node = (u_comp, u_term)
            v_node = (v_comp, v_term)

            wire_adj.setdefault(u_node, []).append(v_node)
            wire_adj.setdefault(v_node, []).append(u_node)

            comp_terminals.setdefault(u_comp, set()).add(u_term)
            comp_terminals.setdefault(v_comp, set()).add(v_term)

        # 2. Identify Sources and Initialize Queue
        # Sources: Power Supplies. Treat all their connected terminals as powered.
        queue: list[tuple[str, str]] = []
        powered_terminals: set[tuple[str, str]] = set()

        for comp in self.electronics.components:
            if comp.type == ElectronicComponentType.POWER_SUPPLY:
                self.is_powered_map[comp.component_id] = 1.0
                if comp.component_id in comp_terminals:
                    for term in comp_terminals[comp.component_id]:
                        node = (comp.component_id, term)
                        if node not in powered_terminals:
                            powered_terminals.add(node)
                            queue.append(node)

        # 3. BFS Propagation
        while queue:
            u_comp, u_term = queue.pop(0)
            u_node = (u_comp, u_term)

            # Mark component as powered if any terminal receives power
            self.is_powered_map[u_comp] = 1.0

            # A. Propagate OUT via Wires (External)
            if u_node in wire_adj:
                for v_node in wire_adj[u_node]:
                    if v_node not in powered_terminals:
                        powered_terminals.add(v_node)
                        queue.append(v_node)
                        # Mark neighbor component as powered immediately upon arrival
                        self.is_powered_map[v_node[0]] = 1.0

            # B. Propagate IN (Internal Component Logic)
            # Determine if we should bridge to other terminals on the same component
            should_bridge = True
            ctype = comp_type_map.get(u_comp)

            if ctype in [ElectronicComponentType.SWITCH, ElectronicComponentType.RELAY]:
                # Only bridge if switch is CLOSED
                if not self.switch_states.get(u_comp, False):
                    should_bridge = False

            if should_bridge and u_comp in comp_terminals:
                for other_term in comp_terminals[u_comp]:
                    if other_term == u_term:
                        continue

                    target_node = (u_comp, other_term)
                    if target_node not in powered_terminals:
                        powered_terminals.add(target_node)
                        # Add to queue to propagate OUT from this new terminal
                        queue.append(target_node)
