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

                    # Determine primary input node for the component (normalized to +)
                    node_name = f"{comp.component_id}_+"
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
        """
        Connectivity-based fallback for is_powered_map if SPICE fails.
        Implements a terminal-aware BFS that respects switch states.
        """
        if not self.electronics:
            return

        from shared.circuit_builder import resolve_node_name

        # 1. Build adjacency list of standardized nodes
        adj: dict[str, list[str]] = {}

        def add_edge(u, v):
            adj.setdefault(u, []).append(v)
            adj.setdefault(v, []).append(u)

        # External wiring
        for wire in self.electronics.wiring:
            u_node = resolve_node_name(
                wire.from_terminal.component, wire.from_terminal.terminal
            )
            v_node = resolve_node_name(
                wire.to_terminal.component, wire.to_terminal.terminal
            )
            add_edge(u_node, v_node)

        # Internal component bridges
        for comp in self.electronics.components:
            if comp.type == ElectronicComponentType.MOTOR:
                # Motors act as conductive bridges between their terminals
                u = resolve_node_name(comp.component_id, "+")
                v = resolve_node_name(comp.component_id, "-")
                add_edge(u, v)
            elif comp.type in (
                ElectronicComponentType.SWITCH,
                ElectronicComponentType.RELAY,
            ):
                # Switches bridge terminals only when closed
                if self.switch_states.get(comp.component_id, True):
                    u = resolve_node_name(comp.component_id, "+")
                    v = resolve_node_name(comp.component_id, "-")
                    add_edge(u, v)

        # 2. BFS from all power sources
        # Standard PSU
        queue = ["supply_v+"]
        # Secondary sources (batteries)
        for comp in self.electronics.components:
            if comp.type == ElectronicComponentType.POWER_SUPPLY:
                queue.append(resolve_node_name(comp.component_id, "+"))

        visited = set(queue)
        powered_nodes = set()

        while queue:
            curr = queue.pop(0)
            powered_nodes.add(curr)
            for neighbor in adj.get(curr, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        # 3. Update is_powered_map
        for comp in self.electronics.components:
            if comp.type == ElectronicComponentType.POWER_SUPPLY:
                self.is_powered_map[comp.component_id] = 1.0
                continue

            # Component is powered if its primary input (+) is connected to a source
            pos_node = resolve_node_name(comp.component_id, "+")
            self.is_powered_map[comp.component_id] = (
                1.0 if pos_node in powered_nodes else 0.0
            )
