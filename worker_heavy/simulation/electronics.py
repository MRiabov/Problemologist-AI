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

        # Build Adjacency Graph: Node = (component_id, terminal_name)
        adj = {}

        def add_edge(u, v):
            if u not in adj:
                adj[u] = []
            if v not in adj:
                adj[v] = []
            adj[u].append(v)
            adj[v].append(u)

        # 1. Add edges from Wiring
        component_terminals = {}  # comp_id -> set(terminals)
        for wire in self.electronics.wiring:
            u = (wire.from_terminal.component, wire.from_terminal.terminal)
            v = (wire.to_terminal.component, wire.to_terminal.terminal)
            add_edge(u, v)

            # Track terminals per component
            if u[0] not in component_terminals:
                component_terminals[u[0]] = set()
            component_terminals[u[0]].add(u[1])
            if v[0] not in component_terminals:
                component_terminals[v[0]] = set()
            component_terminals[v[0]].add(v[1])

        # 2. Add internal component paths
        for comp in self.electronics.components:
            cid = comp.component_id
            terms = list(component_terminals.get(cid, []))
            if len(terms) < 2:
                continue

            # Check component type
            is_conductive = True
            if comp.type in [
                ElectronicComponentType.SWITCH,
                ElectronicComponentType.RELAY,
            ]:
                is_conductive = self.switch_states.get(cid, True)
            elif comp.type == ElectronicComponentType.POWER_SUPPLY:
                # Power supplies don't short + to - internally
                is_conductive = False

            if is_conductive:
                # Connect all terminals to each other (clique)
                for i in range(len(terms)):
                    for j in range(i + 1, len(terms)):
                        add_edge((cid, terms[i]), (cid, terms[j]))

        # 3. BFS from Power Sources
        queue = []
        visited = set()

        # Helper to check if terminal is positive/source
        def is_source_terminal(t_name):
            t_lower = t_name.lower()
            return (
                any(
                    x in t_lower
                    for x in [
                        "+",
                        "v",
                        "pos",
                        "high",
                        "live",
                        "hot",
                        "12",
                        "24",
                        "5",
                    ]
                )
                or t_name == "supply_v+"
            )

        # Check main power supply (implicit 'supply' component)
        if "supply" in component_terminals:
            for t in component_terminals["supply"]:
                if is_source_terminal(t) or t == "v+":
                    node = ("supply", t)
                    if node not in visited:
                        visited.add(node)
                        queue.append(node)

        # Find source nodes (V+ terminals of additional power supplies)
        for comp in self.electronics.components:
            if comp.type == ElectronicComponentType.POWER_SUPPLY:
                cid = comp.component_id
                terms = component_terminals.get(cid, [])
                for t in terms:
                    if is_source_terminal(t):
                        node = (cid, t)
                        if node not in visited:
                            visited.add(node)
                            queue.append(node)

        while queue:
            u = queue.pop(0)
            if u in adj:
                for v in adj[u]:
                    if v not in visited:
                        visited.add(v)
                        queue.append(v)

        # 4. Update map
        powered_components = set(cid for (cid, t) in visited)

        for comp in self.electronics.components:
            val = 1.0 if comp.component_id in powered_components else 0.0
            self.is_powered_map[comp.component_id] = val
