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

    def _resolve_terminal_node(self, comp_id: str, term: str) -> str:
        """Resolve component ID and terminal to a standard graph node name."""
        if term == "supply_v+" or (comp_id == "supply" and term == "v+"):
            return "supply_v+"
        if term == "0" or (comp_id == "supply" and term == "0"):
            return "0"

        # Normalize motor terminals
        if term == "a":
            term = "+"
        if term == "b":
            term = "-"

        return f"{comp_id}_{term}"

    def _fallback_update(self):
        """
        Connectivity-based fallback for is_powered_map if SPICE fails.
        Uses a graph traversal to check if components are connected to both VCC and GND.
        """
        if not self.electronics:
            return

        adj: dict[str, list[str]] = {}  # Adjacency list: node -> list[node]

        def add_edge(u, v):
            if u not in adj:
                adj[u] = []
            if v not in adj:
                adj[v] = []
            adj[u].append(v)
            adj[v].append(u)

        # 1. Add Wiring Edges
        for wire in self.electronics.wiring:
            u = self._resolve_terminal_node(
                wire.from_terminal.component, wire.from_terminal.terminal
            )
            v = self._resolve_terminal_node(
                wire.to_terminal.component, wire.to_terminal.terminal
            )
            add_edge(u, v)

        # 2. Add Internal Component Edges
        for comp in self.electronics.components:
            cid = comp.component_id
            ctype = comp.type

            if ctype in [ElectronicComponentType.SWITCH, ElectronicComponentType.RELAY]:
                # Connect 'in' to 'out' only if closed
                if self.switch_states.get(cid, False):
                    u = self._resolve_terminal_node(cid, "in")
                    v = self._resolve_terminal_node(cid, "out")
                    add_edge(u, v)

            elif ctype == ElectronicComponentType.MOTOR:
                # Motors conduct current, so connect '+' to '-'
                u = self._resolve_terminal_node(cid, "+")
                v = self._resolve_terminal_node(cid, "-")
                add_edge(u, v)

        # 3. Identify Source and Ground Roots
        vcc_roots = set()
        gnd_roots = set()

        # Main Supply
        if self.electronics.power_supply:
            vcc_roots.add("supply_v+")
            gnd_roots.add("0")

        # Batteries (Secondary Supplies)
        for comp in self.electronics.components:
            if comp.type == ElectronicComponentType.POWER_SUPPLY:
                vcc_roots.add(self._resolve_terminal_node(comp.component_id, "+"))
                gnd_roots.add(self._resolve_terminal_node(comp.component_id, "-"))

        # 4. BFS from VCC
        reachable_vcc = set()
        queue = list(vcc_roots)
        reachable_vcc.update(vcc_roots)

        while queue:
            u = queue.pop(0)
            if u in adj:
                for v in adj[u]:
                    if v not in reachable_vcc:
                        reachable_vcc.add(v)
                        queue.append(v)

        # 5. BFS from GND
        reachable_gnd = set()
        queue = list(gnd_roots)
        reachable_gnd.update(gnd_roots)

        while queue:
            u = queue.pop(0)
            if u in adj:
                for v in adj[u]:
                    if v not in reachable_gnd:
                        reachable_gnd.add(v)
                        queue.append(v)

        # 6. Determine Power State
        for comp in self.electronics.components:
            cid = comp.component_id

            # Identify terminals to check based on component type
            pos_term = self._resolve_terminal_node(cid, "+")
            neg_term = self._resolve_terminal_node(cid, "-")

            if comp.type in [
                ElectronicComponentType.SWITCH,
                ElectronicComponentType.RELAY,
            ]:
                pos_term = self._resolve_terminal_node(cid, "in")
                neg_term = self._resolve_terminal_node(cid, "out")

            # Check connectivity (polarity agnostic)
            powered = (pos_term in reachable_vcc and neg_term in reachable_gnd) or (
                pos_term in reachable_gnd and neg_term in reachable_vcc
            )

            self.is_powered_map[cid] = 1.0 if powered else 0.0
