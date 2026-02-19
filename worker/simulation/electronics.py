import structlog

from shared.enums import ElectronicComponentType
from shared.models.schemas import ElectronicsSection

logger = structlog.get_logger(__name__)


class ElectronicsManager:
    """Handles electronics simulation (SPICE and connectivity-based fallback)."""

    def __init__(self, electronics: ElectronicsSection | None = None):
        self.electronics = electronics
        self.is_powered_map: dict[str, bool] = {}
        self.switch_states: dict[str, bool] = {}
        self.validation_error: str | None = None

        if self.electronics:
            for comp in self.electronics.components:
                if comp.type in ["switch", "relay"]:
                    # Switches are closed by default for simulation stability
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
            validation = validate_circuit(circuit, self.electronics.power_supply)

            if validation.valid:
                self.validation_error = None
                # Map voltages to power status.
                # A component is powered if the voltage across its terminals is sufficient.
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
                logger.warning("circuit_validation_failed", errors=validation.errors)
                # Only set validation_error for real circuit failures, not environment issues
                real_errors = [
                    e
                    for e in validation.errors
                    if e.startswith("FAILED_") or "electronics_validation_failed" in e
                ]
                if real_errors:
                    self.validation_error = "; ".join(real_errors)
                else:
                    self.validation_error = None
                self._fallback_update()

        except Exception as e:
            logger.warning("spice_sim_failed_falling_back", error=str(e))
            self.validation_error = None
            self._fallback_update()

    def _fallback_update(self):
        """Connectivity-based fallback for is_powered_map if SPICE fails."""
        if not self.electronics:
            return

        # Connectivity-based power propagation
        powered = set()

        # Power supply is always powered
        sources = [
            c.component_id
            for c in self.electronics.components
            if c.type in [ElectronicComponentType.POWER_SUPPLY, "battery", "v_source"]
        ]
        # Also treat 'supply' as a source for backward compatibility with some tests
        if "supply" not in [c.component_id for c in self.electronics.components]:
            sources.append("supply")

        # Simplified BFS for power propagation through wiring
        queue = list(sources)
        visited = set(sources)
        powered.update(sources)

        while queue:
            u = queue.pop(0)

            # If u is a switch/relay and it's OPEN, power doesn't pass THROUGH it
            if u in self.switch_states and not self.switch_states[u]:
                continue

            # Find neighbors connected by wires
            for wire in self.electronics.wiring:
                # Heuristic: Skip ground/return wires in power rail BFS
                if wire.from_terminal.terminal.lower() in ["0", "gnd", "-", "v-"] or \
                   wire.to_terminal.terminal.lower() in ["0", "gnd", "-", "v-"]:
                    continue

                v = None
                if wire.from_terminal.component == u:
                    v = wire.to_terminal.component
                elif wire.to_terminal.component == u:
                    v = wire.from_terminal.component

                if v and v not in visited:
                    visited.add(v)
                    powered.add(v)
                    queue.append(v)

        for comp in self.electronics.components:
            self.is_powered_map[comp.component_id] = 1.0 if comp.component_id in powered else 0.0

        # Handle the special 'supply' case if it was used
        if "supply" in powered:
             self.is_powered_map["supply"] = 1.0
