import logging
import structlog
from shared.models.schemas import ElectronicsSection

logger = structlog.get_logger(__name__)


class ElectronicsManager:
    """Handles electronics simulation (SPICE and connectivity-based fallback)."""

    def __init__(self, electronics: ElectronicsSection | None = None):
        self.electronics = electronics
        self.is_powered_map: dict[str, bool] = {}
        self.switch_states: dict[str, bool] = {}
        self.errors: list[str] = []

        if self.electronics:
            for comp in self.electronics.components:
                if comp.type in ["switch", "relay"]:
                    self.switch_states[comp.component_id] = False

    def update(self, force: bool = False):
        """Update is_powered_map based on circuit state."""
        if not self.electronics:
            return

        # T004: Inlined SPICE simulation logic (simulated for refactor)
        # In a real implementation, this would call out to a SPICE engine.
        try:
            # Placeholder for actual SPICE logic
            # For now, we always use fallback as SPICE is not implemented
            raise NotImplementedError("SPICE simulation not implemented")
        except Exception as e:
            if not isinstance(e, NotImplementedError):
                logger.warning("spice_sim_failed_falling_back", error=str(e))
            self._fallback_update()

    def _fallback_update(self):
        """Connectivity-based fallback for is_powered_map if SPICE fails."""
        if not self.electronics:
            return

        self.errors = []
        powered_components = set()

        # 1. Identify power sources
        sources = [
            c.component_id
            for c in self.electronics.components
            if c.type == "power_supply"
        ]

        if not sources:
            self.is_powered_map = {}
            return

        # 2. Propagate power (BFS)
        # We consider a component "powered" if it can be reached from a power supply
        # through wires and closed switches.
        queue = list(sources)
        visited = set(sources)
        powered_components.update(sources)

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
                    # Check if connection is blocked by a switch/relay at 'u'
                    is_blocked = False
                    comp_u = next(
                        (c for c in self.electronics.components if c.component_id == u),
                        None,
                    )
                    if comp_u and comp_u.type in ["switch", "relay"]:
                        if not self.switch_states.get(u, False):
                            is_blocked = True

                    if not is_blocked:
                        visited.add(v)
                        powered_components.add(v)
                        queue.append(v)

        # 3. Detect short circuits
        self._check_short_circuit()

        # 4. Map results to actuators (using assembly_part_ref)
        self.is_powered_map = {}
        for comp in self.electronics.components:
            is_powered = comp.component_id in powered_components
            # Power gating: switches/relays are only "powered" if closed (simplified)
            if comp.type in ["switch", "relay"] and not self.switch_states.get(
                comp.component_id, False
            ):
                is_powered = False

            self.is_powered_map[comp.component_id] = is_powered
            if comp.assembly_part_ref:
                self.is_powered_map[comp.assembly_part_ref] = is_powered

    def _check_short_circuit(self):
        """Detect if any power supply is short-circuited."""
        for supply in self.electronics.components:
            if supply.type != "power_supply":
                continue

            terminals = self._get_component_terminals(supply.component_id)
            if len(terminals) < 2:
                continue

            term_list = list(terminals)
            # Check if any two terminals of the power supply are connected via conductive path
            for i in range(len(term_list)):
                for j in range(i + 1, len(term_list)):
                    if self._has_conductive_path(
                        supply.component_id,
                        term_list[i],
                        supply.component_id,
                        term_list[j],
                    ):
                        msg = f"SHORT_CIRCUIT: {supply.component_id} terminals {term_list[i]} and {term_list[j]} connected."
                        self.errors.append(msg)
                        logger.error("electronics_short_circuit", error=msg)
                        return

    def _has_conductive_path(self, start_comp, start_term, end_comp, end_term):
        """Finds if there's a low-resistance path between two terminals."""
        queue = [(start_comp, start_term)]
        visited = set([(start_comp, start_term)])

        while queue:
            curr_comp, curr_term = queue.pop(0)

            if curr_comp == end_comp and curr_term == end_term:
                return True

            # Travel through wires
            for wire in self.electronics.wiring:
                nxt = None
                if (
                    wire.from_terminal.component == curr_comp
                    and wire.from_terminal.terminal == curr_term
                ):
                    nxt = (wire.to_terminal.component, wire.to_terminal.terminal)
                elif (
                    wire.to_terminal.component == curr_comp
                    and wire.to_terminal.terminal == curr_term
                ):
                    nxt = (wire.from_terminal.component, wire.from_terminal.terminal)

                if nxt and nxt not in visited:
                    visited.add(nxt)
                    queue.append(nxt)

            # Travel through conductive components
            comp = next(
                (
                    c
                    for c in self.electronics.components
                    if c.component_id == curr_comp
                ),
                None,
            )
            if comp:
                # Conductive: switch (if closed), connector
                is_conductive = False
                if comp.type in ["switch", "relay"]:
                    if self.switch_states.get(curr_comp, False):
                        is_conductive = True
                elif comp.type == "connector":
                    is_conductive = True

                if is_conductive:
                    for t in self._get_component_terminals(curr_comp):
                        if (curr_comp, t) not in visited:
                            visited.add((curr_comp, t))
                            queue.append((curr_comp, t))
        return False

    def _get_component_terminals(self, component_id: str) -> set[str]:
        terms = set()
        for wire in self.electronics.wiring:
            if wire.from_terminal.component == component_id:
                terms.add(wire.from_terminal.terminal)
            if wire.to_terminal.component == component_id:
                terms.add(wire.to_terminal.terminal)
        return terms
