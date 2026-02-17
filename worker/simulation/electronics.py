import structlog

from shared.models.schemas import ElectronicsSection

logger = structlog.get_logger(__name__)


class ElectronicsManager:
    """Handles electronics simulation (SPICE and connectivity-based fallback)."""

    def __init__(self, electronics: ElectronicsSection | None = None):
        self.electronics = electronics
        self.is_powered_map: dict[str, bool] = {}
        self.switch_states: dict[str, bool] = {}

        if self.electronics:
            from shared.enums import ElectronicComponentType

            for comp in self.electronics.components:
                if comp.type in [
                    ElectronicComponentType.SWITCH,
                    ElectronicComponentType.RELAY,
                ]:
                    self.switch_states[comp.component_id] = False

    def update(self, force: bool = False):
        """Update is_powered_map based on circuit state."""
        if not self.electronics:
            return

        # T004: Inlined SPICE simulation logic (simulated for refactor)
        # In a real implementation, this would call out to a SPICE engine.
        try:
            # Placeholder for actual SPICE logic
            # For now, we always use fallback as SPICE is not fully integrated
            self._fallback_update()
        except Exception as e:
            logger.warning("spice_sim_failed_falling_back", error=str(e))
            self._fallback_update()

    def _fallback_update(self):
        """Connectivity-based fallback for is_powered_map if SPICE fails."""
        if not self.electronics:
            return

        from shared.enums import ElectronicComponentType

        # BFS for connectivity
        powered = set()
        sources = [
            c.component_id
            for c in self.electronics.components
            if c.type == ElectronicComponentType.POWER_SUPPLY
        ]

        # Heuristic: add common source names if they are referenced but not in components
        comp_ids = {c.component_id for c in self.electronics.components}
        for wire in self.electronics.wiring:
            for ref in [wire.from_terminal.component, wire.to_terminal.component]:
                if ref not in comp_ids:
                    if any(s in ref.lower() for s in ["supply", "battery", "v_source", "source"]):
                        if ref not in sources:
                            sources.append(ref)

        # Simplified BFS for power propagation
        queue = list(sources)
        visited = set(sources)
        powered.update(sources)

        # Map components for easy lookup
        comp_map = {c.component_id: c for c in self.electronics.components}

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
                    # Check if connection is closed (if it involves a switch/relay)
                    can_pass = True
                    u_comp = comp_map.get(u)
                    v_comp = comp_map.get(v)

                    for comp in [u_comp, v_comp]:
                        if comp and comp.type in [
                            ElectronicComponentType.SWITCH,
                            ElectronicComponentType.RELAY,
                        ]:
                            if not self.switch_states.get(comp.component_id, False):
                                can_pass = False
                                break

                    if can_pass:
                        visited.add(v)
                        powered.add(v)
                        queue.append(v)

        for comp in self.electronics.components:
            # Power scales could be 0.0 or 1.0 in this fallback
            self.is_powered_map[comp.component_id] = 1.0 if comp.component_id in powered else 0.0
