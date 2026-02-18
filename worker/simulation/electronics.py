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
            # For now, always use fallback since SPICE is not implemented in this snippet
            self._fallback_update()
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
            if c.type == "power_supply"
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
                    # Check if connection is closed (if u is a switch/relay)
                    u_comp = next(
                        (c for c in self.electronics.components if c.component_id == u),
                        None,
                    )
                    can_pass = True
                    if u_comp and u_comp.type in ["switch", "relay"]:
                        if not self.switch_states.get(u, False):
                            can_pass = False

                    if can_pass:
                        visited.add(v)
                        powered.add(v)
                        queue.append(v)

        for comp in self.electronics.components:
            self.is_powered_map[comp.component_id] = comp.component_id in powered
