
import pytest
import numpy as np
from unittest.mock import MagicMock
import sys

# Mock genesis if not installed
if "genesis" not in sys.modules:
    sys.modules["genesis"] = MagicMock()

from worker.simulation.genesis_backend import GenesisBackend
from shared.simulation.backends import SiteState

def test_genesis_backend_cables():
    backend = GenesisBackend()
    # Manually inject cables data
    backend.cables = [
        {"name": "cable_1", "radius": 0.001},
        {"name": "cable_2", "radius": 0.002}
    ]

    names = backend.get_all_tendon_names()
    assert "cable_1" in names
    assert "cable_2" in names
    assert len(names) == 2

def test_genesis_backend_site_state():
    backend = GenesisBackend()
    # Manually inject entity_configs
    backend.entity_configs = {
        "zone_1": {"is_zone": True, "pos": [1, 2, 3], "size": [0.1, 0.1, 0.1]},
        "zone_2": {"is_zone": True, "pos": [4, 5, 6], "size": [0.2, 0.2, 0.2]}
    }

    state1 = backend.get_site_state("zone_1")
    assert state1.pos == (1, 2, 3)
    assert state1.size == (0.1, 0.1, 0.1)

    state2 = backend.get_site_state("zone_2")
    assert state2.pos == (4, 5, 6)

    # Update pos
    backend.set_site_pos("zone_1", np.array([7, 8, 9]))
    state1_updated = backend.get_site_state("zone_1")
    assert state1_updated.pos == (7, 8, 9)
    assert backend.entity_configs["zone_1"]["pos"] == [7, 8, 9]

def test_check_collision_zone_fallback():
    backend = GenesisBackend()
    backend.entity_configs = {
        "zone_forbid": {"is_zone": True, "pos": [0, 0, 0], "size": [1, 1, 1]}
    }

    # Mock get_body_state to return specific positions
    backend.get_body_state = MagicMock()

    # Case 1: Inside zone
    mock_state_in = MagicMock()
    mock_state_in.pos = [0.5, 0.5, 0.5]
    backend.get_body_state.return_value = mock_state_in

    # Need to mock entities.get to return None so it falls back to zone check
    backend.entities = {}

    assert backend.check_collision("body_1", "zone_forbid") is True

    # Case 2: Outside zone
    mock_state_out = MagicMock()
    mock_state_out.pos = [2, 2, 2]
    backend.get_body_state.return_value = mock_state_out

    assert backend.check_collision("body_1", "zone_forbid") is False
