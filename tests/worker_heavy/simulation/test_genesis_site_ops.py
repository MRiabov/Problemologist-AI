import sys
from unittest.mock import MagicMock

# Mock genesis module before importing GenesisBackend
# We need to ensure 'genesis' is in sys.modules so imports work
if "genesis" not in sys.modules:
    mock_genesis = MagicMock()
    sys.modules["genesis"] = mock_genesis

import numpy as np
import pytest
from unittest.mock import patch
from worker_heavy.simulation.genesis_backend import GenesisBackend

@pytest.fixture
def genesis_backend():
    # Since we mocked genesis in sys.modules, patch("genesis.init") might work or be redundant depending on how we want to assert
    # We can just use the mock from sys.modules or patch it.
    # But patch requires the module to be importable.

    with patch("genesis.init"), patch("genesis.Scene"):
        backend = GenesisBackend()
        # Mocking initialized state
        backend._is_built = True
        yield backend

def test_site_ops_min_max(genesis_backend):
    # Setup a zone defined by min/max
    site_name = "test_zone"
    genesis_backend.entity_configs = {
        site_name: {
            "name": site_name,
            "is_zone": True,
            "min": [0.0, 0.0, 0.0],
            "max": [2.0, 2.0, 2.0]
        }
    }

    # Verify initial state
    state = genesis_backend.get_site_state(site_name)
    assert state.pos == (1.0, 1.0, 1.0)
    assert state.size == (1.0, 1.0, 1.0) # Half extents

    # Move site
    new_pos = np.array([5.0, 5.0, 5.0])
    genesis_backend.set_site_pos(site_name, new_pos)

    # Verify update in configs
    cfg = genesis_backend.entity_configs[site_name]
    assert np.allclose(cfg["min"], [4.0, 4.0, 4.0])
    assert np.allclose(cfg["max"], [6.0, 6.0, 6.0])

    # Verify get_site_state reflects new pos
    state = genesis_backend.get_site_state(site_name)
    assert state.pos == (5.0, 5.0, 5.0)
    assert state.size == (1.0, 1.0, 1.0)

def test_site_ops_pos_size(genesis_backend):
    # Setup a zone defined by pos/size
    site_name = "test_zone_pos"
    genesis_backend.entity_configs = {
        site_name: {
            "name": site_name,
            "is_zone": True,
            "pos": [1.0, 1.0, 1.0],
            "size": [0.5, 0.5, 0.5]
        }
    }

    # Verify initial state
    state = genesis_backend.get_site_state(site_name)
    assert state.pos == (1.0, 1.0, 1.0)
    assert state.size == (0.5, 0.5, 0.5)

    # Move site
    new_pos = np.array([3.0, 3.0, 3.0])
    genesis_backend.set_site_pos(site_name, new_pos)

    # Verify update in configs
    cfg = genesis_backend.entity_configs[site_name]
    assert np.allclose(cfg["pos"], [3.0, 3.0, 3.0])
    assert np.allclose(cfg["size"], [0.5, 0.5, 0.5]) # Size unchanged

    # Verify get_site_state reflects new pos
    state = genesis_backend.get_site_state(site_name)
    assert state.pos == (3.0, 3.0, 3.0)

def test_site_ops_entity_move(genesis_backend):
    # Setup a physical entity
    site_name = "physical_ent"
    mock_entity = MagicMock()
    mock_entity.set_pos = MagicMock()
    mock_entity.get_state.return_value = MagicMock(pos=np.array([1, 2, 3]), quat=np.array([1, 0, 0, 0]))

    # Needs to be in entities dict
    genesis_backend.entities = {site_name: mock_entity}

    # We assume get_body_state works via mocking get_body_state on the instance
    mock_body_state = MagicMock()
    mock_body_state.pos = (1.0, 2.0, 3.0)
    mock_body_state.quat = (1.0, 0.0, 0.0, 0.0)
    genesis_backend.get_body_state = MagicMock(return_value=mock_body_state)

    state = genesis_backend.get_site_state(site_name)
    assert state.pos == (1.0, 2.0, 3.0)

    # Move entity
    new_pos = np.array([10.0, 10.0, 10.0])
    genesis_backend.set_site_pos(site_name, new_pos)

    mock_entity.set_pos.assert_called_once()
    args, _ = mock_entity.set_pos.call_args
    assert np.allclose(args[0], new_pos)

def test_site_ops_fail_unknown(genesis_backend):
    with pytest.raises(ValueError, match="not found"):
        genesis_backend.set_site_pos("unknown", np.array([0,0,0]))

    with pytest.raises(ValueError, match="not found"):
        genesis_backend.get_site_state("unknown")

def test_check_collision_after_move(genesis_backend):
    # Setup zone
    site_name = "zone"
    genesis_backend.entity_configs = {
        site_name: {
            "name": site_name,
            "is_zone": True,
            "min": [0.0, 0.0, 0.0],
            "max": [2.0, 2.0, 2.0] # Center at 1,1,1, radius 1
        }
    }

    # Mock body at 4,4,4
    body_name = "body"
    mock_body_state = MagicMock()
    mock_body_state.pos = (4.0, 4.0, 4.0)
    genesis_backend.get_body_state = MagicMock(return_value=mock_body_state)
    genesis_backend.entities = {body_name: MagicMock()}

    # Initially no collision
    assert genesis_backend.check_collision(body_name, site_name) is False

    # Move zone to 4,4,4
    genesis_backend.set_site_pos(site_name, np.array([4.0, 4.0, 4.0]))

    # Verify collision now
    assert genesis_backend.check_collision(body_name, site_name) is True
