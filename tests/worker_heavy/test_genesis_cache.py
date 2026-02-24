
import sys
from unittest.mock import MagicMock, patch

# Mock genesis if not present
sys.modules["genesis"] = MagicMock()

import pytest
from worker_heavy.simulation.genesis_backend import GenesisBackend
from shared.simulation.backends import ContactForce, SimulationScene

@pytest.fixture
def backend():
    # Setup backend with necessary mocks
    backend = GenesisBackend()
    backend.scene = MagicMock()
    backend.scene.sim = MagicMock()
    return backend

def test_get_contact_forces_caching(backend):
    """Test that get_contact_forces uses cache within the same step."""

    # Mock return value for get_contacts
    contact_mock = MagicMock()
    b1 = MagicMock()
    b1.name = "b1"
    b2 = MagicMock()
    b2.name = "b2"
    contact_mock.entities = (b1, b2)

    # Mock numpy conversions
    force_mock = MagicMock()
    force_mock.cpu.return_value.numpy.return_value.tolist.return_value = [1.0, 0.0, 0.0]
    contact_mock.force = force_mock

    pos_mock = MagicMock()
    pos_mock.cpu.return_value.numpy.return_value.tolist.return_value = [0.0, 1.0, 0.0]
    contact_mock.pos = pos_mock

    backend.scene.sim.get_contacts.return_value = [contact_mock]

    # Step 1: First call
    backend._step_counter = 1
    result1 = backend.get_contact_forces()
    assert len(result1) == 1
    assert backend.scene.sim.get_contacts.call_count == 1

    # Step 1: Second call - should use cache
    result2 = backend.get_contact_forces()
    assert len(result2) == 1
    assert result2 is result1  # Should return exact same list object
    assert backend.scene.sim.get_contacts.call_count == 1

    # Step 2: Increment step
    backend._step_counter += 1

    # Step 2: First call - should refetch
    result3 = backend.get_contact_forces()
    assert len(result3) == 1
    assert result3 is not result1 # New list object (though content might be similar)
    assert backend.scene.sim.get_contacts.call_count == 2

    # Step 2: Second call - should use new cache
    result4 = backend.get_contact_forces()
    assert result4 is result3
    assert backend.scene.sim.get_contacts.call_count == 2

def test_load_scene_resets_cache(backend):
    """Test that load_scene resets the cache."""

    # Populate cache
    backend._contact_cache = ["dummy"]
    backend._contact_cache_step = 10
    backend._step_counter = 10

    scene = SimulationScene()
    # Mock load_scene internals to avoid complex logic
    with patch.object(backend, '_load_scene_internal') as mock_internal:
        with patch.object(backend, '_load_mfg_config'):
            backend.load_scene(scene)

    assert backend._contact_cache is None
    assert backend._contact_cache_step == -1
    assert backend._step_counter == 0

def test_close_resets_cache(backend):
    """Test that close() resets the cache."""
    backend._contact_cache = ["dummy"]
    backend._contact_cache_step = 5

    backend.close()

    assert backend._contact_cache is None
    assert backend._contact_cache_step == -1

def test_step_increments_counter(backend):
    """Test that step() increments the counter."""
    backend._step_counter = 0
    backend._is_built = True # Mock built state

    backend.step(0.01)
    assert backend._step_counter == 1

    backend.step(0.01)
    assert backend._step_counter == 2
