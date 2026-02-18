import pytest
from unittest.mock import MagicMock
from worker.simulation.mujoco_backend import MuJoCoBackend

def test_mujoco_close_cleans_up_resources():
    backend = MuJoCoBackend()

    # Mock resources
    backend.model = MagicMock()
    backend.data = MagicMock()
    backend.renderer = MagicMock()

    # Call close
    backend.close()

    # Assertions
    assert backend.model is None
    assert backend.data is None
    assert backend.renderer is None

if __name__ == "__main__":
    pytest.main([__file__])
