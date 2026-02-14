import sys
from unittest.mock import MagicMock, patch
import pytest

# Create a mock for the genesis module
mock_gs = MagicMock()
mock_gs.gpu = "gpu"

# Patch sys.modules to include genesis
# We must do this before importing GenesisBackend if it hasn't been imported yet,
# or reload it if it has. Since pytest runs collection first, it's tricky.
# But putting the import inside the test function or using patch.dict globally helps.

# However, for the module-level import `try: import genesis as gs` to work,
# we need to ensure `genesis` is in sys.modules before that line executes.
# The safest way in a test file is to patch it before importing the module under test.

@pytest.fixture(autouse=True)
def mock_genesis_module():
    with patch.dict(sys.modules, {"genesis": mock_gs}):
        # We need to reload the module to pick up the mock
        if "worker.simulation.genesis_backend" in sys.modules:
            import importlib
            import worker.simulation.genesis_backend
            importlib.reload(worker.simulation.genesis_backend)
        yield

from worker.simulation.genesis_backend import GenesisBackend
from shared.simulation.backends import SimulationScene

def test_genesis_backend_initialization():
    # Reload to ensure init runs with mock
    import importlib
    import worker.simulation.genesis_backend
    importlib.reload(worker.simulation.genesis_backend)

    # Re-import class after reload
    from worker.simulation.genesis_backend import GenesisBackend

    backend = GenesisBackend()
    mock_gs.init.assert_called_with(backend=mock_gs.gpu)

def test_genesis_backend_load_scene_mocked():
    # Reload to ensure consistency
    import importlib
    import worker.simulation.genesis_backend
    importlib.reload(worker.simulation.genesis_backend)
    from worker.simulation.genesis_backend import GenesisBackend

    backend = GenesisBackend()
    scene = SimulationScene(scene_path="test.json")

    # Setup mock Scene instance return value
    mock_scene_instance = MagicMock()
    mock_gs.Scene.return_value = mock_scene_instance

    backend.load_scene(scene)

    # Verify scene creation and build
    mock_gs.Scene.assert_called_with(show_viewer=False)
    mock_scene_instance.build.assert_called_once()

    assert backend.scene_def == scene
    assert backend.gs_scene == mock_scene_instance

def test_genesis_backend_step_mocked():
    import importlib
    import worker.simulation.genesis_backend
    importlib.reload(worker.simulation.genesis_backend)
    from worker.simulation.genesis_backend import GenesisBackend

    backend = GenesisBackend()

    # Manually inject mock scene
    mock_scene_instance = MagicMock()
    backend.gs_scene = mock_scene_instance

    res = backend.step(0.1)

    assert res.success
    assert res.time == 0.1
    mock_scene_instance.step.assert_called_once()

def test_missing_methods_stubs():
    import importlib
    import worker.simulation.genesis_backend
    importlib.reload(worker.simulation.genesis_backend)
    from worker.simulation.genesis_backend import GenesisBackend

    backend = GenesisBackend()

    assert backend.get_all_tendon_names() == []
    assert backend.get_tendon_tension("tendon1") == 0.0

def test_close_clears_scene():
    import importlib
    import worker.simulation.genesis_backend
    importlib.reload(worker.simulation.genesis_backend)
    from worker.simulation.genesis_backend import GenesisBackend

    backend = GenesisBackend()
    backend.gs_scene = MagicMock()
    backend.scene_def = MagicMock()

    backend.close()

    assert backend.gs_scene is None
    assert backend.scene_def is None
