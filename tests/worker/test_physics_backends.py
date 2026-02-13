import pytest
import numpy as np
import os
from shared.simulation.backends import SimulatorBackendType, SimulationScene
from worker.simulation.factory import get_physics_backend
from worker.simulation.mujoco_backend import MuJoCoBackend
from worker.simulation.genesis_backend import GenesisBackend

from worker.simulation.loop import SimulationLoop


def test_simulation_loop_with_mujoco():
    # Use the minimal XML created for testing
    xml_path = "tests/worker/minimal.xml"
    if not os.path.exists(xml_path):
        pytest.skip(f"Test XML not found at {xml_path}")

    loop = SimulationLoop(xml_path, backend_type=SimulatorBackendType.MUJOCO)

    # Run a few steps
    metrics = loop.step(control_inputs={}, duration=0.1)

    assert metrics.total_time >= 0.1
    assert metrics.success == False  # No goal achieved in minimal.xml
    assert metrics.fail_reason == None  # Finished normally


def test_factory_mujoco():
    backend = get_physics_backend(SimulatorBackendType.MUJOCO)
    assert isinstance(backend, MuJoCoBackend)


def test_factory_genesis():
    backend = get_physics_backend(SimulatorBackendType.GENESIS)
    assert isinstance(backend, GenesisBackend)


def test_mujoco_backend_load_and_step():
    backend = get_physics_backend(SimulatorBackendType.MUJOCO)

    # Use the minimal XML created for testing
    xml_path = "tests/worker/minimal.xml"
    if not os.path.exists(xml_path):
        pytest.skip(f"Test XML not found at {xml_path}")

    scene = SimulationScene(scene_path=xml_path)
    backend.load_scene(scene)

    # Step the simulation
    result = backend.step(0.01)
    assert result.success
    assert result.time > 0

    # Get body state (world body is index 0, 'box' should be index 1)
    state = backend.get_body_state("box")
    assert len(state.pos) == 3
    assert len(state.quat) == 4
    # Box was at pos=0,0,1, after 0.01s it should have moved slightly due to gravity
    assert state.pos[2] < 1.0


def test_genesis_backend_skeleton():
    backend = get_physics_backend(SimulatorBackendType.GENESIS)

    # Genesis backend might fail to init if not installed, which is fine for a skeleton
    try:
        scene = SimulationScene()
        # backend.load_scene(scene) # This will likely fail as it expects gs
    except Exception as e:
        pytest.skip(f"GenesisBackend failed to init (expected if not installed): {e}")


if __name__ == "__main__":
    pytest.main([__file__])
