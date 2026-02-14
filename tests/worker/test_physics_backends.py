import os
from pathlib import Path

import pytest

from shared.simulation.backends import SimulatorBackendType
from worker.simulation.builder import GenesisSimulationBuilder, MuJoCoSimulationBuilder
from worker.simulation.factory import get_physics_backend, get_simulation_builder
from worker.simulation.genesis_backend import GenesisBackend
from worker.simulation.loop import SimulationLoop
from worker.simulation.mujoco_backend import MuJoCoBackend


def test_simulation_loop_with_mujoco():
    # Use the minimal XML created for testing
    xml_path = "tests/worker/minimal.xml"
    if not os.path.exists(xml_path):
        # Recreate minimal.xml if it was also deleted
        with open(xml_path, "w") as f:
            f.write("""<mujoco model="test_model">
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="10 10 0.1" rgba=".8 .8 .8 1"/>
        <body name="box" pos="0 0 1">
            <joint name="free" type="free"/>
            <geom name="box_geom" type="box" size="0.1 0.1 0.1" rgba="0 1 0 1" mass="1"/>
        </body>
    </worldbody>
</mujoco>""")

    loop = SimulationLoop(xml_path, backend_type=SimulatorBackendType.MUJOCO)

    # Run a few steps
    metrics = loop.step(control_inputs={}, duration=0.1)

    assert metrics.total_time >= 0.1
    assert metrics.success == False  # No goal achieved in minimal.xml
    assert metrics.fail_reason == None  # Finished normally


def test_simulation_builder_factory():
    output_dir = Path("test_renders")

    builder_mujoco = get_simulation_builder(
        output_dir, backend_type=SimulatorBackendType.MUJOCO
    )
    assert isinstance(builder_mujoco, MuJoCoSimulationBuilder)

    builder_genesis = get_simulation_builder(
        output_dir, backend_type=SimulatorBackendType.GENESIS
    )
    assert isinstance(builder_genesis, GenesisSimulationBuilder)


def test_factory_mujoco():
    backend = get_physics_backend(SimulatorBackendType.MUJOCO)
    assert isinstance(backend, MuJoCoBackend)


def test_factory_genesis():
    backend = get_physics_backend(SimulatorBackendType.GENESIS)
    assert isinstance(backend, GenesisBackend)


if __name__ == "__main__":
    pytest.main([__file__])
