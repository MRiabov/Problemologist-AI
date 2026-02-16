import pytest
from shared.simulation.backends import SimulatorBackendType
from worker.simulation.factory import get_physics_backend, get_simulation_builder
from worker.simulation.mujoco_backend import MuJoCoBackend
from worker.simulation.genesis_backend import GenesisBackend
from pathlib import Path


def test_get_physics_backend():
    mujoco = get_physics_backend(SimulatorBackendType.MUJOCO)
    assert isinstance(mujoco, MuJoCoBackend)

    genesis = get_physics_backend(SimulatorBackendType.GENESIS)
    assert isinstance(genesis, GenesisBackend)


def test_get_physics_backend_invalid():
    with pytest.raises(ValueError):
        get_physics_backend("invalid_backend")


def test_get_simulation_builder(tmp_path):
    from worker.simulation.builder import (
        GenesisSimulationBuilder,
        MuJoCoSimulationBuilder,
    )

    mj_builder = get_simulation_builder(tmp_path, SimulatorBackendType.MUJOCO)
    assert isinstance(mj_builder, MuJoCoSimulationBuilder)

    gn_builder = get_simulation_builder(tmp_path, SimulatorBackendType.GENESIS)
    assert isinstance(gn_builder, GenesisSimulationBuilder)
