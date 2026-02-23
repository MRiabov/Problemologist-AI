import pytest

from shared.simulation.schemas import SimulatorBackendType
from worker_heavy.simulation.factory import get_physics_backend, get_simulation_builder
from worker_heavy.simulation.genesis_backend import GenesisBackend
from worker_heavy.simulation.mujoco_backend import MuJoCoBackend


def test_get_physics_backend():
    mujoco = get_physics_backend(SimulatorBackendType.GENESIS)
    assert isinstance(mujoco, MuJoCoBackend)

    genesis = get_physics_backend(SimulatorBackendType.GENESIS)
    assert isinstance(genesis, GenesisBackend)


def test_get_physics_backend_invalid():
    with pytest.raises(ValueError):
        get_physics_backend("invalid_backend")


def test_get_simulation_builder(tmp_path):
    from worker_heavy.simulation.builder import (
        GenesisSimulationBuilder,
        MuJoCoSimulationBuilder,
    )

    mj_builder = get_simulation_builder(tmp_path, SimulatorBackendType.GENESIS)
    assert isinstance(mj_builder, MuJoCoSimulationBuilder)

    gn_builder = get_simulation_builder(tmp_path, SimulatorBackendType.GENESIS)
    assert isinstance(gn_builder, GenesisSimulationBuilder)
