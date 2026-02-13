from pathlib import Path
from shared.simulation.backends import PhysicsBackend, SimulatorBackendType
from worker.simulation.mujoco_backend import MuJoCoBackend
from worker.simulation.genesis_backend import GenesisBackend


def get_physics_backend(backend_type: SimulatorBackendType) -> PhysicsBackend:
    if backend_type == SimulatorBackendType.MUJOCO:
        return MuJoCoBackend()
    elif backend_type == SimulatorBackendType.GENESIS:
        return GenesisBackend()
    else:
        raise ValueError(f"Unknown backend type: {backend_type}")


def get_simulation_builder(
    output_dir: Path,
    backend_type: SimulatorBackendType = SimulatorBackendType.MUJOCO,
    use_vhacd: bool = False,
):
    from worker.simulation.builder import (
        GenesisSimulationBuilder,
        MuJoCoSimulationBuilder,
    )

    if backend_type == SimulatorBackendType.MUJOCO:
        return MuJoCoSimulationBuilder(output_dir, use_vhacd)
    elif backend_type == SimulatorBackendType.GENESIS:
        return GenesisSimulationBuilder(output_dir, use_vhacd)
    else:
        raise ValueError(f"Unknown backend type: {backend_type}")
