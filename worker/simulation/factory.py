from pathlib import Path

from shared.simulation.backends import PhysicsBackend
from shared.simulation.schemas import SimulatorBackendType


def get_physics_backend(backend_type: SimulatorBackendType) -> PhysicsBackend:
    if backend_type == SimulatorBackendType.MUJOCO:
        from worker.simulation.mujoco_backend import MuJoCoBackend

        return MuJoCoBackend()
    if backend_type == SimulatorBackendType.GENESIS:
        from worker.simulation.genesis_backend import GenesisBackend

        return GenesisBackend()
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
    if backend_type == SimulatorBackendType.GENESIS:
        return GenesisSimulationBuilder(output_dir, use_vhacd)
    raise ValueError(f"Unknown backend type: {backend_type}")
