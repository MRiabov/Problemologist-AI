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
