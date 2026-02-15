from enum import Enum


class SimulatorBackendType(str, Enum):
    MUJOCO = "mujoco"  # Rigid-body only, fast, no FEM/fluids
    GENESIS = "genesis"  # FEM + MPM fluids, requires more compute
