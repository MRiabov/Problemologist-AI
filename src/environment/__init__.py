from gymnasium.envs.registration import register
from .core import CADEnv

register(
    id="CADEnv-v0",
    entry_point="src.environment.core:CADEnv",
)

__all__ = ["CADEnv"]
