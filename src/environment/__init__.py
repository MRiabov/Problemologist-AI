import sys

print(f"DEBUG: environment.__init__ sys.path: {sys.path}")
try:
    import gymnasium

    print(f"DEBUG: gymnasium found at: {gymnasium.__file__}")
except ImportError as e:
    print(f"DEBUG: gymnasium NOT found: {e}")
    raise e

from gymnasium.envs.registration import register
from .core import CADEnv

register(
    id="CADEnv-v0",
    entry_point="src.environment.core:CADEnv",
)

__all__ = ["CADEnv"]
