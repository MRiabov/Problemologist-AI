from pathlib import Path

import build123d as bd
from src.simulation_engine.builder import SceneCompiler


def to_mjcf(
    env_compound: bd.Compound | bd.Shape | list,
    agent_compound: bd.Compound | bd.Shape | list | None = None,
    agent_joints: list | None = None,
    env_labels: list[str] | None = None,
    agent_labels: list[str] | None = None,
) -> str:
    from build123d import Compound, Shape

    # _ASSET_DIR should be set by the runner
    asset_dir = globals().get("_ASSET_DIR", ".agent_storage/temp_assets")
    target_dir = Path("/workspace") / asset_dir
    target_dir.mkdir(parents=True, exist_ok=True)

    compiler = SceneCompiler(asset_dir=str(target_dir))

    def ensure_compound(obj):
        if obj is None:
            return None
        if isinstance(obj, list):
            return Compound(children=obj)
        if isinstance(obj, (Compound, Shape)):
            return obj
        return Compound(children=[obj])

    env_c = ensure_compound(env_compound)
    agent_c = ensure_compound(agent_compound)
    return compiler.compile(env_c, agent_c, agent_joints, env_labels, agent_labels)
