from dataclasses import dataclass
from typing import Any

from build123d import Compound, Part

from shared.type_checking import type_check


@type_check
@dataclass
class SimulationScene:
    """
    Represents a simulation scene with geometry and joint configuration.
    """

    assembly: Compound
    agent_joints: list[dict[str, Any]] | None = None


def _ensure_list(obj: Any) -> list[Any]:
    if obj is None:
        return []
    if isinstance(obj, list):
        return obj
    return [obj]


def to_mjcf(
    env_compound: list[Part | Compound] | Compound | Part | None = None,
    agent_compound: list[Part | Compound] | Compound | Part | None = None,
    agent_joints: list[dict[str, Any]] | None = None,
    env_labels: list[str] | None = None,
    agent_labels: list[str] | None = None,
) -> SimulationScene:
    """
    Constructs a SimulationScene from environment and agent components.

    Args:
        env_compound: Static environment parts (walls, goals, obstacles).
        agent_compound: Dynamic agent parts (robot bodies, manipulated objects).
        agent_joints: Configuration for joints connecting agent parts.
        env_labels: Labels for environment parts (must match env_compound count).
        agent_labels: Labels for agent parts (must match agent_compound count).

    Returns:
        A SimulationScene object ready for simulation.
    """
    env_parts = _ensure_list(env_compound)
    agent_parts = _ensure_list(agent_compound)
    env_labels = _ensure_list(env_labels)
    agent_labels = _ensure_list(agent_labels)

    # Validate label counts
    if len(env_labels) > 0 and len(env_labels) != len(env_parts):
        raise ValueError(
            f"env_labels count ({len(env_labels)}) does not match env_parts count ({len(env_parts)})"
        )
    if len(agent_labels) > 0 and len(agent_labels) != len(agent_parts):
        raise ValueError(
            f"agent_labels count ({len(agent_labels)}) does not match agent_parts count ({len(agent_parts)})"
        )

    # Assign labels to parts
    # Note: We modify the objects in place (adding .label attribute)
    # If they are Compound wrappers, this attribute is on the Python object.
    all_parts = []

    for i, part in enumerate(env_parts):
        label = env_labels[i] if i < len(env_labels) else f"env_part_{i}"
        part.label = label
        all_parts.append(part)

    for i, part in enumerate(agent_parts):
        label = agent_labels[i] if i < len(agent_labels) else f"agent_part_{i}"
        part.label = label
        all_parts.append(part)

    # Create a single assembly compound
    # We use Compound(children=all_parts)
    assembly = Compound(children=all_parts)

    return SimulationScene(assembly=assembly, agent_joints=agent_joints)
