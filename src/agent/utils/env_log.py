from typing import Any, Optional

from src.agent.tools.env_adapter import get_active_env


def log_to_env(
    content: str,
    type: str = "thought",
    agent_role: Optional[str] = None,
    metadata: Optional[dict[str, Any]] = None,
):
    """Logs a message to the active CADEnv if it exists."""
    env = get_active_env()
    if env and hasattr(env, "log_message"):
        env.log_message(
            content=content, type=type, agent_role=agent_role, metadata=metadata
        )
