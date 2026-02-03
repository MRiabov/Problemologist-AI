from typing import Any

from src.agent.tools.env_adapter import get_runtime


def log_to_env(
    content: str,
    msg_type: str = "thought",
    agent_role: str | None = None,
    metadata: dict[str, Any] | None = None,
):
    """Logs a message to the active ToolRuntime if it exists."""
    env = get_runtime()
    if env and hasattr(env, "log_message"):
        env.log_message(
            content=content, msg_type=msg_type, agent_role=agent_role, metadata=metadata
        )
