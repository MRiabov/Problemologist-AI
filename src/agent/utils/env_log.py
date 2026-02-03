from typing import Any, Optional

def log_to_env(
    content: str,
    type: str = "thought",
    agent_role: Optional[str] = None,
    metadata: Optional[dict[str, Any]] = None,
    runtime: Optional[Any] = None,
):
    """Logs a message to the provided runtime/environment."""
    if runtime and hasattr(runtime, "log_message"):
        runtime.log_message(
            content=content, type=type, agent_role=agent_role, metadata=metadata
        )
