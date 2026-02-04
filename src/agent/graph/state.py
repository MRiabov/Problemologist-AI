from typing import Annotated, Any, TypedDict
from typing_extensions import NotRequired

from langchain_core.messages import AnyMessage
from langgraph.graph.message import add_messages


class AgentState(TypedDict):
    """
    The state of the agent as it flows through the LangGraph.
    """

    # Standard LangGraph history: messages are merged using add_messages
    messages: Annotated[list[AnyMessage], add_messages]

    # The current high-level implementation plan
    plan: NotRequired[str]

    # Tracking the number of steps taken to prevent infinite loops
    step_count: NotRequired[int]

    # The reasoning from the coder/actor
    coder_reasoning: NotRequired[str]

    # History of attempts (code and errors)
    full_history: NotRequired[list[dict[str, Any]]]

    # Storage for intermediate data, node-specific state, or temporary variables
    scratchpad: NotRequired[dict[str, Any]]

    # Identifier for the associated ToolRuntime (for stateless tools)
    runtime_id: NotRequired[str]

    # Runtime configuration updates (e.g. system prompt overrides)
    runtime_config: NotRequired[dict[str, Any]]

    # Structured completion signal for routing (avoids magic string matching)
    task_complete: NotRequired[bool]
