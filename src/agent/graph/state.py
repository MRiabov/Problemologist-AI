from typing import Annotated, TypedDict, Any
from langgraph.graph.message import add_messages
from langchain_core.messages import AnyMessage


class AgentState(TypedDict):
    """
    The state of the agent as it flows through the LangGraph.
    """

    # Standard LangGraph history: messages are merged using add_messages
    messages: Annotated[list[AnyMessage], add_messages]

    # The current high-level implementation plan
    plan: str

    # Tracking the number of steps taken to prevent infinite loops
    step_count: int

    # Storage for intermediate data, node-specific state, or temporary variables
    scratchpad: dict[str, Any]
