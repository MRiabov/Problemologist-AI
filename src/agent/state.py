from typing import List, TypedDict
from langchain_core.messages import BaseMessage

class AgentState(TypedDict):
    """The state of the agent passed between nodes in the graph."""
    messages: List[BaseMessage]
    task: str
    plan: str
    todo: str
    current_step: str
    journal: str
    iteration: int
