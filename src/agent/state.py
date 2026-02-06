from langchain_core.messages import BaseMessage
from pydantic import BaseModel, Field


class AgentState(BaseModel):
    """The state of the agent passed between nodes in the graph."""

    messages: list[BaseMessage] = Field(default_factory=list)
    task: str = ""
    plan: str = ""
    todo: str = ""
    current_step: str = ""
    journal: str = ""
    iteration: int = 0
    status: str = ""
    feedback: str = ""
