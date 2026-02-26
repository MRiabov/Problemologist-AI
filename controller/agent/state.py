from enum import StrEnum
from typing import Annotated, Any

from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages
from pydantic import BaseModel, Field, StrictInt, StrictStr


class AgentStatus(StrEnum):
    IDLE = "idle"
    THINKING = "thinking"
    EXECUTING = "executing"
    WAITING = "waiting"
    COMPLETED = "completed"
    FAILED = "failed"
    APPROVED = "approved"
    PLAN_REJECTED = "plan_rejected"
    CODE_REJECTED = "code_rejected"


class AgentState(BaseModel):
    """The state of the agent passed between nodes in the graph."""

    messages: Annotated[list[BaseMessage], add_messages] = Field(default_factory=list)
    task: StrictStr = ""
    plan: StrictStr = ""
    todo: StrictStr = ""
    current_step: StrictStr = ""
    journal: StrictStr = ""
    iteration: StrictInt = 0
    status: AgentStatus = AgentStatus.IDLE
    feedback: StrictStr = ""
    session_id: StrictStr = ""
    context: Any = Field(default=None, exclude=True)
    episode_id: StrictStr = ""
    best_cost: float | None = None
    best_weight_g: float | None = None
    turn_count: StrictInt = 0
