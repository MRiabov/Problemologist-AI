from enum import Enum, StrEnum
from langchain_core.messages import BaseMessage
from pydantic import BaseModel, Field, StrictStr, StrictInt

from src.shared.type_checking import type_check


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


@type_check
class AgentState(BaseModel):
    """The state of the agent passed between nodes in the graph."""

    messages: list[BaseMessage] = Field(default_factory=list)
    task: StrictStr = ""
    plan: StrictStr = ""
    todo: StrictStr = ""
    current_step: StrictStr = ""
    journal: StrictStr = ""
    iteration: StrictInt = 0
    status: AgentStatus = AgentStatus.IDLE
    feedback: StrictStr = ""
