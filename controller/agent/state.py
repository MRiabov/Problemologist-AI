import uuid
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
    start_node: StrictStr | None = None
    plan: StrictStr = ""
    todo: StrictStr = ""
    current_step: StrictStr = ""
    journal: StrictStr = ""
    iteration: StrictInt = 0
    status: AgentStatus = AgentStatus.IDLE
    feedback: StrictStr = ""
    session_id: StrictStr = ""
    worker_session_id: StrictStr | None = None
    episode_id: StrictStr = Field(default_factory=lambda: str(uuid.uuid4()))
    initial_script_sha256: StrictStr | None = None
    best_cost: float | None = None
    best_weight_g: float | None = None
    turn_count: StrictInt = 0
    entry_validation_rejected: bool = False
    entry_validation_terminal: bool = False
    entry_validation_reason_code: StrictStr | None = None
    entry_validation_target_node: StrictStr | None = None
    entry_validation_disposition: StrictStr | None = None
    entry_validation_reroute_target: StrictStr | None = None
    entry_validation_errors: list[dict[str, str | None]] = Field(default_factory=list)
    entry_validation_trace_emitted: bool = False
    worker_client: Any = None
    fs: Any = None

    model_config = {"arbitrary_types_allowed": True}
