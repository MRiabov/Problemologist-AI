from typing import List, Optional, Literal, Dict, Any
from pydantic import BaseModel, Field


class ToolCall(BaseModel):
    tool_name: str
    arguments: Dict[str, Any]


class AgentThought(BaseModel):
    """
    Structured output from the Agent's reasoning step.
    Enforces Chain-of-Thought (CoT) before action.
    """

    thought_process: str = Field(
        ..., description="Key reasoning steps, alternatives considered"
    )
    plan_update: Optional[str] = Field(
        None, description="Updates to the task list if needed"
    )
    action: ToolCall = Field(..., description="The tool to execute")


class AgentObjective(BaseModel):
    """
    Input to the Agent.
    """

    id: str
    description: str
    constraints: List[str] = []
    max_steps: int = 20


class AgentResult(BaseModel):
    """
    Final output after task completion or failure.
    """

    status: Literal["success", "failure"]
    solution_code: Optional[str]
    artifact_paths: List[str]
    summary: str
    cost_usd: float
