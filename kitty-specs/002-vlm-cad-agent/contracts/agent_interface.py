from typing import Any, Literal

from pydantic import BaseModel, Field


class ToolCall(BaseModel):
    tool_name: str
    arguments: dict[str, Any]


class AgentThought(BaseModel):
    """
    Structured output from the Agent's reasoning step.
    Enforces Chain-of-Thought (CoT) before action.
    """

    thought_process: str = Field(
        ..., description="Key reasoning steps, alternatives considered"
    )
    plan_update: str | None = Field(
        None, description="Updates to the task list if needed"
    )
    action: ToolCall = Field(..., description="The tool to execute")


class AgentObjective(BaseModel):
    """
    Input to the Agent.
    """

    id: str
    description: str
    constraints: list[str] = []
    max_unit_cost: float = float("inf")
    target_quantity: int = 1
    max_steps: int = 20


class AgentResult(BaseModel):
    """
    Final output after task completion or failure.
    """

    status: Literal["success", "failure"]
    solution_code: str | None
    artifact_paths: list[str]
    summary: str
    total_cost_usd: float
    unit_cost_usd: float
