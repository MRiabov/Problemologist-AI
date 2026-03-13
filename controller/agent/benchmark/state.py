import uuid
from typing import Annotated

from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages
from pydantic import BaseModel, Field

from shared.enums import ReviewDecision
from shared.simulation.schemas import RandomizationStrategy, ValidationResult

from .models import GenerationSession


class BenchmarkGeneratorState(BaseModel):
    session: GenerationSession
    episode_id: str = Field(
        default_factory=lambda: str(uuid.uuid4())
    )  # UUID string for database recording
    current_script: str = ""  # The Python code being generated
    mjcf_content: str = ""  # The generated MuJoCo XML
    simulation_result: ValidationResult | None = None  # Result of the last check
    review_feedback: str | None = None  # Comments from reviewer
    review_decision: ReviewDecision | None = None  # Structured decision from reviewer
    start_node: str | None = None  # Optional explicit graph entry override
    hard_fail_code: str | None = None  # Structured hard-fail code from execution limits
    entry_validation_rejected: bool = False  # Guard rejected node entry this turn
    entry_validation_terminal: bool = False  # Guard rejection terminated workflow
    entry_validation_reason_code: str | None = None
    entry_validation_target_node: str | None = None
    entry_validation_disposition: str | None = None
    entry_validation_reroute_target: str | None = None
    entry_validation_errors: list[dict[str, str | None]] = Field(default_factory=list)
    entry_validation_trace_emitted: bool = False
    reviewer_handoff_block_count: int = 0  # Consecutive handoff invariant failures
    review_round: int = 0  # Current review iteration
    turn_count: int = 0  # Total graph turns across planner/coder/reviewer loops
    journal: str = ""  # Reasoning journal
    plan: RandomizationStrategy | None = None  # The randomization strategy
    messages: Annotated[list[BaseMessage], add_messages] = Field(default_factory=list)
