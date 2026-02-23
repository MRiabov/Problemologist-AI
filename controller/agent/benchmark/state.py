from typing import Annotated

from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages
from pydantic import BaseModel, Field

from shared.enums import ReviewDecision
from shared.simulation.schemas import RandomizationStrategy, ValidationResult

from .models import GenerationSession


class BenchmarkGeneratorState(BaseModel):
    session: GenerationSession
    current_script: str = ""  # The Python code being generated
    mjcf_content: str = ""  # The generated MuJoCo XML
    simulation_result: ValidationResult | None = None  # Result of the last check
    review_feedback: str | None = None  # Comments from reviewer
    review_decision: ReviewDecision | None = None  # Structured decision from reviewer
    review_round: int = 0  # Current review iteration
    journal: str = ""  # Reasoning journal
    plan: RandomizationStrategy | None = None  # The randomization strategy
    messages: Annotated[list[BaseMessage], add_messages] = Field(default_factory=list)
