from typing import Annotated

from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages
from pydantic import BaseModel, Field
from shared.simulation.schemas import ValidationResult, RandomizationStrategy

from .models import GenerationSession


class BenchmarkGeneratorState(BaseModel):
    session: GenerationSession
    current_script: str = ""  # The Python code being generated
    mjcf_content: str = ""  # The generated MuJoCo XML
    simulation_result: ValidationResult | None = None  # Result of the last check
    review_feedback: str | None = None  # Comments from reviewer
    review_round: int = 0  # Current review iteration
    plan: RandomizationStrategy | None = None  # The randomization strategy
    messages: Annotated[list[BaseMessage], add_messages] = Field(default_factory=list)
