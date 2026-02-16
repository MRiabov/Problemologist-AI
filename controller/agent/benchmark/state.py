from typing import Annotated, TypedDict

from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages
from shared.simulation.schemas import ValidationResult, RandomizationStrategy

from .models import GenerationSession


class BenchmarkGeneratorState(TypedDict):
    session: GenerationSession

    current_script: str  # The Python code being generated

    mjcf_content: str  # The generated MuJoCo XML

    simulation_result: ValidationResult | None  # Result of the last check

    review_feedback: str | None  # Comments from reviewer

    review_round: int  # Current review iteration

    plan: RandomizationStrategy | None  # The randomization strategy

    messages: Annotated[list[BaseMessage], add_messages]  # Chat history
