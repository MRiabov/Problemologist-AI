from typing import Any, TypedDict

from langchain_core.messages import BaseMessage

from .models import GenerationSession


class ValidationResult(TypedDict):
    valid: bool
    cost: float
    logs: list[str]
    render_paths: list[str]


class BenchmarkGeneratorState(TypedDict):
    session: GenerationSession

    current_script: str  # The Python code being generated

    mjcf_content: str  # The generated MuJoCo XML

    simulation_result: ValidationResult | None  # Result of the last check

    review_feedback: str | None  # Comments from reviewer

    plan: dict[str, Any] | None  # The randomization strategy

    messages: list[BaseMessage]  # Chat history
