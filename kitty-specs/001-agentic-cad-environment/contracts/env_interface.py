from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass
class Observation:
    """Agent's view of the world."""

    file_content: str
    console_output: str
    last_render_path: str | None
    task_description: str


@dataclass
class StepResult:
    """Result of an action."""

    observation: Observation
    reward: float
    done: bool
    info: dict[str, Any]


class AgenticCADEnv(ABC):
    """
    The main environment interface.
    Follows a Gymnasium-like pattern but adapted for text-based tool interaction.
    """

    @abstractmethod
    def reset(self, problem_id: str) -> Observation:
        """
        Resets the environment to a new problem scenario.
        Args:
            problem_id: ID of the scenario to load.
        Returns:
            Initial observation.
        """
        pass

    @abstractmethod
    def step(self, tool_name: str, **tool_args) -> StepResult:
        """
        Executes a tool call in the environment.
        Args:
            tool_name: Name of the tool to execute.
            tool_args: Arguments for the tool.
        Returns:
            StepResult containing new observation, etc.
        """
        pass

    @abstractmethod
    def render(self) -> str | None:
        """
        Returns path to a visual render of the current state.
        """
        pass

    @abstractmethod
    def close(self):
        """
        Cleans up resources (physics sim, DB connections).
        """
        pass
