import uuid
from collections.abc import Callable

import dspy
import structlog

from controller.agent.config import settings
from controller.agent.dspy_utils import WorkerInterpreter
from controller.agent.nodes.coder import CoderSignature
from controller.agent.prompt_manager import PromptManager
from controller.agent.tools import get_engineer_tools
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.enums import AgentName

logger = structlog.get_logger(__name__)


from controller.agent.benchmark.nodes import (
    BenchmarkCoderSignature,
    BenchmarkPlannerSignature,
)
from controller.agent.benchmark.tools import (
    get_benchmark_planner_tools,
    get_benchmark_tools,
)


class AgentModule(dspy.Module):
    def __init__(self, agent_name: AgentName, session_id: str | None = None):
        super().__init__()
        self.agent_name = agent_name
        # Setup worker context for optimization
        self.session_id = session_id or f"opt-{agent_name}-{uuid.uuid4().hex[:8]}"

        self.worker_client = WorkerClient(
            base_url=settings.worker_light_url,
            session_id=self.session_id,
            heavy_url=settings.worker_heavy_url,
        )
        self.fs = RemoteFilesystemMiddleware(self.worker_client, agent_role=agent_name)
        self.interpreter = WorkerInterpreter(
            worker_client=self.worker_client,
            session_id=self.session_id,
            episode_id=self.session_id,
        )

        # Configure Agent based on name
        self.pm = PromptManager()

        if agent_name == AgentName.BENCHMARK_PLANNER:
            self.signature = BenchmarkPlannerSignature.with_instructions(
                self.pm.render(AgentName.BENCHMARK_PLANNER, backend_family="api_based")
            )
            self.tools = get_benchmark_planner_tools(self.fs, self.session_id)
        elif agent_name == AgentName.BENCHMARK_CODER:
            self.signature = BenchmarkCoderSignature.with_instructions(
                self.pm.render(AgentName.BENCHMARK_CODER, backend_family="api_based")
            )
            self.tools = get_benchmark_tools(self.fs, self.session_id)
        elif agent_name == AgentName.ENGINEER_CODER:
            self.signature = CoderSignature.with_instructions(
                self.pm.render(AgentName.ENGINEER_CODER, backend_family="api_based")
            )
            self.tools = get_engineer_tools(self.fs, self.session_id)
        else:
            logger.error(
                "unknown_agent_type_rejected",
                agent_name=agent_name,
                session_id=self.session_id,
            )
            msg = f"Unsupported agent type: {agent_name}"
            raise ValueError(msg)

        # Tools are plain callables in this runtime; keep compatibility with
        # wrapper objects only when they expose a callable .func.
        self.tool_fns: dict[str, Callable] = {}
        for tool in self.tools:
            if callable(tool):
                name = getattr(tool, "__name__", str(tool))
                self.tool_fns[name] = tool
                continue

            func = getattr(tool, "func", None)
            if callable(func):
                name = getattr(tool, "name", getattr(func, "__name__", str(func)))
                self.tool_fns[name] = func

        if not self.tool_fns:
            logger.warning(
                "agent_module_no_tools_resolved",
                agent_name=agent_name,
                session_id=self.session_id,
            )

        self.program = dspy.ReAct(
            self.signature,
            tools=list(self.tool_fns.values()),
        )
        logger.info(
            "agent_module_initialized",
            agent_name=agent_name,
            session_id=self.session_id,
        )

    def forward(self, **kwargs):
        # Dynamically map input arguments based on the agent type
        if self.agent_name == AgentName.BENCHMARK_PLANNER:
            # Map dataset keys to signature keys
            # Dataset: task, expected_criteria, objectives
            # Signature: prompt, history, review_feedback
            return self.program(
                prompt=kwargs.get("task", ""),
                history="",  # No history in optimization yet
                review_feedback="",  # No feedback yet
            )
        if self.agent_name == AgentName.BENCHMARK_CODER:
            # Dataset keywords might need adjustment for coder
            return self.program(
                prompt=kwargs.get("task", ""),
                plan=kwargs.get("plan", ""),
                benchmark_definition_yaml=str(kwargs.get("objectives", {})),
                review_feedback="",
                validation_logs="",
            )
        # Default Coder/Engineer behavior
        return self.program(
            current_step=kwargs.get("current_step", kwargs.get("task", "")),
            plan=kwargs.get("plan", ""),
            todo=kwargs.get("todo", ""),
            feedback=kwargs.get("feedback", ""),
        )


def build_eval_program(agent_name: AgentName) -> dspy.Module:
    """
    Factory function to build a DSPy Module for agent evaluation/optimization.
    Handles all the necessary wiring of tools, worker client, and interpreter.
    """
    return AgentModule(agent_name)
