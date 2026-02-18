import uuid

import dspy
import structlog

from controller.agent.config import settings
from controller.agent.dspy_utils import WorkerInterpreter
from controller.agent.nodes.coder import CoderSignature
from controller.agent.prompt_manager import PromptManager
from controller.agent.tools import get_engineer_tools
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware

logger = structlog.get_logger(__name__)


from controller.agent.benchmark.nodes import (
    BenchmarkCoderSignature,
    BenchmarkPlannerSignature,
)
from controller.agent.benchmark.tools import get_benchmark_tools


class AgentModule(dspy.Module):
    def __init__(self, agent_name: str, session_id: str | None = None):
        super().__init__()
        self.agent_name = agent_name
        # Setup worker context for optimization
        self.session_id = session_id or f"opt-{agent_name}-{uuid.uuid4().hex[:8]}"

        self.worker_client = WorkerClient(
            base_url=settings.spec_001_api_url, session_id=self.session_id
        )
        self.fs = RemoteFilesystemMiddleware(self.worker_client)
        self.interpreter = WorkerInterpreter(
            worker_client=self.worker_client, session_id=self.session_id
        )

        # Configure Agent based on name
        self.pm = PromptManager()

        if agent_name == "benchmark_planner":
            self.signature = BenchmarkPlannerSignature.with_instructions(
                self.pm.render("benchmark_planner")
            )
            self.tools = get_benchmark_tools(self.fs, self.session_id)
        elif agent_name == "benchmark_coder":
            self.signature = BenchmarkCoderSignature.with_instructions(
                self.pm.render("benchmark_coder")
            )
            self.tools = get_benchmark_tools(self.fs, self.session_id)
        elif agent_name == "coder" or agent_name == "engineer":
            # "engineer" template corresponds to the coder role in prompts.yaml
            self.signature = CoderSignature.with_instructions(
                self.pm.render("engineer")
            )
            self.tools = get_engineer_tools(self.fs, self.session_id)
        else:
            # Default fallback
            logger.warning("unknown_agent_type_fallback_coder", agent_name=agent_name)
            self.signature = CoderSignature.with_instructions(
                self.pm.render("engineer")
            )
            self.tools = get_engineer_tools(self.fs, self.session_id)

        # Extract raw functions from tools for DSPy
        self.tool_fns = {}
        for t in self.tools:
            func = getattr(t, "func", getattr(t, "_run", None))
            if func:
                self.tool_fns[t.name] = func

        self.program = dspy.CodeAct(
            self.signature,
            tools=list(self.tool_fns.values()),
            interpreter=self.interpreter,
        )
        logger.info(
            "agent_module_initialized",
            agent_name=agent_name,
            session_id=self.session_id,
        )

    def forward(self, **kwargs):
        # Dynamically map input arguments based on the agent type
        if self.agent_name == "benchmark_planner":
            # Map dataset keys to signature keys
            # Dataset: task, expected_criteria, objectives
            # Signature: prompt, history, review_feedback
            return self.program(
                prompt=kwargs.get("task", ""),
                history="",  # No history in optimization yet
                review_feedback="",  # No feedback yet
            )
        if self.agent_name == "benchmark_coder":
            # Dataset keywords might need adjustment for coder
            return self.program(
                prompt=kwargs.get("task", ""),
                plan=kwargs.get("plan", ""),
                objectives_yaml=str(kwargs.get("objectives", {})),
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


def build_eval_program(agent_name: str) -> dspy.Module:
    """
    Factory function to build a DSPy Module for agent evaluation/optimization.
    Handles all the necessary wiring of tools, worker client, and interpreter.
    """
    return AgentModule(agent_name)
