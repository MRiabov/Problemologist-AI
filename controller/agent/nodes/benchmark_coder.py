from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState
from controller.agent.tools import get_benchmark_tools
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class BenchmarkCoderSignature(dspy.Signature):
    """
    Benchmark Coder node: Implements the benchmark geometry in 'script.py'.
    You must follow the plan and TODO list to create the CAD model.
    When done, use SUBMIT to provide a summary of your implementation.
    """

    task = dspy.InputField()
    current_step = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    objectives = dspy.InputField()
    journal = dspy.OutputField(desc="A summary of the implementation work done")


@type_check
class BenchmarkCoderNode(BaseNode):
    async def __call__(self, state: AgentState) -> AgentState:
        todo = state.todo
        current_step = self._get_next_step(todo)
        if not current_step:
            return state.model_copy(
                update={"journal": state.journal + "\nNo more steps in benchmark TODO."}
            )

        objectives = "# No objectives.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("objectives.yaml"):
                objectives = await self.ctx.worker_client.read_file("objectives.yaml")

        inputs = {
            "task": state.task,
            "current_step": current_step,
            "plan": state.plan,
            "todo": todo,
            "objectives": objectives,
        }
        validate_files = ["plan.md", "todo.md", "objectives.yaml", "script.py"]

        prediction, _, journal_entry = await self._run_program(
            dspy.ReAct,
            BenchmarkCoderSignature,
            state,
            inputs,
            get_benchmark_tools,
            validate_files,
            "benchmark_coder",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "journal": state.journal + journal_entry,
                    "iteration": state.iteration + 1,
                    "turn_count": state.turn_count + 1,
                }
            )

        summary = getattr(prediction, "journal", f"Successfully implemented benchmark step: {current_step}")
        journal_entry += f"\n[Benchmark Coder] {summary}"
        new_todo = todo.replace(f"- [ ] {current_step}", f"- [x] {current_step}")

        return state.model_copy(
            update={
                "todo": new_todo,
                "journal": state.journal + journal_entry,
                "current_step": current_step,
                "messages": state.messages + [AIMessage(content=f"Benchmark Coder summary: {summary}")],
                "turn_count": state.turn_count + 1,
            }
        )

    def _get_next_step(self, todo: str) -> str | None:
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                return line.strip().replace("- [ ]", "").strip()
        return None


@type_check
async def benchmark_coder_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.spec_001_api_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role="benchmark_cad_coder",
    )
    node = BenchmarkCoderNode(context=ctx)
    return await node(state)
