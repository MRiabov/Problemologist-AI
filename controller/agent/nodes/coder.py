import re
from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState
from controller.agent.tools import get_engineer_tools
from shared.enums import AgentName
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class CoderSignature(dspy.Signature):
    """
    Coder node: Picks a task from TODO, writes code, executes it, and fixes errors.
    You must use the provided tools to implement the current step in 'script.py'.
    You also receive benchmark-owned read-only benchmark_assembly_definition.yaml
    handoff context copied into this workspace; benchmark caps must be read from
    benchmark_definition.yaml rather than copied from the benchmark assembly.
    When done, use FINISH to provide a summary of your work. The controller
    runtime will materialize the latest execution-review handoff after the
    implementation pass is complete.
    """

    task = dspy.InputField()
    current_step = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    assembly_definition = dspy.InputField()
    objectives = dspy.InputField()
    benchmark_assembly_definition = dspy.InputField()
    steer_context = dspy.InputField(default="")
    feedback = dspy.InputField(desc="Feedback from previous review steps", default="")
    journal = dspy.OutputField(
        desc="A summary of the implementation done for this step"
    )


@type_check
class CoderNode(BaseNode):
    """
    Coder node: Picks a task from TODO, writes code, executes it, and fixes errors.
    Refactored to use DSPy ReAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the coder node logic."""
        # T010: Find next active item in TODO
        todo = state.todo
        if not todo.strip():
            with suppress(Exception):
                todo = await self._read_optional_workspace_file("todo.md", todo)
        current_step = self._get_next_step(todo)
        no_step_mode = current_step is None
        if no_step_mode:
            current_step = "Finalize implementation and hand off for review."

        # WP04: Extract steerability context
        steer_context = await self._get_steer_context(state.messages)

        # Read objectives and assembly_definition for context
        objectives = "# No benchmark_definition.yaml found."
        with suppress(Exception):
            objectives = await self._read_optional_workspace_file(
                "benchmark_definition.yaml", objectives
            )

        assembly_definition = "# No assembly_definition.yaml found."
        with suppress(Exception):
            assembly_definition = await self._read_optional_workspace_file(
                "assembly_definition.yaml", assembly_definition
            )
        benchmark_assembly_definition = await self._read_required_workspace_file(
            "benchmark_assembly_definition.yaml"
        )

        inputs = {
            "task": state.task,
            "current_step": current_step,
            "plan": state.plan,
            "todo": todo,
            "assembly_definition": assembly_definition,
            "objectives": objectives,
            "benchmark_assembly_definition": benchmark_assembly_definition,
            "steer_context": steer_context,
            "feedback": state.feedback,
        }
        validate_files = [
            "plan.md",
            "todo.md",
            "benchmark_definition.yaml",
            "assembly_definition.yaml",
            "script.py",
            "benchmark_assembly_definition.yaml",
        ]

        prediction, _, journal_entry = await self._run_program(
            dspy.ReAct,
            CoderSignature,
            state,
            inputs,
            get_engineer_tools,
            validate_files,
            AgentName.ENGINEER_CODER,
        )

        if not prediction:
            return state.model_copy(
                update={
                    "journal": state.journal + journal_entry,
                    "iteration": state.iteration + 1,
                    "turn_count": state.turn_count + 1,
                }
            )

        # Success
        summary = getattr(
            prediction, "journal", f"Successfully executed step: {current_step}"
        )
        journal_entry += f"\n[Coder] {summary}"

        # Persist TODO progress to disk so downstream reviewer checks use current state.
        new_todo = todo
        if not no_step_mode:
            new_todo = self._mark_current_step_complete(todo, current_step)
            await self._persist_todo_update(new_todo)

        return state.model_copy(
            update={
                "todo": new_todo,
                "journal": state.journal + journal_entry,
                "current_step": current_step,
                "messages": [
                    *state.messages,
                    AIMessage(content=f"Coder summary: {summary}"),
                ],
                "turn_count": state.turn_count + 1,
            }
        )

    def _mark_current_step_complete(self, todo: str, current_step: str) -> str:
        # Prefer exact line replacement for deterministic bookkeeping.
        exact_unchecked = f"- [ ] {current_step}"
        exact_checked = f"- [x] {current_step}"
        if exact_unchecked in todo:
            return todo.replace(exact_unchecked, exact_checked, 1)

        # Fallback: mark the first unchecked line as done.
        lines = todo.splitlines()
        for idx, line in enumerate(lines):
            stripped = line.strip()
            if not stripped.startswith("- [ ]"):
                continue
            lines[idx] = re.sub(r"^\s*-\s*\[\s\]", "- [x]", line, count=1)
            return "\n".join(lines)
        return todo

    async def _persist_todo_update(self, todo: str) -> None:
        try:
            await self.ctx.worker_client.write_file("todo.md", todo, overwrite=True)
        except Exception as exc:
            logger.warning("coder_todo_persist_failed", error=str(exc))

    def _get_next_step(self, todo: str) -> str | None:
        """Extract the first '- [ ]' item from the TODO list."""
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                return line.strip().replace("- [ ]", "").strip()
        return None


# Factory function for LangGraph
@type_check
async def coder_node(state: AgentState) -> AgentState:
    session_id = state.session_id
    if not session_id:
        msg = "Missing required session_id for coder_node"
        raise ValueError(msg)
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.worker_light_url,
        session_id=session_id,
        episode_id=episode_id,
        worker_client=state.worker_client,
        fs=state.fs,
        agent_role=AgentName.ENGINEER_CODER,
    )
    node = CoderNode(context=ctx)
    return await node(state)
