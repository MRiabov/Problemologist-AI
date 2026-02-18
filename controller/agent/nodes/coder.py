import structlog

import dspy
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState
from controller.agent.tools import get_engineer_tools
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class CoderSignature(dspy.Signature):
    """
    Coder node: Picks a task from TODO, writes code, executes it, and fixes errors.
    You must use the provided tools to implement the current step in 'script.py'.
    When done, use SUBMIT to provide a summary of your work.
    """

    current_step = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    journal = dspy.OutputField(
        desc="A summary of the implementation done for this step"
    )


@type_check
class CoderNode(BaseNode):
    """
    Coder node: Picks a task from TODO, writes code, executes it, and fixes errors.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the coder node logic."""
        # T010: Find next active item in TODO
        todo = state.todo
        current_step = self._get_next_step(todo)
        if not current_step:
            return state.model_copy(
                update={"journal": state.journal + "\nNo more steps in TODO."}
            )

        inputs = {
            "current_step": current_step,
            "plan": state.plan,
            "todo": todo,
        }
        validate_files = [
            "plan.md",
            "todo.md",
            "objectives.yaml",
            "assembly_definition.yaml",
            "script.py",
        ]

        prediction, _, journal_entry = await self._run_program(
            program_cls=dspy.CodeAct,
            signature_cls=CoderSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_engineer_tools,
            validate_files=validate_files,
            node_type="coder",
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

        # Mark TODO as done
        new_todo = todo.replace(f"- [ ] {current_step}", f"- [x] {current_step}")

        return state.model_copy(
            update={
                "todo": new_todo,
                "journal": state.journal + journal_entry,
                "current_step": current_step,
                "messages": state.messages
                + [AIMessage(content=f"Coder summary: {summary}")],
                "turn_count": state.turn_count + 1,
            }
        )

    def _get_next_step(self, todo: str) -> str | None:
        """Extract the first '- [ ]' item from the TODO list, ignoring electronics tasks."""
        elec_keywords = ["circuit", "wire", "electronics", "routing", "psu", "power"]
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                task = line.strip().replace("- [ ]", "").strip()
                # If it's an electronics task, skip it (ElectronicsEngineer will handle it)
                if any(kw in task.lower() for kw in elec_keywords):
                    continue
                return task
        return None


# Factory function for LangGraph
@type_check
async def coder_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = CoderNode(context=ctx)
    return await node(state)
