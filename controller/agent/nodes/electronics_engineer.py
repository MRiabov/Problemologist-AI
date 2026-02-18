import asyncio
import structlog
from contextlib import suppress

import dspy
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import ElecAgentHandoverEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class ElectronicsSignature(dspy.Signature):
    """
    Electronics Engineer node: Designs circuits and routes wires.
    You must use the provided tools to implement the current step in 'script.py'.
    When done, use SUBMIT to provide a summary of your work.
    """

    task = dspy.InputField()
    current_step = dspy.InputField()
    plan = dspy.InputField()
    assembly_context = dspy.InputField()
    feedback = dspy.InputField()
    journal = dspy.OutputField(desc="A summary of the electronics work done")


@type_check
class ElectronicsEngineerNode(BaseNode):
    """
    Electronics Engineer node: Designs circuits and routes wires.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the electronics engineer node logic."""
        # 1. Find next electronics task in TODO
        todo = state.todo
        current_step = self._get_next_electronics_step(todo)

        if not current_step:
            try:
                objectives_content = await self.ctx.fs.read_file("objectives.yaml")
                if "electronics_requirements" not in objectives_content:
                    return state
                current_step = "Design circuit and route wires"
            except Exception:
                return state

        # 2. Get current assembly context
        assembly_context = "No assembly context available."
        with suppress(Exception):
            assembly_context = await self.ctx.fs.read_file("assembly_definition.yaml")

        # 3. Handover event
        from controller.observability.tracing import record_worker_events
        from shared.observability.schemas import ElecAgentHandoverEvent

        await record_worker_events(
            episode_id=state.session_id,
            events=[
                ElecAgentHandoverEvent(
                    from_agent="Mechanical Engineer",
                    to_agent="Electronics Engineer",
                    iteration_count=state.iteration,
                )
            ],
        )

        inputs = {
            "task": state.task,
            "current_step": current_step,
            "plan": state.plan,
            "assembly_context": assembly_context,
            "feedback": (
                state.feedback
                if state.status == AgentStatus.CODE_REJECTED
                else "New electronics step. No rejection feedback."
            ),
        }
        validate_files = ["plan.md", "todo.md", "assembly_definition.yaml", "script.py"]

        prediction, artifacts, journal_entry = await self._run_program(
            program_cls=dspy.CodeAct,
            signature_cls=ElectronicsSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_engineer_tools,
            validate_files=validate_files,
            node_type="electronics_engineer",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "journal": state.journal + journal_entry,
                    "turn_count": state.turn_count + 1,
                }
            )

        # Success
        summary = getattr(
            prediction,
            "journal",
            f"Successfully completed electronics task: {current_step}",
        )
        journal_entry += f"\n[Electronics] {summary}"
        new_todo = todo
        if current_step in todo:
            new_todo = todo.replace(f"- [ ] {current_step}", f"- [x] {current_step}")

        return state.model_copy(
            update={
                "todo": new_todo,
                "journal": state.journal + journal_entry,
                "turn_count": state.turn_count + 1,
                "messages": state.messages
                + [AIMessage(content=f"Electronics summary: {summary}")],
            }
        )

    def _get_next_electronics_step(self, todo: str) -> str | None:
        """Extract the first electronics-related '- [ ]' item."""
        elec_keywords = ["circuit", "wire", "electronics", "routing", "psu", "power"]
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                task = line.strip().replace("- [ ]", "").strip()
                if any(kw in task.lower() for kw in elec_keywords):
                    return task
        return None


# Factory function for LangGraph
@type_check
async def electronics_engineer_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = ElectronicsEngineerNode(context=ctx)
    return await node(state)
