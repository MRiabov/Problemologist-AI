from contextlib import suppress

import dspy
import structlog
import yaml
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.enums import AgentName
from shared.models.schemas import ObjectivesYaml
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
    objectives = dspy.InputField()
    feedback = dspy.InputField()
    skills = dspy.InputField()
    journal = dspy.OutputField(desc="A summary of the electronics work done")


@type_check
class ElectronicsEngineerNode(BaseNode):
    """
    Electronics Engineer node: Designs circuits and routes wires.
    Refactored to use DSPy ReAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the electronics engineer node logic."""
        # 1. Find next electronics task in TODO
        todo = state.todo
        current_step = self._get_next_electronics_step(todo)

        # Read objectives for context
        objectives = "# No objectives.yaml found."
        has_electronics_requirements = False
        with suppress(Exception):
            if await self.ctx.worker_client.exists("objectives.yaml"):
                objectives = await self.ctx.worker_client.read_file("objectives.yaml")
                parsed_objectives = ObjectivesYaml.model_validate(
                    yaml.safe_load(objectives) or {}
                )
                has_electronics_requirements = (
                    parsed_objectives.electronics_requirements is not None
                )

        if not current_step:
            if not has_electronics_requirements:
                return state
            current_step = "Design circuit and route wires"

        # 2. Get current assembly context
        assembly_context = "No assembly context available."
        with suppress(Exception):
            assembly_context = await self.ctx.fs.read_file("assembly_definition.yaml")

        # 3. Get skills context
        skills_context = self._get_skills_context()

        # 4. Handover event

        await record_worker_events(
            episode_id=state.episode_id,
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
            "objectives": objectives,
            "feedback": (
                state.feedback
                if state.status == AgentStatus.CODE_REJECTED
                else "New electronics step. No rejection feedback."
            ),
            "skills": skills_context,
        }
        validate_files = ["plan.md", "todo.md", "assembly_definition.yaml", "script.py"]

        prediction, _artifacts, journal_entry = await self._run_program(
            dspy.ReAct,
            ElectronicsSignature,
            state,
            inputs,
            get_engineer_tools,
            validate_files,
            AgentName.ELECTRONICS_ENGINEER,
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
                "messages": [
                    *state.messages,
                    AIMessage(content=f"Electronics summary: {summary}"),
                ],
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
    session_id = state.session_id
    if not session_id:
        msg = "Missing required session_id for electronics_engineer_node"
        raise ValueError(msg)
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.worker_light_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role=AgentName.ELECTRONICS_ENGINEER,
    )
    node = ElectronicsEngineerNode(context=ctx)
    return await node(state)
