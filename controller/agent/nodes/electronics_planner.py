from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import SubmissionValidationEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class ElectronicsPlannerSignature(dspy.Signature):
    """
    Electronics Planner node: Designs the electrical system and power budget.
    You must use the provided tools to update 'plan.md' and 'todo.md' with electronics tasks.
    When done, use SUBMIT to provide a summary of your electrical plan.
    """

    task = dspy.InputField()
    objectives = dspy.InputField()
    skills = dspy.InputField()
    mechanical_plan = dspy.InputField()
    feedback = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the electrical plan created")


@type_check
class ElectronicsPlannerNode(BaseNode):
    """
    Electronics Planner node: Specialized electrical system planning.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the electronics planner node logic."""
        skills_context = self._get_skills_context()

        # Read objectives for context
        objectives = "# No objectives.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("objectives.yaml"):
                objectives = await self.ctx.worker_client.read_file("objectives.yaml")

        inputs = {
            "task": state.task,
            "objectives": objectives,
            "skills": skills_context,
            "mechanical_plan": state.plan,
            "feedback": (
                state.feedback
                if state.status == AgentStatus.PLAN_REJECTED
                else "New electronics planning. No rejection feedback."
            ),
        }
        validate_files = ["plan.md", "todo.md", "assembly_definition.yaml"]

        prediction, artifacts, journal_entry = await self._run_program(
            program_cls=dspy.ReAct,
            signature_cls=ElectronicsPlannerSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_engineer_tools,
            validate_files=validate_files,
            node_type="electronics_planner",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.FAILED,
                    "journal": state.journal + journal_entry,
                    "turn_count": state.turn_count + 1,
                }
            )

        # Success
        await record_worker_events(
            episode_id=state.episode_id,
            events=[
                SubmissionValidationEvent(
                    artifacts_present=list(artifacts.keys()),
                    verification_passed=True,
                    reasoning_trace_quality=1.0,
                    errors=[],
                )
            ],
        )

        summary = getattr(prediction, "summary", "No summary provided.")
        new_plan = artifacts.get("plan.md", state.plan)
        new_todo = artifacts.get("todo.md", state.todo)
        new_journal = (
            state.journal + f"\n[Electronics Planner] {summary}" + journal_entry
        )

        # T008: Persist updated Plan, TODO and Journal to worker filesystem
        await self.ctx.fs.write_file("journal.md", new_journal)

        return state.model_copy(
            update={
                "plan": new_plan,
                "todo": new_todo,
                "journal": new_journal,
                "messages": state.messages
                + [AIMessage(content=f"Electronics Plan summary: {summary}")],
                "turn_count": state.turn_count + 1,
            }
        )


# Factory function for LangGraph
@type_check
async def electronics_planner_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.spec_001_api_url, session_id=session_id
    )
    node = ElectronicsPlannerNode(context=ctx)
    return await node(state)
