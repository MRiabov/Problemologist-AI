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


class PlannerSignature(dspy.Signature):
    """
    Planner node: Analyzes the task and creates plan.md and todo.md using tools.
    You must use the provided tools to create 'plan.md' and 'todo.md' directly.
    When done, use SUBMIT to provide a summary of your plan.
    """

    task = dspy.InputField()
    objectives = dspy.InputField()
    skills = dspy.InputField()
    steer_context = dspy.InputField()
    feedback = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the plan created")


@type_check
class PlannerNode(BaseNode):
    """
    Planner node: Analyzes the task and creates plan.md and todo.md using tools.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the planner node logic."""
        # T006: Read skills
        skills_context = self._get_skills_context()
        # WP04: Extract steerability context
        steer_context = self._get_steer_context(state.messages)

        # Read objectives for context
        objectives = "# No objectives.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("objectives.yaml"):
                objectives = await self.ctx.worker_client.read_file("objectives.yaml")

        inputs = {
            "task": state.task,
            "objectives": objectives,
            "skills": skills_context,
            "steer_context": steer_context,
            "feedback": (
                state.feedback
                if state.status == AgentStatus.PLAN_REJECTED
                else "New planning. No rejection feedback."
            ),
        }
        validate_files = ["plan.md", "todo.md", "assembly_definition.yaml"]

        prediction, artifacts, journal_entry = await self._run_program(
            program_cls=dspy.CodeAct,
            signature_cls=PlannerSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_engineer_tools,
            validate_files=validate_files,
            node_type="planner",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.FAILED,
                    "journal": state.journal + journal_entry,
                }
            )

        # Success

        await record_worker_events(
            episode_id=state.session_id,
            events=[
                SubmissionValidationEvent(
                    artifacts_present=list(artifacts.keys()),
                    verification_passed=True,
                    reasoning_trace_quality=1.0,
                    errors=[],
                )
            ],
        )

        # WP01: Post-validation - Ensure assembly_definition.yaml has been priced
        if "assembly_definition.yaml" in artifacts:
            content = artifacts["assembly_definition.yaml"]
            import yaml
            try:
                data = yaml.safe_load(content)
                if data and (not data.get("totals") or not data["totals"].get("estimated_unit_cost_usd")):
                    logger.warning("planner_output_unpriced", session_id=state.session_id)
                    # We could force a retry here, or just log it.
                    # Per architecture, the script SHOULD have been run.
            except Exception as e:
                logger.warning("planner_output_yaml_error", error=str(e))

        summary = getattr(prediction, "summary", "No summary provided.")
        return state.model_copy(
            update={
                "plan": artifacts.get("plan.md", ""),
                "todo": artifacts.get("todo.md", ""),
                "status": AgentStatus.EXECUTING,
                "journal": state.journal + f"\n[Planner] {summary}" + journal_entry,
                "messages": state.messages
                + [AIMessage(content=f"Plan summary: {summary}")],
            }
        )


# Factory function for LangGraph
@type_check
async def planner_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = PlannerNode(context=ctx)
    return await node(state)
