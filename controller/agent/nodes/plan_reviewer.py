from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_coder_tools
from controller.observability.tracing import record_worker_events
from shared.enums import ReviewDecision
from shared.models.schemas import ReviewResult
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class PlanReviewerSignature(dspy.Signature):
    """
    Plan Reviewer node: Evaluates the proposed mechanical and electrical plans.
    You must use the provided tools to read 'plan.md', 'todo.md', and 'assembly_definition.yaml'.
    Ensure the plan is physically feasible, within budget, and complete.
    When done, use SUBMIT to provide your final ReviewResult.
    """

    task = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    assembly_definition = dspy.InputField()
    objectives = dspy.InputField()
    journal = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


@type_check
class PlanReviewerNode(BaseNode):
    """
    Plan Reviewer node: Evaluates the engineering plan before implementation.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        # Read objectives and assembly_definition for context
        objectives = "# No objectives.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("objectives.yaml"):
                objectives = await self.ctx.worker_client.read_file("objectives.yaml")

        assembly_definition = "# No assembly_definition.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("assembly_definition.yaml"):
                assembly_definition = await self.ctx.worker_client.read_file(
                    "assembly_definition.yaml"
                )

        inputs = {
            "task": state.task,
            "plan": state.plan,
            "todo": state.todo,
            "assembly_definition": assembly_definition,
            "objectives": objectives,
            "journal": state.journal,
        }

        validate_files = ["plan.md", "todo.md", "assembly_definition.yaml"]

        prediction, _artifacts, journal_entry = await self._run_program(
            program_cls=dspy.ReAct,
            signature_cls=PlanReviewerSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_coder_tools,
            validate_files=validate_files,
            node_type="plan_reviewer",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.PLAN_REJECTED,
                    "feedback": f"Plan Reviewer failed to complete: {journal_entry}",
                    "journal": state.journal + journal_entry,
                    "turn_count": state.turn_count + 1,
                }
            )

        review = prediction.review
        decision = review.decision
        feedback = review.reason
        if review.required_fixes:
            feedback += "\nRequired Fixes:\n" + "\n".join(
                [f"- {f}" for f in review.required_fixes]
            )

        journal_entry += (
            f"\nPlan Critic Decision: {decision.value}\nFeedback: {feedback}"
        )

        status_map = {
            ReviewDecision.APPROVED: AgentStatus.APPROVED,
            ReviewDecision.REJECTED: AgentStatus.PLAN_REJECTED,
            ReviewDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
        }

        # Emit ReviewDecisionEvent
        await record_worker_events(
            episode_id=state.episode_id,
            events=[
                ReviewDecisionEvent(
                    decision=decision,
                    reason=feedback,
                    evidence_stats={
                        "is_plan_review": True,
                    },
                )
            ],
        )

        return state.model_copy(
            update={
                "status": status_map.get(decision, AgentStatus.PLAN_REJECTED),
                "feedback": feedback,
                "journal": state.journal + journal_entry,
                "messages": [
                    *state.messages,
                    AIMessage(content=f"Plan Review decision: {decision.value}"),
                ],
                "turn_count": state.turn_count + 1,
            }
        )


# Factory function for LangGraph
@type_check
async def plan_reviewer_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.spec_001_api_url, session_id=session_id
    )
    node = PlanReviewerNode(context=ctx)
    return await node(state)
