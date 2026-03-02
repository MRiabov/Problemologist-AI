from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_benchmark_tools
from controller.observability.tracing import record_worker_events
from shared.enums import ReviewDecision
from shared.models.schemas import ReviewResult
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class BenchmarkPlanReviewerSignature(dspy.Signature):
    """
    Benchmark Plan Reviewer node: Evaluates the proposed benchmark plan.
    Ensure the challenge is solvable, randomization is appropriate, and no excessive DOFs.
    You must use the provided tools to read 'plan.md', 'todo.md', and 'objectives.yaml'.
    When done, use SUBMIT to provide your final ReviewResult.
    """

    task = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    objectives = dspy.InputField()
    journal = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


@type_check
class BenchmarkPlanReviewerNode(BaseNode):
    async def __call__(self, state: AgentState) -> AgentState:
        objectives = "# No objectives.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("objectives.yaml"):
                objectives = await self.ctx.worker_client.read_file("objectives.yaml")

        inputs = {
            "task": state.task,
            "plan": state.plan,
            "todo": state.todo,
            "objectives": objectives,
            "journal": state.journal,
        }

        validate_files = ["plan.md", "todo.md", "objectives.yaml"]

        prediction, _artifacts, journal_entry = await self._run_program(
            program_cls=dspy.ReAct,
            signature_cls=BenchmarkPlanReviewerSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_benchmark_tools,
            validate_files=validate_files,
            node_type="benchmark_plan_reviewer",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.PLAN_REJECTED,
                    "feedback": f"Benchmark Plan Reviewer failed to complete: {journal_entry}",
                    "journal": state.journal + journal_entry,
                    "turn_count": state.turn_count + 1,
                }
            )

        review = prediction.review
        decision = review.decision
        feedback = review.reason
        if review.required_fixes:
            feedback += "\nRequired Fixes:\n" + "\n".join([f"- {f}" for f in review.required_fixes])

        journal_entry += f"\nBenchmark Plan Decision: {decision.value}\nFeedback: {feedback}"

        status_map = {
            ReviewDecision.APPROVED: AgentStatus.APPROVED,
            ReviewDecision.REJECTED: AgentStatus.PLAN_REJECTED,
            ReviewDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
            ReviewDecision.CONFIRM_PLAN_REFUSAL: AgentStatus.FAILED,
            ReviewDecision.REJECT_PLAN_REFUSAL: AgentStatus.PLAN_REJECTED,
        }

        await record_worker_events(
            episode_id=state.episode_id,
            events=[
                ReviewDecisionEvent(
                    decision=decision,
                    reason=feedback,
                    evidence_stats={"is_benchmark_plan_review": True},
                )
            ],
        )

        return state.model_copy(
            update={
                "status": status_map.get(decision, AgentStatus.PLAN_REJECTED),
                "feedback": feedback,
                "journal": state.journal + journal_entry,
                "messages": state.messages + [AIMessage(content=f"Benchmark Plan Review decision: {decision.value}")],
                "turn_count": state.turn_count + 1,
            }
        )


@type_check
async def benchmark_plan_reviewer_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.spec_001_api_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role="benchmark_reviewer",
    )
    node = BenchmarkPlanReviewerNode(context=ctx)
    return await node(state)
