from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.enums import AgentName, ReviewDecision
from shared.models.schemas import ReviewResult
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class ElectronicsReviewerSignature(dspy.Signature):
    """
    Electronics Reviewer node: Evaluates the electrical implementation.
    You must use the provided tools to read 'script.py' and any circuit-related files.
    You also receive read-only benchmark_assembly_definition.yaml context when present.
    Ensure the circuit is correctly defined, wires are routed properly, and it matches the electrical plan.
    When done, provide your final ReviewResult.
    """

    task = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    assembly_definition = dspy.InputField()
    benchmark_assembly_definition = dspy.InputField()
    plan_refusal = dspy.InputField(default="")
    objectives = dspy.InputField()
    journal = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


@type_check
class ElectronicsReviewerNode(BaseNode):
    """
    Electronics Reviewer node: Evaluates the electrical implementation.
    """

    @staticmethod
    def _normalize_electronics_decision(review: ReviewResult) -> ReviewResult:
        """Electronics defects are implementation defects and must route as REJECT_CODE."""
        if review.decision != ReviewDecision.REJECTED:
            return review
        return review.model_copy(update={"decision": ReviewDecision.REJECT_CODE})

    async def __call__(self, state: AgentState) -> AgentState:
        # Read objectives and assembly_definition for context
        objectives = "# No benchmark_definition.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("benchmark_definition.yaml"):
                objectives = await self.ctx.worker_client.read_file(
                    "benchmark_definition.yaml"
                )

        assembly_definition = "# No assembly_definition.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("assembly_definition.yaml"):
                assembly_definition = await self.ctx.worker_client.read_file(
                    "assembly_definition.yaml"
                )
        benchmark_assembly_definition = await self._read_optional_workspace_file(
            "benchmark_assembly_definition.yaml",
            "# No benchmark_assembly_definition.yaml found.",
        )

        plan_refusal = ""
        with suppress(Exception):
            if await self.ctx.worker_client.exists("plan_refusal.md"):
                plan_refusal = await self.ctx.worker_client.read_file("plan_refusal.md")

        inputs = {
            "task": state.task,
            "plan": state.plan,
            "todo": state.todo,
            "assembly_definition": assembly_definition,
            "benchmark_assembly_definition": benchmark_assembly_definition,
            "plan_refusal": plan_refusal,
            "objectives": objectives,
            "journal": state.journal,
        }

        # Validate existence of key files
        validate_files = ["script.py", "assembly_definition.yaml"]

        prediction, _artifacts, journal_entry = await self._run_program(
            program_cls=dspy.ReAct,
            signature_cls=ElectronicsReviewerSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_engineer_tools,
            validate_files=validate_files,
            node_type=AgentName.ELECTRONICS_REVIEWER,
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": f"Electronics Reviewer failed to complete: {journal_entry}",
                    "journal": state.journal + journal_entry,
                    "turn_count": state.turn_count + 1,
                }
            )

        review = ReviewResult.model_validate(prediction.review)
        review = self._normalize_electronics_decision(review)
        try:
            (
                review_decision_path,
                review_comments_path,
            ) = await self._persist_review_result(review, "electronics-review")
        except Exception as exc:
            return state.model_copy(
                update={
                    "status": AgentStatus.FAILED,
                    "feedback": (
                        f"Electronics Reviewer failed to persist review files: {exc}"
                    ),
                    "journal": (
                        state.journal
                        + journal_entry
                        + f"\n[Electronics Reviewer] Review persistence failed: {exc}"
                    ),
                    "turn_count": state.turn_count + 1,
                }
            )
        decision = review.decision
        feedback = review.reason
        if review.required_fixes:
            feedback += "\nRequired Fixes:\n" + "\n".join(
                [f"- {f}" for f in review.required_fixes]
            )

        journal_entry += (
            f"\nElectronics Review Decision: {decision.value}\nFeedback: {feedback}"
        )

        status_map = {
            ReviewDecision.APPROVED: AgentStatus.APPROVED,
            ReviewDecision.REJECTED: AgentStatus.CODE_REJECTED,
            ReviewDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
            ReviewDecision.REJECT_CODE: AgentStatus.CODE_REJECTED,
            ReviewDecision.CONFIRM_PLAN_REFUSAL: AgentStatus.FAILED,
            ReviewDecision.REJECT_PLAN_REFUSAL: AgentStatus.CODE_REJECTED,
        }
        if decision not in status_map:
            return state.model_copy(
                update={
                    "status": AgentStatus.FAILED,
                    "feedback": (
                        "Electronics Reviewer returned unsupported structured "
                        f"decision: {decision}"
                    ),
                    "journal": (
                        state.journal
                        + journal_entry
                        + f"\n[Electronics Reviewer] Unsupported decision: {decision}"
                    ),
                    "turn_count": state.turn_count + 1,
                }
            )

        # Emit ReviewDecisionEvent
        await record_worker_events(
            episode_id=state.episode_id,
            events=[
                ReviewDecisionEvent(
                    decision=decision,
                    reason=feedback,
                    evidence_stats={
                        "is_electronics_review": True,
                        "review_decision_path": review_decision_path,
                        "review_comments_path": review_comments_path,
                    },
                    checklist=review.checklist,
                )
            ],
        )

        return state.model_copy(
            update={
                "status": status_map[decision],
                "feedback": feedback,
                "journal": state.journal + journal_entry,
                "messages": [
                    *state.messages,
                    AIMessage(content=f"Electronics Review decision: {decision.value}"),
                ],
                "turn_count": state.turn_count + 1,
            }
        )


# Factory function for LangGraph
@type_check
async def electronics_reviewer_node(state: AgentState) -> AgentState:
    session_id = state.session_id
    if not session_id:
        msg = "Missing required session_id for electronics_reviewer_node"
        raise ValueError(msg)
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.worker_light_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role=AgentName.ELECTRONICS_REVIEWER,
    )
    node = ElectronicsReviewerNode(context=ctx)
    return await node(state)
