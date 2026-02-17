import asyncio
from enum import StrEnum

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.dspy_utils import init_dspy, wrap_tool_for_dspy
from controller.agent.signatures import ReviewerSignature
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class CriticDecision(StrEnum):
    APPROVE = "APPROVE"
    REJECT_PLAN = "REJECT_PLAN"
    REJECT_CODE = "REJECT_CODE"


@type_check
class ReviewerNode(BaseNode):
    """
    Reviewer node: Evaluates the Coder's output.
    Refactored to use DSPy CodeAct.
    """

    def __init__(self, context: SharedNodeContext):
        super().__init__(context)
        # Initialize DSPy
        init_dspy(session_id=self.ctx.session_id)
        # Initialize tools
        self.raw_tools = get_engineer_tools(self.ctx.fs, self.ctx.session_id)
        self.tools = [wrap_tool_for_dspy(t) for t in self.raw_tools]

        # Define DSPy CodeAct module
        self.reviewer = dspy.CodeAct(ReviewerSignature, tools=self.tools)

    async def __call__(self, state: AgentState) -> AgentState:
        journal_entry = "\n[Reviewer] Starting review phase using DSPy CodeAct."

        try:
            logger.info("reviewer_dspy_invoke_start", session_id=state.session_id)

            # Run DSPy CodeAct module
            result = self.reviewer(
                task=state.task,
                journal=state.journal,
                sim_report="Read 'simulation_report.json' using tools.",
                mfg_report="Read 'workbench_report.md' using tools."
            )

            logger.info("reviewer_dspy_invoke_complete", session_id=state.session_id)

            review_parsed = result.review_result
            decision_str = review_parsed.decision.upper()

            if "APPROVE" in decision_str:
                decision = CriticDecision.APPROVE
            elif "REJECT_PLAN" in decision_str:
                decision = CriticDecision.REJECT_PLAN
            elif "REJECT_CODE" in decision_str:
                decision = CriticDecision.REJECT_CODE
            else:
                decision = CriticDecision.REJECT_CODE

            if review_parsed.required_fixes:
                fixes_text = "\n".join(
                    [f"- {fix}" for fix in review_parsed.required_fixes]
                )
                feedback = f"{review_parsed.reason}\n\nRequired Fixes:\n{fixes_text}"
            else:
                feedback = review_parsed.reason

        except Exception as e:
            logger.error("Reviewer DSPy module failed", error=str(e))
            error_msg = f"Reviewer DSPy error: {e}"
            journal_msg = f"\n[Reviewer] System error during DSPy execution: {e}"
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": error_msg,
                    "journal": state.journal + journal_msg,
                }
            )

        journal_entry += f"\nCritic Decision: {decision.value}\nFeedback: {feedback}"

        status_map = {
            CriticDecision.APPROVE: AgentStatus.APPROVED,
            CriticDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
            CriticDecision.REJECT_CODE: AgentStatus.CODE_REJECTED,
        }

        # Emit ReviewDecisionEvent for observability
        await record_worker_events(
            episode_id=state.session_id,
            events=[
                ReviewDecisionEvent(
                    decision=decision.value.lower(),
                    reason=feedback,
                    evidence_stats={
                        "has_sim_report": True,
                        "has_mfg_report": True,
                    },
                )
            ],
        )

        reviewer_msg = f"Reviewer decision: {decision.value}. Feedback: {feedback}"
        return state.model_copy(
            update={
                "status": status_map.get(decision, AgentStatus.CODE_REJECTED),
                "feedback": feedback,
                "journal": state.journal + journal_entry,
                "messages": [AIMessage(content=reviewer_msg)]
            }
        )


# Factory function for LangGraph
@type_check
async def reviewer_node(state: AgentState) -> AgentState:
    # Use session_id from state
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = ReviewerNode(context=ctx)
    return await node(state)
