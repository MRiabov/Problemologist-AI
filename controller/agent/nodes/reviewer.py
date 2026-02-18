import re
from enum import StrEnum

import dspy
import structlog
import yaml
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage
from pydantic import BaseModel, Field

from controller.agent.config import settings
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


class ReviewResult(BaseModel):
    """Structured output for the reviewer."""

    decision: CriticDecision
    reason: str
    required_fixes: list[str] = Field(default_factory=list)


class ReviewerSignature(dspy.Signature):
    """
    Reviewer node: Evaluates the Coder's output based on simulation and workbench reports.
    You must use the provided tools to read 'simulation_report.json' and 'workbench_report.md'.
    When done, use SUBMIT to provide your final ReviewResult.
    """

    task = dspy.InputField()
    journal = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


@type_check
class ReviewerNode(BaseNode):
    """
    Reviewer node: Evaluates the Coder's output based on simulation and workbench reports.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        # T015: Use DSPy to evaluate success
        from controller.agent.dspy_utils import WorkerInterpreter

        # Use WorkerInterpreter for remote execution
        interpreter = WorkerInterpreter(
            worker_client=self.ctx.worker_client, session_id=state.session_id
        )

        # Get tool signatures for DSPy
        tool_fns = self._get_tool_functions(get_engineer_tools)

        program = dspy.CodeAct(
            ReviewerSignature, tools=list(tool_fns.values()), interpreter=interpreter
        )

        try:
            with dspy.settings.context(lm=self.ctx.dspy_lm):
                logger.info("reviewer_dspy_invoke_start", session_id=state.session_id)
                prediction = program(
                    task=state.task,
                    journal=state.journal,
                )
                logger.info("reviewer_dspy_invoke_complete", session_id=state.session_id)

            review = prediction.review
            decision = review.decision
            feedback = review.reason
            if review.required_fixes:
                feedback += "\nRequired Fixes:\n" + "\n".join(
                    [f"- {f}" for f in review.required_fixes]
                )

        except Exception as e:
            logger.error("Reviewer dspy failed", error=str(e))
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": f"Reviewer system error: {e}",
                    "journal": state.journal + f"\n[Reviewer] System error: {e}",
                }
            )
        finally:
            interpreter.shutdown()

        journal_entry = f"\nCritic Decision: {decision.value}\nFeedback: {feedback}"

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

        return state.model_copy(
            update={
                "status": status_map.get(decision, AgentStatus.CODE_REJECTED),
                "feedback": feedback,
                "journal": state.journal + journal_entry,
                "messages": state.messages
                + [AIMessage(content=f"Review decision: {decision.value}")],
            }
        )


# Factory function for LangGraph
@type_check
async def reviewer_node(state: AgentState) -> AgentState:
    # Use session_id from state
    session_id = state.session_id or settings.default_session_id
    async with SharedNodeContext.lifecycle(
        worker_url=settings.spec_001_api_url, session_id=session_id
    ) as ctx:
        node = ReviewerNode(context=ctx)
        return await node(state)
