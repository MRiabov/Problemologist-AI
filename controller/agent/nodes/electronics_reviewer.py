import logging
import re
from typing import Any

import structlog
import yaml
from langchain_core.messages import HumanMessage, SystemMessage
from langgraph.prebuilt import create_react_agent
from pydantic import BaseModel, Field

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext
from .reviewer import CriticDecision, ReviewResult

logger = structlog.get_logger(__name__)


@type_check
class ElectronicsReviewerNode(BaseNode):
    """
    Electronics Reviewer node: Evaluates the electrical design, circuit safety,
    and wire routing. Focuses on SPICE validation results and connectivity.
    """

    def __init__(self, context: SharedNodeContext):
        super().__init__(context)
        # Initialize tools and agent
        self.tools = get_engineer_tools(self.ctx.fs, self.ctx.session_id)
        self.agent = create_react_agent(self.ctx.llm, self.tools)

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the electronics reviewer node logic."""
        # 1. Prepare prompt
        prompt = self.ctx.pm.render(
            "critic",  # Re-use critic template for now, but with specific electrical context
            task=state.task,
            journal=state.journal,
            sim_report="Read 'electronics_report.json' or 'simulation_report.json' using tools.",
            mfg_report="Read 'assembly_definition.yaml' electronics section using tools.",
        )

        prompt += (
            "\n\nSpecialized Role: You are the Electrical Reviewer. "
            "Focus specifically on the electrical design, circuit safety (short circuits, overcurrent), "
            "power budget, and physical wire routing. "
            "Read 'electronics_report.json' (if it exists) to verify SPICE simulation results. "
            "Output your final decision in the specified YAML frontmatter format."
        )

        messages = [SystemMessage(content=prompt)]

        # Observability
        callbacks = self._get_callbacks(
            name="electronics_reviewer", session_id=state.session_id
        )

        try:
            # 2. Invoke agent
            logger.info("electronics_reviewer_invoke_start", session_id=state.session_id)
            result = await self.agent.ainvoke(
                {"messages": messages}, config={"callbacks": callbacks}
            )
            logger.info(
                "electronics_reviewer_invoke_complete", session_id=state.session_id
            )
            messages = result["messages"]
            content = str(messages[-1].content)

        except Exception as e:
            logger.error("Electronics reviewer agent failed", error=str(e))
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": f"Electronics reviewer system error: {e}",
                    "journal": state.journal + f"\n[Electronics Reviewer] System error: {e}",
                    "messages": messages if "messages" in locals() else [],
                }
            )

        # 3. Parse decision
        try:
            parser_llm = self.ctx.llm.with_structured_output(ReviewResult)
            review_parsed = await parser_llm.ainvoke(
                [
                    SystemMessage(
                        content="Extract the final electrical review decision from the following text."
                    ),
                    HumanMessage(content=content),
                ]
            )

            decision = review_parsed.decision
            feedback = review_parsed.reason
            if review_parsed.required_fixes:
                feedback += "\n\nRequired Fixes:\n" + "\n".join(
                    [f"- {fix}" for fix in review_parsed.required_fixes]
                )
        except Exception:
            # Simple fallback
            if "APPROVE" in content.upper():
                decision = CriticDecision.APPROVE
            else:
                decision = CriticDecision.REJECT_CODE
            feedback = content

        journal_entry = (
            f"\nElectronics Review Decision: {decision.value}\nFeedback: {feedback}"
        )

        # Map decision to status
        status_map = {
            CriticDecision.APPROVE: AgentStatus.APPROVED,
            CriticDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
            CriticDecision.REJECT_CODE: AgentStatus.CODE_REJECTED,
        }

        # Emit ReviewDecisionEvent
        await record_worker_events(
            episode_id=state.session_id,
            events=[
                ReviewDecisionEvent(
                    decision=f"elec_{decision.value.lower()}",
                    reason=feedback,
                    evidence_stats={"is_electronics": True},
                )
            ],
        )

        # If electronics are approved, we might still need a general review,
        # so we set status but let the graph decide the next step.
        return state.model_copy(
            update={
                "status": status_map.get(decision, AgentStatus.CODE_REJECTED),
                "feedback": feedback,
                "journal": state.journal + journal_entry,
                "messages": messages,
            }
        )


@type_check
async def electronics_reviewer_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = ElectronicsReviewerNode(context=ctx)
    return await node(state)
