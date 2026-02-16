import json
import logging
import re
from enum import StrEnum
import yaml
from pydantic import BaseModel, Field
from langchain_core.messages import HumanMessage, SystemMessage
from langgraph.prebuilt import create_react_agent
import structlog

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


@type_check
class ReviewerNode(BaseNode):
    """
    Reviewer node: Evaluates the Coder's output based on simulation and workbench reports.
    Refactored to use LangGraph's prebuilt ReAct agent.
    """

    def __init__(self, context: SharedNodeContext):
        super().__init__(context)
        # Initialize tools and agent
        self.tools = get_engineer_tools(self.ctx.fs, self.ctx.session_id)
        self.agent = create_react_agent(self.ctx.llm, self.tools)

    async def __call__(self, state: AgentState) -> AgentState:
        # T015: Use LLM to evaluate success
        # We instruct the agent to use tools to read reports instead of pre-loading them
        prompt = self.ctx.pm.render(
            "critic",
            task=state.task,
            journal=state.journal,
            sim_report="Read 'simulation_report.json' using tools.",
            mfg_report="Read 'workbench_report.md' using tools.",
        )

        # Add explicit instruction for format (though implicit in prompt template usually)
        prompt += "\n\nIMPORTANT: Read 'simulation_report.json' and 'workbench_report.md' to inform your decision. Output your final decision in the specified YAML frontmatter format."

        messages = [SystemMessage(content=prompt)]

        # Observability
        callbacks = self._get_callbacks(name="reviewer", session_id=state.session_id)

        try:
            # Invoke agent
            logger.info("reviewer_agent_invoke_start", session_id=state.session_id)
            result = await self.agent.ainvoke(
                {"messages": messages}, config={"callbacks": callbacks}
            )
            logger.info("reviewer_agent_invoke_complete", session_id=state.session_id)
            messages = result["messages"]
            content = str(messages[-1].content)  # Final response

        except Exception as e:
            logger.error("Reviewer agent failed", error=str(e))
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": f"Reviewer agent system error: {e}",
                    "journal": state.journal + f"\n[Reviewer] System error: {e}",
                    "messages": messages if "messages" in locals() else [],
                }
            )

        # T018: Decision logic (Structured Parsing)
        try:
            # Use another LLM call to parse the agent's output into a structured model
            # This is safer than regex for ReAct agents that might output conversational text
            parser_llm = self.ctx.llm.with_structured_output(ReviewResult)
            review_parsed = await parser_llm.ainvoke(
                [
                    SystemMessage(
                        content="Extract the final review decision from the following text."
                    ),
                    HumanMessage(content=content),
                ]
            )

            decision = review_parsed.decision
            if review_parsed.required_fixes:
                fixes_text = "\n".join(
                    [f"- {fix}" for fix in review_parsed.required_fixes]
                )
                feedback = f"{review_parsed.reason}\n\nRequired Fixes:\n{fixes_text}"
            else:
                feedback = review_parsed.reason
        except Exception as e:
            logger.warn(
                "Failed to parse review via structured output, falling back to regex",
                error=str(e),
            )
            # Fallback to regex if LLM parser fails (unlikely with structured output)
            match = re.search(
                r"^---\n(.*?)\n---\n(.*)", content, re.DOTALL | re.MULTILINE
            )
            if match:
                yaml_block = match.group(1)
                body = match.group(2)
                data = yaml.safe_load(yaml_block)
                decision_str = data.get("decision", "").upper()
                if "APPROVE" in decision_str:
                    decision = CriticDecision.APPROVE
                elif "REJECT_PLAN" in decision_str:
                    decision = CriticDecision.REJECT_PLAN
                else:
                    decision = CriticDecision.REJECT_CODE
                feedback = body.strip()
            else:
                feedback = f"Error parsing critic output: {e}\nRaw output:\n{content}"

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
                        "has_sim_report": True,  # Assumption: agent read it
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
                "messages": messages,  # Persist trace
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
