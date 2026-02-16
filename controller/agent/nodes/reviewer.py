import json
import logging
import re
from enum import StrEnum
from typing import Any

import yaml
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent

from controller.agent.config import settings
from controller.agent.prompt_manager import PromptManager
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check

logger = logging.getLogger(__name__)


class CriticDecision(StrEnum):
    APPROVE = "APPROVE"
    REJECT_PLAN = "REJECT_PLAN"
    REJECT_CODE = "REJECT_CODE"


@type_check
class ReviewerNode:
    """
    Reviewer node: Evaluates the Coder's output based on simulation and workbench reports.
    Refactored to use LangGraph's prebuilt ReAct agent.
    """

    def __init__(
        self,
        worker_url: str = "http://worker:8001",
        session_id: str = "default-session",
    ):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model=settings.llm_model, temperature=0)
        self.worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        self.fs = RemoteFilesystemMiddleware(self.worker_client)

        # Initialize tools and agent
        self.tools = get_engineer_tools(self.fs, session_id)
        self.agent = create_react_agent(self.llm, self.tools)

    async def __call__(self, state: AgentState) -> AgentState:
        # T015: Use LLM to evaluate success
        # We instruct the agent to use tools to read reports instead of pre-loading them
        prompt = self.pm.render(
            "critic",
            task=state.task,
            journal=state.journal,
            sim_report="Read 'simulation_report.json' using tools.",
            mfg_report="Read 'workbench_report.md' using tools.",
        )

        # Add explicit instruction for format (though implicit in prompt template usually)
        prompt += "\n\nIMPORTANT: Read 'simulation_report.json' and 'workbench_report.md' to inform your decision. Output your final decision in the specified YAML frontmatter format."

        messages = [SystemMessage(content=prompt)]

        try:
            # Invoke agent
            result = await self.agent.ainvoke({"messages": messages})
            messages = result["messages"]
            content = str(messages[-1].content)  # Final response

            # Check if tools were used to verify evidence (optional check, but good for observability)
            # For now, we rely on the agent's final answer.

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

        # T018: Decision logic (Parsing)
        decision = CriticDecision.REJECT_CODE
        feedback = "Failed to parse critic decision."

        try:
            # Flexible match for frontmatter (start of string or after whitespace)
            match = re.search(
                r"^---\n(.*?)\n---\n(.*)", content, re.DOTALL | re.MULTILINE
            )
            if match:
                yaml_block = match.group(1)
                body = match.group(2)
                data = yaml.safe_load(yaml_block)

                decision_str = data.get("decision", "").upper()
                if decision_str == "APPROVE":
                    decision = CriticDecision.APPROVE
                elif decision_str == "REJECT_PLAN":
                    decision = CriticDecision.REJECT_PLAN
                elif decision_str == "REJECT_CODE":
                    decision = CriticDecision.REJECT_CODE

                required_fixes = data.get("required_fixes", [])
                if required_fixes:
                    fixes_text = "\n".join([f"- {fix}" for fix in required_fixes])
                    feedback = f"{body.strip()}\n\nRequired Fixes:\n{fixes_text}"
                else:
                    feedback = body.strip()
            else:
                feedback = f"Critic failed to produce valid frontmatter. Raw output:\n{content}"
        except Exception as e:
            feedback = f"Error parsing critic output: {e}\nRaw output:\n{content}"

        journal_entry = f"\nCritic Decision: {decision.value}\nFeedback: {feedback}"

        status_map = {
            CriticDecision.APPROVE: AgentStatus.APPROVED,
            CriticDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
            CriticDecision.REJECT_CODE: AgentStatus.CODE_REJECTED,
        }

        # Emit ReviewDecisionEvent for observability
        # Note: We don't have explicit bools for has_sim_report etc. easily unless we parse tool calls.
        # We'll set them to True for now or omit.
        # The agent *should* have read them.
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
    node = ReviewerNode(worker_url=settings.spec_001_api_url, session_id=session_id)
    return await node(state)
