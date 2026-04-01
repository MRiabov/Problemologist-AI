from typing import Any

import dspy
import structlog
from langchain_core.messages import AIMessage

from shared.enums import AgentName
from shared.type_checking import type_check

from ..config import settings
from ..state import AgentState
from ..tools import get_cots_search_tools
from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class COTSSearchSignature(dspy.Signature):
    """DSPy signature for the COTS search subagent."""

    prompt = dspy.InputField()
    search_summary = dspy.OutputField(desc="A summary of the components found")


@type_check
class COTSSearchNode(BaseNode):
    """
    COTS Search node: Searches for components based on current needs.
    Executes through the shared BaseNode runtime and provider-native tool loop.
    """

    async def run_search(self, *, state: Any, prompt: str) -> tuple[str | None, str]:
        inputs = {"prompt": prompt}
        prediction, _, journal_entry = await self._run_program(
            dspy.ReAct,
            COTSSearchSignature,
            state,
            inputs,
            get_cots_search_tools,
            [],
            AgentName.COTS_SEARCH,
        )
        summary = getattr(prediction, "search_summary", None) if prediction else None
        return summary, journal_entry

    async def __call__(self, state: AgentState) -> AgentState:
        summary, journal_entry = await self.run_search(state=state, prompt=state.task)

        if not summary:
            return state.model_copy(
                update={
                    "journal": state.journal
                    + f"\n[COTS Search] Failed: {journal_entry}"
                }
            )

        new_journal = state.journal + f"\n[COTS Search] {summary}" + journal_entry

        return state.model_copy(
            update={
                "journal": new_journal,
                "messages": [
                    *state.messages,
                    AIMessage(content=f"COTS Search summary: {summary}"),
                ],
            }
        )


@type_check
async def cots_search_node(state: AgentState) -> AgentState:
    session_id = state.session_id
    if not session_id:
        msg = "Missing required session_id for cots_search_node"
        raise ValueError(msg)
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.worker_light_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role=AgentName.COTS_SEARCH,
    )
    node = COTSSearchNode(context=ctx)
    return await node(state)
