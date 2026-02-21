import dspy
import structlog
from langchain_core.messages import AIMessage

from shared.type_checking import type_check

from ..config import settings
from ..state import AgentState
from ..tools import get_engineer_tools
from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class COTSSearchSignature(dspy.Signature):
    """
    COTS Search node: Searches for components based on current needs.
    You must use the provided tools to perform searches.
    When done, use SUBMIT to provide a summary of the components found.
    """

    task = dspy.InputField()
    plan = dspy.InputField()
    journal = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the components found")


@type_check
class COTSSearchNode(BaseNode):
    """
    COTS Search node: Searches for components based on current needs.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        inputs = {
            "task": state.task,
            "plan": state.plan,
            "journal": state.journal,
        }

        prediction, _, journal_entry = await self._run_program(
            dspy.CodeAct,
            COTSSearchSignature,
            state,
            inputs,
            get_engineer_tools,
            [],
            "cots_search",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "journal": state.journal
                    + f"\n[COTS Search] Failed: {journal_entry}"
                }
            )

        summary = getattr(prediction, "summary", "No summary provided.")
        new_journal = state.journal + f"\n[COTS Search] {summary}" + journal_entry

        return state.model_copy(
            update={
                "journal": new_journal,
                "messages": state.messages
                + [AIMessage(content=f"COTS Search summary: {summary}")],
            }
        )


@type_check
async def cots_search_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.spec_001_api_url, session_id=session_id
    )
    node = COTSSearchNode(context=ctx)
    return await node(state)
