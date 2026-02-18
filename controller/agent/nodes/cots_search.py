import structlog

import dspy
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
        from controller.agent.dspy_utils import WorkerInterpreter

        # Use WorkerInterpreter for remote execution
        interpreter = WorkerInterpreter(
            worker_client=self.ctx.worker_client, session_id=state.session_id
        )

        # Get tool signatures for DSPy
        tool_fns = self._get_tool_functions(get_engineer_tools)

        program = dspy.CodeAct(
            COTSSearchSignature, tools=list(tool_fns.values()), interpreter=interpreter
        )

        try:
            with dspy.settings.context(lm=self.ctx.dspy_lm):
                logger.info(
                    "cots_search_dspy_invoke_start", session_id=state.session_id
                )
                prediction = program(
                    task=state.task,
                    plan=state.plan,
                    journal=state.journal,
                )
                logger.info(
                    "cots_search_dspy_invoke_complete", session_id=state.session_id
                )

            summary = getattr(prediction, "summary", "No summary provided.")
            new_journal = state.journal + f"\n[COTS Search] {summary}"

            return state.model_copy(
                update={
                    "journal": new_journal,
                    "messages": state.messages
                    + [AIMessage(content=f"COTS Search summary: {summary}")],
                }
            )
        except Exception as e:
            logger.error("COTS Search dspy failed", error=str(e))
            return state.model_copy(
                update={"journal": state.journal + f"\n[COTS Search] System error: {e}"}
            )
        finally:
            interpreter.shutdown()


@type_check
async def cots_search_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = COTSSearchNode(context=ctx)
    return await node(state)
