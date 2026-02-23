import dspy
import structlog
from controller.agent.state import AgentState
from .base import BaseNode, SharedNodeContext
from shared.type_checking import type_check
from controller.agent.config import settings

logger = structlog.get_logger(__name__)

class SummarizerSignature(dspy.Signature):
    """
    Summarizer node: Compresses the journal to stay within token limits.
    Provide a concise summary of the key decisions, attempts, and outcomes recorded in the journal.
    Maintain critical technical details while reducing verbosity.
    """
    journal = dspy.InputField()
    summarized_journal = dspy.OutputField(desc="A concise summary of the journal")

@type_check
class SummarizerNode(BaseNode):
    """
    Summarizer node: Compresses the journal when it exceeds length limits.
    """
    async def __call__(self, state: AgentState) -> AgentState:
        # Check if journal actually needs summarization
        # Using a conservative threshold for now (e.g., 5000 characters)
        if len(state.journal) < 5000:
            return state

        logger.info("summarizing_journal", journal_length=len(state.journal), session_id=state.session_id)

        inputs = {"journal": state.journal}

        # Use a simple chain of thought or basic program for summarization
        program = dspy.Predict(SummarizerSignature)

        import asyncio
        with dspy.settings.context(lm=self.ctx.dspy_lm):
            prediction = await asyncio.to_thread(program, **inputs)

        summarized = getattr(prediction, "summarized_journal", state.journal)

        logger.info("journal_summarized", old_length=len(state.journal), new_length=len(summarized))

        return state.model_copy(
            update={
                "journal": f"[Summarized Journal]\n{summarized}",
            }
        )

# Factory function for LangGraph
@type_check
async def summarizer_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url
    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url, session_id=session_id
    )
    node = SummarizerNode(context=ctx)
    return await node(state)
