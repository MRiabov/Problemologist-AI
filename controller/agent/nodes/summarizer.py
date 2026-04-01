import dspy
import structlog

from controller.agent.config import settings
from controller.agent.context_usage import (
    estimate_text_tokens,
    update_episode_context_usage,
)
from controller.agent.state import AgentState
from controller.observability.middleware_helper import record_events
from shared.enums import AgentName
from shared.observability.schemas import ConversationLengthExceededEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class SummarizerSignature(dspy.Signature):
    """DSPy signature for the journalling agent."""

    journal = dspy.InputField()
    summarized_journal = dspy.OutputField(desc="A concise summary of the journal")


@type_check
class SummarizerNode(BaseNode):
    """
    Summarizer node: Compresses the journal when it exceeds length limits.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        threshold = settings.context_compaction_threshold_tokens
        # Check if journal actually needs summarization
        # Uses configured threshold from config/agents_config.yaml.
        if estimate_text_tokens(state.journal) < threshold:
            return state

        logger.info(
            "summarizing_journal",
            journal_length=len(state.journal),
            session_id=state.session_id,
        )

        inputs = {"journal": state.journal}

        prediction, _, _ = await self._run_program(
            dspy.ReAct,
            SummarizerSignature,
            state,
            inputs,
            lambda fs, sid: [],
            [],
            AgentName.JOURNALLING_AGENT,
        )

        summarized = getattr(prediction, "summarized_journal", state.journal)

        if state.episode_id:
            try:
                await record_events(
                    episode_id=state.episode_id,
                    events=[
                        ConversationLengthExceededEvent(
                            previous_length=estimate_text_tokens(state.journal),
                            threshold=threshold,
                            compacted_length=estimate_text_tokens(summarized),
                            agent_id=AgentName.JOURNALLING_AGENT.value,
                            user_session_id=state.session_id or None,
                            episode_id=state.episode_id,
                        )
                    ],
                )
                await update_episode_context_usage(
                    episode_id=state.episode_id,
                    used_tokens=estimate_text_tokens(summarized),
                    max_tokens=threshold,
                )
            except Exception as exc:
                logger.warning(
                    "conversation_length_event_emit_failed",
                    error=str(exc),
                    episode_id=state.episode_id,
                )

        logger.info(
            "journal_summarized",
            old_length=len(state.journal),
            new_length=len(summarized),
        )

        return state.model_copy(
            update={
                "journal": f"[Summarized Journal]\n{summarized}",
            }
        )


# Factory function for LangGraph
@type_check
async def summarizer_node(state: AgentState) -> AgentState:
    session_id = state.session_id
    if not session_id:
        msg = "Missing required session_id for summarizer_node"
        raise ValueError(msg)
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.worker_light_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role=AgentName.JOURNALLING_AGENT,
    )
    node = SummarizerNode(context=ctx)
    return await node(state)
