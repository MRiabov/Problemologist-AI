import logging
from typing import Any

import structlog
from langchain_core.messages import HumanMessage, SystemMessage

from controller.agent.config import settings
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


@type_check
class CompressorNode(BaseNode):
    """
    Compressor node: Summarizes conversation history when token or turn limits are approached.
    """

    def __init__(self, context: SharedNodeContext):
        super().__init__(context)

    async def __call__(self, state: Any) -> dict:
        """Execute the compression logic."""
        messages = state.get("messages", [])

        # WP3: Check if compression is needed
        # We trigger if turn_count is high OR we have many messages
        turn_count = getattr(state, "turn_count", 0)
        if isinstance(state, dict):
            # BenchmarkGeneratorState is a dict, doesn't have turn_count directly in the same way
            # but we can check messages length
            pass

        if len(messages) < 10:
            return {}

        logger.info("compressor_triggered", session_id=self.ctx.session_id, message_count=len(messages))

        # 1. Generate Summary
        summary_prompt = (
            "You are the Token Compression Agent. "
            "Summarize the following conversation history between an AI agent and its environment. "
            "Preserve key decisions, findings, and remaining TODO items. "
            "Be concise but ensure the context is sufficient for the agent to continue its task."
        )

        history_text = "\n".join([f"{m.type}: {m.content}" for m in messages])

        summary_result = await self.ctx.llm.ainvoke(
            [SystemMessage(content=summary_prompt), HumanMessage(content=history_text)]
        )
        summary_content = str(summary_result.content)

        # 2. Reconstruct messages
        # We keep the system prompt (first message usually) and add the summary,
        # plus the last 2 messages for immediate context.
        new_messages = [
            messages[0], # System Prompt
            SystemMessage(content=f"Summary of previous history:\n{summary_content}"),
        ]
        new_messages.extend(messages[-2:])

        logger.info("compression_complete", session_id=self.ctx.session_id, original_count=len(messages), new_count=len(new_messages))

        return {"messages": new_messages}


@type_check
async def compressor_node(state: Any) -> dict:
    session_id = getattr(state, "session_id", None)
    if not session_id and isinstance(state, dict):
        session = state.get("session")
        if session and hasattr(session, "session_id"):
            session_id = str(session.session_id)

    if not session_id:
        session_id = settings.default_session_id

    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = CompressorNode(context=ctx)
    return await node(state)
