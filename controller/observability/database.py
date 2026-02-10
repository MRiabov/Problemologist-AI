import uuid
from typing import Any

from langchain_core.callbacks import BaseCallbackHandler

from controller.observability.broadcast import EpisodeBroadcaster
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Trace
from shared.enums import TraceType


class DatabaseCallbackHandler(BaseCallbackHandler):
    """Callback handler that stores traces in the database."""

    def __init__(
        self,
        episode_id: uuid.UUID,
        langfuse_callback: Any | None = None,
    ):
        self.episode_id = episode_id
        self.session_factory = get_sessionmaker()
        self.broadcaster = EpisodeBroadcaster.get_instance()
        self.langfuse_callback = langfuse_callback

    async def _broadcast_trace(self, trace: Trace) -> None:
        """Helper to broadcast a new trace."""
        await self.broadcaster.broadcast(
            self.episode_id,
            {
                "type": "new_trace",
                "id": trace.id,
                "created_at": trace.created_at.isoformat(),
                "trace_type": trace.trace_type,
                "name": trace.name,
                "content": trace.content,
                "metadata": trace.metadata_vars,
                "langfuse_trace_id": trace.langfuse_trace_id,
            },
        )

    async def on_chain_start(
        self, serialized: dict[str, Any], inputs: dict[str, Any], **_kwargs: Any
    ) -> None:
        pass

    def _get_langfuse_id(self) -> str | None:
        if self.langfuse_callback and hasattr(self.langfuse_callback, "get_trace_id"):
            try:
                return self.langfuse_callback.get_trace_id()
            except Exception:
                return None
        return None

    async def on_tool_start(
        self, serialized: dict[str, Any], input_str: str, **_kwargs: Any
    ) -> None:
        async with self.session_factory() as db:
            trace = Trace(
                episode_id=self.episode_id,
                trace_type=TraceType.TOOL_START,
                name=serialized.get("name"),
                content=input_str,
                langfuse_trace_id=self._get_langfuse_id(),
            )
            db.add(trace)
            await db.commit()
            await db.refresh(trace)
            await self._broadcast_trace(trace)

    async def on_tool_end(self, output: Any, **_kwargs: Any) -> None:
        async with self.session_factory() as db:
            # Ensure output is serializable (ToolMessage is not)
            if not isinstance(output, (str, dict, list, int, float, bool, type(None))):
                output = str(output)

            trace = Trace(
                episode_id=self.episode_id,
                trace_type=TraceType.TOOL_END,
                content=str(output),
                langfuse_trace_id=self._get_langfuse_id(),
            )
            db.add(trace)
            await db.commit()
            await db.refresh(trace)
            await self._broadcast_trace(trace)

    async def on_llm_new_token(self, token: str, **_kwargs: Any) -> None:
        # We might not want to save every token to DB, maybe just full completions
        pass

    async def on_llm_end(self, response: Any, **_kwargs: Any) -> None:
        async with self.session_factory() as db:
            # Extract content from the first generation
            content = ""
            if response.generations and response.generations[0]:
                content = response.generations[0][0].text

            trace = Trace(
                episode_id=self.episode_id,
                trace_type=TraceType.LLM_END,
                content=content,
                langfuse_trace_id=self._get_langfuse_id(),
            )
            db.add(trace)
            await db.commit()
            await db.refresh(trace)
            await self._broadcast_trace(trace)

    async def record_events(self, events: list[dict[str, Any]]) -> None:
        """Record domain-specific events in the database."""
        if not events:
            return

        async with self.session_factory() as db:
            for event_data in events:
                # Store the entire event in content or metadata depending on preference.
                # Here we use content for a summary and metadata for the full data.
                trace = Trace(
                    episode_id=self.episode_id,
                    trace_type=TraceType.EVENT,
                    name=event_data.get("event_type", "generic_event"),
                    content=str(event_data.get("data", {})),
                    metadata_vars=event_data,
                    langfuse_trace_id=self._get_langfuse_id(),
                )
                db.add(trace)

            await db.commit()
            # We don't necessarily need to broadcast every event individually
            # if there are many, but for now we do it for consistency.
            # However, _broadcast_trace expects a Trace object with an ID.
            # Since we added many, we might need to refresh them.
            # To keep it simple, I'll just broadcast them if it doesn't hurt.
            # Actually, let's skip broadcasting individual events for now to avoid flooding the UI.
