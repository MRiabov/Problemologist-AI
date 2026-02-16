import ast
import datetime
import json
import uuid
from typing import Any

import structlog
from langchain_core.callbacks import AsyncCallbackHandler
from langchain_core.outputs import LLMResult
from pydantic import BaseModel

from controller.observability.broadcast import EpisodeBroadcaster
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Trace
from shared.enums import TraceType

logger = structlog.get_logger(__name__)


class TraceBroadcast(BaseModel):
    """Schema for broadcasting a trace event."""

    type: str = "new_trace"
    id: uuid.UUID
    created_at: str
    trace_type: TraceType
    name: str | None
    content: str | None
    metadata: dict[str, Any] | None
    langfuse_trace_id: str | None


class DatabaseCallbackHandler(AsyncCallbackHandler):
    """Callback handler that stores traces in the database."""

    def __init__(
        self,
        episode_id: str | uuid.UUID,
        langfuse_callback: Any | None = None,
    ):
        if isinstance(episode_id, str):
            try:
                self.episode_id = uuid.UUID(episode_id)
            except ValueError:
                # Fallback: create a deterministic UUID from the string
                # This ensures we don't crash and maintain consistency for the same string
                self.episode_id = uuid.uuid5(uuid.NAMESPACE_DNS, episode_id)
        else:
            self.episode_id = episode_id
        self.session_factory = get_sessionmaker()
        self.broadcaster = EpisodeBroadcaster.get_instance()
        self.langfuse_callback = langfuse_callback

    async def _broadcast_trace(self, trace: Trace) -> None:
        """Helper to broadcast a new trace."""
        created_at = trace.created_at or datetime.datetime.now(datetime.UTC)
        payload = TraceBroadcast(
            id=trace.id,
            created_at=created_at.isoformat(),
            trace_type=trace.trace_type,
            name=trace.name,
            content=trace.content,
            metadata=trace.metadata_vars,
            langfuse_trace_id=trace.langfuse_trace_id,
        )
        await self.broadcaster.broadcast(
            self.episode_id, payload.model_dump(mode="json")
        )

    async def on_chain_start(
        self, serialized: dict[str, Any], inputs: dict[str, Any], **_kwargs: Any
    ) -> None:
        logger.info(
            "chain_start", name=serialized.get("name"), episode_id=str(self.episode_id)
        )

    def _get_langfuse_id(self) -> str | None:
        if self.langfuse_callback and hasattr(self.langfuse_callback, "get_trace_id"):
            try:
                return self.langfuse_callback.get_trace_id()
            except Exception:
                return None
        return None

    async def on_llm_start(
        self, serialized: dict[str, Any], prompts: list[str], **_kwargs: Any
    ) -> None:
        logger.info(
            "llm_start", model=serialized.get("name"), episode_id=str(self.episode_id)
        )

        try:
            async with self.session_factory() as db:
                trace = Trace(
                    episode_id=self.episode_id,
                    trace_type=TraceType.LLM_START,
                    name=serialized.get("name"),
                    content=prompts[0] if prompts else "",
                    langfuse_trace_id=self._get_langfuse_id(),
                )
                db.add(trace)
                await db.commit()
                await db.refresh(trace)
                await self._broadcast_trace(trace)
        except Exception as e:
            logger.warning(
                "database_trace_failed", error=str(e), episode_id=str(self.episode_id)
            )

    async def on_tool_start(
        self, serialized: dict[str, Any], input_str: str, **_kwargs: Any
    ) -> None:
        # Try to ensure valid JSON content
        clean_content = input_str
        if (
            input_str
            and not input_str.startswith("{")
            and not input_str.startswith("[")
        ):
            # If it looks like a Python dict string, try to convert to JSON
            try:
                data = ast.literal_eval(input_str)
                if isinstance(data, (dict, list)):
                    clean_content = json.dumps(data)
            except Exception:
                pass

        logger.info(
            "tool_start",
            name=serialized.get("name"),
            input=clean_content,
            episode_id=str(self.episode_id),
        )

        try:
            async with self.session_factory() as db:
                trace = Trace(
                    episode_id=self.episode_id,
                    trace_type=TraceType.TOOL_START,
                    name=serialized.get("name"),
                    content=clean_content,
                    langfuse_trace_id=self._get_langfuse_id(),
                )
                db.add(trace)
                await db.commit()
                await db.refresh(trace)
                await self._broadcast_trace(trace)
        except Exception as e:
            # Silently fail or log a warning if DB is unavailable/constraint fails
            # This is critical for standalone evals or transient issues
            logger.warning(
                "database_trace_failed", error=str(e), episode_id=str(self.episode_id)
            )

    async def on_tool_end(self, output: Any, **_kwargs: Any) -> None:
        # Ensure output is JSON serializable
        if isinstance(output, (dict, list)):
            content = json.dumps(output)
        elif isinstance(output, (str, int, float, bool, type(None))):
            content = str(output) if output is not None else "null"
        else:
            # Fallback for complex objects (like Pydantic models)
            try:
                if hasattr(output, "model_dump"):
                    content = json.dumps(output.model_dump())
                elif hasattr(output, "dict"):
                    content = json.dumps(output.dict())
                else:
                    content = str(output)
            except Exception:
                content = str(output)

        logger.info("tool_end", output=content, episode_id=str(self.episode_id))

        try:
            async with self.session_factory() as db:
                trace = Trace(
                    episode_id=self.episode_id,
                    trace_type=TraceType.TOOL_END,
                    content=content,
                    langfuse_trace_id=self._get_langfuse_id(),
                )
                db.add(trace)
                await db.commit()
                await db.refresh(trace)
                await self._broadcast_trace(trace)
        except Exception as e:
            logger.warning(
                "database_trace_failed", error=str(e), episode_id=str(self.episode_id)
            )

    async def on_llm_new_token(self, token: str, **_kwargs: Any) -> None:
        pass

    async def on_llm_end(self, response: LLMResult, **_kwargs: Any) -> None:
        # Extract content from the first generation
        content = ""
        if response.generations and response.generations[0]:
            content = response.generations[0][0].text

        logger.info("llm_end", content=content, episode_id=str(self.episode_id))

        try:
            async with self.session_factory() as db:
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
        except Exception as e:
            logger.warning(
                "database_trace_failed", error=str(e), episode_id=str(self.episode_id)
            )

    async def record_events(self, events: list[dict[str, Any]]) -> None:
        """Record domain-specific events in the database."""
        if not events:
            return

        try:
            async with self.session_factory() as db:
                for event_data in events:
                    data = event_data.get("data", {})
                    content = (
                        json.dumps(data)
                        if isinstance(data, (dict, list))
                        else str(data)
                    )

                    trace = Trace(
                        episode_id=self.episode_id,
                        trace_type=TraceType.EVENT,
                        name=event_data.get("event_type", "generic_event"),
                        content=content,
                        metadata_vars=event_data,
                        langfuse_trace_id=self._get_langfuse_id(),
                    )
                    db.add(trace)

                await db.commit()
        except Exception as e:
            logger.warning(
                "database_events_failed", error=str(e), episode_id=str(self.episode_id)
            )
            # We don't necessarily need to broadcast every event individually
            # if there are many, but for now we do it for consistency.
            # However, _broadcast_trace expects a Trace object with an ID.
            # Since we added many, we might need to refresh them.
            # To keep it simple, I'll just broadcast them if it doesn't hurt.
            # Actually, let's skip broadcasting individual events for now to avoid flooding the UI.
