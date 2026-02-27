import asyncio
import datetime
import json
import uuid
from typing import Any

import structlog
from langchain_core.callbacks import BaseCallbackHandler
from opentelemetry import trace
from pydantic import BaseModel

from controller.observability.broadcast import EpisodeBroadcaster
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Trace
from shared.enums import TraceType

logger = structlog.get_logger(__name__)


class TraceBroadcast(BaseModel):
    """Schema for broadcasting a trace event."""

    type: str = "new_trace"
    id: int | uuid.UUID
    created_at: str
    trace_type: TraceType
    name: str | None
    content: str | None
    metadata: dict[str, Any] | None
    langfuse_trace_id: str | None


class DatabaseCallbackHandler(BaseCallbackHandler):
    """Standalone recorder that stores explicit traces in the database."""

    def __init__(
        self,
        episode_id: str | uuid.UUID,
        loop: asyncio.AbstractEventLoop | None = None,
    ):
        if isinstance(episode_id, str):
            try:
                self.episode_id = uuid.UUID(episode_id)
            except ValueError:
                self.episode_id = uuid.uuid5(uuid.NAMESPACE_DNS, episode_id)
        else:
            self.episode_id = episode_id
        self.session_factory = get_sessionmaker()
        self.broadcaster = EpisodeBroadcaster.get_instance()
        self.loop = loop or (
            asyncio.get_event_loop() if asyncio.get_event_loop().is_running() else None
        )

    async def _broadcast_trace(self, trace_obj: Trace) -> None:
        """Helper to broadcast a new trace."""
        created_at = trace_obj.created_at or datetime.datetime.now(datetime.UTC)
        payload = TraceBroadcast(
            id=trace_obj.id,
            created_at=created_at.isoformat(),
            trace_type=trace_obj.trace_type,
            name=trace_obj.name,
            content=trace_obj.content,
            metadata=trace_obj.metadata_vars,
            langfuse_trace_id=trace_obj.langfuse_trace_id,
        )
        await self.broadcaster.broadcast(
            self.episode_id, payload.model_dump(mode="json")
        )

    def _get_langfuse_id(self) -> str | None:
        """Extract trace ID from current OpenTelemetry span context."""
        try:
            span = trace.get_current_span()
            if span and span.get_span_context().is_valid:
                return f"{span.get_span_context().trace_id:032x}"
        except Exception:
            pass
        return None

    async def record_node_start(self, node_name: str, input_data: str = "") -> None:
        """Explicitly log the start of an agent node (e.g., Planner, Coder)."""
        logger.info("node_start", name=node_name, episode_id=str(self.episode_id))
        try:
            async with self.session_factory() as db:
                trace_obj = Trace(
                    episode_id=self.episode_id,
                    trace_type=TraceType.LOG,  # Use log for transitions
                    name=node_name,
                    content=f"Starting task phase: {node_name}. Input: {input_data[:100]}...",
                    langfuse_trace_id=self._get_langfuse_id(),
                )
                db.add(trace_obj)
                await db.commit()
                await db.refresh(trace_obj)
                await self._broadcast_trace(trace_obj)
        except Exception as e:
            logger.warning(
                "database_trace_failed", error=str(e), episode_id=str(self.episode_id)
            )

    async def record_node_end(self, node_name: str, output_data: str = "") -> None:
        """Explicitly log the end of an agent node."""
        logger.info("node_end", name=node_name, episode_id=str(self.episode_id))
        try:
            async with self.session_factory() as db:
                trace_obj = Trace(
                    episode_id=self.episode_id,
                    trace_type=TraceType.LOG,
                    name=node_name,
                    content=f"Completed task phase: {node_name}. Result: {output_data[:100]}...",
                    langfuse_trace_id=self._get_langfuse_id(),
                )
                db.add(trace_obj)
                await db.commit()
                await db.refresh(trace_obj)
                await self._broadcast_trace(trace_obj)
        except Exception as e:
            logger.warning(
                "database_trace_failed", error=str(e), episode_id=str(self.episode_id)
            )

    async def record_tool_start(self, tool_name: str, input_data: str) -> int:
        """Explicitly log the start of a tool call."""
        try:
            async with self.session_factory() as db:
                trace_obj = Trace(
                    episode_id=self.episode_id,
                    trace_type=TraceType.TOOL_START,
                    name=tool_name,
                    content=input_data,
                    langfuse_trace_id=self._get_langfuse_id(),
                )
                db.add(trace_obj)
                await db.commit()
                await db.refresh(trace_obj)
                await self._broadcast_trace(trace_obj)
                return trace_obj.id
        except Exception as e:
            logger.warning("database_tool_start_failed", error=str(e))
            return 0

    async def record_tool_end(
        self, trace_id: int, output_data: str, is_error: bool = False
    ) -> None:
        """Explicitly log the end of a tool call, potentially updating metadata."""
        if not trace_id:
            return
        try:
            async with self.session_factory() as db:
                trace_obj = await db.get(Trace, trace_id)
                if trace_obj:
                    # For now, we update the existing TOOL_START trace with results
                    # to keep the UI simple (ActionCard can show output).
                    # Alternatively, we could create a TOOL_END trace.
                    meta = dict(trace_obj.metadata_vars or {})
                    if is_error:
                        meta["error"] = output_data
                    else:
                        meta["output"] = output_data
                    trace_obj.metadata_vars = meta
                    await db.commit()
                    await self._broadcast_trace(trace_obj)
        except Exception as e:
            logger.warning("database_tool_end_failed", error=str(e))

        def record_tool_start_sync(self, tool_name: str, input_data: str) -> int:
            """Synchronous wrapper for record_tool_start."""
            try:
                if self.loop and self.loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(
                        self.record_tool_start(tool_name, input_data), self.loop
                    )
                    return future.result(timeout=10)
                return asyncio.run(self.record_tool_start(tool_name, input_data))
            except Exception as e:
                logger.warning("database_tool_start_sync_failed", error=str(e))
                return 0

        def record_tool_end_sync(
            self, trace_id: int, output_data: str, is_error: bool = False
        ) -> None:
            """Synchronous wrapper for record_tool_end."""
            if not trace_id:
                return
            try:
                if self.loop and self.loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(
                        self.record_tool_end(trace_id, output_data, is_error), self.loop
                    )
                    future.result(timeout=10)
                else:
                    asyncio.run(self.record_tool_end(trace_id, output_data, is_error))
            except Exception as e:
                logger.warning("database_tool_end_sync_failed", error=str(e))

    # Keep generic event recording
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

                    trace_obj = Trace(
                        episode_id=self.episode_id,
                        trace_type=TraceType.EVENT,
                        name=event_data.get("event_type", "generic_event"),
                        content=content,
                        metadata_vars=event_data,
                        langfuse_trace_id=self._get_langfuse_id(),
                    )
                    db.add(trace_obj)

                await db.commit()
        except Exception as e:
            logger.warning(
                "database_events_failed", error=str(e), episode_id=str(self.episode_id)
            )
