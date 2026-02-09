import uuid
from datetime import datetime
from typing import Any

from langchain_core.callbacks import BaseCallbackHandler

from controller.observability.broadcast import EpisodeBroadcaster
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Trace


class DatabaseCallbackHandler(BaseCallbackHandler):
    """Callback handler that stores traces in the database."""

    def __init__(self, episode_id: uuid.UUID):
        self.episode_id = episode_id
        self.session_factory = get_sessionmaker()
        self.broadcaster = EpisodeBroadcaster.get_instance()

    async def _broadcast_trace(self, trace_id: int, raw_trace: dict) -> None:
        """Helper to broadcast a new trace."""
        await self.broadcaster.broadcast(
            self.episode_id,
            {
                "type": "new_trace",
                "id": trace_id,
                "created_at": datetime.utcnow().isoformat(),
                "raw_trace": raw_trace,
            },
        )

    async def on_chain_start(
        self, serialized: dict[str, Any], inputs: dict[str, Any], **_kwargs: Any
    ) -> None:
        pass

    async def on_tool_start(
        self, serialized: dict[str, Any], input_str: str, **_kwargs: Any
    ) -> None:
        async with self.session_factory() as db:
            trace = Trace(
                episode_id=self.episode_id,
                raw_trace={
                    "type": "tool_start",
                    "name": serialized.get("name"),
                    "input": input_str,
                },
            )
            db.add(trace)
            await db.commit()
            await db.refresh(trace)
            await self._broadcast_trace(trace.id, trace.raw_trace)

    async def on_tool_end(self, output: Any, **_kwargs: Any) -> None:
        async with self.session_factory() as db:
            # Ensure output is serializable (ToolMessage is not)
            if not isinstance(output, (str, dict, list, int, float, bool, type(None))):
                output = str(output)

            trace = Trace(
                episode_id=self.episode_id,
                raw_trace={"type": "tool_end", "output": output},
            )
            db.add(trace)
            await db.commit()
            await db.refresh(trace)
            await self._broadcast_trace(trace.id, trace.raw_trace)

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
                raw_trace={"type": "llm_end", "content": content},
            )
            db.add(trace)
            await db.commit()
            await db.refresh(trace)
            await self._broadcast_trace(trace.id, trace.raw_trace)
