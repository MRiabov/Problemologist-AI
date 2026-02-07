import asyncio
import uuid
from collections import defaultdict
from collections.abc import AsyncGenerator
from typing import Any, ClassVar


class EpisodeBroadcaster:
    """Simple in-memory PubSub for broadcasting episode events."""

    _instance: ClassVar["EpisodeBroadcaster | None"] = None
    _subscribers: ClassVar[dict[uuid.UUID, list[asyncio.Queue]]] = defaultdict(list)

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    async def subscribe(self, episode_id: uuid.UUID) -> AsyncGenerator[Any, None]:
        """Subscribe to events for a specific episode."""
        queue = asyncio.Queue()
        self._subscribers[episode_id].append(queue)
        try:
            while True:
                message = await queue.get()
                yield message
        finally:
            self._subscribers[episode_id].remove(queue)
            if not self._subscribers[episode_id]:
                del self._subscribers[episode_id]

    async def broadcast(self, episode_id: uuid.UUID, message: Any) -> None:
        """Broadcast a message to all subscribers of an episode."""
        # 1. Internal Queue Broadcasting (legacy/internal)
        if episode_id in self._subscribers:
            for queue in self._subscribers[episode_id]:
                await queue.put(message)

        # 2. WebSocket Broadcasting (new unified mechanism)
        from controller.api.manager import manager

        await manager.broadcast(episode_id, message)
