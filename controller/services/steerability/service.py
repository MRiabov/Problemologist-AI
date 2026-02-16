import asyncio
import structlog
from typing import Dict, List
from shared.models.steerability import SteerablePrompt

logger = structlog.get_logger(__name__)


class SteerabilityService:
    """
    Manages in-memory queues for interaction steering.
    Each agent session has its own queue of pending prompts.
    """

    def __init__(self):
        self._queues: Dict[str, asyncio.Queue[SteerablePrompt]] = {}
        self._lock = asyncio.Lock()

    async def _get_or_create_queue(
        self, session_id: str
    ) -> asyncio.Queue[SteerablePrompt]:
        async with self._lock:
            if session_id not in self._queues:
                self._queues[session_id] = asyncio.Queue()
                logger.info("steerability_queue_created", session_id=session_id)
            return self._queues[session_id]

    async def enqueue_prompt(self, session_id: str, prompt: SteerablePrompt) -> int:
        """
        Add a prompt to the session's queue.
        Returns the new queue size.
        """
        queue = await self._get_or_create_queue(session_id)
        await queue.put(prompt)
        size = queue.qsize()
        logger.info("prompt_enqueued", session_id=session_id, queue_size=size)
        return size

    async def dequeue_prompt(self, session_id: str) -> SteerablePrompt:
        """
        Retrieve the next prompt from the session's queue.
        Blocks until a prompt is available.
        """
        queue = await self._get_or_create_queue(session_id)
        prompt = await queue.get()
        logger.info(
            "prompt_dequeued", session_id=session_id, remaining_size=queue.qsize()
        )
        return prompt

    async def get_queued_prompts(self, session_id: str) -> List[SteerablePrompt]:
        """
        Return a list of all currently queued prompts for a session (non-destructive).
        """
        async with self._lock:
            if session_id not in self._queues:
                return []
            # Accessing internal _queue is okay for read-only peeking in the same loop
            return list(self._queues[session_id]._queue)

    async def clear_queue(self, session_id: str):
        """Remove all prompts from a session's queue."""
        async with self._lock:
            if session_id in self._queues:
                del self._queues[session_id]
                logger.info("steerability_queue_cleared", session_id=session_id)


# Singleton instance
steerability_service = SteerabilityService()
