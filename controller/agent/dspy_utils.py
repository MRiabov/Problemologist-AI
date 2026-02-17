import asyncio
import structlog
from typing import Any, Callable, Dict, Optional
from controller.clients.worker import WorkerClient

logger = structlog.get_logger(__name__)

class WorkerInterpreter:
    """
    A DSPy CodeInterpreter that executes code on the remote worker.
    Uses the project's 'well formed remote execution environment'.
    """

    def __init__(self, worker_client: WorkerClient, session_id: str):
        self.worker_client = worker_client
        self.session_id = session_id

    def __call__(self, code: str) -> str:
        """Execute code on the worker and return the output."""
        logger.debug("worker_interpreter_execute_start", code_len=len(code))

        # DSPy CodeAct expects a synchronous call.
        # We use a new event loop or run_until_complete if possible.
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        if loop.is_running():
            # This is tricky if we are already in an async loop (which we are in LangGraph)
            # However, DSPy's CodeAct program is often called within an async node.
            # We might need to use a thread or a specialized async interpreter.

            # For now, let's try a workaround for running async in async.
            import nest_asyncio
            nest_asyncio.apply()

        result = loop.run_until_complete(self._execute_remote(code))

        output = f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        if result.exit_code != 0:
            output += f"\nExit Code: {result.exit_code}"

        logger.debug("worker_interpreter_execute_complete", exit_code=result.exit_code)
        return output

    async def _execute_remote(self, code: str):
        """Internal async execution call."""
        return await self.worker_client.execute(code)

    def shutdown(self):
        """Cleanup if needed."""
        pass
