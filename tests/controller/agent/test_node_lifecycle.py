import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from controller.agent.nodes.base import SharedNodeContext
from controller.clients.worker import WorkerClient

@pytest.mark.asyncio
async def test_shared_node_context_lifecycle_closes_client():
    # Mock WorkerClient
    mock_client = AsyncMock(spec=WorkerClient)

    # Mock SharedNodeContext dependencies
    mock_pm = MagicMock()
    mock_llm = MagicMock()
    mock_dspy_lm = MagicMock()
    mock_fs = MagicMock()

    ctx = SharedNodeContext(
        worker_url="http://test",
        session_id="test_session",
        pm=mock_pm,
        llm=mock_llm,
        dspy_lm=mock_dspy_lm,
        worker_client=mock_client,
        fs=mock_fs
    )

    # Use the lifecycle context manager
    async with ctx.lifecycle() as c:
        assert c is ctx
        # client should not be closed yet
        mock_client.aclose.assert_not_called()

    # After exiting, client should be closed
    mock_client.aclose.assert_awaited_once()

if __name__ == "__main__":
    pytest.main([__file__])
