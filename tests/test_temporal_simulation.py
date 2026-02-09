from unittest.mock import AsyncMock, MagicMock

import pytest


@pytest.mark.asyncio
async def test_remote_fs_middleware_temporal():
    from controller.middleware.remote_fs import RemoteFilesystemMiddleware

    mock_worker_client = MagicMock()
    mock_worker_client.session_id = "test-session"
    mock_temporal_client = AsyncMock()
    mock_temporal_client.execute_workflow.return_value = {"stdout": "success"}

    middleware = RemoteFilesystemMiddleware(
        mock_worker_client, temporal_client=mock_temporal_client
    )

    result = await middleware.run_command("print(1)", timeout=10)

    assert result == {"stdout": "success"}
    mock_temporal_client.execute_workflow.assert_called_once()
