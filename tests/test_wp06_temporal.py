import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from fastapi.testclient import TestClient
from src.controller.api.main import app, temporal_client_instance

@pytest.fixture
def mock_temporal_client():
    with patch("src.controller.api.main.temporal_client_instance", new_callable=AsyncMock) as mock:
        yield mock

def test_simulation_run_endpoint(mock_temporal_client):
    client = TestClient(app)
    
    # Mock start_workflow
    mock_temporal_client.start_workflow = AsyncMock()
    mock_temporal_client.start_workflow.return_value.id = "test-workflow-id"
    
    response = client.post(
        "/simulation/run",
        json={"session_id": "test-session", "compound_json": '{"key": "value"}'}
    )
    
    assert response.status_code == 200
    assert response.json()["status"] == "accepted"
    assert response.json()["workflow_id"] == "test-workflow-id"
    
    mock_temporal_client.start_workflow.assert_called_once()

@pytest.mark.asyncio
async def test_remote_fs_middleware_temporal():
    from src.controller.middleware.remote_fs import RemoteFilesystemMiddleware
    
    mock_worker_client = MagicMock()
    mock_worker_client.session_id = "test-session"
    mock_temporal_client = AsyncMock()
    mock_temporal_client.execute_workflow.return_value = {"stdout": "success"}
    
    middleware = RemoteFilesystemMiddleware(mock_worker_client, temporal_client=mock_temporal_client)
    
    result = await middleware.run_command("print(1)", timeout=10)
    
    assert result == {"stdout": "success"}
    mock_temporal_client.execute_workflow.assert_called_once()
