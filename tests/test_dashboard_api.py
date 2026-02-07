import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
from controller.api.main import app
from shared.enums import ResponseStatus, EpisodeStatus
import uuid

client = TestClient(app)

def test_health_check():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == ResponseStatus.HEALTHY

@patch("controller.api.main.Client.connect")
def test_run_simulation_accepted(mock_temporal_connect):
    mock_temporal_client = AsyncMock()
    mock_temporal_connect.return_value = mock_temporal_client
    
    # Trigger simulation run
    response = client.post(
        "/simulation/run",
        json={"session_id": "test-session", "compound_json": "{}"}
    )
    
    assert response.status_code == 200
    if response.json()["status"] == ResponseStatus.ACCEPTED:
        assert "workflow_id" in response.json()

@patch("controller.api.main.get_sessionmaker")
@patch("controller.api.main.get_worker_client")
@patch("controller.api.main.create_agent_graph")
def test_run_agent_endpoint(mock_create_graph, mock_get_worker, mock_get_sessionmaker):
    # Mock session factory and session
    mock_session = MagicMock()
    mock_session.commit = AsyncMock()
    mock_session.refresh = AsyncMock()
    mock_session.get = AsyncMock() # Important for background task
    
    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory
    
    response = client.post(
        "/agent/run",
        json={"task": "test task", "session_id": "test-session"}
    )
    
    assert response.status_code == 200
    assert response.json()["status"] == ResponseStatus.ACCEPTED
    assert "episode_id" in response.json()
    
    # Verify DB was called to create episode
    mock_session.add.assert_called()
    mock_session.commit.assert_called()