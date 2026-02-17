from unittest.mock import AsyncMock, MagicMock, patch

from fastapi.testclient import TestClient

from controller.api.main import app
from shared.enums import ResponseStatus

client = TestClient(app)


def test_health_check():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == ResponseStatus.HEALTHY


@patch("controller.api.main.get_sessionmaker")
@patch("controller.api.main.execute_agent_task")
def test_run_agent_endpoint(mock_execute_task, mock_get_sessionmaker):
    # Mock session factory and session
    mock_session = MagicMock()
    mock_session.commit = AsyncMock()
    mock_session.refresh = AsyncMock()
    mock_session.get = AsyncMock()  # Important for background task

    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory

    response = client.post(
        "/agent/run", json={"task": "test task", "session_id": "test-session"}
    )

    assert response.status_code == 202
    assert response.json()["status"] == ResponseStatus.ACCEPTED
    assert "episode_id" in response.json()

    # Verify DB was called to create episode
    mock_session.add.assert_called()
    mock_session.commit.assert_called()

    # Verify background task was created
    mock_execute_task.assert_called()
