import asyncio
import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from controller.api.main import app
from controller.api.routes.episodes import get_db
from controller.api.tasks import execute_agent_task
from controller.persistence.models import Episode
from shared.enums import EpisodeStatus


@pytest.fixture
def mock_db():
    session = AsyncMock()
    app.dependency_overrides[get_db] = lambda: session
    yield session
    app.dependency_overrides.clear()


client = TestClient(app)


@pytest.mark.asyncio
@patch("controller.api.tasks.get_sessionmaker")
@patch("controller.api.tasks.get_worker_client")
@patch("controller.api.tasks.create_agent_graph")
@patch("controller.api.tasks.RemoteFilesystemBackend")
@patch("controller.api.tasks.initialize_agent_files")
async def test_execute_agent_task_cancelled(
    mock_init_files,
    mock_backend_cls,
    mock_create_graph,
    mock_get_worker,
    mock_get_sessionmaker,
):
    # Setup mocks
    episode_id = uuid.uuid4()
    task = "test task"
    session_id = "test-session"

    mock_session = MagicMock()
    mock_session.commit = AsyncMock()
    mock_session.get = AsyncMock()
    mock_session.add = MagicMock()

    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory

    mock_episode = Episode(id=episode_id, task=task, status=EpisodeStatus.RUNNING)
    mock_session.get.return_value = mock_episode

    mock_backend = mock_backend_cls.return_value
    mock_backend.als_info = AsyncMock(return_value=[])

    # Mock agent to raise CancelledError
    mock_agent = AsyncMock()
    mock_agent.ainvoke.side_effect = asyncio.CancelledError()
    mock_create_graph.return_value = (mock_agent, MagicMock())

    # Execute the task
    with pytest.raises(asyncio.CancelledError):
        await execute_agent_task(episode_id, task, session_id)

    # Verify episode status was updated to CANCELLED
    assert mock_episode.status == EpisodeStatus.CANCELLED

    # Verify traces were added (initial + cancelled)
    assert mock_session.add.call_count >= 2

    # Verify commit was called
    assert mock_session.commit.call_count >= 2


@pytest.mark.asyncio
async def test_interrupt_episode_success(mock_db):
    episode_id = uuid.uuid4()

    # Mock existing running episode
    mock_episode = Episode(id=episode_id, task="task", status=EpisodeStatus.RUNNING)
    mock_result = MagicMock()
    mock_result.scalar_one_or_none.return_value = mock_episode
    mock_db.execute.return_value = mock_result
    mock_db.commit = AsyncMock()

    # Mock TaskTracker
    mock_task = MagicMock()
    mock_task.done.return_value = False

    with patch("controller.api.routes.episodes.task_tracker") as mock_tracker:
        mock_tracker.get_task.return_value = mock_task

        # Call API
        response = client.post(f"/episodes/{episode_id}/interrupt")

        # Verify response
        assert response.status_code == 200
        assert response.json()["status"] == "accepted"

        # Verify task cancellation
        mock_task.cancel.assert_called_once()

        # Verify DB status update
        assert mock_episode.status == EpisodeStatus.CANCELLED
        mock_db.commit.assert_called_once()
