import pytest
import uuid
from unittest.mock import AsyncMock, MagicMock, patch
from controller.api.main import execute_agent_task
from controller.persistence.models import Episode
from shared.enums import EpisodeStatus, AssetType

@pytest.mark.asyncio
@patch("controller.api.main.get_sessionmaker")
@patch("controller.api.main.get_worker_client")
@patch("controller.api.main.create_agent_graph")
@patch("controller.api.main.RemoteFilesystemMiddleware")
async def test_execute_agent_task_success(
    mock_middleware_cls, mock_create_graph, mock_get_worker, mock_get_sessionmaker
):
    # Setup mocks
    episode_id = uuid.uuid4()
    task = "test task"
    session_id = "test-session"
    
    mock_session = MagicMock()
    mock_session.commit = AsyncMock()
    mock_session.get = AsyncMock()
    
    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory
    
    mock_episode = Episode(id=episode_id, task=task, status=EpisodeStatus.RUNNING)
    # Mock db.get to return our episode
    mock_session.get.return_value = mock_episode
    
    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {"messages": [MagicMock(content="agent finished")]}
    mock_create_graph.return_value = mock_agent
    
    # Execute the task
    await execute_agent_task(episode_id, task, session_id)
    
    # Verify episode was updated
    assert mock_episode.status == EpisodeStatus.COMPLETED
    assert mock_episode.plan is not None
    
    # Verify traces were added
    # 1 for start, 1 for finish
    assert mock_session.add.call_count >= 3 # Initial trace, final trace, python asset
    
    # Verify commit was called
    assert mock_session.commit.call_count >= 2

@pytest.mark.asyncio
@patch("controller.api.main.get_sessionmaker")
async def test_execute_agent_task_not_found(mock_get_sessionmaker):
    episode_id = uuid.uuid4()
    
    mock_session = MagicMock()
    mock_session.get = AsyncMock()
    
    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory
    
    # Mock db.get to return None
    mock_session.get.return_value = None
    
    await execute_agent_task(episode_id, "task", "session")
    
    # Should just return without error
    mock_session.commit.assert_not_called()

@pytest.mark.asyncio
@patch("controller.api.main.get_sessionmaker")
@patch("controller.api.main.get_worker_client")
@patch("controller.api.main.create_agent_graph")
async def test_execute_agent_task_failure(
    mock_create_graph, mock_get_worker, mock_get_sessionmaker
):
    episode_id = uuid.uuid4()
    
    mock_session = MagicMock()
    mock_session.commit = AsyncMock()
    mock_session.get = AsyncMock()
    
    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory
    
    mock_episode = Episode(id=episode_id, task="task", status=EpisodeStatus.RUNNING)
    mock_session.get.return_value = mock_episode
    
    # Make agent raise an exception
    mock_agent = AsyncMock()
    mock_agent.ainvoke.side_effect = Exception("Agent crashed")
    mock_create_graph.return_value = mock_agent
    
    await execute_agent_task(episode_id, "task", "session")
    
    # Verify status changed to FAILED
    assert mock_episode.status == EpisodeStatus.FAILED