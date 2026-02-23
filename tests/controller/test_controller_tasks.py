import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.api.tasks import execute_agent_task
from controller.persistence.models import Episode
from shared.enums import EpisodeStatus


@pytest.mark.asyncio
@patch("controller.api.tasks.get_sessionmaker")
@patch("controller.observability.database.get_sessionmaker")
@patch("controller.api.tasks.get_worker_client")
@patch("controller.api.tasks.create_agent_graph")
@patch("controller.api.tasks.RemoteFilesystemBackend")
@patch("controller.api.tasks.initialize_agent_files")
async def test_execute_agent_task_success(
    mock_init_files,
    mock_backend_cls,
    mock_create_graph,
    _mock_get_worker,
    mock_db_sessionmaker,
    mock_get_sessionmaker,
):
    # Setup mocks
    episode_id = uuid.uuid4()
    task = "test task"
    session_id = "test-session"

    mock_session = MagicMock()
    mock_session.commit = AsyncMock()

    async def mock_refresh(obj):
        if hasattr(obj, "id") and obj.id is None:
            obj.id = uuid.uuid4()
        return

    mock_session.refresh = AsyncMock(side_effect=mock_refresh)

    def mock_add(obj):
        if hasattr(obj, "id") and obj.id is None:
            obj.id = uuid.uuid4()

    mock_session.add = MagicMock(side_effect=mock_add)
    mock_session.get = AsyncMock()

    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory
    mock_db_sessionmaker.return_value = mock_session_factory

    mock_episode = Episode(id=episode_id, task=task, status=EpisodeStatus.RUNNING)
    mock_session.get.return_value = mock_episode

    mock_backend = mock_backend_cls.return_value
    mock_backend.als_info = AsyncMock(return_value=[])
    mock_backend.aread = AsyncMock(return_value="print('hello')")

    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {
        "messages": [MagicMock(content="agent finished")]
    }
    langfuse_callback = MagicMock()
    mock_create_graph.return_value = (mock_agent, langfuse_callback)

    # Execute the task
    await execute_agent_task(episode_id, task, session_id)

    # Verify episode was updated
    assert mock_episode.status == EpisodeStatus.COMPLETED


@pytest.mark.asyncio
@patch("controller.api.tasks.get_sessionmaker")
@patch("controller.observability.database.get_sessionmaker")
@patch("controller.api.tasks.get_worker_client")
@patch("controller.api.tasks.create_agent_graph")
@patch("controller.api.tasks.RemoteFilesystemBackend")
@patch("controller.api.tasks.initialize_agent_files")
async def test_execute_agent_task_without_langfuse_callback(
    _mock_init_files,
    mock_backend_cls,
    mock_create_graph,
    _mock_get_worker,
    mock_db_sessionmaker,
    mock_get_sessionmaker,
):
    episode_id = uuid.uuid4()
    mock_session = MagicMock()
    mock_session.commit = AsyncMock()

    async def mock_refresh(obj):
        if hasattr(obj, "id") and obj.id is None:
            obj.id = uuid.uuid4()
        return

    mock_session.refresh = AsyncMock(side_effect=mock_refresh)

    def mock_add(obj):
        if hasattr(obj, "id") and obj.id is None:
            obj.id = uuid.uuid4()

    mock_session.add = MagicMock(side_effect=mock_add)

    mock_session.get = AsyncMock(
        return_value=Episode(id=episode_id, task="task", status=EpisodeStatus.RUNNING)
    )

    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory
    mock_db_sessionmaker.return_value = mock_session_factory

    mock_backend = mock_backend_cls.return_value
    mock_backend.als_info = AsyncMock(return_value=[])

    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {"messages": [MagicMock(content="done")]}
    mock_create_graph.return_value = (mock_agent, None)

    await execute_agent_task(episode_id, "task", "session")

    # Verify callbacks were passed (empty if langfuse is disabled)
    assert mock_agent.ainvoke.called
    args, kwargs = mock_agent.ainvoke.call_args
    callbacks = kwargs["config"]["callbacks"]
    assert len(callbacks) == 0
