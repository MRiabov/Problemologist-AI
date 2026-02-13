import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.api.tasks import execute_agent_task
from controller.observability.database import DatabaseCallbackHandler
from controller.persistence.models import Episode, Trace
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
    mock_session.get = AsyncMock()
    mock_session.refresh = AsyncMock()

    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory
    mock_db_sessionmaker.return_value = mock_session_factory

    mock_episode = Episode(id=episode_id, task=task, status=EpisodeStatus.RUNNING)
    # Mock db.get to return our episode
    mock_session.get.return_value = mock_episode

    mock_backend = mock_backend_cls.return_value
    mock_backend.als_info = AsyncMock(
        return_value=[
            {"path": "script.py", "is_dir": False},
            {"path": "output.mjcf", "is_dir": False},
        ]
    )
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
    assert mock_episode.plan is not None

    # Verify traces were added
    # 1 for start, 1 for finish
    assert mock_session.add.call_count >= 3  # Initial trace, final trace, python asset

    # Verify callbacks passed to agent include DB callback and Langfuse callback
    ainvoke_kwargs = mock_agent.ainvoke.call_args.kwargs
    callbacks = ainvoke_kwargs["config"]["callbacks"]
    assert len(callbacks) == 2
    assert isinstance(callbacks[0], DatabaseCallbackHandler)
    assert callbacks[0].langfuse_callback is langfuse_callback
    assert callbacks[1] is langfuse_callback

    # Verify trace records are linked to a single generated Langfuse trace ID
    added_objects = [
        call.args[0] for call in mock_session.add.call_args_list if call.args
    ]
    trace_rows = [obj for obj in added_objects if isinstance(obj, Trace)]
    assert len(trace_rows) >= 2
    langfuse_ids = {trace.langfuse_trace_id for trace in trace_rows}
    assert len(langfuse_ids) == 1
    trace_id = next(iter(langfuse_ids))
    assert trace_id is not None
    assert len(trace_id) == 32

    # Verify commit was called
    assert mock_session.commit.call_count >= 2


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
    mock_session.refresh = AsyncMock()
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

    callbacks = mock_agent.ainvoke.call_args.kwargs["config"]["callbacks"]
    assert len(callbacks) == 1
    assert isinstance(callbacks[0], DatabaseCallbackHandler)
    assert callbacks[0].langfuse_callback is None


@pytest.mark.asyncio
@patch("controller.api.tasks.get_sessionmaker")
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
@patch("controller.api.tasks.get_sessionmaker")
@patch("controller.api.tasks.get_worker_client")
@patch("controller.api.tasks.create_agent_graph")
@patch("controller.api.tasks.initialize_agent_files")
async def test_execute_agent_task_failure(
    mock_init_files, mock_create_graph, mock_get_worker, mock_get_sessionmaker
):
    episode_id = uuid.uuid4()

    mock_session = MagicMock()
    mock_session.commit = AsyncMock()
    mock_session.refresh = AsyncMock()
    mock_session.get = AsyncMock()

    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory

    mock_episode = Episode(id=episode_id, task="task", status=EpisodeStatus.RUNNING)
    mock_session.get.return_value = mock_episode

    # Make agent raise an exception
    mock_agent = AsyncMock()
    mock_agent.ainvoke.side_effect = Exception("Agent crashed")
    mock_create_graph.return_value = (mock_agent, MagicMock())

    await execute_agent_task(episode_id, "task", "session")

    # Verify status changed to FAILED
    assert mock_episode.status == EpisodeStatus.FAILED


@pytest.mark.asyncio
@patch("controller.api.tasks.get_sessionmaker")
@patch("controller.observability.database.get_sessionmaker")
@patch("controller.api.tasks.get_worker_client")
@patch("controller.api.tasks.create_agent_graph")
@patch("controller.api.tasks.RemoteFilesystemBackend")
@patch("controller.api.tasks.initialize_agent_files")
async def test_execute_agent_task_custom_name(
    mock_init_files,
    mock_backend_cls,
    mock_create_graph,
    _mock_get_worker,
    mock_db_sessionmaker,
    mock_get_sessionmaker,
):
    episode_id = uuid.uuid4()
    task = "test task"
    session_id = "test-session"
    agent_name = "benchmark_planner"

    mock_session = MagicMock()
    mock_session.commit = AsyncMock()
    mock_session.refresh = AsyncMock()
    mock_session.get = AsyncMock()

    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_session
    mock_get_sessionmaker.return_value = mock_session_factory
    mock_db_sessionmaker.return_value = mock_session_factory

    mock_episode = Episode(id=episode_id, task=task, status=EpisodeStatus.RUNNING)
    mock_session.get.return_value = mock_episode

    mock_backend = mock_backend_cls.return_value
    mock_backend.als_info = AsyncMock(return_value=[])

    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {"messages": [MagicMock(content="done")]}
    mock_create_graph.return_value = (mock_agent, None)

    await execute_agent_task(episode_id, task, session_id, agent_name=agent_name)

    # Verify initialize_agent_files called with custom name
    mock_init_files.assert_called_once_with(mock_backend, agent_name=agent_name)

    # Verify create_agent_graph called with custom name
    mock_create_graph.assert_called_once()
    assert mock_create_graph.call_args.kwargs["agent_name"] == agent_name
