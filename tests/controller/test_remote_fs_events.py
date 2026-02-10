import pytest
import uuid
from unittest.mock import MagicMock, AsyncMock, patch
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.clients.worker import WorkerClient
from shared.observability.schemas import (
    SimulationRequestEvent,
    SimulationResultEvent,
    ManufacturabilityCheckEvent,
    PlanSubmissionEngineerEvent,
    LsFilesToolEvent,
    ReadFileToolEvent,
    WriteFileToolEvent,
    EditFileToolEvent,
    RunCommandToolEvent,
    GrepToolEvent,
    SkillReadEvent,
)
from worker.api.schema import EditOp


@pytest.fixture
def mock_worker_client():
    client = MagicMock(spec=WorkerClient)
    client.session_id = str(uuid.uuid4())
    client.simulate = AsyncMock()
    client.validate = AsyncMock()
    client.submit = AsyncMock()
    client.list_files = AsyncMock()
    client.read_file = AsyncMock()
    client.write_file = AsyncMock()
    client.edit_file = AsyncMock()
    client.execute_python = AsyncMock()
    client.grep = AsyncMock()
    return client


@pytest.mark.asyncio
async def test_simulate_emits_events(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)

    mock_worker_client.simulate.return_value = MagicMock(
        model_dump=lambda: {
            "success": True,
            "time_elapsed_s": 1.5,
            "compute_time_ms": 100,
        }
    )

    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.simulate("test_plan.py")

        # Check request event
        calls = mock_record.call_args_list
        assert len(calls) == 2

        # Keyword arguments check
        assert isinstance(calls[0].kwargs["events"][0], SimulationRequestEvent)
        assert calls[0].kwargs["events"][0].script_path == "test_plan.py"

        # Check result event
        assert isinstance(calls[1].kwargs["events"][0], SimulationResultEvent)
        assert calls[1].kwargs["events"][0].success is True


@pytest.mark.asyncio
async def test_validate_emits_events(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)

    mock_worker_client.validate.return_value = MagicMock(
        model_dump=lambda: {"success": True, "price": 50.0}
    )

    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.validate("test_plan.py")

        mock_record.assert_called_once()
        assert isinstance(
            mock_record.call_args.kwargs["events"][0], ManufacturabilityCheckEvent
        )
        assert mock_record.call_args.kwargs["events"][0].result is True


@pytest.mark.asyncio
async def test_submit_emits_events(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)

    mock_worker_client.submit.return_value = MagicMock(
        model_dump=lambda: {"success": True}
    )

    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.submit("test_plan.py")

        mock_record.assert_called_once()
        assert isinstance(
            mock_record.call_args.kwargs["events"][0], PlanSubmissionEngineerEvent
        )
        assert mock_record.call_args.kwargs["events"][0].plan_path == "test_plan.py"


@pytest.mark.asyncio
async def test_ls_files_emits_event(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)
    mock_worker_client.list_files.return_value = []

    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.list_files("/test")
        mock_record.assert_called_once()
        assert isinstance(mock_record.call_args.kwargs["events"][0], LsFilesToolEvent)
        assert mock_record.call_args.kwargs["events"][0].path == "/test"


@pytest.mark.asyncio
async def test_read_file_emits_events(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)
    mock_worker_client.read_file.return_value = "content"

    # Test regular file
    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.read_file("test.txt")
        mock_record.assert_called_once()
        assert isinstance(mock_record.call_args.kwargs["events"][0], ReadFileToolEvent)

    # Test skill file
    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.read_file("skills/my_skill/SKILL.md")
        assert mock_record.call_once
        events = mock_record.call_args.kwargs["events"]
        assert len(events) == 2
        assert isinstance(events[0], ReadFileToolEvent)
        assert isinstance(events[1], SkillReadEvent)
        assert events[1].skill_name == "my_skill"


@pytest.mark.asyncio
async def test_write_file_emits_event(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)
    mock_worker_client.write_file.return_value = True

    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.write_file("test.txt", "new content")
        mock_record.assert_called_once()
        assert isinstance(mock_record.call_args.kwargs["events"][0], WriteFileToolEvent)
        assert (
            mock_record.call_args.kwargs["events"][0].content_snippet == "new content"
        )


@pytest.mark.asyncio
async def test_edit_file_emits_event(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)
    mock_worker_client.edit_file.return_value = True

    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.edit_file("test.txt", [EditOp(old_string="a", new_string="b")])
        mock_record.assert_called_once()
        assert isinstance(mock_record.call_args.kwargs["events"][0], EditFileToolEvent)
        assert mock_record.call_args.kwargs["events"][0].num_edits == 1


@pytest.mark.asyncio
async def test_grep_emits_event(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)
    mock_worker_client.grep.return_value = []

    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.grep("pattern", path="/", glob="*.py")
        mock_record.assert_called_once()
        assert isinstance(mock_record.call_args.kwargs["events"][0], GrepToolEvent)
        assert mock_record.call_args.kwargs["events"][0].pattern == "pattern"


@pytest.mark.asyncio
async def test_run_command_emits_event(mock_worker_client):
    middleware = RemoteFilesystemMiddleware(mock_worker_client)
    mock_worker_client.execute_python.return_value = MagicMock(model_dump=lambda: {})

    with patch(
        "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
    ) as mock_record:
        await middleware.run_command("print('test')")
        mock_record.assert_called_once()
        assert isinstance(
            mock_record.call_args.kwargs["events"][0], RunCommandToolEvent
        )
        assert mock_record.call_args.kwargs["events"][0].command == "print('test')"
