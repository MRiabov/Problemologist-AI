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
)


@pytest.fixture
def mock_worker_client():
    client = MagicMock(spec=WorkerClient)
    client.session_id = str(uuid.uuid4())
    client.simulate = AsyncMock()
    client.validate = AsyncMock()
    client.submit = AsyncMock()
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
