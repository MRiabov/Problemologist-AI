from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from worker.api.routes import get_router
from worker.app import app
from worker.workbenches.models import ManufacturingMethod, WorkbenchResult, WorkbenchMetadata, CostBreakdown

pytestmark = [pytest.mark.integration, pytest.mark.xdist_group(name="physics_sims")]

client = TestClient(app)

@pytest.fixture(autouse=True)
def mock_get_router():
    from pathlib import Path
    mock = MagicMock()
    # Mock the filesystem backend
    mock.local_backend._resolve.side_effect = lambda x: Path("/tmp") / x
    mock.local_backend.root = Path("/tmp")
    app.dependency_overrides[get_router] = lambda: mock
    yield mock
    app.dependency_overrides.clear()

@patch("worker.api.routes.load_component_from_script")
@patch("worker.api.routes.asyncio.get_running_loop")
def test_benchmark_simulate_success(mock_get_loop, mock_load):
    mock_load.return_value = MagicMock()
    mock_loop = MagicMock()
    mock_get_loop.return_value = mock_loop

    mock_result = MagicMock()
    mock_result.success = True
    mock_result.summary = "Stability check passed"
    mock_result.render_paths = ["/path/to/video.mp4"]
    mock_result.mjcf_content = "mjcf"
    mock_result.stress_summaries = []
    mock_result.fluid_metrics = []
    mock_result.confidence = "high"

    # Mock run_in_executor to return our mock_result
    async def mock_run_in_executor(executor, func, *args, **kwargs):
        return mock_result
    mock_loop.run_in_executor.side_effect = mock_run_in_executor

    response = client.post("/benchmark/simulate",
                           json={"script_path": "impl.py"},
                           headers={"X-Session-ID": "test-session"})

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["message"] == "Stability check passed"
    assert "render_paths" in data["artifacts"]

@patch("worker.api.routes.load_component_from_script")
@patch("worker.api.routes.validate")
@patch("worker.api.routes.validate_fem_manufacturability")
def test_benchmark_validate_success(mock_fem_validate, mock_validate, mock_load):
    mock_load.return_value = MagicMock()
    mock_validate.return_value = (True, "Validation successful")
    mock_fem_validate.return_value = (True, "")

    response = client.post("/benchmark/validate",
                           json={"script_path": "impl.py"},
                           headers={"X-Session-ID": "test-session"})

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert "Validation successful" in response.json()["message"]

@patch("worker.api.routes.asyncio.get_running_loop")
def test_benchmark_simulate_error(mock_get_loop):
    mock_loop = MagicMock()
    mock_get_loop.return_value = mock_loop

    async def mock_run_in_executor(executor, func, *args, **kwargs):
        raise Exception("Failed to load component")
    mock_loop.run_in_executor.side_effect = mock_run_in_executor

    response = client.post("/benchmark/simulate",
                           json={"script_path": "missing.py"},
                           headers={"X-Session-ID": "test-session"})

    assert response.status_code == 200
    assert response.json()["success"] is False
    assert "Failed to load component" in response.json()["message"]

@patch("worker.api.routes.load_component_from_script")
@patch("worker.utils.dfm.validate_and_price")
@patch("worker.workbenches.config.load_config")
def test_benchmark_analyze_success(
    mock_load_config, mock_validate_and_price, mock_load
):
    mock_load.return_value = MagicMock()
    mock_load_config.return_value = MagicMock()

    mock_result = WorkbenchResult(
        is_manufacturable=True,
        unit_cost=10.5,
        violations=[],
        metadata=WorkbenchMetadata(
            cost_breakdown=CostBreakdown(
                process="cnc",
                total_cost=10.5,
                unit_cost=10.5,
                material_cost_per_unit=5.0,
                setup_cost=5.5,
                is_reused=False
            )
        ),
    )
    mock_validate_and_price.return_value = mock_result

    # 1. Test default quantity (1)
    response = client.post(
        "/benchmark/analyze",
        json={"script_path": "impl.py", "method": "cnc"},
        headers={"X-Session-ID": "test-session"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["unit_cost"] == 10.5

    mock_validate_and_price.assert_called()
    args, kwargs = mock_validate_and_price.call_args
    assert kwargs.get("method") == ManufacturingMethod.CNC
    assert kwargs.get("quantity") == 1

    # 2. Test custom quantity
    response = client.post(
        "/benchmark/analyze",
        json={"script_path": "impl.py", "method": "cnc", "quantity": 100},
        headers={"X-Session-ID": "test-session"}
    )
    assert response.status_code == 200

    args, kwargs = mock_validate_and_price.call_args
    assert kwargs.get("quantity") == 100
