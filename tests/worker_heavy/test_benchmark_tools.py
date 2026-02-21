from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from worker_heavy.app import app
from shared.workers.workbench_models import ManufacturingMethod, WorkbenchResult

client = TestClient(app)


@patch("worker_heavy.api.routes.create_filesystem_router")
@patch("worker_heavy.api.routes.load_component_from_script")
@patch("worker_heavy.api.routes.run_simulation_task")
def test_benchmark_simulate_success(mock_simulate, mock_load, mock_get_router):
    mock_load.return_value = MagicMock()
    mock_simulate.return_value = MagicMock(
        success=True,
        summary="Stability check passed",
        render_paths=["/path/to/video.mp4"],
        mjcf_content="<mjcf/>",
        stress_summaries=[],
        fluid_metrics=[],
        confidence="high",
    )
    mock_router = MagicMock()
    from pathlib import Path

    mock_router.local_backend.root = Path("/tmp")
    mock_get_router.return_value = mock_router

    response = client.post(
        "/benchmark/simulate",
        json={"script_path": "impl.py"},
        headers={"X-Session-ID": "test-session"},
    )

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["message"] == "Stability check passed"
    assert "render_paths" in data["artifacts"]


@patch("worker_heavy.api.routes.create_filesystem_router")
@patch("worker_heavy.api.routes.load_component_from_script")
@patch("worker_heavy.api.routes.validate")
def test_benchmark_validate_success(mock_validate, mock_load, mock_get_router):
    mock_load.return_value = MagicMock()
    mock_validate.return_value = (True, "Validation successful")
    mock_router = MagicMock()
    from pathlib import Path

    mock_router.local_backend.root = Path("/tmp")
    mock_get_router.return_value = mock_router

    response = client.post(
        "/benchmark/validate",
        json={"script_path": "impl.py"},
        headers={"X-Session-ID": "test-session"},
    )

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert "Validation successful" in response.json()["message"]


@patch("worker_heavy.api.routes.create_filesystem_router")
@patch("worker_heavy.api.routes.load_component_from_script")
@patch("worker_heavy.api.routes.run_simulation_task")
def test_benchmark_simulate_error(mock_simulate, mock_load, mock_get_router):
    mock_simulate.side_effect = Exception("Failed to simulate")
    mock_router = MagicMock()
    from pathlib import Path

    mock_router.local_backend.root = Path("/tmp")
    mock_get_router.return_value = mock_router

    response = client.post(
        "/benchmark/simulate",
        json={"script_path": "missing.py"},
        headers={"X-Session-ID": "test-session"},
    )

    assert response.status_code == 200
    assert response.json()["success"] is False
    assert "Failed to simulate" in response.json()["message"]


@patch("worker_heavy.api.routes.create_filesystem_router")
@patch("worker_heavy.api.routes.load_component_from_script")
@patch("worker_heavy.api.routes.analyze_component")
@patch("worker_heavy.workbenches.config.load_config")
def test_benchmark_analyze_success(
    mock_load_config, mock_analyze, mock_load, mock_get_router
):
    mock_load.return_value = MagicMock()
    mock_load_config.return_value = MagicMock()
    mock_router = MagicMock()
    from pathlib import Path

    mock_router.local_backend.root = Path("/tmp")
    mock_get_router.return_value = mock_router

    from shared.workers.workbench_models import CostBreakdown, WorkbenchMetadata

    mock_result = WorkbenchResult(
        is_manufacturable=True,
        unit_cost=10.5,
        weight_g=100.0,
        violations=[],
        metadata=WorkbenchMetadata(
            cost_breakdown=CostBreakdown(
                process="cnc",
                total_cost=10.5,
                unit_cost=10.5,
                material_cost_per_unit=5.0,
                setup_cost=0.0,
                is_reused=False,
            )
        ),
    )
    mock_analyze.return_value = mock_result

    # 1. Test default quantity (1)
    response = client.post(
        "/benchmark/analyze",
        json={"script_path": "impl.py", "method": "cnc"},
        headers={"X-Session-ID": "test-session"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["unit_cost"] == 10.5

    mock_analyze.assert_called()

    # 2. Test custom quantity (will be extracted from metadata if present)
    response = client.post(
        "/benchmark/analyze",
        json={"script_path": "impl.py", "method": "cnc", "quantity": 100},
        headers={"X-Session-ID": "test-session"},
    )
    assert response.status_code == 200
    mock_analyze.assert_called()
