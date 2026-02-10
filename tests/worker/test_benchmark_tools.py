import pytest
from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient

from worker.api.routes import get_router
from worker.app import app
from worker.workbenches.models import WorkbenchResult, ManufacturingMethod

client = TestClient(app)


@pytest.fixture(autouse=True)
def mock_get_router():
    mock = MagicMock()
    app.dependency_overrides[get_router] = lambda: mock
    yield mock
    app.dependency_overrides.clear()


@patch("worker.api.routes._load_component")
@patch("worker.api.routes.simulate")
def test_benchmark_simulate_success(mock_simulate, mock_load):
    mock_load.return_value = MagicMock()
    mock_simulate.return_value = MagicMock(
        success=True,
        summary="Stability check passed",
        render_paths=["/path/to/video.mp4"],
    )

    response = client.post("/benchmark/simulate", json={"script_path": "impl.py"})

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["message"] == "Stability check passed"
    assert "render_paths" in data["artifacts"]


@patch("worker.api.routes._load_component")
@patch("worker.api.routes.validate")
def test_benchmark_validate_success(mock_validate, mock_load):
    mock_load.return_value = MagicMock()
    mock_validate.return_value = True

    response = client.post("/benchmark/validate", json={"script_path": "impl.py"})

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["message"] == "Validation successful"


@patch("worker.api.routes._load_component")
def test_benchmark_simulate_error(mock_load):
    mock_load.side_effect = Exception("Failed to load component")

    response = client.post("/benchmark/simulate", json={"script_path": "missing.py"})

    assert response.status_code == 200
    assert response.json()["success"] is False
    assert "Failed to load component" in response.json()["message"]


@patch("worker.api.routes._load_component")
@patch("worker.api.routes.validate_and_price")
@patch("worker.api.routes.load_config")
def test_benchmark_analyze_success(mock_load_config, mock_validate_and_price, mock_load):
    mock_load.return_value = MagicMock()
    mock_load_config.return_value = MagicMock()

    mock_result = WorkbenchResult(
        is_manufacturable=True,
        unit_cost=10.5,
        violations=[],
        metadata={"cost_breakdown": {"total": 100.0}}
    )
    mock_validate_and_price.return_value = mock_result

    response = client.post(
        "/benchmark/analyze",
        json={"script_path": "impl.py", "method": "cnc"}
    )

    assert response.status_code == 200
    data = response.json()
    assert data["is_manufacturable"] is True
    assert data["unit_cost"] == 10.5
    assert data["violations"] == []

    # Check that it calls validate_and_price with correct args
    mock_validate_and_price.assert_called_once()
    args = mock_validate_and_price.call_args
    # args[0] is component (mock), args[1] is method
    assert args[0][1] == ManufacturingMethod.CNC
