import pytest
from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient

from worker.api.routes import get_router
from worker.app import app
from worker.workbenches.models import WorkbenchResult

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
def test_benchmark_analyze_success(mock_validate_and_price, mock_load):
    mock_load.return_value = MagicMock()
    mock_validate_and_price.return_value = WorkbenchResult(
        is_manufacturable=True,
        unit_cost=10.0,
        violations=[],
        metadata={"key": "value"}
    )

    response = client.post(
        "/benchmark/analyze",
        json={
            "method": "cnc",
            "config": {"defaults": {}},
            "script_path": "impl.py"
        }
    )

    assert response.status_code == 200
    data = response.json()
    assert data["result"]["is_manufacturable"] is True
    assert data["result"]["unit_cost"] == 10.0
    assert data["result"]["violations"] == []
    assert data["result"]["metadata"] == {"key": "value"}
