from unittest.mock import MagicMock, patch

import pytest
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
@patch("worker.api.routes.validate_and_price")
def test_benchmark_analyze_success(mock_validate_and_price, mock_load):
    mock_load.return_value = MagicMock()
    mock_validate_and_price.return_value = WorkbenchResult(
        is_manufacturable=True,
        unit_cost=10.0,
        violations=[],
        metadata={"dof_count": 0}
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
    assert "events" in data


@patch("worker.api.routes._load_component")
def test_benchmark_analyze_error(mock_load):
    mock_load.side_effect = Exception("Failed to load component")

    response = client.post(
        "/benchmark/analyze",
        json={
            "method": "cnc",
            "config": {"defaults": {}},
            "script_path": "missing.py"
        }
    )

    assert response.status_code == 200
    data = response.json()
    assert data["result"]["is_manufacturable"] is False
    assert "Failed to load component" in data["result"]["violations"][0]
