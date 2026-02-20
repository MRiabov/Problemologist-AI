from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from worker_heavy.app import app

client = TestClient(app)


@patch("worker_heavy.api.routes.create_filesystem_router")
@patch("worker_heavy.api.routes.run_verification_task")
def test_benchmark_verify_success(mock_verify, mock_get_router):
    mock_verify.return_value = {
        "num_runs": 5,
        "success_count": 5,
        "success_rate": 1.0,
        "is_consistent": True,
        "individual_results": [],
        "fail_reasons": []
    }
    mock_router = MagicMock()
    from pathlib import Path
    mock_router.local_backend.root = Path("/tmp")
    mock_get_router.return_value = mock_router

    response = client.post(
        "/benchmark/verify",
        json={
            "script_path": "impl.py",
            "num_runs": 5,
            "jitter_range": [0.002, 0.002, 0.001]
        },
        headers={"X-Session-ID": "test-session"}
    )

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert "Success rate 100.0%" in data["message"]
    assert "Consistent: True" in data["message"]
    assert data["artifacts"]["verification_result"]["success_rate"] == 1.0

@patch("worker_heavy.api.routes.create_filesystem_router")
@patch("worker_heavy.api.routes.run_verification_task")
def test_benchmark_verify_failure(mock_verify, mock_get_router):
    mock_verify.side_effect = Exception("Verification crashed")
    mock_router = MagicMock()
    from pathlib import Path
    mock_router.local_backend.root = Path("/tmp")
    mock_get_router.return_value = mock_router

    response = client.post(
        "/benchmark/verify",
        json={"script_path": "impl.py"},
        headers={"X-Session-ID": "test-session"}
    )

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is False
    assert "Verification crashed" in data["message"]
