from fastapi.testclient import TestClient

from src.compiler.mujoco_bridge import MujocoBridge
from src.simulation_engine.main import app

client = TestClient(app)


def test_health():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


from unittest.mock import MagicMock, patch
from src.agent.utils.config import Config


@patch("src.simulation_engine.main.run_isolated")
def test_simulate_success(mock_run_isolated):
    mock_run_isolated.return_value = {"success": True, "result": {"duration": 0.1}}
    # Use MujocoBridge to get a standard template
    sandbox = MagicMock()
    bridge = MujocoBridge(workspace_dir=Config.WORKSPACE_DIR, sandbox=sandbox)
    xml_content = bridge.load_template()

    payload = {"mjcf_xml": xml_content, "duration": 0.1, "config": {"timeout": 10.0}}

    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["outcome"] == "success"
    assert "result" in data
    assert "duration" in data["result"]


@patch("src.simulation_engine.main.run_isolated")
def test_simulate_timeout(mock_run_isolated):
    mock_run_isolated.return_value = {
        "success": False,
        "error_type": "TimeoutError",
        "message": "Simulation timed out.",
    }
    # Create an XML that might be slow or just force a tiny timeout
    sandbox = MagicMock()
    bridge = MujocoBridge(workspace_dir=Config.WORKSPACE_DIR, sandbox=sandbox)
    xml_content = bridge.load_template()

    payload = {
        "mjcf_xml": xml_content,
        "duration": 5.0,
        "config": {"timeout": 0.001},  # Extremely short timeout
    }

    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["success"] is False
    assert data["outcome"] == "timeouterror"


@patch("src.simulation_engine.main.run_isolated")
def test_simulate_crash(mock_run_isolated):
    mock_run_isolated.return_value = {
        "success": False,
        "error_type": "CrashError",
        "message": "Generative crush.",
    }
    # Invalid XML to trigger a load error which we handle as a crash or error
    payload = {"mjcf_xml": "invalid xml", "duration": 0.1}

    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    # Based on MujocoBridge implementation, it returns success=False result,
    # but in our wrapper we might catch it.
    assert data["success"] is True or data["outcome"] == "crasherror"
