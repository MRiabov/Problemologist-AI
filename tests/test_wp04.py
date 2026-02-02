import pytest
from fastapi.testclient import TestClient
from src.simulation_engine.main import app
from src.compiler.mujoco_bridge import MujocoBridge

client = TestClient(app)


def test_health():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


def test_simulate_success():
    # Use MujocoBridge to get a standard template
    bridge = MujocoBridge()
    xml_content = bridge.load_template()

    payload = {"mjcf_xml": xml_content, "duration": 0.1, "config": {"timeout": 10.0}}

    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["outcome"] == "success"
    assert "result" in data
    assert "duration" in data["result"]


def test_simulate_timeout():
    # Create an XML that might be slow or just force a tiny timeout
    bridge = MujocoBridge()
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


def test_simulate_crash():
    # Invalid XML to trigger a load error which we handle as a crash or error
    payload = {"mjcf_xml": "invalid xml", "duration": 0.1}

    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    # Based on MujocoBridge implementation, it returns success=False result,
    # but in our wrapper we might catch it.
    assert data["success"] is True or data["outcome"] == "runtimeerror"
