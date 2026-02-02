import os
import pytest
from fastapi.testclient import TestClient
from src.simulation_engine.main import app
from src.compiler.mujoco_bridge import MujocoBridge
from tests.fixtures.pusher_bot import create_pusher_geometry, PUSHER_SCRIPT

client = TestClient(app)


def test_pusher_e2e_success(tmp_path):
    """
    Test that a simple pusher can push the object into a success zone.
    """
    bridge = MujocoBridge()

    # 1. Prepare Geometry
    mesh_path = str(tmp_path / "pusher.stl")
    create_pusher_geometry(mesh_path)

    # 2. Inject into XML
    base_xml = bridge.load_template()
    # Position pusher at (0, 0, 0.1)
    injected_xml = bridge.inject_design(base_xml, mesh_path, location=(0, 0, 0.1))

    # 3. Define Goal (slightly forward in X)
    # The pusher applies force in X, so it should move to positive X.
    goal_pos = (1.0, 0.0, 0.1)

    payload = {
        "mjcf_xml": injected_xml,
        "agent_script": PUSHER_SCRIPT,
        "duration": 1.0,
        "goal_pos": goal_pos,
        "config": {"timeout": 10.0},
    }

    # 4. Run Simulation via API
    response = client.post("/simulate", json=payload)
    assert response.status_code == 200

    data = response.json()
    assert data["success"] is True
    assert data["outcome"] == "success"
    assert data["result"]["success"] is True
    assert data["result"]["energy"] > 0
    assert "replay_data" in data["result"]
    assert len(data["result"]["replay_data"]) > 0
