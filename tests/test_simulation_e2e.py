from fastapi.testclient import TestClient

from src.agent.utils.config import Config
from src.compiler.mujoco_bridge import MujocoBridge
from src.simulation_engine.main import app
from tests.fixtures.pusher_bot import PUSHER_SCRIPT, create_pusher_geometry

client = TestClient(app)


def test_pusher_e2e_success(tmp_path):
    """
    Test that a simple pusher can push the object into a success zone.
    """
    bridge = MujocoBridge()

    # 1. Prepare Geometry in the workspace
    workspace_dir = Config.WORKSPACE_DIR.resolve()
    mesh_filename = "pusher.stl"
    mesh_path = str(workspace_dir / mesh_filename)
    create_pusher_geometry(mesh_path)

    # 2. Inject into XML
    base_xml = bridge.load_template()
    # Position pusher at (0, 0, 0.1)
    # We use a RELATIVE path because the sandbox mounts workspace_dir to /workspace
    injected_xml = bridge.inject_design(base_xml, mesh_filename, location=(0, 0, 0.1))

    # 3. Define Goal (slightly forward in X)
    # The pusher applies force in X, so it should move to positive X.
    goal_pos = (1.0, 0.0, 0.1)

    payload = {
        "mjcf_xml": injected_xml,
        "agent_script": PUSHER_SCRIPT,
        "duration": 1.0,
        "goal_pos": goal_pos,
        "config": {"timeout": 30.0},
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
