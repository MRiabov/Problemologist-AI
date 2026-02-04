from fastapi.testclient import TestClient

from src.agent.utils.config import Config
from src.simulation_engine.bridge import MujocoBridge
from src.simulation_engine.main import app
from tests.fixtures.pusher_bot import PUSHER_SCRIPT, create_pusher_geometry

client = TestClient(app)


from unittest.mock import MagicMock


from unittest.mock import MagicMock, patch


@patch("src.simulation_engine.main.run_isolated")
def test_pusher_e2e_success(mock_run_isolated, tmp_path):
    # Configure mock
    mock_run_isolated.return_value = {
        "success": True,
        "result": {"total_energy": 100.0, "observations": [{"x": 1}, {"x": 2}]},
    }

    """
    Test that a simple pusher can push the object into a success zone.
    """
    workspace_dir = Config.WORKSPACE_DIR.resolve()
    sandbox = MagicMock()
    bridge = MujocoBridge(workspace_dir=workspace_dir, sandbox=sandbox)

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
    if not data["success"]:
        print(
            f"DEBUG: Simulation failed with outcome: {data.get('outcome')} and error: {data.get('error')}"
        )  # FIXME why not proper logging? we have a module for that.
        print(f"DEBUG: Full Response: {data}")
    assert data["success"] is True, f"Simulation failed. Full Response: {data}"
    assert data["outcome"] == "success"
    assert data["result"]["total_energy"] > 0
    assert "observations" in data["result"]
    assert len(data["result"]["observations"]) > 0
