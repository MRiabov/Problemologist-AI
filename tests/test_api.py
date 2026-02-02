from fastapi.testclient import TestClient
from src.simulation_engine.main import app

client = TestClient(app)

MJCF = """
<mujoco>
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="5 5 .1"/>
    <body name="target" pos="0 0 0.2">
      <freejoint/>
      <geom type="sphere" size="0.1" rgba="1 0 0 1"/>
    </body>
    <site name="goal" pos="1 0 0.2" size="0.2" rgba="0 1 0 0.3"/>
  </worldbody>
</mujoco>
"""

def test_health():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}

def test_simulate_endpoint():
    payload = {
        "mjcf_xml": MJCF,
        "agent_script": "def control_logic(model, data): pass",
        "duration": 0.1
    }
    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["outcome"] == "success"
    assert "result" in data

def test_simulate_crash():
    payload = {
        "mjcf_xml": MJCF,
        "agent_script": "import os\ndef control_logic(model, data): os._exit(1)",
        "duration": 0.1
    }
    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["success"] is False
    assert data["outcome"] == "crasherror"
