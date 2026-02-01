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
        "model_xml": MJCF,
        "agent_script": "def control(obs): return []",
        "max_steps": 10
    }
    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "TIMEOUT"  # Or SUCCESS if it won, but here it just times out on steps
    assert "metrics" in data

def test_simulate_crash():
    payload = {
        "model_xml": MJCF,
        "agent_script": "import os\ndef control(obs): os._exit(1)",
        "max_steps": 10
    }
    response = client.post("/simulate", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "CRASH"
