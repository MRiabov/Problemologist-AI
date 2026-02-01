import os
import pytest
from src.simulation_engine.runner import run_isolated

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

@pytest.fixture
def model_path(tmp_path):
    p = tmp_path / "test_model.xml"
    p.write_text(MJCF)
    return str(p)

def test_run_isolated_success(model_path):
    script = "def control(obs): return []"
    result = run_isolated(model_path, script, max_steps=10)
    assert result["status"] == "TIMEOUT"  # It didn't win or fail, just finished steps
    assert "metrics" in result

def test_run_isolated_timeout(model_path):
    # Script that hangs
    script = """
import time
def control(obs):
    while True:
        time.sleep(0.1)
    return []
"""
    # Short timeout for test
    result = run_isolated(model_path, script, max_steps=100, timeout=2.0)
    assert result["status"] == "TIMEOUT"
    assert "timed out" in result["message"]

def test_run_isolated_crash(model_path):
    # Script that crashes the process (e.g., segfault if we could, but let's just do exit)
    script = """
import os
def control(obs):
    os._exit(1)
"""
    result = run_isolated(model_path, script, max_steps=100)
    assert result["status"] == "CRASH"
    assert "crashed" in result["message"]

def test_run_isolated_error(model_path):
    # Script with syntax error or similar
    script = "invalid script"
    result = run_isolated(model_path, script, max_steps=10)
    assert result["status"] == "ERROR"
    assert "Script execution failed" in result["message"]
