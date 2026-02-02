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


def test_run_isolated_success():
    script = "def control_logic(model, data): pass"
    result = run_isolated(MJCF, agent_script=script, duration=0.1)
    assert result["success"] is True
    assert "result" in result


def test_run_isolated_timeout():
    # Script that hangs
    script = """
import time
def control_logic(model, data):
    while True:
        time.sleep(0.1)
"""
    # Short timeout for test
    result = run_isolated(MJCF, agent_script=script, duration=5.0, timeout=0.1)
    assert result["success"] is False
    assert result["error_type"] == "TimeoutError"
    assert "timed out" in result["message"]


def test_run_isolated_crash():
    # Script that crashes the process
    script = """
import os
def control_logic(model, data):
    os._exit(1)
"""
    result = run_isolated(MJCF, agent_script=script, duration=0.1)
    assert result["success"] is False
    assert result["error_type"] == "CrashError"
    assert "crashed" in result["message"]


def test_run_isolated_error():
    # Invalid XML or duration
    result = run_isolated("invalid xml", duration=0.1)
    assert result["success"] is True # MujocoBridge handles this by returning success=False in result
    assert result["result"]["success"] is False
