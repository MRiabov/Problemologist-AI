import os
import pytest
from src.simulation_engine.runner import run_isolated

MJCF = """
<mujoco>
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="5 5 .1"/>
    <body name="injected_object" pos="0 0 0.2">
      <freejoint/>
      <geom type="sphere" size="0.1" rgba="1 0 0 1"/>
    </body>
    <site name="goal" pos="1 0 0.2" size="0.2" rgba="0 1 0 0.3"/>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def mjcf_string():
    return MJCF


def test_run_isolated_success(mjcf_string):
    result = run_isolated(mjcf_string, duration=0.1)
    assert result["success"] is True
    assert "result" in result
    assert result["result"]["success"] is True


def test_run_isolated_timeout(mjcf_string):
    # Test wall-clock timeout
    # We use a very short timeout and a long duration to trigger it
    result = run_isolated(mjcf_string, duration=10.0, timeout=0.01)
    assert result["success"] is False
    assert result["error_type"] == "TimeoutError"
    assert "timed out" in result["message"]


def test_run_isolated_crash(mjcf_string):
    # It's hard to trigger a crash without a script that calls os._exit,
    # and the current runner doesn't support scripts.
    # We'll skip or mock this if needed, but for now let's just test invalid XML
    result = run_isolated("invalid xml", duration=0.1)
    assert (
        result["success"] is True
    )  # MujocoBridge handles load error by returning SimResult(success=False)
    assert result["result"]["success"] is False


def test_run_isolated_error(mjcf_string):
    # Test invalid duration
    # MujocoBridge handles the TypeError internally and returns a SimResult with success=False
    result = run_isolated(mjcf_string, duration="invalid")
    assert result["success"] is True
    assert result["result"]["success"] is False
