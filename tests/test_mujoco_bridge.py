import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import pytest

from src.compiler.mujoco_bridge import MujocoBridge, SimResult


@pytest.fixture
def bridge():
    return MujocoBridge()


def test_load_template(bridge):
    content = bridge.load_template("standard.xml")
    assert "<mujoco>" in content
    assert "<worldbody>" in content


def test_load_template_not_found(bridge):
    with pytest.raises(FileNotFoundError):
        bridge.load_template("non_existent.xml")


def test_inject_design(bridge):
    template = bridge.load_template("standard.xml")
    mesh_path = "/tmp/fake_mesh.stl"
    location = (1.0, 2.0, 3.0)

    modified_xml = bridge.inject_design(template, mesh_path, location)

    assert "fake_mesh_stl" in modified_xml
    assert 'pos="1.0 2.0 3.0"' in modified_xml
    assert "<freejoint" in modified_xml

    # Verify XML structure
    root = ET.fromstring(modified_xml)
    asset = root.find("asset")
    assert asset is not None
    mesh = asset.find("./mesh[@name='fake_mesh_stl']")
    assert mesh is not None
    assert mesh.get("file") == mesh_path

    worldbody = root.find("worldbody")
    body = worldbody.find("./body[@name='injected_object']")
    assert body is not None
    assert body.get("pos") == "1.0 2.0 3.0"


def test_run_simulation_internal_basic(bridge):
    # Minimal valid MJCF
    xml_string = """
<mujoco>
  <worldbody>
    <body name="injected_object" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""
    result = bridge._run_simulation_internal(xml_string, duration=0.1)
    assert isinstance(result, SimResult)
    assert result.duration == 0.1
    assert result.success is True  # Default success if no goal_pos and object exists
    assert result.energy >= 0


def test_run_simulation_internal_with_goal(bridge):
    xml_string = """
<mujoco>
  <worldbody>
    <body name="injected_object" pos="0 0 0">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""
    # Success condition: dist < goal_size
    # Object at 0,0,0, Goal at 0,0,0.1, goal_size 0.5 -> Success
    result = bridge._run_simulation_internal(
        xml_string, duration=0.1, goal_pos=(0, 0, 0.1), goal_size=0.5
    )
    assert result.success is True

    # Far away goal -> Failure
    result = bridge._run_simulation_internal(
        xml_string, duration=0.1, goal_pos=(10, 10, 10), goal_size=0.5
    )
    assert result.success is False


def test_run_simulation_internal_with_agent_script(bridge):
    xml_string = """
<mujoco>
  <worldbody>
    <body name="injected_object" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""
    agent_script = """
def control_logic(model, data):
    # Apply some force
    data.qfrc_applied[0] = 10.0
"""
    result = bridge._run_simulation_internal(
        xml_string, duration=0.1, agent_script=agent_script
    )
    assert result.success is True
    assert result.energy > 0  # Should have energy due to applied force


@patch("src.environment.tools._SANDBOX")
@patch("src.environment.tools.WORKSPACE_DIR", "/tmp/test_workspace")
def test_run_simulation_sandboxed(mock_sandbox, bridge):
    # Setup mock
    res_json = (
        '{"duration": 5.0, "energy": 10.0, "success": true, '
        '"damage": 0.0, "replay_data": []}'
    )
    mock_sandbox.run_script.return_value = (
        f"SIM_RESULT:{res_json}",
        "",
        0,
    )

    workspace_path = Path("/tmp/test_workspace")
    if not workspace_path.exists():
        workspace_path.mkdir(parents=True)

    xml_string = "<mujoco></mujoco>"
    result = bridge.run_simulation(xml_string, duration=5.0)

    assert result.success is True
    assert result.energy == 10.0
    assert mock_sandbox.run_script.called

    # Cleanup mock workspace
    # if workspace_path.exists():
    #     import shutil
    #     shutil.rmtree(workspace_path)
