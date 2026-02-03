import xml.etree.ElementTree as ET

import pytest

from src.compiler.mujoco_bridge import MujocoBridge


from unittest.mock import MagicMock, patch


@pytest.fixture
def bridge(tmp_path):
    sandbox = MagicMock()
    return MujocoBridge(workspace_dir=tmp_path, sandbox=sandbox)


@pytest.fixture
def dummy_mesh(tmp_path):
    # minimal valid OBJ
    mesh_path = tmp_path / "dummy.obj"
    content = """v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
"""
    mesh_path.write_text(content)
    return str(mesh_path)


def test_load_template(bridge):
    xml_content = bridge.load_template("standard.xml")
    assert "<mujoco>" in xml_content
    assert "worldbody" in xml_content


def test_inject_design(bridge, dummy_mesh):
    base_xml = bridge.load_template()
    injected = bridge.inject_design(base_xml, dummy_mesh, location=(0, 0, 2))

    assert f'file="{dummy_mesh}"' in injected
    assert 'name="injected_object"' in injected
    assert 'pos="0 0 2"' in injected

    # Parse to verify validity
    root = ET.fromstring(injected)
    assert root.find(".//body[@name='injected_object']") is not None


@pytest.mark.asyncio
@patch("src.environment.sandbox_utils.run_sandboxed_script")
async def test_run_simulation_basic(mock_run_script, bridge, dummy_mesh):
    # Configure mock to return success
    mock_run_script.return_value = {
        "status": "success",
        "observations": [],
        "metrics": {"steps": 10},
        "success": True,
        "total_energy": 50.0,
        "total_damage": 0.0,
        "metadata": {"duration": 0.01},
    }

    base_xml = bridge.load_template()
    injected = bridge.inject_design(base_xml, dummy_mesh, location=(0, 0, 2))

    # Run simulation
    result = await bridge.run_simulation(injected, duration=0.01)

    assert result.success is True, f"Simulation failed: {result.metadata}"
    assert result.metadata["duration"] == 0.01
    assert isinstance(result.total_energy, float)


@pytest.mark.asyncio
async def test_run_simulation_fail_handling(bridge):
    # Pass garbage XML
    result = await bridge.run_simulation("<garbage/>", duration=0.01)
    assert result.success is False
    assert result.total_damage == 0.0
