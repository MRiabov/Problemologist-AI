import xml.etree.ElementTree as ET

import pytest

from src.compiler.mujoco_bridge import MujocoBridge


@pytest.fixture
def bridge():
    return MujocoBridge()


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


def test_run_simulation_basic(bridge, dummy_mesh):
    base_xml = bridge.load_template()
    injected = bridge.inject_design(base_xml, dummy_mesh, location=(0, 0, 2))

    # Run simulation
    result = bridge.run_simulation(injected, duration=0.01)

    assert result.duration == 0.01
    assert isinstance(result.energy, float)
    assert isinstance(result.success, bool)


def test_run_simulation_fail_handling(bridge):
    # Pass garbage XML
    result = bridge.run_simulation("<garbage/>", duration=0.01)
    assert result.success is False
    assert result.damage == 100.0
