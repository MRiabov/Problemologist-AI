import xml.etree.ElementTree as ET
from build123d import Box, Compound, Location
from src.simulation_engine.builder import SceneCompiler
from src.generators.benchmark.constants import UNIT_SCALE

def test_mjcf_dimension_scaling():
    """
    Verifies that CAD dimensions (mm) are correctly scaled to MuJoCo units (m).
    """
    # 1. Create a 100mm box centered at (50, 50, 50)
    box_part = Box(100, 100, 100).move(Location((50, 50, 50)))
    
    env_compound = Compound(children=[box_part])
    
    # 2. Compile to MJCF
    compiler = SceneCompiler()
    xml_string = compiler.compile(env_compound, env_labels=["obstacle_test_box"])
    
    # 3. Parse XML and verify scaling
    root = ET.fromstring(xml_string)
    
    # Check the mesh asset scale
    mesh_asset = root.find(".//mesh[@name='obstacle_test_box']")
    assert mesh_asset is not None
    assert mesh_asset.attrib["scale"] == f"{UNIT_SCALE} {UNIT_SCALE} {UNIT_SCALE}"
    
    # Find the geom for the test box
    # Note: SceneCompiler adds 'geom_' prefix to labels
    test_geom = root.find(".//geom[@name='geom_obstacle_test_box']")
    
    assert test_geom is not None, "Test geom not found in MJCF"
    
    # We use pos="0 0 0" for meshes because they are already in global coords
    # But it might not be in attribs if not explicitly added by builder
    if "pos" in test_geom.attrib:
        assert test_geom.attrib["pos"] == "0 0 0"


def test_zone_scaling():
    """
    Verifies that zones (sites) are correctly scaled.
    """
    zone_part = Box(10, 10, 10).move(Location((100, 0, 0)))
    
    env_compound = Compound(children=[zone_part])
    
    compiler = SceneCompiler()
    xml_string = compiler.compile(env_compound, env_labels=["zone_goal"])
    
    root = ET.fromstring(xml_string)
    
    # Sites represent the center of the zone
    site = root.find(".//site[@name='site_zone_goal']")
    assert site is not None
    
    pos = [float(x) for x in site.attrib["pos"].split()]
    # 100mm should be 0.1m
    assert pos[0] == 100 * UNIT_SCALE
    assert pos[1] == 0.0
    assert pos[2] == 0.0
    
    # The visual box geom for the zone
    geom = root.find(".//geom[@name='zone_goal']")
    assert geom is not None
    pos_geom = [float(x) for x in geom.attrib["pos"].split()]
    assert pos_geom[0] == 100 * UNIT_SCALE