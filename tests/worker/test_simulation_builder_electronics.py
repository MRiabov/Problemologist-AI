import pytest

pytestmark = [pytest.mark.integration, pytest.mark.xdist_group(name="physics_sims")]
import xml.etree.ElementTree as ET

from build123d import Box, Compound, Location

from shared.models.schemas import (
    ElectronicComponent,
    ElectronicsSection,
    PowerSupplyConfig,
    WireConfig,
    WireTerminal,
)
from worker_heavy.simulation.builder import MuJoCoSimulationBuilder


def test_builder_wire_tendons(tmp_path):
    """T014: Verify that MuJoCo builder correctly injects tendons for wires."""
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)

    box1 = Box(10, 10, 10)
    box2 = Box(10, 10, 10)
    box2.location = Location((50, 0, 0))

    from shared.models.schemas import PartMetadata
    box1.metadata = PartMetadata(material_id="aluminum_6061")
    box2.metadata = PartMetadata(material_id="aluminum_6061")
    assembly = Compound(children=[box1, box2])
    assembly.children[0].label = "comp1"
    assembly.children[1].label = "comp2"

    electronics = ElectronicsSection(
        power_supply=psu_config,
        components=[
            ElectronicComponent(component_id="comp1", type="motor"),
            ElectronicComponent(component_id="comp2", type="motor"),
        ],
        wiring=[
            WireConfig(
                wire_id="w1",
                from_terminal=WireTerminal(component="comp1", terminal="+"),
                to_terminal=WireTerminal(component="comp2", terminal="+"),
                gauge_awg=18,
                length_mm=100.0,
                waypoints=[(5, 0, 0), (25, 10, 0), (45, 0, 0)],
                routed_in_3d=True,
            )
        ],
    )

    builder = MuJoCoSimulationBuilder(tmp_path)
    scene_path = builder.build_from_assembly(assembly, electronics=electronics)

    # Parse generated XML
    tree = ET.parse(scene_path)
    root = tree.getroot()

    # Check for sites
    sites = root.findall(".//site")
    site_names = [s.get("name") for s in sites]
    assert "site_w1_0" in site_names
    assert "site_w1_1" in site_names
    assert "site_w1_2" in site_names

    # Check site attachment (T014 refinement)
    # site_w1_0 should be in body 'comp1'
    comp1_body = root.find(".//body[@name='comp1']")
    assert comp1_body is not None
    comp1_sites = [s.get("name") for s in comp1_body.findall("site")]
    assert "site_w1_0" in comp1_sites

    # site_w1_2 should be in body 'comp2'
    comp2_body = root.find(".//body[@name='comp2']")
    assert comp2_body is not None
    comp2_sites = [s.get("name") for s in comp2_body.findall("site")]
    assert "site_w1_2" in comp2_sites

    # Check for tendon
    tendon = root.find(".//tendon/spatial[@name='w1']")
    assert tendon is not None
    assert tendon.get("width") is not None
    assert float(tendon.get("stiffness")) > 0
    assert float(tendon.get("damping")) > 0

    # Check tendon site references
    tendon_sites = [s.get("site") for s in tendon.findall("site")]
    assert tendon_sites == ["site_w1_0", "site_w1_1", "site_w1_2"]


if __name__ == "__main__":
    pytest.main([__file__])
