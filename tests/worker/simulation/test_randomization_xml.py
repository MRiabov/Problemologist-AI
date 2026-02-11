import xml.etree.ElementTree as ET

from worker.simulation.randomization import (
    MaterialAssignment,
    apply_randomization_to_xml,
)


def test_apply_randomization(tmp_path):
    # Create valid MJCF
    input_xml = str(tmp_path / "input.xml")
    with open(input_xml, "w") as f:
        f.write("""<mujoco>
  <worldbody>
    <body name="part1">
      <geom type="box" size="1 1 1" />
    </body>
    <body name="part2">
      <geom type="sphere" size="1" />
    </body>
    <body name="ignored_part">
      <geom type="capsule" size="1 1" />
    </body>
  </worldbody>
</mujoco>""")

    # Defines assignments
    assignments = {
        "part1": MaterialAssignment(
            part_name="part1",
            material_id="mat1",
            color="#FF0000",
            density_g_cm3=2.0,
            friction_coef=0.8,
            restitution=0.5,
        ),
        "part2": MaterialAssignment(
            part_name="part2",
            material_id="mat2",
            color="#00FF00",
            density_g_cm3=0.5,
            friction_coef=0.1,
            restitution=0.9,
        ),
    }

    # Apply
    output_xml = str(tmp_path / "output.xml")
    apply_randomization_to_xml(input_xml, assignments, output_xml)

    # Verify
    tree = ET.parse(output_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")

    # Part 1
    part1 = worldbody.find("./body[@name='part1']")
    geom1 = part1.find("geom")
    # RGB check: FF0000 -> 1.000 0.000 0.000 1
    assert geom1.get("rgba") == "1.000 0.000 0.000 1"
    # Density check: 2.0 g/cm3 -> 2000 kg/m3
    assert geom1.get("density") == "2000.0"
    # Friction check: 0.8 -> "0.8 0.005 0.0001"
    assert geom1.get("friction") == "0.8 0.005 0.0001"

    # Part 2
    part2 = worldbody.find("./body[@name='part2']")
    geom2 = part2.find("geom")
    # RGB check: 00FF00 -> 0.000 1.000 0.000 1
    assert geom2.get("rgba") == "0.000 1.000 0.000 1"
    # Density check: 0.5 g/cm3 -> 500.0 kg/m3
    assert geom2.get("density") == "500.0"

    # Ignored part (should be unchanged/missing attributes)
    part3 = worldbody.find("./body[@name='ignored_part']")
    geom3 = part3.find("geom")
    assert geom3.get("rgba") is None
    assert geom3.get("density") is None
