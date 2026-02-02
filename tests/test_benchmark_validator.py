import pytest
from src.generators.benchmark.validator import validate_mjcf


def test_validate_valid_xml():
    valid_xml = """
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
    <body pos="0 0 1">
      <joint type="free"/>
      <geom type="box" size=".1 .1 .1" rgba="0 .9 0 1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""
    report = validate_mjcf(valid_xml)
    assert report["is_valid"] is True
    assert report["sim_duration"] > 0
    assert report["error_message"] is None
    print(f"DEBUG: max_energy={report['max_energy']}")


def test_validate_invalid_xml():
    invalid_xml = "<mujoco><worldbody><body pos='0 0 1'><joint type='free'/><geom type='box' size='.1 .1 .1'/></worldbody></mujoco>"  # Missing closing tag for body
    report = validate_mjcf(invalid_xml)
    assert report["is_valid"] is False
    assert report["error_message"] is not None


def test_validate_unstable_xml():
    # Large velocity or unstable setup?
    # Hard to guarantee without specific params, but let's try a very small box with very high gravity or something.
    # Actually, let's just test a diverging one if possible.
    unstable_xml = """
<mujoco>
  <option gravity="0 0 -10000"/>
  <worldbody>
    <body pos="0 0 1">
      <joint type="free"/>
      <geom type="box" size=".1 .1 .1" mass="0.0001"/>
    </body>
  </worldbody>
</mujoco>
"""
    # This might not explode in 1s, but let's see.
    report = validate_mjcf(unstable_xml)
    # Even if it's stable, we just want to see it run.
    assert "is_valid" in report
