import pytest

from src.generators.benchmark.manager import execute_build
from unittest.mock import patch

# Example 1 from prompt
EXAMPLE_1 = """
def build(seed: int = 0, scale_factors: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> str:
    # 1. Static Hole (Environment)
    with BuildPart() as hole_part:
        Box(40, 40, 10)
        Cylinder(radius=5.5, height=10, mode=Mode.SUBTRACT)
        hole_env = scale(hole_part.part, by=scale_factors)

    # 2. Goal Zone (Transparent trigger)
    with BuildPart() as goal:
        Cylinder(radius=5.5, height=5)
        goal_env = scale(goal.part, by=scale_factors)

    # 3. Mobile Peg (Agent)
    with BuildPart() as peg:
        Cylinder(radius=5.0, height=30)
        peg_agent = scale(peg.part, by=scale_factors)

    return to_mjcf(
        env_compound=[hole_env, goal_env],
        agent_compound=peg_agent,
        env_labels=["obstacle_hole", "zone_goal"],
        agent_labels=["agent_peg"]
    )
"""

# Example 2 from prompt
EXAMPLE_2 = """
def build(seed: int = 0, scale_factors: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> str:
    # 1. Fixed Base
    with BuildPart() as base:
        Box(100, 20, 5)
        scaled_base = scale(base.part, by=scale_factors)

    # 2. Moving Block
    with BuildPart() as block:
        Box(15, 15, 15)
        scaled_block = scale(block.part, by=scale_factors)

    # 3. Define kinematic relationship
    joints = [{
        "name": "slider_x",
        "type": "slide",
        "pos": "0 0 0.01",
        "axis": "1 0 0"
    }]

    return to_mjcf(
        env_compound=scaled_base,
        agent_compound=scaled_block,
        agent_joints=joints,
        env_labels=["obstacle_base"],
        agent_labels=["agent_block"]
    )
"""


@pytest.mark.benchmark
@patch("src.generators.benchmark.manager.run_sandboxed_script")
def test_example_1_execution(mock_run):
    """Verify Example 1 (Peg-in-Hole) executes correctly in the harness."""
    # Mock the sandbox result to return a valid MJCF string
    mock_run.return_value = {
        "mjcf": """<mujoco>
  <asset>
       <mesh name="agent_peg_0" file="asset_dir/peg.stl" />
  </asset>
  <worldbody>
       <body name="obstacle_hole" />
       <site name="site_zone_goal" />
       <geom type="mesh" mesh="agent_peg_0" />
  </worldbody>
</mujoco>""",
        "error": None,
    }

    xml, _ = execute_build(EXAMPLE_1, seed=42, scale_factors=(1.2, 0.8, 1.0))
    assert "<mujoco" in xml
    assert 'name="obstacle_hole"' in xml
    assert 'name="site_zone_goal"' in xml
    assert 'mesh="agent_peg_0"' in xml


@pytest.mark.benchmark
@patch("src.generators.benchmark.manager.run_sandboxed_script")
def test_example_2_execution(mock_run):
    """Verify Example 2 (Slider) executes correctly in the harness."""
    # Mock the sandbox result
    mock_run.return_value = {
        "mjcf": """<mujoco>
  <worldbody>
       <body name="obstacle_base" />
       <body name="agent_block">
           <geom mesh="agent_block_0" />
           <joint name="slider_x" />
       </body>
  </worldbody>
  <actuator>
       <motor name="motor_slider_x" />
  </actuator>
</mujoco>""",
        "error": None,
    }

    xml, _ = execute_build(EXAMPLE_2, seed=123, scale_factors=(1.0, 1.0, 1.0))
    assert "<mujoco" in xml
    assert 'name="obstacle_base"' in xml
    assert 'mesh="agent_block_0"' in xml
    assert 'joint name="slider_x"' in xml
    assert 'motor name="motor_slider_x"' in xml
