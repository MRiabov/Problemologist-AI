import os
import mujoco
import pytest
import numpy as np
from src.simulation_engine.simulation import SimulationLoop

MJCF = """
<mujoco>
  <statistic extent="2" center="0 0 1"/>
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="5 5 .1"/>
    
    <body name="target" pos="0 0 0.2">
      <freejoint/>
      <geom type="sphere" size="0.1" rgba="1 0 0 1"/>
    </body>
    <site name="goal" pos="1 0 0.2" size="0.2" rgba="0 1 0 0.3"/>
    
    <body name="agent" pos="0 0 0.5">
       <joint name="slide_x" type="slide" axis="1 0 0"/>
       <joint name="slide_y" type="slide" axis="0 1 0"/>
       <geom name="agent_geom" type="box" size="0.1 0.1 0.1"/>
    </body>
    
    <geom name="forbid_zone" type="box" size="0.5 0.5 0.1" pos="2 2 0.1" rgba="0 0 1 0.5"/>
  </worldbody>
  <actuator>
     <motor joint="slide_x" name="act1"/>
     <motor joint="slide_y" name="act2"/>
  </actuator>
</mujoco>
"""


@pytest.fixture
def model_path(tmp_path):
    p = tmp_path / "test_model.xml"
    p.write_text(MJCF)
    return str(p)


def test_simulation_init(model_path):
    sim = SimulationLoop(model_path)
    assert sim.model is not None
    assert sim.data is not None
    assert sim.goal_site_id != -1
    assert sim.target_body_id != -1


def test_simulation_step(model_path):
    sim = SimulationLoop(model_path)
    sim.step()
    assert sim.metrics.steps == 1
    assert sim.metrics.time > 0


def test_agent_control_run(model_path):
    sim = SimulationLoop(model_path)
    script = """
def control(obs):
    # Return 2 controls
    return [1.0, 0.5] 
"""
    result = sim.run(script, max_steps=10)
    # Should run 10 steps
    assert result["status"] == "TIMEOUT"
    assert result["steps"] == 10
    # Check if controls were applied (last step)
    assert sim.data.ctrl[0] == 1.0
    assert sim.data.ctrl[1] == 0.5


def test_win_condition(model_path):
    sim = SimulationLoop(model_path)
    # Target starts away from goal
    assert sim.check_termination() == "RUNNING"

    # Move target to goal (1 0 0.2)
    # Find joint address for target
    target_body_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "target")
    jnt_adr = sim.model.body_jntadr[target_body_id]
    qpos_adr = sim.model.jnt_qposadr[jnt_adr]

    sim.data.qpos[qpos_adr] = 1.0
    sim.data.qpos[qpos_adr + 1] = 0.0
    sim.data.qpos[qpos_adr + 2] = 0.2

    # Run forward to update site_xpos etc
    mujoco.mj_forward(sim.model, sim.data)

    status = sim.check_termination()
    assert status == "WIN"


def test_fail_condition(model_path):
    sim = SimulationLoop(model_path)
    # Move agent to overlap with forbid zone at (2, 2)
    # qpos 0 is x, 1 is y.
    sim.data.qpos[0] = 2.0
    sim.data.qpos[1] = 2.0
    mujoco.mj_forward(sim.model, sim.data)  # Update contacts

    status = sim.check_termination()
    assert status == "FAIL"


def test_metrics(model_path):
    sim = SimulationLoop(model_path)
    sim.run(
        """
def control(obs):
    return [0, 0]
""",
        max_steps=5,
    )

    assert sim.metrics.steps == 5
    assert sim.metrics.time == 5 * sim.model.opt.timestep


def test_energy_calculation(model_path):
    sim = SimulationLoop(model_path)
    # Apply some control to generate energy
    sim.run(
        """
def control(obs):
    return [10.0, 10.0]
""",
        max_steps=10,
    )

    assert sim.metrics.energy > 0
