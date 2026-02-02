import tempfile
from pathlib import Path

import mujoco
from build123d import Box, Compound, Pos

from simulation_engine.builder import SceneCompiler


def test_compiler_basic_scene():
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        # Create environment
        # Obstacle
        b1 = Box(1, 1, 1)
        b1.label = "obstacle_1"

        # Zone
        b2 = Box(0.5, 0.5, 0.5)
        b2.label = "zone_goal"
        b2 = Pos(2, 2, 0) * b2

        env = Compound([b1, b2])

        # Workaround for label preservation in build123d Compound
        s1, s2 = env.solids()
        s1.label = "obstacle_1"
        s2.label = "zone_goal"
        env.solids = lambda: [s1, s2]

        # Create agent
        agent_part = Box(0.2, 0.2, 0.2)
        agent = Compound([agent_part])
        a1 = agent.solids()[0]
        a1.label = "agent_base_0"
        agent.solids = lambda: [a1]

        compiler = SceneCompiler(asset_dir=tmpdir)
        xml_str = compiler.compile(env, agent)

        assert "mujoco" in xml_str
        assert "obstacle_1" in xml_str
        assert "zone_goal" in xml_str
        assert "agent" in xml_str

        # Verify it loads in MuJoCo
        xml_path = tmp_path / "scene.xml"
        xml_path.write_text(xml_str)

        # mujoco.MjModel.from_xml_path(xml_path) might fail if meshes aren't found
        # but we saved them to tmpdir.
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        assert model is not None

        # Geoms:
        # 1. floor (plane)
        # 2. obstacle_1 (mesh)
        # 3. zone_goal (box, visual only)
        # 4. agent_base_0 (mesh)
        assert model.ngeom == 4


def test_compiler_with_joints():
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        env = Compound([Box(0.1, 0.1, 0.1)])  # Default unnamed obstacle

        agent_base = Box(0.2, 0.2, 0.2)
        agent = Compound([agent_base])

        joints = [
            {"name": "arm_joint", "pos": "0 0 0.2", "axis": "0 1 0", "type": "hinge"}
        ]

        compiler = SceneCompiler(asset_dir=tmpdir)
        xml_str = compiler.compile(env, agent, agent_joints=joints)

        assert 'joint name="arm_joint"' in xml_str
        assert "motor" in xml_str
        assert 'joint="arm_joint"' in xml_str
        assert 'gear="10"' in xml_str

        xml_path = tmp_path / "scene_with_joints.xml"
        xml_path.write_text(xml_str)

        model = mujoco.MjModel.from_xml_path(str(xml_path))
        assert model is not None
        assert model.njnt == 1
        assert model.nu == 1  # 1 actuator
