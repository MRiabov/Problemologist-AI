import os
import json
import subprocess
from typing import Optional, List
import mujoco
import xml.etree.ElementTree as ET
from dataclasses import dataclass, asdict
import numpy as np


@dataclass
class SimResult:
    duration: float
    energy: float
    success: bool
    damage: float
    replay_data: Optional[list[dict]] = None


class MujocoBridge:
    def __init__(self):
        # Assumes this file is in src/compiler/
        # and templates are in src/compiler/templates/
        self.template_dir = os.path.join(os.path.dirname(__file__), "templates")

    def load_template(self, template_name: str = "standard.xml") -> str:
        """Load a standard template and return its XML string content.

        Args:
            template_name: Name of the XML file in templates directory.

        Returns:
            The raw string content of the XML file.
        """
        path = os.path.join(self.template_dir, template_name)
        if not os.path.exists(path):
            raise FileNotFoundError(f"Template not found: {path}")

        with open(path, "r") as f:
            return f.read()

    def inject_design(
        self,
        xml_content: str,
        mesh_path: str,
        location: tuple[float, float, float] = (0, 0, 1),
    ) -> str:
        """Inject a mesh into the MJCF XML at the specified location.

        Args:
            xml_content: Base MJCF XML string.
            mesh_path: Absolute path to the mesh file (.stl, .obj, etc).
            location: (x, y, z) tuple for the body position.

        Returns:
            Modified MJCF XML string with the mesh injected as a free body.
        """
        root = ET.fromstring(xml_content)

        # 1. Add asset -> mesh
        asset = root.find("asset")
        if asset is None:
            asset = ET.Element("asset")
            worldbody_node = root.find("worldbody")
            if worldbody_node is not None:
                children = list(root)
                index = children.index(worldbody_node)
                root.insert(index, asset)
            else:
                root.append(asset)

        mesh_name = os.path.basename(mesh_path).replace(".", "_")

        # Avoid duplicate mesh entries
        existing_mesh = asset.find(f"./mesh[@name='{mesh_name}']")
        if existing_mesh is None:
            # We must map 'file' attribute to the absolute path
            ET.SubElement(asset, "mesh", file=mesh_path, name=mesh_name)

        # 2. Add worldbody -> body -> geom
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise ValueError("Invalid MJCF: missing worldbody")

        body_name = "injected_object"
        loc_str = f"{location[0]} {location[1]} {location[2]}"

        body = ET.SubElement(worldbody, "body", name=body_name, pos=loc_str)
        ET.SubElement(body, "freejoint")
        ET.SubElement(body, "geom", type="mesh", mesh=mesh_name)

        return ET.tostring(root, encoding="unicode")

    def run_simulation(
        self,
        xml_string: str,
        duration: float = 5.0,
        agent_script: str = "",
        goal_pos: Optional[tuple[float, float, float]] = None,
        goal_size: float = 0.5,
    ) -> SimResult:
        """Runs the simulation in a sandbox."""
        from src.environment import tools

        runner_filename = "sim_runner.py"
        workspace_dir = tools.WORKSPACE_DIR
        runner_path = os.path.join(workspace_dir, runner_filename)

        # We need to escape the XML string and agent script for the Python runner
        runner_script = f"""
import json
import os
import sys
from src.compiler.mujoco_bridge import MujocoBridge, SimResult

# Add workspace to path
sys.path.append("/workspace")

bridge = MujocoBridge()
xml_string = {repr(xml_string)}
agent_script = {repr(agent_script)}
goal_pos = {repr(goal_pos)}
goal_size = {repr(goal_size)}
duration = {duration}

sim_result = bridge._run_simulation_internal(
    xml_string=xml_string,
    duration=duration,
    agent_script=agent_script,
    goal_pos=goal_pos,
    goal_size=goal_size
)

# Use dataclasses.asdict or manual dict creation
res_dict = {{
    "duration": sim_result.duration,
    "energy": sim_result.energy,
    "success": sim_result.success,
    "damage": sim_result.damage,
    "replay_data": sim_result.replay_data
}}

print(f"SIM_RESULT:{{json.dumps(res_dict)}}")
"""

        try:
            with open(runner_path, "w", encoding="utf-8") as f:
                f.write(runner_script)

            stdout, stderr, returncode = tools._SANDBOX.run_script(
                runner_filename, mount_src=True, timeout=int(duration + 10)
            )

            if os.path.exists(runner_path):
                os.remove(runner_path)

            if "SIM_RESULT:" not in stdout:
                print(f"Sandbox Simulation Error: {stderr}")
                return SimResult(duration, energy=0.0, success=False, damage=100.0)

            result_line = [
                line for line in stdout.split("\n") if "SIM_RESULT:" in line
            ][0]
            res_data = json.loads(result_line.split("SIM_RESULT:")[1])

            return SimResult(**res_data)

        except Exception as e:
            print(f"Error initiating sandboxed simulation: {e}")
            return SimResult(duration, energy=0.0, success=False, damage=100.0)

    def _run_simulation_internal(
        self,
        xml_string: str,
        duration: float = 5.0,
        agent_script: str = "",
        goal_pos: Optional[tuple[float, float, float]] = None,
        goal_size: float = 0.5,
    ) -> SimResult:
        """Actual simulation logic (to be run inside sandbox)."""
        try:
            model = mujoco.MjModel.from_xml_string(xml_string)
        except Exception as e:
            print(f"MuJoCo Load Error: {e}")
            return SimResult(duration, energy=0.0, success=False, damage=100.0)

        data = mujoco.MjData(model)

        # Prepare control logic if provided
        control_func = None
        if agent_script:
            try:
                namespace = {}
                exec(agent_script, namespace)
                if "control_logic" in namespace:
                    control_func = namespace["control_logic"]
            except Exception as e:
                print(f"Control Script Error: {e}")
                return SimResult(duration, energy=0.0, success=False, damage=100.0)

        energy_acc = 0.0
        damage_acc = 0.0
        success = False
        replay_data = []

        try:
            # Determine logging interval (e.g., every 0.1s)
            log_interval = 0.1
            last_log_time = -log_interval

            while data.time < duration:
                if control_func:
                    control_func(model, data)

                mujoco.mj_step(model, data)

                # State Logging for Replay
                if data.time >= last_log_time + log_interval:
                    states = {}
                    for i in range(model.nbody):
                        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
                        states[name or f"body_{i}"] = {
                            "pos": data.xpos[i].tolist(),
                            "quat": data.xquat[i].tolist(),
                        }
                    replay_data.append({"time": data.time, "bodies": states})
                    last_log_time = data.time

                # Energy Metric: Accumulate |power| = |force . velocity|
                power = 0.0
                if model.nu > 0:
                    power += np.dot(data.qfrc_actuator, data.qvel)

                # Also include applied forces (from agent script)
                power += np.dot(data.qfrc_applied, data.qvel)

                energy_acc += abs(power) * model.opt.timestep

                # Goal Detection
                if goal_pos is not None:
                    body_id = mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_BODY, "injected_object"
                    )
                    if body_id != -1:
                        pos = data.xpos[body_id]
                        dist = np.linalg.norm(pos - np.array(goal_pos))
                        if dist < goal_size:
                            success = True
                            # Optional: break early on success?
                            # break

        except Exception as e:
            print(f"Simulation Runtime Error: {e}")
            return SimResult(duration, energy=0.0, success=False, damage=100.0)

        # Final Success check if no goal zone but position is valid
        if goal_pos is None:
            body_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, "injected_object"
            )
            if body_id != -1:
                pos = data.xpos[body_id]
                if not np.any(np.isnan(pos)):
                    success = True

        return SimResult(
            duration=duration,
            energy=energy_acc,
            success=success,
            damage=damage_acc,
            replay_data=replay_data,
        )
