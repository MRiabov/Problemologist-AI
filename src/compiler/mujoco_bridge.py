import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from src.compiler.models import Observation, SimResult
from src.environment.sandbox import PodmanSandbox
from src.environment.sandbox_utils import run_sandboxed_script


class MujocoBridge:
    def __init__(
        self,
        workspace_dir: str | Path | None = None,
        sandbox: PodmanSandbox | None = None,
    ):
        """
        Initialize MujocoBridge.

        Args:
            workspace_dir: Path to the workspace directory. If None, uses legacy fallback.
            sandbox: PodmanSandbox instance for running simulations. If None, uses legacy fallback.
        """
        # Assumes this file is in src/compiler/
        # and templates are in src/compiler/templates/
        self.template_dir = Path(__file__).resolve().parent / "templates"
        self._workspace_dir = Path(workspace_dir) if workspace_dir else None
        self._sandbox = sandbox

    @property
    def workspace_dir(self) -> Path:
        """Get workspace directory."""
        if self._workspace_dir:
            return self._workspace_dir

        from src.agent.utils.config import Config

        return Config.WORKSPACE_DIR

    @property
    def sandbox(self) -> PodmanSandbox:
        """Get sandbox."""
        if self._sandbox:
            return self._sandbox
        raise RuntimeError(
            "Sandbox not initialized in MujocoBridge. Ensure it is passed during __init__ or via ToolRuntime."
        )

    def load_template(self, template_name: str = "standard.xml") -> str:
        """Load a standard template and return its XML string content.

        Args:
            template_name: Name of the XML file in templates directory.

        Returns:
            The raw string content of the XML file.
        """
        path = self.template_dir / template_name
        if not path.exists():
            raise FileNotFoundError(f"Template not found: {path}")

        return path.read_text()

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

        mesh_name = Path(mesh_path).name.replace(".", "_")

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
        goal_pos: tuple[float, float, float] | None = None,
        goal_size: float = 0.5,
    ) -> SimResult:
        """Runs the simulation in a sandbox."""
        runner_filename = "sim_runner.py"
        result_file = "sim_result.json"

        # We need to escape the XML string and agent script for the Python runner
        runner_script = f"""
import json
import os
import sys
from src.compiler.mujoco_bridge import MujocoBridge, SimResult

# Add workspace to path
sys.path.append("/workspace")

bridge = MujocoBridge()
xml_string = {xml_string!r}
agent_script = {agent_script!r}
goal_pos = {goal_pos!r}
goal_size = {goal_size!r}
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
    "success": sim_result.success,
    "total_energy": sim_result.total_energy,
    "total_damage": sim_result.total_damage,
    "observations": [
        {{
            "step": obs.step,
            "time": obs.time,
            "state_vector": obs.state_vector.tolist(),
            "energy_consumed": obs.energy_consumed,
            "damage_detected": obs.damage_detected
        }}
        for obs in sim_result.observations
    ],
    "metadata": sim_result.metadata
}}

with open("/workspace/{result_file}", "w") as f:
    json.dump(res_dict, f)
"""
        res = run_sandboxed_script(
            sandbox=self.sandbox,
            script_content=runner_script,
            result_file_name=result_file,
            runner_file_name=runner_filename,
            mount_src=True,
            timeout=int(duration + 10),
        )

        if res.get("status") == "error":
            if res.get("error_type") == "TimeoutError":
                return SimResult(
                    success=False, total_energy=0.0, total_damage=100.0, observations=[]
                )
            if "CRASH_DETECTED" in res.get("message", ""):
                raise RuntimeError(res["message"])

            print(
                f"Sandbox Simulation Error: {res.get('stderr') or res.get('message')}"
            )
            return SimResult(
                success=False, total_energy=0.0, total_damage=100.0, observations=[]
            )

        # Reconstruct observations
        observations = [
            Observation(
                step=o["step"],
                time=o["time"],
                state_vector=np.array(o["state_vector"]),
                energy_consumed=o["energy_consumed"],
                damage_detected=o["damage_detected"],
            )
            for o in res.get("observations", [])
        ]

        return SimResult(
            success=res.get("success", False),
            total_energy=res.get("total_energy", 0.0),
            total_damage=res.get("total_damage", 0.0),
            observations=observations,
            metadata=res.get("metadata", {}),
        )

    def _run_simulation_internal(
        self,
        xml_string: str,
        duration: float = 5.0,
        agent_script: str = "",
        goal_pos: tuple[float, float, float] | None = None,
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
        observations = []

        try:
            # Determine logging interval (e.g., every 0.1s)
            log_interval = 0.1
            last_log_time = -log_interval

            while data.time < duration:
                if control_func:
                    control_func(model, data)

                mujoco.mj_step(model, data)

                # State Logging (Observation)
                if data.time >= last_log_time + log_interval:
                    state_vector = []
                    for i in range(model.nbody):
                        # Use mj_id2name to get body name if needed
                        # names = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
                        state_vector.extend(data.xpos[i].tolist())
                        state_vector.extend(data.xquat[i].tolist())

                    obs = Observation(
                        step=len(observations),
                        time=data.time,
                        state_vector=np.array(state_vector),
                        energy_consumed=energy_acc,
                        damage_detected=damage_acc,
                    )
                    observations.append(obs)
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
            success=success,
            total_energy=energy_acc,
            total_damage=damage_acc,
            observations=observations,
            metadata={"duration": duration, "final_time": data.time},
        )
