from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

import mujoco
import numpy as np

from src.environment.sandbox_utils import run_sandboxed_script


@runtime_checkable
class AgentProtocol(Protocol):
    def control(self, obs: dict[str, Any]) -> np.ndarray | list[float]: ...


@dataclass
class SimulationMetrics:
    energy: float = 0.0
    time: float = 0.0
    collisions: int = 0
    steps: int = 0

    def to_dict(self):
        return asdict(self)


class SimulationLoop:
    def __init__(self, model_path: str):
        """
        Initialize the simulation loop with a path to MJCF model.
        """
        try:
            self.model_path = str(Path(model_path).resolve())
            self.model = mujoco.MjModel.from_xml_path(self.model_path)
        except Exception as e:
            raise ValueError(f"Failed to load model from {model_path}: {e}") from e

        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.metrics = SimulationMetrics()

        # Cache IDs for termination checks
        self.goal_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "goal"
        )
        self.target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target"
        )

        # Cache site names to optimize _get_observations
        self.site_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            for i in range(self.model.nsite)
        ]

        # Cache forbidden geometries for optimized termination checks
        self.forbidden_geom_ids = set()
        for i in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if name and "forbid" in name.lower():
                self.forbidden_geom_ids.add(i)

    def step(self):
        """
        Performs one physics tick and updates metrics.
        """
        # Apply standard stepping
        mujoco.mj_step(self.model, self.data)

        # Update Metrics
        dt = self.model.opt.timestep
        self.metrics.steps += 1
        self.metrics.time = self.metrics.steps * dt

        # Collisions: simply count contacts
        self.metrics.collisions += self.data.ncon

        # Energy calculation
        # Power = actuator_force * actuator_velocity
        power = self.data.actuator_force * self.data.actuator_velocity
        self.metrics.energy += float(np.sum(np.abs(power)) * dt)

    def run(
        self,
        agent_script: str,
        max_steps: int = 1000,
        sandbox: Any = None,
        workspace_dir: str | Path | None = None,
    ) -> dict[str, Any]:
        """
        Runs the simulation with the provided agent script inside a sandbox.
        """
        # Legacy fallback if sandbox is not provided
        if sandbox is None:
            raise ValueError("Sandbox must be provided to run()")

        if workspace_dir is None:
            if hasattr(sandbox, "workspace_dir"):
                workspace_dir = sandbox.workspace_dir
            else:
                from src.agent.utils.config import Config

                workspace_dir = Config.WORKSPACE_DIR

        container_model_path = "/workspace/model.xml"
        runner_filename = "sim_engine_runner.py"
        result_file = "sim_result.json"

        runner_script = f"""
import json
import os
import sys
import numpy as np
from src.simulation_engine.simulation import SimulationLoop

# Initialize SimulationLoop inside the sandbox
sim = SimulationLoop("{container_model_path}")
agent_script = {agent_script!r}
max_steps = {max_steps}

# Call the internal run logic
result = sim._run_internal(agent_script, max_steps)

# Write result to file
with open("/workspace/{result_file}", "w") as f:
    json.dump(result, f)
"""

        extra_mounts = [(self.model_path, container_model_path)]

        res = run_sandboxed_script(
            sandbox=sandbox,
            script_content=runner_script,
            result_file_name=result_file,
            runner_file_name=runner_filename,
            extra_mounts=extra_mounts,
            mount_src=True,
        )

        if res.get("status") == "error":
            return {
                "status": "ERROR",
                "message": res.get("message", "Unknown error"),
                "metrics": self.metrics.to_dict(),
            }

        # Update local metrics for tests that inspect them
        if "metrics" in res:
            self.metrics.steps = res["metrics"]["steps"]
            self.metrics.time = res["metrics"]["time"]
            self.metrics.energy = res["metrics"]["energy"]
            self.metrics.collisions = res["metrics"]["collisions"]

        return res

    def _run_internal(self, agent_script: str, max_steps: int = 1000) -> dict[str, Any]:
        """Actual simulation logic to be run inside sandbox."""
        # Create safe scope
        scope = {
            "np": np,
            "numpy": np,
            "math": __import__("math"),
        }

        # Exec script
        try:
            exec(agent_script, scope)
        except Exception as e:
            return {
                "status": "ERROR",
                "message": f"Script execution failed: {e}",
                "metrics": self.metrics.to_dict(),
            }

        if "control" not in scope:
            return {
                "status": "ERROR",
                "message": "No 'control' function found in script",
                "metrics": self.metrics.to_dict(),
            }

        control_func = scope["control"]

        # Reset simulation
        mujoco.mj_resetData(self.model, self.data)
        self.metrics = SimulationMetrics()

        for _ in range(max_steps):
            # Get observations
            obs = self._get_observations()

            # Get Action
            try:
                action = control_func(obs)
            except Exception as e:
                return {
                    "status": "ERROR",
                    "message": f"Agent control failed: {e}",
                    "metrics": self.metrics.to_dict(),
                }

            # Check action shape
            if hasattr(action, "__len__") and len(action) != self.model.nu:
                # Warn or fail?
                # For robustness, slice or pad? Or just let assignment fail?
                # Let's try to assign and catch error
                pass

            try:
                self.data.ctrl[:] = action
            except Exception as e:
                return {
                    "status": "ERROR",
                    "message": f"Invalid action format: {e}",
                    "metrics": self.metrics.to_dict(),
                }

            # Step
            self.step()

            # Check termination
            status = self.check_termination()
            if status != "RUNNING":
                return {
                    "status": status,
                    "metrics": self.metrics.to_dict(),
                    "steps": self.metrics.steps,
                    "ctrl": self.data.ctrl.tolist(),
                }

        return {
            "status": "TIMEOUT",
            "metrics": self.metrics.to_dict(),
            "steps": self.metrics.steps,
            "ctrl": self.data.ctrl.tolist(),
        }

    def _get_observations(self) -> dict[str, Any]:
        """
        Collects sensors and state for the agent.
        """
        return {
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
            "time": self.data.time,
            "sensordata": self.data.sensordata.copy()
            if self.model.nsensor > 0
            else np.array([]),
            "site_xpos": {
                name: self.data.site_xpos[i].copy()
                for i, name in enumerate(self.site_names)
            },
        }

    def check_termination(self) -> str:
        """
        Checks for WIN or FAIL conditions.
        """
        # WIN CONDITION
        if self.goal_site_id != -1 and self.target_body_id != -1:
            goal_pos = self.data.site_xpos[self.goal_site_id]
            target_pos = self.data.xpos[self.target_body_id]
            dist = np.linalg.norm(goal_pos - target_pos)
            if dist < 0.05:  # Threshold 5cm
                return "WIN"

        # FAIL CONDITION: Collision with "Forbid"
        # We need to iterate contacts
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            # Fast integer check against pre-calculated set
            if (
                contact.geom1 in self.forbidden_geom_ids
                or contact.geom2 in self.forbidden_geom_ids
            ):
                return "FAIL"

        return "RUNNING"
