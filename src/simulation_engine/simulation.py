import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

import mujoco
import numpy as np


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
            raise ValueError(f"Failed to load model from {model_path}: {e}")

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
        # Pre-cache names to avoid expensive lookups in critical loops
        self.site_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            for i in range(self.model.nsite)
        ]
        self.geom_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            for i in range(self.model.ngeom)
        ]
        # If target is not a body, maybe a geom? Prompt said "xpos['target']" which usually refers to body xpos.
        # But if it fails, we assume no win condition based on these names.

    def step(self):
        """
        Performs one physics tick and updates metrics.
        """
        # Calculate energy for this step before stepping?
        # Energy = Power * dt = (Force * Velocity) * dt
        # We can approximate Force as ctrl (if direct) or actuator force.
        # For now, simplistic approximation using ctrl and qvel (if dimensions match approx)
        # But accurately, we should use mjData.actuator_force and mjData.actuator_velocity if available
        # or just skip complex energy calc as 'approximation' is requested.
        # T011 says: sum(torque * velocity * dt)

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
        # Energy = sum(|power|) * dt
        power = self.data.actuator_force * self.data.actuator_velocity
        self.metrics.energy += float(np.sum(np.abs(power)) * dt)

    def run(self, agent_script: str, max_steps: int = 1000) -> dict[str, Any]:
        """
        Runs the simulation with the provided agent script inside a sandbox.
        """
        from src.environment import tools

        workspace = Path(tools.WORKSPACE_DIR)
        sandbox = tools._SANDBOX

        runner_filename = "sim_engine_runner.py"
        runner_path = workspace / runner_filename

        # In-container path for the model
        # We'll mount the host's model_path to /workspace/model.xml
        container_model_path = "/workspace/model.xml"

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
# We need to expose the internal run logic since we are technically calling it
# But we can just use _run_internal if we define one.
# For now, we'll just implement it here to avoid deep refactoring.
result = sim._run_internal(agent_script, max_steps)

print(f"SIM_ENGINE_RESULT:{{json.dumps(result)}}")
"""
        try:
            runner_path.write_text(runner_script, encoding="utf-8")

            stdout, stderr, rc = sandbox.run_script(
                runner_filename,
                mount_src=True,
                extra_mounts=[(self.model_path, container_model_path)],
                timeout=60,
            )

            if runner_path.exists():
                runner_path.unlink()

            if rc != 0:
                return {
                    "status": "TIMEOUT" if rc == 124 else "ERROR",
                    "message": f"Sandbox execution failed (code {rc}): {stderr}",
                    "error_type": "CrashError" if rc != 124 else "TimeoutError",
                    "metrics": self.metrics.to_dict(),
                }

            if "SIM_ENGINE_RESULT:" not in stdout:
                return {
                    "status": "ERROR",
                    "message": f"Sandbox simulation failed: {stderr}",
                    "metrics": self.metrics.to_dict(),
                }

            result_line = [
                line for line in stdout.split("\n") if "SIM_ENGINE_RESULT:" in line
            ][0]
            res_data = json.loads(result_line.split("SIM_ENGINE_RESULT:")[1])

            # Update local metrics for tests that inspect them
            if "metrics" in res_data:
                self.metrics.steps = res_data["metrics"]["steps"]
                self.metrics.time = res_data["metrics"]["time"]
                self.metrics.energy = res_data["metrics"]["energy"]
                self.metrics.collisions = res_data["metrics"]["collisions"]

            return res_data

        except Exception as e:
            return {
                "status": "ERROR",
                "message": f"Error initiating sandboxed simulation: {e}",
                "metrics": self.metrics.to_dict(),
            }

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
            "sensordata": (
                self.data.sensordata.copy()
                if self.model.nsensor > 0
                else np.array([])
            ),
            "site_xpos": {
                self.site_names[i]: self.data.site_xpos[i].copy()
                for i in range(self.model.nsite)
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
            # Use cached geometry names instead of looking them up
            name1 = self.geom_names[contact.geom1]
            name2 = self.geom_names[contact.geom2]

            # Check if one is forbid and other is part of agent
            # We assume "forbid" is in the name of forbidden zone
            # And we need to know what is the agent.
            # Assuming if one is forbid and other is NOT forbid and NOT ground, it's a fail?
            # Or if "forbid" is touched by anything?

            if name1 and "forbid" in name1.lower():
                return "FAIL"
            if name2 and "forbid" in name2.lower():
                return "FAIL"

        return "RUNNING"
