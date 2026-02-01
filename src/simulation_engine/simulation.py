import mujoco
import numpy as np
import time
from typing import Dict, Any, Optional, List, Protocol, runtime_checkable, Union
from dataclasses import dataclass, field, asdict


@runtime_checkable
class AgentProtocol(Protocol):
    def control(self, obs: Dict[str, Any]) -> Union[np.ndarray, List[float]]: ...


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
            self.model = mujoco.MjModel.from_xml_path(model_path)
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
        # If target is not a body, maybe a geom? Prompt said "xpos['target']" which usually refers to body xpos.
        # But if it fails, we assume no win condition based on these names.

        # Cache forbidden geom IDs
        self.forbidden_geom_ids = set()
        for i in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if name and "forbid" in name.lower():
                self.forbidden_geom_ids.add(i)

        # Cache site names
        self.site_names = {
            i: mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            for i in range(self.model.nsite)
        }

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
        self.metrics.energy += np.sum(np.abs(power)) * dt

    def run(self, agent_script: str, max_steps: int = 1000) -> Dict[str, Any]:
        """
        Runs the simulation with the provided agent script.

        Args:
            agent_script: A string containing Python code. Must define `control(obs)`.
            max_steps: Maximum simulation steps.

        Returns:
            Dict containing status, metrics, and failure reason if any.
        """
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
                }

        return {
            "status": "TIMEOUT",
            "metrics": self.metrics.to_dict(),
            "steps": self.metrics.steps,
        }

    def _get_observations(self) -> Dict[str, Any]:
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
        if not self.forbidden_geom_ids:
            return "RUNNING"

        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            if (
                contact.geom1 in self.forbidden_geom_ids
                or contact.geom2 in self.forbidden_geom_ids
            ):
                return "FAIL"

        return "RUNNING"
