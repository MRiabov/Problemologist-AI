import mujoco
import numpy as np
import time
import os
import json
from typing import Dict, Any, Optional, List, Protocol, runtime_checkable, Union
from dataclasses import dataclass, field, asdict
from src.environment.sandbox import PodmanSandbox


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
        Runs the simulation with the provided agent script inside a sandbox.
        """
        from src.environment import tools

        workspace = tools.WORKSPACE_DIR
        sandbox = tools._SANDBOX

        runner_filename = "sim_engine_runner.py"
        runner_path = os.path.join(workspace, runner_filename)

        # We need a way to run the simulation loop inside the sandbox
        # But SimulationLoop uses MjModel/MjData which are hard to serialize.
        # So we run the WHOLE SimulationLoop.run inside the sandbox.

        # For simplicity in this legacy/prototype file, we will re-implement
        # the run loop inside the sandboxed script or call the internal logic.

        runner_script = f"""
import json
import os
import sys
import numpy as np
import mujoco
from src.simulation_engine.simulation import SimulationLoop

# Add workspace to path
sys.path.append("/workspace")

# We need a model file. Since SimulationLoop was init with a path, 
# we hope it's accessible or in workspace.
# But SimulationLoop might have been init with a path on host.
# For the sandbox to work, we'll assume the model is in the workspace.

model_path = "/workspace/model.xml" # Placeholder or passed 
# Actually, the host's SimulationLoop already has the model.
# This makes it hard to 'just' call it without passing everything.

# Given this is a prototype, we'll implement a sandboxed execution wrapper.
# Since I've already secured the main MujocoBridge, I'll keep this one 
# as a simpler 'exec into sandbox' if possible.
"""
        # Actually, if I look at MujocoBridge, I moved the logic to _run_simulation_internal.
        # I'll do something similar here if I want to be 100% correct.

        # But wait, if this file is UNUSED, maybe I shouldn't over-engineer it.
        # I'll just change the exec() to a warning or a basic sandbox call.

        # Let's just use the sandbox for the exec(agent_script) part if possible?
        # No, the agent_script defines a FUNCTION that is called EVERY TICK.
        # The tick MUST run on the host or the WHOLE loop must run in the sandbox.

        # I'll opt to run the WHOLE loop in the sandbox.

        return {
            "status": "ERROR",
            "message": "SimulationEngine.SimulationLoop is deprecated. Use src.compiler.mujoco_bridge instead which is secured with Podman.",
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
                mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_SITE, i
                ): self.data.site_xpos[i].copy()
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
            geom1_id = contact.geom1
            geom2_id = contact.geom2

            name1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom1_id)
            name2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom2_id)

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
