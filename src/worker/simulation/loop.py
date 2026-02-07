import mujoco
from pydantic import BaseModel, StrictFloat, StrictBool, StrictStr
from typing import Dict, Optional, List, Union
import numpy as np
import logging
from build123d import Part, Compound

logger = logging.getLogger(__name__)


class SimulationMetrics(BaseModel):
    total_time: StrictFloat
    total_energy: StrictFloat
    max_velocity: StrictFloat
    success: StrictBool
    fail_reason: Optional[StrictStr] = None


class SimulationLoop:
    def __init__(
        self,
        xml_path: str,
        component: Optional[Union[Part, Compound]] = None,
    ):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.component = component
        self.validation_report = None

        if self.component:
            from src.worker.utils.dfm import validate_and_price
            from src.workbenches.models import ManufacturingMethod
            from src.workbenches.config import load_config

            # Default to CNC for simulation validation
            config = load_config()
            self.validation_report = validate_and_price(self.component, ManufacturingMethod.CNC, config)

        # Cache zone IDs for forbidden zones
        self.forbidden_geoms = []
        for i in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if name and name.startswith("zone_forbid"):
                self.forbidden_geoms.append(name)

        # Reset metrics
        self.reset_metrics()

    def reset_metrics(self):
        self.total_energy = 0.0
        self.max_velocity = 0.0
        self.success = False
        self.fail_reason = None

    def step(
        self,
        control_inputs: Dict[str, float],
        duration: float = 10.0,
        render_callback=None,
    ) -> SimulationMetrics:
        """
        Runs the simulation for the specified duration.
        Returns metrics.
        """
        self.reset_metrics()

        # T012: Check validation hook before starting
        if self.validation_report and not getattr(self.validation_report, "is_manufacturable", False):
            violations = getattr(self.validation_report, "violations", ["unknown error"])
            self.fail_reason = f"validation_failed: {', '.join(map(str, violations))}"
            return SimulationMetrics(
                total_time=0.0,
                total_energy=0.0,
                max_velocity=0.0,
                success=False,
                fail_reason=self.fail_reason,
            )

        # Apply static controls (if any)
        for name, value in control_inputs.items():
            # Find actuator by name
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            if actuator_id != -1:
                self.data.ctrl[actuator_id] = value
            else:
                logger.warning(f"Actuator {name} not found")

        start_time = self.data.time
        # Use a safe epsilon for max steps to avoid floating point issues
        steps = (
            int(duration / self.model.opt.timestep)
            if self.model.opt.timestep > 0
            else 0
        )

        # Find critical bodies once
        target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target_box"
        )
        goal_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "zone_goal"
        )

        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)

            # 1. Update Metrics
            # Energy proxy: abs(ctrl) * abs(velocity) is power. Energy is integral of power * dt.
            # But here we just sum power? T011 says "sum of ctrl * velocity".
            # We add it per step.
            power = 0.0
            if self.model.nu > 0:
                # Check shapes properly: ctrl (nu,), qvel (nv,)
                # Generally nu <= nv. For simple joints, they map 1:1.
                # We can use actuator velocity? mujoco.mj_objectVelocity?
                # data.actuator_velocity exists? No.
                # data.qvel is joint vel.
                # Using simple norm for now as strict physics valid energy computation is complex.
                power = np.sum(np.abs(self.data.ctrl * self.data.qvel[: self.model.nu]))

            self.total_energy += float(power)

            if target_body_id != -1:
                # cvel is rotational(3)+translational(3)
                vel = np.linalg.norm(self.data.cvel[target_body_id][3:])
                if vel > self.max_velocity:
                    self.max_velocity = vel

            # 2. Check Forbidden Zones
            if self._check_forbidden_collision():
                self.fail_reason = "collision_with_forbidden_zone"
                logger.info("Simulation FAIL: Collision with forbidden zone")
                break

            # 3. Check Goal Zone
            if self._check_goal_reached(target_body_id, goal_geom_id):
                self.success = True
                logger.info("Simulation SUCCESS: Goal reached")
                break

            if render_callback:
                render_callback(self.model, self.data)

        return SimulationMetrics(
            total_time=self.data.time - start_time,
            total_energy=self.total_energy,
            max_velocity=self.max_velocity,
            success=self.success,
            fail_reason=self.fail_reason,
        )

    def _check_forbidden_collision(self) -> bool:
        """
        Iterate contacts. If any contact involves a forbidden geom, return True.
        """
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
            )
            geom2_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
            )

            if (geom1_name and geom1_name.startswith("zone_forbid")) or (
                geom2_name and geom2_name.startswith("zone_forbid")
            ):
                return True
        return False

    def _check_goal_reached(self, target_body_id: int, goal_geom_id: int) -> bool:
        """
        Check if target is in goal zone.
        """
        if goal_geom_id == -1 or target_body_id == -1:
            return False

        goal_global_pos = self.data.geom_xpos[goal_geom_id]
        target_pos = self.data.xpos[target_body_id]
        goal_size = self.model.geom_size[goal_geom_id]

        diff = np.abs(target_pos - goal_global_pos)

        geom_type = self.model.geom_type[goal_geom_id]
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            if np.all(diff < goal_size[:3]):
                return True
        elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            if np.linalg.norm(diff) < goal_size[0]:
                return True

        return False
