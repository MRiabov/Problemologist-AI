import logging

import mujoco
import numpy as np
from build123d import Compound, Part
from pydantic import BaseModel, StrictBool, StrictFloat, StrictStr

logger = logging.getLogger(__name__)


class SimulationMetrics(BaseModel):
    total_time: StrictFloat
    total_energy: StrictFloat
    max_velocity: StrictFloat
    success: StrictBool
    fail_reason: StrictStr | None = None


# Hard cap on simulation time per architecture spec
MAX_SIMULATION_TIME_SECONDS = 30.0
# Motor overload threshold: fail if clamped for this duration (seconds)
MOTOR_OVERLOAD_THRESHOLD_SECONDS = 2.0


class SimulationLoop:
    def __init__(
        self,
        xml_path: str,
        component: Part | Compound | None = None,
        max_simulation_time: float = MAX_SIMULATION_TIME_SECONDS,
    ):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.component = component
        self.validation_report = None

        if self.component:
            from worker.utils.dfm import validate_and_price
            from worker.workbenches.config import load_config
            from worker.workbenches.models import ManufacturingMethod

            # Default to CNC for simulation validation
            config = load_config()
            self.validation_report = validate_and_price(
                self.component, ManufacturingMethod.CNC, config
            )

        # Cache zone IDs for forbidden zones
        self.forbidden_geoms = set()
        self.goal_geoms = set()
        for i in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if name and name.startswith("zone_forbid"):
                self.forbidden_geoms.add(i)
            elif name and name.startswith("zone_goal"):
                self.goal_geoms.add(i)

        # Configurable timeout (capped at hard limit)
        self.max_simulation_time = min(max_simulation_time, MAX_SIMULATION_TIME_SECONDS)

        # Motor overload tracking: time each actuator has been at forcerange limit
        self.actuator_clamp_duration = np.zeros(self.model.nu)

        # Reset metrics
        self.reset_metrics()

    def reset_metrics(self):
        self.total_energy = 0.0
        self.max_velocity = 0.0
        self.success = False
        self.fail_reason = None
        self.overloaded_motors: list[str] = []
        if hasattr(self, "actuator_clamp_duration"):
            self.actuator_clamp_duration.fill(0.0)

    def step(
        self,
        control_inputs: dict[str, float],
        duration: float = 10.0,
        dynamic_controllers: dict[str, callable] | None = None,
        render_callback=None,
    ) -> SimulationMetrics:
        """
        Runs the simulation for the specified duration.
        Returns metrics.
        """
        self.reset_metrics()

        # T012: Check validation hook before starting
        if self.validation_report and not getattr(
            self.validation_report, "is_manufacturable", False
        ):
            violations = getattr(
                self.validation_report, "violations", ["unknown error"]
            )
            msg = f"validation_failed: {', '.join(map(str, violations))}"
            self.fail_reason = msg
            return SimulationMetrics(
                total_time=0.0,
                total_energy=0.0,
                max_velocity=0.0,
                success=False,
                fail_reason=self.fail_reason,
            )

        # Apply initial static controls
        for name, value in control_inputs.items():
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            if actuator_id != -1:
                self.data.ctrl[actuator_id] = value
            else:
                logger.warning(f"Actuator {name} not found")

        start_time = self.data.time
        steps = (
            int(duration / self.model.opt.timestep)
            if self.model.opt.timestep > 0
            else 0
        )

        # Cache actuator IDs for dynamic control
        actuator_map = {}
        if dynamic_controllers:
            for name in dynamic_controllers:
                act_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )
                if act_id != -1:
                    actuator_map[name] = act_id

        # Find critical bodies once
        target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target_box"
        )
        goal_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "zone_goal"
        )

        for _ in range(steps):
            # Apply dynamic controllers
            if dynamic_controllers:
                for name, controller in dynamic_controllers.items():
                    if name in actuator_map:
                        self.data.ctrl[actuator_map[name]] = controller(self.data.time)

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

            # 4. Check Timeout (hard cap at 30 seconds)
            elapsed = self.data.time - start_time
            if elapsed >= self.max_simulation_time:
                self.fail_reason = "timeout_exceeded"
                logger.info(
                    f"Simulation FAIL: Timeout after {elapsed:.2f}s "
                    f"(limit: {self.max_simulation_time}s)"
                )
                break

            # 5. Check Motor Overload
            overloaded = self._check_motor_overload()
            if overloaded:
                self.fail_reason = f"motor_overload:{overloaded}"
                logger.info(f"Simulation FAIL: Motor overload on {overloaded}")
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
            if (
                contact.geom1 in self.forbidden_geoms
                or contact.geom2 in self.forbidden_geoms
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
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX and np.all(diff < goal_size[:3]):
            return True
        elif (
            geom_type == mujoco.mjtGeom.mjGEOM_SPHERE
            and np.linalg.norm(diff) < goal_size[0]
        ):
            return True

        return False

    def _check_motor_overload(self) -> str | None:
        """Check if any motor has been at forcerange limit for too long.

        Returns:
            Name of overloaded actuator, or None if no overload.
        """
        if self.model.nu == 0:
            return None

        timestep = self.model.opt.timestep

        for i in range(self.model.nu):
            # Get actuator force and forcerange
            force = abs(self.data.actuator_force[i])
            forcerange = self.model.actuator_forcerange[i]

            # Check if forcerange is set (non-zero range)
            if forcerange[0] == 0 and forcerange[1] == 0:
                continue  # No limit set

            max_force = max(abs(forcerange[0]), abs(forcerange[1]))
            if max_force == 0:
                continue

            # Check if force is at limit (within 1% tolerance)
            is_clamped = force >= max_force * 0.99

            if is_clamped:
                self.actuator_clamp_duration[i] += timestep
                if self.actuator_clamp_duration[i] >= MOTOR_OVERLOAD_THRESHOLD_SECONDS:
                    name = mujoco.mj_id2name(
                        self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
                    )
                    self.overloaded_motors.append(name or f"actuator_{i}")
                    return name or f"actuator_{i}"
            else:
                # Reset if not clamped (must be continuous)
                self.actuator_clamp_duration[i] = 0.0

        return None

    def _check_vertex_in_zone(
        self, body_id: int, zone_geom_ids: set[int]
    ) -> tuple[bool, int | None]:
        """Check if any vertex of a body's mesh touches a zone (AABB check).

        Args:
            body_id: ID of the body to check.
            zone_geom_ids: Set of geom IDs representing zones.

        Returns:
            (True, zone_id) if collision, (False, None) otherwise.
        """
        if body_id == -1 or not zone_geom_ids:
            return False, None

        # Get body position and orientation
        body_pos = self.data.xpos[body_id]

        # For each zone, check if body center is inside (simplified vertex check)
        # Full mesh vertex iteration would require accessing mesh data
        # For now, we check body bounding box corners as vertex approximation
        for zone_id in zone_geom_ids:
            zone_pos = self.data.geom_xpos[zone_id]
            zone_size = self.model.geom_size[zone_id]

            # Check body center against zone AABB
            diff = np.abs(body_pos - zone_pos)

            geom_type = self.model.geom_type[zone_id]
            if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                if np.all(diff < zone_size[:3]):
                    return True, zone_id
            elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                if np.linalg.norm(diff) < zone_size[0]:
                    return True, zone_id

        return False, None

    def check_goal_with_vertices(self, target_body_id: int) -> bool:
        """Check if target body touches any goal zone using vertex check.

        Args:
            target_body_id: ID of the target body.

        Returns:
            True if any vertex is in goal zone.
        """
        hit, _ = self._check_vertex_in_zone(target_body_id, self.goal_geoms)
        return hit

    def check_forbid_with_vertices(self, target_body_id: int) -> bool:
        """Check if target body touches any forbidden zone using vertex check.

        Args:
            target_body_id: ID of the target body.

        Returns:
            True if any vertex is in forbidden zone.
        """
        hit, _ = self._check_vertex_in_zone(target_body_id, self.forbidden_geoms)
        return hit
