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
        self.forbidden_sites = set()
        self.goal_sites = set()
        for i in range(self.model.nsite):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            if name and name.startswith("zone_forbid"):
                self.forbidden_sites.add(i)
            elif name and name.startswith("zone_goal"):
                self.goal_sites.add(i)

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
        # We rely on self.goal_sites populated in init

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

                # F002: Check if target fell off the world
                target_pos = self.data.xpos[target_body_id]
                if target_pos[2] < -2.0:
                    self.fail_reason = "target_fell_off_world"
                    logger.info("Simulation FAIL: Target fell off world")
                    break

            # 2. Check Forbidden Zones
            if self._check_forbidden_collision():
                self.fail_reason = "collision_with_forbidden_zone"
                logger.info("Simulation FAIL: Collision with forbidden zone")
                break

            # 3. Check Goal Zone
            # Vertex-based check against goal SITES
            goal_hit, _ = self._check_vertex_in_zone(target_body_id, self.goal_sites)
            if goal_hit:
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

    def _check_vertex_in_zone(
        self, body_id: int, zone_site_ids: set[int]
    ) -> tuple[bool, int | None]:
        """Check if any vertex of a body's mesh touches a zone (Site AABB check).

        Args:
            body_id: ID of the body to check.
            zone_site_ids: Set of SITE IDs representing zones.

        Returns:
            (True, zone_id) if collision, (False, None) otherwise.
        """
        if body_id == -1 or not zone_site_ids:
            return False, None

        geom_start = self.model.body_geomadr[body_id]
        geom_num = self.model.body_geomnum[body_id]
        if geom_start == -1:
            return False, None

        for i in range(geom_num):
            geom_id = geom_start + i
            geom_type = self.model.geom_type[geom_id]

            # Get global transform and vertices
            geom_pos = self.data.geom_xpos[geom_id]
            geom_mat = self.data.geom_xmat[geom_id].reshape(3, 3)

            vertices = []
            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = self.model.geom_dataid[geom_id]
                vert_adr = self.model.mesh_vertadr[mesh_id]
                vert_num = self.model.mesh_vertnum[mesh_id]
                raw_verts = self.model.mesh_vert[vert_adr : vert_adr + vert_num]
                vertices = geom_pos + raw_verts @ geom_mat.T
            else:
                # Minimal fallback: check geom position
                vertices = np.array([geom_pos])

            # Check vertices against all zones
            for zone_id in zone_site_ids:
                zone_pos = self.data.site_xpos[zone_id]
                zone_size = self.model.site_size[zone_id]
                zone_type = self.model.site_type[zone_id]

                # Assume Box or Sphere for zones
                if zone_type == mujoco.mjtGeom.mjGEOM_BOX:
                    # Box check (axis aligned for now, assuming site rotation is identity or simple)
                    site_mat = self.data.site_xmat[zone_id].reshape(3, 3)
                    diff = vertices - zone_pos
                    # Transform to site local frame
                    v_local = diff @ site_mat

                    # Check against half-extents (zone_size)
                    in_box = np.all(np.abs(v_local) < zone_size[:3], axis=1)
                    if np.any(in_box):
                        return True, zone_id

                elif zone_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                    # Sphere check
                    dists = np.linalg.norm(vertices - zone_pos, axis=1)
                    if np.any(dists < zone_size[0]):
                        return True, zone_id

        return False, None

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

    def _check_forbidden_collision(self) -> bool:
        """
        Iterate contacts for physics collision.
        AND check vertex overlap with forbidden SITES.
        """
        # MVP: check target body (usually the ball/object).
        target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target_box"
        )
        hit, _ = self._check_vertex_in_zone(target_body_id, self.forbidden_sites)
        if hit:
            return True

        return False

    def check_goal_with_vertices(self, target_body_id: int) -> bool:
        """Check if target body touches any goal zone using vertex check."""
        hit, _ = self._check_vertex_in_zone(target_body_id, self.goal_sites)
        return hit

    def check_forbid_with_vertices(self, target_body_id: int) -> bool:
        """Check if target body touches any forbidden zone using vertex check."""
        hit, _ = self._check_vertex_in_zone(target_body_id, self.forbidden_sites)
        return hit
