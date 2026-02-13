import numpy as np
import structlog
from build123d import Compound, Part
from pydantic import BaseModel, StrictBool, StrictFloat, StrictStr
from shared.simulation.backends import SimulatorBackendType, SimulationScene
from worker.simulation.factory import get_physics_backend

logger = structlog.get_logger(__name__)


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
        backend_type: SimulatorBackendType = SimulatorBackendType.MUJOCO,
    ):
        self.backend = get_physics_backend(backend_type)
        scene = SimulationScene(scene_path=xml_path)
        self.backend.load_scene(scene)

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

        # Cache zone names for forbidden zones
        self.forbidden_sites = []
        self.goal_sites = []
        for name in self.backend.get_all_site_names():
            if name and name.startswith("zone_forbid"):
                self.forbidden_sites.append(name)
            elif name and name.startswith("zone_goal"):
                self.goal_sites.append(name)

        logger.info(
            "SimulationLoop_init",
            goal_sites=self.goal_sites,
            forbidden_sites=self.forbidden_sites,
        )

        # Configurable timeout (capped at hard limit)
        self.max_simulation_time = min(max_simulation_time, MAX_SIMULATION_TIME_SECONDS)

        # Motor overload tracking: time each actuator has been at forcerange limit
        self.actuator_clamp_duration = {}  # name -> duration

        # Reset metrics
        self.reset_metrics()

    def reset_metrics(self):
        self.total_energy = 0.0
        self.max_velocity = 0.0
        self.success = False
        self.fail_reason = None
        self.overloaded_motors: list[str] = []
        for name in self.actuator_clamp_duration:
            self.actuator_clamp_duration[name] = 0.0

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
        self.backend.apply_control(control_inputs)

        # We assume backend has a fixed or default timestep
        # For MuJoCo it's usually 0.002
        # We'll use a small step and loop
        dt = 0.002  # Default step for loop logic
        steps = int(duration / dt)

        start_time = 0.0  # We should probably get current time from backend
        current_time = 0.0

        # Find critical bodies
        target_body_name = "target_box"
        all_bodies = self.backend.get_all_body_names()
        if target_body_name not in all_bodies:
            target_body_name = None
            for name in all_bodies:
                if name == "target_box":
                    target_body_name = name
                    break

        logger.info("SimulationLoop_step_start", target_body_name=target_body_name)

        for _ in range(steps):
            # Apply dynamic controllers
            if dynamic_controllers:
                ctrls = {}
                for name, controller in dynamic_controllers.items():
                    ctrls[name] = controller(current_time)
                self.backend.apply_control(ctrls)

            # Step backend
            res = self.backend.step(dt)
            current_time = res.time
            if not res.success:
                self.fail_reason = res.failure_reason
                break

            # 2. Update Metrics
            # Energy proxy
            actuator_names = self.backend.get_all_actuator_names()
            power = 0.0
            for name in actuator_names:
                state = self.backend.get_actuator_state(name)
                power += abs(state.ctrl * state.velocity)
            self.total_energy += float(power)

            if target_body_name:
                state = self.backend.get_body_state(target_body_name)
                vel = np.linalg.norm(state.vel)
                if vel > self.max_velocity:
                    self.max_velocity = vel

                # F002: Check if target fell off the world
                if state.pos[2] < -2.0:
                    self.fail_reason = "target_fell_off_world"
                    logger.info("simulation_fail", reason="target_fell_off_world")
                    break

            # 3. Check Forbidden Zones
            if self._check_forbidden_collision(target_body_name):
                self.fail_reason = "collision_with_forbidden_zone"
                logger.info("simulation_fail", reason="collision_with_forbidden_zone")
                break

            # 4. Check Goal Zone
            if target_body_name and self.check_goal_with_vertices(target_body_name):
                self.success = True
                logger.info("Simulation_SUCCESS", goal_hit=True)
                break

            # 5. Check Timeout
            if current_time >= self.max_simulation_time:
                self.fail_reason = "timeout_exceeded"
                logger.info(
                    "simulation_fail",
                    reason="timeout_exceeded",
                    elapsed=current_time,
                    limit=self.max_simulation_time,
                )
                break

            # 6. Check Motor Overload
            overloaded = self._check_motor_overload()
            if overloaded:
                self.fail_reason = f"motor_overload:{overloaded}"
                logger.info(
                    "simulation_fail", reason="motor_overload", motor=overloaded
                )
                break

            if render_callback:
                # This might need adjustment as render_callback usually takes model/data
                # For now, we might skip it or pass the backend
                pass

        return SimulationMetrics(
            total_time=current_time,
            total_energy=self.total_energy,
            max_velocity=self.max_velocity,
            success=self.success,
            fail_reason=self.fail_reason,
        )

    def _check_motor_overload(self) -> str | None:
        actuator_names = self.backend.get_all_actuator_names()
        dt = 0.002  # matching our loop dt

        for name in actuator_names:
            state = self.backend.get_actuator_state(name)
            forcerange = state.forcerange

            if forcerange[0] == 0 and forcerange[1] == 0:
                continue

            max_force = max(abs(forcerange[0]), abs(forcerange[1]))
            if max_force == 0:
                continue

            is_clamped = abs(state.force) >= max_force * 0.99

            if is_clamped:
                self.actuator_clamp_duration[name] = (
                    self.actuator_clamp_duration.get(name, 0.0) + dt
                )
                if (
                    self.actuator_clamp_duration[name]
                    >= MOTOR_OVERLOAD_THRESHOLD_SECONDS
                ):
                    self.overloaded_motors.append(name)
                    return name
            else:
                self.actuator_clamp_duration[name] = 0.0

        return None

    def _check_forbidden_collision(self, target_body_name: str | None) -> bool:
        all_bodies = self.backend.get_all_body_names()
        for body_name in all_bodies:
            if body_name == "world" or body_name == "0":
                continue
            if body_name and body_name.startswith("zone_forbid"):
                continue

            for zone_name in self.forbidden_sites:
                if self.backend.check_collision(body_name, zone_name):
                    return True
        return False

    def check_goal_with_vertices(self, target_body_name: str) -> bool:
        for zone_name in self.goal_sites:
            if self.backend.check_collision(target_body_name, zone_name):
                return True
        return False

    def check_forbid_with_vertices(self, target_body_name: str) -> bool:
        for zone_name in self.forbidden_sites:
            if self.backend.check_collision(target_body_name, zone_name):
                return True
        return False
