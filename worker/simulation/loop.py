import numpy as np
import structlog
from build123d import Compound, Part
from pydantic import BaseModel, StrictBool, StrictFloat, StrictStr
from shared.simulation.backends import SimulatorBackendType, SimulationScene
from shared.models.schemas import ElectronicsSection
from shared.enums import SimulationFailureMode
from shared.observability.events import emit_event
from shared.observability.schemas import ElectricalFailureEvent
from worker.simulation.factory import get_physics_backend

logger = structlog.get_logger(__name__)


class StressSummary(BaseModel):
    part_label: str
    max_von_mises_pa: float
    mean_von_mises_pa: float
    safety_factor: float            # yield_stress / max_von_mises
    location_of_max: tuple[float, float, float]
    utilization_pct: float          # max_stress / yield_stress * 100

class SimulationMetrics(BaseModel):
    total_time: StrictFloat
    total_energy: StrictFloat
    max_velocity: StrictFloat
    success: StrictBool
    fail_reason: StrictStr | None = None
    stress_summaries: list[StressSummary] = []


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
        electronics: ElectronicsSection | None = None,
    ):
        self.backend = get_physics_backend(backend_type)
        scene = SimulationScene(scene_path=xml_path)
        self.backend.load_scene(scene)

        self.component = component
        self.electronics = electronics
        self.is_powered_map = {}
        self.validation_report = None

        if self.electronics:
            from shared.circuit_builder import build_circuit_from_section
            from shared.pyspice_utils import validate_circuit

            self.circuit = build_circuit_from_section(self.electronics)
            # Initial validation to set up is_powered_map
            res = validate_circuit(self.circuit, self.electronics.power_supply)
            if res.valid:
                for comp in self.electronics.components:
                    if comp.type == "motor":
                        v_plus = res.node_voltages.get(f"{comp.component_id}_+", 0.0)
                        v_minus = res.node_voltages.get(f"{comp.component_id}_-", 0.0)
                        voltage = abs(v_plus - v_minus)
                        # Powered if > 80% of rated voltage
                        is_powered = (
                            1.0
                            if voltage > 0.8 * (comp.rated_voltage or 24.0)
                            else 0.0
                        )
                        if comp.assembly_part_ref:
                            self.is_powered_map[comp.assembly_part_ref] = is_powered
            else:
                logger.warning(
                    "electronics_validation_failed", errors=res.errors
                )
                # If circuit is invalid, all motors are unpowered
                for comp in self.electronics.components:
                    if comp.type == "motor" and comp.assembly_part_ref:
                        self.is_powered_map[comp.assembly_part_ref] = 0.0

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

        self.stress_summaries = []

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
        self.stress_summaries = []

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

        STRESS_REPORT_INTERVAL = 50

        for step_idx in range(steps):
            # Apply dynamic controllers
            if dynamic_controllers:
                ctrls = {}
                for name, controller in dynamic_controllers.items():
                    val = controller(current_time)
                    # Gate by electronics (T014)
                    # If electronics section exists, use is_powered_map (defaults to 0.0 for known motors)
                    # If no electronics section, default to 1.0 (backward compatibility)
                    if hasattr(self, "electronics") and self.electronics:
                        power_scale = getattr(self, "is_powered_map", {}).get(name, 0.0)
                        # If the actuator is not in electronics, it might be a passive part or something else.
                        # But usually all motors should be in electronics if it exists.
                        val *= power_scale
                    ctrls[name] = val
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

            # 2.1 Stress Monitoring & Breakage Detection
            if step_idx % STRESS_REPORT_INTERVAL == 0:
                for body_name in all_bodies:
                    stress_field = self.backend.get_stress_field(body_name)
                    if stress_field is not None and len(stress_field.stress) > 0:
                        max_stress = np.max(stress_field.stress)
                        mean_stress = np.mean(stress_field.stress)
                        max_idx = np.argmax(stress_field.stress)
                        max_loc = stress_field.nodes[max_idx]

                        # We'd need material data here for yield/ultimate
                        # For now, placeholder values or fetch from config
                        yield_stress = 276e6  # Default Aluminum
                        ultimate_stress = 310e6

                        summary = StressSummary(
                            part_label=body_name,
                            max_von_mises_pa=max_stress,
                            mean_von_mises_pa=mean_stress,
                            safety_factor=yield_stress / max_stress
                            if max_stress > 0
                            else 100.0,
                            location_of_max=tuple(max_loc.tolist()),
                            utilization_pct=max_stress / yield_stress * 100.0,
                        )
                        self.stress_summaries.append(summary)

                        # F004: Part Breakage
                        if max_stress > ultimate_stress:
                            self.fail_reason = f"PART_BREAKAGE:{body_name}"
                            logger.info(
                                "simulation_fail",
                                reason="PART_BREAKAGE",
                                part=body_name,
                                stress=max_stress,
                            )
                            break
                if self.fail_reason and "PART_BREAKAGE" in self.fail_reason:
                    break

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

            # 7. Check Wire Failure (T015)
            if self.electronics:
                wire_broken = False
                for wire in self.electronics.wiring:
                    if getattr(wire, "routed_in_3d", False):
                        try:
                            tension = self.backend.get_tendon_tension(wire.wire_id)
                            # Heuristic for tensile strength: AWG 18 ~200N, AWG 24 ~50N
                            # Limit = (Area in mm2) * (Tensile strength in MPa)
                            # Copper is ~200-250 MPa.
                            # Using a conservative limit for now.
                            limit = 100.0 * (1.26 ** (18 - wire.gauge_awg))
                            if tension > limit:
                                self.fail_reason = f"{SimulationFailureMode.WIRE_TORN}:{wire.wire_id}"
                                emit_event(ElectricalFailureEvent(
                                    failure_type="wire_torn",
                                    component_id=wire.wire_id,
                                    message=f"Wire {wire.wire_id} torn due to high tension ({tension:.2f}N > {limit:.2f}N)"
                                ))
                                logger.info(
                                    "simulation_fail",
                                    reason="wire_torn",
                                    wire=wire.wire_id,
                                    tension=tension,
                                )
                                wire_broken = True
                                # If wire breaks, update circuit
                                # Since SPICE is slow, we'll assume a broken wire kills the circuit for simplicity in the loop.
                                for motor_name in self.is_powered_map:
                                    self.is_powered_map[motor_name] = 0.0
                                break
                        except Exception:
                            # Tendon might not be in backend if not routed
                            pass
                if wire_broken:
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
            stress_summaries=self.stress_summaries,
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
