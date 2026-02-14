from pathlib import Path

import numpy as np
import structlog
from build123d import Compound, Part
from pydantic import BaseModel, StrictBool, StrictFloat, StrictStr

from shared.enums import SimulationFailureMode
from shared.models.schemas import ElectronicsSection, ObjectivesYaml
from shared.observability.events import emit_event
from shared.observability.schemas import (
    ElectricalFailureEvent,
    PhysicsInstabilityEvent,
    SimulationBackendSelectedEvent,
)
from shared.simulation.backends import SimulationScene, SimulatorBackendType
from worker.simulation.factory import get_physics_backend

logger = structlog.get_logger(__name__)


class StressSummary(BaseModel):
    part_label: str
    max_von_mises_pa: float
    mean_von_mises_pa: float
    safety_factor: float  # yield_stress / max_von_mises
    location_of_max: tuple[float, float, float]
    utilization_pct: float  # max_stress / yield_stress * 100


class FluidMetricResult(BaseModel):
    metric_type: str  # "fluid_containment" | "flow_rate"
    fluid_id: str
    measured_value: float
    target_value: float
    passed: bool


class SimulationMetrics(BaseModel):
    total_time: StrictFloat
    total_energy: StrictFloat
    max_velocity: StrictFloat
    success: StrictBool
    fail_reason: StrictStr | None = None
    stress_summaries: list[StressSummary] = []
    fluid_metrics: list[FluidMetricResult] = []
    confidence: StrictStr = "high"


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
        objectives: ObjectivesYaml | None = None,
        smoke_test_mode: bool = False,
    ):
        self.backend = get_physics_backend(backend_type)
        self.smoke_test_mode = smoke_test_mode
        self.particle_budget = 5000 if smoke_test_mode else 100000

        # Emit backend selection event (WP2)
        emit_event(
            SimulationBackendSelectedEvent(
                backend=backend_type.value
                if hasattr(backend_type, "value")
                else backend_type,
                fem_enabled=objectives.physics.fem_enabled if objectives else False,
                compute_target=objectives.physics.compute_target
                if objectives
                else "auto",
            )
        )
        scene = SimulationScene(scene_path=xml_path)
        self.backend.load_scene(scene)

        self.component = component
        self.electronics = electronics
        self.objectives = objectives
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
                            1.0 if voltage > 0.8 * (comp.rated_voltage or 24.0) else 0.0
                        )
                        if comp.assembly_part_ref:
                            self.is_powered_map[comp.assembly_part_ref] = is_powered
            else:
                logger.warning("electronics_validation_failed", errors=res.errors)
                # If circuit is invalid, all motors are unpowered
                for comp in self.electronics.components:
                    if comp.type == "motor" and comp.assembly_part_ref:
                        self.is_powered_map[comp.assembly_part_ref] = 0.0

        if self.component:
            from worker.utils.dfm import validate_and_price
            from worker.workbenches.config import load_config
            from worker.workbenches.models import ManufacturingMethod

            # WP2: Load custom configuration from working directory if present
            working_dir = Path(xml_path).parent
            custom_config_path = working_dir / "manufacturing_config.yaml"
            if custom_config_path.exists():
                self.config = load_config(str(custom_config_path))
            else:
                self.config = load_config()

            fem_required = (
                self.objectives.physics.fem_enabled
                if self.objectives and self.objectives.physics
                else False
            )
            self.validation_report = validate_and_price(
                self.component,
                ManufacturingMethod.CNC,
                self.config,
                fem_required=fem_required,
            )

            # Build material lookup
            self.material_lookup = {}
            children = getattr(self.component, "children", [])
            if not children:
                children = [self.component]
            for child in children:
                label = getattr(child, "label", None)
                if label:
                    self.material_lookup[label] = child.metadata.get("material_id")
        else:
            self.config = None
            self.material_lookup = {}

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
        self.fluid_metrics = []

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
        self.fluid_metrics = []

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
        # We'll use a small step and loop
        dt = 0.002  # Default step for loop logic
        steps = int(duration / dt)

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

        stress_report_interval = 50

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
                if res.failure_reason == "instability_detected":
                    self.fail_reason = "PHYSICS_INSTABILITY"
                    emit_event(
                        PhysicsInstabilityEvent(
                            kinetic_energy=0.0,  # Placeholder
                            threshold=0.0,
                            step=step_idx,
                        )
                    )
                else:
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
            if step_idx % stress_report_interval == 0:
                for body_name in all_bodies:
                    stress_field = self.backend.get_stress_field(body_name)
                    if stress_field is not None and len(stress_field.stress) > 0:
                        max_stress = np.max(stress_field.stress)
                        mean_stress = np.mean(stress_field.stress)
                        max_idx = np.argmax(stress_field.stress)
                        max_loc = stress_field.nodes[max_idx]

                        # WP2: Fetch material data from config
                        mat_id = self.material_lookup.get(body_name)
                        mat_def = None
                        if mat_id and self.config:
                            mat_def = self.config.materials.get(mat_id)
                            # Also check CNC specific materials if not in global
                            if not mat_def and self.config.cnc:
                                mat_def = self.config.cnc.materials.get(mat_id)

                        yield_stress = (
                            mat_def.yield_stress_pa
                            if mat_def and mat_def.yield_stress_pa
                            else 276e6
                        )
                        ultimate_stress = (
                            mat_def.ultimate_stress_pa
                            if mat_def and mat_def.ultimate_stress_pa
                            else 310e6
                        )

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

            # 2.2 Fluid & Stress Objectives (WP2)
            if self.objectives and self.objectives.objectives:
                # 2.2.1 Stress Objectives
                for so in self.objectives.objectives.stress_objectives:
                    stress_field = self.backend.get_stress_field(so.part_label)
                    if stress_field is not None and len(stress_field.stress) > 0:
                        max_s = np.max(stress_field.stress)
                        if max_s > so.max_von_mises_mpa * 1e6:
                            self.fail_reason = (
                                f"STRESS_OBJECTIVE_EXCEEDED:{so.part_label}"
                            )
                            logger.info(
                                "simulation_fail",
                                reason="STRESS_OBJECTIVE_EXCEEDED",
                                part=so.part_label,
                                stress=max_s,
                            )
                            break
                if self.fail_reason:
                    break

                # 2.2.2 Fluid Objectives (Continuous checks)
                for fo in self.objectives.objectives.fluid_objectives:
                    if hasattr(fo, "eval_at") and fo.eval_at == "continuous":
                        # Perform fluid containment/flow check
                        # This requires particle positions from backend
                        pass

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
                                self.fail_reason = (
                                    f"{SimulationFailureMode.WIRE_TORN}:{wire.wire_id}"
                                )
                                emit_event(
                                    ElectricalFailureEvent(
                                        failure_type="wire_torn",
                                        component_id=wire.wire_id,
                                        message=f"Wire {wire.wire_id} torn due to high tension ({tension:.2f}N > {limit:.2f}N)",
                                    )
                                )
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

        # Final fluid objectives evaluation (eval_at='end')
        if self.objectives and self.objectives.objectives:
            for fo in self.objectives.objectives.fluid_objectives:
                if not hasattr(fo, "eval_at") or fo.eval_at == "end":
                    particles = self.backend.get_particle_positions()
                    if particles is not None:
                        if fo.type == "fluid_containment":
                            # Count particles inside containment_zone
                            zone = fo.containment_zone
                            inside = np.all(
                                (particles >= zone.min) & (particles <= zone.max),
                                axis=1,
                            )
                            ratio = (
                                np.sum(inside) / len(particles)
                                if len(particles) > 0
                                else 0.0
                            )
                            passed = ratio >= fo.threshold
                            result = FluidMetricResult(
                                metric_type="fluid_containment",
                                fluid_id=fo.fluid_id,
                                measured_value=float(ratio),
                                target_value=fo.threshold,
                                passed=passed,
                            )
                            self.fluid_metrics.append(result)

                            from shared.observability.schemas import (
                                FluidContainmentCheckEvent,
                            )

                            emit_event(
                                FluidContainmentCheckEvent(
                                    fluid_id=fo.fluid_id,
                                    ratio=float(ratio),
                                    threshold=fo.threshold,
                                    passed=passed,
                                )
                            )
                            if not passed:
                                self.fail_reason = (
                                    self.fail_reason
                                    or f"FLUID_OBJECTIVE_FAILED:{fo.fluid_id}"
                                )
                        elif fo.type == "flow_rate":
                            # Flow rate at end is less common, but we could measure cumulative flow
                            pass

        return SimulationMetrics(
            total_time=current_time,
            total_energy=self.total_energy,
            max_velocity=self.max_velocity,
            success=self.success if self.fail_reason is None else False,
            fail_reason=self.fail_reason,
            stress_summaries=self.stress_summaries,
            fluid_metrics=self.fluid_metrics,
            confidence="approximate" if self.smoke_test_mode else "high",
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
