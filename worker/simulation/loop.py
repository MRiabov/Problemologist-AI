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
    PartBreakageEvent,
    PhysicsInstabilityEvent,
    SimulationBackendSelectedEvent,
    StressSummaryEvent,
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
    stress_fields: dict[str, dict] = {}  # part_label -> {"nodes": ..., "stress": ...}
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

        self.stress_summaries = []
        self.fluid_metrics = []
        self.actuator_clamp_duration = {}

        # T015: derive is_powered_map from electronics.circuit state
        self.is_powered_map = {}
        self.switch_states = {}
        self._electronics_dirty = True
        if self.electronics:
            for comp in self.electronics.components:
                if comp.type in ["switch", "relay"]:
                    self.switch_states[comp.component_id] = True
            self._update_electronics(force=True)

        # Reset metrics
        self.reset_metrics()

    def _update_electronics(self, force=False):
        """Update is_powered_map based on circuit state."""
        if not self.electronics:
            return

        if not force and not self._electronics_dirty:
            return

        from shared.circuit_builder import build_circuit_from_section
        from shared.pyspice_utils import validate_circuit

        try:
            self.circuit = build_circuit_from_section(
                self.electronics, self.switch_states
            )
            res = validate_circuit(self.circuit, self.electronics.power_supply)
            if res.valid:
                logger.debug(
                    "electronics_simulation_success", total_draw=res.total_draw_a
                )
                for comp in self.electronics.components:
                    if comp.type == "motor":
                        v_diff = abs(
                            res.node_voltages.get(f"{comp.component_id}_+", 0.0)
                            - res.node_voltages.get(f"{comp.component_id}_-", 0.0)
                        )
                        is_powered = 1.0 if v_diff > 0.1 else 0.0
                        self.is_powered_map[comp.component_id] = is_powered
                        if comp.assembly_part_ref:
                            self.is_powered_map[comp.assembly_part_ref] = is_powered
                self._electronics_dirty = False
                return
            else:
                logger.warning("electronics_validation_failed", errors=res.errors)
        except Exception as e:
            logger.debug("electronics_simulation_failed_using_fallback", error=str(e))

        # Fallback: Simple connectivity-based power gating
        self._fallback_electronics_update()
        self._electronics_dirty = False

    def _fallback_electronics_update(self):
        """Connectivity-based fallback for is_powered_map if SPICE fails."""
        if not self.electronics:
            return

        # Simple BFS to find nodes connected to supply_v+
        adj = {}  # node -> set of connected nodes

        def add_edge(u, v):
            if u not in adj:
                adj[u] = set()
            if v not in adj:
                adj[v] = set()
            adj[u].add(v)
            adj[v].add(u)

        # 1. Add wires
        for wire in self.electronics.wiring:
            # Use same normalization as circuit_builder
            def norm(c, t):
                if t == "supply_v+" or (c == "supply" and t == "v+"):
                    return "supply_v+"
                if t == "0" or (c == "supply" and t == "0"):
                    return "0"
                if t == "a":
                    t = "+"
                if t == "b":
                    t = "-"
                return f"{c}_{t}"

            add_edge(
                norm(wire.from_terminal.component, wire.from_terminal.terminal),
                norm(wire.to_terminal.component, wire.to_terminal.terminal),
            )

        # 2. Add closed switches
        for comp in self.electronics.components:
            if comp.type in ["switch", "relay"]:
                if self.switch_states.get(comp.component_id, True):
                    add_edge(f"{comp.component_id}_in", f"{comp.component_id}_out")

        # 3. Traverse from supply_v+
        powered_nodes = set()
        stack = ["supply_v+"]
        visited = {"supply_v+"}
        while stack:
            u = stack.pop()
            powered_nodes.add(u)
            for v in adj.get(u, []):
                if v not in visited:
                    visited.add(v)
                    stack.append(v)

        # 4. Map motors: powered if both + and - terminals are reached?
        # No, a motor is powered if + is connected to VCC and - is connected to GND.
        # Let's find nodes connected to GND (0)
        gnd_nodes = set()
        stack = ["0"]
        visited = {"0"}
        while stack:
            u = stack.pop()
            gnd_nodes.add(u)
            for v in adj.get(u, []):
                if v not in visited:
                    visited.add(v)
                    stack.append(v)

        for comp in self.electronics.components:
            if comp.type == "motor":
                is_powered = (
                    1.0
                    if (
                        f"{comp.component_id}_+" in powered_nodes
                        and f"{comp.component_id}_-" in gnd_nodes
                    )
                    else 0.0
                )
                self.is_powered_map[comp.component_id] = is_powered
                if comp.assembly_part_ref:
                    self.is_powered_map[comp.assembly_part_ref] = is_powered

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
        dt = 0.002  # Default step for loop logic
        steps = int(duration / dt)

        current_time = 0.0

        # Find critical bodies
        target_body_name = "target_box"
        all_bodies = self.backend.get_all_body_names()
        if target_body_name not in all_bodies:
            target_body_name = None
            # Fallback: look for target_box OR any body with 'target' or 'bucket' in name
            for name in all_bodies:
                if "target" in name.lower() or "bucket" in name.lower():
                    target_body_name = name
                    break

        logger.info("SimulationLoop_step_start", target_body_name=target_body_name)

        stress_report_interval = 1 if self.smoke_test_mode else 50

        for step_idx in range(steps):
            # T015: Update electronics if state changed
            if self.electronics and self._electronics_dirty:
                self._update_electronics()

            # Apply dynamic controllers
            if dynamic_controllers:
                ctrls = {}
                for name, controller in dynamic_controllers.items():
                    val = controller(current_time)
                    if self.electronics:
                        power_scale = self.is_powered_map.get(name, 0.0)
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

                        emit_event(
                            StressSummaryEvent(
                                part_label=body_name,
                                max_von_mises=max_stress,
                                safety_factor=summary.safety_factor,
                            )
                        )

                        # F004: Part Breakage
                        if max_stress > ultimate_stress:
                            self.fail_reason = (
                                f"{SimulationFailureMode.PART_BREAKAGE}:{body_name}"
                            )
                            emit_event(
                                PartBreakageEvent(
                                    part_label=body_name,
                                    stress_mpa=max_stress / 1e6,
                                    ultimate_mpa=ultimate_stress / 1e6,
                                    location=tuple(max_loc.tolist()),
                                    step=step_idx,
                                )
                            )
                            logger.info(
                                "simulation_fail",
                                reason=SimulationFailureMode.PART_BREAKAGE,
                                part=body_name,
                                stress=max_stress,
                            )
                            # Internal break for the body loop
                            break
                # Check for breakage in the outer loop
                if (
                    self.fail_reason
                    and SimulationFailureMode.PART_BREAKAGE in self.fail_reason
                ):
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
                            self.fail_reason = f"{SimulationFailureMode.STRESS_OBJECTIVE_EXCEEDED}:{so.part_label}"
                            logger.info(
                                "simulation_fail",
                                reason=SimulationFailureMode.STRESS_OBJECTIVE_EXCEEDED,
                                part=so.part_label,
                                stress=max_s,
                            )
                            break
                if self.fail_reason:
                    break

                # 2.2.2 Fluid Objectives (Continuous checks)
                for fo in self.objectives.objectives.fluid_objectives:
                    if getattr(fo, "eval_at", "end") == "continuous":
                        if fo.type == "fluid_containment":
                            particles = self.backend.get_particle_positions()
                            if particles is not None and len(particles) > 0:
                                zone = fo.containment_zone
                                inside = np.all(
                                    (particles >= zone.min) & (particles <= zone.max),
                                    axis=1,
                                )
                                ratio = np.sum(inside) / len(particles)
                                if ratio < fo.threshold:
                                    self.fail_reason = (
                                        f"FLUID_CONTAINMENT_FAILED:{fo.fluid_id}"
                                    )
                                    break
                if self.fail_reason:
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
                                # If wire breaks, remove it and re-evaluate circuit
                                self.electronics.wiring.remove(wire)
                                self._electronics_dirty = True
                                self._update_electronics()
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
                            z_min = np.array(zone.min)
                            z_max = np.array(zone.max)
                            inside = np.all(
                                (particles >= z_min) & (particles <= z_max),
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
                                    or f"{SimulationFailureMode.FLUID_OBJECTIVE_FAILED}:{fo.fluid_id}"
                                )
                        elif fo.type == "flow_rate":
                            # Flow rate at end: measure cumulative flow
                            particles = self.backend.get_particle_positions()
                            if particles is not None and len(particles) > 0:
                                p0 = np.array(fo.gate_plane_point)
                                n = np.array(fo.gate_plane_normal)
                                # Distance from plane: (p - p0) . n
                                distances = np.dot(particles - p0, n)
                                passed_count = np.sum(distances > 0)
                                # Heuristic: 1 particle ~= 0.001L (1ml) for MVP
                                measured_volume_l = passed_count * 0.001
                                measured_rate = (
                                    measured_volume_l / current_time
                                    if current_time > 0
                                    else 0.0
                                )

                                passed = measured_rate >= fo.target_rate_l_per_s * (
                                    1.0 - fo.tolerance
                                )
                                result = FluidMetricResult(
                                    metric_type="flow_rate",
                                    fluid_id=fo.fluid_id,
                                    measured_value=float(measured_rate),
                                    target_value=fo.target_rate_l_per_s,
                                    passed=passed,
                                )
                                self.fluid_metrics.append(result)

                                from shared.observability.schemas import (
                                    FlowRateCheckEvent,
                                )

                                emit_event(
                                    FlowRateCheckEvent(
                                        fluid_id=fo.fluid_id,
                                        measured_rate=float(measured_rate),
                                        target_rate=fo.target_rate_l_per_s,
                                        passed=passed,
                                    )
                                )
                                if not passed:
                                    self.fail_reason = (
                                        self.fail_reason
                                        or f"FLOW_RATE_FAILED:{fo.fluid_id}"
                                    )

        # Final success determination:
        # If any explicit failure was recorded, it's not a success.
        if self.fail_reason:
            is_success = False
        elif target_body_name and self.goal_sites:
            # If target object AND goals are required, success depends on goal achievement
            is_success = self.success
        else:
            # If no failures and either no target or no goal sites, it's a pass
            is_success = True

        return SimulationMetrics(
            total_time=current_time,
            total_energy=self.total_energy,
            max_velocity=self.max_velocity,
            success=is_success,
            fail_reason=self.fail_reason,
            stress_summaries=self.stress_summaries,
            stress_fields=self._get_stress_fields(),
            fluid_metrics=self.fluid_metrics,
            confidence="approximate" if self.smoke_test_mode else "high",
        )

    def _get_stress_fields(self) -> dict[str, dict]:
        fields = {}
        if not self.objectives or not self.objectives.objectives:
            return fields

        for so in self.objectives.objectives.stress_objectives:
            field = self.backend.get_stress_field(so.part_label)
            if field is not None:
                fields[so.part_label] = {
                    "nodes": field.nodes.tolist(),
                    "stress": field.stress.tolist(),
                }
        return fields

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
