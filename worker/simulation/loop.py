from pathlib import Path

import numpy as np
import structlog
from build123d import Compound, Part

from shared.enums import SimulationFailureMode
from shared.models.schemas import ElectronicsSection, ObjectivesYaml
from shared.models.simulation import FluidMetricResult
from shared.observability.events import emit_event
from shared.observability.schemas import (
    ElectricalFailureEvent,
    SimulationBackendSelectedEvent,
)
from shared.simulation.backends import SimulationScene
from shared.simulation.schemas import SimulatorBackendType
from worker.simulation.electronics import ElectronicsManager
from worker.simulation.evaluator import SuccessEvaluator
from worker.simulation.factory import get_physics_backend
from worker.simulation.metrics import MetricCollector, SimulationMetrics
from worker.utils.dfm import validate_and_price
from worker.utils.rendering import VideoRenderer
from worker.workbenches.config import load_config
from worker.workbenches.models import ManufacturingMethod

logger = structlog.get_logger(__name__)


# Hard cap on simulation time per architecture spec
MAX_SIMULATION_TIME_SECONDS = 30.0
# Motor overload threshold: fail if clamped for this duration (seconds)
MOTOR_OVERLOAD_THRESHOLD_SECONDS = 2.0
# Standard simulation step for MuJoCo (2ms)
SIMULATION_STEP_S = 0.002


class SimulationLoop:
    def __init__(
        self,
        xml_path: str | Path,
        component: Part | Compound | None = None,
        max_simulation_time: float = MAX_SIMULATION_TIME_SECONDS,
        backend_type: SimulatorBackendType = SimulatorBackendType.GENESIS,
        electronics: ElectronicsSection | None = None,
        objectives: ObjectivesYaml | None = None,
        smoke_test_mode: bool = False,
    ):
        # WP2: Validate that fluids are NOT requested if using MuJoCo
        if (
            backend_type == SimulatorBackendType.MUJOCO
            and objectives
            and objectives.fluids
        ):
            raise ValueError(
                "MuJoCo backend does not support fluids. Use Genesis instead."
            )

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
        self.electronics_validation_error = None

        if self.component:
            # WP2: Load custom configuration from working directory if present
            working_dir = Path(xml_path).parent
            custom_config_path = working_dir / "manufacturing_config.yaml"
            if custom_config_path.exists():
                self.config = load_config(custom_config_path)
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
                    metadata = getattr(child, "metadata", None)
                    self.material_lookup[label] = getattr(metadata, "material_id", None)
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

        # Cache dynamic names to avoid repeated API/FFI calls in loop (Performance)
        self.actuator_names = self.backend.get_all_actuator_names()
        self.body_names = [
            b for b in self.backend.get_all_body_names() if b not in ["world", "0"]
        ]

        # Pre-calculate actuator limits for overload check
        self.actuators_to_monitor = []
        for name in self.actuator_names:
            limit = 1.0  # default fallback
            try:
                # MuJoCo optimization: check if force limited and get range
                import mujoco

                if hasattr(self.backend, "model") and self.backend.model:
                    idx = mujoco.mj_name2id(
                        self.backend.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                    )
                    if idx >= 0:
                        if self.backend.model.actuator_forcelimited[idx]:
                            limit = self.backend.model.actuator_forcerange[idx][1]
                            self.actuators_to_monitor.append((name, limit))
                        # If not limited, we don't monitor it (consistent with original logic)
                        continue
            except Exception:
                # Genesis or other backend: monitor all with default limit
                pass

            # Fallback for non-MuJoCo or failed lookup
            self.actuators_to_monitor.append((name, limit))

        logger.info(
            "SimulationLoop_init",
            goal_sites=self.goal_sites,
            forbidden_sites=self.forbidden_sites,
            monitored_actuators=len(self.actuators_to_monitor),
        )

        # Configurable timeout (capped at hard limit)
        self.max_simulation_time = min(max_simulation_time, MAX_SIMULATION_TIME_SECONDS)

        self.stress_summaries = []
        self.fluid_metrics = []
        self.actuator_clamp_duration = {}

        # T016: Persistent state for flow rate tracking
        self.prev_particle_distances = {}  # objective_id -> distances array
        self.cumulative_crossed_count = {}  # objective_id -> int

        # Initial capture of distances for flow rate objectives
        if self.objectives and self.objectives.objectives:
            particles = self.backend.get_particle_positions()
            if particles is not None and len(particles) > 0:
                for fo in self.objectives.objectives.fluid_objectives:
                    if fo.type == "flow_rate":
                        p0 = np.array(fo.gate_plane_point)
                        n = np.array(fo.gate_plane_normal)
                        distances = np.dot(particles - p0, n)
                        obj_id = f"{fo.fluid_id}_{fo.type}"
                        self.prev_particle_distances[obj_id] = distances

        self.electronics_manager = ElectronicsManager(self.electronics)
        self.metric_collector = MetricCollector()
        self.success_evaluator = SuccessEvaluator(
            max_simulation_time=self.max_simulation_time,
            motor_overload_threshold=MOTOR_OVERLOAD_THRESHOLD_SECONDS,
        )

        # T015: derive is_powered_map from electronics.circuit state
        if self.electronics:
            self._update_electronics(force=True)

        # Reset metrics
        self.reset_metrics()

    def _update_electronics(self, force=False):
        """Update is_powered_map based on circuit state."""
        self.electronics_manager.update(force=force)
        # Bridge back is_powered_map for existing code compatibility if needed,
        # but better to use electronics_manager.is_powered_map directly.
        self.is_powered_map = self.electronics_manager.is_powered_map

    def reset_metrics(self):
        self.metric_collector.reset()
        self.success = False
        self.fail_reason = None
        self.overloaded_motors: list[str] = []
        self.stress_summaries = []
        self.fluid_metrics = []

    def step(
        self,
        control_inputs: dict[str, float],
        duration: float = 10.0,
        dynamic_controllers: dict[str, callable] | None = None,
        render_callback=None,
        video_path: Path | None = None,
    ) -> SimulationMetrics:
        """
        Runs the simulation for the specified duration.
        Returns metrics.
        """
        self.reset_metrics()

        video_renderer = None
        if video_path:
            video_renderer = VideoRenderer(video_path)

        # T012: Check validation hook before starting
        if self.validation_report and not getattr(
            self.validation_report, "is_manufacturable", False
        ):
            violations = getattr(self.validation_report, "violations", None) or [
                "unknown error"
            ]
            msg = f"validation_failed: {', '.join(map(str, violations))}"
            self.fail_reason = SimulationFailureMode.VALIDATION_FAILED
            return SimulationMetrics(
                total_time=0.0,
                total_energy=0.0,
                max_velocity=0.0,
                success=False,
                fail_reason=msg,
                fail_mode=self.fail_reason,
            )

        # Check electronics validation status
        if self.electronics_validation_error:
            self.fail_reason = SimulationFailureMode.VALIDATION_FAILED
            return SimulationMetrics(
                total_time=0.0,
                total_energy=0.0,
                max_velocity=0.0,
                success=False,
                fail_reason=self.electronics_validation_error,
                fail_mode=self.fail_reason,
                confidence="high",
            )

        # Apply initial static controls
        # T015: Power gate initial controls
        gated_controls = {}
        for name, val in control_inputs.items():
            if self.electronics:
                power_scale = self.is_powered_map.get(name, 0.0)
                val *= power_scale
            gated_controls[name] = val
        self.backend.apply_control(gated_controls)

        # We assume backend has a fixed or default timestep
        # For MuJoCo it's usually 0.002
        # We'll use a small step and loop
        dt = 0.002  # Default step for loop logic
        steps = int(duration / dt)

        current_time = 0.0

        # Find critical bodies
        target_body_name = "target_box"
        # Use cached body names + world/0 (which we filtered out but might need for target search?)
        # Actually 'target_box' is usually a moving body so it should be in self.body_names
        if target_body_name not in self.body_names:
            target_body_name = None
            # Fallback: look for target_box OR
            # any body with 'target' or 'bucket' in name
            for name in self.body_names:
                if "target" in name.lower() or "bucket" in name.lower():
                    target_body_name = name
                    break

        logger.info("SimulationLoop_step_start", target_body_name=target_body_name)

        for step_idx in range(steps):
            # T015: Update electronics if state changed
            if self.electronics and self._electronics_dirty:
                self._update_electronics()
                # Re-apply static controls with new power status
                re_gated = {}
                for name, val in control_inputs.items():
                    power_scale = self.is_powered_map.get(name, 0.0)
                    re_gated[name] = val * power_scale
                self.backend.apply_control(re_gated)

                if self.electronics_validation_error:
                    self.fail_reason = self.electronics_validation_error
                    break

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

            # 2. Update Metrics
            energy = sum(
                abs(
                    self.backend.get_actuator_state(n).ctrl
                    * self.backend.get_actuator_state(n).velocity
                )
                for n in self.actuator_names
            )

            target_vel = 0.0
            target_pos = None
            if target_body_name:
                state = self.backend.get_body_state(target_body_name)
                target_vel = np.linalg.norm(state.vel)
                target_pos = state.pos

            # TODO: Get max stress from backend
            max_stress = 0.0
            self.metric_collector.update(dt, energy, target_vel, max_stress)

            # 4. Check Success/Failure conditions
            fail_reason = self.success_evaluator.check_failure(
                current_time,
                target_pos,
                state.vel if target_pos is not None else np.zeros(3),
            )
            if fail_reason:
                self.fail_reason = fail_reason
                break

            if not res.success:
                self.fail_reason = (
                    res.failure_reason
                    if isinstance(res.failure_reason, SimulationFailureMode)
                    else SimulationFailureMode.PHYSICS_INSTABILITY
                )
                break

            # Check Forbidden Zones
            if self.forbidden_sites and self._check_forbidden_collision():
                self.fail_reason = SimulationFailureMode.FORBID_ZONE_HIT
                break

            # Check Goal Zone
            if target_body_name and self.check_goal_with_vertices(target_body_name):
                self.success = True
                break

            # Check Motor Overload
            if self._check_motor_overload():
                self.fail_reason = SimulationFailureMode.OVERCURRENT
                break

            # 7. Check Wire Failure (T015)
            if self.electronics:
                from shared.wire_utils import get_awg_properties

                wire_broken = False
                for wire in self.electronics.wiring:
                    if getattr(wire, "routed_in_3d", False):
                        try:
                            tension = self.backend.get_tendon_tension(wire.wire_id)
                            # T016: Use accurate tensile strength from AWG lookup
                            props = get_awg_properties(wire.gauge_awg)
                            limit = props["tensile_strength_n"]
                            if tension > limit:
                                self.fail_reason = SimulationFailureMode.WIRE_TORN
                                emit_event(
                                    ElectricalFailureEvent(
                                        failure_type="wire_torn",
                                        component_id=wire.wire_id,
                                        message=(
                                            f"Wire {wire.wire_id} torn due to "
                                            f"high tension ({tension:.2f}N > "
                                            f"{limit:.2f}N)"
                                        ),
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

            if video_renderer and step_idx % 15 == 0:  # ~33 fps with dt=0.002
                # Setup a reasonable camera for video if not already set
                # For MuJoCo we can use "free" or a named camera.
                # For Genesis we use "main".
                frame = self.backend.render_camera("main", 640, 480)
                particles = self.backend.get_particle_positions()
                video_renderer.add_frame(frame, particles=particles)

        # Finalize video
        if video_renderer:
            video_renderer.save()

        # Final stress evaluation
        self.stress_summaries = self.backend.get_stress_summaries()

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
                                    or SimulationFailureMode.FLUID_OBJECTIVE_FAILED
                                )
                        elif fo.type == "flow_rate":
                            # T016: Use cumulative crossed count for more accurate
                            # flow rate check
                            obj_id = f"{fo.fluid_id}_{fo.type}"
                            passed_count = self.cumulative_crossed_count.get(obj_id, 0)

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
                                    or SimulationFailureMode.FLUID_OBJECTIVE_FAILED
                                )

        # Final success determination:
        # Success if no failures AND (goal achieved IF goals exist,
        # or fluid/stress objectives passed)
        has_other_objectives = False
        if (
            self.objectives
            and self.objectives.objectives
            and (
                self.objectives.objectives.fluid_objectives
                or self.objectives.objectives.stress_objectives
            )
        ):
            has_other_objectives = True

        if self.fail_reason:
            is_success = False
        elif self.goal_sites:
            # If target object AND goals are required, success depends on
            # goal achievement
            is_success = self.success
        elif has_other_objectives:
            # If no failures and fluid/stress objectives passed, it's a success
            is_success = True
        else:
            # Stability test or goal-less benchmark
            # (legacy behavior expects False if no goal)
            is_success = False

        metrics = self.metric_collector.get_metrics()

        if fail_reason:
            is_success = False
        elif self.goal_sites:
            is_success = self.success
        elif has_other_objectives:
            is_success = True
        else:
            is_success = False

        return SimulationMetrics(
            total_time=current_time,
            total_energy=metrics.total_energy,
            max_velocity=metrics.max_velocity,
            success=is_success,
            fail_reason=str(self.fail_reason) if self.fail_reason else None,
            fail_mode=self.fail_reason
            if isinstance(self.fail_reason, SimulationFailureMode)
            else None,
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

    def _check_motor_overload(self) -> bool:
        # Check all position/torque actuators for saturation
        from worker.simulation.loop import SIMULATION_STEP_S

        # Use pre-calculated monitor list
        for name, limit in self.actuators_to_monitor:
            state = self.backend.get_actuator_state(name)
            if self.success_evaluator.check_motor_overload(
                [name], [state.force], limit, SIMULATION_STEP_S
            ):
                return True
        return False

    def check_goal_with_vertices(self, body_name: str) -> bool:
        """Check if any vertices of body_name are inside any of the goal sites."""
        return any(
            self.backend.check_collision(body_name, goal_site)
            for goal_site in self.goal_sites
        )

    def _check_forbidden_collision(self) -> bool:
        # Use cached body names (already filtered)
        return any(
            self.backend.check_collision(b, z)
            for b in self.body_names
            for z in self.forbidden_sites
        )
