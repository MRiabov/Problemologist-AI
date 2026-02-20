from pathlib import Path

import numpy as np
import structlog
from build123d import Compound, Part

from shared.enums import FluidEvalAt, FluidObjectiveType, SimulationFailureMode
from shared.models.schemas import ElectronicsSection, ObjectivesYaml
from shared.models.simulation import FluidMetricResult, SimulationMetrics
from shared.observability.events import emit_event
from shared.observability.schemas import (
    ElectricalFailureEvent,
    SimulationBackendSelectedEvent,
)
from shared.simulation.backends import SimulationScene
from shared.simulation.schemas import SimulatorBackendType
from shared.wire_utils import check_wire_clearance, get_awg_properties
from shared.workers.workbench_models import ManufacturingMethod
from worker_heavy.simulation.electronics import ElectronicsManager
from worker_heavy.simulation.evaluator import SuccessEvaluator
from worker_heavy.simulation.factory import get_physics_backend
from worker_heavy.simulation.metrics import MetricCollector
from worker_heavy.utils.dfm import validate_and_price
from worker_heavy.utils.rendering import VideoRenderer
from worker_heavy.workbenches.config import load_config

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
        session_id: str | None = None,
        particle_budget: int | None = None,
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

        self.backend = get_physics_backend(
            backend_type,
            session_id=session_id,
            smoke_test_mode=smoke_test_mode,
            particle_budget=particle_budget,
        )
        self.smoke_test_mode = smoke_test_mode
        self.backend_type = backend_type
        # Propagate smoke test mode to backend for optimization (in case of cache hit)
        if hasattr(self.backend, "smoke_test_mode"):
            self.backend.smoke_test_mode = smoke_test_mode
        if hasattr(self.backend, "particle_budget"):
            self.backend.particle_budget = particle_budget

        self.particle_budget = particle_budget or (5000 if smoke_test_mode else 100000)

        try:
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
            scene = SimulationScene(
                scene_path=str(xml_path),
                config={"particle_budget": self.particle_budget},
            )
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
                    logger.info(
                        "loop_loaded_custom_config",
                        path=str(custom_config_path),
                        materials=list(self.config.cnc.materials.keys())
                        if self.config.cnc
                        else [],
                    )
                else:
                    self.config = load_config()
                    logger.info(
                        "loop_loaded_default_config",
                        materials=list(self.config.materials.keys()),
                    )

                fem_required = (
                    self.objectives.physics.fem_enabled
                    if self.objectives and self.objectives.physics
                    else False
                )

                # Resolve manufacturing method from component metadata
                mfg_method = ManufacturingMethod.CNC
                children = getattr(self.component, "children", [])
                if not children:
                    children = [self.component]

                for child in children:
                    meta = getattr(child, "metadata", None)
                    if meta and getattr(meta, "manufacturing_method", None):
                        mfg_method = meta.manufacturing_method
                        break

                self.validation_report = validate_and_price(
                    self.component,
                    mfg_method,
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
                        self.material_lookup[label] = getattr(
                            metadata, "material_id", None
                        )
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
            self.max_simulation_time = min(
                max_simulation_time, MAX_SIMULATION_TIME_SECONDS
            )

            # Performance optimizations: cache backend info
            self.actuator_names = self.backend.get_all_actuator_names()
            self.body_names = [
                b
                for b in self.backend.get_all_body_names()
                if b not in ["world", "0"] and not b.startswith("zone_")
            ]

            # Cache actuator limits for monitoring
            self._monitor_names = []
            self._monitor_limits = []
            for name in self.actuator_names:
                try:
                    state = self.backend.get_actuator_state(name)
                    # Only monitor if there is a non-zero force range
                    if (
                        state.forcerange
                        and len(state.forcerange) >= 2
                        and state.forcerange[1] > state.forcerange[0]
                    ):
                        self._monitor_names.append(name)
                        self._monitor_limits.append(state.forcerange[1])
                    elif backend_type != SimulatorBackendType.MUJOCO:
                        # Default for other backends (e.g. Genesis)
                        self._monitor_names.append(name)
                        self._monitor_limits.append(1000.0)
                except Exception:
                    pass

            self.stress_summaries = []
            self.fluid_metrics = []
            self.actuator_clamp_duration = {}

            # T016: Persistent state for flow rate tracking
            self.prev_particle_distances = {}  # objective_id -> distances array
            self.cumulative_crossed_count = {}  # objective_id -> int

            # T017: Set electronics for fluid damage detection
            if self.electronics:
                elec_names = [comp.component_id for comp in self.electronics.components]
                if hasattr(self.backend, "set_electronics"):
                    self.backend.set_electronics(elec_names)

            # Initial capture of distances for flow rate objectives
            if self.objectives and self.objectives.objectives:
                particles = self.backend.get_particle_positions()
                if particles is not None and len(particles) > 0:
                    for i, fo in enumerate(self.objectives.objectives.fluid_objectives):
                        if fo.type == FluidObjectiveType.FLOW_RATE:
                            p0 = np.array(fo.gate_plane_point)
                            n = np.array(fo.gate_plane_normal)
                            distances = np.dot(particles - p0, n)
                            obj_id = f"{fo.fluid_id}_{fo.type}_{i}"
                            self.prev_particle_distances[obj_id] = distances

            self.electronics_manager = ElectronicsManager(self.electronics)
            self._electronics_dirty = False
            self.metric_collector = MetricCollector()
            self.success_evaluator = SuccessEvaluator(
                max_simulation_time=self.max_simulation_time,
                motor_overload_threshold=MOTOR_OVERLOAD_THRESHOLD_SECONDS,
                simulation_bounds=self.objectives.simulation_bounds
                if self.objectives
                else None,
            )

            # T015: derive is_powered_map from electronics.circuit state
            if self.electronics:
                self._update_electronics(force=True)
                self.wire_clearance_error = self._validate_wire_clearance()
            else:
                self.wire_clearance_error = None

            # Reset metrics
            self.reset_metrics()
        except Exception as e:
            import traceback

            logger.error(
                "SimulationLoop_init_failed",
                error=str(e),
                traceback=traceback.format_exc(),
            )
            raise

    def _validate_wire_clearance(self) -> str | None:
        """T011: Check wire clearance using shared util."""
        # Only check if we have both electronics wiring and a component to check against
        if not self.electronics or not self.electronics.wiring or not self.component:
            return None

        # Ensure component is a Compound (build123d API requirements for distance check)
        check_comp = self.component
        if isinstance(self.component, Part):
            check_comp = Compound(children=[self.component])

        for wire in self.electronics.wiring:
            # Skip wires that don't have enough waypoints or aren't routed in 3D
            if not wire.waypoints or len(wire.waypoints) < 2:
                continue

            # Using default clearance of 2.0mm for now, or could come from constraints
            if not check_wire_clearance(wire.waypoints, check_comp, clearance_mm=2.0):
                return f"Wire clearance violation detected for wire {wire.wire_id}."

        return None

    def _update_electronics(self, force=False):
        """Update is_powered_map based on circuit state."""
        self.electronics_manager.update(force=force)
        # Bridge back is_powered_map for existing code compatibility if needed,
        # but better to use electronics_manager.is_powered_map directly.
        self.is_powered_map = self.electronics_manager.is_powered_map
        self.electronics_validation_error = self.electronics_manager.validation_error

    @property
    def switch_states(self) -> dict[str, bool]:
        """Expose electronics switch states for the agent/activity."""
        return self.electronics_manager.switch_states

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

        # Check wire clearance validation status
        if self.wire_clearance_error:
            self.fail_reason = SimulationFailureMode.VALIDATION_FAILED
            return SimulationMetrics(
                total_time=0.0,
                total_energy=0.0,
                max_velocity=0.0,
                success=False,
                fail_reason=self.wire_clearance_error,
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

        # WP2: Use backend-specific dt to avoid over-simulating in smoke test mode
        dt = SIMULATION_STEP_S
        if self.smoke_test_mode and self.backend_type == SimulatorBackendType.GENESIS:
            dt = 0.05

        steps = int(duration / dt)

        current_time = 0.0

        # Find critical bodies
        target_body_name = "target_box"
        all_bodies = self.backend.get_all_body_names()
        if target_body_name not in all_bodies:
            target_body_name = None
            # Fallback: look for target_box OR
            # any body with 'target' or 'bucket' in name
            for name in all_bodies:
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

            # OPTIMIZATION: Only check expensive conditions every N steps
            # T018: Keep interval small for collisions/overload to avoid missing events
            check_interval = 1
            if step_idx % check_interval == 0 or step_idx == steps - 1:
                # 2. Update Metrics
                actuator_states = {
                    n: self.backend.get_actuator_state(n) for n in self.actuator_names
                }
                energy = sum(
                    abs(state.ctrl * state.velocity)
                    for state in actuator_states.values()
                )

                target_vel = 0.0
                target_pos = None
                if target_body_name:
                    state = self.backend.get_body_state(target_body_name)
                    target_vel = np.linalg.norm(state.vel)
                    target_pos = state.pos

                max_stress = self.backend.get_max_stress()
                self.metric_collector.update(
                    dt * check_interval, energy, target_vel, max_stress
                )

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

                # Check Forbidden Zones (T018 optimization: skip if no zones)
                if self.forbidden_sites and self._check_forbidden_collision():
                    self.fail_reason = SimulationFailureMode.FORBID_ZONE_HIT
                    break

                # Check Goal Zone
                if target_body_name and self.check_goal_with_vertices(target_body_name):
                    self.success = True
                    break

                # T016: Track Flow Rate
                if self.objectives and self.objectives.objectives:
                    has_flow_rate = any(
                        fo.type == FluidObjectiveType.FLOW_RATE
                        for fo in self.objectives.objectives.fluid_objectives
                    )
                    if has_flow_rate:
                        particles = self.backend.get_particle_positions()
                        if particles is not None and len(particles) > 0:
                            for i, fo in enumerate(
                                self.objectives.objectives.fluid_objectives
                            ):
                                if fo.type == FluidObjectiveType.FLOW_RATE:
                                    obj_id = f"{fo.fluid_id}_{fo.type}_{i}"
                                    p0 = np.array(fo.gate_plane_point)
                                    n = np.array(fo.gate_plane_normal)
                                    current_distances = np.dot(particles - p0, n)
                                    prev_distances = self.prev_particle_distances.get(
                                        obj_id
                                    )
                                    if prev_distances is not None and len(
                                        prev_distances
                                    ) == len(current_distances):
                                        crossed_pos = (prev_distances < 0) & (
                                            current_distances >= 0
                                        )
                                        count = np.sum(crossed_pos)
                                        self.cumulative_crossed_count[obj_id] = (
                                            self.cumulative_crossed_count.get(obj_id, 0)
                                            + count
                                        )
                                    self.prev_particle_distances[obj_id] = (
                                        current_distances
                                    )

                # Check Motor Overload
                if self._check_motor_overload(dt * check_interval):
                    self.fail_reason = SimulationFailureMode.MOTOR_OVERLOAD
                    break

            # WP2: Check for part breakage (INT-103)
            if self.objectives and self.objectives.physics.fem_enabled:
                broken_part = self._check_part_breakage()
                if broken_part:
                    self.fail_reason = SimulationFailureMode.PART_BREAKAGE
                    # Return immediately from step loop on breakage
                    break

            # 7. Check Wire Failure (T015) - Outside FEM block (T019)
            if self.electronics:
                wire_broken = False
                for wire in self.electronics.wiring:
                    if getattr(wire, "routed_in_3d", False):
                        try:
                            tension = self.backend.get_tendon_tension(wire.wire_id)
                            # T016: Use accurate tensile strength from AWG lookup
                            props = get_awg_properties(wire.gauge_awg)
                            limit = props["tensile_strength_n"]
                            if tension > limit:
                                self.fail_reason = f"{SimulationFailureMode.WIRE_TORN.value}:{wire.wire_id}"
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
                try:
                    # T024: Use "main" if available, otherwise first camera, or fallback
                    cameras = self.backend.get_all_camera_names()
                    cam_to_use = "main"
                    if "main" not in cameras and cameras:
                        cam_to_use = cameras[0]

                    frame = self.backend.render_camera(cam_to_use, 640, 480)
                    particles = self.backend.get_particle_positions()
                    video_renderer.add_frame(frame, particles=particles)
                except Exception as e:
                    # T024: Skip rendering if display is not available (e.g. CI without GPU)
                    if "EGL" in str(e) or "display" in str(e).lower():
                        logger.warning(
                            "camera_render_failed_skipping_video", error=str(e)
                        )
                        video_renderer = None  # Stop attempting to render video
                    else:
                        raise

        # Finalize video
        if video_renderer:
            video_renderer.save()

        # Final stress evaluation
        self.stress_summaries = self.backend.get_stress_summaries()

        # Final fluid objectives evaluation (eval_at='end')
        if self.objectives and self.objectives.objectives:
            for i, fo in enumerate(self.objectives.objectives.fluid_objectives):
                if not hasattr(fo, "eval_at") or fo.eval_at == FluidEvalAt.END:
                    particles = self.backend.get_particle_positions()
                    if particles is not None:
                        if fo.type == FluidObjectiveType.FLUID_CONTAINMENT:
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
                        elif fo.type == FluidObjectiveType.FLOW_RATE:
                            # T016: Use cumulative crossed count for more accurate
                            # flow rate check
                            obj_id = f"{fo.fluid_id}_{fo.type}_{i}"
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
        # or fluid/stress objectives passed, or it is a stability-only test)
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
            # If no fail_reason, we consider it a success (simulation was stable)
            is_success = True

        metrics = self.metric_collector.get_metrics()

        return SimulationMetrics(
            total_time=current_time,
            total_energy=metrics.total_energy,
            max_velocity=metrics.max_velocity,
            max_stress=metrics.max_stress,
            success=is_success,
            fail_reason=str(self.fail_reason) if self.fail_reason else None,
            fail_mode=self.fail_reason
            if isinstance(self.fail_reason, SimulationFailureMode)
            else None,
            stress_summaries=self.stress_summaries,
            stress_fields=self._get_stress_fields(),
            fluid_metrics=self.fluid_metrics,
            events=metrics.events,
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

    def _check_part_breakage(self) -> str | None:
        """Checks if any FEM part has exceeded its ultimate stress limit."""
        summaries = self.backend.get_stress_summaries()
        if not summaries:
            # Check max stress as fallback
            max_s = self.backend.get_max_stress()
            if max_s > 1e9:  # Just for debug
                logger.info("high_max_stress_no_summaries", max_stress=max_s)

        for summary in summaries:
            label = summary.part_label
            mat_id = self.material_lookup.get(label)
            logger.info(
                "checking_breakage",
                label=label,
                mat_id=mat_id,
                max_stress=summary.max_von_mises_pa,
            )
            if not mat_id:
                continue

            # Look up material properties
            mat_def = self.config.materials.get(mat_id)
            # Try method-specific if not found globally
            if not mat_def and self.config.cnc:
                mat_def = self.config.cnc.materials.get(mat_id)

            if mat_def and mat_def.ultimate_stress_pa:
                if summary.max_von_mises_pa > mat_def.ultimate_stress_pa:
                    from shared.observability.events import emit_event
                    from shared.observability.schemas import PartBreakageEvent

                    emit_event(
                        PartBreakageEvent(
                            part_label=label,
                            material_id=mat_id,
                            max_stress_pa=summary.max_von_mises_pa,
                            ultimate_stress_pa=mat_def.ultimate_stress_pa,
                        )
                    )
                    logger.info(
                        "part_breakage_detected",
                        part=label,
                        stress=summary.max_von_mises_pa,
                        limit=mat_def.ultimate_stress_pa,
                    )
                    return label
        return None

    def _check_motor_overload(self, dt: float) -> bool:
        # Check all position/torque actuators for saturation
        if not self._monitor_names:
            return False

        forces = [
            abs(self.backend.get_actuator_state(n).force) for n in self._monitor_names
        ]

        return self.success_evaluator.check_motor_overload(
            self._monitor_names, forces, self._monitor_limits, dt
        )

    def check_goal_with_vertices(self, body_name: str) -> bool:
        """Check if any vertices of body_name are inside any of the goal sites."""
        return any(
            self.backend.check_collision(body_name, goal_site)
            for goal_site in self.goal_sites
        )

    def _check_forbidden_collision(self) -> bool:
        return any(
            self.backend.check_collision(b, z)
            for b in self.body_names
            for z in self.forbidden_sites
        )
