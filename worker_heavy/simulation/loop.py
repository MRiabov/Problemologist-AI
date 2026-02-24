from pathlib import Path

import numpy as np
import structlog
from build123d import Compound, Part

from shared.enums import FluidEvalAt, FluidObjectiveType, FailureReason
from shared.models.schemas import ElectronicsSection, ObjectivesYaml
from shared.models.simulation import (
    FluidMetricResult,
    SimulationFailure,
    SimulationMetrics,
)
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
from worker_heavy.simulation.objectives import ObjectiveEvaluator
from worker_heavy.simulation.media import MediaRecorder
from worker_heavy.simulation.factory import get_physics_backend
from worker_heavy.simulation.metrics import MetricCollector
from worker_heavy.utils.dfm import validate_and_price
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
        smoke_test_mode: bool | None = None,
        session_id: str | None = None,
        particle_budget: int | None = None,
    ):
        from worker_heavy.config import settings

        if smoke_test_mode is None:
            smoke_test_mode = settings.smoke_test_mode
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
            scene_config = {"particle_budget": self.particle_budget}
            if objectives and objectives.simulation_bounds:
                scene_config["simulation_bounds"] = (
                    objectives.simulation_bounds.model_dump()
                )

            scene = SimulationScene(
                scene_path=str(xml_path),
                config=scene_config,
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

            self.actuator_clamp_duration = {}

            # T017: Set electronics for fluid damage detection
            if self.electronics:
                elec_names = [comp.component_id for comp in self.electronics.components]
                if hasattr(self.backend, "set_electronics"):
                    self.backend.set_electronics(elec_names)

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
            self.objective_evaluator = ObjectiveEvaluator(
                objectives=self.objectives,
                material_lookup=self.material_lookup,
                config=self.config,
            )
            self.objective_evaluator.initialize_flow_rate(self.backend)

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

    @property
    def stress_summaries(self):
        return self.objective_evaluator.stress_summaries

    @property
    def fluid_metrics(self):
        return self.objective_evaluator.fluid_metrics

    def reset_metrics(self):
        self.metric_collector.reset()
        self.success = False
        self.fail_reason = None
        self.overloaded_motors: list[str] = []
        self.objective_evaluator.reset()
        # WP2: Re-initialize flow rate capture after reset to ensure first-step crossings are caught
        if hasattr(self, "backend") and self.backend:
            self.objective_evaluator.initialize_flow_rate(self.backend)

    def step(
        self,
        control_inputs: dict[str, float],
        duration: float = 10.0,
        dynamic_controllers: dict[str, callable] | None = None,
        render_callback=None,
        video_path: Path | None = None,
        reset_metrics: bool = True,
    ) -> SimulationMetrics:
        """
        Runs the simulation for the specified duration.
        Returns metrics.
        """
        if reset_metrics:
            self.reset_metrics()

        # 1. Pre-simulation validation
        validation_error_metrics = self._perform_pre_simulation_validation()
        if validation_error_metrics:
            return validation_error_metrics

        # 2. Setup recorders
        media_recorder = MediaRecorder(video_path)

        # 3. Apply initial controls
        self._apply_gated_controls(control_inputs)

        # 4. Determine timestep and steps
        dt = self._get_simulation_timestep()
        steps = int(duration / dt)
        current_time = 0.0

        # 5. Find target body
        target_body_name = self._identify_target_body()
        logger.info("SimulationLoop_step_start", target_body_name=target_body_name)

        # 6. Main simulation loop
        for step_idx in range(steps):
            self.current_step_idx = step_idx

            # Update electronics if state changed
            if self.electronics and self._electronics_dirty:
                self._update_electronics()
                self._electronics_dirty = False
                self._apply_gated_controls(control_inputs)
                if self.electronics_validation_error:
                    # In this case self.fail_reason might be set by _update_electronics
                    # which uses self.electronics_manager.validation_error
                    if isinstance(self.electronics_validation_error, SimulationFailure):
                        self.fail_reason = self.electronics_validation_error
                    else:
                        self.fail_reason = SimulationFailure(
                            reason=FailureReason.VALIDATION_FAILED,
                            detail=str(self.electronics_validation_error),
                        )
                    break

            # Apply dynamic controllers
            if dynamic_controllers:
                self._apply_gated_controls({}, current_time, dynamic_controllers)

            # Step backend
            res = self.backend.step(dt)
            current_time = res.time

            # Check failures and update metrics
            # T018: Keep interval small for collisions/overload to avoid missing events
            check_interval = 1
            if step_idx % check_interval == 0 or step_idx == steps - 1:
                if self._check_simulation_failure(
                    res, current_time, dt * check_interval, target_body_name
                ):
                    break

            # Check wire failure
            if self._handle_wire_failure():
                break

            # Video recording
            media_recorder.update(step_idx, self.backend)

        # 7. Finalization
        media_recorder.save()
        self.objective_evaluator.evaluate_final(self.backend, current_time)

        return self._build_simulation_metrics(current_time)

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

    def _check_motor_overload(self, dt: float) -> str | None:
        # Check all position/torque actuators for saturation
        if not self._monitor_names:
            return None

        forces = [
            abs(self.backend.get_actuator_state(n).force) for n in self._monitor_names
        ]

        # SuccessEvaluator.check_motor_overload returns bool but we want the name of the motor
        # Wait, the evaluator's check_motor_overload returns bool.
        # Let's see how it's implemented on main.
        # It returns bool. If true, it means SOME motor is overloaded.
        if self.success_evaluator.check_motor_overload(
            self._monitor_names, forces, self._monitor_limits, dt
        ):
            # Find which one
            for i, name in enumerate(self._monitor_names):
                if (
                    self.success_evaluator.motor_overload_timer.get(name, 0)
                    >= self.success_evaluator.motor_overload_threshold
                ):
                    return name
        return None

    def check_goal_with_vertices(self, body_name: str) -> bool:
        """Check if any vertices of body_name are inside any of the goal sites."""
        return any(
            self.backend.check_collision(body_name, goal_site)
            for goal_site in self.goal_sites
        )

    def _check_forbidden_collision(self) -> str | None:
        """Check for collisions with forbidden zones. Returns the name of the colliding body."""
        for b in self.body_names:
            for z in self.forbidden_sites:
                if self.backend.check_collision(b, z):
                    return b
        return None

    def _perform_pre_simulation_validation(self) -> SimulationMetrics | None:
        """Check validation status before starting simulation."""
        if self.validation_report and not getattr(
            self.validation_report, "is_manufacturable", False
        ):
            violations = getattr(self.validation_report, "violations", None) or [
                "unknown error"
            ]
            msg = f"validation_failed: {', '.join(map(str, violations))}"
            self.fail_reason = SimulationFailure(
                reason=FailureReason.VALIDATION_FAILED, detail=msg
            )
            return SimulationMetrics(
                total_time=0.0,
                total_energy=0.0,
                max_velocity=0.0,
                success=False,
                fail_reason=str(self.fail_reason),
                fail_mode=self.fail_reason.reason,
                failure=self.fail_reason,
            )

        if self.electronics_validation_error:
            if isinstance(self.electronics_validation_error, SimulationFailure):
                self.fail_reason = self.electronics_validation_error
            else:
                self.fail_reason = SimulationFailure(
                    reason=FailureReason.VALIDATION_FAILED,
                    detail=str(self.electronics_validation_error),
                )
            return SimulationMetrics(
                total_time=0.0,
                total_energy=0.0,
                max_velocity=0.0,
                success=False,
                fail_reason=str(self.fail_reason),
                fail_mode=self.fail_reason.reason,
                failure=self.fail_reason,
                confidence="high",
            )

        if self.wire_clearance_error:
            self.fail_reason = SimulationFailure(
                reason=FailureReason.VALIDATION_FAILED,
                detail=self.wire_clearance_error,
            )
            return SimulationMetrics(
                total_time=0.0,
                total_energy=0.0,
                max_velocity=0.0,
                success=False,
                fail_reason=str(self.fail_reason),
                fail_mode=self.fail_reason.reason,
                failure=self.fail_reason,
                confidence="high",
            )
        return None

    def _get_simulation_timestep(self) -> float:
        """Determine appropriate dt for the backend."""
        if hasattr(self.backend, "timestep"):
            return self.backend.timestep
        if hasattr(self.backend, "model") and hasattr(self.backend.model, "opt"):
            return self.backend.model.opt.timestep
        if self.smoke_test_mode and self.backend_type == SimulatorBackendType.GENESIS:
            if self.objectives and self.objectives.physics.fem_enabled:
                return 0.002
            return 0.05
        return SIMULATION_STEP_S

    def _apply_gated_controls(
        self,
        control_inputs: dict[str, float],
        current_time: float | None = None,
        dynamic_controllers: dict[str, callable] | None = None,
    ):
        """Apply power-gated controls to the backend."""
        ctrls = {}
        if dynamic_controllers and current_time is not None:
            for name, controller in dynamic_controllers.items():
                val = controller(current_time)
                if self.electronics:
                    power_scale = self.is_powered_map.get(name, 0.0)
                    val *= power_scale
                ctrls[name] = val
        else:
            for name, val in control_inputs.items():
                if self.electronics:
                    power_scale = self.is_powered_map.get(name, 0.0)
                    val *= power_scale
                ctrls[name] = val
        self.backend.apply_control(ctrls)

    def _check_simulation_failure(
        self, res, current_time: float, dt_interval: float, target_body_name: str | None
    ) -> bool:
        """Aggregate failure checks from backends and evaluators."""
        # 1. Update Metrics
        actuator_states = {
            n: self.backend.get_actuator_state(n) for n in self.actuator_names
        }
        energy = sum(
            abs(state.ctrl * state.velocity) for state in actuator_states.values()
        )

        target_vel = 0.0
        if target_body_name:
            state = self.backend.get_body_state(target_body_name)
            target_vel = np.linalg.norm(state.vel)

        max_stress = self.backend.get_max_stress()
        self.metric_collector.update(dt_interval, energy, target_vel, max_stress)

        # 2. Objective Evaluator Checks
        fail_sim = self.objective_evaluator.update(
            self.backend,
            current_time,
            dt_interval,
            getattr(self, "current_step_idx", 0),
        )
        if fail_sim:
            self.fail_reason = fail_sim
            return True

        # 3. Backend failure checks
        if not res.success:
            self.fail_reason = self._resolve_backend_failure(res)
            return True

        # 4. SuccessEvaluator checks
        for bname in self.body_names:
            bstate = self.backend.get_body_state(bname)
            eval_fail_reason = self.success_evaluator.check_failure(
                current_time, bstate.pos, bstate.vel
            )
            if eval_fail_reason:
                if eval_fail_reason == FailureReason.OUT_OF_BOUNDS:
                    logger.warning(
                        "out_of_bounds_detected",
                        body=bname,
                        pos=bstate.pos,
                        bounds=self.objectives.simulation_bounds.model_dump()
                        if self.objectives and self.objectives.simulation_bounds
                        else None,
                    )
                self.fail_reason = SimulationFailure(
                    reason=eval_fail_reason, detail=bname
                )
                return True

        # 5. Collision checks
        if self.forbidden_sites:
            colliding_body = self._check_forbidden_collision()
            if colliding_body:
                self.fail_reason = SimulationFailure(
                    reason=FailureReason.FORBID_ZONE_HIT, detail=colliding_body
                )
                return True

        # 6. Goal reached
        if target_body_name and self.check_goal_with_vertices(target_body_name):
            self.success = True
            return True

        # 7. Motor overload
        overloaded_motor = self._check_motor_overload(dt_interval)
        if overloaded_motor:
            self.fail_reason = SimulationFailure(
                reason=FailureReason.MOTOR_OVERLOAD, detail=overloaded_motor
            )
            return True

        return False

    def _resolve_backend_failure(self, res) -> SimulationFailure:
        """Translate backend failure into SimulationFailure."""
        logger.info("DEBUG_backend_failure", reason=res.failure_reason)
        if res.failure:
            return res.failure

        # Legacy support for string reasons
        if isinstance(res.failure_reason, str):
            if res.failure_reason.startswith("PART_BREAKAGE"):
                part_name = (
                    res.failure_reason.split(":")[1]
                    if ":" in res.failure_reason
                    else None
                )
                # Check for stress objective violation upgrade
                if part_name and self.objectives and self.objectives.objectives:
                    for so in self.objectives.objectives.stress_objectives:
                        if so.part_label.lower() == part_name.lower():
                            logger.info(
                                "stress_objective_exceeded_via_breakage",
                                part=part_name,
                            )
                            return SimulationFailure(
                                reason=FailureReason.STRESS_OBJECTIVE_EXCEEDED,
                                detail=part_name,
                            )
                return SimulationFailure(
                    reason=FailureReason.PART_BREAKAGE, detail=part_name
                )
            if res.failure_reason.startswith("ELECTRONICS_FLUID_DAMAGE"):
                return SimulationFailure(reason=FailureReason.ELECTRONICS_FLUID_DAMAGE)

        # Default fallback: re-check breakage or assume instability
        if self.objectives and self.objectives.physics.fem_enabled:
            broken_part = self.objective_evaluator.check_part_breakage(
                self.backend, getattr(self, "current_step_idx", 0)
            )
            if broken_part:
                return SimulationFailure(
                    reason=FailureReason.PART_BREAKAGE, detail=broken_part
                )

        return SimulationFailure(reason=FailureReason.PHYSICS_INSTABILITY)

    def _identify_target_body(self) -> str | None:
        """Identify the primary target body for objective tracking."""
        all_bodies = self.backend.get_all_body_names()

        # Priority 1: Check objectives for moved_object label
        if self.objectives and self.objectives.moved_object:
            label = self.objectives.moved_object.label
            if label in all_bodies:
                return label

        # Priority 2: Standard target_box name
        target_body_name = "target_box"
        if target_body_name in all_bodies:
            return target_body_name

        # Priority 3: Heuristic search
        for name in all_bodies:
            if "target" in name.lower() or "bucket" in name.lower():
                return name
        return None

    def _build_simulation_metrics(self, current_time: float) -> SimulationMetrics:
        """Construct the final SimulationMetrics object."""
        metrics = self.metric_collector.get_metrics()

        # WP2: Sync fail_reason from objective_evaluator if it was set during evaluate_final
        if not self.fail_reason and self.objective_evaluator.fail_reason:
            self.fail_reason = self.objective_evaluator.fail_reason

        # Final success determination
        has_other_objectives = bool(
            self.objectives
            and self.objectives.objectives
            and (
                self.objectives.objectives.fluid_objectives
                or self.objectives.objectives.stress_objectives
            )
        )

        if self.fail_reason:
            is_success = False
        elif self.goal_sites:
            is_success = self.success
        elif has_other_objectives:
            is_success = True
        else:
            is_success = True

        confidence = "high"
        if self.smoke_test_mode:
            confidence = "approximate"
        # T017: If particle reduction occurred, it's an approximation
        if hasattr(self.backend, "current_particle_multiplier"):
            multiplier = self.backend.current_particle_multiplier
            if isinstance(multiplier, (int, float)):
                # We need to be careful about floating point comparison
                if multiplier < (self.particle_budget / 100000.0) - 1e-5:
                    confidence = "approximate"

        return SimulationMetrics(
            total_time=current_time,
            total_energy=metrics.total_energy,
            max_velocity=metrics.max_velocity,
            max_stress=metrics.max_stress,
            success=is_success,
            fail_reason=str(self.fail_reason) if self.fail_reason else None,
            fail_mode=self.fail_reason.reason if self.fail_reason else None,
            failure=self.fail_reason,
            stress_summaries=self.stress_summaries,
            stress_fields=self._get_stress_fields(),
            fluid_metrics=self.fluid_metrics,
            events=metrics.events,
            confidence=confidence,
        )

    def _handle_wire_failure(self) -> bool:
        """Check for wire tension and breakage."""
        if not self.electronics:
            return False

        wire_broken = False
        for wire in self.electronics.wiring:
            if getattr(wire, "routed_in_3d", False):
                try:
                    tension = self.backend.get_tendon_tension(wire.wire_id)
                    props = get_awg_properties(wire.gauge_awg)
                    limit = props["tensile_strength_n"]
                    if tension > limit:
                        self.fail_reason = SimulationFailure(
                            reason=FailureReason.WIRE_TORN, detail=wire.wire_id
                        )
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
                        self.electronics.wiring.remove(wire)
                        self._electronics_dirty = True
                        self._update_electronics()
                        self._electronics_dirty = False
                        break
                except Exception:
                    pass
        return wire_broken
