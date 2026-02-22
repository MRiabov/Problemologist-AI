import numpy as np
import structlog

from shared.enums import FluidEvalAt, FluidObjectiveType, SimulationFailureMode
from shared.models.simulation import FluidMetricResult
from shared.observability.events import emit_event
from shared.observability.schemas import (
    FlowRateCheckEvent,
    FluidContainmentCheckEvent,
    PartBreakageEvent,
)

logger = structlog.get_logger(__name__)


class ObjectiveEvaluator:
    """Evaluates simulation objectives (stress, fluid, breakage)."""

    def __init__(self, objectives, material_lookup, config):
        self.objectives = objectives
        self.material_lookup = material_lookup
        self.config = config
        self.reset()

    def reset(self):
        """Reset evaluator state."""
        self.prev_particle_distances = {}  # objective_id -> distances array
        self.cumulative_crossed_count = {}  # objective_id -> int
        self.fluid_metrics = []
        self.fail_reason = None
        self.stress_summaries = []

    def initialize_flow_rate(self, backend):
        """Initial capture of distances for flow rate objectives."""
        if self.objectives and self.objectives.objectives:
            particles = backend.get_particle_positions()
            if particles is not None and len(particles) > 0:
                for i, fo in enumerate(self.objectives.objectives.fluid_objectives):
                    if fo.type == FluidObjectiveType.FLOW_RATE:
                        p0 = np.array(fo.gate_plane_point)
                        n = np.array(fo.gate_plane_normal)
                        distances = np.dot(particles - p0, n)
                        obj_id = f"{fo.fluid_id}_{fo.type}_{i}"
                        self.prev_particle_distances[obj_id] = distances

    def update(self, backend, current_time, dt_interval, current_step_idx=0):
        """Update metrics and check for objective violations during simulation."""
        if not self.objectives or not self.objectives.objectives:
            return None

        # 1. Check Stress Objectives
        self.stress_summaries = backend.get_stress_summaries()
        for so in self.objectives.objectives.stress_objectives:
            summary = next(
                (s for s in self.stress_summaries if s.part_label == so.part_label),
                None,
            )
            if summary and summary.max_von_mises_pa > (so.max_von_mises_mpa * 1e6):
                self.fail_reason = SimulationFailureMode.STRESS_OBJECTIVE_EXCEEDED
                logger.info(
                    "stress_objective_exceeded",
                    part=so.part_label,
                    stress=summary.max_von_mises_pa,
                    limit=so.max_von_mises_mpa * 1e6,
                )
                return self.fail_reason

        # 2. Check Part Breakage
        if self.objectives.physics.fem_enabled:
            broken_part = self.check_part_breakage(backend, current_step_idx)
            if broken_part:
                self.fail_reason = SimulationFailureMode.PART_BREAKAGE
                return self.fail_reason

        # 3. Track Flow Rate
        has_flow_rate = any(
            fo.type == FluidObjectiveType.FLOW_RATE
            for fo in self.objectives.objectives.fluid_objectives
        )
        if has_flow_rate:
            particles = backend.get_particle_positions()
            if particles is not None and len(particles) > 0:
                for i, fo in enumerate(self.objectives.objectives.fluid_objectives):
                    if fo.type == FluidObjectiveType.FLOW_RATE:
                        obj_id = f"{fo.fluid_id}_{fo.type}_{i}"
                        p0 = np.array(fo.gate_plane_point)
                        n = np.array(fo.gate_plane_normal)
                        current_distances = np.dot(particles - p0, n)
                        prev_distances = self.prev_particle_distances.get(obj_id)
                        if prev_distances is not None and len(prev_distances) == len(
                            current_distances
                        ):
                            crossed_pos = (prev_distances < 0) & (current_distances >= 0)
                            count = np.sum(crossed_pos)
                            self.cumulative_crossed_count[obj_id] = (
                                self.cumulative_crossed_count.get(obj_id, 0) + count
                            )
                        self.prev_particle_distances[obj_id] = current_distances

        return None

    def evaluate_final(self, backend, current_time):
        """Final evaluation of end-of-simulation objectives."""
        if not self.objectives or not self.objectives.objectives:
            return

        # Ensure we have the latest stress summaries
        self.stress_summaries = backend.get_stress_summaries()

        for i, fo in enumerate(self.objectives.objectives.fluid_objectives):
            if not hasattr(fo, "eval_at") or fo.eval_at == FluidEvalAt.END:
                particles = backend.get_particle_positions()
                if particles is not None:
                    if fo.type == FluidObjectiveType.FLUID_CONTAINMENT:
                        self._evaluate_fluid_containment(fo, particles)
                    elif fo.type == FluidObjectiveType.FLOW_RATE:
                        self._evaluate_flow_rate(fo, i, current_time)

    def _evaluate_fluid_containment(self, fo, particles):
        zone = fo.containment_zone
        z_min = np.array(zone.min)
        z_max = np.array(zone.max)
        inside = np.all((particles >= z_min) & (particles <= z_max), axis=1)
        ratio = np.sum(inside) / len(particles) if len(particles) > 0 else 0.0
        passed = ratio >= fo.threshold
        result = FluidMetricResult(
            metric_type="fluid_containment",
            fluid_id=fo.fluid_id,
            measured_value=float(ratio),
            target_value=fo.threshold,
            passed=passed,
        )
        self.fluid_metrics.append(result)
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
                self.fail_reason or SimulationFailureMode.FLUID_OBJECTIVE_FAILED
            )

    def _evaluate_flow_rate(self, fo, index, current_time):
        obj_id = f"{fo.fluid_id}_{fo.type}_{index}"
        passed_count = self.cumulative_crossed_count.get(obj_id, 0)
        # Heuristic: 1 particle ~= 0.001L (1ml) for MVP
        measured_volume_l = passed_count * 0.001
        measured_rate = measured_volume_l / current_time if current_time > 0 else 0.0
        passed = measured_rate >= fo.target_rate_l_per_s * (1.0 - fo.tolerance)
        result = FluidMetricResult(
            metric_type="flow_rate",
            fluid_id=fo.fluid_id,
            measured_value=float(measured_rate),
            target_value=fo.target_rate_l_per_s,
            passed=passed,
        )
        self.fluid_metrics.append(result)
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
                self.fail_reason or SimulationFailureMode.FLUID_OBJECTIVE_FAILED
            )

    def check_part_breakage(self, backend, current_step_idx):
        """Checks if any FEM part has exceeded its ultimate stress limit."""
        # Use cached summaries if available from the update loop
        summaries = self.stress_summaries
        for summary in summaries:
            label = summary.part_label
            mat_id = self.material_lookup.get(label)
            if not mat_id:
                continue

            mat_def = self.config.materials.get(mat_id)
            if not mat_def and self.config.cnc:
                mat_def = self.config.cnc.materials.get(mat_id)

            if mat_def and mat_def.ultimate_stress_pa:
                is_broken = False
                if np.isnan(summary.max_von_mises_pa):
                    logger.warning("part_breakage_nan_detected", part=label)
                    is_broken = True
                elif summary.max_von_mises_pa > mat_def.ultimate_stress_pa:
                    is_broken = True

                if is_broken:
                    emit_event(
                        PartBreakageEvent(
                            part_label=label,
                            stress_mpa=summary.max_von_mises_pa / 1e6
                            if not np.isnan(summary.max_von_mises_pa)
                            else 0.0,
                            ultimate_mpa=mat_def.ultimate_stress_pa / 1e6,
                            location=summary.location_of_max or (0, 0, 0),
                            step=current_step_idx,
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
