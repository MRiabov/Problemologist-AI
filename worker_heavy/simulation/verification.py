"""Multi-run simulation verification with runtime randomization.

Implements runtime randomization verification per architecture spec:
- Run simulation multiple times (default 5) with perturbed initial positions
- Check for consistency across runs to detect flaky designs
- Use MuJoCo's support for parallel simulations
"""

import logging
from typing import Any

import numpy as np
from build123d import Compound
from pydantic import BaseModel

from shared.enums import MotorControlMode
from shared.models.schemas import ElectronicsSection, ObjectivesYaml, AssemblyPartConfig
from shared.simulation.schemas import SimulatorBackendType
from worker_heavy.simulation.loop import SimulationLoop, SimulationMetrics
from worker_heavy.utils.controllers import sinusoidal

logger = logging.getLogger(__name__)

# Default number of verification runs
DEFAULT_NUM_RUNS = 5


class MultiRunResult(BaseModel):
    """Result of multi-run verification."""

    num_runs: int
    success_count: int
    success_rate: float
    is_consistent: bool  # True if all runs agree on success/fail
    individual_results: list[SimulationMetrics]
    fail_reasons: list[str]  # Unique failure reasons across runs


def verify_with_jitter(
    scene_path: str,
    component: Compound | None = None,
    electronics: ElectronicsSection | None = None,
    objectives: ObjectivesYaml | None = None,
    moving_parts: list[AssemblyPartConfig] | None = None,
    jitter_range: tuple[float, float, float] = (0.002, 0.002, 0.001),
    num_runs: int = DEFAULT_NUM_RUNS,
    duration: float = 10.0,
    seed: int = 42,
    backend_type: SimulatorBackendType = SimulatorBackendType.GENESIS,
    smoke_test_mode: bool = False,
    session_id: str | None = None,
    particle_budget: int | None = None,
) -> MultiRunResult:
    """Run simulation multiple times with perturbed initial positions.

    Args:
        scene_path: Path to scene file (MJCF/XML/JSON).
        component: The CAD component being simulated.
        electronics: Electronics configuration.
        objectives: Simulation objectives.
        moving_parts: List of moving parts configurations.
        jitter_range: (±x, ±y, ±z) position jitter in simulation units.
        num_runs: Number of verification runs.
        duration: Duration of each simulation run.
        seed: Base random seed for reproducibility.
        backend_type: Physics backend to use.
        smoke_test_mode: Whether to run in smoke test mode.
        session_id: Optional session ID for backend caching.
        particle_budget: Optional particle budget override.

    Returns:
        MultiRunResult with aggregated statistics.
    """
    results: list[SimulationMetrics] = []
    rng = np.random.default_rng(seed)

    # Prepare dynamic controllers and control inputs base
    # Note: We re-create lambda functions for each run if needed, but the logic is static per assembly
    # However, to be safe and consistent with simulate(), we'll set them up here.
    base_dynamic_controllers = {}
    base_control_inputs = {}

    if moving_parts:
        try:
            for part in moving_parts:
                if part.control:
                    if part.control.mode == MotorControlMode.SINUSOIDAL:
                        base_dynamic_controllers[part.part_name] = lambda t, p=part.control: (
                            sinusoidal(t, p.speed, p.frequency or 1.0)
                        )
                    elif part.control.mode == MotorControlMode.CONSTANT:
                        base_control_inputs[part.part_name] = part.control.speed
                    elif part.control.mode == MotorControlMode.ON_OFF:
                        # T019: Handle ON_OFF mode using frequency toggle
                        freq = part.control.frequency or 1.0
                        period = 1.0 / freq
                        base_dynamic_controllers[part.part_name] = (
                            lambda t, p=part.control, per=period: (
                                p.speed if (t % per) < (per / 2) else 0.0
                            )
                        )
        except Exception as e:
            logger.warning("failed_to_load_controllers_verification", error=str(e))


    for run_idx in range(num_runs):
        # Create fresh simulation state for each run
        loop = SimulationLoop(
            scene_path,
            component=component,
            backend_type=backend_type,
            electronics=electronics,
            objectives=objectives,
            smoke_test_mode=smoke_test_mode,
            session_id=session_id,
            particle_budget=particle_budget,
        )

        # Apply position jitter via backend-agnostic method if possible
        # For now, we manually apply it to the target body if it exists
        target_body_name = "target_box"

        jitter = [
            rng.uniform(-jitter_range[0], jitter_range[0]),
            rng.uniform(-jitter_range[1], jitter_range[1]),
            rng.uniform(-jitter_range[2], jitter_range[2]),
        ]

        # Use backend-agnostic jitter application (WP01)
        loop.backend.apply_jitter(target_body_name, jitter)

        # Run simulation
        metrics = loop.step(
            control_inputs=base_control_inputs.copy(),
            duration=duration,
            dynamic_controllers=base_dynamic_controllers.copy(),
        )

        results.append(metrics)
        logger.info(
            f"Verification run {run_idx} finished",
            success=metrics.success,
            fail_reason=metrics.fail_reason,
        )

    # Aggregate results
    success_count = sum(1 for r in results if r.success)
    success_rate = success_count / num_runs if num_runs > 0 else 0.0

    # Check consistency: all runs should agree
    outcomes = [r.success for r in results]
    is_consistent = len(set(outcomes)) == 1

    # Collect unique failure reasons
    fail_reasons = list(
        set(r.fail_reason for r in results if r.fail_reason is not None)
    )

    return MultiRunResult(
        num_runs=num_runs,
        success_count=success_count,
        success_rate=success_rate,
        is_consistent=is_consistent,
        individual_results=results,
        fail_reasons=fail_reasons,
    )
