"""Multi-run simulation verification with runtime randomization.

Implements runtime randomization verification per architecture spec:
- Run simulation multiple times (default 5) with perturbed initial positions
- Check for consistency across runs to detect flaky designs
- Use MuJoCo's support for parallel simulations
"""

import logging
from typing import Any

# import mujoco  # Moved to lazy imports
import numpy as np
from pydantic import BaseModel

from worker.simulation.loop import SimulationLoop, SimulationMetrics

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


def apply_position_jitter(
    data: Any,
    target_body_id: int,
    jitter_range: tuple[float, float, float],
    rng: np.random.Generator,
) -> None:
    """Apply random position jitter to a body.

    Args:
        data: MuJoCo data object.
        target_body_id: ID of the body to jitter.
        jitter_range: (±x, ±y, ±z) jitter in simulation units.
        rng: NumPy random generator for reproducibility.
    """
    if target_body_id == -1:
        return

    # For free joints, qpos indices are: x, y, z, qw, qx, qy, qz
    # We only jitter position (first 3 values)
    jitter = np.array(
        [
            rng.uniform(-jitter_range[0], jitter_range[0]),
            rng.uniform(-jitter_range[1], jitter_range[1]),
            rng.uniform(-jitter_range[2], jitter_range[2]),
        ]
    )

    # Find qpos address for this body (assuming free joint is first)
    data.qpos[0:3] += jitter


def verify_with_jitter(
    xml_path: str,
    control_inputs: dict[str, float],
    jitter_range: tuple[float, float, float] = (0.002, 0.002, 0.001),
    num_runs: int = DEFAULT_NUM_RUNS,
    duration: float = 10.0,
    seed: int = 42,
    dynamic_controllers: dict[str, Any] | None = None,
) -> MultiRunResult:
    """Run simulation multiple times with perturbed initial positions.

    Args:
        xml_path: Path to MJCF file.
        control_inputs: Static control inputs for actuators.
        jitter_range: (±x, ±y, ±z) position jitter in simulation units.
        num_runs: Number of verification runs.
        duration: Duration of each simulation run.
        seed: Base random seed for reproducibility.
        dynamic_controllers: Optional dynamic controller functions.

    Returns:
        MultiRunResult with aggregated statistics.
    """
    results: list[SimulationMetrics] = []
    rng = np.random.default_rng(seed)

    from shared.simulation.schemas import SimulatorBackendType

    for run_idx in range(num_runs):
        # Create fresh simulation state for each run
        loop = SimulationLoop(xml_path, backend_type=SimulatorBackendType.MUJOCO)

        # Find target body for jittering
        import mujoco

        target_body_id = mujoco.mj_name2id(
            loop.backend.model, mujoco.mjtObj.mjOBJ_BODY, "target_box"
        )

        # Apply position jitter
        apply_position_jitter(loop.backend.data, target_body_id, jitter_range, rng)

        # Forward kinematics after position change
        import mujoco

        mujoco.mj_forward(loop.backend.model, loop.backend.data)

        # Run simulation
        metrics = loop.step(
            control_inputs,
            duration=duration,
            dynamic_controllers=dynamic_controllers,
        )

        results.append(metrics)
        logger.info(
            f"Verification run {run_idx} finished",
            success=metrics.success,
            fail_reason=metrics.fail_reason,
        )
        logger.debug(
            f"Run {run_idx + 1}/{num_runs}: "
            f"success={metrics.success}, fail_reason={metrics.fail_reason}"
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
