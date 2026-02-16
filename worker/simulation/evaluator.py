import numpy as np
from shared.enums import SimulationFailureMode


class SuccessEvaluator:
    """Evaluates simulation success or failure conditions."""

    def __init__(
        self, max_simulation_time: float, motor_overload_threshold: float = 2.0
    ):
        self.max_simulation_time = max_simulation_time
        self.motor_overload_threshold = motor_overload_threshold
        self.motor_overload_timer: dict[str, float] = {}

    def check_failure(
        self,
        total_time: float,
        qpos: np.ndarray,
        qvel: np.ndarray,
        contacts: list = None,
    ) -> str | None:
        """
        Check for various failure modes.
        Returns failure reason string or None if still running.
        """
        # 1. Timeout
        if total_time >= self.max_simulation_time:
            return "timeout_exceeded"

        # 2. Physics Instability (NaNs)
        if np.any(np.isnan(qpos)) or np.any(np.isnan(qvel)):
            return "PHYSICS_INSTABILITY"

        # 3. Fell off world
        # Heuristic: Z < -2.0
        # Assume free joint: qpos[2] is Z
        if len(qpos) >= 3 and qpos[2] < -2.0:
            return "target_fell_off_world"

        return None

    def check_motor_overload(
        self, motor_names: list[str], forces: list[float], limit: float, dt: float
    ) -> bool:
        """Identify motors stalled at their limit."""
        for i, name in enumerate(motor_names):
            if abs(forces[i]) >= limit * 0.99:
                self.motor_overload_timer[name] = (
                    self.motor_overload_timer.get(name, 0) + dt
                )
                if self.motor_overload_timer[name] >= self.motor_overload_threshold:
                    return True
            else:
                self.motor_overload_timer[name] = 0
        return False

    def is_in_zone(
        self, pos: np.ndarray, zone_pos: np.ndarray, zone_size: np.ndarray
    ) -> bool:
        """Check if a position is within a box zone."""
        return np.all(np.abs(pos - zone_pos) <= zone_size)
