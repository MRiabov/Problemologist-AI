import numpy as np

from shared.enums import SimulationFailureMode


class SuccessEvaluator:
    """Evaluates simulation success or failure conditions."""

    def __init__(
        self,
        max_simulation_time: float,
        motor_overload_threshold: float = 2.0,
        simulation_bounds: any = None,
    ):
        self.max_simulation_time = max_simulation_time
        self.motor_overload_threshold = motor_overload_threshold
        self.simulation_bounds = simulation_bounds
        self.motor_overload_timer: dict[str, float] = {}

    def check_failure(
        self,
        total_time: float,
        qpos: np.ndarray,
        qvel: np.ndarray,
        contacts: list = None,
    ) -> SimulationFailureMode | None:
        """
        Check for various failure modes.
        Returns failure reason or None if still running.
        """
        # 1. Timeout
        if total_time >= self.max_simulation_time:
            return SimulationFailureMode.TIMEOUT

        # 2. Physics Instability (NaNs)
        if qpos is not None and qvel is not None:
            if np.any(np.isnan(qpos)) or np.any(np.isnan(qvel)):
                return SimulationFailureMode.PHYSICS_INSTABILITY

        # 3. Fell off world / Out of Bounds
        if qpos is not None and len(qpos) >= 3:
            if self.simulation_bounds:
                # Use provided simulation bounds
                b_min = self.simulation_bounds.min
                b_max = self.simulation_bounds.max
                if (
                    qpos[0] < b_min[0]
                    or qpos[1] < b_min[1]
                    or qpos[2] < b_min[2]
                    or qpos[0] > b_max[0]
                    or qpos[1] > b_max[1]
                    or qpos[2] > b_max[2]
                ):
                    return SimulationFailureMode.OUT_OF_BOUNDS
            else:
                # Fallback to heuristic: Z < -2.0
                if qpos[2] < -2.0:
                    return SimulationFailureMode.OUT_OF_BOUNDS

        return None

    def check_motor_overload(
        self,
        motor_names: list[str],
        forces: list[float],
        limits: list[float],
        dt: float,
    ) -> bool:
        """Identify motors stalled at their limit."""
        for i, name in enumerate(motor_names):
            limit = limits[i]
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
