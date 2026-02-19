"""
Position-based motor controllers for MuJoCo.

These controllers output TARGET POSITIONS (not torques). They are designed
to be used with MuJoCo's `<position>` actuator type, which internally applies
PD control: torque = kp * (target - pos) - kv * vel

For these to work correctly, the MJCF must use:
  <actuator>
    <position name="servo" joint="hinge" kp="100" kv="10"/>
  </actuator>

NOT <motor> (which expects direct torque input).
"""

from collections.abc import Callable


def waypoint(schedule: list[tuple[float, float]]) -> Callable[[float], float]:
    """
    Returns a controller that outputs target positions at scheduled times.

    Args:
        schedule: List of (time_seconds, target_position_rad) tuples.
                  Must be sorted by time in ascending order.

    Returns:
        A controller function that takes time `t` and returns target position.

    Example:
        # At 0s: go to 0 rad, at 2s: go to π/4, at 5s: go to 0
        ctrl = waypoint([(0, 0), (2, 0.785), (5, 0)])
        # At t=3s, ctrl(3) returns 0.785 (the target set at t=2)
    """
    if not schedule:
        return lambda _: 0.0

    # Sort by time just in case
    sorted_schedule = sorted(schedule, key=lambda x: x[0])

    def controller(t: float) -> float:
        # Find the most recent waypoint at or before time t
        target = sorted_schedule[0][1]
        for time, angle in sorted_schedule:
            if t >= time:
                target = angle
            else:
                break
        return target

    return controller


def hold_position(target: float) -> Callable[[float], float]:
    """
    Returns a controller that holds a fixed target position.

    Args:
        target: Target position in radians (for hinge) or meters (for slide).

    Returns:
        A controller function that always returns the target position.
    """
    return lambda _: float(target)


def oscillate(
    center: float,
    amplitude: float,
    frequency: float = 1.0,
    phase: float = 0.0,
) -> Callable[[float], float]:
    """
    Returns a controller that oscillates the target position sinusoidally.

    position(t) = center + amplitude * sin(2π * frequency * t + phase)

    Args:
        center: Center position (radians or meters).
        amplitude: Oscillation amplitude.
        frequency: Oscillation frequency in Hz.
        phase: Phase offset in radians.

    Returns:
        A controller function for sinusoidal position oscillation.
    """
    import math

    def controller(t: float) -> float:
        return center + amplitude * math.sin(2 * math.pi * frequency * t + phase)

    return controller


def rotate_to(current: float, target: float, kp: float = 1.0) -> float:
    """
    Calculate a control signal (e.g. torque) based on position error.

    Args:
        current: Current position (e.g. from sensor).
        target: Target position.
        kp: Proportional gain.

    Returns:
        Control signal (kp * (target - current)).
    """
    return kp * (target - current)
