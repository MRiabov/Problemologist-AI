import math
from collections.abc import Callable


def constant(power: float) -> Callable[[float], float]:
    """
    Returns a constant power output regardless of time.
    """
    return lambda _: float(power)


def sinusoidal(
    t: float, power: float, frequency: float = 1.0, phase: float = 0.0
) -> float:
    """
    Returns a sinusoidal power output based on time.
    Formula: power * sin(2 * pi * frequency * t + phase)
    """
    return power * math.sin(2 * math.pi * frequency * t + phase)


def square(t: float, time_on_off: list[tuple[float, float]], power: float) -> float:
    """
    Returns power if t is within any of the (start, end) intervals in time_on_off.
    Otherwise returns 0.0.
    """
    for start, end in time_on_off:
        if start <= t <= end:
            return float(power)
    return 0.0


def trapezoidal(
    t: float, time_on_off: list[tuple[float, float]], power: float, ramp_up_time: float
) -> float:
    """
    Returns power with smooth ramp up and ramp down.
    Specifically, it's a 'trapezoidal' function in signals.
    """
    for start, end in time_on_off:
        if start <= t <= end:
            # Check ramp up
            if t < start + ramp_up_time:
                return power * (t - start) / ramp_up_time
            # Check ramp down
            if t > end - ramp_up_time:
                return power * (end - t) / ramp_up_time
            return float(power)
    return 0.0
