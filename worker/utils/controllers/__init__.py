"""Motor controllers for MuJoCo simulations."""

from .position_based import hold_position, oscillate, waypoint
from .time_based import constant, sinusoidal, square, trapezoidal

# Time-based controllers: output torque/force for <motor> actuators
# Position-based controllers: output target position for <position> actuators
__all__ = [
    "constant",
    "hold_position",
    "oscillate",
    "sinusoidal",
    "square",
    "trapezoidal",
    "waypoint",
]
