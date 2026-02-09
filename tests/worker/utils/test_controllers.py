import math

from worker.utils.controllers.position_based import rotate_to
from worker.utils.controllers.time_based import (
    constant,
    sinusoidal,
    square,
    trapezoidal,
)


def test_constant_controller():
    c = constant(5.0)
    assert c(0) == 5.0
    assert c(10.5) == 5.0


def test_sinusoidal_controller():
    # Frequency 1Hz, Power 10, t=0.25 -> sin(pi/2) = 1.0 -> 10.0
    val = sinusoidal(0.25, 10.0, frequency=1.0)
    assert math.isclose(val, 10.0)

    # t=0.5 -> sin(pi) = 0
    val = sinusoidal(0.5, 10.0, frequency=1.0)
    assert math.isclose(val, 0.0, abs_tol=1e-7)


def test_square_controller():
    # On between 1-2 and 4-5
    intervals = [(1.0, 2.0), (4.0, 5.0)]
    assert square(0.5, intervals, 10.0) == 0.0
    assert square(1.5, intervals, 10.0) == 10.0
    assert square(3.0, intervals, 10.0) == 0.0
    assert square(4.5, intervals, 10.0) == 10.0


def test_trapezoidal_controller():
    # On between 1-3, power 10, ramp 0.5
    intervals = [(1.0, 3.0)]
    # Halfway through ramp up (1.25) -> 5.0
    assert math.isclose(trapezoidal(1.25, intervals, 10.0, 0.5), 5.0)
    # Full power at 2.0
    assert trapezoidal(2.0, intervals, 10.0, 0.5) == 10.0
    # Halfway through ramp down (2.75) -> 5.0
    assert math.isclose(trapezoidal(2.75, intervals, 10.0, 0.5), 5.0)
    # Off at 0.5
    assert trapezoidal(0.5, intervals, 10.0, 0.5) == 0.0


def test_rotate_to_controller():
    # kp=2, target=1.0, current=0.5 -> error=0.5 -> 1.0
    assert rotate_to(0.5, 1.0, kp=2.0) == 1.0
    # kp=1, target=0, current=5.0 -> error=-5.0 -> -5.0
    assert rotate_to(5.0, 0.0, kp=1.0) == -5.0
