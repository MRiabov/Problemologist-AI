import numpy as np
import pytest
from worker_heavy.simulation.evaluator import SuccessEvaluator
from shared.enums import FailureReason

def test_motor_overload_logic():
    evaluator = SuccessEvaluator(max_simulation_time=10.0, motor_overload_threshold=2.0)

    motor_names = ["m1"]
    limits = [1.0]
    dt = 0.1

    # Not overloaded initially
    assert not evaluator.check_motor_overload(motor_names, [0.5], limits, dt)
    assert evaluator.motor_overload_timer["m1"] == 0

    # Saturated but below threshold
    for _ in range(10): # 1.0s
        assert not evaluator.check_motor_overload(motor_names, [0.95], limits, dt)

    assert evaluator.motor_overload_timer["m1"] == pytest.approx(1.0)

    # Continue until threshold
    for _ in range(10): # another 1.0s -> 2.0s total
        overloaded = evaluator.check_motor_overload(motor_names, [0.95], limits, dt)

    assert overloaded
    assert evaluator.motor_overload_timer["m1"] >= 2.0

def test_motor_overload_reset_logic():
    evaluator = SuccessEvaluator(max_simulation_time=10.0, motor_overload_threshold=2.0)
    motor_names = ["m1"]
    limits = [1.0]
    dt = 0.5

    # Saturate for 1.5s
    evaluator.check_motor_overload(motor_names, [1.0], limits, dt)
    evaluator.check_motor_overload(motor_names, [1.0], limits, dt)
    evaluator.check_motor_overload(motor_names, [1.0], limits, dt)
    assert evaluator.motor_overload_timer["m1"] == 1.5

    # Drop below limit -> should reset
    evaluator.check_motor_overload(motor_names, [0.1], limits, dt)
    assert evaluator.motor_overload_timer["m1"] == 0

def test_out_of_bounds_logic():
    from shared.models.schemas import BoundingBox
    bounds = BoundingBox(min=[-10, -10, -10], max=[10, 10, 10])
    evaluator = SuccessEvaluator(max_simulation_time=10.0, simulation_bounds=bounds)

    # Inside
    assert evaluator.check_failure(1.0, np.array([0, 0, 0]), np.array([0, 0, 0])) is None

    # Outside X
    assert evaluator.check_failure(1.0, np.array([11, 0, 0]), np.array([0, 0, 0])) == FailureReason.OUT_OF_BOUNDS

    # Outside Z (below)
    assert evaluator.check_failure(1.0, np.array([0, 0, -11]), np.array([0, 0, 0])) == FailureReason.OUT_OF_BOUNDS

def test_physics_instability_logic():
    evaluator = SuccessEvaluator(max_simulation_time=10.0)

    # NaN pos
    assert evaluator.check_failure(1.0, np.array([np.nan, 0, 0]), np.array([0, 0, 0])) == FailureReason.PHYSICS_INSTABILITY

    # Extreme pos
    assert evaluator.check_failure(1.0, np.array([10001, 0, 0]), np.array([0, 0, 0])) == FailureReason.PHYSICS_INSTABILITY
