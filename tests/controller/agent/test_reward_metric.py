from types import SimpleNamespace

import pytest

from controller.agent.dspy_utils import cad_simulation_metric


def test_metric_script_fails():
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=SimpleNamespace(max_unit_cost=10.0, max_weight=5.0),
    )
    prediction = SimpleNamespace(script_compiled=False)

    score = cad_simulation_metric(gold, prediction)
    # Based on reward_config.yaml for cad_engineer, script_compiles weight is 0.05, min score is 0.02
    assert score == 0.02


def test_metric_cad_fails():
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=SimpleNamespace(max_unit_cost=10.0, max_weight=5.0),
    )
    prediction = SimpleNamespace(script_compiled=True, cad_geometry_valid=False)

    score = cad_simulation_metric(gold, prediction)
    # score = script_compiles.weight = 0.05
    assert score == 0.05


def test_metric_full_success():
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=SimpleNamespace(max_unit_cost=10.0, max_weight=5.0),
    )
    prediction = SimpleNamespace(
        script_compiled=True,
        cad_geometry_valid=True,
        manufacturability_valid=True,
        parts_within_build_zone=True,
        actual_cost=5.0,
        actual_weight=2.0,
        simulation_success=True,
    )

    score = cad_simulation_metric(gold, prediction)
    # Sum of weights: 0.05 + 0.08 + 0.07 + 0.05 + 0.10 + 0.05 + 0.60 = 1.0
    assert score == 1.0


def test_metric_partial_sim():
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=SimpleNamespace(max_unit_cost=10.0, max_weight=5.0),
    )
    prediction = SimpleNamespace(
        script_compiled=True,
        cad_geometry_valid=True,
        manufacturability_valid=True,
        parts_within_build_zone=True,
        actual_cost=5.0,
        actual_weight=2.0,
        simulation_success=False,
        simulation_ran=True,
        min_distance_to_goal=5.0,
        initial_distance=10.0,
    )

    score = cad_simulation_metric(gold, prediction)
    # Prefixes sum: 0.05 + 0.08 + 0.07 + 0.05 + 0.10 + 0.05 = 0.40
    # Sim partial: 0.60 * (1 - 5/10) * 0.4 = 0.60 * 0.5 * 0.4 = 0.12
    # Total: 0.40 + 0.12 = 0.52
    assert score == 0.52


def test_metric_cost_overage():
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=SimpleNamespace(max_unit_cost=10.0, max_weight=5.0),
    )
    prediction = SimpleNamespace(
        script_compiled=True,
        cad_geometry_valid=True,
        manufacturability_valid=True,
        parts_within_build_zone=True,
        actual_cost=15.0,  # 1.5x cap
        actual_weight=2.0,
        simulation_success=True,
    )

    score = cad_simulation_metric(gold, prediction)
    # Cost penalty: max(0, 1 - max(0, 1.5 - 1)) = 0.5
    # Total score: 1.0 - (0.10 * (1 - 0.5)) = 0.95
    assert score == pytest.approx(0.95)
