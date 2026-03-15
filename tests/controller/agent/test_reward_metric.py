from types import SimpleNamespace

import pytest

from controller.agent.dspy_utils import cad_simulation_metric
from shared.enums import AgentName

ENGINEER_EXECUTION_CHECKLIST_PASS = {
    "latest_revision_verified": "pass",
    "validation_success": "pass",
    "simulation_success": "pass",
    "visual_evidence_checked": "pass",
    "dynamic_evidence_checked": "pass",
    "plan_fidelity": "pass",
    "robustness": "pass",
    "cost_weight_compliance": "pass",
    "manufacturability_compliance": "pass",
    "dof_deviation_justified": "pass",
}


def test_metric_script_fails():
    gold = SimpleNamespace(
        agent_name=AgentName.ENGINEER_CODER,
        objectives=SimpleNamespace(
            constraints=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0)
        ),
    )
    prediction = SimpleNamespace(script_compiled=False)

    result = cad_simulation_metric(gold, prediction)
    # Based on reward_config.yaml for cad_engineer, script_compiles weight is 0.05, min score is 0.02
    assert result.score == 0.02


def test_metric_cad_fails():
    gold = SimpleNamespace(
        agent_name=AgentName.ENGINEER_CODER,
        objectives=SimpleNamespace(
            constraints=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0)
        ),
    )
    prediction = SimpleNamespace(script_compiled=True, cad_geometry_valid=False)

    result = cad_simulation_metric(gold, prediction)
    # score = script_compiles.weight = 0.05
    assert result.score == 0.05


def test_metric_full_success():
    gold = SimpleNamespace(
        agent_name=AgentName.ENGINEER_CODER,
        objectives=SimpleNamespace(
            constraints=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0)
        ),
    )
    prediction = SimpleNamespace(
        script_compiled=True,
        cad_geometry_valid=True,
        manufacturability_valid=True,
        parts_within_build_zone=True,
        actual_cost=5.0,
        actual_weight=2.0,
        simulation_success=True,
        reviewer_accepted=True,
        feedback_response_score=1.0,
        bad_feedback_resistance_score=1.0,
        checklist=ENGINEER_EXECUTION_CHECKLIST_PASS,
    )

    result = cad_simulation_metric(gold, prediction)
    # Sum of weights: 0.40 hard checks + 0.45 simulation + 0.15 judge = 1.0
    assert result.score == 1.0


def test_metric_partial_sim():
    gold = SimpleNamespace(
        agent_name=AgentName.ENGINEER_CODER,
        objectives=SimpleNamespace(
            constraints=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0)
        ),
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

    result = cad_simulation_metric(gold, prediction)
    # Prefixes sum: 0.05 + 0.08 + 0.07 + 0.05 + 0.10 + 0.05 = 0.40
    # Sim partial: 0.45 * (1 - 5/10) * 0.4 = 0.45 * 0.5 * 0.4 = 0.09
    # Total: 0.40 + 0.09 = 0.49
    assert result.score == pytest.approx(0.49)


def test_metric_cost_overage():
    gold = SimpleNamespace(
        agent_name=AgentName.ENGINEER_CODER,
        objectives=SimpleNamespace(
            constraints=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0)
        ),
    )
    prediction = SimpleNamespace(
        script_compiled=True,
        cad_geometry_valid=True,
        manufacturability_valid=True,
        parts_within_build_zone=True,
        actual_cost=15.0,  # 1.5x cap
        actual_weight=2.0,
        simulation_success=True,
        reviewer_accepted=True,
        feedback_response_score=1.0,
        bad_feedback_resistance_score=1.0,
        checklist=ENGINEER_EXECUTION_CHECKLIST_PASS,
    )

    result = cad_simulation_metric(gold, prediction)
    # Cost penalty: max(0, 1 - max(0, 1.5 - 1)) = 0.5
    # Total score: 1.0 - (0.10 * (1 - 0.5)) = 0.95
    assert result.score == pytest.approx(0.95)
