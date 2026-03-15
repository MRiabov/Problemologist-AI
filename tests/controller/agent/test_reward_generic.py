from types import SimpleNamespace

import pytest

from controller.agent.dspy_utils import cad_simulation_metric
from shared.enums import AgentName

BENCHMARK_PLAN_CHECKLIST_PASS = {
    "artifacts_complete": "pass",
    "cross_artifact_consistency": "pass",
    "geometry_feasible": "pass",
    "objective_clearance_valid": "pass",
    "randomization_valid": "pass",
    "motion_contract_complete": "pass",
    "benchmark_dof_minimality": "pass",
    "ambiguity_free": "pass",
}


BENCHMARK_EXECUTION_CHECKLIST_PASS = {
    "latest_revision_verified": "pass",
    "validation_success": "pass",
    "simulation_success": "pass",
    "visual_evidence_checked": "pass",
    "dynamic_evidence_checked": "pass",
    "plan_fidelity": "pass",
    "robustness": "pass",
    "constraint_compliance": "pass",
    "benchmark_motion_justified": "pass",
}


def test_metric_benchmark_planner_basic():
    """Test using benchmark_planner milestones from reward_config.yaml"""
    gold = SimpleNamespace(
        agent_name=AgentName.BENCHMARK_PLANNER,
        objectives=SimpleNamespace(
            constraints=SimpleNamespace(max_unit_cost=100.0, max_weight_g=50.0)
        ),
    )
    # Give it all milestones except reviewer_accepted
    prediction = SimpleNamespace(
        plan_artifacts_present=True,
        yaml_schema_valid=True,
        cost_within_cap=True,
        estimated_cost=80.0,
        weight_within_cap=True,
        estimated_weight=40.0,
        geometry_consistent=True,
        cots_ids_valid=True,
        reviewer_accepted=False,
    )

    result = cad_simulation_metric(gold, prediction)
    # Weights for benchmark_planner:
    # present: 0.05
    # schema: 0.10
    # cost: 0.10
    # weight: 0.05
    # geometry: 0.10
    # cots: 0.05
    # Sum: 0.45
    assert result.score == pytest.approx(0.45)


def test_metric_benchmark_planner_cost_overage():
    gold = SimpleNamespace(
        agent_name=AgentName.BENCHMARK_PLANNER,
        objectives=SimpleNamespace(
            constraints=SimpleNamespace(max_unit_cost=100.0, max_weight_g=50.0)
        ),
    )
    # 20% over cost cap -> penalty 0.8
    prediction = SimpleNamespace(
        plan_artifacts_present=True,
        yaml_schema_valid=True,
        cost_within_cap=True,
        estimated_cost=120.0,
        weight_within_cap=False,
        geometry_consistent=False,
        cots_ids_valid=False,
        reviewer_accepted=False,
    )

    result = cad_simulation_metric(gold, prediction)
    # present: 0.05
    # schema: 0.10
    # cost: 0.10 * 0.8 = 0.08
    assert result.score == pytest.approx(0.05 + 0.10 + 0.08)


def test_metric_cad_engineer_failure_formula():
    """Verify that cad_engineer simulation failure formula is used."""
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
        min_distance_to_goal=7.0,
        initial_distance=10.0,
    )
    # Base: 0.05(script) + 0.08(cad) + 0.07(mfg) + 0.05(zone) + 0.10(cost) + 0.05(weight) = 0.40
    # Failure formula: 0.45 * (1 - 7/10) * 0.4 = 0.45 * 0.3 * 0.4 = 0.054
    # Total: 0.454
    result = cad_simulation_metric(gold, prediction)
    assert result.score == pytest.approx(0.454)


def test_metric_benchmark_plan_reviewer_full_success():
    """Benchmark plan reviewer should score full credit on complete, correct review."""
    gold = SimpleNamespace(
        agent_name=AgentName.BENCHMARK_PLAN_REVIEWER,
        checklist=BENCHMARK_PLAN_CHECKLIST_PASS,
    )
    prediction = SimpleNamespace(
        review_artifacts_complete=True,
        decision_correct=True,
        review_actionable=True,
        checklist=BENCHMARK_PLAN_CHECKLIST_PASS,
    )

    result = cad_simulation_metric(gold, prediction)
    assert result.score == pytest.approx(1.0)


def test_metric_reviewer_generic_partial_judge_score():
    """A mismatched reviewer checklist item should remove only that item's weight."""
    gold = SimpleNamespace(
        agent_name=AgentName.BENCHMARK_REVIEWER,
        checklist=BENCHMARK_EXECUTION_CHECKLIST_PASS,
    )
    prediction_checklist = dict(BENCHMARK_EXECUTION_CHECKLIST_PASS)
    prediction_checklist["robustness"] = "fail"
    prediction = SimpleNamespace(
        review_artifacts_complete=True,
        decision_correct=True,
        review_actionable=True,
        checklist=prediction_checklist,
    )
    # Full score is 1.0. Robustness has checklist weight 0.04, so one mismatch yields 0.96.
    result = cad_simulation_metric(gold, prediction)
    assert result.score == pytest.approx(0.96)
