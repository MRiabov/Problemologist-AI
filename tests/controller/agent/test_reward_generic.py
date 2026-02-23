from types import SimpleNamespace
import pytest
from controller.agent.dspy_utils import cad_simulation_metric
from shared.models.schemas import ObjectivesYaml, Constraints, ObjectivesSection, BoundingBox, MovedObject, StaticRandomization

def create_valid_objectives(max_cost=100.0, max_weight=50.0):
    return ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10, 10, 10), max=(12, 12, 12)),
            build_zone=BoundingBox(min=(0, 0, 0), max=(100, 100, 100)),
        ),
        simulation_bounds=BoundingBox(min=(-100, -100, -100), max=(100, 100, 100)),
        moved_object=MovedObject(
            label="test",
            shape="sphere",
            start_position=(0, 0, 0),
            runtime_jitter=(0, 0, 0),
            static_randomization=StaticRandomization()
        ),
        constraints=Constraints(max_unit_cost=max_cost, max_weight_g=max_weight)
    )

def test_metric_benchmark_planner_basic():
    """Test using benchmark_planner milestones from reward_config.yaml"""
    gold = SimpleNamespace(
        agent_name="benchmark_planner",
        objectives=create_valid_objectives(max_cost=100.0, max_weight=50.0),
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
    assert result.score == pytest.approx(0.45)


def test_metric_benchmark_planner_cost_overage():
    gold = SimpleNamespace(
        agent_name="benchmark_planner",
        objectives=create_valid_objectives(max_cost=100.0, max_weight=50.0),
    )
    # 20% over cost cap -> penalty 0.8
    prediction = SimpleNamespace(
        plan_artifacts_present=True,
        yaml_schema_valid=True,
        cost_within_cap=True,
        estimated_cost=120.0,
        estimated_weight=40.0, # Added this to avoid formula fail
        weight_within_cap=False,
        geometry_consistent=False,
        cots_ids_valid=False,
        reviewer_accepted=False,
    )

    result = cad_simulation_metric(gold, prediction)
    # present: 0.05
    # schema: 0.10
    # cost: 0.10 * 0.8 = 0.08
    # weight: 0.05 (formula evaluates to 1.0 even if weight_within_cap is False in prediction)
    assert result.score == pytest.approx(0.05 + 0.10 + 0.08 + 0.05)


def test_metric_cad_engineer_failure_formula():
    """Verify that cad_engineer simulation failure formula is used."""
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=create_valid_objectives(max_cost=10.0, max_weight=5.0),
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
    # Failure formula: 0.60 * (1 - 7/10) * 0.4 = 0.60 * 0.3 * 0.4 = 0.072
    # Total: 0.472
    result = cad_simulation_metric(gold, prediction)
    assert result.score == pytest.approx(0.472)


def test_metric_reviewer_generic_binary():
    """Test benchmark_reviewer with binary milestones."""
    gold = SimpleNamespace(agent_name="benchmark_reviewer")
    prediction = SimpleNamespace(
        review_artifacts_complete=True, decision_correct=True, review_actionable=True
    )
    # reviewer weights: 0.10, 0.60, 0.30 -> Sum 1.0
    result = cad_simulation_metric(gold, prediction)
    assert result.score == pytest.approx(1.0)
