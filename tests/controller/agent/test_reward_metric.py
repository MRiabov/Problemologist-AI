from types import SimpleNamespace
import pytest
from controller.agent.dspy_utils import cad_simulation_metric
from shared.models.schemas import ObjectivesYaml, Constraints, ObjectivesSection, BoundingBox, MovedObject, StaticRandomization

def create_valid_objectives(max_cost=10.0, max_weight=5.0):
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

def test_metric_script_fails():
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=create_valid_objectives(max_cost=10.0, max_weight=5.0),
    )
    prediction = SimpleNamespace(script_compiled=False)

    result = cad_simulation_metric(gold, prediction)
    assert result.score == 0.02

def test_metric_cad_fails():
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=create_valid_objectives(max_cost=10.0, max_weight=5.0),
    )
    prediction = SimpleNamespace(script_compiled=True, cad_geometry_valid=False)

    result = cad_simulation_metric(gold, prediction)
    assert result.score == 0.05

def test_metric_full_success():
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
        simulation_success=True,
    )

    result = cad_simulation_metric(gold, prediction)
    assert result.score == 1.0

def test_metric_partial_sim():
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
        min_distance_to_goal=5.0,
        initial_distance=10.0,
    )

    result = cad_simulation_metric(gold, prediction)
    # Prefixes sum: 0.05 + 0.08 + 0.07 + 0.05 + 0.10 + 0.05 = 0.40
    # Sim partial: 0.60 * (1 - 5/10) * 0.4 = 0.60 * 0.5 * 0.4 = 0.12
    # Total: 0.40 + 0.12 = 0.52
    assert result.score == 0.52

def test_metric_cost_overage():
    gold = SimpleNamespace(
        agent_name="cad_engineer",
        objectives=create_valid_objectives(max_cost=10.0, max_weight=5.0),
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

    result = cad_simulation_metric(gold, prediction)
    # Cost penalty: max(0, 1 - max(0, 1.5 - 1)) = 0.5
    # Total score: 1.0 - (0.10 * (1 - 0.5)) = 0.95
    assert result.score == pytest.approx(0.95)
