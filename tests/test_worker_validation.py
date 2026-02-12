import os
from pathlib import Path

from build123d import Box, BuildPart, Compound

from tests.observability.test_observability_utils import (
    assert_event_emitted,
    clear_emitted_events,
)
from worker.utils.validation import simulate, validate


def test_geometric_validation():
    # 1. Valid box
    with BuildPart() as p:
        Box(10, 10, 10)

    print(f"Validating single box: {validate(p.part)}")
    assert validate(p.part)[0]

    # 2. Overlapping boxes
    with BuildPart() as p1:
        Box(10, 10, 10)
    with BuildPart() as p2:
        Box(10, 10, 10)

    comp = Compound(label="overlapping", children=[p1.part, p2.part])
    print(f"Validating overlapping boxes: {validate(comp)}")

    # We don't have automatic event emission in validate() yet, but
    # we will test it via validate_objectives_yaml
    assert not validate(comp)[0]


def test_objectives_validation_events():
    from worker.utils.file_validation import validate_objectives_yaml

    clear_emitted_events()

    invalid_yaml = "invalid: { ["
    success, _ = validate_objectives_yaml(invalid_yaml)
    assert success is False
    # YAML parse error doesn't emit LogicFailureEvent yet based on my reading of file_validation.py:48-55
    # only ValidationError does.

    # Let's use something that definitely fails Pydantic validation
    success, _ = validate_objectives_yaml("some_field: unexpected")
    assert not success
    assert_event_emitted("logic_failure", file_path="objectives.yaml")


def test_plan_validation_events():
    from worker.utils.file_validation import validate_plan_md_structure

    clear_emitted_events()

    invalid_plan = "# Some Heading\nNo required sections."
    success, _ = validate_plan_md_structure(invalid_plan, plan_type="benchmark")
    assert not success
    assert_event_emitted("lint_failure_docs", file_path="plan.md")


def test_simulation():
    # Valid stable box
    with BuildPart() as p:
        Box(10, 10, 10)

    res = simulate(p.part)
    print(f"Simulation result: {res.success}, {res.summary}")
    assert res.success


def test_handover():
    from worker.utils.handover import submit_for_review

    with BuildPart() as p:
        Box(10, 10, 10)

    os.environ["SESSION_ID"] = "test_session"
    plan_path = Path("plan.md")
    todo_path = Path("todo.md")
    obj_path = Path("objectives.yaml")
    cost_path = Path("preliminary_cost_estimation.yaml")
    try:
        obj_path.write_text("""
objectives:
  goal_zone:
    min: [0, 0, 0]
    max: [10, 10, 10]
  build_zone:
    min: [0, 0, 0]
    max: [100, 100, 100]
  forbid_zones: []
simulation_bounds:
  min: [-100, -100, 0]
  max: [100, 100, 100]
moved_object:
  label: "test_ball"
  shape: "sphere"
  start_position: [5, 5, 5]
  runtime_jitter: [0, 0, 0]
constraints:
  max_unit_cost: 100.0
  max_weight: 10.0
randomization:
  static_variation_id: "test"
  runtime_jitter_enabled: false
""")
        cost_path.write_text("""
version: "1.0"
constraints:
  benchmark_max_unit_cost_usd: 100.0
  benchmark_max_weight_kg: 10.0
  planner_target_max_unit_cost_usd: 80.0
  planner_target_max_weight_kg: 8.0
manufactured_parts: []
cots_parts: []
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
  estimate_confidence: "high"
""")
        plan_path.write_text(
            """# Engineering Plan

## 1. Solution Overview
Simple test plan

## 2. Parts List
- Part A

## 3. Assembly Strategy
1. Assemble Part A

## 4. Cost & Weight Budget
- Part A: $1.00

## 5. Risk Assessment
- Low risk
""",
            encoding="utf-8",
        )
        todo_path.write_text(
            """# TODO List

- [x] Read objectives.yaml
- [-] Optional task
""",
            encoding="utf-8",
        )
        res = submit_for_review(p.part)
    finally:
        plan_path.unlink(missing_ok=True)
        todo_path.unlink(missing_ok=True)
        obj_path.unlink(missing_ok=True)
        cost_path.unlink(missing_ok=True)
    assert res

    renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    import json

    with manifest_path.open() as f:
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test_session"
        assert len(manifest["renders"]) == 24


if __name__ == "__main__":
    # Use local renders dir
    os.environ["RENDERS_DIR"] = "./renders"
    Path("./renders").mkdir(parents=True, exist_ok=True)
    test_geometric_validation()
    test_simulation()
    test_handover()
