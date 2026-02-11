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
    assert validate(p.part) == True

    # 2. Overlapping boxes
    with BuildPart() as p1:
        Box(10, 10, 10)
    with BuildPart() as p2:
        Box(10, 10, 10)

    comp = Compound(label="overlapping", children=[p1.part, p2.part])
    print(f"Validating overlapping boxes: {validate(comp)}")

    # We don't have automatic event emission in validate() yet, but we will test it via validate_objectives_yaml
    assert validate(comp) == False


def test_objectives_validation_events():
    from worker.utils.file_validation import validate_objectives_yaml

    clear_emitted_events()

    invalid_yaml = "invalid: { ["
    success, errors = validate_objectives_yaml(invalid_yaml)
    assert success is False
    # YAML parse error doesn't emit LogicFailureEvent yet based on my reading of file_validation.py:48-55
    # only ValidationError does.

    invalid_data = "project_name: 123"  # Should be string, but depends on schema.
    # Let's use something that definitely fails Pydantic validation
    success, errors = validate_objectives_yaml("some_field: unexpected")
    assert success is False
    assert_event_emitted("logic_failure", file_path="objectives.yaml")


def test_plan_validation_events():
    from worker.utils.file_validation import validate_plan_md_structure

    clear_emitted_events()

    invalid_plan = "# Some Heading\nNo required sections."
    success, errors = validate_plan_md_structure(invalid_plan, plan_type="benchmark")
    assert success is False
    assert_event_emitted("lint_failure_docs", file_path="plan.md")


def test_simulation():
    # Valid stable box
    with BuildPart() as p:
        Box(10, 10, 10)

    res = simulate(p.part)
    print(f"Simulation result: {res.success}, {res.summary}")
    assert res.success == True


def test_handover():
    from worker.utils.handover import submit_for_review

    with BuildPart() as p:
        Box(10, 10, 10)

    os.environ["SESSION_ID"] = "test_session"
    plan_path = Path("plan.md")
    todo_path = Path("todo.md")
    try:
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
    assert res == True

    renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    with open(manifest_path) as f:
        import json

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
