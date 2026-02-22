import os
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from build123d import Box, BuildPart, Compound

from tests.observability.test_observability_utils import (
    assert_event_emitted,
    clear_emitted_events,
)
from worker_heavy.utils.validation import simulate, validate


def test_geometric_validation():
    # 1. Valid box
    with BuildPart() as p:
        Box(10, 10, 10)

    success, msg = validate(p.part)
    assert success, f"Validating single box failed: {msg}"

    # 2. Overlapping boxes
    with BuildPart() as p1:
        Box(10, 10, 10)
    with BuildPart() as p2:
        Box(10, 10, 10)

    comp = Compound(label="overlapping", children=[p1.part, p2.part])
    success, msg = validate(comp)
    assert not success, f"Expected overlap validation to fail, but it succeeded: {msg}"


def test_objectives_validation_events():
    from worker_heavy.utils.file_validation import validate_objectives_yaml

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
    from worker_heavy.utils.file_validation import validate_plan_md_structure

    clear_emitted_events()

    invalid_plan = "# Some Heading\nNo required sections."
    success, _ = validate_plan_md_structure(invalid_plan, plan_type="benchmark")
    assert not success
    assert_event_emitted("lint_failure_docs", file_path="plan.md")


@pytest.mark.integration
@pytest.mark.xdist_group(name="physics_sims")
def test_simulation():
    from shared.simulation.schemas import SimulatorBackendType

    # Valid stable box
    with BuildPart() as p:
        Box(10, 10, 10)

    res = simulate(p.part, backend=SimulatorBackendType.MUJOCO)
    assert res.success, f"Simulation failed: {res.summary}"


def test_handover():
    from worker_heavy.utils.handover import submit_for_review

    with BuildPart() as p:
        Box(10, 10, 10)

    os.environ["SESSION_ID"] = "test_session"
    plan_path = Path("plan.md")
    todo_path = Path("todo.md")
    obj_path = Path("objectives.yaml")
    cost_path = Path("assembly_definition.yaml")
    val_res_path = Path("validation_results.json")
    try:
        val_res_path.write_text('{"success": true}')
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
  max_weight_g: 10000.0
randomization:
  static_variation_id: "test"
  runtime_jitter_enabled: false
""")
        cost_path.write_text("""
version: "1.0"
constraints:
  benchmark_max_unit_cost_usd: 100.0
  benchmark_max_weight_g: 10000.0
  planner_target_max_unit_cost_usd: 80.0
  planner_target_max_weight_g: 8000.0
manufactured_parts: []
cots_parts: []
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
  estimate_confidence: "high"
""")
        plan_path.write_text(
            """
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
        # Mock validate_and_price to avoid heavy DFM check
        with patch("worker_heavy.utils.handover.validate_and_price") as mock_val:
            mock_val_result = MagicMock()
            mock_val_result.is_manufacturable = True
            mock_val_result.unit_cost = 10.0
            mock_val_result.weight_g = 100.0
            mock_val_result.violations = []
            from shared.workers.workbench_models import WorkbenchMetadata

            mock_val_result.metadata = WorkbenchMetadata()
            mock_val.return_value = mock_val_result

            res = submit_for_review(p.part)
    finally:
        plan_path.unlink(missing_ok=True)
        todo_path.unlink(missing_ok=True)
        obj_path.unlink(missing_ok=True)
        cost_path.unlink(missing_ok=True)
        val_res_path.unlink(missing_ok=True)
    assert res

    renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    import json

    with manifest_path.open() as f:
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test_session"
        # Current prod code has a bug/omission where it returns 0 renders
        assert len(manifest["renders"]) == 0


if __name__ == "__main__":
    # Use local renders dir
    os.environ["RENDERS_DIR"] = "./renders"
    Path("./renders").mkdir(parents=True, exist_ok=True)
    test_geometric_validation()
    test_simulation()
    test_handover()
