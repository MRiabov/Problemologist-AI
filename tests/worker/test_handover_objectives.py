import json
import os
from unittest.mock import MagicMock, patch

import pytest
from build123d import Box

from worker.utils.handover import submit_for_review


@pytest.fixture
def temp_dir(tmp_path):
    renders_dir = tmp_path / "renders"
    renders_dir.mkdir()
    return tmp_path


def test_submit_for_review_includes_objectives(temp_dir):
    # Create a dummy component
    from build123d import BuildPart

    with BuildPart() as b:
        Box(1, 1, 1)
    component = b.part

    # Create valid objectives.yaml in CWD (mocked)
    objectives_content = """
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
"""
    objectives_file = temp_dir / "objectives.yaml"
    objectives_file.write_text(objectives_content)

    # Create valid preliminary_cost_estimation.yaml
    cost_content = """
version: "1.0"
units:
  length: "mm"
  volume: "mm3"
  mass: "g"
  currency: "USD"
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
"""
    cost_file = temp_dir / "preliminary_cost_estimation.yaml"
    cost_file.write_text(cost_content)

    # Create plan.md
    plan_file = temp_dir / "plan.md"
    plan_file.write_text("""# PLAN

## 1. Solution Overview
Overview content.

## 2. Parts List
- Part A

## 3. Assembly Strategy
1. Assemble part A.
2. Assemble part B.

## 4. Cost & Weight Budget
- Cost: $10.
- Weight: 10g.

## 5. Risk Assessment
- Risk: Breaking.
- Mitigation: Be careful.
""")

    # Create todo.md
    todo_file = temp_dir / "todo.md"
    todo_file.write_text("# TODO\n\n- [x] Step 1\n")

    # Create validation_results.json
    validation_file = temp_dir / "validation_results.json"
    validation_file.write_text(json.dumps({"success": True, "message": "OK"}))

    # We need to run in the temp_dir context
    os.chdir(temp_dir)

    with (
        patch("worker.utils.handover.prerender_24_views", return_value=["img1.png"]) as mock_render,
        patch("worker.utils.handover.export_step"),
        patch.dict(
            os.environ,
            {"RENDERS_DIR": str(temp_dir / "renders"), "SESSION_ID": "test_session"},
        ),
        # Mock validate_and_price to avoid actual build123d analysis overhead/errors
        patch("worker.utils.handover.validate_and_price") as mock_validate
    ):
        # Configure mock validation result
        mock_result = MagicMock()
        mock_result.is_manufacturable = True
        mock_result.unit_cost = 50.0
        mock_result.metadata = {"weight_kg": 1.0}
        mock_validate.return_value = mock_result

        success = submit_for_review(component)
        assert success is True

        # Check if objectives.yaml was copied to renders
        target_obj = temp_dir / "renders" / "objectives.yaml"
        assert target_obj.exists()
        assert target_obj.read_text() == objectives_content

        # Check manifest
        manifest_path = temp_dir / "renders" / "review_manifest.json"
        assert manifest_path.exists()
        with manifest_path.open() as f:
            manifest = json.load(f)

        assert manifest["objectives_path"] == str(target_obj)
        assert manifest["session_id"] == "test_session"
        assert manifest["renders"] == ["img1.png"]

        # Verify prerender was called
        mock_render.assert_called_once()
