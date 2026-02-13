import json
import os
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from worker.workbenches.models import WorkbenchResult

# We need to mock build123d before importing modules that use it
# or use patch on the module level.


@patch("worker.utils.handover.validate_and_price")
@patch("worker.utils.handover.export_step")
def test_submit_for_review(mock_export_step, mock_validate_and_price, tmp_path):
    from worker.utils.handover import submit_for_review

    # Setup
    renders_dir = tmp_path / "renders"
    renders_dir.mkdir()
    os.environ["RENDERS_DIR"] = str(renders_dir)
    os.environ["SESSION_ID"] = "test-session"
    os.environ["TIMESTAMP"] = "2026-02-07"

    mock_component = MagicMock()

    # Mock validation result
    mock_result = WorkbenchResult(is_manufacturable=True, unit_cost=10.0, violations=[], metadata={})
    mock_validate_and_price.return_value = mock_result

    # Create valid objectives.yaml in tmp_path
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
    (tmp_path / "objectives.yaml").write_text(objectives_content)

    # Create valid preliminary_cost_estimation.yaml in tmp_path
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
    (tmp_path / "preliminary_cost_estimation.yaml").write_text(cost_content)

    # Create plan.md with valid structure
    plan_content = """# Plan

## 1. Solution Overview
Overview.

## 2. Parts List
- Part A
- Part B

## 3. Assembly Strategy
1. Step 1
2. Step 2

## 4. Cost & Weight Budget
- Cost: $10
- Weight: 10g

## 5. Risk Assessment
- Risk 1
"""
    (tmp_path / "plan.md").write_text(plan_content)

    # Create todo.md with completed items
    todo_content = """# TODO

- [x] Step 1
- [x] Step 2
"""
    (tmp_path / "todo.md").write_text(todo_content)

    # Create validation_results.json
    validation_content = json.dumps({"success": True})
    (tmp_path / "validation_results.json").write_text(validation_content)

    # Change to tmp_path so submit_for_review finds the file
    old_cwd = Path.cwd()
    os.chdir(tmp_path)
    try:
        # Execute
        result = submit_for_review(mock_component)
    finally:
        os.chdir(old_cwd)

    # Assert
    assert result is True
    mock_export_step.assert_called_once()
    mock_validate_and_price.assert_called()

    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    with manifest_path.open() as f:
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test-session"
        assert manifest["renders"] == []


@patch("worker.utils.rendering.SimulationBuilder")
@patch("worker.utils.rendering.mujoco")
def test_prerender_24_views(
    mock_mujoco, mock_sim_builder, tmp_path
):
    from worker.utils.rendering import prerender_24_views

    # Setup
    mock_builder = MagicMock()
    mock_sim_builder.return_value = mock_builder
    mock_builder.build_from_assembly.return_value = Path("scene.xml")

    mock_model = MagicMock()
    mock_mujoco.MjModel.from_xml_path.return_value = mock_model
    mock_data = MagicMock()
    mock_mujoco.MjData.return_value = mock_data

    mock_renderer = MagicMock()
    mock_mujoco.Renderer.return_value = mock_renderer
    # mock_renderer.render returns a numpy array (image)
    mock_renderer.render.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

    mock_component = MagicMock()
    # Mock bounding box
    mock_bbox = MagicMock()
    mock_bbox.min.X = 0; mock_bbox.min.Y = 0; mock_bbox.min.Z = 0
    mock_bbox.max.X = 10; mock_bbox.max.Y = 10; mock_bbox.max.Z = 10
    mock_bbox.size.X = 10; mock_bbox.size.Y = 10; mock_bbox.size.Z = 10
    mock_component.bounding_box.return_value = mock_bbox

    # Execute
    renders = prerender_24_views(mock_component, output_dir=str(tmp_path))

    # Assert
    assert len(renders) == 24
    mock_builder.build_from_assembly.assert_called_once_with(mock_component)

    for render_path in renders:
        assert Path(render_path).parent == tmp_path
