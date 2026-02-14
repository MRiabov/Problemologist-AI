import json
import os
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

# We need to mock build123d before importing modules that use it
# or use patch on the module level.


@patch("worker.utils.handover.prerender_24_views")
@patch("worker.utils.handover.export_step")
def test_submit_for_review(mock_export_step, mock_prerender, tmp_path):
    from worker.utils.handover import submit_for_review
    from build123d import Box

    # Setup
    renders_dir = tmp_path / "renders"
    renders_dir.mkdir()
    os.environ["RENDERS_DIR"] = str(renders_dir)
    os.environ["SESSION_ID"] = "test-session"
    os.environ["TIMESTAMP"] = "2026-02-07"

    # Use a real build123d object to satisfy beartype
    mock_component = Box(10, 10, 10).translate((5, 5, 5))
    mock_component.metadata = {"manufacturing_method": "cnc"}

    mock_prerender.return_value = ["/path/to/render1.png"]

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
  max_unit_cost: 200.0
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
    plan_content = """
# PLAN
## 1. Solution Overview
Something.
## 2. Parts List
- Part A
## 3. Assembly Strategy
1. Step 1
## 4. Cost & Weight Budget
- $10
## 5. Risk Assessment
- High risk
"""
    (tmp_path / "plan.md").write_text(plan_content)
    (tmp_path / "todo.md").write_text("- [x] Done")
    (tmp_path / "validation_results.json").write_text('{"success": true}')

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
    mock_prerender.assert_called_once_with(mock_component, output_dir=str(renders_dir))
    mock_export_step.assert_called_once()

    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    with manifest_path.open() as f:
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test-session"
        assert "/path/to/render1.png" in manifest["renders"]


@patch("worker.utils.rendering.get_simulation_builder")
@patch("worker.utils.rendering.mujoco")
def test_prerender_24_views(mock_mujoco, mock_builder_factory, tmp_path):
    from worker.utils.rendering import prerender_24_views

    # Setup
    mock_builder = MagicMock()
    mock_builder_factory.return_value = mock_builder
    mock_component = MagicMock()
    # Fix BoundingBox mock to return real numbers to avoid max() failure
    mock_bbox = MagicMock()
    mock_bbox.min.X = 0
    mock_bbox.min.Y = 0
    mock_bbox.min.Z = 0
    mock_bbox.max.X = 1
    mock_bbox.max.Y = 1
    mock_bbox.max.Z = 1
    mock_bbox.size.X = 1
    mock_bbox.size.Y = 1
    mock_bbox.size.Z = 1
    mock_component.bounding_box.return_value = mock_bbox

    mock_mujoco.Renderer.return_value.render.return_value = np.zeros(
        (480, 640, 3), dtype=np.uint8
    )

    # Execute
    renders = prerender_24_views(mock_component, output_dir=str(tmp_path))

    # Assert
    assert len(renders) == 24
    assert mock_mujoco.Renderer.called
    assert mock_mujoco.mj_step.called

    for render_path in renders:
        assert Path(render_path).parent == tmp_path
