import json
import os
from pathlib import Path
from unittest.mock import MagicMock, patch

from build123d import Box, BuildPart


@patch("worker_heavy.utils.handover.validate_and_price")
@patch("worker_heavy.utils.handover.export_step")
def test_submit_for_review(mock_export_step, mock_validate, tmp_path):
    from worker_heavy.utils.handover import submit_for_review

    # Setup
    renders_dir = tmp_path / "renders"
    renders_dir.mkdir()
    os.environ["RENDERS_DIR"] = str(renders_dir)
    os.environ["SESSION_ID"] = "test-session"
    os.environ["TIMESTAMP"] = "2026-02-07"

    mock_component = MagicMock()

    # Mock validate_and_price return value
    mock_val_result = MagicMock()
    mock_val_result.is_manufacturable = True
    mock_val_result.unit_cost = 10.0
    mock_val_result.weight_g = 100.0
    mock_val_result.violations = []
    # Note: production code uses WorkbenchMetadata object, but tests often mock as dict
    # We should ideally use the object to match my fix
    from shared.workers.workbench_models import WorkbenchMetadata

    mock_val_result.metadata = WorkbenchMetadata()
    mock_validate.return_value = mock_val_result

    # Create mandatory files with correct sections
    plan_content = """
## 1. Solution Overview
Test solution
## 2. Parts List
- Part A
## 3. Assembly Strategy
1. Step 1
## 4. Cost & Weight Budget
- $10
## 5. Risk Assessment
- Low
"""
    (tmp_path / "plan.md").write_text(plan_content)
    (tmp_path / "todo.md").write_text("# TODO\n- [x] Done")
    (tmp_path / "validation_results.json").write_text('{"success": true}')

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
  max_weight_g: 10000.0
randomization:
  static_variation_id: "test"
  runtime_jitter_enabled: false
"""
    (tmp_path / "objectives.yaml").write_text(objectives_content)

    # Create valid assembly_definition.yaml in tmp_path
    cost_content = """
version: "1.0"
units:
  length: "mm"
  volume: "mm3"
  mass: "g"
  currency: "USD"
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
"""
    (tmp_path / "assembly_definition.yaml").write_text(cost_content)

    # Execute
    result = submit_for_review(mock_component, cwd=tmp_path)

    # Assert
    assert result is True
    mock_export_step.assert_called_once()

    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    with manifest_path.open() as f:
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test-session"


@patch("worker_heavy.utils.rendering.get_simulation_builder")
@patch("worker_heavy.simulation.factory.get_physics_backend")
def test_prerender_24_views(mock_get_backend, mock_get_builder, tmp_path):
    from worker_heavy.utils.rendering import prerender_24_views

    # Setup
    mock_builder = MagicMock()
    mock_get_builder.return_value = mock_builder
    mock_builder.build_from_assembly.return_value = tmp_path / "scene.xml"
    (tmp_path / "scene.xml").write_text("<mujoco/>")

    mock_backend = MagicMock()
    mock_get_backend.return_value = mock_backend
    # mock_backend.render_camera should return a numpy array (image)
    import numpy as np

    mock_backend.render_camera.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

    with BuildPart() as p:
        Box(10, 10, 10)
    mock_component = p.part

    # Execute
    renders = prerender_24_views(mock_component, output_dir=str(tmp_path))

    # Assert
    assert len(renders) == 24
    assert mock_get_builder.called
    assert mock_get_backend.called
    assert mock_backend.render_camera.call_count == 24

    for render_path in renders:
        assert Path(render_path).parent == tmp_path
