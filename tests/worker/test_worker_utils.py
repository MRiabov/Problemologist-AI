import json
import os
from pathlib import Path
from unittest.mock import MagicMock, patch
import numpy as np

# We need to mock build123d before importing modules that use it
# or use patch on the module level.


@patch("worker.utils.handover.validate_and_price")
@patch("worker.utils.handover.prerender_24_views")
@patch("worker.utils.handover.export_step")
def test_submit_for_review(mock_export_step, mock_prerender, mock_v_and_p, tmp_path):
    from worker.utils.handover import submit_for_review

    # Setup
    renders_dir = tmp_path / "renders"
    renders_dir.mkdir()
    os.environ["RENDERS_DIR"] = str(renders_dir)
    os.environ["SESSION_ID"] = "test-session"
    os.environ["TIMESTAMP"] = "2026-02-07"

    mock_component = MagicMock()
    mock_component.metadata = {}
    mock_prerender.return_value = ["/path/to/render1.png"]
    from worker.workbenches.models import WorkbenchResult
    mock_v_and_p.return_value = WorkbenchResult(is_manufacturable=True, unit_cost=10.0, weight_g=100.0)

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
    # Create mandatory plan.md and todo.md
    (tmp_path / "plan.md").write_text("## 1. Solution Overview\n## 2. Parts List\n- Part 1\n## 3. Assembly Strategy\n1. Step 1\n## 4. Cost & Weight Budget\n- Budget 1\n## 5. Risk Assessment\n- Risk 1")
    (tmp_path / "todo.md").write_text("- [x] task completed")
    (tmp_path / "objectives.yaml").write_text(objectives_content)
    (tmp_path / "validation_results.json").write_text(json.dumps({"is_valid": True}))

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
    mock_prerender.assert_called_once()
    mock_export_step.assert_called_once()

    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    with manifest_path.open() as f:
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test-session"
        assert "/path/to/render1.png" in manifest["renders"]


@patch("worker.utils.rendering.Image")
@patch("worker.utils.rendering.mujoco")
@patch("worker.utils.rendering.get_simulation_builder")
def test_prerender_24_views(
    mock_builder_factory, mock_mujoco, mock_image, tmp_path
):
    from worker.utils.rendering import prerender_24_views

    # Setup
    mock_builder = MagicMock()
    mock_builder_factory.return_value = mock_builder
    mock_builder.build_from_assembly.return_value = tmp_path / "scene.xml"
    (tmp_path / "scene.xml").write_text("<mujoco/>")

    mock_renderer = MagicMock()
    mock_mujoco.Renderer.return_value = mock_renderer
    mock_renderer.render.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

    mock_component = MagicMock()
    mock_component.bounding_box.return_value.size.X = 10
    mock_component.bounding_box.return_value.size.Y = 10
    mock_component.bounding_box.return_value.size.Z = 10
    mock_component.bounding_box.return_value.min.X = 0
    mock_component.bounding_box.return_value.min.Y = 0
    mock_component.bounding_box.return_value.min.Z = 0
    mock_component.bounding_box.return_value.max.X = 10
    mock_component.bounding_box.return_value.max.Y = 10
    mock_component.bounding_box.return_value.max.Z = 10

    # Execute
    renders = prerender_24_views(mock_component, output_dir=str(tmp_path))

    # Assert
    assert len(renders) == 24
    assert mock_builder.build_from_assembly.called
    assert mock_renderer.render.call_count == 24

    for render_path in renders:
        assert Path(render_path).parent == tmp_path
