import json
import os
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# We need to mock build123d before importing modules that use it
# or use patch on the module level.


@patch("worker.utils.handover.prerender_24_views")
@patch("worker.utils.handover.export_step")
def test_submit_for_review(mock_export_step, mock_prerender, tmp_path):
    from worker.utils.handover import submit_for_review

    # Setup
    renders_dir = tmp_path / "renders"
    renders_dir.mkdir()
    os.environ["RENDERS_DIR"] = str(renders_dir)
    os.environ["SESSION_ID"] = "test-session"
    os.environ["TIMESTAMP"] = "2026-02-07"

    mock_component = MagicMock()
    # Mock build123d.Compound attributes/methods that are accessed
    mock_component.area = 100.0
    mock_component.volume = 1000.0
    mock_component.children = []
    mock_component.bounding_box.return_value.min.X = 0
    mock_component.bounding_box.return_value.min.Y = 0
    mock_component.bounding_box.return_value.min.Z = 0
    mock_component.bounding_box.return_value.max.X = 10
    mock_component.bounding_box.return_value.max.Y = 10
    mock_component.bounding_box.return_value.max.Z = 10
    mock_component.is_valid = True

    # Mock solids() for 3DP/IM analysis
    mock_solid = MagicMock()
    mock_solid.is_closed = True
    mock_component.solids.return_value = [mock_solid]

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
  max_unit_cost: 100.0
  max_weight: 10.0
randomization:
  static_variation_id: "test"
  runtime_jitter_enabled: false
"""
    (tmp_path / "objectives.yaml").write_text(objectives_content)

    # Create valid plan.md
    plan_content = """# Plan
## 1. Solution Overview
Solution...
## 2. Parts List
Parts...
## 3. Assembly Strategy
Strategy...
## 4. Cost & Weight Budget
Budget...
## 5. Risk Assessment
Risks...
"""
    (tmp_path / "plan.md").write_text(plan_content)

    # Create valid todo.md
    (tmp_path / "todo.md").write_text("- [x] Task 1")

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
    (tmp_path / "assembly_definition.yaml").write_text(cost_content)

    # Validation results mock
    (tmp_path / "validation_results.json").write_text("{}")

    # Change to tmp_path so submit_for_review finds the file
    old_cwd = Path.cwd()
    os.chdir(tmp_path)
    try:
        # Mock DFM analysis to pass
        with patch("worker.utils.dfm.validate_and_price") as mock_dfm:
            mock_result = MagicMock()
            mock_result.is_manufacturable = True
            mock_result.unit_cost = 50.0
            mock_result.metadata = {"weight_kg": 0.5}
            mock_dfm.return_value = mock_result

            # Execute
            result = submit_for_review(mock_component, cwd=tmp_path)
    finally:
        os.chdir(old_cwd)

    # Assert
    assert result is True
    mock_prerender.assert_called()
    mock_export_step.assert_called_once()

    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    with manifest_path.open() as f:
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test-session"
        assert "/path/to/render1.png" in manifest["renders"]


@patch("worker.utils.rendering.get_simulation_builder")
@patch("worker.utils.rendering.Image")
def test_prerender_24_views(
    mock_image, mock_get_builder, tmp_path
):
    # Mock mujoco in sys.modules
    mock_mujoco = MagicMock()
    with patch.dict(sys.modules, {"mujoco": mock_mujoco}):
        from worker.utils.rendering import prerender_24_views

        # Setup
        mock_builder = MagicMock()
        mock_get_builder.return_value = mock_builder
        mock_builder.build_from_assembly.return_value = tmp_path / "scene.xml"
        (tmp_path / "scene.xml").touch()

        mock_renderer = MagicMock()
        mock_mujoco.Renderer.return_value = mock_renderer
        mock_renderer.render.return_value = b"fake_image_data"

        # MjModel and MjData need to be mocked
        mock_model = MagicMock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model

        mock_data = MagicMock()
        mock_mujoco.MjData.return_value = mock_data

        mock_component = MagicMock()
        # Mock bbox for camera setup
        mock_component.bounding_box.return_value.min.X = -1
        mock_component.bounding_box.return_value.max.X = 1
        mock_component.bounding_box.return_value.min.Y = -1
        mock_component.bounding_box.return_value.max.Y = 1
        mock_component.bounding_box.return_value.min.Z = -1
        mock_component.bounding_box.return_value.max.Z = 1
        mock_component.bounding_box.return_value.size.X = 2
        mock_component.bounding_box.return_value.size.Y = 2
        mock_component.bounding_box.return_value.size.Z = 2

        # Execute
        renders = prerender_24_views(mock_component, output_dir=str(tmp_path))

        # Assert
        assert len(renders) == 24
        assert mock_get_builder.called
        assert mock_mujoco.MjModel.from_xml_path.called
        # Each view should have a render call
        assert mock_renderer.render.call_count == 24

        for render_path in renders:
            assert Path(render_path).parent == tmp_path
