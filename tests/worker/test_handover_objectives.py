import json
import os
from unittest.mock import patch

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

    # We need to run in the temp_dir context
    os.chdir(temp_dir)

    with (
        patch("worker.utils.handover.prerender_24_views", return_value=["img1.png"]),
        patch("worker.utils.handover.export_step"),
        patch.dict(
            os.environ,
            {"RENDERS_DIR": str(temp_dir / "renders"), "SESSION_ID": "test_session"},
        ),
    ):
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
