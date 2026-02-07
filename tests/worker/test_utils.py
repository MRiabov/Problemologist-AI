import json
import os
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
    os.environ["RENDERS_DIR"] = str(tmp_path)
    os.environ["SESSION_ID"] = "test-session"
    os.environ["TIMESTAMP"] = "2026-02-07"

    mock_component = MagicMock()
    mock_prerender.return_value = ["/path/to/render1.png"]

    # Execute
    result = submit_for_review(mock_component)

    # Assert
    assert result is True
    mock_prerender.assert_called_once_with(mock_component)
    mock_export_step.assert_called_once()

    manifest_path = tmp_path / "review_manifest.json"
    assert manifest_path.exists()

    with open(manifest_path, "r") as f:
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test-session"
        assert "/path/to/render1.png" in manifest["renders"]


@patch("worker.utils.rendering.pv.Plotter")
@patch("worker.utils.rendering.pv.read")
@patch("worker.utils.rendering.export_stl")
def test_prerender_24_views(
    mock_export_stl, mock_pv_read, mock_plotter_class, tmp_path
):
    from worker.utils.rendering import prerender_24_views

    # Setup
    mock_plotter = MagicMock()
    mock_plotter_class.return_value = mock_plotter
    mock_component = MagicMock()

    # Execute
    renders = prerender_24_views(mock_component, output_dir=str(tmp_path))

    # Assert
    assert len(renders) == 24
    assert mock_export_stl.called
    assert mock_pv_read.called
    # Each view should have a screenshot
    assert mock_plotter.screenshot.call_count == 24

    for render_path in renders:
        assert Path(render_path).parent == tmp_path
