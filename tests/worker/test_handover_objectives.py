import json
import os
import shutil
from pathlib import Path
from unittest.mock import MagicMock, patch
import pytest
from build123d import Compound, Box
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

    # Create dummy objectives.yaml in CWD (mocked)
    objectives_content = "theme: test"
    objectives_file = temp_dir / "objectives.yaml"
    objectives_file.write_text(objectives_content)

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
        with open(manifest_path, "r") as f:
            manifest = json.load(f)

        assert manifest["objectives_path"] == str(target_obj)
        assert manifest["session_id"] == "test_session"
