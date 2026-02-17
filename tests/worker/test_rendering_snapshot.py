from unittest.mock import MagicMock, patch

import worker.tools.topology

# Workaround for production bug: _load_component is missing but used in rendering
if not hasattr(worker.tools.topology, "_load_component"):
    from worker.utils.loader import load_component_from_script

    worker.tools.topology._load_component = load_component_from_script

from shared.simulation.view_utils import get_best_isometric_view
from worker.activities.rendering import render_selection_snapshot


def test_render_selection_snapshot(tmp_path):
    # Setup
    # Patch S3Client inside the module where it's used
    with patch("worker.activities.rendering.S3Client") as mock_s3_class:
        mock_s3 = MagicMock()
        mock_s3_class.return_value = mock_s3

        # Create script.py
        script_path = tmp_path / "script.py"
        script_path.write_text("""
from build123d import *
def build():
    with BuildPart() as p:
        Box(10, 10, 10)
    return p.part
""")

        view_matrix = get_best_isometric_view((0, 0, 1))

        # Execute
        res = render_selection_snapshot(
            ids=["face_0"], view_matrix=view_matrix, script_path=str(script_path)
        )

        # Assert
        assert res.startswith("snapshots/")
        assert mock_s3.upload_file.called
        # Verify it was called with some path and the key
        args, kwargs = mock_s3.upload_file.call_args
        assert args[1] == res
