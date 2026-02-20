
import pytest
from pathlib import Path
from unittest.mock import patch, MagicMock
from worker_heavy.utils.mesh_utils import tetrahedralize, MeshProcessingError

@patch("subprocess.run")
def test_tetrahedralize_tetgen_conversion(mock_run, tmp_path):
    input_stl = tmp_path / "input.stl"
    input_stl.write_text("solid test\nendsolid test")
    output_msh = tmp_path / "output.msh"

    # Mock successful tetgen execution
    mock_run.return_value = MagicMock(returncode=0, stderr="")

    # Create dummy .node and .ele files that tetgen would produce
    # Standard TetGen uses 1-based indexing by default

    # .node format: <# of vertices> <dim (3)> <# attributes> <# boundary markers>
    # then lines of <vertex #> <x> <y> <z> ...
    node_file = tmp_path / "input.1.node"
    node_file.write_text("4 3 0 0\n1 0.0 0.0 0.0\n2 1.0 0.0 0.0\n3 0.0 1.0 0.0\n4 0.0 0.0 1.0\n")

    # .ele format: <# of tetrahedrons> <nodes per tet (4)> <# attributes>
    # then lines of <tet #> <node> <node> <node> <node> ...
    ele_file = tmp_path / "input.1.ele"
    ele_file.write_text("1 4 0\n1 1 2 3 4\n")

    # We need to ensure the temp directory used inside _tetrahedralize_tetgen
    # is the same as tmp_path or we mock TemporaryDirectory to return tmp_path.

    with patch("tempfile.TemporaryDirectory") as mock_temp_dir:
        mock_temp_dir.return_value.__enter__.return_value = str(tmp_path)

        result_path = tetrahedralize(input_stl, output_msh, method="tetgen")

        assert result_path.exists()
        assert result_path.stat().st_size > 0
        content = result_path.read_text()
        assert "$MeshFormat" in content or "$Nodes" in content
