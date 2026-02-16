import pytest
from worker.tools.topology import inspect_topology
import os


def test_inspect_topology_face(tmp_path):
    # Create a temporary script.py
    script_path = tmp_path / "script.py"
    script_path.write_text("""
from build123d import *
def build():
    with BuildPart() as p:
        Box(10, 10, 10)
    return p.part
""")

    res = inspect_topology("face_0", script_path=str(script_path))
    assert res["type"] == "face"
    assert "normal" in res
    assert "center" in res
    assert res["area"] == pytest.approx(100.0)


def test_inspect_topology_part(tmp_path):
    script_path = tmp_path / "script.py"
    script_path.write_text("""
from build123d import *
def build():
    with BuildPart() as p:
        Box(10, 10, 10)
    return p.part
""")

    res = inspect_topology("part_0", script_path=str(script_path))
    assert res["type"] == "part"
    assert "center" in res
    assert "bbox" in res


def test_inspect_topology_invalid(tmp_path):
    script_path = tmp_path / "script.py"
    script_path.write_text("""
from build123d import *
def build():
    return Box(1,1,1)
""")
    with pytest.raises(ValueError):
        inspect_topology("invalid_id", script_path=str(script_path))
