from pathlib import Path

import pytest
from build123d import Box

from src.compiler.geometry import export_mesh, generate_colliders


def test_export_mesh(tmp_path):
    box = Box(10, 10, 10)
    stl_path = tmp_path / "test.stl"
    result = export_mesh(box, str(stl_path))
    assert result == str(stl_path)
    assert stl_path.exists()
    assert stl_path.stat().st_size > 0


def test_generate_colliders():
    box = Box(10, 10, 10)
    colliders = generate_colliders(box)
    assert len(colliders) == 1
    # Check if it's a solid and has volume
    assert colliders[0].volume > 0
    # Bounding box of a 10x10x10 cube should be the same cube
    assert pytest.approx(colliders[0].volume) == 1000.0
