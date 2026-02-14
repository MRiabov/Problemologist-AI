
import io
import hashlib
import tempfile
import trimesh
import pytest
from build123d import Box, Part
from worker.workbenches.analysis_utils import (
    analyze_geometry,
    part_to_trimesh,
    compute_part_hash,
)

def test_analyze_geometry_returns_correct_types():
    """
    Verify analyze_geometry returns a Trimesh object and a string hash.
    """
    part = Box(10, 10, 10)
    mesh, part_hash = analyze_geometry(part)

    assert isinstance(mesh, trimesh.Trimesh)
    assert isinstance(part_hash, str)
    assert len(part_hash) == 64  # SHA256 hex digest length
    # Box should have 8 vertices (or more depending on triangulation)
    assert len(mesh.vertices) >= 8
    assert len(mesh.faces) >= 12

def test_analyze_geometry_consistency():
    """
    Verify analyze_geometry returns consistent results for the same part.
    """
    part = Box(10, 10, 10)
    mesh1, hash1 = analyze_geometry(part)
    mesh2, hash2 = analyze_geometry(part)

    assert hash1 == hash2
    assert len(mesh1.vertices) == len(mesh2.vertices)

def test_compute_part_hash_consistency_with_analyze_geometry():
    """
    Verify compute_part_hash returns the same hash as analyze_geometry.
    """
    part = Box(10, 10, 10)
    _, geo_hash = analyze_geometry(part)
    standalone_hash = compute_part_hash(part)

    assert geo_hash == standalone_hash

def test_part_to_trimesh_works():
    """
    Verify part_to_trimesh returns a valid mesh.
    """
    part = Box(5, 5, 5)
    mesh = part_to_trimesh(part)

    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.volume > 0
    # Expected volume is 125, but mesh volume might be slightly approximate
    assert abs(mesh.volume - 125) < 1.0

def test_performance_regression_check():
    """
    Simple benchmark to ensure analyze_geometry is faster than calling both separately.
    """
    import time
    part = Box(20, 20, 20)

    iterations = 10

    # Measure separate calls (simulate old behavior)
    start_time = time.time()
    for _ in range(iterations):
        _ = part_to_trimesh(part)
        _ = compute_part_hash(part)
    separate_time = time.time() - start_time

    # Measure combined call
    start_time = time.time()
    for _ in range(iterations):
        _, _ = analyze_geometry(part)
    combined_time = time.time() - start_time

    print(f"\nSeparate time: {separate_time:.4f}s")
    print(f"Combined time: {combined_time:.4f}s")

    assert combined_time < separate_time
