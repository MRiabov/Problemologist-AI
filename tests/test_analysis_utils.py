import pytest
from build123d import Box
from src.workbenches.analysis_utils import (
    part_to_trimesh,
    load_config,
    analyze_wall_thickness,
)


def test_part_to_trimesh():
    # Create a simple box
    box = Box(10, 10, 10)

    # Convert to trimesh
    mesh = part_to_trimesh(box)

    # Assertions
    assert mesh.is_watertight
    assert len(mesh.vertices) > 0
    # A box should have 12 triangles (2 per face)
    assert len(mesh.faces) == 12


def test_analyze_wall_thickness():
    # Box 10x10x10
    box = Box(10, 10, 10)
    mesh = part_to_trimesh(box)
    stats = analyze_wall_thickness(mesh)

    assert "min_mm" in stats
    assert "max_mm" in stats
    assert "average_mm" in stats
    # For a solid 10mm box, raycasting from one side to the other should find 10mm
    assert stats["max_mm"] >= 9.9
    assert stats["min_mm"] >= 9.9


def test_load_config():
    config = load_config()

    # Check structure
    assert "cnc" in config
    assert "injection_molding" in config
    assert "defaults" in config

    # Check specific values
    assert config["cnc"]["constraints"]["min_tool_radius_mm"] == 3.0
    assert config["injection_molding"]["materials"]["abs"]["density_g_cm3"] == 1.04
