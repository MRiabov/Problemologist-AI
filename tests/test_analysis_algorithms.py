import pytest
from build123d import Box, Wedge, Cylinder, Compound
from src.workbenches.analysis_utils import part_to_trimesh, check_draft_angle, check_undercuts, check_wall_thickness
import numpy as np

def test_check_draft_angle():
    # Box has vertical walls (0 draft relative to Z pull)
    box = Box(10, 10, 10)
    mesh_box = part_to_trimesh(box)
    violations = check_draft_angle(mesh_box, (0, 0, 1), 2.0)
    assert len(violations) > 0

    # Correct Wedge usage (args are width, depth, height, x2_start, x2_end, y2_start, y2_end)
    # or similar depending on version. Let's use simple parameters.
    wedge = Wedge(10, 10, 10, 5, 10, 5, 10)
    mesh_wedge = part_to_trimesh(wedge)
    violations_wedge = check_draft_angle(mesh_wedge, (0, 0, 1), 2.0)
    assert len(violations_wedge) < len(violations)

def test_check_undercuts():
    # Box: Bottom face is occluded from ABOVE (+Z pull).
    # In a 2-part mold, the parting line would be in the middle, but here we assume 
    # a single approach/pull direction for simplicity.
    box = Box(10, 10, 10)
    mesh_box = part_to_trimesh(box)
    undercuts = check_undercuts(mesh_box, (0, 0, 1))
    # Bottom face (2 triangles) should be occluded by top face
    assert len(undercuts) > 0

    # Mushroom/T-shape
    top = Box(20, 20, 5).translate((0,0,10))
    stem = Box(5, 5, 10).translate((0,0,0))
    t_shape = top + stem
    mesh_t = part_to_trimesh(t_shape)
    undercuts_t = check_undercuts(mesh_t, (0, 0, 1))
    assert len(undercuts_t) > len(undercuts)

def test_cylinder_draft():
    # Cylinder has vertical walls
    cyl = Cylinder(radius=5, height=10)
    mesh_cyl = part_to_trimesh(cyl)
    violations = check_draft_angle(mesh_cyl, (0, 0, 1), 2.0)
    assert len(violations) > 0
def test_check_wall_thickness():
    # Solid box 10x10x10
    box = Box(10, 10, 10)
    mesh_box = part_to_trimesh(box)
    
    # If we check for thickness 1-5mm, a 10mm box should fail (too thick)
    violations = check_wall_thickness(mesh_box, 1.0, 5.0)
    assert len(violations) > 0
    
    # If we check for thickness 1-15mm, it should pass
    violations_pass = check_wall_thickness(mesh_box, 1.0, 15.0)
    assert len(violations_pass) == 0

    # Thin plate
    plate = Box(20, 20, 0.5)
    mesh_plate = part_to_trimesh(plate)
    # Check for min 1.0mm
    violations_thin = check_wall_thickness(mesh_plate, 1.0, 5.0)
    assert len(violations_thin) > 0
