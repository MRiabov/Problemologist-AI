import pytest
import numpy as np
from build123d import Box, Part
from src.workbenches.analysis_utils import (
    part_to_trimesh,
    check_draft_angle,
    check_undercuts,
    check_wall_thickness,
    analyze_wall_thickness,
    load_config
)

class TestAnalysisUtils:

    @pytest.fixture(scope="class")
    def box_mesh(self):
        """Creates a 10x10x10 box mesh for testing."""
        part = Box(10, 10, 10)
        return part_to_trimesh(part)

    def test_load_config(self):
        """Verify load_config returns a dictionary with expected keys."""
        config = load_config()
        assert isinstance(config, dict)
        assert "injection_molding" in config
        assert "cnc" in config

    def test_check_draft_angle(self, box_mesh):
        """
        Verify draft angle check on a simple box.
        A 10x10x10 box with Z pull vector has vertical sides (90 deg).
        If min draft > 0, these sides should violate.
        """
        # Pull vector Z. Sides are 90 deg.
        # 2 deg min draft -> 88-92 deg is violation.
        violations = check_draft_angle(box_mesh, pull_vector=(0,0,1), min_angle_deg=2.0)

        # Expect 8 triangles (4 faces * 2 triangles/face) to violate.
        # Depending on mesh generation, faces might vary, but should be > 0.
        assert len(violations) >= 8

    def test_check_undercuts_box(self, box_mesh):
        """
        Verify undercut check on a box.
        From +Z, bottom faces are occluded (undercuts).
        From -Z, top faces are occluded.
        Real undercuts (intersection) should be 0.
        """
        undercuts_plus = check_undercuts(box_mesh, direction=(0,0,1))
        undercuts_minus = check_undercuts(box_mesh, direction=(0,0,-1))

        # Occlusion check: Bottom faces should be occluded from +Z
        assert len(undercuts_plus) > 0
        assert len(undercuts_minus) > 0

        # Real undercuts (unreachable from either side) should be 0
        real_undercuts = set(undercuts_plus).intersection(set(undercuts_minus))
        assert len(real_undercuts) == 0

    def test_analyze_wall_thickness_box(self, box_mesh):
        """
        Verify wall thickness analysis on a 10x10x10 box.
        Thickness should be approx 10mm.
        """
        stats = analyze_wall_thickness(box_mesh, sample_count=100)

        # Max thickness should be at least 10mm (could be higher at corners)
        assert stats["max_mm"] >= 9.9
        assert stats["min_mm"] > 0
        assert stats["average_mm"] > 0

    def test_check_wall_thickness_constraints(self, box_mesh):
        """
        Verify wall thickness violations.
        If max allowed is 5mm, box (10mm) should fail.
        """
        violations = check_wall_thickness(box_mesh, min_mm=1.0, max_mm=5.0)
        assert len(violations) > 0
