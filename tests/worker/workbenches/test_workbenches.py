import pytest
import numpy as np
from build123d import *
from worker.workbenches.models import ManufacturingConfig, MethodConfig, MaterialDefinition

from worker.workbenches.analysis_utils import part_to_trimesh, check_wall_thickness

from worker.workbenches.injection_molding import analyze_im
from worker.workbenches.print_3d import analyze_3dp

def get_mock_config():
    return ManufacturingConfig(
        defaults={"material": "abs"},
        injection_molding=MethodConfig(
            materials={
                "abs": MaterialDefinition(
                    name="ABS",
                    density_g_cm3=1.04,
                    cost_per_kg=2.5,
                    machine_hourly_rate=60.0
                )
            },
            constraints={
                "min_wall_thickness_mm": 1.0,
                "max_wall_thickness_mm": 4.0,
                "min_draft_angle_deg": 0.0 # Disable draft check for simple box
            },
            costs={"base_mold_cost": 5000.0}
        ),
        three_dp=MethodConfig(
            materials={
                "abs": MaterialDefinition(
                    name="ABS (3DP)",
                    density_g_cm3=1.04,
                    cost_per_kg=20.0,
                    machine_hourly_rate=20.0
                )
            },
            constraints={"min_wall_thickness_mm": 0.8},
            costs={"setup_fee": 10.0}
        )
    )

def test_check_wall_thickness_thin():
    # Create a very thin box (0.5mm thick)
    with BuildPart() as p:
        Box(10, 10, 0.5)

    mesh = part_to_trimesh(p.part)
    # Check against min=1.0mm
    violations = check_wall_thickness(mesh, min_mm=1.0, max_mm=4.0)
    # Note: check_wall_thickness uses random sampling, so it might flaky for very small parts if not enough samples?
    # But for a box, normals are consistent.
    # However, sample_size=1000 is plenty for a simple box (12 triangles).

    assert len(violations) > 0, "Should detect thin wall"
    assert "too thin" in violations[0]

def test_check_wall_thickness_thick():
    # Create a very thick box (5.0mm thick)
    with BuildPart() as p:
        Box(10, 10, 5.0)

    mesh = part_to_trimesh(p.part)
    # Check against max=4.0mm
    violations = check_wall_thickness(mesh, min_mm=1.0, max_mm=4.0)
    assert len(violations) > 0, "Should detect thick wall"
    assert "too thick" in violations[0]

def test_check_wall_thickness_ok():
    # Create a box within limits (2.0mm thick).
    # Note: check_wall_thickness casts rays along normals.
    # For a Box(10, 10, 2), the side faces are 10mm apart, which > 4.0mm.
    # So we need a box where all dimensions are <= max_mm.
    with BuildPart() as p:
        Box(3.0, 3.0, 2.0)

    mesh = part_to_trimesh(p.part)
    violations = check_wall_thickness(mesh, min_mm=1.0, max_mm=4.0)
    assert len(violations) == 0, f"Should be valid, got {violations}"

def test_analyze_im_thin_wall():
    # Create a thin part
    with BuildPart() as p:
        Box(10, 10, 0.5)

    config = get_mock_config()

    # We expect check_wall_thickness to be called and return violation
    result = analyze_im(p.part, config)
    assert not result.is_manufacturable
    assert any("too thin" in v for v in result.violations)

def test_analyze_3dp_thin_wall():
    # Create a thin part
    with BuildPart() as p:
        Box(10, 10, 0.5)

    config = get_mock_config()

    result = analyze_3dp(p.part, config)
    # This assertion will fail until we update analyze_3dp
    assert not result.is_manufacturable
    assert any("too thin" in v for v in result.violations)

def test_analyze_3dp_real_config():
    from worker.workbenches.config import load_config

    # Load actual config
    config = load_config()
    constraints = config.three_dp.constraints
    assert constraints.get("min_wall_thickness_mm") == 0.8

    # Create a thin part (0.5mm < 0.8mm)
    with BuildPart() as p:
        Box(10, 10, 0.5)

    result = analyze_3dp(p.part, config)
    assert not result.is_manufacturable, "Should fail due to thin wall"
    assert any("too thin" in v for v in result.violations)
