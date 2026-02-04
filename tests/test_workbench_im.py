import pytest
from build123d import Box, Wedge, Cylinder
from src.workbenches.injection_molding import InjectionMoldingWorkbench


def test_im_validate():
    workbench = InjectionMoldingWorkbench()

    # 1. Draft Failure (Box with vertical walls)
    box = Box(10, 10, 10)
    violations = workbench.validate(box)
    assert any(
        "Draft Violation" in v
        for g in [v for v in violations]
        for v in [g]
        if "Draft" in g
    )
    # Wait, simple list comprehension:
    assert any("Draft Violation" in v for v in violations)

    # 2. Thickness Failure (Thick block)
    # Default max is 4.0mm. 10mm block should fail.
    assert any("Wall Thickness Violation" in v for v in violations)

    # 3. Valid Case (Rotated Box)
    box_valid = Box(2, 2, 2, rotation=(5, 5, 5))
    violations_c = workbench.validate(box_valid)
    assert len(violations_c) == 0
    # Verify cost result contains wall thickness stats
    res = workbench.calculate_cost(box, quantity=1)
    assert "wall_thickness_stats" in res.details
    assert res.details["wall_thickness_stats"]["max_mm"] >= 9.9


def test_im_cost():
    workbench = InjectionMoldingWorkbench()
    box = Box(10, 10, 10)

    res_1 = workbench.calculate_cost(box, quantity=1)
    res_10000 = workbench.calculate_cost(box, quantity=10000)

    from src.workbenches.models import CostBreakdown

    assert isinstance(res_1, CostBreakdown)
    assert res_1.total_cost is not None
    assert res_1.details is not None

    cost_1 = res_1.total_cost
    cost_10000 = res_10000.total_cost

    # Unit cost at q=1 is massive (mostly tooling)
    unit_cost_1 = cost_1 / 1.0
    # Unit cost at q=10000 is low (tooling amortized)
    unit_cost_10k = cost_10000 / 10000.0

    assert unit_cost_1 > 5000.0
    assert unit_cost_10k < 10.0
    assert unit_cost_10k < unit_cost_1

    # Verify details fields (renamed from breakdown)
    details = res_1.details
    assert "wall_thickness_stats" in details
    stats = details["wall_thickness_stats"]
    assert stats["min_mm"] > 0
    assert stats["max_mm"] > 0
    assert stats["average_mm"] > 0
    # For a 10x10x10 solid box, thickness should be ~10mm
    assert stats["max_mm"] >= 10.0
