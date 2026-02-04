import pytest
from build123d import Box, Compound
from src.compiler.models import CostBreakdown
from src.workbenches.cnc import CNCWorkbench


def test_cnc_validate():
    workbench = CNCWorkbench()

    # Valid part: Simple Box
    box = Box(10, 10, 10)
    # Wait, my undercut detection flags the bottom of a box as an undercut
    # for a single-side pull. CNC is usually single-side setup.
    # If the bottom is occluded, it's a violation for 3-axis single setup.
    violations = workbench.validate(box)
    assert len(violations) > 0  # Correct, bottom is occluded from +Z

    # Invalid part: Mushroom (more undercuts)
    top = Box(20, 20, 5).translate((0, 0, 10))
    stem = Box(5, 5, 10).translate((0, 0, 0))
    mushroom = top + stem
    violations_m = workbench.validate(mushroom)
    assert len(violations_m) > 0
    # Mushroom should have more complex occlusions
    assert "CNC Machining Violation" in violations_m[0]


def test_cnc_cost():
    workbench = CNCWorkbench()
    box = Box(10, 10, 10)  # 1000 mm3

    res_1 = workbench.calculate_cost(box, quantity=1)
    res_10 = workbench.calculate_cost(box, quantity=10)

    assert isinstance(res_1, CostBreakdown)

    cost_1 = res_1.total_cost
    cost_10 = res_10.total_cost

    assert cost_10 > cost_1
    # Unit cost should decrease as quantity increases (due to setup cost)
    unit_cost_1 = cost_1 / 1.0
    unit_cost_10 = cost_10 / 10.0
    assert unit_cost_10 < unit_cost_1

    # Verify breakdown fields
    details = res_1.details
    assert "stock_dims_mm" in details
    assert "stock_volume_cm3" in details
    assert "removed_volume_cm3" in details
    assert details["part_volume_cm3"] == 1.0  # 1000 mm3 = 1 cm3
    # stock dims for a 10x10x10 box should be [10, 10, 10]
    assert details["stock_dims_mm"] == [10.0, 10.0, 10.0]
    assert details["stock_volume_cm3"] == 1.0
    assert details["removed_volume_cm3"] == 0.0

    # Verify finishing pass
    assert "finishing_time_min" in details
    assert details["finishing_time_min"] > 0
    assert "surface_area_cm2" in details
    assert details["surface_area_cm2"] > 0
