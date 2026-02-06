import pytest
from build123d import Box
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench

def test_cnc_workbench_functionality():
    # Create a simple part (a box)
    part = Box(10, 10, 10)

    wb = CNCWorkbench()

    # Test calculate_cost
    cost = wb.calculate_cost(part, quantity=1)
    assert cost.process == "cnc_milling"
    assert cost.total_cost > 0
    assert cost.details["stock_volume_cm3"] > 0

    # Test validate
    # A simple box has a bottom face (2 triangles) which is an undercut for 3-axis milling from Top.
    violations = wb.validate(part)
    # We expect exactly 1 violation message regarding undercuts
    assert len(violations) == 1
    assert "2 undercut faces" in violations[0]

def test_injection_molding_functionality():
    # Create a simple part
    part = Box(20, 20, 5)

    wb = InjectionMoldingWorkbench()

    # Test calculate_cost
    cost = wb.calculate_cost(part, quantity=100)
    assert cost.process == "injection_molding"
    assert cost.total_cost > 0
    assert cost.details["tooling_cost"] > 0

    # Test validate
    # A box has vertical walls (90 deg to pull).
    # Draft angle requirement is min_draft_angle_deg (e.g. 2 deg).
    # So vertical walls (0 draft) should be violations.
    violations = wb.validate(part)
    # Expect violations due to lack of draft
    assert len(violations) > 0
    assert "IM Draft Violation" in violations[0]
