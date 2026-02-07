from build123d import Box, Part

from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench


def test_cnc_workbench():
    wb = CNCWorkbench()
    part = Part(Box(10, 10, 10))

    # Validate
    violations = wb.validate(part)
    # Expect violations for a simple box (undercuts, etc)
    assert len(violations) > 0

    # Cost
    cost = wb.calculate_cost(part, quantity=10)
    assert cost.process == "cnc_milling"
    assert cost.total_cost > 0
    assert cost.unit_cost > 0

def test_injection_molding_workbench():
    wb = InjectionMoldingWorkbench()
    part = Part(Box(10, 10, 10))

    # Validate
    violations = wb.validate(part)
    # Expect violations for a simple box (draft, thickness)
    assert len(violations) > 0

    # Cost
    cost = wb.calculate_cost(part, quantity=1000)
    assert cost.process == "injection_molding"
    assert cost.total_cost > 0
    assert cost.unit_cost > 0
