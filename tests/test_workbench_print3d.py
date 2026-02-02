import pytest
from build123d import Box, Compound

from workbenches.print_3d import Print3DWorkbench


def test_print3d_validate_success():
    workbench = Print3DWorkbench()
    box = Box(10, 10, 10)
    violations = workbench.validate(box)
    assert len(violations) == 0


def test_print3d_validate_multiple_solids():
    workbench = Print3DWorkbench()
    box1 = Box(10, 10, 10)
    box2 = Box(10, 10, 10).translate((20, 0, 0))
    compound = Compound(children=[box1, box2])
    violations = workbench.validate(compound)
    assert any("single body" in v.lower() for v in violations)


def test_print3d_cost():
    workbench = Print3DWorkbench(material_cost=0.1)
    box = Box(10, 10, 10)  # Volume = 1000
    cost = workbench.calculate_cost(box)
    assert pytest.approx(cost) == 100.0
