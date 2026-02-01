import pytest
from build123d import Box, Wedge, Cylinder
from src.workbenches.injection_molding import InjectionMoldingWorkbench

def test_im_validate():
    workbench = InjectionMoldingWorkbench()
    
    # 1. Draft Failure (Box with vertical walls)
    box = Box(10, 10, 10)
    violations = workbench.validate(box)
    assert any("Draft Violation" in v for g in [v for v in violations] for v in [g] if "Draft" in g)
    # Wait, simple list comprehension:
    assert any("Draft Violation" in v for v in violations)
    
    # 2. Thickness Failure (Thick block)
    # Default max is 4.0mm. 10mm block should fail.
    assert any("Wall Thickness Violation" in v for v in violations)

    # 3. Valid Case (Thin Pyramid/Wedge)
    # A pyramid with sloped walls should pass draft check.
    # Height 5, base 10x10, apex at center -> ~45 degree slope.
    # We need to make it thin enough for wall thickness if we check it.
    # But for now let's just ensure it passes or has fewer violations.
    pyramid = Wedge(10, 10, 5, 5, 5, 5, 5)
    violations_p = workbench.validate(pyramid)
    # Pyramid might still fail thickness if it's a solid block.
    # But it should NOT fail draft.
    assert not any("Draft Violation" in v for v in violations_p)
    
def test_im_cost():
    workbench = InjectionMoldingWorkbench()
    box = Box(10, 10, 10)
    
    cost_1 = workbench.calculate_cost(box, quantity=1)
    cost_10000 = workbench.calculate_cost(box, quantity=10000)
    
    # Unit cost at q=1 is massive (mostly tooling)
    unit_cost_1 = cost_1 / 1.0
    # Unit cost at q=10000 is low (tooling amortized)
    unit_cost_10k = cost_10000 / 10000.0
    
    assert unit_cost_1 > 5000.0
    assert unit_cost_10k < 10.0
    assert unit_cost_10k < unit_cost_1
