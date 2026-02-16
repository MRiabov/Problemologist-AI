import pytest
from build123d import Box, Pos

from worker.workbenches.config import load_config
from worker.workbenches.print_3d import analyze_3dp, calculate_3dp_cost


@pytest.fixture
def config():
    return load_config()


def test_3dp_valid_part(config):
    part = Box(10, 10, 10)
    result = analyze_3dp(part, config)
    assert result.is_manufacturable is True
    assert len(result.violations) == 0
    assert result.unit_cost > 0
    assert "cost_breakdown" in result.metadata


def test_3dp_invalid_part_multi_body(config):
    # Create two disjoint solids
    # We can fuse them into a compound or just add them
    b1 = Box(10, 10, 10)
    b2 = Box(10, 10, 10)
    # Move b2 away
    b2 = Pos(20, 20, 20) * b2

    # Combine into a compound/part
    part = b1 + b2

    result = analyze_3dp(part, config)
    # analyze_3dp checks for single body
    assert result.is_manufacturable is False
    assert any("single body" in v for v in result.violations)


def test_3dp_cost_calculation(config):
    part = Box(10, 10, 10)  # 1000 mm3 = 1 cm3

    cost1 = calculate_3dp_cost(part, config, quantity=1)
    cost10 = calculate_3dp_cost(part, config, quantity=10)

    # Total cost should be roughly 10x but setup fee is constant
    # So unit cost for 10 should be lower than for 1
    assert cost10.total_cost > cost1.total_cost
    assert cost10.unit_cost < cost1.unit_cost

    # Check values roughly
    # Setup fee = 10.0 (default in config)
    # Material (ABS) = 1.04 g/cm3 * $20/kg * 1 cm3 = 1.04 g * $0.02/g = $0.0208
    # Run time = 1 cm3 / 15 cm3/hr = 1/15 hr = 0.0667 hr
    # Run cost = 0.0667 hr * $20/hr = $1.333
    # Unit cost ~ 1.35
    # Total for 1 ~ 11.35

    assert 11.0 < cost1.total_cost < 12.0


def test_3dp_reuse_discount(config):
    part = Box(10, 10, 10)
    context = {}

    cost1 = calculate_3dp_cost(part, config, quantity=1, context=context)
    cost2 = calculate_3dp_cost(part, config, quantity=1, context=context)

    assert cost2.setup_cost < cost1.setup_cost
    assert cost2.is_reused is True


def test_3dp_overhang_violation(config):
    # T-shape: vertical post, horizontal bar on top.
    # The horizontal bar has overhangs (faces pointing down not at min_z).
    post = Pos(0, 0, 10) * Box(10, 10, 20)  # From z=0 to z=20
    top = Pos(0, 0, 22.5) * Box(30, 10, 5)  # From z=20 to z=25

    part = post + top

    # Overhang angle check uses 45 degrees by default.
    # The overhang is horizontal (90 degrees from vertical, 0 degrees from -Z), so it should fail.

    result = analyze_3dp(part, config)
    # Should have overhang violations
    assert any("Overhang Violation" in v for v in result.violations)


def test_3dp_wall_thickness_violation(config):
    # Create a hollow box with thin walls
    # Box 10x10x10. Hollow it out.
    outer = Box(10, 10, 10)
    inner = Box(9.8, 9.8, 9.8)  # 0.1mm wall roughly

    part = outer - inner

    # Constraint is 0.8mm
    result = analyze_3dp(part, config)

    assert any("Wall thickness too thin" in v for v in result.violations)
