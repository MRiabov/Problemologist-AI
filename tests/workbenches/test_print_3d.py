import pytest
from build123d import Box, BuildPart, Location, Mode, Sphere
from worker.workbenches.config import load_config
from worker.workbenches.print_3d import analyze_3dp, calculate_3dp_cost


@pytest.fixture
def config():
    return load_config()


def test_3dp_valid_part(config):
    # A simple solid box should be manufacturable
    with BuildPart() as p:
        Box(20, 20, 20)

    result = analyze_3dp(p.part, config)
    assert result.is_manufacturable is True
    assert len(result.violations) == 0
    assert result.unit_cost > 0


def test_3dp_disjoint_solids(config):
    # Two disjoint solids should fail "Single Body Check"
    # Create two separate solids and combine them into a single Part (Compound)
    # Note: Box() returns a Solid. The '+' operator on TopoDS shapes creates a Compound if they don't fuse.

    s1 = Box(10, 10, 10)
    s2 = Sphere(5).move(Location((30, 0, 0)))

    p2 = s1 + s2

    result = analyze_3dp(p2, config)
    assert result.is_manufacturable is False
    assert any("single body" in v.lower() for v in result.violations)


def test_3dp_cost_calculation(config):
    # Create a 10x10x10 mm box = 1 cm3
    with BuildPart() as p:
        Box(10, 10, 10)

    # Calculate for 1 unit
    cost1 = calculate_3dp_cost(p.part, config, quantity=1)

    # Calculate for 10 units
    cost10 = calculate_3dp_cost(p.part, config, quantity=10)

    # Setup fee is fixed, so total cost for 10 should be less than 10 * cost1
    assert cost10.total_cost < (cost1.total_cost * 10)

    # Unit cost should decrease
    assert cost10.unit_cost < cost1.unit_cost

    # Verify values roughly
    # Volume = 1 cm3
    # Material Cost (ABS) ~ $20/kg * 1.04 g/cm3 / 1000 = $0.0208
    # Machine Time ~ 1 cm3 / 15 cm3/hr = 0.066 hr
    # Machine Hourly Rate = $20.0 (from config)
    # Machine Cost ~ 0.066 hr * $20/hr = $1.33
    # Setup Fee ~ $10.00

    # Cost1 ~ 10 + 0.02 + 1.33 = 11.35

    assert 11.0 < cost1.total_cost < 12.0

    # Test reuse discount
    context = {}
    cost_first = calculate_3dp_cost(p.part, config, quantity=1, context=context)
    cost_second = calculate_3dp_cost(p.part, config, quantity=1, context=context)

    assert cost_second.setup_cost < cost_first.setup_cost
    assert cost_second.is_reused is True
