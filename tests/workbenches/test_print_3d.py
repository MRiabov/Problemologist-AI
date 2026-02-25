import pytest
from build123d import Box, Pos

from worker_heavy.workbenches.config import load_config
from worker_heavy.workbenches.print_3d import analyze_3dp, calculate_3dp_cost


@pytest.fixture
def config():
    return load_config()


def test_3dp_valid_part(config):
    part = Box(10, 10, 10)
    result = analyze_3dp(part, config)
    assert result.is_manufacturable is True
    assert len(result.violations) == 0
    assert result.unit_cost > 0
    assert result.metadata.cost_breakdown is not None


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
    from shared.workers.workbench_models import WorkbenchContext

    context = WorkbenchContext()

    cost1 = calculate_3dp_cost(part, config, quantity=1, context=context)
    cost2 = calculate_3dp_cost(part, config, quantity=1, context=context)

    assert cost2.setup_cost < cost1.setup_cost
    assert cost2.is_reused is True
