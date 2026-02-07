import pytest
from build123d import Box, Pos

from src.workbenches.config import load_config
from src.workbenches.print_3d import Print3DWorkbench, analyze_3dp

def test_analyze_3dp_basic():
    part = Box(10, 10, 10)
    config = load_config()

    result = analyze_3dp(part, config)

    assert result.is_manufacturable is True
    assert result.unit_cost > 0
    assert len(result.violations) == 0

    # Check if cost breakdown is present
    breakdown = result.metadata.get("cost_breakdown")
    assert breakdown is not None
    assert breakdown["process"] == "print_3d"
    assert breakdown["details"]["part_volume_cm3"] == pytest.approx(1.0, 0.1)

def test_analyze_3dp_invalid_geometry():
    # Create a compound of non-touching solids (multi-body)
    p1 = Box(10, 10, 10)
    p2 = Box(10, 10, 10)
    # Move p2 away so they don't touch
    p2 = p2.move(Pos(20, 0, 0))
    part = p1 + p2

    config = load_config()
    result = analyze_3dp(part, config)

    assert result.is_manufacturable is False
    assert any("single body" in v for v in result.violations)

def test_workbench_class():
    wb = Print3DWorkbench()
    part = Box(10, 10, 10)

    violations = wb.validate(part)
    assert len(violations) == 0

    cost = wb.calculate_cost(part)
    assert cost.total_cost > 0
    assert cost.process == "print_3d"

def test_cost_calculation_details():
    # 10x10x10 mm box = 1000 mm3 = 1 cm3
    part = Box(10, 10, 10)
    config = load_config()

    # From default config (ABS):
    # density = 1.04 g/cm3
    # cost_per_kg = 20.0
    # machine_hourly_rate = 20.0
    # setup_fee = 10.0

    # Material Cost:
    # mass = 1 cm3 * 1.04 g/cm3 = 1.04 g = 0.00104 kg
    # cost = 0.00104 kg * $20/kg = $0.0208

    # Machine Time:
    # volume = 1 cm3
    # rate = 0.015 cm3/s
    # time_s = 1 / 0.015 = 66.66 s
    # time_hr = 66.66 / 3600 = 0.0185 hr
    # cost = 0.0185 hr * $20/hr = $0.3703

    # Total Unit Cost = 0.0208 + 0.3703 = 0.3911
    # Total Cost = Setup (10) + Unit Cost = 10.3911
    # Amortized Unit Cost (quantity=1) = 10.3911

    result = analyze_3dp(part, config)
    breakdown = result.metadata["cost_breakdown"]

    assert breakdown["unit_cost"] == pytest.approx(10.39, abs=0.05)
    assert breakdown["setup_cost"] == 10.0
    assert breakdown["total_cost"] == pytest.approx(10.39, abs=0.05)
