import pytest
from build123d import Box

from worker.utils.dfm import validate_and_price
from worker.workbenches.config import load_config
from worker.workbenches.models import ManufacturingMethod


def test_facade_cnc():
    part = Box(10, 10, 10)
    config = load_config()
    result = validate_and_price(part, ManufacturingMethod.CNC, config)
    assert result.is_manufacturable is True
    assert result.unit_cost > 0


def test_facade_im():
    # A thin box to avoid too many violations but might still have draft issues
    part = Box(10, 10, 2)
    config = load_config()
    result = validate_and_price(part, ManufacturingMethod.INJECTION_MOLDING, config)
    assert result.unit_cost > 0
    # In my previous WP, a box with vertical walls fails draft check of 2.0 deg
    assert "Insufficient draft angle" in str(result.violations)


def test_facade_3dp():
    part = Box(5, 5, 5)
    config = load_config()
    result = validate_and_price(part, ManufacturingMethod.THREE_DP, config)
    assert result.is_manufacturable is True
    assert result.unit_cost > 0


def test_facade_invalid_method():
    part = Box(10, 10, 10)
    config = load_config()
    with pytest.raises(ValueError, match="Unsupported manufacturing method"):
        # We cast to avoid type errors in some IDEs, but we want to test runtime failure
        validate_and_price(part, "nonexistent", config)
