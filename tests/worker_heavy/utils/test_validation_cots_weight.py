import pytest
from build123d import Box
from shared.models.schemas import CotsPartEstimate
from worker_heavy.utils.validation import calculate_assembly_totals

def test_calculate_assembly_totals_cots_weight():
    # Setup a dummy component
    component = Box(10, 10, 10)

    # Create COTS parts with known IDs from the codebase
    # Using IDs found in shared/cots/parts/electronics.py and motors.py
    cots_parts = [
        CotsPartEstimate(
            part_id="LRS-350-24", # PowerSupply, weight 760.0
            quantity=1,
            unit_cost_usd=35.0,
            manufacturer="Generic",
            source="Catalog"
        ),
        CotsPartEstimate(
            part_id="SG90", # ServoMotor, weight 9.0
            quantity=2,
            unit_cost_usd=2.5,
            manufacturer="Generic",
            source="Catalog"
        ),
        CotsPartEstimate(
            part_id="XT60", # Connector, weight 7.0
            quantity=10,
            unit_cost_usd=0.8,
            manufacturer="Generic",
            source="Catalog"
        )
    ]

    # Calculate totals
    cost, weight = calculate_assembly_totals(component, cots_parts=cots_parts)

    # Expected weight:
    # LRS-350-24: 760.0 * 1 = 760.0
    # SG90: 9.0 * 2 = 18.0
    # XT60: 7.0 * 10 = 70.0
    # Total = 848.0

    # Expected cost:
    # 35.0 * 1 + 2.5 * 2 + 0.8 * 10 = 35 + 5 + 8 = 48.0

    assert cost == 48.0

    # Currently this will fail because weight calculation is stubbed
    # Asserting exact value to confirm it fails, or asserts 0 if implemented as pass
    assert weight == 848.0
