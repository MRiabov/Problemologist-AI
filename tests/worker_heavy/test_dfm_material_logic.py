import pytest
from build123d import Box

from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import (
    CNCMethodConfig,
    ManufacturingConfig,
    ManufacturingMethod,
    MaterialDefinition,
)
from worker_heavy.utils.dfm import validate_and_price


@pytest.fixture
def mock_config():
    return ManufacturingConfig(
        defaults={"material": "aluminum_6061"},
        materials={
            "aluminum_6061": MaterialDefinition(
                name="Aluminum 6061",
                density_g_cm3=2.7,
                cost_per_kg=6.0,
                youngs_modulus_pa=68.9e9,
                poissons_ratio=0.33,
                yield_stress_pa=276e6,
                ultimate_stress_pa=310e6,
            ),
            "no_fem_material": MaterialDefinition(
                name="No FEM Material",
                density_g_cm3=2.7,
                cost_per_kg=6.0,
                # Missing FEM fields
            ),
        },
        cnc=CNCMethodConfig(
            materials={
                "aluminum_6061": MaterialDefinition(
                    name="Aluminum 6061",
                    density_g_cm3=2.7,
                    cost_per_kg=6.0,
                    machine_hourly_rate=80.0,
                ),
                "steel": MaterialDefinition(
                    name="Steel",
                    density_g_cm3=7.8,
                    cost_per_kg=2.0,
                    machine_hourly_rate=100.0,
                ),
            }
        ),
    )


def test_validate_and_price_uses_metadata_material(mock_config):
    # Create a part with 'steel' material in metadata
    part = Box(10, 10, 10)
    part.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="steel"
    )

    result = validate_and_price(part, ManufacturingMethod.CNC, mock_config, quantity=1)

    # Steel density is 7.8, Vol is 1000mm3 = 1cm3.
    # Weight should be 1 * 7.8 = 7.8g.
    # If it used default (aluminum 2.7), weight would be 2.7g.
    assert result.weight_g == pytest.approx(7.8)


def test_validate_and_price_fem_gate_missing_fields(mock_config):
    # Create a part with a material that lacks FEM fields
    part = Box(10, 10, 10)
    part.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="no_fem_material"
    )

    # fem_required=True should trigger violation
    result = validate_and_price(
        part, ManufacturingMethod.CNC, mock_config, quantity=1, fem_required=True
    )

    assert not result.is_manufacturable
    assert any("missing required FEM fields" in v for v in result.violations)


def test_validate_and_price_fem_gate_success(mock_config):
    # Aluminum has FEM fields in global section
    part = Box(10, 10, 10)
    part.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum_6061"
    )

    result = validate_and_price(
        part, ManufacturingMethod.CNC, mock_config, quantity=1, fem_required=True
    )

    assert result.is_manufacturable
    assert not any("FEM" in v for v in result.violations)


def test_validate_and_price_no_metadata_uses_default(mock_config):
    # Create a part WITHOUT metadata
    part = Box(10, 10, 10)
    if hasattr(part, "metadata"):
        del part.metadata

    # Should not crash and should use default (aluminum_6061)
    result = validate_and_price(part, ManufacturingMethod.CNC, mock_config, quantity=1)

    # Aluminum density is 2.7. Weight should be 1 * 2.7 = 2.7g.
    assert result.weight_g == pytest.approx(2.7)


def test_validate_and_price_empty_config_fallback():
    # Test fallback when config materials are empty
    empty_config = ManufacturingConfig(
        defaults={"material": "abs"},
        materials={},
        cnc=CNCMethodConfig(materials={}),
    )
    part = Box(10, 10, 10)

    # Should not crash and use hardcoded fallback
    result = validate_and_price(part, ManufacturingMethod.CNC, empty_config, quantity=1)
    assert result.weight_g > 0
