from build123d import Box, BuildPart, Compound, Align
from worker_heavy.utils.validation import validate
from worker_heavy.utils.dfm import _is_within_bounds, validate_and_price
from shared.models.schemas import BoundingBox, PartMetadata
from shared.workers.workbench_models import ManufacturingConfig, ManufacturingMethod, MaterialDefinition, MethodConfig

def test_geometric_validation_logic():
    # 1. Valid box
    with BuildPart() as p:
        Box(10, 10, 10)

    success, msg = validate(p.part)
    assert success, f"Validating single box failed: {msg}"

    # 2. Overlapping boxes
    with BuildPart() as p1:
        Box(10, 10, 10)
    with BuildPart() as p2:
        Box(10, 10, 10)

    comp = Compound(label="overlapping", children=[p1.part, p2.part])
    success, msg = validate(comp)
    assert not success, f"Expected overlap validation to fail, but it succeeded: {msg}"

def test_within_bounds_logic():
    with BuildPart() as p:
        Box(10, 10, 10, align=(Align.MIN, Align.MIN, Align.MIN))

    # Exactly inside
    build_zone = BoundingBox(min=[0, 0, 0], max=[10, 10, 10])
    valid, msg = _is_within_bounds(p.part, build_zone)
    assert valid, msg

    # Slightly outside X
    build_zone_small = BoundingBox(min=[0, 0, 0], max=[9.9, 10, 10])
    valid, msg = _is_within_bounds(p.part, build_zone_small)
    assert not valid
    assert "X max" in msg

def test_fem_validation_logic():
    # Setup config with a material missing FEM fields
    mat_no_fem = MaterialDefinition(
        name="Cheap Plastic",
        density_g_cm3=1.0,
        cost_per_kg=1.0
    )
    mat_with_fem = MaterialDefinition(
        name="Engineering Steel",
        density_g_cm3=7.8,
        cost_per_kg=5.0,
        youngs_modulus_pa=200e9,
        poissons_ratio=0.3,
        yield_stress_pa=250e6,
        ultimate_stress_pa=400e6
    )

    config = ManufacturingConfig(
        materials={"steel": mat_with_fem, "plastic": mat_no_fem},
        cnc=MethodConfig(materials={"steel": mat_with_fem, "plastic": mat_no_fem})
    )

    with BuildPart() as p:
        Box(10, 10, 10)
    p.part.metadata = PartMetadata(material_id="plastic")

    # Mock analyze_cnc to return a successful result
    from unittest.mock import patch
    from shared.workers.workbench_models import WorkbenchResult

    with patch("worker_heavy.utils.dfm.analyze_cnc") as mock_cnc:
        mock_cnc.return_value = WorkbenchResult(is_manufacturable=True, unit_cost=10.0, weight_g=100.0)

        # 1. FEM NOT required -> should pass
        res = validate_and_price(p.part, ManufacturingMethod.CNC, config, fem_required=False)
        assert res.is_manufacturable

        # 2. FEM required, material has NO FEM fields -> should fail
        res = validate_and_price(p.part, ManufacturingMethod.CNC, config, fem_required=True)
        assert not res.is_manufacturable
        assert any("missing required FEM fields" in v for v in res.violations)

        # 3. FEM required, material HAS FEM fields -> should pass
        p.part.metadata = PartMetadata(material_id="steel")
        res = validate_and_price(p.part, ManufacturingMethod.CNC, config, fem_required=True)
        assert res.is_manufacturable
