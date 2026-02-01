import pytest
from src.cots.providers.bd_warehouse import BDWarehouseProvider

def test_bdwarehouse_provider_initialization():
    provider = BDWarehouseProvider()
    assert len(provider.summaries) > 0
    assert any("Nema17" in s.id for s in provider.summaries)

def test_bdwarehouse_search_motors():
    provider = BDWarehouseProvider()
    results = provider.search("Nema")
    assert len(results) >= 3
    assert any(r.id == "bd_warehouse:motor:Nema17" for r in results)

def test_bdwarehouse_get_part_recipe():
    provider = BDWarehouseProvider()
    part_id = "bd_warehouse:motor:Nema17"
    preview = provider.get_preview(part_id)
    
    assert preview.id == part_id
    assert 'from bd_warehouse.open_builds import StepperMotor' in preview.recipe
    assert 'part = StepperMotor("Nema17")' in preview.recipe

def test_bdwarehouse_get_part_instantiation():
    # This test actually instantiates the part using build123d/bd_warehouse
    provider = BDWarehouseProvider()
    part_id = "bd_warehouse:motor:Nema17"
    part = provider.get_part(part_id)
    
    # Instantiate
    obj = part.factory()
    assert obj is not None
    # Check if it has a label or children, implying it's a valid object
    assert hasattr(obj, "label")
    assert "StepperMotor" in obj.label
