import pytest
from src.cots.providers.bd_warehouse import BDWarehouseProvider

def test_bdwarehouse_provider_initialization():
    provider = BDWarehouseProvider()
    assert len(provider.summaries) > 0
    assert any("Nema17" in s.id for s in provider.summaries)
    assert any("bearing" in s.id for s in provider.summaries)
    assert any("fastener" in s.id for s in provider.summaries)

def test_bdwarehouse_search():
    provider = BDWarehouseProvider()
    
    # Search motors
    results = provider.search("Nema")
    assert len(results) >= 3
    assert any(r.id == "bd_warehouse:motor:Nema17" for r in results)
    
    # Search bearings
    results = provider.search("bearing")
    assert len(results) > 0
    
    # Search fasteners
    results = provider.search("screw")
    assert len(results) > 0
    
    results = provider.search("nut")
    assert len(results) > 0

def test_bdwarehouse_get_part_recipe():
    provider = BDWarehouseProvider()
    
    # Motor recipe
    preview = provider.get_preview("bd_warehouse:motor:Nema17")
    assert 'from bd_warehouse.open_builds import StepperMotor' in preview.recipe
    assert 'part = StepperMotor("Nema17")' in preview.recipe
    
    # Bearing recipe
    bearing_id = [s.id for s in provider.summaries if "bearing" in s.id][0]
    preview = provider.get_preview(bearing_id)
    assert 'from bd_warehouse.bearing import SingleRowDeepGrooveBallBearing' in preview.recipe
    assert 'part = SingleRowDeepGrooveBallBearing' in preview.recipe
    
    # Fastener recipe
    screw_id = [s.id for s in provider.summaries if "screw" in s.id][0]
    preview = provider.get_preview(screw_id)
    assert 'from bd_warehouse.fastener import SocketHeadCapScrew' in preview.recipe
    assert 'part = SocketHeadCapScrew' in preview.recipe

def test_bdwarehouse_instantiation():
    provider = BDWarehouseProvider()
    
    # Test Motor
    part = provider.get_part("bd_warehouse:motor:Nema17")
    obj = part.factory()
    assert obj is not None
    
    # Test Bearing
    bearing_id = [s.id for s in provider.summaries if "bearing" in s.id][0]
    part = provider.get_part(bearing_id)
    obj = part.factory()
    assert obj is not None
    
    # Test Fastener
    screw_id = [s.id for s in provider.summaries if "screw" in s.id][0]
    part = provider.get_part(screw_id)
    obj = part.factory()
    assert obj is not None