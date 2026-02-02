import pytest
from pathlib import Path

from src.cots.providers.bd_warehouse import BDWarehouseProvider


def test_bdwarehouse_provider_initialization():
    provider = BDWarehouseProvider()
    assert len(provider.summaries) > 0
    assert any("Nema17" in s.id for s in provider.summaries)
    assert any("bearing" in s.id for s in provider.summaries)
    assert any("fastener" in s.id for s in provider.summaries)
    assert any("beam" in s.id for s in provider.summaries)


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

    # Search beams
    results = provider.search("v_slot")
    assert len(results) > 0
    results = provider.search("c_beam")
    assert len(results) > 0


def test_bdwarehouse_get_preview_details():
    provider = BDWarehouseProvider()

    # Motor preview
    preview = provider.get_preview("bd_warehouse:motor:Nema17")

    # 1. Verify image path
    assert preview.image_path != ""
    assert Path(preview.image_path).exists()
    assert preview.image_path.endswith(".png")

    # 2. Verify description is not default
    assert "No description available" not in preview.description
    assert "stepper motor" in preview.description.lower()

    # Bearing preview
    bearing_id = next(s.id for s in provider.summaries if "bearing" in s.id)
    preview = provider.get_preview(bearing_id)
    assert Path(preview.image_path).exists()
    assert "No description available" not in preview.description


def test_bdwarehouse_instantiation():
    provider = BDWarehouseProvider()

    # Test Motor
    part = provider.get_part("bd_warehouse:motor:Nema17")
    obj = part.factory()
    assert obj is not None

    # Test Bearing
    bearing_id = next(s.id for s in provider.summaries if "bearing" in s.id)
    part = provider.get_part(bearing_id)
    obj = part.factory()
    assert obj is not None

    # Test Fastener
    screw_id = next(s.id for s in provider.summaries if "screw" in s.id)
    part = provider.get_part(screw_id)
    obj = part.factory()
    assert obj is not None

    # Test Beam
    beam_id = next(s.id for s in provider.summaries if "beam" in s.id)
    part = provider.get_part(beam_id)
    obj = part.factory()
    assert obj is not None
