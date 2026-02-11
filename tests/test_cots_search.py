from unittest.mock import patch

import pytest

from shared.cots.agent import search_cots_catalog
from shared.cots.models import COTSItem


@pytest.fixture
def mock_search_parts():
    with patch("shared.cots.agent.search_parts") as mock:
        yield mock


def test_search_cots_catalog_success(mock_search_parts):
    # Setup mock return data
    mock_search_parts.return_value = [
        COTSItem(
            part_id="M3-10",
            name="M3 Bolt 10mm",
            category="fastener",
            unit_cost=0.10,
            weight_g=1.5,
            import_recipe="Box(3)",
            metadata={"material": "steel"},
        ),
        COTSItem(
            part_id="M3-12",
            name="M3 Bolt 12mm",
            category="fastener",
            unit_cost=0.12,
            weight_g=1.7,
            import_recipe="Box(3)",
            metadata={"material": "steel"},
        ),
    ]

    # Invoke tool with correct arguments dict
    result = search_cots_catalog.invoke({"query": "M3 bolts", "max_cost": 0.20})

    # Verify search_parts was called with correct constraints
    args, _ = mock_search_parts.call_args
    search_query = args[0]
    assert search_query.query == "M3 bolts"
    assert search_query.constraints["max_cost"] == 0.20

    # Verify result formatting
    assert "M3-10" in result
    assert "M3 Bolt 10mm" in result
    assert "0.1" in result
    assert "Box(3)" in result


def test_search_cots_catalog_no_results(mock_search_parts):
    mock_search_parts.return_value = []

    result = search_cots_catalog.invoke({"query": "Unobtainium"})

    assert "No parts found matching the criteria" in result


def test_search_cots_catalog_all_args(mock_search_parts):
    mock_search_parts.return_value = []

    search_cots_catalog.invoke(
        {
            "query": "Motor",
            "max_weight_g": 100.0,
            "max_cost": 50.0,
            "category": "motor",
            "limit": 10,
        }
    )

    args, _ = mock_search_parts.call_args
    search_query = args[0]
    assert search_query.constraints["max_weight"] == 100.0
    assert search_query.constraints["max_cost"] == 50.0
    assert search_query.constraints["category"] == "motor"
    assert search_query.limit == 10
