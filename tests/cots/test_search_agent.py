import pytest
from unittest.mock import MagicMock, patch
from shared.cots.agent import search_cots_catalog, create_cots_search_agent
from shared.cots.models import COTSItem

@pytest.fixture
def mock_parts():
    return [
        COTSItem(
            part_id="test_bolt_m6",
            name="M6 Hex Bolt",
            category="fastener",
            unit_cost=0.5,
            weight_g=15.0,
            import_recipe="""from bd_warehouse.fastener import HexBolt
bolt = HexBolt(size='M6')""",
            metadata={"size": "M6"}
        )
    ]

def test_search_cots_catalog_tool(mock_parts):
    with patch("shared.cots.agent.search_parts") as mock_search:
        mock_search.return_value = mock_parts
        
        # Tools in LangChain are invoked via .invoke()
        result = search_cots_catalog.invoke({
            "query": "bolt", 
            "max_weight_g": 20.0
        })
        
        assert "M6 Hex Bolt" in result
        assert "test_bolt_m6" in result
        assert "Weight: 15.00g" in result
        mock_search.assert_called_once()
        # Verify it was called with SearchQuery
        args, _ = mock_search.call_args
        sq = args[0]
        assert sq.query == "bolt"
        assert sq.constraints["max_weight"] == 20.0

def test_search_cots_catalog_no_results():
    with patch("shared.cots.agent.search_parts") as mock_search:
        mock_search.return_value = []
        
        result = search_cots_catalog.invoke({"query": "nonexistent"})
        
        assert "No parts found" in result

def test_create_cots_search_agent():
    with patch("shared.cots.agent.ChatOpenAI") as mock_chat:
        agent = create_cots_search_agent()
        assert agent is not None
