from shared.cots.base import COTSPart
from shared.cots.runtime import SearchQuery, search_parts
from tests.observability.test_observability_utils import (
    assert_event_emitted,
    clear_emitted_events,
)


class MockCOTS(COTSPart):
    def __init__(self, part_number="test-v1", price=10.0, weight_g=100.0):
        super().__init__(
            category="test_cat",
            part_number=part_number,
            data={"price": price, "weight_g": weight_g},
        )


def test_component_usage_event():
    clear_emitted_events()
    _ = MockCOTS()
    assert_event_emitted("component_usage", category="test_cat", part_number="test-v1")


def test_cots_search_event():
    # We need a db path, let's mock search_parts or use a dummy file
    # For now, let's just mock the emission if needed or use a real call if possible
    # Actually, search_parts hits a real sqlite db.

    # Let's try to just call it with a non-existent db and catch the error if it still emits
    # wait, search_parts emits event AFTER search.

    clear_emitted_events()
    query = SearchQuery(query="motor", limit=5)

    # We might need to mock create_engine or just provide a dummy db
    # To keep it simple, I'll just verify the emission logic exists.
    try:
        search_parts(query, "non_existent.db")
    except Exception:
        pass

    # Even if it fails, it might not reach emit_event.
    # Actually looking at shared/cots/runtime.py:67, it emits AFTER stmt.all()
    # So we need it to succeed or mock it.

    from unittest.mock import MagicMock, patch

    # Mock Session context manager and query chain
    mock_session = MagicMock()
    mock_meta = MagicMock()
    mock_meta.catalog_version = "v1"
    mock_meta.bd_warehouse_commit = "sha123"
    mock_meta.generated_at = MagicMock()
    mock_meta.generated_at.isoformat.return_value = "2023-01-01T00:00:00"

    mock_session.query.return_value.order_by.return_value.limit.return_value.first.return_value = mock_meta

    # Mock for search results (COTSItemORM)
    mock_session.query.return_value.filter.return_value.limit.return_value.all.return_value = []

    with patch("shared.cots.runtime.Session") as MockSession:
        MockSession.return_value.__enter__.return_value = mock_session
        search_parts(query, "dummy.db")

    assert_event_emitted("cots_search", query="motor")
