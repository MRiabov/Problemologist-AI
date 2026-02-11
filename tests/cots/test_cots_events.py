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

    from unittest.mock import patch

    with patch("shared.cots.runtime.Session"):
        search_parts(query, "dummy.db")

    assert_event_emitted("cots_search", query="motor")
