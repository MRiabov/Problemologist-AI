import pytest
from worker_light.utils.docs import get_docs_for

def test_get_docs_for_nonexistent():
    res = get_docs_for("nonexistent_entity_12345")
    assert "No documentation found" in res

def test_get_docs_for_string():
    res = get_docs_for("test")
    assert isinstance(res, str)
