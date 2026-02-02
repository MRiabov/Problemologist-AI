import pytest
import shutil
import os
from pathlib import Path
from src.agent.tools.memory import read_journal, write_journal, workspace


@pytest.fixture(autouse=True)
def setup_memory_workspace():
    # Use a separate test workspace for memory tests to avoid conflicts
    test_dir_name = "test_memory_workspace"
    original_root = workspace.root_dir

    # Set the workspace root to a test directory
    workspace.root_dir = Path(test_dir_name).resolve()

    if workspace.root_dir.exists():
        shutil.rmtree(workspace.root_dir)
    workspace.root_dir.mkdir(parents=True, exist_ok=True)

    yield

    # Cleanup
    if workspace.root_dir.exists():
        shutil.rmtree(workspace.root_dir)

    # Restore original root (though not strictly necessary if process ends)
    workspace.root_dir = original_root


def test_read_empty_journal():
    result = read_journal.invoke({"topic": ""})
    assert result == "Journal is empty."


def test_write_and_read_journal():
    write_journal.invoke({"entry": "Test Entry", "tags": ["test"]})
    result = read_journal.invoke({"topic": ""})
    assert "Test Entry" in result
    assert "Tags: test" in result


def test_filter_journal_by_content():
    write_journal.invoke({"entry": "Apple pie recipe", "tags": ["food"]})
    write_journal.invoke({"entry": "Car maintenance", "tags": ["auto"]})

    result = read_journal.invoke({"topic": "pie"})
    assert "Apple pie recipe" in result
    assert "Car maintenance" not in result


def test_filter_journal_by_tag():
    write_journal.invoke({"entry": "Apple pie recipe", "tags": ["food"]})
    write_journal.invoke({"entry": "Car maintenance", "tags": ["auto"]})

    result = read_journal.invoke({"topic": "auto"})
    assert "Car maintenance" in result
    assert "Apple pie recipe" not in result


def test_filter_no_match():
    write_journal.invoke({"entry": "Apple pie recipe", "tags": ["food"]})

    result = read_journal.invoke({"topic": "space"})
    assert "No journal entries found matching topic: 'space'" in result


def test_filter_case_insensitive():
    write_journal.invoke({"entry": "Apple pie recipe", "tags": ["food"]})

    result = read_journal.invoke({"topic": "APPLE"})
    assert "Apple pie recipe" in result
