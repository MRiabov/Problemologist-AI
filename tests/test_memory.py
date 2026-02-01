import pytest
import os
from src.agent.tools import memory
from src.agent.utils.workspace import Workspace

@pytest.fixture
def temp_workspace(tmp_path):
    # tmp_path is a pathlib.Path object provided by pytest
    new_workspace = Workspace(root_dir=str(tmp_path))

    # Patch the workspace in the memory module
    original_workspace = memory.workspace
    memory.workspace = new_workspace

    yield new_workspace

    # Restore original workspace
    memory.workspace = original_workspace

def test_write_and_read_journal(temp_workspace):
    memory.write_journal.invoke({"entry": "First entry", "tags": ["test"]})
    content = memory.read_journal.invoke({"topic": ""})
    assert "First entry" in content
    assert "Tags: test" in content

def test_read_journal_filtering(temp_workspace):
    memory.write_journal.invoke({"entry": "This is about Python programming."})
    memory.write_journal.invoke({"entry": "This is about Rust programming."})
    memory.write_journal.invoke({"entry": "This is about generic coding."})

    # Filter for "Python"
    python_content = memory.read_journal.invoke({"topic": "Python"})
    assert "Python programming" in python_content
    assert "Rust programming" not in python_content

    # Filter for "Rust"
    rust_content = memory.read_journal.invoke({"topic": "Rust"})
    assert "Rust programming" in rust_content
    assert "Python programming" not in rust_content

    # Filter for "coding"
    coding_content = memory.read_journal.invoke({"topic": "coding"})
    assert "generic coding" in coding_content
    assert "Rust programming" not in coding_content

    # Verify full content
    full_content = memory.read_journal.invoke({"topic": ""})
    assert "Python programming" in full_content
    assert "Rust programming" in full_content
    assert "generic coding" in full_content

def test_read_journal_empty(temp_workspace):
    result = memory.read_journal.invoke({"topic": ""})
    assert result == "Journal is empty."
