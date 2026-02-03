import shutil
import json
from pathlib import Path
from datetime import datetime
import pytest

from src.environment.runtime import ToolRuntime
from src.agent.utils.config import Config


@pytest.fixture
def test_workspace(tmp_path):
    workspace = tmp_path / "workspace"
    workspace.mkdir(parents=True, exist_ok=True)
    return str(workspace)


@pytest.fixture
def runtime(test_workspace):
    return ToolRuntime(test_workspace)


# CADEnv fixture removed


def test_runtime_write_file(runtime, test_workspace):
    content = "test content"
    path = "test.txt"
    result = runtime.write_file(content, path, mode="overwrite")
    assert "Successfully wrote" in result

    full_path = Path(test_workspace) / path
    assert full_path.exists()
    assert full_path.read_text() == content


def test_runtime_write_file_append(runtime, test_workspace):
    runtime.write_file("line 1\n", "append.txt", mode="overwrite")
    result = runtime.write_file("line 2", "append.txt", mode="append")
    assert "Successfully appended" in result

    full_path = Path(test_workspace) / "append.txt"
    assert full_path.read_text() == "line 1\nline 2"


def test_runtime_write_file_journal_append(runtime, test_workspace):
    content = "Thinking about the design."
    result = runtime.write_file(content, "journal.md", mode="append")
    assert "Successfully appended" in result

    full_path = Path(test_workspace) / "journal.md"
    content_read = full_path.read_text()
    assert "## [" in content_read
    assert content in content_read
    today = datetime.now().strftime("%Y-%m-%d")
    assert today in content_read


def test_runtime_write_file_linter_integration(runtime):
    content = "x = "
    result = runtime.write_file(content, "error.py")
    assert "Successfully wrote" in result
    assert "Linter Feedback" in result
    assert "Syntax Error" in result or "expression" in result.lower()


def test_runtime_write_file_security(runtime):
    # Try to write outside workspace
    result = runtime.write_file("harmful", "../outside.txt")
    assert "Error: Path ../outside.txt is outside the workspace" in result


def test_runtime_edit_file_success(runtime, test_workspace):
    runtime.write_file("original text", "edit_me.txt")
    result = runtime.edit_file("edit_me.txt", "original", "modified")
    assert "Successfully edited" in result

    full_path = Path(test_workspace) / "edit_me.txt"
    assert full_path.read_text() == "modified text"


def test_runtime_edit_file_not_found(runtime):
    runtime.write_file("some text", "missing.txt")
    result = runtime.edit_file("missing.txt", "not_there", "replacement")
    assert "Error: 'find' string not found" in result


def test_runtime_edit_file_ambiguous(runtime):
    runtime.write_file("repeat repeat", "ambiguous.txt")
    result = runtime.edit_file("ambiguous.txt", "repeat", "once")
    assert "Error: 'find' string is ambiguous" in result


# CADEnv tests removed as CADEnv is deprecated.
# ToolRuntime dispatch logic is tested implicitly by agent integration tests and explicitly in test_core.py.
# The remaining tests in this file focused on ToolRuntime specific methods are still valid.
