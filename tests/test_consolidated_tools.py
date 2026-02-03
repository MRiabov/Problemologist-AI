import shutil
from pathlib import Path
from datetime import datetime
import pytest
from src.environment import tools
from src.environment.core import CADEnv


@pytest.fixture(autouse=True)
def setup_workspace():
    """Ensure a clean workspace for each test."""
    workspace = Path(tools.WORKSPACE_DIR)
    if workspace.exists():
        shutil.rmtree(workspace)
    workspace.mkdir(parents=True, exist_ok=True)
    yield


def test_write_file_overwrite():
    content = "test content"
    path = "test.txt"
    result = tools.write_file(content, path, mode="overwrite")
    assert "Successfully wrote" in result

    full_path = Path(tools.WORKSPACE_DIR) / path
    assert full_path.exists()
    assert full_path.read_text() == content


def test_write_file_append():
    tools.write_file("line 1\n", "append.txt", mode="overwrite")
    result = tools.write_file("line 2", "append.txt", mode="append")
    assert "Successfully appended" in result

    full_path = Path(tools.WORKSPACE_DIR) / "append.txt"
    assert full_path.read_text() == "line 1\nline 2"


def test_write_file_journal_append():
    content = "Thinking about the design."
    result = tools.write_file(content, "journal.md", mode="append")
    assert "Successfully appended" in result

    full_path = Path(tools.WORKSPACE_DIR) / "journal.md"
    content_read = full_path.read_text()
    assert "## [" in content_read
    assert content in content_read
    # Verify timestamp format YYYY-MM-DD
    today = datetime.now().strftime("%Y-%m-%d")
    assert today in content_read


def test_write_file_linter_integration():
    # Write a python file with an error
    content = "x = "
    result = tools.write_file(content, "error.py")
    assert "Successfully wrote" in result
    assert "Linter Feedback" in result
    assert "Syntax Error" in result or "expression" in result.lower()


def test_write_file_security():
    # Try to write outside workspace
    result = tools.write_file("harmful", "../outside.txt")
    assert "Error: Path ../outside.txt is outside the workspace" in result


def test_edit_file_success():
    tools.write_file("original text", "edit_me.txt")
    result = tools.edit_file("edit_me.txt", "original", "modified")
    assert "Successfully edited" in result

    full_path = Path(tools.WORKSPACE_DIR) / "edit_me.txt"
    assert full_path.read_text() == "modified text"


def test_edit_file_not_found():
    tools.write_file("some text", "missing.txt")
    result = tools.edit_file("missing.txt", "not_there", "replacement")
    assert "Error: 'find' string not found" in result


def test_edit_file_ambiguous():
    tools.write_file("repeat repeat", "ambiguous.txt")
    result = tools.edit_file("ambiguous.txt", "repeat", "once")
    assert "Error: 'find' string is ambiguous" in result


def test_edit_file_linter_integration():
    tools.write_file("x = 1", "linter_edit.py")
    # Introduce an error via edit
    result = tools.edit_file("linter_edit.py", "x = 1", "x = ")
    assert "Successfully edited" in result
    assert "Linter Feedback" in result


def test_cadenv_write_file_integration(tmp_path):
    # Setup CADEnv with temporary database
    db_path = tmp_path / "test.db"
    workspace = tmp_path / "workspace"
    env = CADEnv(db_url=f"sqlite:///{db_path}", workspace_dir=str(workspace))
    env.reset()

    # Test multi-argument write_file
    action = {
        "tool": 0,  # write_file
        "arguments": "new content|||sub/test.txt|||overwrite",
    }
    _obs, _reward, _terminated, _truncated, _info = env.step(action)

    assert "Successfully wrote" in _obs["last_output"]
    full_path = workspace / "sub/test.txt"
    assert full_path.exists()
    assert full_path.read_text() == "new content"


def test_cadenv_edit_file_integration(tmp_path):
    db_path = tmp_path / "test_edit.db"
    workspace = tmp_path / "workspace"
    env = CADEnv(db_url=f"sqlite:///{db_path}", workspace_dir=str(workspace))
    env.reset()

    # Pre-write a file
    (workspace / "edit_test.txt").write_text("hello world")

    # Test multi-argument edit_file
    action = {
        "tool": 1,  # edit_file
        "arguments": "edit_test.txt|||world|||universe",
    }
    _obs, _reward, _terminated, _truncated, _info = env.step(action)

    assert "Successfully edited" in _obs["last_output"]
    assert (workspace / "edit_test.txt").read_text() == "hello universe"


def test_cadenv_edit_file_backward_compatibility(tmp_path):
    db_path = tmp_path / "test_edit.db"
    workspace = tmp_path / "workspace"
    env = CADEnv(db_url=f"sqlite:///{db_path}", workspace_dir=str(workspace))
    env.reset()

    # Pre-write design.py
    (workspace / "design.py").write_text("x = 10")

    # Test legacy find|||replace format (implicitly targeting design.py)
    action = {
        "tool": 1,  # edit_file
        "arguments": "10|||20",
    }
    _obs, _reward, _terminated, _truncated, _info = env.step(action)

    assert "Successfully edited" in _obs["last_output"]
    assert (workspace / "design.py").read_text() == "x = 20"
