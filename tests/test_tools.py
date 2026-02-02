import shutil
from pathlib import Path

import pytest

from src.environment import tools


@pytest.fixture(autouse=True)
def setup_workspace():
    """Ensure a clean workspace for each test."""
    workspace = Path(tools.WORKSPACE_DIR)
    if workspace.exists():
        shutil.rmtree(workspace)
    workspace.mkdir(parents=True, exist_ok=True)
    yield
    # Optional: cleanup after tests
    # shutil.rmtree(tools.WORKSPACE_DIR)


def test_write_script():
    content = "print('hello')"
    result = tools.write_script(content, "hello.py")
    assert "Successfully wrote" in result

    path = Path(tools.WORKSPACE_DIR) / "hello.py"
    assert path.exists()
    assert path.read_text() == content


def test_edit_script_success():
    tools.write_script("line1\nline2\nline3", "edit.py")
    result = tools.edit_script("edit.py", "line2", "line_two")
    assert "Successfully edited" in result

    path = Path(tools.WORKSPACE_DIR) / "edit.py"
    content = path.read_text()
    assert "line_two" in content
    assert "line2" not in content


def test_edit_script_not_found():
    tools.write_script("line1", "edit.py")
    result = tools.edit_script("edit.py", "missing", "found")
    assert "Error: 'find' string not found" in result


def test_edit_script_ambiguous():
    tools.write_script("line\nline", "edit.py")
    result = tools.edit_script("edit.py", "line", "new")
    assert "Error: 'find' string is ambiguous" in result


def test_preview_design():
    # Write a simple build123d script
    content = """
from build123d import *
with BuildPart() as p:
    Box(10, 10, 10)
"""
    tools.write_script(content, "design.py")
    result = tools.preview_design("design.py")
    assert "Preview generated" in result

    # It might be png or svg depending on implementation details, check result
    workspace = Path(tools.WORKSPACE_DIR)
    if "design.png" in result:
        path = workspace / "design.png"
    else:
        path = workspace / "design.svg"

    assert path.exists()


def test_search_docs():
    result = tools.search_docs("Box")
    assert "Box" in result
    # It might not return stubs.md if other docs outrank it
    assert result.count("===") == 6  # 2 per document (start and end)
