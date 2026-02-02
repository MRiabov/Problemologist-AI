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
    # Count can vary based on docs, just check it's non-zero
    assert result.count("===") >= 6

def test_check_manufacturability_assembly():
    content = """
from build123d import Box
part1 = Box(10, 10, 10)
part1.label = "quantity:1|process:print_3d"
part2 = Box(5, 5, 5).translate((20, 0, 0))
part2.label = "quantity:1|process:print_3d"
result = [part1, part2]
"""
    tools.write_script(content, "design.py")
    report = tools.check_manufacturability("design.py", process="print_3d", quantity=1)
    
    print(f"DEBUG: Report: {report}")
    if "status" not in report:
        pytest.fail(f"Report missing status: {report}")
    
    if report["status"] != "pass":
        print(f"DEBUG: Violations: {report['violations']}")
    assert report["status"] == "pass"
    assert len(report["parts"]) == 2
    # Part 1 volume = 1000, Cost = 1000 * 0.05 = 50.0
    # Part 2 volume = 125, Cost = 125 * 0.05 = 6.25
    # Total = 56.25
    assert pytest.approx(report["cost_analysis"]["total_cost"]) == 56.25
