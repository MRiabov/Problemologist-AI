import pytest
from worker.utils.file_validation import validate_node_output


def test_validate_node_output_planner_success():
    files = {
        "plan.md": "# Engineering Plan\n## Context\nSome context\n## Proposed Changes\nChange 1",
        "todo.md": "- [ ] task 1",
    }
    is_valid, errors = validate_node_output("planner", files)
    assert is_valid
    assert not errors


def test_validate_node_output_planner_missing_file():
    files = {
        "plan.md": "# Engineering Plan\n## Context\nSome context\n## Proposed Changes\nChange 1"
    }
    is_valid, errors = validate_node_output("planner", files)
    assert not is_valid
    assert "Missing required file: todo.md" in errors


def test_validate_node_output_template_placeholder():
    files = {
        "plan.md": "# Engineering Plan\n## Context\nSome context\n## Proposed Changes\n[Add implementation details here]",
        "todo.md": "- [ ] task 1",
    }
    is_valid, errors = validate_node_output("planner", files)
    assert not is_valid
    assert any("contains template placeholders" in e for e in errors)


def test_validate_node_output_invalid_plan_structure():
    files = {"plan.md": "Just some text without headers", "todo.md": "- [ ] task 1"}
    is_valid, errors = validate_node_output("planner", files)
    assert not is_valid
    assert any("plan.md:" in e for e in errors)


def test_validate_node_output_coder_success():
    files = {
        "plan.md": "# Engineering Plan\n## Context\nSome context\n## Proposed Changes\nChange 1",
        "todo.md": "- [ ] task 1",
        "objectives.yaml": "x_min: 0.0\nx_max: 10.0\ny_min: 0.0\ny_max: 10.0\nz_min: 0.0\nz_max: 10.0\n",
    }
    is_valid, errors = validate_node_output("coder", files)
    assert is_valid
    assert not errors


def test_validate_node_output_objectives_template():
    files = {
        "plan.md": "# Engineering Plan\n## Context\nSome context\n## Proposed Changes\nChange 1",
        "todo.md": "- [ ] task 1",
        "objectives.yaml": "x_min: x_min\nx_max: 10.0\n",
    }
    is_valid, errors = validate_node_output("coder", files)
    assert not is_valid
    assert any("objectives.yaml:" in e for e in errors)
