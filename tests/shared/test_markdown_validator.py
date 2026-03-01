import pytest
from shared.workers.markdown_validator import validate_todo_md

def test_validate_todo_md_no_deletions():
    baseline = "- [ ] Task 1\n- [ ] Task 2"
    current = "- [x] Task 1\n- [ ] Task 2"
    result = validate_todo_md(current, baseline_content=baseline)
    assert result.is_valid
    assert not result.violations

def test_validate_todo_md_with_deletions():
    baseline = "- [ ] Task 1\n- [ ] Task 2"
    current = "- [x] Task 1"
    result = validate_todo_md(current, baseline_content=baseline)
    assert not result.is_valid
    assert any("Checkboxes were deleted" in v for v in result.violations)

def test_validate_todo_md_with_addition():
    baseline = "- [ ] Task 1"
    current = "- [x] Task 1\n- [ ] Task 2"
    result = validate_todo_md(current, baseline_content=baseline)
    assert result.is_valid

def test_validate_todo_md_invalid_format():
    content = "- [invalid] Task 1"
    result = validate_todo_md(content)
    assert not result.is_valid
    assert any("invalid checkbox" in v for v in result.violations)
