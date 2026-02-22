import pytest
from shared.workers.markdown_validator import validate_plan_md, validate_todo_md

def test_validate_plan_md_valid():
    content = """
## 1. Solution Overview
Something.
## 2. Parts List
- Part A
- Part B
## 3. Assembly Strategy
1. Step 1
2. Step 2
## 4. Cost & Weight Budget
| Part | Cost |
|------|------|
| A    | $1   |
## 5. Risk Assessment
- Risk 1
"""
    result = validate_plan_md(content)
    assert result.is_valid is True
    assert not result.violations

def test_validate_plan_md_missing_sections():
    content = "## 1. Solution Overview\nOnly one section."
    result = validate_plan_md(content)
    assert result.is_valid is False
    assert len(result.violations) == 4
    assert "Missing required section: ## 2. Parts List" in result.violations

def test_validate_plan_md_invalid_parts_list():
    content = """
## 1. Solution Overview
Something.
## 2. Parts List
No list here.
## 3. Assembly Strategy
1. Step 1
## 4. Cost & Weight Budget
- $10
## 5. Risk Assessment
- Low
"""
    result = validate_plan_md(content)
    assert result.is_valid is False
    assert any("Parts List must contain a bullet list" in v for v in result.violations)

def test_validate_todo_md_valid():
    content = "- [ ] Task 1\n- [x] Task 2\n- [/] Task 3\n- [-] Task 4"
    result = validate_todo_md(content)
    assert result.is_valid is True

def test_validate_todo_md_invalid():
    content = "- [invalid] Task 1"
    result = validate_todo_md(content)
    assert result.is_valid is False
    assert "Found 1 invalid checkbox(es)" in result.violations[0]

def test_validate_todo_md_require_completion():
    content = "- [x] Done\n- [ ] Not done"
    result = validate_todo_md(content, require_completion=True)
    assert result.is_valid is False
    assert "All TODO items must be completed or skipped" in result.violations[0]

    content_ok = "- [x] Done\n- [-] Skipped"
    result_ok = validate_todo_md(content_ok, require_completion=True)
    assert result_ok.is_valid is True
