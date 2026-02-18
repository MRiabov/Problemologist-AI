from worker.utils.file_validation import validate_node_output

# Valid plan content for engineering nodes
VALID_ENGINEERING_PLAN = """## 1. Solution Overview
General idea.
## 2. Parts List
- part 1
## 3. Assembly Strategy
1. step 1
## 4. Cost & Weight Budget
- budget 10
## 5. Risk Assessment
- small risk
"""

# Valid objectives.yaml content
VALID_OBJECTIVES_YAML = """
objectives:
  goal_zone:
    min: [0, 0, 0]
    max: [10, 10, 10]
  build_zone:
    min: [-100, -100, -100]
    max: [100, 100, 100]
simulation_bounds:
  min: [-200, -200, -200]
  max: [200, 200, 200]
moved_object:
  label: "cube"
  shape: "box"
  start_position: [0, 0, 0]
  runtime_jitter: [0, 0, 0]
constraints:
  max_unit_cost: 100
  max_weight: 5
"""


def test_validate_node_output_planner_success():
    files = {
        "plan.md": VALID_ENGINEERING_PLAN,
        "todo.md": "- [ ] task 1",
        "assembly_definition.yaml": "version: '1.0'",
    }
    is_valid, errors = validate_node_output("planner", files)
    assert is_valid, f"Validation failed with errors: {errors}"
    assert not errors


def test_validate_node_output_planner_missing_file():
    files = {"plan.md": "## 1. Solution Overview\nValid"}
    is_valid, errors = validate_node_output("planner", files)
    assert not is_valid
    assert "Missing required file: todo.md" in errors


def test_validate_node_output_template_placeholder():
    files = {
        "plan.md": "## 1. Solution Overview\nTODO: write this",
        "todo.md": "- [ ] task 1",
        "assembly_definition.yaml": "version: '1.0'",
    }
    is_valid, errors = validate_node_output("planner", files)
    assert not is_valid
    assert any("contains template placeholders" in e for e in errors)


def test_validate_node_output_invalid_plan_structure():
    files = {
        "plan.md": "Just some text without headers",
        "todo.md": "- [ ] task 1",
        "assembly_definition.yaml": "version: '1.0'",
    }
    is_valid, errors = validate_node_output("planner", files)
    assert not is_valid
    assert any("plan.md:" in e for e in errors)


def test_validate_node_output_coder_success():
    files = {
        "plan.md": VALID_ENGINEERING_PLAN,
        "todo.md": "- [ ] task 1",
        "objectives.yaml": VALID_OBJECTIVES_YAML,
    }
    is_valid, errors = validate_node_output("coder", files)
    assert is_valid, f"Validation failed with errors: {errors}"
    assert not errors


def test_validate_node_output_objectives_template():
    files = {
        "plan.md": "## 1. Solution Overview\nValid",
        "todo.md": "- [ ] task 1",
        "objectives.yaml": "objectives:\n  goal_zone:\n    min: [x, y, z]\n",
    }
    is_valid, errors = validate_node_output("coder", files)
    assert not is_valid
    assert any("objectives.yaml:" in e for e in errors) or any(
        "contains template placeholders" in e for e in errors
    )
