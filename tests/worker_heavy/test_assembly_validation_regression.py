from worker_heavy.utils.file_validation import validate_node_output

def test_assembly_definition_validation_gap():
    # Setup
    valid_plan = """
# Learning Objective
Goal: Test

# Geometry
Shape: Box

# Objectives
- Objective 1
"""
    valid_todo = "- [ ] Task 1"

    # Invalid YAML (bad type for cost)
    invalid_assembly_def = """
version: "1.0"
constraints:
  benchmark_max_unit_cost_usd: "NOT_A_NUMBER"
  benchmark_max_weight_g: 1000.0
manufactured_parts: []
cots_parts: []
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
  estimate_confidence: "high"
"""

    files_content_map = {
        "plan.md": valid_plan,
        "todo.md": valid_todo,
        "assembly_definition.yaml": invalid_assembly_def
    }

    # Execute
    # Now that the fix is applied, this should return False
    is_valid, errors = validate_node_output("planner", files_content_map)

    print(f"Validation result: {is_valid}, Errors: {errors}")

    assert is_valid == False, "Validation PASSED (Bug still present or new logic flawed)!"

    # Assert that errors about assembly_definition.yaml EXIST
    has_yaml_error = any("assembly_definition.yaml" in e for e in errors)
    assert has_yaml_error, "No assembly_definition.yaml errors found! (Validation might have run but not caught the issue?)"

    # Check for specific pydantic error if possible
    has_type_error = any("Input should be a valid number" in e for e in errors)
    if has_type_error:
        print("Found expected type error.")

if __name__ == "__main__":
    try:
        test_assembly_definition_validation_gap()
        print("Test PASSED (Fix Verified)")
    except AssertionError as e:
        print(f"Test FAILED: {e}")
